"""Read and write rosbag2 bags without ROS (SIM-09): the sqlite3 storage and MCAP.

`open_bag(path)` gives the messages of a bag in the order they were
recorded, each with its topic, type, the time the recorder received it
(`log_time_ns`) and its CDR bytes; `BagMessage.decode()` turns the bytes into
a message (cdr.py). `path` is a bag folder (with rosbag2's metadata.yaml), a
.db3 file or a .mcap file.

`write_bag(path, records, storage)` writes a bag the same reader, and
`ros2 bag`, can open: the tests make their bags with it, and the replay's
`core` mode is run on them in CI.

MCAP: files written without chunk compression are read with nothing but
the standard library. zstd and lz4 chunks need the `zstandard` or `lz4`
module; without it the reader says so, and how to record without
compression. The summary section and indexes are not used: the file is read
front to back, which is all a replay needs.
"""

from __future__ import annotations

import sqlite3
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Optional, Tuple

from . import cdr
from .msgdefs import Layouts, normalise, parse_ros2msg

MCAP_MAGIC = b"\x89MCAP0\r\n"
SQLITE3, MCAP = "sqlite3", "mcap"
# MCAP record opcodes
_HEADER, _FOOTER, _SCHEMA, _CHANNEL, _MESSAGE, _CHUNK, _DATA_END = 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x0F


class BagError(ValueError):
    """A bag that cannot be read or written. The message says why."""


@dataclass(frozen=True)
class BagMessage:
    topic: str
    type_name: str  # "pkg/msg/Type"
    log_time_ns: int  # when the recorder received it, on the recorder's clock
    data: bytes  # CDR
    layouts: Layouts

    def decode(self) -> cdr.Message:
        return cdr.decode(self.type_name, self.data, self.layouts)


@dataclass(frozen=True)
class Record:
    """One message to write: `message` is a cdr.Message, a dict or CDR bytes."""

    topic: str
    type_name: str
    log_time_ns: int
    message: object


# --- reading ---------------------------------------------------------------------------


def _bag_files(path: Path) -> Tuple[str, List[Path]]:
    """(storage, files in recording order) for a bag folder or a single file."""
    if path.is_file():
        if path.suffix == ".db3":
            return SQLITE3, [path]
        if path.suffix == ".mcap":
            return MCAP, [path]
        raise BagError(f"{path}: a bag file ends in .db3 or .mcap")
    if not path.is_dir():
        raise BagError(f"{path}: no such bag")
    for storage, pattern in ((MCAP, "*.mcap"), (SQLITE3, "*.db3")):
        files = sorted(path.glob(pattern))
        if files:
            return storage, files
    raise BagError(f"{path}: no .mcap or .db3 file in the folder")


def _read_sqlite3(file: Path, layouts: Layouts) -> Iterator[BagMessage]:
    try:
        connection = sqlite3.connect(f"file:{file}?mode=ro", uri=True)
        topics = {row[0]: (row[1], row[2]) for row in connection.execute("SELECT id, name, type FROM topics")}
        rows = connection.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp, id")
        for topic_id, timestamp, data in rows:
            name, type_name = topics[topic_id]
            yield BagMessage(name, type_name, int(timestamp), bytes(data), layouts)
    except sqlite3.Error as error:
        raise BagError(f"{file}: not a rosbag2 sqlite3 file ({error})") from error
    finally:
        try:
            connection.close()
        except Exception:  # noqa: BLE001  (never opened)
            pass


class _Bytes:
    """A cursor over the bytes of an MCAP file or chunk."""

    def __init__(self, data: bytes, what: str) -> None:
        self.data, self.at, self.what = data, 0, what

    def take(self, size: int) -> bytes:
        if self.at + size > len(self.data):
            raise BagError(f"{self.what}: the file ends in the middle of a record")
        chunk = self.data[self.at : self.at + size]
        self.at += size
        return chunk

    def number(self, code: str) -> int:
        return struct.unpack("<" + code, self.take(struct.calcsize(code)))[0]

    def string(self) -> str:
        return self.take(self.number("I")).decode("utf-8", errors="replace")

    def left(self) -> int:
        return len(self.data) - self.at


def _decompress(compression: str, data: bytes, size: int, what: str) -> bytes:
    if compression == "":
        return data
    if compression == "zstd":
        try:
            import zstandard
        except ImportError as error:
            raise BagError(
                f"{what}: its chunks are zstd-compressed and the zstandard module is not installed. "
                "Record without compression (sim/replay/record.launch.py does) or install python3-zstandard"
            ) from error
        return zstandard.ZstdDecompressor().decompress(data, max_output_size=size)
    if compression == "lz4":
        try:
            import lz4.frame
        except ImportError as error:
            raise BagError(f"{what}: its chunks are lz4-compressed and the lz4 module is not installed. Record without compression or install python3-lz4") from error
        return lz4.frame.decompress(data)
    raise BagError(f"{what}: unknown chunk compression {compression!r}")


def _read_mcap(file: Path, layouts: Layouts) -> Iterator[BagMessage]:
    data = file.read_bytes()
    what = str(file)
    if not data.startswith(MCAP_MAGIC):
        raise BagError(f"{what}: not an MCAP file")
    schemas: Dict[int, Tuple[str, str]] = {}  # id -> (type name, encoding)
    channels: Dict[int, Tuple[str, str]] = {}  # id -> (topic, type name)

    def records(cursor: _Bytes) -> Iterator[BagMessage]:
        while cursor.left() >= 9:
            opcode = cursor.number("B")
            body = _Bytes(cursor.take(cursor.number("Q")), what)
            if opcode == _SCHEMA:
                schema_id, name, encoding = body.number("H"), body.string(), body.string()
                text = body.take(body.number("I"))
                schemas[schema_id] = (name, encoding)
                if encoding == "ros2msg":
                    layouts.add(parse_ros2msg(name, text.decode("utf-8", errors="replace")))
            elif opcode == _CHANNEL:
                channel_id, schema_id, topic, encoding = body.number("H"), body.number("H"), body.string(), body.string()
                if encoding != "cdr":
                    raise BagError(f"{what}: topic {topic} is encoded as {encoding!r}, not cdr")
                channels[channel_id] = (topic, schemas.get(schema_id, ("", ""))[0])
            elif opcode == _MESSAGE:
                channel_id, _sequence, log_time, _publish_time = body.number("H"), body.number("I"), body.number("Q"), body.number("Q")
                if channel_id not in channels:
                    raise BagError(f"{what}: a message on channel {channel_id}, which the file never declared")
                topic, type_name = channels[channel_id]
                yield BagMessage(topic, type_name, log_time, body.take(body.left()), layouts)
            elif opcode == _CHUNK:
                _start, _end, size, _crc, compression = body.number("Q"), body.number("Q"), body.number("Q"), body.number("I"), body.string()
                inner = _decompress(compression, body.take(body.number("Q")), size, what)
                yield from records(_Bytes(inner, what))
            elif opcode in (_DATA_END, _FOOTER):
                return  # what follows is the summary: indexes of what has been read already

    cursor = _Bytes(data, what)
    cursor.take(len(MCAP_MAGIC))
    yield from records(cursor)


def open_bag(path) -> Iterator[BagMessage]:
    """Every message of a bag, in the order the recorder received them."""
    storage, files = _bag_files(Path(path))
    layouts = Layouts()
    reader = _read_mcap if storage == MCAP else _read_sqlite3
    if len(files) == 1:
        yield from reader(files[0], layouts)
        return
    merged: List[BagMessage] = []
    for file in files:  # a split bag: its files follow each other in time, but sort to be sure
        merged.extend(reader(file, layouts))
    merged.sort(key=lambda message: message.log_time_ns)
    yield from merged


def read_topics(path, topics: Optional[Iterable[str]] = None) -> Iterator[BagMessage]:
    """As `open_bag`, keeping only the named topics (all of them when none is named)."""
    wanted = None if topics is None else set(topics)
    for message in open_bag(path):
        if wanted is None or message.topic in wanted:
            yield message


# --- writing ---------------------------------------------------------------------------


def _full_type(type_name: str) -> str:
    package, name = normalise(type_name).split("/")
    return f"{package}/msg/{name}"


def _cdr_of(record: Record, layouts: Layouts) -> bytes:
    if isinstance(record.message, (bytes, bytearray)):
        return bytes(record.message)
    return cdr.encode(record.type_name, record.message, layouts)


def _write_sqlite3(file: Path, records: List[Record], layouts: Layouts) -> None:
    connection = sqlite3.connect(file)
    try:
        connection.executescript(
            "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);"
            "CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, timestamp INTEGER NOT NULL, data BLOB NOT NULL);"
            "CREATE INDEX timestamp_idx ON messages (timestamp ASC);"
        )
        ids: Dict[str, int] = {}
        for record in records:
            if record.topic not in ids:
                ids[record.topic] = len(ids) + 1
                connection.execute("INSERT INTO topics VALUES (?, ?, ?, 'cdr', '')", (ids[record.topic], record.topic, _full_type(record.type_name)))
            connection.execute(
                "INSERT INTO messages(topic_id, timestamp, data) VALUES (?, ?, ?)", (ids[record.topic], int(record.log_time_ns), _cdr_of(record, layouts))
            )
        connection.commit()
    finally:
        connection.close()


def _mcap_record(opcode: int, body: bytes) -> bytes:
    return struct.pack("<BQ", opcode, len(body)) + body


def _mcap_string(text: str) -> bytes:
    raw = text.encode("utf-8")
    return struct.pack("<I", len(raw)) + raw


def _write_mcap(file: Path, records: List[Record], layouts: Layouts, chunked: bool) -> None:
    from .msgdefs import DEFINITIONS

    declared = bytearray()
    messages = bytearray()
    channels: Dict[str, int] = {}
    for sequence, record in enumerate(records):
        if record.topic not in channels:
            channel_id = len(channels) + 1
            channels[record.topic] = channel_id
            name = _full_type(record.type_name)
            text = DEFINITIONS.get(normalise(record.type_name), "").encode("utf-8")  # the type's own fields; the types it uses are known to the reader
            declared += _mcap_record(_SCHEMA, struct.pack("<H", channel_id) + _mcap_string(name) + _mcap_string("ros2msg") + struct.pack("<I", len(text)) + text)
            declared += _mcap_record(_CHANNEL, struct.pack("<HH", channel_id, channel_id) + _mcap_string(record.topic) + _mcap_string("cdr") + struct.pack("<I", 0))
        body = struct.pack("<HIQQ", channels[record.topic], sequence, int(record.log_time_ns), int(record.log_time_ns)) + _cdr_of(record, layouts)
        messages += _mcap_record(_MESSAGE, body)
    data = bytes(declared) + bytes(messages)
    if chunked:
        times = [int(record.log_time_ns) for record in records] or [0]
        data = _mcap_record(_CHUNK, struct.pack("<QQQI", min(times), max(times), len(data), 0) + _mcap_string("") + struct.pack("<Q", len(data)) + data)
    header = _mcap_record(_HEADER, _mcap_string("ros2") + _mcap_string("argus sim/replay"))
    footer = _mcap_record(_DATA_END, struct.pack("<I", 0)) + _mcap_record(_FOOTER, struct.pack("<QQI", 0, 0, 0))
    file.write_bytes(MCAP_MAGIC + header + data + footer + MCAP_MAGIC)


def write_bag(path, records: Iterable[Record], storage: str = MCAP, chunked: bool = True) -> Path:
    """Write a bag folder at `path` holding `records`, in the order given. Returns the folder."""
    if storage not in (SQLITE3, MCAP):
        raise BagError(f"storage must be {SQLITE3} or {MCAP}, not {storage!r}")
    folder = Path(path)
    folder.mkdir(parents=True, exist_ok=False)
    records = sorted(records, key=lambda record: int(record.log_time_ns))
    layouts = Layouts()
    file = folder / (f"{folder.name}_0.mcap" if storage == MCAP else f"{folder.name}_0.db3")
    if storage == MCAP:
        _write_mcap(file, records, layouts, chunked)
    else:
        _write_sqlite3(file, records, layouts)
    counts: Dict[Tuple[str, str], int] = {}
    for record in records:
        key = (record.topic, _full_type(record.type_name))
        counts[key] = counts.get(key, 0) + 1
    start = int(records[0].log_time_ns) if records else 0
    end = int(records[-1].log_time_ns) if records else 0
    lines = [
        "rosbag2_bagfile_information:",
        "  version: 5",
        f"  storage_identifier: {storage}",
        "  duration:",
        f"    nanoseconds: {end - start}",
        "  starting_time:",
        f"    nanoseconds_since_epoch: {start}",
        f"  message_count: {len(records)}",
        "  topics_with_message_count:" + ("" if counts else " []"),
    ]
    for (topic, type_name), count in counts.items():
        lines += [
            "    - topic_metadata:",
            f"        name: {topic}",
            f"        type: {type_name}",
            "        serialization_format: cdr",
            "        offered_qos_profiles: \"\"",
            f"      message_count: {count}",
        ]
    lines += [
        "  compression_format: \"\"",
        "  compression_mode: \"\"",
        "  relative_file_paths:",
        f"    - {file.name}",
        "  files:",
        f"    - path: {file.name}",
        "      starting_time:",
        f"        nanoseconds_since_epoch: {start}",
        "      duration:",
        f"        nanoseconds: {end - start}",
        f"      message_count: {len(records)}",
    ]
    (folder / "metadata.yaml").write_text("\n".join(lines) + "\n")
    return folder
