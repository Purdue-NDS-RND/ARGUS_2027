"""Reading and writing rosbag2 bags without ROS (SIM-09): sqlite3 and MCAP."""

import sqlite3
import struct
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from replay import bag, topics  # noqa: E402
from replay.bag import Record  # noqa: E402
from replay.cdr import Message  # noqa: E402

import replay_flight as flight  # noqa: E402

RECORDS = [
    flight.pose_record(flight.T0, east=1.0),
    flight.size_record(flight.T0 + 5 * flight.MS),
    flight.detection_record(flight.T0, flight.T0 + 40 * flight.MS, (320.0, 180.0)),
    flight.pose_record(flight.T0 + 33 * flight.MS, east=2.0),
]


@pytest.mark.parametrize("storage, chunked", [("sqlite3", True), ("mcap", True), ("mcap", False)])
def test_a_bag_written_here_reads_back_the_same_in_recording_order(tmp_path, storage, chunked):
    folder = bag.write_bag(tmp_path / "flight", reversed(RECORDS), storage=storage, chunked=chunked)
    messages = list(bag.open_bag(folder))
    assert [m.log_time_ns for m in messages] == sorted(r.log_time_ns for r in RECORDS)
    assert [m.topic for m in messages] == [topics.POSE, topics.LEGACY_IMAGE_SIZE, topics.POSE, topics.contract_topic("target_cv")]
    assert messages[0].type_name == "geometry_msgs/msg/PoseStamped" and messages[0].decode().pose.position.x == 1.0
    detections = messages[3].decode()
    assert detections.detections[0].results[0].hypothesis.class_id == "bullseye" and detections.detections[0].bbox.center.position.x == 320.0
    # The file alone, without the folder, is a bag too; and one topic can be asked for.
    (file,) = [path for path in folder.iterdir() if path.suffix in (".db3", ".mcap")]
    assert len(list(bag.open_bag(file))) == 4
    assert [m.decode().pose.position.x for m in bag.read_topics(folder, [topics.POSE])] == [1.0, 2.0]


def test_the_two_storages_hold_the_same_messages(tmp_path):
    one = list(bag.open_bag(bag.write_bag(tmp_path / "a", RECORDS, storage="sqlite3")))
    other = list(bag.open_bag(bag.write_bag(tmp_path / "b", RECORDS, storage="mcap")))
    assert [(m.topic, m.type_name, m.log_time_ns, m.data) for m in one] == [(m.topic, m.type_name, m.log_time_ns, m.data) for m in other]


def test_the_folder_has_rosbag2s_metadata(tmp_path):
    import yaml

    folder = bag.write_bag(tmp_path / "flight", RECORDS, storage="mcap")
    info = yaml.safe_load((folder / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    assert info["storage_identifier"] == "mcap" and info["message_count"] == 4 and info["relative_file_paths"] == ["flight_0.mcap"]
    counts = {entry["topic_metadata"]["name"]: entry["message_count"] for entry in info["topics_with_message_count"]}
    assert counts[topics.POSE] == 2 and info["duration"]["nanoseconds"] == 38 * flight.MS  # a pose heard 2 ms after its stamp, to a detection heard at 40 ms
    with pytest.raises(FileExistsError):
        bag.write_bag(folder, RECORDS)  # a bag is never written over another


def test_the_sqlite3_file_is_rosbag2s_schema(tmp_path):
    folder = bag.write_bag(tmp_path / "flight", RECORDS, storage="sqlite3")
    connection = sqlite3.connect(folder / "flight_0.db3")
    assert connection.execute("SELECT name, type, serialization_format FROM topics ORDER BY id").fetchall()[0] == (topics.POSE, "geometry_msgs/msg/PoseStamped", "cdr")
    assert connection.execute("SELECT COUNT(*), MIN(timestamp) FROM messages").fetchone() == (4, RECORDS[0].log_time_ns)
    connection.close()


def test_an_mcap_file_is_the_records_the_format_says(tmp_path):
    folder = bag.write_bag(tmp_path / "flight", RECORDS[:1], storage="mcap", chunked=False)
    data = (folder / "flight_0.mcap").read_bytes()
    assert data.startswith(bag.MCAP_MAGIC) and data.endswith(bag.MCAP_MAGIC)
    at = len(bag.MCAP_MAGIC)
    opcodes = []
    while at < len(data) - len(bag.MCAP_MAGIC):
        opcode, length = struct.unpack_from("<BQ", data, at)
        opcodes.append(opcode)
        at += 9 + length
    assert opcodes == [0x01, 0x03, 0x04, 0x05, 0x0F, 0x02]  # header, schema, channel, message, data end, footer
    assert at == len(data) - len(bag.MCAP_MAGIC)


def test_an_mcap_bag_brings_its_own_layouts(tmp_path):
    # A type this harness does not list, declared by the file's own schema, as rosbag2 writes it.
    schema = b"std_msgs/Header header\nfloat32 x\n================================================================================\nMSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id\n"
    body = struct.pack("<iI", 5, 6) + struct.pack("<I", 1) + b"\x00" + b"\x00" * 3 + struct.pack("<f", 2.5)
    records = (
        bag._mcap_record(0x01, bag._mcap_string("ros2") + bag._mcap_string("test"))
        + bag._mcap_record(0x03, struct.pack("<H", 1) + bag._mcap_string("odd_msgs/msg/Thing") + bag._mcap_string("ros2msg") + struct.pack("<I", len(schema)) + schema)
        + bag._mcap_record(0x04, struct.pack("<HH", 1, 1) + bag._mcap_string("/odd") + bag._mcap_string("cdr") + struct.pack("<I", 0))
        + bag._mcap_record(0x05, struct.pack("<HIQQ", 1, 0, 77, 77) + b"\x00\x01\x00\x00" + body)
        + bag._mcap_record(0x07, b"\x00" * 12)  # a record kind the reader has no use for: skipped
        + bag._mcap_record(0x0F, struct.pack("<I", 0))
        + bag._mcap_record(0x05, b"never read: after the end of the data")
    )
    file = tmp_path / "odd.mcap"
    file.write_bytes(bag.MCAP_MAGIC + records + bag.MCAP_MAGIC)
    (message,) = list(bag.open_bag(file))
    assert (message.topic, message.type_name, message.log_time_ns) == ("/odd", "odd_msgs/msg/Thing", 77)
    assert message.decode() == Message(header=Message(stamp=Message(sec=5, nanosec=6), frame_id=""), x=2.5)


def test_bags_that_cannot_be_read_say_why(tmp_path):
    with pytest.raises(bag.BagError, match="no such bag"):
        list(bag.open_bag(tmp_path / "missing"))
    (tmp_path / "empty").mkdir()
    with pytest.raises(bag.BagError, match="no .mcap or .db3 file"):
        list(bag.open_bag(tmp_path / "empty"))
    (tmp_path / "bad.mcap").write_bytes(b"not an mcap file")
    with pytest.raises(bag.BagError, match="not an MCAP file"):
        list(bag.open_bag(tmp_path / "bad.mcap"))
    (tmp_path / "bad.db3").write_bytes(b"not sqlite")
    with pytest.raises(bag.BagError, match="not a rosbag2 sqlite3 file"):
        list(bag.open_bag(tmp_path / "bad.db3"))
    (tmp_path / "x.txt").write_text("")
    with pytest.raises(bag.BagError, match="ends in .db3 or .mcap"):
        list(bag.open_bag(tmp_path / "x.txt"))
    cut = bag.write_bag(tmp_path / "cut", RECORDS, storage="mcap", chunked=False) / "cut_0.mcap"
    cut.write_bytes(cut.read_bytes()[:200])
    with pytest.raises(bag.BagError, match="ends in the middle of a record"):
        list(bag.open_bag(cut))
    with pytest.raises(bag.BagError, match="storage must be"):
        bag.write_bag(tmp_path / "other", RECORDS, storage="parquet")


def test_a_compressed_chunk_without_its_library_says_how_to_record_without(tmp_path, monkeypatch):
    monkeypatch.setitem(sys.modules, "zstandard", None)  # as on a machine without it
    chunk = struct.pack("<QQQI", 0, 0, 10, 0) + bag._mcap_string("zstd") + struct.pack("<Q", 4) + b"\x00" * 4
    file = tmp_path / "z.mcap"
    file.write_bytes(bag.MCAP_MAGIC + bag._mcap_record(0x01, bag._mcap_string("ros2") + bag._mcap_string("t")) + bag._mcap_record(0x06, chunk) + bag.MCAP_MAGIC)
    with pytest.raises(bag.BagError, match="zstd-compressed and the zstandard module is not installed. Record without compression"):
        list(bag.open_bag(file))
    unknown = struct.pack("<QQQI", 0, 0, 10, 0) + bag._mcap_string("brotli") + struct.pack("<Q", 0)
    file.write_bytes(bag.MCAP_MAGIC + bag._mcap_record(0x06, unknown) + bag.MCAP_MAGIC)
    with pytest.raises(bag.BagError, match="unknown chunk compression 'brotli'"):
        list(bag.open_bag(file))


def test_a_split_bag_is_read_as_one_in_time_order(tmp_path):
    folder = tmp_path / "split"
    first = bag.write_bag(tmp_path / "one", RECORDS[2:], storage="sqlite3")
    second = bag.write_bag(tmp_path / "two", RECORDS[:2], storage="sqlite3")
    folder.mkdir()
    (first / "one_0.db3").rename(folder / "split_0.db3")
    (second / "two_0.db3").rename(folder / "split_1.db3")
    assert [m.log_time_ns for m in bag.open_bag(folder)] == sorted(r.log_time_ns for r in RECORDS)
