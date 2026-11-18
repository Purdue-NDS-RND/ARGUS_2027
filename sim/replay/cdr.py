"""CDR, the wire form of a ROS 2 message, to and from plain Python values (SIM-09). No ROS imports.

`decode(type, data)` gives a `Message` whose fields read like the ROS
message's (`message.header.stamp.sec`, `message.detections[0].bbox.size_x`),
so the flight code that takes a ROS message takes this too. `encode(type,
value)` writes the same bytes ROS would, from a `Message`, a dict or any
object with those attributes; a field that is missing is zero.

The form is the one rosbag2 stores: a four-byte header saying little- or
big-endian, then the fields in order, each number aligned to its own size
counted from the end of that header, a string as its length (with the
closing zero byte) and its bytes, a sequence as its length and its elements.
ROS sometimes adds up to three bytes of padding after the last field (it
does for a message of fixed size that ends on a single byte) and leaves
whatever was in memory in padding bytes; a reader ignores both, so
`decode` accepts them and `encode` writes neither.
"""

from __future__ import annotations

import struct
from typing import Any, List, Optional

from .msgdefs import PRIMITIVES, DefinitionError, Layouts

HEADER_LITTLE = b"\x00\x01\x00\x00"
HEADER_BIG = b"\x00\x00\x00\x00"


class CdrError(ValueError):
    """Bytes that are not the message they were said to be. The message says where it went wrong."""


class Message:
    """A decoded message: its fields as attributes, nothing else."""

    def __init__(self, **fields: Any) -> None:
        self.__dict__.update(fields)

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Message) and self.__dict__ == other.__dict__

    def __repr__(self) -> str:
        return "Message(" + ", ".join(f"{name}={value!r}" for name, value in self.__dict__.items()) + ")"


class _Reader:
    def __init__(self, data: bytes, layouts: Layouts) -> None:
        if len(data) < 4 or data[0] != 0 or data[1] not in (0, 1):
            raise CdrError("the message does not start with a CDR header")
        self._order = "<" if data[1] == 1 else ">"
        self._data = memoryview(data)[4:]
        self._at = 0
        self._layouts = layouts

    def _take(self, size: int) -> memoryview:
        if self._at + size > len(self._data):
            raise CdrError(f"the message ends at byte {len(self._data)} but a field needs {size} more from byte {self._at}")
        chunk = self._data[self._at : self._at + size]
        self._at += size
        return chunk

    def _align(self, size: int) -> None:
        self._at += (-self._at) % size

    def primitive(self, type_name: str) -> Any:
        code, size = PRIMITIVES[type_name]
        self._align(size)
        return struct.unpack(self._order + code, self._take(size))[0]

    def primitives(self, type_name: str, count: int) -> List[Any]:
        code, size = PRIMITIVES[type_name]
        if count == 0:
            return []
        self._align(size)
        return list(struct.unpack(f"{self._order}{count}{code}", self._take(size * count)))

    def string(self) -> str:
        length = self.primitive("uint32")
        if length == 0:
            return ""
        raw = bytes(self._take(length))
        return raw[:-1].decode("utf-8", errors="replace")

    def value(self, type_name: str) -> Any:
        if type_name in PRIMITIVES:
            return self.primitive(type_name)
        if type_name == "string":
            return self.string()
        message = Message()
        for name, element, count in self._layouts.fields(type_name):
            if count is None:
                message.__dict__[name] = self.value(element)
                continue
            length = self.primitive("uint32") if count == -1 else count
            if element in PRIMITIVES:
                message.__dict__[name] = self.primitives(element, length)
            else:
                message.__dict__[name] = [self.value(element) for _ in range(length)]
        return message

    def done(self) -> bool:
        return len(self._data) - self._at < 4  # only padding can be left


def decode(type_name: str, data: bytes, layouts: Optional[Layouts] = None) -> Message:
    """The message in `data`, said to be of `type_name` ("pkg/msg/Type" or "pkg/Type")."""
    reader = _Reader(bytes(data), layouts or Layouts())
    try:
        message = reader.value(type_name)
    except (struct.error, DefinitionError) as error:
        raise CdrError(f"{type_name}: {error}") from error
    if not reader.done():
        raise CdrError(f"{type_name}: bytes are left over after the last field, so the layout is not this message's")
    return message


class _Writer:
    def __init__(self, layouts: Layouts) -> None:
        self._out = bytearray()
        self._layouts = layouts

    def _align(self, size: int) -> None:
        self._out.extend(b"\x00" * ((-len(self._out)) % size))

    def primitive(self, type_name: str, value: Any) -> None:
        code, size = PRIMITIVES[type_name]
        self._align(size)
        if value is None:
            value = False if type_name == "bool" else 0
        self._out.extend(struct.pack("<" + code, value))

    def string(self, value: Any) -> None:
        raw = ("" if value is None else str(value)).encode("utf-8") + b"\x00"
        self.primitive("uint32", len(raw))
        self._out.extend(raw)

    def value(self, type_name: str, value: Any) -> None:
        if type_name in PRIMITIVES:
            self.primitive(type_name, value)
            return
        if type_name == "string":
            self.string(value)
            return
        for name, element, count in self._layouts.fields(type_name):
            field = value.get(name) if isinstance(value, dict) else getattr(value, name, None)
            if count is None:
                self.value(element, field)
                continue
            items = list(field) if field is not None else []
            if count == -1:
                self.primitive("uint32", len(items))
            else:
                if len(items) > count:
                    raise CdrError(f"{type_name}.{name} holds {count} values, not {len(items)}")
                items = items + [None] * (count - len(items))
            for item in items:
                self.value(element, item)

    def data(self) -> bytes:
        return HEADER_LITTLE + bytes(self._out)


def encode(type_name: str, value: Any, layouts: Optional[Layouts] = None) -> bytes:
    """The CDR bytes of `value` as a message of `type_name`. Little-endian, as ROS writes on every machine we have."""
    writer = _Writer(layouts or Layouts())
    try:
        writer.value(type_name, value)
    except (struct.error, DefinitionError, TypeError) as error:
        raise CdrError(f"{type_name}: {error}") from error
    return writer.data()


def stamp_ns(header: Any) -> int:
    """A header's stamp in whole nanoseconds."""
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
