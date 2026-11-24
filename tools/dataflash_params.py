#!/usr/bin/env python3
"""Read the parameters out of an ArduPilot dataflash log (.BIN). Standard library only. (PLT-08)

  python3 tools/dataflash_params.py <log.BIN>                 # NAME,VALUE lines, sorted, on stdout
  python3 tools/dataflash_params.py <log.BIN> --out file.param

A dataflash log begins by writing every parameter as a PARM message, so a
flight's log is also a record of how the flight controller was set for that
flight. This is how the 2026 aircraft's settings are known: nobody kept its
parameter file, but three of its logs are in the repo.

The log is a stream of messages, each `A3 95 <type>` and a body. FMT
messages (type 128) say how the others are laid out, and PARM's layout is
taken from its FMT, not assumed. A parameter set more than once (changed in
flight) keeps its last value; `changed` lists those. The firmware's version
line is read from the first MSG that has one.
"""

from __future__ import annotations

import argparse
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

HEAD = b"\xa3\x95"
FMT_TYPE, FMT_LENGTH = 128, 89
# Dataflash field letters -> struct codes (ArduPilot's AP_Logger/LogStructure.h).
FIELD = {"a": "64s", "b": "b", "B": "B", "h": "h", "H": "H", "i": "i", "I": "I", "f": "f", "d": "d", "n": "4s", "N": "16s", "Z": "64s",
         "c": "h", "C": "H", "e": "i", "E": "I", "L": "i", "M": "B", "q": "q", "Q": "Q"}  # fmt: skip


class DataflashError(ValueError):
    """Not a dataflash log, or one with no parameters in it."""


@dataclass
class LogParameters:
    values: Dict[str, float] = field(default_factory=dict)
    changed: Dict[str, List[float]] = field(default_factory=dict)  # name -> every value it had, for those set more than once to different values
    firmware: Optional[str] = None


def _text(raw: bytes) -> str:
    return raw.split(b"\0", 1)[0].decode("ascii", "replace")


def messages(path, names=None):
    """Every message of a dataflash log, in the order written: (name, {column: value}).

    `names`, when given, are the only messages yielded (FMT is always read, to learn the others' layouts). A
    message's layout is the one its FMT gave; text columns are left as bytes (`_text` turns one into a string).
    A torn write is skipped by looking for the next message head.
    """
    data = Path(path).read_bytes()
    layouts: Dict[int, Tuple[str, int, struct.Struct, List[str]]] = {FMT_TYPE: ("FMT", FMT_LENGTH, struct.Struct("<BB4s16s64s"), ["Type", "Length", "Name", "Format", "Columns"])}
    wanted = None if names is None else set(names)
    position, end = 0, len(data)
    while position + 3 <= end:
        if data[position : position + 2] != HEAD or data[position + 2] not in layouts:
            position += 1  # resynchronise: a torn write, or a message whose FMT has not been seen
            continue
        name, length, layout, columns = layouts[data[position + 2]]
        if position + length > end:
            break
        if layout.size != length - 3:
            position += length
            continue
        if name == "FMT" or wanted is None or name in wanted:
            row = dict(zip(columns, layout.unpack_from(data, position + 3)))
            if name == "FMT":
                try:
                    packed = struct.Struct("<" + "".join(FIELD[letter] for letter in _text(row["Format"])))
                except KeyError:
                    packed = struct.Struct("<")  # a field letter this reader does not know: the message is skipped by its length
                layouts[row["Type"]] = (_text(row["Name"]), row["Length"], packed, _text(row["Columns"]).split(","))
            if wanted is None or name in wanted:
                yield name, row
        position += length


def read_parameters(path) -> LogParameters:
    result = LogParameters()
    for name, row in messages(path, ("PARM", "MSG")):
        if name == "PARM" and "Name" in row and "Value" in row:
            parameter, value = _text(row["Name"]), float(row["Value"])
            if parameter in result.values and result.values[parameter] != value:
                result.changed.setdefault(parameter, [result.values[parameter]]).append(value)
            result.values[parameter] = value
        elif name == "MSG" and result.firmware is None and "Message" in row:
            message = _text(row["Message"])
            if message.startswith(("ArduCopter", "ArduPlane", "ArduRover", "ArduSub")):
                result.firmware = message
    if not result.values:
        raise DataflashError(f"{path}: no parameters found (is it an ArduPilot dataflash .BIN log?)")
    return result


def format_value(value: float) -> str:
    """A parameter's value as a ground station writes it: whole numbers without a point, others to float precision."""
    return str(int(value)) if value == int(value) and abs(value) < 1e15 else f"{value:.9g}"


def as_param_file(parameters: LogParameters, source: str) -> str:
    lines = [f"# Parameters read from the dataflash log {source} by tools/dataflash_params.py."]
    if parameters.firmware:
        lines.append(f"# Firmware: {parameters.firmware}")
    for name in sorted(parameters.changed):
        lines.append(f"# {name} changed during the log: {', '.join(format_value(v) for v in parameters.changed[name])} (the last is kept)")
    lines += [f"{name},{format_value(parameters.values[name])}" for name in sorted(parameters.values)]
    return "\n".join(lines) + "\n"


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Read the parameters out of an ArduPilot dataflash log.")
    parser.add_argument("log", type=Path)
    parser.add_argument("--out", type=Path, help="write the parameter file here instead of printing it")
    args = parser.parse_args(argv)
    try:
        parameters = read_parameters(args.log)
    except (OSError, DataflashError) as problem:
        print(f"dataflash_params: {problem}", file=sys.stderr)
        return 2
    text = as_param_file(parameters, args.log.name)
    if args.out:
        args.out.write_text(text)
        print(f"{len(parameters.values)} parameters ({parameters.firmware or 'firmware not named'}) -> {args.out}")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
