#!/usr/bin/env python3
"""Check the replay's message layouts against ROS itself (SIM-09). Runs wherever ROS is installed.

  sim/run_sim.sh --exec python3 sim/replay/check_defs.py

msgdefs.py lists, by hand, the layouts of the message types the recorder's
topics carry, and cdr.py reads and writes them without ROS. Here, with ROS
there, each listed type is checked three ways:

  1. its fields and their types are the ones the installed message has;
  2. a message filled with distinct values, serialised by ROS, is decoded
     by cdr.py to the same values;
  3. the same values encoded by cdr.py are bytes ROS reads back unchanged,
     and as many bytes as ROS writes, give or take its padding at the end.

A type whose package is not installed is reported and skipped. Exit code 0
only if every installed type passes.

`ros_field_text` and `fill` are plain functions with no ROS in them.
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path
from typing import Any, List, Tuple

SIM_DIR = Path(__file__).resolve().parents[1]
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))

from replay import cdr, msgdefs  # noqa: E402

_ROS_NAMES = {"boolean": "bool", "octet": "byte", "float": "float32", "double": "float64"}


def ros_field_text(ros_type: str) -> Tuple[str, Any]:
    """(element type, count) for a type as rosidl writes it: "double[9]", "sequence<vision_msgs/Detection2D>", "string"."""
    text, count = ros_type.strip(), None
    if text.startswith("sequence<") and text.endswith(">"):
        text, count = text[len("sequence<") : -1].split(",")[0].strip(), -1
    elif text.endswith("]"):
        text, _, inside = text[:-1].partition("[")
        count = int(inside)
    text = _ROS_NAMES.get(text, text)
    return msgdefs.normalise(text), count


def fill(type_name: str, layouts: msgdefs.Layouts, counter: List[int]) -> Any:
    """A value of a type with every field different from every other, so a field read from the wrong place shows."""
    counter[0] += 1
    n = counter[0]
    if type_name == "string":
        return "s" * (n % 5) + str(n)  # lengths that move the alignment about
    if type_name == "bool":
        return n % 2 == 1
    if type_name in ("float32",):
        return float(n) * 0.5
    if type_name == "float64":
        return float(n) + 0.125
    if type_name in msgdefs.PRIMITIVES:
        return n % 100
    message = cdr.Message()
    for name, element, count in layouts.fields(type_name):
        if count is None:
            message.__dict__[name] = fill(element, layouts, counter)
        else:
            message.__dict__[name] = [fill(element, layouts, counter) for _ in range(2 if count == -1 else count)]
    return message


def _to_ros(value: Any, ros_message: Any, type_name: str, layouts: msgdefs.Layouts) -> None:
    for name, element, count in layouts.fields(type_name):
        field = getattr(value, name)
        if element in msgdefs.PRIMITIVES or element == "string":
            setattr(ros_message, name, field if count is None else list(field))
        elif count is None:
            _to_ros(field, getattr(ros_message, name), element, layouts)
        else:
            package, short = element.split("/")
            cls = getattr(importlib.import_module(f"{package}.msg"), short)
            items = []
            for item in field:
                made = cls()
                _to_ros(item, made, element, layouts)
                items.append(made)
            setattr(ros_message, name, items)


def _same(value: Any, ros_value: Any, type_name: str, layouts: msgdefs.Layouts, path: str, problems: List[str]) -> None:
    for name, element, count in layouts.fields(type_name):
        ours, theirs = getattr(value, name), getattr(ros_value, name)
        if element in msgdefs.PRIMITIVES or element == "string":
            theirs = theirs if count is None else list(theirs)
            if ours != theirs:
                problems.append(f"{path}.{name}: {ours!r} here, {theirs!r} in ROS")
        elif count is None:
            _same(ours, theirs, element, layouts, f"{path}.{name}", problems)
        else:
            if len(ours) != len(theirs):
                problems.append(f"{path}.{name}: {len(ours)} items here, {len(theirs)} in ROS")
            for index, (a, b) in enumerate(zip(ours, theirs)):
                _same(a, b, element, layouts, f"{path}.{name}[{index}]", problems)


def check_type(type_name: str, layouts: msgdefs.Layouts) -> List[str]:
    """What is wrong with one listed layout against the installed message; an empty list if nothing."""
    from rclpy.serialization import deserialize_message, serialize_message

    package, short = type_name.split("/")
    cls = getattr(importlib.import_module(f"{package}.msg"), short)
    problems = []
    theirs = [(name, *ros_field_text(text)) for name, text in cls.get_fields_and_field_types().items()]
    ours = [(name, element, count) for name, element, count in layouts.fields(type_name)]
    if ours != theirs:
        return [f"fields differ: here {ours}, ROS {theirs}"]
    value = fill(type_name, layouts, [0])
    ros_message = cls()
    _to_ros(value, ros_message, type_name, layouts)
    serialised = bytes(serialize_message(ros_message))
    decoded = cdr.decode(type_name, serialised, layouts)
    if decoded != value:
        _same(decoded, ros_message, type_name, layouts, short, problems)
        problems.append("ROS's bytes do not decode to the values that went in")
    encoded = cdr.encode(type_name, value, layouts)
    back = deserialize_message(encoded, cls)
    _same(value, back, type_name, layouts, short, problems)
    # The bytes themselves are not compared: ROS leaves whatever was in memory in padding, and may add up to
    # three bytes of it after the last field.
    if not 0 <= len(serialised) - len(encoded) <= 3:
        problems.append(f"the message written here is {len(encoded)} bytes and ROS's is {len(serialised)}")
    return problems


def main() -> int:
    layouts = msgdefs.Layouts()
    failed = skipped = 0
    for type_name in sorted(msgdefs.DEFINITIONS):
        try:
            problems = check_type(type_name, layouts)
        except ImportError as error:
            skipped += 1
            print(f"SKIP {type_name}: not installed ({error})")
            continue
        if problems:
            failed += 1
            print(f"FAIL {type_name}")
            for problem in problems[:6]:
                print(f"     {problem}")
        else:
            print(f"ok   {type_name}")
    print(f"{len(msgdefs.DEFINITIONS) - failed - skipped} of {len(msgdefs.DEFINITIONS)} layouts match ROS; {failed} differ; {skipped} not installed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
