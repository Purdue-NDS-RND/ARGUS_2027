"""argus_interfaces: the message and service files say what FEATURES.md (SETUP-1) says.

Other packages will be written against these definitions, so a renamed
field or a changed type is a break in every one of them. The files are
parsed here as text, with no ROS, and compared field by field with the
spec; the package's build files are checked for the pieces rosidl needs.
"""

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parents[1]

PRIMITIVES = {"bool", "byte", "char", "float32", "float64", "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "string", "wstring"}
FIELD = re.compile(r"^(?P<type>[A-Za-z0-9_/]+)(?P<array>\[\d*\])?\s+(?P<name>[A-Za-z][A-Za-z0-9_]*)(?:\s*=\s*(?P<value>\S+))?$")


def parse(text: str) -> list:
    """(type, name) for each field and (type, NAME, value) for each constant, in file order."""
    entries = []
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        match = FIELD.match(line)
        assert match, f"cannot read interface line: {line!r}"
        kind = match["type"] + (match["array"] or "")
        if match["value"] is not None:
            entries.append((kind, match["name"], match["value"]))
        else:
            entries.append((kind, match["name"]))
    return entries


def read(relative: str) -> str:
    return (PACKAGE / relative).read_text(encoding="utf-8")


# --- the definitions, as the spec gives them ---------------------------------------


def test_marker_track():
    assert parse(read("msg/MarkerTrack.msg")) == [
        ("uint8", "TENTATIVE", "0"),
        ("uint8", "CONFIRMED", "1"),
        ("uint8", "COASTING", "2"),
        ("uint8", "LOST", "3"),
        ("std_msgs/Header", "header"),
        ("uint32", "track_id"),
        ("uint8", "state"),
        ("geometry_msgs/Point", "position_enu"),
        ("float64[4]", "covariance_en"),
        ("float32[10]", "digit_log_prob"),
        ("int8", "digit"),
        ("uint32", "sightings"),
        ("bool", "outside"),
    ]


def test_marker_track_array():
    assert parse(read("msg/MarkerTrackArray.msg")) == [("std_msgs/Header", "header"), ("MarkerTrack[]", "tracks")]


def test_stage_timing():
    assert parse(read("msg/StageTiming.msg")) == [("std_msgs/Header", "header"), ("string", "stage"), ("float32", "latency_ms")]


def test_get_crops():
    parts = read("srv/GetCrops.srv").split("\n---\n")
    assert len(parts) == 2, "a service has a request and a response, split by one --- line"
    assert parse(parts[0]) == [("builtin_interfaces/Time", "stamp"), ("sensor_msgs/RegionOfInterest[]", "rois")]
    assert parse(parts[1]) == [("sensor_msgs/Image[]", "crops"), ("bool", "ok"), ("string", "message")]


def test_digit_can_say_unknown_and_the_state_constants_fit_their_field():
    text = read("msg/MarkerTrack.msg")
    entries = parse(text)
    assert ("int8", "digit") in entries  # signed, so -1 can mean unknown
    assert "-1 when unknown" in text
    constants = [entry for entry in entries if len(entry) == 3]
    state_type = next(entry[0] for entry in entries if entry[1] == "state")
    assert all(entry[0] == state_type for entry in constants)
    assert sorted(int(entry[2]) for entry in constants) == [0, 1, 2, 3]


# --- every file is well formed and built -----------------------------------------------

INTERFACES = sorted(path.relative_to(PACKAGE).as_posix() for folder in ("msg", "srv") for path in (PACKAGE / folder).iterdir())


def test_the_package_has_exactly_the_four_interfaces_of_the_spec():
    assert INTERFACES == ["msg/MarkerTrack.msg", "msg/MarkerTrackArray.msg", "msg/StageTiming.msg", "srv/GetCrops.srv"]


@pytest.mark.parametrize("relative", INTERFACES)
def test_types_and_names_are_ones_rosidl_accepts(relative):
    local = {Path(item).stem for item in INTERFACES if item.startswith("msg/")}
    for part in read(relative).split("\n---\n"):
        names = []
        for entry in parse(part):
            base = re.sub(r"\[\d*\]$", "", entry[0])
            assert base in PRIMITIVES or base in local or re.fullmatch(r"[a-z][a-z0-9_]*/[A-Z][A-Za-z0-9]*", base), entry
            if len(entry) == 3:
                assert re.fullmatch(r"[A-Z][A-Z0-9_]*", entry[1]), f"constant {entry[1]} must be upper case"
            else:
                assert re.fullmatch(r"[a-z][a-z0-9_]*", entry[1]) and "__" not in entry[1] and not entry[1].endswith("_"), entry
            names.append(entry[1])
        assert len(names) == len(set(names)), f"{relative}: a name is used twice"
    assert Path(relative).stem[0].isupper() and "_" not in Path(relative).stem  # CamelCase file names


def packages_used() -> set:
    used = set()
    for relative in INTERFACES:
        for part in read(relative).split("\n---\n"):
            used |= {entry[0].split("/")[0] for entry in parse(part) if "/" in entry[0]}
    return used


def test_cmake_generates_every_interface_and_names_every_dependency():
    cmake = read("CMakeLists.txt")
    block = re.search(r"rosidl_generate_interfaces\(\$\{PROJECT_NAME\}(.*?)\)", cmake, re.DOTALL)
    assert block, "CMakeLists.txt must call rosidl_generate_interfaces"
    listed = re.findall(r'"((?:msg|srv)/[^"]+)"', block.group(1))
    assert sorted(listed) == INTERFACES
    dependencies = set(block.group(1).split("DEPENDENCIES", 1)[1].split())
    assert dependencies == packages_used() == {"builtin_interfaces", "geometry_msgs", "sensor_msgs", "std_msgs"}
    for package in sorted(packages_used() | {"ament_cmake", "rosidl_default_generators"}):
        assert f"find_package({package} REQUIRED)" in cmake
    assert "project(argus_interfaces)" in cmake and "ament_package()" in cmake


def test_package_xml_is_an_ament_cmake_interface_package():
    root = ET.fromstring(read("package.xml"))
    assert root.findtext("name") == "argus_interfaces"
    assert root.findtext("export/build_type") == "ament_cmake"
    assert "rosidl_interface_packages" in [item.text for item in root.findall("member_of_group")]
    buildtools = {item.text for item in root.findall("buildtool_depend")}
    assert {"ament_cmake", "rosidl_default_generators"} <= buildtools
    assert {item.text for item in root.findall("depend")} == packages_used()
    assert "rosidl_default_runtime" in {item.text for item in root.findall("exec_depend")}
