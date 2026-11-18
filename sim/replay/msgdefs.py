"""The ROS 2 message layouts the replay harness reads and writes (SIM-09). No ROS imports.

A bag stores each message as CDR bytes and, for the sqlite3 storage, only
the type's name: the layout has to come from somewhere. These are the
layouts of the types the recorder's topics carry, as ROS 2 Humble has them
(vision_msgs 4.1, mavros_msgs 2.x), written the way a .msg file is: one
"type name" per line, `[]` for a sequence, `[N]` for a fixed array.

`sim/replay/check_defs.py` compares every layout here with the installed
message packages, and round-trips a message of each through ROS's own
serialiser, wherever ROS is available (the sim container, the aircraft).

An MCAP bag carries its own layouts (ros2msg schemas); `parse_ros2msg`
reads those, so a type that is not listed here can still be decoded from
an MCAP file.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

PRIMITIVES = {
    # name: (struct format character, size in bytes)
    "bool": ("?", 1),
    "byte": ("B", 1),
    "char": ("B", 1),
    "uint8": ("B", 1),
    "int8": ("b", 1),
    "uint16": ("H", 2),
    "int16": ("h", 2),
    "uint32": ("I", 4),
    "int32": ("i", 4),
    "uint64": ("Q", 8),
    "int64": ("q", 8),
    "float32": ("f", 4),
    "float64": ("d", 8),
}

DEFINITIONS: Dict[str, str] = {
    "builtin_interfaces/Time": "int32 sec\nuint32 nanosec",
    "std_msgs/Header": "builtin_interfaces/Time stamp\nstring frame_id",
    "rosgraph_msgs/Clock": "builtin_interfaces/Time clock",
    "geometry_msgs/Point": "float64 x\nfloat64 y\nfloat64 z",
    "geometry_msgs/Vector3": "float64 x\nfloat64 y\nfloat64 z",
    "geometry_msgs/Quaternion": "float64 x\nfloat64 y\nfloat64 z\nfloat64 w",
    "geometry_msgs/Pose": "geometry_msgs/Point position\ngeometry_msgs/Quaternion orientation",
    "geometry_msgs/PoseStamped": "std_msgs/Header header\ngeometry_msgs/Pose pose",
    "geometry_msgs/PoseWithCovariance": "geometry_msgs/Pose pose\nfloat64[36] covariance",
    "geometry_msgs/Twist": "geometry_msgs/Vector3 linear\ngeometry_msgs/Vector3 angular",
    "geometry_msgs/TwistStamped": "std_msgs/Header header\ngeometry_msgs/Twist twist",
    "geometry_msgs/PointStamped": "std_msgs/Header header\ngeometry_msgs/Point point",
    "sensor_msgs/Range": "std_msgs/Header header\nuint8 radiation_type\nfloat32 field_of_view\nfloat32 min_range\nfloat32 max_range\nfloat32 range",
    "sensor_msgs/Imu": (
        "std_msgs/Header header\ngeometry_msgs/Quaternion orientation\nfloat64[9] orientation_covariance\n"
        "geometry_msgs/Vector3 angular_velocity\nfloat64[9] angular_velocity_covariance\n"
        "geometry_msgs/Vector3 linear_acceleration\nfloat64[9] linear_acceleration_covariance"
    ),
    "sensor_msgs/NavSatStatus": "int8 status\nuint16 service",
    "sensor_msgs/NavSatFix": (
        "std_msgs/Header header\nsensor_msgs/NavSatStatus status\nfloat64 latitude\nfloat64 longitude\nfloat64 altitude\n"
        "float64[9] position_covariance\nuint8 position_covariance_type"
    ),
    "sensor_msgs/RegionOfInterest": "uint32 x_offset\nuint32 y_offset\nuint32 height\nuint32 width\nbool do_rectify",
    "sensor_msgs/CameraInfo": (
        "std_msgs/Header header\nuint32 height\nuint32 width\nstring distortion_model\nfloat64[] d\nfloat64[9] k\nfloat64[9] r\nfloat64[12] p\n"
        "uint32 binning_x\nuint32 binning_y\nsensor_msgs/RegionOfInterest roi"
    ),
    "mavros_msgs/State": "std_msgs/Header header\nbool connected\nbool armed\nbool guided\nbool manual_input\nstring mode\nuint8 system_status",
    "mavros_msgs/ExtendedState": "std_msgs/Header header\nuint8 vtol_state\nuint8 landed_state",
    "mavros_msgs/RCOut": "std_msgs/Header header\nuint16[] channels",
    "mavros_msgs/PositionTarget": (
        "std_msgs/Header header\nuint8 coordinate_frame\nuint16 type_mask\ngeometry_msgs/Point position\ngeometry_msgs/Vector3 velocity\n"
        "geometry_msgs/Vector3 acceleration_or_force\nfloat32 yaw\nfloat32 yaw_rate"
    ),
    "mavros_msgs/GlobalPositionTarget": (
        "std_msgs/Header header\nuint8 coordinate_frame\nuint16 type_mask\nfloat64 latitude\nfloat64 longitude\nfloat32 altitude\n"
        "geometry_msgs/Vector3 velocity\ngeometry_msgs/Vector3 acceleration_or_force\nfloat32 yaw\nfloat32 yaw_rate"
    ),
    "vision_msgs/Point2D": "float64 x\nfloat64 y",
    "vision_msgs/Pose2D": "vision_msgs/Point2D position\nfloat64 theta",
    "vision_msgs/BoundingBox2D": "vision_msgs/Pose2D center\nfloat64 size_x\nfloat64 size_y",
    "vision_msgs/BoundingBox3D": "geometry_msgs/Pose center\ngeometry_msgs/Vector3 size",
    "vision_msgs/ObjectHypothesis": "string class_id\nfloat64 score",
    "vision_msgs/ObjectHypothesisWithPose": "vision_msgs/ObjectHypothesis hypothesis\ngeometry_msgs/PoseWithCovariance pose",
    "vision_msgs/Detection2D": "std_msgs/Header header\nvision_msgs/ObjectHypothesisWithPose[] results\nvision_msgs/BoundingBox2D bbox\nstring id",
    "vision_msgs/Detection2DArray": "std_msgs/Header header\nvision_msgs/Detection2D[] detections",
    "vision_msgs/Detection3D": "std_msgs/Header header\nvision_msgs/ObjectHypothesisWithPose[] results\nvision_msgs/BoundingBox3D bbox\nstring id",
    "vision_msgs/Detection3DArray": "std_msgs/Header header\nvision_msgs/Detection3D[] detections",
}

Field = Tuple[str, str, Optional[int]]  # (name, element type, None for one value, -1 for a sequence, N for a fixed array)


class DefinitionError(ValueError):
    """A message type whose layout is not known or cannot be read. The message says which."""


def normalise(type_name: str) -> str:
    """"geometry_msgs/msg/PoseStamped" and "geometry_msgs/PoseStamped" are the same type: the short form is used here."""
    parts = str(type_name).split("/")
    if len(parts) == 3 and parts[1] == "msg":
        return f"{parts[0]}/{parts[2]}"
    return str(type_name)


def parse_fields(text: str, package: str = "") -> List[Field]:
    """The fields of one .msg text. Comments, blank lines and constants are skipped; a bare type name is in `package`."""
    fields: List[Field] = []
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) >= 2 and ("=" in parts[1] or (len(parts) > 2 and parts[2].startswith("="))):
            continue  # a constant: "uint8 FOO=1" or "uint8 FOO = 1"
        if len(parts) < 2:
            raise DefinitionError(f"cannot read the field line {raw!r}")
        type_text, name = parts[0], parts[1]
        count: Optional[int] = None
        if type_text.endswith("]"):
            type_text, _, inside = type_text[:-1].partition("[")
            if inside.startswith("<="):
                count = -1  # a bounded sequence is written as a sequence
            else:
                count = int(inside) if inside else -1
        if type_text.startswith("string<=") or type_text.startswith("wstring"):
            type_text = "string" if type_text.startswith("string") else type_text
        if type_text not in PRIMITIVES and type_text != "string":
            if type_text == "Header":
                type_text = "std_msgs/Header"
            elif "/" not in type_text:
                type_text = f"{package}/{type_text}" if package else type_text
            type_text = normalise(type_text)
        fields.append((name, type_text, count))
    return fields


def parse_ros2msg(type_name: str, schema_text: str) -> Dict[str, List[Field]]:
    """The layouts in an MCAP ros2msg schema: the type's own text, then each type it uses after a line of "=" and "MSG: pkg/Type"."""
    layouts: Dict[str, List[Field]] = {}
    name = normalise(type_name)
    block: List[str] = []

    def close(current: str, lines: List[str]) -> None:
        layouts[current] = parse_fields("\n".join(lines), current.split("/")[0])

    for line in schema_text.splitlines():
        if line.startswith("===="):
            continue
        if line.startswith("MSG: "):
            close(name, block)
            name, block = normalise(line[5:].strip()), []
            continue
        block.append(line)
    close(name, block)
    return layouts


class Layouts:
    """Every known layout, parsed once: the ones listed here, plus any a bag brought with it."""

    def __init__(self) -> None:
        self._fields: Dict[str, List[Field]] = {}

    def add(self, layouts: Dict[str, List[Field]]) -> None:
        """Layouts read from a bag's own schemas. One already known is kept: the listed ones are checked against ROS."""
        for name, fields in layouts.items():
            if normalise(name) not in DEFINITIONS:
                self._fields.setdefault(normalise(name), fields)

    def known(self, type_name: str) -> bool:
        name = normalise(type_name)
        return name in self._fields or name in DEFINITIONS

    def fields(self, type_name: str) -> List[Field]:
        name = normalise(type_name)
        if name not in self._fields:
            if name not in DEFINITIONS:
                raise DefinitionError(f"no layout for message type {type_name} (sim/replay/msgdefs.py lists the ones the recorder's topics use)")
            self._fields[name] = parse_fields(DEFINITIONS[name], name.split("/")[0])
        return self._fields[name]
