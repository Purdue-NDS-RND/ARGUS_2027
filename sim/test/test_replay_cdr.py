"""The replay harness's message layouts and CDR codec (SIM-09). No ROS."""

import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

from replay import cdr, msgdefs, topics  # noqa: E402
from replay.cdr import Message  # noqa: E402


def header(sec=12, nanosec=345, frame_id="map"):
    return Message(stamp=Message(sec=sec, nanosec=nanosec), frame_id=frame_id)


def test_a_point_stamped_is_the_bytes_ros_writes():
    # By hand: the CDR header; sec, nanosec; the string's length with its closing zero, "cam", the zero;
    # then the doubles, aligned to 8 from the end of the CDR header.
    expected = (
        b"\x00\x01\x00\x00"
        + struct.pack("<iI", 7, 9)
        + struct.pack("<I", 4)
        + b"cam\x00"
        + struct.pack("<ddd", 0.25, -0.5, 1200.0)
    )
    message = Message(header=header(7, 9, "cam"), point=Message(x=0.25, y=-0.5, z=1200.0))
    assert cdr.encode("geometry_msgs/msg/PointStamped", message) == expected
    assert cdr.decode("geometry_msgs/msg/PointStamped", expected) == message
    assert cdr.stamp_ns(cdr.decode("geometry_msgs/PointStamped", expected).header) == 7_000_000_009


def test_numbers_are_aligned_to_their_own_size_after_a_string():
    # frame_id "ab": length 3, then 3 bytes; the next float64 starts at the next multiple of 8.
    data = cdr.encode("geometry_msgs/msg/PointStamped", Message(header=header(1, 2, "ab"), point=Message(x=1.5, y=0.0, z=0.0)))
    body = data[4:]
    assert body[8:12] == struct.pack("<I", 3) and body[12:15] == b"ab\x00"
    assert body[15:16] == b"\x00" and struct.unpack("<d", body[16:24])[0] == 1.5
    # A uint8 then a float32 (sensor_msgs/Range): three bytes of padding between them.
    ranged = cdr.encode("sensor_msgs/msg/Range", Message(header=header(0, 0, ""), radiation_type=1, field_of_view=0.5, min_range=0.1, max_range=40.0, range=12.5))
    body = ranged[4:]
    assert body[8:12] == struct.pack("<I", 1) and body[12:13] == b"\x00"  # the empty string is its zero byte alone
    assert body[13:14] == b"\x01" and body[14:16] == b"\x00\x00" and struct.unpack("<f", body[16:20])[0] == 0.5


def test_a_detection_array_round_trips_with_its_sequences_and_nested_messages():
    hypothesis = Message(
        hypothesis=Message(class_id="bullseye", score=0.875),
        pose=Message(pose=Message(position=Message(x=0.0, y=0.0, z=0.0), orientation=Message(x=0.0, y=0.0, z=0.0, w=1.0)), covariance=[0.0] * 36),
    )
    detection = Message(
        header=header(100, 5, "down_camera_optical"),
        results=[hypothesis],
        bbox=Message(center=Message(position=Message(x=320.5, y=180.25), theta=0.0), size_x=40.0, size_y=38.0),
        id="",
    )
    array = Message(header=header(100, 5, "down_camera_optical"), detections=[detection, detection])
    data = cdr.encode("vision_msgs/msg/Detection2DArray", array)
    back = cdr.decode("vision_msgs/msg/Detection2DArray", data)
    assert back == array and back.detections[1].results[0].hypothesis.class_id == "bullseye"
    assert back.detections[0].bbox.center.position.x == 320.5 and len(back.detections[0].results[0].pose.covariance) == 36


def test_padding_ros_adds_after_the_last_field_is_read_past():
    roi = cdr.encode("sensor_msgs/msg/RegionOfInterest", {"x_offset": 1, "do_rectify": True})
    assert len(roi) == 4 + 17 and roi[-1:] == b"\x01"  # four numbers and one byte: no padding written here
    as_ros_writes_it = roi + b"\x5a\x00\x7f"  # ROS rounds this one up to 24 bytes, with whatever was in memory
    assert cdr.decode("sensor_msgs/msg/RegionOfInterest", as_ros_writes_it) == cdr.decode("sensor_msgs/msg/RegionOfInterest", roi)


def test_what_is_missing_is_written_as_zero_and_other_objects_and_dicts_are_taken():
    data = cdr.encode("geometry_msgs/msg/PoseStamped", {"header": {"stamp": {"sec": 3}}, "pose": SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=3.0))})
    back = cdr.decode("geometry_msgs/msg/PoseStamped", data)
    assert (back.header.stamp.sec, back.header.stamp.nanosec, back.header.frame_id) == (3, 0, "")
    assert (back.pose.position.y, back.pose.orientation.w) == (2.0, 0.0)
    info = cdr.decode("sensor_msgs/msg/CameraInfo", cdr.encode("sensor_msgs/msg/CameraInfo", {"width": 640, "height": 360, "d": [0.1, 0.2], "k": [1.0] * 9}))
    assert (info.width, info.height, info.d, len(info.k), len(info.p), info.roi.do_rectify) == (640, 360, [0.1, 0.2], 9, 12, False)


def test_every_recorded_type_is_known_and_round_trips_empty():
    layouts = msgdefs.Layouts()
    for topic, type_name in sorted(topics.RECORDED.items()):
        assert layouts.known(type_name), f"{topic}: {type_name}"
        assert cdr.decode(type_name, cdr.encode(type_name, {})) == cdr.decode(type_name, cdr.encode(type_name, Message()))
    assert layouts.known("std_msgs/msg/Header")  # the frame taps' type


def test_bytes_that_are_not_the_message_are_refused():
    good = cdr.encode("geometry_msgs/msg/PointStamped", {})
    with pytest.raises(cdr.CdrError, match="ends at byte"):
        cdr.decode("geometry_msgs/msg/PointStamped", good[:-8])
    with pytest.raises(cdr.CdrError, match="left over"):
        cdr.decode("geometry_msgs/msg/PointStamped", good + b"\x00" * 16)
    with pytest.raises(cdr.CdrError, match="CDR header"):
        cdr.decode("geometry_msgs/msg/PointStamped", b"\x07\x07")
    with pytest.raises(cdr.CdrError, match="no layout for message type"):
        cdr.decode("nav_msgs/msg/Odometry", good)
    with pytest.raises(cdr.CdrError, match="holds 9 values"):
        cdr.encode("sensor_msgs/msg/CameraInfo", {"k": [0.0] * 10})


def test_big_endian_is_read():
    big = b"\x00\x00\x00\x00" + struct.pack(">iI", 7, 9) + struct.pack(">I", 1) + b"\x00" + b"\x00" * 3 + struct.pack(">ddd", 1.0, 2.0, 3.0)
    assert cdr.decode("geometry_msgs/msg/PointStamped", big).point.z == 3.0


def test_a_layout_a_bag_brings_with_it_is_read_from_its_schema_text():
    schema = "# a comment\nstd_msgs/Header header\nuint8 KIND_A=1\nuint8 kind\nfloat32[] values\nPart[2] parts\nstring<=8 label\n================\nMSG: my_msgs/Part\nint16 a\nbool ok\n"
    layouts = msgdefs.Layouts()
    layouts.add(msgdefs.parse_ros2msg("my_msgs/msg/Thing", schema))
    assert layouts.fields("my_msgs/msg/Thing") == [
        ("header", "std_msgs/Header", None),
        ("kind", "uint8", None),
        ("values", "float32", -1),
        ("parts", "my_msgs/Part", 2),
        ("label", "string", None),
    ]
    thing = Message(header=header(), kind=1, values=[0.5, 1.5], parts=[Message(a=-3, ok=True), Message(a=4, ok=False)], label="x")
    assert cdr.decode("my_msgs/msg/Thing", cdr.encode("my_msgs/msg/Thing", thing, layouts), layouts) == thing
    # A layout listed here is not replaced by a bag's: the listed ones are the checked ones.
    layouts.add({"geometry_msgs/Point": [("x", "float32", None)]})
    assert layouts.fields("geometry_msgs/msg/Point")[0] == ("x", "float64", None)


def test_topic_names():
    assert topics.contract_topic("target_cv") == "/perception/target_cv/detections"
    assert topics.frame_tap_topic("/camera/image") == "/replay/frame_seen/camera/image"
    recorded = topics.record_topics()
    assert "/mavros/local_position/pose" in recorded and "/estimator/targets" in recorded and "/replay/frame_seen/target_cv/annotated" in recorded
    assert "/camera/image" not in recorded and len(recorded) == len(set(recorded))
    sys.path.insert(0, str(SIM.parent / "src" / "drone_mission_core"))
    from drone_mission_core import detection_contract, estimator_feed, frame_time

    assert topics.contract_topic("target_cv") == detection_contract.topic_for("target_cv") and topics.ESTIMATOR_TARGETS == estimator_feed.TARGETS_TOPIC
    assert (topics.POSE, topics.VELOCITY, topics.RANGEFINDER) == (frame_time.POSE_TOPIC, frame_time.VELOCITY_TOPIC, frame_time.RANGEFINDER_TOPIC)
    assert topics.LEGACY_IMAGE_SIZE == estimator_feed.LEGACY_IMAGE_SIZE_TOPIC
