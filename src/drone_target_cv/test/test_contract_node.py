"""The detection contract through the real node classes (FND-02): target_cv with its flag on and off, and the legacy adapter.

rclpy and the message packages are replaced by stand-ins, as the executor's
tests do, so this runs without ROS. The vision_msgs stand-ins have only the
real field names (test_detection_contract.py). The detector is the real one,
on a drawn bullseye.
"""

import sys
import types
from pathlib import Path
from unittest import mock

import cv2
import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "drone_mission_core"), str(SRC / "drone_mission_core" / "test")]

from drone_mission_core import detection_contract as contract  # noqa: E402
from test_detection_contract import FakeMessages, Header, Message, message  # noqa: E402

Point = message("Point", x=float, y=float, z=float)
PointStamped = message("PointStamped", header=Header, point=Point)
CameraInfo = message("CameraInfo", header=Header, width=int, height=int)
CONTRACT_TOPIC = "/perception/target_cv/detections"
TARGET_TOPIC = "/drone_package_drop/target_detection"
SIZE_TOPIC = "/drone_package_drop/image_size"


class Publisher:
    def __init__(self, topic):
        self.topic, self.sent = topic, []

    def publish(self, item):
        self.sent.append(item)


class FakeNode:
    """What the two nodes use of rclpy.node.Node, with parameters set by the test."""

    overrides: dict = {}
    publishers_elsewhere = 0  # other nodes publishing the 2026 target topic

    def __init__(self, name):
        self.name, self.declared, self.publishers, self.subscriptions = name, {}, {}, {}
        self.logger = mock.MagicMock()

    def declare_parameter(self, name, default=None):
        self.declared[name] = default

    def get_parameter(self, name):
        assert name in self.declared, f"{name} read before it was declared"
        value = self.overrides.get(name, self.declared[name])
        kinds = types.SimpleNamespace(bool_value=bool(value), string_value=str(value), double_value=float(value) if isinstance(value, (int, float)) else 0.0)
        return types.SimpleNamespace(value=value, get_parameter_value=lambda: kinds)

    def create_publisher(self, message_type, topic, qos):
        self.publishers[topic] = Publisher(topic)
        return self.publishers[topic]

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscriptions[topic] = callback
        return topic

    def count_publishers(self, topic):
        return (1 if topic in self.publishers else 0) + self.publishers_elsewhere

    def get_logger(self):
        return self.logger

    def get_clock(self):
        now = Header().stamp
        now.sec, now.nanosec = 999, 5
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: now, nanoseconds=999_000_000_005))

    def __getattr__(self, name):  # create_service, create_timer, destroy_subscription
        if name.startswith(("create_", "destroy_")):
            return mock.MagicMock()
        raise AttributeError(name)


@pytest.fixture
def ros(monkeypatch):
    """Stand-ins for ROS; vision_msgs is absent until a test puts it there."""
    stand_ins = {
        "rclpy": mock.MagicMock(),
        "rclpy.node": types.SimpleNamespace(Node=FakeNode),
        "rclpy.qos": mock.MagicMock(),
        "geometry_msgs": mock.MagicMock(),
        "geometry_msgs.msg": types.SimpleNamespace(PointStamped=PointStamped),
        "sensor_msgs": mock.MagicMock(),
        "sensor_msgs.msg": types.SimpleNamespace(CompressedImage=mock.MagicMock(), Image=mock.MagicMock(), CameraInfo=CameraInfo),
        "std_srvs": mock.MagicMock(),
        "std_srvs.srv": mock.MagicMock(),
        "builtin_interfaces": mock.MagicMock(),
        "builtin_interfaces.msg": types.SimpleNamespace(Time=lambda sec, nanosec: types.SimpleNamespace(sec=sec, nanosec=nanosec)),
    }
    for name, module in stand_ins.items():
        monkeypatch.setitem(sys.modules, name, module)
    for name in [n for n in sys.modules if n.split(".")[0] in ("vision_msgs",) or n in ("drone_target_cv.target_cv", "drone_target_cv.legacy_adapter", "drone_target_cv.contract_output")]:
        monkeypatch.delitem(sys.modules, name)
    monkeypatch.setattr(FakeNode, "overrides", {})
    monkeypatch.setattr(FakeNode, "publishers_elsewhere", 0)
    return monkeypatch


def with_vision_msgs(monkeypatch):
    monkeypatch.setitem(sys.modules, "vision_msgs", types.SimpleNamespace(msg=FakeMessages))
    monkeypatch.setitem(sys.modules, "vision_msgs.msg", FakeMessages)


def detector_node(monkeypatch, **parameters):
    monkeypatch.setattr(FakeNode, "overrides", {"start_enabled": False, **parameters})
    from drone_target_cv import target_cv

    return target_cv.TargetCV()


def bullseye_frame(centre=(400, 200), radius=60):
    image = np.full((360, 640, 3), (60, 140, 60), dtype=np.uint8)
    for index, ring in enumerate(range(radius, 0, -radius // 5)):
        cv2.circle(image, centre, ring, (30, 30, 220) if index % 2 == 0 else (240, 240, 240), -1)
    return image


def image_header(sec=184, nanosec=701_000_000, frame_id="down_camera_optical"):
    header = Header()
    header.stamp.sec, header.stamp.nanosec, header.frame_id = sec, nanosec, frame_id
    return header


def test_flag_off_the_node_makes_no_contract_publisher_and_needs_no_vision_msgs(ros):
    node = detector_node(ros)  # vision_msgs is not importable here: flag off, the node must not try
    assert node.declared["features.detection_contract"] is False
    assert CONTRACT_TOPIC not in node.publishers and set(node.publishers) == {TARGET_TOPIC, SIZE_TOPIC}
    node._process_frame(bullseye_frame(), image_header(), 640, 360)
    point = node.publishers[TARGET_TOPIC].sent[-1]
    assert abs(point.point.x - 0.25) < 0.02 and abs(point.point.y - (200 / 360 * 2 - 1)) < 0.02 and point.point.z > 0.0
    assert (point.header.stamp.sec, point.header.frame_id) == (999, "camera")  # 2026: this node's clock, frame "camera"
    assert "vision_msgs" not in sys.modules and "drone_target_cv.contract_output" not in sys.modules


def test_flag_on_each_frame_goes_out_on_the_contract_and_the_2026_topics_are_unchanged(ros):
    with_vision_msgs(ros)
    plain = detector_node(ros)
    plain._process_frame(bullseye_frame(), image_header(), 640, 360)
    node = detector_node(ros, **{"features.detection_contract": True})
    assert set(node.publishers) == {TARGET_TOPIC, SIZE_TOPIC, CONTRACT_TOPIC}
    node._process_frame(bullseye_frame(), image_header(), 640, 360)

    found = contract.decode(node.publishers[CONTRACT_TOPIC].sent[-1])  # real field names, and inside the contract
    assert (found.stamp_ns, found.frame_id) == (184_701_000_000, "down_camera_optical")  # the image's, not the node's clock
    (bullseye,) = found.detections
    assert bullseye.class_id == "bullseye" and 0.0 < bullseye.score <= 1.0
    assert abs(bullseye.cx - 400) <= 3 and abs(bullseye.cy - 200) <= 3 and 100 <= bullseye.width <= 130

    for topic in (TARGET_TOPIC, SIZE_TOPIC):  # what the missions read is what the flag-off node sends
        ours, theirs = node.publishers[topic].sent, plain.publishers[topic].sent
        assert [(m.point.x, m.point.y, m.point.z, m.header.stamp.sec, m.header.frame_id) for m in ours] == [
            (m.point.x, m.point.y, m.point.z, m.header.stamp.sec, m.header.frame_id) for m in theirs
        ]

    node._process_frame(np.full((360, 640, 3), (60, 140, 60), dtype=np.uint8), image_header(nanosec=734_000_000), 640, 360)
    nothing = contract.decode(node.publishers[CONTRACT_TOPIC].sent[-1])
    assert nothing.detections == () and nothing.stamp_ns == 184_734_000_000  # looked, found nothing
    assert node.publishers[TARGET_TOPIC].sent[-1].point.x == -1.0


def test_flag_on_an_image_with_no_stamp_or_frame_is_not_published_on_the_contract_but_still_on_the_2026_topic(ros):
    with_vision_msgs(ros)
    node = detector_node(ros, **{"features.detection_contract": True})
    node._process_frame(bullseye_frame(), image_header(sec=0, nanosec=0), 640, 360)
    node._process_frame(bullseye_frame(), image_header(frame_id=""), 640, 360)
    assert node.publishers[CONTRACT_TOPIC].sent == [] and node.logger.error.call_count == 2
    assert len(node.publishers[TARGET_TOPIC].sent) == 2 and node.publishers[TARGET_TOPIC].sent[-1].point.x != -1.0


def test_flag_on_a_failure_on_the_contract_side_does_not_stop_the_2026_detection(ros):
    with_vision_msgs(ros)
    node = detector_node(ros, **{"features.detection_contract": True})
    node._contract_pub.publish = mock.MagicMock(side_effect=RuntimeError("publisher gone"))
    node._process_frame(bullseye_frame(), image_header(), 640, 360)
    assert node.publishers[TARGET_TOPIC].sent[-1].point.x != -1.0
    assert "Detection contract output failed" in str(node.logger.error.call_args)


def test_flag_on_without_vision_msgs_the_node_still_starts_as_the_2026_detector(ros):
    node = detector_node(ros, **{"features.detection_contract": True})  # vision_msgs cannot be imported here
    assert CONTRACT_TOPIC not in node.publishers and "running as the 2026 detector" in str(node.logger.error.call_args)
    node._process_frame(bullseye_frame(), image_header(), 640, 360)
    assert node.publishers[TARGET_TOPIC].sent[-1].point.x != -1.0


# --- the adapter ---


def adapter_node(monkeypatch, **parameters):
    with_vision_msgs(monkeypatch)
    monkeypatch.setattr(FakeNode, "overrides", parameters)
    from drone_target_cv import legacy_adapter

    return legacy_adapter.LegacyAdapter()


def camera_info(width=3840, height=2160):
    info = CameraInfo()
    info.width, info.height = width, height
    return info


def contract_message(*detections, stamp_ns=184_701_000_000):
    return contract.encode(contract.DetectionSet(stamp_ns, "down_camera_optical", tuple(detections)), FakeMessages)


def test_the_adapter_feeds_the_2026_topics_from_a_detector_that_only_speaks_the_contract(ros):
    node = adapter_node(ros, detector="ring_fit")
    assert set(node.subscriptions) == {"/perception/ring_fit/detections", "/camera/camera_info"}
    on_detections = node.subscriptions["/perception/ring_fit/detections"]
    on_detections(contract_message(contract.Detection("bullseye", 0.9, 2880.0, 540.0, 300.0, 280.0)))
    assert node.publishers[TARGET_TOPIC].sent == []  # the image size is not known yet
    node.subscriptions["/camera/camera_info"](camera_info())
    on_detections(contract_message(contract.Detection("marker", 0.99, 10.0, 10.0, 5.0, 5.0), contract.Detection("bullseye", 0.9, 2880.0, 540.0, 300.0, 280.0)))
    point, size = node.publishers[TARGET_TOPIC].sent[-1], node.publishers[SIZE_TOPIC].sent[-1]
    assert (point.point.x, point.point.y) == (0.5, -0.5) and point.header.frame_id == "camera"
    assert (point.header.stamp.sec, point.header.stamp.nanosec) == (184, 701_000_000)  # the frame's time, carried through
    assert (size.point.x, size.point.y) == (3840.0, 2160.0)
    on_detections(contract_message(stamp_ns=184_734_000_000))  # looked, found nothing
    assert (node.publishers[TARGET_TOPIC].sent[-1].point.x, node.publishers[TARGET_TOPIC].sent[-1].point.y) == (-1.0, -1.0)
    on_detections(contract_message(contract.Detection("bullseye", 0.2, 2880.0, 540.0, 300.0, 280.0), stamp_ns=184_767_000_000))
    assert node.publishers[TARGET_TOPIC].sent[-1].point.x == -1.0  # under the score floor: nothing found
    assert node.declared["min_score"] == 0.25


def test_the_adapter_does_not_pass_on_a_detection_stamped_ahead_of_its_clock(ros):
    node = adapter_node(ros, detector="ring_fit")  # its clock reads 999 s
    node.subscriptions["/camera/camera_info"](camera_info())
    ahead = contract_message(contract.Detection("bullseye", 0.9, 2880.0, 540.0, 300.0, 280.0), stamp_ns=1_794_966_718 * 1_000_000_000)
    node.subscriptions["/perception/ring_fit/detections"](ahead)
    assert node.publishers[TARGET_TOPIC].sent == [] and "ahead of this node's clock" in str(node.logger.error.call_args)


def test_the_adapter_has_no_default_detector_and_stays_silent_beside_another_publisher(ros):
    with pytest.raises(ValueError, match="which detector"):
        adapter_node(ros)
    node = adapter_node(ros, detector="target_cv")
    node.subscriptions["/camera/camera_info"](camera_info(640, 360))
    ros.setattr(FakeNode, "publishers_elsewhere", 1)  # the 2026 detector is publishing the 2026 topic itself
    node.subscriptions[CONTRACT_TOPIC](contract_message(contract.Detection("bullseye", 0.9, 400.0, 200.0, 120.0, 120.0)))
    assert node.publishers[TARGET_TOPIC].sent == [] and "not publishing beside it" in str(node.logger.error.call_args)
    ros.setattr(FakeNode, "publishers_elsewhere", 0)
    node.subscriptions[CONTRACT_TOPIC](contract_message(contract.Detection("bullseye", 0.9, 400.0, 200.0, 120.0, 120.0)))
    assert len(node.publishers[TARGET_TOPIC].sent) == 1


def test_the_adapter_refuses_a_message_from_outside_the_contract(ros):
    node = adapter_node(ros, detector="ring_fit")
    node.subscriptions["/camera/camera_info"](camera_info())
    outside = contract_message(contract.Detection("bullseye", 0.9, 400.0, 200.0, 120.0, 120.0))
    outside.detections[0].results[0].hypothesis.class_id = "7"  # a numeric id, as the 2026 YOLO node sends
    node.subscriptions["/perception/ring_fit/detections"](outside)
    assert node.publishers[TARGET_TOPIC].sent == [] and "outside the detection contract" in str(node.logger.error.call_args)
