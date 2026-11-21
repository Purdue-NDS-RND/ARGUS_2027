"""The green X and bucket detector through its real node class (REC-1), with its flag on and off.

rclpy and the message packages are the stand-ins of test_contract_node.py, so
this runs without ROS. The detector is the real one, on mats rendered by
tools/synth_targets.py at the sim camera's size.
"""

import math
import sys
import types
from pathlib import Path

import cv2
import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "drone_target_cv"), str(SRC / "drone_mission_core"), str(SRC / "argus_camera"), str(SRC.parent / "tools"), str(Path(__file__).resolve().parent)]

import synth_targets  # noqa: E402
import test_contract_node as stand_ins  # noqa: E402
from drone_mission_core import detection_contract as contract  # noqa: E402
from drone_mission_core import target_estimator  # noqa: E402
from test_contract_node import image_header, ros, with_vision_msgs  # noqa: E402, F401  (ros is a fixture)
from test_ring_fit_node import QoSProfile, RecordingNode, image_message  # noqa: E402

TOPIC, HANDLE_TOPIC = "/perception/recovery_cv/detections", "/perception/recovery_cv/handle"
IMAGE_TOPIC = "camera/image"
WIDTH, HEIGHT, FOCAL_PX = 640, 360, 389.0  # the sim's delivery camera (sim_delivery.yaml)


class Point32:
    def __init__(self):
        self.x = self.y = self.z = 0.0


class PolygonStamped:
    def __init__(self):
        self.header = image_header(0, 0, "")
        self.polygon = types.SimpleNamespace(points=[])


def node_with(monkeypatch, **parameters):
    monkeypatch.setitem(sys.modules, "rclpy.node", types.SimpleNamespace(Node=RecordingNode))
    qos = types.SimpleNamespace(QoSProfile=QoSProfile, HistoryPolicy=types.SimpleNamespace(KEEP_LAST="keep_last"), ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT="best_effort"), qos_profile_sensor_data="sensor_data")
    monkeypatch.setitem(sys.modules, "rclpy.qos", qos)
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", types.SimpleNamespace(PointStamped=stand_ins.PointStamped, PoseStamped=object, PolygonStamped=PolygonStamped, Point32=Point32))
    for module in ("drone_target_cv.recovery_cv_node", "drone_target_cv.ring_fit_node"):
        monkeypatch.delitem(sys.modules, module, raising=False)
    monkeypatch.setattr(stand_ins.FakeNode, "overrides", {"camera_mode": "sim_delivery", "airframe": "sim_iris_downcam", **parameters})
    from drone_target_cv import recovery_cv_node

    return recovery_cv_node.RecoveryCvNode()


def mat(height_m=6.0, bucket=True, offset=(50.0, -30.0), rotation_deg=20.0, size=(WIDTH, HEIGHT)):
    options = synth_targets.RenderOptions(width=size[0], height=size[1], mpp=height_m / FOCAL_PX, rotation_deg=rotation_deg, offset_px=offset, background="noise", supersample=2)
    return synth_targets.render(synth_targets.XMat(bucket=bucket), options, seed=2)


def test_flag_off_the_node_does_nothing_and_needs_nothing(ros):
    ros.delitem(sys.modules, "drone_target_cv.green_x", raising=False)  # (another test file in the same run may have loaded the detector)
    node = node_with(ros)
    assert node.declared["features.recovery_cv"] is False
    assert node.subscriptions == {} and node.publishers == {}
    for module in ("vision_msgs", "argus_camera.robot_description", "drone_target_cv.green_x"):
        assert module not in sys.modules, module
    node._on_image(None)  # and an image, were one to arrive, is not looked at


def test_flag_on_the_x_and_the_bucket_go_out_on_the_contract_from_the_newest_frame(ros):
    with_vision_msgs(ros)
    node = node_with(ros, **{"features.recovery_cv": True})
    assert node.logger.error.call_args is None
    assert set(node.subscriptions) == {IMAGE_TOPIC} and set(node.publishers) == {TOPIC, HANDLE_TOPIC}
    assert node.qos[IMAGE_TOPIC].settings == {"depth": 1, "history": "keep_last", "reliability": "best_effort"}  # the lead's wave 3 note: a queue of one

    image, truth = mat()
    node.subscriptions[IMAGE_TOPIC](image_message(image))
    found = contract.decode(node.publishers[TOPIC].sent[-1])
    assert (found.stamp_ns, found.frame_id) == (184_701_000_000, "down_camera_optical")  # the image's own stamp and frame
    by_class = {detection.class_id: detection for detection in found.detections}
    assert set(by_class) == {"green_x", "bucket"}
    x, bucket = by_class["green_x"], by_class["bucket"]
    assert math.hypot(x.cx - truth["center_px"][0], x.cy - truth["center_px"][1]) <= 2.0 and x.score >= 0.5
    assert math.hypot(bucket.cx - truth["center_px"][0], bucket.cy - truth["center_px"][1]) <= 3.0
    # The X's box holds its arms; the bucket's is a bucket's size. Each class has its own plane height for whoever projects it.
    assert x.width == x.height and 0.6 * 6.57 * FOCAL_PX / 6.0 <= x.width <= 1.15 * 6.57 * FOCAL_PX / 6.0
    assert bucket.width == pytest.approx(0.2 * FOCAL_PX / 6.0, rel=0.4)
    assert (target_estimator.plane_height_m("green_x"), target_estimator.plane_height_m("bucket")) == (0.0, 0.2)
    assert node.counts["looked at"] == 1 and node.counts["x found"] == 1 and node.counts["bucket found"] == 1
    # No handle on a drawn bucket this small: the handle's message goes out all the same, with the frame's stamp and no points.
    (no_handle,) = node.publishers[HANDLE_TOPIC].sent
    assert no_handle.polygon.points == [] and (no_handle.header.stamp.sec, no_handle.header.stamp.nanosec) == (184, 701_000_000)
    assert node.slowest_ms > 0.0

    # A frame with nothing in it still gives a message: looked, and saw nothing.
    grass = np.full((HEIGHT, WIDTH, 3), (70, 130, 80), dtype=np.uint8)
    node.subscriptions[IMAGE_TOPIC](image_message(grass, nanosec=801_000_000))
    empty = contract.decode(node.publishers[TOPIC].sent[-1])
    assert empty.detections == () and empty.stamp_ns == 184_801_000_000 and node.counts["looked at"] == 2 and node.counts["x found"] == 1
    assert len(node.publishers[HANDLE_TOPIC].sent) == 2 and node.publishers[HANDLE_TOPIC].sent[-1].header.stamp.nanosec == 801_000_000


def test_a_handle_that_shows_goes_out_as_its_two_ends_with_the_images_stamp(ros):
    with_vision_msgs(ros)
    node = node_with(ros, **{"features.recovery_cv": True})
    image, truth = mat(height_m=3.0, offset=(0.0, 0.0), rotation_deg=0.0)  # close in: the bucket is 26 px across
    cx, cy = truth["center_px"]
    radius = 0.1 * FOCAL_PX / 3.0
    cv2.line(image, (int(cx - radius), int(cy)), (int(cx + radius), int(cy)), (25, 25, 25), 3)
    node.subscriptions[IMAGE_TOPIC](image_message(image))
    (handle,) = node.publishers[HANDLE_TOPIC].sent  # one message for the one frame
    assert (handle.header.stamp.sec, handle.header.stamp.nanosec, handle.header.frame_id) == (184, 701_000_000, "down_camera_optical")
    (a, b) = handle.polygon.points
    assert abs(a.y - cy) <= 3.0 and abs(b.y - cy) <= 3.0 and abs(abs(b.x - a.x) - 2.0 * radius) <= 0.5 * radius


def test_what_the_node_cannot_use_is_said_and_not_published(ros):
    with_vision_msgs(ros)
    # No camera named: the node says so and stays out of the way.
    unnamed = node_with(ros, **{"features.recovery_cv": True, "camera_mode": ""})
    assert unnamed.subscriptions == {} and unnamed.publishers == {} and "camera_mode and airframe must both be set" in unnamed.logger.error.call_args[0][0]
    node = node_with(ros, **{"features.recovery_cv": True})
    # A frame that is not the calibrated mode's size: its pixels would not be the contract's.
    big, _ = mat(size=(1280, 720))
    node.subscriptions[IMAGE_TOPIC](image_message(big))
    assert node.publishers[TOPIC].sent == [] and node.counts["not the calibrated size"] == 1 and "calibrated at 640 x 360" in node.logger.error.call_args[0][0]
    # No stamp, an encoding it cannot read: not used.
    image, _ = mat()
    node.subscriptions[IMAGE_TOPIC](image_message(image, sec=0, nanosec=0))
    node.subscriptions[IMAGE_TOPIC](image_message(image, encoding="mono8"))
    assert node.publishers[TOPIC].sent == [] and node.counts["looked at"] == 0
