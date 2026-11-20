"""The ring fit through its real node class (CEN-13), with its flag on and off.

rclpy and the message packages are the stand-ins of test_contract_node.py, so
this runs without ROS. The fit is the real one, on views rendered by
tools/synth_targets.py; the camera's mount is the sim Iris's, from
argus_camera's robot.yaml.
"""

import math
import sys
import types
from pathlib import Path

import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "drone_target_cv"), str(SRC / "drone_mission_core"), str(SRC / "argus_camera"), str(SRC.parent / "tools"), str(Path(__file__).resolve().parent)]

import test_contract_node as stand_ins  # noqa: E402
from drone_mission_core import detection_contract as contract  # noqa: E402
from test_contract_node import image_header, ros, with_vision_msgs  # noqa: E402, F401  (ros is a fixture)
from test_ring_fit import FOCAL_PX, HEIGHT, WIDTH, view  # noqa: E402

TOPIC = "/perception/ring_fit/detections"
IMAGE_TOPIC, POSE_TOPIC = "camera/image", "/mavros/local_position/pose"
# The sim Iris's mount (robot.yaml, image_yaw_deg 0): the image's right is the body's right, its top the nose.
BODY_FROM_OPTICAL = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


class RecordingNode(stand_ins.FakeNode):
    """FakeNode, remembering the QoS each subscription was made with."""

    def __init__(self, name):
        super().__init__(name)
        self.qos = {}

    def create_subscription(self, message_type, topic, callback, qos):
        self.qos[topic] = qos
        return super().create_subscription(message_type, topic, callback, qos)


class QoSProfile:
    def __init__(self, **settings):
        self.settings = settings


def node_with(monkeypatch, **parameters):
    monkeypatch.setitem(sys.modules, "rclpy.node", types.SimpleNamespace(Node=RecordingNode))
    monkeypatch.setitem(sys.modules, "rclpy.qos", types.SimpleNamespace(QoSProfile=QoSProfile, HistoryPolicy=types.SimpleNamespace(KEEP_LAST="keep_last"), ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT="best_effort"), qos_profile_sensor_data="sensor_data"))
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", types.SimpleNamespace(PointStamped=stand_ins.PointStamped, PoseStamped=object))
    monkeypatch.delitem(sys.modules, "drone_target_cv.ring_fit_node", raising=False)
    monkeypatch.setattr(stand_ins.FakeNode, "overrides", {"camera_mode": "sim_delivery", "airframe": "sim_iris_downcam", **parameters})
    from drone_target_cv import ring_fit_node

    return ring_fit_node.RingFitNode()


def quaternion_of(rotation):
    """A unit quaternion (x, y, z, w) for a rotation matrix whose trace is positive enough (small tilts, any heading near east)."""
    w = math.sqrt(max(0.0, 1.0 + rotation[0, 0] + rotation[1, 1] + rotation[2, 2])) / 2.0
    return ((rotation[2, 1] - rotation[1, 2]) / (4 * w), (rotation[0, 2] - rotation[2, 0]) / (4 * w), (rotation[1, 0] - rotation[0, 1]) / (4 * w), w)


def pose(ns, up_m, world_from_body=np.eye(3)):
    x, y, z, w = quaternion_of(np.asarray(world_from_body))
    stamp = types.SimpleNamespace(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)
    return types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=stamp),
        pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.0, y=0.0, z=up_m), orientation=types.SimpleNamespace(x=x, y=y, z=z, w=w)),
    )


def image_message(image, sec=184, nanosec=701_000_000, frame_id="down_camera_optical", encoding="bgr8"):
    data = image if encoding == "bgr8" else image[:, :, ::-1]
    return types.SimpleNamespace(header=image_header(sec, nanosec, frame_id), height=image.shape[0], width=image.shape[1], encoding=encoding, data=np.ascontiguousarray(data).tobytes())


STAMP_NS = 184_701_000_000


def flying(node, up_m=2.4, world_from_body=np.eye(3), around_ns=STAMP_NS):
    for offset_ms in (-66, -33, 0, 33):
        node.subscriptions[POSE_TOPIC](pose(around_ns + offset_ms * 1_000_000, up_m, world_from_body))


def test_flag_off_the_node_does_nothing_and_needs_nothing(ros):
    node = node_with(ros)
    assert node.declared["features.ring_fit"] is False
    assert node.subscriptions == {} and node.publishers == {}
    for module in ("vision_msgs", "argus_camera.robot_description"):
        assert module not in sys.modules, module
    node._on_image(None)  # and an image, were one to arrive, is not looked at


def test_flag_on_images_are_taken_with_a_queue_of_one_and_the_centre_goes_out_on_the_contract(ros):
    with_vision_msgs(ros)
    node = node_with(ros, **{"features.ring_fit": True})
    assert set(node.subscriptions) == {IMAGE_TOPIC, POSE_TOPIC} and set(node.publishers) == {TOPIC}
    assert node.qos[IMAGE_TOPIC].settings == {"depth": 1, "history": "keep_last", "reliability": "best_effort"}  # the newest frame, nothing queued behind it

    image, truth = view(2.4, offset_px=(120.0, -60.0))
    flying(node)
    node.subscriptions[IMAGE_TOPIC](image_message(image))
    found = contract.decode(node.publishers[TOPIC].sent[-1])
    assert (found.stamp_ns, found.frame_id) == (STAMP_NS, "down_camera_optical")  # the image's own stamp and frame
    (bullseye,) = found.detections
    assert bullseye.class_id == "bullseye" and math.hypot(bullseye.cx - truth[0], bullseye.cy - truth[1]) < 1.5

    node.subscriptions[IMAGE_TOPIC](image_message(image, encoding="rgb8"))
    again = contract.decode(node.publishers[TOPIC].sent[-1]).detections[0]
    assert math.hypot(again.cx - truth[0], again.cy - truth[1]) < 1.5

    plain = np.full((HEIGHT, WIDTH, 3), (60, 140, 60), dtype=np.uint8)
    node.subscriptions[IMAGE_TOPIC](image_message(plain, nanosec=734_000_000))
    nothing = contract.decode(node.publishers[TOPIC].sent[-1])
    assert nothing.detections == () and nothing.stamp_ns == 184_734_000_000  # looked, found nothing
    assert node.counts == {"frames": 3, "looked at": 3, "rings found": 2}


def test_a_tilted_vehicles_view_is_levelled_with_its_attitude_at_the_frames_time(ros):
    with_vision_msgs(ros)
    # The renderer tilts the ground under a camera looking down; seen from the ground the camera, and so the vehicle, is tilted.
    tilt = (0.0, -8.0)
    tx, ty = math.radians(tilt[0]), math.radians(tilt[1])
    rx = np.array([[1, 0, 0], [0, math.cos(tx), -math.sin(tx)], [0, math.sin(tx), math.cos(tx)]])
    ry = np.array([[math.cos(ty), 0, math.sin(ty)], [0, 1, 0], [-math.sin(ty), 0, math.cos(ty)]])
    world_from_optical = (np.diag([1.0, -1.0, -1.0]) @ (ry @ rx)).T
    world_from_body = world_from_optical @ BODY_FROM_OPTICAL.T
    image, truth = view(1.5, offset_px=(60.0, 40.0), tilt_deg=tilt)

    node = node_with(ros, **{"features.ring_fit": True})
    flying(node, up_m=1.5, world_from_body=world_from_body)
    node.subscriptions[IMAGE_TOPIC](image_message(image))
    (levelled,) = contract.decode(node.publishers[TOPIC].sent[-1]).detections
    assert math.hypot(levelled.cx - truth[0], levelled.cy - truth[1]) < 1.5

    # The same frame with the vehicle reported level: fitted as it is, and declined or worse.
    wrong = node_with(ros, **{"features.ring_fit": True})
    flying(wrong, up_m=1.5)
    wrong.subscriptions[IMAGE_TOPIC](image_message(image))
    unlevelled = contract.decode(wrong.publishers[TOPIC].sent[-1]).detections
    assert unlevelled == () or math.hypot(unlevelled[0].cx - truth[0], unlevelled[0].cy - truth[1]) > math.hypot(levelled.cx - truth[0], levelled.cy - truth[1])


def test_the_node_does_not_look_above_four_metres_or_without_a_pose_of_the_frames_moment(ros):
    with_vision_msgs(ros)
    image, _truth = view(2.4)
    node = node_with(ros, **{"features.ring_fit": True})
    node.subscriptions[IMAGE_TOPIC](image_message(image))
    assert node.publishers[TOPIC].sent == []  # no pose: not flying
    flying(node, up_m=4.2)
    node.subscriptions[IMAGE_TOPIC](image_message(image))
    assert node.publishers[TOPIC].sent == [] and node.declared["max_height_m"] == 4.0
    flying(node, up_m=3.9, around_ns=STAMP_NS + 5_000_000_000)  # five seconds on, lower
    node.subscriptions[IMAGE_TOPIC](image_message(image, sec=189))
    assert len(node.publishers[TOPIC].sent) == 1
    # The pose stream stalls: a frame 0.2 s after the last pose has neither a height nor a tilt, and is not looked at.
    node.subscriptions[IMAGE_TOPIC](image_message(image, sec=189, nanosec=701_000_000 + 233_000_000))
    assert len(node.publishers[TOPIC].sent) == 1
    assert node.counts["no pose of the frame's moment"] == 2 and node.counts["above the height limit"] == 1


def test_a_frame_that_is_not_the_calibrated_modes_size_is_not_used(ros):
    with_vision_msgs(ros)
    image, _truth = view(2.4)
    import cv2

    node = node_with(ros, **{"features.ring_fit": True})
    flying(node)
    node.subscriptions[IMAGE_TOPIC](image_message(cv2.resize(image, (1280, 720))))  # the contract's pixels are the calibrated mode's
    assert node.publishers[TOPIC].sent == [] and node.counts["not the calibrated size"] == 1 and node.logger.error.called


def test_every_ten_seconds_of_frames_the_node_says_what_it_did(ros):
    with_vision_msgs(ros)
    image, _truth = view(2.4)
    node = node_with(ros, **{"features.ring_fit": True})
    for second in range(0, 12):
        flying(node, around_ns=(184 + second) * 1_000_000_000 + 701_000_000)
        node.subscriptions[IMAGE_TOPIC](image_message(image, sec=184 + second))
    said = [call.args[0] for call in node.logger.info.call_args_list if "frames so far" in call.args[0]]
    assert len(said) == 1 and "11 frames so far, looked at 10, rings found in 10" in said[0]  # said as the eleventh frame arrives


def test_an_image_without_a_stamp_or_a_frame_or_in_an_unknown_encoding_is_not_used(ros):
    with_vision_msgs(ros)
    image, _truth = view(2.4)
    node = node_with(ros, **{"features.ring_fit": True})
    flying(node)
    node.subscriptions[IMAGE_TOPIC](image_message(image, sec=0, nanosec=0))
    node.subscriptions[IMAGE_TOPIC](image_message(image, frame_id=""))
    node.subscriptions[IMAGE_TOPIC](image_message(image, encoding="mono16"))
    assert node.publishers[TOPIC].sent == []


def test_flag_on_with_a_mount_that_cannot_be_read_the_node_says_so_and_stays_out_of_the_way(ros):
    with_vision_msgs(ros)
    for parameters in ({"airframe": ""}, {"camera_mode": ""}, {"airframe": "argus_2027"}, {"camera_mode": "imx519_720p60"}):  # none; none; a sim camera on the aircraft; a mode with a stand-in calibration
        node = node_with(ros, **{"features.ring_fit": True, **parameters})
        assert node.publishers == {} or node._publisher is None
        assert node.logger.error.called
        image, _truth = view(2.4)
        if IMAGE_TOPIC in node.subscriptions:
            node.subscriptions[IMAGE_TOPIC](image_message(image))  # must not raise


def test_the_estimators_figure_for_this_detectors_noise_is_the_ring_fits_own():
    from drone_mission_core import estimator_feed
    from drone_target_cv import ring_fit

    assert estimator_feed.DETECTOR_PIXEL_SIGMA_PX[ring_fit.DETECTOR_NAME] == ring_fit.PIXEL_SIGMA_PX
    assert estimator_feed.RING_FIT_DETECTOR == ring_fit.DETECTOR_NAME and contract.topic_for(ring_fit.DETECTOR_NAME) == TOPIC
