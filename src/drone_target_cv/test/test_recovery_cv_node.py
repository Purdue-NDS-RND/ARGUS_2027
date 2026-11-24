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


# --- visual homing in the node (REC-7b) ---------------------------------------------------------------------

POSE_TOPIC = "/mavros/local_position/pose"
STATE_TOPIC = "/mavros/extended_state"
GROUND_PX_PER_M = 80.0  # the made-up ground's own scale
GROUND_MIDDLE = (1200.0, 1200.0)  # the local frame's origin in it


def pose_message(stamp_ns, position, yaw_rad=0.0):
    half = 0.5 * yaw_rad
    return types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)),
        pose=types.SimpleNamespace(position=types.SimpleNamespace(x=position[0], y=position[1], z=position[2]), orientation=types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))),
    )


def seen_from(ground_picture, position, yaw_rad=0.0):
    """What the sim's downward camera sees of a flat picture of the ground from a pose: rendered through the node's own camera model and mount."""
    from drone_mission_core import projection_v2
    from drone_mission_core.frame_projection import load_camera

    camera, mount = load_camera("sim_delivery", "sim_iris_downcam")
    orientation = (0.0, 0.0, math.sin(0.5 * yaw_rad), math.cos(0.5 * yaw_rad))
    corners = [(0.0, 0.0), (WIDTH - 1.0, 0.0), (WIDTH - 1.0, HEIGHT - 1.0), (0.0, HEIGHT - 1.0)]
    on_ground = []
    for corner in corners:
        point = projection_v2.project(camera, mount, corner, position, orientation, 0.0)
        on_ground.append((GROUND_MIDDLE[0] + point.east_m * GROUND_PX_PER_M, GROUND_MIDDLE[1] - point.north_m * GROUND_PX_PER_M))
    warp = cv2.getPerspectiveTransform(np.float32(on_ground), np.float32(corners))
    grey = cv2.warpPerspective(ground_picture, warp, (WIDTH, HEIGHT), flags=cv2.INTER_AREA, borderValue=110)
    pixel_of = lambda east, north: projection_v2.pixel_of_ground_point(camera, mount, (east, north, 0.0), position, orientation)  # noqa: E731
    return cv2.cvtColor(grey, cv2.COLOR_GRAY2BGR), pixel_of


def homing_node(ros, **flags):
    with_vision_msgs(ros)
    ros.setitem(sys.modules, "mavros_msgs.msg", types.SimpleNamespace(ExtendedState=object))
    return node_with(ros, **{"features.recovery_cv": True, **flags})


def test_the_homing_flags_are_off_by_default_and_then_nothing_of_homing_is_loaded_or_subscribed(ros):
    for module in ("drone_target_cv.home_marker", "drone_target_cv.keyframe_homing"):
        ros.delitem(sys.modules, module, raising=False)
    node = homing_node(ros)
    assert node.declared["features.marker_homing"] is False and node.declared["features.keyframe_homing"] is False and node.declared["home_marker_id"] == 0
    assert set(node.subscriptions) == {IMAGE_TOPIC} and "drone_target_cv.home_marker" not in sys.modules and "drone_target_cv.keyframe_homing" not in sys.modules
    image, _truth = mat()
    node.subscriptions[IMAGE_TOPIC](image_message(image))
    assert {detection.class_id for detection in contract.decode(node.publishers[TOPIC].sent[-1]).detections} == {"green_x", "bucket"}


def test_with_marker_homing_on_the_home_marker_goes_out_in_a_message_of_its_own_after_the_frames_x_and_bucket(ros):
    from drone_target_cv import home_marker

    if not home_marker.available():
        pytest.skip("this OpenCV has no aruco module with the AprilTag 36h11 dictionary")

    node = homing_node(ros, **{"features.marker_homing": True})
    assert set(node.subscriptions) == {IMAGE_TOPIC}  # the marker needs no pose
    ground_picture = np.full((2400, 2400), 60, dtype=np.uint8)
    tag = home_marker.picture(0, cell_px=8)  # 0.8 m across the border at 80 px a metre, with its white margin
    place = (2.0, -1.0)  # east, north of the tag's centre
    left, top = int(GROUND_MIDDLE[0] + place[0] * GROUND_PX_PER_M) - 40, int(GROUND_MIDDLE[1] - place[1] * GROUND_PX_PER_M) - 40
    ground_picture[top : top + 80, left : left + 80] = tag
    image, pixel_of = seen_from(ground_picture, (1.2, -0.4, 5.0), yaw_rad=0.6)
    node.subscriptions[IMAGE_TOPIC](image_message(image))
    first, second = (contract.decode(message) for message in node.publishers[TOPIC].sent[-2:])
    assert first.detections == () and first.stamp_ns == second.stamp_ns  # the frame's own message first (no X and no bucket in this picture), then the marker's
    found = second.detections
    assert [detection.class_id for detection in found] == ["home_marker"] and len(node.publishers[HANDLE_TOPIC].sent) == 1
    expected = pixel_of(*place)
    assert math.hypot(found[0].cx - expected[0], found[0].cy - expected[1]) < 2.0 and found[0].score == home_marker.SCORE
    assert found[0].width == pytest.approx(0.8 * FOCAL_PX / 5.0, rel=0.45) and target_estimator.plane_height_m("home_marker") == 0.0
    # The marker is looked for at most thirty times a second: a frame 20 ms after the last is not looked at for it.
    sent = len(node.publishers[TOPIC].sent)
    node.subscriptions[IMAGE_TOPIC](image_message(image, nanosec=721_000_000))
    assert len(node.publishers[TOPIC].sent) == sent + 1
    node.subscriptions[IMAGE_TOPIC](image_message(image, nanosec=821_000_000))
    assert len(node.publishers[TOPIC].sent) == sent + 3 and node.slowest_homing_ms > 0.0
    # Another id is not the home marker.
    other = homing_node(ros, **{"features.marker_homing": True, "home_marker_id": 7})
    other.subscriptions[IMAGE_TOPIC](image_message(image))
    assert [contract.decode(message).detections for message in other.publishers[TOPIC].sent] == [()]
    # An OpenCV without the aruco module: said once, the marker is not looked for, and the X and the bucket go on.
    ros.setattr(home_marker, "available", lambda: False)
    without = homing_node(ros, **{"features.marker_homing": True})
    assert "has no aruco module" in without.logger.error.call_args[0][0] and without._marker_id is None
    mat_image, _truth = mat()
    without.subscriptions[IMAGE_TOPIC](image_message(mat_image))
    assert {detection.class_id for detection in contract.decode(without.publishers[TOPIC].sent[-1]).detections} == {"green_x", "bucket"}


def test_with_keyframe_homing_on_the_climbs_pictures_are_kept_and_the_takeoff_point_is_found_on_the_way_back(ros):
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import test_home_finders as finders

    node = homing_node(ros, **{"features.keyframe_homing": True})
    assert set(node.subscriptions) == {IMAGE_TOPIC, POSE_TOPIC, STATE_TOPIC}
    ground_picture = finders.ground(seed=5, size=2400)
    home = (0.5, -0.3)
    clock = [100 * 1_000_000_000]

    def at(position, yaw_rad=0.0, frame=True, landed=False):
        clock[0] += 400_000_000  # (two and a half frames a second: under the three a second the keyframes are matched at)
        node.subscriptions[STATE_TOPIC](types.SimpleNamespace(landed_state=1 if landed else 2))
        for back_ms in (60, 30, 0):
            node.subscriptions[POSE_TOPIC](pose_message(clock[0] - back_ms * 1_000_000, position, yaw_rad))
        if not frame:
            return None, None
        # The frame comes 20 ms before the pose of its moment, as it does on a busy computer: it is held, and answered when the pose comes.
        image, pixel_of = seen_from(ground_picture, position, yaw_rad)
        frame_ns = clock[0] + 20_000_000
        sent_before = len(node.publishers[TOPIC].sent)
        node.subscriptions[IMAGE_TOPIC](image_message(image, sec=frame_ns // 1_000_000_000, nanosec=frame_ns % 1_000_000_000))
        assert [detection.class_id for detection in contract.decode(node.publishers[TOPIC].sent[-1]).detections] == [] and len(node._held) == 1
        assert sent_before + 1 == len(node.publishers[TOPIC].sent)  # the frame's own message went out at once, before any homing
        for ahead_ms in (10, 40):
            node.subscriptions[POSE_TOPIC](pose_message(clock[0] + ahead_ms * 1_000_000, position, yaw_rad))
        assert node._held == []
        homing = [contract.decode(message) for message in node.publishers[TOPIC].sent[sent_before + 1 :]]
        assert all(found.stamp_ns == frame_ns and found.frame_id == "down_camera_optical" for found in homing) and len(homing) <= 1
        return (homing[0].detections if homing else ()), pixel_of

    for _ in range(4):  # in the air before any landing has been seen (a node started in flight): no takeoff point, whatever the height
        at((home[0] + 5.0, home[1], 0.05), frame=False)
    assert node._climb.home is None
    for _ in range(8):  # standing on the ground, by the flight controller's own word
        at((home[0], home[1], 0.1), frame=False, landed=True)
    assert node._climb.home is not None and node._climb.takeoffs == 1
    for step in range(1, 60):  # straight up to 14.9 m, a quarter of a metre a frame
        found, _pixel_of = at((home[0], home[1], 0.1 + 0.25 * step))
        assert found == ()  # the climb's pictures are kept, and nothing is reported from them
    assert node.counts["keyframes kept"] == 4 and all(abs(keyframe.height_m - wanted) <= 0.75 for keyframe, wanted in zip(node._keyframes.keyframes, (4.0, 7.0, 10.0, 14.0)))
    at((6.0, 4.0, 15.0), frame=False)  # away
    assert node._climb.left
    found, pixel_of = at((home[0] + 1.6, home[1] - 0.9, 5.1), yaw_rad=0.8)  # and back, carrying, from another heading
    assert [detection.class_id for detection in found] == ["takeoff_point"] and node.counts["takeoff point found"] == 1
    assert target_estimator.plane_height_m("takeoff_point") == 0.0
    expected = pixel_of(*home)
    assert math.hypot(found[0].cx - expected[0], found[0].cy - expected[1]) < 4.0  # pixels: 0.05 m on the ground from 5 m
    # Far from the takeoff point nothing is looked for.
    found, _pixel_of = at((home[0] + 30.0, home[1], 5.0))
    assert found == () and node.counts["takeoff point found"] == 1
    # Frames whose poses never come are not kept for ever.
    for index in range(6):
        late = clock[0] + (index + 1) * 1_000_000_000
        node.subscriptions[IMAGE_TOPIC](image_message(np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8), sec=late // 1_000_000_000, nanosec=late % 1_000_000_000))
    assert len(node._held) <= 3 and node.counts["frames with no pose for keyframe homing"] >= 3
