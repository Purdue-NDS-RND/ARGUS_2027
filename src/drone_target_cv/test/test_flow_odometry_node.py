"""Optical-flow odometry through its real node class (REC-5), with its flag on and off.

rclpy and the message packages are the stand-ins of test_contract_node.py, so this runs without ROS. The flow is
the real one, on ground pictures made by test_flow_odometry.py at the sim's recovery flow camera's size.
"""

import json
import math
import sys
import types
from pathlib import Path

import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "drone_target_cv"), str(SRC / "drone_mission_core"), str(SRC / "argus_camera"), str(Path(__file__).resolve().parent)]

import test_contract_node as stand_ins  # noqa: E402
import test_flow_odometry as pictures  # noqa: E402
from test_contract_node import image_header, ros  # noqa: E402, F401  (ros is a fixture)
from test_ring_fit_node import QoSProfile, RecordingNode, pose  # noqa: E402

IMAGE_TOPIC, POSE_TOPIC, RANGE_TOPIC = "/argus/camera/flow/image_raw", "/mavros/local_position/pose", "/mavros/rangefinder/rangefinder"
VELOCITY, VELOCITY_ENU, STATUS, SEND = "/argus/flow/velocity", "/argus/flow/velocity_enu", "/argus/flow/status", "/mavros/optical_flow/raw/send"
T0 = 184_000_000_000
FRAME_NS = 50_000_000  # the sim's recovery flow camera: 20 Hz


class Vector3:
    def __init__(self):
        self.x = self.y = self.z = 0.0


class TwistStamped:
    def __init__(self):
        self.header = image_header(0, 0, "")
        self.twist = types.SimpleNamespace(linear=Vector3(), angular=Vector3())


class OpticalFlow:
    def __init__(self):
        self.header = image_header(0, 0, "")
        self.flow, self.flow_comp_m, self.flow_rate = Vector3(), Vector3(), Vector3()
        self.quality, self.ground_distance = 0, 0.0


class String:
    def __init__(self):
        self.data = ""


def node_with(monkeypatch, **parameters):
    monkeypatch.setitem(sys.modules, "rclpy.node", types.SimpleNamespace(Node=RecordingNode))
    qos = types.SimpleNamespace(QoSProfile=QoSProfile, HistoryPolicy=types.SimpleNamespace(KEEP_LAST="keep_last"), ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT="best_effort"), qos_profile_sensor_data="sensor data")
    monkeypatch.setitem(sys.modules, "rclpy.qos", qos)
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", types.SimpleNamespace(PointStamped=stand_ins.PointStamped, PoseStamped=object, TwistStamped=TwistStamped))
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", types.SimpleNamespace(Image=object, Range=object, CameraInfo=stand_ins.CameraInfo, CompressedImage=object))
    monkeypatch.setitem(sys.modules, "mavros_msgs", types.SimpleNamespace())
    monkeypatch.setitem(sys.modules, "mavros_msgs.msg", types.SimpleNamespace(OpticalFlow=OpticalFlow))
    monkeypatch.setitem(sys.modules, "std_msgs", types.SimpleNamespace())
    monkeypatch.setitem(sys.modules, "std_msgs.msg", types.SimpleNamespace(String=String))
    for module in ("drone_target_cv.flow_odometry_node", "drone_target_cv.ring_fit_node"):
        monkeypatch.delitem(sys.modules, module, raising=False)
    monkeypatch.setattr(stand_ins.FakeNode, "overrides", {"camera_mode": "sim_flow_recovery", "airframe": "sim_iris_recovery", **parameters})
    from drone_target_cv import flow_odometry_node

    return flow_odometry_node.FlowOdometryNode()


def mono(picture, stamp_ns, encoding="mono8"):
    image = np.clip(picture, 0, 255).astype(np.uint8)
    return types.SimpleNamespace(header=image_header(stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000, "flow_camera_optical"), height=image.shape[0], width=image.shape[1], step=image.shape[1],
                                 encoding=encoding, data=image.tobytes())  # fmt: skip


def rangefinder(stamp_ns, metres, least=0.2, most=12.0):
    return types.SimpleNamespace(header=image_header(stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000, "rangefinder"), range=metres, min_range=least, max_range=most)


def fly(node, frames, height_m=10.0, world_from_body=np.eye(3), ranges=True):
    """Feed the node poses, rangefinder readings and frames in the order a flight gives them."""
    for index, picture in enumerate(frames):
        stamp_ns = T0 + index * FRAME_NS
        for offset_ms in (-40, -20, 0, 20):
            node.subscriptions[POSE_TOPIC](pose(stamp_ns + offset_ms * 1_000_000, height_m, world_from_body))
        if ranges:
            node.subscriptions[RANGE_TOPIC](rangefinder(stamp_ns - 10_000_000, height_m))
        node.subscriptions[IMAGE_TOPIC](mono(picture, stamp_ns))


def test_the_node_keeps_nothing_under_a_name_rclpys_own_node_uses():
    """Found in the sim: the node kept its publishers in `_publishers`, which is rclpy's own list, and could not start."""
    import re

    source = (SRC / "drone_target_cv" / "drone_target_cv" / "flow_odometry_node.py").read_text()
    taken = {"_publishers", "_subscriptions", "_clients", "_services", "_timers", "_guards", "_waitables", "_parameters", "_clock", "_logger", "_executor", "_context", "_handle"}
    assert not {name for name in re.findall(r"self\.(_[a-z_]+)\s*(?::[^=]+)?=", source)} & taken


def test_flag_off_the_node_does_nothing_and_needs_nothing(ros):
    ros.delitem(sys.modules, "drone_target_cv.flow_odometry", raising=False)
    node = node_with(ros)
    assert node.declared["features.flow_odometry"] is False and node.subscriptions == {} and node.publishers == {}
    for module in ("argus_camera.robot_description", "drone_target_cv.flow_odometry", "drone_mission_core.pose_buffer"):
        assert module not in sys.modules, module
    node._on_image(None)  # an image, were one to arrive, is not looked at


def test_flag_on_a_flight_forward_goes_out_as_velocity_status_and_optical_flow_stamped_with_the_frame(ros):
    quiet = node_with(ros, **{"features.flow_odometry": True})
    assert SEND not in quiet.publishers and quiet.declared["send_optical_flow"] is False  # nothing goes to the flight controller unless a params file asks
    node = node_with(ros, **{"features.flow_odometry": True, "send_optical_flow": True})
    assert node.logger.error.call_args is None
    assert set(node.subscriptions) == {IMAGE_TOPIC, POSE_TOPIC, RANGE_TOPIC} and set(node.publishers) == {VELOCITY, VELOCITY_ENU, STATUS, SEND}
    assert node.qos[IMAGE_TOPIC].settings["depth"] == 1  # the newest frame only
    field = pictures.ground()
    focal = 233.87135
    per_frame = 2.0 * focal / 10.0 * 0.05  # 2 m/s forward at 10 m and 20 Hz: 2.34 px a frame towards the tail
    fly(node, [pictures.view(field, dy=per_frame * index) for index in range(8)])
    velocity = node.publishers[VELOCITY].sent
    assert len(velocity) == 4 and velocity[-1].header.frame_id == "base_link"  # from the fifth frame on
    assert velocity[-1].twist.linear.x == pytest.approx(2.0, abs=0.1) and velocity[-1].twist.linear.y == pytest.approx(0.0, abs=0.1)
    stamp = velocity[-1].header.stamp
    assert stamp.sec * 1_000_000_000 + stamp.nanosec == T0 + 7 * FRAME_NS
    enu = node.publishers[VELOCITY_ENU].sent[-1]
    assert enu.header.frame_id == "map" and enu.twist.linear.x == pytest.approx(2.0, abs=0.1) and enu.twist.linear.y == pytest.approx(0.0, abs=0.1)  # level and heading east
    status = json.loads(node.publishers[STATUS].sent[-1].data)
    # REC-6 reads this status to know that source set 2 is fed: "sent" is the node's send_optical_flow, and the feed's own reader takes it.
    from drone_mission_core import gps_health

    assert status["sent"] is True and gps_health.FLOW_STATUS_TOPIC == STATUS
    feed = gps_health.GpsHealthFeed()
    feed.on_flow_status(node.publishers[STATUS].sent[-1].data)
    assert feed.flow_sent(status["stamp_ns"] * 1.0e-9) is (status["quality"] >= gps_health.VISION_MIN_QUALITY) and status["quality"] >= gps_health.VISION_MIN_QUALITY
    held_back = node_with(ros, **{"features.flow_odometry": True})
    assert held_back.declared["send_optical_flow"] is False
    assert status["quality"] > 100 and status["why"] == "" and status["height_m"] == pytest.approx(10.0) and status["dt_s"] == pytest.approx(0.2) and status["patches"] == 0
    sent = node.publishers[SEND].sent[-1]
    assert sent.quality == status["quality"] and sent.ground_distance == pytest.approx(10.0) and sent.flow.y == pytest.approx(4 * per_frame, abs=0.35)
    assert sent.flow_comp_m.x == pytest.approx(2.0, abs=0.1) and sent.header.frame_id == "flow_camera_optical"
    assert sent.flow_comp_m.y == pytest.approx(velocity[-1].twist.linear.y, abs=1e-6)  # forward and left, the same as the twist
    assert node.counts == {"frames": 8, "samples": 4, "with quality": 4}


def test_the_heading_and_the_tilt_are_the_pose_historys_at_the_frames_own_time(ros):
    node = node_with(ros, **{"features.flow_odometry": True})
    field = pictures.ground()
    north = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # the nose to the north
    fly(node, [pictures.view(field, dy=2.0 * index) for index in range(8)], world_from_body=north)
    enu = node.publishers[VELOCITY_ENU].sent[-1]
    assert enu.twist.linear.y > 1.5 and abs(enu.twist.linear.x) < 0.1  # forward is north
    from drone_target_cv import flow_odometry_node

    attitude = flow_odometry_node.attitude_from_quaternion((0.0, math.sin(-0.05), 0.0, math.cos(-0.05)))  # 0.1 rad about the body's left, backwards: nose up
    assert attitude.pitch_rad == pytest.approx(0.1) and attitude.roll_rad == pytest.approx(0.0) and attitude.yaw_rad == pytest.approx(0.0)
    rolled = flow_odometry_node.attitude_from_quaternion((math.sin(0.05), 0.0, 0.0, math.cos(0.05)))  # about the nose: right side down
    assert rolled.roll_rad == pytest.approx(0.1) and rolled.pitch_rad == pytest.approx(0.0)
    assert flow_odometry_node.attitude_from_quaternion((0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))).yaw_rad == pytest.approx(math.pi / 2)


def test_the_height_is_the_height_estimators_the_rangefinders_in_its_range_and_the_vehicles_own_above_it(ros):
    """Found in the sim: the transit is at 15 m, above the rangefinder's 12 m, and with the bare rangefinder most samples had no height."""
    node = node_with(ros, **{"features.flow_odometry": True})
    field = pictures.ground()
    fly(node, [pictures.view(field, dy=2.0 * index) for index in range(8)], height_m=10.0)
    status = json.loads(node.publishers[STATUS].sent[-1].data)
    assert status["height_source"] == "rangefinder" and status["height_m"] == pytest.approx(10.0, abs=0.05) and status["quality"] > 0
    high = node_with(ros, **{"features.flow_odometry": True})
    for index in range(8):  # at 15 m the rangefinder says it is out of range: the reading it gives is its maximum
        stamp_ns = T0 + index * FRAME_NS
        for offset_ms in (-40, -20, 0, 20):
            high.subscriptions[POSE_TOPIC](pose(stamp_ns + offset_ms * 1_000_000, 15.0))
        high.subscriptions[RANGE_TOPIC](rangefinder(stamp_ns - 10_000_000, 12.0))
        high.subscriptions[IMAGE_TOPIC](mono(pictures.view(field, dy=2.0 * index), stamp_ns))
    status = json.loads(high.publishers[STATUS].sent[-1].data)
    assert status["height_source"] == "baro" and status["height_m"] == pytest.approx(15.0) and 0 < status["quality"] <= 127  # held down: the height is above home, not above the ground
    assert high.publishers[VELOCITY].sent[-1].twist.linear.x == pytest.approx(2.0 * 4 * 15.0 / (233.87135 * 0.2), abs=0.15)  # scaled by the 15 m
    # With no rangefinder at all the height is still the vehicle's own, and says so.
    none = node_with(ros, **{"features.flow_odometry": True})
    fly(none, [pictures.view(field, dy=2.0 * index) for index in range(8)], ranges=False)
    assert json.loads(none.publishers[STATUS].sent[-1].data)["height_source"] == "baro" and none.publishers[VELOCITY].sent != []


def test_a_frame_waits_for_the_pose_history_to_reach_its_moment_and_is_not_waited_for_for_ever(ros):
    """Found in the sim: a frame reaches the node before the pose that covers it, and a third of the samples had no attitude."""
    node = node_with(ros, **{"features.flow_odometry": True})
    field = pictures.ground()
    frames = [pictures.view(field, dy=2.0 * index) for index in range(8)]
    for index, picture in enumerate(frames):  # each frame first, the poses round its moment 30 ms later
        stamp_ns = T0 + index * FRAME_NS
        node.subscriptions[RANGE_TOPIC](rangefinder(stamp_ns - 10_000_000, 10.0))
        node.subscriptions[IMAGE_TOPIC](mono(picture, stamp_ns))
        assert len(node._waiting) == 1  # held: the history has not reached it
        for offset_ms in (-40, -20, 0, 20):
            node.subscriptions[POSE_TOPIC](pose(stamp_ns + offset_ms * 1_000_000, 10.0))
        assert node._waiting == []  # handled as soon as a pose at or after its moment came
    assert node.counts == {"frames": 8, "samples": 4, "with quality": 4} and node.whys == {}
    stamps = [item.header.stamp.sec * 1_000_000_000 + item.header.stamp.nanosec for item in node.publishers[VELOCITY].sent]
    assert stamps == [T0 + index * FRAME_NS for index in range(4, 8)]  # in order, each with its own frame's stamp
    # No pose ever comes: the frames are not held for ever. Each is handled when a frame a quarter of a second newer has come, with no attitude and so no velocity.
    blind = node_with(ros, **{"features.flow_odometry": True, "send_optical_flow": True})
    for index in range(20):
        blind.subscriptions[IMAGE_TOPIC](mono(frames[index % 8], T0 + index * FRAME_NS))
    assert len(blind._waiting) <= 6 and blind.publishers[VELOCITY].sent == [] and blind.counts["samples"] >= 9 and blind.counts["with quality"] == 0
    assert blind.publishers[SEND].sent[-1].quality == 0  # the flight controller is told the sensor has nothing to say


def test_low_over_the_ground_the_corner_patches_are_followed_and_a_bare_surface_has_no_velocity(ros):
    node = node_with(ros, **{"features.flow_odometry": True})
    field = pictures.ground()
    fly(node, [pictures.view(field, dy=3.0 * index) for index in range(8)], height_m=2.0)  # under corners_below_m (3 m)
    status = json.loads(node.publishers[STATUS].sent[-1].data)
    assert status["patches"] >= 5 and status["quality"] > 0 and node.publishers[VELOCITY].sent[-1].twist.linear.x == pytest.approx(12.0 * 2.0 / (233.87135 * 0.2), abs=0.08)
    bare = node_with(ros, **{"features.flow_odometry": True})
    fly(bare, [np.full((200, 320), 128.0)] * 8)
    assert bare.publishers[VELOCITY].sent == [] and "no texture" in json.loads(bare.publishers[STATUS].sent[-1].data)["why"]
    assert bare.whys == {"no texture in the picture (a bare surface)": 4}


def test_a_frame_of_the_wrong_size_or_encoding_or_without_a_stamp_is_not_used_and_the_optical_flow_can_be_kept_back(ros):
    node = node_with(ros, **{"features.flow_odometry": True})
    assert SEND not in node.publishers
    field = pictures.ground()
    node.subscriptions[IMAGE_TOPIC](mono(np.zeros((360, 640)), T0))
    assert node.whys == {"not the calibrated size": 1} and "is 640 x 360 but the mode is 320 x 200" in node.logger.error.call_args[0][0]
    node.subscriptions[IMAGE_TOPIC](mono(pictures.view(field), T0, encoding="yuv422"))
    assert "unsupported image encoding" in node.logger.error.call_args[0][0]
    node.subscriptions[IMAGE_TOPIC](mono(pictures.view(field), 0))
    assert "image without a stamp" in node.logger.error.call_args[0][0] and node.counts["samples"] == 0
    # A colour camera's frame is taken too (the OV9782 is a colour sensor).
    colour = np.repeat(np.clip(pictures.view(field), 0, 255).astype(np.uint8)[:, :, None], 3, axis=2)
    message = types.SimpleNamespace(header=image_header(184, 5, "flow_camera_optical"), height=200, width=320, encoding="bgr8", data=colour.tobytes())
    node.subscriptions[IMAGE_TOPIC](message)
    assert node.counts["frames"] == 2


def test_a_mode_that_is_not_the_flow_pictures_shape_or_is_not_named_stops_the_node_with_a_reason(ros):
    wide = node_with(ros, **{"features.flow_odometry": True, "camera_mode": "sim_delivery"})  # 640 x 360
    assert wide.publishers == {} and "not the flow picture's shape" in wide.logger.error.call_args[0][0]
    unnamed = node_with(ros, **{"features.flow_odometry": True, "camera_mode": ""})
    assert unnamed.subscriptions == {} and "camera_mode and airframe must both be set" in unnamed.logger.error.call_args[0][0]


def test_a_frame_that_cannot_be_handled_is_dropped_and_the_node_goes_on(ros):
    """The flight-safety review: an exception in the flow ended the node for the rest of the flight."""
    node = node_with(ros, **{"features.flow_odometry": True})
    field = pictures.ground()
    frames = [pictures.view(field, dy=2.0 * index) for index in range(14)]
    real, calls = node._odometry.add, []

    def failing(*arguments, **more):
        calls.append(1)
        if len(calls) == 6:
            raise RuntimeError("the correlation blew up")
        return real(*arguments, **more)

    node._odometry.add = failing
    fly(node, frames)
    assert node.whys == {"a frame could not be handled": 1} and "a frame could not be handled and was dropped (the correlation blew up)" in node.logger.error.call_args[0][0]
    assert node.counts["frames"] == 14 and node.counts["with quality"] >= 4  # after the failure the held frames start again, and samples come again
    assert json.loads(node.publishers[STATUS].sent[-1].data)["quality"] > 0


def test_the_velocity_is_placed_on_the_vehicle_by_the_mounts_own_rotation_and_a_camera_not_looking_down_is_refused(ros, monkeypatch):
    """The flight-safety review, twice: the mount was read and thrown away, and then read through a field the loader's mount does not have."""
    from drone_mission_core import frame_projection, projection_v2

    real = frame_projection.load_camera
    camera, mount = real("sim_flow_recovery", "sim_iris_recovery")
    assert isinstance(mount, projection_v2.CameraMount) and mount.rotation_body_from_optical == ((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0))  # what the node is really given

    def mounted(rotation):
        return lambda mode, airframe, *more: (camera, projection_v2.CameraMount(position_m=(0.05, 0.0, -0.08), rotation_body_from_optical=rotation, optical_frame="flow_camera_optical"))

    field = pictures.ground()
    frames = [pictures.view(field, dy=2.0 * index) for index in range(8)]  # the content moves down the picture
    usual = node_with(ros, **{"features.flow_odometry": True})
    fly(usual, frames)
    assert usual.publishers[VELOCITY].sent[-1].twist.linear.x > 1.5  # top to the nose: the vehicle goes forward
    monkeypatch.setattr(frame_projection, "load_camera", mounted(((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))))
    back = node_with(ros, **{"features.flow_odometry": True})
    fly(back, frames)
    assert back.publishers[VELOCITY].sent[-1].twist.linear.x < -1.5  # the same pictures, the picture's top to the tail: it goes backwards
    monkeypatch.setattr(frame_projection, "load_camera", mounted(((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))))
    forward_looking = node_with(ros, **{"features.flow_odometry": True})
    assert forward_looking.subscriptions == {} and "the flow camera must look straight down" in forward_looking.logger.error.call_args[0][0]
