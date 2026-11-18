"""Projection v2 in the mission logger (FND-05), through the real node class with its flag off and on.

rclpy, the message packages, cv_bridge, image_geometry and scipy are
replaced by stand-ins, so this runs without ROS. The node writes its logs
under the home folder; here that is a temporary folder.
"""

import csv
import math
import sys
import types
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "vision_pipeline"), str(SRC / "drone_mission_core"), str(SRC / "argus_camera")]

from drone_mission_core import projection_v2 as v2  # noqa: E402
from drone_mission_core.frame_projection import FrameProjectionError, load_camera  # noqa: E402

S = 1_000_000_000
MS = 1_000_000
HOME = (34.175, -118.4825)
NOSE_NORTH = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
CAMERA, MOUNT = load_camera("sim_delivery", "sim_iris_downcam")
OWN_K = [CAMERA.fx, 0.0, CAMERA.cx, 0.0, CAMERA.fy, CAMERA.cy, 0.0, 0.0, 1.0]
MARKER = (4.0, 6.0, 0.0)  # east, north of home
ON = {"features.projection_v2": True, "camera_mode": "sim_delivery", "airframe": "sim_iris_downcam"}


class FakeNode:
    overrides: dict = {}

    def __init__(self, name):
        self.declared, self.subscriptions, self.logger = {}, {}, mock.MagicMock()

    def declare_parameter(self, name, default=None):
        self.declared[name] = default

    def get_parameter(self, name):
        assert name in self.declared, f"{name} read before it was declared"
        value = self.overrides.get(name, self.declared[name])
        kinds = SimpleNamespace(bool_value=bool(value), string_value=str(value), double_value=float(value) if isinstance(value, (int, float)) else 0.0)
        return SimpleNamespace(value=value, get_parameter_value=lambda: kinds)

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscriptions[topic] = callback

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=50 * S))


@pytest.fixture
def logger_module(monkeypatch, tmp_path):
    stand_ins = {name: mock.MagicMock() for name in (
        "rclpy", "rclpy.qos", "cv_bridge", "image_geometry", "scipy", "scipy.spatial", "scipy.spatial.transform",
        "geometry_msgs", "geometry_msgs.msg", "mavros_msgs", "mavros_msgs.msg", "sensor_msgs", "sensor_msgs.msg", "vision_msgs", "vision_msgs.msg",
    )}  # fmt: skip
    stand_ins["rclpy.node"] = types.SimpleNamespace(Node=FakeNode)
    for name, module in stand_ins.items():
        monkeypatch.setitem(sys.modules, name, module)
    for name in [n for n in sys.modules if n in ("vision_pipeline.mission_logger", "vision_pipeline.logger_projection")]:
        monkeypatch.delitem(sys.modules, name)
    from vision_pipeline import mission_logger

    monkeypatch.setattr(mission_logger.os.path, "expanduser", lambda path: path.replace("~", str(tmp_path)))
    monkeypatch.setattr(FakeNode, "overrides", {})
    return mission_logger


def start(module, monkeypatch, **parameters):
    monkeypatch.setattr(FakeNode, "overrides", parameters)
    return module.MissionLogger()


def stamp(ns):
    return SimpleNamespace(sec=ns // S, nanosec=ns % S)


def position_at(ns):
    return (3.0 * (ns - 50 * S) / S, 0.0, 20.0)  # east at 3 m/s at 20 m


def feed(node, k=OWN_K, frame_ns=50 * S + 400 * MS):
    """camera_info, home, a second of poses, one frame, then one detection of MARKER in that frame."""
    node.subscriptions["/camera/camera_info"](SimpleNamespace(width=640, height=360, k=k, d=[0.0] * 5, p=[0.0] * 12))
    node.subscriptions["/mavros/home_position/home"](SimpleNamespace(geo=SimpleNamespace(latitude=HOME[0], longitude=HOME[1], altitude=215.0)))
    for index in range(31):
        ns = 50 * S + index * 33 * MS
        east, north, up = position_at(ns)
        node.subscriptions["/mavros/local_position/pose"](
            SimpleNamespace(
                header=SimpleNamespace(stamp=stamp(ns)),
                pose=SimpleNamespace(position=SimpleNamespace(x=east, y=north, z=up), orientation=SimpleNamespace(x=NOSE_NORTH[0], y=NOSE_NORTH[1], z=NOSE_NORTH[2], w=NOSE_NORTH[3])),
            )
        )
    node.cv_bridge.imgmsg_to_cv2 = lambda message, desired_encoding: np.zeros((360, 640, 3), dtype=np.uint8)
    node.subscriptions["/camera/image_raw"](SimpleNamespace(header=SimpleNamespace(stamp=stamp(frame_ns))))
    u, v = v2.pixel_of_ground_point(CAMERA, MOUNT, MARKER, position_at(frame_ns), NOSE_NORTH)
    detection = SimpleNamespace(bbox=SimpleNamespace(center=SimpleNamespace(position=SimpleNamespace(x=u, y=v)), size_x=30.0, size_y=28.0), results=[SimpleNamespace(hypothesis=SimpleNamespace(score=0.9))])
    node.subscriptions["/drone_control/detection"](SimpleNamespace(header=SimpleNamespace(stamp=stamp(frame_ns)), detections=[detection]))
    for handle in (node._f_full_v1, node._f_full_v2):
        handle.flush()
    return [list(csv.reader(open(path)))[1:] for path in (node.csv_full_v1, node.csv_full_v2)]


def east_north(row):
    latitude, longitude = float(row[1]), float(row[2])
    return (math.radians(longitude - HOME[1]) * 6378137.0 * math.cos(math.radians(HOME[0])), math.radians(latitude - HOME[0]) * 6378137.0)


def test_flag_off_the_logger_is_the_2026_one_and_the_new_module_is_not_loaded(logger_module, monkeypatch):
    node = start(logger_module, monkeypatch)
    assert node.declared["features.projection_v2"] is False and node._projection_v2 is None
    assert "vision_pipeline.logger_projection" not in sys.modules
    calls = []
    monkeypatch.setattr(node, "_raycast_to_gps", lambda u, v, pose: calls.append((u, v)) or None)
    monkeypatch.setattr(node, "_get_pose_at_time_v1_nearest", lambda stamp, history: "nearest")
    monkeypatch.setattr(node, "_get_pose_at_time_v2_interpolated", lambda stamp, history: "interpolated")
    full_v1, full_v2 = feed(node)
    assert len(calls) == 2 and full_v1 == [] and full_v2 == []  # the 2026 raycast was asked, once for each pose


def test_flag_on_a_detection_is_logged_where_the_marker_is(logger_module, monkeypatch):
    node = start(logger_module, monkeypatch, **ON)
    monkeypatch.setattr(node, "_raycast_to_gps", lambda *a: (_ for _ in ()).throw(AssertionError("the 2026 raycast ran with the flag on")))
    full_v1, full_v2 = feed(node)
    assert len(full_v1) == 1 and full_v1 == full_v2  # projection v2 has one answer; both logs carry it
    assert east_north(full_v1[0]) == pytest.approx(MARKER[:2], abs=0.02)  # the logs keep seven decimals of a degree, about 1 cm
    assert node._projection_v2_problem is None


def test_flag_on_frames_that_were_already_undistorted_log_no_position(logger_module, monkeypatch):
    node = start(logger_module, monkeypatch, **ON)
    grabber_k = [CAMERA.fx * 0.93, 0.0, CAMERA.cx + 4.0, 0.0, CAMERA.fy * 0.93, CAMERA.cy - 2.0, 0.0, 0.0, 1.0]  # a new camera matrix, as the 2026 image_grabber publishes
    full_v1, full_v2 = feed(node, k=grabber_k)
    assert full_v1 == [] and full_v2 == []
    assert "will log no target positions" in str(node.logger.error.call_args) and "not that mode's raw frames" in node._projection_v2_problem


def test_flag_on_a_detection_from_a_frame_with_no_pose_logs_no_position(logger_module, monkeypatch):
    node = start(logger_module, monkeypatch, **ON)
    full_v1, _full_v2 = feed(node, frame_ns=50 * S + 1200 * MS)  # 0.2 s after the newest pose
    assert full_v1 == [] and "no pose for the frame's capture time" in str(node.logger.warn.call_args)


def test_flag_on_the_logger_does_not_start_on_a_camera_that_is_a_guess(logger_module, monkeypatch):
    with pytest.raises(ValueError, match="camera_mode and airframe"):
        start(logger_module, monkeypatch, **{"features.projection_v2": True})
    with pytest.raises(FrameProjectionError, match="not verified"):
        start(logger_module, monkeypatch, **{**ON, "camera_mode": "imx519_4k", "airframe": "argus_2027"})
