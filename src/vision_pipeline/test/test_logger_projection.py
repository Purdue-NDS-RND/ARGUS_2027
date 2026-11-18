"""Projection v2 in the mission logger (FND-05): pixel to latitude and longitude, behind features.projection_v2. No ROS."""

import ast
import math
import sys
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "vision_pipeline"), str(SRC / "drone_mission_core"), str(SRC / "argus_camera")]

from drone_mission_core import projection_v2 as v2  # noqa: E402
from drone_mission_core.frame_projection import FrameProjectionError  # noqa: E402
from vision_pipeline.logger_projection import R_EARTH_M, LoggerProjection, enu_to_latlon  # noqa: E402

S = 1_000_000_000
MS = 1_000_000
HOME = (34.175, -118.4825)
NOSE_NORTH = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
LOGGER = SRC / "vision_pipeline" / "vision_pipeline" / "mission_logger.py"


def flying():
    """East at 3 m/s at 20 m, nose north, 30 poses a second for a second from t = 50 s."""
    projection = LoggerProjection("sim_delivery", "sim_iris_downcam")
    for index in range(31):
        projection.add_pose(50 * S + index * 33 * MS, (3.0 * index * 0.033, 0.0, 20.0), NOSE_NORTH)
    return projection


def test_a_marker_seen_from_a_moving_vehicle_is_logged_where_it_is():
    projection = flying()
    marker = (4.0, 6.0, 0.0)
    stamp = 50 * S + 400 * MS
    u, v = v2.pixel_of_ground_point(projection.camera, projection.mount, marker, (3.0 * 0.4, 0.0, 20.0), NOSE_NORTH)
    latitude, longitude = projection.latlon(u, v, stamp, (640, 360), *HOME)
    north_m = math.radians(latitude - HOME[0]) * R_EARTH_M
    east_m = math.radians(longitude - HOME[1]) * R_EARTH_M * math.cos(math.radians(HOME[0]))
    assert (east_m, north_m) == pytest.approx((4.0, 6.0), abs=0.01)  # the 2026 raycast sent this ray upward and logged nothing


def test_no_pose_for_the_frame_or_another_image_size_logs_nothing_and_says_why():
    projection = flying()
    assert projection.latlon(320, 180, 60 * S, (640, 360), *HOME) is None and "no pose" in projection.last_refusal
    assert projection.latlon(320, 180, 50 * S + 400 * MS, (3840, 2160), *HOME) is None and "3840 x 2160" in projection.last_refusal
    assert projection.latlon(320, 180, 50 * S + 400 * MS, (640, 360), *HOME) is not None and projection.last_refusal is None


def test_the_ground_altitude_parameter_is_the_plane_the_ray_meets():
    projection = flying()
    position = (3.0 * 0.4, 0.0, 20.0)
    raised = (4.0, 6.0, 2.0)  # ground 2 m above home
    u, v = v2.pixel_of_ground_point(projection.camera, projection.mount, raised, position, NOSE_NORTH)
    on_it = projection.latlon(u, v, 50 * S + 400 * MS, (640, 360), *HOME, ground_altitude_m=2.0)
    assert on_it == pytest.approx(enu_to_latlon(4.0, 6.0, *HOME), abs=1.0e-9)
    assert projection.latlon(u, v, 50 * S + 400 * MS, (640, 360), *HOME) != pytest.approx(on_it, abs=1.0e-7)


def test_flat_earth_step_is_the_loggers_own():
    latitude, longitude = enu_to_latlon(100.0, 200.0, *HOME)
    assert latitude == pytest.approx(HOME[0] + (200.0 / 6378137.0) * (180.0 / math.pi), abs=1.0e-12)
    assert longitude == pytest.approx(HOME[1] + (100.0 / (6378137.0 * math.cos(math.radians(HOME[0])))) * (180.0 / math.pi), abs=1.0e-12)


def test_the_logger_does_not_start_on_a_camera_that_is_a_guess():
    with pytest.raises(ValueError, match="camera_mode and airframe"):
        LoggerProjection("", "")
    with pytest.raises(FrameProjectionError, match="not verified"):
        LoggerProjection("imx519_4k", "argus_2027")


def test_frames_that_are_not_the_modes_raw_frames_are_recognised_by_their_camera_info():
    projection = LoggerProjection("sim_delivery", "sim_iris_downcam")
    camera = projection.camera
    own_k = [camera.fx, 0.0, camera.cx, 0.0, camera.fy, camera.cy, 0.0, 0.0, 1.0]
    assert projection.camera_info_problem(640, 360, own_k, [0.0] * 5) is None
    assert projection.camera_info_problem(640, 360, own_k, []) is None  # no distortion sent, none in the mode
    # The 2026 image_grabber undistorts each frame and publishes the new camera matrix it made for the result.
    undistorted = [camera.fx * 0.93, 0.0, camera.cx + 4.0, 0.0, camera.fy * 0.93, camera.cy - 2.0, 0.0, 0.0, 1.0]
    assert "not that mode's raw frames" in projection.camera_info_problem(640, 360, undistorted, [0.0] * 5)
    assert "not that mode's raw frames" in projection.camera_info_problem(640, 360, own_k, [-0.23, 0.05, 0.0, 0.0, 0.0])
    assert "1280 x 720" in projection.camera_info_problem(1280, 720, own_k, [0.0] * 5)
    assert projection.camera_info_problem(640, 360, [1.0, 2.0], [0.0] * 5) is not None


def test_the_logger_node_uses_it_behind_a_flag_that_is_off():
    import yaml

    source = LOGGER.read_text()
    declared = [
        call
        for call in ast.walk(ast.parse(source))
        if isinstance(call, ast.Call) and getattr(call.func, "attr", "") == "declare_parameter" and call.args and getattr(call.args[0], "value", None) == "features.projection_v2"
    ]
    assert len(declared) == 1 and declared[0].args[1].value is False
    header = source.split("class MissionLogger")[0]
    assert "logger_projection" not in header and "drone_mission_core" not in header  # imported only with the flag on
    assert source.count("self._raycast_to_gps(u, v, pose_v") == 2  # the 2026 raycast is still what runs with the flag off
    branch = source.split("if self._projection_v2 is not None:\n                # Projection v2 has one answer")[1].split("image_filename")[0]
    assert "self._project_v2_to_gps(u, v, msg.header.stamp, w, h)" in branch and "else:" in branch
    params = yaml.safe_load((SRC / "vision_pipeline" / "config" / "vision_params.yaml").read_text())
    assert params["mission_logger"]["ros__parameters"]["features"] == {"projection_v2": False}
    assert set(params) == {"image_grabber", "yolo_node", "mission_logger"}  # no features block at the top level
    aircraft = yaml.safe_load((SRC / "drone_target_cv" / "config" / "mipi_params.yaml").read_text())
    assert aircraft["mission_logger"]["ros__parameters"]["features"] == {"projection_v2": False} and "features" not in aircraft
