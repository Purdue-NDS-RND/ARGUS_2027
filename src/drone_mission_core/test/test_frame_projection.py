"""Projection v2's wiring (FND-05): the camera a mission names, the target offset, and the executor's side.

The executor is the real node class on the stand-ins of
test_executor_feature_overrides.py, so this runs without ROS.
"""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(ROOT / "src" / "argus_camera"))
from test_executor_feature_overrides import FakeNode, executor_module, start  # noqa: E402, F401  (executor_module is a fixture)

from drone_mission_core import frame_projection  # noqa: E402
from drone_mission_core import projection_v2 as v2  # noqa: E402
from drone_mission_core.frame_projection import FrameProjectionError, TargetOffset, camera_from_config, load_camera  # noqa: E402
from drone_mission_core.pose_buffer import PoseBuffer  # noqa: E402

FLAG = "use_frame_time_pose"
SEQUENCES = ROOT / "src" / "drone_mission_demo" / "config" / "sequences"
S = 1_000_000_000
MS = 1_000_000
LEVEL = (0.0, 0.0, 0.0, 1.0)  # nose east


# --- which camera ---


def test_the_sims_camera_and_mount_load_from_the_camera_packages_config():
    camera, mount = load_camera("sim_delivery", "sim_iris_downcam")
    assert (camera.name, camera.width, camera.height, camera.measured) == ("sim_delivery", 640, 360, True)
    assert camera.fx == pytest.approx(389.16536) and mount.position_m == (0.0, 0.0, 0.0) and mount.optical_frame == "down_camera_optical"


@pytest.mark.parametrize(
    "mode, airframe, word",
    [
        ("imx519_4k", "argus_2027", "not verified"),  # the aircraft's mount has not been measured
        ("imx519_720p60", "sim_iris_downcam", "not calibrated"),  # a stand-in calibration is not flown on
        ("imx519_8k", "sim_iris_downcam", "not a sensor mode"),
        ("sim_delivery", "argus_2028", "no airframe"),
        ("ov9782_flow", "sim_iris_downcam", "has no calibration yet"),
    ],
)
def test_a_camera_that_is_a_guess_is_refused_with_what_to_do(mode, airframe, word):
    with pytest.raises(FrameProjectionError, match=word):
        load_camera(mode, airframe)


def test_a_mode_on_a_camera_the_airframe_does_not_carry_is_refused(tmp_path):
    import shutil

    import yaml

    config = ROOT / "src" / "argus_camera" / "config"
    shutil.copytree(config / "calibration", tmp_path / "calibration")
    shutil.copy(config / "modes.yaml", tmp_path / "modes.yaml")
    robot = yaml.safe_load((config / "robot.yaml").read_text())
    robot["airframes"]["sim_iris_downcam"]["cameras"] = {"side": robot["airframes"]["sim_iris_downcam"]["cameras"]["down"]}
    (tmp_path / "robot.yaml").write_text(yaml.safe_dump(robot))
    with pytest.raises(FrameProjectionError, match="no camera 'down'"):
        load_camera("sim_delivery", "sim_iris_downcam", config_dir=tmp_path)


def test_a_sim_camera_is_refused_on_an_airframe_that_is_not_the_sims(tmp_path):
    import shutil

    import yaml

    config = ROOT / "src" / "argus_camera" / "config"
    shutil.copytree(config, tmp_path, dirs_exist_ok=True)
    robot = yaml.safe_load((config / "robot.yaml").read_text())
    robot["airframes"]["argus_2027"]["verified"] = True  # the day the aircraft's mounts are measured
    (tmp_path / "robot.yaml").write_text(yaml.safe_dump(robot))
    # The sim's survey and 4K cameras are ideal pinholes at the aircraft's own image sizes: the size check would not tell them apart.
    for mode in ("sim_survey", "sim_still4k", "sim_delivery", "sim_flow"):
        with pytest.raises(FrameProjectionError, match=f"{mode} is a sim camera .* argus_2027 is not a sim vehicle"):
            load_camera(mode, "argus_2027", config_dir=tmp_path)
    # And the other way: the aircraft's calibrated camera is not flown on the sim vehicle's mount (a lens at the body origin).
    with pytest.raises(FrameProjectionError, match="imx519_4k is a real camera .* sim_iris_downcam is a sim vehicle"):
        load_camera("imx519_4k", "sim_iris_downcam", config_dir=tmp_path)
    assert load_camera("imx519_4k", "argus_2027", config_dir=tmp_path)[0].width == 3840  # the aircraft's own pair loads once verified
    assert load_camera("sim_survey", "sim_iris_downcam", config_dir=tmp_path)[0].width == 1280  # in the sim it loads


def test_a_mission_config_must_name_both_once_the_flag_is_on():
    assert camera_from_config({"camera_mode": "sim_delivery", "airframe": "sim_iris_downcam"}, "package_delivery")[0].name == "sim_delivery"
    with pytest.raises(FrameProjectionError, match="package_delivery.*camera_mode and airframe"):
        camera_from_config({}, "package_delivery")
    with pytest.raises(FrameProjectionError, match="must name airframe"):
        camera_from_config({"camera_mode": "sim_delivery", "airframe": " "}, "package_delivery")


def test_only_the_sim_sequences_name_a_camera_and_the_aircrafts_do_not_yet():
    import yaml

    named = {}
    for path in sorted(SEQUENCES.glob("*.yaml")):
        for mission in yaml.safe_load(path.read_text())["mission_sequence"]["missions"]:
            config = mission.get("config") or {}
            if "camera_mode" in config or "airframe" in config:
                named[path.name] = (mission["type"], config.get("camera_mode"), config.get("airframe"))
    assert named == {
        "package_delivery_demo.yaml": ("package_delivery", "sim_delivery", "sim_iris_downcam"),
        "package_drop_demo.yaml": ("package_drop", "sim_delivery", "sim_iris_downcam"),
        # The generated sim copies of the live files (CEN-11) name the sim's camera; the live files stay without one.
        "sim_package_delivery_live.yaml": ("package_delivery", "sim_delivery", "sim_iris_downcam"),
        "sim_package_drop_live.yaml": ("package_drop", "sim_delivery", "sim_iris_downcam"),
        # The 2027 files (CEN-10) name the aircraft's mode and airframe, which do not load until they are measured; their sim copies the sim's.
        "package_delivery_2027.yaml": ("package_delivery", "imx519_720p60", "argus_2027"),
        "package_drop_2027.yaml": ("package_drop", "imx519_720p60", "argus_2027"),
        "sim_package_delivery_2027.yaml": ("package_delivery", "sim_delivery", "sim_iris_downcam"),
        "sim_package_drop_2027.yaml": ("package_drop", "sim_delivery", "sim_iris_downcam"),
        # The sim's survey sequence (LOC-3) names the sim's survey camera, for the planner's footprint.
        "search_pattern_demo.yaml": ("search_pattern", "sim_survey", "sim_iris_downcam"),
        "sim_package_recovery.yaml": ("package_recovery", "sim_delivery", "sim_iris_downcam"),
    }


# --- the offset a mission steers on ---


def hovering(position=(5.0, 5.0, 10.0)):
    buffer = PoseBuffer()
    for index in range(31):
        buffer.add_pose(100 * S + index * 33 * MS, position, LEVEL)
    return buffer


def test_the_offset_is_the_targets_point_minus_where_the_vehicle_is_now():
    camera, mount = load_camera("sim_delivery", "sim_iris_downcam")
    buffer = hovering()
    offset = TargetOffset(v2.GroundProjector(camera, mount, buffer.pose_at))
    target = (7.0, 4.0, 0.0)
    u, v = v2.pixel_of_ground_point(camera, mount, target, (5.0, 5.0, 10.0), LEVEL)
    x_norm, y_norm = u / 640 * 2 - 1, v / 360 * 2 - 1  # as target_cv publishes it
    east, north = offset.offset(x_norm, y_norm, (640, 360), 100 * S + 500 * MS, (6.0, 5.5))  # the vehicle has moved on since
    assert (east, north) == pytest.approx((1.0, -1.5), abs=1.0e-6)
    assert offset.last_refusal is None and (offset.last_point.east_m, offset.last_point.north_m) == pytest.approx((7.0, 4.0), abs=1.0e-6)
    assert offset.offset(0.0, 0.0, (640, 360), 100 * S + 500 * MS, (5.0, 5.0)) == pytest.approx((0.0, 0.0), abs=1.0e-9)  # image centre: straight down


def test_the_offset_refuses_another_image_size_and_a_frame_with_no_pose():
    camera, mount = load_camera("sim_delivery", "sim_iris_downcam")
    offset = TargetOffset(v2.GroundProjector(camera, mount, hovering().pose_at))
    assert offset.offset(0.0, 0.0, (1280, 720), 100 * S + 500 * MS, (5.0, 5.0)) is None
    assert "1280 x 720" in offset.last_refusal and "not scaled" in offset.last_refusal
    assert offset.offset(0.0, 0.0, (640, 360), 105 * S, (5.0, 5.0)) is None and "no pose" in offset.last_refusal
    assert offset.last_point is None


# --- the executor ---


@pytest.fixture
def subscriptions(monkeypatch):
    made = []
    monkeypatch.setattr(FakeNode, "create_subscription", lambda self, message_type, topic, callback, qos: made.append((topic, callback)), raising=False)
    return made


def stamp(ns):
    return SimpleNamespace(sec=ns // S, nanosec=ns % S)


def pose_message(ns, up=12.0):
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp(ns)), pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=up), orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))


def test_flag_off_the_executor_has_no_ground_height_and_the_context_gives_no_pose_or_projector(executor_module, monkeypatch, subscriptions):
    node = start(executor_module, monkeypatch)
    assert node._ground_height is None and frame_projection is not None
    assert "/mavros/rangefinder/rangefinder" not in [topic for topic, _ in subscriptions]
    context = node._mission_context
    assert context.pose_at(100 * S) is None and context.ground_projector(None, None) is None
    assert all(mission._frame_camera is None for mission in node._sequence if hasattr(mission, "_frame_camera"))


def test_flag_on_the_context_projects_with_the_executors_pose_history_and_the_rangefinders_ground(executor_module, monkeypatch, subscriptions):
    node = start(executor_module, monkeypatch, feature_overrides=FLAG)
    context = node._mission_context
    delivery = next(mission for mission in node._sequence if mission.name == "package_delivery")
    assert delivery._frame_camera[0].name == "sim_delivery"  # named by the demo sequence, loaded at start-up
    feeds = {topic: callback for topic, callback in subscriptions}  # the buffer's own pose subscription is the later one
    for index in range(31):
        feeds["/mavros/local_position/pose"](pose_message(200 * S + index * 33 * MS))
    assert context.pose_at(200 * S + 500 * MS).position == pytest.approx((1.0, 2.0, 12.0))
    assert context.pose_at(190 * S) is None  # no pose for that instant: never the latest instead

    projector = context.ground_projector(*delivery._frame_camera)
    centre = (320.0, 180.0)
    assert projector.project_pixel_to_ground(centre, 200 * S + 500 * MS).height_source == "home"
    for ms in (380, 430, 480):
        feeds["/mavros/rangefinder/rangefinder"](SimpleNamespace(header=SimpleNamespace(stamp=stamp(200 * S + ms * MS)), range=10.5, min_range=0.0, max_range=1000.0))
    point = projector.project_pixel_to_ground(centre, 200 * S + 500 * MS)
    assert point.height_source == "rangefinder" and point.up_m == pytest.approx(1.5)  # 12 m up, 10.5 m above the ground
    assert (point.east_m, point.north_m) == pytest.approx((1.0, 2.0)) and point.range_m == pytest.approx(10.5)
    assert math.isfinite(point.d_pixel[0, 0])


@pytest.mark.parametrize("sequence", ["package_delivery_live.yaml", "package_drop_live.yaml", "final_curcuit_delivery.yaml", "final_waypoint_drop.yaml"])
def test_flag_on_a_sequence_that_names_no_camera_does_not_start(executor_module, monkeypatch, sequence):
    with pytest.raises(ValueError, match="use_frame_time_pose is on, so its config must name camera_mode and airframe"):
        start(executor_module, monkeypatch, sequence_file=str(SEQUENCES / sequence), feature_overrides=FLAG)
    start(executor_module, monkeypatch, sequence_file=str(SEQUENCES / sequence))  # flag off: starts as in 2026


def test_flag_on_a_sequence_with_no_tracking_mission_needs_no_camera(executor_module, monkeypatch):
    node = start(executor_module, monkeypatch, sequence_file=str(SEQUENCES / "square_only.yaml"), feature_overrides=FLAG)
    assert node._pose_buffer is not None and node._ground_height is not None
