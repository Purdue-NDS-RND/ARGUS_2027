"""PLT-07: what the bringup launch starts, from a profile (drone_mission_core/bringup.py). No ROS."""

import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "drone_mission_core"))

from drone_mission_core import bringup  # noqa: E402
from drone_mission_core.bringup import BringupError  # noqa: E402
from drone_mission_core.features import FeatureFlagError  # noqa: E402

PARAMS = REPO / "src" / "drone_mission_demo" / "config" / "params"
SIM_PARAMS = yaml.safe_load((PARAMS / "sim_target_mission.yaml").read_text())


def profile(tmp_path, text):
    path = tmp_path / "profile.yaml"
    path.write_text(text)
    return path


def test_the_2026_profile_turns_nothing_on():
    plan = bringup.load_bringup(REPO / "profiles" / "comp_2026.yaml", SIM_PARAMS)
    assert plan.feature_overrides == "" and plan.node_flags == {} and plan.node_flags_argument() == ""
    assert "mission_executor" in plan.nodes and "simple_takeoff_service" in plan.nodes
    assert not set(plan.nodes) & set(bringup.FLAG_STARTED_NODES)


def test_a_profile_gives_the_mission_flags_the_node_flags_and_the_nodes(tmp_path):
    path = profile(tmp_path, """
description: the estimator with the ring fit
enable: [use_frame_time_pose, use_target_estimator, use_ring_fit]
nodes:
  target_cv: [stamp_from_image, detection_contract]
  ring_fit: [ring_fit]
start:
  detector: target_cv
""")
    plan = bringup.load_bringup(path, SIM_PARAMS)
    assert plan.feature_overrides == "use_frame_time_pose,use_target_estimator,use_ring_fit"
    assert plan.node_flags == {"target_cv": ("stamp_from_image", "detection_contract"), "ring_fit": ("ring_fit",)}
    assert plan.node_flags_argument() == "ring_fit.ring_fit,target_cv.stamp_from_image,target_cv.detection_contract"
    assert plan.nodes == ("simple_takeoff_service", "target_cv", "mission_executor", "ring_fit", "session_logger")
    assert bringup.parse_node_flags(plan.node_flags_argument()) == {"ring_fit": ("ring_fit",), "target_cv": ("stamp_from_image", "detection_contract")}
    # Keyed by the node's name, as the params file is: a wildcard entry loses to the params file's when a global rule comes first.
    assert bringup.feature_parameters("target_cv", plan.node_flags) == {"target_cv": {"ros__parameters": {"features": {"stamp_from_image": True, "detection_contract": True}}}}
    assert bringup.feature_parameters("mission_executor", plan.node_flags) == {}


def test_a_node_that_exists_for_its_flag_is_started_only_with_that_flag():
    assert "ring_fit" not in bringup.nodes_to_start("none", "target_cv", {})
    assert "ring_fit" not in bringup.nodes_to_start("none", "target_cv", {"ring_fit": ("other",)})
    assert bringup.nodes_to_start("none", "none", {"loc_node": ("loc_node", "marker_tracker"), "recovery_cv": ("recovery_cv",)}, log_session=False) == ("simple_takeoff_service", "mission_executor", "recovery_cv", "loc_node")


@pytest.mark.parametrize("camera, first", [("mipi", "mipi_grabber"), ("usb", "usb_grabber")])
def test_the_camera_named_is_the_grabber_started(camera, first):
    nodes = bringup.nodes_to_start(camera, "target_cv", {})
    assert nodes[0] == first and sum(node in ("mipi_grabber", "usb_grabber") for node in nodes) == 1
    assert all(node in bringup.NODES for node in nodes)


def test_the_2027_camera_driver_is_never_started_by_the_bringup():
    """It is a boot service (PLT-02), and PLT-03 lets nothing else start it."""
    assert not any(package == "argus_camera" for package, _executable in bringup.NODES.values())
    with pytest.raises(BringupError, match="camera is one of"):
        bringup.nodes_to_start("argus", "none", {})


def test_every_node_the_bringup_can_start_is_a_console_script_of_its_package():
    for name, (package, executable) in bringup.NODES.items():
        setup = (REPO / "src" / package / "setup.py").read_text()
        assert f"{executable} = " in setup, name


def test_what_the_flight_needs_started_can_be_said_over_the_profile(tmp_path):
    plan = bringup.load_bringup(REPO / "profiles" / "comp_2026.yaml", SIM_PARAMS, camera="mipi", detector="target_cv")
    assert plan.nodes == ("mipi_grabber", "simple_takeoff_service", "target_cv", "mission_executor", "session_logger")
    path = profile(tmp_path, "description: x\nenable: []\nnodes: {target_cv: [stamp_from_image]}\nstart: {detector: target_cv}\n")
    with pytest.raises(BringupError, match="does not start"):
        bringup.load_bringup(path, SIM_PARAMS, detector="none")  # the flags' node taken away


def test_the_shipped_profiles_can_all_be_flown_with_the_sim_params():
    names = sorted(path.name for path in (REPO / "profiles").glob("*.yaml"))
    assert "comp_2026.yaml" in names and "delivery_estimator.yaml" in names
    for name in names:
        bringup.load_bringup(REPO / "profiles" / name, SIM_PARAMS)
    made = bringup.load_bringup(REPO / "profiles" / "delivery_estimator.yaml", SIM_PARAMS)
    assert made.feature_overrides == "use_frame_time_pose,use_target_estimator" and made.node_flags == {"target_cv": ("stamp_from_image", "detection_contract")}


@pytest.mark.parametrize(
    "text, word",
    [
        ("description: x\nenable: []\nnodes: {target_cv: [no_such_flag]}\nstart: {detector: target_cv}\n", "not declared"),
        ("description: x\nenable: []\nnodes: {ring_fit: [ring_fit]}\nstart: {detector: target_cv}\nextra: 1\n", "unknown key"),
        ("description: x\nenable: []\nnodes: {target_cv: [stamp_from_image]}\n", "does not start"),
        ("description: x\nenable: []\nnodes: {target_cv: stamp_from_image}\nstart: {detector: target_cv}\n", "list of flag names"),
        ("description: x\nenable: []\nnodes: {target_cv: [stamp_from_image, stamp_from_image]}\nstart: {detector: target_cv}\n", "twice"),
        ("description: x\nenable: []\nstart: {camera: lidar}\n", "camera is one of"),
        ("description: x\nenable: []\nstart: {detector: yolo}\n", "detector is one of"),
        ("description: x\nenable: []\nstart: {camera: argus}\n", "camera is one of"),
        ("description: x\nenable: []\nstart: {camera: mipi, camera_mode: imx519_720p60}\n", "'start' has"),
        ("description: x\nenable: []\nstart: {camera: mipi, speed: fast}\n", "'start' has"),
        ("description: x\nenable: [use_target_estimator]\n", "use_frame_time_pose"),
    ],
)
def test_a_profile_that_cannot_be_flown_is_refused(tmp_path, text, word):
    with pytest.raises((BringupError, FeatureFlagError), match=word):
        bringup.load_bringup(profile(tmp_path, text), SIM_PARAMS)


def test_a_flag_true_in_a_shipped_params_file_is_said():
    params = {"target_cv": {"ros__parameters": {"features": {"stamp_from_image": True}}}}
    assert "keeps every flag false" in bringup.undeclared_node_flags({"target_cv": ("stamp_from_image",)}, params)[0]
    assert bringup.undeclared_node_flags({"target_cv": ("stamp_from_image",)}, SIM_PARAMS) == []


@pytest.mark.parametrize("text", ["target_cv", "target_cv.", ".flag", "Target.flag", "a.b,a.b"])
def test_a_node_flags_argument_that_is_not_node_dot_flag_is_refused(text):
    with pytest.raises(BringupError):
        bringup.parse_node_flags(text)


def test_an_empty_node_flags_argument_is_no_flags():
    assert bringup.parse_node_flags("") == {} and bringup.parse_node_flags(None) == {} and bringup.parse_node_flags(" , ") == {}


def test_the_bringup_can_start_every_node_a_2026_launch_file_starts():
    """So that `go` with the 2026 profile can stand in for any of them: same executables, same params file, no flags."""
    import re

    can_start = {executable for _package, executable in bringup.NODES.values()}
    launch_files = sorted((REPO / "src" / "drone_mission_demo" / "launch").glob("*.launch.py"))
    assert len(launch_files) >= 14
    for path in launch_files:
        started = set(re.findall(r'executable="([a-z_]+)"', path.read_text()))
        assert started <= can_start, f"{path.name} starts {sorted(started - can_start)}"


def test_the_launch_gives_node_flags_in_a_file_keyed_by_the_nodes_name_and_never_as_a_dictionary():
    """Found in the sim: a dictionary goes out under `/**` and lost to the params file with use_sim_time set; the flags stayed off."""
    text = (REPO / "src" / "drone_mission_demo" / "launch" / "argus_bringup.launch.py").read_text()
    assert "yaml.safe_dump(flags, out)" in text and "parameters.append(flags_file)" in text and "parameters.append(flags)" not in text
    written = yaml.safe_load(yaml.safe_dump(bringup.feature_parameters("ring_fit", {"ring_fit": ("ring_fit",)})))
    assert written == {"ring_fit": {"ros__parameters": {"features": {"ring_fit": True}}}}
    # Same shape as the shipped params file's own block for that node, so the later file replaces exactly those values.
    assert SIM_PARAMS["ring_fit"]["ros__parameters"]["features"]["ring_fit"] is False
