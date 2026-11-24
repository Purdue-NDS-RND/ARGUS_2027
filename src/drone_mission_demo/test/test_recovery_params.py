"""Recovery's executor keeps 2026's abort mode, RTL; the mission tells it when the abort is a landing (REC-7)."""

import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]
PARAMS = REPO / "src" / "drone_mission_demo" / "config" / "params"
sys.path.insert(0, str(REPO / "src" / "drone_mission_core"))

from drone_mission_core import recovery_return  # noqa: E402


def test_the_recovery_params_are_2026s_value_for_value_abort_mode_and_all():
    recovery = yaml.safe_load((PARAMS / "recovery_mission.yaml").read_text())
    base = yaml.safe_load((PARAMS / "mission_params.yaml").read_text())
    assert set(recovery) == set(base) == {"simple_takeoff_service", "mission_executor"}  # no top-level `features:`, no other node
    executor = recovery["mission_executor"]["ros__parameters"]
    # A failed recovery comes home unless the mission has named a landing (the lead's rule, 23 November 2026).
    assert executor["abort_rtl_mode"] == recovery_return.ABORT_RTL == "RTL"
    # Nothing differs from the 2026 file: the 20 Hz tick and the take-off service's tuned values (the 5 s settle
    # after arming among them) are 2026's, so a launch that passes only this file takes off as 2026 did. (The
    # flight-safety review: without the block the service fell back to its code's defaults.)
    assert recovery == base and executor["loop_rate_hz"] == 20.0
    assert recovery["simple_takeoff_service"]["ros__parameters"]["post_arm_settle_s"] == 5.0


def test_the_sims_recovery_params_are_the_recovery_abort_mode_with_both_detectors_off():
    sim = yaml.safe_load((PARAMS / "sim_recovery_mission.yaml").read_text())
    survey = yaml.safe_load((PARAMS / "sim_survey_mission.yaml").read_text())
    assert set(sim) == {"simple_takeoff_service", "mission_executor", "loc_node", "recovery_cv", "flow_odometry"}  # no top-level `features:`
    # The flow odometry (REC-5): the recovery vehicle's flow camera, its flag false.
    flow = sim["flow_odometry"]["ros__parameters"]
    assert flow["features"] == {"flow_odometry": False} and flow["camera_mode"] == "sim_flow_recovery" and flow["airframe"] == "sim_iris_recovery"
    assert sim["simple_takeoff_service"] == survey["simple_takeoff_service"]
    executor = sim["mission_executor"]["ros__parameters"]
    assert executor == {"loop_rate_hz": 20.0, "abort_rtl_mode": recovery_return.ABORT_RTL, "estimator_detectors": ["recovery_cv", "sim_oracle"]}
    # The oracle's block is the survey's, on the recovery vehicle's camera and running from the start; every flag false.
    oracle, surveys = sim["loc_node"]["ros__parameters"], survey["loc_node"]["ros__parameters"]
    assert {key for key in oracle if oracle[key] != surveys[key]} == {"camera_mode", "start_enabled"}
    assert (oracle["camera_mode"], oracle["start_enabled"], oracle["sim_only"], oracle["detector"]) == ("sim_delivery", True, True, "oracle")
    assert not any(oracle["features"].values()) and sim["recovery_cv"]["ros__parameters"]["features"] == {"recovery_cv": False}
    assert sim["recovery_cv"]["ros__parameters"]["camera_mode"] == "sim_delivery"
    # The launch sends the oracle's boxes to the topic the estimator listens to for a detector of that name.
    from drone_mission_core import detection_contract, estimator_feed

    launch = (REPO / "src" / "drone_mission_demo" / "launch" / "sim_package_recovery.launch.py").read_text()
    assert f'remappings=[("/loc/detections", "{detection_contract.topic_for("sim_oracle")}")]' in launch
    assert "sim_recovery_mission.yaml" in launch and "sim_package_recovery.yaml" in launch
    assert estimator_feed.DETECTOR_PIXEL_SIGMA_PX["recovery_cv"] == 3.0 and "sim_oracle" not in estimator_feed.DETECTOR_PIXEL_SIGMA_PX  # the oracle's boxes are weighed as boxes


def test_every_params_file_keeps_the_2026_abort_mode():
    for file in sorted(PARAMS.glob("*.yaml")):
        executor = (yaml.safe_load(file.read_text()) or {}).get("mission_executor", {}).get("ros__parameters", {})
        assert executor.get("abort_rtl_mode", "RTL") == "RTL", file.name
    declared = (REPO / "src" / "drone_mission_core" / "drone_mission_core" / "mission_executor.py").read_text()
    assert 'self.declare_parameter("abort_rtl_mode", "RTL")' in declared  # the executor's own default is 2026's
