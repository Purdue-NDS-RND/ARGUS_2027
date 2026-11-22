"""The recovery mission's sim scenarios (REC-2): what the build hands the runner for each."""

import json
import sys
from pathlib import Path

import pytest
import yaml

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

from scenario.build import build_run  # noqa: E402
from target_placement import offset_ne_m  # noqa: E402

SCENARIOS = SIM / "scenarios" / "recovery"


def built(name, tmp_path, variant="on"):
    run = build_run(SCENARIOS / f"{name}.yaml", tmp_path / f"{name}-{variant}", variant=variant)
    manifest = json.loads((run / "scenario.json").read_text())
    sequence = yaml.safe_load((run / "sequence.yaml").read_text())["mission_sequence"]
    params = yaml.safe_load((run / "params.yaml").read_text())
    return run, manifest, sequence, params


@pytest.mark.parametrize("name, node", [("pickup_oracle", "loc_node"), ("pickup_detector", "recovery_cv")])
def test_a_recovery_mission_scenario_turns_the_mission_and_its_one_detector_on(name, node, tmp_path):
    run, manifest, sequence, params = built(name, tmp_path)
    assert manifest["launch"]["file"] == "sim_package_recovery.launch.py" and manifest["launch"]["arguments"][node] == "true"
    assert manifest["launch"]["arguments"]["use_sim_time"] == "true" and manifest["mission"] == "recovery"
    other = "recovery_cv" if node == "loc_node" else "loc_node"
    assert other not in manifest["launch"]["arguments"]
    on = {flag for flag, value in sequence["features"].items() if value}
    assert on == {"use_frame_time_pose", "use_target_estimator", "use_height_estimator", "package_recovery"}
    # The pickup's coordinates are the scenario's bucket's, from the site's origin.
    recovery = next(item for item in sequence["missions"] if item["type"] == "package_recovery")
    truth = json.loads((run / "truth.json").read_text())
    north, east = offset_ne_m(truth["origin"]["latitude_deg"], truth["origin"]["longitude_deg"], *recovery["config"]["pickup"]["latlon"])
    assert abs(east - 12.0) < 0.02 and abs(north - 8.0) < 0.02 and truth["bucket"]["name"] == "bucket"
    # The hook and the bucket are the payload bridge's, and the vehicle is the recovery one.
    assert manifest["payload_bridge"]["bucket"] == "bucket" and manifest["payload_bridge"]["hook_offset_m"] == [0.0, 0.0, -0.55]
    assert params["mission_executor"]["ros__parameters"]["abort_rtl_mode"] == "LAND"
    # The checks that compare a bullseye's detections and track with the truth have nothing to compare here (the
    # first flight was failed by them, with the bucket set down 6 mm from the point): they are not run or asked for.
    import estimator_check
    import projection_check
    from runner.events import tools_needed

    assert manifest["target"] is False and "target" not in truth
    import loc_check

    assert not projection_check.wanted(manifest) and not estimator_check.wanted(manifest) and not loc_check.wanted(manifest)  # (the node is the oracle here, not a survey's pipeline)
    tools = tools_needed(manifest)
    assert {"payload_bridge", "truth_poses", "height_check", "time_check"} <= tools and not {"projection_check", "estimator_check", "loc_check"} & tools
    assert ("recovery_cv_check" in tools) == (node == "recovery_cv")
    if node == "loc_node":
        features = params["loc_node"]["ros__parameters"]["features"]
        assert features["loc_node"] and features["sim_oracle_detector"] and not params["recovery_cv"]["ros__parameters"]["features"]["recovery_cv"]
        oracle = json.loads((run / "oracle_objects.json").read_text())
        assert {item["class_id"] for item in oracle["objects"]} == {"green_x", "bucket"} and oracle["vehicle"] == "iris_recovery"
    else:
        assert params["recovery_cv"]["ros__parameters"]["features"]["recovery_cv"] and not any(params["loc_node"]["ros__parameters"]["features"].values())


def test_the_gates_flight_is_the_oracle_pickup_with_a_coordinate_error_and_a_glitch(tmp_path):
    offsets = set()
    for seed in (1, 2, 3):
        run = build_run(SCENARIOS / "gate_pickup.yaml", tmp_path / f"gate-{seed}", variant="on", seed=seed)
        manifest = json.loads((run / "scenario.json").read_text())
        sequence = yaml.safe_load((run / "sequence.yaml").read_text())["mission_sequence"]
        truth = json.loads((run / "truth.json").read_text())
        pickup = next(item for item in sequence["missions"] if item["type"] == "package_recovery")["config"]["pickup"]["latlon"]
        north, east = offset_ne_m(truth["origin"]["latitude_deg"], truth["origin"]["longitude_deg"], *pickup)
        off = ((east - 12.0) ** 2 + (north - 8.0) ** 2) ** 0.5
        assert off <= 8.0 + 0.02
        offsets.add(round(off, 2))
        assert manifest["gps_events"] == [{"at_s": 41.0, "kind": "glitch", "north_m": 5.0, "for_s": 10.0}]
        assert manifest["launch"]["arguments"]["loc_node"] == "true" and manifest["target"] is False
    assert len(offsets) == 3  # the seed draws the error


def test_with_the_flags_off_the_recovery_scenario_flies_the_shipped_sequence_and_starts_no_detector(tmp_path):
    _run, manifest, sequence, params = built("pickup_oracle", tmp_path, variant="off")
    assert not any(sequence["features"].values()) and manifest["flags"] == {} and manifest["node_flags"] == {}
    assert "loc_node" not in manifest["launch"]["arguments"] and "use_sim_time" not in manifest["launch"]["arguments"]
    assert not any(params["loc_node"]["ros__parameters"]["features"].values())
