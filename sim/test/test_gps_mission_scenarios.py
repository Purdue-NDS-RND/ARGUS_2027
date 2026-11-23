"""PLT-13's sim scenarios: the waypoint circuit with holds and the time trial as an AUTO mission."""

import json
import sys
from pathlib import Path

import pytest
import yaml

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

from scenario.build import build_run  # noqa: E402

CASES = [
    ("waypoint_navigation/circuit_hold", "waypoint_navigation", {"waypoint_timeout_hold"}, "calm"),
    ("waypoint_navigation/circuit_hold_wind", "waypoint_navigation", {"waypoint_timeout_hold"}, "moderate"),
    ("time_trial/circuit_auto", "time_trial", {"time_trial_auto"}, "calm"),
    ("time_trial/circuit_auto_wind", "time_trial", {"time_trial_auto"}, "moderate"),
    ("time_trial/circuit_flying", "time_trial", {"time_trial_auto", "flying_start"}, "calm"),
]


@pytest.mark.parametrize("name, mission, flags, wind", CASES)
def test_a_gps_scenario_turns_on_its_one_flag_over_the_site_circuit(name, mission, flags, wind, tmp_path):
    run = build_run(SIM / "scenarios" / f"{name}.yaml", tmp_path / "on", variant="on")
    manifest = json.loads((run / "scenario.json").read_text())
    sequence = yaml.safe_load((run / "sequence.yaml").read_text())["mission_sequence"]
    assert manifest["mission"] == mission and manifest["wind"]["level"] == wind and manifest["sim_time"] is False
    assert {flag for flag, value in sequence["features"].items() if value} == flags
    config = next(item["config"] for item in sequence["missions"] if item["type"] == "gps_waypoint")
    truth = json.loads((run / "truth.json").read_text())
    assert len(truth["waypoints"]) == 7 and Path(config["waypoint_file"]).is_file()
    if mission == "waypoint_navigation":
        assert (config["hold_time_s"], config["waypoint_timeout_s"], config["arrival_radius_m"]) == (3.0, 90.0, 1.0)  # the 2026 radius, with a longer hold inside it
    else:
        assert config["mode"] == "circuit_time_trial" and config["hold_time_s"] == 0.0
    # Flags off: every flag false. The scenario's settings stand (the 2026 mission holds 3 s too); the timeout is
    # then read by nothing.
    off = build_run(SIM / "scenarios" / f"{name}.yaml", tmp_path / "off", variant="off")
    shipped = yaml.safe_load((off / "sequence.yaml").read_text())["mission_sequence"]
    assert not any(shipped["features"].values())


def test_the_timeout_scenario_holds_one_waypoint_by_name_and_gives_one_too_little_time(tmp_path):
    run = build_run(SIM / "scenarios" / "waypoint_navigation" / "circuit_timeout.yaml", tmp_path / "on", variant="on")
    sequence = yaml.safe_load((run / "sequence.yaml").read_text())["mission_sequence"]
    config = next(item["config"] for item in sequence["missions"] if item["type"] == "gps_waypoint")
    assert sequence["features"]["waypoint_timeout_hold"] is True and config["waypoint_holds_s"] == {"wp2": 6.0} and config["waypoint_timeouts_s"] == {"wp4": 3.0}
    assert config["hold_time_s"] == 0.5 and config["waypoint_timeout_s"] is None  # the 2026 hold elsewhere; no timeout but wp4's
