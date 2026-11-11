"""sim/gps: deciding when a flight is over, the run summary, and script guards.

flight_log.py decides when run_mission.sh stops a mission launch and whether
the run passed. Getting it wrong either stops a launch while the vehicle is
still flying, or reports PASS for a vehicle that never left the ground (both
happened while bringing this sim up). No ROS needed.
"""

import re
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
GPS = ROOT / "sim" / "gps"
sys.path.insert(0, str(GPS))

import flight_log  # noqa: E402
from flight_log import DISARM_HOLD_S, FlightTracker, format_summary, summarize  # noqa: E402

ON_GROUND, IN_AIR, TAKEOFF, LANDING = 1, 2, 3, 4


def flown(tracker: FlightTracker, t0: float = 0.0) -> float:
    """Arm, take off, fly, land. Returns the landing time."""
    tracker.update(t0, landed_state=ON_GROUND)
    tracker.update(t0 + 1, armed=True)
    tracker.update(t0 + 5, landed_state=TAKEOFF)
    tracker.update(t0 + 10, landed_state=IN_AIR)
    tracker.update(t0 + 80, landed_state=LANDING)
    tracker.update(t0 + 120, landed_state=ON_GROUND)
    return t0 + 120


def test_flight_is_over_only_after_disarm_has_lasted():
    tracker = FlightTracker()
    landed = flown(tracker)
    assert not tracker.finished(landed + 10)  # still armed
    tracker.update(landed + 1, armed=False)
    assert not tracker.finished(landed + 1 + DISARM_HOLD_S / 2)
    assert tracker.finished(landed + 1 + DISARM_HOLD_S)
    assert tracker.airborne


def test_brief_disarm_glitch_mid_climb_does_not_end_the_flight():
    # The case that stopped a real launch: one armed=false state message while
    # extended_state still said on ground, a few seconds into the climb.
    tracker = FlightTracker()
    tracker.update(0, landed_state=ON_GROUND)
    tracker.update(1, armed=True)
    tracker.update(10, armed=False)
    tracker.update(10.2, armed=True)
    assert not tracker.finished(10.2 + 10 * DISARM_HOLD_S)
    assert tracker.disarm_time is None


def test_disarm_on_ground_without_takeoff_ends_but_is_not_airborne():
    tracker = FlightTracker()
    tracker.update(0, landed_state=ON_GROUND)
    tracker.update(1, armed=True)
    tracker.update(11, armed=False)  # ArduPilot auto-disarm, takeoff refused
    assert tracker.finished(11 + DISARM_HOLD_S)
    assert not tracker.airborne


def test_disarm_before_any_arm_is_ignored():
    tracker = FlightTracker()
    tracker.update(0, armed=False, landed_state=ON_GROUND)
    assert tracker.disarm_time is None
    assert not tracker.finished(100)


def write_run(tmp_path: Path, *, airborne: bool = True, completed=("takeoff", "wp", "rtl"),
              sequence_done: bool = True) -> Path:
    (tmp_path / "expected.txt").write_text("takeoff\nwp\nrtl\n")
    launch = [f"[mission_executor-2] [INFO] [1.0] [mission_executor]: Mission '{name}' completed"
              for name in completed]
    if sequence_done:
        launch.append("[mission_executor-2] [INFO] [2.0] [mission_executor]: Mission sequence complete")
    launch += [
        "[mission_executor-2] [INFO] [3.0] [mission_executor]: [wp] GPS waypoint advisory summary:",
        "[mission_executor-2] [INFO] [3.0] [mission_executor]: [wp]   wp1: closest 0.42 m",
        "[mission_executor-2] [INFO] [3.0] [mission_executor]: [wp]   wp2: closest 1.75 m",
        "[mission_executor-2] [INFO] [3.0] [mission_executor]: [wp]   circuit elapsed: 61.25 s",
    ]
    (tmp_path / "launch.log").write_text("\n".join(launch) + "\n")
    states = ["100.0 armed=0 mode=STABILIZE landed=1", "101.0 armed=1 mode=GUIDED landed=1"]
    if airborne:
        states += ["105.0 armed=1 mode=GUIDED landed=2", "190.0 armed=1 mode=RTL landed=1"]
    states += ["200.0 link-down (ignored)", "201.0 armed=0 mode=RTL landed=1"]
    (tmp_path / "state.log").write_text("\n".join(states) + "\n")
    # The 0.40 sample is before arming and must not count.
    (tmp_path / "rtf.log").write_text("99.0 0.40\n120.0 0.98\n150.0 1.00\n")
    return tmp_path


def test_summary_of_a_good_run(tmp_path):
    summary = summarize(write_run(tmp_path))
    assert summary.passed
    assert summary.flight_s == pytest.approx(100.0)
    assert summary.closest_m == [("wp1", 0.42), ("wp2", 1.75)]
    assert summary.circuit_s == pytest.approx(61.25)
    assert summary.rtf_samples == [0.98, 1.00]
    text = format_summary(summary)
    assert text.startswith("RESULT: PASS")
    assert "(wall-clock): 100.0 s" in text
    assert "mean 0.99" in text


def test_summary_fails_when_the_vehicle_never_left_the_ground(tmp_path):
    summary = summarize(write_run(tmp_path, airborne=False))
    assert not summary.passed
    assert "left the ground: NO" in format_summary(summary)


def test_summary_fails_when_a_mission_did_not_complete(tmp_path):
    summary = summarize(write_run(tmp_path, completed=("takeoff",), sequence_done=False))
    assert not summary.passed
    assert summary.missing == ["wp", "rtl"]


def test_main_exit_code_follows_the_result(tmp_path, capsys):
    assert flight_log.main(["flight_log.py", str(write_run(tmp_path))]) == 0
    assert flight_log.main(["flight_log.py", str(write_run(tmp_path, airborne=False))]) == 1


SCRIPTS = ["common.sh", "env.sh", "start_mavros.sh", "set_home.sh", "run_mission.sh", "fly_all.sh"]


@pytest.mark.parametrize("name", SCRIPTS)
def test_scripts_parse(name):
    subprocess.run(["bash", "-n", str(GPS / name)], check=True)


def test_ros_domain_is_not_the_default():
    # Host networking shares localhost with the host's ROS 2 Jazzy, so
    # ROS_LOCALHOST_ONLY alone does not isolate us.
    env = (GPS / "env.sh").read_text()
    match = re.search(r"ROS_DOMAIN_ID=\$\{ARGUS_ROS_DOMAIN_ID:-(\d+)\}", env)
    assert match and int(match.group(1)) != 0
    common = (GPS / "common.sh").read_text()
    assert f"ARGUS_ROS_DOMAIN_ID:-{match.group(1)}" in common


def test_mavros_does_not_take_the_qgc_port():
    text = (GPS / "start_mavros.sh").read_text()
    default = re.search(r"FCU_URL:-([^}]+)\}", text).group(1)
    assert "14550" not in default


def test_home_altitude_matches_terrain_not_a_guess():
    # SITL's rangefinder measures to SRTM terrain: at 850 m it read 18.5 m on
    # the ground and ArduPilot refused the 10 m GUIDED takeoff.
    text = (GPS / "set_home.sh").read_text()
    alt = float(re.search(r"HOME_ALT=\$\{HOME_ALT:-([0-9.]+)\}", text).group(1))
    assert 829.0 < alt < 834.0


def test_mavros_plugin_list_keeps_what_our_code_uses():
    import yaml  # noqa: PLC0415  (only this test needs it)

    data = yaml.safe_load((GPS / "mavros_pluginlists.yaml").read_text())
    denied = set(data["/**"]["ros__parameters"]["plugin_denylist"])
    assert {"param", "waypoint", "geofence", "rallypoint"} <= denied
    assert not denied & {"sys_status", "command", "setpoint_raw", "setpoint_position",
                         "local_position", "global_position", "imu", "sys_time", "rangefinder"}
