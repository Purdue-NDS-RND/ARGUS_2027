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
from flight_log import DISARM_HOLD_S, FlightTracker, FlightWatch, format_summary, summarize  # noqa: E402

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


class Clocks:
    """Stand-ins for time.time() and time.monotonic(). A laptop suspend moves
    the wall clock forward and leaves the monotonic clock where it was."""

    def __init__(self) -> None:
        self.wall = 1_800_000_000.0
        self.mono = 5_000.0

    def run(self, seconds: float) -> None:
        self.wall += seconds
        self.mono += seconds

    def suspend(self, seconds: float) -> None:
        self.wall += seconds


@pytest.fixture
def clocks(monkeypatch):
    c = Clocks()
    monkeypatch.setattr(flight_log.time, "time", lambda: c.wall)
    monkeypatch.setattr(flight_log.time, "monotonic", lambda: c.mono)
    return c


def test_suspend_mid_flight_does_not_fire_the_flight_timeout(clocks):
    # Flight 3 of the first sim session: the laptop suspended a few seconds
    # into the time trial. On the wall clock a 2 h suspend is 2 h elapsed.
    watch = FlightWatch(FlightTracker())  # default clock, as flight_monitor.py uses it
    watch.tracker.update(watch.now(), landed_state=ON_GROUND)
    clocks.run(5)
    watch.tracker.update(watch.now(), armed=True)
    watch.tracker.update(watch.now(), landed_state=IN_AIR)
    clocks.run(10)
    clocks.suspend(2 * 3600)
    assert watch.check() is None  # still flying: keep waiting
    clocks.run(watch.timeout_s)
    assert watch.check() == flight_log.TIMED_OUT  # real elapsed time still counts


def test_suspend_before_arming_does_not_fire_the_arm_timeout(clocks):
    watch = FlightWatch(FlightTracker())
    clocks.run(10)
    clocks.suspend(3600)
    assert watch.check() is None
    clocks.run(watch.arm_timeout_s)
    assert watch.check() == flight_log.NEVER_ARMED


def test_watch_ends_after_a_disarm_on_the_ground(clocks):
    watch = FlightWatch(FlightTracker())
    flown(watch.tracker, t0=watch.now())
    watch.tracker.update(watch.now() + 121, armed=False)
    clocks.run(121 + DISARM_HOLD_S)
    assert watch.check() == flight_log.DONE


def test_flight_time_comes_from_the_monotonic_stamps(tmp_path):
    # Armed 100 s by the monotonic clock, with a 2 h suspend in the middle.
    run = write_run(tmp_path)
    (run / "state.log").write_text(
        "100.0 armed=0 mode=STABILIZE landed=1 mono=5000.0\n"
        "101.0 armed=1 mode=GUIDED landed=1 mono=5001.0\n"
        "105.0 armed=1 mode=GUIDED landed=2 mono=5005.0\n"
        "7390.0 armed=1 mode=RTL landed=1 mono=5090.0\n"
        "7400.0 link-down (ignored) mono=5100.0\n"
        "7401.0 armed=0 mode=RTL landed=1 mono=5101.0\n"
    )
    summary = summarize(run)
    assert summary.passed
    assert summary.flight_s == pytest.approx(100.0)
    assert "(monotonic): 100.0 s" in format_summary(summary)


def test_flight_monitor_takes_its_times_from_the_watch():
    # flight_monitor.py needs rclpy, so check its source: the only wall-clock
    # reads left are the epoch stamps it prints for lining up with rtf.log.
    text = (GPS / "flight_monitor.py").read_text()
    assert "FlightWatch(" in text and "watch.check()" in text
    for line in text.splitlines():
        if "time.time()" in line:
            assert line.strip().startswith("print(f\"{time.time():.3f}"), line
    assert "time.monotonic" not in text  # one clock, owned by FlightWatch


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


STUB_INHIBIT = """#!/usr/bin/env bash
echo "$*" >> "$INHIBIT_LOG"
(( $(wc -l < "$INHIBIT_LOG") > 4 )) && { echo "re-exec loop" >&2; exit 99; }
[[ -n ${INHIBIT_FAIL:-} ]] && { echo "Failed to inhibit: Access denied" >&2; exit 1; }
while [[ ${1:-} == --* ]]; do shift; done
[[ ${1:-} == true ]] && exit 0   # the probe
exec "$@"
"""


def run_under_stub_inhibit(tmp_path: Path, **env_extra) -> tuple[subprocess.CompletedProcess, list[str]]:
    """A script that sources common.sh and calls inhibit_sleep, as
    run_mission.sh and fly_all.sh do, with a stand-in systemd-inhibit."""
    import os  # noqa: PLC0415

    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    stub = bin_dir / "systemd-inhibit"
    stub.write_text(STUB_INHIBIT)
    stub.chmod(0o755)
    script = tmp_path / "fly.sh"
    script.write_text(
        f'source "{GPS / "common.sh"}"\n'
        'inhibit_sleep "$0" "$@"\n'
        'echo "flying $* locked=${ARGUS_SLEEP_INHIBITED:-no}"\n'
    )
    log = tmp_path / "inhibit.log"
    env = {k: v for k, v in os.environ.items() if k != "ARGUS_SLEEP_INHIBITED"}
    env.update(PATH=f"{bin_dir}:{env['PATH']}", INHIBIT_LOG=str(log), **env_extra)
    proc = subprocess.run(["bash", str(script), "square_only", "--yes"], env=env,
                          capture_output=True, text=True, check=True, timeout=30)
    calls = log.read_text().splitlines() if log.exists() else []
    return proc, calls


def test_flights_run_once_under_a_sleep_and_lid_lock(tmp_path):
    proc, calls = run_under_stub_inhibit(tmp_path)
    assert proc.stdout.splitlines() == ["flying square_only --yes locked=1"]
    probe, lock = calls
    assert "--what=sleep:idle:handle-lid-switch" in lock and "--mode=block" in lock
    assert lock.endswith("fly.sh square_only --yes")


def test_a_script_already_locked_takes_no_second_lock(tmp_path):
    # fly_all.sh holds the lock; the run_mission.sh it calls must not re-exec.
    proc, calls = run_under_stub_inhibit(tmp_path, ARGUS_SLEEP_INHIBITED="1")
    assert proc.stdout.splitlines() == ["flying square_only --yes locked=1"]
    assert calls == []


def test_no_lock_warns_and_still_flies_once(tmp_path):
    proc, calls = run_under_stub_inhibit(tmp_path, INHIBIT_FAIL="1")
    assert proc.stdout.splitlines() == ["flying square_only --yes locked=no"]
    assert "WARNING: could not take a systemd-inhibit lock" in proc.stderr
    assert len(calls) == 1  # the probe only


@pytest.mark.parametrize("name", ["run_mission.sh", "fly_all.sh"])
def test_flight_scripts_take_the_lock_before_flying(name):
    text = (GPS / name).read_text()
    lock = text.index('inhibit_sleep "$0" "$@"')
    flies = text.index("docker exec -d") if name == "run_mission.sh" else text.index("for sequence in")
    assert lock < flies


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
