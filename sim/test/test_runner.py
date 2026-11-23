"""sim/runner: what turns a flight in the sim into a scored, reported run.

The runner flies with nobody watching, so these pieces decide things a person
used to: which log lines are a release, when the flight is over (in sim time,
not wall time), whether the sim ran too slowly for the result to count, and
what the report says. No ROS needed; the ROS node in fly.py only feeds them.
"""

import json
import math
import sys
from pathlib import Path

import pytest
import yaml

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

from runner import events, result, suites  # noqa: E402
from runner.trace import TraceWriter, real_time_factor  # noqa: E402
from runner.watch import EndWatch  # noqa: E402

from judge.record import Record  # noqa: E402


# --- events from what the mission and the autopilot say ------------------------


@pytest.mark.parametrize(
    "line, fake",
    [
        ("[package_delivery] Fake delivery complete", True),
        ("[package_drop] Fake drop complete", True),
        ("[package_delivery] Releasing gripper payload", False),
        ("[package_drop] Closing sprayer for payload drop", False),
    ],
)
def test_release_lines_become_release_events(line, fake):
    assert events.from_mission_log(line) == [{"event": "release", "fake": fake, "what": line}]


@pytest.mark.parametrize("line", ["[package_delivery] Holding on target for 5.0 s before relaunch", "Mission 'rtl' completed", ""])
def test_other_mission_lines_are_not_events(line):
    assert events.from_mission_log(line) == []


def test_autopilot_text_becomes_fence_and_failsafe_events():
    assert events.from_statustext("Polygon fence breached") == [{"event": "fence_breach", "what": "Polygon fence breached"}]
    assert events.from_statustext("Max altitude fence breached") == [{"event": "fence_breach", "what": "Max altitude fence breached"}]
    assert events.from_statustext("EKF Failsafe: changed to LAND Mode") == [{"event": "failsafe", "what": "EKF Failsafe: changed to LAND Mode"}]
    assert events.from_statustext("Radio Failsafe - Continuing Auto Mode") == [{"event": "failsafe", "what": "Radio Failsafe - Continuing Auto Mode"}]
    assert events.from_statustext("Fence breach cleared") == []
    assert events.from_statustext("EKF3 IMU0 is using GPS") == []
    assert events.from_statustext("PrecLand: Failsafe Measures") == []  # a status line at landing, not a failsafe


def test_arm_watcher_reports_changes_and_ignores_a_dropped_link():
    watcher = events.ArmWatcher()
    assert watcher.update(connected=True, armed=False) is None
    assert watcher.update(connected=True, armed=True) == "armed"
    assert watcher.update(connected=True, armed=True) is None
    # MAVROS says armed=false while it thinks the link is down: that is not a disarm.
    assert watcher.update(connected=False, armed=False) is None
    assert watcher.update(connected=True, armed=True) is None
    assert watcher.update(connected=True, armed=False) == "disarmed"


def test_mode_watcher_reports_each_new_mode_once():
    watcher = events.ModeWatcher()
    assert watcher.update(connected=True, mode="GUIDED") == "GUIDED"
    assert watcher.update(connected=True, mode="GUIDED") is None
    assert watcher.update(connected=False, mode="") is None  # link down: MAVROS publishes an empty mode
    assert watcher.update(connected=True, mode="GUIDED") is None
    assert watcher.update(connected=True, mode="RTL") == "RTL"


def test_ground_station_commands_inside_the_flight_are_human_input():
    messages = [
        (100.0, 255, "HEARTBEAT"),  # the ground station being there is not a command
        (101.0, 255, "REQUEST_DATA_STREAM"),
        (102.0, 255, "PARAM_REQUEST_LIST"),
        (105.0, 255, "SET_MODE"),
        (106.0, 255, "RC_CHANNELS_OVERRIDE"),
        (107.0, 1, "COMMAND_LONG"),  # our own companion (MAVROS), not a person
        (108.0, 245, "PARAM_SET"),  # our own tools
        (109.0, 255, "COMMAND_LONG"),
        (300.0, 255, "MISSION_ITEM_INT"),
    ]
    found = events.human_inputs(messages)
    assert [(item["wall"], item["event"], item["what"]) for item in found] == [
        (105.0, "gcs_command", "SET_MODE"),
        (106.0, "rc_override", "RC_CHANNELS_OVERRIDE"),
        (109.0, "gcs_command", "COMMAND_LONG"),
        (300.0, "gcs_command", "MISSION_ITEM_INT"),
    ]


# --- the trace ------------------------------------------------------------------


def test_trace_writer_writes_what_the_judge_reads(tmp_path):
    path = tmp_path / "trace.jsonl"
    writer = TraceWriter(path, pose_interval_s=0.05)
    assert writer.pose(1.00, "vehicle", 1.0, 2.0, 3.0) is True
    assert writer.pose(1.02, "vehicle", 1.1, 2.0, 3.0) is False  # too soon: 20 Hz is enough
    assert writer.pose(1.02, "bucket", 5.0, 6.0, 0.1) is True  # each object has its own rate
    assert writer.pose(1.05, "vehicle", 1.2, 2.0, 3.0) is True
    writer.event(1.5, "release", fake=True)
    writer.clock(2.0, 1000.0)
    writer.close()
    record = Record.load(path)
    assert [(p.t, p.e) for p in record.track("vehicle")] == [(1.0, 1.0), (1.05, 1.2)]
    assert record.final("bucket").u == pytest.approx(0.1)
    assert record.events_of("release")[0]["fake"] is True
    assert record.events_of("clock")[0]["wall"] == 1000.0


def test_trace_writer_keeps_the_vehicles_true_heading_when_it_is_given(tmp_path):
    from runner.trace import yaw_of_quaternion

    path = tmp_path / "trace.jsonl"
    writer = TraceWriter(path)
    half = math.radians(90.0) / 2.0
    writer.pose(1.0, "vehicle", 1.0, 2.0, 3.0, yaw=yaw_of_quaternion(0.0, 0.0, math.sin(half), math.cos(half)))
    writer.pose(1.0, "bucket", 5.0, 6.0, 0.1)
    writer.close()
    record = Record.load(path)
    assert record.track("vehicle")[0].yaw == pytest.approx(math.pi / 2.0, abs=1e-4)  # nose along +y: north
    assert record.track("bucket")[0].yaw is None
    assert "yaw" not in json.loads(path.read_text().splitlines()[1])
    assert yaw_of_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)  # nose along +x: east


def test_trace_writer_survives_being_killed_mid_run(tmp_path):
    path = tmp_path / "trace.jsonl"
    writer = TraceWriter(path)
    writer.pose(1.0, "vehicle", 0.0, 0.0, 0.0)
    writer.event(1.0, "armed")
    # No close(): every line is already on disk.
    assert len(Record.load(path).events) == 1


def clock_lines(pairs):
    return [{"t": sim, "event": "clock", "wall": wall} for sim, wall in pairs]


def test_real_time_factor_is_sim_time_over_wall_time_during_the_flight():
    lines = clock_lines([(0.0, 1000.0), (10.0, 1020.0), (50.0, 1070.0), (100.0, 1120.0), (110.0, 1160.0)])
    record = Record.from_lines(lines)
    assert real_time_factor(record, 10.0, 100.0) == pytest.approx(90.0 / 100.0)
    assert real_time_factor(record, 0.0, 10.0) == pytest.approx(0.5)
    assert real_time_factor(Record.from_lines([]), 0.0, 10.0) is None


# --- when the flight is over ---------------------------------------------------------


class SimClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


ON_GROUND, IN_AIR = 1, 2


def test_flight_ends_when_disarmed_on_the_ground_and_settled():
    clock = SimClock()
    watch = EndWatch(clock, timeout_s=600.0, arm_timeout_s=120.0, settle_s=5.0)
    watch.update(landed_state=ON_GROUND)
    assert watch.check() is None
    clock.t = 10.0
    watch.update(armed=True)
    clock.t = 20.0
    watch.update(landed_state=IN_AIR)
    clock.t = 200.0
    watch.update(landed_state=ON_GROUND)
    watch.update(armed=False)
    assert watch.check() is None
    clock.t = 204.0
    assert watch.check() is None  # still settling
    clock.t = 205.5
    assert watch.check() == "DONE"


def test_flight_ends_on_true_height_when_the_autopilot_sends_no_landed_state():
    """A flight flown by a tool, not the 2026 takeoff service: nobody asked for the landed state."""
    clock = SimClock()
    height = {"value": 0.0}
    watch = EndWatch(clock, timeout_s=600.0, settle_s=5.0, height=lambda: height["value"])
    clock.t = 10.0
    watch.update(armed=True)
    height["value"] = 15.0
    clock.t = 100.0
    watch.update(armed=False)
    clock.t = 110.0
    assert watch.check() is None  # disarmed 15 m up is a falling vehicle, not a landing
    height["value"] = 0.02
    assert watch.check() == "DONE"  # down, and the disarm has lasted

    clock.t = 0.0
    watch = EndWatch(clock, timeout_s=600.0, settle_s=5.0, height=lambda: 0.02)
    clock.t = 10.0
    watch.update(armed=True)
    clock.t = 100.0
    watch.update(armed=False)
    clock.t = 104.0
    assert watch.check() is None  # the run settles for 5 s after the disarm
    clock.t = 105.5
    assert watch.check() == "DONE"


def test_true_height_does_not_end_a_flight_that_is_still_armed():
    clock = SimClock()
    watch = EndWatch(clock, timeout_s=600.0, height=lambda: 0.0)
    clock.t = 10.0
    watch.update(armed=True)
    clock.t = 100.0
    assert watch.check() is None  # on the ground and armed: the delivery mission's ground dwell


def test_flight_times_out_in_sim_time_not_wall_time():
    clock = SimClock()
    watch = EndWatch(clock, timeout_s=600.0, arm_timeout_s=120.0)
    clock.t = 10.0
    watch.update(armed=True)
    watch.update(landed_state=IN_AIR)
    clock.t = 600.0
    assert watch.check() is None
    clock.t = 601.0
    assert watch.check() == "TIMEOUT"


def test_vehicle_that_never_arms_is_reported():
    clock = SimClock()
    watch = EndWatch(clock, timeout_s=600.0, arm_timeout_s=120.0)
    clock.t = 119.0
    assert watch.check() is None
    clock.t = 121.0
    assert watch.check() == "NEVER_ARMED"


def test_watch_starts_counting_from_when_it_was_made():
    clock = SimClock()
    clock.t = 5000.0  # the sim had been up a while
    watch = EndWatch(clock, timeout_s=600.0, arm_timeout_s=120.0)
    clock.t = 5100.0
    assert watch.check() is None


# --- the result of a run ---------------------------------------------------------------


def make_run(tmp_path, outcome="DONE", rtf=0.95, score=None, manifest=None, flight=(10.0, 210.0)):
    run = tmp_path / "run"
    run.mkdir(parents=True, exist_ok=True)
    base = {
        "name": "baseline", "mission": "delivery", "seed": 1, "variant": "off", "flags": {}, "timeout_s": 600.0,
        "wind": {"level": "calm", "mean_mps": 0.0, "gust_mps": 0.0, "discrete_gust_mps": 0.0, "direction_deg": 0.0}, "gps_events": [],
    }  # fmt: skip
    (run / "scenario.json").write_text(json.dumps({**base, **(manifest or {})}))
    if outcome is not None:
        (run / "flight.json").write_text(json.dumps({"outcome": outcome, "t_launch": 5.0, "t_end": flight[1] + 5.0}))
    t_arm, t_disarm = flight
    lines = [{"t": t_arm, "event": "armed"}, {"t": t_disarm, "event": "disarmed"}]
    lines += clock_lines([(t, 1000.0 + t / rtf) for t in range(0, int(t_disarm) + 20, 5)])
    (run / "trace.jsonl").write_text("".join(json.dumps(line) + "\n" for line in lines))
    if score is not None:
        (run / "score.json").write_text(json.dumps(score))
    return run


SCORE = {
    "mission": "delivery", "points": 16.0, "base_points": 12.8, "autonomy_bonus": 3.2,
    "details": {"distance_m": 0.2, "released": True, "release_height_m": 0.0},
    "common": {"flights": 1, "single_flight": True, "autonomous": True, "human_inputs": [], "fence_breaches": [], "reported_fence_breaches": 0, "failsafes": []},
    "notes": [],
}  # fmt: skip


def test_scored_flight_at_good_speed_is_ok(tmp_path):
    summary = result.run_result(make_run(tmp_path, score=SCORE))
    assert summary["status"] == "OK"
    assert summary["points"] == 16.0
    assert summary["headline"] == "0.20 m from the target"
    assert summary["real_time_factor"] == pytest.approx(0.95, abs=0.01)
    assert summary["flight_s"] == pytest.approx(200.0)


def test_headline_for_the_geofence_check():
    passed = {"mission": "geofence", "details": {"passed": True, "response_delay_s": 0.04, "max_excursion_m": 0.99}}
    assert result.headline(passed) == "response +0.04 s from the line, 0.99 m outside"
    inside = {"mission": "geofence", "details": {"passed": True, "response_delay_s": None, "max_excursion_m": 0.0}}
    assert result.headline(inside) == "stopped before the line, 0.00 m outside"
    assert result.headline({"mission": "geofence", "details": {"passed": False}}) == "fence not held"


@pytest.mark.parametrize("rtf, status", [(0.82, "OK"), (0.78, "INVALID")])
def test_run_slower_than_0_8_real_time_is_invalid(rtf, status, tmp_path):
    summary = result.run_result(make_run(tmp_path, rtf=rtf, score=SCORE))
    assert summary["status"] == status
    if status == "INVALID":
        assert "real-time factor" in summary["reason"]
        assert summary["points"] == 16.0  # still shown, but it does not count


@pytest.mark.parametrize(
    "outcome, score, status",
    [("TIMEOUT", SCORE, "TIMEOUT"), ("NEVER_ARMED", None, "NEVER_ARMED"), (None, None, "ERROR"), ("DONE", None, "ERROR")],
)
def test_flights_that_did_not_finish_say_so(outcome, score, status, tmp_path):
    summary = result.run_result(make_run(tmp_path, outcome=outcome, score=score))
    assert summary["status"] == status
    assert summary["reason"]


def test_report_has_the_numbers_a_person_looks_for(tmp_path):
    score = json.loads(json.dumps(SCORE))
    score["common"]["fence_breaches"] = [{"kind": "lateral", "t_start": 50.0, "t_end": 52.0, "max_excursion_m": 3.2}]
    score["common"]["failsafes"] = [{"t": 51.0, "what": "Fence"}]
    score["notes"] = ["delivery: something worth reading"]
    report = result.render_report(result.run_result(make_run(tmp_path, score=score)))
    for expected in ("delivery/baseline", "seed 1", "flags off", "OK", "16.00", "0.20 m from the target", "0.95", "200", "3.2 m", "Fence", "something worth reading"):
        assert expected in report, expected


def test_suite_table_puts_flags_off_beside_flags_on(tmp_path):
    rows = []
    for variant, points, distance in (("off", 15.0, 1.0), ("on", 16.0, 0.2)):
        score = json.loads(json.dumps(SCORE))
        score["points"], score["details"]["distance_m"] = points, distance
        rows.append(result.run_result(make_run(tmp_path / variant, score=score, manifest={"variant": variant})))
    rows.append(result.run_result(make_run(tmp_path / "slow", rtf=0.5, score=SCORE, manifest={"seed": 2})))
    table = result.suite_table(rows)
    lines = [line for line in table.splitlines() if line.startswith("| delivery/baseline")]
    assert len(lines) == 2  # one row per scenario and seed
    assert "15.00" in lines[0] and "16.00" in lines[0]
    assert lines[0].index("15.00") < lines[0].index("16.00")  # off (2026) first, then on
    assert "1.00 m" in lines[0] and "0.20 m" in lines[0]
    assert "INVALID" in lines[1] and "not run" in lines[1]
    assert "3 runs: 2 OK, 1 INVALID" in table


# --- suites ---------------------------------------------------------------------------------


def test_shipped_suites_name_scenarios_that_exist():
    loaded = suites.load_suites()
    assert {"smoke", "nightly"} <= set(loaded)
    assert suites.expand("gates")  # the gates come from sim/scenarios/gates.yaml
    for name, suite in loaded.items():
        assert suite["scenarios"], name
        for scenario in suite["scenarios"]:
            assert (SIM / "scenarios" / f"{scenario}.yaml").is_file(), scenario


def test_smoke_suite_is_one_flags_off_flight():
    runs = suites.expand("smoke")
    assert [(r["scenario"].relative_to(SIM / "scenarios").as_posix(), r["seed"], r["variant"]) for r in runs] == [("delivery/baseline.yaml", 1, "off")]


def test_nightly_suite_is_every_scenario_three_seeds_off_and_on():
    runs = suites.expand("nightly")
    scenarios = {r["scenario"] for r in runs}
    assert len(runs) == len(scenarios) * 3 * 2
    assert {r["variant"] for r in runs} == {"off", "on"}
    assert {r["seed"] for r in runs} == {1, 2, 3}


def test_seeds_and_variants_can_be_overridden_and_a_scenario_can_be_named_directly():
    runs = suites.expand("delivery/clutter", seeds=[4, 5], variants=["on"])
    assert [(r["scenario"].name, r["seed"], r["variant"]) for r in runs] == [("clutter.yaml", 4, "on"), ("clutter.yaml", 5, "on")]
    by_path = suites.expand(str(SIM / "scenarios" / "drop" / "baseline.yaml"))
    assert by_path[0]["scenario"] == SIM / "scenarios" / "drop" / "baseline.yaml"
    assert (by_path[0]["seed"], by_path[0]["variant"]) == (1, "off")


def test_scenario_can_be_named_by_a_relative_file_path(tmp_path, monkeypatch):
    monkeypatch.chdir(SIM.parent)
    assert suites.expand("sim/scenarios/drop/baseline.yaml")[0]["scenario"] == SIM / "scenarios" / "drop" / "baseline.yaml"
    monkeypatch.chdir(SIM / "scenarios")
    assert suites.expand("drop/baseline.yaml")[0]["scenario"] == SIM / "scenarios" / "drop" / "baseline.yaml"
    with pytest.raises(ValueError, match="no suite or scenario"):
        suites.expand("suites.yaml")  # the suites file is not a scenario
    with pytest.raises(ValueError, match="no suite or scenario"):
        suites.expand(str(SIM / "scenarios" / "gates.yaml"))


def test_unknown_suite_or_scenario_is_an_error():
    with pytest.raises(ValueError, match="no suite or scenario"):
        suites.expand("juggling")


def test_gates_suite_is_the_flights_of_the_acceptance_gates():
    runs = suites.expand("gates")
    baseline = [r for r in runs if r["scenario"].as_posix().endswith("delivery/baseline.yaml") and r["variant"] == "off"]
    assert sorted(r["seed"] for r in baseline) == [1, 2, 3, 4, 5]  # G0
    assert yaml.safe_load((SIM / "scenarios" / "suites.yaml").read_text())  # plain YAML
    with pytest.raises(ValueError, match="fixed"):
        suites.expand("G0", seeds=[1])  # a gate's flights are part of its rule


# --- sim time for a wall-clock time, and the ground-station audit ------------------------------


def test_sim_time_at_a_wall_time_is_interpolated_between_clock_marks():
    from runner.trace import sim_time_at

    record = Record.from_lines(clock_lines([(0.0, 1000.0), (10.0, 1020.0), (20.0, 1030.0)]))
    assert sim_time_at(record, 1010.0) == pytest.approx(5.0)
    assert sim_time_at(record, 1025.0) == pytest.approx(15.0)
    assert sim_time_at(record, 999.0) is None  # before the recording
    assert sim_time_at(record, 1031.0) is None  # after it


def test_gcs_audit_adds_commands_inside_the_recording_to_the_trace(tmp_path):
    from runner import gcs_audit

    run = tmp_path / "run"
    run.mkdir()
    lines = clock_lines([(0.0, 1000.0), (100.0, 1100.0)]) + [{"t": 10.0, "event": "armed"}, {"t": 90.0, "event": "disarmed"}]
    (run / "trace.jsonl").write_text("".join(json.dumps(line) + "\n" for line in lines))
    messages = [
        (900.0, 255, "SET_MODE", None),  # before the recording began: not in the trace at all
        (1050.0, 255, "SET_MODE", None),
        (1060.0, 255, "COMMAND_LONG", 512),  # the ground station asking for a message: housekeeping
        (1065.0, 255, "COMMAND_LONG", 410),  # MAVProxy asking for the home position, as it does in the sim
        (1070.0, 255, "COMMAND_LONG", 400),  # arm or disarm: a person
        (1075.0, 1, "COMMAND_LONG", 400),  # our own companion computer
    ]
    added = gcs_audit.audit(run, messages)
    assert [(item["t"], item["event"], item["what"]) for item in added] == [(50.0, "gcs_command", "SET_MODE"), (70.0, "gcs_command", "COMMAND_LONG 400")]
    record = Record.load(run / "trace.jsonl")
    assert [item["t"] for item in record.events_of("gcs_command")] == [50.0, 70.0]


def test_judge_takes_the_bonus_away_after_the_audit(tmp_path):
    """The audit's events are the ones the judge counts as human input."""
    from judge import judge_run
    from runner import gcs_audit

    run = tmp_path / "run"
    run.mkdir()
    lines = clock_lines([(0.0, 1000.0), (100.0, 1100.0)]) + [{"t": 10.0, "event": "armed"}, {"t": 90.0, "event": "disarmed"}, {"t": 50.0, "event": "release", "fake": True}]
    for i in range(81):
        height = 10.0 if 5 <= i <= 75 else 0.0
        lines.append({"t": 10.0 + i, "pose": "vehicle", "e": 0.0, "n": 0.0, "u": height})
    (run / "trace.jsonl").write_text("".join(json.dumps(line) + "\n" for line in lines))
    truth = {"takeoff": {"e": 0.0, "n": 0.0}, "target": {"e": 1.0, "n": 0.0}}
    assert judge_run("delivery", Record.load(run / "trace.jsonl"), truth, {"max_release_height_m": 50.0})["common"]["autonomous"] is True
    gcs_audit.audit(run, [(1040.0, 255, "RC_CHANNELS_OVERRIDE", None)])
    after = judge_run("delivery", Record.load(run / "trace.jsonl"), truth, {"max_release_height_m": 50.0})
    assert after["common"]["autonomous"] is False and after["autonomy_bonus"] == 0.0


# --- scenarios the sim cannot fly faithfully yet --------------------------------------------------


def test_scenario_that_needs_wind_or_gps_faults_is_not_flown_calm():
    calm = {"wind": {"level": "calm", "mean_mps": 0.0, "gust_mps": 0.0, "discrete_gust_mps": 0.0}, "gps_events": []}
    windy = {"wind": {"level": "moderate", "mean_mps": 4.0, "gust_mps": 1.0, "discrete_gust_mps": 0.0}, "gps_events": []}
    glitch = {**calm, "gps_events": [{"at_s": 40.0, "kind": "glitch"}]}
    assert events.unsupported(calm, set()) == ""
    assert "wind" in events.unsupported(windy, set())
    assert events.unsupported(windy, {"wind"}) == ""
    assert "GPS" in events.unsupported(glitch, {"wind"})
    assert events.unsupported(glitch, {"wind", "gps_events"}) == ""


def test_run_that_was_not_flown_is_invalid_with_the_reason(tmp_path):
    run = make_run(tmp_path, outcome=None)
    (run / "flight.json").write_text(json.dumps({"outcome": "INVALID", "reason": "the scenario was not flown: it asks for wind"}))
    summary = result.run_result(run)
    assert summary["status"] == "INVALID"
    assert "wind" in summary["reason"]


def test_result_cli_writes_the_report_and_the_suite_summary(tmp_path):
    import subprocess

    good = make_run(tmp_path / "a", score=SCORE)
    slow = make_run(tmp_path / "b", rtf=0.5, score=SCORE, manifest={"seed": 2})
    done = subprocess.run(
        [sys.executable, str(SIM / "runner" / "result.py"), "--suite", str(tmp_path / "suite"), "--title", "smoke", str(good), str(slow)],
        capture_output=True,
        text=True,
    )
    assert done.returncode == 1  # not every run is OK
    assert "OK" in (good / "report.md").read_text()
    assert json.loads((slow / "result.json").read_text())["status"] == "INVALID"
    summary = (tmp_path / "suite" / "summary.md").read_text()
    assert summary.startswith("# smoke") and "2 runs: 1 OK, 1 INVALID" in summary
    only_good = subprocess.run([sys.executable, str(SIM / "runner" / "result.py"), str(good)], capture_output=True, text=True)
    assert only_good.returncode == 0


# --- the host script ---------------------------------------------------------------------------------


def run_script(*args):
    import subprocess

    return subprocess.run(["bash", str(SIM / "run_scenarios.sh"), *args], capture_output=True, text=True)


def test_run_scenarios_parses_and_lists_a_suite_without_starting_anything():
    import subprocess

    assert subprocess.run(["bash", "-n", str(SIM / "run_scenarios.sh")]).returncode == 0
    listed = run_script("--list", "gates")
    assert listed.returncode == 0, listed.stderr
    lines = listed.stdout.strip().splitlines()
    assert len(lines) == 43  # G0's five deliveries, G1's four holds, G2's six drops, G3's three set-downs, G4's three surveys, G5's ten recoveries and G6's twelve GPS flights
    assert all(line.endswith(" off") and "delivery/baseline.yaml" in line for line in lines[:5])
    assert [line.split()[1] for line in lines[:5]] == ["1", "2", "3", "4", "5"]
    assert all(line.endswith(" 1 on") and "drop/hold_" in line for line in lines[5:9])
    assert all(line.endswith(" on") and "drop/gated" in line for line in lines[9:15])
    assert all(line.endswith(" on") and "delivery/set_down.yaml" in line for line in lines[15:18])
    assert all(line.endswith(" on") and "localisation/gate_field.yaml" in line for line in lines[18:21])
    assert all(line.endswith(" on") and "recovery/gate_pickup.yaml" in line for line in lines[21:31])
    assert sum("waypoint_navigation/circuit_hold" in line for line in lines[31:]) == 6 and sum("time_trial/circuit_auto" in line for line in lines[31:]) == 6


def test_run_scenarios_takes_seeds_and_variants():
    listed = run_script("--list", "delivery/clutter", "--seeds", "2,3", "--variants", "off,on")
    assert [line.split()[1:] for line in listed.stdout.strip().splitlines()] == [["2", "off"], ["2", "on"], ["3", "off"], ["3", "on"]]


def test_run_scenarios_refuses_what_it_does_not_know():
    assert run_script("--list", "juggling").returncode == 2
    assert run_script().returncode == 2
    assert run_script("--list", "smoke", "--variants", "sideways").returncode == 2
    assert run_script("--warp", "smoke").returncode == 2


def test_run_scenarios_keeps_the_laptop_awake_and_always_stops_the_sim():
    text = (SIM / "run_scenarios.sh").read_text()
    lock = 'inhibit_sleep "$SIM_DIR/run_scenarios.sh"'  # as sim/gps/fly_all.sh does, but by full path
    assert lock in text and text.index(lock) < text.index("fly_one() {")
    assert 'READY_TIMEOUT_S=${READY_TIMEOUT_S:-900}' in text
    body = text[text.index("fly_one() {") : text.index("\n}\n", text.index("fly_one() {"))]
    assert body.count("stop_sim") >= 2  # a fresh container before, and torn down after
    assert "--detach" in body and "READY" in body and "sim/runner/fly.py" in body and "judge/cli.py" in body


# --- the host loop, with a stand-in for the sim --------------------------------------------------


STUB_RUN_SIM = """#!/usr/bin/env bash
# A stand-in for sim/run_sim.sh: no container, a canned flight.
echo "$*" >> "$STUB_LOG"
case "$1" in
    --down) rm -f "$STUB_DIR/running" ;;
    --detach)
        [[ $STUB_MODE == no-start ]] && exit 1
        [[ $STUB_MODE != dies ]] && touch "$STUB_DIR/running"
        [[ $STUB_MODE == ok ]] && date > "$RUN_DIR/READY"
        exit 0 ;;
    --exec)
        run_dir=${!#}
        if [[ $* == *fly.py* ]]; then
            cp "$STUB_DIR/trace.jsonl" "$run_dir/trace.jsonl"
            echo '{"outcome": "DONE", "t_launch": 5.0, "t_end": 120.0}' > "$run_dir/flight.json"
            echo "flight: DONE"
        fi ;;
esac
"""
STUB_DOCKER = """#!/usr/bin/env bash
# `docker inspect` says whether the stand-in sim is up; `docker logs` prints nothing.
if [[ $1 == inspect ]]; then
    if [[ -f $STUB_DIR/running ]]; then echo true; else echo false; fi
fi
"""


def stub_env(tmp_path, mode, ready_timeout="900"):
    import os

    stub = tmp_path / "stub"
    (stub / "bin").mkdir(parents=True)
    for name, text in (("run_sim.sh", STUB_RUN_SIM), ("bin/docker", STUB_DOCKER)):
        (stub / name).write_text(text)
        (stub / name).chmod(0o755)
    lines = [{"t": 10.0, "event": "armed"}, {"t": 110.0, "event": "disarmed"}, {"t": 60.0, "event": "release", "fake": True}]
    lines += clock_lines([(float(t), 1000.0 + t) for t in range(0, 125, 5)])
    knots = [(10, 0, 0, 0), (20, 0, 0, 15), (40, 10.0, 15.0, 15), (58, 10.0, 15.0, 0), (65, 10.0, 15.0, 0), (75, 10.0, 15.0, 12), (95, 0, 0, 12), (108, 0, 0, 0)]
    for (t0, e0, n0, u0), (t1, e1, n1, u1) in zip(knots, knots[1:]):
        for i in range(10):
            f = i / 10
            lines.append({"t": t0 + (t1 - t0) * f, "pose": "vehicle", "e": e0 + (e1 - e0) * f, "n": n0 + (n1 - n0) * f, "u": 0.195 + u0 + (u1 - u0) * f})
    (stub / "trace.jsonl").write_text("".join(json.dumps(line) + "\n" for line in lines))
    return {
        **os.environ,
        "PATH": f"{stub / 'bin'}:{os.environ['PATH']}",
        "ARGUS_RUN_SIM": str(stub / "run_sim.sh"),
        "ARGUS_SLEEP_INHIBITED": "1",  # no systemd lock in a test
        "STUB_DIR": str(stub), "STUB_LOG": str(stub / "calls.log"), "STUB_MODE": mode,
        "READY_TIMEOUT_S": ready_timeout,
    }  # fmt: skip


def run_with_stub(tmp_path, mode, *args, ready_timeout="900", cwd=None, script=None):
    import subprocess

    env = stub_env(tmp_path, mode, ready_timeout)
    done = subprocess.run(["bash", script or str(SIM / "run_scenarios.sh"), *args], capture_output=True, text=True, env=env, cwd=cwd, timeout=120)
    calls = (tmp_path / "stub" / "calls.log").read_text().splitlines() if (tmp_path / "stub" / "calls.log").exists() else []
    suite_dir = None
    for line in done.stdout.splitlines():
        if line.startswith("Summary: "):
            suite_dir = (SIM.parent / line.split("Summary: ", 1)[1]).parent
    return done, calls, suite_dir


def test_host_loop_flies_every_flight_of_a_suite_on_a_fresh_sim_and_tears_it_down(tmp_path):
    done, calls, suite_dir = run_with_stub(tmp_path, "ok", "delivery/baseline", "--seeds", "1,2")
    try:
        assert done.returncode == 0, done.stdout + done.stderr
        assert "2 runs: 2 OK" in done.stdout
        assert done.stdout.count("flight: DONE") == 2
        # For each flight: down (fresh), start detached, fly, audit, down.
        kinds = [call.split()[0] for call in calls]
        assert kinds == ["--down", "--detach", "--exec", "--exec", "--down"] * 2
        assert "sim/runner/fly.py" in calls[2] and "sim/runner/gcs_audit.py" in calls[3]
        runs = sorted((suite_dir / "runs").iterdir())
        assert [run.name for run in runs] == ["delivery-baseline-s1-off", "delivery-baseline-s2-off"]
        for run in runs:
            assert json.loads((run / "result.json").read_text())["status"] == "OK"
            assert (run / "report.md").is_file() and (run / "score.json").is_file()
        assert "| delivery/baseline | 2 | OK |" in (suite_dir / "summary.md").read_text()
    finally:
        _remove(suite_dir)


@pytest.mark.parametrize("mode, said", [("never-ready", "was not READY after 1 s"), ("dies", "stopped before it was READY"), ("no-start", "did not start")])
def test_host_loop_reports_a_sim_that_never_gets_ready_and_still_tears_it_down(mode, said, tmp_path):
    done, calls, suite_dir = run_with_stub(tmp_path, mode, "smoke", ready_timeout="1")
    try:
        assert done.returncode == 1
        assert said in done.stdout
        assert "1 runs: 1 ERROR" in done.stdout
        assert calls[-1] == "--down"
        assert not any("fly.py" in call for call in calls)  # nothing is flown on a sim that is not ready
        run = suite_dir / "runs" / "delivery-baseline-s1-off"
        assert json.loads((run / "result.json").read_text())["status"] == "ERROR"
    finally:
        _remove(suite_dir)


def test_host_loop_works_when_called_by_a_relative_path_from_another_folder(tmp_path):
    """The script changes directory; the sleep lock re-runs it and must still find it."""
    import os

    env_overrides = {"ARGUS_SLEEP_INHIBITED": ""}  # let it try to take the lock, as a person's shell would
    stub = stub_env(tmp_path, "ok")
    stub.update(env_overrides)
    import subprocess

    done = subprocess.run(["bash", "./run_scenarios.sh", "smoke"], capture_output=True, text=True, env=stub, cwd=str(SIM), timeout=120)
    suite_dir = None
    for line in done.stdout.splitlines():
        if line.startswith("Summary: "):
            suite_dir = (SIM.parent / line.split("Summary: ", 1)[1]).parent
    try:
        assert done.returncode == 0, done.stdout + done.stderr
        assert "1 runs: 1 OK" in done.stdout
    finally:
        _remove(suite_dir)
    assert os.path.isfile(SIM / "run_scenarios.sh")


def _remove(folder):
    """Take a test's suite folder out of sim/.state again."""
    import shutil

    if folder is not None and folder.is_dir() and folder.parent.name == "suites" and ".state" in folder.parts:
        shutil.rmtree(folder)


# --- tools beside the flight, and the judges' list ---------------------------------------------------


TOOL = """import pathlib, signal, sys, time
run = pathlib.Path(sys.argv[1])
(run / (pathlib.Path(__file__).stem + ".started")).write_text(" ".join(sys.argv[1:]))
def stop(*_):
    (run / (pathlib.Path(__file__).stem + ".stopped")).write_text("interrupted")
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
print("tool running", flush=True)
while True:
    time.sleep(0.05)
"""


def test_side_tools_start_with_the_run_folder_and_stop_when_the_flight_ends(tmp_path):
    import time

    from runner.helpers import SideTools

    scripts = {}
    for name in ("wind", "gps_events"):
        scripts[name] = tmp_path / f"{name}.py"
        scripts[name].write_text(TOOL)
    run = tmp_path / "run"
    run.mkdir()
    tools = SideTools(scripts, run)
    tools.start({"wind"})  # only what the scenario needs
    deadline = time.monotonic() + 10.0
    while not (run / "wind.started").exists() and time.monotonic() < deadline:
        time.sleep(0.05)
    assert (run / "wind.started").read_text() == str(run)
    assert not (run / "gps_events.started").exists()
    codes = tools.stop()
    assert codes == {"wind": 0}
    assert (run / "wind.stopped").read_text() == "interrupted"
    assert "tool running" in (run / "wind.log").read_text()
    assert tools.stop() == {}  # nothing left running


def test_side_tools_kill_a_tool_that_ignores_the_interrupt(tmp_path):
    from runner.helpers import SideTools

    script = tmp_path / "wind.py"
    script.write_text("import signal, time\nsignal.signal(signal.SIGINT, signal.SIG_IGN)\nprint('up', flush=True)\nwhile True:\n    time.sleep(0.05)\n")
    run = tmp_path / "run"
    run.mkdir()
    tools = SideTools({"wind": script}, run)
    tools.start({"wind"})
    import time

    deadline = time.monotonic() + 10.0
    while "up" not in (run / "wind.log").read_text() and time.monotonic() < deadline:
        time.sleep(0.05)
    assert tools.stop(grace_s=0.3)["wind"] == -9


def test_list_capture_copies_the_list_while_flying_and_not_after(tmp_path):
    import os

    from runner.helpers import ListCapture

    run = tmp_path / "run"
    run.mkdir()
    list_file = tmp_path / "judges_list.csv"
    capture = ListCapture(list_file, run)
    assert capture.poll(flying=True) is False  # nothing written yet
    list_file.write_text("ID,Latitude,Longitude\n3,34.1,-118.4\n")
    assert capture.poll(flying=True) is True
    assert capture.poll(flying=True) is False  # unchanged
    assert (run / "judges_list.csv").read_text().count("\n") == 2
    list_file.write_text("ID,Latitude,Longitude\n3,34.1,-118.4\n7,34.2,-118.5\n")
    os.utime(list_file, ns=(list_file.stat().st_atime_ns, list_file.stat().st_mtime_ns + 1_000_000))
    assert capture.poll(flying=False) is True  # noted...
    assert (run / "judges_list.csv").read_text().count("\n") == 2  # ...but the judge keeps the list as it stood in flight


def test_list_capture_ignores_a_list_left_by_an_earlier_run(tmp_path):
    import os

    from runner.helpers import ListCapture

    run = tmp_path / "run"
    run.mkdir()
    list_file = tmp_path / "judges_list.csv"
    list_file.write_text("ID,Latitude,Longitude\n9,34.0,-118.0\n")  # there before this run began
    capture = ListCapture(list_file, run)
    assert capture.poll(flying=True) is False
    assert not (run / "judges_list.csv").exists()
    list_file.write_text("ID,Latitude,Longitude\n1,34.3,-118.3\n")
    os.utime(list_file, ns=(list_file.stat().st_atime_ns, list_file.stat().st_mtime_ns + 1_000_000))
    assert capture.poll(flying=True) is True
    assert "1,34.3" in (run / "judges_list.csv").read_text()
    assert ListCapture(None, run).poll(flying=True) is False  # a mission with no list


# --- lines the mission's log must have (LOC-3's guard flights) ----------------------


def test_a_run_whose_log_lacks_a_line_the_scenario_expects_is_a_fail(tmp_path):
    """A survey flight passes its waypoints whether or not the height guard acted; `expect_log` makes the guard's
    own lines part of the verdict."""
    wanted = ["survey waiting:", "back at the survey height"]
    run = make_run(tmp_path / "a", score=SCORE, manifest={"expect_log": wanted})
    (run / "launch.log").write_text("[mission_executor]: [search_pattern] survey waiting: 8.05 m on the rangefinder\n[mission_executor]: [search_pattern] back at the survey height: going on\n")
    done = result.run_result(run)
    assert done["status"] == "OK" and done["expect_log"] == {"wanted": wanted, "missing": []}
    assert "| Lines expected in the mission's log | 2 of 2 found |" in result.render_report(done)

    run = make_run(tmp_path / "b", score=SCORE, manifest={"expect_log": wanted})
    (run / "launch.log").write_text("[mission_executor]: [search_pattern] survey waiting: 8.05 m on the rangefinder\n")
    missed = result.run_result(run)
    assert missed["status"] == "FAIL" and "back at the survey height" in missed["reason"] and "survey waiting" not in missed["reason"]
    assert "1 of 2 found; missing: back at the survey height" in result.render_report(missed)

    absent = result.run_result(make_run(tmp_path / "c", score=SCORE, manifest={"expect_log": wanted}))  # no log at all
    assert absent["status"] == "FAIL" and absent["expect_log"]["missing"] == wanted
    plain = result.run_result(make_run(tmp_path / "d", score=SCORE))
    assert plain["status"] == "OK" and plain["expect_log"] is None and "Lines expected" not in result.render_report(plain)
