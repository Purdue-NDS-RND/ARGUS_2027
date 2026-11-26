"""The sim's check of the height estimator against the true height (CEN-03): the arithmetic, the verdict and the runner's use of it. No ROS."""

import json
import math
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import height_check  # noqa: E402
from runner import events, result  # noqa: E402
from scenario.build import build_run  # noqa: E402

from test_runner import SCORE, make_run  # noqa: E402  (the run-folder fixture)

SCENARIO = SIM / "scenarios" / "delivery" / "height_estimator.yaml"
S = 1_000_000_000
LEVEL = (0.0, 0.0, 0.0, 1.0)


def tilted(deg):
    half = math.radians(deg) / 2.0
    return (math.sin(half), 0.0, 0.0, math.cos(half))


def fly(checker, heights, range_bias_m=0.0, ranges=True, tilt_deg=0.0, start_s=10.0, ekf_bias_m=0.0):
    """Feed a flight through the heights given, 30 poses a second: the truth, MAVROS's pose (with an EKF height bias) and the rangefinder.

    The vehicle's model origin is 0.2 m above the ground where it stands,
    as in the sim: true height counts from there.
    """
    orientation = tilted(tilt_deg)
    checker.on_truth(start_s - 1.0, (0.0, 0.0, 0.2), LEVEL)  # standing on the ground before the flight
    for index, height in enumerate(heights):
        t_s = start_s + index / 30.0
        checker.on_truth(t_s, (0.0, 0.0, 0.2 + height), orientation)
        stamp_ns = int(round(t_s * S))
        if ranges:
            checker.on_range(stamp_ns, height / math.cos(math.radians(tilt_deg)) + range_bias_m, 0.0, 1000.0)
        checker.on_pose(stamp_ns, (0.0, 0.0, height + ekf_bias_m), orientation, t_s)
    checker.on_truth(start_s + len(heights) / 30.0, (0.0, 0.0, 0.2 + heights[-1]), orientation)


def descent(top=15.0, bottom=1.0, hold=90):
    """Hold at the top for three seconds, then come down at 0.5 m/s."""
    heights = [top] * hold
    while heights[-1] > bottom:
        heights.append(heights[-1] - 0.5 / 30.0)
    return heights


def test_the_lines():
    assert height_check.FLAG == "use_height_estimator"
    assert height_check.RANGE_LIMIT_M == 0.10  # SIM-05's line for the rangefinder itself
    assert height_check.IN_BAND_M == (0.5, 11.0) and height_check.IN_BAND_MIN_SHARE == 0.9
    assert height_check.ABOVE_SENSOR_M == 12.5


def test_a_descent_from_15_m_passes_with_the_barometer_above_12_m_and_the_rangefinder_below():
    checker = height_check.Checker()
    fly(checker, descent(), ekf_bias_m=0.25)  # the EKF's height is a quarter of a metre off: the rangefinder's is not
    report = checker.report()
    assert report["passed"] is True and report["problems"] == []
    assert report["rangefinder"]["count"] > 500 and report["rangefinder"]["p95_m"] < 0.01
    assert report["baro"]["count"] > 90 and abs(report["baro"]["median_m"] - 0.25) < 0.01
    assert report["above_sensor_range"] == {"count": report["above_sensor_range"]["count"], "rangefinder": 0}
    assert report["above_sensor_range"]["count"] >= 90
    assert report["in_band"]["rangefinder_share"] > 0.99
    assert report["baro"]["reasons"].get("out of range", 0) > 90
    lines = height_check.summary_lines(report)
    assert lines[0].startswith("rangefinder source: PASS, median 0.00 m, p95 0.00 m over ")
    assert any(line.startswith("baro source: median 0.25 m") for line in lines)


def test_a_rangefinder_height_that_is_off_fails_the_run():
    checker = height_check.Checker()
    fly(checker, descent(), range_bias_m=0.3)
    report = checker.report()
    assert report["passed"] is False
    assert any("m from the true height at the 95th percentile" in problem and "limit 0.1 m" in problem for problem in report["problems"])
    # 0.30 m at the stamp; 0.275 m against the truth over the reading's 50 ms, in which a 0.5 m/s descent covers 0.025 m.
    assert report["rangefinder"]["p95_at_stamp_m"] == pytest.approx(0.30, abs=0.002) and report["rangefinder"]["p95_m"] == pytest.approx(0.275, abs=0.002)


def test_falling_back_to_the_barometer_where_the_rangefinder_should_work_fails_the_run():
    checker = height_check.Checker()
    fly(checker, descent(), ranges=False)
    report = checker.report()
    assert report["passed"] is False and report["rangefinder"]["count"] == 0
    assert any("of the samples between 0.5 and 11 m" in problem for problem in report["problems"])
    assert report["baro"]["reasons"] == {"no reading": report["baro"]["count"]}


def test_a_flight_tilted_over_the_limit_is_on_the_barometer_and_that_is_not_held_against_it():
    checker = height_check.Checker()
    fly(checker, [6.0] * 200, tilt_deg=20.0)  # the band is judged only where the truth says the vehicle was near level
    report = checker.report()
    assert report["rangefinder"]["count"] == 0 and report["baro"]["reasons"].get("tilt", 0) > 150
    assert report["in_band"]["count"] == 0 and report["passed"] is True
    assert any("nothing to judge" in line for line in height_check.summary_lines(report))


def test_a_flight_that_stays_above_the_sensors_range_has_nothing_to_judge_and_passes():
    checker = height_check.Checker()
    fly(checker, [15.0] * 200)
    report = checker.report()
    assert report["passed"] is True and report["rangefinder"]["count"] == 0 and report["above_sensor_range"]["count"] > 150


def test_a_rangefinder_height_while_the_truth_says_tilted_well_over_the_limit_fails_the_run():
    checker = height_check.Checker()
    fly(checker, [6.0] * 200, tilt_deg=20.0)
    assert checker.report()["tilted"] == {"count": checker.report()["tilted"]["count"], "rangefinder": 0}
    assert checker.report()["tilted"]["count"] > 150 and height_check.TILTED_OVER_DEG == 17.0
    truth = height_check.TrueHeight()
    for index in range(100):
        truth.add(10.0 + index / 30.0, (0.0, 0.0, 6.0 if index else 0.0), tilted(20.0))
    report = height_check.build_report([(10.0 + index / 30.0, 6.0, "rangefinder", "") for index in range(2, 90)], truth)
    assert report["passed"] is False and any("tilted over 17 degrees" in problem for problem in report["problems"])


def test_no_poses_heard_is_a_failure():
    report = height_check.Checker().report()
    assert report["passed"] is False and "no height" in report["problems"][0]


def test_a_report_is_read_back_only_if_it_is_whole(tmp_path):
    assert height_check.read_height_check(tmp_path) is None
    (tmp_path / "height_check.json").write_text(json.dumps({"passed": True, "problems": []}))
    assert height_check.read_height_check(tmp_path)["passed"] is True
    (tmp_path / "height_check.json").write_text("{not json")
    assert height_check.read_height_check(tmp_path) is None


# --- the runner ---


def test_the_check_runs_beside_flights_with_the_flag_on_and_only_those(tmp_path):
    calm = {"wind": {"level": "calm"}, "gps_events": []}
    both = {"use_frame_time_pose": True, "use_height_estimator": True}
    assert "height_check" not in events.tools_needed({**calm, "sim_time": True, "flags": {"use_frame_time_pose": True}})
    assert events.tools_needed({**calm, "sim_time": True, "flags": both}) == {"time_check", "rate_check", "projection_check", "height_check"}
    assert "the height estimator against the true height" in events.unsupported({**calm, "flags": both}, {"projection_check"})
    assert '"height_check": SIM_DIR / "height_check.py"' in (SIM / "runner" / "fly.py").read_text()
    on = json.loads((build_run(SCENARIO, tmp_path / "on", variant="on") / "scenario.json").read_text())
    off = json.loads((build_run(SCENARIO, tmp_path / "off", variant="off") / "scenario.json").read_text())
    assert height_check.wanted(on) and on["sim_time"] is True and on["flags"] == both
    assert not height_check.wanted(off) and off.get("sim_time") is not True and off["flags"] == {}


def test_the_scenario_is_in_the_nightly_suite():
    import yaml

    assert "delivery/height_estimator" in yaml.safe_load((SIM / "scenarios" / "suites.yaml").read_text())["nightly"]["scenarios"]


PROJECTION_OK = {"passed": True, "problems": [], "limit_m": 0.3, "at_15_m": {"count": 240, "median_m": 0.08, "p95_m": 0.17, "max_m": 0.24}}
TIME_OK = {"passed": True, "problems": [], "pose": {"count": 4000, "median_ms": 4.0, "p95_abs_ms": 5.3, "max_abs_ms": 6.6}}
RATES_OK = {"passed": True, "problems": [], "topics": {"/mavros/local_position/pose": {"measured_hz": 30.2, "asked_hz": 30.0, "status": "PASS"}}}


def flag_on_run(tmp_path, report):
    run = make_run(tmp_path, score=SCORE, manifest={"variant": "on", "sim_time": True, "flags": {"use_frame_time_pose": True, "use_height_estimator": True}})
    (run / "time_check.json").write_text(json.dumps(TIME_OK))
    (run / "rate_check.json").write_text(json.dumps(RATES_OK))
    (run / "projection_check.json").write_text(json.dumps(PROJECTION_OK))
    if report is not None:
        (run / "height_check.json").write_text(json.dumps(report))
    return run


def flown_report(**kwargs):
    checker = height_check.Checker()
    fly(checker, descent(), **kwargs)
    return checker.report()


def test_a_flag_on_run_counts_only_with_a_passing_height_check(tmp_path):
    passed = result.run_result(flag_on_run(tmp_path / "a", flown_report()))
    assert passed["status"] == "OK"
    assert "| Height estimator against the true height | rangefinder source: PASS, median 0.00 m, p95 0.00 m over " in result.render_report(passed)

    missed = result.run_result(flag_on_run(tmp_path / "b", flown_report(range_bias_m=0.3)))
    assert missed["status"] == "FAIL" and missed["reason"].startswith("the height estimator missed its pass line: ")

    absent = result.run_result(flag_on_run(tmp_path / "c", None))
    assert absent["status"] == "INVALID" and "height_check.json" in absent["reason"]


def test_a_run_without_the_flag_is_not_asked_for_a_height_check(tmp_path):
    run = make_run(tmp_path, score=SCORE, manifest={"variant": "off", "flags": {}})
    done = result.run_result(run)
    assert done["height_check"] is None and "height_check" not in (done.get("reason") or "")
    assert "Height estimator" not in result.render_report(done)


# --- a scheduled rangefinder fault (LOC-1's dips) -----------------------------------


def test_heights_during_a_scenarios_rangefinder_fault_are_counted_and_not_judged(tmp_path):
    assert height_check.fault_windows(tmp_path) == []  # no injector log: no faults
    (tmp_path / "gps_events.jsonl").write_text(
        json.dumps({"t": 640.0, "event": "glitch", "what": "on"}) + "\n"
        + json.dumps({"t": 650.0, "event": "rangefinder", "what": "on", "scale": 0.75}) + "\n"
        + "not json\n"
        + json.dumps({"t": 658.0, "event": "rangefinder", "what": "off"}) + "\n"
        + json.dumps({"t": 700.0, "event": "rangefinder", "what": "on", "scale": 0.7}) + "\n"
    )
    windows = height_check.fault_windows(tmp_path)
    assert windows == [(650.0, 658.0 + height_check.FAULT_SETTLE_S), (700.0, math.inf)]  # the second never ended

    def report(faults):
        truth = height_check.TrueHeight()
        truth.add(599.0, (0.0, 0.0, 0.2), LEVEL)
        samples = []
        for index in range(1800):  # a minute at 10.7 m
            t_s = 600.0 + index / 30.0
            truth.add(t_s, (0.0, 0.0, 10.9), LEVEL)
            faulted = 650.0 <= t_s <= 658.5
            samples.append((t_s - 0.001, 10.7 * (0.75 if faulted else 1.0), "rangefinder", ""))
        return height_check.build_report(samples, truth, faults)

    unknown = report(None)
    assert not unknown["passed"] and "from the true height" in unknown["problems"][0] and unknown["during_a_rangefinder_fault"] == 0
    known = report(windows[:1])
    assert known["passed"] and known["during_a_rangefinder_fault"] == 270 and known["rangefinder"]["count"] == known["samples"] - 270
    assert any("rangefinder fault: 270 heights, not judged" in line for line in height_check.summary_lines(known))
    assert "checker.report(fault_windows(run_dir))" in (SIM / "height_check.py").read_text()


# --- a reading is a little old when it is stamped ------------------------------------------------------------


def fly_aged(checker, heights, age_s, start_s=10.0):
    """As fly(), but each rangefinder reading is the true height `age_s` before the stamp it carries.

    That is the sim's rangefinder as MAVROS gives it, and the aircraft's: ArduPilot reads the sensor at 20 Hz and a
    reading goes out in the next DISTANCE_SENSOR message, stamped when it is sent. In nine recorded flights the age
    was a constant of the flight, between 0 and 40 ms (docs/results/2026-11-25-height-check-reading-age.md).
    """
    checker.on_truth(start_s - 1.0, (0.0, 0.0, 0.2), LEVEL)

    def height_at(t_s):
        place = min(len(heights) - 1.0, max(0.0, (t_s - start_s) * 30.0))
        low = int(place)
        high = min(len(heights) - 1, low + 1)
        return heights[low] + (place - low) * (heights[high] - heights[low])

    for index, height in enumerate(heights):
        t_s = start_s + index / 30.0
        checker.on_truth(t_s, (0.0, 0.0, 0.2 + height), LEVEL)
        stamp_ns = int(round(t_s * S))
        checker.on_range(stamp_ns, height_at(t_s - age_s), 0.0, 1000.0)
        checker.on_pose(stamp_ns, (0.0, 0.0, height), LEVEL, t_s)
    checker.on_truth(start_s + len(heights) / 30.0, (0.0, 0.0, 0.2 + heights[-1]), LEVEL)


def drop_flight(climb_mps=2.5):
    """A drop's heights at 30 Hz: up to 11 m at the climb rate, 3 s there, down to 2 m at 0.35 m/s, 5 s there, and up again."""
    heights = [0.3]
    while heights[-1] < 11.0:
        heights.append(heights[-1] + climb_mps / 30.0)
    heights += [11.0] * 90
    while heights[-1] > 2.0:
        heights.append(heights[-1] - 0.35 / 30.0)
    heights += [2.0] * 150
    while heights[-1] < 11.0:
        heights.append(heights[-1] + climb_mps / 30.0)
    return heights


def test_a_reading_one_period_old_in_a_fast_climb_is_the_readings_age_not_the_estimators_error():
    """Gate G2, 25 November: 0.10 m at the 95th percentile in drop flights, every one of them in the two 2.5 m/s climbs,
    where a reading 40 ms old is 0.10 m behind. The same heights against the truth at the moment the sensor read are right."""
    assert height_check.READING_PERIOD_S == 0.05  # ArduPilot's rangefinder is read at 20 Hz
    checker = height_check.Checker()
    fly_aged(checker, drop_flight(), age_s=0.042)
    report = checker.report()
    assert report["passed"], report["problems"]
    rangefinder = report["rangefinder"]
    # At the stamp the climbs read 0.105 m low, and the climbs are over a twentieth of the flight; against the truth over the reading's period they read right.
    assert rangefinder["p95_at_stamp_m"] == pytest.approx(0.105, abs=0.005) and rangefinder["p95_at_stamp_m"] > height_check.RANGE_LIMIT_M
    assert rangefinder["p95_m"] < 0.01 and rangefinder["reading_period_s"] == 0.05
    assert rangefinder["max_m"] < 0.12  # a few heights where a climb ends: the estimator's ground is the middle of five readings, two of them still the climb's
    text = " ".join(height_check.summary_lines(report))
    assert "p95 0.00 m" in text and "p95 0.11 m against the truth at the stamp alone, a reading being up to 50 ms old then" in text


def test_a_hover_and_a_slow_descent_get_nothing_from_the_readings_age():
    """Where the release gate, the set-down and the cone descent use the height, a reading's age is worth under 0.02 m: an error of 0.12 m there fails as before."""
    for heights in ([6.0] * 600, descent(top=10.0, bottom=1.0)):
        checker = height_check.Checker()
        fly(checker, heights, range_bias_m=0.12)
        report = checker.report()
        assert not report["passed"] and "from the true height at the 95th percentile" in report["problems"][0], report
        assert report["rangefinder"]["p95_m"] >= 0.10


def test_a_reading_older_than_one_period_or_ahead_of_the_truth_still_fails():
    old = height_check.Checker()
    fly_aged(old, drop_flight(), age_s=0.12)  # 0.30 m behind in the climbs: 0.175 m more than one period explains
    report = old.report()
    assert not report["passed"] and report["rangefinder"]["p95_m"] == pytest.approx(0.175, abs=0.01)
    ahead = height_check.Checker()
    fly_aged(ahead, drop_flight(), age_s=-0.048)  # a height that leads the truth: nothing a sensor's age explains
    report = ahead.report()
    assert not report["passed"] and report["rangefinder"]["p95_m"] == pytest.approx(0.12, abs=0.01)


def test_a_recorded_flight_is_judged_again_from_its_bag_and_its_trace(tmp_path, capsys):
    """`height_check.py --bag <run folder>`: the same estimator and the same rules, on the poses and readings the flight's bag holds."""
    from replay.bag import write_bag
    from replay_flight import MS, pose_record, range_record

    run = tmp_path / "run"
    run.mkdir()
    heights = drop_flight()
    records, trace = [], [{"t": 9.0, "pose": "vehicle", "e": 0.0, "n": 0.0, "u": 0.2, "yaw": 0.0}]
    for index, height in enumerate(heights):
        ns = 10 * S + index * (S // 30)
        aged = heights[max(0, index - 1)] + (heights[index] - heights[max(0, index - 1)]) * (1.0 - 0.042 * 30.0)  # 42 ms old
        records += [pose_record(ns, up=height), range_record(ns, range_m=aged)]
        trace.append({"t": ns / S, "pose": "vehicle", "e": 0.0, "n": 0.0, "u": 0.2 + height, "yaw": 0.0})
        trace.append({"t": ns / S, "pose": "parcel", "e": 0.0, "n": 0.0, "u": 0.0})
    trace.append({"t": 10.0 + len(heights) / 30.0, "pose": "vehicle", "e": 0.0, "n": 0.0, "u": 0.2 + heights[-1], "yaw": 0.0})
    write_bag(run / "bag", records)
    (run / "trace.jsonl").write_text("".join(json.dumps(row) + "\n" for row in trace))
    assert MS == 1_000_000
    report = height_check.replay(run)
    assert report["passed"] and report["rangefinder"]["p95_at_stamp_m"] == pytest.approx(0.105, abs=0.005) and report["rangefinder"]["count"] > 1000
    assert report["from"] == "bag" and not (run / "height_check.json").exists()  # the flight's own report is not written over
    assert height_check.main(["--bag", str(run)]) == 0
    out = capsys.readouterr().out
    assert "height check: PASS" in out and "from the bag" in out and "the trace has no tilt" in out
    assert height_check.main(["--bag", str(tmp_path / "nowhere")]) == 2
