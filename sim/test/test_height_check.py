"""The sim's check of the height estimator against the true height (CEN-03): the arithmetic, the verdict and the runner's use of it. No ROS."""

import json
import math
import sys
from pathlib import Path

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
    assert any("0.30 m from the true height at the 95th percentile" in problem and "limit 0.1 m" in problem for problem in report["problems"])


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
