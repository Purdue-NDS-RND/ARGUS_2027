"""The A/B harness (CEN-12): the same seeded scenarios with flags off and on, and the verdict on turning a flag on."""

import json
import sys
from pathlib import Path

import pytest
import yaml

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

from ab import run_ab  # noqa: E402


def run(scenario, seed, variant, status="OK", distance=0.05, released=True, flight_s=100.0, low=False, flew_low=False, mission="drop", high=False):
    details = {"released": released, "distance_m": distance if released else None}
    details.update({"released_low": low, "flew_low": flew_low} if mission == "drop" else {"released_high": high})
    return {"scenario": scenario, "seed": seed, "variant": variant, "status": status, "reason": "" if status == "OK" else "it went wrong", "flight_s": flight_s, "mission": mission, "details": details}


def pairs(scenario, off, on, **more):
    """Runs of one scenario: `off` and `on` are lists of distances, one a seed; None is a flight with no release."""
    runs = []
    for variant, distances in (("off", off), ("on", on)):
        for seed, distance in enumerate(distances, start=1):
            runs.append(run(scenario, seed, variant, distance=distance or 0.0, released=distance is not None, **more))
    return runs


PLAN = {"flag": "drop_release_gates", "mission": "drop", "calm": ["drop/calm"], "wind": ["drop/wind"], "seeds": [1, 2, 3]}


def test_the_shipped_plans_name_scenarios_that_exist_with_the_flag_on_and_wind_where_they_say():
    plans = run_ab.load_plans()
    assert set(plans) >= {"drop", "delivery"} and plans["drop"]["flag"] == "drop_release_gates" and plans["delivery"]["flag"] == "delivery_soft_set_down"
    for name, plan in plans.items():
        assert plan["seeds"] and plan["calm"], name
        for group in ("calm", "wind"):
            for scenario in plan[group]:
                data = yaml.safe_load((SIM / "scenarios" / f"{scenario}.yaml").read_text())
                assert data["flags"][plan["flag"]] is True and data["mission"] == plan["mission"], scenario
                wind = data.get("wind") or {}
                if group == "calm":
                    assert wind.get("level", "calm") == "calm", scenario
                else:
                    assert wind.get("level") == "strong", scenario  # the sim's 7 m/s


def test_the_flights_are_every_scenario_every_seed_off_and_on():
    commands = run_ab.commands(PLAN)
    assert commands == [
        ["sim/run_scenarios.sh", "drop/calm", "--seeds", "1,2,3", "--variants", "off,on"],
        ["sim/run_scenarios.sh", "drop/wind", "--seeds", "1,2,3", "--variants", "off,on"],
    ]


def test_the_report_has_error_time_and_failures_for_each_side():
    runs = pairs("drop/calm", off=[0.30, 0.40, 0.20], on=[0.05, 0.10, 0.03], flight_s=100.0) + pairs("drop/wind", off=[0.6, 0.9, 0.5], on=[0.20, 0.28, 0.25])
    report = run_ab.compare(PLAN, runs)
    calm = report["groups"]["calm"]
    assert calm["off"]["runs"] == 3 and calm["off"]["median_m"] == pytest.approx(0.30) and calm["off"]["max_m"] == pytest.approx(0.40)
    assert calm["on"]["median_m"] == pytest.approx(0.05) and calm["on"]["failures"] == 0 and calm["on"]["median_flight_s"] == pytest.approx(100.0)
    assert report["groups"]["wind"]["on"]["median_m"] == pytest.approx(0.25)
    assert report["passed"] is True and report["problems"] == []
    text = run_ab.render(PLAN, report)
    assert "| calm | 3 | 0 | 0.30 m | 0.40 m | 100 s | 3 | 0 | 0.05 m | 0.10 m | 100 s |" in text
    assert "PASS: drop_release_gates may be turned on" in text


def test_the_drops_pass_lines_are_0_15_m_calm_and_0_30_m_in_wind_on_the_median():
    over_calm = pairs("drop/calm", off=[0.3, 0.3, 0.3], on=[0.16, 0.20, 0.10]) + pairs("drop/wind", off=[0.6] * 3, on=[0.2] * 3)
    report = run_ab.compare(PLAN, over_calm)
    assert report["passed"] is False and "calm: flags-on median error 0.16 m is over 0.15 m" in report["problems"][0]
    over_wind = pairs("drop/calm", off=[0.3] * 3, on=[0.05] * 3) + pairs("drop/wind", off=[0.6] * 3, on=[0.31, 0.35, 0.20])
    assert "wind: flags-on median error 0.31 m is over 0.3 m" in run_ab.compare(PLAN, over_wind)["problems"][0]
    assert "PASS" not in run_ab.render(PLAN, run_ab.compare(PLAN, over_wind)).split("## Verdict")[1].split("\n")[2]


def test_a_floor_violation_with_the_flags_on_fails_whatever_the_error():
    runs = pairs("drop/calm", off=[0.3] * 3, on=[0.05] * 3) + pairs("drop/wind", off=[0.6] * 3, on=[0.2] * 3)
    runs[3]["details"]["released_low"] = True  # the first flags-on calm flight released under 1.83 m
    report = run_ab.compare(PLAN, runs)
    assert report["passed"] is False and any("floor" in problem and "drop/calm seed 1" in problem for problem in report["problems"])
    runs[3]["details"]["released_low"] = False
    runs[4]["details"]["flew_low"] = True
    assert any("floor" in problem for problem in run_ab.compare(PLAN, runs)["problems"])


def test_more_failures_with_the_flags_on_than_off_fails_and_a_failure_is_a_flight_that_did_not_score():
    runs = pairs("drop/calm", off=[0.3, None, 0.3], on=[0.05, None, 0.05]) + pairs("drop/wind", off=[0.6] * 3, on=[0.2] * 3)
    assert run_ab.compare(PLAN, runs)["passed"] is True  # one failure each: no more than flags off
    runs = pairs("drop/calm", off=[0.3] * 3, on=[0.05, None, 0.05]) + pairs("drop/wind", off=[0.6] * 3, on=[0.2] * 3)
    report = run_ab.compare(PLAN, runs)
    assert report["passed"] is False and "calm: 1 failure with the flags on, 0 with them off" in report["problems"][0]
    # A flight that did not end as a flight should is a failure too, released or not.
    for status in ("TIMEOUT", "NEVER_ARMED", "ERROR", "SOMETHING_NEW"):
        runs = pairs("drop/calm", off=[0.3] * 3, on=[0.05] * 3) + pairs("drop/wind", off=[0.6] * 3, on=[0.2] * 3)
        runs[3]["status"] = status
        assert run_ab.compare(PLAN, runs)["groups"]["calm"]["on"]["failures"] == 1, status


def test_a_check_beside_the_flight_that_missed_its_line_is_reported_but_is_not_a_failed_flight():
    """The runner marks a flight FAIL when a check that runs beside it (projection, estimator or height against the
    truth) misses its line. Those checks run only with the flags on, so counting them as failed flights would hold
    against the 2027 side something the 2026 side is never asked. They are counted apart, and named."""
    runs = pairs("drop/calm", off=[0.3] * 3, on=[0.05] * 3) + pairs("drop/wind", off=[0.6] * 3, on=[0.2, 0.09, 0.25])
    missed = next(run for run in runs if run["scenario"] == "drop/wind" and run["variant"] == "on" and run["seed"] == 2)
    missed["status"], missed["reason"] = "FAIL", "projection v2 missed its pass line: 0.30 m at 15 m"
    report = run_ab.compare(PLAN, runs)
    wind = report["groups"]["wind"]["on"]
    assert wind["failures"] == 0 and wind["check_misses"] == 1 and wind["median_m"] == pytest.approx(0.2)  # its release still counts
    assert report["passed"] is True
    assert report["check_misses"] == ["drop/wind seed 2, flags on: projection v2 missed its pass line: 0.30 m at 15 m"]
    text = run_ab.render(PLAN, report)
    assert "## Checks beside the flights that missed their line" in text and "drop/wind seed 2, flags on" in text
    assert "PASS" in text and "1 flags-on flight missed a check beside it" in text  # the verdict line itself says so
    # A FAIL flight in which nothing was released is still a failed flight.
    missed["details"]["released"] = False
    assert run_ab.compare(PLAN, runs)["groups"]["wind"]["on"]["failures"] == 1
    # Only the three checks that run beside a flags-on flight are "a check that missed": any other reason the
    # runner gives for FAIL (a package that never left the hook, a sensor that never published, a slow sim) is
    # a failed flight, released or not.
    missed["details"]["released"] = True
    for reason, is_check in (
        ("projection v2 missed its pass line: x", True),
        ("the target estimator missed a pass line: x", True),
        ("the height estimator missed its pass line: x", True),
        ("the payload bridge never let the package go", False),
        ("a sensor the scenario asked for never published", False),
        ("", False),
    ):
        missed["reason"] = reason
        side = run_ab.compare(PLAN, runs)["groups"]["wind"]["on"]
        assert (side["check_misses"], side["failures"]) == ((1, 0) if is_check else (0, 1)), reason


def test_an_incomplete_comparison_is_not_a_pass():
    runs = pairs("drop/calm", off=[0.3] * 3, on=[0.05] * 2) + pairs("drop/wind", off=[0.6] * 3, on=[0.2] * 3)
    report = run_ab.compare(PLAN, runs)
    assert report["passed"] is False and any("drop/calm seed 3 was not flown with the flags on" in problem for problem in report["problems"])
    assert run_ab.compare(PLAN, [])["passed"] is False
    # An INVALID run (the sim ran too slowly) is not a failure of either side: it has to be flown again.
    runs = pairs("drop/calm", off=[0.3] * 3, on=[0.05] * 3) + pairs("drop/wind", off=[0.6] * 3, on=[0.2] * 3)
    runs[0]["status"] = "INVALID"
    report = run_ab.compare(PLAN, runs)
    assert report["passed"] is False and any("INVALID" in problem for problem in report["problems"]) and report["groups"]["calm"]["off"]["failures"] == 0


def test_a_delivery_is_judged_on_failures_and_its_release_height_and_its_error_is_reported():
    plan = {"flag": "delivery_soft_set_down", "mission": "delivery", "calm": ["delivery/calm"], "wind": [], "seeds": [1, 2, 3]}
    runs = pairs("delivery/calm", off=[0.17, 0.2, 0.1], on=[0.02, 0.03, 0.01], mission="delivery")
    report = run_ab.compare(plan, runs)
    assert report["passed"] is True and report["groups"]["calm"]["on"]["median_m"] == pytest.approx(0.02)
    runs[4]["details"]["released_high"] = True
    assert any("above 1.83 m" in problem for problem in run_ab.compare(plan, runs)["problems"])


def test_results_are_read_from_the_suite_folders_with_each_runs_score(tmp_path):
    suite = tmp_path / "drop-calm-20261120T000000Z"
    run_dir = suite / "runs" / "drop-calm-s1-on"
    run_dir.mkdir(parents=True)
    (run_dir / "score.json").write_text(json.dumps({"mission": "drop", "details": {"released": True, "distance_m": 0.04, "released_low": False, "flew_low": False}}))
    (suite / "results.json").write_text(json.dumps([{"scenario": "drop/calm", "seed": 1, "variant": "on", "status": "OK", "reason": "", "flight_s": 91.5, "run_dir": str(run_dir)}]))
    (loaded,) = run_ab.load_runs([suite])
    assert loaded["details"]["distance_m"] == 0.04 and loaded["mission"] == "drop" and loaded["flight_s"] == 91.5
    # A run that left no score (it never armed) is still a run, and a failure.
    (run_dir / "score.json").unlink()
    (loaded,) = run_ab.load_runs([suite])
    assert loaded["details"] == {} and run_ab.failed(loaded)


def test_the_command_line_reports_from_finished_suites_without_flying(tmp_path, capsys):
    suite = tmp_path / "suite"
    suite.mkdir()
    (suite / "results.json").write_text("[]")
    out = tmp_path / "ab.md"
    assert run_ab.main(["drop", "--from", str(suite), "--report", str(out)]) == 1  # nothing flown: not a pass
    assert "FAIL" in out.read_text() and "was not flown" in out.read_text()
    with pytest.raises(SystemExit):
        run_ab.main(["no_such_plan", "--from", str(suite)])
