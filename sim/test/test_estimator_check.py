"""The sim's check of the target estimator against the truth (FND-06): the arithmetic, the verdict and the runner's use of it. No ROS."""

import json
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import estimator_check  # noqa: E402
from runner import events, result  # noqa: E402
from scenario.build import build_run  # noqa: E402

from test_runner import SCORE, make_run  # noqa: E402  (the run-folder fixture)

ESTIMATOR = SIM / "scenarios" / "delivery" / "estimator.yaml"
TARGET = (10.0, 15.0)


def track(error_m=0.05, state="CONFIRMED", track_id=1, class_id="bullseye", sigma_m=0.1):
    return {"id": f"{class_id}/{track_id}/{state}", "east_m": TARGET[0] + error_m, "north_m": TARGET[1], "sigma_m": sigma_m}


def descent(checker, errors=None):
    """A flight that hovers at 15 m and comes down to the ground, the tracks published at every 2 cm of height."""
    errors = errors or {}
    for _ in range(100):
        checker.on_height(15.0)
        checker.on_targets([track(errors.get(15.0, 0.05))])
    height = 15.0
    while height > 0.3:
        height -= 0.02
        checker.on_height(height)
        band = min((6.0, 2.4), key=lambda h: abs(h - height))
        checker.on_targets([track(errors.get(band, 0.05))])


def test_the_lines_are_the_specs():
    assert estimator_check.LINES == ((15.0, 1.5, 0.30), (6.0, 1.0, 0.15), (2.4, 0.4, 0.08))
    assert estimator_check.MIN_IN_BAND == 20 and estimator_check.FLAG == "use_target_estimator"


def test_a_flight_whose_track_is_on_the_target_at_every_height_passes():
    checker = estimator_check.Checker(TARGET)
    descent(checker)
    report = checker.report()
    assert report["passed"] and report["problems"] == []
    assert report["at_15_m"]["error"]["p95_m"] == pytest.approx(0.05) and report["at_15_m"]["passed"]
    assert report["at_6_m"]["error"]["count"] >= 20 and report["at_2_4_m"]["error"]["count"] >= 20
    assert report["at_15_m"]["within_3_sigma"] == 1.0 and report["at_15_m"]["median_sigma_m"] == pytest.approx(0.1)
    assert report["confirmed_track_ids"] == [1]
    assert estimator_check.summary_lines(report)[0].startswith("at 15 m: PASS, median 0.05 m, p95 0.05 m over")


@pytest.mark.parametrize("height_m, error_m, name", [(15.0, 0.35, "at_15_m"), (6.0, 0.2, "at_6_m"), (2.4, 0.1, "at_2_4_m")])
def test_each_height_has_its_own_line(height_m, error_m, name):
    checker = estimator_check.Checker(TARGET)
    descent(checker, errors={height_m: error_m})
    report = checker.report()
    assert not report["passed"] and not report[name]["passed"]
    assert sum(1 for line in estimator_check.LINES if report[estimator_check.band_name(line[0])]["passed"]) == 2
    assert f"from the truth at the 95th percentile at {height_m:g} m" in report["problems"][0]


def test_only_a_confirmed_bullseye_track_counts_and_the_oldest_if_there_are_two():
    checker = estimator_check.Checker(TARGET)
    checker.on_targets([track()])  # before any height is known: heard, not judged
    checker.on_height(15.0)
    checker.on_targets([track(state="TENTATIVE")])
    checker.on_targets([track(state="LOST"), track(class_id="bucket")])
    checker.on_targets([])
    checker.on_targets([track(error_m=3.0, track_id=7), track(error_m=0.1, track_id=2)])
    checker.on_targets([{"id": "not a track id", "east_m": 0.0, "north_m": 0.0, "sigma_m": 0.1}])
    report = checker.report()
    assert report["messages"] == 6 and report["samples"] == 5
    assert report["at_15_m"]["without_a_confirmed_track"] == 4 and report["at_15_m"]["error"]["count"] == 1
    assert report["at_15_m"]["error"]["max_m"] == pytest.approx(0.1) and report["confirmed_track_ids"] == [2]
    assert report["at_15_m"]["states"] == {"CONFIRMED": 1, "LOST": 1, "TENTATIVE": 1, "none": 2}
    assert not report["passed"] and "only 1 samples of a confirmed bullseye track between 13.5 and 16.5 m" in report["problems"][0]


def test_a_flight_with_no_tracks_published_fails_and_says_so():
    report = estimator_check.Checker(TARGET).report()
    assert not report["passed"] and report["problems"] == ["nothing was heard on /estimator/targets"]
    assert estimator_check.summary_lines(report)[0] == "at 15 m: no confirmed bullseye track in 0 samples"


def test_track_ids_are_read_as_the_estimator_writes_them():
    assert estimator_check.parse_track_id("bullseye/12/COASTING") == ("bullseye", 12, "COASTING")
    assert estimator_check.parse_track_id("bullseye/x/COASTING") is None and estimator_check.parse_track_id("") is None
    sys.path.insert(0, str(SIM.parent / "src" / "drone_mission_core"))
    from drone_mission_core.estimator_feed import TARGETS_TOPIC, encode_targets  # the writer, for the format
    from drone_mission_core.target_estimator import Measurement, TargetEstimator

    estimator = TargetEstimator()
    estimator.update([Measurement("bullseye", 1.0, 2.0, [[0.01, 0.0], [0.0, 0.01]])], 10)

    class Any:
        def __getattr__(self, name):
            value = [0.0] * 36 if name == "covariance" else ([] if name in ("detections", "results") else Any())
            object.__setattr__(self, name, value)
            return value

    messages = type("Messages", (), {"Detection3DArray": Any, "Detection3D": Any, "ObjectHypothesisWithPose": Any})
    (detection,) = encode_targets(estimator.estimates(10), 10, messages).detections
    assert estimator_check.parse_track_id(detection.id) == ("bullseye", 1, "TENTATIVE")
    assert TARGETS_TOPIC == estimator_check.TARGETS_TOPIC


def test_report_file_round_trip(tmp_path):
    assert estimator_check.read_estimator_check(tmp_path) is None
    (tmp_path / "estimator_check.json").write_text(json.dumps({"passed": True, "problems": []}))
    assert estimator_check.read_estimator_check(tmp_path)["passed"] is True
    (tmp_path / "estimator_check.json").write_text("{not json")
    assert estimator_check.read_estimator_check(tmp_path) is None


# --- the runner ---


def test_the_check_runs_beside_flights_with_the_flag_on_and_only_those(tmp_path):
    calm = {"wind": {"level": "calm"}, "gps_events": []}
    both = {"use_frame_time_pose": True, "use_target_estimator": True}
    assert "estimator_check" not in events.tools_needed({**calm, "sim_time": True, "flags": {"use_frame_time_pose": True}})
    assert events.tools_needed({**calm, "sim_time": True, "flags": both}) == {"time_check", "rate_check", "projection_check", "estimator_check"}
    assert "the estimator against the truth" in events.unsupported({**calm, "flags": both}, {"projection_check"})
    assert '"estimator_check": SIM_DIR / "estimator_check.py"' in (SIM / "runner" / "fly.py").read_text()
    on = json.loads((build_run(ESTIMATOR, tmp_path / "on", variant="on") / "scenario.json").read_text())
    off = json.loads((build_run(ESTIMATOR, tmp_path / "off", variant="off") / "scenario.json").read_text())
    assert estimator_check.wanted(on) and on["sim_time"] is True
    assert on["node_flags"]["target_cv"] == {"stamp_from_image": True, "detection_contract": True}
    assert not estimator_check.wanted(off) and off.get("sim_time") is not True and off["flags"] == {}


def test_the_scenario_is_in_the_nightly_suite():
    import yaml

    assert "delivery/estimator" in yaml.safe_load((SIM / "scenarios" / "suites.yaml").read_text())["nightly"]["scenarios"]


def _band(p95, limit, passed=True):
    return {"limit_m": limit, "samples": 60, "without_a_confirmed_track": 0, "error": {"count": 60, "median_m": p95 / 2.0, "p95_m": p95, "max_m": p95}, "passed": passed}


PASSED = {"passed": True, "problems": [], "at_15_m": _band(0.12, 0.3), "at_6_m": _band(0.06, 0.15), "at_2_4_m": _band(0.04, 0.08)}
MISSED = {
    "passed": False,
    "problems": ["the bullseye track is 0.21 m from the truth at the 95th percentile at 2.4 m (limit 0.08 m)"],
    "at_15_m": _band(0.12, 0.3),
    "at_6_m": _band(0.06, 0.15),
    "at_2_4_m": _band(0.21, 0.08, passed=False),
}
PROJECTION_OK = {"passed": True, "problems": [], "limit_m": 0.3, "at_15_m": {"count": 240, "median_m": 0.08, "p95_m": 0.17, "max_m": 0.24}}
TIME_OK = {"passed": True, "problems": [], "pose": {"count": 4000, "median_ms": 4.0, "p95_abs_ms": 5.3, "max_abs_ms": 6.6}}
RATES_OK = {"passed": True, "problems": [], "topics": {"/mavros/local_position/pose": {"measured_hz": 30.2, "asked_hz": 30.0, "status": "PASS"}}}


def flag_on_run(tmp_path, report, flags=None):
    flags = {"use_frame_time_pose": True, "use_target_estimator": True} if flags is None else flags
    run = make_run(tmp_path, score=SCORE, manifest={"variant": "on", "sim_time": True, "flags": flags})
    (run / "time_check.json").write_text(json.dumps(TIME_OK))
    (run / "rate_check.json").write_text(json.dumps(RATES_OK))
    (run / "projection_check.json").write_text(json.dumps(PROJECTION_OK))
    if report is not None:
        (run / "estimator_check.json").write_text(json.dumps(report))
    return run


def test_a_flag_on_run_counts_only_with_a_passing_estimator_check(tmp_path):
    passed = result.run_result(flag_on_run(tmp_path / "a", PASSED))
    assert passed["status"] == "OK"
    text = result.render_report(passed)
    assert "| Estimator's bullseye track against the truth | at 15 m: PASS, median 0.06 m, p95 0.12 m over 60 samples (limit 0.3 m) |" in text
    assert "at 2.4 m: PASS, median 0.02 m, p95 0.04 m over 60 samples (limit 0.08 m)" in text
    missed = result.run_result(flag_on_run(tmp_path / "b", MISSED))
    assert missed["status"] == "FAIL" and "0.21 m from the truth" in missed["reason"]
    assert "at 2.4 m: FAIL" in result.render_report(missed)
    absent = result.run_result(flag_on_run(tmp_path / "c", None))
    assert absent["status"] == "INVALID" and "estimator_check.json" in absent["reason"]


def test_a_run_without_the_flag_needs_no_estimator_check(tmp_path):
    summary = result.run_result(flag_on_run(tmp_path, None, flags={"use_frame_time_pose": True}))
    assert summary["status"] == "OK" and summary["estimator_check"] is None
    assert "Estimator's bullseye track" not in result.render_report(summary)
