"""The sim's check of projection v2 against the truth (FND-05): the arithmetic, the verdict and the runner's use of it. No ROS."""

import json
import math
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import projection_check  # noqa: E402
from runner import events, result  # noqa: E402
from scenario.build import build_run  # noqa: E402

from test_runner import SCORE, make_run  # noqa: E402  (the run-folder fixture)

FRAME_TIME = SIM / "scenarios" / "delivery" / "frame_time.yaml"
S = 1_000_000_000
MS = 1_000_000


def quaternion(roll_deg=0.0, yaw_deg=90.0):
    roll, yaw = math.radians(roll_deg) / 2.0, math.radians(yaw_deg) / 2.0
    return (math.sin(roll) * math.cos(yaw), math.sin(roll) * math.sin(yaw), math.cos(roll) * math.sin(yaw), math.cos(roll) * math.cos(yaw))


def fly_over(checker, height_m=15.0, seconds=3.0, latency_s=0.15, speed=2.0, pixel_error=0.0):
    """Feed a flight east past the target at one height, rocking in roll, with detections `latency_s` behind the poses."""
    from drone_mission_core import projection_v2 as v2

    def pose(t):
        return ((checker.target[0] - 3.0 + speed * t, checker.target[1] - 1.0, height_m), quaternion(roll_deg=5.0 * math.sin(2.0 * t)))

    checker.image_size = (640, 360)
    for index in range(int(seconds * 30)):
        t = index / 30.0
        position, orientation = pose(t)
        checker.poses.add_pose(100 * S + round(t * S), position, orientation)
        checker.poses.add_velocity(100 * S + round(t * S), (speed, 0.0, 0.0))
        if t >= latency_s + 0.2 and index % 2 == 0:
            frame_t = t - latency_s
            u, v = v2.pixel_of_ground_point(checker.camera, checker.mount, (*checker.target, 0.0), *pose(frame_t))
            checker.on_detection(100 * S + round(frame_t * S), (u + pixel_error) / 640 * 2 - 1, v / 360 * 2 - 1)


def test_the_target_is_taken_from_the_truth_relative_to_the_take_off_point():
    assert projection_check.target_in_local_frame({"target": {"e": 10.0, "n": 15.0}, "takeoff": {"e": 0.0, "n": 0.0}}) == (10.0, 15.0)
    assert projection_check.target_in_local_frame({"target": {"e": 10.0, "n": 15.0}, "takeoff": {"e": 4.0, "n": -1.0}}) == (6.0, 16.0)


def test_a_flight_whose_detections_are_projected_with_the_frame_time_pose_passes():
    checker = projection_check.Checker((10.0, 15.0))
    fly_over(checker)
    report = checker.report()
    assert report["passed"] and report["problems"] == []
    assert report["at_15_m"]["count"] >= 20 and report["at_15_m"]["p95_m"] < 0.01
    # The same pixels with the pose at arrival, 150 ms later at 2 m/s and rocking: what 2026 did.
    assert report["at_15_m_with_the_latest_pose"]["p95_m"] > 0.3
    assert report["height_sources"] == {"home": report["projected"]} and report["refusals"] == {}


def test_a_projection_that_is_off_by_more_than_the_limit_fails_and_says_by_how_much():
    checker = projection_check.Checker((10.0, 15.0))
    fly_over(checker, pixel_error=12.0)  # 12 px at 15 m is about 0.46 m
    report = checker.report()
    assert not report["passed"] and "from the truth at the 95th percentile at 15 m (limit 0.3 m)" in report["problems"][0]
    assert 0.4 < report["at_15_m"]["median_m"] < 0.55
    worst = report["worst_at_15_m"]
    assert len(worst) == 5 and worst[0]["error_m"] == round(report["at_15_m"]["max_m"], 3) and -1.0 <= worst[0]["x_norm"] <= 1.0
    assert report["height_sources_at_15_m"] == {"home": report["at_15_m"]["count"]}


def test_frames_in_which_the_target_was_partly_out_of_the_picture_are_counted_apart_and_not_judged():
    """On arrival the 5 m bullseye comes in over the picture's edge, and the detector reports the middle of the part
    in view: up to 2.5 m off, and the projection's fault in none of it. With the target's size known from the
    truth, those frames are told apart by where the whole target was, not by how wrong the detection is."""
    from drone_mission_core import projection_v2 as v2

    assert projection_check.target_size_from({"target": {"name": "t"}, "objects": [{"name": "t", "kind": "bullseye", "size_m": 5.0}]}) == 5.0
    assert projection_check.target_size_from({"target": {"e": 1.0, "n": 2.0}}) is None  # an older truth file: every frame is judged, as before

    def arrive(target_size_m):
        checker = projection_check.Checker((10.0, 15.0), target_size_m=target_size_m)
        checker.image_size = (640, 360)
        # Coming in from the south at 15 m, nose north: the target enters at the top of the picture.
        for index in range(240):
            t = index / 30.0
            north = 15.0 - 8.0 + min(8.0, 2.0 * t)  # arrives over the target after 4 s, then holds
            position, orientation = (10.0, north, 15.0), quaternion(roll_deg=0.0)
            checker.poses.add_pose(100 * S + round(t * S), position, orientation)
            checker.poses.add_velocity(100 * S + round(t * S), (0.0, 2.0 if t < 4.0 else 0.0, 0.0))
            if index % 2 == 0 and index > 6:
                u, v = v2.pixel_of_ground_point(checker.camera, checker.mount, (10.0, 15.0, 0.0), position, orientation)
                top = v - 2.5 / 15.0 * 389.165  # the target's far edge, in pixels
                if top < 0.0:  # partly out of view: the detector's centre is the middle of the part in view
                    v = 0.5 * (0.0 + min(360.0, v + 2.5 / 15.0 * 389.165))
                if 0.0 <= v < 360.0:
                    checker.on_detection(100 * S + round(t * S), u / 640 * 2 - 1, v / 360 * 2 - 1)
        return checker.report()

    blind = arrive(None)
    assert not blind["passed"] and blind["at_15_m"]["max_m"] > 1.0  # every frame judged: the arrival fails the flight
    told = arrive(5.0)
    assert told["passed"], told["problems"]
    assert told["at_15_m"]["p95_m"] < 0.01 and told["at_15_m"]["count"] >= 20
    partly = told["at_15_m_target_partly_out_of_view"]
    assert partly["count"] >= 5 and partly["max_m"] > 1.0 and partly["count"] + told["at_15_m"]["count"] == blind["at_15_m"]["count"]
    # A projection that really is off still fails, on the frames with the whole target in view.
    checker = projection_check.Checker((10.0, 15.0), target_size_m=1.5)
    fly_over(checker, pixel_error=12.0)
    assert not checker.report()["passed"]


def test_too_few_detections_at_15_m_is_not_a_pass():
    checker = projection_check.Checker((10.0, 15.0))
    fly_over(checker, height_m=8.0)
    report = checker.report()
    assert not report["passed"] and "only 0 projected detections between 13.5 and 16.5 m" in report["problems"][0]
    assert report["all_heights"]["count"] >= 20 and report["at_15_m"] is None


def test_nothing_heard_and_nothing_projectable_are_failures_with_the_reason():
    checker = projection_check.Checker((10.0, 15.0))
    assert checker.report()["problems"] == ["no detection of the target was heard"]
    checker.image_size = (640, 360)
    checker.on_detection(100 * S, -1.0, -1.0)  # the detector's "nothing found" is not a detection
    assert checker.detections == 0
    checker.on_detection(100 * S, 0.1, 0.1)  # no pose history at all
    report = checker.report()
    assert not report["passed"] and "none of 1 detections could be projected" in report["problems"][0]
    assert report["refusals"] == {"no pose for the frame's capture time": 1}


def test_the_rangefinders_ground_is_used_when_it_is_heard():
    checker = projection_check.Checker((10.0, 15.0))
    fly_over(checker, seconds=1.0)  # poses first: a reading needs the pose at its own moment
    for ms in (400, 450, 500, 550, 600):
        checker.ground.add_range(100 * S + ms * MS, 15.0, 0.0, 1000.0)
    checker.records.clear()
    fly_over(checker, seconds=1.0)
    assert checker.report()["height_sources"].get("rangefinder", 0) > 0


def test_report_file_round_trip(tmp_path):
    assert projection_check.read_projection_check(tmp_path) is None
    (tmp_path / "projection_check.json").write_text(json.dumps({"passed": True, "problems": []}))
    assert projection_check.read_projection_check(tmp_path)["passed"] is True
    (tmp_path / "projection_check.json").write_text("{not json")
    assert projection_check.read_projection_check(tmp_path) is None


# --- the runner ---


def test_the_check_runs_beside_flights_with_the_flag_on_and_only_those(tmp_path):
    calm = {"wind": {"level": "calm"}, "gps_events": []}
    assert "projection_check" not in events.tools_needed({**calm, "sim_time": True, "flags": {}})
    assert "projection_check" not in events.tools_needed({**calm, "sim_time": True, "flags": {"use_frame_time_pose": False}})
    assert events.tools_needed({**calm, "sim_time": True, "flags": {"use_frame_time_pose": True}}) == {"time_check", "rate_check", "projection_check"}
    assert "projection v2" in events.unsupported({**calm, "flags": {"use_frame_time_pose": True}}, set())
    assert '"projection_check": SIM_DIR / "projection_check.py"' in (SIM / "runner" / "fly.py").read_text()
    on = json.loads((build_run(FRAME_TIME, tmp_path / "on", variant="on") / "scenario.json").read_text())
    off = json.loads((build_run(FRAME_TIME, tmp_path / "off", variant="off") / "scenario.json").read_text())
    assert projection_check.wanted(on) and on["sim_time"] is True and on["node_flags"]["target_cv"]["stamp_from_image"] is True
    assert not projection_check.wanted(off) and off.get("sim_time") is not True


PASSED = {"passed": True, "problems": [], "limit_m": 0.3, "at_15_m": {"count": 240, "median_m": 0.08, "p95_m": 0.17, "max_m": 0.24}, "at_15_m_with_the_latest_pose": {"count": 240, "median_m": 0.2, "p95_m": 0.61, "max_m": 0.9}}
MISSED = {"passed": False, "problems": ["projected target is 0.42 m from the truth at the 95th percentile at 15 m (limit 0.3 m)"], "limit_m": 0.3, "at_15_m": {"count": 240, "median_m": 0.3, "p95_m": 0.42, "max_m": 0.6}}
TIME_OK = {"passed": True, "problems": [], "pose": {"count": 4000, "median_ms": 4.0, "p95_abs_ms": 5.3, "max_abs_ms": 6.6}}
RATES_OK = {"passed": True, "problems": [], "topics": {"/mavros/local_position/pose": {"measured_hz": 30.2, "asked_hz": 30.0, "status": "PASS"}}}


def flag_on_run(tmp_path, report, flags=None):
    run = make_run(tmp_path, score=SCORE, manifest={"variant": "on", "sim_time": True, "flags": {"use_frame_time_pose": True} if flags is None else flags})
    (run / "time_check.json").write_text(json.dumps(TIME_OK))
    (run / "rate_check.json").write_text(json.dumps(RATES_OK))
    if report is not None:
        (run / "projection_check.json").write_text(json.dumps(report))
    return run


def test_a_flag_on_run_counts_only_with_a_passing_projection_check(tmp_path):
    passed = result.run_result(flag_on_run(tmp_path / "a", PASSED))
    assert passed["status"] == "OK"
    text = result.render_report(passed)
    assert "Projected target against the truth at 15 m | PASS: median 0.08 m, p95 0.17 m over 240 detections (limit 0.3 m); with the latest pose instead, p95 0.61 m" in text
    missed = result.run_result(flag_on_run(tmp_path / "b", MISSED))
    assert missed["status"] == "FAIL" and "0.42 m from the truth" in missed["reason"]
    absent = result.run_result(flag_on_run(tmp_path / "c", None))
    assert absent["status"] == "INVALID" and "projection_check.json" in absent["reason"]


def test_a_run_without_the_flag_needs_no_projection_check(tmp_path):
    summary = result.run_result(flag_on_run(tmp_path, None, flags={}))
    assert summary["status"] == "OK" and summary["projection_check"] is None
    assert "Projected target" not in result.render_report(summary)
