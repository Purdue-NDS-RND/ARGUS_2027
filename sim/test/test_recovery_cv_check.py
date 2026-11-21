"""The sim check of the green X and bucket detector (REC-1): the verdict's arithmetic, with no ROS."""

import json
import math
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import recovery_cv_check as check  # noqa: E402
from runner import events  # noqa: E402

SIZE = (1280, 720)


def corners(cx=640.0, cy=360.0, side=365.0):
    """The mat's four corners in the picture: a 5 m mat is 365 px a side from 10.67 m."""
    half = side / 2.0
    return [(cx - half, cy - half), (cx + half, cy - half), (cx + half, cy + half), (cx - half, cy + half)]


def frame(x_error=0.8, bucket_error=1.5, whole=True, span=479.0, height_m=10.67, speed=0.0):
    return {"whole": whole, "span_px": span, "height_m": height_m, "speed_mps": speed, "px_per_m": 72.9, "x_error_px": x_error, "bucket_error_px": bucket_error}


def test_one_frame_is_the_detectors_pixels_against_the_truths():
    found = [("green_x", 643.0, 356.0), ("bucket", 641.0, 361.0)]
    one = check.frame_of(found, (640.0, 360.0), (640.5, 360.5), corners(), SIZE, 10.67, 5.0, 0.3)
    assert one["speed_mps"] == 0.3 and one["px_per_m"] == pytest.approx(73.0) and check.frame_of(found, (640.0, 360.0), None, corners(), SIZE, 10.67, 5.0)["speed_mps"] is None
    assert one["whole"] is True and one["x_error_px"] == pytest.approx(5.0) and one["bucket_error_px"] == pytest.approx(math.hypot(0.5, 0.5))
    assert one["span_px"] == pytest.approx(check.ARM_SPAN_OVER_MAT * 365.0) and check.ARM_SPAN_OVER_MAT * 5.0 == pytest.approx(6.571, abs=0.001)
    # Nothing reported: no error to give. The mat partly out of the picture: not a frame the detector is held to.
    nothing = check.frame_of([], (640.0, 360.0), (640.0, 360.0), corners(), SIZE, 10.67, 5.0)
    assert nothing["x_error_px"] is None and nothing["bucket_error_px"] is None and nothing["whole"] is True
    edge = check.frame_of(found, (1200.0, 360.0), None, corners(cx=1200.0), SIZE, 10.67, 5.0)
    assert edge["whole"] is False and edge["bucket_error_px"] is None
    assert check.frame_of(found, None, None, None, SIZE, 10.67, 5.0)["whole"] is False  # the mat behind the camera


def test_a_flight_passes_when_the_x_is_found_nearly_always_and_within_the_specs_line():
    good = check.build_report([frame() for _ in range(60)] + [frame(whole=False) for _ in range(40)])
    assert good["passed"] and good["frames"] == 100 and good["frames_with_the_whole_mat"] == 60 and good["x_found"] == 60 and good["x_found_share"] == 1.0
    assert good["x_error_px"] == {"median": 0.8, "p95": 0.8, "max": 0.8} and good["x_error_px_slow"]["frames"] == 60 and good["bucket_found"] == 60 and good["heights_m"] == {"lowest": 10.67, "highest": 10.67}
    assert good["x_reported_without_the_whole_mat"] == 40
    lines = check.summary_lines(good)
    assert lines[0].startswith("PASS: the X reported in 60 of 60 frames") and "the bucket in 60" in lines[0]
    json.dumps(good)
    # The bucket is reported and not judged: a flight that never saw it still passes on the X.
    assert check.build_report([frame(bucket_error=None) for _ in range(30)])["passed"]


def test_a_flight_fails_on_each_line_it_misses():
    assert (check.MIN_FRAMES, check.FOUND_SHARE, check.CENTRE_PX, check.CENTRE_SHARE_OF_SPAN, check.EXTRA_PX) == (20, 0.95, 2.0, 0.01, 2.0)
    assert check.STILL_MPS == 1.0 and check.TRUTH_PERIOD_S == pytest.approx(1.0 / 30.0)
    few = check.build_report([frame() for _ in range(19)])
    assert not few["passed"] and "only 19 frame(s) had the whole mat" in few["problems"][0]
    missed = check.build_report([frame() for _ in range(90)] + [frame(x_error=None) for _ in range(10)])
    assert not missed["passed"] and "reported in 90 of the 100 frames" in missed["problems"][0]
    # The spec's line at this size is 4.79 px (1% of a 479 px span); 2 px more is allowed for the truth's timing.
    assert check.build_report([frame(x_error=6.7) for _ in range(40)])["passed"]
    far = check.build_report([frame(x_error=7.0) for _ in range(40)])
    assert not far["passed"] and "over what it is allowed at the 95th percentile" in far["problems"][0]
    assert check.summary_lines(far)[-1].startswith("problem: ")
    # At a survey's speed the truth itself is one of its periods out: 4 m/s for 33 ms is 13 cm, 9.7 px at 72.9 px a
    # metre. A centre 15 px out at that speed is within its allowance (4.79 + 2 + 9.72); the same error standing still is not.
    moving = check.build_report([frame(x_error=15.0, speed=4.0) for _ in range(40)])
    assert moving["passed"] and moving["x_over_its_allowance_px_p95"] == pytest.approx(15.0 - 4.79 - 2.0 - 4.0 / 30.0 * 72.9, abs=0.01)
    assert moving["x_error_px"]["p95"] == 15.0 and moving["speeds_mps"] == {"median": 4.0, "max": 4.0} and moving["x_error_px_slow"]["frames"] == 0
    assert "at up to 4.0 m/s" in check.summary_lines(moving)[0]
    assert not check.build_report([frame(x_error=17.0, speed=4.0) for _ in range(40)])["passed"]
    assert not check.build_report([frame(x_error=15.0, speed=0.0) for _ in range(40)])["passed"]
    # A frame whose true speed is not known has no allowance to be given, and is not judged on its centre.
    unknown = check.build_report([frame(speed=None) for _ in range(40)])
    assert not unknown["passed"] and "had a true speed to judge its centre by" in unknown["problems"][0]
    # One wild frame in forty does not fail a flight: the line is on the 95th percentile.
    assert check.build_report([frame() for _ in range(39)] + [frame(x_error=40.0)])["passed"]
    assert not check.build_report([])["passed"]


def test_the_check_is_wanted_by_the_nodes_flag_and_read_back_from_the_run_folder(tmp_path):
    on = {"node_flags": {"recovery_cv": {"recovery_cv": True}}}
    assert check.wanted(on) and not check.wanted({"node_flags": {"recovery_cv": {"recovery_cv": False}}}) and not check.wanted({})
    assert "recovery_cv_check" in events.tools_needed(on) and "recovery_cv_check" not in events.tools_needed({})
    assert "sim/recovery_cv_check.py" in events.unsupported(on, tools_present=set())
    assert check.read_check(tmp_path) is None
    (tmp_path / "recovery_cv_check.json").write_text(json.dumps(check.build_report([frame() for _ in range(30)])))
    assert check.read_check(tmp_path)["passed"] is True
    (tmp_path / "recovery_cv_check.json").write_text("{not json")
    assert check.read_check(tmp_path) is None


def test_the_survey_over_the_mat_builds_with_the_node_started_and_its_flag_on_only_in_the_run(tmp_path):
    import yaml
    from scenario.build import build_run

    scenario = SIM / "scenarios" / "waypoint_navigation" / "survey_x_mat.yaml"
    run = build_run(scenario, tmp_path / "on", variant="on")
    manifest = json.loads((run / "scenario.json").read_text())
    assert manifest["launch"]["file"] == "search_pattern_demo.launch.py" and manifest["launch"]["arguments"]["recovery_cv"] == "true" and manifest["truth_poses"] is True
    settings = yaml.safe_load((run / "params.yaml").read_text())["recovery_cv"]["ros__parameters"]
    assert settings == {"camera_mode": "sim_survey", "airframe": "sim_iris_downcam", "image_topic": "camera/image", "features": {"recovery_cv": True}}
    kinds = {item["kind"] for item in json.loads((run / "truth.json").read_text())["objects"]}
    assert kinds == {"x_mat", "bucket"}
    off = build_run(scenario, tmp_path / "off", variant="off")
    assert "recovery_cv" not in json.loads((off / "scenario.json").read_text())["launch"]["arguments"]
    assert yaml.safe_load((off / "params.yaml").read_text())["recovery_cv"]["ros__parameters"]["features"]["recovery_cv"] is False
    # The shipped params file has the flag off, under the node's own ros__parameters.
    shipped = yaml.safe_load((SIM.parent / "src" / "drone_mission_demo" / "config" / "params" / "sim_survey_mission.yaml").read_text())
    assert shipped["recovery_cv"]["ros__parameters"]["features"] == {"recovery_cv": False} and "features" not in shipped
