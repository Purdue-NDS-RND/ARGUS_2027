"""REC-5: the sim's check of the flow odometry against the true velocity (sim/flow_check.py). No ROS."""

import math
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import flow_check as fc  # noqa: E402


def flight(height_m=10.0, speed=(3.0, 1.0), seconds=40.0):
    """A straight flight at a height, as Gazebo gives its poses (50 a second)."""
    track = fc.TrueTrack()
    track.add(0.0, (0.0, 0.0, 0.2))
    for step in range(1, int(seconds * 50)):
        t = step / 50.0
        track.add(t, (speed[0] * t, speed[1] * t, 0.2 + height_m))
    return track


def samples(count=200, east=3.0, north=1.0, quality=200, why="", start_s=2.0, noise=0.0):
    return [{"stamp_ns": int((start_s + index * 0.05) * 1e9), "dt_s": 0.2, "quality": quality, "why": why,
             "east_mps": east + noise * math.sin(index), "north_mps": north + noise * math.cos(index)} for index in range(count)]  # fmt: skip


def test_the_true_mean_velocity_is_the_displacement_over_the_samples_own_time():
    track = flight()
    found = track.mean_velocity(10.0, 0.2)
    assert found["east_mps"] == pytest.approx(3.0) and found["north_mps"] == pytest.approx(1.0) and found["height_m"] == pytest.approx(10.0)
    assert track.mean_velocity(0.1, 0.2) is None and track.mean_velocity(100.0, 0.2) is None and track.mean_velocity(10.0, 0.0) is None
    gap = fc.TrueTrack()
    for t in (0.0, 1.0, 2.0):
        gap.add(t, (t, 0.0, 5.0))
    assert gap.mean_velocity(1.5, 0.2) is None  # poses a second apart are not a track to measure by


def test_samples_that_match_the_truth_pass_and_the_rms_is_the_errors_length():
    report = fc.build_report(samples(noise=0.05), flight())
    assert report["passed"] and report["problems"] == [] and report["rms_mps"] == pytest.approx(0.05, abs=0.005) and report["in_band_with_quality"] == 200 and report["quality_share"] == 1.0
    assert "velocity error RMS 0.050 m/s over 200 samples at 5 to 15 m (100% of the samples there had quality); the line is 0.15 m/s" in fc.summary_lines(report)[0]
    # A scale error (the height or the lens wrong by a tenth) shows along the flight.
    scaled = fc.build_report(samples(east=3.3, north=1.1), flight())
    assert not scaled["passed"] and scaled["rms_mps"] == pytest.approx(0.316, abs=0.01) and scaled["along_mean_mps"] == pytest.approx(0.316, abs=0.01) and abs(scaled["across_mean_mps"]) < 0.01
    assert "is 0.316 m/s, over 0.15 m/s" in scaled["problems"][0]
    # A sign or axis error is far over the line.
    assert fc.build_report(samples(east=-3.0, north=-1.0), flight())["rms_mps"] > 6.0


def test_a_flight_that_did_not_show_the_odometry_fails_with_that_reason_and_other_heights_are_not_judged():
    bare = fc.build_report(samples(quality=0, why="no texture in the picture (a bare surface)"), flight())
    assert not bare["passed"] and "only 0 sample(s) with quality at 5 to 15 m (of 200 there)" in bare["problems"][0] and bare["rms_mps"] is None
    assert bare["no_quality"] == {"no texture in the picture (a bare surface)": 200} and "no sample with quality" in fc.summary_lines(bare)[0]
    few = fc.build_report(samples(count=40), flight())
    assert not few["passed"] and "only 40 sample(s)" in few["problems"][0]
    high = fc.build_report(samples(east=4.0), flight(height_m=25.0))  # wrong by 1 m/s, at 25 m: not in the band
    assert not high["passed"] and high["in_band"] == 0 and high["rms_elsewhere_mps"] == pytest.approx(1.0) and "only 0 sample(s)" in high["problems"][0]
    assert fc.build_report([], flight())["passed"] is False
    late = fc.build_report(samples(start_s=500.0), flight())  # samples with no truth around them are not counted either way
    assert late["unmatched"] == 200 and late["in_band"] == 0


def test_the_check_is_wanted_by_the_nodes_flag_and_reads_the_nodes_own_status_fields():
    assert fc.wanted({"node_flags": {"flow_odometry": {"flow_odometry": True}}}) and not fc.wanted({"node_flags": {"flow_odometry": {"flow_odometry": False}}}) and not fc.wanted({})
    checker = fc.Checker()
    checker.on_status('{"stamp_ns": 2000000000, "quality": 200, "why": "", "dt_s": 0.2, "east_mps": 3.0, "north_mps": 1.0, "height_m": 10.0}')
    checker.on_status("not json")
    checker.on_status('{"quality": 3}')
    assert len(checker._samples) == 1
    node = (SIM.parent / "src/drone_target_cv/drone_target_cv/flow_odometry_node.py").read_text()
    assert f'STATUS_TOPIC = "{fc.STATUS_TOPIC}"' in node and all(f'"{key}"' in node for key in ("stamp_ns", "quality", "why", "dt_s", "east_mps", "north_mps"))
    assert fc.RMS_LIMIT_MPS == 0.15 and fc.BAND_M == (5.0, 15.0)  # the spec's pass line
