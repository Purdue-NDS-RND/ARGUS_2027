"""The MAVROS rate check's verdict (PLT-06). No ROS."""

import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import rate_check  # noqa: E402

GOOD = {
    "/mavros/local_position/pose": 606,
    "/mavros/imu/data": 1000,
    "/mavros/global_position/global": 400,
    "/mavros/gpsstatus/gps1/raw": 100,
    "/mavros/battery": 40,
    "/mavros/sys_status": 40,
    "/mavros/vfr_hud": 400,
    "/mavros/extended_state": 40,
    "/mavros/rangefinder/rangefinder": 400,
}


def test_topics_are_judged_against_the_stream_keepers_rates():
    assert rate_check.expected_hz("LOCAL_POSITION_NED") == 30.0
    assert rate_check.expected_hz("ATTITUDE") == 50.0
    assert {mavlink for _type, mavlink, _optional in rate_check.TOPICS.values()} <= {name for name, _rate in rate_check.MESSAGE_RATES_HZ.values()}


def test_rates_at_or_near_what_was_asked_pass():
    report = rate_check.judge(GOOD, 20.0)
    assert report["passed"] and report["problems"] == []
    pose = report["topics"]["/mavros/local_position/pose"]
    assert pose["measured_hz"] == 30.3 and pose["asked_hz"] == 30.0 and pose["status"] == "PASS"


def test_a_rate_under_90_percent_fails_and_one_at_90_passes():
    report = rate_check.judge({**GOOD, "/mavros/local_position/pose": 85}, 20.0)  # 4.25 Hz, as MAVProxy leaves it
    assert not report["passed"]
    assert report["problems"] == ["/mavros/local_position/pose is at 4.2 Hz, under 90% of the 30 Hz asked (LOCAL_POSITION_NED)"]
    assert rate_check.judge({**GOOD, "/mavros/local_position/pose": 540}, 20.0)["passed"]  # 27 Hz
    assert not rate_check.judge({**GOOD, "/mavros/local_position/pose": 538}, 20.0)["passed"]


def test_a_silent_topic_fails_unless_its_sensor_is_optional():
    report = rate_check.judge({**GOOD, "/mavros/imu/data": 0}, 20.0)
    assert not report["passed"] and "silent" in report["problems"][0]
    report = rate_check.judge({**GOOD, "/mavros/rangefinder/rangefinder": 0}, 20.0)  # no rangefinder fitted
    assert report["passed"] and report["topics"]["/mavros/rangefinder/rangefinder"]["status"] == "silent"
    # A rangefinder that is there but slow still fails.
    assert not rate_check.judge({**GOOD, "/mavros/rangefinder/rangefinder": 100}, 20.0)["passed"]


def test_a_flight_is_judged_on_its_median_window_so_a_quiet_takeoff_does_not_fail_it():
    # 160 s: the first 30 s at the takeoff service's 20 Hz while the keeper keeps out of the way, then 50 Hz.
    attitude = [20.0] * 6 + [50.0] * 26
    counts = {**{topic: count * 8 for topic, count in GOOD.items()}, "/mavros/imu/data": int(sum(rate * 5.0 for rate in attitude))}
    assert not rate_check.judge(counts, 160.0)["passed"]  # on the mean (44.4 Hz) it would fail
    report = rate_check.judge(counts, 160.0, {"/mavros/imu/data": attitude})
    row = report["topics"]["/mavros/imu/data"]
    assert report["passed"] and row["measured_hz"] == 50.0 and row["lowest_window_hz"] == 20.0 and row["windows"] == 32 and row["low_windows"] == 6
    assert row["mean_hz"] == pytest.approx(44.4, abs=0.1)
    # A rate that is low most of the time still fails, and so does one that is low for more than a quarter of it.
    mostly_low = [20.0] * 20 + [50.0] * 12
    assert not rate_check.judge(counts, 160.0, {"/mavros/imu/data": mostly_low})["passed"]
    over_a_quarter = rate_check.judge(counts, 160.0, {"/mavros/imu/data": [20.0] * 9 + [50.0] * 23})
    assert not over_a_quarter["passed"] and "9 of its 32 five-second windows" in over_a_quarter["problems"][0]
    assert rate_check.judge(counts, 160.0, {"/mavros/imu/data": [20.0] * 8 + [50.0] * 24})["passed"] and rate_check.MAX_LOW_SHARE == 0.25
    assert rate_check.median([3.0, 1.0, 2.0]) == 2.0 and rate_check.median([4.0, 1.0, 2.0, 3.0]) == 2.5 and rate_check.WINDOW_S == 5.0


def test_report_file_round_trip(tmp_path):
    import json

    assert rate_check.read_rate_check(tmp_path) is None
    (tmp_path / "rate_check.json").write_text(json.dumps(rate_check.judge(GOOD, 20.0)))
    assert rate_check.read_rate_check(tmp_path)["passed"] is True
    (tmp_path / "rate_check.json").write_text("{not json")
    assert rate_check.read_rate_check(tmp_path) is None


def test_a_clock_that_did_not_advance_is_a_failure_not_a_division_by_zero():
    report = rate_check.judge(GOOD, 0.0)
    assert not report["passed"] and "clock" in report["problems"][0]
