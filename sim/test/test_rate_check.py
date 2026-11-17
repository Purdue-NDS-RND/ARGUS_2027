"""The MAVROS rate check's verdict (PLT-06). No ROS."""

import sys
from pathlib import Path

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
