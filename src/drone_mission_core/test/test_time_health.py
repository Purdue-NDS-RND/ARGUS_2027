"""The stale-data guard's health logic (FND-03). No ROS.

The rule under test, from the spec: UNHEALTHY when pose age exceeds 100 ms,
frame age exceeds 250 ms, or the time-sync offset jumps by more than 10 ms.
"""

import ast
import math
from pathlib import Path

import pytest

from drone_mission_core import time_health
from drone_mission_core.time_health import TimeHealth

MS = 1.0e-3
OFFSET_NS = 5_000_000


def ns(seconds: float, offset_ns: int = OFFSET_NS) -> int:
    """A synchronised pose stamp at about `seconds`: whole milliseconds, as the flight controller counts, plus the offset."""
    return int(round(seconds * 1000.0)) * 1_000_000 + offset_ns % 1_000_000


def fresh(now_s: float = 100.0, **limits) -> TimeHealth:
    """A monitor with pose, frame and time sync all current at now_s."""
    health = TimeHealth(**limits)
    health.on_timesync(now_s - 0.2, 5_000_000, 2.0)
    health.on_timesync(now_s - 0.1, 5_000_000, 2.0)
    health.on_pose(ns(now_s - 0.02))
    health.on_frame(now_s - 0.05)
    return health


def test_the_limits_are_the_ones_in_the_spec():
    assert time_health.POSE_AGE_LIMIT_S == 0.100
    assert time_health.FRAME_AGE_LIMIT_S == 0.250
    assert time_health.OFFSET_JUMP_LIMIT_S == 0.010


def test_the_limits_the_spec_does_not_give_are_the_recorded_ones():
    # docs/build/PROGRESS.md, Decisions, FND-03.
    assert time_health.OFFSET_SETTLE_S == 1.0
    assert time_health.TIMESYNC_SILENCE_LIMIT_S == 2.0
    assert time_health.SYNC_PROOF_LIMIT_S == 0.5


def test_fresh_data_is_healthy_and_the_report_carries_the_numbers():
    report = fresh().report(100.0)
    assert report.healthy and report.reasons == ()
    assert report.pose_age_s == pytest.approx(0.02)
    assert report.frame_age_s == pytest.approx(0.05)
    assert report.timesync_age_s == pytest.approx(0.1)
    assert report.offset_ms == pytest.approx(5.0)
    assert report.round_trip_ms == pytest.approx(2.0)
    assert report.last_offset_step_ms == pytest.approx(0.0)


@pytest.mark.parametrize("age_ms, healthy", [(50.0, True), (99.0, True), (101.0, False), (500.0, False)])
def test_pose_older_than_100_ms_is_unhealthy(age_ms, healthy):
    health = fresh()
    health.on_pose(ns(100.0 - age_ms * MS))
    report = health.report(100.0)
    assert report.healthy is healthy
    assert any("pose" in reason for reason in report.reasons) is (not healthy)


@pytest.mark.parametrize("age_ms, healthy", [(249.0, True), (250.0, True), (251.0, False), (2000.0, False)])
def test_frame_older_than_250_ms_is_unhealthy(age_ms, healthy):
    health = fresh()
    health.on_frame(100.0 - age_ms * MS)
    report = health.report(100.0)
    assert report.healthy is healthy
    assert any("frame" in reason for reason in report.reasons) is (not healthy)


def test_data_goes_stale_as_time_passes_without_new_messages():
    health = fresh()
    assert health.report(100.0).healthy
    assert not health.report(100.2).healthy  # the pose is now 220 ms old
    health.on_pose(ns(100.19))
    health.on_frame(100.19)
    health.on_timesync(100.19, 5_000_000, 2.0)
    assert health.report(100.2).healthy


@pytest.mark.parametrize("step_ms, healthy", [(0.0, True), (9.9, True), (-9.9, True), (10.1, False), (-10.1, False), (5000.0, False)])
def test_an_offset_step_of_more_than_10_ms_is_unhealthy(step_ms, healthy):
    health = fresh()
    health.on_timesync(100.0, 5_000_000 + int(step_ms * 1_000_000), 2.0)
    report = health.report(100.0)
    assert report.healthy is healthy
    assert report.last_offset_step_ms == pytest.approx(step_ms)
    assert any("offset" in reason for reason in report.reasons) is (not healthy)


def test_slow_drift_is_not_a_jump():
    # The sim's flight controller clock drifts about 0.2 ms per second against /clock.
    health = fresh()
    offset = 5_000_000
    for step in range(1, 200):
        offset += 20_000
        now = 100.0 + 0.1 * step
        health.on_timesync(now, offset, 2.0)
        health.on_pose(ns(now, offset))
        health.on_frame(now)
        assert health.report(now).healthy


def test_a_jump_stays_unhealthy_until_the_offset_has_been_steady_for_the_settle_time():
    health = fresh()
    health.on_timesync(100.0, 55_000_000, 2.0)  # +50 ms
    for tenth in range(1, 10):
        now = 100.0 + 0.1 * tenth
        health.on_timesync(now, 55_000_000, 2.0)
        health.on_pose(ns(now))
        health.on_frame(now)
        assert not health.report(now).healthy, f"healthy again only {tenth / 10} s after the jump"
    health.on_timesync(101.0, 55_000_000, 2.0)
    health.on_pose(ns(101.0))
    health.on_frame(101.0)
    assert health.report(101.0 + time_health.OFFSET_SETTLE_S - 1.0 + 0.001).healthy


def test_a_second_jump_restarts_the_settle_time():
    health = fresh()
    health.on_timesync(100.0, 55_000_000, 2.0)
    health.on_timesync(100.8, 5_000_000, 2.0)  # and back
    health.on_pose(ns(101.3))
    health.on_frame(101.3)
    health.on_timesync(101.3, 5_000_000, 2.0)
    assert not health.report(101.3).healthy  # 1.3 s after the first jump, 0.5 s after the second
    health.on_pose(ns(101.85))
    health.on_frame(101.85)
    health.on_timesync(101.85, 5_000_000, 2.0)
    assert health.report(101.85).healthy


def test_nothing_heard_yet_is_unhealthy_and_says_what_is_missing():
    report = TimeHealth().report(100.0)
    assert not report.healthy
    assert len(report.reasons) == 3
    assert report.pose_age_s is None and report.frame_age_s is None and report.timesync_age_s is None
    for missing in ("pose", "frame", "time sync"):
        assert any(missing in reason for reason in report.reasons)


def test_each_missing_input_alone_is_unhealthy():
    for skip in ("pose", "frame", "timesync"):
        health = TimeHealth()
        if skip != "pose":
            health.on_pose(ns(99.99))
        if skip != "frame":
            health.on_frame(99.99)
        if skip != "timesync":
            health.on_timesync(99.99, 5_000_000, 2.0)
        assert not health.report(100.0).healthy, skip


def test_time_sync_that_went_silent_is_unhealthy():
    # Without time sync MAVROS stamps a pose when it arrives, so the pose-age check sees nothing.
    health = fresh()
    for tenth in range(1, 40):
        now = 100.0 + 0.1 * tenth
        health.on_pose(ns(now))
        health.on_frame(now)
        silent_s = now - 99.9
        if abs(silent_s - time_health.TIMESYNC_SILENCE_LIMIT_S) > 0.05:  # not on the boundary itself
            assert health.report(now).healthy is (silent_s < time_health.TIMESYNC_SILENCE_LIMIT_S), now


def test_a_stamp_from_the_future_is_as_bad_as_a_stale_one():
    health = fresh()
    health.on_pose(ns(100.0 + 0.150))
    assert not health.report(100.0).healthy
    health = fresh()
    health.on_frame(100.0 + 0.300)
    assert not health.report(100.0).healthy
    health = fresh()
    health.on_pose(ns(100.0 + 0.005))  # a few ms ahead is clock granularity, not a fault
    report = health.report(100.0)
    assert report.healthy and report.pose_age_s == pytest.approx(-0.005)


def test_pose_stamps_that_are_arrival_times_are_unhealthy():
    # Before its filter has converged MAVROS stamps a pose with the ROS clock on arrival.
    health = TimeHealth()
    health.on_timesync(99.9, OFFSET_NS, 2.0)
    health.on_frame(99.99)
    health.on_pose(99_990_123_456)  # not whole milliseconds plus the offset
    report = health.report(100.0)
    assert not report.healthy and report.pose_age_s == pytest.approx(0.009876544)
    assert report.reasons == ("pose stamps are arrival times, not flight-controller time (MAVROS has not applied its time sync)",)
    health.on_pose(ns(99.995))
    assert health.report(100.0).healthy


def test_an_offset_of_zero_or_less_is_never_applied_so_it_proves_nothing():
    health = TimeHealth()
    health.on_timesync(99.9, 0, 2.0)
    health.on_frame(99.99)
    health.on_pose(99_990_000_000)
    assert any("arrival times" in reason for reason in health.report(100.0).reasons)


def test_a_pose_and_its_offset_may_arrive_in_either_order():
    health = TimeHealth()
    health.on_frame(99.99)
    health.on_timesync(99.8, 7_000_300, 2.0)
    health.on_pose(ns(99.99, 7_000_900))  # stamped with an offset the monitor has not heard yet
    assert not health.report(100.0).healthy
    health.on_timesync(99.995, 7_000_900, 2.0)
    assert health.report(100.0).healthy


def test_synchronised_stamps_must_keep_coming():
    health = fresh()
    for step in range(1, 12):
        now = 100.0 + 0.1 * step
        health.on_timesync(now, OFFSET_NS, 2.0)
        health.on_frame(now)
        health.on_pose(int(round(now * 1000.0)) * 1_000_000 + 123_456)  # recent, but arrival times
        report = health.report(now)
        if now - 99.98 > time_health.SYNC_PROOF_LIMIT_S + 0.05:
            assert not report.healthy and any("arrival times" in reason for reason in report.reasons), now
        elif now - 99.98 < time_health.SYNC_PROOF_LIMIT_S - 0.05:
            assert report.healthy, now


def test_a_sync_proof_from_before_a_backward_clock_step_does_not_count():
    health = fresh(now_s=100.0)
    # The clock stepped back 30 s; new poses are arrival times, recent on the new clock.
    health.on_timesync(70.0, OFFSET_NS, 2.0)
    health.on_frame(70.0)
    health.on_pose(70_000_123_456)
    report = health.report(70.01)
    assert not report.healthy and any("arrival times" in reason for reason in report.reasons)


def test_is_synced_is_whole_milliseconds_plus_a_recent_offset():
    offset = 43_720_117
    stamp = 184_701 * 1_000_000 + offset
    assert time_health.is_synced(stamp, [offset])
    assert time_health.is_synced(stamp, [43_707_000, offset, 43_733_500])
    assert not time_health.is_synced(stamp + 1, [offset])
    assert not time_health.is_synced(stamp, [])
    assert not time_health.is_synced(184_701_000_000, [0]) and not time_health.is_synced(184_696_000_000, [-5_000_000])
    wall = 1_794_949_916_123_456_789  # wall-clock size: the arithmetic is exact in whole nanoseconds
    assert time_health.is_synced(184_701 * 1_000_000 + wall, [wall]) and not time_health.is_synced(184_701 * 1_000_000 + wall + 3, [wall])


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf, None, "soon", 99.99, True])
def test_a_stamp_that_is_not_a_number_does_not_count_as_data(bad):
    health = TimeHealth()
    health.on_pose(bad)
    health.on_frame(bad)
    if not isinstance(bad, float) or not math.isfinite(bad):
        health.on_timesync(bad, 5_000_000, 2.0)
    if isinstance(bad, (bool, float)) and (bad is True or math.isfinite(bad)):
        return  # a finite float or a bool is only wrong as a pose stamp, which is whole nanoseconds
    report = health.report(100.0)
    assert not report.healthy and report.pose_age_s is None and report.frame_age_s is None and report.timesync_age_s is None
    # and it does not wipe good data already held
    health = fresh()
    health.on_pose(bad)
    health.on_frame(bad)
    assert health.report(100.0).healthy


def test_a_pose_stamp_must_be_whole_nanoseconds():
    health = TimeHealth()
    health.on_pose(99.99)
    health.on_pose(True)
    assert health.report(100.0).pose_age_s is None


def test_a_clock_that_is_not_a_number_is_unhealthy():
    assert not fresh().report(math.nan).healthy


def test_limits_can_be_set_for_a_test_but_default_to_the_spec():
    health = fresh(pose_age_limit_s=0.01)
    assert not health.report(100.0).healthy  # the pose is 20 ms old
    assert fresh().report(100.0).healthy


def test_the_module_has_no_ros_in_it():
    tree = ast.parse(Path(time_health.__file__).read_text(encoding="utf-8"))
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {alias.name.split(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom):
            imported.add((node.module or "").split(".")[0])
    assert imported <= {"__future__", "collections", "dataclasses", "math", "typing"}
