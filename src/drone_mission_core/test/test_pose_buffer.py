"""The pose history buffer (FND-04). No ROS.

Spec: keep 5 s of pose and velocity; pose_at(t) interpolates position
linearly and orientation by SLERP between the bracketing samples; None if
the bracketing gap exceeds 0.12 s; at most 30 ms of extrapolation past the
newest sample, using velocity; never the latest pose as a fallback.
"""

import ast
import math
from pathlib import Path

import pytest

from drone_mission_core import pose_buffer
from drone_mission_core.pose_buffer import PoseBuffer

S = 1_000_000_000
MS = 1_000_000


def yaw_quaternion(yaw_deg: float):
    half = math.radians(yaw_deg) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def angle_between_deg(q1, q2) -> float:
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def truth(t_s: float):
    """1 m/s east, 0.5 m/s north, climbing 0.2 m/s, yawing 30 deg/s."""
    return (1.0 * t_s, 0.5 * t_s, 10.0 + 0.2 * t_s), yaw_quaternion(30.0 * t_s)


def flying(rate_hz: float = 30.0, seconds: float = 3.0, start_s: float = 100.0) -> PoseBuffer:
    buffer = PoseBuffer()
    for index in range(int(seconds * rate_hz) + 1):
        t = index / rate_hz
        position, orientation = truth(t)
        stamp = int(round((start_s + t) * S))
        buffer.add_pose(stamp, position, orientation)
        buffer.add_velocity(stamp, (1.0, 0.5, 0.2))
    return buffer


def test_the_limits_are_the_ones_in_the_spec():
    assert pose_buffer.HISTORY_S == 5.0
    assert pose_buffer.MAX_GAP_S == 0.12
    assert pose_buffer.MAX_EXTRAPOLATION_S == 0.030


@pytest.mark.parametrize("rate_hz", [30.0, 20.0, 10.0])
def test_a_1_mps_and_30_deg_per_s_flight_is_rebuilt_to_1_cm_and_0_2_deg(rate_hz):
    buffer = flying(rate_hz)
    worst_m, worst_deg = 0.0, 0.0
    for step in range(0, 2900, 7):  # every 7 ms, so between samples at every phase
        t = step / 1000.0
        pose = buffer.pose_at(int(round((100.0 + t) * S)))
        assert pose is not None, t
        position, orientation = truth(t)
        worst_m = max(worst_m, math.dist(pose.position, position))
        worst_deg = max(worst_deg, angle_between_deg(pose.orientation, orientation))
        assert not pose.extrapolated
    assert worst_m <= 0.01 and worst_deg <= 0.2
    assert worst_m < 1e-6  # the motion is linear, so it is in fact exact


def test_a_stamp_that_is_a_sample_returns_that_sample():
    buffer = flying()
    pose = buffer.pose_at(101 * S)
    position, orientation = truth(1.0)
    assert pose.position == pytest.approx(position) and pose.orientation == pytest.approx(orientation)
    assert pose.stamp_ns == 101 * S


def test_orientation_is_slerp_the_short_way_round_and_stays_unit_length():
    buffer = PoseBuffer()
    buffer.add_pose(0, (0.0, 0.0, 0.0), yaw_quaternion(170.0))
    flipped = tuple(-value for value in yaw_quaternion(-170.0))  # the same attitude, written with the other sign
    buffer.add_pose(100 * MS, (0.0, 0.0, 0.0), flipped)
    pose = buffer.pose_at(50 * MS)
    assert angle_between_deg(pose.orientation, yaw_quaternion(180.0)) < 1e-6  # through 180, not back through 0
    assert math.sqrt(sum(value * value for value in pose.orientation)) == pytest.approx(1.0)


def test_a_quaternion_that_is_not_unit_length_is_normalised_and_a_zero_one_is_refused():
    buffer = PoseBuffer()
    assert buffer.add_pose(0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 2.0))
    assert buffer.pose_at(0).orientation == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert not buffer.add_pose(10 * MS, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
    assert not buffer.add_pose(20 * MS, (math.nan, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert len(buffer) == 1


# --- refusals -------------------------------------------------------------------------------------------


def test_a_gap_of_more_than_0_12_s_between_the_bracketing_samples_is_refused():
    buffer = PoseBuffer()
    for stamp_ms in (0, 50, 100, 230, 280):  # 130 ms missing between 100 and 230
        buffer.add_pose(stamp_ms * MS, (stamp_ms / 1000.0, 0.0, 0.0), yaw_quaternion(0.0))
    assert buffer.pose_at(75 * MS) is not None
    assert buffer.pose_at(150 * MS) is None
    assert buffer.pose_at(101 * MS) is None and buffer.pose_at(229 * MS) is None
    assert buffer.pose_at(100 * MS) is not None and buffer.pose_at(230 * MS) is not None  # the samples themselves
    assert buffer.pose_at(255 * MS) is not None


def test_a_gap_of_exactly_0_12_s_is_still_interpolated():
    buffer = PoseBuffer()
    buffer.add_pose(0, (0.0, 0.0, 0.0), yaw_quaternion(0.0))
    buffer.add_pose(120 * MS, (0.12, 0.0, 0.0), yaw_quaternion(0.0))
    assert buffer.pose_at(60 * MS).position == pytest.approx((0.06, 0.0, 0.0))
    buffer.add_pose(241 * MS, (0.241, 0.0, 0.0), yaw_quaternion(0.0))
    assert buffer.pose_at(180 * MS) is None


def test_before_the_oldest_sample_or_with_nothing_stored_there_is_no_pose():
    assert PoseBuffer().pose_at(5 * S) is None
    buffer = flying()
    assert buffer.pose_at(100 * S - 1) is None
    assert buffer.pose_at(99 * S) is None


def test_extrapolation_goes_30_ms_past_the_newest_sample_and_no_further():
    buffer = flying(seconds=3.0)  # newest sample at 103.0 s
    newest = 103 * S
    for ahead_ms in (1, 10, 29, 30):
        pose = buffer.pose_at(newest + ahead_ms * MS)
        assert pose is not None and pose.extrapolated, ahead_ms
        position, orientation = truth(3.0 + ahead_ms / 1000.0)
        assert math.dist(pose.position, position) <= 0.01
        assert angle_between_deg(pose.orientation, orientation) <= 0.2
    assert buffer.pose_at(newest + 31 * MS) is None
    assert buffer.pose_at(newest + 500 * MS) is None


def test_extrapolation_uses_the_reported_velocity_not_the_last_two_positions():
    buffer = PoseBuffer()
    buffer.add_pose(0, (0.0, 0.0, 0.0), yaw_quaternion(0.0))
    buffer.add_pose(50 * MS, (0.0, 0.0, 0.0), yaw_quaternion(0.0))  # the positions say it is standing still
    buffer.add_velocity(50 * MS, (2.0, -1.0, 0.5))
    pose = buffer.pose_at(70 * MS)
    assert pose.position == pytest.approx((0.04, -0.02, 0.01))


def test_extrapolation_never_falls_back_to_the_latest_pose():
    # No velocity: refuse.
    buffer = PoseBuffer()
    buffer.add_pose(0, (1.0, 2.0, 3.0), yaw_quaternion(0.0))
    buffer.add_pose(50 * MS, (1.0, 2.0, 3.0), yaw_quaternion(0.0))
    assert buffer.pose_at(60 * MS) is None
    # A velocity that is too old to be the newest pose's: refuse.
    buffer.add_velocity(0, (1.0, 0.0, 0.0))
    buffer.add_pose(200 * MS, (1.0, 2.0, 3.0), yaw_quaternion(0.0))
    buffer.add_pose(250 * MS, (1.0, 2.0, 3.0), yaw_quaternion(0.0))
    assert buffer.pose_at(260 * MS) is None
    # One pose only: the turn rate is unknown, so refuse.
    lonely = PoseBuffer()
    lonely.add_pose(0, (1.0, 2.0, 3.0), yaw_quaternion(0.0))
    lonely.add_velocity(0, (1.0, 0.0, 0.0))
    assert lonely.pose_at(10 * MS) is None
    # The last two poses too far apart: refuse.
    sparse = PoseBuffer()
    sparse.add_pose(0, (0.0, 0.0, 0.0), yaw_quaternion(0.0))
    sparse.add_pose(200 * MS, (0.0, 0.0, 0.0), yaw_quaternion(0.0))
    sparse.add_velocity(200 * MS, (1.0, 0.0, 0.0))
    assert sparse.pose_at(210 * MS) is None


# --- history ----------------------------------------------------------------------------------------------


def test_five_seconds_are_kept_and_older_samples_are_dropped():
    buffer = flying(rate_hz=30.0, seconds=8.0)  # 100.0 to 108.0 s
    assert buffer.pose_at(int(103.5 * S)) is not None
    assert buffer.pose_at(int(102.9 * S)) is None  # more than 5 s before the newest
    assert buffer.oldest_ns() >= 103 * S and buffer.newest_ns() == 108 * S
    assert 150 <= len(buffer) <= 152


def test_samples_may_arrive_out_of_order_and_a_repeated_stamp_is_ignored():
    buffer = PoseBuffer()
    for stamp_ms in (0, 100, 50):
        assert buffer.add_pose(stamp_ms * MS, (stamp_ms / 1000.0, 0.0, 0.0), yaw_quaternion(0.0))
    assert not buffer.add_pose(50 * MS, (9.0, 9.0, 9.0), yaw_quaternion(0.0))
    assert buffer.pose_at(25 * MS).position == pytest.approx((0.025, 0.0, 0.0))
    assert buffer.pose_at(75 * MS).position == pytest.approx((0.075, 0.0, 0.0))
    assert len(buffer) == 3


def test_a_stamp_far_in_the_past_does_not_wipe_the_history():
    buffer = flying()
    assert not buffer.add_pose(1 * S, (0.0, 0.0, 0.0), yaw_quaternion(0.0))  # older than the 5 s window
    assert len(buffer) == 91 and buffer.pose_at(101 * S) is not None


def test_velocity_is_kept_and_interpolated_under_the_same_gap_rule():
    buffer = PoseBuffer()
    buffer.add_velocity(0, (0.0, 0.0, 0.0))
    buffer.add_velocity(100 * MS, (1.0, 2.0, -1.0))
    assert buffer.velocity_at(50 * MS) == pytest.approx((0.5, 1.0, -0.5))
    assert buffer.velocity_at(100 * MS) == pytest.approx((1.0, 2.0, -1.0))
    assert buffer.velocity_at(101 * MS) is None and buffer.velocity_at(-1) is None
    buffer.add_velocity(300 * MS, (1.0, 2.0, -1.0))
    assert buffer.velocity_at(200 * MS) is None


def test_stamps_are_whole_nanoseconds():
    buffer = PoseBuffer()
    assert not buffer.add_pose(0.5, (0.0, 0.0, 0.0), yaw_quaternion(0.0))
    assert not buffer.add_velocity(None, (0.0, 0.0, 0.0))
    assert buffer.pose_at(0.5) is None and len(buffer) == 0


def test_the_module_has_no_ros_in_it():
    tree = ast.parse(Path(pose_buffer.__file__).read_text(encoding="utf-8"))
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {alias.name.split(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom):
            imported.add((node.module or "").split(".")[0])
    assert imported <= {"__future__", "bisect", "dataclasses", "math", "threading", "typing"}
