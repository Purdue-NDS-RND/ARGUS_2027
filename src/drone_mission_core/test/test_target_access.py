"""The rules for acting on a track and the small conversions behind MissionContext's new calls (FND-07). No ROS."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from drone_mission_core import target_access as ta
from drone_mission_core.target_access import PrimaryTarget


def target(state="CONFIRMED", age_s=0.1, class_id="bullseye"):
    return PrimaryTarget(class_id=class_id, track_id=3, state=state, position=(1.0, 2.0, 0.0), covariance=((0.01, 0.0), (0.0, 0.01)), sigma_m=0.1, age_s=age_s, for_ns=5)


def test_the_limits_are_the_specs():
    assert ta.DESCEND_MAX_AGE_S == 0.5 and ta.RELEASE_MAX_AGE_S == 0.3


def test_descending_needs_a_confirmed_track_no_older_than_half_a_second():
    assert ta.descent_permission(target(age_s=0.5))
    late = ta.descent_permission(target(age_s=0.51))
    assert not late and late.reason == "the bullseye track is 0.51 s old (limit 0.5 s to descend)"
    for state in ("TENTATIVE", "COASTING", "LOST"):
        refused = ta.descent_permission(target(state=state, age_s=0.0))
        assert not refused and refused.reason == f"the bullseye track is {state}, not CONFIRMED"
    none = ta.descent_permission(None)
    assert not none and none.reason == "no track to descend on"


def test_releasing_needs_a_confirmed_track_no_older_than_0_3_s():
    assert ta.release_permission(target(age_s=0.3))
    late = ta.release_permission(target(age_s=0.31))
    assert not late and late.reason == "the bullseye track is 0.31 s old (limit 0.3 s to release)"
    assert ta.descent_permission(target(age_s=0.31))  # old enough to stop a release, not a descent
    assert not ta.release_permission(target(state="COASTING", age_s=0.0)) and not ta.release_permission(None)


@pytest.mark.parametrize("age_s", [math.nan, math.inf, -1.0])
def test_an_age_that_is_not_a_time_since_the_last_hit_permits_nothing(age_s):
    assert not ta.descent_permission(target(age_s=age_s)) and not ta.release_permission(target(age_s=age_s))


def test_a_hit_stamped_a_few_milliseconds_ahead_of_the_clock_is_fresh_not_refused():
    assert ta.descent_permission(target(age_s=-0.004)) and ta.release_permission(target(age_s=-0.004))


def test_a_permission_reads_as_true_or_false_and_says_why_not():
    assert bool(ta.Permission(True)) is True and ta.Permission(True).reason == ""
    assert bool(ta.Permission(False, "because")) is False


def test_a_permission_names_the_track_it_is_about():
    assert ta.descent_permission(target()).track_id == 3 and ta.release_permission(target(state="COASTING")).track_id == 3
    assert ta.descent_permission(None).track_id is None


def test_within_is_about_distance_in_time_either_way():
    second = 1_000_000_000
    assert ta.within(10 * second, 10 * second + second // 2, 0.5) and not ta.within(10 * second, 10 * second + second // 2 + 1, 0.5)
    assert not ta.within(12 * second, 10 * second, 0.5)  # stamped well ahead of the clock: not usable either
    assert not ta.within(None, 10 * second, 0.5)
    assert (ta.POSE_MAX_AGE_S, ta.FIX_MAX_AGE_S, ta.FIX_POSE_MAX_APART_S) == (0.5, 2.0, 0.25)


def test_the_missions_view_of_an_estimate():
    estimate = SimpleNamespace(
        track_id=7, class_id="bucket", state="COASTING", east_m=1.5, north_m=-2.0, up_m=0.2, covariance=((0.04, 0.01), (0.01, 0.09)), sigma_m=0.31, age_s=0.4, hits=9, score=0.8, last_hit_ns=1, for_ns=99
    )
    view = ta.from_estimate(estimate)
    assert view == PrimaryTarget("bucket", 7, "COASTING", (1.5, -2.0, 0.2), ((0.04, 0.01), (0.01, 0.09)), 0.31, 0.4, 99)


def test_height_above_ground_says_where_it_came_from():
    assert ta.height_agl(12.0, (1.5, "rangefinder")) == ta.HeightAgl(10.5, "rangefinder")
    assert ta.height_agl(12.0, (0.0, "home")) == ta.HeightAgl(12.0, "baro")  # no healthy rangefinder: height above home
    assert ta.height_agl(12.0, None) == ta.HeightAgl(12.0, "baro")
    assert ta.height_agl(12.0, (math.nan, "rangefinder")) == ta.HeightAgl(12.0, "baro")
    assert ta.height_agl(math.nan, None) is None
    assert ta.height_agl(12.0, None, pose_age_s=0.2).pose_age_s == 0.2


def test_a_latitude_and_longitude_in_the_local_frame():
    here = (40.0, -86.0)
    # 100 m north and 50 m east of the vehicle, which is at (20, -30) in the local frame.
    north_deg = 100.0 / ta.METRES_PER_DEGREE
    east_deg = 50.0 / (ta.METRES_PER_DEGREE * math.cos(math.radians(40.0 + north_deg / 2.0)))
    east, north = ta.latlon_to_local_enu(here[0] + north_deg, here[1] + east_deg, here[0], here[1], 20.0, -30.0)
    assert (east, north) == pytest.approx((70.0, 70.0), abs=1e-6)
    assert ta.latlon_to_local_enu(here[0], here[1], here[0], here[1], 20.0, -30.0) == pytest.approx((20.0, -30.0))
    # The scale is ArduPilot's own, so the answer agrees with the frame its local position is in.
    assert ta.METRES_PER_DEGREE == pytest.approx(0.011131884502145034 * 1.0e7)


def test_longitude_goes_the_short_way_round_and_bad_numbers_give_nothing():
    east, _north = ta.latlon_to_local_enu(0.0, -179.9995, 0.0, 179.9995, 0.0, 0.0)
    assert east == pytest.approx(0.001 * ta.METRES_PER_DEGREE)
    assert ta.latlon_to_local_enu(math.nan, 0.0, 0.0, 0.0, 0.0, 0.0) is None
    assert ta.latlon_to_local_enu(91.0, 0.0, 0.0, 0.0, 0.0, 0.0) is None
    assert ta.latlon_to_local_enu(0.0, 181.0, 0.0, 0.0, 0.0, 0.0) is None
    assert ta.latlon_to_local_enu(None, 0.0, 0.0, 0.0, 0.0, 0.0) is None


def test_the_module_has_no_ros_and_no_numpy_in_it():
    tree = ast.parse(Path(ta.__file__).read_text(encoding="utf-8"))
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {alias.name.split(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom):
            imported.add((node.module or "").split(".")[0])
    assert imported <= {"__future__", "dataclasses", "math", "typing"}
