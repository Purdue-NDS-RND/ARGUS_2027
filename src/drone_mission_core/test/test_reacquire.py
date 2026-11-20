"""Smart re-acquisition (CEN-06): how high to climb to look for a lost target again. No ROS."""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_mission_core import reacquire as rq  # noqa: E402
from drone_mission_core.features import FLAG_REQUIRES  # noqa: E402


def test_the_numbers_are_the_specs():
    assert (rq.SIGMAS, rq.MARGIN_M, rq.HOLD_S, rq.HALF_FOOTPRINT_PER_M) == (3.0, 2.5, 10.0, 0.463)
    assert rq.REACQUIRE_FLAG == "smart_reacquire"
    # The track that was lost, and where it was, come from the centring on the estimator.
    assert FLAG_REQUIRES[rq.REACQUIRE_FLAG] == ("use_target_estimator", "centre_on_estimator")


def test_the_height_is_where_the_footprint_covers_three_sigma_and_the_margin():
    # h = (3 sigma + 2.5) / 0.463
    assert rq.reacquire_height(0.10, floor_m=0.0, ceiling_m=40.0) == pytest.approx((0.3 + 2.5) / 0.463)
    assert rq.reacquire_height(0.0, floor_m=0.0, ceiling_m=40.0) == pytest.approx(2.5 / 0.463)
    assert rq.reacquire_height(1.0, floor_m=0.0, ceiling_m=40.0) == pytest.approx(5.5 / 0.463)
    # Another camera mode's footprint.
    assert rq.reacquire_height(0.10, floor_m=0.0, ceiling_m=40.0, half_footprint_per_m=0.7) == pytest.approx(2.8 / 0.7)


def test_the_vehicle_never_comes_down_to_look_and_never_goes_over_the_ceiling():
    assert rq.reacquire_height(0.10, floor_m=15.0, ceiling_m=40.0) == 15.0  # already higher: it stays
    assert rq.reacquire_height(8.0, floor_m=3.0, ceiling_m=40.0) == 40.0  # (24 + 2.5) / 0.463 = 57 m
    assert rq.reacquire_height(0.10, floor_m=45.0, ceiling_m=40.0) == 45.0  # above the ceiling already: no climb, and no descent either


@pytest.mark.parametrize("sigma", [None, float("nan"), float("inf"), -0.1, "wide"])
def test_a_sigma_that_is_not_one_is_refused(sigma):
    with pytest.raises(ValueError, match="sigma"):
        rq.reacquire_height(sigma, floor_m=3.0, ceiling_m=40.0)


def test_half_the_footprint_comes_from_the_calibrated_camera_on_its_short_side():
    sim = SimpleNamespace(width=640, height=360, fx=389.165, fy=389.165)
    assert rq.half_footprint_per_m(sim) == pytest.approx(180.0 / 389.165)  # 0.4625: the spec's 0.463
    tall = SimpleNamespace(width=360, height=640, fx=389.165, fy=389.165)
    assert rq.half_footprint_per_m(tall) == pytest.approx(180.0 / 389.165)
    assert rq.half_footprint_per_m(None) == 0.463  # no calibrated camera: the spec's number
    for bad in (SimpleNamespace(width=640, height=360, fx=0.0, fy=389.0), SimpleNamespace(width=640)):
        assert rq.half_footprint_per_m(bad) == 0.463


def test_the_climb_is_given_time_in_proportion_to_its_height():
    assert rq.climb_limit_s(0.0) == rq.CLIMB_BASE_S
    assert rq.climb_limit_s(10.0) == pytest.approx(rq.CLIMB_BASE_S + 10.0 / rq.SLOWEST_CLIMB_MPS)
    assert rq.climb_limit_s(-3.0) == rq.CLIMB_BASE_S and math.isfinite(rq.climb_limit_s(float("nan")))


def test_the_vehicle_moves_at_most_the_margin_toward_the_estimate_and_looks_from_higher_for_the_rest():
    """A lost target is a reason to climb, not to fly off sideways: the look point is the estimate when it is within
    2.5 m, and 2.5 m toward it otherwise, with what is left added to what the footprint has to cover."""
    assert rq.MAX_SHIFT_M == rq.MARGIN_M == 2.5
    point, left = rq.look_point((0.0, 0.0), (1.5, 2.0))  # 2.5 m away: over it
    assert point == pytest.approx((1.5, 2.0)) and left == 0.0
    point, left = rq.look_point((0.0, 0.0), (6.0, 8.0))  # 10 m away
    assert point == pytest.approx((1.5, 2.0)) and left == pytest.approx(7.5)
    point, left = rq.look_point((3.0, 3.0), (3.0, 3.0))
    assert point == (3.0, 3.0) and left == 0.0
    assert rq.reacquire_height(0.10, floor_m=0.0, ceiling_m=40.0, offset_m=7.5) == pytest.approx((0.3 + 2.5 + 7.5) / 0.463)
    with pytest.raises(ValueError, match="offset"):
        rq.reacquire_height(0.10, floor_m=0.0, ceiling_m=40.0, offset_m=float("nan"))
