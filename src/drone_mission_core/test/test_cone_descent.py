"""The cone descent onto the bucket's handle (REC-3): how much sideways error each height allows, and what to do about it."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_mission_core import cone_descent as cd  # noqa: E402
from drone_mission_core.cone_descent import ABORT, CLIMB, DESCEND, HOLD, LOST  # noqa: E402

CAPTURE = 0.10  # the mechanism's capture half-width (the sim's; the mechanical team's replaces it)


def cone(**settings):
    return cd.ConeDescent(capture_half_width_m=CAPTURE, **settings)


def test_the_cone_is_the_specs_from_three_metres_down_to_the_handle():
    allowed = lambda height: cd.allowed_error_m(height, CAPTURE)  # noqa: E731
    assert allowed(10.0) == allowed(3.0) == 1.0  # 1.0 m at 3 m and above
    assert allowed(0.8) == pytest.approx(0.48)
    assert allowed(1.9) == pytest.approx((1.0 + 0.48) / 2.0)  # narrowing evenly between the two
    assert allowed(0.4) == allowed(0.2) == allowed(0.0) == pytest.approx(0.8 * CAPTURE)  # 0.8 of the capture half-width below 0.4 m
    assert allowed(0.6) == pytest.approx((0.48 + 0.08) / 2.0)
    heights = [0.05 * index for index in range(0, 81)]
    assert all(allowed(a) <= allowed(b) + 1.0e-12 for a, b in zip(heights, heights[1:]))  # never wider lower down
    assert allowed(-0.2) == pytest.approx(0.08)  # the hook past the handle: still the mechanism's own width
    with pytest.raises(ValueError, match="capture half-width"):
        cd.allowed_error_m(1.0, 0.0)
    with pytest.raises(ValueError, match="capture half-width"):
        cd.ConeDescent(capture_half_width_m=0.7)  # 0.8 of it would be wider than the cone at 0.8 m: not a mechanism this cone is for


def test_the_descent_rate_is_from_half_a_metre_a_second_down_to_a_tenth_slower_when_low():
    assert cd.descent_rate_mps(5.0) == cd.descent_rate_mps(3.0) == 0.5
    assert cd.descent_rate_mps(0.8) == cd.descent_rate_mps(0.3) == cd.descent_rate_mps(0.0) == 0.1
    assert cd.descent_rate_mps(1.9) == pytest.approx(0.3)
    assert (cd.MAX_RATE_MPS, cd.MIN_RATE_MPS) == (0.5, 0.1)


def test_inside_the_cone_it_descends_up_to_half_as_far_again_it_holds_and_further_out_it_climbs():
    guide = cone()
    inside = guide.update(hook_height_m=2.0, error_m=0.5, detection_age_s=0.05, now_s=10.0)
    assert inside.action == DESCEND and inside.rate_mps == pytest.approx(cd.descent_rate_mps(2.0)) and inside.cone_m == pytest.approx(cd.allowed_error_m(2.0, CAPTURE))
    on_the_line = guide.update(2.0, inside.cone_m, 0.05, 10.1)
    assert on_the_line.action == DESCEND
    holding = guide.update(2.0, 1.2 * inside.cone_m, 0.05, 10.2)
    assert holding.action == HOLD and holding.rate_mps == 0.0 and "outside the cone" in holding.reason
    edge = guide.update(2.0, 1.5 * inside.cone_m, 0.05, 10.3)
    assert edge.action == HOLD
    far = guide.update(2.0, 1.6 * inside.cone_m, 0.05, 10.4)
    assert far.action == CLIMB and far.rate_mps == cd.CLIMB_RATE_MPS > 0.0 and "climbing" in far.reason
    assert (cd.HOLD_OUT_TO, cd.CLIMB_RATE_MPS) == (1.5, 0.5)
    # A negative error is nonsense, and no error is nothing to descend on.
    with pytest.raises(ValueError):
        guide.update(2.0, -0.1, 0.05, 10.5)
    blind = cone().update(2.0, None, None, 10.6)
    assert blind.action == HOLD and "no error" in blind.reason


def test_an_error_that_does_not_shrink_by_a_fifth_in_three_seconds_ends_the_descent():
    guide = cone()
    start = guide.update(2.0, 1.0, 0.05, 100.0)  # outside the cone (0.76 m at 2 m): holding
    assert start.action == HOLD
    # It shrinks, but by less than a fifth in three seconds: abort, and climb.
    for step in range(1, 30):
        answer = guide.update(2.0, 1.0 - 0.005 * step, 0.05, 100.0 + 0.1 * step)
        assert answer.action == HOLD, step
    stalled = guide.update(2.0, 0.85, 0.05, 103.0)
    assert stalled.action == ABORT and stalled.rate_mps == cd.CLIMB_RATE_MPS and "did not shrink by 20% in 3 s" in stalled.reason and guide.aborted
    # The abort stands until the mission starts the descent again, whatever is seen meanwhile.
    assert guide.update(2.0, 0.1, 0.05, 104.0).action == ABORT
    guide.reset()
    assert not guide.aborted and guide.update(2.0, 0.1, 0.05, 105.0).action == DESCEND
    assert (cd.STALL_SHRINK, cd.STALL_S) == (0.2, 3.0)


def test_an_error_that_keeps_shrinking_by_a_fifth_is_given_its_time_and_the_cone_resets_the_clock():
    guide = cone()
    error, now = 1.1, 200.0
    guide.update(2.0, error, 0.05, now)
    # Each 2.5 s it is a fifth smaller: slow, and never stalled.
    for _ in range(2):
        now += 2.5
        error *= 0.79
        assert guide.update(2.0, error, 0.05, now).action in (HOLD, DESCEND)
    assert error < cd.allowed_error_m(2.0, CAPTURE) and guide.update(2.0, error, 0.05, now + 0.1).action == DESCEND
    # Inside the cone the clock is not running: ten seconds of steady descent with the same error is no stall.
    for step in range(100):
        assert guide.update(2.0, error, 0.05, now + 0.2 + 0.1 * step).action == DESCEND
    # Out again: a new three seconds from here.
    guide.update(2.0, 1.0, 0.05, now + 11.0)
    assert guide.update(2.0, 1.0, 0.05, now + 13.9).action == HOLD
    assert guide.update(2.0, 1.0, 0.05, now + 14.1).action == ABORT
    # A clock that steps back starts the three seconds again; it does not abort on a negative wait.
    again = cone()
    again.update(2.0, 1.0, 0.05, 50.0)
    assert again.update(2.0, 1.0, 0.05, 20.0).action == HOLD and again.update(2.0, 1.0, 0.05, 22.9).action == HOLD
    assert again.update(2.0, 1.0, 0.05, 23.1).action == ABORT


def test_below_eight_tenths_of_a_metre_it_descends_only_on_a_detection_under_a_fifth_of_a_second_old():
    guide = cone()
    fresh = guide.update(0.6, 0.05, 0.19, 300.0)
    assert fresh.action == DESCEND and fresh.rate_mps == 0.1
    stale = guide.update(0.6, 0.05, 0.21, 300.1)
    assert stale.action == HOLD and "0.21 s old" in stale.reason and "0.2 s" in stale.reason
    unseen = guide.update(0.6, 0.05, None, 300.2)
    assert unseen.action == HOLD and "no detection" in unseen.reason
    # At and above 0.8 m the estimator's track may carry the error between frames, for as long as the estimator itself
    # takes a frame (a second): not for ever, and not with no detection at all.
    from drone_mission_core import target_access

    assert guide.update(0.8, 0.05, 0.4, 300.3).action == DESCEND and guide.update(2.0, 0.05, 0.5, 300.4).action == DESCEND
    old = guide.update(2.0, 0.05, 0.6, 300.5)
    assert old.action == HOLD and "at most 0.5 s" in old.reason and guide.update(2.0, 0.05, None, 300.6).action == HOLD
    # Half a second: the age every other descent in the repo is held to, taken from the one place that rule lives.
    assert (cd.FRESH_BELOW_M, cd.MAX_DETECTION_AGE_S) == (0.8, 0.2) and cd.MAX_TRACK_AGE_S == target_access.DESCEND_MAX_AGE_S == 0.5


def test_with_nothing_to_descend_on_it_holds_for_a_second_and_a_half_and_then_says_the_handle_is_lost():
    """The flight-safety review: a hold for want of a detection had no end. After 1.5 s the cone says LOST, and
    the mission's lost-target rule (hold 1.5 s, then climb) takes over. It is not a stall, and not an abort."""
    guide = cone()
    for step in range(15):  # 1.4 s with a detection half a second old, the hook 0.6 m above the handle
        assert guide.update(0.6, 0.05, 0.5, 400.0 + 0.1 * step).action == HOLD, step
    lost = guide.update(0.6, 0.05, 0.5, 401.5)
    assert lost.action == LOST and lost.rate_mps == 0.0 and "for 1.5 s: the handle is lost" in lost.reason and not guide.aborted
    # One fresh detection and the descent goes on; the wait starts again from nothing.
    assert guide.update(0.6, 0.05, 0.05, 401.6).action == DESCEND
    assert guide.update(0.6, 0.05, 0.5, 401.7).action == HOLD and guide.update(0.6, 0.05, 0.5, 403.1).action == HOLD
    assert guide.update(0.6, 0.05, 0.5, 403.2).action == LOST
    # The same for no error at all, and for a height that did not come (a rangefinder reading dropped): held, then
    # lost, never an exception that would end the mission with the hook over the bucket.
    for missing in (dict(hook_height_m=2.0, error_m=None, detection_age_s=0.05), dict(hook_height_m=None, error_m=0.1, detection_age_s=0.05), dict(hook_height_m=float("nan"), error_m=0.1, detection_age_s=0.05)):
        other = cone()
        assert other.update(now_s=10.0, **missing).action == HOLD
        assert other.update(now_s=11.4, **missing).action == HOLD and other.update(now_s=11.5, **missing).action == LOST
    assert cd.BLIND_HOLD_S == 1.5


def test_the_rates_are_under_the_final_descents_cap_a_climb_has_somewhere_to_go_and_an_abort_stands_whatever_is_known():
    from fractions import Fraction

    from drone_mission_core import safety_limits

    cap = safety_limits.PHASE_SPEED_CAPS_MPS["final_descent"]
    assert all(cd.descent_rate_mps(height) <= cap for height in (0.0, 0.5, 1.0, 2.0, 3.0, 10.0)) and cd.CLIMB_RATE_MPS <= cap
    # Any real number type is a number, not only Python's float: vision code produces NumPy's, which are
    # numbers.Real as these fractions are (safety_limits takes them for the same reason).
    guide = cone()
    assert guide.update(Fraction(2), Fraction(3, 10), Fraction(1, 20), 0.0).action == DESCEND
    # At 3 m and above the cone is as wide as it gets: an error far outside it is held, not climbed away from,
    # and the stall rule ends the descent. A climb with no ceiling is never asked for.
    high = cone()
    assert high.update(3.5, 2.0, 0.05, 0.0).action == HOLD and "no wider higher up" in high.update(3.5, 2.0, 0.05, 0.1).reason
    assert high.update(2.9, 2.0, 0.05, 0.2).action == CLIMB  # below 3 m a climb widens the cone
    aborted = high.update(3.5, 2.0, 0.05, 3.1)
    assert aborted.action == ABORT
    # Once aborted, the answer is ABORT whatever is known: a height that did not come does not raise, or hold.
    assert high.update(None, None, None, 3.2).action == ABORT and high.update(float("nan"), 0.1, 0.05, 3.3).action == ABORT


def test_a_descent_flown_by_the_cone_arrives_at_the_handle_inside_the_capture_width():
    """A vehicle that closes a tenth of its sideways error every tick (20 Hz), started 0.9 m out at 3 m: it comes
    down inside the cone the whole way, and is within 0.8 of the capture half-width when the hook is at the handle."""
    guide = cone()
    height, error, now = 3.0, 0.9, 0.0
    outside = 0
    while height > 0.0 and now < 120.0:
        answer = guide.update(height, error, 0.05, now)
        assert answer.action in (DESCEND, HOLD), (now, answer)
        outside += error > answer.cone_m
        if answer.action == DESCEND:
            height -= answer.rate_mps * 0.05
        error = max(0.01, error * 0.98)
        now += 0.05
    assert height <= 0.0 and error <= 0.8 * CAPTURE and now < 30.0 and outside == 0
    # A vehicle that drifts out low down is held, then climbed away from, and never descends outside the cone.
    drifter = cone()
    assert drifter.update(0.5, 0.2, 0.05, 0.0).action in (HOLD, CLIMB)
    assert drifter.update(0.3, 0.2, 0.05, 0.1).action == CLIMB  # 2.5 times the 0.08 m allowed there


# --- the spec audit's points ------------------------------------------------------------------


def test_an_error_that_came_down_by_a_fifth_has_three_seconds_more_while_still_outside_the_cone():
    """The 20%: progress restarts the three seconds. Without that, this descent would be given up five seconds in,
    with the error a fifth smaller than it was and still coming down."""
    guide = cone()
    assert guide.update(2.0, 1.14, 0.05, 0.0).action == HOLD  # outside the 0.76 m cone at 2 m, inside 1.5 times it
    assert guide.update(2.0, 0.90, 0.05, 2.5).action == HOLD  # 21% smaller, 2.5 s on: the three seconds start again from here
    assert guide.update(2.0, 0.90, 0.05, 5.0).action == HOLD  # 5 s from the start, 2.5 s from the progress: not a stall
    assert guide.update(2.0, 0.90, 0.05, 5.4).action == HOLD
    assert guide.update(2.0, 0.90, 0.05, 5.6).action == ABORT  # 3.1 s with no more progress
    # A fifth is the line: 19% in three seconds is a stall.
    slow = cone()
    slow.update(2.0, 1.14, 0.05, 0.0)
    assert slow.update(2.0, 1.14 * 0.81, 0.05, 2.9).action == HOLD and slow.update(2.0, 1.14 * 0.81, 0.05, 3.0).action == ABORT


def test_the_error_is_the_hooks_not_the_vehicles_or_the_image_centres():
    # A vehicle exactly over the handle, pointing east, its hook 0.10 m ahead of it and 0.05 m to its left.
    assert cd.hook_error_m((10.0, 20.0), (10.0, 20.0), 0.0, (0.10, 0.05)) == pytest.approx(math.hypot(0.10, 0.05))
    # With the hook over the handle the error is nothing, whichever way the vehicle points.
    assert cd.hook_error_m((10.10, 20.05), (10.0, 20.0), 0.0, (0.10, 0.05)) == pytest.approx(0.0, abs=1.0e-12)
    assert cd.hook_error_m((9.95, 20.10), (10.0, 20.0), math.pi / 2.0, (0.10, 0.05)) == pytest.approx(0.0, abs=1.0e-12)  # pointing north: ahead is north, left is west
    assert cd.hook_error_m((10.0, 20.0), (10.3, 20.4), 1.0) == pytest.approx(0.5)  # no offset: the vehicle's own point
    # A position or a heading that did not come this tick gives no error, and the cone holds on that: nothing raises
    # with the hook over the bucket. Only the hook's own place on the vehicle, which is configuration, is refused.
    assert cd.hook_error_m((10.0, float("nan")), (10.0, 20.0), 0.0) is None and cd.hook_error_m(None, (10.0, 20.0), 0.0) is None
    assert cd.hook_error_m((10.0, 20.0), (10.0, 20.0), None) is None
    with pytest.raises(ValueError):
        cd.hook_error_m((10.0, 20.0), (10.0, 20.0), 0.0, (float("nan"), 0.0))
    assert cd.hook_height_above_handle_m(None, 0.55, 0.3) is None and cd.hook_height_above_handle_m(float("nan"), 0.55, 0.3) is None
    missing = cone().update(cd.hook_height_above_handle_m(None, 0.55, 0.3), cd.hook_error_m(None, (10.0, 20.0), 0.0), 0.05, 0.0)
    assert missing.action == HOLD and cone().update(2.0, float("nan"), 0.05, 0.0).action == HOLD
    # The height the cone is asked with is the hook's above the handle: a vehicle 1.4 m up with its hook 0.55 m
    # under it is 0.55 m above a handle 0.3 m up, and the cone is already narrowing to the mechanism's width.
    height = cd.hook_height_above_handle_m(1.4, 0.55, 0.3)
    assert height == pytest.approx(0.55) and cd.allowed_error_m(height, CAPTURE) < 0.48
    assert cd.hook_height_above_handle_m(0.8, 0.55, 0.3) == pytest.approx(-0.05)  # the hook just past the handle
    with pytest.raises(ValueError):
        cd.hook_height_above_handle_m(1.4, -0.1, 0.3)


def test_a_detection_stamped_ahead_of_the_clock_is_not_a_fresh_one():
    guide = cone()
    assert guide.update(0.6, 0.05, -0.02, 0.0).action == DESCEND  # a frame's stamp a few milliseconds ahead: new
    ahead = guide.update(0.6, 0.05, -0.5, 0.1)
    assert ahead.action == HOLD and "no detection" in ahead.reason and cd.CLOCK_SLACK_S == 0.05


def test_a_detection_that_comes_and_goes_does_not_keep_the_cone_holding_for_ever():
    """The spec audit, round two: with the age asked before the cone, a detection every quarter of a second kept
    restarting both clocks, and the cone held without end with the hook 0.6 m over the handle."""
    guide = cone()
    answers = []
    for tick in range(70):  # 3.5 s at 20 Hz; a detection every 0.25 s, so its age runs 0.05 to 0.25 s
        age = 0.05 + 0.05 * (tick % 5)
        answers.append(guide.update(0.6, 0.35, age, 0.05 * tick).action)  # 0.35 m out where the cone is 0.28 m, and not shrinking
    assert answers[0] == HOLD and ABORT in answers and answers.index(ABORT) == 60  # three seconds, whatever the detections' ages
    # Outside the cone the age does not hold back a climb either: an old track that says the hook is far out is climbed away from.
    assert cone().update(0.6, 1.0, 5.0, 0.0).action == CLIMB and cone().update(0.6, 1.0, None, 0.0).action == CLIMB
    # Inside the cone the same coming and going is a descent in bursts: down on the fresh ticks, still on the stale ones.
    inside = cone()
    bursts = [inside.update(0.6, 0.05, 0.05 + 0.05 * (tick % 5), 0.05 * tick).action for tick in range(40)]
    assert bursts.count(DESCEND) == 32 and bursts.count(HOLD) == 8 and LOST not in bursts
    # An error that was outside and not shrinking, unseen for a second, has not started shrinking: the three seconds run on.
    patient = cone()
    patient.update(2.0, 1.0, 0.05, 0.0)
    assert patient.update(2.0, None, None, 1.0).action == HOLD and patient.update(2.0, 1.0, 0.05, 2.0).action == HOLD
    assert patient.update(2.0, 1.0, 0.05, 3.0).action == ABORT
