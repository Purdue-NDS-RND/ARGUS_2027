"""Engaging the handle, checking the bucket is on the hook, carrying it without swing, and letting it go (REC-4)."""

import math
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_mission_core import recovery_grip as rg  # noqa: E402
from drone_mission_core import safety_limits  # noqa: E402

# --- engage -----------------------------------------------------------------------------------


def test_a_plunge_goes_past_the_handle_closes_the_gripper_once_and_needs_the_acknowledgement():
    engage = rg.Engage(rg.EngagePlan(mode="plunge", plunge_depth_m=0.06))
    above = engage.update(hook_height_m=0.10, ack=None, now_s=0.0)
    assert above.action == rg.MOVE and above.target_height_m == -0.06 and above.rate_mps == rg.ENGAGE_RATE_MPS and not above.close_gripper
    at_depth = engage.update(-0.055, None, 1.0)  # within a centimetre of 6 cm past the handle
    assert at_depth.action == rg.CLOSE and at_depth.close_gripper  # the one tick the gripper is told to close
    waiting = engage.update(-0.06, None, 1.2)
    assert waiting.action == rg.WAIT and not waiting.close_gripper
    assert at_depth.attempt == engage.attempt >= 1  # the number the close command's acknowledgement has to carry
    done = engage.update(-0.06, rg.Ack(engage.attempt, 0), 1.4)
    assert done.action == rg.ENGAGED and engage.engaged and engage.update(-0.06, None, 5.0).action == rg.ENGAGED
    assert rg.ENGAGE_RATE_MPS <= safety_limits.PHASE_SPEED_CAPS_MPS["final_descent"]


def test_a_gripper_that_refuses_or_does_not_answer_is_not_engaged():
    refused = rg.Engage(rg.EngagePlan(mode="plunge"))
    refused.update(-0.2, None, 0.0)
    answer = refused.update(-0.2, rg.Ack(refused.attempt, 4), 0.1)  # MAV_RESULT_FAILED
    assert answer.action == rg.REFUSED and "result 4" in answer.reason and not refused.engaged
    silent = rg.Engage(rg.EngagePlan(mode="plunge", ack_timeout_s=1.0))
    silent.update(-0.2, None, 0.0)
    assert silent.update(-0.2, None, 0.9).action == rg.WAIT
    late = silent.update(-0.2, None, 1.0)
    assert late.action == rg.REFUSED and "no acknowledgement" in late.reason
    first = silent.attempt
    assert silent.update(-0.2, rg.Ack(first, 0), 1.1).action == rg.REFUSED  # an answer after the wait does not make a grab of a refusal
    silent.reset()
    assert silent.update(0.1, None, 2.0).action == rg.MOVE and silent.attempt > first
    # The flight-safety review: an acknowledgement kept from the attempt before is not this attempt's, however long it is shown.
    closing = silent.update(-0.2, rg.Ack(first, 0), 2.1)
    assert closing.action == rg.CLOSE and closing.attempt == silent.attempt
    assert silent.update(-0.2, rg.Ack(first, 0), 2.2).action == rg.WAIT and silent.update(-0.2, 0, 2.3).action == rg.WAIT  # nor is a bare result
    assert silent.update(-0.2, rg.Ack(silent.attempt, 0), 2.4).action == rg.ENGAGED
    # Nor does a mission that makes a new Engage for each try get the old number back.
    again = rg.Engage(rg.EngagePlan(mode="plunge"))
    assert again.attempt not in (first, silent.attempt)
    again.update(-0.2, None, 0.0)
    assert again.update(-0.2, rg.Ack(silent.attempt, 0), 0.1).action == rg.WAIT
    # A height that did not come: wait where it is; nothing is closed on a guess.
    blind = rg.Engage(rg.EngagePlan(mode="plunge"))
    assert blind.update(None, None, 0.0).action == rg.WAIT and not blind.update(float("nan"), None, 0.1).close_gripper
    # The way down to the depth has an end: a hook that does not get there, or a height that never comes, is a refusal.
    stuck = rg.Engage(rg.EngagePlan(mode="plunge", move_timeout_s=4.0))
    assert stuck.update(0.10, None, 0.0).action == rg.MOVE and stuck.update(None, None, 3.9).action == rg.WAIT
    gave_up = stuck.update(0.10, None, 4.0)
    assert gave_up.action == rg.REFUSED and "not brought to the engage's depth in 4 s" in gave_up.reason and not gave_up.close_gripper


def test_a_sweep_closes_at_the_handles_height_and_then_crosses_it():
    plan = rg.EngagePlan(mode="sweep", sweep_length_m=0.30)
    engage = rg.Engage(plan)
    coming = engage.update(0.2, None, 0.0)
    assert coming.action == rg.MOVE and coming.target_height_m == 0.0 and coming.sweep_m == pytest.approx(-0.15)  # to one side of the handle, at its height
    closing = engage.update(0.005, None, 1.0)
    assert closing.action == rg.CLOSE and closing.close_gripper
    crossing = engage.update(0.0, rg.Ack(engage.attempt, 0), 1.2)
    assert crossing.action == rg.MOVE and crossing.sweep_m == pytest.approx(0.15)  # across to the other side: the hook takes the handle on the way
    assert engage.update(0.0, None, 1.3, swept_m=0.10).action == rg.MOVE
    assert engage.update(0.0, None, 2.0, swept_m=0.15).action == rg.ENGAGED
    # A crossing that is never seen to arrive ends too.
    lost = rg.Engage(rg.EngagePlan(mode="sweep", move_timeout_s=5.0))
    lost.update(0.0, None, 0.0)
    assert lost.update(0.0, rg.Ack(lost.attempt, 0), 0.2).action == rg.MOVE and lost.update(0.0, None, 5.1, swept_m=None).action == rg.MOVE
    assert lost.update(0.0, None, 5.2).action == rg.REFUSED and "did not cross" in lost.update(0.0, None, 5.3).reason
    with pytest.raises(ValueError, match="plunge or sweep"):
        rg.EngagePlan(mode="grab")
    with pytest.raises(ValueError):
        rg.EngagePlan(mode="plunge", plunge_depth_m=-0.1)
    with pytest.raises(ValueError, match="not a plunge"):
        rg.EngagePlan(mode="plunge", plunge_depth_m=5.0)  # centimetres written as metres


# --- lift and verify ----------------------------------------------------------------------------


def test_the_lift_is_the_suspension_length_and_half_a_metre_held_a_second_and_a_half():
    assert rg.lift_height_m(0.55) == pytest.approx(1.05) and (rg.LIFT_MARGIN_M, rg.VERIFY_HOLD_S) == (0.5, 1.5)
    with pytest.raises(ValueError):
        rg.lift_height_m(-0.1)


def test_the_expected_throttle_ratio_is_the_specs_for_a_kilogram_on_four():
    # Thrust goes with the weight and throttle with the square root of thrust: 1 kg on 4 kg is about 1.12.
    assert rg.expected_throttle_ratio(4.0, 1.0) == pytest.approx(math.sqrt(1.25)) == pytest.approx(1.118, abs=0.001)
    assert abs(rg.expected_throttle_ratio(4.0, 1.0) - 1.125) < 0.01  # the spec's "about 1.125"
    assert rg.expected_throttle_ratio(4.0, 2.0) > rg.expected_throttle_ratio(4.0, 1.0) and rg.expected_throttle_ratio(4.0, 0.0) == 1.0
    with pytest.raises(ValueError):
        rg.expected_throttle_ratio(0.0, 1.0)


def hover(check, throttle, start, seconds, loaded):
    for step in range(int(round(seconds * 20)) + 1):  # a sample at each end of the hover
        (check.loaded_sample if loaded else check.baseline_sample)(throttle + 0.004 * math.sin(step), start + 0.05 * step, 0.05)


def test_attached_is_the_throttle_risen_the_bucket_still_in_the_hook_camera_and_the_latch_closed():
    check = rg.AttachCheck(vehicle_kg=4.0, load_kg=1.0)
    hover(check, 0.40, 0.0, 2.0, loaded=False)
    hover(check, 0.40 * 1.115, 10.0, 1.5, loaded=True)
    verdict = check.verdict(visual_fixed=True, latch_closed=True)
    assert verdict.attached and verdict.throttle_ratio == pytest.approx(1.115, abs=0.005) and verdict.expected_ratio == pytest.approx(1.118, abs=0.001)
    assert verdict.parts == {"throttle": True, "visual": True, "latch": True}
    # No latch switch fitted, no hook camera: the throttle alone decides, and the verdict says what it did not have.
    alone = check.verdict(visual_fixed=None, latch_closed=None)
    assert alone.attached and alone.parts == {"throttle": True, "visual": None, "latch": None}
    # Any one of the three saying no is not attached.
    assert not check.verdict(visual_fixed=False, latch_closed=True).attached and not check.verdict(visual_fixed=True, latch_closed=False).attached
    # The throttle did not rise (the hook came up empty): not attached, whatever a switch says.
    empty = rg.AttachCheck(4.0, 1.0)
    hover(empty, 0.40, 0.0, 2.0, loaded=False)
    hover(empty, 0.405, 10.0, 1.5, loaded=True)
    missed = empty.verdict(visual_fixed=None, latch_closed=True)
    assert not missed.attached and missed.parts["throttle"] is False and "throttle" in missed.reason
    assert rg.MIN_RISE_SHARE == 0.5  # half of the rise the load should give is enough to say it is there
    # Either side of that line: for 1 kg on 4 kg the load should give 1.118, so the line is 1.059.
    for ratio, attached in ((1.07, True), (1.05, False)):
        near = rg.AttachCheck(4.0, 1.0)
        hover(near, 0.40, 0.0, 2.0, loaded=False)
        for step in range(31):
            near.loaded_sample(0.40 * ratio, 10.0 + 0.05 * step, 0.05)
        assert near.verdict().attached is attached, ratio


def test_a_throttle_stream_that_has_stopped_does_not_fill_a_hover_with_one_old_reading():
    """The flight-safety review: the newest throttle, re-read every tick after the stream stopped, was a sample every tick."""
    check = rg.AttachCheck(4.0, 1.0)
    hover(check, 0.40, 0.0, 2.0, loaded=False)
    for step in range(40):  # two seconds of ticks; the throttle last came at 10.0 s (the climb's: high)
        check.loaded_sample(0.60, 10.0 + 0.05 * step, age_s=0.05 * step)
    assert not check.ready and not check.verdict().attached and "held 0.2 s" in check.verdict().reason  # the first 0.2 s were still now
    stale = rg.AttachCheck(4.0, 1.0)
    for step in range(40):
        stale.baseline_sample(0.40, 0.05 * step, age_s=5.0)
        stale.baseline_sample(0.40, 0.05 * step, age_s=None)  # an age that is not known is not fresh
    assert stale.baseline_throttle is None
    # The watch on the way home: a throttle that old is not heard, so it is neither a fall nor a reassurance.
    watch = rg.LoadWatch(0.40, 0.446)
    assert watch.update(0.446, 0.0, 0.05) is False and watch.unheard_s(0.0) == 0.0
    for step in range(1, 60):
        assert watch.update(0.40, 0.05 * step, age_s=0.05 * step + 0.3) is False  # the low reading is old
    assert watch.unheard_s(3.0) == pytest.approx(3.0) and not watch.lost


def test_the_verdict_waits_for_its_holds_and_is_not_given_on_too_little():
    check = rg.AttachCheck(4.0, 1.0)
    assert not check.ready and not check.verdict().attached and "no hover throttle before the grab" in check.verdict().reason
    hover(check, 0.40, 0.0, 2.0, loaded=False)
    hover(check, 0.45, 10.0, 1.0, loaded=True)  # a second of the second and a half
    assert not check.ready and "held" in check.verdict().reason
    hover(check, 0.45, 11.0, 0.6, loaded=True)
    assert check.ready and check.verdict().attached
    # Nonsense samples are not samples.
    check.loaded_sample(float("nan"), 20.0, 0.0)
    check.baseline_sample(None, 20.0, 0.0)
    assert check.verdict().attached
    with pytest.raises(TypeError):
        check.loaded_sample(0.45, 21.0)  # the age is not left out: a reading of unknown age is not a sample
    with pytest.raises(TypeError):
        rg.LoadWatch(0.40, 0.446).update(0.446, 0.0)


def test_the_throttle_falling_back_on_the_way_home_means_the_bucket_is_gone():
    watch = rg.LoadWatch(baseline_throttle=0.40, loaded_throttle=0.446)
    for step in range(100):  # five seconds carrying
        assert watch.update(0.446 + 0.01 * math.sin(step), 0.05 * step, 0.05) is False
    # A gust takes the throttle down for a moment: not a loss.
    assert watch.update(0.40, 5.0, 0.05) is False and watch.update(0.445, 5.3, 0.05) is False
    # It falls back to the empty hover and stays there a second: lost.
    lost = [watch.update(0.402, 6.0 + 0.05 * step, 0.05) for step in range(25)]
    assert lost[0] is False and lost[19] is False and lost[20] is True and watch.lost
    assert (rg.LOST_SHARE, rg.LOST_AFTER_S) == (0.25, 1.0)
    with pytest.raises(ValueError):
        rg.LoadWatch(baseline_throttle=0.40, loaded_throttle=0.39)  # a load that lowered the throttle is no load to watch


# --- swing shaping ------------------------------------------------------------------------------


def swing_after(ramp_s, length_m=0.72, v=3.0, settle_s=6.0):
    """The swing (metres, peak) left after the pivot's speed has gone from 0 to `v` in `ramp_s`: a pendulum, integrated."""
    g, dt = 9.80665, 0.0005
    theta = omega = 0.0
    t, peak = 0.0, 0.0
    while t < ramp_s + settle_s:
        acceleration = v / ramp_s if t < ramp_s else 0.0
        omega += (-(g / length_m) * math.sin(theta) - (acceleration / length_m) * math.cos(theta)) * dt
        theta += omega * dt
        t += dt
        if t > ramp_s:
            peak = max(peak, abs(length_m * math.sin(theta)))
    return peak


def test_a_ramp_that_lasts_a_whole_pendulum_period_leaves_no_swing():
    period = rg.pendulum_period_s(0.72)
    assert period == pytest.approx(2.0 * math.pi * math.sqrt(0.72 / 9.80665)) == pytest.approx(1.70, abs=0.01)
    assert rg.ramp_duration_s(0.72) == period and rg.ramp_duration_s(0.72, periods=2) == pytest.approx(2.0 * period)
    shaped = swing_after(period)
    half = swing_after(period / 2.0)
    step = swing_after(0.05)
    assert shaped < 0.02 and half > 0.25 and step > 0.25 and shaped < 0.1 * half  # two centimetres against a quarter of a metre and more
    assert swing_after(2.0 * period) < 0.02
    with pytest.raises(ValueError):
        rg.pendulum_period_s(0.0)
    with pytest.raises(ValueError):
        rg.ramp_duration_s(0.72, periods=0)
    # A speed that is no speed is refused when the ramp or the leg is made: not flown as "already there".
    for bad in (0.0, None, float("nan"), -1.0):
        with pytest.raises(ValueError):
            rg.ShapedLeg(10.0, bad, 0.72)
    for bad in (None, float("nan"), -1.0):
        with pytest.raises(ValueError):
            rg.ShapedRamp(0.0, bad, 0.72, started_s=0.0)


def test_the_shaped_speed_ramps_evenly_over_the_period_and_never_passes_the_carrying_cap():
    ramp = rg.ShapedRamp(from_mps=0.0, to_mps=2.0, length_m=0.72, started_s=10.0)
    period = rg.pendulum_period_s(0.72)
    assert ramp.speed_mps(9.0) == 0.0 and ramp.speed_mps(10.0) == 0.0
    assert ramp.speed_mps(10.0 + period / 2.0) == pytest.approx(1.0) and ramp.speed_mps(10.0 + period) == pytest.approx(2.0) == ramp.speed_mps(99.0)
    assert not ramp.done(10.0 + 0.9 * period) and ramp.done(10.0 + period)
    assert ramp.distance_m == pytest.approx(0.5 * 2.0 * period)  # the ground covered while ramping: when to start slowing
    # Down again is the same ramp the other way.
    stop = rg.ShapedRamp(2.0, 0.0, 0.72, started_s=20.0)
    assert stop.speed_mps(20.0 + period / 4.0) == pytest.approx(1.5) and stop.speed_mps(30.0) == 0.0
    # Asked for more than the carrying cap, it goes to the cap.
    fast = rg.ShapedRamp(0.0, 5.0, 0.72, started_s=0.0)
    assert fast.to_mps == safety_limits.PHASE_SPEED_CAPS_MPS["carrying"] == 3.0 and fast.speed_mps(100.0) == 3.0
    assert rg.SWING_FLAG == "swing_shaping"


def test_a_leg_is_flown_with_a_shaped_start_and_a_shaped_stop_and_a_short_one_slower():
    period = rg.pendulum_period_s(0.72)
    leg = rg.ShapedLeg(distance_m=14.4, speed_mps=3.0, length_m=0.72)
    assert leg.speed_mps == 3.0 and leg.ramp_s == period and leg.duration_s == pytest.approx(2.0 * period + (14.4 - 3.0 * period) / 3.0)
    assert leg.travelled_m(0.0) == 0.0 and leg.travelled_m(leg.duration_s) == pytest.approx(14.4) == leg.travelled_m(leg.duration_s + 10.0)
    assert leg.travelled_m(period) == pytest.approx(0.5 * 3.0 * period)  # the first ramp's ground
    # Its speed is never over the asked one, and it sets off and arrives at rest.
    step = 0.01
    speeds = [(leg.travelled_m(t + step) - leg.travelled_m(t)) / step for t in [0.01 * i for i in range(int(leg.duration_s / 0.01))]]
    assert max(speeds) <= 3.0 + 1.0e-6 and speeds[0] < 0.05 and speeds[-1] < 0.05
    # Two metres is too short for two ramps at 3 m/s: flown at the speed two ramps just fit in.
    short = rg.ShapedLeg(2.0, 3.0, 0.72)
    assert short.speed_mps == pytest.approx(2.0 / period) and short.cruise_s == pytest.approx(0.0) and short.duration_s == pytest.approx(2.0 * period)
    assert rg.ShapedLeg(0.0, 3.0, 0.72).travelled_m(1.0) == 0.0 and rg.ShapedLeg(5.0, 9.0, 0.72).speed_mps <= 3.0


# --- the release guard --------------------------------------------------------------------------


def now(**given):
    """The ages of a landed state and a rangefinder reading that have just come."""
    return {"height_age_s": 0.05, "landed_age_s": 0.5, **given}


def test_the_bucket_is_let_go_landed_or_low_on_a_healthy_rangefinder_with_the_set_down_felt():
    ceiling = rg.MAX_RELEASE_HEIGHT_M
    assert ceiling == min(1.5, 1.83) == 1.5 and rg.SIX_FEET_M == 1.83
    assert ceiling <= safety_limits.SETDOWN_MAX_RELEASE_HEIGHT_M < 1.8288 < rg.SIX_FEET_M  # as a ceiling, 6 ft is under 1.83
    assert rg.release_decision(landed=True, rangefinder_healthy=False, height_m=None, set_down_detected=False, **now()).action == rg.RELEASE
    ok = rg.release_decision(landed=False, rangefinder_healthy=True, height_m=0.9, set_down_detected=True, **now())
    assert ok.action == rg.RELEASE and ok.reason == ""
    high = rg.release_decision(False, True, 1.6, True, **now())
    assert high.action == rg.HOLD_LOAD and "1.60 m" in high.reason and "1.5 m" in high.reason
    assert rg.release_decision(False, True, 1.5, True, **now()).action == rg.RELEASE  # at the ceiling
    not_down = rg.release_decision(False, True, 0.9, False, **now())
    assert not_down.action == rg.HOLD_LOAD and "set-down" in not_down.reason
    # No healthy rangefinder: the height is not known well enough to let go in the air. Land first.
    for height in (0.5, None, float("nan")):
        blind = rg.release_decision(False, False, height, True, **now())
        assert blind.action == rg.LAND_FIRST and "rangefinder" in blind.reason
    assert rg.release_decision(False, True, None, True, **now()).action == rg.LAND_FIRST  # healthy, it says, and gives no height


def test_a_height_or_a_landed_state_from_another_moment_lets_nothing_go():
    """The reviews: a rangefinder frozen at a low reading, and a landed state with no age (left to REC-4 by FND-09)."""
    frozen = rg.release_decision(False, True, 0.9, True, height_age_s=0.5, landed_age_s=0.1)
    assert frozen.action == rg.LAND_FIRST and "from this moment" in frozen.reason
    assert rg.release_decision(False, True, 0.9, True, height_age_s=0.2, landed_age_s=0.1).action == rg.RELEASE
    assert rg.release_decision(False, True, 0.9, True, height_age_s=None, landed_age_s=0.1).action == rg.LAND_FIRST
    # "Landed", heard three seconds ago, in the air now at 4 m: not a release.
    old = rg.release_decision(True, True, 4.0, False, height_age_s=0.05, landed_age_s=3.0)
    assert old.action == rg.HOLD_LOAD and rg.LANDED_MAX_AGE_S == 2.0
    assert rg.release_decision(True, False, None, False, height_age_s=None, landed_age_s=None).action == rg.LAND_FIRST
    assert rg.release_decision(True, False, None, False, height_age_s=None, landed_age_s=2.0).action == rg.RELEASE
    with pytest.raises(TypeError):
        rg.release_decision(True, True, 0.5, True)  # the ages are not left out


def test_ten_thousand_random_states_never_release_above_six_feet_or_on_a_bad_rangefinder_in_the_air():
    rng = random.Random(4)
    released = 0
    for _ in range(10_000):
        landed = rng.random() < 0.1
        healthy = rng.random() < 0.7
        height = rng.choice([None, float("nan"), rng.uniform(-0.5, 12.0), rng.uniform(1.4, 1.9)])
        felt = rng.random() < 0.6
        height_age = rng.choice([0.0, 0.1, 0.2, 0.21, 1.0, None, float("nan"), -1.0])
        landed_age = rng.choice([0.0, 1.0, 2.0, 2.1, 30.0, None])
        answer = rg.release_decision(landed, healthy, height, felt, height_age_s=height_age, landed_age_s=landed_age)
        if answer.action != rg.RELEASE:
            continue
        released += 1
        on_the_ground = landed and landed_age is not None and landed_age <= 2.0
        if not on_the_ground:
            assert healthy and felt and height is not None and height == height and height <= 1.5 < 1.8288, (landed, healthy, height, felt)
            assert height_age is not None and 0.0 <= height_age <= 0.2, height_age
    assert released > 300  # and it does release when it may
