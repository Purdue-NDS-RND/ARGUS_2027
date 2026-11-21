"""Where the bucket is taken back to, in which frame the way there is flown, and how it is set down (REC-7)."""

import math
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_mission_core import recovery_grip as rg  # noqa: E402
from drone_mission_core import recovery_return as rr  # noqa: E402
from drone_mission_core import safety_limits  # noqa: E402

LAT, LON = 40.4237, -86.9212
TAKEOFF = rr.Anchor(e=2.0, n=-1.0, lat=LAT, lon=LON)  # where the vehicle took off, in both frames
PUBLISHED = (LAT + 30.0 / 111_194.9, LON)  # a point the organisers publish, 30 m north of the takeoff

# --- the target -------------------------------------------------------------------------------


def test_the_return_target_is_takeoff_a_published_point_or_a_marker():
    assert rr.parse_return_target(None) == rr.parse_return_target("takeoff") == rr.ReturnTarget(rr.TAKEOFF)
    assert rr.parse_return_target({"latlon": [40.5, -86.9]}) == rr.ReturnTarget(rr.LATLON, lat=40.5, lon=-86.9)
    assert rr.parse_return_target({"marker": 7}) == rr.ReturnTarget(rr.MARKER, marker="7")
    assert rr.parse_return_target({"marker": "home", "near": [40.5, -86.9]}) == rr.ReturnTarget(rr.MARKER, lat=40.5, lon=-86.9, marker="home")
    for bad in ("home", {"latlon": [91.0, 0.0]}, {"latlon": [40.5]}, {"latlon": [float("nan"), 0.0]}, {"latlon": [40.5, -86.9], "marker": 7}, {"marker": None}, 3, {}):
        with pytest.raises(ValueError):
            rr.parse_return_target(bad)


def test_a_point_goes_between_the_two_frames_through_the_takeoffs_pair():
    e, n = TAKEOFF.to_local(*PUBLISHED)
    assert (e, n) == pytest.approx((2.0, 29.0), abs=0.01)
    lat, lon = TAKEOFF.to_global(2.0 + 100.0, -1.0)
    assert lat == pytest.approx(LAT, abs=1e-9) and (lon - LON) * 111_194.9 * math.cos(math.radians(LAT)) == pytest.approx(100.0, abs=0.01)
    assert TAKEOFF.to_local(*TAKEOFF.to_global(57.0, -33.0)) == pytest.approx((57.0, -33.0), abs=1e-6)
    blind = rr.Anchor(e=2.0, n=-1.0)  # GPS was never trusted: no pair
    assert not blind.has_global and blind.to_local(*PUBLISHED) is None and blind.to_global(0.0, 0.0) is None


# --- the frame of the way back ----------------------------------------------------------------


def test_with_gps_trusted_the_way_back_is_a_global_setpoint_and_untrusted_a_local_one():
    takeoff = rr.ReturnTarget(rr.TAKEOFF)
    trusted = rr.return_goal(takeoff, gps_trusted=True, takeoff=TAKEOFF)
    assert trusted.frame == rr.GLOBAL and (trusted.lat, trusted.lon) == (LAT, LON) and (trusted.e, trusted.n) == (2.0, -1.0) and trusted.uncertainty_m == 0.0
    untrusted = rr.return_goal(takeoff, gps_trusted=False, takeoff=TAKEOFF, flown_untrusted_m=80.0)
    assert untrusted.frame == rr.LOCAL and (untrusted.e, untrusted.n) == (2.0, -1.0) and untrusted.lat is None and untrusted.lon is None
    assert untrusted.uncertainty_m == pytest.approx(4.0) and rr.DRIFT_SHARE == 0.05  # the worse end of "3 to 5% of distance flown"
    # A published point: global while GPS is trusted; untrusted, the same point in the EKF's frame through the takeoff's pair.
    published = rr.parse_return_target({"latlon": list(PUBLISHED)})
    there = rr.return_goal(published, gps_trusted=True, takeoff=TAKEOFF)
    assert there.frame == rr.GLOBAL and (there.lat, there.lon) == PUBLISHED and (there.e, there.n) == pytest.approx((2.0, 29.0), abs=0.01)
    local = rr.return_goal(published, gps_trusted=False, takeoff=TAKEOFF)
    assert local.frame == rr.LOCAL and (local.e, local.n) == pytest.approx((2.0, 29.0), abs=0.01) and local.reason == ""
    # With no pair at all (GPS never trusted) a published point cannot be placed: back to where it took off, and it says so.
    lost = rr.return_goal(published, gps_trusted=False, takeoff=rr.Anchor(e=2.0, n=-1.0))
    assert lost.frame == rr.LOCAL and (lost.e, lost.n) == (2.0, -1.0) and "cannot be placed" in lost.reason
    # Trusted, it says, with no pair to show for it: a takeoff return is still flown, in the frame there is.
    assert rr.return_goal(takeoff, gps_trusted=True, takeoff=rr.Anchor(e=2.0, n=-1.0)).frame == rr.LOCAL
    # A latitude and longitude written the wrong way round is a pair of numbers that passes for one (-86.9 is a
    # latitude): thousands of kilometres off. Not flown towards, trusted or not: the takeoff, and it says why.
    swapped = rr.parse_return_target({"latlon": [LON, LAT]})
    for trusted_now in (True, False):
        refused = rr.return_goal(swapped, gps_trusted=trusted_now, takeoff=TAKEOFF)
        assert (refused.e, refused.n) == (2.0, -1.0) and "from the takeoff" in refused.reason and refused.lat in (LAT, None)
    edge = rr.parse_return_target({"latlon": list(TAKEOFF.to_global(2.0, -1.0 + 999.0))})
    assert rr.return_goal(edge, gps_trusted=True, takeoff=TAKEOFF).reason == "" and rr.MAX_RETURN_DISTANCE_M == 1000.0


def test_a_global_setpoints_height_is_given_in_homes_terms():
    # Home is 0.19 m over the local frame's zero (the sim's): 3 m of local height is 2.81 m above home.
    assert rr.global_height_m(3.0, 0.19) == pytest.approx(2.81) and rr.global_height_m(3.0, 0.0) == 3.0
    with pytest.raises(ValueError):
        rr.global_height_m(3.0, None)


def test_untrusted_never_gives_a_global_setpoint_whatever_else_is_true():
    rng = random.Random(7)
    targets = [rr.ReturnTarget(rr.TAKEOFF), rr.parse_return_target({"latlon": list(PUBLISHED)}), rr.parse_return_target({"marker": 3}), rr.parse_return_target({"marker": 3, "near": list(PUBLISHED)})]
    seen_global = 0
    for _ in range(5000):
        trusted = rng.choice([True, False, None, 1, "yes", 0])
        anchor = rng.choice([TAKEOFF, rr.Anchor(e=rng.uniform(-50, 50), n=rng.uniform(-50, 50))])
        fix = rng.choice([None, rr.HomingFix(e=rng.uniform(-5, 5), n=rng.uniform(-5, 5), age_s=rng.choice([0.0, 0.3, 2.0]))])
        goal = rr.return_goal(rng.choice(targets), gps_trusted=trusted, takeoff=anchor, flown_untrusted_m=rng.uniform(0, 300), vehicle_en=(rng.uniform(-40, 40), rng.uniform(-40, 40)), homing=fix)
        assert goal.frame in (rr.GLOBAL, rr.LOCAL) and math.isfinite(goal.e) and math.isfinite(goal.n)
        if goal.frame == rr.GLOBAL:
            seen_global += 1
            assert trusted is True and anchor.has_global and goal.lat is not None  # and only then
        else:
            assert goal.lat is None and goal.lon is None
    assert seen_global > 200


def test_within_ten_metres_a_fresh_homing_fix_takes_over_and_only_then():
    marker = rr.parse_return_target({"marker": "home"})
    fix = rr.HomingFix(e=3.1, n=-0.2, age_s=0.1)  # the marker seen 1.4 m from where the takeoff is thought to be
    far = rr.return_goal(marker, gps_trusted=True, takeoff=TAKEOFF, vehicle_en=(2.0, 20.0), homing=fix)
    assert far.frame == rr.GLOBAL and not far.homing  # 21 m out: not yet
    near = rr.return_goal(marker, gps_trusted=True, takeoff=TAKEOFF, vehicle_en=(2.0, 7.0), homing=fix)
    assert near.frame == rr.LOCAL and near.homing and (near.e, near.n) == (3.1, -0.2) and rr.HOMING_RANGE_M == 10.0
    old = rr.return_goal(marker, gps_trusted=True, takeoff=TAKEOFF, vehicle_en=(2.0, 7.0), homing=rr.HomingFix(3.1, -0.2, age_s=0.8))
    assert not old.homing and old.frame == rr.GLOBAL
    # A "marker" 30 m from where the return point should be is some other marker.
    wrong = rr.return_goal(marker, gps_trusted=True, takeoff=TAKEOFF, vehicle_en=(2.0, 7.0), homing=rr.HomingFix(2.0, 29.0, age_s=0.1))
    assert not wrong.homing
    # With no fix a marker target is flown to its place: the takeoff, or the point given with it.
    assert (rr.return_goal(marker, gps_trusted=False, takeoff=TAKEOFF).e, rr.return_goal(marker, gps_trusted=False, takeoff=TAKEOFF).n) == (2.0, -1.0)
    # Homing is for any target, when something gives a fix (REC-7b): a takeoff return within 10 m takes it too.
    assert rr.return_goal(rr.ReturnTarget(rr.TAKEOFF), gps_trusted=False, takeoff=TAKEOFF, vehicle_en=(2.0, 7.0), homing=fix).homing


def test_the_way_is_walked_in_the_goals_own_frame():
    goal = rr.return_goal(rr.ReturnTarget(rr.TAKEOFF), gps_trusted=True, takeoff=TAKEOFF)
    start = rr.Anchor(e=14.0, n=7.0, lat=TAKEOFF.to_global(14.0, 7.0)[0], lon=TAKEOFF.to_global(14.0, 7.0)[1])
    assert rr.leg_length_m(start, goal) == pytest.approx(math.hypot(12.0, 8.0), abs=0.01)
    half = rr.setpoint_along(start, goal, 0.5)
    assert half.frame == rr.GLOBAL and TAKEOFF.to_local(half.lat, half.lon) == pytest.approx((8.0, 3.0), abs=0.01)
    assert rr.setpoint_along(start, goal, 1.7).lat == goal.lat and rr.setpoint_along(start, goal, -1.0).lat == pytest.approx(start.lat)
    local = rr.return_goal(rr.ReturnTarget(rr.TAKEOFF), gps_trusted=False, takeoff=TAKEOFF)
    quarter = rr.setpoint_along(start, local, 0.25)
    assert quarter.frame == rr.LOCAL and (quarter.e, quarter.n) == pytest.approx((11.0, 5.0)) and quarter.lat is None


# --- the set-down -------------------------------------------------------------------------------

EMPTY, LOADED = 0.40, 0.45
PLAN = rr.SetDownPlan(touch_height_m=0.90)  # the vehicle's height above the ground when the hanging bucket's bottom touches


def reading(now, height, throttle=LOADED, **given):
    base = dict(now_s=now, height_m=height, height_age_s=0.05, rangefinder_healthy=True, throttle=throttle, throttle_age_s=0.05, landed=False, landed_age_s=0.5, ack=None)
    base.update(given)
    return rr.SetDownReading(**base)


def test_the_bucket_is_down_when_the_throttle_has_come_half_way_back_for_three_tenths_of_a_second():
    feel = rr.BucketSetDown(EMPTY, LOADED)
    assert feel.line == pytest.approx(0.425) and (rr.SET_DOWN_SHARE, rr.SET_DOWN_HOLD_S) == (0.5, 0.3)
    assert not feel.update(0.45, 0.0, 0.05) and not feel.update(0.42, 0.05, 0.05)
    answers = [feel.update(0.41, 0.05 * step, 0.05) for step in range(2, 9)]
    assert answers == [False] * 5 + [True, True]  # 0.3 s after the first low reading
    assert not feel.update(0.44, 0.5, 0.05) and not feel.update(0.41, 0.55, 0.05)  # back up: the hold starts again
    # An old throttle, or a gap in the ticks, is not three tenths of a second of anything.
    stale = rr.BucketSetDown(EMPTY, LOADED)
    assert not any(stale.update(0.41, 0.05 * step, 0.05 * step + 0.3) for step in range(20))
    gappy = rr.BucketSetDown(EMPTY, LOADED)
    assert not any(gappy.update(0.41, 0.5 * step, 0.05) for step in range(6))
    with pytest.raises(ValueError):
        rr.BucketSetDown(0.45, 0.40)
    with pytest.raises(TypeError):
        feel.update(0.41, 1.0)  # the age is not left out


def fly(rule, vehicle_up, seconds, start=0.0, touch=0.90, ack_after=0.2, refuse=0):
    """A vehicle that does what the rule answers, over a bucket that takes its own weight when the vehicle is at `touch`."""
    log, t, opened_at, attempt, refused = [], start, None, None, 0
    for _ in range(int(seconds * 20)):
        down = vehicle_up <= touch + 1e-9
        attached = opened_at is None or refused < 0
        throttle = EMPTY if (down or not attached) else LOADED
        ack = None
        if opened_at is not None and t - opened_at >= ack_after:
            ack = rg.Ack(attempt, 0)
        answer = rule.update(reading(t, vehicle_up, throttle, ack=ack))
        if answer.open_gripper:
            if refused < refuse:
                refused += 1  # this command gets a refusal for an answer
                rule_ack = rg.Ack(answer.attempt, 4)
                log.append((round(t, 2), answer.phase, answer.action, round(vehicle_up, 3)))
                t += 0.05
                answer = rule.update(reading(t, vehicle_up, throttle, ack=rule_ack))
            else:
                opened_at, attempt = t, answer.attempt
        log.append((round(t, 2), answer.phase, answer.action, round(vehicle_up, 3)))
        vehicle_up += answer.rate_up_mps * 0.05
        t += 0.05
        if answer.action in (rr.DONE, rr.FAULT, rr.LAND):
            break
    return log, vehicle_up


def test_lower_then_set_down_slowly_then_let_go_and_see_that_it_went():
    rule = rr.SetDown(PLAN, EMPTY, LOADED)
    log, up = fly(rule, 3.0, 40.0)
    phases = [phase for _t, phase, _a, _u in log]
    assert phases[0] == rr.LOWER and [p for i, p in enumerate(phases) if i == 0 or p != phases[i - 1]] == [rr.LOWER, rr.SET_DOWN, rr.RELEASE, rr.CONFIRM]
    assert log[-1][2] == rr.DONE and rule.released and rule.confirmed
    # Lowered at the final descent's speed to half a metre over the touch, then at the set-down's 0.12 m/s.
    first_slow = next(u for _t, phase, _a, u in log if phase == rr.SET_DOWN)
    assert first_slow == pytest.approx(PLAN.touch_height_m + rr.LOWER_MARGIN_M, abs=0.03)
    assert rule.max_rate_seen[rr.LOWER] <= safety_limits.PHASE_SPEED_CAPS_MPS["final_descent"] and rule.max_rate_seen[rr.SET_DOWN] == pytest.approx(safety_limits.PHASE_SPEED_CAPS_MPS["set_down"])
    # Let go with the vehicle no higher than the touch: far under the 1.5 m, and only after the set-down was felt.
    released_at = next(u for _t, phase, _a, u in log if phase == rr.RELEASE)
    assert released_at <= PLAN.touch_height_m + 0.01 < rg.MAX_RELEASE_HEIGHT_M
    # Then up by half a metre, and the throttle is still the empty hover's: it went.
    assert up == pytest.approx(released_at + rr.CONFIRM_CLIMB_M, abs=0.05)


def test_a_gripper_that_refuses_is_asked_three_times_and_then_the_bucket_is_landed_with():
    patient = rr.SetDown(PLAN, EMPTY, LOADED)
    log, _up = fly(patient, 1.5, 40.0, refuse=2)
    assert log[-1][2] == rr.DONE and patient.release_attempts == 3
    stuck = rr.SetDown(PLAN, EMPTY, LOADED)
    log, _up = fly(stuck, 1.5, 40.0, refuse=3)
    assert log[-1][2] == rr.FAULT and not stuck.released and "3 times" in stuck.last_reason and rr.MAX_RELEASE_TRIES == 3
    # No answer at all is a refusal too, after the wait.
    silent = rr.SetDown(PLAN, EMPTY, LOADED)
    log, _up = fly(silent, 1.5, 60.0, ack_after=1.0e9)
    assert log[-1][2] == rr.FAULT and "no acknowledgement" in silent.last_reason


def test_one_late_height_in_the_confirming_climb_does_not_end_it_unconfirmed():
    """The spec audit: the lowering was given a second of grace for a late rangefinder reading, the climb was not."""
    rule = rr.SetDown(PLAN, EMPTY, LOADED)
    t, up, hiccup = 0.0, 1.0, False
    while t < 30.0:
        throttle = EMPTY if up <= 0.90 or rule.phase == rr.CONFIRM else LOADED
        ack = rg.Ack(rule.attempt, 0) if rule.phase == rr.RELEASE else None
        late = rule.phase == rr.CONFIRM and not hiccup
        hiccup = hiccup or late
        answer = rule.update(reading(t, up, throttle, ack=ack, height_age_s=0.3 if late else 0.05))
        assert not (late and answer.action != rr.HOLD)
        up += answer.rate_up_mps * 0.05
        t += 0.05
        if answer.action == rr.DONE:
            break
    assert hiccup and rule.confirmed
    # A second without a height in the climb: done, and not confirmed.
    lost = rr.SetDown(PLAN, EMPTY, LOADED)
    t, up = 0.0, 1.0
    while t < 30.0:
        in_climb = lost.phase == rr.CONFIRM
        answer = lost.update(reading(t, up, EMPTY if up <= 0.90 or in_climb else LOADED, ack=rg.Ack(lost.attempt, 0) if lost.phase == rr.RELEASE else None, rangefinder_healthy=not in_climb))
        up += answer.rate_up_mps * 0.05
        t += 0.05
        if answer.action == rr.DONE:
            break
    assert lost.released and not lost.confirmed and "lost before the climb" in lost.last_reason


def test_a_bucket_still_on_the_hook_after_the_release_is_set_down_again():
    rule = rr.SetDown(PLAN, EMPTY, LOADED)
    t, up, answer = 0.0, 1.0, None
    while t < 30.0:  # down, felt, released, acknowledged, up
        throttle = EMPTY if up <= 0.90 else LOADED  # the latch did not let go: lifted again, the throttle is the loaded one
        ack = rg.Ack(rule.attempt, 0) if rule.phase == rr.RELEASE else None
        answer = rule.update(reading(t, up, throttle, ack=ack))
        up += answer.rate_up_mps * 0.05
        t += 0.05
        if rule.set_downs == 2 or answer.action in (rr.DONE, rr.FAULT):
            break
    assert rule.set_downs == 2 and rule.phase == rr.SET_DOWN and not rule.confirmed and "still on the hook" in rule.last_reason


def test_no_set_down_felt_by_the_floor_or_no_rangefinder_means_land_first_and_let_go_landed():
    # The throttle never falls (a rope gone slack would not show it either): at the floor it waits, then lands.
    rule = rr.SetDown(PLAN, EMPTY, LOADED)
    log, up = fly(rule, 1.2, 40.0, touch=-1.0)
    assert log[-1][2] == rr.LAND and up == pytest.approx(PLAN.touch_height_m - rr.FLOOR_BELOW_TOUCH_M, abs=0.02) and "not felt" in rule.last_reason
    assert not rule.released
    landed = rule.update(reading(50.0, 0.1, LOADED, landed=True, landed_age_s=0.2))
    assert landed.action == rr.OPEN and landed.open_gripper and landed.phase == rr.RELEASE
    done = rule.update(reading(50.2, 0.1, LOADED, landed=True, landed_age_s=0.2, ack=rg.Ack(landed.attempt, 0)))
    assert done.action == rr.DONE and rule.released and not rule.confirmed  # let go on the ground: there is no climb to confirm by
    # The rangefinder goes: no height to let go on in the air. Land first.
    blind = rr.SetDown(PLAN, EMPTY, LOADED)
    assert blind.update(reading(0.0, 2.0)).action == rr.DESCEND
    for bad in (dict(rangefinder_healthy=False), dict(height_m=None), dict(height_age_s=1.0)):
        rule = rr.SetDown(PLAN, EMPTY, LOADED)
        first = rule.update(reading(0.0, 2.0, **bad))
        assert first.action == rr.HOLD and first.rate_up_mps == 0.0 and "rangefinder" in first.reason  # nothing is lowered on a height that is not known
        assert rule.update(reading(0.95, 2.0, **bad)).action == rr.HOLD
        gone = rule.update(reading(1.0, 2.0, **bad))
        assert gone.action == rr.LAND and "rangefinder" in gone.reason and rr.HEIGHT_LOST_S == 1.0
    # One late reading is not a rangefinder lost (the first sim flight by the rule landed with the bucket for one):
    # it holds for that tick and goes on down when the height is back.
    late = rr.SetDown(PLAN, EMPTY, LOADED)
    assert [late.update(reading(0.05 * tick, 2.0, height_age_s=0.25 if tick == 3 else 0.05)).action for tick in range(6)] == [rr.DESCEND] * 3 + [rr.HOLD] + [rr.DESCEND] * 2
    assert late.update(reading(0.5, 2.0, height_age_s=0.3)).action == rr.HOLD and late.update(reading(1.4, 2.0, height_age_s=0.3)).action == rr.HOLD  # the second's clock started anew
    assert late.update(reading(1.5, 2.0, height_age_s=0.3)).action == rr.LAND
    # Landing, it stays landing: a rangefinder that comes back does not send it down by the rule again.
    blind.update(reading(0.1, 2.0, rangefinder_healthy=False))
    blind.update(reading(1.2, 2.0, rangefinder_healthy=False))
    assert blind.update(reading(1.3, 2.0)).action == rr.LAND


def test_ten_thousand_random_set_downs_never_open_the_gripper_high_or_blind():
    rng = random.Random(11)
    opened = 0
    for _ in range(10_000):
        rule = rr.SetDown(PLAN, EMPTY, LOADED)
        t = 0.0
        for _tick in range(rng.randrange(1, 40)):
            t += rng.choice([0.05, 0.05, 0.05, 0.5])
            height = rng.choice([None, float("nan"), rng.uniform(0.0, 6.0), rng.uniform(0.7, 1.9)])
            healthy = rng.random() < 0.85
            height_age = rng.choice([0.0, 0.05, 0.19, 0.3, None])
            landed = rng.random() < 0.03
            landed_age = rng.choice([0.1, 1.0, 5.0, None])
            answer = rule.update(rr.SetDownReading(t, height, height_age, healthy, rng.choice([EMPTY, LOADED, 0.41, None]), rng.choice([0.0, 0.05, 0.5]), landed, landed_age, None))
            if answer.open_gripper:
                opened += 1
                on_the_ground = landed and landed_age is not None and landed_age <= rg.LANDED_MAX_AGE_S
                if not on_the_ground:
                    assert healthy and height is not None and height == height and height <= 1.5 < 1.8288 and height_age is not None and height_age <= 0.2
                break
    assert opened > 50


def test_recovery_aborts_by_landing_not_by_rtl():
    assert rr.RECOVERY_ABORT_MODE == "LAND"
