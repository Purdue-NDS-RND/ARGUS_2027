"""REC-6: the GPS health rule (drone_mission_core/gps_health.py), on flights made here. No ROS."""

import math
import random

import pytest

from drone_mission_core import gps_health as gh

RATE_HZ = 10.0


def fly(seconds, velocity=(0.0, 0.0), step=None, drift=None, vision=True, vision_noise=0.05, gps_noise=0.15, gps_speed_noise=0.05, seed=1, health=None, start_s=0.0, satellites=14, hdop=0.8,
        static_target=None, quality=200, glitch_flag=None):  # fmt: skip
    """Feed a monitor a flight at RATE_HZ and return (the monitor, the rating at each tick as (t, Rating)).

    `velocity` is the true velocity. `step` = (at_s, east_m, north_m): the GPS position steps by that much and stays.
    `drift` = (from_s, east_mps, north_mps): the GPS position drifts away at that rate, its velocity saying nothing of it.
    `static_target` = True: a target on the ground is tracked, placed with the GPS-driven position (so it shares the GPS's error).
    """
    rng = random.Random(seed)
    health = health or gh.GpsHealth()
    out = []
    east, north = getattr(health, "test_position", (0.0, 0.0))  # a flight goes on from where the last leg left it
    for index in range(int(seconds * RATE_HZ)):
        t = start_s + index / RATE_HZ
        east, north = east + velocity[0] / RATE_HZ, north + velocity[1] / RATE_HZ
        off_e = off_n = 0.0
        if step is not None and t >= step[0]:
            off_e, off_n = step[1], step[2]
        if drift is not None and t >= drift[0]:
            off_e, off_n = off_e + drift[1] * (t - drift[0]), off_n + drift[2] * (t - drift[0])
        health.add_gps(t, east + off_e + rng.gauss(0.0, gps_noise), north + off_n + rng.gauss(0.0, gps_noise), velocity[0] + rng.gauss(0.0, gps_speed_noise), velocity[1] + rng.gauss(0.0, gps_speed_noise))
        health.add_fix_quality(t, 3, satellites, hdop)
        if vision:
            health.add_vision(t, velocity[0] + rng.gauss(0.0, vision_noise), velocity[1] + rng.gauss(0.0, vision_noise), quality)
        if static_target:
            health.add_static_target(t, 12.0 + off_e + rng.gauss(0.0, 0.03), 8.0 + off_n + rng.gauss(0.0, 0.03))  # the target's place moves with the GPS's error
        if glitch_flag is not None:
            health.add_ekf(t, t >= glitch_flag, 0.1, 0.1)
        out.append((t, health.rating(t)))
    health.test_position = (east, north)
    return health, out


def first(ratings, state):
    return next((t for t, rating in ratings if rating.state == state), None)


@pytest.mark.parametrize("vision", [True, False])
@pytest.mark.parametrize("velocity", [(0.0, 0.0), (3.0, -1.0)])
def test_a_five_metre_step_is_untrusted_within_a_second_and_a_half(vision, velocity):
    """The spec's first pass line, hovering and flying, with vision and without it."""
    _health, ratings = fly(30.0, velocity=velocity, step=(20.0, 3.0, 4.0), vision=vision)
    assert first(ratings, gh.UNTRUSTED) is not None and 20.0 <= first(ratings, gh.UNTRUSTED) <= 21.5
    assert all(rating.state == gh.TRUSTED for t, rating in ratings if t < 20.0)
    reasons = " | ".join(reason for t, rating in ratings if t <= 21.5 for reason in rating.reasons)
    assert "beyond what was flown in 1 s (by vision)" in reasons if vision else "beyond what was flown in 1 s (by its own velocity)" in reasons
    # The spec's R3 line: a 10 m step, the same.
    _health, ratings = fly(30.0, velocity=velocity, step=(20.0, -10.0, 0.0), vision=vision)
    assert 20.0 <= first(ratings, gh.UNTRUSTED) <= 21.5


def test_a_half_metre_a_second_drift_is_untrusted_within_five_seconds_with_vision_or_a_target_that_stands_still():
    """The spec's second pass line. A drifting receiver's velocity says nothing: it takes something that does not drift with it."""
    _health, ratings = fly(40.0, drift=(20.0, 0.3, 0.4))  # 0.5 m/s
    assert first(ratings, gh.UNTRUSTED) is not None and 20.0 < first(ratings, gh.UNTRUSTED) <= 25.0
    assert "drifts" in next(rating.reasons[0] for t, rating in ratings if rating.state == gh.UNTRUSTED)
    # Flying at 3 m/s while it drifts: still found.
    _health, ratings = fly(40.0, velocity=(3.0, 0.0), drift=(20.0, 0.0, -0.5))
    assert 20.0 < first(ratings, gh.UNTRUSTED) <= 25.0
    # Without vision, over the recovery's X (which does not move): the X's own apparent drift.
    _health, ratings = fly(40.0, drift=(20.0, 0.3, 0.4), vision=False, static_target=True)
    assert 20.0 < first(ratings, gh.UNTRUSTED) <= 25.0 and "a target that stands still is moving" in " ".join(ratings[-1][1].reasons)
    # With neither, a slow drift cannot be told from flight, and the rule says nothing rather than guess.
    _health, ratings = fly(40.0, drift=(20.0, 0.3, 0.4), vision=False)
    assert first(ratings, gh.UNTRUSTED) is None


def test_thirty_minutes_of_clean_flight_raise_no_alarm():
    """The spec's third pass line: hovering, flying, turning, with the noise a real receiver and the flow have. Several seeds."""
    for seed in range(4):
        health = gh.GpsHealth()
        alarms = suspects = 0
        t = 0.0
        legs = [((0.0, 0.0), 120.0), ((4.0, 0.0), 60.0), ((0.0, -3.0), 60.0), ((-2.0, 2.0), 90.0), ((0.0, 0.0), 120.0)] * 4  # 30 minutes
        for velocity, seconds in legs:
            _health, ratings = fly(seconds, velocity=velocity, health=health, start_s=t, seed=seed * 100 + int(t), gps_noise=0.25, vision_noise=0.1, static_target=velocity == (0.0, 0.0))
            if velocity != (0.0, 0.0):
                health.forget_static_target()
            alarms += sum(1 for _t, rating in ratings if rating.state == gh.UNTRUSTED)
            suspects += sum(1 for _t, rating in ratings if rating.state == gh.SUSPECT)
            t += seconds
        assert t == 1800.0 and alarms == 0, (seed, alarms)
        assert suspects < 0.05 * 18000, (seed, suspects)  # and it is SUSPECT a few hundredths of the time at most (0.25 m of white noise on every fix is harsher than a receiver's)


def test_untrusted_holds_until_ten_seconds_clean_and_says_how_long_it_has_been_clean():
    health, ratings = fly(30.0, step=(20.0, 5.0, 0.0))
    assert ratings[-1][1].state == gh.UNTRUSTED and ratings[-1][1].detected_at_s == pytest.approx(first(ratings, gh.UNTRUSTED))
    # The step is over (the position stays where it stepped to: nothing more moves). Clean time counts up; the state holds for ten seconds of it.
    _health, later = fly(20.0, health=health, start_s=30.0, step=(0.0, 5.0, 0.0), seed=7)
    back = first(later, gh.TRUSTED)
    last_wrong = max(t for t, rating in ratings + later if rating.reasons)
    assert back is not None and back - last_wrong == pytest.approx(gh.CLEAN_S, abs=0.2)
    held = [rating for t, rating in later if t < back]
    assert all(rating.state == gh.UNTRUSTED for rating in held) and held[-1].clean_for_s == pytest.approx(gh.CLEAN_S, abs=0.2) and held[0].clean_for_s < held[-1].clean_for_s
    assert gh.CLEAN_S == 10.0  # the spec's
    # A second fault inside the ten seconds starts the count again.
    health, ratings = fly(30.0, step=(20.0, 5.0, 0.0))
    _health, again = fly(8.0, health=health, start_s=30.0, step=(0.0, 5.0, 0.0), seed=3)
    _health, more = fly(15.0, health=health, start_s=38.0, step=(38.5, 5.0, 6.0), seed=4)
    assert first(more, gh.TRUSTED) is None or first(more, gh.TRUSTED) > 38.5 + gh.CLEAN_S


def test_a_poor_fix_is_suspect_and_never_untrusted_by_itself_and_the_estimators_flag_is_untrusted():
    for change, word in (({"satellites": 5}, "5 satellites, under 8"), ({"hdop": 2.4}, "HDOP 2.4, over 1.4")):
        _health, ratings = fly(10.0, **change)
        assert ratings[-1][1].state == gh.SUSPECT and word in ratings[-1][1].reasons[0] and first(ratings, gh.UNTRUSTED) is None
    health = gh.GpsHealth()
    health.add_fix_quality(0.0, 2, 12, 0.9)
    assert health.rating(0.5).state == gh.SUSPECT and "no GPS position yet" in health.rating(0.5).reasons and "the fix is type 2, under a 3D fix" in health.rating(0.5).reasons
    _health, ratings = fly(10.0, glitch_flag=6.0)
    assert first(ratings, gh.UNTRUSTED) == pytest.approx(6.0) and "the flight controller's estimator says the GPS is glitching" in ratings[-1][1].reasons
    # GPS that stops coming is SUSPECT (the fix's own age), not a jump.
    health, ratings = fly(10.0)
    assert health.rating(12.5).state == gh.SUSPECT and "no GPS position for 1 s" in health.rating(12.5).reasons
    # A receiver's ordinary wander (0.4 m in a step) is not a fault.
    _health, ratings = fly(30.0, step=(20.0, 0.4, 0.0))
    assert first(ratings, gh.UNTRUSTED) is None


def test_vision_without_quality_or_with_gaps_is_not_used_and_numbers_that_are_not_numbers_are_not_kept():
    # Vision that says "moving" with no quality must not turn a hover into a drift.
    _health, ratings = fly(30.0, quality=10, vision_noise=2.0)
    assert first(ratings, gh.UNTRUSTED) is None
    health = gh.GpsHealth()
    health.add_gps(1.0, math.nan, 0.0, 0.0, 0.0)
    health.add_vision(1.0, math.inf, 0.0, 200)
    health.add_static_target(1.0, 0.0, math.nan)
    assert len(health._gps) == 0 and len(health._vision) == 0 and len(health._static) == 0
    health.add_gps(2.0, 0.0, 0.0, 0.0, 0.0)
    health.add_gps(1.5, 9.0, 9.0, 0.0, 0.0)  # an older fix arriving late is not a jump
    assert len(health._gps) == 1
    assert gh.FLAG == "ekf_source_switching"


# --- the source set ------------------------------------------------------------------------------------------


def rated(state, reasons=(), clean_for_s=0.0):
    return gh.Rating(state, tuple(reasons), 0.0, clean_for_s)


def ready():
    """A switch whose flight controller has answered for set 1, as at the start of every run."""
    switch = gh.SourceSwitch()
    assert switch.acknowledged == 1 and not switch.confirmed
    assert switch.update(rated(gh.TRUSTED), 0.0) == (1, "making sure the flight controller is on source set 1")  # whatever an earlier run left it on
    assert switch.update(rated(gh.TRUSTED), 0.05) == (None, None)
    assert switch.answered(1, True, 0.07) == "source set 1 acknowledged by the flight controller" and switch.confirmed
    assert switch.update(rated(gh.TRUSTED), 0.1) == (None, None)
    return switch


def test_untrusted_asks_for_source_set_two_at_once_and_again_until_it_is_acknowledged():
    switch = ready()
    assert switch.update(rated(gh.SUSPECT, ["5 satellites, under 8"]), 0.2, no_gps_set_fed=True) == (None, None)  # SUSPECT switches nothing
    command, say = switch.update(rated(gh.UNTRUSTED, ["the GPS position moved 5.0 m beyond what was flown in 1 s (by vision)"]), 10.0, no_gps_set_fed=True)
    assert command == 2 and say == "GPS UNTRUSTED: asking the flight controller for source set 2 (the GPS position moved 5.0 m beyond what was flown in 1 s (by vision))"
    assert switch.update(rated(gh.UNTRUSTED), 10.5, no_gps_set_fed=True) == (None, None)  # asked half a second ago: not again yet
    assert switch.update(rated(gh.UNTRUSTED), 11.0, no_gps_set_fed=True) == (2, None)  # no answer for a second: asked again
    assert switch.answered(2, False, 11.1) is None and switch.acknowledged == 1  # refused: still on the first set
    assert switch.update(rated(gh.UNTRUSTED), 12.1, no_gps_set_fed=True) == (2, None)
    assert switch.answered(2, True, 12.2) == "source set 2 acknowledged by the flight controller" and switch.acknowledged == 2 and switch.switched_at_s == 12.2
    assert switch.update(rated(gh.UNTRUSTED), 13.5, no_gps_set_fed=True) == (None, None)  # on the set it wants: nothing more to send
    # Once on set 2 it is not left because the flow stops: back is the mission's choice, with GPS trusted.
    assert switch.update(rated(gh.UNTRUSTED), 14.0, no_gps_set_fed=False) == (None, None) and switch.wanted == 2


def test_source_set_two_is_not_asked_for_while_nothing_feeds_it():
    """The flight-safety review: unfed, set 2 is dead reckoning, and the flight controller acknowledges the command all the same."""
    switch = ready()
    command, say = switch.update(rated(gh.UNTRUSTED, ["a jump"]), 10.0, no_gps_set_fed=False)
    assert command is None and say == "GPS UNTRUSTED: source set 2 is not asked for, nothing is feeding it (no optical flow is being sent to the flight controller); it stays on set 1 (a jump)"
    for step in range(1, 100):
        assert switch.update(rated(gh.UNTRUSTED, ["a jump"]), 10.0 + 0.05 * step) == (None, None)  # said once, and nothing is sent (unfed is the default)
    assert switch.wanted == 1 and switch.acknowledged == 1
    # The flow comes (the vehicle is low enough for it): now the set is asked for.
    assert switch.update(rated(gh.UNTRUSTED, ["a jump"]), 15.0, no_gps_set_fed=True)[0] == 2
    # A new fault is declined, and said, anew.
    other = ready()
    assert other.update(rated(gh.UNTRUSTED, ["x"]), 1.0)[1] is not None and other.update(rated(gh.TRUSTED, (), 12.0), 20.0) == (None, None)
    assert "is not asked for" in other.update(rated(gh.UNTRUSTED, ["y"]), 30.0)[1]


def test_the_first_set_is_asked_for_again_only_when_gps_is_trusted_again_and_the_mission_allows_it():
    switch = ready()
    switch.update(rated(gh.UNTRUSTED, ["x"]), 10.0, no_gps_set_fed=True)
    switch.answered(2, True, 10.1)
    assert switch.update(rated(gh.UNTRUSTED, (), 6.0), 16.0, may_go_back=True) == (None, None)  # the monitor still holds UNTRUSTED
    assert switch.update(rated(gh.SUSPECT, ["HDOP 1.6, over 1.4"]), 20.2, may_go_back=True) == (None, None)  # SUSPECT is not trusted
    assert switch.update(rated(gh.TRUSTED, (), 10.0), 20.5, may_go_back=False) == (None, None) and switch.acknowledged == 2  # the mission has not chosen
    command, say = switch.update(rated(gh.TRUSTED, (), 12.0), 22.0, may_go_back=True)
    assert command == 1 and say == "GPS TRUSTED: asking the flight controller for source set 1 (GPS clean for 12 s)"
    assert switch.answered(1, True, 22.1) == "source set 1 acknowledged by the flight controller" and switch.acknowledged == 1
    assert switch.update(rated(gh.UNTRUSTED, ["y"]), 30.0, no_gps_set_fed=True)[0] == 2  # and a new fault switches again


def test_a_command_that_is_never_acknowledged_leaves_the_set_unknown_and_goes_on_being_asked():
    """The flight-safety review: the command may have been taken and its answer lost. The set is then not known, never "still set 1"."""
    switch = ready()
    sent, said = [], []
    for step in range(221):
        command, say = switch.update(rated(gh.UNTRUSTED, ["x"]), 10.0 + 0.05 * step, no_gps_set_fed=True)
        if command is not None:
            sent.append(round(10.0 + 0.05 * step, 2))
        if say:
            said.append(say)
    assert sent == [10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 20.0] and switch.failed and switch.acknowledged is None  # five a second apart, then every five seconds
    assert said[-1] == "source set 2 was not acknowledged after 5 tries: which set the flight controller is on is not known; asking every 5 s" and len(said) == 2
    # GPS trusted again and the way back allowed: set 1 is asked for, because the set is not known to be 1.
    assert switch.update(rated(gh.TRUSTED, (), 12.0), 40.0, may_go_back=True)[0] == 1 and switch.acknowledged is None
    assert switch.answered(1, True, 40.1) == "source set 1 acknowledged by the flight controller" and switch.acknowledged == 1 and not switch.failed
    # A refused answer says nothing about where the flight controller is.
    other = ready()
    assert other.answered(2, False, 1.0) is None and other.acknowledged == 1
    # Set 2 asked for and not answered, and GPS trusted again before the five tries are over: the unanswered command may have been
    # taken, so the set is not known and set 1 is asked for, not believed.
    late = ready()
    assert late.update(rated(gh.UNTRUSTED, ["x"]), 10.0, no_gps_set_fed=True)[0] == 2
    command, say = late.update(rated(gh.TRUSTED, (), 12.0), 10.5, may_go_back=True)
    assert command == 1 and late.acknowledged is None and say == "GPS TRUSTED: asking the flight controller for source set 1 (GPS clean for 12 s)"


def test_a_fault_that_arrives_while_the_way_back_is_being_asked_for_is_not_lost():
    """The spec audit: set 1 commanded, GPS UNTRUSTED before the answer came. The tracker said 2, the flight controller went to 1, and nothing was sent."""
    switch = ready()
    switch.update(rated(gh.UNTRUSTED, ["x"]), 10.0, no_gps_set_fed=True)
    switch.answered(2, True, 10.1)
    assert switch.update(rated(gh.TRUSTED, (), 12.0), 30.0, may_go_back=True)[0] == 1  # the way back is asked for
    command, say = switch.update(rated(gh.UNTRUSTED, ["a new fault"]), 30.05, no_gps_set_fed=True)  # and a fault arrives before the answer
    assert command == 2 and "asking the flight controller for source set 2" in say and switch.acknowledged is None  # the set-1 command is out unanswered: the set is not known, and set 2 is asked for at once
    said = switch.answered(1, True, 30.1)  # the flight controller did go to set 1
    assert switch.acknowledged == 1 and said == "source set 1 acknowledged by the flight controller, and set 2 is the one wanted now: asking again"
    assert switch.update(rated(gh.UNTRUSTED, ["a new fault"]), 30.15, no_gps_set_fed=True) == (2, "making sure the flight controller is on source set 2")  # on the next tick, not a second later
    assert switch.answered(2, True, 30.2) == "source set 2 acknowledged by the flight controller" and switch.acknowledged == 2


def test_a_mission_that_ends_off_the_first_set_asks_for_it_or_says_why_not():
    """The flight-safety review: what flies next flies on GPS without asking the monitor."""
    assert ready().leave(gh.TRUSTED, 5.0) == (None, None)
    switch = ready()
    switch.update(rated(gh.UNTRUSTED, ["x"]), 10.0, no_gps_set_fed=True)
    switch.answered(2, True, 10.1)
    assert switch.leave(gh.UNTRUSTED, 12.0) == (None, "the mission ends with the flight controller on source set 2 and the GPS UNTRUSTED: it is left there, and nothing after this should be flown on GPS")
    assert switch.leave(gh.TRUSTED, 30.0) == (1, "the mission ends with the flight controller on source set 2: asking for source set 1 (once: nothing asks again after this)")
    assert switch.answered(1, True, 30.1) == "source set 1 acknowledged by the flight controller" and switch.leave(gh.TRUSTED, 31.0) == (None, None)
    unknown = gh.SourceSwitch()  # never answered at all
    for step in range(120):
        unknown.update(rated(gh.TRUSTED), 0.05 * step)
    assert unknown.acknowledged is None and unknown.leave(gh.SUSPECT, 6.0)[1].startswith("the mission ends with the flight controller on a source set that is not known: asking for source set 1")


def test_a_jump_is_not_judged_without_a_velocity_and_a_clock_apart_from_the_stamps_empties_nothing():
    """The flight-safety review: a missing velocity was taken as standing still (a fast transit read as a jump), and the windows were cut by the caller's clock."""
    health = gh.GpsHealth()
    for step in range(60):  # 6 m/s north, no velocity heard, no vision
        health.add_gps(100.0 + 0.2 * step, 0.0, 6.0 * 0.2 * step)
        rating = health.rating(0.2 * step)
        assert rating.state != gh.UNTRUSTED
    assert rating.state == gh.SUSPECT and "no GPS velocity and no vision to judge a jump by" in rating.reasons
    # The same flight with its velocity, stamped 100 s away from the caller's clock: clean, and a 5 m step in it is found.
    health = gh.GpsHealth()
    for step in range(60):
        health.add_gps(100.0 + 0.2 * step, 0.0, 6.0 * 0.2 * step + (5.0 if step >= 40 else 0.0), 0.0, 6.0)
        rating = health.rating(0.2 * step)
        if step < 40:
            assert rating.state == gh.TRUSTED, (step, rating)
    assert rating.state == gh.UNTRUSTED and rating.detected_at_s is not None and rating.detected_at_s - 8.0 <= 1.5
    # Stamps that step back by more than a second start the history again instead of dropping every fix until they catch up.
    health.add_gps(50.0, 0.0, 0.0, 0.0, 0.0)
    assert len(health._gps) == 1 and health._gps[-1].t == 50.0
    health.add_gps(49.9, 0.0, 0.0, 0.0, 0.0)  # a late message: dropped
    assert len(health._gps) == 1
    # No fix lately is said by arrival, not by the stamps.
    assert "no GPS position for 1 s" not in health.rating(20.0).reasons and "no GPS position for 1 s" in health.rating(21.5).reasons


def test_the_rating_costs_little_at_the_flow_cameras_rate():
    """The flight-safety review: the rating runs in the 50 ms tick. Ninety flow samples a second for six seconds, ten fixes a second."""
    import time

    health = gh.GpsHealth()
    for step in range(540):
        health.add_vision(step / 90.0, 0.5, 0.0, 200)
    for step in range(60):
        health.add_gps(step / 10.0, 0.05 * step, 0.0, 0.5, 0.0)
    health.rating(5.9)
    started = time.perf_counter()
    for _ in range(20):
        health.rating(5.9)
    assert (time.perf_counter() - started) / 20 < 0.005


def test_the_estimators_variance_is_suspect_over_its_line_and_the_command_number_is_the_one_sent():
    health = gh.GpsHealth()
    health.add_gps(0.0, 0.0, 0.0, 0.0, 0.0)
    health.add_ekf(0.5, False, 0.5, gh.VARIANCE_SUSPECT + 0.05)
    assert health.rating(0.6).state == gh.SUSPECT and "the estimator's variance is 0.85" in health.rating(0.6).reasons
    health.add_ekf(0.7, False, 0.5, gh.VARIANCE_SUSPECT - 0.05)
    assert not any("variance" in reason for reason in health.rating(0.8).reasons) and gh.VARIANCE_SUSPECT == 0.8
    from pathlib import Path

    context = (Path(__file__).resolve().parents[1] / "drone_mission_core" / "mission_context.py").read_text()
    assert "from .gps_health import SET_EKF_SOURCE_SET" in context and "request.command = SET_EKF_SOURCE_SET" in context and gh.SET_EKF_SOURCE_SET == 42007


# --- the executor's side -------------------------------------------------------------------------------------


def test_the_feed_places_the_raw_fix_in_metres_and_hears_the_flows_samples():
    feed = gh.GpsHealthFeed()
    lat0, lon0 = 34.175, -118.4825
    for step in range(300):  # hovering for 30 s, then the receiver steps 5 m north
        t = step * 0.1
        north = 5.0 if t >= 20.0 else 0.0
        feed.on_velocity(t, 0.0, 0.0)
        feed.on_fix(t, lat0 + north / gh.METRES_PER_DEGREE, lon0)
        feed.on_quality(t, 3, 14, 80)
        feed.on_flow_status('{"stamp_ns": %d, "quality": 200, "why": "", "dt_s": 0.2, "east_mps": 0.0, "north_mps": 0.0, "height_m": 2.5}' % int(t * 1e9))
        rating = feed.rating(t)
        if rating.state == gh.UNTRUSTED:
            break
    assert rating.state == gh.UNTRUSTED and 20.0 <= t <= 21.5 and "by vision" in " ".join(rating.reasons)
    # A fix that says it has none, a place that is not one, and a status that is not JSON are not kept.
    quiet = gh.GpsHealthFeed()
    quiet.on_fix(0.0, lat0, lon0, has_fix=False)
    quiet.on_fix(0.1, 0.0, 0.0)
    quiet.on_fix(0.2, math.nan, lon0)
    quiet.on_flow_status("not json")
    quiet.on_flow_status('{"quality": 200}')
    assert len(quiet.health._gps) == 0 and len(quiet.health._vision) == 0
    # Found in the sim: on the way home at 15 m the flow read 0.3 m/s off and the GPS was called drifting. Above 5 m the flow is not the judge.
    quiet.on_flow_status('{"stamp_ns": 5000000000, "quality": 255, "why": "", "dt_s": 0.2, "east_mps": 0.4, "north_mps": 0.0, "height_m": 14.8}')
    assert len(quiet.health._vision) == 0
    quiet.on_flow_status('{"stamp_ns": 5100000000, "quality": 255, "why": "", "dt_s": 0.2, "east_mps": 0.0, "north_mps": 0.0, "height_m": 4.9}')
    assert len(quiet.health._vision) == 1 and gh.VISION_MAX_HEIGHT_M == 5.0
    for not_a_status in ("[1, 2]", "null", "7", '"sent"', "{"):
        quiet.on_flow_status(not_a_status)  # nothing raised into the executor's spin, nothing kept
    assert len(quiet.health._vision) == 1
    # Source set 2 is fed only while a sample with quality is said to have been sent to the flight controller, and lately.
    assert not quiet.flow_sent(5.2)  # none of those said "sent"
    quiet.on_flow_status('{"stamp_ns": 6000000000, "quality": 0, "why": "bare", "dt_s": 0.2, "east_mps": 0.0, "north_mps": 0.0, "height_m": 3.0, "sent": true}')
    assert not quiet.flow_sent(6.1)  # sent, without quality
    quiet.on_flow_status('{"stamp_ns": 6200000000, "quality": 180, "why": "", "dt_s": 0.2, "east_mps": 0.0, "north_mps": 0.0, "height_m": 12.0, "sent": true}')
    assert quiet.flow_sent(6.3) and not quiet.flow_sent(7.3) and gh.FLOW_SENT_MAX_AGE_S == 1.0
    quiet.on_quality(1.0, 3, 5, 65535)  # eph not known: not an HDOP
    assert "5 satellites, under 8" in quiet.rating(1.0).reasons and not any("HDOP" in reason for reason in quiet.rating(1.0).reasons)
    from pathlib import Path

    node = (Path(__file__).resolve().parents[3] / "src/drone_target_cv/drone_target_cv/flow_odometry_node.py").read_text()
    assert f'STATUS_TOPIC = "{gh.FLOW_STATUS_TOPIC}"' in node


def test_nothing_is_subscribed_with_the_flag_off_and_four_topics_with_it_on(monkeypatch):
    import sys
    import types
    from types import SimpleNamespace

    made = []
    node = SimpleNamespace(create_subscription=lambda kind, topic, callback, qos: made.append((topic, callback)) or topic, get_logger=lambda: SimpleNamespace(info=lambda text: None))
    assert gh.attach_gps_health(node, [SimpleNamespace(features={"package_recovery": True}), SimpleNamespace(features={"ekf_source_switching": False})]) is None and made == []
    for name, module in (("geometry_msgs", types.SimpleNamespace()), ("geometry_msgs.msg", types.SimpleNamespace(TwistStamped=object)), ("mavros_msgs", types.SimpleNamespace()),
                         ("mavros_msgs.msg", types.SimpleNamespace(GPSRAW=object)), ("rclpy", types.SimpleNamespace()), ("rclpy.qos", types.SimpleNamespace(qos_profile_sensor_data="sensor data")),
                         ("sensor_msgs", types.SimpleNamespace()), ("sensor_msgs.msg", types.SimpleNamespace(NavSatFix=object)), ("std_msgs", types.SimpleNamespace()),
                         ("std_msgs.msg", types.SimpleNamespace(String=object))):  # fmt: skip
        monkeypatch.setitem(sys.modules, name, module)
    feed = gh.attach_gps_health(node, [SimpleNamespace(features={"package_recovery": True, "ekf_source_switching": True})])
    assert isinstance(feed, gh.GpsHealthFeed) and [topic for topic, _callback in made] == [gh.RAW_FIX_TOPIC, gh.RAW_VELOCITY_TOPIC, gh.RAW_QUALITY_TOPIC, gh.FLOW_STATUS_TOPIC]
    stamp = SimpleNamespace(sec=5, nanosec=0)
    made[1][1](SimpleNamespace(header=SimpleNamespace(stamp=stamp), twist=SimpleNamespace(linear=SimpleNamespace(x=1.0, y=2.0))))
    made[0][1](SimpleNamespace(header=SimpleNamespace(stamp=stamp), latitude=34.0, longitude=-118.0, status=SimpleNamespace(status=0)))
    made[2][1](SimpleNamespace(header=SimpleNamespace(stamp=stamp), fix_type=3, satellites_visible=12, eph=90))
    assert len(feed.health._gps) == 1 and feed.health._gps[0].v_east == 1.0 and feed.health._quality == (5.0, 3, 12, 0.9)
