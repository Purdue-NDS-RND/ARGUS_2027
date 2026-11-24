"""The recovery flight's states (REC-2), flown against a small world: a vehicle that goes where it is told, a bucket
on the ground, a gripper that answers, a throttle that knows what hangs on the hook."""

import copy
import dataclasses
import math
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_mission_core import recovery_flight as rf  # noqa: E402
from drone_mission_core import recovery_grip as rg  # noqa: E402
from drone_mission_core import recovery_return as rr  # noqa: E402
from drone_mission_core.features import get_feature_flag  # noqa: E402

LAT, LON = 40.4237, -86.9212
HOOK, HANDLE, SUSPENSION = 0.55, 0.3, 0.25
EMPTY, LOADED = 0.40, 0.45
SQUARE = rf.expanding_square(15.0, 25.0)


def plan(**given):
    base = dict(hook_below_m=HOOK, suspension_length_m=SUSPENSION, vehicle_kg=4.0, load_kg=1.0, handle_up_m=HANDLE, search_waypoints=SQUARE)
    base.update(given)
    return rf.Plan(**base)


class World:
    """Everything outside the flight: where things are, what is seen, what the gripper and the throttle do."""

    def __init__(self, bucket=(40.0, 30.0), pickup=None, see_above_agl=0.0, lose_below_hook_m=None, grab_misses=0, open_result=0, gps=True, home_up=0.0,
                 land_refusals=0, sigma=0.1, lose_for_good=False, rangefinder=True, close_result=0, x_only=False, boundary=None):  # fmt: skip
        self.t = 100.0
        self.pos = [0.0, 0.0, 15.0]
        self.bucket = list(bucket)  # where the bucket stands (or hangs)
        self.pickup = tuple(pickup or bucket)
        self.see_above_agl, self.lose_below_hook_m = see_above_agl, lose_below_hook_m
        self.grab_misses, self.open_result, self.gps, self.home_up = grab_misses, open_result, gps, home_up
        self.attached = False
        self.touch_agl = None  # the vehicle's height at which the hanging bucket's bottom is on the ground
        self.pending_ack = None
        self.ack = None
        self.landing = False
        self.anchor = rr.Anchor(0.0, 0.0, LAT, LON)
        self.commands, self.opens, self.closes, self.modes = [], [], [], []
        self.throttle_override = None
        self.min_hook_height = 99.0
        self.seen = []  # for each tick, whether the bucket was in the readings
        self.land_refusals, self.sigma, self.lose_for_good, self.rangefinder = land_refusals, sigma, lose_for_good, rangefinder
        self.close_result, self.x_only, self.boundary = close_result, x_only, boundary
        self.lost_once = False
        self.max_agl_after_loss = 0.0

    def agl(self):
        return self.pos[2]

    def hook_height(self):
        return self.agl() - HOOK - HANDLE

    def standing(self):
        return not self.attached or self.agl() <= self.touch_agl + 1.0e-9

    def readings(self):
        e, n, up = self.pos
        target = None
        offset = math.hypot(self.bucket[0] - e, self.bucket[1] - n)
        visible = self.agl() >= self.see_above_agl and offset <= 0.45 * max(self.agl(), 1.0)
        if self.lose_below_hook_m is not None and self.hook_height() < self.lose_below_hook_m:
            visible = False
        if self.lose_below_hook_m is not None and self.hook_height() < self.lose_below_hook_m:
            self.lost_once = True
        if self.lost_once:
            self.max_agl_after_loss = max(self.max_agl_after_loss, self.agl())
        if self.lost_once and self.lose_for_good:
            visible = False
        if visible and not self.attached:
            target = rf.Target(self.bucket[0], self.bucket[1], self.sigma, 0.05, True, not self.x_only)
        self.seen.append(target is not None)
        throttle = self.throttle_override if self.throttle_override is not None else (EMPTY if self.standing() else LOADED)
        landed = self.landing and (self.agl() <= 0.0 or (self.attached and self.agl() <= self.touch_agl))
        return rf.Readings(
            now_s=self.t, e=e, n=n, up=up, yaw_rad=math.pi / 2.0, agl_m=self.agl(), agl_from_rangefinder=self.rangefinder, range_age_s=0.05 if self.rangefinder else None,
            mode=rf.LAND if self.landing else "GUIDED", boundary=self.boundary,
            throttle=throttle, throttle_age_s=0.05, landed=landed, landed_age_s=0.1, fix=self.anchor.to_global(e, n) if self.gps else None,
            home_up_m=self.home_up, pickup_en=self.pickup, target=target, ack=self.ack,
        )  # fmt: skip

    def apply(self, command, dt=0.05):
        self.commands.append(command)
        if command.mode == rf.LAND:
            self.modes.append(command.mode)
            self.land_refusals -= 1
            self.landing = self.land_refusals < 0  # (the first `land_refusals` requests are not taken)
        if command.gripper == rf.CLOSE:
            self.closes.append((round(self.t, 2), command.attempt))
            near = math.hypot(self.bucket[0] - self.pos[0], self.bucket[1] - self.pos[1]) <= 0.12 and abs(self.hook_height()) <= 0.15
            if near and self.grab_misses <= 0:
                self.attached, self.touch_agl = True, self.agl()
            self.grab_misses -= 1
            self.pending_ack = (self.t + 0.1, rg.Ack(command.attempt, self.close_result))
        if command.gripper == rf.OPEN:
            self.opens.append((round(self.t, 2), round(self.agl(), 3), self.standing()))
            if self.open_result == 0:
                self.attached = False
            self.pending_ack = (self.t + 0.1, rg.Ack(command.attempt, self.open_result))
        if self.pending_ack is not None and self.t >= self.pending_ack[0]:
            self.ack, self.pending_ack = self.pending_ack[1], None
        goal = None
        if self.landing:
            floor = self.touch_agl if self.attached else 0.0
            self.pos[2] = max(floor, self.pos[2] - 0.5 * dt)
        elif command.frame == rf.LOCAL:
            goal = (command.e, command.n, command.up)
        elif command.frame == rf.GLOBAL:
            east, north = self.anchor.to_local(command.lat, command.lon)
            goal = (east, north, command.alt + self.home_up)
        if goal is not None:
            for axis, speed in ((0, 5.0), (1, 5.0), (2, 2.0)):
                delta = goal[axis] - self.pos[axis]
                self.pos[axis] += max(-speed * dt, min(speed * dt, delta))
            if self.attached:
                self.pos[2] = max(self.pos[2], self.touch_agl - 0.2)  # the bucket is on the ground under it
        if self.attached:
            self.bucket = [self.pos[0], self.pos[1]]
        self.min_hook_height = min(self.min_hook_height, self.hook_height())
        self.t += dt


def fly(flight, world, seconds=900.0, until=None):
    command = None
    for _ in range(int(seconds * 20)):
        command = flight.update(world.readings())
        world.apply(command)
        if command.status != rf.RUNNING or (until is not None and until(flight, world)):
            break
    return command


WALK = [rf.CHECKS, rf.TRANSIT, rf.ACQUIRE, rf.APPROACH, rf.CENTRE, rf.CONE_DESCENT, rf.ENGAGE, rf.LIFT, rf.VERIFY_ATTACH, rf.CLIMB, rf.RETURN, rf.SET_DOWN, rf.CLEAR, rf.LANDING, rf.COMPLETE]


def test_the_state_walk_from_the_checks_to_the_bucket_set_down_at_the_takeoff():
    world, flight = World(), rf.RecoveryFlight(plan())
    last = fly(flight, world)
    assert last.status == rf.SUCCESS and flight.transitions == WALK and flight.attempts == 1 and flight.released
    assert math.hypot(*world.bucket) < 0.35 and not world.attached  # the bucket stands at the takeoff point
    assert len(world.closes) == 1 and len(world.opens) == 1 and world.opens[0][2] is True  # let go once, standing on the ground
    assert world.opens[0][1] <= rg.MAX_RELEASE_HEIGHT_M < 1.83 and world.modes == [rf.LAND]  # the one landing: beside the bucket, at the end
    # GPS trusted: the way back was by global setpoints, and only the way back.
    frames = [command.frame for command in world.commands]
    assert flight.global_setpoints > 20 and frames.count(rf.GLOBAL) == flight.global_setpoints
    assert math.hypot(world.pos[0] - 2.0, world.pos[1]) < 0.35 and world.pos[2] == 0.0  # landed clear, two metres to the side
    # With land_after off the mission ends hovering there, for the sequence's next mission.
    hover_world, hover = World(), rf.RecoveryFlight(plan(land_after=False))
    assert fly(hover, hover_world).status == rf.SUCCESS and hover.transitions[-2:] == [rf.CLEAR, rf.COMPLETE] and hover_world.modes == [] and hover_world.pos[2] > 3.0


def test_untrusted_gps_never_commands_a_global_setpoint():
    for world, the_plan in ((World(), plan(gps_untrusted=True)), (World(gps=False, pickup=(40.0, 30.0)), plan())):
        flight = rf.RecoveryFlight(the_plan)
        last = fly(flight, world)
        assert last.status == rf.SUCCESS and flight.global_setpoints == 0 and all(command.frame != rf.GLOBAL for command in world.commands)
        assert math.hypot(*world.bucket) < 0.35
    # Trusted, and no height to give a global setpoint (home's height in the local frame not known): local too.
    world = World(home_up=None)
    assert fly(rf.RecoveryFlight(plan()), world).status == rf.SUCCESS and all(command.frame != rf.GLOBAL for command in world.commands)
    # A global setpoint's height is given in home's terms.
    world = World(home_up=0.19)
    flight = rf.RecoveryFlight(plan())
    assert fly(flight, world).status == rf.SUCCESS
    heights = {round(command.up - command.alt, 3) for command in world.commands if command.frame == rf.GLOBAL}
    assert heights == {0.19}


def test_no_descent_below_point_eight_after_the_handle_is_lost():
    world, flight = World(lose_below_hook_m=0.7), rf.RecoveryFlight(plan())
    fly(flight, world, until=lambda f, w: f.state == rf.LOST_TARGET)
    assert flight.state == rf.LOST_TARGET and "lost" in flight.reason
    # From the tick the handle went out of the readings (the hook under 0.8 m), the commanded height never went
    # down again: the cone held its 1.5 s and said lost, and the hook stayed where the loss found it.
    last_seen = max(i for i, seen in enumerate(world.seen) if seen)
    after = [command.up for command in world.commands[last_seen + 1 :]]
    assert len(after) >= 30 and all(b >= a - 1.0e-9 for a, b in zip(after, after[1:])) and after[0] >= world.commands[last_seen].up - 0.006
    assert 0.6 < world.min_hook_height < 0.71
    # It climbs, sees the bucket again from above, and tries again: three descents in all, then the mission fails,
    # having first gone up and two metres to the side: the landing that follows a failure is not onto the bucket.
    last = fly(flight, world)
    assert last.status == rf.FAILURE and flight.attempts == 3 and "3 descents" in last.reason and flight.transitions[-2:] == [rf.RETREAT, rf.FAILED]
    assert world.closes == [] and world.min_hook_height > 0.6  # nothing was closed, and the hook never went lower
    assert math.hypot(world.pos[0] - world.bucket[0], world.pos[1] - world.bucket[1]) > 1.6 and world.agl() > 3.0


def test_a_detector_that_sees_nothing_below_three_metres_ends_in_three_lost_descents():
    """REC-1 as built: no X or bucket below about 3 m. The mission does not go down blind."""
    world, flight = World(see_above_agl=3.0), rf.RecoveryFlight(plan())
    last = fly(flight, world)
    assert last.status == rf.FAILURE and flight.attempts == 3 and rf.LOST_TARGET in flight.transitions
    assert world.min_hook_height > 3.0 - HOOK - HANDLE - 0.3 and world.closes == []  # the hook stayed two metres up


def test_a_pickup_point_that_is_off_is_found_by_the_expanding_square():
    world, flight = World(bucket=(40.0, 30.0), pickup=(58.0, 42.0)), rf.RecoveryFlight(plan())
    last = fly(flight, world)
    assert last.status == rf.SUCCESS and flight.transitions[:5] == [rf.CHECKS, rf.TRANSIT, rf.ACQUIRE, rf.SEARCH, rf.APPROACH]
    # The square's speed: the commanded point moves at 4 m/s, no faster.
    assert rf.SEARCH in flight.transitions and math.hypot(*world.bucket) < 0.35
    # Nothing there at all: the search ends, and the mission fails without having descended.
    empty = World(bucket=(500.0, 500.0), pickup=(40.0, 30.0))
    flight = rf.RecoveryFlight(plan())
    searching, last = [], None
    for _ in range(int(900.0 * 20)):
        last = flight.update(empty.readings())
        if flight.state == rf.SEARCH:
            searching.append(last)
        empty.apply(last)
        if last.status != rf.RUNNING:
            break
    assert last.status == rf.FAILURE and "search ended" in last.reason
    # The square was flown at 20 m, its commanded point moving at 4 m/s and no faster, round every corner.
    walked = [math.hypot(b.e - a.e, b.n - a.n) / 0.05 for a, b in zip(searching[1:], searching[2:])]
    assert max(walked) == pytest.approx(4.0, abs=1.0e-6) and sum(1 for speed in walked if speed > 3.99) > 0.9 * len(walked)
    assert empty.pos[2] == pytest.approx(20.0, abs=0.3) and abs(len(walked) * 0.05 - 300.0 / 4.0) < 8.0  # 300 m of square: 75 s


def test_the_expanding_square_covers_the_radius():
    square = rf.expanding_square(15.0, 25.0)
    assert square[:4] == ((15.0, 0.0), (15.0, 15.0), (-15.0, 15.0), (-15.0, -15.0))
    assert max(max(abs(east), abs(north)) for east, north in square) >= 25.0
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(((0.0, 0.0),) + square, square))
    assert 100.0 < length <= 300.0  # at 4 m/s, 75 s with a 15 m spacing (the mission takes the spacing from the picture's footprint)
    with pytest.raises(ValueError):
        rf.expanding_square(0.0, 25.0)


def test_a_hook_that_comes_up_empty_goes_back_to_centre_and_tries_again():
    world, flight = World(grab_misses=1), rf.RecoveryFlight(plan())
    last = fly(flight, world)
    assert last.status == rf.SUCCESS and flight.attempts == 2 and len(world.closes) == 2
    walk = flight.transitions
    first_verify = walk.index(rf.VERIFY_ATTACH)
    assert walk[first_verify + 1 : first_verify + 4] == [rf.ABORT_CLIMB, rf.CENTRE, rf.CONE_DESCENT] and world.opens and len(world.opens) == 1
    # Three empty hooks: the mission fails (beside the bucket, not over it), and the gripper was never opened in the air.
    world, flight = World(grab_misses=9), rf.RecoveryFlight(plan())
    last = fly(flight, world)
    assert last.status == rf.FAILURE and flight.attempts == 3 and world.opens == [] and "3 descents" in last.reason
    assert rf.RETREAT in flight.transitions and math.hypot(world.pos[0] - world.bucket[0], world.pos[1] - world.bucket[1]) > 1.6


def test_a_gripper_that_will_not_open_lands_with_the_bucket_and_nothing_is_dropped():
    world, flight = World(open_result=4), rf.RecoveryFlight(plan())
    last = fly(flight, world)
    assert last.status == rf.FAILURE and rf.CARRYING_FAULT in flight.transitions and world.modes == [rf.LAND]
    assert world.attached and len(world.opens) == 3 and all(standing for _t, _agl, standing in world.opens)  # asked three times, each with the bucket standing
    assert "landed with the bucket" in last.reason


def test_the_bucket_lost_on_the_way_fails_the_mission_where_it_is():
    world, flight = World(), rf.RecoveryFlight(plan())
    fly(flight, world, until=lambda f, w: f.state == rf.RETURN and w.t - f._state_since_s > 2.0)
    world.throttle_override = EMPTY  # the load is gone
    world.attached = False
    last = fly(flight, world)
    assert last.status == rf.FAILURE and "bucket was lost" in last.reason and world.opens == []


def test_out_of_time_fails_before_the_bucket_is_on_and_lands_with_it_after():
    world, flight = World(bucket=(500.0, 500.0), pickup=(40.0, 30.0)), rf.RecoveryFlight(plan(time_budget_s=20.0))
    last = fly(flight, world)
    assert last.status == rf.FAILURE and "time budget" in last.reason
    # With the bucket on the hook a state that outstays its limit is a carrying fault: land with it.
    world, flight = World(), rf.RecoveryFlight(plan())
    fly(flight, world, until=lambda f, w: f.state == rf.CLIMB)
    world.pos[2] = world.pos[2]  # (the vehicle is then held where it is: the climb never ends)
    stuck = copy.copy(world)
    for _ in range(int(40.0 * 20)):
        readings = world.readings()
        command = flight.update(readings)
        world.t += 0.05
        if command.mode == rf.LAND:
            break
    assert flight.state == rf.CARRYING_FAULT and command.mode == rf.LAND and "CLIMB lasted" in flight.reason and stuck.opens == []


def test_swing_shaping_walks_the_way_back_over_whole_pendulum_periods():
    shaped_plan = plan(swing_shaping=True, carry_speed_mps=3.0, pendulum_m=0.72)
    world, flight = World(), rf.RecoveryFlight(shaped_plan)
    fly(flight, world, until=lambda f, w: f.state == rf.SET_DOWN)
    back = [command for command in world.commands if command.frame == rf.GLOBAL]
    steps = [math.hypot(b.e - a.e, b.n - a.n) / 0.05 for a, b in zip(back, back[1:])]
    leg = rg.ShapedLeg(50.0, 3.0, 0.72)
    assert max(steps) <= 3.0 + 1.0e-6 and steps[0] < 0.2 and steps[-1] < 0.2  # from rest, to rest, never over the cap
    assert abs(len(back) * 0.05 - leg.duration_s) < 1.5
    # Without the flag the commanded point is walked at the carry speed, 1 m/s.
    world, flight = World(), rf.RecoveryFlight(plan())
    fly(flight, world, until=lambda f, w: f.state == rf.SET_DOWN)
    back = [command for command in world.commands if command.frame == rf.GLOBAL]
    assert max(math.hypot(b.e - a.e, b.n - a.n) / 0.05 for a, b in zip(back, back[1:])) == pytest.approx(1.0, abs=1.0e-6)


def test_ten_thousand_random_set_downs_never_open_the_gripper_above_six_feet_or_blind():
    world, flight = World(), rf.RecoveryFlight(plan())
    fly(flight, world, until=lambda f, w: f.state == rf.SET_DOWN)
    assert flight.state == rf.SET_DOWN
    rng = random.Random(21)
    opened = 0
    for _ in range(10_000):
        one = copy.deepcopy(flight)
        t = world.t
        for _tick in range(rng.randrange(1, 30)):
            t += rng.choice([0.05, 0.05, 0.05, 0.4])
            agl = rng.choice([None, rng.uniform(0.0, 6.0), rng.uniform(0.3, 1.9)])
            from_rangefinder = rng.random() < 0.85
            range_age = rng.choice([0.0, 0.05, 0.19, 0.3, None])
            landed = rng.random() < 0.03
            landed_age = rng.choice([0.1, 1.0, 5.0, None])
            command = one.update(rf.Readings(now_s=t, e=0.0, n=0.0, up=agl if agl is not None else 1.0, yaw_rad=0.0, agl_m=agl, agl_from_rangefinder=from_rangefinder, range_age_s=range_age,
                                             throttle=rng.choice([EMPTY, LOADED, 0.41, None]), throttle_age_s=rng.choice([0.0, 0.05, 0.5]), landed=landed, landed_age_s=landed_age,
                                             fix=None, home_up_m=0.0, pickup_en=(40.0, 30.0)))  # fmt: skip
            if command.gripper == rf.OPEN:
                opened += 1
                on_the_ground = landed and landed_age is not None and landed_age <= rg.LANDED_MAX_AGE_S
                if not on_the_ground:
                    assert from_rangefinder and agl is not None and agl <= 1.5 < 1.83 and range_age is not None and range_age <= 0.2, (agl, from_rangefinder, range_age)
                break
            assert command.gripper is None
    assert opened > 50


def test_the_plan_is_checked_and_flags_with_nothing_behind_them_are_refused():
    config = {"hook_below_m": 0.55, "suspension_length_m": 0.25, "vehicle_kg": 4.0, "load_kg": 1.0, "return_target": {"latlon": [LAT, LON]}, "engage": {"mode": "plunge", "plunge_depth_m": 0.04}}
    made = rf.plan_from_config(config, {rf.FLAG: True, rf.SWING_FLAG: True}, get_feature_flag, SQUARE, 0.8)
    assert made.swing_shaping and not made.gps_untrusted and made.return_target.kind == rr.LATLON and made.engage.plunge_depth_m == 0.04 and made.footprint_per_metre == 0.8
    assert rf.plan_from_config(config, {rf.UNTRUSTED_FLAG: True}, get_feature_flag).gps_untrusted
    assert set(rf.UNBUILT_FLAGS) == {"recovery_telemetry", "visual_attach_check"}
    for flag in rf.UNBUILT_FLAGS:
        with pytest.raises(ValueError, match=flag):
            rf.plan_from_config(config, {flag: True}, get_feature_flag)
    for bad in ({"vehicle_kg": 0.0}, {"carry_speed_mps": 9.0}, {"approach_height_m": 3.0}, {"carry_height_m": 0.5}, {"max_attempts": 0}, {"hook_forward_left_m": [0.1]}, {"engage": {"mode": "grab"}}, {"engage": {"depth": 1}}, {"return_target": "home"}):
        with pytest.raises(ValueError):
            rf.plan_from_config({**config, **bad}, {}, get_feature_flag)
    with pytest.raises(ValueError, match="hook_below_m"):
        rf.plan_from_config({"suspension_length_m": 0.25, "vehicle_kg": 4.0, "load_kg": 1.0}, {}, get_feature_flag)


def test_no_position_or_no_pickup_place_fails_on_the_spot_before_anything_moves():
    flight = rf.RecoveryFlight(plan())
    assert flight.update(rf.Readings(now_s=0.0)).status == rf.RUNNING
    assert flight.update(rf.Readings(now_s=10.1)).status == rf.FAILURE
    # Coordinates the wrong way round are thousands of kilometres off: not flown to.
    world, flight = World(pickup=(40.0, 30.0)), rf.RecoveryFlight(plan())
    world.pickup = (600_000.0, -600_000.0)
    last = fly(flight, world, seconds=5.0)
    assert last.status == rf.FAILURE and "not flown to" in last.reason and len(world.commands) == 1
    world, flight = World(pickup=(40.0, 30.0)), rf.RecoveryFlight(plan())
    world.pickup = None
    last = fly(flight, world, seconds=15.0)
    assert last.status == rf.FAILURE and "could not be placed" in last.reason and all(math.hypot(c.e, c.n) < 1.0e-6 for c in world.commands if c.frame == rf.LOCAL)


# --- the reviews' cases ---------------------------------------------------------------------------


def test_a_landing_that_is_not_taken_is_asked_for_again_until_it_is():
    """The flight-safety review: one request, refused or lost, left the mission running for ever with no setpoint."""
    world, flight = World(open_result=4, land_refusals=3), rf.RecoveryFlight(plan())
    last = fly(flight, world)
    assert last.status == rf.FAILURE and "landed with the bucket" in last.reason and len(world.modes) == 4  # three not taken, the fourth was
    times = [i for i, command in enumerate(world.commands) if command.mode == rf.LAND]
    assert all(abs((b - a) * 0.05 - rf.LAND_RETRY_S) < 0.11 for a, b in zip(times, times[1:]))
    # Never taken at all: after two minutes the mission fails, and the executor's own abort takes over.
    world, flight = World(open_result=4, land_refusals=10_000), rf.RecoveryFlight(plan())
    last = fly(flight, world)
    assert last.status == rf.FAILURE and "no landing seen 120 s" in last.reason and world.attached and len(world.opens) == 3
    # The landing beside the bucket at the end is asked for again in the same way.
    world, flight = World(land_refusals=2), rf.RecoveryFlight(plan())
    assert fly(flight, world).status == rf.SUCCESS and len(world.modes) == 3
    # Once LAND has been seen it is not asked for again: a pilot who takes the vehicle out of it is not put back.
    world, flight = World(), rf.RecoveryFlight(plan())
    fly(flight, world, until=lambda f, w: f.state == rf.LANDING and w.landing and w.t - f._state_since_s > 0.5)
    asked = len(world.modes)
    for _ in range(int(10.0 * 20)):
        world.t += 0.05
        r = world.readings()
        assert flight.update(rf.Readings(**{**r.__dict__, "mode": "LOITER", "landed": False})).mode is None
    assert asked == 1


def test_near_the_ground_the_height_is_the_rangefinders_or_nothing_is_lowered():
    """The flight-safety review: a height above home was taken for the height above the ground in the cone."""
    world, flight = World(rangefinder=False), rf.RecoveryFlight(plan())
    last = fly(flight, world)
    assert last.status == rf.FAILURE and world.closes == [] and flight.attempts == 3
    assert world.min_hook_height > 3.0 - 0.35  # CENTRE's height was the lowest the hook came: the cone never took it down


def test_the_x_alone_finds_and_centres_and_the_cone_wants_the_buckets_own_track():
    world, flight = World(x_only=True), rf.RecoveryFlight(plan())
    last = fly(flight, world)
    assert last.status == rf.FAILURE and rf.CONE_DESCENT in flight.transitions and world.closes == [] and world.min_hook_height > 3.0 - 0.35


def test_a_descent_outside_the_cone_needs_a_confirmed_track_of_this_moment():
    world, flight = World(), rf.RecoveryFlight(plan())
    fly(flight, world, until=lambda f, w: f.state == rf.APPROACH and w.agl() < 12.0)
    height = world.agl()
    readings = world.readings()
    coasting = rf.Target(readings.target.e, readings.target.n, 0.1, 0.9, False)  # the track is coasting: nothing has hit it for 0.9 s
    for _ in range(20):
        world.t += 0.05
        r = world.readings()
        command = flight.update(rf.Readings(**{**r.__dict__, "target": coasting}))
        world.pos[2] += max(-0.1, min(0.1, command.up - world.pos[2]))
    assert world.agl() >= height - 0.11 and flight.state == rf.APPROACH  # held, not lowered, while the track coasted


def test_the_lost_climb_goes_as_high_as_the_tracks_uncertainty_asks_and_no_higher_than_ten_metres():
    # footprint 0.9 m per metre of height: three sigma either way of a 1 m sigma is 6 m, seen whole from 6.67 m.
    for sigma, top in ((1.0, 6.0 / 0.9), (3.0, rf.LOST_MAX_HEIGHT_M)):
        world, flight = World(lose_below_hook_m=0.7, lose_for_good=True, sigma=sigma), rf.RecoveryFlight(plan())
        fly(flight, world, until=lambda f, w: f.state == rf.SEARCH)
        assert flight.state == rf.SEARCH and world.max_agl_after_loss == pytest.approx(top, abs=0.35), sigma


def test_a_close_that_is_not_acknowledged_is_still_lifted_and_checked_and_an_unknown_hook_is_landed_with():
    # Refused, and the handle taken all the same: the throttle says so, and the bucket is carried home.
    world, flight = World(close_result=4), rf.RecoveryFlight(plan())
    last = fly(flight, world)
    assert last.status == rf.SUCCESS and len(world.closes) == 1 and "not acknowledged" in " ".join([flight.reason] + [c.reason for c in world.commands])
    # The attach check cannot be made (no throttle heard in the hold): the hook may hold the bucket. Land with it.
    world, flight = World(), rf.RecoveryFlight(plan())
    fly(flight, world, until=lambda f, w: f.state == rf.LIFT)
    for _ in range(int(60.0 * 20)):
        r = world.readings()
        command = flight.update(rf.Readings(**{**r.__dict__, "throttle": None, "throttle_age_s": None}))
        world.apply(command)
        if command.status != rf.RUNNING:
            break
    assert rf.CARRYING_FAULT in flight.transitions and rf.ABORT_CLIMB not in flight.transitions and world.opens == [] and "could not be made" in flight.reason


def test_a_position_missing_for_a_tick_is_waited_out_and_for_a_second_is_not():
    world, flight = World(), rf.RecoveryFlight(plan())
    fly(flight, world, until=lambda f, w: f.state == rf.TRANSIT and w.t > 103.0)
    assert flight.update(rf.Readings(now_s=world.t + 0.05)).status == rf.RUNNING and flight.update(rf.Readings(now_s=world.t + 0.9)).status == rf.RUNNING
    world.t += 0.95
    assert flight.update(world.readings()).status == rf.RUNNING  # back: the second starts anew
    assert flight.update(rf.Readings(now_s=world.t + 0.5)).status == rf.RUNNING and flight.update(rf.Readings(now_s=world.t + 1.6)).status == rf.FAILURE


def test_the_pickup_and_the_search_keep_five_metres_inside_the_flight_boundary():
    field = ((-20.0, -20.0), (60.0, -20.0), (60.0, 50.0), (-20.0, 50.0))
    world, flight = World(boundary=field, bucket=(500.0, 500.0), pickup=(40.0, 30.0)), rf.RecoveryFlight(plan())
    last = fly(flight, world)
    assert last.status == rf.FAILURE and "search ended" in last.reason
    flown = [(command.e, command.n) for command in world.commands if command.frame == rf.LOCAL]
    assert max(e for e, _n in flown) <= 55.0 + 1.0e-6 and max(n for _e, n in flown) <= 45.0 + 1.0e-6 and min(n for _e, n in flown) >= -15.0 - 1.0e-6
    outside = World(boundary=field, pickup=(58.0, 30.0))
    last = fly(rf.RecoveryFlight(plan()), outside, seconds=5.0)
    assert last.status == rf.FAILURE and "flight boundary" in last.reason and len(outside.commands) == 1


def test_a_sweep_and_a_setting_the_mission_does_not_have_are_refused_when_the_plan_is_made():
    config = {"hook_below_m": 0.55, "suspension_length_m": 0.25, "vehicle_kg": 4.0, "load_kg": 1.0}
    with pytest.raises(ValueError, match="plunge only"):
        rf.plan_from_config({**config, "engage": {"mode": "sweep"}}, {}, get_feature_flag)
    with pytest.raises(ValueError, match="handle_height_m"):
        rf.plan_from_config({**config, "handle_height_m": 0.3}, {}, get_feature_flag)  # (it is handle_up_m)


# --- the abort's mode (the lead's rule of 23 November 2026, as built: recovery_return.py, "Abort") -----------------


def abort_modes(flight, world, seconds=900.0, until=None):
    """Fly, noting the abort mode in each state the flight passes through."""
    seen = {}
    for _ in range(int(seconds * 20)):
        command = flight.update(world.readings())
        seen.setdefault(flight.state, set()).add(flight.abort_mode())
        world.apply(command)
        if command.status != rf.RUNNING or (until is not None and until(flight, world)):
            break
    return seen, command


def test_the_abort_is_rtl_until_the_gripper_closes_on_the_handle_and_a_landing_for_the_rest_of_the_mission():
    world, flight = World(), rf.RecoveryFlight(plan())
    assert flight.abort_mode() == "RTL" and not flight.gripper_has_closed
    seen, last = abort_modes(flight, world)
    assert last.status == rf.SUCCESS
    for state in (rf.TRANSIT, rf.ACQUIRE, rf.APPROACH, rf.CENTRE, rf.CONE_DESCENT):  # (CHECKS is over within the first tick)
        assert seen[state] == {"RTL"}, state
    assert seen[rf.ENGAGE] == {"RTL", "LAND"}  # RTL on the way down the last of it, LAND from the close
    # With the bucket on; then let go but not yet seen to be off; then standing at the takeoff point, where an RTL would come down.
    for state in (rf.LIFT, rf.VERIFY_ATTACH, rf.CLIMB, rf.RETURN, rf.SET_DOWN, rf.CLEAR, rf.LANDING, rf.COMPLETE):
        assert seen[state] == {"LAND"}, state
    assert flight.abort_mode() == "LAND" and flight.gripper_has_closed


def test_a_hook_the_attach_check_called_empty_is_still_landed_with():
    # The throttle said nothing was taken; the gripper is still closed, and the check can be wrong.
    world, flight = World(grab_misses=1), rf.RecoveryFlight(plan())
    seen, _ = abort_modes(flight, world, until=lambda f, w: f.attempts == 2 and f.state == rf.CONE_DESCENT)
    assert flight.attempts == 2 and seen[rf.CONE_DESCENT] == {"RTL", "LAND"}  # RTL on the first descent, LAND on the second
    assert flight.abort_mode() == "LAND"


def test_a_failure_before_any_close_aborts_by_rtl_and_every_failure_after_one_by_landing():
    # Out of time before anything has been closed on: the executor's abort brings the vehicle home.
    world, flight = World(bucket=(500.0, 500.0), pickup=(40.0, 30.0)), rf.RecoveryFlight(plan(time_budget_s=20.0))
    assert fly(flight, world).status == rf.FAILURE and flight.state == rf.FAILED
    assert flight.abort_mode() == "RTL" and not flight.gripper_has_closed
    assert flight.update(world.readings()).status == rf.FAILURE and flight.abort_mode() == "RTL"  # and it stays so
    # A gripper that will not open: landed with the bucket, failed, and still a landing whatever the executor asks next.
    world, flight = World(open_result=4), rf.RecoveryFlight(plan())
    assert fly(flight, world).status == rf.FAILURE and flight.state == rf.FAILED and flight.abort_mode() == "LAND"
    # In the air with the bucket on and no way on (the carrying fault itself): LAND, never RTL.
    world, flight = World(), rf.RecoveryFlight(plan())
    fly(flight, world, until=lambda f, w: f.state == rf.CLIMB)
    for _ in range(int(40.0 * 20)):
        flight.update(world.readings())
        world.t += 0.05
        assert flight.abort_mode() == "LAND"
    assert flight.state == rf.CARRYING_FAULT
    # The bucket thought lost on the way (the throttle's word, with the gripper still closed).
    world, flight = World(), rf.RecoveryFlight(plan())
    fly(flight, world, until=lambda f, w: f.state == rf.RETURN and w.t - f._state_since_s > 2.0)
    world.throttle_override = EMPTY
    world.attached = False
    assert fly(flight, world).status == rf.FAILURE and flight.abort_mode() == "LAND"


def test_a_failure_after_the_release_lands_where_it_is_and_not_on_the_bucket_at_home():
    """The spec audit's row: with the bucket set down at the takeoff point, an RTL would come down on it."""
    world, flight = World(), rf.RecoveryFlight(plan(time_budget_s=10_000.0))
    fly(flight, world, until=lambda f, w: f.state == rf.CLEAR)
    assert flight.released and flight.abort_mode() == "LAND"
    frozen = world.readings()
    for _ in range(int(60.0 * 20)):  # CLEAR outstays its limit: the vehicle is held where it is
        world.t += 0.05
        command = flight.update(dataclasses.replace(frozen, now_s=world.t))
        if command.status != rf.RUNNING:
            break
    assert flight.state in (rf.FAILED, rf.CARRYING_FAULT) and flight.abort_mode() == "LAND"


def test_with_no_position_to_fly_home_on_the_abort_is_a_landing_whatever_is_on_the_hook():
    """The flight-safety review's blocker: an RTL asked for without a position is refused and asked for again for ever."""
    # No pose: the mission fails for want of a position, with nothing on the hook, and the abort is LAND.
    world, flight = World(), rf.RecoveryFlight(plan())
    fly(flight, world, until=lambda f, w: f.state == rf.APPROACH)
    assert flight.abort_mode() == "RTL"
    blind = dataclasses.replace(world.readings(), e=None, n=None, up=None, yaw_rad=None)
    for _ in range(40):
        world.t += 0.05
        command = flight.update(dataclasses.replace(blind, now_s=world.t))
        assert flight.abort_mode() == "LAND"  # from the first tick without one, before the mission gives up
    assert command.status == rf.FAILURE and "no position" in command.reason and not flight.gripper_has_closed
    assert flight.update(dataclasses.replace(world.readings(), now_s=world.t + 1.0)).status == rf.FAILURE and flight.abort_mode() == "LAND"  # kept from the tick it failed on
    # No GPS fix: a pose, but nothing an RTL can be flown on. It is RTL again when the fix is.
    world, flight = World(), rf.RecoveryFlight(plan())
    fly(flight, world, until=lambda f, w: f.state == rf.APPROACH)
    flight.update(dataclasses.replace(world.readings(), fix=None))
    assert flight.abort_mode() == "LAND"
    # ...and RTL again only when the fix has been back for two seconds: one that comes and goes does not flip it.
    flips = []
    for tick in range(80):
        world.t += 0.05
        dropped = tick < 20 and tick % 2 == 0  # the first second: every other fix is missing
        flight.update(dataclasses.replace(world.readings(), now_s=world.t, fix=None if dropped else world.readings().fix))
        flips.append(flight.abort_mode())
    assert flips[:20] == ["LAND"] * 20 and flips.count("RTL") > 0 and flips[-1] == "RTL"
    assert flips.index("RTL") == pytest.approx(19 + 40, abs=2)  # two seconds after the last missing fix
    assert "LAND" not in flips[flips.index("RTL") :]
    # GPS not trusted (the flag): never RTL, from before the first tick.
    world, flight = World(pickup=(40.0, 30.0)), rf.RecoveryFlight(plan(gps_untrusted=True))
    assert flight.abort_mode() == "LAND"
    seen, last = abort_modes(flight, world)
    assert last.status == rf.SUCCESS and set().union(*seen.values()) == {"LAND"}


# --- visual homing (REC-7b) ---------------------------------------------------------------------------------


def fly_seeing_home(flight, world, fix_for, seconds=900.0):
    """The flight with a homing fix in its readings: `fix_for(flight, world)` is the fix seen this tick, or None."""
    import dataclasses

    command = None
    for _ in range(int(seconds * 20)):
        command = flight.update(dataclasses.replace(world.readings(), homing=fix_for(flight, world)))
        world.apply(command)
        if command.status != rf.RUNNING:
            break
    return command


def seen_within(place, radius_m=3.0, age_s=0.05, jitter=None):
    def fix_for(flight, world):
        if flight.state != rf.RETURN or math.hypot(world.pos[0] - place[0], world.pos[1] - place[1]) > radius_m:
            return None
        shift = (0.0, 0.0) if jitter is None else jitter(world.t)
        return rr.HomingFix(place[0] + shift[0], place[1] + shift[1], age_s)

    return fix_for


def test_a_homing_fix_takes_the_set_down_over_in_the_last_metres_and_without_the_flag_it_is_not_looked_at():
    marker = (1.5, -1.0)  # where the return point really is: 1.8 m from where the takeoff's coordinates say
    world, flight = World(), rf.RecoveryFlight(plan(homing=True))
    assert fly_seeing_home(flight, world, seen_within(marker, radius_m=6.0)).status == rf.SUCCESS
    assert math.hypot(world.bucket[0] - marker[0], world.bucket[1] - marker[1]) < 0.35 and flight.homing_takes == 1
    assert flight._legs == 1  # a place taken by homing begins the way back anew without spending the legs a carry is allowed
    off_world, off = World(), rf.RecoveryFlight(plan())
    assert fly_seeing_home(off, off_world, seen_within(marker, radius_m=6.0)).status == rf.SUCCESS
    assert math.hypot(*off_world.bucket) < 0.35 and off._legs == 1 and off.homed is None and off.homing_takes == 0  # the takeoff point, as ever
    made = rf.plan_from_config({"hook_below_m": 0.55, "suspension_length_m": 0.25, "vehicle_kg": 4.0, "load_kg": 1.0}, {rf.FLAG: True, "marker_homing": True}, get_feature_flag, SQUARE, 0.8)
    assert made.homing and rf.HOMING_FLAGS == ("marker_homing", "keyframe_homing") and not set(rf.HOMING_FLAGS) & set(rf.UNBUILT_FLAGS)
    base = {"hook_below_m": 0.55, "suspension_length_m": 0.25, "vehicle_kg": 4.0, "load_kg": 1.0}
    assert rf.plan_from_config(base, {rf.FLAG: True, "keyframe_homing": True}, get_feature_flag, SQUARE, 0.8).homing
    # Each kind of homing knows one place, and is refused for a return target it does not know.
    assert rf.plan_from_config({**base, "return_target": {"marker": 0, "near": [LAT, LON]}}, {rf.FLAG: True, "marker_homing": True}, get_feature_flag, SQUARE, 0.8).homing
    for flag, target, word in (("keyframe_homing", {"latlon": [LAT, LON]}, "know only the takeoff point"), ("keyframe_homing", {"marker": 0, "near": [LAT, LON]}, "know only the takeoff point"),
                               ("marker_homing", {"latlon": [LAT, LON]}, "a home marker lies at the takeoff point or is the return target itself")):  # fmt: skip
        with pytest.raises(ValueError, match=word):
            rf.plan_from_config({**base, "return_target": target}, {rf.FLAG: True, flag: True}, get_feature_flag, SQUARE, 0.8)


@pytest.mark.parametrize(
    "place, age_s, boundary, why",
    [
        ((4.5, 1.0), 0.05, None, "further than four metres from where the return point was thought to be"),
        ((1.5, -1.0), 5.0, None, "an old fix"),
        ((1.5, -1.0), float("nan"), None, "not a number"),
        ((-2.0, 0.0), 0.05, ((-6.0, -60.0), (80.0, -60.0), (80.0, 80.0), (-6.0, 80.0)), "under five metres inside the flight boundary"),
    ],
)
def test_a_homing_fix_that_is_far_old_or_outside_the_boundary_is_not_believed(place, age_s, boundary, why):
    world, flight = World(), rf.RecoveryFlight(plan(homing=True))
    world.boundary = boundary
    assert fly_seeing_home(flight, world, seen_within(place, radius_m=8.0, age_s=age_s)).status == rf.SUCCESS, why
    assert flight.homed is None and math.hypot(*world.bucket) < 0.35, why


def test_one_sighting_decides_nothing_and_a_place_is_taken_only_after_a_second_of_agreement():
    """The flight-safety review: a fix believed on one tick was kept for the rest of the mission."""
    marker = (1.5, -1.0)
    for seen_for_s, taken in ((0.05, False), (0.9, False), (1.2, True)):
        world, flight = World(), rf.RecoveryFlight(plan(homing=True))
        first = []

        def briefly(flight_, world_):
            fix = seen_within(marker, radius_m=6.0)(flight_, world_)
            if fix is None:
                return None
            first.append(world_.t)
            return fix if world_.t - first[0] <= seen_for_s else None

        assert fly_seeing_home(flight, world, briefly).status == rf.SUCCESS
        assert (flight.homing_takes == 1) is taken, seen_for_s
        target = marker if taken else (0.0, 0.0)
        assert math.hypot(world.bucket[0] - target[0], world.bucket[1] - target[1]) < 0.35, seen_for_s
    assert rf.HOMING_AGREE_S == 1.0


def test_a_place_taken_is_given_up_when_it_is_not_seen_again_or_is_seen_where_it_cannot_be():
    marker = (3.0, -2.0)
    # Seen for two seconds from nine metres out and never again: eight seconds later the way back is by the coordinates again.
    world, flight = World(), rf.RecoveryFlight(plan(homing=True, carry_speed_mps=0.5))  # (a slow carry: still on its way when the eight seconds are up)
    first, given_up = [], []

    def then_gone(flight_, world_):
        fix = seen_within(marker, radius_m=9.5)(flight_, world_)
        if fix is not None:
            first.append(world_.t)
        if first and flight_.homed is None and flight_.homing_takes == 1 and not given_up:
            given_up.append(world_.t)
        return fix if first and world_.t - first[0] <= 2.0 else None

    assert fly_seeing_home(flight, world, then_gone).status == rf.SUCCESS
    assert flight.homing_takes == 1 and flight.homed is None and math.hypot(*world.bucket) < 0.35
    assert given_up and given_up[0] - first[0] == pytest.approx(2.0 + rf.HOMING_KEEP_S, abs=0.2) and rf.HOMING_KEEP_S == 8.0
    # Seen where the return point cannot be (the same track, gone six metres off): what it said before is not kept.
    world, flight = World(), rf.RecoveryFlight(plan(homing=True))
    first = []

    def then_wild(flight_, world_):
        fix = seen_within(marker, radius_m=9.5)(flight_, world_)
        if fix is None:
            return None
        first.append(world_.t)
        return fix if world_.t - first[0] <= 2.0 else rr.HomingFix(marker[0] + 6.0, marker[1], 0.05)

    assert fly_seeing_home(flight, world, then_wild).status == rf.SUCCESS
    assert flight.homing_takes == 1 and flight.homed is None and math.hypot(*world.bucket) < 0.35


def test_the_place_taken_is_kept_against_jitter_and_an_estimate_that_keeps_moving_cannot_turn_the_carry_into_a_fault():
    marker = (1.5, -1.0)
    world, flight = World(), rf.RecoveryFlight(plan(homing=True))
    jitter = lambda t: (0.1 * math.sin(37.0 * t), 0.1 * math.cos(23.0 * t))  # noqa: E731  (an estimate settling: a tenth of a metre about its place)
    assert fly_seeing_home(flight, world, seen_within(marker, radius_m=6.0, jitter=jitter)).status == rf.SUCCESS
    assert flight.homing_takes == 1 and math.hypot(world.bucket[0] - marker[0], world.bucket[1] - marker[1]) < 0.45  # one place, not one a tick
    # A fix that moves a metre while it is flown to is taken instead, after its own second of agreement.
    moved_world, moved = World(), rf.RecoveryFlight(plan(homing=True))
    first_seen = []

    def moving(flight_, world_):
        fix = seen_within(marker, radius_m=8.0)(flight_, world_)
        if fix is None:
            return None
        first_seen.append(world_.t)
        return fix if world_.t - first_seen[0] < 2.0 else rr.HomingFix(marker[0] + 1.0, marker[1], 0.05)

    assert fly_seeing_home(moved, moved_world, moving).status == rf.SUCCESS
    assert moved.homing_takes == 2 and math.hypot(moved_world.bucket[0] - (marker[0] + 1.0), moved_world.bucket[1] - marker[1]) < 0.35
    # The flight-safety review: an estimate that hops half a metre every second and a half spends three takes and no more, and the
    # carry ends with the bucket set down (at the last place taken), not in a fault with the bucket on the hook.
    hop_world, hopping = World(), rf.RecoveryFlight(plan(homing=True, carry_speed_mps=0.3))
    hop = lambda t: (0.5 * (int(t / 1.5) % 4), 0.0)  # noqa: E731
    assert fly_seeing_home(hopping, hop_world, seen_within((1.0, -1.0), radius_m=9.0, jitter=hop)).status == rf.SUCCESS
    assert hopping.homing_takes == rf.HOMING_MAX_TAKES == 3 and hopping._legs == 1 and not hop_world.attached
    assert rf.HOMING_RELATCH_M == 0.3 and rf.HOMING_MAX_CORRECTION_M == 4.0


def test_arrived_at_the_coordinates_with_fixes_still_agreeing_the_set_down_waits_for_the_place_and_no_longer_than_three_seconds():
    """Found in the sim: the marker came into view two seconds before the coordinates were reached, and the bucket was set down 1.5 m from it
    while its fixes were still serving their second of agreement."""
    marker = (-1.2, -0.9)  # a metre and a half beyond the coordinates on the vehicle's own way in: nearest when it arrives
    world, flight = World(), rf.RecoveryFlight(plan(homing=True))
    assert fly_seeing_home(flight, world, seen_within(marker, radius_m=1.7)).status == rf.SUCCESS  # in view only from 0.2 m before the coordinates are reached
    assert flight.homing_takes == 1 and flight._homing_wait_since is not None and math.hypot(world.bucket[0] - marker[0], world.bucket[1] - marker[1]) < 0.35
    # Fixes that never settle (a place that hops every half second) hold the set-down three seconds and no more: then the coordinates.
    hop_world, hopping = World(), rf.RecoveryFlight(plan(homing=True))
    hop = lambda t: (0.5 * (int(t / 0.5) % 2), 0.0)  # noqa: E731
    arrived = []

    def hopping_fix(flight_, world_):
        if flight_.state == rf.SET_DOWN and not arrived:
            arrived.append(world_.t)
        return seen_within(marker, radius_m=1.7, jitter=hop)(flight_, world_)

    assert fly_seeing_home(hopping, hop_world, hopping_fix).status == rf.SUCCESS
    assert hopping.homing_takes == 0 and math.hypot(*hop_world.bucket) < 0.35
    assert arrived[0] - hopping._homing_wait_since == pytest.approx(rf.HOMING_WAIT_S, abs=0.2) and rf.HOMING_WAIT_S == 3.0
    # Fixes that stop as the vehicle arrives are not waited for.
    gone_world, gone = World(), rf.RecoveryFlight(plan(homing=True))
    first = []

    def gone_fix(flight_, world_):
        fix = seen_within(marker, radius_m=1.7)(flight_, world_)
        if fix is not None:
            first.append(world_.t)
        return fix if first and world_.t - first[0] <= 0.3 else None

    assert fly_seeing_home(gone, gone_world, gone_fix).status == rf.SUCCESS and gone.homing_takes == 0 and math.hypot(*gone_world.bucket) < 0.35


def test_fixes_must_keep_coming_to_agree_and_too_many_changes_of_the_way_back_end_homing_without_a_fault(monkeypatch):
    marker = (3.0, -2.0)
    # Seen for 0.7 s, not for a second, seen for 0.7 s again: never a second of agreement.
    world, flight = World(), rf.RecoveryFlight(plan(homing=True, carry_speed_mps=0.5))
    first = []

    def in_bursts(flight_, world_):
        fix = seen_within(marker, radius_m=9.5)(flight_, world_)
        if fix is None:
            return None
        first.append(world_.t)
        since = world_.t - first[0]
        return fix if since <= 0.7 or 1.7 <= since <= 2.4 else None

    assert fly_seeing_home(flight, world, in_bursts).status == rf.SUCCESS and flight.homing_takes == 0 and math.hypot(*world.bucket) < 0.35
    assert rf.HOMING_GAP_S == 0.6
    # The flight-safety review: a goal that keeps going to and from the place taken (the vehicle at the edge of the last ten metres)
    # began the way back anew each time, uncounted. Past eight such changes homing is off for the rest of the way back.
    flips = [0]
    real = rr.return_goal

    def flipping(target, **more):
        if more.get("homing") is not None:
            flips[0] += 1
            if (flips[0] // 20) % 2:  # a second with the place as the goal, a second without, and so on
                more = {**more, "homing": None}
        return real(target, **more)

    monkeypatch.setattr(rf.back, "return_goal", flipping)
    world, flight = World(), rf.RecoveryFlight(plan(homing=True, carry_speed_mps=0.3))
    assert fly_seeing_home(flight, world, seen_within(marker, radius_m=9.5)).status == rf.SUCCESS
    assert flight._homing_off and flight.homed is None and flight._legs == 1 and flight._homing_leg_changes > rf.HOMING_MAX_LEG_CHANGES == 8
    assert math.hypot(*world.bucket) < 0.35 and not world.attached  # set down at the coordinates, not landed with as a fault
