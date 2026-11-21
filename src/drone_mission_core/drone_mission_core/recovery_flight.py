"""The recovery flight (REC-2): the states of the `package_recovery` mission. No ROS.

`RecoveryFlight(plan).update(readings)` is one tick: what the vehicle knows
goes in (`Readings`), what it should do comes out (`Command`: a setpoint in
the local or the global frame, a gripper command, a mode, and whether the
mission goes on). The mission (drone_mission_demo/missions/
package_recovery_mission.py) is the thin side: it fills the readings from
the mission context and hands the command to it.

The states, in order:

  CHECKS         a position, a height above the ground and the pickup's place
                 in the local frame; the takeoff's place in both frames is
                 taken here, for the way back.
  TRANSIT        to the pickup's coordinates at `transit_height_m` (15 m).
  ACQUIRE        hold there for a confirmed track of the bucket or the X.
  SEARCH         none: an expanding square at `search_height_m` (20 m) and
                 `search_speed_mps` (4 m/s) about the pickup's coordinates.
  APPROACH       over the track at `approach_height_m` (6 m).
  CENTRE         the hook over the handle, `centre_above_handle_m` (3 m)
                 above it; the empty hover's throttle is taken here.
  CONE_DESCENT   down the cone (cone_descent.py): descend, hold, climb, lost
                 or abort, on the hook's own error and height.
  ENGAGE         recovery_grip.Engage: past the handle, the gripper closed,
                 its acknowledgement.
  LIFT           up by the suspension's length and half a metre.
  VERIFY_ATTACH  recovery_grip.AttachCheck on the hold's throttle. Not
                 attached: back to CENTRE.
  CLIMB          to `carry_height_m`.
  RETURN         recovery_return.return_goal: a global setpoint with GPS
                 trusted, a local one otherwise, walked along the leg (over
                 whole pendulum periods with swing shaping on).
  SET_DOWN       recovery_return.SetDown: lower, set down, release through
                 the guard, confirm.
  CLEAR          up and `clear_aside_m` (2 m) to the side.
  LANDING        the flight controller's own landing there, beside the bucket
                 (`land_after`, on unless the sequence says otherwise); then
                 COMPLETE.

and to the side:

  ABORT_CLIMB    the cone gave the attempt up, or the engage was refused:
                 back up to CENTRE's height and try again.
  LOST_TARGET    the track is gone (the cone's own 1.5 s hold is served
                 first): climb until the picture's footprint covers three
                 standard deviations of where it was, 10 m at most; a track
                 again is CENTRE again, none is a SEARCH.
  CARRYING_FAULT with the bucket on the hook and no way on: nothing is let
                 go, the vehicle lands with it, and the mission fails once it
                 is down.

Three attempts at the handle in all (`max_attempts`); then the mission
fails. Before the bucket is on the hook the mission also fails when its time
budget runs out or a state outstays its limit; with the bucket on, either is
a CARRYING_FAULT. The bucket lost on the way (recovery_grip.LoadWatch) fails
the mission where it is.

GPS is taken as trusted while there is a fix, unless the plan says
`gps_untrusted`: then no global setpoint is ever commanded. (REC-6's monitor
will say more than "there is a fix".)

Every height in the plan is above the ground. The commanded height is walked
in the local frame from where the vehicle is, by the difference between the
height above the ground it has and the one it should have, so nothing here
needs to know where the ground is in the local frame.

The standard library and this package's rules.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, List, Optional, Sequence, Tuple

from . import cone_descent as cone_rule
from . import recovery_grip as grip
from . import recovery_return as back
from .safety_limits import capped_speed

FLAG = "package_recovery"
UNTRUSTED_FLAG = "recovery_gps_untrusted"
SWING_FLAG = grip.SWING_FLAG
# Flags the spec names whose behaviour is another feature's and not built yet: a sequence that turns one on is
# refused on the ground, not flown as if it were there.
UNBUILT_FLAGS = {
    "recovery_telemetry": "the recovery telemetry is not built",
    "ekf_source_switching": "EKF source switching is REC-6's",
    "visual_attach_check": "nothing tracks the bucket in the hook camera yet (REC-1)",
    "marker_homing": "marker homing is REC-7b's",
    "keyframe_homing": "keyframe homing is REC-7b's",
}

(CHECKS, TRANSIT, ACQUIRE, SEARCH, APPROACH, CENTRE, CONE_DESCENT, ENGAGE, LIFT, VERIFY_ATTACH, CLIMB, RETURN, SET_DOWN, CLEAR, COMPLETE) = (
    "CHECKS", "TRANSIT", "ACQUIRE", "SEARCH", "APPROACH", "CENTRE", "CONE_DESCENT", "ENGAGE", "LIFT", "VERIFY_ATTACH", "CLIMB", "RETURN", "SET_DOWN", "CLEAR", "COMPLETE",
)  # fmt: skip
ABORT_CLIMB, LOST_TARGET, CARRYING_FAULT, FAILED, LANDING = "ABORT_CLIMB", "LOST_TARGET", "CARRYING_FAULT", "FAILED", "LANDING"
CARRYING = (CLIMB, RETURN, SET_DOWN, CARRYING_FAULT)  # the bucket is on the hook

RUNNING, SUCCESS, FAILURE = "running", "success", "failure"
LOCAL, GLOBAL = back.LOCAL, back.GLOBAL
CLOSE, OPEN = "close", "open"
LAND = "LAND"

ARRIVE_M = 2.0  # at a transit or search waypoint
OVER_M = 1.0  # over the track, for the approach
CENTRED_M = 0.3  # the hook this close over the handle, and
CENTRED_S = 1.0  # for this long, before the cone is entered
HEIGHT_M = 0.3  # at a height
ACQUIRE_S = 5.0  # held over the pickup's coordinates for a track before searching
APPROACH_SPEED_MPS = 2.0
CLIMB_RATE_MPS = 0.5
LIFT_RATE_MPS = 0.3
ENGAGE_BELOW_M = 0.1  # the cone hands over to the engage with the hook this far above the handle
TRACK_GONE_S = 1.5  # with no track for this long outside the cone: LOST_TARGET (the cone has its own 1.5 s)
LOST_MAX_HEIGHT_M = 10.0
LOST_WAIT_S = 5.0  # at the top of the lost climb, for a track, before searching
VERIFY_EXTRA_S = 5.0  # the attach check's hold may run this much over, waiting for fresh throttle
ARRIVED_BACK_M = 0.3
MAX_TICK_S = 0.2  # a longer gap between ticks walks a setpoint no further than this much time
CHECKS_S = 10.0
MAX_PICKUP_DISTANCE_M = 1000.0  # a pickup further than this from the vehicle is a mistake in the coordinates
# How long a state may last before it is given up (seconds). RETURN's and SEARCH's are worked out from their legs.
STATE_LIMIT_S = {TRANSIT: 120.0, APPROACH: 60.0, CENTRE: 60.0, CONE_DESCENT: 90.0, ENGAGE: 30.0, LIFT: 30.0, VERIFY_ATTACH: 15.0, CLIMB: 30.0, SET_DOWN: 120.0, CLEAR: 30.0, ABORT_CLIMB: 30.0, LOST_TARGET: 60.0, LANDING: 90.0}  # fmt: skip
LEG_SLACK_S = 30.0


@dataclass(frozen=True)
class Plan:
    """The mission's configuration, checked."""

    hook_below_m: float  # how far under the vehicle's origin the hook's tip hangs
    suspension_length_m: float  # from the hook to the bucket's bottom, hanging
    vehicle_kg: float
    load_kg: float
    capture_half_width_m: float = 0.1
    hook_forward_left_m: Tuple[float, float] = (0.0, 0.0)
    handle_up_m: float = 0.3  # the handle above the ground, the bucket standing
    transit_height_m: float = 15.0
    search_height_m: float = 20.0
    search_speed_mps: float = 4.0
    search_waypoints: Tuple[Tuple[float, float], ...] = ()  # east and north from the pickup's coordinates
    approach_height_m: float = 6.0
    centre_above_handle_m: float = 3.0
    carry_height_m: float = 5.0
    carry_speed_mps: float = 1.0
    swing_shaping: bool = False
    pendulum_m: float = 0.7  # the hinge to the middle of what hangs, for the shaping
    gps_untrusted: bool = False
    return_target: back.ReturnTarget = back.ReturnTarget(back.TAKEOFF)
    engage: grip.EngagePlan = grip.EngagePlan()
    footprint_per_metre: float = 0.9  # the picture's shorter side on the ground, per metre of height
    clear_aside_m: float = 2.0
    time_budget_s: float = 600.0
    max_attempts: int = 3
    land_after: bool = True  # land beside the bucket once clear of it; false leaves the vehicle hovering there for the next mission

    def __post_init__(self) -> None:
        positive = ("suspension_length_m", "vehicle_kg", "load_kg", "capture_half_width_m", "transit_height_m", "search_height_m", "search_speed_mps", "approach_height_m", "centre_above_handle_m", "carry_height_m", "carry_speed_mps", "pendulum_m", "footprint_per_metre", "time_budget_s")  # fmt: skip
        for name in positive + ("hook_below_m", "handle_up_m", "clear_aside_m"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0.0 or (name in positive and value <= 0.0):
                raise ValueError(f"{name} must be a number {'above 0' if name in positive else 'of 0 or more'}, got {value!r}")
        if not isinstance(self.land_after, bool):
            raise ValueError(f"land_after must be true or false, got {self.land_after!r}")
        if isinstance(self.max_attempts, bool) or not isinstance(self.max_attempts, int) or self.max_attempts < 1:
            raise ValueError(f"max_attempts must be a whole number of 1 or more, got {self.max_attempts!r}")
        if not self.approach_height_m > self.centre_above_handle_m + self.hook_below_m + self.handle_up_m:
            raise ValueError("approach_height_m must be above the height CENTRE holds (centre_above_handle_m and the hook and the handle)")
        if self.carry_speed_mps != capped_speed(grip.CARRY_PHASE, self.carry_speed_mps) or self.search_speed_mps != capped_speed("survey", self.search_speed_mps):
            raise ValueError(f"carry_speed_mps {self.carry_speed_mps:g} or search_speed_mps {self.search_speed_mps:g} is over its phase's speed cap")
        if self.carry_height_m <= self.hook_below_m + self.suspension_length_m:
            raise ValueError("carry_height_m must clear the ground with the bucket hanging: above hook_below_m and suspension_length_m")


@dataclass(frozen=True)
class Target:
    """The estimator's track of the bucket (or of the X), as the mission reads it."""

    e: float
    n: float
    sigma_m: float
    age_s: float  # since the frame of its last hit
    confirmed: bool


@dataclass(frozen=True)
class Readings:
    now_s: float
    e: Optional[float] = None  # the vehicle in the local frame
    n: Optional[float] = None
    up: Optional[float] = None
    yaw_rad: Optional[float] = None
    agl_m: Optional[float] = None  # height above the ground
    agl_from_rangefinder: bool = False
    range_age_s: Optional[float] = None  # the rangefinder's newest reading's age
    throttle: Optional[float] = None  # 0 to 1
    throttle_age_s: Optional[float] = None
    landed: Optional[bool] = None
    landed_age_s: Optional[float] = None
    fix: Optional[Tuple[float, float]] = None  # a usable GPS fix now: latitude, longitude
    home_up_m: Optional[float] = None  # home's height in the local frame (the local height less the height above home)
    pickup_en: Optional[Tuple[float, float]] = None  # the pickup's coordinates in the local frame, once they can be placed
    target: Optional[Target] = None
    ack: Optional[grip.Ack] = None  # the newest answer to a gripper command


@dataclass(frozen=True)
class Command:
    status: str = RUNNING
    frame: Optional[str] = None  # LOCAL: fly to e, n, up. GLOBAL: to lat, lon at `alt` above home. None: no setpoint
    e: float = 0.0
    n: float = 0.0
    up: float = 0.0
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    gripper: Optional[str] = None  # CLOSE or OPEN, on the one tick it is commanded
    attempt: int = 0  # the number its acknowledgement must carry
    mode: Optional[str] = None  # LAND, on the tick it is asked for
    reason: str = ""


class RecoveryFlight:
    def __init__(self, plan: Plan) -> None:
        self.plan = plan
        self.state = CHECKS
        self.attempts = 0  # descents onto the handle begun
        self.reason = ""  # why it failed, or the last thing worth saying
        self.transitions: List[str] = [CHECKS]
        self.global_setpoints = 0
        self.released = False
        self._started_s: Optional[float] = None
        self._state_since_s = 0.0
        self._last_s: Optional[float] = None
        self._cmd: Optional[Tuple[float, float, float]] = None  # the commanded point in the local frame
        self._pickup: Optional[Tuple[float, float]] = None
        self._takeoff: Optional[back.Anchor] = None
        self._last_target: Optional[Target] = None
        self._target_seen_s: Optional[float] = None
        self._search_index = 0
        self._search_centre: Optional[Tuple[float, float]] = None
        self._centred_since_s: Optional[float] = None
        self._cone = cone_rule.ConeDescent(plan.capture_half_width_m)
        self._engage = grip.Engage(plan.engage)
        self._attach = grip.AttachCheck(plan.vehicle_kg, plan.load_kg)
        self._watch: Optional[grip.LoadWatch] = None
        self._set_down: Optional[back.SetDown] = None
        self._touch_range_m: Optional[float] = None
        self._lift_to_agl: Optional[float] = None
        self._leg: Any = None
        self._leg_start: Optional[back.Anchor] = None
        self._leg_goal: Optional[back.ReturnGoal] = None
        self._leg_since_s = 0.0
        self._flown_untrusted_m = 0.0
        self._previous_en: Optional[Tuple[float, float]] = None
        self._lost_top_since_s: Optional[float] = None
        self._landing = False
        self._clear_goal: Optional[Tuple[float, float, float]] = None
        self._verify_since_s: Optional[float] = None
        self._closed_attempt: Optional[int] = None  # the engage attempt whose gripper close has been commanded

    # --- small helpers -----------------------------------------------------------------

    def _to(self, state: str, now: float, why: str = "") -> None:
        self.state = state
        self._state_since_s = now
        self.transitions.append(state)
        if why:
            self.reason = why

    def _fail(self, now: float, why: str) -> Command:
        self._to(FAILED, now, why)
        return Command(FAILURE, reason=why)

    def _fault(self, now: float, why: str) -> None:
        """With the bucket on the hook: nothing is let go; land with it."""
        self._to(CARRYING_FAULT, now, why)

    def _hold(self, r: Readings, reason: str = "") -> Command:
        if self._cmd is None:
            self._cmd = (r.e, r.n, r.up)
        return Command(RUNNING, LOCAL, *self._cmd, reason=reason)

    def _local(self, e: float, n: float, up: float, reason: str = "") -> Command:
        self._cmd = (e, n, up)
        return Command(RUNNING, LOCAL, e, n, up, reason=reason)

    def _up_for(self, r: Readings, agl_wanted_m: float) -> float:
        """The local height at which the vehicle is `agl_wanted_m` above the ground it is over."""
        return r.up + (agl_wanted_m - r.agl_m)

    def _walk(self, r: Readings, e: float, n: float, speed_mps: float, dt: float) -> Tuple[float, float]:
        """The commanded place moved toward a goal at a speed (from where it was commanded, not from the vehicle)."""
        from_e, from_n = (self._cmd[0], self._cmd[1]) if self._cmd is not None else (r.e, r.n)
        distance = math.hypot(e - from_e, n - from_n)
        step = speed_mps * dt
        if distance <= step or distance <= 0.0:
            return e, n
        return from_e + (e - from_e) * step / distance, from_n + (n - from_n) * step / distance

    def _climb_toward(self, r: Readings, up_goal: float, rate_mps: float, dt: float) -> float:
        """The commanded height walked toward a goal at a rate."""
        now_up = self._cmd[2] if self._cmd is not None else r.up
        step = rate_mps * dt
        return up_goal if abs(up_goal - now_up) <= step else now_up + math.copysign(step, up_goal - now_up)

    def _over_handle(self, r: Readings, target: Target) -> Tuple[float, float]:
        """Where the vehicle's origin belongs for the hook to be over the handle."""
        forward, left = self.plan.hook_forward_left_m
        return (target.e - (forward * math.cos(r.yaw_rad) - left * math.sin(r.yaw_rad)), target.n - (forward * math.sin(r.yaw_rad) + left * math.cos(r.yaw_rad)))

    def _hook_height(self, r: Readings) -> Optional[float]:
        return cone_rule.hook_height_above_handle_m(r.agl_m, self.plan.hook_below_m, self.plan.handle_up_m)

    def _centre_agl(self) -> float:
        return self.plan.centre_above_handle_m + self.plan.hook_below_m + self.plan.handle_up_m

    def _trusted(self, r: Readings) -> bool:
        return not self.plan.gps_untrusted and r.fix is not None

    # --- one tick ----------------------------------------------------------------------

    def update(self, r: Readings) -> Command:
        now = float(r.now_s)
        if self._started_s is None:
            self._started_s = self._state_since_s = now
        dt = 0.0 if self._last_s is None else min(MAX_TICK_S, max(0.0, now - self._last_s))
        self._last_s = now
        if self.state == COMPLETE:
            return Command(SUCCESS)
        if self.state == FAILED:
            return Command(FAILURE, reason=self.reason)
        if r.e is None or r.n is None or r.up is None or r.yaw_rad is None:
            if self.state == CHECKS and now - self._started_s < CHECKS_S:
                return Command(RUNNING, reason="waiting for a position")
            if self.state in CARRYING:
                return self._carrying_fault(r, now, "no position")
            return self._fail(now, "no position from the flight controller")
        if self._previous_en is not None and not self._trusted(r):
            self._flown_untrusted_m += math.hypot(r.e - self._previous_en[0], r.n - self._previous_en[1])
        elif self._trusted(r):
            self._flown_untrusted_m = 0.0
        self._previous_en = (r.e, r.n)
        if r.target is not None:
            self._last_target, self._target_seen_s = r.target, now
        if self._watch is not None and self.state in (CLIMB, RETURN):
            if self._watch.update(r.throttle, now, r.throttle_age_s):
                return self._fail(now, "the throttle fell back to the empty hover's on the way: the bucket was lost")

        carrying = self.state in CARRYING
        limit = STATE_LIMIT_S.get(self.state)
        if self.state == RETURN and self._leg_goal is not None:
            limit = self._leg_duration() + LEG_SLACK_S
        if self.state == SEARCH:
            limit = self._search_length_m() / self.plan.search_speed_mps + LEG_SLACK_S + 60.0
        out_of_time = now - self._started_s >= self.plan.time_budget_s
        outstayed = limit is not None and now - self._state_since_s >= limit
        if self.state not in (CARRYING_FAULT, CHECKS) and (outstayed or (out_of_time and not carrying)):
            why = f"{self.state} lasted over {limit:g} s" if outstayed else f"the time budget of {self.plan.time_budget_s:g} s ran out in {self.state}"
            if carrying:
                self._fault(now, why)
            elif self.state in (ENGAGE, LIFT, VERIFY_ATTACH):
                return self._fail(now, why + " (the gripper may be closed on the handle: nothing was let go)")
            else:
                return self._fail(now, why)

        handler = {
            CHECKS: self._checks, TRANSIT: self._transit, ACQUIRE: self._acquire, SEARCH: self._search, APPROACH: self._approach, CENTRE: self._centre,
            CONE_DESCENT: self._cone_descent, ENGAGE: self._engaging, LIFT: self._lift, VERIFY_ATTACH: self._verify, CLIMB: self._climb, RETURN: self._return,
            SET_DOWN: self._setting_down, CLEAR: self._clear, LANDING: self._landing_beside, ABORT_CLIMB: self._abort_climb, LOST_TARGET: self._lost, CARRYING_FAULT: self._carrying_fault,
        }[self.state]  # fmt: skip
        if self.state == CARRYING_FAULT:
            return self._carrying_fault(r, now, self.reason)
        command = handler(r, now, dt)
        if command.frame == GLOBAL:
            self.global_setpoints += 1
        return command

    # --- out to the bucket -------------------------------------------------------------

    def _checks(self, r: Readings, now: float, dt: float) -> Command:
        if self._takeoff is None:
            # Where the mission begins is over the takeoff point: its place in both frames, for the way back.
            fix = r.fix if self._trusted(r) else None
            self._takeoff = back.Anchor(r.e, r.n, *(fix or (None, None)))
        elif not self._takeoff.has_global and self._trusted(r):
            self._takeoff = back.Anchor(self._takeoff.e, self._takeoff.n, *r.fix)
        if r.pickup_en is not None and r.agl_m is not None:
            away = math.hypot(r.pickup_en[0] - r.e, r.pickup_en[1] - r.n)
            if away > MAX_PICKUP_DISTANCE_M:
                return self._fail(now, f"the pickup's coordinates are {away:.0f} m from the vehicle, over the {MAX_PICKUP_DISTANCE_M:g} m a pickup may be: not flown to")
            self._pickup = (float(r.pickup_en[0]), float(r.pickup_en[1]))
            self._search_centre = self._pickup
            self._to(TRANSIT, now)
            return self._hold(r)
        if now - self._started_s >= CHECKS_S:
            return self._fail(now, "no height above the ground" if r.agl_m is None else "the pickup's coordinates could not be placed in the local frame (no GPS fix with a position)")
        return self._hold(r, "waiting for a height and the pickup's place")

    def _need_height(self, r: Readings, now: float) -> Optional[Command]:
        if r.agl_m is not None:
            return None
        return self._hold(r, "no height above the ground: holding")

    def _seen(self, r: Readings) -> Optional[Target]:
        """A confirmed track of this tick."""
        return r.target if r.target is not None and r.target.confirmed else None

    def _transit(self, r: Readings, now: float, dt: float) -> Command:
        blind = self._need_height(r, now)
        if blind is not None:
            return blind
        up = self._up_for(r, self.plan.transit_height_m)
        if math.hypot(r.e - self._pickup[0], r.n - self._pickup[1]) <= ARRIVE_M and abs(r.agl_m - self.plan.transit_height_m) <= 2.0 * HEIGHT_M:
            self._to(ACQUIRE, now)
        return self._local(self._pickup[0], self._pickup[1], up)

    def _acquire(self, r: Readings, now: float, dt: float) -> Command:
        if self._seen(r) is not None:
            self._to(APPROACH, now)
            return self._hold(r)
        if now - self._state_since_s >= ACQUIRE_S:
            if not self.plan.search_waypoints:
                return self._fail(now, "no track of the bucket or the X over the pickup's coordinates, and no search is planned")
            self._search_index = 0
            self._to(SEARCH, now, "nothing seen over the pickup's coordinates: searching")
        return self._hold(r)

    def _search_length_m(self) -> float:
        points = [(0.0, 0.0), *self.plan.search_waypoints]
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))

    def _search(self, r: Readings, now: float, dt: float) -> Command:
        if self._seen(r) is not None:
            self._to(APPROACH, now)
            return self._hold(r)
        blind = self._need_height(r, now)
        if blind is not None:
            return blind
        if self._search_index >= len(self.plan.search_waypoints):
            return self._fail(now, "the search ended with no track of the bucket or the X")
        east, north = self.plan.search_waypoints[self._search_index]
        goal = (self._search_centre[0] + east, self._search_centre[1] + north)
        if math.hypot(r.e - goal[0], r.n - goal[1]) <= ARRIVE_M:
            self._search_index += 1
        e, n = self._walk(r, goal[0], goal[1], self.plan.search_speed_mps, dt)
        return self._local(e, n, self._climb_toward(r, self._up_for(r, self.plan.search_height_m), 2.0, dt) if dt else r.up)

    def _track(self, r: Readings, now: float) -> Optional[Target]:
        """The track to fly on outside the cone: this tick's, or the last one for TRACK_GONE_S."""
        if r.target is not None:
            return r.target
        if self._last_target is not None and self._target_seen_s is not None and now - self._target_seen_s < TRACK_GONE_S:
            return self._last_target
        return None

    def _approach(self, r: Readings, now: float, dt: float) -> Command:
        target = self._track(r, now)
        if target is None:
            self._to(LOST_TARGET, now, "the track went during the approach")
            return self._hold(r)
        blind = self._need_height(r, now)
        if blind is not None:
            return blind
        over = self._over_handle(r, target)
        e, n = self._walk(r, over[0], over[1], capped_speed("survey", APPROACH_SPEED_MPS), dt)
        up = self._climb_toward(r, self._up_for(r, self.plan.approach_height_m), 2.0, dt)
        if math.hypot(r.e - over[0], r.n - over[1]) <= OVER_M and abs(r.agl_m - self.plan.approach_height_m) <= 2.0 * HEIGHT_M:
            self._begin_centre(now)
        return self._local(e, n, up)

    def _begin_centre(self, now: float) -> None:
        self._centred_since_s = None
        self._to(CENTRE, now)

    def _centre(self, r: Readings, now: float, dt: float) -> Command:
        target = self._track(r, now)
        if target is None:
            self._to(LOST_TARGET, now, "the track went over the bucket")
            return self._hold(r)
        blind = self._need_height(r, now)
        if blind is not None:
            return blind
        over = self._over_handle(r, target)
        e, n = self._walk(r, over[0], over[1], capped_speed("centring", 1.0), dt)
        up = self._climb_toward(r, self._up_for(r, self._centre_agl()), CLIMB_RATE_MPS, dt)
        error = cone_rule.hook_error_m((target.e, target.n), (r.e, r.n), r.yaw_rad, self.plan.hook_forward_left_m)
        settled = error is not None and error <= CENTRED_M and abs(r.agl_m - self._centre_agl()) <= HEIGHT_M and target.confirmed and target.age_s <= cone_rule.MAX_TRACK_AGE_S
        if not settled:
            self._centred_since_s = None
        else:
            self._centred_since_s = self._centred_since_s if self._centred_since_s is not None else now
            if self._attach.baseline_throttle is None or self.attempts == 0:
                self._attach.baseline_sample(r.throttle, now, r.throttle_age_s)  # the empty hover, before the grab
            if now - self._centred_since_s >= CENTRED_S:
                if self.attempts >= self.plan.max_attempts:
                    return self._fail(now, f"{self.plan.max_attempts} descents onto the handle did not take the bucket")
                self.attempts += 1
                self._cone.reset()
                self._engage.reset()
                self._to(CONE_DESCENT, now)
        return self._local(e, n, up)

    def _cone_descent(self, r: Readings, now: float, dt: float) -> Command:
        target = r.target
        hook_height = self._hook_height(r)
        error = None if target is None else cone_rule.hook_error_m((target.e, target.n), (r.e, r.n), r.yaw_rad, self.plan.hook_forward_left_m)
        age = None if target is None or not target.confirmed else target.age_s
        answer = self._cone.update(hook_height, error, age, now)
        if answer.action == cone_rule.LOST:
            self._to(LOST_TARGET, now, f"the cone lost the handle ({answer.reason})")
            return self._hold(r)
        if answer.action == cone_rule.ABORT:
            self._to(ABORT_CLIMB, now, f"the cone gave the descent up ({answer.reason})")
            return self._hold(r)
        e, n = (self._cmd[0], self._cmd[1]) if self._cmd is not None else (r.e, r.n)
        if target is not None:
            over = self._over_handle(r, target)
            e, n = self._walk(r, over[0], over[1], capped_speed("centring", 1.0), dt)
        up = self._cmd[2] if self._cmd is not None else r.up
        if answer.action == cone_rule.DESCEND:
            if hook_height is not None and hook_height <= ENGAGE_BELOW_M:
                self._to(ENGAGE, now)
                return self._local(e, n, up)
            up -= answer.rate_mps * dt
        elif answer.action == cone_rule.CLIMB:
            up += answer.rate_mps * dt
        return self._local(e, n, up, answer.reason)

    def _engaging(self, r: Readings, now: float, dt: float) -> Command:
        target = r.target
        # The engage's height rests on a detection as young as the cone asks for this low: without one it is not
        # given a height, and nothing is closed or lowered on a guess.
        fresh = target is not None and target.confirmed and target.age_s <= cone_rule.MAX_DETECTION_AGE_S
        closed = self._engage.attempt == self._closed_attempt
        hook_height = self._hook_height(r) if (fresh or closed) else None
        answer = self._engage.update(hook_height, r.ack, now)
        e, n, up = self._cmd if self._cmd is not None else (r.e, r.n, r.up)
        if fresh and not closed:
            over = self._over_handle(r, target)
            e, n = self._walk(r, over[0], over[1], capped_speed("centring", 1.0), dt)
        if answer.action == grip.MOVE:
            return self._local(e, n, up - answer.rate_mps * dt)
        if answer.action == grip.CLOSE:
            self._closed_attempt = answer.attempt
            command = self._local(e, n, up)
            return Command(RUNNING, LOCAL, command.e, command.n, command.up, gripper=CLOSE, attempt=answer.attempt)
        if answer.action == grip.REFUSED:
            self._to(ABORT_CLIMB, now, f"the engage was refused ({answer.reason})")
            return self._local(e, n, up)
        if answer.action == grip.ENGAGED:
            if r.agl_m is None:
                return self._fail(now, "no height above the ground at the engage (the gripper is closed on the handle: nothing was let go)")
            # The rangefinder's reading with the hook at the handle of the standing bucket: the height at which the
            # hanging bucket will touch the ground again, for the set-down.
            self._touch_range_m = r.agl_m
            self._lift_to_agl = r.agl_m + grip.lift_height_m(self.plan.suspension_length_m)
            self._to(LIFT, now)
        return self._local(e, n, up)

    def _lift(self, r: Readings, now: float, dt: float) -> Command:
        blind = self._need_height(r, now)
        if blind is not None:
            return blind
        up = self._climb_toward(r, self._up_for(r, self._lift_to_agl), LIFT_RATE_MPS, dt)
        if r.agl_m >= self._lift_to_agl - 0.05:
            self._verify_since_s = now
            self._to(VERIFY_ATTACH, now)
        return self._local(self._cmd[0], self._cmd[1], up)

    def _verify(self, r: Readings, now: float, dt: float) -> Command:
        self._attach.loaded_sample(r.throttle, now, r.throttle_age_s)
        held = now - self._verify_since_s
        if held < grip.VERIFY_HOLD_S or (not self._attach.ready and held < grip.VERIFY_HOLD_S + VERIFY_EXTRA_S):
            return self._hold(r)
        verdict = self._attach.verdict()
        if verdict.attached:
            self._watch = grip.LoadWatch(self._attach.baseline_throttle, self._attach.loaded_throttle)
            self._to(CLIMB, now)
            return self._hold(r)
        # Not attached: back to CENTRE for another descent. The attach check starts again; the empty hover's
        # throttle stands.
        fresh = grip.AttachCheck(self.plan.vehicle_kg, self.plan.load_kg)
        fresh._baseline = list(self._attach._baseline)
        self._attach = fresh
        self._to(ABORT_CLIMB, now, f"not attached: {verdict.reason}")
        return self._hold(r)

    # --- side states -------------------------------------------------------------------

    def _abort_climb(self, r: Readings, now: float, dt: float) -> Command:
        blind = self._need_height(r, now)
        if blind is not None:
            return blind
        if self.attempts >= self.plan.max_attempts:
            return self._fail(now, f"{self.plan.max_attempts} descents onto the handle did not take the bucket ({self.reason})")
        goal = self._up_for(r, self._centre_agl())
        up = self._climb_toward(r, goal, CLIMB_RATE_MPS, dt)
        if r.agl_m >= self._centre_agl() - HEIGHT_M:
            self._begin_centre(now)
        return self._local(self._cmd[0], self._cmd[1], max(up, self._cmd[2]) if goal >= self._cmd[2] else up)

    def _lost(self, r: Readings, now: float, dt: float) -> Command:
        if self._seen(r) is not None:
            self._lost_top_since_s = None
            if self.attempts >= self.plan.max_attempts:
                return self._fail(now, f"{self.plan.max_attempts} descents onto the handle did not take the bucket ({self.reason})")
            self._to(APPROACH if r.agl_m is not None and r.agl_m > self._centre_agl() + HEIGHT_M + 1.0 else CENTRE, now)
            self._centred_since_s = None
            return self._hold(r)
        blind = self._need_height(r, now)
        if blind is not None:
            return blind
        sigma = self._last_target.sigma_m if self._last_target is not None else 1.0
        # High enough for the picture's shorter side to span three standard deviations either way, 10 m at most.
        top = min(LOST_MAX_HEIGHT_M, max(self._centre_agl(), 6.0 * sigma / self.plan.footprint_per_metre))
        if r.agl_m < top - HEIGHT_M:
            self._lost_top_since_s = None
            return self._local(self._cmd[0], self._cmd[1], self._climb_toward(r, self._up_for(r, top), CLIMB_RATE_MPS, dt))
        self._lost_top_since_s = self._lost_top_since_s if self._lost_top_since_s is not None else now
        if now - self._lost_top_since_s >= LOST_WAIT_S:
            self._lost_top_since_s = None
            if not self.plan.search_waypoints:
                return self._fail(now, "the bucket was lost and no search is planned")
            if self._last_target is not None:
                self._search_centre = (self._last_target.e, self._last_target.n)
            self._search_index = 0
            self._to(SEARCH, now, "the bucket was not seen again from above: searching")
        return self._hold(r)

    def _carrying_fault(self, r: Readings, now: float, why: str) -> Command:
        if self.state != CARRYING_FAULT:
            self._fault(now, why)
        if r.landed is True and grip._fresh(r.landed_age_s, grip.LANDED_MAX_AGE_S):
            return self._fail(now, f"landed with the bucket on the hook: {self.reason}")
        if not self._landing:
            self._landing = True
            return Command(RUNNING, mode=LAND, reason=self.reason)
        return Command(RUNNING, reason=self.reason)

    # --- back with the bucket ----------------------------------------------------------

    def _climb(self, r: Readings, now: float, dt: float) -> Command:
        blind = self._need_height(r, now)
        if blind is not None:
            return blind
        up = self._climb_toward(r, self._up_for(r, self.plan.carry_height_m), CLIMB_RATE_MPS, dt)
        if abs(r.agl_m - self.plan.carry_height_m) <= HEIGHT_M:
            self._leg_goal = None
            self._to(RETURN, now)
        return self._local(self._cmd[0], self._cmd[1], up)

    def _leg_duration(self) -> float:
        if self._leg is not None:
            return self._leg.duration_s
        return back.leg_length_m(self._leg_start, self._leg_goal) / self.plan.carry_speed_mps

    def _return(self, r: Readings, now: float, dt: float) -> Command:
        trusted = self._trusted(r)
        goal = back.return_goal(self.plan.return_target, gps_trusted=trusted, takeoff=self._takeoff, flown_untrusted_m=self._flown_untrusted_m, vehicle_en=(r.e, r.n))
        if goal.frame == GLOBAL and r.home_up_m is None:
            goal = back.return_goal(self.plan.return_target, gps_trusted=False, takeoff=self._takeoff, flown_untrusted_m=self._flown_untrusted_m, vehicle_en=(r.e, r.n))  # no height to give a global setpoint
        if self._leg_goal is None or goal.frame != self._leg_goal.frame or math.hypot(goal.e - self._leg_goal.e, goal.n - self._leg_goal.n) > 0.05:
            # A new leg from where the vehicle is: at the start, and whenever the goal or its frame changes.
            self._leg_start = back.Anchor(r.e, r.n, *(r.fix if goal.frame == GLOBAL else (None, None)))
            self._leg_goal, self._leg_since_s = goal, now
            length = back.leg_length_m(self._leg_start, goal)
            self._leg = grip.ShapedLeg(length, self.plan.carry_speed_mps, self.plan.pendulum_m) if self.plan.swing_shaping and length > 0.0 else None
            if goal.reason:
                self.reason = goal.reason
        length = back.leg_length_m(self._leg_start, self._leg_goal)
        elapsed = now - self._leg_since_s
        if length <= 0.0:
            share = 1.0
        elif self._leg is not None:
            share = self._leg.travelled_m(elapsed) / length
        else:
            share = min(1.0, elapsed * self.plan.carry_speed_mps / length)
        point = back.setpoint_along(self._leg_start, self._leg_goal, share)
        up = self._cmd[2] if self._cmd is not None else r.up
        if share >= 1.0 and math.hypot(r.e - self._leg_goal.e, r.n - self._leg_goal.n) <= ARRIVED_BACK_M:
            if self._attach.baseline_throttle is None or self._attach.loaded_throttle is None or self._touch_range_m is None:
                self._fault(now, "nothing to feel the set-down by (no hover throttle from the attach check, or no height from the engage)")
                return self._carrying_fault(r, now, self.reason)
            self._set_down = back.SetDown(back.SetDownPlan(self._touch_range_m), self._attach.baseline_throttle, self._attach.loaded_throttle)
            self._to(SET_DOWN, now)
            return self._local(self._leg_goal.e, self._leg_goal.n, up)
        self._cmd = (point.e, point.n, up)
        if point.frame == GLOBAL:
            return Command(RUNNING, GLOBAL, point.e, point.n, up, point.lat, point.lon, back.global_height_m(up, r.home_up_m))
        return Command(RUNNING, LOCAL, point.e, point.n, up)

    def _setting_down(self, r: Readings, now: float, dt: float) -> Command:
        height = r.agl_m if r.agl_from_rangefinder else None
        answer = self._set_down.update(
            back.SetDownReading(now, height, r.range_age_s, r.agl_from_rangefinder, r.throttle, r.throttle_age_s, r.landed, r.landed_age_s, r.ack)
        )
        self.released = self._set_down.released
        e, n, up = self._cmd
        if answer.action == back.FAULT:
            self._fault(now, answer.reason or self._set_down.last_reason)
            return self._carrying_fault(r, now, self.reason)
        if answer.action == back.DONE:
            if self._landing:
                self._to(COMPLETE, now, "let go on the ground")
                return Command(SUCCESS, reason=self.reason)
            self._clear_goal = None
            self._to(CLEAR, now)
            return self._local(e, n, up)
        if answer.action == back.LAND:
            if not self._landing:
                self._landing = True
                return Command(RUNNING, mode=LAND, reason=answer.reason)
            return Command(RUNNING, reason=answer.reason)
        if self._landing:
            return Command(RUNNING, gripper=OPEN if answer.open_gripper else None, attempt=answer.attempt)
        command = self._local(e, n, up + answer.rate_up_mps * dt)
        if answer.open_gripper:
            return Command(RUNNING, LOCAL, command.e, command.n, command.up, gripper=OPEN, attempt=answer.attempt)
        return command

    def _clear(self, r: Readings, now: float, dt: float) -> Command:
        blind = self._need_height(r, now)
        if blind is not None:
            return blind
        if self._clear_goal is None:
            # Up clear of the bucket, then to the side: the next mission lands away from it.
            self._clear_goal = (self._cmd[0] + self.plan.clear_aside_m, self._cmd[1], self._centre_agl())
        east, north, agl = self._clear_goal
        up = self._climb_toward(r, self._up_for(r, agl), CLIMB_RATE_MPS, dt)
        e, n = self._cmd[0], self._cmd[1]
        if r.agl_m >= agl - HEIGHT_M:
            e, n = self._walk(r, east, north, capped_speed("centring", 1.0), dt)
            if math.hypot(r.e - east, r.n - north) <= ARRIVED_BACK_M:
                if self.plan.land_after:
                    self._to(LANDING, now)
                    return Command(RUNNING, mode=LAND)
                self._to(COMPLETE, now)
                return Command(SUCCESS)
        return self._local(e, n, up)

    def _landing_beside(self, r: Readings, now: float, dt: float) -> Command:
        if r.landed is True and grip._fresh(r.landed_age_s, grip.LANDED_MAX_AGE_S):
            self._to(COMPLETE, now)
            return Command(SUCCESS)
        return Command(RUNNING)


HOME_HEIGHT_TOPIC = "/mavros/global_position/rel_alt"  # std_msgs/Float64: the height above home
HOME_HEIGHT_MAX_AGE_S = 2.0


class HomeHeight:
    """Home's height in the local frame: the local height less the height above home, taken together. A global
    setpoint's height is above home; this is what turns a local height into one (recovery_return.global_height_m)."""

    def __init__(self) -> None:
        self._newest: Optional[Tuple[int, float]] = None

    def add(self, stamp_ns: int, height_above_home_m: Any, local_up_m: Any) -> None:
        if all(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) for value in (height_above_home_m, local_up_m)):
            self._newest = (int(stamp_ns), float(local_up_m) - float(height_above_home_m))

    def newest(self, now_ns: int) -> Optional[float]:
        """Home's height in the local frame, or None when none was heard within HOME_HEIGHT_MAX_AGE_S."""
        if self._newest is None or not 0 <= int(now_ns) - self._newest[0] <= int(HOME_HEIGHT_MAX_AGE_S * 1.0e9):
            return None
        return self._newest[1]


def attach_home_height(node: Any, specs: Any) -> Optional[HomeHeight]:
    """A HomeHeight fed by MAVROS's height above home through `node`'s own subscription, or None with the flag off.
    The executor's thin ROS side."""
    from .features import get_feature_flag

    if not any(get_feature_flag(spec.features, FLAG) for spec in specs):
        return None
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float64

    feed = HomeHeight()

    def on_height(message: Any) -> None:
        pose = getattr(node, "_local_pose", None)
        if pose is not None:  # (the message has no stamp of its own: it is paired with the pose of the moment it came)
            feed.add(int(node.get_clock().now().nanoseconds), message.data, pose.pose.position.z)

    node.create_subscription(Float64, HOME_HEIGHT_TOPIC, on_height, qos_profile_sensor_data)
    node.get_logger().info(f"Home height feed on ({FLAG}): {HOME_HEIGHT_TOPIC}, for the height of the recovery's global setpoints")
    return feed


def expanding_square(spacing_m: float, radius_m: float) -> Tuple[Tuple[float, float], ...]:
    """An expanding square's corners about (0, 0), east and north: legs of 1, 1, 2, 2, 3, 3... spacings, out to the
    radius. With a spacing of the picture's footprint every place within the radius is seen."""
    if not (isinstance(spacing_m, (int, float)) and isinstance(radius_m, (int, float))) or isinstance(spacing_m, bool) or not spacing_m > 0.0 or not radius_m > 0.0:
        raise ValueError(f"an expanding square needs a spacing and a radius above 0, got {spacing_m!r} and {radius_m!r}")
    points: List[Tuple[float, float]] = []
    east = north = 0.0
    directions = ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0))
    leg, turn = 1, 0
    while True:
        for _ in range(2):
            de, dn = directions[turn % 4]
            east, north = east + de * leg * spacing_m, north + dn * leg * spacing_m
            points.append((east, north))
            turn += 1
        if leg * spacing_m / 2.0 >= radius_m:
            return tuple(points)
        leg += 1


def plan_from_config(config: Any, features: Any, get_flag: Any, search_waypoints: Sequence[Tuple[float, float]] = (), footprint_per_metre: Optional[float] = None) -> Plan:
    """The plan from a mission's `config` block and the sequence's flags. Raises ValueError for what cannot be flown."""
    config = dict(config or {})
    for flag, why in UNBUILT_FLAGS.items():
        if get_flag(features, flag):
            raise ValueError(f"the flag {flag} is on, and {why}")
    known = {
        "hook_below_m", "suspension_length_m", "vehicle_kg", "load_kg", "capture_half_width_m", "hook_forward_left_m", "handle_up_m", "transit_height_m",
        "search_height_m", "search_speed_mps", "approach_height_m", "centre_above_handle_m", "carry_height_m", "carry_speed_mps", "pendulum_m", "clear_aside_m",
        "time_budget_s", "max_attempts", "land_after",
    }  # fmt: skip
    values = {key: config[key] for key in known if key in config}
    if "hook_forward_left_m" in values:
        offset = values["hook_forward_left_m"]
        if not isinstance(offset, (list, tuple)) or len(offset) != 2 or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in offset):
            raise ValueError(f"hook_forward_left_m must be [forward, left] in metres, got {offset!r}")
        values["hook_forward_left_m"] = (float(offset[0]), float(offset[1]))
    for needed in ("hook_below_m", "suspension_length_m", "vehicle_kg", "load_kg"):
        if needed not in values:
            raise ValueError(f"{needed} is not given: the mechanism's and the aircraft's numbers have no defaults")
    engage = config.get("engage") or {}
    if not isinstance(engage, dict):
        raise ValueError(f"engage must be a block of the engage's settings, got {engage!r}")
    extra = dict(search_waypoints=tuple(search_waypoints), swing_shaping=bool(get_flag(features, SWING_FLAG)), gps_untrusted=bool(get_flag(features, UNTRUSTED_FLAG)))
    if footprint_per_metre is not None:
        extra["footprint_per_metre"] = footprint_per_metre
    try:
        return Plan(**values, **extra, return_target=back.parse_return_target(config.get("return_target")), engage=grip.EngagePlan(**engage))
    except TypeError as error:
        raise ValueError(f"the engage's settings are not ones it has ({error})") from error


__all__ = ["Plan", "Readings", "Target", "Command", "RecoveryFlight", "HomeHeight", "attach_home_height", "expanding_square", "plan_from_config", "FLAG", "UNTRUSTED_FLAG", "SWING_FLAG", "UNBUILT_FLAGS"]
