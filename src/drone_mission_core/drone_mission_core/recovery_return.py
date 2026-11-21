"""Taking the bucket back and setting it down (REC-7, the GPS path). No ROS.

Rules the recovery mission (REC-2) flies by; nothing here commands anything.

**Where to.** The mission's `return_target` is `takeoff` (the default), a
published point `{latlon: [lat, lon]}`, or `{marker: <id>}` (a marker laid
at the return point, with `near: [lat, lon]` when it is not at the takeoff).
`parse_return_target` reads it and refuses anything else.

**In which frame.** `return_goal(...)` gives the point to fly to:

  GPS trusted    a global setpoint (latitude and longitude);
  untrusted      a local one, in the EKF's own frame. A published point is
                 put into that frame through the takeoff's pair (where the
                 vehicle took off, in both frames, taken while GPS was
                 trusted). The EKF's frame drifts without GPS by about 3 to
                 5% of the distance flown; `uncertainty_m` is the 5%.

Trusted means the mission said exactly `True` and there is a pair: anything
else is untrusted, and untrusted never gives a global setpoint. A published
point that cannot be placed (no pair was ever taken), or that lies more than
MAX_RETURN_DISTANCE_M from the takeoff (a latitude and longitude written the
wrong way round passes as a pair of numbers), is answered with the takeoff,
and the reason says so.

A goal names where over the ground, not how high. The two frames do not
share a zero for height either: a global setpoint's height is above home,
a local one's is in the EKF's frame, and the sim's differ by 0.19 m.
`global_height_m` gives the height to send with a global setpoint for a
height meant in the local frame; the mission sends each setpoint its height
in that setpoint's own frame, so the carry does not step when the frame
changes.

Within HOMING_RANGE_M of the goal a fresh `HomingFix` (the return point as
something sees it, in the local frame: a marker or keyframes, REC-7b) takes
the goal over, if it is itself within HOMING_RANGE_M of where the goal was
thought to be. Nothing gives a fix yet.

`setpoint_along(start, goal, share)` is the point that far along the way, in
the goal's frame: with the bucket on the hook the setpoint is walked there
(recovery_grip.ShapedLeg says how far along), not handed to the autopilot as
one far point to dash to. The leg's start is fixed: when the goal changes on
the way (trust lost, a homing fix taken), the mission starts a new leg from
where the vehicle is, so the walked setpoint does not jump.

**The set-down.** `SetDown(plan, empty_throttle, loaded_throttle).update(reading)`
each tick over the return point; the two throttles are the attach check's
(recovery_grip.AttachCheck). Its phases:

  LOWER     down at LOWER_RATE_MPS to LOWER_MARGIN_M over the height at which
            the hanging bucket touches (`touch_height_m`).
  SET_DOWN  down at the set-down's 0.12 m/s until the bucket is felt to stand:
            the throttle has come SET_DOWN_SHARE of the way back from the
            loaded hover to the empty one, for SET_DOWN_HOLD_S (`BucketSetDown`).
            Not felt by FLOOR_BELOW_TOUCH_M under the touch height, it waits
            FLOOR_WAIT_S and then lands with the bucket.
  RELEASE   the release guard (recovery_grip.release_decision) allows it; the
            gripper is told to open (one tick) and its acknowledgement, result
            0 for this attempt, is waited for. A refusal or no answer is asked
            again, MAX_RELEASE_TRIES in all; then FAULT: the bucket stays on.
  CONFIRM   up CONFIRM_CLIMB_M and a look at the throttle: still the empty
            hover's, the bucket went (DONE); the loaded one again, it is still
            on the hook, and it is set down again.

With no healthy, fresh rangefinder height it holds (nothing is lowered on a
height that is not known), and after HEIGHT_LOST_S of that the answer is
LAND, and it stays LAND: the bucket is let go once the flight controller
says landed. One late reading is not a rangefinder lost: the first sim
flight by this rule landed with the bucket for a single reading 0.25 s
old. Landed at any time is a release (the guard's first rule).

(CEN-05's contact detector, set_down.py, is not used here: it asks for 15%
of the descent's throttle to be gone, and a 1 kg bucket coming off a 4 kg
aircraft takes off about 11%. This one is scaled by the two hovers the
attach check measured on the day.)

**Abort.** Recovery's executor is given `abort_rtl_mode: LAND`
(RECOVERY_ABORT_MODE; config/params/recovery_mission.yaml): an RTL that the
flight controller refuses, as it does without a position, is asked for again
for ever, and a landing needs no position.

The standard library and this package's limits.
"""

from __future__ import annotations

import math
import numbers
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from .recovery_grip import ACK_OK, HOLD_LOAD, LAND_FIRST, LANDED_MAX_AGE_S, Ack, new_attempt, release_decision
from .recovery_grip import RELEASE as MAY_RELEASE
from .safety_limits import capped_speed
from .set_down import CONTACT_HOLD_S, MAX_TICK_GAP_S, SET_DOWN_RATE_MPS, SIGNAL_MAX_AGE_S
from .target_access import DESCEND_MAX_AGE_S

RECOVERY_ABORT_MODE = "LAND"

# --- where to -----------------------------------------------------------------------------------

TAKEOFF, LATLON, MARKER = "takeoff", "latlon", "marker"
GLOBAL, LOCAL = "global", "local"
METRES_PER_DEGREE = 6_371_008.8 * math.pi / 180.0  # of latitude, on the mean sphere
DRIFT_SHARE = 0.05  # of the distance flown without GPS: the worse end of "3 to 5%"
HOMING_RANGE_M = 10.0
MAX_RETURN_DISTANCE_M = 1000.0  # a published return point further than this from the takeoff is a mistake, not a goal
HOMING_MAX_AGE_S = DESCEND_MAX_AGE_S  # a homing fix older than this is not where the return point is now
CLOCK_SLACK_S = 0.05


def _number(value: Any) -> bool:
    return isinstance(value, numbers.Real) and not isinstance(value, bool) and bool(math.isfinite(value))


def _fresh(age_s: Any, oldest_s: float) -> bool:
    return _number(age_s) and -CLOCK_SLACK_S <= age_s <= oldest_s


def _latlon(value: Any, what: str) -> Tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2 or not all(_number(v) for v in value) or abs(value[0]) > 90.0 or abs(value[1]) > 180.0:
        raise ValueError(f"{what} must be [latitude, longitude] in degrees, got {value!r}")
    return float(value[0]), float(value[1])


@dataclass(frozen=True)
class ReturnTarget:
    kind: str  # TAKEOFF, LATLON or MARKER
    lat: Optional[float] = None  # the published point, or where a marker is laid when not at the takeoff
    lon: Optional[float] = None
    marker: Optional[str] = None  # which marker (REC-7b homes on it)


def parse_return_target(value: Any) -> ReturnTarget:
    """The mission's `return_target`: nothing or `takeoff`, `{latlon: [lat, lon]}`, or `{marker: id, near: [lat, lon]}`."""
    if value is None or value == TAKEOFF:
        return ReturnTarget(TAKEOFF)
    if isinstance(value, dict) and set(value) == {"latlon"}:
        lat, lon = _latlon(value["latlon"], "return_target.latlon")
        return ReturnTarget(LATLON, lat=lat, lon=lon)
    if isinstance(value, dict) and "marker" in value and set(value) <= {"marker", "near"} and value["marker"] is not None and str(value["marker"]) != "":
        lat, lon = _latlon(value["near"], "return_target.near") if "near" in value else (None, None)
        return ReturnTarget(MARKER, lat=lat, lon=lon, marker=str(value["marker"]))
    raise ValueError(f"return_target is 'takeoff', {{latlon: [lat, lon]}} or {{marker: id, near: [lat, lon]}}, got {value!r}")


@dataclass(frozen=True)
class Anchor:
    """One place in both frames: local east and north in the EKF's frame, and (taken while GPS was trusted)
    its latitude and longitude. The takeoff's is what the return rests on."""

    e: float
    n: float
    lat: Optional[float] = None
    lon: Optional[float] = None

    @property
    def has_global(self) -> bool:
        return _number(self.lat) and _number(self.lon)

    def to_local(self, lat: float, lon: float) -> Optional[Tuple[float, float]]:
        """A latitude and longitude in the local frame, or None with no pair to go by. Flat-earth about the pair:
        good to centimetres over a field."""
        if not self.has_global:
            return None
        east = (float(lon) - self.lon) * METRES_PER_DEGREE * math.cos(math.radians(self.lat))
        north = (float(lat) - self.lat) * METRES_PER_DEGREE
        return self.e + east, self.n + north

    def to_global(self, e: float, n: float) -> Optional[Tuple[float, float]]:
        if not self.has_global:
            return None
        lat = self.lat + (float(n) - self.n) / METRES_PER_DEGREE
        lon = self.lon + (float(e) - self.e) / (METRES_PER_DEGREE * math.cos(math.radians(self.lat)))
        return lat, lon


@dataclass(frozen=True)
class HomingFix:
    """The return point as something sees it now, in the local frame (REC-7b gives these)."""

    e: float
    n: float
    age_s: float


@dataclass(frozen=True)
class ReturnGoal:
    frame: str  # GLOBAL: fly to lat, lon. LOCAL: fly to e, n
    e: float  # the goal in the local frame, whichever frame it is flown in (distances are taken here)
    n: float
    lat: Optional[float] = None  # only for GLOBAL
    lon: Optional[float] = None
    uncertainty_m: float = 0.0  # how far the local frame may have drifted under the goal
    homing: bool = False  # the goal is a homing fix
    reason: str = ""  # said when the goal is not the one asked for


def return_goal(
    target: ReturnTarget,
    *,
    gps_trusted: Any,
    takeoff: Anchor,
    flown_untrusted_m: float = 0.0,
    vehicle_en: Optional[Tuple[float, float]] = None,
    homing: Optional[HomingFix] = None,
) -> ReturnGoal:
    """Where to take the bucket, and in which frame to fly there. `flown_untrusted_m` is the distance flown since
    GPS was last trusted; `vehicle_en` and `homing` are for the last ten metres."""
    trusted = gps_trusted is True and takeoff.has_global
    e, n, lat, lon, reason = takeoff.e, takeoff.n, takeoff.lat, takeoff.lon, ""
    if target.lat is not None and target.lon is not None:
        placed = takeoff.to_local(target.lat, target.lon)
        if placed is None:
            reason = "the return point's latitude and longitude cannot be placed: no position was taken in both frames while GPS was trusted; returning to the takeoff"
        elif math.hypot(placed[0] - takeoff.e, placed[1] - takeoff.n) > MAX_RETURN_DISTANCE_M:
            reason = f"the return point is {math.hypot(placed[0] - takeoff.e, placed[1] - takeoff.n):.0f} m from the takeoff, over the {MAX_RETURN_DISTANCE_M:g} m a return point may be: returning to the takeoff"
        else:
            (e, n), lat, lon = placed, target.lat, target.lon
    drift = 0.0 if trusted else DRIFT_SHARE * max(0.0, float(flown_untrusted_m)) if _number(flown_untrusted_m) else 0.0
    if (
        homing is not None
        and vehicle_en is not None
        and _number(homing.e)
        and _number(homing.n)
        and _fresh(homing.age_s, HOMING_MAX_AGE_S)
        and math.hypot(vehicle_en[0] - e, vehicle_en[1] - n) <= HOMING_RANGE_M
        and math.hypot(homing.e - e, homing.n - n) <= HOMING_RANGE_M
    ):
        return ReturnGoal(LOCAL, float(homing.e), float(homing.n), uncertainty_m=drift, homing=True, reason=reason)
    if trusted:
        return ReturnGoal(GLOBAL, e, n, lat, lon, 0.0, False, reason)
    return ReturnGoal(LOCAL, e, n, uncertainty_m=drift, reason=reason)


def global_height_m(local_up_m: float, home_up_m: float) -> float:
    """The height to send with a global setpoint (above home) so that the vehicle flies at `local_up_m` of the local
    frame. `home_up_m` is home's own height in the local frame: the local height less the height above home, both
    read at one moment. A flight controller that sets home when it arms has moved it by then: a mission reads the
    two after arming, or shows that home did not move. (The sim's check flight reads them on the ground before
    arming, and its global legs arrived within 0.06 m of their height.)"""
    if not _number(local_up_m) or not _number(home_up_m):
        raise ValueError(f"heights must be numbers, got {local_up_m!r} and {home_up_m!r}")
    return float(local_up_m) - float(home_up_m)


def leg_length_m(start: Anchor, goal: ReturnGoal) -> float:
    return math.hypot(goal.e - start.e, goal.n - start.n)


def setpoint_along(start: Anchor, goal: ReturnGoal, share: float) -> ReturnGoal:
    """The point `share` (0 to 1) of the way from `start` to the goal, as a setpoint in the goal's frame."""
    share = min(1.0, max(0.0, float(share)))
    if share >= 1.0:
        return goal
    e, n = start.e + share * (goal.e - start.e), start.n + share * (goal.n - start.n)
    if goal.frame != GLOBAL:
        return ReturnGoal(LOCAL, e, n, uncertainty_m=goal.uncertainty_m, homing=goal.homing, reason=goal.reason)
    if not start.has_global:
        raise ValueError("a global leg starts from a place with a latitude and longitude")
    return ReturnGoal(GLOBAL, e, n, start.lat + share * (goal.lat - start.lat), start.lon + share * (goal.lon - start.lon), 0.0, False, goal.reason)


# --- the set-down -------------------------------------------------------------------------------

LOWER, SET_DOWN, RELEASE, CONFIRM, LANDING = "lower", "set_down", "release", "confirm", "landing"
DESCEND, HOLD, CLIMB, OPEN, LAND, DONE, FAULT = "descend", "hold", "climb", "open", "land", "done", "fault"

LOWER_RATE_MPS = 0.3  # down to the margin over the touch height
LOWER_MARGIN_M = 0.5  # the slow descent starts this far over the height at which the bucket touches
FLOOR_BELOW_TOUCH_M = 0.15  # and goes no further than this under it
FLOOR_WAIT_S = 2.0  # at the floor with the set-down not felt: this long, then land
HEIGHT_LOST_S = 1.0  # without a healthy, fresh rangefinder height for this long: no healthy rangefinder, land first
SET_DOWN_SHARE = 0.5  # of the way back from the loaded hover's throttle to the empty one's
SET_DOWN_HOLD_S = CONTACT_HOLD_S  # for this long, without a break
ACK_TIMEOUT_S = 2.0
MAX_RELEASE_TRIES = 3
CONFIRM_CLIMB_M = 0.5
CONFIRM_HOLD_S = 0.5  # the throttle is looked at for this long after the climb
CONFIRM_BLIND_S = 2.0  # with no throttle to look at for this long, the release stands unconfirmed


class BucketSetDown:
    """The bucket has taken its own weight: the throttle has come back towards the empty hover's and stays there."""

    def __init__(self, empty_throttle: float, loaded_throttle: float) -> None:
        if not _number(empty_throttle) or not _number(loaded_throttle) or empty_throttle <= 0.0 or loaded_throttle <= empty_throttle:
            raise ValueError(f"the loaded hover's throttle must be above the empty hover's, got {loaded_throttle!r} and {empty_throttle!r}")
        self.line = float(loaded_throttle) - SET_DOWN_SHARE * (float(loaded_throttle) - float(empty_throttle))
        self._since_s: Optional[float] = None
        self._last_s: Optional[float] = None

    def low(self, throttle: Any, age_s: Any) -> Optional[bool]:
        """Whether a throttle is the empty hover's side of the line. None when it is not a reading of now."""
        if not _number(throttle) or not _fresh(age_s, SIGNAL_MAX_AGE_S):
            return None
        return float(throttle) <= self.line

    def update(self, throttle: Any, now_s: float, age_s: Any) -> bool:
        """One tick, with the throttle's age. True while the set-down is felt."""
        now = float(now_s)
        broken = self._last_s is None or not 0.0 <= now - self._last_s <= MAX_TICK_GAP_S
        self._last_s = now
        if self.low(throttle, age_s) is not True:
            self._since_s = None
            return False
        if self._since_s is None or broken:
            self._since_s = now
        return now - self._since_s >= SET_DOWN_HOLD_S - 1.0e-9


@dataclass(frozen=True)
class SetDownPlan:
    touch_height_m: float  # the vehicle's height above the ground when the hanging bucket's bottom touches it

    def __post_init__(self) -> None:
        if not _number(self.touch_height_m) or self.touch_height_m <= FLOOR_BELOW_TOUCH_M:
            raise ValueError(f"touch_height_m must be a number of metres above {FLOOR_BELOW_TOUCH_M:g}, got {self.touch_height_m!r}")


@dataclass(frozen=True)
class SetDownReading:
    now_s: float
    height_m: Any  # the rangefinder's height above the ground
    height_age_s: Any
    rangefinder_healthy: Any
    throttle: Any  # 0 to 1
    throttle_age_s: Any
    landed: Any  # the flight controller's landed state
    landed_age_s: Any
    ack: Optional[Ack]  # the answer to this rule's gripper command, if one has come


@dataclass(frozen=True)
class SetDownAnswer:
    phase: str
    action: str  # DESCEND, HOLD, CLIMB, OPEN, LAND, DONE or FAULT
    rate_up_mps: float = 0.0  # the vertical speed to fly: negative is down. The horizontal position is held
    open_gripper: bool = False  # true on the one tick the gripper is told to open
    attempt: int = 0  # the number the open command's Ack must carry
    reason: str = ""


class SetDown:
    def __init__(self, plan: SetDownPlan, empty_throttle: float, loaded_throttle: float) -> None:
        self.plan = plan
        self._feel = BucketSetDown(empty_throttle, loaded_throttle)
        self.phase = LOWER
        self.attempt = 0
        self.release_attempts = 0
        self.set_downs = 0
        self.released = False  # the gripper's open was acknowledged
        self.confirmed = False  # and the throttle after a climb said the bucket stayed behind
        self.last_reason = ""
        self.max_rate_seen: Dict[str, float] = {}
        self._landing = False
        self._final: Optional[str] = None  # DONE or FAULT, once reached
        self._opened_at_s: Optional[float] = None
        self._floor_since_s: Optional[float] = None
        self._height_lost_since_s: Optional[float] = None
        self._confirm_at_m: Optional[float] = None
        self._judged_since_s: Optional[float] = None
        self._judged_low: Optional[bool] = None
        self._unheard_since_s: Optional[float] = None

    def _answer(self, action: str, rate_up: float = 0.0, reason: str = "", open_gripper: bool = False) -> SetDownAnswer:
        if reason:
            self.last_reason = reason
        self.max_rate_seen[self.phase] = max(self.max_rate_seen.get(self.phase, 0.0), abs(rate_up))
        if action in (DONE, FAULT):
            self._final = action
        return SetDownAnswer(self.phase, action, rate_up, open_gripper, self.attempt, reason)

    def _open(self, now: float) -> SetDownAnswer:
        if self.release_attempts >= MAX_RELEASE_TRIES:
            return self._answer(FAULT, reason=f"the gripper was asked to open {MAX_RELEASE_TRIES} times and the bucket is still on the hook: land with it")
        self.phase = RELEASE
        self.attempt = new_attempt()
        self.release_attempts += 1
        self._opened_at_s = now
        return self._answer(OPEN, open_gripper=True)

    def _land(self, why: str) -> SetDownAnswer:
        self._landing = True
        self.phase = LANDING
        return self._answer(LAND, reason=why)

    def update(self, reading: SetDownReading) -> SetDownAnswer:
        """One tick over the return point."""
        r = reading
        now = float(r.now_s)
        felt = self._feel.update(r.throttle, now, r.throttle_age_s)
        if self._final is not None:
            return self._answer(self._final)
        on_ground = r.landed is True and _fresh(r.landed_age_s, LANDED_MAX_AGE_S)
        height_ok = r.rangefinder_healthy is True and _number(r.height_m) and _fresh(r.height_age_s, SIGNAL_MAX_AGE_S)

        if self.phase == RELEASE:
            return self._await_ack(r, now, on_ground)
        if self.phase == CONFIRM:
            return self._confirm(r, now, height_ok)
        if on_ground or self._landing:
            decision = release_decision(r.landed, r.rangefinder_healthy, r.height_m, felt, height_age_s=r.height_age_s, landed_age_s=r.landed_age_s)
            if on_ground and decision.action == MAY_RELEASE:
                return self._open(now)
            return self._land(self.last_reason if self._landing else decision.reason)
        if not height_ok:
            if self._height_lost_since_s is None or now < self._height_lost_since_s:
                self._height_lost_since_s = now
            if now - self._height_lost_since_s < HEIGHT_LOST_S:
                return self._answer(HOLD, reason="no healthy rangefinder height from this moment: holding")
            return self._land(f"no healthy rangefinder height for {HEIGHT_LOST_S:g} s: the bucket is not lowered or let go in the air on a height that is not known; land first")
        self._height_lost_since_s = None

        height = float(r.height_m)
        if self.phase == LOWER:
            if height > self.plan.touch_height_m + LOWER_MARGIN_M:
                return self._answer(DESCEND, -capped_speed("final_descent", LOWER_RATE_MPS))
            self.phase = SET_DOWN
            self.set_downs += 1
        # SET_DOWN
        if felt:
            decision = release_decision(r.landed, r.rangefinder_healthy, r.height_m, True, height_age_s=r.height_age_s, landed_age_s=r.landed_age_s)
            if decision.action == MAY_RELEASE:
                return self._open(now)
            if decision.action == LAND_FIRST:
                return self._land(decision.reason)
            assert decision.action == HOLD_LOAD  # felt, and too high to let go from: on down
        floor = self.plan.touch_height_m - FLOOR_BELOW_TOUCH_M
        if height > floor:
            self._floor_since_s = None
            return self._answer(DESCEND, -capped_speed("set_down", SET_DOWN_RATE_MPS))
        if self._floor_since_s is None or now < self._floor_since_s:
            self._floor_since_s = now
        if now - self._floor_since_s >= FLOOR_WAIT_S:
            return self._land(f"the set-down was not felt by {floor:.2f} m, {FLOOR_BELOW_TOUCH_M:g} m under where the bucket touches: land first")
        return self._answer(HOLD)

    def _await_ack(self, r: SetDownReading, now: float, on_ground: bool) -> SetDownAnswer:
        refusal = ""
        if isinstance(r.ack, Ack) and r.ack.attempt == self.attempt:
            if int(r.ack.result) == ACK_OK:
                self.released = True
                if on_ground or self._landing or not _number(r.height_m):
                    self.phase = RELEASE
                    return self._answer(DONE, reason="let go on the ground" if (on_ground or self._landing) else "let go; no height to climb and confirm by")
                self.phase = CONFIRM
                self._height_lost_since_s = None
                self._confirm_at_m = float(r.height_m) + CONFIRM_CLIMB_M
                self._judged_since_s = self._judged_low = self._unheard_since_s = None
                return self._answer(CLIMB, capped_speed("set_down", SET_DOWN_RATE_MPS))
            refusal = f"the gripper's open was answered with result {int(r.ack.result)}, not {ACK_OK}"
        elif now - (self._opened_at_s if self._opened_at_s is not None else now) >= ACK_TIMEOUT_S:
            refusal = f"no acknowledgement of the gripper's open in {ACK_TIMEOUT_S:g} s"
        if not refusal:
            return self._answer(HOLD)
        if self.release_attempts >= MAX_RELEASE_TRIES:
            return self._answer(FAULT, reason=f"the gripper did not open, asked {MAX_RELEASE_TRIES} times: {refusal}. Land with the bucket")
        self.phase = LANDING if self._landing else SET_DOWN  # asked again when the guard next allows it
        return self._answer(HOLD, reason=refusal)

    def _confirm(self, r: SetDownReading, now: float, height_ok: bool) -> SetDownAnswer:
        if not height_ok:
            # As when lowering: one late reading is a tick's hold, and only a second without a height ends it.
            if self._height_lost_since_s is None or now < self._height_lost_since_s:
                self._height_lost_since_s = now
            if now - self._height_lost_since_s < HEIGHT_LOST_S:
                return self._answer(HOLD)
            return self._answer(DONE, reason="let go; the rangefinder's height was lost before the climb that would confirm it")
        self._height_lost_since_s = None
        if float(r.height_m) < self._confirm_at_m:
            return self._answer(CLIMB, capped_speed("set_down", SET_DOWN_RATE_MPS))
        low = self._feel.low(r.throttle, r.throttle_age_s)
        if low is None:
            self._judged_since_s = self._judged_low = None
            if self._unheard_since_s is None:
                self._unheard_since_s = now
            if now - self._unheard_since_s >= CONFIRM_BLIND_S:
                return self._answer(DONE, reason="let go; no throttle was heard to confirm it by")
            return self._answer(HOLD)
        self._unheard_since_s = None
        if self._judged_low is not low or self._judged_since_s is None:
            self._judged_low, self._judged_since_s = low, now
        if now - self._judged_since_s < CONFIRM_HOLD_S - 1.0e-9:
            return self._answer(HOLD)
        if low:
            self.confirmed = True
            return self._answer(DONE)
        # The loaded hover's throttle again: the latch opened, it said, and the bucket came up with the hook.
        self.released = False
        self.phase = SET_DOWN
        self.set_downs += 1
        self._floor_since_s = None
        return self._answer(HOLD, reason="the bucket is still on the hook after the release: setting it down again")
