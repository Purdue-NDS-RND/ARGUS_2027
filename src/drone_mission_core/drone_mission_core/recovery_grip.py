"""Taking hold of the bucket, knowing it is held, carrying it, and letting it go (REC-4). No ROS.

Four rules the recovery mission (REC-2) flies by. Nothing here commands
anything, and nothing reads this yet. The mechanism's own numbers (how far
past the handle a plunge goes, how long a sweep is, the suspension's
length, the vehicle's and the load's mass) are arguments: the mechanical
team gives them (REC-9).

**Engage.** `Engage(plan).update(hook_height_m, ack_result, now_s)`, each
tick with the hook over the handle:

  plunge  the hook goes `plunge_depth_m` past the handle, the gripper is
          told to close (one tick: `close_gripper`), and the flight
          controller's acknowledgement is waited for. Result 0 is ENGAGED;
          any other result, or none within `ack_timeout_s`, is REFUSED.
          The acknowledgement is an `Ack(attempt, result)`: one that names
          another attempt's close is not this one's and is not heard.
  sweep   the hook comes to the handle's height half a sweep to one side,
          the gripper closes and is acknowledged, and the hook is carried
          across to the other side, taking the handle on the way.

The hook's way to its depth, and a sweep's crossing, each end in REFUSED
after `move_timeout_s`. The height the engage is given is the mission's:
it must rest on a detection as young as the cone descent asks for under
0.8 m (cone_descent.py); the engage takes it as given and has no age of
its own to ask.

**Lift and verify.** Climb to `lift_height_m(suspension)` (the suspension's
length and 0.5 m) and hold 1.5 s. `AttachCheck` compares the hover throttle
of that hold with the hover throttle before the grab: a load raises it by
`expected_throttle_ratio` (thrust goes with weight, throttle with the
square root of thrust: about 1.12 for 1 kg on 4 kg). Attached is the
throttle up by at least MIN_RISE_SHARE of that rise, and neither the hook
camera nor the latch switch saying otherwise (either may be absent: None).
`LoadWatch` then watches the way home: the throttle back near the empty
hover for LOST_AFTER_S is the bucket gone. Every throttle comes with its
age, and one older than set_down.SIGNAL_MAX_AGE_S is not a sample: a
stream that has stopped cannot fill a hover with one old reading.

**Swing shaping** (the mission flag `swing_shaping`). A bucket on the hook
is a pendulum. A change of speed made at a constant rate over exactly one
of its periods (or two, or three) leaves it hanging still at the end;
`ShapedRamp` is that change, and never asks for more than the carrying
phase's speed cap. A speed that is not a number above zero is refused
when the ramp or the leg is made, not flown.

**Release guard.** `release_decision(landed, rangefinder_healthy, height_m,
set_down_detected, height_age_s=..., landed_age_s=...)`: let go when landed
(a landed state heard within LANDED_MAX_AGE_S); or in the air with a
healthy rangefinder whose reading is no older than SIGNAL_MAX_AGE_S, at or
under MAX_RELEASE_HEIGHT_M, with the set-down felt (the throttle dropping
as the bucket takes its own weight: set_down.py's detector). With no
healthy rangefinder the answer is to land first. The ceiling is
min(1.5 m, 1.83 m); 1.83 m (6 ft) is a constant of the code, and a release
above it scores nothing. (As a ceiling 6 ft is 1.8288 m, so the minimum is
also taken with safety_limits' 1.82 m: with 1.5 m as it is, the same.)

The height is the rangefinder's as given. A rangefinder whose beam meets
the bucket hanging under the body reads the bucket, not the ground: where
it is mounted beside the hook is the airframe's to settle, and the set-down
having been felt is what stands behind the height.

The standard library and this package's limits.
"""

from __future__ import annotations

import itertools
import math
import numbers
from dataclasses import dataclass, field
from statistics import median
from typing import Any, Dict, List, NamedTuple, Optional, Tuple

from .safety_limits import (
    DROP_MIN_RELEASE_HEIGHT_M,
    PHASE_SPEED_CAPS_MPS,
    RECOVERY_MAX_RELEASE_HEIGHT_M,
    SETDOWN_MAX_RELEASE_HEIGHT_M,
    capped_speed,
    recovery_release_allowed,
)
from .set_down import SIGNAL_MAX_AGE_S

GRAVITY_MPS2 = 9.80665

# --- engage -------------------------------------------------------------------------------------

MOVE, CLOSE, WAIT, ENGAGED, REFUSED = "move", "close", "wait", "engaged", "refused"
ENGAGE_RATE_MPS = 0.1  # the hook's speed in the engage: the cone's own slowest
ENGAGE_WITHIN_M = 0.01  # the hook is "there" this close to where the engage wants it
ACK_OK = 0  # MAV_RESULT_ACCEPTED
MAX_PLUNGE_DEPTH_M = 0.5  # no plan takes the hook further past the handle than this: a slip of units, not a mechanism
_ATTEMPTS = itertools.count(1)  # every attempt of every Engage has a number of its own


CLOCK_SLACK_S = 0.05  # a reading stamped this far ahead of the clock is still now; further ahead, its age is not known


def _number(value: Any) -> bool:
    return isinstance(value, numbers.Real) and not isinstance(value, bool) and bool(math.isfinite(value))


def _fresh(age_s: Any, oldest_s: float) -> bool:
    """Whether a reading this old is still now."""
    return _number(age_s) and -CLOCK_SLACK_S <= age_s <= oldest_s


class Ack(NamedTuple):
    """The flight controller's answer to a gripper command, and which attempt's command it answers."""

    attempt: int
    result: int


@dataclass(frozen=True)
class EngagePlan:
    mode: str = "plunge"  # "plunge" or "sweep": the mechanism decides
    plunge_depth_m: float = 0.05  # how far past the handle the hook goes before the gripper closes
    sweep_length_m: float = 0.30  # how far the hook is carried across the handle, centred on it
    ack_timeout_s: float = 2.0  # how long the gripper's acknowledgement is waited for
    move_timeout_s: float = 10.0  # how long the hook may take to its depth, and a sweep to cross

    def __post_init__(self) -> None:
        if self.mode not in ("plunge", "sweep"):
            raise ValueError(f"the engage is a plunge or sweep, got {self.mode!r}")
        for name in ("plunge_depth_m", "sweep_length_m", "ack_timeout_s", "move_timeout_s"):
            value = getattr(self, name)
            if not _number(value) or value < 0.0:
                raise ValueError(f"{name} must be a number, 0 or more, got {value!r}")
        if self.plunge_depth_m > MAX_PLUNGE_DEPTH_M:
            raise ValueError(f"plunge_depth_m is {self.plunge_depth_m!r}: more than {MAX_PLUNGE_DEPTH_M:g} m past the handle is not a plunge")


@dataclass(frozen=True)
class EngageAnswer:
    action: str  # MOVE, CLOSE, WAIT, ENGAGED or REFUSED
    target_height_m: float = 0.0  # where the hook should be, above the handle (negative: past it)
    sweep_m: float = 0.0  # where the hook should be along the sweep, from the handle (0 for a plunge)
    rate_mps: float = 0.0  # how fast to go there
    close_gripper: bool = False  # true on the one tick the gripper is told to close
    reason: str = ""
    attempt: int = 0  # which attempt this is: the number the close command's Ack must carry


class Engage:
    def __init__(self, plan: EngagePlan) -> None:
        self.plan = plan
        self.reset()

    def reset(self) -> None:
        """A new attempt, with a number of its own: an acknowledgement of an earlier attempt's close is not heard."""
        self.attempt = next(_ATTEMPTS)  # (not counted per Engage: a mission that makes a new one each try still gets a new number)
        self.engaged = False
        self._began_s: Optional[float] = None
        self._closed_at_s: Optional[float] = None
        self._crossing_since_s: Optional[float] = None
        self._acknowledged = False
        self._refusal: Optional[str] = None

    def _answer(self, action: str, height: float, sweep: float, rate: float = 0.0, close: bool = False, reason: str = "") -> EngageAnswer:
        return EngageAnswer(action, height, sweep, rate, close, reason, self.attempt)

    def _refuse(self, height: float, sweep: float, why: str) -> EngageAnswer:
        self._refusal = why
        return self._answer(REFUSED, height, sweep, reason=why)

    def update(self, hook_height_m: Any, ack: Optional[Ack], now_s: float, swept_m: Optional[float] = None) -> EngageAnswer:
        """One tick. `hook_height_m` is the hook's height above the handle (None: not known); `ack` the flight
        controller's answer to this attempt's close command, `Ack(attempt, result)` (None: not yet); `swept_m`,
        for a sweep, where the hook is along it from the handle."""
        plan = self.plan
        sweep = plan.mode == "sweep"
        depth = 0.0 if sweep else -float(plan.plunge_depth_m)
        start = -plan.sweep_length_m / 2.0 if sweep else 0.0
        now = float(now_s)
        if self.engaged:
            return self._answer(ENGAGED, depth, -start if sweep else 0.0)
        if self._refusal is not None:
            return self._answer(REFUSED, depth, start, reason=self._refusal)
        if self._began_s is None or now < self._began_s:
            self._began_s = now
        if self._closed_at_s is None:
            if now - self._began_s >= plan.move_timeout_s:
                return self._refuse(depth, start, f"the hook was not brought to the engage's depth in {plan.move_timeout_s:g} s")
            if not _number(hook_height_m):
                return self._answer(WAIT, depth, start, reason="the hook's height above the handle is not known: nothing is closed on a guess")
            if float(hook_height_m) > depth + ENGAGE_WITHIN_M:
                return self._answer(MOVE, depth, start, ENGAGE_RATE_MPS)
            self._closed_at_s = now
            return self._answer(CLOSE, depth, start, close=True)
        if not self._acknowledged:
            if now - self._closed_at_s >= plan.ack_timeout_s:
                return self._refuse(depth, start, f"no acknowledgement of the gripper's close in {plan.ack_timeout_s:g} s")
            if not isinstance(ack, Ack) or ack.attempt != self.attempt:
                return self._answer(WAIT, depth, start)  # nothing yet, or the answer to another attempt's command
            if int(ack.result) != ACK_OK:
                return self._refuse(depth, start, f"the gripper's close was answered with result {int(ack.result)}, not {ACK_OK}")
            self._acknowledged = True
            self._crossing_since_s = now
        if not sweep:
            self.engaged = True
            return self._answer(ENGAGED, depth, 0.0)
        # The sweep: closed and acknowledged, the hook is carried across to the other side.
        if _number(swept_m) and float(swept_m) >= -start - ENGAGE_WITHIN_M:
            self.engaged = True
            return self._answer(ENGAGED, depth, -start)
        if now - self._crossing_since_s >= plan.move_timeout_s:
            return self._refuse(depth, -start, f"the sweep did not cross the handle in {plan.move_timeout_s:g} s")
        return self._answer(MOVE, depth, -start, ENGAGE_RATE_MPS)


# --- lift and verify ----------------------------------------------------------------------------

LIFT_MARGIN_M = 0.5  # the lift is the suspension's length and this
VERIFY_HOLD_S = 1.5  # the hover the attach check is made in
MIN_RISE_SHARE = 0.5  # of the throttle rise the load should give: this much of it, and the load is there
MIN_SAMPLES = 5
LOST_SHARE = 0.25  # on the way home: the throttle within this share of the rise above the empty hover is the load gone
LOST_AFTER_S = 1.0  # for this long


def lift_height_m(suspension_length_m: float) -> float:
    """How far to climb from the engage before checking: the bucket clear of the ground by LIFT_MARGIN_M."""
    if not _number(suspension_length_m) or suspension_length_m < 0.0:
        raise ValueError(f"the suspension's length must be a number of metres, 0 or more, got {suspension_length_m!r}")
    return float(suspension_length_m) + LIFT_MARGIN_M


def expected_throttle_ratio(vehicle_kg: float, load_kg: float) -> float:
    """The hover throttle with the load over the hover throttle without: thrust goes with weight, throttle with its square root."""
    if not _number(vehicle_kg) or not _number(load_kg) or vehicle_kg <= 0.0 or load_kg < 0.0:
        raise ValueError(f"masses must be numbers, the vehicle's above 0 and the load's 0 or more, got {vehicle_kg!r} and {load_kg!r}")
    return math.sqrt(1.0 + float(load_kg) / float(vehicle_kg))


@dataclass(frozen=True)
class AttachVerdict:
    attached: bool
    throttle_ratio: Optional[float]  # the loaded hover's throttle over the empty hover's
    expected_ratio: float
    parts: Dict[str, Optional[bool]] = field(default_factory=dict)  # throttle, visual, latch: True, False, or None for "not there to ask"
    reason: str = ""  # why not, when not


class AttachCheck:
    """The hover throttle before the grab against the hover throttle after the lift."""

    def __init__(self, vehicle_kg: float, load_kg: float) -> None:
        self.expected_ratio = expected_throttle_ratio(vehicle_kg, load_kg)
        self._baseline: List[float] = []
        self._loaded: List[Tuple[float, float]] = []

    def baseline_sample(self, throttle: Any, now_s: float, age_s: Any) -> None:
        """The throttle (0 to 1) in the hover before the grab: the hook over the handle, nothing on it.
        `age_s` is how old the reading is, and has to be given; one older than SIGNAL_MAX_AGE_S is not a sample."""
        if _number(throttle) and throttle > 0.0 and _fresh(age_s, SIGNAL_MAX_AGE_S):
            self._baseline.append(float(throttle))

    def loaded_sample(self, throttle: Any, now_s: float, age_s: Any) -> None:
        """The throttle in the hold after the lift, with its age."""
        if _number(throttle) and throttle > 0.0 and _number(now_s) and _fresh(age_s, SIGNAL_MAX_AGE_S):
            self._loaded.append((float(now_s), float(throttle)))

    @property
    def baseline_throttle(self) -> Optional[float]:
        return median(self._baseline) if len(self._baseline) >= MIN_SAMPLES else None

    @property
    def loaded_throttle(self) -> Optional[float]:
        return median(value for _t, value in self._loaded) if len(self._loaded) >= MIN_SAMPLES else None

    @property
    def held_s(self) -> float:
        return self._loaded[-1][0] - self._loaded[0][0] if self._loaded else 0.0

    @property
    def ready(self) -> bool:
        """Enough of both hovers to give a verdict."""
        return self.baseline_throttle is not None and self.loaded_throttle is not None and self.held_s >= VERIFY_HOLD_S - 1.0e-9

    def verdict(self, visual_fixed: Optional[bool] = None, latch_closed: Optional[bool] = None) -> AttachVerdict:
        """Attached or not. `visual_fixed`: the bucket stays fixed in the hook camera (None: no such camera's word);
        `latch_closed`: the latch switch (None: none fitted)."""
        parts: Dict[str, Optional[bool]] = {"throttle": None, "visual": visual_fixed, "latch": latch_closed}
        if self.baseline_throttle is None:
            return AttachVerdict(False, None, self.expected_ratio, parts, "no hover throttle before the grab to compare with")
        if not self.ready:
            return AttachVerdict(False, None, self.expected_ratio, parts, f"the lift has been held {self.held_s:.1f} s of {VERIFY_HOLD_S:g} s")
        ratio = self.loaded_throttle / self.baseline_throttle
        needed = 1.0 + MIN_RISE_SHARE * (self.expected_ratio - 1.0)
        parts["throttle"] = ratio >= needed
        said_no = [name for name in ("throttle", "visual", "latch") if parts[name] is False]
        if not said_no:
            return AttachVerdict(True, ratio, self.expected_ratio, parts)
        why = []
        if "throttle" in said_no:
            why.append(f"the hover throttle is {ratio:.3f} of what it was before the grab (at least {needed:.3f} with the load on)")
        if "visual" in said_no:
            why.append("the bucket does not stay fixed in the hook camera")
        if "latch" in said_no:
            why.append("the latch switch is open")
        return AttachVerdict(False, ratio, self.expected_ratio, parts, "; ".join(why))


class LoadWatch:
    """On the way home: the throttle falling back to the empty hover's is the bucket lost."""

    def __init__(self, baseline_throttle: float, loaded_throttle: float) -> None:
        if not _number(baseline_throttle) or not _number(loaded_throttle) or baseline_throttle <= 0.0 or loaded_throttle <= baseline_throttle:
            raise ValueError(f"the loaded hover's throttle must be above the empty hover's, got {loaded_throttle!r} and {baseline_throttle!r}")
        self._line = float(baseline_throttle) + LOST_SHARE * (float(loaded_throttle) - float(baseline_throttle))
        self._under_since_s: Optional[float] = None
        self._heard_s: Optional[float] = None
        self._since_s: Optional[float] = None
        self.lost = False

    def update(self, throttle: Any, now_s: float, age_s: Any) -> bool:
        """One tick of the return, with the throttle's age. True once the bucket is taken to be gone (and from then on)."""
        if self._since_s is None:
            self._since_s = float(now_s)
        if self.lost:
            return True
        heard = _number(throttle) and _fresh(age_s, SIGNAL_MAX_AGE_S)
        if heard:
            self._heard_s = float(now_s)
        if not heard or float(throttle) > self._line:
            self._under_since_s = None  # (no throttle heard is not a throttle fallen)
            return False
        if self._under_since_s is None or now_s < self._under_since_s:
            self._under_since_s = float(now_s)
        self.lost = float(now_s) - self._under_since_s >= LOST_AFTER_S
        return self.lost

    def unheard_s(self, now_s: float) -> float:
        """How long the watch has gone without a throttle to judge by: no evidence the bucket is there, and none
        that it is gone. What a long silence means on the way home is the mission's to say."""
        last = self._heard_s if self._heard_s is not None else self._since_s
        return 0.0 if last is None else max(0.0, float(now_s) - last)


# --- swing shaping ------------------------------------------------------------------------------

SWING_FLAG = "swing_shaping"
CARRY_PHASE = "carrying"


def pendulum_period_s(length_m: float) -> float:
    """The period of a pendulum this long: the hinge to the middle of what hangs."""
    if not _number(length_m) or length_m <= 0.0:
        raise ValueError(f"a pendulum's length must be a number of metres above 0, got {length_m!r}")
    return 2.0 * math.pi * math.sqrt(float(length_m) / GRAVITY_MPS2)


def ramp_duration_s(length_m: float, periods: int = 1) -> float:
    """How long a change of speed lasts so as to leave no swing: a whole number of the pendulum's periods."""
    if isinstance(periods, bool) or not isinstance(periods, int) or periods < 1:
        raise ValueError(f"a ramp lasts a whole number of periods, 1 or more, got {periods!r}")
    return periods * pendulum_period_s(length_m)


class ShapedRamp:
    """A change of speed at a constant rate over whole pendulum periods, held to the carrying phase's cap."""

    def __init__(self, from_mps: float, to_mps: float, length_m: float, started_s: float, periods: int = 1) -> None:
        if not _number(from_mps) or not _number(to_mps) or from_mps < 0.0 or to_mps < 0.0:
            raise ValueError(f"a ramp runs between two speeds in m/s, 0 or more, got {from_mps!r} and {to_mps!r}")
        self.from_mps = capped_speed(CARRY_PHASE, from_mps)
        self.to_mps = capped_speed(CARRY_PHASE, to_mps)
        self.duration_s = ramp_duration_s(length_m, periods)
        self.started_s = float(started_s)

    def speed_mps(self, now_s: float) -> float:
        share = min(1.0, max(0.0, (float(now_s) - self.started_s) / self.duration_s))
        return self.from_mps + share * (self.to_mps - self.from_mps)

    def done(self, now_s: float) -> bool:
        return float(now_s) - self.started_s >= self.duration_s - 1.0e-9

    @property
    def distance_m(self) -> float:
        """The ground covered during the ramp: how far before a stop the slowing has to begin."""
        return 0.5 * (self.from_mps + self.to_mps) * self.duration_s


class ShapedLeg:
    """A straight leg flown with a shaped start and a shaped stop: how far along it the vehicle should be at each moment.

    Speed up over one pendulum period, cruise, slow over one period. A leg too short for both ramps at the asked
    speed is flown slower, so that the two ramps just fit: the ramps keep their length in time, which is what
    leaves no swing.
    """

    def __init__(self, distance_m: float, speed_mps: float, length_m: float) -> None:
        if not _number(distance_m) or distance_m < 0.0:
            raise ValueError(f"a leg's length must be a number of metres, 0 or more, got {distance_m!r}")
        if not _number(speed_mps) or speed_mps <= 0.0:
            # (Refused, not flown: a leg with no speed would hand back its far end as the place to be at once.)
            raise ValueError(f"a leg's speed must be a number of m/s above 0, got {speed_mps!r}")
        self.distance_m = float(distance_m)
        self.ramp_s = ramp_duration_s(length_m)
        self.speed_mps = min(capped_speed(CARRY_PHASE, speed_mps), self.distance_m / self.ramp_s)  # two ramps cover speed x period
        self.cruise_s = (self.distance_m - self.speed_mps * self.ramp_s) / self.speed_mps if self.speed_mps > 0.0 else 0.0
        self.duration_s = 2.0 * self.ramp_s + self.cruise_s if self.speed_mps > 0.0 else 0.0

    def travelled_m(self, elapsed_s: float) -> float:
        """How far along the leg the vehicle should be this long after setting off."""
        t = min(max(0.0, float(elapsed_s)), self.duration_s)
        v, ramp = self.speed_mps, self.ramp_s
        if v <= 0.0:
            return self.distance_m  # a leg of no length: already there
        if t <= ramp:
            return 0.5 * v * t * t / ramp
        if t <= ramp + self.cruise_s:
            return 0.5 * v * ramp + v * (t - ramp)
        left = self.duration_s - t
        return self.distance_m - 0.5 * v * left * left / ramp


# --- the release guard --------------------------------------------------------------------------

RELEASE, HOLD_LOAD, LAND_FIRST = "release", "hold", "land_first"
SIX_FEET_M = DROP_MIN_RELEASE_HEIGHT_M  # 1.83: the rules' height. A constant of the code, not a setting
# min(1.5 m, 1.83 m), and with them the limits' own 1.82 m: 6 ft is 1.8288 m, and a ceiling is rounded down.
MAX_RELEASE_HEIGHT_M = min(RECOVERY_MAX_RELEASE_HEIGHT_M, SIX_FEET_M, SETDOWN_MAX_RELEASE_HEIGHT_M)
LANDED_MAX_AGE_S = 2.0  # a landed state heard longer ago than this is not the vehicle's state now


@dataclass(frozen=True)
class ReleaseAnswer:
    action: str  # RELEASE, HOLD_LOAD (not yet) or LAND_FIRST
    reason: str = ""  # empty for RELEASE


def release_decision(landed: bool, rangefinder_healthy: bool, height_m: Any, set_down_detected: bool, *, height_age_s: Any, landed_age_s: Any) -> ReleaseAnswer:
    """Whether the bucket may be let go now. `height_m` is the rangefinder's height above the ground and
    `height_age_s` how old that reading is; `landed_age_s` is how long ago the flight controller's landed state
    was heard. Both ages have to be given: a state or a height of unknown age is not one to let go on."""
    if landed is True and _fresh(landed_age_s, LANDED_MAX_AGE_S):
        return ReleaseAnswer(RELEASE)
    if rangefinder_healthy is not True or not _number(height_m) or not _fresh(height_age_s, SIGNAL_MAX_AGE_S):
        return ReleaseAnswer(LAND_FIRST, "no healthy rangefinder height from this moment: the bucket is not let go in the air on a height that is not known; land first")
    if float(height_m) > MAX_RELEASE_HEIGHT_M or not recovery_release_allowed(float(height_m)):
        return ReleaseAnswer(HOLD_LOAD, f"{float(height_m):.2f} m up: over the {MAX_RELEASE_HEIGHT_M:g} m a release is allowed from (the rules' ceiling is {SIX_FEET_M:g} m)")
    if set_down_detected is not True:
        return ReleaseAnswer(HOLD_LOAD, "the set-down has not been felt: the bucket has not taken its own weight")
    return ReleaseAnswer(RELEASE)


assert MAX_RELEASE_HEIGHT_M <= SETDOWN_MAX_RELEASE_HEIGHT_M < SIX_FEET_M and PHASE_SPEED_CAPS_MPS[CARRY_PHASE] > 0.0  # the limits this module rests on are there
