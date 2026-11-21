"""The cone descent onto the bucket's handle (REC-3). No ROS; the standard library and this package's own limits.

The recovery mission comes down on the handle inside a cone: the sideways
error it may have shrinks with the hook's height above the handle (after
the NimbRo MBZIRC approach).

  allowed_error_m(height)   1.0 m at 3 m and above, narrowing evenly to
                            0.48 m at 0.8 m, then evenly to 0.8 of the
                            mechanism's capture half-width at 0.4 m, and
                            that from there down.

`ConeDescent.update(hook_height_m, error_m, detection_age_s, now_s)` is
asked every tick of the descent and answers with what to do:

  DESCEND  the error is inside the cone and rests on a detection young
           enough: come down at `rate_mps`, 0.5 m/s at 3 m and above,
           slowing evenly to 0.1 m/s at 0.8 m and below (never over the
           final descent's speed cap, safety_limits).
  HOLD     the error is up to 1.5 times the cone: stay at this height and
           keep centring. Also, for up to BLIND_HOLD_S: nothing to descend
           on (no error, no height, or no detection young enough).
  CLIMB    the error is beyond 1.5 times the cone, below 3 m: climb at
           `rate_mps` while centring, to where the cone is wider. At 3 m
           and above the cone is as wide as it gets, and it holds instead.
  LOST     there has been nothing to descend on for BLIND_HOLD_S: the
           handle is lost, and the mission's lost-target rule takes over.
  ABORT    outside the cone, the error has not shrunk by 20% in 3 s: the
           descent is over, climb. The answer stays ABORT until `reset()`:
           the mission decides whether there is another attempt, and how
           high the climb goes (the cone gives a rate, not a ceiling).

A descent rests on a detection: one no older than 0.2 s with the hook below
0.8 m (the spec's rule), and no older than MAX_TRACK_AGE_S above it, where
the estimator's track may carry the error between frames.

The error is the sideways distance from the hook's point to the handle, in
metres, not from the camera's centre or the vehicle's: `hook_error_m` works
it out from where the vehicle is, which way it points and where the hook
hangs on it. The height is the hook's above the handle, not the vehicle's
above the ground: `hook_height_above_handle_m`.

Nothing here commands anything, and nothing reads this yet: the recovery
mission (REC-2) is what flies by it.
"""

from __future__ import annotations

import math
import numbers
from dataclasses import dataclass
from typing import Any, Optional

from .safety_limits import capped_speed
from .target_access import DESCEND_MAX_AGE_S

DESCEND, HOLD, CLIMB, LOST, ABORT = "descend", "hold", "climb", "lost", "abort"

WIDE_ABOVE_M, WIDE_ERROR_M = 3.0, 1.0  # at 3 m and above: 1.0 m
NARROW_AT_M, NARROW_ERROR_M = 0.8, 0.48  # at 0.8 m: 0.48 m
CAPTURE_BELOW_M, CAPTURE_SHARE = 0.4, 0.8  # below 0.4 m: 0.8 of the mechanism's capture half-width
MAX_RATE_MPS, MIN_RATE_MPS = 0.5, 0.1  # the descent's rate, at 3 m and above and at 0.8 m and below
HOLD_OUT_TO = 1.5  # times the cone: beyond it the vehicle climbs
CLIMB_RATE_MPS = 0.5
STALL_SHRINK, STALL_S = 0.2, 3.0  # outside the cone the error must shrink by this share in this long
FRESH_BELOW_M, MAX_DETECTION_AGE_S = 0.8, 0.2  # below this height a descent needs a detection this young
MAX_TRACK_AGE_S = DESCEND_MAX_AGE_S  # at 0.8 m and above: the age every other descent in this repo is held to (target_access)
BLIND_HOLD_S = 1.5  # this long with nothing to descend on, and the handle is lost (REC-2's lost-target hold)
SPEED_PHASE = "final_descent"  # the cap the rates are held to (safety_limits.PHASE_SPEED_CAPS_MPS)
CLOCK_SLACK_S = 0.05  # a detection stamped this little ahead of the clock is taken as new; further ahead, its age is not known


def _number(value: Any) -> bool:
    """A real, finite number of any type (a NumPy float is one), not a bool."""
    return isinstance(value, numbers.Real) and not isinstance(value, bool) and bool(math.isfinite(value))


def _between(value: float, low: float, at_low: float, high: float, at_high: float) -> float:
    """Evenly from `at_low` at `low` to `at_high` at `high`."""
    return at_low + (at_high - at_low) * (value - low) / (high - low)


def allowed_error_m(hook_height_m: float, capture_half_width_m: float) -> float:
    """The sideways error the cone allows with the hook this far above the handle."""
    if not _number(capture_half_width_m) or not 0.0 < CAPTURE_SHARE * capture_half_width_m <= NARROW_ERROR_M:
        raise ValueError(
            f"the capture half-width must be above 0 and at most {NARROW_ERROR_M / CAPTURE_SHARE:g} m "
            f"({CAPTURE_SHARE:g} of it is the cone's width at the handle, which cannot be wider than the cone at {NARROW_AT_M:g} m), got {capture_half_width_m!r}"
        )
    if not _number(hook_height_m):
        raise ValueError(f"the hook's height above the handle must be a number of metres, got {hook_height_m!r}")
    at_handle = CAPTURE_SHARE * float(capture_half_width_m)
    height = float(hook_height_m)
    if height >= WIDE_ABOVE_M:
        return WIDE_ERROR_M
    if height >= NARROW_AT_M:
        return _between(height, NARROW_AT_M, NARROW_ERROR_M, WIDE_ABOVE_M, WIDE_ERROR_M)
    if height >= CAPTURE_BELOW_M:
        return _between(height, CAPTURE_BELOW_M, at_handle, NARROW_AT_M, NARROW_ERROR_M)
    return at_handle


def descent_rate_mps(hook_height_m: float) -> float:
    """How fast to come down inside the cone: slower when low."""
    height = float(hook_height_m)
    if height >= WIDE_ABOVE_M:
        rate = MAX_RATE_MPS
    elif height <= NARROW_AT_M:
        rate = MIN_RATE_MPS
    else:
        rate = _between(height, NARROW_AT_M, MIN_RATE_MPS, WIDE_ABOVE_M, MAX_RATE_MPS)
    return capped_speed(SPEED_PHASE, rate)


def hook_error_m(handle_en, vehicle_en, yaw_rad: Optional[float], hook_forward_left_m=(0.0, 0.0)) -> Optional[float]:
    """The sideways distance from the hook's point to the handle, in metres. None when a position or the heading is
    not known this tick (None, or not a finite number): `ConeDescent.update` holds on None, and does not raise.

    `handle_en` and `vehicle_en` are (east, north) in the same local frame; `yaw_rad` is the vehicle's heading
    there (0 east, counter-clockwise); `hook_forward_left_m` is where the hook hangs on the vehicle, forward
    and left of the point `vehicle_en` is the position of. The spec's rule: the error is the hook's, not the
    image centre's. A hook 0.1 m ahead of that point makes a vehicle sitting exactly over the handle 0.1 m out.
    """
    if handle_en is None or vehicle_en is None or yaw_rad is None:
        return None  # one of them is not known this tick: no error to give, and the cone holds on None
    values = (*handle_en, *vehicle_en, yaw_rad, *hook_forward_left_m)
    if len(values) != 7 or not all(_number(value) for value in hook_forward_left_m):
        raise ValueError(f"the hook's place on the vehicle must be two numbers of metres, got {hook_forward_left_m!r}")
    if not all(_number(value) for value in values):
        return None
    forward, left = float(hook_forward_left_m[0]), float(hook_forward_left_m[1])
    hook_east = float(vehicle_en[0]) + forward * math.cos(yaw_rad) - left * math.sin(yaw_rad)
    hook_north = float(vehicle_en[1]) + forward * math.sin(yaw_rad) + left * math.cos(yaw_rad)
    return math.hypot(float(handle_en[0]) - hook_east, float(handle_en[1]) - hook_north)


def hook_height_above_handle_m(vehicle_up_m: Optional[float], hook_below_m: float, handle_up_m: Optional[float]) -> Optional[float]:
    """The hook's height above the handle: the vehicle's height, less how far the hook hangs under it, less the handle's.
    None when either height is not known this tick: `ConeDescent.update` holds on None, and does not raise.

    All three in the same frame's heights (the handle's is the bucket's plane height, 0.3 m above the ground in
    the estimator's table). Negative when the hook is past the handle.
    """
    if not _number(hook_below_m) or hook_below_m < 0.0:
        raise ValueError(f"the hook hangs a number of metres, 0 or more, under the vehicle, got {hook_below_m!r}")
    if not _number(vehicle_up_m) or not _number(handle_up_m):
        return None
    return float(vehicle_up_m) - float(hook_below_m) - float(handle_up_m)


@dataclass(frozen=True)
class ConeAnswer:
    action: str  # DESCEND, HOLD, CLIMB, LOST or ABORT
    rate_mps: float  # down for DESCEND, up for CLIMB and ABORT, 0 for HOLD and LOST
    cone_m: float  # the sideways error allowed at this height
    reason: str  # empty for DESCEND


class ConeDescent:
    def __init__(self, capture_half_width_m: float) -> None:
        allowed_error_m(0.0, capture_half_width_m)  # refuses a width the cone is not for
        self.capture_half_width_m = float(capture_half_width_m)
        self.aborted = False
        self._abort: Optional[ConeAnswer] = None
        self._outside_since_s: Optional[float] = None  # when the error it is measured against was taken
        self._outside_error_m: Optional[float] = None  # the error then
        self._blind_since_s: Optional[float] = None  # since when there has been nothing to descend on

    def reset(self) -> None:
        """A new attempt: the abort is cleared and the clocks start again."""
        self.aborted, self._abort = False, None
        self._outside_since_s = self._outside_error_m = self._blind_since_s = None

    def _blind(self, now: float, cone: float, why: str) -> ConeAnswer:
        """Nothing to descend on: hold, and after BLIND_HOLD_S say the handle is lost.

        The stall's clock is left as it is: an error that was outside the cone and not shrinking has not
        started shrinking by being unseen for a moment.
        """
        if self._blind_since_s is None or now < self._blind_since_s:  # (a clock that stepped back starts the wait again)
            self._blind_since_s = now
        if now - self._blind_since_s >= BLIND_HOLD_S:
            return ConeAnswer(LOST, 0.0, cone, f"{why}, for {BLIND_HOLD_S:g} s: the handle is lost")
        return ConeAnswer(HOLD, 0.0, cone, why)

    def update(self, hook_height_m: Any, error_m: Optional[float], detection_age_s: Optional[float], now_s: float) -> ConeAnswer:
        """One tick. `hook_height_m` is the hook's height above the handle (None: not known); `error_m` the hook's
        sideways distance from the handle (None: not known); `detection_age_s` the age of the newest detection the
        error rests on, on the mission's clock (None: there is none); `now_s` that clock."""
        if not _number(now_s):
            raise ValueError(f"the cone's time must be a number of seconds, got {now_s!r}")
        now = float(now_s)
        climb = capped_speed(SPEED_PHASE, CLIMB_RATE_MPS)
        if self._abort is not None:  # before anything is asked of the height: an abort stands whatever is known now
            return ConeAnswer(ABORT, climb, self._abort.cone_m, self._abort.reason)
        if not _number(hook_height_m):
            # No height (a rangefinder reading that did not come): the cone's width is not known. Its narrowest is taken.
            return self._blind(now, CAPTURE_SHARE * self.capture_half_width_m, "the hook's height above the handle is not known")
        height = float(hook_height_m)
        cone = allowed_error_m(height, self.capture_half_width_m)
        if error_m is None or (isinstance(error_m, numbers.Real) and not isinstance(error_m, bool) and not math.isfinite(error_m)):
            return self._blind(now, cone, "no error to descend on: the handle's place is not known")  # None, or not a finite number
        if not _number(error_m) or error_m < 0.0:
            raise ValueError(f"the error is a distance in metres, 0 or more, got {error_m!r}")
        error = float(error_m)
        if error <= cone:
            self._outside_since_s = self._outside_error_m = None  # inside: the stall's clock is not running
            # A descent rests on a detection young enough for this height. (Holding, climbing and giving up do not
            # come nearer the handle, and are decided on the error whatever its age.)
            oldest = MAX_DETECTION_AGE_S if height < FRESH_BELOW_M else MAX_TRACK_AGE_S
            where = f"under {FRESH_BELOW_M:g} m" if height < FRESH_BELOW_M else f"at {FRESH_BELOW_M:g} m and above"
            if detection_age_s is None or not _number(detection_age_s) or detection_age_s < -CLOCK_SLACK_S:
                # (A detection stamped well ahead of the clock has no age that can be trusted.)
                return self._blind(now, cone, f"{where} with no detection to descend on")
            if detection_age_s > oldest:
                return self._blind(now, cone, f"{where} the newest detection is {float(detection_age_s):.2f} s old (at most {oldest:g} s to descend on)")
            self._blind_since_s = None
            return ConeAnswer(DESCEND, descent_rate_mps(height), cone, "")
        self._blind_since_s = None  # there is an error to act on: the wait for something to descend on is over
        # Outside the cone: the error must be shrinking. It is measured against what it was when this spell
        # outside began, or when it last came down by a fifth.
        if self._outside_since_s is None or now < self._outside_since_s:  # (a clock that stepped back starts the wait again)
            self._outside_since_s, self._outside_error_m = now, error
        elif error <= (1.0 - STALL_SHRINK) * self._outside_error_m:
            self._outside_since_s, self._outside_error_m = now, error
        elif now - self._outside_since_s >= STALL_S:
            self.aborted = True
            self._abort = ConeAnswer(
                ABORT, climb, cone, f"{error:.2f} m from the handle, outside the cone's {cone:.2f} m, and the error did not shrink by {100 * STALL_SHRINK:.0f}% in {STALL_S:g} s: the descent is over"
            )
            return self._abort
        if error <= HOLD_OUT_TO * cone:
            return ConeAnswer(HOLD, 0.0, cone, f"{error:.2f} m from the handle, outside the cone's {cone:.2f} m at this height: holding")
        if height >= WIDE_ABOVE_M:
            # The cone is as wide here as it gets: climbing would not bring the error inside it.
            return ConeAnswer(HOLD, 0.0, cone, f"{error:.2f} m from the handle, over {HOLD_OUT_TO:g} times the cone's {cone:.2f} m, and the cone is no wider higher up: holding")
        return ConeAnswer(CLIMB, climb, cone, f"{error:.2f} m from the handle, over {HOLD_OUT_TO:g} times the cone's {cone:.2f} m at this height: climbing")
