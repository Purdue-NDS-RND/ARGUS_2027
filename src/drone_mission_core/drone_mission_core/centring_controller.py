"""The retuned centring controller (CEN-02). No ROS imports, standard library only.

Behind the mission flag `centring_retune`, which defaults to false and needs
`centre_on_estimator`. With the flag off, centring on the estimator uses the
2026 gain and speed cap from the sequence, and nothing here is used.

With it on, the horizontal velocity command over a target is

  1. the error with a soft deadband of 0.03 m taken off its length (an error
     inside it commands nothing; just outside it commands a little, not a
     step as the 2026 deadband does);
  2. times the gain Kp, 0.6 per second;
  3. capped at clamp(0.08 x h, 0.25, 1.0) m/s, h being the height above the
     ground: slow near the ground, where a metre of image is a long way, and
     never over the centring cap in safety_limits.py;
  4. changed by no more than 0.5 m/s in a second (a slew limit), so the
     vehicle is not asked to tilt hard over a target it is trying to hold
     still above.

Before step 1 the error is given a lead: it is taken for where the vehicle
will be half a second on at its present velocity (the target stands still,
so that is the error less velocity x 0.5 s). The spec does not have this
term. In Gazebo the loop without it swings against the vehicle's slow
response to a velocity command, with a period of about 7 s, and in strong
wind the hold error at 6 m was 0.34 m at the 95th percentile; with it,
0.22 to 0.27 m (docs/results/2026-11-20-centring-retune.md). With no velocity
reported the lead is left out, the gain is held to 0.6 and the control is
the spec's; `lead_applied` says so, for the mission's log.

The other numbers are the spec's and are the defaults here. A 2027 sequence may set
its own under new keys (`centring_kp`, `centring_lead_s` and the rest, see `CentringGains`); the
2026 keys (`centering_gain_mps_per_m`, `centering_deadband_m`,
`tracking_low_pass_alpha`) are not read. `tools/cen_loop_sim.py` runs this
class in a simulated loop, so a gain can be tried before it is flown.
"""

from __future__ import annotations

import math
import numbers
from dataclasses import dataclass, fields
from typing import Any, Mapping, Optional, Tuple

from .safety_limits import PHASE_SPEED_CAPS_MPS

RETUNE_FLAG = "centring_retune"
NOMINAL_TICK_S = 0.05  # the executor's 20 Hz
# After this long without a step the last command is no guide to what the vehicle is doing: start again from rest.
RESTART_AFTER_S = 0.5
# The highest gain flown in Gazebo. In tools/cen_loop_sim.py (--sweep-kp), with the lead, a step of 0.8 m
# overshoots 0.02 m at 1.0 and 0.06 m at 1.5 and rings by 2.5; without the lead it overshoots 0.12 m at 1.0.
KP_MAX = 1.0
# Without the lead the loop is only known to be steady at the spec's gain: the highest allowed when a sequence
# asks for a lead shorter than LEAD_FOR_KP_MAX_S, and what a higher gain is brought down to on a step with no
# velocity to lead with.
KP_MAX_WITHOUT_LEAD = 0.6
LEAD_FOR_KP_MAX_S = 0.5  # the lead the gains above KP_MAX_WITHOUT_LEAD were flown and simulated with
LEAD_MAX_S = 1.0
# A height from the barometer is above home, not above the ground, and drifts: for the speed cap it is taken
# as this much lower than it says, so ground a little above home does not let the vehicle correct faster
# near it than a rangefinder's height would.
BARO_HEIGHT_MARGIN_M = 2.0


@dataclass(frozen=True)
class CentringGains:
    kp: float = 0.6  # m/s of command per metre of error
    soft_deadband_m: float = 0.03
    speed_cap_per_m_height: float = 0.08  # m/s of cap per metre of height above the ground
    speed_cap_min_mps: float = 0.25
    speed_cap_max_mps: float = 1.0
    slew_mps2: float = 0.5
    lead_s: float = 0.5  # the error is taken for where the vehicle will be this far ahead, at its present velocity

    @classmethod
    def from_config(cls, config: Mapping[str, Any]) -> "CentringGains":
        """The gains a mission's config asks for under the `centring_` keys; the spec's where a key is left out.

        Raises ValueError, naming the key, for a value that is not a usable number.
        """
        values = {}
        for field in fields(cls):
            key = f"centring_{field.name}"
            if key not in config:
                continue
            value = config[key]
            if not isinstance(value, numbers.Real) or isinstance(value, bool) or not math.isfinite(value):
                raise ValueError(f"{key} is {value!r}, not a number")
            value = float(value)
            may_be_zero = field.name in ("soft_deadband_m", "lead_s")
            if value < 0.0 or (value == 0.0 and not may_be_zero):
                raise ValueError(f"{key} is {value:g}: it must be {'zero or more' if may_be_zero else 'more than zero'}")
            values[field.name] = value
        gains = cls(**values)
        if gains.kp > KP_MAX:
            raise ValueError(f"centring_kp is {gains.kp:g}: over {KP_MAX:g} the loop rings (try it in tools/cen_loop_sim.py first)")
        if gains.lead_s < LEAD_FOR_KP_MAX_S and gains.kp > KP_MAX_WITHOUT_LEAD:
            raise ValueError(
                f"centring_kp is {gains.kp:g} with centring_lead_s {gains.lead_s:g}: "
                f"with a lead under {LEAD_FOR_KP_MAX_S:g} s the gain may be {KP_MAX_WITHOUT_LEAD:g} at most"
            )
        if gains.lead_s > LEAD_MAX_S:
            raise ValueError(f"centring_lead_s is {gains.lead_s:g}: over {LEAD_MAX_S:g} s the vehicle's velocity is no guide to where it will be")
        if gains.speed_cap_min_mps > gains.speed_cap_max_mps:
            raise ValueError(f"centring_speed_cap_min_mps ({gains.speed_cap_min_mps:g}) is over centring_speed_cap_max_mps ({gains.speed_cap_max_mps:g})")
        return gains


def soft_deadband(east_m: float, north_m: float, deadband_m: float) -> Tuple[float, float]:
    """The error with `deadband_m` taken off its length, direction kept. Zero inside the deadband."""
    length = math.hypot(east_m, north_m)
    if length <= deadband_m or length == 0.0:
        return 0.0, 0.0
    scale = (length - deadband_m) / length
    return east_m * scale, north_m * scale


def speed_cap(height_agl_m: Optional[float], gains: CentringGains) -> float:
    """The fastest horizontal correction allowed at this height above the ground, in m/s.

    A height that is not known (None, not a number, negative) gives the
    lowest cap. The result is never over safety_limits' cap for centring.
    """
    ceiling = min(gains.speed_cap_max_mps, PHASE_SPEED_CAPS_MPS["centring"])
    floor = min(gains.speed_cap_min_mps, ceiling)
    if height_agl_m is None or isinstance(height_agl_m, bool) or not isinstance(height_agl_m, numbers.Real) or not math.isfinite(height_agl_m) or height_agl_m < 0.0:
        return floor
    return max(floor, min(ceiling, gains.speed_cap_per_m_height * float(height_agl_m)))


class CentringController:
    """step(): this tick's horizontal velocity command (east, north) from the error to the aim point."""

    def __init__(self, gains: Optional[CentringGains] = None) -> None:
        self.gains = gains or CentringGains()
        self._command: Tuple[float, float] = (0.0, 0.0)
        self._last_s: Optional[float] = None
        self.lead_applied = True  # False after a step whose lead was wanted but had no velocity to work with

    def reset(self) -> None:
        """Forget the last command: the next step starts from rest."""
        self._command = (0.0, 0.0)
        self._last_s = None

    def step(
        self,
        east_error_m: float,
        north_error_m: float,
        height_agl_m: Optional[float],
        now_s: float,
        speed_limit_mps: Optional[float] = None,
        velocity_en_mps: Optional[Tuple[float, float]] = None,
    ) -> Tuple[float, float]:
        """The command for an error of (east, north) metres at `height_agl_m`, at time `now_s` (any clock that counts seconds).

        `speed_limit_mps` is a caller's own limit, applied as well as the
        height's cap. An error that is not a number commands a stop, reached
        through the slew limit like any other change.
        """
        gains = self.gains
        if self._last_s is None or now_s - self._last_s > RESTART_AFTER_S or now_s < self._last_s:
            self._command = (0.0, 0.0)
            dt = NOMINAL_TICK_S
        else:
            dt = now_s - self._last_s
        self._last_s = now_s

        wanted = (0.0, 0.0)
        if math.isfinite(east_error_m) and math.isfinite(north_error_m):
            east_error_m, north_error_m = float(east_error_m), float(north_error_m)
            kp = gains.kp
            self.lead_applied = True
            if gains.lead_s > 0.0:
                if velocity_en_mps is not None and all(math.isfinite(float(value)) for value in velocity_en_mps[:2]):
                    # The lead: the target stands still, so in `lead_s` the error will be smaller by what the vehicle moves.
                    east_error_m -= float(velocity_en_mps[0]) * gains.lead_s
                    north_error_m -= float(velocity_en_mps[1]) * gains.lead_s
                else:
                    # No velocity to lead with: the gain is held to what is known to be steady without the lead.
                    self.lead_applied = False
                    kp = min(kp, KP_MAX_WITHOUT_LEAD)
            east, north = soft_deadband(east_error_m, north_error_m, gains.soft_deadband_m)
            wanted = (kp * east, kp * north)
            cap = speed_cap(height_agl_m, gains)
            if speed_limit_mps is not None and math.isfinite(speed_limit_mps):
                cap = min(cap, max(0.0, float(speed_limit_mps)))
            speed = math.hypot(*wanted)
            if speed > cap:
                wanted = (wanted[0] * cap / speed, wanted[1] * cap / speed) if speed > 0.0 else (0.0, 0.0)

        change = (wanted[0] - self._command[0], wanted[1] - self._command[1])
        size, allowed = math.hypot(*change), gains.slew_mps2 * dt
        if size > allowed:
            change = (change[0] * allowed / size, change[1] * allowed / size) if size > 0.0 else (0.0, 0.0)
        self._command = (self._command[0] + change[0], self._command[1] + change[1])
        return self._command
