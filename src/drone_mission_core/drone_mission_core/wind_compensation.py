"""Wind compensation for the retuned centring controller (CEN-07). No ROS imports, standard library only.

Behind the mission flag `wind_compensation`, which defaults to false and
needs `centring_retune`. With the flag off nothing here is used, and the
controller of CEN-02 commands as it did.

The retuned controller is proportional: its command is the error times a
gain. In a steady wind the flight controller does not quite fly the
velocity it is asked for, and a proportional loop then comes to rest where
its command makes up the difference: downwind of the target, by the push
over the gain (0.3 m/s over 0.6 a second is half a metre). Two terms are
added to the controller's command to take that out:

  the observer  the vehicle's reported velocity less the velocity that was
                commanded `delay_s` ago, low-passed over `observer_tau_s`,
                is the push the wind adds. It is taken off the command. It
                acts within seconds and needs no error to build up;
  the integral  the error, summed at `ki` while the vehicle is within
                `integral_zone_m` of the target and all but standing
                (`integral_max_speed_mps`): what the observer leaves
                (a velocity report that is itself biased) is taken out
                here, slowly. It is not summed while the command is at
                its cap: that would only store up an overshoot.

Each term is limited (`observer_limit_mps`, `integral_limit_mps`), and the
sum of the controller's command and the two terms is held to the same speed
cap the controller's own command is held to, and with the controller's slew
limit given the command sent changes no faster than it. Neither term changes
on a step whose track is only remembered (`learn` false), and both start
from nothing when the centring starts again (`reset`) or three ticks have
passed without a step. With no velocity reported the observer stands still
and the integral works alone.

`tools/cen_loop_sim.py --steady-wind` runs it in the simulated loop.
"""

from __future__ import annotations

import math
import numbers
from collections import deque
from dataclasses import dataclass, fields
from typing import Any, Deque, Mapping, Optional, Sequence, Tuple

FLAG = "wind_compensation"
RESTART_AFTER_S = 0.15  # three ticks without a step (the mission held still, or commanded something else): what was learnt is no guide, start again from nothing
NOMINAL_TICK_S = 0.05


@dataclass(frozen=True)
class WindGains:
    ki: float = 0.10  # m/s of command per metre of error, per second
    integral_limit_mps: float = 0.5
    integral_zone_m: float = 0.6  # the error is summed only inside this: on the way in from far off it is not wind
    integral_max_speed_mps: float = 0.2  # ... and only while the vehicle is slower than this over the ground: an error it is flying off is not the wind's
    observer_tau_s: float = 3.0
    observer_limit_mps: float = 0.8
    delay_s: float = 0.4  # how long the vehicle takes to be flying a velocity it was asked for (the loop simulator's lag, and Gazebo's)

    # The sequence's keys for these, all optional: `wind_ki`, `wind_integral_limit_mps`, and so on.
    @classmethod
    def from_config(cls, config: Mapping[str, Any]) -> "WindGains":
        values = {}
        for item in fields(cls):
            key = f"wind_{item.name}"
            if key not in config:
                continue
            value = config[key]
            if isinstance(value, bool) or not isinstance(value, numbers.Real) or not math.isfinite(value):
                raise ValueError(f"{key} must be a number, got {value!r}")
            values[item.name] = float(value)
        gains = cls(**values)
        for name, low, high in (("ki", 0.0, 0.5), ("integral_limit_mps", 0.0, 1.0), ("integral_zone_m", 0.1, 5.0), ("integral_max_speed_mps", 0.05, 1.0), ("observer_tau_s", 1.0, 20.0), ("observer_limit_mps", 0.0, 1.0), ("delay_s", 0.0, 1.0)):
            if not low <= getattr(gains, name) <= high:
                raise ValueError(f"wind_{name} must be from {low:g} to {high:g}, got {getattr(gains, name):g}")
        return gains


def _limited(vector: Tuple[float, float], limit: float) -> Tuple[float, float]:
    size = math.hypot(*vector)
    return vector if size <= limit or size <= 0.0 else (vector[0] * limit / size, vector[1] * limit / size)


class WindCompensation:
    """step(): the controller's command with the wind's push taken out, held to the same cap."""

    def __init__(self, gains: Optional[WindGains] = None) -> None:
        self.gains = gains or WindGains()
        self.reset()

    def reset(self) -> None:
        """Forget what was learnt: the next step starts from nothing (a new centring, or the target was lost)."""
        self.push: Tuple[float, float] = (0.0, 0.0)  # the observer's estimate of what the wind adds to the vehicle's velocity
        self.integral: Tuple[float, float] = (0.0, 0.0)
        self._sent: Deque[Tuple[float, Tuple[float, float]]] = deque()  # (when, what) of the commands sent and not yet old enough to compare
        self._asked: Optional[Tuple[float, float]] = None  # the newest command that is `delay_s` old: what the vehicle should be flying now
        self._last_s: Optional[float] = None
        self._out: Optional[Tuple[float, float]] = None  # the command sent on the step before, for the slew limit

    def step(
        self,
        command_en_mps: Sequence[float],
        error_en_m: Sequence[float],
        velocity_en_mps: Optional[Sequence[float]],
        now_s: float,
        speed_cap_mps: float,
        slew_mps2: Optional[float] = None,
        learn: bool = True,
    ) -> Tuple[float, float]:
        """The command to send: `command_en_mps` (the controller's own, for an error of `error_en_m`) less the push, plus the integral.

        With `slew_mps2` (the controller's own slew limit) the command sent changes by no more than that from one step to the
        next, the two terms' changes included. The first step after a reset is the controller's own command, which the
        controller has itself begun again from a stop.

        `learn` false (the track is only remembered, not seen on this frame): neither term changes on this step. An error
        to where the target was thought to be is not the wind's, and is not summed."""
        gains = self.gains
        if self._last_s is None or now_s - self._last_s > RESTART_AFTER_S or now_s < self._last_s:
            self.reset()
            dt = NOMINAL_TICK_S
        else:
            dt = now_s - self._last_s
        self._last_s = now_s
        base = (float(command_en_mps[0]), float(command_en_mps[1]))
        if not all(math.isfinite(value) for value in (*base, float(error_en_m[0]), float(error_en_m[1]))):
            return base  # nothing to learn from, and nothing of ours is added to what is not a number

        # The observer: what the vehicle did, less what it was asked to do `delay_s` ago.
        asked = None
        while self._sent and now_s - self._sent[0][0] >= gains.delay_s:
            asked = self._sent.popleft()[1]
        if asked is not None:
            self._asked = asked
        asked = self._asked
        if learn and velocity_en_mps is not None and asked is not None and all(math.isfinite(float(value)) for value in velocity_en_mps[:2]):
            share = min(1.0, dt / gains.observer_tau_s)
            seen = (float(velocity_en_mps[0]) - asked[0], float(velocity_en_mps[1]) - asked[1])
            self.push = _limited((self.push[0] + share * (seen[0] - self.push[0]), self.push[1] + share * (seen[1] - self.push[1])), gains.observer_limit_mps)

        # The integral: only close in, where an error that stays is the wind's and not the way there; and not while the command is
        # at its cap, where summing would only store up an overshoot for when the vehicle arrives (found in the loop simulator: a
        # capped approach against the wind arrived with the integral full and sat half a metre past the target).
        cap = max(0.0, float(speed_cap_mps))
        moving = velocity_en_mps is not None and all(math.isfinite(float(value)) for value in velocity_en_mps[:2]) and math.hypot(float(velocity_en_mps[0]), float(velocity_en_mps[1])) > gains.integral_max_speed_mps
        if learn and math.hypot(float(error_en_m[0]), float(error_en_m[1])) <= gains.integral_zone_m and not moving:
            summed = _limited((self.integral[0] + gains.ki * float(error_en_m[0]) * dt, self.integral[1] + gains.ki * float(error_en_m[1]) * dt), gains.integral_limit_mps)
            if math.hypot(base[0] - self.push[0] + summed[0], base[1] - self.push[1] + summed[1]) < cap or math.hypot(*summed) < math.hypot(*self.integral):
                self.integral = summed

        out = _limited((base[0] - self.push[0] + self.integral[0], base[1] - self.push[1] + self.integral[1]), cap)
        if slew_mps2 is not None and self._out is not None and math.isfinite(slew_mps2) and slew_mps2 > 0.0:
            # The controller's slew limit holds its own command; this holds the sum to the same limit, so that neither term
            # (a velocity report gone wild moves the observer fast) can step the command the flight controller is sent.
            change = (out[0] - self._out[0], out[1] - self._out[1])
            size, allowed = math.hypot(*change), slew_mps2 * dt
            if size > allowed:
                out = _limited((self._out[0] + change[0] * allowed / size, self._out[1] + change[1] * allowed / size), cap)
        self._out = out
        self._sent.append((now_s, out))
        return out
