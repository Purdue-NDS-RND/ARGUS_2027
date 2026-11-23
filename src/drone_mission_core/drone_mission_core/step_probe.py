"""The command step probe (PLT-05): velocity steps for timing the flight controller's response. No ROS imports.

The latency probe's second test needs commands that jump: a delivery's own
are too smooth to time anything against (the centring controller's largest
change between two ticks is a few centimetres a second). This rule holds
the vehicle still, commands a short burst of velocity, holds it still
again, and repeats, there and back so that it ends where it began. Flown
with the topics recorded, sim/replay/command_delay.py reads the actuation
delay off the bag: each step of the command to the first tilt.

Behind the mission flag `command_step_probe`. The mission
(drone_mission_demo/missions/step_probe_mission.py) is the thin side.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

FLAG = "command_step_probe"
MAX_STEP_TRAVEL_M = 3.0  # speed times duration: how far one step may carry the vehicle
MAX_TICK_GAP_S = 1.0  # between two updates: more than this, or time running backwards, and the schedule cannot be trusted


@dataclass(frozen=True)
class StepPlan:
    steps: int = 6
    step_speed_mps: float = 1.0
    step_s: float = 1.0
    rest_s: float = 3.0  # zero velocity before each step and after the last: the baseline the response is seen against
    max_drift_m: float = 10.0  # from where the probe began; further than this ends it as a failure
    heading_deg: float = 0.0  # direction of the first step, anticlockwise from east


@dataclass(frozen=True)
class Command:
    east_mps: float = 0.0
    north_mps: float = 0.0
    done: bool = False
    failed: bool = False
    reason: str = ""


_LIMITS = {
    "steps": (1, 20),
    "step_speed_mps": (0.3, 2.0),
    "step_s": (0.3, 2.0),
    "rest_s": (2.0, 30.0),
    "max_drift_m": (3.0, 20.0),
    "heading_deg": (-360.0, 360.0),
}


def plan_from_config(config: dict) -> StepPlan:
    """A plan from a mission's config. Raises ValueError for a setting it does not know or a number outside what should be flown."""
    values = {}
    for key, value in (config or {}).items():
        if key not in _LIMITS:
            raise ValueError(f"unknown setting '{key}' (the step probe has: {', '.join(_LIMITS)})")
        low, high = _LIMITS[key]
        whole = key == "steps"
        if isinstance(value, bool) or not isinstance(value, int if whole else (int, float)) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"{key} must be {'a whole number' if whole else 'a number'} from {low} to {high}, got {value!r}")
        values[key] = value if whole else float(value)
    plan = StepPlan(**values)
    if plan.step_speed_mps * plan.step_s > MAX_STEP_TRAVEL_M:
        raise ValueError(f"step_speed_mps x step_s is {plan.step_speed_mps * plan.step_s:.1f} m: a step may carry the vehicle {MAX_STEP_TRAVEL_M:.0f} m at most")
    return plan


def duration_s(plan: StepPlan) -> float:
    return plan.rest_s + plan.steps * (plan.step_s + plan.rest_s)


class StepProbe:
    """update(now, east, north) each tick -> the velocity to command. Time is whatever clock the caller's `now` is on."""

    def __init__(self, plan: StepPlan) -> None:
        self.plan = plan
        self._start_s: float | None = None
        self._last_s = 0.0
        self._anchor = (0.0, 0.0)
        self._ended: Command | None = None
        self.steps_begun = 0

    def update(self, now_s: float, east_m: float, north_m: float) -> Command:
        if self._ended is not None:
            return self._ended
        plan = self.plan
        if self._start_s is None:
            self._start_s, self._last_s, self._anchor = now_s, now_s, (east_m, north_m)
        gap, self._last_s = now_s - self._last_s, now_s
        if gap < 0.0 or gap > MAX_TICK_GAP_S:
            # The steps are timed off this clock: one that steps (or an executor that stalled) could repeat a step or run on.
            self._ended = Command(failed=True, reason=f"the clock moved {gap:+.2f} s between two updates")
            return self._ended
        away = math.hypot(east_m - self._anchor[0], north_m - self._anchor[1])
        if away > plan.max_drift_m:
            self._ended = Command(failed=True, reason=f"the vehicle is {away:.1f} m from where the probe began (limit {plan.max_drift_m:.1f} m)")
            return self._ended
        elapsed = now_s - self._start_s
        if elapsed >= duration_s(plan):
            self._ended = Command(done=True)
            return self._ended
        cycle = plan.step_s + plan.rest_s
        index, within = divmod(elapsed, cycle)
        if within < plan.rest_s:
            return Command()
        self.steps_begun = max(self.steps_begun, int(index) + 1)
        sign = 1.0 if int(index) % 2 == 0 else -1.0
        heading = math.radians(plan.heading_deg)
        east, north = sign * plan.step_speed_mps * math.cos(heading), sign * plan.step_speed_mps * math.sin(heading)
        return Command(east_mps=round(east, 9) + 0.0, north_mps=round(north, 9) + 0.0)  # + 0.0: no minus zero in the log
