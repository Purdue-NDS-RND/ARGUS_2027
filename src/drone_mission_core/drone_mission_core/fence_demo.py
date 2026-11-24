"""The geofence demonstration (OPS-02): fly at the fence, so that the flight controller is seen to stop the vehicle. No ROS imports.

The readiness video has to show the fence acting. This rule holds the
vehicle still for a few seconds and then commands one steady velocity along
a compass bearing, towards the flight boundary, and keeps commanding it. What
ArduCopter 4.6 then does, measured in the sim
(docs/results/2026-11-23-readiness-demo.md):

  held      Its avoidance (AVOID_ENABLE, on in SITL and on the aircraft)
            slows the vehicle and stops it at the fence's margin, 2 m inside
            the line, with the velocity still commanded. Every flight showed
            this, at 2 m/s and at 5 m/s, and the mission's own flight stayed
            held through a 30 s push.
  breached  In one flight (sim/fence_check.py's, which commands the same
            velocity over MAVLink directly) the vehicle went over the line
            after 14 s of being held, and the fence's action (RTL) took it
            within a tenth of a second, 1 m outside. Why that flight crossed
            and the mission's did not is not known.

So the rule does not stop at the first sign of the hold, and takes either
ending. A vehicle stopped within HELD_WINDOW_M of the boundary is at the
fence: the velocity stays commanded for up to HOLD_PUSH_S. If the fence's
action takes the vehicle in that time, the mission
(drone_mission_demo/missions/fence_demo_mission.py) sees the mode change and
stops commanding: a breach, answered. If the time runs out with the vehicle
still held and still under command, the rule says `held`: the flight
controller kept it inside, and the mission asks for the way home.

The rule's other job is the case where the fence does neither. The run has a
length, `max_run_m`, beyond the boundary at `boundary_m` (both from the day's
coordinates: tools/field_setup.py): a vehicle that has flown that far and is
still under command has no working fence in front of it, and the run ends as
a failure, holding. So does a run that stops gaining ground anywhere but at
the fence, one that leaves its line sideways (the avoidance slides a vehicle
along a fence it meets at an angle; the bearing is to be square to the edge,
which tools/field_setup.py checks), one that takes too long, and one whose
clock steps.

A velocity is commanded, not a position: ArduCopter in GUIDED refuses a
position target outside its fence and stays put where it is, which would
show nothing (sim/fence_check.py found the same).

Behind the mission flag `fence_demo`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

FLAG = "fence_demo"
MAX_TICK_GAP_S = 1.0  # between two updates: more than this, or time running backwards, and the run cannot be trusted
TIMEOUT_FACTOR = 2.0  # the run may take this many times its length over its speed, and no longer
STALL_S = 5.0  # a run that has gained less than STALL_GAIN_M in this long has been stopped by something
STALL_GAIN_M = 0.5
HELD_WINDOW_M = 8.0  # stopped no further than this short of the boundary is held by the fence: its margin, the avoidance's own, GPS error
HELD_PAST_M = 1.0  # and no further than this past where the boundary is said to be
CROSS_TRACK_M = 10.0  # further than this to the side of the run's line, the vehicle is not flying the run (sliding along the fence, or blown off)
HOLD_PUSH_S = 30.0  # stopped at the fence, the velocity stays commanded this long: twice the 14 s after which one sim flight crossed


@dataclass(frozen=True)
class DemoPlan:
    bearing_deg: float  # compass bearing of the run: degrees clockwise from north
    boundary_m: float  # along the bearing, from where the run begins: where the flight boundary is
    max_run_m: float  # along the bearing: this far and still under command is a failure (the fence did neither)
    speed_mps: float = 5.0  # the speed the sim flew it at
    settle_s: float = 3.0  # held still before the run


@dataclass(frozen=True)
class Command:
    east_mps: float = 0.0
    north_mps: float = 0.0
    running: bool = False  # the run towards the fence has begun
    held: bool = False  # the run is over: the vehicle is stopped at the fence, still under command
    failed: bool = False
    reason: str = ""


_LIMITS = {
    "bearing_deg": (0.0, 360.0),
    "boundary_m": (10.0, 485.0),
    "max_run_m": (15.0, 500.0),
    "speed_mps": (0.5, 6.0),  # the avoidance stopped 5 m/s inside the line in the sim; faster than 6 is not flown at a fence
    "settle_s": (1.0, 30.0),
}
_REQUIRED = ("bearing_deg", "boundary_m", "max_run_m")  # the site's own: there is no safe default direction to fly at a fence


def plan_from_config(config: dict) -> DemoPlan:
    """A plan from a mission's config. Raises ValueError for a missing or unknown setting, or a number outside what should be flown."""
    values = {}
    for key, value in (config or {}).items():
        if key not in _LIMITS:
            raise ValueError(f"unknown setting '{key}' (the fence demonstration has: {', '.join(_LIMITS)})")
        low, high = _LIMITS[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"{key} must be a number from {low:g} to {high:g}, got {value!r}")
        values[key] = float(value)
    missing = [key for key in _REQUIRED if key not in values]
    if missing:
        raise ValueError(f"{' and '.join(missing)} must be given: the direction and the length of the run are the site's (tools/field_setup.py writes them)")
    if values["max_run_m"] <= values["boundary_m"]:
        raise ValueError(f"max_run_m ({values['max_run_m']:g}) must be past boundary_m ({values['boundary_m']:g}): the run ends beyond the boundary, not before it")
    return DemoPlan(**values)


def run_timeout_s(plan: DemoPlan) -> float:
    """How long the run may take: twice its length at its speed, and the push at the fence."""
    return TIMEOUT_FACTOR * plan.max_run_m / plan.speed_mps + HOLD_PUSH_S


class FenceDemo:
    """Settle, then run along the bearing until told to stop from outside, or until the run's length or time is up."""

    def __init__(self, plan: DemoPlan) -> None:
        self.plan = plan
        self._start_s = None
        self._last_s = None
        self._run_start_s = None
        self._origin = None
        self.run_m = 0.0  # how far along the bearing the run has gone
        self._best_m = 0.0
        self._best_s = None  # when the run last gained ground
        self.at_fence_since = None  # since when the vehicle has been stopped at the fence with the velocity still commanded
        self._ended = None

    def update(self, now_s: float, east_m: float, north_m: float) -> Command:
        if self._ended is not None:
            return self._ended
        if self._last_s is not None and not 0.0 <= now_s - self._last_s <= MAX_TICK_GAP_S:
            return self._fail(f"the clock moved by {now_s - self._last_s:+.1f} s between two ticks")
        self._last_s = now_s
        if self._start_s is None:
            self._start_s = now_s
        if now_s - self._start_s < self.plan.settle_s:
            return Command()
        bearing = math.radians(self.plan.bearing_deg)
        east_unit, north_unit = math.sin(bearing), math.cos(bearing)
        if self._run_start_s is None:
            self._run_start_s, self._origin, self._best_s = now_s, (east_m, north_m), now_s
        self.run_m = (east_m - self._origin[0]) * east_unit + (north_m - self._origin[1]) * north_unit
        aside_m = abs((east_m - self._origin[0]) * north_unit - (north_m - self._origin[1]) * east_unit)
        if aside_m > CROSS_TRACK_M:
            # The aircraft's avoidance slides along a fence it meets at an angle: the run would go on gaining ground
            # along the line while the vehicle travels down the fence, over ground nobody chose.
            return self._fail(f"the vehicle is {aside_m:.1f} m to the side of the run's line ({self.run_m:.1f} m along it): it is not flying at the fence but along something")
        if self.run_m >= self._best_m + STALL_GAIN_M:
            self._best_m, self._best_s = self.run_m, now_s
            self.at_fence_since = None  # gaining ground again
        elif now_s - self._best_s > STALL_S:
            short_m = self.plan.boundary_m - self.run_m
            if not -HELD_PAST_M <= short_m <= HELD_WINDOW_M:
                return self._fail(f"the vehicle stopped {self.run_m:.1f} m into the run, {short_m:.1f} m from the boundary, and is still under command: it is not the fence that holds it")
            # Stopped at the fence: the flight controller holds it. The velocity stays commanded (the vehicle may yet
            # go over, and the fence's action take it); when the push's time is up it is held for good.
            self.at_fence_since = self._best_s if self.at_fence_since is None else self.at_fence_since
            if now_s - self.at_fence_since > HOLD_PUSH_S:
                self._ended = Command(held=True, running=True, reason=f"stopped {self.run_m:.1f} m into the run, {short_m:.1f} m short of the boundary, for {HOLD_PUSH_S:.0f} s with {self.plan.speed_mps:.1f} m/s still commanded")
                return self._ended
        if self.run_m >= self.plan.max_run_m:
            return self._fail(f"the run reached {self.run_m:.1f} m (its length is {self.plan.max_run_m:g} m) and the vehicle is still under command: the fence did not act")
        if now_s - self._run_start_s > run_timeout_s(self.plan):
            return self._fail(f"the run has taken {now_s - self._run_start_s:.0f} s for {self.run_m:.1f} m (at most {run_timeout_s(self.plan):.0f} s): the vehicle is not following")
        return Command(self.plan.speed_mps * east_unit, self.plan.speed_mps * north_unit, running=True)

    def _fail(self, reason: str) -> Command:
        self._ended = Command(failed=True, reason=reason, running=self._run_start_s is not None)
        return self._ended
