"""Decide when a flight in the sim is over, on sim time. No ROS."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Callable

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gps"))
from flight_log import LANDED_ON_GROUND, FlightTracker  # noqa: E402  (sim/gps: armed, then disarmed on the ground for 3 s)

DONE, TIMEOUT, NEVER_ARMED = "DONE", "TIMEOUT", "NEVER_ARMED"
ON_GROUND_HEIGHT_M = 0.3  # true height above where the vehicle started, below which it is down


class EndWatch:
    """Flight over, timed out, or never armed. Every duration is in sim seconds.

    `clock` returns sim time. The sim runs slower than real time on a club
    laptop and stops during a suspend; wall-clock timeouts would fire early.
    After the disarm the run goes on for `settle_s`, so a dropped payload or
    a set-down bucket has come to rest when the recording stops.

    "On the ground" is the autopilot's landed state when it sends one. It is
    only streamed if something asked for it (the 2026 takeoff service does);
    a flight flown by another tool never reports it. So `height`, the
    vehicle's true height above where it started, also counts: disarmed and
    below 0.3 m is on the ground.
    """

    def __init__(
        self,
        clock: Callable[[], float],
        timeout_s: float,
        arm_timeout_s: float = 300.0,
        settle_s: float = 5.0,
        height: Callable[[], float | None] | None = None,
    ) -> None:
        self.clock = clock
        self.height = height
        self.timeout_s = timeout_s
        self.arm_timeout_s = arm_timeout_s
        self.settle_s = settle_s
        self.tracker = FlightTracker()
        self.start = clock()

    def update(self, armed: bool | None = None, landed_state: int | None = None) -> None:
        self.tracker.update(self.clock(), armed=armed, landed_state=landed_state)

    def check(self) -> str | None:
        now = self.clock()
        tracker = self.tracker
        if self.height is not None and tracker.disarm_time is not None and not tracker.armed and tracker.landed_state != LANDED_ON_GROUND:
            height = self.height()
            if height is not None and height < ON_GROUND_HEIGHT_M:
                tracker.landed_state = LANDED_ON_GROUND
        if tracker.finished(now) and now - tracker.disarm_time >= self.settle_s:
            return DONE
        elapsed = now - self.start
        if tracker.arm_time is None and elapsed > self.arm_timeout_s:
            return NEVER_ARMED
        if elapsed > self.timeout_s:
            return TIMEOUT
        return None
