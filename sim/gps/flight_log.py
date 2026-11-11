#!/usr/bin/env python3
"""Read one sim/gps/run_mission.sh run and say whether the flight passed.

No ROS imports: flight_monitor.py feeds FlightTracker, and the summary reads
only the text files a run leaves in its folder:

  expected.txt  mission names from the sequence YAML, one per line
  launch.log    output of the mission launch (executor and takeoff service)
  state.log     "<epoch> armed=<0|1> mode=<mode> landed=<n> mono=<s>" from
                flight_monitor.py (runs before mono= was added have no mono=)
  rtf.log       "<epoch> <real_time_factor>" samples of Gazebo's world stats

Flight time comes from the monotonic stamps (mono=), not the epoch ones: the
wall clock jumps forward across a laptop suspend, while the monotonic clock
stops, as SITL and Gazebo do. The epoch stamps line state.log up with rtf.log.

Usage: python3 sim/gps/flight_log.py <run folder>
"""

from __future__ import annotations

import re
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

LANDED_ON_GROUND = 1  # mavros_msgs/ExtendedState.LANDED_STATE_ON_GROUND
AIRBORNE_STATES = {2, 3, 4}  # IN_AIR, TAKEOFF, LANDING
DISARM_HOLD_S = 3.0  # a disarm counts only once it has lasted this long
ARM_TIMEOUT_S = 300.0  # flight_monitor.py gives up if nothing arms by then
FLIGHT_TIMEOUT_S = 1800.0  # ...or if the flight is not over by then

# flight_monitor.py exit codes
DONE, TIMED_OUT, NEVER_ARMED = 0, 2, 3

_COMPLETED = re.compile(r"Mission '([^']+)' completed")
_SEQUENCE_DONE = "Mission sequence complete"
_CLOSEST = re.compile(r"\[([^\]]+)\]\s+(\S+): closest ([0-9.]+) m")
_CIRCUIT = re.compile(r"\[([^\]]+)\]\s+circuit elapsed: ([0-9.]+) s")
_PROBLEM = re.compile(r"\[(ERROR|FATAL)\]|\bfail(ed|ure)\b|\babort", re.IGNORECASE)
_STATE = re.compile(r"^([0-9.]+) armed=([01]) mode=(\S*) landed=(-?\d+)(?: mono=([0-9.]+))?")


class FlightTracker:
    """Decide when a flight is over from MAVROS state updates.

    Over means: the vehicle armed, then disarmed while on the ground. The
    monitor stops the launch at that point, whether or not it ever left the
    ground: our takeoff service sends the takeoff command once, so after an
    on-ground disarm nothing more will happen. `airborne` tells the two apart.

    The disarm has to last DISARM_HOLD_S: a single stale or glitched state
    message must not end a flight (it once stopped a launch mid-climb).
    """

    def __init__(self) -> None:
        self.armed = False
        self.landed_state = 0
        self.arm_time: float | None = None
        self.disarm_time: float | None = None
        self.airborne = False

    def update(self, now: float, armed: bool | None = None, landed_state: int | None = None) -> None:
        if landed_state is not None:
            self.landed_state = landed_state
            if self.armed and landed_state in AIRBORNE_STATES:
                self.airborne = True
        if armed is None or armed == self.armed:
            return
        self.armed = armed
        if armed:
            if self.arm_time is None:
                self.arm_time = now
            self.disarm_time = None
        elif self.arm_time is not None:
            self.disarm_time = now

    def finished(self, now: float) -> bool:
        return (
            self.arm_time is not None
            and self.disarm_time is not None
            and not self.armed
            and self.landed_state == LANDED_ON_GROUND
            and now - self.disarm_time >= DISARM_HOLD_S
        )


class FlightWatch:
    """Decide when flight_monitor.py stops: flight over, or a timeout.

    Elapsed time is measured on time.monotonic(), never time.time(). The wall
    clock jumps forward by however long the laptop was suspended, so on resume
    every timeout fired at once and the launch was stopped mid-flight. The
    monotonic clock stops during a suspend, as SITL and Gazebo do. The tracker
    must be fed now() stamps, so DISARM_HOLD_S is on the same clock.
    """

    def __init__(
        self,
        tracker: FlightTracker,
        timeout_s: float = FLIGHT_TIMEOUT_S,
        arm_timeout_s: float = ARM_TIMEOUT_S,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self.tracker = tracker
        self.timeout_s = timeout_s
        self.arm_timeout_s = arm_timeout_s
        self.clock = clock if clock is not None else time.monotonic
        self.start = self.clock()

    def now(self) -> float:
        return self.clock()

    def check(self) -> int | None:
        """None while the monitor should keep waiting, else its exit code."""
        now = self.clock()
        if self.tracker.finished(now):
            return DONE
        elapsed = now - self.start
        if self.tracker.arm_time is None and elapsed > self.arm_timeout_s:
            return NEVER_ARMED
        if elapsed > self.timeout_s:
            return TIMED_OUT
        return None


@dataclass
class RunSummary:
    expected: list[str]
    completed: list[str] = field(default_factory=list)
    sequence_complete: bool = False
    closest_m: list[tuple[str, float]] = field(default_factory=list)
    circuit_s: float | None = None
    problems: list[str] = field(default_factory=list)
    arm_time: float | None = None  # epoch, to line up with rtf.log
    disarm_time: float | None = None
    arm_mono: float | None = None  # monotonic, for the flight time
    disarm_mono: float | None = None
    final_landed: int | None = None
    airborne: bool = False
    rtf_samples: list[float] = field(default_factory=list)

    @property
    def flight_clock(self) -> str:
        return "monotonic" if self.arm_mono is not None else "wall-clock"

    @property
    def flight_s(self) -> float | None:
        if self.arm_time is None or self.disarm_time is None:
            return None
        if self.arm_mono is not None and self.disarm_mono is not None:
            return self.disarm_mono - self.arm_mono
        return self.disarm_time - self.arm_time

    @property
    def missing(self) -> list[str]:
        return [name for name in self.expected if name not in self.completed]

    @property
    def passed(self) -> bool:
        return (
            bool(self.expected)
            and not self.missing
            and self.sequence_complete
            and self.flight_s is not None
            and self.airborne
            and self.final_landed == LANDED_ON_GROUND
        )

    @property
    def rtf_mean(self) -> float | None:
        if not self.rtf_samples:
            return None
        return sum(self.rtf_samples) / len(self.rtf_samples)


def parse_launch_log(text: str, summary: RunSummary) -> None:
    for line in text.splitlines():
        if (match := _COMPLETED.search(line)) and match.group(1) not in summary.completed:
            summary.completed.append(match.group(1))
        if _SEQUENCE_DONE in line:
            summary.sequence_complete = True
        if match := _CLOSEST.search(line):
            summary.closest_m.append((match.group(2), float(match.group(3))))
        if match := _CIRCUIT.search(line):
            summary.circuit_s = float(match.group(2))
        if _PROBLEM.search(line):
            summary.problems.append(line.strip())


def parse_state_log(text: str, summary: RunSummary) -> None:
    """Replay the state lines twice: on epoch stamps and on monotonic ones.

    The monotonic replay counts only if every state line has a mono= stamp.
    """
    tracker = FlightTracker()
    mono_tracker: FlightTracker | None = FlightTracker()
    for line in text.splitlines():
        if match := _STATE.match(line.strip()):
            armed = match.group(2) == "1"
            landed = int(match.group(4))
            tracker.update(float(match.group(1)), armed=armed, landed_state=landed)
            if match.group(5) is None:
                mono_tracker = None
            elif mono_tracker is not None:
                mono_tracker.update(float(match.group(5)), armed=armed, landed_state=landed)
            summary.final_landed = landed
    summary.arm_time = tracker.arm_time
    summary.disarm_time = tracker.disarm_time
    summary.airborne = tracker.airborne
    if mono_tracker is not None:
        summary.arm_mono = mono_tracker.arm_time
        summary.disarm_mono = mono_tracker.disarm_time


def parse_rtf_log(text: str, summary: RunSummary) -> None:
    """Keep the samples taken while the vehicle was armed (the flight)."""
    start, end = summary.arm_time, summary.disarm_time
    for line in text.splitlines():
        parts = line.split()
        if len(parts) != 2:
            continue
        try:
            stamp, rtf = float(parts[0]), float(parts[1])
        except ValueError:
            continue
        if start is not None and stamp < start:
            continue
        if end is not None and stamp > end:
            continue
        summary.rtf_samples.append(rtf)


def summarize(run_dir: Path) -> RunSummary:
    def read(name: str) -> str:
        path = run_dir / name
        return path.read_text(errors="replace") if path.exists() else ""

    summary = RunSummary(expected=read("expected.txt").split())
    parse_launch_log(read("launch.log"), summary)
    parse_state_log(read("state.log"), summary)
    parse_rtf_log(read("rtf.log"), summary)
    return summary


def format_summary(summary: RunSummary) -> str:
    lines = [f"RESULT: {'PASS' if summary.passed else 'FAIL'}"]
    lines.append(f"missions completed: {', '.join(summary.completed) or 'none'}")
    if summary.missing:
        lines.append(f"missions not completed: {', '.join(summary.missing)}")
    lines.append(f"sequence complete: {'yes' if summary.sequence_complete else 'no'}")
    landed = {None: "unknown", LANDED_ON_GROUND: "on ground"}.get(
        summary.final_landed, f"landed_state {summary.final_landed}"
    )
    lines.append(f"armed -> disarmed: {'yes' if summary.flight_s is not None else 'no'}; final state {landed}")
    if summary.flight_s is not None and not summary.airborne:
        lines.append("left the ground: NO (armed and disarmed without taking off)")
    if summary.flight_s is not None:
        lines.append(f"flight time, arm to disarm ({summary.flight_clock}): {summary.flight_s:.1f} s")
    if summary.rtf_mean is not None:
        lines.append(
            f"Gazebo real-time factor while armed: mean {summary.rtf_mean:.2f}, "
            f"min {min(summary.rtf_samples):.2f}, max {max(summary.rtf_samples):.2f} "
            f"({len(summary.rtf_samples)} samples)"
        )
    for name, metres in summary.closest_m:
        lines.append(f"closest to {name}: {metres:.2f} m")
    if summary.circuit_s is not None:
        lines.append(f"circuit elapsed (wall-clock): {summary.circuit_s:.2f} s")
    if summary.problems:
        lines.append("log lines to look at:")
        lines.extend(f"  {line}" for line in summary.problems[:20])
    return "\n".join(lines)


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__.strip().splitlines()[-1], file=sys.stderr)
        return 2
    summary = summarize(Path(argv[1]))
    print(format_summary(summary))
    return 0 if summary.passed else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
