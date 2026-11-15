"""Write the run's trace (see sim/judge/record.py) and read the sim's speed from it. No ROS."""

from __future__ import annotations

import json
from pathlib import Path


class TraceWriter:
    """Appends trace lines as they happen; every line is on disk at once."""

    def __init__(self, path: Path, pose_interval_s: float = 0.05) -> None:
        self._file = open(path, "a", buffering=1)
        self._interval = pose_interval_s
        self._last_pose: dict[str, float] = {}

    def pose(self, t: float, name: str, e: float, n: float, u: float) -> bool:
        """Record a true pose, at most once per pose interval for each name."""
        last = self._last_pose.get(name)
        if last is not None and t - last < self._interval - 1.0e-9:
            return False
        self._last_pose[name] = t
        self._write({"t": round(t, 4), "pose": name, "e": round(e, 4), "n": round(n, 4), "u": round(u, 4)})
        return True

    def event(self, t: float, kind: str, **fields) -> None:
        self._write({"t": round(t, 4), "event": kind, **fields})

    def clock(self, t: float, wall: float) -> None:
        """A mark tying sim time to wall time, for the real-time factor."""
        self.event(t, "clock", wall=round(wall, 3))

    def _write(self, line: dict) -> None:
        self._file.write(json.dumps(line) + "\n")

    def close(self) -> None:
        self._file.close()


def _marks(record) -> list[tuple[float, float]]:
    return [(item["t"], float(item["wall"])) for item in record.events_of("clock")]


def real_time_factor(record, t0: float, t1: float) -> float | None:
    """Sim seconds per wall second between sim times t0 and t1, from the clock marks."""
    inside = [mark for mark in _marks(record) if t0 <= mark[0] <= t1]
    if len(inside) < 2 or inside[-1][1] <= inside[0][1]:
        return None
    return (inside[-1][0] - inside[0][0]) / (inside[-1][1] - inside[0][1])


def sim_time_at(record, wall: float) -> float | None:
    """The sim time at a wall-clock time, between the clock marks; None outside them."""
    marks = _marks(record)
    for (t0, w0), (t1, w1) in zip(marks, marks[1:]):
        if w0 <= wall <= w1:
            return t0 if w1 == w0 else t0 + (t1 - t0) * (wall - w0) / (w1 - w0)
    return None
