"""What a sim run leaves for the judge: true poses and events on sim time.

The recorder writes `trace.jsonl`, one JSON object per line, either

  {"t": 12.35, "pose": "vehicle", "e": 1.0, "n": 2.0, "u": 3.0, "yaw": 1.5708}
  {"t": 30.10, "event": "release", "fake": true}

`t` is sim time in seconds. Poses are Gazebo's true model positions in the
world frame (east, north, up in metres from the world origin), never the EKF.
The vehicle's `u` has `vehicle_ground_offset_m` taken off (the height of the
model origin when it stands on the ground), so 0 means on the ground.
`yaw` is on the vehicle's poses only: its true heading in radians from east,
counter-clockwise. Older traces have none.

Events the judges read:

  armed, disarmed        from the flight controller's state
  release                payload or bucket released; "fake": true for fake_drop
  list_written           the localisation list was written or rewritten
  contact                "object" touched down, with "impulse_ns"
  failsafe               "what" the flight controller reported
  fence_breach           the flight controller reported a breach
  gcs_command, rc_override   human input; "what" says which command
  mode                   the flight mode changed, with "mode" (geofence judge)

Any other event is kept and ignored. The runner also writes `clock` (sim
time against "wall" time, for the real-time factor), `launch` and `end`.
"""

from __future__ import annotations

import bisect
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

VEHICLE = "vehicle"

# Above this the vehicle counts as flying. A flight is an arm-to-disarm span
# in which it got this high; delivery's touchdown and relaunch stay one flight.
AIRBORNE_HEIGHT_M = 0.5


@dataclass(frozen=True)
class Pose:
    t: float
    e: float
    n: float
    u: float
    yaw: float | None = None  # true heading from east, counter-clockwise, in radians; the vehicle's, when the recorder wrote it


@dataclass(frozen=True)
class Flight:
    t_arm: float
    t_disarm: float


class Record:
    def __init__(self, poses: dict[str, list[Pose]], events: list[dict]) -> None:
        self.poses = {name: sorted(track, key=lambda pose: pose.t) for name, track in poses.items()}
        self.events = sorted(events, key=lambda item: item["t"])
        self._times = {name: [pose.t for pose in track] for name, track in self.poses.items()}

    @classmethod
    def from_lines(cls, lines: Iterable[dict], vehicle_ground_offset_m: float = 0.0) -> "Record":
        poses: dict[str, list[Pose]] = {}
        events: list[dict] = []
        for number, line in enumerate(lines, start=1):
            try:
                t = float(line["t"])
                if "pose" in line:
                    up = float(line["u"])
                    if line["pose"] == VEHICLE:
                        up -= vehicle_ground_offset_m
                    yaw = None if line.get("yaw") is None else float(line["yaw"])
                    poses.setdefault(str(line["pose"]), []).append(
                        Pose(t, float(line["e"]), float(line["n"]), up, yaw)
                    )
                elif "event" in line:
                    events.append({**line, "t": t, "event": str(line["event"])})
                else:
                    raise KeyError("pose or event")
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f"trace line {number}: bad or missing {error}: {line!r}") from error
        return cls(poses, events)

    @classmethod
    def load(cls, path: Path, vehicle_ground_offset_m: float = 0.0) -> "Record":
        lines = []
        with open(path) as handle:
            for number, text in enumerate(handle, start=1):
                if not text.strip():
                    continue
                try:
                    lines.append(json.loads(text))
                except json.JSONDecodeError as error:
                    raise ValueError(f"{path}: line {number} is not JSON: {error}") from error
        return cls.from_lines(lines, vehicle_ground_offset_m)

    # --- poses ---

    def track(self, name: str, t0: float | None = None, t1: float | None = None) -> list[Pose]:
        track = self.poses.get(name, [])
        if t0 is None and t1 is None:
            return track
        times = self._times.get(name, [])
        lo = 0 if t0 is None else bisect.bisect_left(times, t0)
        hi = len(track) if t1 is None else bisect.bisect_right(times, t1)
        return track[lo:hi]

    def pose_at(self, name: str, t: float) -> Pose | None:
        """The pose at time t, interpolated; the first or last pose outside the track."""
        track = self.poses.get(name)
        if not track:
            return None
        times = self._times[name]
        i = bisect.bisect_right(times, t)
        if i == 0:
            return track[0]
        if i == len(track):
            return track[-1]
        a, b = track[i - 1], track[i]
        f = 0.0 if b.t == a.t else (t - a.t) / (b.t - a.t)
        return Pose(t, a.e + (b.e - a.e) * f, a.n + (b.n - a.n) * f, a.u + (b.u - a.u) * f)

    def first(self, name: str) -> Pose | None:
        track = self.poses.get(name)
        return track[0] if track else None

    def final(self, name: str) -> Pose | None:
        track = self.poses.get(name)
        return track[-1] if track else None

    # --- events ---

    def events_of(self, *kinds: str, t0: float | None = None, t1: float | None = None) -> list[dict]:
        return [
            item
            for item in self.events
            if item["event"] in kinds
            and (t0 is None or item["t"] >= t0)
            and (t1 is None or item["t"] <= t1)
        ]

    # --- flights ---

    def flights(self, airborne_height_m: float = AIRBORNE_HEIGHT_M) -> list[Flight]:
        """Arm-to-disarm spans in which the vehicle left the ground.

        A trace with no arm or disarm events at all (a hand-made record) is
        one span over the whole vehicle track. An arm with no disarm after it
        runs to the end of the track.
        """

        vehicle = self.poses.get(VEHICLE, [])
        if not vehicle:
            return []
        changes = self.events_of("armed", "disarmed")
        spans: list[tuple[float, float]] = []
        if not changes:
            spans.append((vehicle[0].t, vehicle[-1].t))
        else:
            t_arm: float | None = None
            for item in changes:
                if item["event"] == "armed" and t_arm is None:
                    t_arm = item["t"]
                elif item["event"] == "disarmed" and t_arm is not None:
                    spans.append((t_arm, item["t"]))
                    t_arm = None
            if t_arm is not None:
                spans.append((t_arm, max(t_arm, vehicle[-1].t)))
        return [
            Flight(t0, t1)
            for t0, t1 in spans
            if any(pose.u > airborne_height_m for pose in self.track(VEHICLE, t0, t1))
        ]
