"""Pose history: where the vehicle was at any instant in the last five seconds. No ROS imports.

A camera frame is taken some tens of milliseconds before anything can act on
it. To put a detection on the ground the vehicle's pose at the moment of
capture is needed, not the latest one. `PoseBuffer` keeps the recent poses
and velocities from MAVROS and answers `pose_at(stamp)`:

  - between two samples: position linearly, attitude by SLERP;
  - None if the two samples are more than 0.12 s apart (data was lost there);
  - up to 30 ms past the newest sample: position carried on by the reported
    velocity, attitude by the turn between the last two samples;
  - None otherwise. It never answers with the latest pose as a stand-in.

Stamps are whole nanoseconds on the clock the samples were stamped with.
Samples may arrive a little out of order. A sample stamped well before the
newest one is not put among them: one such sample is ignored, and three in a
row mean the clock itself stepped back, so the history, which is on the old
clock, is dropped and started again.
"""

from __future__ import annotations

import bisect
import math
import threading
from dataclasses import dataclass
from typing import List, Optional, Tuple

HISTORY_S = 5.0
MAX_GAP_S = 0.12
MAX_EXTRAPOLATION_S = 0.030
MIN_TURN_SPAN_S = 0.010  # the attitude is carried on from two poses at least this far apart
CLOCK_STEP_SAMPLES = 3  # this many samples in a row from well before the newest: the clock stepped back

Vector = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]  # x, y, z, w


@dataclass(frozen=True)
class PoseAt:
    stamp_ns: int
    position: Vector  # east, north, up as MAVROS's local_position/pose gives them
    orientation: Quaternion
    extrapolated: bool  # True if the stamp was past the newest sample


def _finite(values) -> bool:
    try:
        return all(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) for value in values)
    except TypeError:
        return False


def _slerp(a: Quaternion, b: Quaternion, fraction: float) -> Quaternion:
    """From a to b the short way round; fraction may run past 1 to carry the turn on."""
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0.0:  # the same attitude written with the other sign
        b = tuple(-value for value in b)
        dot = -dot
    dot = min(1.0, dot)
    angle = math.acos(dot)
    if angle < 1.0e-6:
        mixed = tuple(x + fraction * (y - x) for x, y in zip(a, b))
    else:
        sin_angle = math.sin(angle)
        weight_a = math.sin((1.0 - fraction) * angle) / sin_angle
        weight_b = math.sin(fraction * angle) / sin_angle
        mixed = tuple(weight_a * x + weight_b * y for x, y in zip(a, b))
    norm = math.sqrt(sum(value * value for value in mixed))
    return tuple(value / norm for value in mixed)


class PoseBuffer:
    def __init__(self, history_s: float = HISTORY_S, max_gap_s: float = MAX_GAP_S, max_extrapolation_s: float = MAX_EXTRAPOLATION_S) -> None:
        self._history_ns = int(history_s * 1.0e9)
        self._max_gap_ns = int(round(max_gap_s * 1.0e9))
        self._max_extrapolation_ns = int(round(max_extrapolation_s * 1.0e9))
        self._lock = threading.Lock()
        self._pose_stamps: List[int] = []
        self._poses: List[Tuple[Vector, Quaternion]] = []
        self._velocity_stamps: List[int] = []
        self._velocities: List[Vector] = []
        self._min_turn_span_ns = int(round(MIN_TURN_SPAN_S * 1.0e9))
        self._behind = {"pose": 0, "velocity": 0}  # samples in a row from well before the newest
        self.clock_steps = 0

    def __len__(self) -> int:
        return len(self._pose_stamps)

    def oldest_ns(self) -> Optional[int]:
        return self._pose_stamps[0] if self._pose_stamps else None

    def newest_ns(self) -> Optional[int]:
        return self._pose_stamps[-1] if self._pose_stamps else None

    def _insert(self, kind: str, stamps: List[int], values: list, stamp_ns: int, value, history_ns: int) -> bool:
        if stamps and stamp_ns < stamps[-1] - self._max_gap_ns:
            # From well before the newest: a stray stamp, or the clock has stepped back.
            self._behind[kind] += 1
            if self._behind[kind] < CLOCK_STEP_SAMPLES:
                return False
            self.clock_steps += 1  # the clock stepped: what is stored is on the old clock
            self._pose_stamps.clear()
            self._poses.clear()
            self._velocity_stamps.clear()
            self._velocities.clear()
        self._behind[kind] = 0
        index = bisect.bisect_left(stamps, stamp_ns)
        if index < len(stamps) and stamps[index] == stamp_ns:
            return False  # that stamp is already stored
        stamps.insert(index, stamp_ns)
        values.insert(index, value)
        drop = bisect.bisect_left(stamps, stamps[-1] - history_ns)
        if drop:
            del stamps[:drop]
            del values[:drop]
        return True

    def add_pose(self, stamp_ns, position, orientation) -> bool:
        """Store one pose. False if it was not stored (bad stamp or numbers, repeated stamp, too old)."""
        if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int):
            return False
        try:
            position, orientation = tuple(position), tuple(orientation)
        except TypeError:
            return False
        if len(position) != 3 or len(orientation) != 4 or not _finite(position) or not _finite(orientation):
            return False
        norm = math.sqrt(sum(value * value for value in orientation))
        if norm < 1.0e-9:
            return False
        orientation = tuple(value / norm for value in orientation)
        position = tuple(float(value) for value in position)
        with self._lock:
            return self._insert("pose", self._pose_stamps, self._poses, stamp_ns, (position, orientation), self._history_ns)

    def add_velocity(self, stamp_ns, linear) -> bool:
        """Store one velocity (east, north, up, in m/s), as MAVROS's local_position/velocity_local gives it."""
        if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int):
            return False
        try:
            linear = tuple(linear)
        except TypeError:
            return False
        if len(linear) != 3 or not _finite(linear):
            return False
        with self._lock:
            return self._insert("velocity", self._velocity_stamps, self._velocities, stamp_ns, tuple(float(value) for value in linear), self._history_ns)

    def _velocity_for(self, stamp_ns: int) -> Optional[Vector]:
        """The newest velocity at or before stamp_ns, if it is recent enough to belong to that moment."""
        index = bisect.bisect_right(self._velocity_stamps, stamp_ns) - 1
        if index < 0 or stamp_ns - self._velocity_stamps[index] > self._max_gap_ns:
            return None
        return self._velocities[index]

    def pose_at(self, stamp_ns) -> Optional[PoseAt]:
        """The pose at stamp_ns, or None if the buffer cannot say (see the module text)."""
        if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int):
            return None
        with self._lock:
            stamps = self._pose_stamps
            if not stamps or stamp_ns < stamps[0]:
                return None
            newest = stamps[-1]
            if stamp_ns > newest:
                return self._extrapolate(stamp_ns)
            index = bisect.bisect_left(stamps, stamp_ns)
            if stamps[index] == stamp_ns:
                position, orientation = self._poses[index]
                return PoseAt(stamp_ns, position, orientation, False)
            before, after = stamps[index - 1], stamps[index]
            if after - before > self._max_gap_ns:
                return None
            fraction = (stamp_ns - before) / (after - before)
            (p0, q0), (p1, q1) = self._poses[index - 1], self._poses[index]
            position = tuple(a + fraction * (b - a) for a, b in zip(p0, p1))
            return PoseAt(stamp_ns, position, _slerp(q0, q1, fraction), False)

    def _extrapolate(self, stamp_ns: int) -> Optional[PoseAt]:
        stamps = self._pose_stamps
        newest = stamps[-1]
        ahead_ns = stamp_ns - newest
        if ahead_ns > self._max_extrapolation_ns or len(stamps) < 2:
            return None
        # The turn to carry on is taken from the newest pose and the nearest one at least 10 ms before it:
        # two poses a few ms apart (one stamped late) would turn a small difference into a large rate.
        earlier = bisect.bisect_right(stamps, newest - self._min_turn_span_ns) - 1
        if earlier < 0:
            return None
        step_ns = newest - stamps[earlier]
        if step_ns > self._max_gap_ns:
            return None
        velocity = self._velocity_for(newest)
        if velocity is None:
            return None
        (_p0, q0), (p1, q1) = self._poses[earlier], self._poses[-1]
        ahead_s = ahead_ns * 1.0e-9
        position = tuple(value + rate * ahead_s for value, rate in zip(p1, velocity))
        return PoseAt(stamp_ns, position, _slerp(q0, q1, 1.0 + ahead_ns / step_ns), True)

    def velocity_at(self, stamp_ns) -> Optional[Vector]:
        """The velocity at stamp_ns, interpolated between two samples no more than 0.12 s apart, or None."""
        if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int):
            return None
        with self._lock:
            stamps = self._velocity_stamps
            if not stamps or stamp_ns < stamps[0] or stamp_ns > stamps[-1]:
                return None
            index = bisect.bisect_left(stamps, stamp_ns)
            if stamps[index] == stamp_ns:
                return self._velocities[index]
            before, after = stamps[index - 1], stamps[index]
            if after - before > self._max_gap_ns:
                return None
            fraction = (stamp_ns - before) / (after - before)
            return tuple(a + fraction * (b - a) for a, b in zip(self._velocities[index - 1], self._velocities[index]))
