"""Stale-data guard: is the time base good enough to steer on? No ROS imports.

A vision mission lines up a camera frame with the vehicle's pose at the
moment the frame was taken. That only works while three things hold:

  - poses keep arriving and are recent, and their stamps really are
    flight-controller time moved onto the ROS clock by MAVROS's time sync.
    Until its filter has converged MAVROS stamps a pose with the time it
    arrived, which always looks recent and hides the link delay,
  - frames keep arriving and are recent,
  - the time-sync offset is steady. A step in it means one of the two clocks
    jumped, and every stamp made around the step is wrong by that much.

`TimeHealth` is fed the stamps and the time-sync status and answers, for any
instant, HEALTHY or UNHEALTHY with the reasons. Missions that have the
`time_health_guard` flag on treat UNHEALTHY as "no target": they hold, and
never descend (time_guard.py). With the flag off nothing here runs.

All times are on one clock, the ROS clock of the node that owns the monitor
(sim time in the sim): seconds, except pose stamps and time-sync offsets,
which are whole nanoseconds so they can be compared exactly.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Any, Iterable, Optional, Tuple

# The limits from the FND-03 spec.
POSE_AGE_LIMIT_S = 0.100
FRAME_AGE_LIMIT_S = 0.250
OFFSET_JUMP_LIMIT_S = 0.010
# After a jump the report stays UNHEALTHY until the offset has gone this long
# without another: the stamps made around the jump are still in use.
OFFSET_SETTLE_S = 1.0
# No time-sync status for this long is UNHEALTHY too. Without time sync MAVROS
# stamps a pose with its arrival time, which always looks fresh, so the
# pose-age check alone would see nothing wrong.
TIMESYNC_SILENCE_LIMIT_S = 2.0
# The newest pose whose stamp was seen to be synchronised flight-controller
# time must be no older than this, or the stamps are arrival times.
SYNC_PROOF_LIMIT_S = 0.5
_RECENT = 4  # offsets and pose stamps kept to match one against the other


def is_synced(stamp_ns: int, offsets_ns: Iterable[int]) -> bool:
    """Whether a MAVROS stamp is flight-controller time plus one of the given time-sync offsets.

    The flight controller counts whole milliseconds since boot and MAVROS
    adds its offset estimate in nanoseconds, so a synchronised stamp minus
    the offset it was made with is a whole number of milliseconds. A stamp
    taken from the ROS clock on arrival is not, except by a one-in-a-million
    accident. MAVROS uses an offset only when it is above zero.
    """
    return any(offset > 0 and (stamp_ns - offset) % 1_000_000 == 0 for offset in offsets_ns)


@dataclass(frozen=True)
class TimeHealthReport:
    """One answer from the guard. `reasons` is empty exactly when `healthy` is True."""

    healthy: bool
    reasons: Tuple[str, ...]
    pose_age_s: Optional[float]
    frame_age_s: Optional[float]
    timesync_age_s: Optional[float]
    offset_ms: Optional[float]  # MAVROS's estimate of ROS clock minus flight-controller clock
    last_offset_step_ms: Optional[float]  # change between the two newest estimates
    round_trip_ms: Optional[float]  # of the newest time-sync exchange


def _number(value: Any) -> Optional[float]:
    """value as a finite float, or None if it is not one."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


class TimeHealth:
    """Watches pose age, frame age and the time-sync offset."""

    def __init__(
        self,
        pose_age_limit_s: float = POSE_AGE_LIMIT_S,
        frame_age_limit_s: float = FRAME_AGE_LIMIT_S,
        offset_jump_limit_s: float = OFFSET_JUMP_LIMIT_S,
        offset_settle_s: float = OFFSET_SETTLE_S,
        timesync_silence_limit_s: float = TIMESYNC_SILENCE_LIMIT_S,
    ) -> None:
        self._pose_age_limit_s = float(pose_age_limit_s)
        self._frame_age_limit_s = float(frame_age_limit_s)
        self._offset_jump_limit_s = float(offset_jump_limit_s)
        self._offset_settle_s = float(offset_settle_s)
        self._timesync_silence_limit_s = float(timesync_silence_limit_s)
        self._pose_stamp_s: Optional[float] = None
        self._pose_stamps_ns: deque = deque(maxlen=_RECENT)
        self._offsets_ns: deque = deque(maxlen=_RECENT)
        self._synced_pose_stamp_s: Optional[float] = None
        self._frame_stamp_s: Optional[float] = None
        self._timesync_time_s: Optional[float] = None
        self._offset_s: Optional[float] = None
        self._last_step_s: Optional[float] = None
        self._last_jump_time_s: Optional[float] = None
        self._last_jump_s: Optional[float] = None
        self._round_trip_ms: Optional[float] = None

    def on_pose(self, stamp_ns: Any) -> None:
        """The header stamp of a pose from MAVROS, in whole nanoseconds."""
        if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int):
            return
        self._pose_stamp_s = stamp_ns * 1.0e-9
        self._pose_stamps_ns.append(stamp_ns)
        self._note_synced_poses()

    def _note_synced_poses(self) -> None:
        # A pose and the offset it was stamped with arrive on two topics, in
        # either order, so each new one is matched against the recent others.
        for stamp_ns in self._pose_stamps_ns:
            if is_synced(stamp_ns, self._offsets_ns):
                stamp_s = stamp_ns * 1.0e-9
                if self._synced_pose_stamp_s is None or stamp_s > self._synced_pose_stamp_s:
                    self._synced_pose_stamp_s = stamp_s

    def on_frame(self, stamp_s: Any) -> None:
        """The stamp of a camera frame, or of a detection made from one."""
        stamp = _number(stamp_s)
        if stamp is not None:
            self._frame_stamp_s = stamp

    def on_timesync(self, now_s: Any, estimated_offset_ns: Any, round_trip_ms: Any) -> None:
        """One /mavros/timesync_status message, with the clock when it arrived."""
        now, offset_ns = _number(now_s), _number(estimated_offset_ns)
        if now is None or offset_ns is None:
            return
        self._offsets_ns.append(int(offset_ns))
        self._note_synced_poses()
        offset = offset_ns * 1.0e-9
        if self._offset_s is not None:
            self._last_step_s = offset - self._offset_s
            if abs(self._last_step_s) > self._offset_jump_limit_s:
                self._last_jump_time_s = now
                self._last_jump_s = self._last_step_s
        self._offset_s = offset
        self._timesync_time_s = now
        self._round_trip_ms = _number(round_trip_ms)

    def report(self, now_s: Any) -> TimeHealthReport:
        now = _number(now_s)
        if now is None:
            return TimeHealthReport(False, ("the clock is not a number",), None, None, None, None, None, None)
        reasons = []

        def age(stamp: Optional[float], limit_s: float, what: str) -> Optional[float]:
            if stamp is None:
                reasons.append(f"no {what} yet")
                return None
            value = now - stamp
            if value > limit_s:
                reasons.append(f"{what} is {value * 1000.0:.0f} ms old (limit {limit_s * 1000.0:.0f} ms)")
            elif value < -limit_s:
                reasons.append(f"{what} is stamped {-value * 1000.0:.0f} ms ahead of the clock")
            return value

        pose_age = age(self._pose_stamp_s, self._pose_age_limit_s, "pose")
        # Two-sided, like the ages: after the clock steps back, a proof from before the step is no proof.
        if pose_age is not None and (self._synced_pose_stamp_s is None or abs(now - self._synced_pose_stamp_s) > SYNC_PROOF_LIMIT_S):
            reasons.append("pose stamps are arrival times, not flight-controller time (MAVROS has not applied its time sync)")
        frame_age = age(self._frame_stamp_s, self._frame_age_limit_s, "frame")
        timesync_age = None
        if self._timesync_time_s is None:
            reasons.append("no time sync status yet")
        else:
            timesync_age = now - self._timesync_time_s
            if timesync_age > self._timesync_silence_limit_s:
                reasons.append(f"time sync has been silent for {timesync_age:.1f} s")
            if self._last_jump_time_s is not None and now - self._last_jump_time_s < self._offset_settle_s:
                reasons.append(
                    f"time sync offset jumped by {self._last_jump_s * 1000.0:+.1f} ms "
                    f"(limit {self._offset_jump_limit_s * 1000.0:.0f} ms) {now - self._last_jump_time_s:.1f} s ago"
                )
        return TimeHealthReport(
            healthy=not reasons,
            reasons=tuple(reasons),
            pose_age_s=pose_age,
            frame_age_s=frame_age,
            timesync_age_s=timesync_age,
            offset_ms=None if self._offset_s is None else self._offset_s * 1000.0,
            last_offset_step_ms=None if self._last_step_s is None else self._last_step_s * 1000.0,
            round_trip_ms=self._round_trip_ms,
        )
