"""The localisation pipeline's runtime: on and off, the newest frame only, and the status (LOC-5). No ROS.

  - The pipeline is off until it is started (`set_enabled(True)`, the
    node's /loc/set_enabled service; the survey mission starts it at its
    first track and stops it at its end). While it is off, frames are not
    kept.
  - `offer` takes a frame; `step` processes the newest one offered. A frame
    still waiting when a newer one arrives is dropped and counted: if the
    pipeline falls behind it skips frames, it never queues them.
  - A frame whose capture time is newer than the newest pose waits, up to
    POSE_WAIT_S, for the pose that places its boxes (MAVROS's pose of a
    moment arrives after the frame of that moment can). After the wait it
    is processed anyway: its boxes go out, counted as not placed.
  - A fault in one frame (a detector that raises) is counted and said in
    the status; the next frame is processed.
  - `status` is what goes out on /loc/status: frames processed, the rate
    (frames finished in the last RATE_WINDOW_S over that time, so that a
    stall shows), frames dropped, detections, how many were
    put on the ground, the tracker's tracks and confirmed IDs, how old the
    last frame was when its turn came, and whether the rate is under the
    minimum (8 Hz; the target is 15).
"""

from __future__ import annotations

import threading
from collections import deque
from typing import Any, Callable, Deque, Dict, Optional

MIN_RATE_HZ = 8.0
TARGET_RATE_HZ = 15.0
RATE_WINDOW_S = 5.0
POSE_WAIT_S = 0.15  # how long a frame waits for the pose of its capture time before it is processed without one
MIN_FRAMES_FOR_RATE = 5  # fewer than this in the window and the rate is not yet called low
MIN_RATE_SPAN_S = 0.5  # nor is there a rate at all over less time than this since the start
S = 1_000_000_000


class LocRuntime:
    def __init__(self, pipeline: Any, clock_ns: Callable[[], int], start_enabled: bool = False) -> None:
        self.pipeline = pipeline
        self._clock_ns = clock_ns
        self.enabled = bool(start_enabled)
        self.digit_error = ""  # why the digit reader, asked for, is not running (the node sets it)
        self._waiting: Any = None
        self._lock = threading.Lock()  # frames may be offered from the camera's own thread
        self._done: Deque[int] = deque()  # when each frame of the last RATE_WINDOW_S was finished
        self._counts: Dict[str, int] = {"frames": 0, "dropped": 0, "detections": 0, "placed": 0, "refused": 0, "faults": 0}
        self._frames_this_run = 0
        self._enabled_since_ns: Optional[int] = self._clock_ns() if self.enabled else None
        self._last_fault = ""
        self._waiting_for_pose = False
        self._pose_wait_since_ns: Optional[int] = None
        self._last_refusal = ""
        self._frame_age_ms: Optional[float] = None
        self._last_stamp_ns: Optional[int] = None
        self._timings_ms: Dict[str, float] = {}

    def set_enabled(self, enabled: bool) -> str:
        """Start or stop the pipeline. The answer is the service's message."""
        if bool(enabled) == self.enabled:
            return f"localisation pipeline already {'started' if self.enabled else 'stopped'}"
        self.enabled = bool(enabled)
        if self.enabled:
            self._frames_this_run = 0
            self._enabled_since_ns = self._clock_ns()
            self._pose_wait_since_ns = None
            return "localisation pipeline started"
        self._waiting = None
        self._done.clear()
        return f"localisation pipeline stopped after {self._frames_this_run} frames"

    def note_fault(self, reason: str) -> None:
        """A fault outside a frame's processing (the camera's reader): counted and said in the status like any other."""
        self._counts["faults"] += 1
        self._last_fault = str(reason)

    def offer(self, frame: Any) -> None:
        if not self.enabled:
            return
        with self._lock:
            if self._waiting is not None:
                self._counts["dropped"] += 1
            self._waiting = frame

    def step(self) -> Any:
        """Process the newest frame offered, if the pipeline is on and there is one. The frame's result, or None."""
        with self._lock:
            if not self.enabled or self._waiting is None:
                return None
            frame = self._waiting
            now = self._clock_ns()
            if not self.pipeline.pose_ready(int(frame.capture_ns)):
                # The wait is counted from when the pipeline began to wait, not from the frame now waiting: frames
                # that keep arriving must not start it again, or a pose that has stopped would stop the pipeline.
                if self._pose_wait_since_ns is None:
                    self._pose_wait_since_ns = now
                if now - self._pose_wait_since_ns < POSE_WAIT_S * S and now - int(frame.capture_ns) < POSE_WAIT_S * S:
                    self._waiting_for_pose = True
                    return None
            # Its turn has come, with its pose or without: the next frame has a wait of its own.
            self._pose_wait_since_ns = None
            self._waiting_for_pose = False
            self._waiting = None
        self._frame_age_ms = (self._clock_ns() - int(frame.capture_ns)) / 1.0e6
        try:
            result = self.pipeline.process(frame)
        except Exception as error:  # noqa: BLE001  (one bad frame must not stop the survey's pipeline)
            self._counts["faults"] += 1
            self._last_fault = f"{type(error).__name__}: {error}"
            return None
        now = self._clock_ns()
        self._done.append(now)
        self._prune(now)
        self._counts["frames"] += 1
        self._frames_this_run += 1
        self._counts["detections"] += len(result.detections.detections)
        self._counts["placed"] += len(result.ground)
        self._counts["refused"] += result.refused
        if result.refusal:
            self._last_refusal = result.refusal
        if getattr(result, "fault", ""):
            self.note_fault(result.fault)
        self._last_stamp_ns = result.stamp_ns
        self._timings_ms = dict(result.timings_ms)
        return result

    def _prune(self, now: int) -> None:
        while self._done and now - self._done[0] > RATE_WINDOW_S * S:
            self._done.popleft()

    def rate_hz(self) -> float:
        """Frames finished in the last RATE_WINDOW_S, over that time (or over the time since the start, when that is shorter): a stall shows."""
        now = self._clock_ns()
        self._prune(now)
        if not self.enabled or self._enabled_since_ns is None or not self._done:
            return 0.0
        over_ns = min(RATE_WINDOW_S * S, now - self._enabled_since_ns)
        if over_ns < MIN_RATE_SPAN_S * S:
            return 0.0
        return len(self._done) * float(S) / over_ns

    def _below_minimum(self, rate: float) -> bool:
        """On, and under the minimum: over enough frames to say so, or with a whole window gone by since the start (few frames, or none)."""
        if not self.enabled or rate >= MIN_RATE_HZ:
            return False
        waited = self._enabled_since_ns is not None and self._clock_ns() - self._enabled_since_ns >= RATE_WINDOW_S * S
        return len(self._done) >= MIN_FRAMES_FOR_RATE or bool(waited)

    def status(self) -> Dict[str, Any]:
        rate = self.rate_hz()
        tracker = getattr(self.pipeline, "tracker", None)
        tracks = int(tracker.track_count()) if tracker is not None and hasattr(tracker, "track_count") else 0
        confirmed = [int(value) for value in tracker.confirmed_ids()] if tracker is not None and hasattr(tracker, "confirmed_ids") else []
        reader = getattr(self.pipeline, "digit_reader", None)
        digits = None  # None without a digit reader
        if self.digit_error:
            digits = {"error": self.digit_error}  # the reader was asked for and could not start: the boxes go on without it
        elif reader is not None and hasattr(reader, "read_count"):
            digits = {
                "read": int(reader.read_count),
                "abstained": int(reader.abstained_count),
                "skipped": int(getattr(reader, "skipped_count", 0)),  # crops not looked at: over a frame's limit of crops or of time
                "faults": int(getattr(reader, "faults", 0)),
                "last_ms": round(float(getattr(reader, "last_ms", 0.0)), 2),  # what the last frame's crops took
                "classifier": str(getattr(reader.classifier, "name", "")),
            }
        votes = getattr(self.pipeline, "digit_votes", None)
        if votes is not None and tracker is not None and hasattr(tracker, "tracks"):
            decided = {str(state.track_id): int(state.digit) for state in tracker.tracks() if getattr(state, "digit", -1) >= 0}
            vote = {
                "tracks": len(votes.tracks()),
                "decided": decided,
                "contested": votes.contested(),
                "unique_ids": bool(votes.unique_ids),
                "solves": int(getattr(votes, "solves", 0)),
                "last_solve_ms": round(float(getattr(votes, "last_solve_ms", 0.0)), 2),  # what giving the digits out last took
                "faults": int(getattr(tracker, "vote_faults", 0)),
                "fault": str(getattr(tracker, "vote_fault", "")),  # the last one: the tracks went out without digits
            }
            digits = {**(digits or {}), "vote": vote}  # which tracks have a digit, and which claims the assignment could not settle
        return {
            "enabled": self.enabled,
            "detector": str(getattr(self.pipeline.detector, "name", type(self.pipeline.detector).__name__)),
            **self._counts,
            "rate_hz": round(rate, 2),
            "below_minimum_rate": self._below_minimum(rate),
            "frame_age_ms": None if self._frame_age_ms is None else round(self._frame_age_ms, 1),
            "last_stamp_ns": self._last_stamp_ns,
            "timings_ms": {key: round(value, 2) for key, value in self._timings_ms.items()},
            "waiting_for_pose": self._waiting_for_pose,
            "digits": digits,
            "tracks": tracks,
            "track_positions": None if tracker is None else ("median" if getattr(tracker, "median_positions", False) else "filter"),
            "confirmed_ids": confirmed,
            "last_fault": self._last_fault,
            "last_refusal": self._last_refusal,
        }
