"""Capture time: when a frame was taken, on the caller's clock (FND-01). No ROS imports.

A frame is useful to steering only with the moment it was taken. Two
drivers know that moment in two different ways:

  - GStreamer (the IMX519 on CSI): each buffer has a presentation timestamp,
    the pipeline's running time when the frame entered it. Added to the
    pipeline's base time it is a time on the pipeline's clock.
    `nvarguscamerasrc` does not pass the sensor's own timestamp through, so
    a fixed per-camera offset, measured with the LED probe (PLT-05) and
    stored with the sensor mode, is subtracted as well.
  - V4L2 (the OV9782 on USB): the kernel stamps each buffer on the
    monotonic clock.

Either way the frame's age on that clock, taken off the caller's clock now,
is its capture time on the caller's clock. A timestamp that is missing or
from the future cannot be right and is not used: the frame is stamped when
it was read, the source of the stamp says so, and the fallback is counted.
A timestamp that says the frame is over a second old may well be right (the
pipeline stalled), so that frame is not given a fresh stamp: it is dropped,
and counted.
"""

from __future__ import annotations

from typing import Callable, Optional, Tuple

FUTURE_TOLERANCE_NS = 2_000_000  # a stamp this far ahead of "now" is clock granularity, not a fault
_NO_PTS_FROM = 1 << 62  # GStreamer's CLOCK_TIME_NONE is 2**64 - 1


def stamp_from_pts(pts_ns: int, base_time_ns: int, pipeline_now_ns: int, clock_now_ns: int, offset_ns: int = 0) -> int:
    """Capture time on the caller's clock from a GStreamer buffer's PTS.

    `base_time_ns` and `pipeline_now_ns` are on the pipeline's clock;
    `clock_now_ns` is the caller's clock at the same moment.
    """
    age_ns = pipeline_now_ns - (base_time_ns + pts_ns)
    return clock_now_ns - age_ns - offset_ns


def stamp_from_kernel(buffer_ns: int, monotonic_now_ns: int, clock_now_ns: int, offset_ns: int = 0) -> int:
    """Capture time on the caller's clock from a V4L2 buffer's kernel timestamp (monotonic clock)."""
    return clock_now_ns - (monotonic_now_ns - buffer_ns) - offset_ns


class CaptureClock:
    """Stamps frames for one camera. `clock` gives nanoseconds on the caller's clock; times here are nanoseconds."""

    def __init__(self, clock: Callable[[], int], offset_s: float = 0.0, max_age_s: float = 1.0) -> None:
        self._clock = clock
        self._offset_ns = int(round(offset_s * 1.0e9))
        self._max_age_ns = int(max_age_s * 1.0e9)
        self.fallbacks = 0  # frames whose own timestamp could not be used
        self.stale = 0  # frames dropped because their own timestamp was over max_age_s old
        self.last_problem: Optional[str] = None

    def _fallback(self, problem: str) -> Tuple[int, str]:
        self.fallbacks += 1
        self.last_problem = problem
        return int(self._clock()), "read"

    def _checked(self, age_ns: int, source: str) -> Optional[Tuple[int, str]]:
        if age_ns < -FUTURE_TOLERANCE_NS:
            return self._fallback(f"the buffer's timestamp is {-age_ns / 1.0e6:.0f} ms from the future")
        if age_ns > self._max_age_ns:
            self.stale += 1
            self.last_problem = f"a frame {age_ns / 1.0e9:.1f} s old by its own timestamp was dropped"
            return None  # old, and it must not go out looking new
        return int(self._clock()) - age_ns - self._offset_ns, source

    def from_pts(self, pts_ns: Optional[int], base_time_ns: int, pipeline_now_ns: int) -> Optional[Tuple[int, str]]:
        """(capture time, "pts"); (read time, "read") if the PTS cannot be used; None for a frame too old to pass on."""
        if pts_ns is None or pts_ns < 0 or pts_ns >= _NO_PTS_FROM:
            return self._fallback("no PTS on the buffer")
        return self._checked(pipeline_now_ns - (base_time_ns + pts_ns), "pts")

    def from_kernel(self, buffer_ns: Optional[int], monotonic_now_ns: int) -> Optional[Tuple[int, str]]:
        """(capture time, "kernel"); (read time, "read") if the kernel's timestamp cannot be used; None for a frame too old to pass on."""
        if buffer_ns is None or buffer_ns <= 0:
            return self._fallback("no kernel timestamp on the buffer")
        return self._checked(monotonic_now_ns - buffer_ns, "kernel")

    def on_read(self) -> Tuple[int, str]:
        """For a source with no timestamp of its own: the clock now."""
        return int(self._clock()), "read"
