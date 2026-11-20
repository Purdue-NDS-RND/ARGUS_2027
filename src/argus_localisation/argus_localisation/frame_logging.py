"""The survey's frame logging policy (LOC-11). No ROS.

Behind the node flag `features.survey_frame_log`. The writer is
argus_camera's FrameLog (one folder a log, JPEG files and an index, encoding
on worker threads); this is what localisation asks of it:

  - every frame the camera gives while the survey's pipeline is on, at JPEG
    quality 90, into a new folder for each survey under `frame_log_dir`
    (about 42 MB a second from the 4K mode, to the NVMe);
  - nothing before the pipeline is started or after it is stopped;
  - the encoding threads run at a lower priority than everything else on
    the machine (the Orin Nano has no hardware JPEG encoder, and the same
    cores fly the aircraft);
  - the logger gives way to the detector. Once a second the node hands the
    pipeline's status to `review`: if the detector is under nine tenths of
    the rate it should have (its target, or the camera's rate when that is
    lower), or under its minimum rate, or the log's own worker queue
    overflowed, the logger keeps half as many frames (every 2nd, 4th, ...
    down to one in MAX_KEEP_EVERY). After RECOVER_S with none of those it
    takes one step back up. Frames the detector drops because the camera is
    faster than it are not a reason: that is how the pipeline is built.
    The first STARTUP_GRACE_S of a survey are not judged.

`frame_log_dir` must exist (a disk that is not mounted is not written
beside). The logging stops, and says so in the status, when the disk has
under `min_free_bytes` left, when one survey's log reaches `max_bytes`, or
when a write fails; the pipeline is not touched by any of these. Stopping
does not wait for the frames still queued: they are finished behind.
"""

from __future__ import annotations

import shutil
import threading
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from argus_camera.frame_log import FrameLog, FrameLogError

FLAG = "survey_frame_log"
JPEG_QUALITY = 90
DEFAULT_WORKERS = 3
MAX_KEEP_EVERY = 16  # the slowest the logger goes: one frame in sixteen (about one a second at 15 Hz)
RECOVER_S = 10.0  # this long with nothing wrong before the logger takes a step back up
RATE_SHARE = 0.9  # the detector under this share of the rate it should have is short of CPU
STARTUP_GRACE_S = 3.0  # the first seconds of a survey, when the detector's rate is over too few frames to judge
WORKER_NICENESS = 10  # the encoding threads' priority: below everything that flies the aircraft
MIN_FREE_BYTES = 20.0e9  # the logging stops with this much of the disk left, for whatever else writes there
QUEUE_FRAMES_PER_WORKER = 2
S = 1_000_000_000


def _disk_free(path: Path) -> float:
    return float(shutil.disk_usage(path).free)


class SurveyFrameLog:
    def __init__(
        self,
        root: Any,
        jpeg_quality: int = JPEG_QUALITY,
        workers: int = DEFAULT_WORKERS,
        max_bytes: Optional[int] = None,
        expected_rate_hz: float = 15.0,
        min_free_bytes: float = MIN_FREE_BYTES,
        make_log: Callable[..., Any] = FrameLog,
        disk_free: Callable[[Path], float] = _disk_free,
    ) -> None:
        if not str(root).strip():
            raise ValueError("frame_log_dir must be set: the folder the survey's frames are logged under")
        if isinstance(jpeg_quality, bool) or not isinstance(jpeg_quality, int) or not 1 <= jpeg_quality <= 100:
            raise ValueError(f"the frame log's JPEG quality is a whole number from 1 to 100, got {jpeg_quality!r}")
        if isinstance(workers, bool) or not isinstance(workers, int) or workers < 0:
            raise ValueError(f"the frame log's workers is a whole number, 0 or more, got {workers!r}")
        if max_bytes is not None and not max_bytes > 0:
            raise ValueError(f"the frame log's size limit must be above 0, got {max_bytes!r}")
        if not expected_rate_hz > 0.0 or min_free_bytes < 0.0:
            raise ValueError("the frame log needs the detector's rate above 0 and a free-space floor of 0 or more")
        self.root = Path(str(root)).expanduser()
        self._settings = {
            "image_format": "jpg",
            "jpeg_quality": jpeg_quality,
            "max_bytes": max_bytes,
            "workers": workers,
            "queue_frames": QUEUE_FRAMES_PER_WORKER * max(workers, 1),
            "worker_niceness": WORKER_NICENESS,
        }
        self._expected_rate_hz, self._min_free_bytes = float(expected_rate_hz), float(min_free_bytes)
        self._make_log, self._disk_free = make_log, disk_free
        self._log: Any = None
        self._closing: List[Any] = []  # logs whose queued frames are still being finished behind
        self._closers: List[threading.Thread] = []
        self._lock = threading.Lock()
        self._folder: Optional[Path] = None
        self._error = ""
        self.keep_every = 1
        self._reason = ""
        self._offered = 0
        self._thinned = 0
        self._totals = {"written": 0, "dropped": 0}
        self._seen_log_dropped = 0
        self._started_ns: Optional[int] = None
        self._well_since_ns: Optional[int] = None

    # --- the survey's start and end -----------------------------------------------------

    def _room(self) -> Optional[str]:
        """Why the disk cannot be logged to, or None."""
        if not self.root.is_dir():
            return f"frame_log_dir '{self.root}' is not a folder that exists (is the disk mounted?)"
        try:
            free = self._disk_free(self.root)
        except OSError as error:
            return f"the free space of '{self.root}' cannot be read ({error})"
        if free < self._min_free_bytes:
            return f"the disk under '{self.root}' has {free / 1.0e9:.1f} GB free, under the {self._min_free_bytes / 1.0e9:g} GB the logging leaves"
        return None

    def start(self, stamp_ns: int) -> Optional[Path]:
        """The survey has started: a new folder, named by the start's time in whole seconds. None if it cannot be made."""
        self.stop()
        problem = self._room()
        if problem is not None:
            self._error = problem
            return None
        name, number = f"survey_{int(stamp_ns) // S:09d}", 1
        folder = self.root / name
        while folder.exists():  # a second start in the same second: a folder of its own all the same
            number += 1
            folder = self.root / f"{name}_{number}"
        try:
            self._log = self._make_log(folder, **self._settings)
        except (FrameLogError, OSError) as error:
            self._log, self._error = None, f"{type(error).__name__}: {error}"
            return None
        self._folder, self._error = folder, ""
        self.keep_every, self._reason, self._offered = 1, "", 0
        self._seen_log_dropped, self._started_ns, self._well_since_ns = 0, int(stamp_ns), None
        return folder

    def _finish(self, log: Any) -> None:
        try:
            log.close()
        except Exception as error:  # noqa: BLE001  (a log that will not close is said, and nothing waits on it)
            self._error = f"{type(error).__name__}: {error}"
        with self._lock:
            self._totals["written"] += int(log.written)
            self._totals["dropped"] += int(log.dropped)
            if log in self._closing:
                self._closing.remove(log)

    def stop(self, wait_s: Optional[float] = None) -> None:
        """The survey is over. The frames still queued are finished on a thread of their own: whoever stops the
        survey does not wait on the disk. `wait_s` waits that long for them all the same (shutdown, and tests)."""
        with self._lock:  # (a failed write on the camera's thread and the survey's stop can both get here)
            log, self._log = self._log, None
            if log is not None:
                self._closing.append(log)
        if log is not None:
            closer = threading.Thread(target=self._finish, args=(log,), name="frame_log_close", daemon=True)
            closer.start()
            self._closers.append(closer)
        if wait_s is not None:
            for closer in self._closers:
                closer.join(timeout=wait_s)
        self._closers = [closer for closer in self._closers if closer.is_alive()]

    def _fail(self, why: str) -> None:
        """Stop logging for a reason that is the disk's or the writer's: said in the status until the next start."""
        self.stop()
        self._error = why

    # --- frames and the rate --------------------------------------------------------------

    def offer(self, frame: Any) -> bool:
        """One frame from the camera. True if it went to the log. Never raises: a write that fails stops the logging."""
        log = self._log
        if log is None:
            return False
        self._offered += 1
        if (self._offered - 1) % self.keep_every:
            self._thinned += 1
            return False
        try:
            return bool(log.write(frame))
        except Exception as error:  # noqa: BLE001  (the camera's thread and the detector must not pay for the disk)
            self._fail(f"{type(error).__name__}: {error}")
            return False

    def review(self, pipeline_status: Dict[str, Any], now_ns: int) -> None:
        """Once a second, with the pipeline's status: slow down if the detector or the log is short of CPU, come back when neither is."""
        log = self._log
        if log is None:
            return
        problem = self._room()
        if problem is not None:
            self._fail(problem)
            return
        reasons = []
        rate = pipeline_status.get("rate_hz")
        judged = self._started_ns is not None and int(now_ns) - self._started_ns >= STARTUP_GRACE_S * S
        if judged and pipeline_status.get("below_minimum_rate"):
            reasons.append("the detector is under its minimum rate")
        elif judged and isinstance(rate, (int, float)) and rate < RATE_SHARE * self._expected_rate_hz:
            reasons.append(f"the detector is at {rate:.1f} frames a second, under {RATE_SHARE * self._expected_rate_hz:.1f}")
        if int(log.dropped) > self._seen_log_dropped:
            reasons.append(f"the log dropped {int(log.dropped) - self._seen_log_dropped} frame(s) at its own queue")
        self._seen_log_dropped = int(log.dropped)
        if reasons:
            self._well_since_ns = None
            self._reason = "; ".join(reasons)
            self.keep_every = min(MAX_KEEP_EVERY, self.keep_every * 2)
            return
        if self._well_since_ns is None:
            self._well_since_ns = int(now_ns)
        if self.keep_every > 1 and int(now_ns) - self._well_since_ns >= RECOVER_S * S:
            self.keep_every //= 2
            self._well_since_ns = int(now_ns)
            self._reason = "" if self.keep_every == 1 else self._reason

    def status(self) -> Dict[str, Any]:
        log = self._log
        with self._lock:
            behind = list(self._closing)
            written, dropped = self._totals["written"], self._totals["dropped"]
        for item in ([log] if log is not None else []) + behind:
            written, dropped = written + int(item.written), dropped + int(item.dropped)
        return {
            "logging": log is not None and not bool(getattr(log, "full", False)),
            "folder": None if self._folder is None else str(self._folder),
            "written": written,
            "dropped": dropped,
            "thinned": self._thinned,
            "keep_every": self.keep_every,
            "reason": self._reason,
            "full": bool(getattr(log, "full", False)) if log is not None else False,
            "low_priority": None if log is None else not bool(getattr(log, "niceness_refused", False)),  # False: the encoding threads could not be given their lower priority
            "error": self._error or (str(log.error) if log is not None and getattr(log, "error", None) else ""),
        }
