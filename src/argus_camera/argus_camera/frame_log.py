"""Frame log core: write frames to disk with an index. No ROS imports.

One folder per log: frame_NNNNNN.png (lossless, so a digit crop taken later
is the pixels the camera gave) and index.jsonl with one line per frame:
sequence number, capture time, sensor mode, file, size. It can thin by time
and it stops at a size limit instead of filling the disk. What to log and
when is the caller's policy (LOC-11); this is only the writer.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

import cv2

from .frame_source import Frame


class FrameLogError(RuntimeError):
    """A frame log that cannot be started. The message says why."""


class FrameLog:
    def __init__(self, folder, min_interval_s: float = 0.0, max_bytes: Optional[int] = None) -> None:
        self.folder = Path(folder)
        self._min_interval_ns = int(min_interval_s * 1.0e9)
        self._max_bytes = max_bytes
        self._last_ns: Optional[int] = None
        self._bytes = 0
        self.written = 0
        self.skipped = 0
        self.full = False
        index = self.folder / "index.jsonl"
        if index.exists():
            raise FrameLogError(f"{self.folder} already holds a frame log; each log gets a new folder")
        self.folder.mkdir(parents=True, exist_ok=True)
        self._index = open(index, "w", encoding="utf-8")

    def write(self, frame: Frame) -> bool:
        """Store one frame. False if it was skipped (too soon after the last, the log is full or closed)."""
        if self._index is None or self.full:
            self.skipped += 1
            return False
        if self._last_ns is not None and frame.capture_ns - self._last_ns < self._min_interval_ns:
            self.skipped += 1
            return False
        if self._max_bytes is not None and self._bytes >= self._max_bytes:
            self.full = True
            self.skipped += 1
            return False
        name = f"frame_{frame.seq:06d}.png"
        path = self.folder / name
        if not cv2.imwrite(str(path), frame.image):
            raise FrameLogError(f"could not write {path}")
        self._bytes += path.stat().st_size
        self._last_ns = frame.capture_ns
        height, width = frame.image.shape[:2]
        record = {"seq": frame.seq, "capture_ns": frame.capture_ns, "mode": frame.mode, "file": name, "width": width, "height": height}
        self._index.write(json.dumps(record) + "\n")
        self._index.flush()
        self.written += 1
        return True

    def close(self) -> None:
        if self._index is not None:
            self._index.close()
            self._index = None
