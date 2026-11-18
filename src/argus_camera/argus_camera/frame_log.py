"""Frame log core: write frames to disk with an index. No ROS imports.

One folder per log: frame_NNNNNN.jpg or .png and index.jsonl with one line
per frame: sequence number, capture time, sensor mode, file, size. JPEG at
quality 90 is the default, which is what the survey logs (LOC-11); PNG keeps
every pixel for the cases that need it.

With `workers` above zero the encoding and the disk write happen on worker
threads and `write` only queues the frame. When the queue is full the frame
is dropped and counted: the logger gives way before whatever is feeding it.
It can also thin by time, and it stops at a size limit instead of filling
the disk. What to log and when is the caller's policy; this is the writer.
"""

from __future__ import annotations

import json
import queue
import threading
from pathlib import Path
from typing import List, Optional

import cv2

from .frame_source import Frame

FORMATS = {"jpg": ".jpg", "png": ".png"}


class FrameLogError(RuntimeError):
    """A frame log that cannot be started or written. The message says why."""


class FrameLog:
    def __init__(
        self,
        folder,
        image_format: str = "jpg",
        jpeg_quality: int = 90,
        min_interval_s: float = 0.0,
        max_bytes: Optional[int] = None,
        workers: int = 0,
        queue_frames: int = 4,
    ) -> None:
        if image_format not in FORMATS:
            raise FrameLogError(f"image_format must be one of {', '.join(FORMATS)}, got {image_format!r}")
        self.folder = Path(folder)
        self._extension = FORMATS[image_format]
        self._encode_args = [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)] if image_format == "jpg" else []
        self._min_interval_ns = int(min_interval_s * 1.0e9)
        self._max_bytes = max_bytes
        self._last_ns: Optional[int] = None
        self._bytes = 0
        self._lock = threading.Lock()
        self.written = 0
        self.skipped = 0  # thinned by time, or refused because the log was full or closed
        self.dropped = 0  # refused because the worker queue was full
        self.full = False
        self.error: Optional[str] = None
        index = self.folder / "index.jsonl"
        if index.exists():
            raise FrameLogError(f"{self.folder} already holds a frame log; each log gets a new folder")
        self.folder.mkdir(parents=True, exist_ok=True)
        self._index = open(index, "w", encoding="utf-8")
        self._queue: Optional[queue.Queue] = queue.Queue(maxsize=max(1, queue_frames)) if workers > 0 else None
        self._threads: List[threading.Thread] = []
        for number in range(max(0, workers)):
            thread = threading.Thread(target=self._work, name=f"frame_log_{number}", daemon=True)
            thread.start()
            self._threads.append(thread)

    def write(self, frame: Frame) -> bool:
        """Take one frame. False if it was not taken: too soon after the last, the log full or closed, or the queue full."""
        with self._lock:
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
            self._last_ns = frame.capture_ns
        if self._queue is None:
            self._store(frame)
            return True
        try:
            self._queue.put_nowait(frame)
        except queue.Full:
            with self._lock:
                self.dropped += 1
            return False
        return True

    def _store(self, frame: Frame) -> None:
        name = f"frame_{frame.seq:06d}{self._extension}"
        path = self.folder / name
        if not cv2.imwrite(str(path), frame.image, self._encode_args):
            raise FrameLogError(f"could not write {path}")
        size = path.stat().st_size
        height, width = frame.image.shape[:2]
        record = {"seq": frame.seq, "capture_ns": frame.capture_ns, "mode": frame.mode, "file": name, "width": width, "height": height}
        with self._lock:
            self._bytes += size
            self.written += 1
            if self._index is not None:
                self._index.write(json.dumps(record) + "\n")
                self._index.flush()

    def _work(self) -> None:
        while True:
            frame = self._queue.get()
            try:
                if frame is None:
                    return
                self._store(frame)
            except Exception as error:  # noqa: BLE001  (a worker must not die silently: the log stops and says why)
                with self._lock:
                    self.error = str(error)
                    self.full = True
            finally:
                self._queue.task_done()

    def close(self) -> None:
        """Finish what is queued, stop the workers and close the index."""
        if self._queue is not None:
            self._queue.join()
            for _thread in self._threads:
                self._queue.put(None)
            for thread in self._threads:
                thread.join(timeout=5.0)
            self._threads = []
        with self._lock:
            if self._index is not None:
                self._index.close()
                self._index = None
