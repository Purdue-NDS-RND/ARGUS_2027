"""What the camera node publishes where, and the small decisions around it. No ROS imports.

The node's own topics are /argus/camera/<camera>/image_raw and camera_info.
With `legacy_topics` on it also serves the three names the 2026 grabbers
publish, so the 2026 detector and session logger can run on this driver:
/camera/image, /camera/image/compressed (JPEG) and /camera/camera_info, at
the 2026 publishing rate.
"""

from __future__ import annotations

import time
from typing import Callable, Dict, Iterable, List, Optional

import cv2
import numpy as np

from .modes import SensorMode

LEGACY_IMAGE = "/camera/image"
LEGACY_COMPRESSED = "/camera/image/compressed"
LEGACY_INFO = "/camera/camera_info"
LEGACY_RATE_HZ = 30.0  # image_publishing_rate in the 2026 params
LEGACY_JPEG_QUALITY = 20  # compressed_quality in the 2026 live params
STARTUP_GUARD_S = 10.0  # a second publisher on a 2026 name this soon after the node starts is a set-up mistake: the node stops


class TopicError(ValueError):
    """Topics that cannot be used together. The message says why."""


def topic_names(mode: SensorMode, legacy: bool) -> Dict[str, List[str]]:
    """The topics the node publishes for a mode: its own, and the 2026 names when `legacy` is on."""
    base = f"/argus/camera/{mode.camera}"
    names = {"image": [f"{base}/image_raw"], "info": [f"{base}/camera_info"], "legacy_image": [], "legacy_compressed": [], "legacy_info": []}
    if legacy:
        names["legacy_image"], names["legacy_compressed"], names["legacy_info"] = [LEGACY_IMAGE], [LEGACY_COMPRESSED], [LEGACY_INFO]
    return names


def _absolute(topic: str) -> str:
    return "/" + topic.strip().strip("/")


def check_input_topic(input_topic: str, names: Dict[str, List[str]]) -> str:
    """The input topic of a relay mode, refused if the node also publishes on it (it would read its own output)."""
    topic = _absolute(input_topic)
    published = {_absolute(name) for group in names.values() for name in group}
    if topic in published:
        raise TopicError(
            f"input_topic {topic} is also a topic this node publishes: it would relay its own output for ever. "
            "Turn legacy_topics off or take the input from another topic"
        )
    return topic


def other_publishers(counts: Dict[str, int], names: Dict[str, List[str]]) -> List[str]:
    """Legacy topics that someone else is publishing on too, given the publisher count of each topic (ours included)."""
    legacy = names["legacy_image"] + names["legacy_compressed"] + names["legacy_info"]
    return sorted(topic for topic in legacy if counts.get(topic, 0) > 1)


class RateLimiter:
    """Lets frames through at no more than a rate, by their capture time."""

    def __init__(self, rate_hz: float) -> None:
        # A frame that arrives a little early for its slot still counts (60 fps thinned to 30 is every second
        # frame), but never more than a tenth early: whatever the source's rate, no more than 1.11 times the limit passes.
        self._interval_ns = int(0.9 * 1.0e9 / rate_hz)
        self._last_ns: Optional[int] = None

    def allow(self, capture_ns: int) -> bool:
        if self._last_ns is not None and 0 <= capture_ns - self._last_ns < self._interval_ns:
            return False
        self._last_ns = capture_ns
        return True


def encode_jpeg(image: np.ndarray, quality: int) -> Optional[bytes]:
    """The image as JPEG bytes, or None if it could not be encoded."""
    ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    return encoded.tobytes() if ok else None


def image_from_message(data, height: int, width: int, encoding: str) -> Optional[np.ndarray]:
    """A BGR image from the bytes of an image message, or None for an encoding this does not read."""
    channels = {"bgr8": 3, "rgb8": 3, "mono8": 1}.get(encoding)
    if channels is None or len(data) != height * width * channels:
        return None
    image = np.frombuffer(data, dtype=np.uint8).reshape((height, width, channels) if channels == 3 else (height, width))
    return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if encoding == "rgb8" else image


class Pacer:
    """Waits until the next frame is due, so a source that does not pace itself runs at the mode's rate."""

    def __init__(self, rate_hz: float, clock: Callable[[], float] = time.monotonic, sleep: Callable[[float], None] = time.sleep) -> None:
        self._period = 1.0 / rate_hz
        self._clock, self._sleep = clock, sleep
        self._due: Optional[float] = None

    def wait(self) -> None:
        now = self._clock()
        if self._due is None or now - self._due > 5.0 * self._period:
            self._due = now  # first frame, or so far behind that catching up would be a burst
        self._due += self._period
        if self._due > now:
            self._sleep(self._due - now)
