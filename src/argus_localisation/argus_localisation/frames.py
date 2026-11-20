"""A frame for the pipeline from a sensor_msgs/Image (the sim's camera comes over a topic). No ROS imports.

The aircraft's frames come from the camera's own FrameSource inside the
node and never cross a topic; the sim's Gazebo camera has no other way in.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class TopicFrame:
    image: np.ndarray  # BGR
    seq: int
    capture_ns: int  # the image's own stamp
    mode: str
    stamp_source: str = "header"
    frame_id: str = ""  # the image's own optical frame, when the message names one


def image_to_bgr(message: Any) -> np.ndarray:
    """The message's pixels as a BGR array. Raises ValueError for an encoding the pipeline cannot use."""
    data = np.frombuffer(message.data, dtype=np.uint8)
    channels = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4}.get(message.encoding)
    if channels is None:
        raise ValueError(f"unsupported image encoding: {message.encoding}")
    image = data.reshape(int(message.height), int(message.width), channels)[:, :, :3]
    return image if message.encoding.startswith("bgr") else image[:, :, ::-1]


def frame_from_message(message: Any, seq: int, mode: str) -> TopicFrame:
    stamp = message.header.stamp
    return TopicFrame(
        np.ascontiguousarray(image_to_bgr(message)),
        int(seq),
        int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),
        str(mode),
        frame_id=str(getattr(message.header, "frame_id", "") or ""),
    )
