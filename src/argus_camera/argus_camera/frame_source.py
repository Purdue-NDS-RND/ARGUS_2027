"""FrameSource: one interface to every camera. No ROS imports.

A process that needs frames without a ROS hop (the localisation node at 4K,
the optical-flow code) embeds a FrameSource; camera_node.py publishes the
same frames as topics. A source gives frames exactly as the sensor mode
delivers them: a frame of another size is an error, never resized.

`capture_ns` is the best capture time the source has, on the clock it was
given. Here that is the clock read when the frame came out of the driver;
FND-01 replaces it with the sensor's own time.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Callable, Optional

import cv2
import numpy as np

from .modes import SensorMode


class FrameSourceError(RuntimeError):
    """A camera that will not open or gives the wrong frames. The message names the mode."""


@dataclass(frozen=True)
class Frame:
    image: np.ndarray  # as the sensor mode gives it: no resize, no undistortion
    seq: int
    capture_ns: int
    mode: str


class FrameSource(ABC):
    def __init__(self, mode: SensorMode, clock: Callable[[], int]) -> None:
        # `clock` gives nanoseconds on the clock every other stamp in the process uses: the ROS clock in a node
        # (sim time in the sim). There is no default, so that nobody gets wall time by leaving it out.
        self.mode = mode
        self._clock = clock
        self._seq = 0
        self._open = False

    def open(self) -> None:
        self._open = True

    def close(self) -> None:
        self._open = False

    def __enter__(self) -> "FrameSource":
        self.open()
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    def read(self) -> Optional[Frame]:
        """The next frame, or None if there is none (closed, or the camera gave nothing). Never an old frame again."""
        if not self._open:
            return None
        image = self._grab()
        if image is None:
            return None
        height, width = image.shape[:2]
        if (width, height) != (self.mode.width, self.mode.height):
            raise FrameSourceError(
                f"camera gave a {width} x {height} frame but mode {self.mode.name} is {self.mode.width} x {self.mode.height}; "
                "the driver does not resize frames"
            )
        frame = Frame(image=image, seq=self._seq, capture_ns=int(self._clock()), mode=self.mode.name)
        self._seq += 1
        return frame

    @abstractmethod
    def _grab(self) -> Optional[np.ndarray]:
        """One image from the device, or None."""


class SyntheticSource(FrameSource):
    """A moving test pattern in the mode's size: for tests and for a bench with no camera."""

    def _grab(self) -> Optional[np.ndarray]:
        image = np.zeros((self.mode.height, self.mode.width, 3), dtype=np.uint8)
        image[:, :, 1] = 40
        x = (self._seq * 16) % self.mode.width
        image[:, x : x + 8, :] = 255  # a white bar that moves 16 pixels per frame
        return image


def csi_pipeline(mode: SensorMode) -> str:
    """The GStreamer pipeline for a CSI sensor on the Jetson, at the mode's own size and rate."""
    fps = int(round(mode.fps))
    controls = ""
    if mode.exposure_time_s is not None:
        exposure_ns = int(round(mode.exposure_time_s * 1.0e9))
        controls += f" exposuretimerange='{exposure_ns} {exposure_ns}'"
    if mode.wb_mode is not None:
        controls += f" wbmode={mode.wb_mode}"
    return (
        f"nvarguscamerasrc sensor-id={mode.sensor_id}{controls} ! "
        f"video/x-raw(memory:NVMM), width={mode.width}, height={mode.height}, format=NV12, framerate={fps}/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! "
        "appsink drop=1 max-buffers=1"
    )


def _open_capture(mode: SensorMode) -> Any:
    if mode.driver == "csi":
        return cv2.VideoCapture(csi_pipeline(mode), cv2.CAP_GSTREAMER)
    capture = cv2.VideoCapture(mode.device, cv2.CAP_V4L2)
    if mode.pixel_format:
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*mode.pixel_format))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, mode.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, mode.height)
    capture.set(cv2.CAP_PROP_FPS, mode.fps)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


class CaptureSource(FrameSource):
    """A CSI or USB camera through OpenCV's capture. `opener` makes the capture object; tests pass their own."""

    def __init__(self, mode: SensorMode, clock: Callable[[], int], opener: Callable[[SensorMode], Any] = _open_capture) -> None:
        super().__init__(mode, clock)
        self._opener = opener
        self._capture: Any = None

    def open(self) -> None:
        self._capture = self._opener(self.mode)
        if self._capture is None or not self._capture.isOpened():
            where = f"sensor {self.mode.sensor_id}" if self.mode.driver == "csi" else self.mode.device
            raise FrameSourceError(f"camera for mode {self.mode.name} ({self.mode.driver}, {where}) did not open")
        super().open()

    def close(self) -> None:
        super().close()
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def _grab(self) -> Optional[np.ndarray]:
        ok, image = self._capture.read()
        return image if ok and image is not None else None


def open_source(mode: SensorMode, *, clock: Callable[[], int], synthetic: bool = False, opener: Optional[Callable[[SensorMode], Any]] = None) -> FrameSource:
    """The FrameSource for a mode (not opened yet). `clock` stamps the frames; `synthetic` gives the test pattern in that mode's size."""
    if synthetic:
        return SyntheticSource(mode, clock)
    if mode.driver == "ros_topic":
        raise FrameSourceError(f"mode {mode.name} reads a ROS topic: only the camera node can do that, not a FrameSource")
    return CaptureSource(mode, clock, opener or _open_capture)
