"""FrameSource: one interface to every camera. No ROS imports.

A process that needs frames without a ROS hop (the localisation node at 4K,
the optical-flow code) embeds a FrameSource; camera_node.py publishes the
same frames as topics. A source gives frames exactly as the sensor mode
delivers them: a frame of another size is an error, never resized.

`capture_ns` is when the frame was taken, on the clock the source was given
(capture_clock.py): from the buffer's PTS for the CSI camera through
GStreamer, from the kernel's buffer time for the USB camera. `stamp_source`
says which, and reads "read" when the source could only note the time the
frame came out of the driver.
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Callable, Optional, Tuple

import cv2
import numpy as np

from .capture_clock import CaptureClock
from .modes import SensorMode


class FrameSourceError(RuntimeError):
    """A camera that will not open or gives the wrong frames. The message names the mode."""


@dataclass(frozen=True)
class Frame:
    image: np.ndarray  # as the sensor mode gives it: no resize, no undistortion
    seq: int
    capture_ns: int
    mode: str
    stamp_source: str = "read"  # "pts", "kernel", or "read" (the time the frame left the driver)


class FrameSource(ABC):
    def __init__(self, mode: SensorMode, clock: Callable[[], int]) -> None:
        # `clock` gives nanoseconds on the clock every other stamp in the process uses: the ROS clock in a node
        # (sim time in the sim). There is no default, so that nobody gets wall time by leaving it out.
        self.mode = mode
        self.capture_clock = CaptureClock(clock, mode.capture_offset_s)
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
        grabbed = self._grab()
        if grabbed is None:
            return None
        image, capture_ns, stamp_source = grabbed
        height, width = image.shape[:2]
        if (width, height) != (self.mode.width, self.mode.height):
            raise FrameSourceError(
                f"camera gave a {width} x {height} frame but mode {self.mode.name} is {self.mode.width} x {self.mode.height}; "
                "the driver does not resize frames"
            )
        frame = Frame(image=image, seq=self._seq, capture_ns=capture_ns, mode=self.mode.name, stamp_source=stamp_source)
        self._seq += 1
        return frame

    @abstractmethod
    def _grab(self) -> Optional[Tuple[np.ndarray, int, str]]:
        """One image from the device with its capture time and where that time came from, or None."""


class SyntheticSource(FrameSource):
    """A moving test pattern in the mode's size: for tests and for a bench with no camera."""

    def _grab(self) -> Optional[Tuple[np.ndarray, int, str]]:
        image = np.zeros((self.mode.height, self.mode.width, 3), dtype=np.uint8)
        image[:, :, 1] = 40
        x = (self._seq * 16) % self.mode.width
        image[:, x : x + 8, :] = 255  # a white bar that moves 16 pixels per frame
        return (image, *self.capture_clock.on_read())


def csi_pipeline(mode: SensorMode, sink: str = "appsink drop=1 max-buffers=1") -> str:
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
        f"{sink}"
    )


def load_gst() -> Any:
    """GStreamer's Python bindings, initialised, or None where they are not installed (python3-gi, gir1.2-gstreamer-1.0)."""
    try:
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst

        Gst.init(None)
        return Gst
    except Exception:  # noqa: BLE001  (not installed, or no usable GStreamer: the caller falls back)
        return None


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
    """A CSI or USB camera through OpenCV's capture. `opener` makes the capture object; tests pass their own.

    A USB (V4L2) frame is stamped from the kernel's buffer time. OpenCV's
    GStreamer capture gives no usable capture time, so a CSI frame read this
    way is stamped when it is read; GstCsiSource is the CSI path with real
    capture times.
    """

    def __init__(
        self,
        mode: SensorMode,
        clock: Callable[[], int],
        opener: Callable[[SensorMode], Any] = _open_capture,
        monotonic: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        super().__init__(mode, clock)
        self._opener = opener
        self._monotonic = monotonic
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

    def _grab(self) -> Optional[Tuple[np.ndarray, int, str]]:
        ok, image = self._capture.read()
        if not ok or image is None:
            return None
        if self.mode.driver == "v4l2":
            buffer_ns = int(round(float(self._capture.get(cv2.CAP_PROP_POS_MSEC)) * 1.0e6))  # the kernel's time for this buffer
            stamp = self.capture_clock.from_kernel(buffer_ns, self._monotonic())
            return None if stamp is None else (image, *stamp)  # a frame over a second old is not passed on
        return (image, *self.capture_clock.on_read())


class GstCsiSource(FrameSource):
    """The CSI camera through GStreamer's own bindings, which give each buffer's PTS and the pipeline's clock."""

    PULL_TIMEOUT_NS = 100_000_000
    START_TIMEOUT_NS = 5_000_000_000

    def __init__(self, mode: SensorMode, clock: Callable[[], int], gst: Any = None) -> None:
        super().__init__(mode, clock)
        self._gst = gst
        self._pipeline: Any = None
        self._sink: Any = None

    def open(self) -> None:
        gst = self._gst or load_gst()
        if gst is None:
            raise FrameSourceError(f"mode {self.mode.name}: GStreamer's Python bindings are not installed (python3-gi, gir1.2-gstreamer-1.0)")
        self._gst = gst
        self._pipeline = gst.parse_launch(csi_pipeline(self.mode, sink="appsink name=sink drop=true max-buffers=1 sync=false"))
        self._sink = self._pipeline.get_by_name("sink")
        failed = self._pipeline.set_state(gst.State.PLAYING) == gst.StateChangeReturn.FAILURE
        # The change to PLAYING may finish later: wait for it, so a camera that is not there fails here and loudly.
        failed = failed or self._pipeline.get_state(self.START_TIMEOUT_NS)[0] == gst.StateChangeReturn.FAILURE
        if failed:
            self._pipeline.set_state(gst.State.NULL)
            raise FrameSourceError(f"camera for mode {self.mode.name} (csi, sensor {self.mode.sensor_id}) did not start")
        super().open()

    def close(self) -> None:
        super().close()
        if self._pipeline is not None:
            self._pipeline.set_state(self._gst.State.NULL)
            self._pipeline = None

    def _grab(self) -> Optional[Tuple[np.ndarray, int, str]]:
        sample = self._sink.emit("try-pull-sample", self.PULL_TIMEOUT_NS)
        if sample is None:
            error = self._pipeline.get_bus().pop_filtered(self._gst.MessageType.ERROR)
            if error is not None:  # the pipeline has failed: say so, instead of giving no frames for ever
                raise FrameSourceError(f"camera pipeline for mode {self.mode.name} reported an error: {error.parse_error()[0]}")
            return None
        buffer = sample.get_buffer()
        ok, info = buffer.map(self._gst.MapFlags.READ)
        if not ok:
            return None
        try:
            image = np.frombuffer(info.data, dtype=np.uint8).reshape((self.mode.height, self.mode.width, 3)).copy()
        finally:
            buffer.unmap(info)
        pts = None if buffer.pts == self._gst.CLOCK_TIME_NONE else int(buffer.pts)
        stamp = self.capture_clock.from_pts(pts, self._pipeline.get_base_time(), self._pipeline.get_clock().get_time())
        return None if stamp is None else (image, *stamp)  # a frame over a second old is not passed on


CSI_BACKENDS = ("opencv", "gstreamer")


def open_source(
    mode: SensorMode,
    *,
    clock: Callable[[], int],
    synthetic: bool = False,
    opener: Optional[Callable[[SensorMode], Any]] = None,
    csi_backend: str = "opencv",
) -> FrameSource:
    """The FrameSource for a mode (not opened yet). `clock` stamps the frames; `synthetic` gives the test pattern in that mode's size.

    `csi_backend` chooses how a CSI camera is read: "opencv" (OpenCV's
    capture, frames stamped when read) or "gstreamer" (GStreamer's own
    bindings, frames stamped from their PTS). It is a choice, never a guess
    from what is installed: "gstreamer" without the bindings is an error.
    """
    if csi_backend not in CSI_BACKENDS:
        raise FrameSourceError(f"csi_backend must be one of {', '.join(CSI_BACKENDS)}, got {csi_backend!r}")
    if synthetic:
        return SyntheticSource(mode, clock)
    if mode.driver == "ros_topic":
        raise FrameSourceError(f"mode {mode.name} reads a ROS topic: only the camera node can do that, not a FrameSource")
    if mode.driver == "csi" and csi_backend == "gstreamer":
        gst = load_gst()
        if gst is None:
            raise FrameSourceError(f"mode {mode.name}: csi_backend is gstreamer but GStreamer's Python bindings are not installed (python3-gi, gir1.2-gstreamer-1.0)")
        return GstCsiSource(mode, clock, gst)
    return CaptureSource(mode, clock, opener or _open_capture)
