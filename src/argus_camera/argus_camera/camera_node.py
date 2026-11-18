"""argus_camera node (PLT-03): publishes one sensor mode's raw frames and its camera_info. Thin wrapper.

  ros2 launch argus_camera argus_camera.launch.py mode:=imx519_720p60

Topics: /argus/camera/<camera>/image_raw and .../camera_info. With
legacy_topics:=true it also serves the names the 2026 grabbers publish
(/camera/image, /camera/image/compressed, /camera/camera_info, at the 2026
rate), so the 2026 detector and session logger can run on this driver. That
is off by default and the node stops if anything else publishes on those
names: two cameras on one topic would feed the detector interleaved frames.
It is not part of the 2026 start-up: the 2026 grabbers are untouched.

A csi or v4l2 mode is read through the library's FrameSource in a thread. A
ros_topic mode (the sim) takes frames from `input_topic`, where ros_gz_bridge
publishes the Gazebo camera. Either way a frame goes out as it came in, with
the mode's own K and D, stamped with its capture time.
"""

from __future__ import annotations

import array
import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from argus_camera import publishing
from argus_camera.calibration import camera_info_fields, for_mode
from argus_camera.frame_source import open_source
from argus_camera.modes import get_mode, load_modes


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("argus_camera")
        self.declare_parameter("mode", "imx519_720p60")
        self.declare_parameter("config_dir", "")
        self.declare_parameter("synthetic", False)  # the test pattern in the mode's size, for a bench with no camera
        self.declare_parameter("legacy_topics", False)
        self.declare_parameter("legacy_rate_hz", publishing.LEGACY_RATE_HZ)
        self.declare_parameter("legacy_jpeg_quality", publishing.LEGACY_JPEG_QUALITY)
        self.declare_parameter("input_topic", "/camera/image")  # ros_topic modes only: where the sim's bridge publishes
        self.declare_parameter("csi_backend", "opencv")  # "gstreamer": frames stamped from their PTS (needs python3-gi)
        config_dir = self.get_parameter("config_dir").value or str(Path(get_package_share_directory("argus_camera")) / "config")
        self._mode = get_mode(load_modes(Path(config_dir) / "modes.yaml"), self.get_parameter("mode").value)
        if self._mode.in_process_only:
            raise RuntimeError(f"mode {self._mode.name} is not published as topics: the process that needs it embeds FrameSource")
        calibration = for_mode(self._mode, config_dir)
        if not calibration.measured:
            self.get_logger().warn(f"Mode {self._mode.name}: calibration is not measured ({calibration.basis})")
        self._info = CameraInfo(**camera_info_fields(calibration))
        self._info.header.frame_id = self._mode.frame_id

        self._names = publishing.topic_names(self._mode, bool(self.get_parameter("legacy_topics").value))
        make = lambda kind, key: [self.create_publisher(kind, name, qos_profile_sensor_data) for name in self._names[key]]  # noqa: E731
        self._image_pubs, self._info_pubs = make(Image, "image"), make(CameraInfo, "info")
        self._legacy_image_pubs, self._legacy_info_pubs = make(Image, "legacy_image"), make(CameraInfo, "legacy_info")
        self._legacy_compressed_pubs = make(CompressedImage, "legacy_compressed")
        self._legacy_limit = publishing.RateLimiter(float(self.get_parameter("legacy_rate_hz").value))
        self._jpeg_quality = int(self.get_parameter("legacy_jpeg_quality").value)

        self._source = None
        self._stop = threading.Event()
        self._thread = None
        self._failure = None
        self._published = 0
        self._stamp_source = "the input's own stamp"
        self._synthetic = bool(self.get_parameter("synthetic").value)
        self._started = time.monotonic()
        self.create_timer(5.0, self._log_rate)
        self.create_timer(1.0, self._watch)
        if self._mode.driver == "ros_topic" and not self._synthetic:
            topic = publishing.check_input_topic(self.get_parameter("input_topic").value, self._names)
            self.create_subscription(Image, topic, self._on_input, qos_profile_sensor_data)
            self.get_logger().info(f"argus_camera: mode {self._mode.name} from {topic}")
        else:
            self._source = open_source(self._mode, clock=self._now_ns, synthetic=self._synthetic, csi_backend=str(self.get_parameter("csi_backend").value))
            self._source.open()
            self._thread = threading.Thread(target=self._capture_loop, name="argus_camera_capture", daemon=True)
            self._thread.start()
            self.get_logger().info(f"argus_camera: mode {self._mode.name}, {self._mode.width} x {self._mode.height} at {self._mode.fps:g} fps")
            if self._synthetic:
                self.get_logger().warn("argus_camera: publishing a TEST PATTERN, not a camera (synthetic:=true)")

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _log_rate(self) -> None:
        count, self._published = self._published, 0
        if count == 0:
            self.get_logger().warn("argus_camera: no frame from the camera in the last 5 s")
        else:
            fallbacks = 0 if self._source is None else self._source.capture_clock.fallbacks
            stale = 0 if self._source is None else self._source.capture_clock.stale
            note = f", {fallbacks} frames stamped at the read because their own time was unusable" if fallbacks else ""
            note += f", {stale} frames dropped as over a second old" if stale else ""
            self.get_logger().info(f"argus_camera: {count / 5.0:.1f} frames a second published (mode {self._mode.fps:g}), capture time from {self._stamp_source}{note}")

    def _watch(self) -> None:
        """Stop the node, loudly, if the capture thread failed, or if at start-up someone else publishes on the 2026 names.

        A second publisher that turns up later, in flight, is reported but does
        not stop the node: losing the only camera is worse than a stray frame.
        """
        others = publishing.other_publishers({name: self.count_publishers(name) for group in self._names.values() for name in group}, self._names)
        if others and self._failure is None:
            problem = f"another node also publishes on {', '.join(others)}; two cameras on one topic interleave their frames"
            if time.monotonic() - self._started < publishing.STARTUP_GUARD_S:
                self._failure = problem
            else:
                self.get_logger().error(f"argus_camera: {problem}", throttle_duration_sec=5.0)
        if self._failure is not None:
            self.get_logger().fatal(f"argus_camera stopping: {self._failure}")
            raise SystemExit(1)

    def _capture_loop(self) -> None:
        pacer = publishing.Pacer(self._mode.fps) if self._synthetic else None  # a real camera paces itself
        try:
            while not self._stop.is_set() and rclpy.ok():
                frame = self._source.read()
                if frame is None:
                    self._stop.wait(0.005)
                    continue
                message = Image(height=frame.image.shape[0], width=frame.image.shape[1], encoding="bgr8", is_bigendian=0)
                message.step = frame.image.strides[0]
                # As an array: given plain bytes, rclpy checks every byte in Python,
                # which takes a third of a second for one 720p frame.
                message.data = array.array("B", frame.image.tobytes())
                stamp = Time(sec=frame.capture_ns // 1_000_000_000, nanosec=frame.capture_ns % 1_000_000_000)
                self._stamp_source = {"pts": "the buffer's PTS", "kernel": "the kernel's buffer time", "read": "the read (no better time)"}[frame.stamp_source]
                self._publish(message, stamp, frame.capture_ns, frame.image)
                if pacer is not None:
                    pacer.wait()
        except Exception as error:  # noqa: BLE001  (a dead capture thread must stop the node, not leave it looking alive)
            if rclpy.ok():
                self._failure = f"the capture thread failed: {error}"

    def _on_input(self, message: Image) -> None:
        if (message.width, message.height) != (self._mode.width, self._mode.height):
            self.get_logger().error(
                f"{message.width} x {message.height} frame on the input but mode {self._mode.name} is "
                f"{self._mode.width} x {self._mode.height}; dropped",
                throttle_duration_sec=5.0,
            )
            return
        stamp = message.header.stamp  # the sim camera's own stamp is the capture time
        self._publish(message, stamp, stamp.sec * 1_000_000_000 + stamp.nanosec, None)

    def _publish(self, image: Image, stamp: Time, capture_ns: int, pixels) -> None:
        image.header.stamp = stamp
        image.header.frame_id = self._mode.frame_id
        self._info.header.stamp = stamp
        try:
            for publisher in self._image_pubs:
                publisher.publish(image)
            for publisher in self._info_pubs:
                publisher.publish(self._info)
            if self._legacy_image_pubs and self._legacy_limit.allow(capture_ns):
                self._publish_legacy(image, pixels)
        except Exception:  # noqa: BLE001  (publishing while the node shuts down)
            if rclpy.ok():
                raise
            return
        self._published += 1

    def _publish_legacy(self, image: Image, pixels) -> None:
        for publisher in self._legacy_image_pubs:
            publisher.publish(image)
        for publisher in self._legacy_info_pubs:
            publisher.publish(self._info)
        if pixels is None:
            pixels = publishing.image_from_message(image.data, image.height, image.width, image.encoding)
        jpeg = None if pixels is None else publishing.encode_jpeg(pixels, self._jpeg_quality)
        if jpeg is not None:
            compressed = CompressedImage(header=image.header, format="jpeg")
            compressed.data = array.array("B", jpeg)
            for publisher in self._legacy_compressed_pubs:
                publisher.publish(compressed)

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._source is not None:
            self._source.close()


def main() -> None:
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
