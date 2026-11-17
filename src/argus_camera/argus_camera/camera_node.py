"""argus_camera node (PLT-03): publishes one sensor mode's raw frames and its camera_info. Thin wrapper.

  ros2 launch argus_camera argus_camera.launch.py mode:=imx519_720p60

Topics: /argus/camera/<camera>/image_raw and .../camera_info, and (unless
legacy_topics is false) the 2026 names /camera/image and /camera/camera_info,
so target_cv and the 2026 missions run on this driver unchanged. It is not
part of the 2026 start-up: the 2026 grabbers are untouched.

A csi or v4l2 mode is read through the library's FrameSource in a thread. A
ros_topic mode (the sim) takes frames from `input_topic`, where ros_gz_bridge
publishes the Gazebo camera. Either way a frame goes out as it came in, with
the mode's own K and D, stamped with its capture time.
"""

from __future__ import annotations

import array
import threading
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from argus_camera.calibration import camera_info_fields, for_mode
from argus_camera.frame_source import open_source
from argus_camera.modes import get_mode, load_modes


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("argus_camera")
        self.declare_parameter("mode", "imx519_720p60")
        self.declare_parameter("config_dir", "")
        self.declare_parameter("synthetic", False)  # the test pattern in the mode's size, for a bench with no camera
        self.declare_parameter("legacy_topics", True)
        self.declare_parameter("input_topic", "/sim/camera/image")  # ros_topic modes only
        config_dir = self.get_parameter("config_dir").value or str(Path(get_package_share_directory("argus_camera")) / "config")
        self._mode = get_mode(load_modes(Path(config_dir) / "modes.yaml"), self.get_parameter("mode").value)
        if self._mode.in_process_only:
            raise RuntimeError(f"mode {self._mode.name} is not published as topics: the process that needs it embeds FrameSource")
        calibration = for_mode(self._mode, config_dir)
        if not calibration.measured:
            self.get_logger().warn(f"Mode {self._mode.name}: calibration is not measured ({calibration.basis})")
        self._info = CameraInfo(**camera_info_fields(calibration))
        self._info.header.frame_id = self._mode.frame_id

        base = f"/argus/camera/{self._mode.camera}"
        self._image_pubs = [self.create_publisher(Image, f"{base}/image_raw", qos_profile_sensor_data)]
        self._info_pubs = [self.create_publisher(CameraInfo, f"{base}/camera_info", qos_profile_sensor_data)]
        if self.get_parameter("legacy_topics").value:
            self._image_pubs.append(self.create_publisher(Image, "/camera/image", qos_profile_sensor_data))
            self._info_pubs.append(self.create_publisher(CameraInfo, "/camera/camera_info", qos_profile_sensor_data))

        self._source = None
        self._stop = threading.Event()
        self._thread = None
        self._published = 0
        self.create_timer(5.0, self._log_rate)
        if self._mode.driver == "ros_topic" and not self.get_parameter("synthetic").value:
            topic = self.get_parameter("input_topic").value
            self.create_subscription(Image, topic, self._on_input, qos_profile_sensor_data)
            self.get_logger().info(f"argus_camera: mode {self._mode.name} from {topic}")
        else:
            self._source = open_source(self._mode, synthetic=bool(self.get_parameter("synthetic").value), clock=self._now_ns)
            self._source.open()
            self._thread = threading.Thread(target=self._capture_loop, name="argus_camera_capture", daemon=True)
            self._thread.start()
            self.get_logger().info(f"argus_camera: mode {self._mode.name}, {self._mode.width} x {self._mode.height} at {self._mode.fps:g} fps")

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _log_rate(self) -> None:
        count, self._published = self._published, 0
        self.get_logger().info(f"argus_camera: {count / 5.0:.1f} frames a second published (mode {self._mode.fps:g})")

    def _capture_loop(self) -> None:
        period_s = 1.0 / self._mode.fps
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
            self._publish(message, Time(sec=frame.capture_ns // 1_000_000_000, nanosec=frame.capture_ns % 1_000_000_000))
            if self.get_parameter("synthetic").value:
                self._stop.wait(period_s)  # a real camera paces itself

    def _on_input(self, message: Image) -> None:
        if (message.width, message.height) != (self._mode.width, self._mode.height):
            self.get_logger().error(
                f"{message.width} x {message.height} frame on the input but mode {self._mode.name} is "
                f"{self._mode.width} x {self._mode.height}; dropped",
                throttle_duration_sec=5.0,
            )
            return
        self._publish(message, message.header.stamp)  # the sim camera's own stamp is the capture time

    def _publish(self, image: Image, stamp: Time) -> None:
        image.header.stamp = stamp
        image.header.frame_id = self._mode.frame_id
        self._info.header.stamp = stamp
        try:
            for publisher in self._image_pubs:
                publisher.publish(image)
            for publisher in self._info_pubs:
                publisher.publish(self._info)
        except Exception:  # noqa: BLE001  (publishing while the node shuts down)
            if rclpy.ok():
                raise
            return
        self._published += 1

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
