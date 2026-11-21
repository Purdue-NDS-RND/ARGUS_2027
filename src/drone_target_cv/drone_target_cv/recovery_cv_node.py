"""The green X and bucket detector as a node (REC-1): a thin wrapper around green_x.py.

Behind the node flag `features.recovery_cv`, which defaults to false. With the
flag off the node makes no subscription and no publisher and does nothing.

With it on, each frame of the downward camera is looked at for the recovery
mat's green X and the bucket on it, and what is found goes out on the
detection contract (FND-02), on /perception/recovery_cv/detections, stamped
with the image's own stamp and frame: a `green_x` whose box is centred where
the arms cross, and a `bucket` whose box is its fitted ellipse's. The
bucket's handle goes out for every frame looked at on
/perception/recovery_cv/handle (a `geometry_msgs/PolygonStamped` in the
same pixels and with the same stamp): its two ends when it shows, and no
points when it does not, so a reader never holds a handle from an
earlier frame for want of a newer message. Each class is put on
the ground by whoever reads it at that class's own plane height
(target_estimator.PLANE_HEIGHT_M: the X at 0, the bucket's rim at 0.2 m).

  - Images are taken with a queue of one: the node always works on the
    newest frame and never on one that waited behind another.
  - The camera is named by the node's own parameters `camera_mode` and
    `airframe`, read by the same loader the estimator uses, with the same
    refusals. A frame that is not the calibrated mode's size is not used:
    its pixels would not be the contract's.
  - A frame with nothing in it still gives a message, with no detections:
    a reader can tell "looked and saw nothing" from "did not look".
  - Every ten seconds of frames it says in the log how many it looked at
    and how many had the X and the bucket.
"""

from __future__ import annotations

import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node

from drone_target_cv.ring_fit_node import _stamp_ns, image_to_bgr

DETECTOR_NAME = "recovery_cv"
HANDLE_TOPIC = "/perception/recovery_cv/handle"
REPORT_EVERY_S = 10.0


class RecoveryCvNode(Node):
    def __init__(self) -> None:
        super().__init__(DETECTOR_NAME)
        self.declare_parameter("features.recovery_cv", False)
        self.declare_parameter("image_topic", "camera/image")
        self.declare_parameter("camera_mode", "")
        self.declare_parameter("airframe", "")
        self._enabled = bool(self.get_parameter("features.recovery_cv").get_parameter_value().bool_value)
        self._publisher = None
        self._handle_publisher = None
        self.counts = {"frames": 0, "looked at": 0, "x found": 0, "bucket found": 0}
        self.slowest_ms = 0.0  # the longest the detector has taken over one frame
        self._last_report_ns: Optional[int] = None
        if not self._enabled:
            self.get_logger().info("Recovery detector off (features.recovery_cv): nothing subscribed, nothing published")
            return
        try:
            self._start()
        except Exception as error:  # noqa: BLE001  (a detector that cannot start must say so and stay out of the way)
            self._enabled = False
            self._publisher = self._handle_publisher = None
            self.get_logger().error(f"features.recovery_cv is on but the recovery detector could not start ({error}): nothing is published")

    def _start(self) -> None:
        import vision_msgs.msg as vision_messages
        from geometry_msgs.msg import Point32, PolygonStamped
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image

        from drone_mission_core import detection_contract as contract
        from drone_mission_core.frame_projection import load_camera
        from drone_target_cv import green_x

        self._contract, self._vision_messages, self._green_x = contract, vision_messages, green_x
        self._PolygonStamped, self._Point32 = PolygonStamped, Point32
        mode = str(self.get_parameter("camera_mode").get_parameter_value().string_value).strip()
        airframe = str(self.get_parameter("airframe").get_parameter_value().string_value).strip()
        if not mode or not airframe:
            raise ValueError("camera_mode and airframe must both be set: the sensor mode the images come from, and the airframe in argus_camera's robot.yaml")
        camera, _mount = load_camera(mode, airframe)  # refuses what projection v2 refuses
        self._size = (int(camera.width), int(camera.height))
        # The newest frame only: a frame that waited behind another is older than the one after it.
        newest_only = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        image_topic = str(self.get_parameter("image_topic").get_parameter_value().string_value)
        self._publisher = self.create_publisher(vision_messages.Detection2DArray, contract.topic_for(DETECTOR_NAME), 10)
        self._handle_publisher = self.create_publisher(PolygonStamped, HANDLE_TOPIC, 10)
        self.create_subscription(Image, image_topic, self._on_image, newest_only)
        self.get_logger().info(
            f"Recovery detector on (features.recovery_cv): {image_topic}, queue of one, camera {mode} on {airframe} "
            f"({self._size[0]} x {self._size[1]}), publishing on {contract.topic_for(DETECTOR_NAME)}"
        )

    def _report(self, stamp_ns: int) -> None:
        if self._last_report_ns is None:
            self._last_report_ns = stamp_ns
            return
        if abs(stamp_ns - self._last_report_ns) < int(REPORT_EVERY_S * 1.0e9):
            return
        self._last_report_ns = stamp_ns
        skipped = ", ".join(f"{why} {count}" for why, count in self.counts.items() if why not in ("frames", "looked at", "x found", "bucket found"))
        self.get_logger().info(
            f"Recovery detector: {self.counts['frames']} frames so far, looked at {self.counts['looked at']}, "
            f"the X in {self.counts['x found']}, the bucket in {self.counts['bucket found']}; the slowest frame took {self.slowest_ms:.0f} ms"
            + (f"; not looked at: {skipped}" if skipped else "")
        )

    def _on_image(self, message: Any) -> None:
        if not self._enabled:
            return
        try:
            frame = image_to_bgr(message)
        except ValueError as error:
            self.get_logger().error(f"Recovery detector: {error}", throttle_duration_sec=5.0)
            return
        self._process_frame(frame, message.header)

    def _process_frame(self, frame: Any, header: Any) -> None:
        """Look at one frame and publish what it gave. Nothing is published for a frame the node did not look at."""
        stamp = header.stamp
        if not (stamp.sec or stamp.nanosec) or not header.frame_id:
            self.get_logger().error("Recovery detector: image without a stamp or a frame id: not used", throttle_duration_sec=5.0)
            return
        stamp_ns = _stamp_ns(stamp)
        self.counts["frames"] += 1
        self._report(stamp_ns)
        if (frame.shape[1], frame.shape[0]) != self._size:
            self.counts["not the calibrated size"] = self.counts.get("not the calibrated size", 0) + 1
            self.get_logger().error(
                f"Recovery detector: the image is {frame.shape[1]} x {frame.shape[0]} but the camera mode is calibrated at {self._size[0]} x {self._size[1]}: not used",
                throttle_duration_sec=5.0,
            )
            return
        try:
            self.counts["looked at"] += 1
            started = time.perf_counter()
            result = self._green_x.detect(frame)
            self.slowest_ms = max(self.slowest_ms, (time.perf_counter() - started) * 1000.0)
            detections = self._green_x.to_detections(result)
            if result.x is not None and not self.counts["x found"]:
                self.get_logger().info(f"Recovery detector: the X first found, at ({result.x.cx:.0f}, {result.x.cy:.0f}) px with confidence {result.x.score:.2f}")
            if result.bucket is not None and not self.counts["bucket found"]:
                self.get_logger().info(f"Recovery detector: the bucket first found, at ({result.bucket.cx:.0f}, {result.bucket.cy:.0f}) px")
            self.counts["x found"] += result.x is not None
            self.counts["bucket found"] += result.bucket is not None
            found = self._contract.DetectionSet(stamp_ns=stamp_ns, frame_id=str(header.frame_id), detections=detections)
            self._publisher.publish(self._contract.encode(found, self._vision_messages))
            handle = self._PolygonStamped()  # for every frame looked at: two ends, or no points for "no handle in this frame"
            handle.header.stamp, handle.header.frame_id = header.stamp, header.frame_id
            for x, y in (result.bucket.handle if result.bucket is not None and result.bucket.handle is not None else ()):
                point = self._Point32()
                point.x, point.y, point.z = float(x), float(y), 0.0
                handle.polygon.points.append(point)
            self._handle_publisher.publish(handle)
        except Exception as error:  # noqa: BLE001  (one bad frame must not stop the node)
            self.get_logger().error(f"Recovery detector failed on a frame: {error}", throttle_duration_sec=5.0)


def main(args=None) -> None:
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = RecoveryCvNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # the launch is stopping
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
