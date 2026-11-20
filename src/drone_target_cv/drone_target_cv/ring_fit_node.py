"""The near-field bullseye centre finder as a node (CEN-13): a thin wrapper around ring_fit.py.

Behind the node flag `features.ring_fit`, which defaults to false. With the
flag off the node makes no subscription and no publisher and does nothing.

With it on, below `max_height_m` (4 m) it fits the bullseye's rings in each
frame and publishes the centre on the detection contract (FND-02) as a
`bullseye`, on /perception/ring_fit/detections, stamped with the image's
own stamp and frame. The estimator takes it beside the 2026 detector's when
the mission flag `use_ring_fit` is on.

  - Images are taken with a queue of one: the node always works on the
    newest frame and never on one that waited behind another.
  - The camera is named by the node's own parameters `camera_mode` and
    `airframe`, read from argus_camera's calibration and robot.yaml by the
    same loader the estimator uses, with the same refusals (a mode that is
    not calibrated, an airframe that is not measured, a sim camera on a real
    airframe). A frame that is not the calibrated mode's size is not used:
    its pixels would not be the contract's. They must name the camera the
    sequence's missions name: nothing compares the two, and the estimator
    projects this node's pixels with the missions' calibration.
  - Each frame is levelled with the vehicle's attitude at the frame's time
    (MAVROS's pose, kept for a few seconds), the mode's camera matrix and
    the camera's mount, so that a tilted vehicle's view of the rings is
    fitted as circles.
  - A frame with no pose within 0.1 s of its stamp is not looked at: the
    node would know neither the height nor the tilt for it. Nor is a frame
    taken above `max_height_m`. The height is the pose's, above home.
  - Every ten seconds of frames it says in the log how many it looked at
    and how many gave rings.
"""

from __future__ import annotations

from collections import deque
from typing import Any, Deque, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

POSE_TOPIC = "/mavros/local_position/pose"
POSE_MAX_APART_S = 0.1  # a pose further than this from a frame's stamp is not that frame's pose
REPORT_EVERY_S = 10.0
POSE_HISTORY = 150  # five seconds at 30 Hz


def image_to_bgr(message: Any) -> np.ndarray:
    """A sensor_msgs/Image as a BGR array. Raises ValueError for an encoding the fit cannot use."""
    data = np.frombuffer(message.data, dtype=np.uint8)
    if message.encoding in ("bgr8", "rgb8"):
        image = data.reshape(message.height, message.width, 3)
        return image if message.encoding == "bgr8" else image[:, :, ::-1].copy()
    if message.encoding in ("bgra8", "rgba8"):
        image = data.reshape(message.height, message.width, 4)[:, :, :3]
        return image.copy() if message.encoding == "bgra8" else image[:, :, ::-1].copy()
    raise ValueError(f"unsupported image encoding: {message.encoding}")


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class RingFitNode(Node):
    def __init__(self) -> None:
        super().__init__("ring_fit")
        self.declare_parameter("features.ring_fit", False)
        self.declare_parameter("image_topic", "camera/image")
        self.declare_parameter("max_height_m", 4.0)
        self.declare_parameter("sim_hsv", True)
        self.declare_parameter("camera_mode", "")
        self.declare_parameter("airframe", "")
        self._enabled = bool(self.get_parameter("features.ring_fit").get_parameter_value().bool_value)
        self._publisher = None
        self._poses: Deque[Tuple[int, Tuple[float, float, float, float], float]] = deque(maxlen=POSE_HISTORY)
        self.counts = {"frames": 0, "looked at": 0, "rings found": 0}
        self._last_report_ns: Optional[int] = None
        if not self._enabled:
            self.get_logger().info("Ring fit off (features.ring_fit): nothing subscribed, nothing published")
            return
        try:
            self._start()
        except Exception as error:  # noqa: BLE001  (a detector that cannot start must say so and stay out of the way)
            self._enabled = False
            self._publisher = None
            self.get_logger().error(f"features.ring_fit is on but the ring fit could not start ({error}): nothing is published")

    def _start(self) -> None:
        import vision_msgs.msg as vision_messages
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import Image

        from drone_mission_core import detection_contract as contract
        from drone_mission_core.frame_projection import load_camera
        from drone_mission_core.projection_v2 import rotation_from_quaternion
        from drone_target_cv import ring_fit

        self._contract, self._vision_messages, self._ring_fit = contract, vision_messages, ring_fit
        self._rotation_from_quaternion = rotation_from_quaternion
        self._max_height_m = float(self.get_parameter("max_height_m").get_parameter_value().double_value)
        sim = bool(self.get_parameter("sim_hsv").get_parameter_value().bool_value)
        self._config = ring_fit.RingFitConfig(hsv_v_min=50 if sim else 45)  # as the 2026 detector's node sets its own
        mode = str(self.get_parameter("camera_mode").get_parameter_value().string_value).strip()
        airframe = str(self.get_parameter("airframe").get_parameter_value().string_value).strip()
        if not mode or not airframe:
            raise ValueError("camera_mode and airframe must both be set: the sensor mode the images come from, and the airframe in argus_camera's robot.yaml")
        camera, mount = load_camera(mode, airframe)  # refuses what projection v2 refuses
        self._size = (int(camera.width), int(camera.height))
        self._camera_matrix = np.array([[camera.fx, 0.0, camera.cx], [0.0, camera.fy, camera.cy], [0.0, 0.0, 1.0]])
        self._body_from_optical = np.asarray(mount.rotation_body_from_optical, dtype=np.float64)

        # The newest frame only: a frame that waited behind another is older than the one after it.
        newest_only = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        image_topic = str(self.get_parameter("image_topic").get_parameter_value().string_value)
        self._publisher = self.create_publisher(vision_messages.Detection2DArray, contract.topic_for(ring_fit.DETECTOR_NAME), 10)
        self.create_subscription(Image, image_topic, self._on_image, newest_only)
        self.create_subscription(PoseStamped, POSE_TOPIC, self._on_pose, qos_profile_sensor_data)
        self.get_logger().info(
            f"Ring fit on (features.ring_fit): {image_topic} below {self._max_height_m:g} m, queue of one, "
            f"camera {mode} on {airframe} ({self._size[0]} x {self._size[1]}), publishing on {contract.topic_for(ring_fit.DETECTOR_NAME)}"
        )

    def _on_pose(self, message: Any) -> None:
        orientation, position = message.pose.orientation, message.pose.position
        self._poses.append((_stamp_ns(message.header.stamp), (orientation.x, orientation.y, orientation.z, orientation.w), float(position.z)))

    def _pose_for(self, stamp_ns: int) -> Optional[Tuple[int, Tuple[float, float, float, float], float]]:
        """The pose of a frame's moment: the nearest to its stamp, if within POSE_MAX_APART_S. None otherwise."""
        if not self._poses:
            return None
        pose = min(self._poses, key=lambda held: abs(held[0] - stamp_ns))
        return pose if abs(pose[0] - stamp_ns) <= int(POSE_MAX_APART_S * 1.0e9) else None

    def _skip(self, why: str) -> None:
        self.counts[why] = self.counts.get(why, 0) + 1

    def _report(self, stamp_ns: int) -> None:
        """Every REPORT_EVERY_S of frames: how many were looked at, how many gave rings, and why the rest were not."""
        if self._last_report_ns is None:
            self._last_report_ns = stamp_ns
            return
        if abs(stamp_ns - self._last_report_ns) < int(REPORT_EVERY_S * 1.0e9):
            return
        self._last_report_ns = stamp_ns
        skipped = ", ".join(f"{why} {count}" for why, count in self.counts.items() if why not in ("frames", "looked at", "rings found"))
        self.get_logger().info(
            f"Ring fit: {self.counts['frames']} frames so far, looked at {self.counts['looked at']}, rings found in {self.counts['rings found']}"
            + (f"; not looked at: {skipped}" if skipped else "")
        )

    def _on_image(self, message: Any) -> None:
        if not self._enabled:
            return
        try:
            frame = image_to_bgr(message)
        except ValueError as error:
            self.get_logger().error(f"Ring fit: {error}", throttle_duration_sec=5.0)
            return
        self._process_frame(frame, message.header)

    def _process_frame(self, frame: np.ndarray, header: Any) -> None:
        """Fit one frame and publish what it gave. Nothing is published for a frame the node did not look at."""
        stamp = header.stamp
        if not (stamp.sec or stamp.nanosec) or not header.frame_id:
            self.get_logger().error("Ring fit: image without a stamp or a frame id: not used", throttle_duration_sec=5.0)
            return
        stamp_ns = _stamp_ns(stamp)
        self.counts["frames"] += 1
        self._report(stamp_ns)
        if (frame.shape[1], frame.shape[0]) != self._size:
            self._skip("not the calibrated size")
            self.get_logger().error(
                f"Ring fit: the image is {frame.shape[1]} x {frame.shape[0]} but the camera mode is calibrated at {self._size[0]} x {self._size[1]}: not used",
                throttle_duration_sec=5.0,
            )
            return
        pose = self._pose_for(stamp_ns)
        if pose is None:
            self._skip("no pose of the frame's moment")  # neither its height nor its tilt is known
            return
        if pose[2] > self._max_height_m:
            self._skip("above the height limit")  # too high for the rings to be the better centre
            return
        try:
            world_from_optical = self._rotation_from_quaternion(pose[1]) @ self._body_from_optical
            level = self._ring_fit.nadir_homography(self._camera_matrix, world_from_optical)
        except ValueError:
            self._skip("camera not looking down")
            return
        try:
            self.counts["looked at"] += 1
            detection = self._ring_fit.to_detection(self._ring_fit.fit_rings(frame, self._config, level))
            if detection is not None:
                self.counts["rings found"] += 1
            found = self._contract.DetectionSet(stamp_ns=stamp_ns, frame_id=str(header.frame_id), detections=() if detection is None else (detection,))
            self._publisher.publish(self._contract.encode(found, self._vision_messages))
        except Exception as error:  # noqa: BLE001  (one bad frame must not stop the node)
            self.get_logger().error(f"Ring fit failed on a frame: {error}", throttle_duration_sec=5.0)


def main(args=None) -> None:
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = RingFitNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # the launch is stopping
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
