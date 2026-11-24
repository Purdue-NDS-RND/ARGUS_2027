"""Optical-flow odometry as a node (REC-5): a thin wrapper around flow_odometry.py.

Behind the node flag `features.flow_odometry`, which defaults to false. With
the flag off the node makes no subscription and no publisher and does
nothing.

With it on, each frame of the flow camera becomes one sample:

  - the attitude is the pose history's at the frame's own stamp (the pose
    buffer of FND-04, fed here from MAVROS's local pose). A frame reaches
    the node before the pose that covers its moment does, so a frame waits,
    up to FRAME_WAIT_S, for the history to reach it: no pose of that moment
    by then, no velocity;
  - the height is the height estimator's (CEN-03, the module the missions
    use): the rangefinder's, straightened by the attitude, while the vehicle
    is in the sensor's range and not tilted; above that, the vehicle's
    height in the local frame. The sample's status says which. No height,
    no velocity (the flow is scaled by it);
  - under `corners_below_m` above the ground the hook or the bucket is in
    the middle of the picture, and the eight patches round its border are
    followed instead of the whole of it. The node cannot see whether a load
    is in view: the height is the rule, and an airframe with a hook always
    in view sets it above any height it flies at.

What goes out, each stamped with the frame's stamp:

  /argus/flow/velocity        geometry_msgs/TwistStamped, frame `base_link`:
                              linear.x forward and linear.y left, m/s.
                              Published only for a sample with quality.
  /argus/flow/velocity_enu    the same velocity as east (x) and north (y).
  /argus/flow/status          std_msgs/String, JSON: quality, why not, the
                              shift, the height, the patches. Every sample.
  /mavros/optical_flow/raw/send   mavros_msgs/OpticalFlow (MAVLink's
                              OPTICAL_FLOW), every sample, quality 0
                              included, only when `send_optical_flow` is
                              true. It is false unless a params file says
                              otherwise: how MAVROS's plugin and the flight
                              controller read the message's pixel fields has
                              not been checked against either (REC-5's notes).

Images are taken with a queue of one. A frame that is not the calibrated
mode's shape is not used.
"""

from __future__ import annotations

import json
import math
from typing import Any, Optional, Tuple

import rclpy
from rclpy.node import Node

from drone_target_cv.ring_fit_node import _stamp_ns, image_to_bgr

NODE_NAME = "flow_odometry"
VELOCITY_TOPIC = "/argus/flow/velocity"
VELOCITY_ENU_TOPIC = "/argus/flow/velocity_enu"
STATUS_TOPIC = "/argus/flow/status"
OPTICAL_FLOW_TOPIC = "/mavros/optical_flow/raw/send"
FRAME_WAIT_S = 0.25  # how long a frame waits for the pose history to reach its moment
MAX_WAITING = 8  # frames held at once; an older one is handled as it is
REPORT_EVERY_S = 10.0
S = 1_000_000_000


def image_to_picture(message: Any):
    """A sensor_msgs/Image as an array the flow can use: the flow camera's own single channel, or a colour picture. Raises ValueError otherwise."""
    import numpy as np

    if message.encoding in ("mono8", "8UC1"):
        step = int(getattr(message, "step", 0) or message.width)
        data = np.frombuffer(message.data, dtype=np.uint8)
        if data.size != message.height * step or step < message.width:
            raise ValueError(f"a {message.width} x {message.height} {message.encoding} image with {data.size} bytes")
        return data.reshape(message.height, step)[:, : message.width]
    return image_to_bgr(message)


def attitude_from_quaternion(orientation: Tuple[float, float, float, float]):
    """MAVROS's orientation as the flow's roll, pitch and heading (the rule is flow_odometry.attitude_from_quaternion)."""
    from drone_target_cv import flow_odometry

    return flow_odometry.attitude_from_quaternion(orientation)


class FlowOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        self.declare_parameter("features.flow_odometry", False)
        self.declare_parameter("image_topic", "/argus/camera/flow/image_raw")
        self.declare_parameter("camera_mode", "")
        self.declare_parameter("airframe", "")
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("range_topic", "/mavros/rangefinder/rangefinder")
        self.declare_parameter("send_optical_flow", False)  # off until the message's units and axes are checked against MAVROS's plugin and the flight controller (REC-5's notes)
        self.declare_parameter("corners_below_m", 3.0)
        self.declare_parameter("frames_apart", 4)  # the spec's: up to this many frames between the two pictures; fewer when the picture moves too far
        self._enabled = bool(self.get_parameter("features.flow_odometry").get_parameter_value().bool_value)
        self.counts = {"frames": 0, "samples": 0, "with quality": 0}
        self.whys: dict = {}
        self._last_report_ns: Optional[int] = None
        self._waiting: list = []  # frames held for the pose history to reach them: (stamp, picture, header)
        self._outputs: dict = {}
        if not self._enabled:
            self.get_logger().info("Flow odometry off (features.flow_odometry): nothing subscribed, nothing published")
            return
        try:
            self._start()
        except Exception as error:  # noqa: BLE001  (a node that cannot start must say so and stay out of the way)
            self._enabled = False
            self._outputs = {}
            self.get_logger().error(f"features.flow_odometry is on but the flow odometry could not start ({error}): nothing is published")

    def _start(self) -> None:
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from mavros_msgs.msg import OpticalFlow
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import Image, Range
        from std_msgs.msg import String

        from drone_mission_core.frame_projection import load_camera
        from drone_mission_core.height_estimator import HeightEstimator
        from drone_mission_core.pose_buffer import PoseBuffer
        from drone_target_cv import flow_odometry

        self._flow, self._TwistStamped, self._OpticalFlow, self._String = flow_odometry, TwistStamped, OpticalFlow, String
        mode = str(self.get_parameter("camera_mode").get_parameter_value().string_value).strip()
        airframe = str(self.get_parameter("airframe").get_parameter_value().string_value).strip()
        if not mode or not airframe:
            raise ValueError("camera_mode and airframe must both be set: the flow camera's sensor mode, and the airframe in argus_camera's robot.yaml")
        camera, mount = load_camera(mode, airframe)
        self._size = (int(camera.width), int(camera.height))
        if abs(self._size[0] / self._size[1] - flow_odometry.WIDTH / flow_odometry.HEIGHT) > 0.02:
            raise ValueError(f"the mode {mode} is {self._size[0]} x {self._size[1]}: not the flow picture's shape ({flow_odometry.WIDTH} x {flow_odometry.HEIGHT})")
        self._focal_px = float(camera.fx) * flow_odometry.WIDTH / self._size[0]  # of the 320-wide picture the flow is measured on
        apart = int(self.get_parameter("frames_apart").value)
        if not 1 <= apart <= 16:
            raise ValueError(f"frames_apart must be 1 to 16, got {apart}")
        # The mount says how the picture sits on the vehicle; a camera that does not look straight down is refused there.
        self._odometry = flow_odometry.FlowOdometry(self._focal_px, frames_apart=apart, body_from_optical=mount.rotation_body_from_optical)
        self._poses = PoseBuffer()
        self._heights = HeightEstimator(self._poses.pose_at)
        self._send = bool(self.get_parameter("send_optical_flow").get_parameter_value().bool_value)
        self._corners_below_m = float(self.get_parameter("corners_below_m").get_parameter_value().double_value)
        newest_only = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        image_topic = str(self.get_parameter("image_topic").get_parameter_value().string_value)
        self._outputs = {VELOCITY_TOPIC: self.create_publisher(TwistStamped, VELOCITY_TOPIC, 10), VELOCITY_ENU_TOPIC: self.create_publisher(TwistStamped, VELOCITY_ENU_TOPIC, 10),
                            STATUS_TOPIC: self.create_publisher(String, STATUS_TOPIC, 10)}  # fmt: skip
        if self._send:
            self._outputs[OPTICAL_FLOW_TOPIC] = self.create_publisher(OpticalFlow, OPTICAL_FLOW_TOPIC, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("pose_topic").get_parameter_value().string_value), self._on_pose, qos_profile_sensor_data)
        self.create_subscription(Range, str(self.get_parameter("range_topic").get_parameter_value().string_value), self._on_range, qos_profile_sensor_data)
        self.create_subscription(Image, image_topic, self._on_image, newest_only)
        self.get_logger().info(
            f"Flow odometry on (features.flow_odometry): {image_topic}, camera {mode} on {airframe} ({self._size[0]} x {self._size[1]}), "
            f"focal length {self._focal_px:.1f} px at {flow_odometry.WIDTH} wide, frames {apart} apart"
            + (f", sent to the flight controller on {OPTICAL_FLOW_TOPIC}" if self._send else ", not sent to the flight controller")
        )

    def _on_pose(self, message: Any) -> None:
        position, orientation = message.pose.position, message.pose.orientation
        self._poses.add_pose(_stamp_ns(message.header.stamp), (position.x, position.y, position.z), (orientation.x, orientation.y, orientation.z, orientation.w))
        self._handle_waiting()

    def _on_range(self, message: Any) -> None:
        self._heights.add_range(_stamp_ns(message.header.stamp), float(message.range), float(message.min_range), float(message.max_range))

    def height_at(self, stamp_ns: int, pose: Any) -> Tuple[Optional[float], str]:
        """(the height above the ground for a frame, where it is from): the height estimator's, or (None, "") with no pose of that moment."""
        if pose is None:
            return None, ""
        height = self._heights.height(pose.position[2], int(stamp_ns), orientation=pose.orientation)
        return (None, "") if height is None else (float(height.value_m), str(height.source))

    def _on_image(self, message: Any) -> None:
        if not self._enabled:
            return
        try:
            frame = image_to_picture(message)
        except ValueError as error:
            self.get_logger().error(f"Flow odometry: {error}", throttle_duration_sec=5.0)
            return
        header = message.header
        stamp = header.stamp
        if not (stamp.sec or stamp.nanosec):
            self.get_logger().error("Flow odometry: image without a stamp: not used", throttle_duration_sec=5.0)
            return
        self.counts["frames"] += 1
        if (frame.shape[1], frame.shape[0]) != self._size:
            self.whys["not the calibrated size"] = self.whys.get("not the calibrated size", 0) + 1
            self.get_logger().error(f"Flow odometry: the image is {frame.shape[1]} x {frame.shape[0]} but the mode is {self._size[0]} x {self._size[1]}: not used", throttle_duration_sec=5.0)
            return
        # Kept as the small flow picture, in the order the frames came: the message's own buffer is not held on to.
        self._waiting.append((_stamp_ns(stamp), self._flow.prepare(frame), header))
        self._handle_waiting()

    def _handle_waiting(self) -> None:
        """Handle, oldest first, every held frame the pose history has reached, and any that has waited its time out."""
        newest_pose = self._poses.newest_ns()
        newest_frame = self._waiting[-1][0] if self._waiting else 0
        while self._waiting:
            stamp_ns, picture, header = self._waiting[0]
            reached = newest_pose is not None and newest_pose >= stamp_ns
            waited_out = newest_frame - stamp_ns > int(FRAME_WAIT_S * S) or len(self._waiting) > MAX_WAITING
            if not reached and not waited_out:
                return
            self._waiting.pop(0)
            self._process_frame(stamp_ns, picture, header)

    def _process_frame(self, stamp_ns: int, picture: Any, header: Any) -> None:
        # One bad frame must not end the node: what fails is said, and the next frame is taken.
        try:
            pose = self._poses.pose_at(stamp_ns)
            attitude = None if pose is None else attitude_from_quaternion(pose.orientation)
            height, source = self.height_at(stamp_ns, pose)
            sample = self._odometry.add(picture, stamp_ns, attitude, height, corners_only=height is not None and height < self._corners_below_m)
            if sample is not None:
                self._publish(self._flow.with_height_source(sample, source), header, source)
        except Exception as error:  # noqa: BLE001
            self.whys["a frame could not be handled"] = self.whys.get("a frame could not be handled", 0) + 1
            self._odometry.reset()
            self.get_logger().error(f"Flow odometry: a frame could not be handled and was dropped ({error})", throttle_duration_sec=5.0)
        self._report(stamp_ns)

    def _publish(self, sample: Any, header: Any, height_source: str = "") -> None:
        self.counts["samples"] += 1
        status = self._String()
        status.data = json.dumps({"stamp_ns": sample.stamp_ns, "quality": sample.quality, "why": sample.why, "shift_px": [round(value, 3) for value in sample.shift_px],
                                  "height_m": round(sample.height_m, 3), "height_source": height_source, "dt_s": round(sample.dt_s, 4), "patches": sample.patches,
                                  "forward_mps": round(sample.forward_mps, 4), "right_mps": round(sample.right_mps, 4), "east_mps": round(sample.east_mps, 4), "north_mps": round(sample.north_mps, 4)})  # fmt: skip
        self._outputs[STATUS_TOPIC].publish(status)
        if self._send:
            fields = self._flow.mavlink_fields(sample, self._focal_px)
            flow = self._OpticalFlow()
            flow.header.stamp, flow.header.frame_id = header.stamp, header.frame_id
            flow.flow.x, flow.flow.y = float(fields["flow"][0]), float(fields["flow"][1])
            flow.flow_comp_m.x, flow.flow_comp_m.y = float(fields["flow_comp_m"][0]), float(fields["flow_comp_m"][1])  # forward and left, as the twist below
            flow.flow_rate.x, flow.flow_rate.y = float(fields["flow_rate"][0]), float(fields["flow_rate"][1])
            flow.quality, flow.ground_distance = int(fields["quality"]), float(fields["ground_distance"])
            self._outputs[OPTICAL_FLOW_TOPIC].publish(flow)
        if sample.quality == 0:
            self.whys[sample.why] = self.whys.get(sample.why, 0) + 1
            return
        self.counts["with quality"] += 1
        for topic, frame_id, x, y in ((VELOCITY_TOPIC, "base_link", sample.forward_mps, -sample.right_mps), (VELOCITY_ENU_TOPIC, "map", sample.east_mps, sample.north_mps)):
            twist = self._TwistStamped()
            twist.header.stamp, twist.header.frame_id = header.stamp, frame_id
            twist.twist.linear.x, twist.twist.linear.y = float(x), float(y)
            self._outputs[topic].publish(twist)

    def _report(self, stamp_ns: int) -> None:
        if self._last_report_ns is None:
            self._last_report_ns = stamp_ns
            return
        if abs(stamp_ns - self._last_report_ns) < int(REPORT_EVERY_S * S):
            return
        self._last_report_ns = stamp_ns
        without = "; ".join(f"{why}: {count}" for why, count in sorted(self.whys.items(), key=lambda item: -item[1])[:3])
        self.get_logger().info(
            f"Flow odometry: {self.counts['frames']} frames so far, {self.counts['samples']} samples, {self.counts['with quality']} with quality" + (f"; without: {without}" if without else "")
        )


def main() -> None:
    rclpy.init()
    node = FlowOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
