"""Feeding the pose history buffer from MAVROS (FND-04): the executor's thin ROS side.

Behind the mission flag `use_frame_time_pose`, which defaults to false. With
the flag off the executor keeps no pose history and makes no extra
subscription. With it on in the sequence, `attach_pose_buffer` gives the
executor a `PoseBuffer` (pose_buffer.py) holding the last five seconds of
MAVROS's pose and velocity, for whatever needs the pose at a frame's capture
time (FND-05's projection, FND-07's context call).

With the same flag on, `attach_ground_height` gives the executor a
`GroundHeight` (projection_v2.py) fed by MAVROS's rangefinder topic, so the
projection knows where the ground is when the rangefinder is healthy.
"""

from __future__ import annotations

from typing import Any, Iterable, Optional

from .features import get_feature_flag
from .pose_buffer import PoseBuffer

FRAME_TIME_FLAG = "use_frame_time_pose"
POSE_TOPIC = "/mavros/local_position/pose"
VELOCITY_TOPIC = "/mavros/local_position/velocity_local"  # linear velocity east, north, up
RANGEFINDER_TOPIC = "/mavros/rangefinder/rangefinder"  # sensor_msgs/Range, the distance along the downward beam


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def attach_pose_buffer(node: Any, specs: Iterable[Any]) -> Optional[PoseBuffer]:
    """A PoseBuffer fed by `node`'s own subscriptions, or None when the flag is off.

    The subscriptions are the buffer's own, beside the executor's, so the
    executor's 2026 pose callback is not touched.
    """
    if not any(get_feature_flag(spec.features, FRAME_TIME_FLAG) for spec in specs):
        return None
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from rclpy.qos import qos_profile_sensor_data

    buffer = PoseBuffer()

    def on_pose(message: Any) -> None:
        position, orientation = message.pose.position, message.pose.orientation
        buffer.add_pose(
            _stamp_ns(message.header.stamp),
            (position.x, position.y, position.z),
            (orientation.x, orientation.y, orientation.z, orientation.w),
        )

    def on_velocity(message: Any) -> None:
        linear = message.twist.linear
        buffer.add_velocity(_stamp_ns(message.header.stamp), (linear.x, linear.y, linear.z))

    node.create_subscription(PoseStamped, POSE_TOPIC, on_pose, qos_profile_sensor_data)
    node.create_subscription(TwistStamped, VELOCITY_TOPIC, on_velocity, qos_profile_sensor_data)
    node.get_logger().info("Pose history on (use_frame_time_pose): keeping 5 s of pose and velocity")
    return buffer


def attach_ground_height(node: Any, pose_buffer: Optional[PoseBuffer]) -> Optional[Any]:
    """A GroundHeight fed by the rangefinder through `node`'s own subscription, or None when there is no pose buffer (the flag is off)."""
    if pose_buffer is None:
        return None
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Range

    from .projection_v2 import GroundHeight  # needs numpy: imported only with the flag on

    ground = GroundHeight(pose_buffer.pose_at)

    def on_range(message: Any) -> None:
        ground.add_range(_stamp_ns(message.header.stamp), message.range, message.min_range, message.max_range)

    node.create_subscription(Range, RANGEFINDER_TOPIC, on_range, qos_profile_sensor_data)
    return ground
