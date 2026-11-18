"""Legacy adapter (FND-02): keeps the 2026 target-detection topics fed from a detector on the detection contract.

  ros2 run drone_target_cv detection_legacy_adapter --ros-args -p detector:=ring_fit

The 2026 missions read /drone_package_drop/target_detection (a point: x and y
from -1 to 1 across the image, the area in z; -1, -1 for nothing found) and
/drone_package_drop/image_size. A detector that publishes only on the
contract (/perception/<detector>/detections) is turned back into those two
here. Thin wrapper: the conversion and its two guards (a floor on the score,
no stamp from the future) are in contract_output.py.

The 2026 missions also wait for the service
/drone_package_drop/set_target_cv_enabled before they start. The adapter
does not serve it: the detector it adapts has to.

Nothing shipped starts this node. It must not run beside the 2026 detector,
which publishes those two topics itself: `detector` has no default, and
while anything else publishes the 2026 target topic the adapter stays
silent and says so.
"""

from __future__ import annotations

import rclpy
import vision_msgs.msg as vision_messages
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

from drone_mission_core import detection_contract as contract
from drone_target_cv.contract_output import ADAPTER_MIN_SCORE, legacy_messages, stamp_is_usable


class LegacyAdapter(Node):
    def __init__(self) -> None:
        super().__init__("detection_legacy_adapter")
        self.declare_parameter("detector", "")  # which detector's contract topic to adapt; there is no default
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("min_score", ADAPTER_MIN_SCORE)  # a bullseye under this is "nothing found" for the missions
        self._min_score = float(self.get_parameter("min_score").value)
        self._image_size = None  # from camera_info: the contract's pixels are the full image's
        self._point_pub = self.create_publisher(PointStamped, "/drone_package_drop/target_detection", 10)
        self._size_pub = self.create_publisher(PointStamped, "/drone_package_drop/image_size", 10)
        detector = str(self.get_parameter("detector").value)
        if not detector:
            raise ValueError("detection_legacy_adapter: say which detector to adapt, for example -p detector:=ring_fit")
        topic = contract.topic_for(detector)
        self.create_subscription(vision_messages.Detection2DArray, topic, self._on_detections, 10)
        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value, self._on_camera_info, qos_profile_sensor_data)
        self.get_logger().info(f"Legacy adapter: {topic} -> /drone_package_drop/target_detection")

    def _on_camera_info(self, message: CameraInfo) -> None:
        if message.width and message.height:
            self._image_size = (int(message.width), int(message.height))

    def _on_detections(self, message) -> None:
        if self._image_size is None:
            return  # the image size is not known yet, so a pixel cannot be put on the -1..1 scale
        if self.count_publishers("/drone_package_drop/target_detection") > 1:
            # Something else, most likely the 2026 detector, is already feeding the missions.
            self.get_logger().error(
                "Another node publishes /drone_package_drop/target_detection: the adapter is not publishing beside it",
                throttle_duration_sec=5.0,
            )
            return
        try:
            found = contract.decode(message)
        except contract.ContractError as error:
            self.get_logger().error(f"Message outside the detection contract: {error}", throttle_duration_sec=5.0)
            return
        if not stamp_is_usable(found.stamp_ns, self.get_clock().now().nanoseconds):
            self.get_logger().error(
                "Detection stamped ahead of this node's clock (another time base?): not passed to the missions",
                throttle_duration_sec=5.0,
            )
            return
        point, size = legacy_messages(found, self._image_size, self._min_score)
        stamp = Time(sec=found.stamp_ns // 1_000_000_000, nanosec=found.stamp_ns % 1_000_000_000)
        for publisher, values in ((self._size_pub, size), (self._point_pub, point)):
            out = PointStamped()
            out.header.stamp = stamp  # the image's capture time, as the contract carries it
            out.header.frame_id = "camera"  # the 2026 frame name
            out.point.x, out.point.y, out.point.z = values
            publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = LegacyAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
