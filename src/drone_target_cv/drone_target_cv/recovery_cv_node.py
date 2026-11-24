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

Visual homing (REC-7b), two more node flags, each false by default and each
needing `features.recovery_cv`: with `features.marker_homing` the home
marker (an AprilTag of id `home_marker_id`, home_marker.py) is looked for in
each frame, and with `features.keyframe_homing` the pictures of the takeoff
climb are kept and the takeoff point is looked for in the frames of the way
back (keyframe_homing.py; this one reads the vehicle's pose on `pose_topic`
to know the climb and where the takeoff point is in each kept picture).
The marker goes out as a `home_marker`, the takeoff point as a
`takeoff_point`, each in a message of its own with the frame's stamp, after
the frame's X and bucket have gone out: the pickup's detections do not wait
on homing. Keyframe homing also reads the flight controller's landed state
(`extended_state_topic`): the takeoff point is where the vehicle stands when
it is on the ground. The marker is looked for at most MARKER_HZ times a
second and the keyframes matched at most KEYFRAME_HZ; the slowest of either
is in the node's ten-second report.
"""

from __future__ import annotations

import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node

from drone_target_cv.ring_fit_node import _stamp_ns, image_to_bgr

MARKER_HZ = 30.0  # the home marker is looked for at most this often: a look costs about 5 ms (the sim's container), and on the approach the tag is readable in few frames
KEYFRAME_HZ = 3.0  # the takeoff point is looked for in at most this many frames a second (ORB and a homography each time: up to 0.3 s in the sim's container)
EXTENDED_STATE_TOPIC = "/mavros/extended_state"
LANDED, IN_AIR = 1, 2  # mavros_msgs/ExtendedState landed_state
HELD_FRAMES = 3  # frames kept for keyframe homing until the pose history reaches their moment
HELD_MAX_AGE_S = 0.5  # a held frame the poses have not reached in this long is dropped
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
        self.declare_parameter("features.marker_homing", False)
        self.declare_parameter("features.keyframe_homing", False)
        self.declare_parameter("home_marker_id", 0)
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("extended_state_topic", EXTENDED_STATE_TOPIC)
        self._landed: Optional[bool] = None  # the flight controller's landed state, for keyframe homing (None: not heard)
        self._last_marker_ns: Optional[int] = None
        self._last_keyframe_ns: Optional[int] = None
        self.slowest_homing_ms = 0.0
        self._marker_id: Optional[int] = None  # the home marker's id, when it is looked for
        self._keyframes: Any = None  # keyframe_homing.KeyframeHoming, when the takeoff's pictures are kept
        self._held: list = []  # (stamp, picture, frame id) of frames waiting for the pose of their moment
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
        self._start_homing(camera, _mount)
        self.get_logger().info(
            f"Recovery detector on (features.recovery_cv): {image_topic}, queue of one, camera {mode} on {airframe} "
            f"({self._size[0]} x {self._size[1]}), publishing on {contract.topic_for(DETECTOR_NAME)}"
        )

    def _start_homing(self, camera: Any, mount: Any) -> None:
        """Visual homing (REC-7b), each part behind its own flag. A part that cannot start says so and is left out: the detector goes on."""
        if bool(self.get_parameter("features.marker_homing").get_parameter_value().bool_value):
            from drone_target_cv import home_marker

            if home_marker.available():
                self._home_marker, self._marker_id = home_marker, int(self.get_parameter("home_marker_id").value)
                self.counts["home marker found"] = 0
                self.get_logger().info(f"Home marker on (features.marker_homing): AprilTag {home_marker.FAMILY} id {self._marker_id}, reported as {home_marker.CLASS_ID}")
            else:
                self.get_logger().error("features.marker_homing is on but this OpenCV has no aruco module with the AprilTag 36h11 dictionary: the home marker is not looked for")
        if bool(self.get_parameter("features.keyframe_homing").get_parameter_value().bool_value):
            from geometry_msgs.msg import PoseStamped
            from mavros_msgs.msg import ExtendedState
            from rclpy.qos import qos_profile_sensor_data

            from drone_mission_core import projection_v2
            from drone_mission_core.pose_buffer import PoseBuffer
            from drone_target_cv import keyframe_homing

            self._projection, self._camera, self._mount = projection_v2, camera, mount
            self._poses, self._climb, self._keyframes = PoseBuffer(), keyframe_homing.Climb(), keyframe_homing.KeyframeHoming()
            self._keyframe_homing = keyframe_homing
            self.counts["keyframes kept"] = self.counts["takeoff point found"] = 0
            pose_topic = str(self.get_parameter("pose_topic").get_parameter_value().string_value)
            state_topic = str(self.get_parameter("extended_state_topic").get_parameter_value().string_value)
            self.create_subscription(PoseStamped, pose_topic, self._on_pose, qos_profile_sensor_data)
            self.create_subscription(ExtendedState, state_topic, self._on_extended_state, 10)
            self.get_logger().info(
                f"Keyframe homing on (features.keyframe_homing): pictures kept at {', '.join(f'{h:g}' for h in self._keyframes.heights_m)} m of the takeoff climb, "
                f"the vehicle's pose from {pose_topic}, on the ground or not from {state_topic}"
            )

    def _on_extended_state(self, message: Any) -> None:
        state = int(getattr(message, "landed_state", 0))
        self._landed = True if state == LANDED else False if state == IN_AIR else None  # (taking off, landing, undefined: not known)

    def _on_pose(self, message: Any) -> None:
        position, orientation = message.pose.position, message.pose.orientation
        stamp_ns = _stamp_ns(message.header.stamp)
        self._poses.add_pose(stamp_ns, (position.x, position.y, position.z), (orientation.x, orientation.y, orientation.z, orientation.w))
        if self._climb.update(stamp_ns * 1.0e-9, (position.x, position.y, position.z), self._landed):
            self._keyframes.clear()
            self.get_logger().info(f"Keyframe homing: the takeoff point is ({position.x:.2f}, {position.y:.2f}) in the local frame (takeoff {self._climb.takeoffs})")
        self._handle_held()

    def _handle_held(self) -> None:
        """Keyframe homing for the held frames the pose history has reached, oldest first: each one's `takeoff_point`, if it has one, goes out
        in a message of its own with the frame's own stamp (the X and the bucket of that frame went out when it came)."""
        newest = self._poses.newest_ns()
        while self._held and newest is not None:
            stamp_ns, frame, frame_id = self._held[0]
            if newest < stamp_ns:
                if newest < stamp_ns - int(HELD_MAX_AGE_S * 1.0e9) or len(self._held) > HELD_FRAMES:
                    self._held.pop(0)  # the poses are not coming, or frames are: not kept for ever
                    self.counts["frames with no pose for keyframe homing"] = self.counts.get("frames with no pose for keyframe homing", 0) + 1
                    continue
                return
            self._held.pop(0)
            try:
                detections = self._keyframe_detections(frame, stamp_ns)
                if detections and self._publisher is not None:
                    self._publisher.publish(self._contract.encode(self._contract.DetectionSet(stamp_ns=stamp_ns, frame_id=frame_id, detections=tuple(detections)), self._vision_messages))
            except Exception as error:  # noqa: BLE001  (one frame that cannot be handled must not stop the node, nor the poses)
                self.get_logger().error(f"Keyframe homing failed on a frame: {error}", throttle_duration_sec=5.0)

    def _homing(self, frame: Any, stamp_ns: int, frame_id: str) -> None:
        """Homing's share of a frame, after the frame's X and bucket have gone out: the home marker if it is in the frame (a message of
        its own), and the frame held for keyframe homing until the pose of its moment has come (a frame is often here before its pose)."""
        if self._marker_id is not None and (self._last_marker_ns is None or abs(stamp_ns - self._last_marker_ns) >= int(1.0e9 / MARKER_HZ)):
            self._last_marker_ns = stamp_ns
            started = time.perf_counter()
            sighting = self._home_marker.find(frame, self._marker_id)
            self.slowest_homing_ms = max(self.slowest_homing_ms, (time.perf_counter() - started) * 1000.0)
            if sighting is not None:
                if not self.counts["home marker found"]:
                    self.get_logger().info(f"Home marker first found, at ({sighting.u:.0f}, {sighting.v:.0f}) px, {sighting.side_px:.0f} px across")
                self.counts["home marker found"] += 1
                found = self._contract.DetectionSet(stamp_ns=stamp_ns, frame_id=frame_id, detections=(self._contract.Detection(self._home_marker.CLASS_ID, self._home_marker.SCORE, *self._home_marker.box(sighting)),))
                self._publisher.publish(self._contract.encode(found, self._vision_messages))
        if self._keyframes is not None and self._climb.home is not None:
            self._held.append((int(stamp_ns), frame.copy(), str(frame_id)))
            self._handle_held()

    def _keyframe_detections(self, frame: Any, stamp_ns: int) -> list:
        """The takeoff point in a frame whose pose is known: kept as one of the climb's pictures, or looked for in it."""
        pose = self._poses.pose_at(stamp_ns)
        if pose is None or self._climb.home is None:
            return []
        height = self._climb.height_m(pose.position)
        started = time.perf_counter()
        try:
            return self._keyframe_step(frame, stamp_ns, pose, height)
        finally:
            self.slowest_homing_ms = max(self.slowest_homing_ms, (time.perf_counter() - started) * 1000.0)

    def _keyframe_step(self, frame: Any, stamp_ns: int, pose: Any, height: float) -> list:
        if self._climb.keeping(pose.position):
            if self._keyframes.wanted(height) is not None:
                home_px = self._projection.pixel_of_ground_point(self._camera, self._mount, self._climb.home, pose.position, pose.orientation)
                if self._keyframes.offer(frame, height, home_px):
                    self.counts["keyframes kept"] += 1
                    self.get_logger().info(f"Keyframe homing: a picture kept at {height:.1f} m, the takeoff point at ({home_px[0]:.0f}, {home_px[1]:.0f}) px in it ({len(self._keyframes.keyframes)} kept)")
            return []
        if not self._climb.searching(pose.position):
            return []
        if self._last_keyframe_ns is not None and abs(stamp_ns - self._last_keyframe_ns) < int(1.0e9 / KEYFRAME_HZ):
            return []
        self._last_keyframe_ns = stamp_ns
        sighting = self._keyframes.find(frame, height)
        if sighting is None:
            return []
        if not self.counts["takeoff point found"]:
            self.get_logger().info(f"Keyframe homing: the takeoff point first found, at ({sighting.u:.0f}, {sighting.v:.0f}) px by {sighting.inliers} matches with the picture kept at {sighting.keyframe_height_m:.1f} m")
        self.counts["takeoff point found"] += 1
        side = 0.1 * min(self._size)  # (the box only says how sure the pixel is: a tenth of the picture)
        return [self._contract.Detection(self._keyframe_homing.CLASS_ID, sighting.score, sighting.u, sighting.v, side, side)]

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
            + (f"; homing's slowest frame took {self.slowest_homing_ms:.0f} ms" if self._marker_id is not None or self._keyframes is not None else "")
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
            if self._marker_id is not None or self._keyframes is not None:
                try:
                    self._homing(frame, stamp_ns, str(header.frame_id))  # after the frame's own detections are out: they do not wait on it
                except Exception as error:  # noqa: BLE001  (homing that fails on a frame must not take the node with it)
                    self.get_logger().error(f"Visual homing failed on a frame: {error}", throttle_duration_sec=5.0)
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
