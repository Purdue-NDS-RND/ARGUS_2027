"""Executor-owned context shared with missions."""

from __future__ import annotations

import math
from typing import Any, Callable, Optional

from geometry_msgs.msg import PointStamped, PoseStamped
from mavros_msgs.msg import (
    AttitudeTarget,
    ExtendedState,
    GlobalPositionTarget,
    PositionTarget,
    State,
)
from mavros_msgs.srv import CommandLong, CommandTOL, SetMode
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import SetBool

from .target_access import (
    ACTUATION_DELAY_S,
    ESTIMATOR_OFF,
    FIX_MAX_AGE_S,
    FIX_POSE_MAX_APART_S,
    POSE_MAX_AGE_S,
    Permission,
    descent_permission,
    from_estimate,
    height_agl,
    latlon_to_local_enu,
    release_permission,
    within,
)

ACTUATION_DELAY_NS = int(ACTUATION_DELAY_S * 1.0e9)


class MissionContext:
    """Facade over shared telemetry, services, and managed setpoints."""

    def __init__(self, node: Node) -> None:
        self._node = node
        self._managed_local_setpoint: Optional[PoseStamped] = None

    @property
    def mavros_state(self) -> Optional[State]:
        return self._node._mavros_state

    @property
    def local_pose(self) -> Optional[PoseStamped]:
        return self._node._local_pose

    @property
    def extended_state(self) -> Optional[ExtendedState]:
        return self._node._extended_state

    @property
    def global_gps(self) -> Optional[NavSatFix]:
        return self._node._global_gps

    @property
    def target_detection(self) -> Optional[PointStamped]:
        return self._node._target_detection

    @property
    def image_size(self) -> Optional[tuple[int, int]]:
        return self._node._image_size

    @property
    def logger(self) -> RcutilsLogger:
        return self._node.get_logger()

    def now(self) -> Time:
        return self._node.get_clock().now()

    def time_health(self):
        """The stale-data guard's report (time_health.py), or None with the time_health_guard flag off."""
        monitor = getattr(self._node, "_time_health", None)
        if monitor is None:
            return None
        return monitor.report(self.now().nanoseconds * 1.0e-9)

    def pose_at(self, stamp_ns: int):
        """The vehicle's pose at a past instant, from the pose history (pose_buffer.py).

        None with the use_frame_time_pose flag off, and None when the history
        has no pose for that instant. It is never the latest pose instead.
        """
        buffer = getattr(self._node, "_pose_buffer", None)
        if buffer is None:
            return None
        return buffer.pose_at(int(stamp_ns))

    def vehicle_velocity(self):
        """The vehicle's newest velocity (east, north, up) in m/s from the pose history, or None.

        None with the use_frame_time_pose flag off, or when the newest
        velocity is more than 0.2 s from now.
        """
        buffer = getattr(self._node, "_pose_buffer", None)
        if buffer is None:
            return None
        return buffer.newest_velocity(self._now_ns())

    def ground_projector(self, camera, mount):
        """Projection v2 (projection_v2.py) on this executor's pose history and ground height.

        None with the use_frame_time_pose flag off.
        """
        buffer = getattr(self._node, "_pose_buffer", None)
        if buffer is None:
            return None
        from .projection_v2 import GroundProjector  # needs numpy: imported only with the flag on

        return GroundProjector(
            camera,
            mount,
            buffer.pose_at,
            getattr(self._node, "_ground_height", None),
        )

    # --- the target estimator (FND-07) --------------------------------------------------
    #
    # With the use_target_estimator flag off there is no estimator: no target,
    # not ready, no permission. The rules for acting on a track are answered
    # here (target_access.py), not in each mission.

    def _now_ns(self) -> int:
        return int(self.now().nanoseconds)

    def estimator_ready(self) -> bool:
        """Whether the target estimator is running and could place a frame taken now (the pose history is live)."""
        feed = getattr(self._node, "_target_estimator", None)
        return feed is not None and feed.ready(self._now_ns())

    def actuation_time_ns(self) -> int:
        """The time a command given now takes effect: now + 0.08 s. The estimator's outputs are given for this time."""
        return self._now_ns() + ACTUATION_DELAY_NS

    def primary_target(self, class_id: str, *, near=None):
        """The track of a class to act on, as a `target_access.PrimaryTarget`: position, covariance, age and state.

        None with the estimator off or when the class has no track. `near`
        (east, north) picks the track nearest a point among equals. The
        track may be COASTING, LOST or TENTATIVE: ask `may_descend` and
        `may_release` before acting on it.
        """
        feed = getattr(self._node, "_target_estimator", None)
        if feed is None:
            return None
        estimate = feed.primary(class_id, self._now_ns(), near)
        return None if estimate is None else from_estimate(estimate)

    def tracked_target(self, class_id: str, track_id: int):
        """The track with this id, of this class, as a `target_access.PrimaryTarget`, or None when it is gone or the estimator is off.

        For a mission that keeps to the track it chose (from
        `primary_target(...).track_id`), whichever track is the class's
        primary one now.
        """
        feed = getattr(self._node, "_target_estimator", None)
        if feed is None:
            return None
        for estimate in feed.estimates(self._now_ns(), class_id):
            if estimate.track_id == track_id:
                return from_estimate(estimate)
        return None

    def _target_to_judge(self, class_id: str, near, track_id: Optional[int]):
        """(the track a rule is asked about, or None; why there is none)."""
        feed = getattr(self._node, "_target_estimator", None)
        if feed is None:
            return None, ESTIMATOR_OFF
        if track_id is None:
            return self.primary_target(class_id, near=near), ""
        for estimate in feed.estimates(self._now_ns(), class_id):
            if estimate.track_id == track_id:
                return from_estimate(estimate), ""
        return None, f"{class_id} track {track_id} is gone"

    def may_descend(self, class_id: str, *, near=None, track_id: Optional[int] = None) -> Permission:
        """Descending needs a CONFIRMED track no older than 0.5 s. False, with the reason, otherwise.

        Give `track_id` (from `primary_target(...).track_id`) to ask about the
        track being flown to; without it the answer is about whichever track
        of the class is primary now, which may be another target.
        """
        target, why_none = self._target_to_judge(class_id, near, track_id)
        if target is None and why_none:
            return Permission(False, why_none, track_id)
        return descent_permission(target)

    def may_release(self, class_id: str, *, near=None, track_id: Optional[int] = None) -> Permission:
        """Releasing needs a CONFIRMED track no older than 0.3 s. False, with the reason, otherwise. `track_id` as for `may_descend`."""
        target, why_none = self._target_to_judge(class_id, near, track_id)
        if target is None and why_none:
            return Permission(False, why_none, track_id)
        return release_permission(target)

    @staticmethod
    def _stamp_ns(message) -> Optional[int]:
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        return None if stamp is None else int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def height_agl(self, ground_below_home_m: float = 0.0):
        """Height above the ground as a `target_access.HeightAgl` (value and "rangefinder" or "baro"), or None.

        The rangefinder's when projection v2's ground height has a healthy
        one (use_frame_time_pose on); otherwise the height above home, which
        is not the height above the ground wherever the ground is not at
        home's height: read `source`. None with no pose yet, or when the
        newest pose is more than 0.5 s from now (the vehicle has moved since);
        `pose_age_s` says how old the pose was, for a caller near a limit.

        `ground_below_home_m` is how far the ground here is known to be below
        home (negative: above it); it is added to the height above home.

        With use_height_estimator on (CEN-03) the answer is the height
        estimator's: the rangefinder only inside the sensor's range and under
        15 degrees of tilt, the height above home plus the known ground
        offset otherwise, with `reason` saying why.
        """
        pose = self._node._local_pose
        now_ns = self._now_ns()
        if pose is None or not within(self._stamp_ns(pose), now_ns, POSE_MAX_AGE_S):
            return None
        pose_age_s = (now_ns - self._stamp_ns(pose)) * 1.0e-9
        estimator = getattr(self._node, "_height_estimator", None)
        if estimator is not None:
            orientation = pose.pose.orientation
            return estimator.height(
                pose.pose.position.z, now_ns, ground_below_home_m, pose_age_s, (orientation.x, orientation.y, orientation.z, orientation.w)
            )
        ground_height = getattr(self._node, "_ground_height", None)
        ground = None if ground_height is None else ground_height.ground_up(now_ns)
        return height_agl(pose.pose.position.z, ground, pose_age_s, ground_below_home_m)

    def rangefinder_range(self):
        """The rangefinder's newest reading as (range in metres, age in seconds), or None.

        The reading as it came, even outside the sensor's range: not a height
        (that is `height_agl`). None with the use_height_estimator flag off,
        or before any reading. For the delivery's contact rule (CEN-05).
        """
        estimator = getattr(self._node, "_height_estimator", None)
        if estimator is None:
            return None
        return estimator.newest_range(self._now_ns())

    def throttle(self):
        """The flight controller's newest throttle as (0 to 1, age in seconds), or None.

        None with the delivery_soft_set_down flag off, or before any was heard.
        """
        feed = getattr(self._node, "_throttle", None)
        if feed is None:
            return None
        return feed.newest(self._now_ns())

    def latlon_to_local_enu(self, latitude: float, longitude: float):
        """(east, north) of a latitude and longitude in MAVROS's local frame, or None when it cannot be said now.

        The GPS fix is paired with the vehicle's local position of the same
        moment: from the pose history when there is one, otherwise the
        newest pose if it is within 0.25 s of the fix. None with no fix (or a
        status that says no fix), a fix more than 2 s old, or no pose of the
        fix's moment; ask again on the next tick.
        """
        fix, pose = self._node._global_gps, self._node._local_pose
        if fix is None or pose is None:
            return None
        if getattr(getattr(fix, "status", None), "status", 0) < 0:  # sensor_msgs/NavSatStatus: -1 is no fix
            return None
        fix_ns = self._stamp_ns(fix)
        if not within(fix_ns, self._now_ns(), FIX_MAX_AGE_S):
            return None
        at_fix = self.pose_at(fix_ns)
        if at_fix is not None:
            here_east, here_north = at_fix.position[0], at_fix.position[1]
        elif within(self._stamp_ns(pose), fix_ns, FIX_POSE_MAX_APART_S):
            here_east, here_north = pose.pose.position.x, pose.pose.position.y
        else:
            return None
        return latlon_to_local_enu(latitude, longitude, fix.latitude, fix.longitude, here_east, here_north)

    def seconds_since(self, start_time: Time) -> float:
        return (self.now() - start_time).nanoseconds / 1e9

    def landing_state_available(self) -> bool:
        extended_state = self.extended_state
        if extended_state is None:
            return False
        return extended_state.landed_state != ExtendedState.LANDED_STATE_UNDEFINED

    def vehicle_is_landed(self) -> bool:
        extended_state = self.extended_state
        if extended_state is None:
            return False
        return extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND

    def takeoff_service_ready(self) -> bool:
        return self._node._takeoff_client.service_is_ready()

    def mode_service_ready(self) -> bool:
        return self._node._mode_client.service_is_ready()

    def command_service_ready(self) -> bool:
        return self._node._command_client.service_is_ready()

    def target_cv_control_ready(self) -> bool:
        return self._node._target_cv_control_client.service_is_ready()

    def request_takeoff(
        self,
        altitude_m: float,
        done_callback: Callable[[Future[Any]], None] | None = None,
    ) -> Future[Any]:
        request = CommandTOL.Request()
        request.altitude = float(altitude_m)
        future = self._node._takeoff_client.call_async(request)
        if done_callback is not None:
            future.add_done_callback(done_callback)
        return future

    def request_mode_change(
        self,
        mode_name: str,
        done_callback: Callable[[Future[Any]], None] | None = None,
    ) -> Future[Any]:
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode_name
        future = self._node._mode_client.call_async(request)
        if done_callback is not None:
            future.add_done_callback(done_callback)
        return future

    def command_gripper(
        self,
        *,
        release: bool,
        done_callback: Callable[[Future[Any]], None] | None = None,
    ) -> Future[Any]:
        request = CommandLong.Request()
        request.broadcast = False
        request.command = 211
        request.confirmation = 0
        request.param1 = 1.0
        request.param2 = 0.0 if release else 1.0
        future = self._node._command_client.call_async(request)
        if done_callback is not None:
            future.add_done_callback(done_callback)
        return future

    def command_sprayer(
        self,
        *,
        enable: bool,
        done_callback: Callable[[Future[Any]], None] | None = None,
    ) -> Future[Any]:
        request = CommandLong.Request()
        request.broadcast = False
        request.command = 216
        request.confirmation = 0
        request.param1 = 1.0 if enable else 0.0
        request.param2 = 0.0
        future = self._node._command_client.call_async(request)
        if done_callback is not None:
            future.add_done_callback(done_callback)
        return future

    def set_target_cv_enabled(
        self,
        enabled: bool,
        done_callback: Callable[[Future[Any]], None] | None = None,
    ) -> Future[Any]:
        request = SetBool.Request()
        request.data = bool(enabled)
        future = self._node._target_cv_control_client.call_async(request)
        if done_callback is not None:
            future.add_done_callback(done_callback)
        return future

    def clear_target_tracking_state(self) -> None:
        self._node._target_detection = None
        self._node._image_size = None

    def set_local_position_setpoint(
        self,
        east_m: float,
        north_m: float,
        up_m: float,
        yaw_deg: float = 90.0,
    ) -> None:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(east_m)
        msg.pose.position.y = float(north_m)
        msg.pose.position.z = float(up_m)

        yaw_rad = math.radians(yaw_deg)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        self._node._managed_global_setpoint = None
        self._node._managed_local_velocity_setpoint = None
        self._node._managed_attitude_setpoint = None
        self._managed_local_setpoint = msg

    def clear_local_position_setpoint(self) -> None:
        self._managed_local_setpoint = None

    def set_global_position_setpoint(
        self,
        latitude: float,
        longitude: float,
        altitude_m: float,
        yaw_deg: float = 90.0,
        lock_yaw: bool = True,
    ) -> None:
        msg = GlobalPositionTarget()
        msg.header.frame_id = "map"
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        msg.latitude = float(latitude)
        msg.longitude = float(longitude)
        msg.altitude = float(altitude_m)
        msg.yaw = math.radians(yaw_deg)
        msg.type_mask = (
            GlobalPositionTarget.IGNORE_VX
            | GlobalPositionTarget.IGNORE_VY
            | GlobalPositionTarget.IGNORE_VZ
            | GlobalPositionTarget.IGNORE_AFX
            | GlobalPositionTarget.IGNORE_AFY
            | GlobalPositionTarget.IGNORE_AFZ
            | GlobalPositionTarget.IGNORE_YAW_RATE
        )
        if not lock_yaw:
            msg.type_mask |= GlobalPositionTarget.IGNORE_YAW
        self._managed_local_setpoint = None
        self._node._managed_local_velocity_setpoint = None
        self._node._managed_attitude_setpoint = None
        self._node._managed_global_setpoint = msg

    def clear_global_position_setpoint(self) -> None:
        self._node._managed_global_setpoint = None

    def set_local_velocity_setpoint(
        self,
        east_mps: float,
        north_mps: float,
        up_mps: float = 0.0,
        yaw_deg: float = 90.0,
    ) -> None:
        msg = PositionTarget()
        msg.header.frame_id = "map"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        msg.velocity.x = float(east_mps)
        msg.velocity.y = float(north_mps)
        msg.velocity.z = float(up_mps)
        msg.yaw = math.radians(yaw_deg)
        self._managed_local_setpoint = None
        self._node._managed_global_setpoint = None
        self._node._managed_attitude_setpoint = None
        self._node._managed_local_velocity_setpoint = msg

    def clear_local_velocity_setpoint(self) -> None:
        self._node._managed_local_velocity_setpoint = None

    def set_attitude_climb_rate_setpoint(
        self,
        climb_rate_mps: float,
        yaw_deg: float = 90.0,
        max_climb_rate_mps: float = 2.5,
    ) -> None:
        msg = AttitudeTarget()
        msg.header.frame_id = "map"
        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE
            | AttitudeTarget.IGNORE_PITCH_RATE
            | AttitudeTarget.IGNORE_YAW_RATE
        )

        yaw_rad = math.radians(yaw_deg)
        msg.orientation.w = math.cos(yaw_rad / 2.0)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(yaw_rad / 2.0)

        capped_max_climb = max(float(max_climb_rate_mps), 0.01)
        normalized_climb = max(
            -1.0,
            min(float(climb_rate_mps) / capped_max_climb, 1.0),
        )
        msg.thrust = 0.5 + 0.5 * normalized_climb

        self._managed_local_setpoint = None
        self._node._managed_global_setpoint = None
        self._node._managed_local_velocity_setpoint = None
        self._node._managed_attitude_setpoint = msg

    def clear_attitude_setpoint(self) -> None:
        self._node._managed_attitude_setpoint = None

    def clear_all_setpoints(self) -> None:
        self.clear_local_position_setpoint()
        self.clear_global_position_setpoint()
        self.clear_local_velocity_setpoint()
        self.clear_attitude_setpoint()

    def publish_managed_setpoints(self) -> None:
        if self._managed_local_setpoint is None:
            attitude_setpoint = self._node._managed_attitude_setpoint
            if attitude_setpoint is not None:
                attitude_setpoint.header.stamp = self.now().to_msg()
                self._node._attitude_setpoint_pub.publish(attitude_setpoint)
                return

            local_velocity_setpoint = self._node._managed_local_velocity_setpoint
            if local_velocity_setpoint is not None:
                local_velocity_setpoint.header.stamp = self.now().to_msg()
                self._node._local_velocity_setpoint_pub.publish(local_velocity_setpoint)
                return

            global_setpoint = self._node._managed_global_setpoint
            if global_setpoint is not None:
                global_setpoint.header.stamp = self.now().to_msg()
                self._node._global_setpoint_pub.publish(global_setpoint)
            return

        self._managed_local_setpoint.header.stamp = self.now().to_msg()
        self._node._local_setpoint_pub.publish(self._managed_local_setpoint)
