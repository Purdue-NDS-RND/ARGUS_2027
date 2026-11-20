"""Shared helpers for vision-guided payload missions."""

from __future__ import annotations

import math
from dataclasses import dataclass

from drone_mission_core.centring import CENTRE_FLAG, EstimatorCentring
from drone_mission_core.centring_controller import (
    BARO_HEIGHT_MARGIN_M,
    RETUNE_FLAG,
    CentringController,
    CentringGains,
)
from drone_mission_core.features import get_feature_flag
from drone_mission_core.frame_time import FRAME_TIME_FLAG
from drone_mission_core.mission_api import BaseMission, MissionStatus
from drone_mission_core.mission_context import MissionContext
from drone_mission_core.reacquire import REACQUIRE_FLAG
from drone_mission_core.time_guard import (
    DEFAULT_HOLD_TIMEOUT_S,
    TIME_HEALTH_FLAG,
    TimeGuard,
)
from drone_mission_core.tracking_projection import (
    DEFAULT_CAMERA_CALIBRATION_HEIGHT_PX,
    DEFAULT_CAMERA_CALIBRATION_WIDTH_PX,
    DEFAULT_CAMERA_FX_PX,
    DEFAULT_CAMERA_FY_PX,
    CameraIntrinsics,
    GroundProjection,
    ground_offset_to_velocity,
    project_normalized_detection_to_ground_offset,
)
from rclpy.time import Time


EARTH_RADIUS_M = 6_371_000.0
# Projection v2 only (use_frame_time_pose): a frame older than this gives no
# tracking solution. A remembered ground point would otherwise let the vehicle
# reach "centred" by its own movement, with no fresh image to say so.
DEFAULT_FRAME_TIME_MAX_AGE_S = 0.5
MAX_ESTIMATOR_CENTERING_TOLERANCE_M = 1.0  # "centred on the estimator's track" is never looser than this
TRANSIT_YAW_DEG = 90.0
HOLD_YAW_DEG = 90.0


@dataclass(frozen=True)
class TrackingVelocityCommand:
    tracking_error_m: float
    velocity_east_mps: float
    velocity_north_mps: float
    vertical_velocity_mps: float
    reached_target_altitude: bool


@dataclass(frozen=True)
class RecoveryCommandResult:
    failed: bool
    reached_altitude: bool


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Horizontal distance in meters between two GPS points."""

    lat1_r, lat2_r = math.radians(lat1), math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon / 2.0) ** 2
    )
    return EARTH_RADIUS_M * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


class RedBullseyeMissionBase(BaseMission):
    """Common targeting and centering helpers for payload missions."""

    def __init__(self, spec) -> None:
        super().__init__(spec)
        # Projection v2 (FND-05), only with the use_frame_time_pose flag on:
        # the camera model and mount named by the mission's config. Loaded
        # when the sequence is built, so a sensor mode that is not calibrated
        # or an airframe nobody has measured stops the executor on the
        # ground, not the mission in the air.
        self._frame_camera = None
        self._target_offset = None
        self._last_frame_time_refusal_log: Time | None = None
        self._frame_time_max_age_s = DEFAULT_FRAME_TIME_MAX_AGE_S
        if get_feature_flag(spec.features, FRAME_TIME_FLAG):
            from drone_mission_core.frame_projection import camera_from_config

            self._frame_camera = camera_from_config(spec.config, spec.name)
            self._frame_time_max_age_s = float(
                spec.config.get("frame_time_max_age_s", DEFAULT_FRAME_TIME_MAX_AGE_S)
            )
        # Centring on the estimator (CEN-01), only with the centre_on_estimator
        # flag on: the error steered on is to the estimator's track, for the
        # payload's release point. The offset is read when the sequence is
        # built, so an airframe nobody has measured stops the executor on the
        # ground.
        self._estimator_centring: EstimatorCentring | None = None
        self._last_centring_log: Time | None = None
        self._last_centring_refusal_log: Time | None = None
        if get_feature_flag(spec.features, CENTRE_FLAG):
            from drone_mission_core import frame_projection

            self._estimator_centring = EstimatorCentring(
                frame_projection.payload_offset_from_config(spec.config, spec.name)
            )
        # How close to the estimator's track counts as centred (CEN-10), where
        # the sequence says: read with centre_on_estimator on only, and checked
        # here, when the sequence is built. A track that may not be descended
        # on is given an infinite error, so the tolerance has to be finite.
        self._estimator_centering_tolerance_m: float | None = None
        if self._estimator_centring is not None and "estimator_centering_tolerance_m" in spec.config:
            tolerance = spec.config["estimator_centering_tolerance_m"]
            if (
                isinstance(tolerance, bool)
                or not isinstance(tolerance, (int, float))
                or not 0.0 < tolerance <= MAX_ESTIMATOR_CENTERING_TOLERANCE_M
            ):
                raise ValueError(
                    f"mission '{spec.name}': estimator_centering_tolerance_m must be a distance above 0 m and "
                    f"no more than {MAX_ESTIMATOR_CENTERING_TOLERANCE_M:g} m, got {tolerance!r}"
                )
            self._estimator_centering_tolerance_m = float(tolerance)
        # Smart re-acquisition (CEN-06), only with the smart_reacquire flag on:
        # a lost track is looked for from the height that shows three sigma
        # of it and a margin, in place of the 2026 one-metre climb.
        self._smart_reacquire = None
        if get_feature_flag(spec.features, REACQUIRE_FLAG):
            from drone_mission_core.reacquire import half_footprint_per_m

            from .smart_reacquire import SmartReacquire

            camera = None if self._frame_camera is None else self._frame_camera[0]
            self._smart_reacquire = SmartReacquire(self, half_footprint_per_m(camera))
        # The retuned controller (CEN-02), only with the centring_retune flag
        # on: soft deadband, Kp, a speed cap that follows the height above the
        # ground, and a slew limit, in place of the 2026 gain and speed cap.
        # Its gains are read (the spec's, or the sequence's `centring_` keys)
        # when the sequence is built, so a bad one stops the executor on the
        # ground.
        self._centring_controller: CentringController | None = None
        self._last_centring_lead_log: Time | None = None
        if get_feature_flag(spec.features, RETUNE_FLAG):
            self._centring_controller = CentringController(
                CentringGains.from_config(spec.config)
            )

    def _load_common_vision_config(self, config: dict) -> None:
        self._target_latitude = float(config.get("target_latitude", 0.0))
        self._target_longitude = float(config.get("target_longitude", 0.0))
        self._transit_altitude_m = float(config.get("transit_altitude_m", 20.0))
        self._fake_drop = bool(config.get("fake_drop", False))
        self._centering_tolerance_m = float(config.get("centering_tolerance_m", 0.35))
        if self._estimator_centering_tolerance_m is not None:
            # centre_on_estimator only (CEN-10): how close counts as centred on
            # the estimator's track. The 2026 tolerance was tuned for the raw
            # detection's noise and stays what the flag-off path uses.
            self._centering_tolerance_m = self._estimator_centering_tolerance_m
        self._arrival_radius_m = float(config.get("arrival_radius_m", 3.0))
        self._arrival_alt_tolerance_m = float(
            config.get("arrival_alt_tolerance_m", 2.0)
        )
        self._not_found_ascent_m = float(config.get("not_found_ascent_m", 5.0))
        self._max_recovery_altitude_m = float(
            config.get("max_recovery_altitude_m", self._transit_altitude_m + 10.0)
        )
        self._max_recovery_attempts = int(config.get("max_recovery_attempts", 3))
        self._centering_dwell_s = float(config.get("centering_dwell_s", 1.0))
        self._centering_deadband_m = float(config.get("centering_deadband_m", 0.08))
        self._tracking_low_pass_alpha = float(
            config.get("tracking_low_pass_alpha", 0.25)
        )
        self._centering_gain_mps_per_m = float(
            config.get("centering_gain_mps_per_m", 0.75)
        )
        self._max_centering_speed_mps = float(
            config.get("max_centering_speed_mps", 2.0)
        )
        self._camera_intrinsics = CameraIntrinsics(
            fx_px=float(config.get("camera_fx_px", DEFAULT_CAMERA_FX_PX)),
            fy_px=float(config.get("camera_fy_px", DEFAULT_CAMERA_FY_PX)),
            calibration_width_px=float(
                config.get(
                    "camera_calibration_width_px",
                    DEFAULT_CAMERA_CALIBRATION_WIDTH_PX,
                )
            ),
            calibration_height_px=float(
                config.get(
                    "camera_calibration_height_px",
                    DEFAULT_CAMERA_CALIBRATION_HEIGHT_PX,
                )
            ),
        )
        self._camera_yaw_offset_deg = float(config.get("camera_yaw_offset_deg", 0.0))
        self._min_projection_altitude_m = float(
            config.get("min_projection_altitude_m", 0.75)
        )
        self._max_projection_distance_m = float(
            config.get("max_projection_distance_m", 25.0)
        )
        self._target_timeout_s = float(config.get("target_timeout_s", 2.0))
        self._target_loss_grace_s = float(
            config.get("target_loss_grace_s", self._target_timeout_s)
        )
        self._recovery_alt_tolerance_m = float(
            config.get("recovery_alt_tolerance_m", self._arrival_alt_tolerance_m)
        )
        self._failed = self._target_latitude == 0.0 and self._target_longitude == 0.0
        # Only read with the time_health_guard flag on (FND-03).
        self._time_guard_hold_timeout_s = float(
            config.get("time_guard_hold_timeout_s", DEFAULT_HOLD_TIMEOUT_S)
        )

    def _initialize_common_vision_state(self) -> None:
        self._actuator_requested = False
        self._recovery_attempts = 0
        self._centering_dwell_start: Time | None = None
        self._target_loss_start: Time | None = None
        self._recovery_target_altitude: float | None = None
        self._recovery_hold_position: tuple[float, float] | None = None
        self._target_cv_enabled = False
        self._target_cv_enable_requested = False
        self._filtered_tracking_east_m: float | None = None
        self._filtered_tracking_north_m: float | None = None
        self._last_velocity_log_time: Time | None = None
        self._time_guard = TimeGuard(self.name, self._time_guard_hold_timeout_s)
        self._time_guard_hold: tuple[float, float, float] | None = None
        if self._estimator_centring is not None:
            self._estimator_centring.forget()

    def _time_is_unhealthy(self, context: MissionContext) -> bool:
        """The stale-data guard (FND-03). Always False with its flag off."""
        if not get_feature_flag(self.spec.features, TIME_HEALTH_FLAG):
            return False
        unhealthy = self._time_guard.unhealthy(context)
        if not unhealthy:
            self._time_guard_hold = None
        return unhealthy

    def _hold_for_time_guard(
        self,
        context: MissionContext,
        column_position: tuple[float, float] | None,
    ) -> MissionStatus:
        """Stop a blind descent while the time base is unhealthy.

        The hold point is fixed when the hold starts: the column's frozen
        position (the vehicle's own before the column is frozen) at the
        vehicle's own height. Not the column's height setpoint: that runs
        ahead of the vehicle, and holding it would let the descent go on.
        The mission fails once the hold has lasted
        time_guard_hold_timeout_s, so the failure policy acts. The caller
        has checked that the pose and the GPS fix exist.
        """
        if self._time_guard.hold_expired(context):
            context.logger.error(
                f"[{self.name}] time base UNHEALTHY for "
                f"{self._time_guard_hold_timeout_s:.1f} s during a descent; failing"
            )
            return MissionStatus.FAILURE
        if self._time_guard_hold is None:
            if column_position is None:
                column_position = (
                    context.global_gps.latitude,
                    context.global_gps.longitude,
                )
            self._time_guard_hold = (
                column_position[0],
                column_position[1],
                float(context.local_pose.pose.position.z),
            )
        context.set_global_position_setpoint(
            self._time_guard_hold[0],
            self._time_guard_hold[1],
            self._time_guard_hold[2],
            yaw_deg=HOLD_YAW_DEG,
            lock_yaw=True,
        )
        return MissionStatus.RUNNING

    def _enter_common_mission(self, context: MissionContext) -> None:
        context.clear_all_setpoints()
        context.clear_target_tracking_state()
        self._request_target_cv_enable(context)
        if self._failed:
            context.logger.error(
                f"[{self.name}] target_latitude and target_longitude must be set"
            )

    def _exit_common_mission(self, context: MissionContext) -> None:
        self._reset_tracking_filter()
        context.clear_all_setpoints()
        context.clear_target_tracking_state()
        if context.target_cv_control_ready():
            context.set_target_cv_enabled(False)

    def _get_centering_descent_command(
        self,
        context: MissionContext,
        *,
        target_altitude_m: float,
        descent_rate_mps: float,
        max_centering_speed_mps: float | None = None,
        target_altitude_tolerance_m: float | None = None,
    ) -> TrackingVelocityCommand | None:
        if context.local_pose is None:
            return None

        tracking = self._get_tracking_solution(
            context,
            max_centering_speed_mps=max_centering_speed_mps,
        )
        if tracking is None:
            return None

        tracking_error_m, velocity_east, velocity_north = tracking
        altitude_error = abs(context.local_pose.pose.position.z - target_altitude_m)
        altitude_tolerance_m = self._arrival_alt_tolerance_m
        if target_altitude_tolerance_m is not None:
            altitude_tolerance_m = max(0.0, float(target_altitude_tolerance_m))
        reached_target_altitude = altitude_error <= altitude_tolerance_m

        vertical_velocity = 0.0
        if (
            tracking_error_m <= self._centering_tolerance_m
            and not reached_target_altitude
        ):
            vertical_velocity = -abs(descent_rate_mps)

        return TrackingVelocityCommand(
            tracking_error_m=tracking_error_m,
            velocity_east_mps=velocity_east,
            velocity_north_mps=velocity_north,
            vertical_velocity_mps=vertical_velocity,
            reached_target_altitude=reached_target_altitude,
        )

    def _has_recent_target_detection(self, context: MissionContext) -> bool:
        if self._time_is_unhealthy(context):
            return False  # an unhealthy time base is "no target" (flag on only)
        if self._estimator_centring is not None:
            # centre_on_estimator: the target is the estimator's track, not the raw detection.
            return self._estimator_solution(context) is not None
        detection = context.target_detection
        if detection is None:
            return False
        if detection.point.x == -1.0 and detection.point.y == -1.0:
            return False

        detection_time = Time.from_msg(detection.header.stamp)
        return context.seconds_since(detection_time) < self._target_timeout_s

    def _get_tracking_solution(
        self,
        context: MissionContext,
        max_centering_speed_mps: float | None = None,
    ) -> tuple[float, float, float] | None:
        if self._estimator_centring is not None:
            # centre_on_estimator (CEN-01): the estimator's track, aimed for the payload.
            return self._estimator_tracking_solution(context, max_centering_speed_mps)

        if not self._has_recent_target_detection(context):
            return None

        detection = context.target_detection
        if detection is None or context.local_pose is None:
            return None

        image_size = context.image_size
        if image_size is None:
            return None

        if self._frame_camera is not None:
            # use_frame_time_pose: the pose at the frame's capture time (FND-05).
            projection = self._frame_time_projection(context, detection, image_size)
        else:
            orientation = context.local_pose.pose.orientation
            projection = project_normalized_detection_to_ground_offset(
                x_norm=float(detection.point.x),
                y_norm=float(detection.point.y),
                image_width=int(image_size[0]),
                image_height=int(image_size[1]),
                intrinsics=self._camera_intrinsics,
                qx=float(orientation.x),
                qy=float(orientation.y),
                qz=float(orientation.z),
                qw=float(orientation.w),
                altitude_m=float(context.local_pose.pose.position.z),
                camera_yaw_offset_deg=self._camera_yaw_offset_deg,
                min_projection_altitude_m=self._min_projection_altitude_m,
                max_projection_distance_m=self._max_projection_distance_m,
            )
        if projection is None:
            return None

        east_error_m = projection.east_m
        north_error_m = projection.north_m
        if abs(east_error_m) <= self._centering_deadband_m:
            east_error_m = 0.0
        if abs(north_error_m) <= self._centering_deadband_m:
            north_error_m = 0.0

        alpha = max(0.0, min(self._tracking_low_pass_alpha, 1.0))
        if (
            self._filtered_tracking_east_m is None
            or self._filtered_tracking_north_m is None
        ):
            self._filtered_tracking_east_m = east_error_m
            self._filtered_tracking_north_m = north_error_m
        else:
            self._filtered_tracking_east_m = self._filtered_tracking_east_m + (
                alpha * (east_error_m - self._filtered_tracking_east_m)
            )
            self._filtered_tracking_north_m = self._filtered_tracking_north_m + (
                alpha * (north_error_m - self._filtered_tracking_north_m)
            )

        filtered_error_m = math.hypot(
            self._filtered_tracking_east_m,
            self._filtered_tracking_north_m,
        )
        max_speed = self._max_centering_speed_mps
        if max_centering_speed_mps is not None:
            max_speed = max(0.0, float(max_centering_speed_mps))

        velocity_east, velocity_north = ground_offset_to_velocity(
            east_error_m=self._filtered_tracking_east_m,
            north_error_m=self._filtered_tracking_north_m,
            gain_mps_per_m=self._centering_gain_mps_per_m,
            max_speed_mps=max_speed,
        )
        return filtered_error_m, velocity_east, velocity_north

    def _estimator_solution(self, context: MissionContext):
        """centre_on_estimator only: this tick's `CentringSolution`, or None when there is no track to steer on.

        The two 2026 limits on a usable target apply as before: none below
        min_projection_altitude_m, and none further from the vehicle than
        max_projection_distance_m.
        """
        centring = self._estimator_centring
        if context.local_pose is None:
            return None
        if float(context.local_pose.pose.position.z) < self._min_projection_altitude_m:
            return None
        near = context.latlon_to_local_enu(self._target_latitude, self._target_longitude)
        solution = centring.solve(context, near)
        if solution is not None and solution.error_m > self._max_projection_distance_m:
            return None
        if solution is None and centring.track_id is not None and centring.last_refusal:
            last = self._last_centring_refusal_log
            if last is None or context.seconds_since(last) >= 2.0:
                self._last_centring_refusal_log = context.now()
                context.logger.warn(f"[{self.name}] no target to centre on: {centring.last_refusal}")
        return solution

    def _estimator_tracking_solution(
        self,
        context: MissionContext,
        max_centering_speed_mps: float | None = None,
    ) -> tuple[float, float, float] | None:
        """centre_on_estimator only: (error, east velocity, north velocity) from the estimator's track.

        The error is from the vehicle to where it must be for the payload to
        be over the track. The estimator has filtered it, so the 2026
        low-pass filter and deadband are not applied.

        On a track the context's descend rule says no to (COASTING, or not
        hit for 0.5 s) the steering goes on, toward where the estimator
        expects the target, but the error is reported as infinite: the
        mission does not descend, does not count itself centred, and so does
        not start its fixed column or its release on a point it only
        remembers. No track that can be steered on is no solution, which the
        callers treat as a lost target.
        """
        solution = None
        if not self._time_is_unhealthy(context):
            solution = self._estimator_solution(context)
        if solution is None:
            if self._centring_controller is not None:
                # The mission holds still on a lost target: what was being
                # commanded is no guide to the next command.
                self._centring_controller.reset()
            return None
        if self._centring_controller is not None:
            # centring_retune (CEN-02).
            velocity_east, velocity_north = self._retuned_velocity(
                context, solution, max_centering_speed_mps
            )
        else:
            max_speed = self._max_centering_speed_mps
            if max_centering_speed_mps is not None:
                max_speed = max(0.0, float(max_centering_speed_mps))
            velocity_east, velocity_north = ground_offset_to_velocity(
                east_error_m=solution.east_m,
                north_error_m=solution.north_m,
                gain_mps_per_m=self._centering_gain_mps_per_m,
                max_speed_mps=max_speed,
            )
        if not solution.may_descend:
            last = self._last_centring_log
            if last is None or context.seconds_since(last) >= 2.0:
                self._last_centring_log = context.now()
                context.logger.warn(
                    f"[{self.name}] not descending: {solution.why_not or 'the descend rule says no'}"
                )
            return math.inf, velocity_east, velocity_north
        return solution.error_m, velocity_east, velocity_north

    def _retuned_velocity(
        self,
        context: MissionContext,
        solution,
        max_centering_speed_mps: float | None,
    ) -> tuple[float, float]:
        """centring_retune only: this tick's command from the retuned controller.

        The speed cap follows the height above the ground, from
        `context.height_agl()`. A rangefinder's height is used as it is; a
        barometer's, which is above home and not above the ground, is taken
        as BARO_HEIGHT_MARGIN_M lower; with no height the cap is the lowest.
        A caller's own speed limit applies as well. If the vehicle's
        velocity is not there for the controller's lead, the log says so.
        """
        height = context.height_agl()
        cap_height_m = None
        if height is not None:
            cap_height_m = height.value_m
            if height.source != "rangefinder":
                cap_height_m = max(0.0, cap_height_m - BARO_HEIGHT_MARGIN_M)
        command = self._centring_controller.step(
            solution.east_m,
            solution.north_m,
            cap_height_m,
            context.now().nanoseconds * 1.0e-9,
            speed_limit_mps=max_centering_speed_mps,
            velocity_en_mps=context.vehicle_velocity(),
        )
        if not self._centring_controller.lead_applied:
            last = self._last_centring_lead_log
            if last is None or context.seconds_since(last) >= 2.0:
                self._last_centring_lead_log = context.now()
                context.logger.warn(
                    f"[{self.name}] centring without its lead: no velocity from the pose history"
                )
        return command

    def _frame_time_projection(
        self,
        context: MissionContext,
        detection,
        image_size,
    ) -> GroundProjection | None:
        """Projection v2 (FND-05), flag on only: where the target is from where the vehicle is now.

        The target's point on the ground is found with the pose the vehicle
        had when the frame was taken, the mode's own calibration and the
        camera's mount; the error to steer on is that point minus the
        vehicle's position now. The two 2026 limits apply as before. With
        no pose for the frame's time there is no projection, which the
        callers treat as no tracking solution.
        """
        position = context.local_pose.pose.position
        if float(position.z) < self._min_projection_altitude_m:
            return None
        frame_time = Time.from_msg(detection.header.stamp)
        frame_age_s = context.seconds_since(frame_time)
        if frame_age_s > self._frame_time_max_age_s:
            # The point is fixed on the ground, so the error to it shrinks as
            # the vehicle moves: without this the mission could centre and
            # descend on a detector that has stopped.
            self._log_frame_time_refusal(
                context,
                f"the frame is {frame_age_s * 1000.0:.0f} ms old (limit {self._frame_time_max_age_s * 1000.0:.0f} ms)",
            )
            return None
        if self._target_offset is None:
            from drone_mission_core.frame_projection import TargetOffset

            projector = context.ground_projector(*self._frame_camera)
            if projector is None:
                self._log_frame_time_refusal(context, "the executor keeps no pose history")
                return None
            self._target_offset = TargetOffset(projector)
        offset = self._target_offset.offset(
            float(detection.point.x),
            float(detection.point.y),
            image_size,
            frame_time.nanoseconds,
            (float(position.x), float(position.y)),
        )
        if offset is None:
            self._log_frame_time_refusal(context, self._target_offset.last_refusal)
            return None
        east_m, north_m = offset
        horizontal_error_m = math.hypot(east_m, north_m)
        if horizontal_error_m > self._max_projection_distance_m:
            return None
        return GroundProjection(
            east_m=east_m,
            north_m=north_m,
            horizontal_error_m=horizontal_error_m,
        )

    def _log_frame_time_refusal(self, context: MissionContext, reason) -> None:
        """Say why projection v2 gave nothing, at most every two seconds."""
        last = self._last_frame_time_refusal_log
        if last is not None and context.seconds_since(last) < 2.0:
            return
        self._last_frame_time_refusal_log = context.now()
        context.logger.warn(
            f"[{self.name}] no ground projection for this detection ({reason}): no tracking solution"
        )

    def _target_loss_grace_expired(self, context: MissionContext) -> bool:
        if self._smart_reacquire is not None and self._smart_reacquire.track_is_lost(context):
            # smart_reacquire: the estimator has called the track LOST after
            # its own two seconds, and that is the grace.
            return True
        if self._target_loss_start is None:
            self._target_loss_start = context.now()
            context.logger.warn(
                f"[{self.name}] Target tracking lost; holding for "
                f"{self._target_loss_grace_s:.1f} s before recovery"
            )
            return False
        return context.seconds_since(self._target_loss_start) >= self._target_loss_grace_s

    def _clear_target_loss_grace(self) -> None:
        self._target_loss_start = None

    def _hold_tracking_loss_grace(self, context: MissionContext) -> bool:
        self._reset_tracking_filter()
        context.set_local_velocity_setpoint(
            0.0,
            0.0,
            0.0,
            yaw_deg=HOLD_YAW_DEG,
        )
        self._log_velocity_command(
            context,
            east_mps=0.0,
            north_mps=0.0,
            up_mps=0.0,
        )
        return self._target_loss_grace_expired(context)

    def _update_target_recovery(self, context: MissionContext) -> RecoveryCommandResult:
        if context.global_gps is None or context.local_pose is None:
            return RecoveryCommandResult(failed=False, reached_altitude=False)

        if self._time_is_unhealthy(context) and self._time_guard.hold_expired(context):
            # Flag on only. Climbing to look again does not mend a time base,
            # and with a dead pose the climb would never be seen to finish.
            context.logger.error(
                f"[{self.name}] time base UNHEALTHY for "
                f"{self._time_guard_hold_timeout_s:.1f} s; giving up the target recovery"
            )
            return RecoveryCommandResult(failed=True, reached_altitude=False)

        if self._smart_reacquire is not None:
            # smart_reacquire (CEN-06): None when there is no estimate to look
            # round, and then the 2026 recovery below is flown.
            smart = self._smart_reacquire.update(context)
            if smart is not None:
                return smart

        if self._recovery_target_altitude is None or self._recovery_hold_position is None:
            if self._recovery_attempts >= self._max_recovery_attempts:
                context.logger.error(
                    f"[{self.name}] Target recovery exceeded "
                    f"{self._max_recovery_attempts} attempts"
                )
                return RecoveryCommandResult(failed=True, reached_altitude=False)
            if self._estimator_centring is not None:
                # centre_on_estimator: the mission is looking again, so the track
                # it was flying to is given up and the next one found is taken.
                self._estimator_centring.forget()

            current_altitude = context.local_pose.pose.position.z
            next_altitude = current_altitude + self._not_found_ascent_m
            if (
                current_altitude >= self._max_recovery_altitude_m
                or next_altitude > self._max_recovery_altitude_m
            ):
                context.logger.error(
                    f"[{self.name}] Recovery climb would exceed "
                    f"{self._max_recovery_altitude_m:.1f} m"
                )
                return RecoveryCommandResult(failed=True, reached_altitude=False)

            self._recovery_target_altitude = min(
                next_altitude,
                self._max_recovery_altitude_m,
            )
            self._recovery_hold_position = (
                context.global_gps.latitude,
                context.global_gps.longitude,
            )
            self._recovery_attempts += 1
            context.logger.info(
                f"[{self.name}] Target lost, climbing to "
                f"{self._recovery_target_altitude:.1f} m "
                f"(attempt {self._recovery_attempts}/{self._max_recovery_attempts})"
            )

        context.set_global_position_setpoint(
            self._recovery_hold_position[0],
            self._recovery_hold_position[1],
            self._recovery_target_altitude,
            yaw_deg=HOLD_YAW_DEG,
            lock_yaw=True,
        )
        altitude_error = abs(
            context.local_pose.pose.position.z - self._recovery_target_altitude
        )
        return RecoveryCommandResult(
            failed=False,
            reached_altitude=altitude_error <= self._recovery_alt_tolerance_m,
        )

    def _reset_tracking_filter(self) -> None:
        self._filtered_tracking_east_m = None
        self._filtered_tracking_north_m = None

    def _log_velocity_command(
        self,
        context: MissionContext,
        *,
        east_mps: float,
        north_mps: float,
        up_mps: float,
    ) -> None:
        now = context.now()
        if self._last_velocity_log_time is not None:
            elapsed_s = (now - self._last_velocity_log_time).nanoseconds / 1e9
            if elapsed_s < 1.0:
                return

        context.logger.info(
            f"[{self.name}] Velocity command: "
            f"east={east_mps:.3f} m/s, "
            f"north={north_mps:.3f} m/s, "
            f"up={up_mps:.3f} m/s"
        )
        self._last_velocity_log_time = now

    def _hold_current_position(
        self,
        context: MissionContext,
        altitude_m: float | None = None,
    ) -> None:
        if context.global_gps is None or context.local_pose is None:
            return

        hold_altitude = (
            context.local_pose.pose.position.z
            if altitude_m is None
            else float(altitude_m)
        )
        context.set_global_position_setpoint(
            context.global_gps.latitude,
            context.global_gps.longitude,
            hold_altitude,
            yaw_deg=HOLD_YAW_DEG,
            lock_yaw=True,
        )

    def _request_target_cv_enable(self, context: MissionContext) -> None:
        if self._target_cv_enabled or self._target_cv_enable_requested:
            return
        if not context.target_cv_control_ready():
            return

        context.logger.info(f"[{self.name}] Enabling target detection")
        context.set_target_cv_enabled(True, self._on_target_cv_enable_response)
        self._target_cv_enable_requested = True

    def _on_target_cv_enable_response(self, future) -> None:
        self._target_cv_enable_requested = False
        try:
            result = future.result()
            self._target_cv_enabled = bool(result is not None and result.success)
        except Exception:
            self._target_cv_enabled = False
