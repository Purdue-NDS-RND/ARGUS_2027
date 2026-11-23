"""Package delivery mission implementation."""

from __future__ import annotations

import math
from enum import Enum, auto

from drone_mission_core.features import get_feature_flag
from drone_mission_core.mission_api import MissionStatus
from drone_mission_core.mission_context import MissionContext
from drone_mission_core.registry import register_mission
from drone_mission_core.safety_limits import set_down_release_allowed
from drone_mission_core.set_down import AFTER_RELEASE_WAIT_S, BARO_MARGIN_M, SET_DOWN_FLAG
from rclpy.time import Time
from .red_bullseye_mission_base import (
    HOLD_YAW_DEG,
    TRANSIT_YAW_DEG,
    RedBullseyeMissionBase,
    haversine_distance,
)

FINAL_DESCENT_BUFFER_M = 1.0


class PackageDeliveryState(Enum):
    INIT = auto()
    WAITING_FOR_CONNECTION = auto()
    WAITING_FOR_GPS = auto()
    TRANSIT_TO_TARGET = auto()
    ACQUIRE_TARGET = auto()
    TARGET_NOT_FOUND = auto()
    TRACK_AND_DESCEND = auto()
    FINAL_FIXED_COLUMN_DESCENT = auto()
    GROUND_DWELL = auto()
    GUIDED_RELAUNCH = auto()
    COMPLETE = auto()
    FINAL_SET_DOWN = auto()  # 2027, delivery_soft_set_down only: in place of FINAL_FIXED_COLUMN_DESCENT (delivery_set_down.py)


@register_mission("package_delivery")
class PackageDeliveryMission(RedBullseyeMissionBase):
    """Fly to a GPS target, land on it in GUIDED, deliver, and relaunch."""

    def __init__(self, spec) -> None:
        super().__init__(spec)
        # The soft set-down's one measurement (CEN-05), only with its flag
        # on. Checked when the sequence is built, as the camera's calibration
        # is, so that a sequence without it stops the executor on the ground
        # and not the delivery in the air with the package on the hook.
        if get_feature_flag(spec.features, SET_DOWN_FLAG):
            from drone_mission_core.set_down import ContactDetector

            try:
                ContactDetector(spec.config.get("package_height_m"))
            except ValueError as error:
                raise ValueError(
                    f"mission '{spec.name}': {SET_DOWN_FLAG} is on but the mission cannot set down: {error}"
                ) from error

    def on_enter(self, context: MissionContext) -> None:
        config = self.spec.config
        self._load_common_vision_config(config)
        self._landing_check_threshold_m = float(
            config.get(
                "landing_check_threshold_m",
                config.get("touchdown_altitude_m", 0.35),
            )
        )
        self._touchdown_handoff_tolerance_m = float(
            config.get("touchdown_handoff_tolerance_m", 0.2)
        )
        self._relaunch_altitude_m = float(
            config.get("relaunch_altitude_m", self._transit_altitude_m)
        )
        self._descent_rate_mps = float(config.get("descent_rate_mps", 0.5))
        self._final_descent_rate_mps = float(
            config.get("final_descent_rate_mps", 0.2)
        )
        self._touchdown_min_altitude_m = float(
            config.get("touchdown_min_altitude_m", -2.0)
        )
        self._delivery_dwell_s = float(config.get("delivery_dwell_s", 5.0))
        self._guided_relaunch_rate_mps = float(
            config.get("guided_relaunch_rate_mps", 0.6)
        )
        self._guided_relaunch_max_climb_rate_mps = float(
            config.get("guided_relaunch_max_climb_rate_mps", 2.5)
        )
        self._touchdown_dwell_s = float(config.get("touchdown_dwell_s", 0.5))

        self._state = PackageDeliveryState.INIT
        self._initialize_common_vision_state()
        self._delivery_complete = False
        self._touchdown_debounce_start: Time | None = None
        self._ground_dwell_start: Time | None = None
        self._final_descent_hold_position: tuple[float, float] | None = None
        self._final_descent_target_altitude_m: float | None = None
        self._final_descent_last_update: Time | None = None
        # The soft set-down (CEN-05), only with the delivery_soft_set_down flag on.
        self._set_down = None
        self._ground_below_home_m = 0.0
        self._released_at: Time | None = None
        if get_feature_flag(self.spec.features, SET_DOWN_FLAG):
            from .delivery_set_down import SoftSetDown

            self._ground_below_home_m = float(config.get("ground_below_home_m", 0.0))
            self._set_down = SoftSetDown(self, config.get("package_height_m"))

        self._enter_common_mission(context)

    def on_exit(self, context: MissionContext) -> None:
        self._exit_common_mission(context)

    def update(self, context: MissionContext) -> MissionStatus:
        if self._failed:
            return MissionStatus.FAILURE

        handler = {
            PackageDeliveryState.INIT: self._handle_init,
            PackageDeliveryState.WAITING_FOR_CONNECTION: self._handle_waiting_for_connection,
            PackageDeliveryState.WAITING_FOR_GPS: self._handle_waiting_for_gps,
            PackageDeliveryState.TRANSIT_TO_TARGET: self._handle_transit_to_target,
            PackageDeliveryState.ACQUIRE_TARGET: self._handle_acquire_target,
            PackageDeliveryState.TARGET_NOT_FOUND: self._handle_target_not_found,
            PackageDeliveryState.TRACK_AND_DESCEND: self._handle_track_and_descend,
            PackageDeliveryState.FINAL_FIXED_COLUMN_DESCENT: self._handle_final_fixed_column_descent,
            PackageDeliveryState.GROUND_DWELL: self._handle_ground_dwell,
            PackageDeliveryState.GUIDED_RELAUNCH: self._handle_guided_relaunch,
            PackageDeliveryState.COMPLETE: self._handle_complete,
            PackageDeliveryState.FINAL_SET_DOWN: self._handle_final_set_down,
        }.get(self._state, self._handle_invalid_state)
        return handler(context)

    def _handle_init(self, context: MissionContext) -> MissionStatus:
        if not self._target_cv_enabled:
            self._request_target_cv_enable(context)
            return MissionStatus.WAITING
        self._transition_to(PackageDeliveryState.WAITING_FOR_CONNECTION, context)
        return MissionStatus.RUNNING

    def _handle_waiting_for_connection(self, context: MissionContext) -> MissionStatus:
        if context.mavros_state is None or not context.mavros_state.connected:
            return MissionStatus.WAITING
        self._transition_to(PackageDeliveryState.WAITING_FOR_GPS, context)
        return MissionStatus.RUNNING

    def _handle_waiting_for_gps(self, context: MissionContext) -> MissionStatus:
        if context.global_gps is None or context.global_gps.status.status < 0:
            return MissionStatus.WAITING
        self._transition_to(PackageDeliveryState.TRANSIT_TO_TARGET, context)
        return MissionStatus.RUNNING

    def _handle_transit_to_target(self, context: MissionContext) -> MissionStatus:
        context.set_global_position_setpoint(
            self._target_latitude,
            self._target_longitude,
            self._transit_altitude_m,
            yaw_deg=TRANSIT_YAW_DEG,
            lock_yaw=False,
        )

        if context.global_gps is None or context.local_pose is None:
            return MissionStatus.WAITING
        
        context.logger.info(f"[{self.name}] En route to GPS delivery zone at " f"({self._target_latitude:.7f}, {self._target_longitude:.7f})")

        ground_distance = haversine_distance(
            context.global_gps.latitude,
            context.global_gps.longitude,
            self._target_latitude,
            self._target_longitude,
        )
        altitude_error = abs(
            context.local_pose.pose.position.z - self._transit_altitude_m
        )
        if (
            ground_distance <= self._arrival_radius_m
            and altitude_error <= self._arrival_alt_tolerance_m
        ):
            context.logger.info(f"[{self.name}] Arrived at GPS delivery zone")
            self._transition_to(PackageDeliveryState.ACQUIRE_TARGET, context)
        return MissionStatus.RUNNING

    def _handle_acquire_target(self, context: MissionContext) -> MissionStatus:
        if context.global_gps is None or context.local_pose is None:
            return MissionStatus.WAITING

        self._hold_current_position(context)

        if not self._has_recent_target_detection(context):
            if self._target_loss_grace_expired(context):
                self._transition_to(PackageDeliveryState.TARGET_NOT_FOUND, context)
            return MissionStatus.RUNNING

        self._clear_target_loss_grace()
        if self._centering_dwell_start is None:
            self._centering_dwell_start = context.now()
            context.logger.info(
                f"[{self.name}] Target acquired, holding lock for "
                f"{self._centering_dwell_s:.1f} s"
            )
            return MissionStatus.RUNNING

        if context.seconds_since(self._centering_dwell_start) < self._centering_dwell_s:
            return MissionStatus.RUNNING

        self._transition_to(PackageDeliveryState.TRACK_AND_DESCEND, context)
        return MissionStatus.RUNNING

    def _handle_target_not_found(self, context: MissionContext) -> MissionStatus:
        if context.global_gps is None or context.local_pose is None:
            return MissionStatus.WAITING

        recovery = self._update_target_recovery(context)
        if recovery.failed:
            return MissionStatus.FAILURE
        if recovery.reached_altitude:
            self._transition_to(PackageDeliveryState.ACQUIRE_TARGET, context)
        return MissionStatus.RUNNING

    def _handle_track_and_descend(self, context: MissionContext) -> MissionStatus:
        if context.local_pose is None:
            return MissionStatus.WAITING

        descent_rate = self._descent_rate_mps
        if (
            context.local_pose.pose.position.z
            <= self._landing_check_threshold_m + FINAL_DESCENT_BUFFER_M
        ):
            descent_rate = self._final_descent_rate_mps

        command = self._get_centering_descent_command(
            context,
            target_altitude_m=self._landing_check_threshold_m,
            descent_rate_mps=descent_rate,
            target_altitude_tolerance_m=self._touchdown_handoff_tolerance_m,
        )
        if command is None:
            if self._hold_tracking_loss_grace(context):
                self._transition_to(PackageDeliveryState.TARGET_NOT_FOUND, context)
            return MissionStatus.RUNNING

        self._clear_target_loss_grace()
        if command.tracking_error_m <= self._centering_tolerance_m:
            # delivery_soft_set_down: also from under the handoff window, where
            # the mission is after looking for the target again from low down.
            under_the_handoff = (
                self._set_down is not None
                and context.local_pose.pose.position.z
                <= self._landing_check_threshold_m + self._touchdown_handoff_tolerance_m
            )
            if command.reached_target_altitude or under_the_handoff:
                if not context.landing_state_available():
                    context.logger.error(
                        f"[{self.name}] /mavros/extended_state is required before "
                        "final touchdown descent"
                    )
                    return MissionStatus.FAILURE
                if self._set_down is not None:
                    # FINAL_SET_DOWN goes on from here on the estimator,
                    # where 2026 froze its GPS column.
                    context.logger.info(
                        f"[{self.name}] Entering final set-down at "
                        f"{context.local_pose.pose.position.z:.2f} m "
                        f"(threshold {self._landing_check_threshold_m:.2f} m)"
                    )
                    self._set_down.enter(context)
                    self._transition_to(PackageDeliveryState.FINAL_SET_DOWN, context)
                    return MissionStatus.RUNNING
                context.logger.info(
                    f"[{self.name}] Entering final touchdown column at "
                    f"{context.local_pose.pose.position.z:.2f} m "
                    f"(threshold {self._landing_check_threshold_m:.2f} m, "
                    f"handoff tolerance {self._touchdown_handoff_tolerance_m:.2f} m)"
                )
                self._transition_to(
                    PackageDeliveryState.FINAL_FIXED_COLUMN_DESCENT,
                    context,
                )
                return MissionStatus.RUNNING

        context.set_local_velocity_setpoint(
            command.velocity_east_mps,
            command.velocity_north_mps,
            command.vertical_velocity_mps,
            yaw_deg=HOLD_YAW_DEG,
        )
        self._log_velocity_command(
            context,
            east_mps=command.velocity_east_mps,
            north_mps=command.velocity_north_mps,
            up_mps=command.vertical_velocity_mps,
        )
        return MissionStatus.RUNNING

    def _handle_final_fixed_column_descent(
        self,
        context: MissionContext,
    ) -> MissionStatus:
        if context.local_pose is None or context.global_gps is None:
            return MissionStatus.WAITING
        if not context.landing_state_available():
            context.logger.error(
                f"[{self.name}] Lost /mavros/extended_state during touchdown descent"
            )
            return MissionStatus.FAILURE

        if context.vehicle_is_landed():
            context.set_local_velocity_setpoint(
                0.0,
                0.0,
                0.0,
                yaw_deg=HOLD_YAW_DEG,
            )
            if self._touchdown_debounce_start is None:
                self._touchdown_debounce_start = context.now()
                context.logger.info(
                    f"[{self.name}] Ground contact detected, debouncing for "
                    f"{self._touchdown_dwell_s:.1f} s"
                )
                return MissionStatus.RUNNING

            if context.seconds_since(self._touchdown_debounce_start) < self._touchdown_dwell_s:
                return MissionStatus.RUNNING

            context.logger.info(f"[{self.name}] Touchdown confirmed by FCU landed state")
            self._transition_to(PackageDeliveryState.GROUND_DWELL, context)
            return MissionStatus.RUNNING

        self._touchdown_debounce_start = None
        if self._time_is_unhealthy(context):
            # Flag on only: no descent on an unhealthy time base. The vehicle
            # holds its height, and the column carries on when it clears.
            self._final_descent_last_update = None
            return self._hold_for_time_guard(
                context,
                self._final_descent_hold_position,
            )
        if self._final_descent_hold_position is None:
            self._final_descent_hold_position = (
                context.global_gps.latitude,
                context.global_gps.longitude,
            )
            self._final_descent_target_altitude_m = context.local_pose.pose.position.z
            self._final_descent_last_update = context.now()
            context.logger.info(
                f"[{self.name}] Freezing touchdown column at "
                f"({self._final_descent_hold_position[0]:.7f}, "
                f"{self._final_descent_hold_position[1]:.7f})"
            )

        if self._final_descent_target_altitude_m is None:
            self._final_descent_target_altitude_m = context.local_pose.pose.position.z

        now = context.now()
        elapsed_s = 0.0
        if self._final_descent_last_update is not None:
            elapsed_s = max(
                0.0,
                min((now - self._final_descent_last_update).nanoseconds / 1e9, 0.5),
            )
        self._final_descent_last_update = now

        current_altitude = context.local_pose.pose.position.z
        max_descent_step = abs(self._final_descent_rate_mps) * elapsed_s
        self._final_descent_target_altitude_m = min(
            self._final_descent_target_altitude_m,
            current_altitude,
        )
        self._final_descent_target_altitude_m = max(
            self._touchdown_min_altitude_m,
            self._final_descent_target_altitude_m - max_descent_step,
        )

        context.set_global_position_setpoint(
            self._final_descent_hold_position[0],
            self._final_descent_hold_position[1],
            self._final_descent_target_altitude_m,
            yaw_deg=HOLD_YAW_DEG,
            lock_yaw=True,
        )
        return MissionStatus.RUNNING

    def _handle_final_set_down(self, context: MissionContext) -> MissionStatus:
        """delivery_soft_set_down only (CEN-05): delivery_set_down.py does the work, this turns its answer into a state."""
        from . import delivery_set_down

        if context.local_pose is None or context.global_gps is None:
            return MissionStatus.WAITING
        if self._set_down.holding and self._time_is_unhealthy(context):
            # As in the 2026 column: no descent on an unhealthy time base.
            self._set_down.pause()
            return self._hold_for_time_guard(context, self._set_down.hold_point)
        answer = self._set_down.update(context)
        if answer == delivery_set_down.CONTACT:
            self._transition_to(PackageDeliveryState.GROUND_DWELL, context)
        elif answer == delivery_set_down.REACQUIRE:
            self._transition_to(PackageDeliveryState.TARGET_NOT_FOUND, context)
        elif answer == delivery_set_down.GIVE_UP:
            self._failed = True
            return MissionStatus.FAILURE
        return MissionStatus.RUNNING

    def _handle_soft_release(self, context: MissionContext) -> MissionStatus:
        """delivery_soft_set_down only: release on contact, wait a second, climb. Not the 2026 dwell, and no wait for "landed"."""
        if context.mavros_state is not None and not context.mavros_state.armed:
            context.logger.error(
                f"[{self.name}] Vehicle disarmed on the ground before relaunch. "
                "Increase FCU DISARM_DELAY."
            )
            return MissionStatus.FAILURE
        context.set_local_velocity_setpoint(0.0, 0.0, 0.0, yaw_deg=HOLD_YAW_DEG)
        release_allowed = False  # the set-down rule's word on this tick, for the actuation lock (PLT-09)
        if not self._delivery_complete and not self._actuator_requested:
            # The rules' ceiling for a set-down, at the moment of the release:
            # at or under 6 ft by the height as it is now, or landed by the
            # flight controller's word. A height that is not the
            # rangefinder's counts as BARO_MARGIN_M higher than it says.
            height = context.height_agl(self._ground_below_home_m)
            landed = context.landing_state_available() and context.vehicle_is_landed()
            against_ceiling_m = math.nan
            if height is not None:
                against_ceiling_m = height.value_m + (0.0 if height.source == "rangefinder" else BARO_MARGIN_M)
            if not set_down_release_allowed(against_ceiling_m, landed):
                context.logger.warn(
                    f"[{self.name}] not releasing: the height is "
                    f"{'unknown' if height is None else f'{height.value_m:.2f} m'}, "
                    "the flight controller does not say landed"
                )
                self._set_down.contact_again()
                self._transition_to(PackageDeliveryState.FINAL_SET_DOWN, context)
                return MissionStatus.RUNNING
            release_allowed = True
        if self._ground_dwell_start is None:
            self._ground_dwell_start = context.now()
            if self._fake_drop:
                context.logger.info(f"[{self.name}] Fake delivery complete")
                self._delivery_complete = True
        if not self._delivery_complete:
            if self._actuator_requested:
                return MissionStatus.RUNNING
            if not context.command_service_ready():
                return MissionStatus.WAITING
            context.logger.info(f"[{self.name}] Releasing gripper payload")
            self._actuator_requested = True
            # A no-op without the lock: this is the release state, entered on contact, and the height is under the ceiling or the vehicle landed.
            context.permit_actuation("gripper", state=self._state.name, gates_hold=release_allowed, why="the set-down rule did not allow a release on this tick")
            context.command_gripper(
                release=True,
                done_callback=self._on_gripper_response,
            )
            return MissionStatus.RUNNING
        if self._released_at is None:
            self._released_at = context.now()
        if context.seconds_since(self._released_at) < AFTER_RELEASE_WAIT_S:
            return MissionStatus.RUNNING
        self._transition_to(PackageDeliveryState.GUIDED_RELAUNCH, context)
        return MissionStatus.RUNNING

    def _handle_ground_dwell(self, context: MissionContext) -> MissionStatus:
        if self._set_down is not None:
            return self._handle_soft_release(context)
        if context.mavros_state is not None and not context.mavros_state.armed:
            context.logger.error(
                f"[{self.name}] Vehicle disarmed on the ground before relaunch. "
                "Increase FCU DISARM_DELAY or reduce delivery_dwell_s."
            )
            return MissionStatus.FAILURE
        if not context.landing_state_available():
            context.logger.error(
                f"[{self.name}] Lost /mavros/extended_state during ground dwell"
            )
            return MissionStatus.FAILURE
        if not context.vehicle_is_landed():
            self._transition_to(
                PackageDeliveryState.FINAL_FIXED_COLUMN_DESCENT,
                context,
            )
            return MissionStatus.RUNNING

        context.set_local_velocity_setpoint(
            0.0,
            0.0,
            0.0,
            yaw_deg=HOLD_YAW_DEG,
        )

        if self._ground_dwell_start is None:
            self._ground_dwell_start = context.now()
            context.logger.info(
                f"[{self.name}] Holding on target for "
                f"{self._delivery_dwell_s:.1f} s before relaunch"
            )
            if self._fake_drop:
                context.logger.info(f"[{self.name}] Fake delivery complete")
                self._delivery_complete = True

        if not self._delivery_complete and not self._fake_drop:
            if self._actuator_requested:
                pass
            elif not context.command_service_ready():
                return MissionStatus.WAITING
            else:
                context.logger.info(f"[{self.name}] Releasing gripper payload")
                context.command_gripper(
                    release=True,
                    done_callback=self._on_gripper_response,
                )
                self._actuator_requested = True

        if context.seconds_since(self._ground_dwell_start) < self._delivery_dwell_s:
            return MissionStatus.RUNNING
        if not self._delivery_complete:
            return MissionStatus.RUNNING

        self._transition_to(PackageDeliveryState.GUIDED_RELAUNCH, context)
        return MissionStatus.RUNNING

    def _handle_guided_relaunch(self, context: MissionContext) -> MissionStatus:
        if context.local_pose is None:
            return MissionStatus.WAITING
        if context.mavros_state is not None and not context.mavros_state.armed:
            context.logger.error(
                f"[{self.name}] Vehicle disarmed before guided relaunch. "
                "Touch-and-go requires staying armed on the ground."
            )
            return MissionStatus.FAILURE

        current_alt = context.local_pose.pose.position.z
        if current_alt >= self._relaunch_altitude_m * 0.9:
            context.clear_all_setpoints()
            self._transition_to(PackageDeliveryState.COMPLETE, context)
            return MissionStatus.SUCCESS

        context.set_attitude_climb_rate_setpoint(
            abs(self._guided_relaunch_rate_mps),
            yaw_deg=HOLD_YAW_DEG,
            max_climb_rate_mps=self._guided_relaunch_max_climb_rate_mps,
        )
        return MissionStatus.RUNNING

    def _handle_complete(self, _context: MissionContext) -> MissionStatus:
        return MissionStatus.SUCCESS

    def _handle_invalid_state(self, _context: MissionContext) -> MissionStatus:
        return MissionStatus.FAILURE

    def _on_gripper_response(self, future) -> None:
        self._actuator_requested = False
        try:
            result = future.result()
            if result is not None and result.success:
                self._delivery_complete = True
        except Exception:
            self._delivery_complete = False

    def _transition_to(
        self,
        new_state: PackageDeliveryState,
        context: MissionContext | None = None,
    ) -> None:
        if new_state == self._state:
            return

        self._centering_dwell_start = None
        self._target_loss_start = None
        self._last_velocity_log_time = None
        self._reset_tracking_filter()
        if new_state != PackageDeliveryState.TARGET_NOT_FOUND:
            self._recovery_target_altitude = None
            self._recovery_hold_position = None
        if new_state != PackageDeliveryState.FINAL_FIXED_COLUMN_DESCENT:
            self._touchdown_debounce_start = None
            self._final_descent_hold_position = None
            self._final_descent_target_altitude_m = None
            self._final_descent_last_update = None
        if new_state != PackageDeliveryState.GROUND_DWELL:
            self._ground_dwell_start = None

        if context is not None:
            context.logger.info(
                f"[{self.name}] State: {self._state.name} -> {new_state.name}"
            )
        self._state = new_state
