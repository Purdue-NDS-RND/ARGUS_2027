"""The bucket recovery (REC-2): fly to the pickup's coordinates, hook the bucket, bring it back, set it down.

Behind the mission flag `package_recovery` (default false; needs
`use_target_estimator` and `use_height_estimator`). With the flag off the
mission is skipped: it returns SUCCESS on its first tick and commands
nothing.

With it on, the flight is drone_mission_core/recovery_flight.py's, state by
state (CHECKS, TRANSIT, ACQUIRE, SEARCH, APPROACH, CENTRE, CONE_DESCENT,
ENGAGE, LIFT, VERIFY_ATTACH, CLIMB, RETURN, SET_DOWN, CLEAR; ABORT_CLIMB,
LOST_TARGET and CARRYING_FAULT to the side). This file is the thin side:
each tick it reads the mission context into the flight's `Readings` and
hands the flight's `Command` back to the context.

  the vehicle       the local pose (position and heading)
  the height        `height_agl()`: above the ground, and whether it is the
                    rangefinder's
  the throttle      `throttle()` (VFR_HUD), for the attach check, the loss
                    watch and the set-down
  the bucket        the estimator's track of class `bucket`, or of `green_x`
                    when there is none: the nearest to where it was last
  GPS               the newest fix, when it says it has one and is under 2 s
                    old; `home_up_m()` for a global setpoint's height
  the pickup        `pickup: {latlon: [lat, lon]}` placed in the local frame
                    by `latlon_to_local_enu`, or `pickup: {local: [e, n]}`
  the gripper       `command_gripper`, whose answer comes back as the flight's
                    `Ack` with the attempt it was sent for

The abort: each tick the mission tells the executor which mode a failure of
the sequence asks for (`context.set_abort_mode`): RTL while the gripper has
never closed and the vehicle has a position to fly home on, LAND from the
first close for the rest of the mission and whenever there is no pose, no
GPS fix or GPS is not trusted (recovery_return.py, "Abort"). It never gives
a landing back when it leaves.

The mission's other flags: `swing_shaping` (the way back walked over whole
pendulum periods), `ekf_source_switching` (REC-6: the GPS health monitor's
rating is asked every tick; on UNTRUSTED the flight flies as without GPS, and
the flight controller is asked for source set 2 only while the flow odometry
is sending it flow, for unfed that set is dead reckoning; set 1 is made sure
of at the start, and asked for again only when the rating is TRUSTED again
and the mission is on its way out or home)
and `recovery_gps_untrusted` (no global setpoint, ever; the
pickup must then be given as `local`). The flags the spec names whose
behaviour is not built (`recovery_telemetry`, `visual_attach_check`) are
refused when on: the mission is not constructed. `marker_homing` and
`keyframe_homing` (REC-7b) make the mission take the estimator's confirmed
track of the home marker (`home_marker`) or of the takeoff point as the
climb's keyframes show it (`takeoff_point`) as the place to set the bucket
down: after its fixes have agreed for a second, inside the last ten metres,
within 4 m (and the drift allowed for; never more than REC-7's 10 m) of
where the return point was thought to be. The detector node's flags of the
same names are what finds them.

The configuration is checked when the mission is constructed, so numbers
that cannot be flown stop the executor on the ground. The mechanism's and
the aircraft's numbers (`hook_below_m`, `suspension_length_m`, `vehicle_kg`,
`load_kg`) have no defaults.
"""

from __future__ import annotations

import math
from typing import Any, Optional, Tuple

from drone_mission_core.features import get_feature_flag
from drone_mission_core.mission_api import BaseMission, MissionStatus
from drone_mission_core.registry import register_mission

FLAG = "package_recovery"  # recovery_flight.FLAG (a test holds the two together); named here so that the flag-off path imports nothing new
SEARCH_RADIUS_M = 25.0
FIX_MAX_AGE_S = 2.0
POSE_MAX_AGE_S = 0.5
TARGET_CLASSES = ("bucket", "green_x")  # the bucket's own track first: the handle is on it. The X's finds and centres; the cone and the engage go by the bucket's only
CONFIRMED = "CONFIRMED"
# REC-7b: which of the estimator's tracks each homing flag takes for the return point. The marker's own class and the takeoff point's
# are two classes: a mission told to home on a marker never takes the keyframes' takeoff point for it.
HOME_CLASSES = (("marker_homing", "home_marker"), ("keyframe_homing", "takeoff_point"))
HOMING_MAX_SIGMA_M = 0.5  # a track less sure of its place than this is not a place to set a bucket down on
SOURCE_FLAG = "ekf_source_switching"  # gps_health.FLAG (a test holds the two together)
STATIC_TARGET_STATES = ("ACQUIRE", "APPROACH", "CENTRE", "CONE_DESCENT")  # the bucket is tracked and stands on the ground
SOURCE_SET_BACK_STATES = ("TRANSIT", "ACQUIRE", "SEARCH", "RETURN")  # where the mission lets the flight controller go back to the GPS's source set
LOG_EVERY_S = 2.0


def _stamp_s(message: Any) -> Optional[float]:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    return None if stamp is None else float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _yaw_rad(orientation: Any) -> float:
    return math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y), 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))


@register_mission("package_recovery")
class PackageRecoveryMission(BaseMission):
    """Hook the bucket at the pickup's coordinates and set it down at the return point."""

    def __init__(self, spec) -> None:
        super().__init__(spec)
        self._enabled = bool(get_feature_flag(spec.features, FLAG))
        # REC-6: the GPS health monitor's word and the flight controller's source set. Off: a fix is trusted as before, and nothing is commanded.
        self._source_switching = self._enabled and bool(get_feature_flag(spec.features, SOURCE_FLAG))
        self.flight: Any = None
        if not self._enabled:
            return
        from drone_mission_core import coverage_planner, recovery_flight
        from drone_mission_core.frame_projection import camera_from_config

        self._rf = recovery_flight
        config = spec.config or {}
        try:
            self._pickup_latlon, self._pickup_local = self._pickup(config.get("pickup"))
            self._ground_below_home_m = config.get("ground_below_home_m", 0.0)
            if isinstance(self._ground_below_home_m, bool) or not isinstance(self._ground_below_home_m, (int, float)) or not math.isfinite(self._ground_below_home_m):
                raise ValueError(f"ground_below_home_m must be a number of metres, got {self._ground_below_home_m!r}")
            camera, _mount = camera_from_config(config, spec.name)  # raises for a mode or airframe that cannot be loaded
            across, along = coverage_planner.footprint(camera, 1.0)
            search_height = float(config.get("search_height_m", 20.0))
            spacing = coverage_planner.track_spacing(min(across, along) * search_height)
            self.plan = recovery_flight.plan_from_config(
                {key: value for key, value in config.items() if key not in ("pickup", "ground_below_home_m", "camera_mode", "airframe", "flight_boundary")},
                spec.features,
                get_feature_flag,
                recovery_flight.expanding_square(spacing, SEARCH_RADIUS_M),
                min(across, along),
            )
            self._boundary_latlon = None
            if config.get("flight_boundary") is not None:
                corners = config["flight_boundary"]
                if not isinstance(corners, (list, tuple)) or len(corners) < 3:
                    raise ValueError("flight_boundary must be three or more [latitude, longitude] corners")
                self._boundary_latlon = [self._pickup({"latlon": corner})[0] for corner in corners]
            if self.plan.gps_untrusted and self._pickup_local is None:
                raise ValueError(f"with {recovery_flight.UNTRUSTED_FLAG} on the pickup is given in the local frame (pickup: {{local: [east, north]}}): a latitude and longitude cannot be placed without trusting GPS")
        except ValueError as error:
            raise ValueError(f"mission '{spec.name}': {error}") from error

    @staticmethod
    def _pickup(value: Any) -> Tuple[Optional[Tuple[float, float]], Optional[Tuple[float, float]]]:
        def pair(raw: Any, what: str) -> Tuple[float, float]:
            if not isinstance(raw, (list, tuple)) or len(raw) != 2 or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in raw):
                raise ValueError(f"pickup.{what} must be two numbers, got {raw!r}")
            return float(raw[0]), float(raw[1])

        if isinstance(value, dict) and set(value) == {"latlon"}:
            lat, lon = pair(value["latlon"], "latlon")
            if abs(lat) > 90.0 or abs(lon) > 180.0:
                raise ValueError(f"pickup.latlon must be [latitude, longitude] in degrees, got {value['latlon']!r}")
            return (lat, lon), None
        if isinstance(value, dict) and set(value) == {"local"}:
            return None, pair(value["local"], "local")
        raise ValueError(f"pickup is {{latlon: [lat, lon]}} or {{local: [east, north]}}, got {value!r}")

    # --- the mission API -------------------------------------------------------------

    def on_enter(self, context: Any) -> None:
        if not self._enabled:
            return
        self.flight = self._rf.RecoveryFlight(self.plan)
        self._ack: Any = None
        self._yaw_deg: Optional[float] = None
        self._placed: Optional[Tuple[float, float]] = self._pickup_local
        self._near: Optional[Tuple[float, float]] = None
        self._boundary: Any = None  # the flight boundary in the local frame, once its corners can be placed
        self._said_state = ""
        self._said_homed: Optional[Tuple[float, float]] = None
        self._home_classes = tuple(class_id for flag, class_id in HOME_CLASSES if self.plan.homing and get_feature_flag(self.spec.features, flag))
        if self._home_classes and self.plan.return_target.kind == self._rf.back.MARKER:
            # The marker's id is the detector node's to look for: the mission cannot see which id it was told.
            context.logger.warn(f"[{self.name}] the return target is marker {self.plan.return_target.marker}: the recovery detector's home_marker_id must be {self.plan.return_target.marker} (it is not checked from here)")
        self._last_log_s = -1.0e9
        self._abort_mode: Optional[str] = None  # what the executor was last told
        # REC-6 (flag ekf_source_switching): the flight controller's source set, by its acknowledgements. None with the flag off.
        self._source_switch: Any = None
        if self._source_switching:
            from drone_mission_core import gps_health

            self._gps_health = gps_health
            self._source_switch = gps_health.SourceSwitch()
            self._gps_state = ""
            self._static_kind: Optional[bool] = None  # which track the monitor was last told of as standing still
        # A landing an earlier mission of the sequence left named (it failed with something on its hook) is not undone here.
        self._abort_land_inherited = context.named_abort_mode() == self._rf.back.ABORT_LAND
        if self._abort_land_inherited or self.flight.abort_mode() == self._rf.back.ABORT_LAND:
            # Known before the first tick (GPS not trusted): named now, so that a failure on that tick is a landing too.
            context.set_abort_mode(self._rf.back.ABORT_LAND)
            self._abort_mode = self._rf.back.ABORT_LAND
        if self._boundary_latlon is None:
            context.logger.warn(f"[{self.name}] no flight_boundary is given: the pickup and the search are held only to 1000 m of the vehicle and to the flight controller's fence")

    def on_exit(self, context: Any) -> None:
        if self._enabled:
            context.clear_all_setpoints()  # (the abort's mode is left as it stands: a failure's abort comes after this)
            if getattr(self, "_source_switch", None) is not None:
                # What flies after this mission flies on GPS without asking the monitor: the flight controller is not left off set 1 unsaid.
                source_set, say = self._source_switch.leave(self._gps_state, context.now().nanoseconds * 1.0e-9)
                if say:
                    context.logger.warn(f"[{self.name}] {say}")
                if source_set is not None:
                    asked = int(source_set)
                    context.command_ekf_source_set(asked, done_callback=lambda future: self._on_source_set(context, asked, future))

    def update(self, context: Any) -> MissionStatus:
        if not self._enabled:
            context.logger.info(f"[{self.name}] skipped: the {FLAG} flag is off")
            return MissionStatus.SUCCESS
        rf = self._rf
        readings = self._read(context)
        if self._source_switch is not None:
            switch = self._source_switch
            if not switch.confirmed and not switch.failed:
                # Nothing is flown before the flight controller has said which source set it is on (an earlier run may have left it
                # on set 2): the vehicle holds where the takeoff left it, for the five tries at most. An untrusted first tick would
                # also be the whole flight's: the takeoff's place is taken on it.
                self._watch_gps(context, readings)
                if not self._said_state:
                    context.logger.info(f"[{self.name}] holding until the flight controller answers for source set {switch.wanted}")
                    self._said_state = "HOLD"
                return MissionStatus.RUNNING
            readings = self._watch_gps(context, readings)
        command = self.flight.update(readings)
        abort_mode = rf.back.ABORT_LAND if self._abort_land_inherited else self.flight.abort_mode()
        if abort_mode != self._abort_mode:
            # RTL while the gripper has never closed and there is a position to fly home on; LAND otherwise.
            context.set_abort_mode(rf.back.ABORT_LAND if abort_mode == rf.back.ABORT_LAND else None)
            context.logger.info(f"[{self.name}] a failure of the sequence would now ask for {abort_mode}")
            self._abort_mode = abort_mode
        if self.flight.homed != self._said_homed:
            self._said_homed = self.flight.homed
            if self._said_homed is None:
                context.logger.warn(f"[{self.name}] homing: the place taken for the return point is given up (not seen again, or seen where it cannot be): the way back is by the coordinates")
            else:
                context.logger.info(f"[{self.name}] homing: the return point is taken as seen, at ({self._said_homed[0]:.2f}, {self._said_homed[1]:.2f}) in the local frame")
        if self.flight.state != self._said_state:
            context.logger.info(f"[{self.name}] State: {self._said_state or 'START'} -> {self.flight.state}" + (f" ({self.flight.reason})" if self.flight.reason else ""))
            self._said_state = self.flight.state
        elif command.reason and readings.now_s - self._last_log_s >= LOG_EVERY_S:
            context.logger.info(f"[{self.name}] {self.flight.state}: {command.reason}")
            self._last_log_s = readings.now_s
        self._apply(context, command)
        if command.status == rf.SUCCESS:
            context.logger.info(f"[{self.name}] done: the bucket is set down ({self.flight.attempts} descent(s))")
            return MissionStatus.SUCCESS
        if command.status == rf.FAILURE:
            context.logger.error(f"[{self.name}] failed: {command.reason}")
            return MissionStatus.FAILURE
        return MissionStatus.RUNNING

    # --- GPS health and the source set (REC-6) ------------------------------------------

    def _watch_gps(self, context: Any, readings: Any) -> Any:
        """Ask the monitor, tell the flight whether GPS is trusted, and keep the flight controller on the source set that goes with it."""
        import dataclasses

        health = self._gps_health
        # The bucket on the mat does not move: while it stands there its estimated place is something the GPS's drift shows in.
        # Only while the flight controller is on the GPS's set (off it, the estimate's own drift would read as the GPS's), and one
        # track at a time: the bucket's and the X's are two places, and a change from one to the other is not a drift.
        switch = self._source_switch
        on_gps = switch.acknowledged == health.GPS_SET and switch.wanted == health.GPS_SET
        target = readings.target if on_gps and self.flight.state in STATIC_TARGET_STATES else None
        kind = None if target is None else bool(target.is_bucket)
        if target is None or kind != self._static_kind:
            context.note_static_target(None, None)
        if target is not None:
            context.note_static_target(target.e, target.n)
        self._static_kind = kind
        rating = context.gps_health()
        if rating is None:
            return readings  # no monitor on the executor: as without the flag
        if rating.state != self._gps_state:
            why = f" ({'; '.join(rating.reasons)})" if rating.reasons else ""
            if rating.state == health.UNTRUSTED:
                context.logger.warn(f"[{self.name}] GPS {rating.state}{why}")
            else:
                context.logger.info(f"[{self.name}] GPS {rating.state}{why}")
            self._gps_state = rating.state
        # Back to the GPS's set only where the mission chooses: on the way out or the way home, never over the bucket or with it near the ground.
        may_go_back = self.flight.state in SOURCE_SET_BACK_STATES
        source_set, say = self._source_switch.update(rating, readings.now_s, may_go_back=may_go_back, no_gps_set_fed=bool(context.no_gps_source_fed()))
        if say:
            context.logger.warn(f"[{self.name}] {say}")
        if source_set is not None:
            asked = int(source_set)
            context.command_ekf_source_set(asked, done_callback=lambda future: self._on_source_set(context, asked, future))
        # The flight flies on the GPS only while the monitor trusts it AND the flight controller has said it is on the GPS's set
        # (before its first answer it may be on the set an earlier run left it on).
        trusted = rating.state != health.UNTRUSTED and self._source_switch.acknowledged == health.GPS_SET and self._source_switch.confirmed
        return dataclasses.replace(readings, gps_trusted=trusted)

    def _on_source_set(self, context: Any, source_set: int, future: Any) -> None:
        try:
            result = future.result()
            accepted = result is not None and bool(result.success)
        except Exception:  # noqa: BLE001  (an answer that did not come is not an acknowledgement)
            accepted = False
        said = self._source_switch.answered(source_set, accepted, context.now().nanoseconds * 1.0e-9)
        if said:
            context.logger.warn(f"[{self.name}] {said}")

    # --- the context in, the command out -----------------------------------------------

    def _read(self, context: Any) -> Any:
        rf = self._rf
        now_s = context.now().nanoseconds * 1.0e-9
        values: dict = {"now_s": now_s, "ack": self._ack}
        pose = context.local_pose
        state = getattr(context, "mavros_state", None)
        stamp = None if pose is None else _stamp_s(pose)
        # A position counts while the flight controller is connected and the pose is of this moment.
        if pose is not None and (state is None or not getattr(state, "connected", False) or stamp is None or not -0.1 <= now_s - stamp <= POSE_MAX_AGE_S):
            pose = None
        values["mode"] = getattr(state, "mode", None)
        if pose is not None:
            position = pose.pose.position
            values.update(e=float(position.x), n=float(position.y), up=float(position.z), yaw_rad=_yaw_rad(pose.pose.orientation))
            if self._yaw_deg is None:
                self._yaw_deg = math.degrees(values["yaw_rad"])  # the heading the mission began with is kept
        height = context.height_agl(self._ground_below_home_m)
        if height is not None:
            values.update(agl_m=float(height.value_m), agl_from_rangefinder=height.source == "rangefinder")
        ranged = context.rangefinder_range()
        if ranged is not None:
            values["range_age_s"] = float(ranged[1])
        throttle = context.throttle()
        if throttle is not None:
            values.update(throttle=float(throttle[0]), throttle_age_s=float(throttle[1]))
        extended = context.extended_state
        if extended is not None and context.landing_state_available():
            stamp = _stamp_s(extended)
            values.update(landed=bool(context.vehicle_is_landed()), landed_age_s=None if stamp is None else now_s - stamp)
        fix = context.global_gps
        if fix is not None and getattr(getattr(fix, "status", None), "status", 0) >= 0:  # sensor_msgs/NavSatStatus: -1 is no fix
            stamp = _stamp_s(fix)
            if stamp is not None and -0.1 <= now_s - stamp <= FIX_MAX_AGE_S:  # (a stamp a moment ahead of the clock is of this moment, as for the pose)
                values["fix"] = (float(fix.latitude), float(fix.longitude))
        values["home_up_m"] = context.home_up_m()
        if self._placed is None and self._pickup_latlon is not None:
            placed = context.latlon_to_local_enu(*self._pickup_latlon)
            if placed is not None:
                self._placed = (float(placed[0]), float(placed[1]))
        if self._boundary_latlon is not None and self._boundary is None:
            placed = [context.latlon_to_local_enu(lat, lon) for lat, lon in self._boundary_latlon]
            if all(point is not None for point in placed):
                self._boundary = tuple((float(point[0]), float(point[1])) for point in placed)
        values["boundary"] = self._boundary
        # (With a boundary given, the pickup is not handed over until the boundary can be placed too: CHECKS waits.)
        values["pickup_en"] = self._placed if self._boundary_latlon is None or self._boundary is not None else None
        for class_id in TARGET_CLASSES:
            track = context.primary_target(class_id, near=self._near)
            if track is not None and track.state in (CONFIRMED, "COASTING"):
                self._near = (track.position[0], track.position[1])
                values["target"] = rf.Target(track.position[0], track.position[1], float(track.sigma_m), float(track.age_s), track.state == CONFIRMED, class_id == "bucket")
                break
        for class_id in self._home_classes:
            # The return point as the home marker or the takeoff's keyframes show it (REC-7b): the estimator's own confirmed track, sure of its place.
            home = context.primary_target(class_id)
            if home is not None and home.state == CONFIRMED and float(home.sigma_m) <= HOMING_MAX_SIGMA_M:
                values["homing"] = rf.back.HomingFix(float(home.position[0]), float(home.position[1]), float(home.age_s))
                break
        return rf.Readings(**values)

    def _apply(self, context: Any, command: Any) -> None:
        rf = self._rf
        yaw_deg = self._yaw_deg if self._yaw_deg is not None else 90.0
        if command.mode is not None:
            context.clear_all_setpoints()  # the flight controller's own landing: nothing of ours is commanded over it
            context.request_mode_change(command.mode)
        elif command.frame == rf.LOCAL:
            context.clear_global_position_setpoint()
            context.set_local_position_setpoint(command.e, command.n, command.up, yaw_deg=yaw_deg)
        elif command.frame == rf.GLOBAL:
            context.clear_local_position_setpoint()
            context.set_global_position_setpoint(command.lat, command.lon, command.alt, yaw_deg=yaw_deg)
        if command.gripper is not None:
            attempt = command.attempt
            # A no-op without the actuation lock (PLT-09). The flight commands the gripper only where its rules say so:
            # the close in ENGAGE (the cone brought the hook to the handle), the open in SET_DOWN (the release rule).
            # The lock is told the state, and that the command is the one that state's rule gives: an open from
            # ENGAGE, or a close from SET_DOWN, is no state's rule and is refused.
            expected_state = rf.SET_DOWN if command.gripper == rf.OPEN else rf.ENGAGE
            context.permit_actuation("gripper", state=self.flight.state, gates_hold=self.flight.state == expected_state, why=f"{command.gripper} is not {self.flight.state}'s command")
            context.command_gripper(release=command.gripper == rf.OPEN, done_callback=lambda future: self._on_gripper(future, attempt))

    def _on_gripper(self, future: Any, attempt: int) -> None:
        """The flight controller's answer to a gripper command, kept with the attempt it answers."""
        from drone_mission_core.recovery_grip import Ack

        try:
            response = future.result()
            accepted = response is not None and getattr(response, "success", False)
            result = int(getattr(response, "result", 0)) if accepted else 4  # MAV_RESULT_FAILED
        except Exception:  # noqa: BLE001  (a service call that failed is a command not accepted)
            result = 4
        self._ack = Ack(attempt, result)
