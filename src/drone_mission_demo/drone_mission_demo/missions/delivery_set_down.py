"""The delivery's soft set-down (CEN-05): the work of the FINAL_SET_DOWN state.

Only with the mission flag `delivery_soft_set_down` on. It takes the place
of the 2026 fixed GPS column (FINAL_FIXED_COLUMN_DESCENT): from the handoff
height the mission goes on coming down on the estimator's track, and sets
the package down.

One tick, in this order:

  1. In the hold (below): the GPS point the vehicle was over when the hold
     was taken, with the height setpoint going down at 0.12 m/s. Nothing
     steers any more, whatever the track does, so that no correction drags
     the package; the height coming back up does not undo it (close to the
     ground the rangefinder goes under its range and the height is the
     barometer's). Contact is asked every tick (set_down.ContactDetector:
     the rangefinder at or below the package's height, no vertical speed,
     the throttle dropped, together for 0.3 s), if the hold was taken on the
     rangefinder's height: a rangefinder that was not giving the height then
     is not believed about the ground now. The flight controller's own
     "landed", held for the mission's `touchdown_dwell_s`, is contact too,
     and the only one on a barometer height. HOLD_TIMEOUT_S in the hold
     without contact fails the mission.
  2. Before the hold: the flight controller says landed (the ground came
     early, on a height that read high): stop steering, and take it as
     contact after `touchdown_dwell_s`. No height above the ground: hold,
     and fail after NO_HEIGHT_TIMEOUT_S. No track that can be steered on:
     the mission's own loss grace, then its recovery. Otherwise steer the
     payload onto the track (the mission's centring on the estimator) and
     come down while centred on a track that may be descended on, at the
     mission's descent rate, easing to 0.12 m/s (set_down.descent_rate).
  3. The hold is taken on the tick the vehicle is both low and centred on a
     track that may be descended on. Low: 0.6 m or less above the ground
     (a height that is not the rangefinder's counts as half a metre lower
     than it says), or within 0.05 m of the height under which the mission
     takes no target (`min_projection_altitude_m`, on the flight
     controller's height). Low and not centred: steer, do not descend.

The mission (package_delivery_mission.py) turns the answer into a state:
CONTACT is GROUND_DWELL, where the release is commanded; REACQUIRE is
TARGET_NOT_FOUND; GIVE_UP fails the mission. It also holds the descent
while the time base is unhealthy, as it does in the 2026 column.

No ROS here: only the mission and its context are used.
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

from drone_mission_core import set_down
from drone_mission_core.safety_limits import PHASE_SPEED_CAPS_MPS

RUNNING, CONTACT, REACQUIRE, GIVE_UP = "running", "contact", "reacquire", "give up"
LOG_EVERY_S = 2.0
RANGEFINDER = "rangefinder"


class SoftSetDown:
    def __init__(self, mission: Any, package_height_m: Any) -> None:
        self._mission = mission
        self._detector = set_down.ContactDetector(package_height_m)  # raises ValueError for a height that cannot be one
        self.hold_point: Optional[Tuple[float, float]] = None  # (latitude, longitude) held from the hold on
        self._hold_started_s: Optional[float] = None
        self._range_believed = False  # the hold was taken on the rangefinder's height
        self._height_setpoint_m: Optional[float] = None
        self._last_step_s: Optional[float] = None
        self._landed_since_s: Optional[float] = None
        self._no_height_since_s: Optional[float] = None
        self._last_log_s: Dict[str, float] = {}

    def enter(self, context: Any) -> None:
        """Start a spell in FINAL_SET_DOWN: nothing is held yet, and contact is asked afresh."""
        self.hold_point = None
        self._hold_started_s = None
        self._range_believed = False
        self._height_setpoint_m = None
        self._last_step_s = None
        self._landed_since_s = None
        self._no_height_since_s = None
        self._detector.reset()
        self._last_log_s = {}

    @property
    def holding(self) -> bool:
        return self.hold_point is not None

    def pause(self) -> None:
        """The descent is being held from outside (the time guard): the height setpoint does not run on meanwhile."""
        self._last_step_s = None
        self._detector.reset()

    def contact_again(self) -> None:
        """The mission came back from the release state without releasing: contact is asked afresh."""
        self._detector.reset()
        self._landed_since_s = None

    def _say(self, context: Any, now_s: float, text: str, warn: bool = False, about: str = "descent") -> None:
        """Log a line, at most every LOG_EVERY_S for each thing it is `about`."""
        last = self._last_log_s.get(about)
        if last is not None and now_s - last < LOG_EVERY_S:
            return
        self._last_log_s[about] = now_s
        # Two lines, not one with the method chosen: rclpy lets a line of code log at one severity only.
        if warn:
            context.logger.warn(f"[{self._mission.name}] {text}")
        else:
            context.logger.info(f"[{self._mission.name}] {text}")

    def _reading(self, context: Any) -> set_down.ContactReading:
        ranged, velocity, throttle = context.rangefinder_range(), context.vehicle_velocity(), context.throttle()
        return set_down.ContactReading(
            range_m=None if ranged is None else ranged[0],
            range_age_s=None if ranged is None else ranged[1],
            vertical_speed_mps=None if velocity is None else float(velocity[2]),
            throttle=None if throttle is None else throttle[0],
            throttle_age_s=None if throttle is None else throttle[1],
        )

    def _landed_s(self, context: Any, now_s: float) -> Optional[float]:
        """How long the flight controller has said landed without a break, or None when it does not say so."""
        if context.landing_state_available() and context.vehicle_is_landed():
            if self._landed_since_s is None:
                self._landed_since_s = now_s
            return now_s - self._landed_since_s
        self._landed_since_s = None
        return None

    def _landed_contact(self, context: Any, waiting_on: str) -> str:
        mission = self._mission
        context.logger.warn(
            f"[{mission.name}] contact: the flight controller says landed, and has for {mission._touchdown_dwell_s:g} s; {waiting_on}"
        )
        return CONTACT

    def update(self, context: Any) -> str:
        """One tick. Sets this tick's setpoint and says what the mission should do next."""
        from .red_bullseye_mission_base import HOLD_YAW_DEG

        mission = self._mission
        now_s = context.now().nanoseconds * 1.0e-9
        reading = self._reading(context)
        landed_s = self._landed_s(context, now_s)
        if self.hold_point is not None:
            return self._set_down(context, now_s, reading, landed_s)

        self._detector.observe(reading, now_s)  # the throttle while coming down, for the contact rule later
        if landed_s is not None:
            # The ground came before the hold: a height that read high. Nothing steers on the ground.
            context.set_local_velocity_setpoint(0.0, 0.0, 0.0, yaw_deg=HOLD_YAW_DEG)
            if landed_s >= mission._touchdown_dwell_s:
                return self._landed_contact(context, "the hold height was never reached by the height the mission had")
            return RUNNING
        height = context.height_agl(mission._ground_below_home_m)
        if height is None:
            context.set_local_velocity_setpoint(0.0, 0.0, 0.0, yaw_deg=HOLD_YAW_DEG)
            if self._no_height_since_s is None:
                self._no_height_since_s = now_s
            if now_s - self._no_height_since_s >= set_down.NO_HEIGHT_TIMEOUT_S:
                context.logger.error(
                    f"[{mission.name}] final set-down: no height above the ground for {set_down.NO_HEIGHT_TIMEOUT_S:g} s; giving up"
                )
                return GIVE_UP
            self._say(context, now_s, "final set-down: no height above the ground, holding", warn=True, about="height")
            return RUNNING
        self._no_height_since_s = None

        solution = mission._estimator_solution(context)
        tracking = None if solution is None else mission._estimator_tracking_solution(context)
        if solution is None or tracking is None:
            if mission._hold_tracking_loss_grace(context):
                return REACQUIRE
            return RUNNING
        mission._clear_target_loss_grace()
        steer_error_m, velocity_east, velocity_north = tracking  # infinite while the track may not be descended on
        centred = steer_error_m <= mission._centering_tolerance_m

        from_rangefinder = height.source == RANGEFINDER
        working_m = set_down.working_height(height.value_m, from_rangefinder)
        vehicle_up_m = float(context.local_pose.pose.position.z)
        at_the_floor = vehicle_up_m <= mission._min_projection_altitude_m + set_down.FLOOR_MARGIN_M
        low = working_m <= set_down.HOLD_BELOW_M or at_the_floor
        if low and centred:
            self.hold_point = (context.global_gps.latitude, context.global_gps.longitude)
            self._hold_started_s = now_s
            self._range_believed = from_rangefinder
            self._height_setpoint_m = vehicle_up_m
            self._last_step_s = now_s
            why = "" if working_m <= set_down.HOLD_BELOW_M else " (under the height the mission steers down to)"
            context.logger.info(
                f"[{mission.name}] final set-down: at {height.value_m:.2f} m on the {height.source}{why}, error {solution.error_m:.2f} m: "
                f"holding the position and descending at {set_down.SET_DOWN_RATE_MPS:g} m/s"
            )
            return self._set_down(context, now_s, reading, landed_s)

        vertical = 0.0
        if centred and not low:
            rate = min(abs(mission._descent_rate_mps), PHASE_SPEED_CAPS_MPS["final_descent"])
            vertical = -set_down.descent_rate(working_m, rate)
        context.set_local_velocity_setpoint(velocity_east, velocity_north, vertical, yaw_deg=HOLD_YAW_DEG)
        self._say(context, now_s, f"final set-down: {height.value_m:.2f} m on the {height.source}, error {solution.error_m:.2f} m")
        return RUNNING

    def _set_down(self, context: Any, now_s: float, reading: set_down.ContactReading, landed_s: Optional[float]) -> str:
        """In the hold: the held point, the height setpoint going down at the set-down rate, and contact."""
        from .red_bullseye_mission_base import HOLD_YAW_DEG

        mission = self._mission
        elapsed_s = 0.0 if self._last_step_s is None else max(0.0, min(now_s - self._last_step_s, 0.5))
        self._last_step_s = now_s
        # The setpoint never stands above the vehicle, and goes down from there at the set-down rate, as far as
        # the mission's `touchdown_min_altitude_m`: on the ground it runs on below the vehicle, which is what
        # makes the flight controller take the throttle off.
        self._height_setpoint_m = max(
            mission._touchdown_min_altitude_m,
            min(self._height_setpoint_m, float(context.local_pose.pose.position.z)) - set_down.SET_DOWN_RATE_MPS * elapsed_s,
        )
        context.set_global_position_setpoint(self.hold_point[0], self.hold_point[1], self._height_setpoint_m, yaw_deg=HOLD_YAW_DEG, lock_yaw=True)

        detector = self._detector
        touching = detector.update(reading, now_s)
        if touching and self._range_believed:
            context.logger.info(
                f"[{mission.name}] contact: rangefinder {reading.range_m:.2f} m (package {detector.package_height_m:g} m), "
                f"vertical speed {reading.vertical_speed_mps:+.2f} m/s, throttle {reading.throttle:.2f} against {detector.baseline:.2f} "
                f"while descending, together for {set_down.CONTACT_HOLD_S:g} s"
            )
            return CONTACT
        if landed_s is not None and landed_s >= mission._touchdown_dwell_s:
            if self._range_believed:
                return self._landed_contact(context, f"the contact rule was still waiting on {', '.join(detector.missing) or 'its 0.3 s'}")
            return self._landed_contact(context, "the height at the hold was not the rangefinder's, so its reading is not asked")

        if now_s - self._hold_started_s >= set_down.HOLD_TIMEOUT_S:
            context.logger.error(
                f"[{mission.name}] set-down: no contact within {set_down.HOLD_TIMEOUT_S:g} s of holding the position "
                f"(waiting on {', '.join(detector.missing) or 'nothing'}), and the flight controller does not say landed; giving up"
            )
            return GIVE_UP
        if not self._range_believed:
            self._say(context, now_s, "set-down: waiting on the flight controller's landed state (the height at the hold was not the rangefinder's)", about="contact")
        elif detector.missing:
            self._say(context, now_s, f"set-down: waiting on {', '.join(detector.missing)}", about="contact")
        else:
            self._say(context, now_s, f"set-down: contact signals together for {detector.held_s:.2f} s", about="contact")
        return RUNNING
