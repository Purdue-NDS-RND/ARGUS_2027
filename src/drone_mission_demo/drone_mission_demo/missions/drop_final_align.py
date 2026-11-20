"""The drop's FINAL_ALIGN state (CEN-04): descend on the estimator to the release height and let go when the gates hold.

Used by package_drop_mission.py only with the mission flag
`drop_release_gates` on, in place of the 2026 fixed GPS column
(FINAL_FIXED_DROP_COLUMN) and timed hover. With the flag off this module is
not used.

Each tick, in this order:

  1. No height: hold still.
  2. Under 2.0 m above the ground (safety_limits.RELEASE_GATE_FLOOR_M): climb,
     straight up, and do nothing else until back above it.
  3. No track to steer on: the mission's own handling of a lost target (hold
     for the grace time, then look again).
  4. Steer the payload over the track, as TRACK_AND_DESCEND does, and move
     toward the release height for the sensor that gives the height
     (release_gates.release_band): down to the middle of the band while
     centred on a track that may be descended on, slower as it comes near;
     up when under the band.
  5. Read the gates (release_gates.py). Held for half a second: release.
  6. 25 s here without that: look for the target again, once. A second 25 s:
     release anyway if within half a metre on a track the release rule
     accepts and not under the floor; otherwise give up, and the sequence's
     failure policy moves on.

The release itself is the mission's DROP_PAYLOAD state, entered at once and
where the vehicle is: no climb to the 2026 drop height and no hover on a GPS
point first.
"""

from __future__ import annotations

import math
from typing import Any, Optional

from drone_mission_core import release_gates
from drone_mission_core.centring import yaw_of
from drone_mission_core.safety_limits import PHASE_SPEED_CAPS_MPS, RELEASE_GATE_FLOOR_M

RUNNING, RELEASE, REACQUIRE, GIVE_UP = "running", "release", "reacquire", "give up"
CLIMB_BACK_MPS = 0.3  # from under the floor
INTO_BAND_MPS = 0.2  # from under the release band, above the floor
MIN_DESCENT_MPS = 0.1
APPROACH_PER_S = 0.3  # descent rate per metre above the release height, so the band is entered slowly
AT_HEIGHT_M = 0.05  # this close above the release height the descent stops
LOG_EVERY_S = 2.0


def tilt_deg(orientation: Any) -> float:
    """The vehicle's tilt from level, in degrees, from its quaternion."""
    x, y = float(orientation.x), float(orientation.y)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


class FinalAlign:
    def __init__(self, mission: Any) -> None:
        self._mission = mission
        self._hold = release_gates.GateHold()
        self._started_s: Optional[float] = None
        self._last_log_s: dict = {}
        self._timed_out = False  # this spell's 25 s have passed
        self.timeouts = 0  # 25 s spells without a release, over the whole mission
        self.last_reading: Optional[release_gates.GateReading] = None

    def enter(self, context: Any) -> None:
        """Start a spell in FINAL_ALIGN: its 25 s count from now. The count of spells is kept."""
        self._hold.reset()
        self._started_s = context.now().nanoseconds * 1.0e-9
        self._last_log_s = {}
        self._timed_out = False

    def gates_again(self) -> None:
        """The mission came back from the release state without releasing: the gates' half second starts afresh. The 25 s go on."""
        self._hold.reset()

    def _can_command(self, context: Any) -> bool:
        """Whether a release could be commanded this tick: a fake drop commands nothing."""
        return bool(self._mission._fake_drop) or bool(context.command_service_ready())

    def _can_look_again(self) -> bool:
        mission = self._mission
        return mission._recovery_attempts < mission._max_recovery_attempts

    def _say(self, context: Any, now_s: float, text: str, warn: bool = False, about: str = "gates") -> None:
        """Log a line, at most every LOG_EVERY_S for each thing it is `about`."""
        last = self._last_log_s.get(about)
        if last is not None and now_s - last < LOG_EVERY_S:
            return
        self._last_log_s[about] = now_s
        # Two lines, not one with the method chosen: rclpy lets a line of code log at one severity only, and raises otherwise.
        if warn:
            context.logger.warn(f"[{self._mission.name}] {text}")
        else:
            context.logger.info(f"[{self._mission.name}] {text}")

    def _stop(self, context: Any, up_mps: float = 0.0) -> None:
        from .red_bullseye_mission_base import HOLD_YAW_DEG

        context.set_local_velocity_setpoint(0.0, 0.0, up_mps, yaw_deg=HOLD_YAW_DEG)

    def update(self, context: Any) -> str:
        """One tick. Sets this tick's setpoint and says what the mission should do next."""
        from .red_bullseye_mission_base import HOLD_YAW_DEG

        mission = self._mission
        now_s = context.now().nanoseconds * 1.0e-9
        if self._started_s is None:
            self.enter(context)
        height = context.height_agl(mission._ground_below_home_m)
        if height is None:
            self._hold.reset()
            self._stop(context)
            self._say(context, now_s, "final align: no height above the ground, holding", warn=True, about="height")
            return self._after_timeout(context, now_s, None)
        if height.value_m < RELEASE_GATE_FLOOR_M:
            # Before anything else: back above our own floor, which keeps the rules' 1.83 m out of reach.
            self._hold.reset()
            self._stop(context, CLIMB_BACK_MPS)
            self._say(context, now_s, f"final align: {height.value_m:.2f} m is below {RELEASE_GATE_FLOOR_M:g} m, climbing back", warn=True, about="floor")
            return RUNNING

        solution = mission._estimator_solution(context)
        tracking = None if solution is None else mission._estimator_tracking_solution(context)
        if solution is None or tracking is None:
            self._hold.reset()
            if mission._hold_tracking_loss_grace(context):
                return REACQUIRE
            return RUNNING
        mission._clear_target_loss_grace()
        steer_error_m, velocity_east, velocity_north = tracking  # infinite while the track may not be descended on

        low, target, high = release_gates.release_band(height.source)
        vertical = 0.0
        if height.value_m > target + AT_HEIGHT_M:
            # Down to the middle of the band, not just inside its top edge: the height then has room either way.
            if steer_error_m <= mission._centering_tolerance_m:
                rate = min(abs(mission._descent_rate_mps), PHASE_SPEED_CAPS_MPS["final_descent"])
                vertical = -min(rate, max(MIN_DESCENT_MPS, APPROACH_PER_S * (height.value_m - target)))
        elif height.value_m < low:
            vertical = INTO_BAND_MPS
        context.set_local_velocity_setpoint(velocity_east, velocity_north, vertical, yaw_deg=HOLD_YAW_DEG)

        velocity = context.vehicle_velocity()
        reading = release_gates.GateReading(
            error_m=solution.error_m,
            sigma_m=solution.sigma_m,
            speed_mps=None if velocity is None else math.hypot(float(velocity[0]), float(velocity[1])),
            tilt_deg=tilt_deg(context.local_pose.pose.orientation),
            may_release=bool(context.may_release(mission._estimator_centring.class_id, track_id=solution.track_id)),
            height_m=height.value_m,
            height_source=height.source,
        )
        self.last_reading = reading
        held = self._hold.update(reading, now_s)
        if held and not self._can_command(context):
            # Stay on the gates, steering, until the command can go: a release sent later would be on gates that held earlier.
            self._say(context, now_s, "release gates hold, but the release command cannot be sent yet: staying on the target", warn=True, about="command")
            return self._after_timeout(context, now_s, reading)
        if held:
            context.logger.info(
                f"[{mission.name}] release gates held for {release_gates.HOLD_S:g} s: error {reading.error_m:.2f} m, sigma {reading.sigma_m:.2f} m, "
                f"speed {reading.speed_mps:.2f} m/s, tilt {reading.tilt_deg:.1f} deg, {reading.height_m:.2f} m on the {reading.height_source}"
            )
            return RELEASE
        waiting = ", ".join(self._hold.last_failing)
        self._say(
            context,
            now_s,
            f"release gates: waiting on {waiting} ({height.value_m:.2f} m on the {height.source}, error {solution.error_m:.2f} m)"
            if waiting
            else f"release gates: all holding for {self._hold.held_s:.2f} s",
        )
        return self._after_timeout(context, now_s, reading)

    def _after_timeout(self, context: Any, now_s: float, reading: Optional[release_gates.GateReading]) -> str:
        if now_s - self._started_s < release_gates.ALIGN_TIMEOUT_S:
            return RUNNING
        mission = self._mission
        if not self._timed_out:
            self._timed_out = True
            self.timeouts += 1
            self._hold.reset()
            if self.timeouts == 1 and self._can_look_again():
                context.logger.warn(
                    f"[{mission.name}] the release gates did not hold within {release_gates.ALIGN_TIMEOUT_S:g} s: looking for the target again"
                )
                return REACQUIRE
            if self.timeouts == 1:
                # The approach used every recovery attempt: there is no looking again, so this timeout is the last.
                context.logger.warn(
                    f"[{mission.name}] the release gates did not hold within {release_gates.ALIGN_TIMEOUT_S:g} s and no recovery attempt is left"
                )
        if reading is not None and release_gates.last_resort_allowed(reading):
            if not self._can_command(context):
                self._say(context, now_s, "the last timeout has passed and the target is near, but the release command cannot be sent yet", warn=True, about="last resort")
                return RUNNING
            context.logger.warn(
                f"[{mission.name}] the release gates did not hold within the last {release_gates.ALIGN_TIMEOUT_S:g} s: "
                f"releasing at the current estimate, {reading.error_m:.2f} m off"
            )
            return RELEASE
        context.logger.error(
            f"[{mission.name}] the release gates did not hold within the last {release_gates.ALIGN_TIMEOUT_S:g} s and the target is not within "
            f"{release_gates.LAST_RESORT_MAX_ERROR_M:g} m on a fresh track at a height a release is allowed at: no release"
        )
        return GIVE_UP

    def release_allowed_now(self, context: Any) -> bool:
        """At the moment of the release itself: still not under the lowest release height for the sensor that gives the height now."""
        height = context.height_agl(self._mission._ground_below_home_m)
        return height is not None and release_gates.release_height_allowed(height.value_m, height.source)
