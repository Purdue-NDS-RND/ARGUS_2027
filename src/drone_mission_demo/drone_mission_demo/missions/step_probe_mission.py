"""The command step probe (PLT-05): short velocity steps in a hover, for timing the flight controller's response.

Behind the mission flag `command_step_probe` (default false). With the flag
off the mission is skipped: it returns SUCCESS on its first tick and
commands nothing.

With it on, the steps are drone_mission_core/step_probe.py's: zero velocity
for `rest_s`, `step_speed_mps` for `step_s`, zero again, the next step the
other way, `steps` times. The vehicle must already be flying in GUIDED (a
takeoff mission comes first).

The mission never takes the vehicle back from somebody else. A mode that is
not GUIDED is the safety pilot's or a failsafe's: the mission clears its
setpoints, commands nothing, asks for no mode and does not fail (a failure
would have the sequence's `abort_and_rtl` ask for RTL over the pilot). It
waits. If GUIDED comes back the probe is over: no more steps, the vehicle is
held where it is, and the sequence goes on.

It ends as a failure, holding, when the vehicle is `max_drift_m` from where
it began, when the clock steps, when there has been no pose of this moment
for 5 s, or when the pose comes back after more than a second away in the
middle of the probe. A pose counts while the flight controller is connected
and the pose is under 0.5 s old; without one no step is commanded.

Fly it with the topics recorded, then
`python3 sim/replay/command_delay.py <bag>` gives the actuation delay.
"""

from __future__ import annotations

import math
from typing import Any, Optional

from drone_mission_core.features import get_feature_flag
from drone_mission_core.mission_api import BaseMission, MissionStatus
from drone_mission_core.registry import register_mission

FLAG = "command_step_probe"  # step_probe.FLAG (a test holds the two together); named here so that the flag-off path imports nothing new
NO_POSE_LIMIT_S = 5.0
POSE_MAX_AGE_S = 0.5


def _yaw_deg(q: Any) -> float:
    """Heading from a pose's quaternion, in the setpoints' convention: degrees anticlockwise from east."""
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def _stamp_s(message: Any) -> Optional[float]:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    return None if stamp is None else float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


@register_mission("command_step_probe")
class CommandStepProbeMission(BaseMission):
    """Velocity steps in a hover, there and back."""

    def __init__(self, spec) -> None:
        super().__init__(spec)
        self._enabled = bool(get_feature_flag(spec.features, FLAG))
        self.probe: Any = None
        if not self._enabled:
            return
        from drone_mission_core import step_probe

        self._sp = step_probe
        try:
            self.plan = step_probe.plan_from_config(spec.config or {})
        except ValueError as error:
            raise ValueError(f"mission '{spec.name}': {error}") from error

    def on_enter(self, context: Any) -> None:
        if not self._enabled:
            return
        self.probe = self._sp.StepProbe(self.plan)
        self._yaw_deg: Optional[float] = None
        self._pose_seen = context.now()
        self._stepping = False
        self._taken = False  # somebody else has had the vehicle since the probe began
        context.logger.info(f"[{self.name}] {self.plan.steps} steps of {self.plan.step_speed_mps:.1f} m/s for {self.plan.step_s:.1f} s, {self._sp.duration_s(self.plan):.0f} s in all")

    def _fresh_pose(self, context: Any) -> Any:
        """The local pose, while the flight controller is connected and the pose is of this moment; else None."""
        pose, state = context.local_pose, context.mavros_state
        stamp = None if pose is None else _stamp_s(pose)
        now_s = context.now().nanoseconds * 1.0e-9
        if pose is None or state is None or not getattr(state, "connected", False) or stamp is None or not -0.1 <= now_s - stamp <= POSE_MAX_AGE_S:
            return None
        return pose

    def update(self, context: Any) -> MissionStatus:
        if not self._enabled:
            context.logger.info(f"[{self.name}] skipped: the {FLAG} flag is off")
            return MissionStatus.SUCCESS
        mode = getattr(context.mavros_state, "mode", None)
        if mode != "GUIDED":
            # Not ours to undo: the safety pilot's switch, or a failsafe. Nothing is commanded, no mode is asked
            # for, and the mission does not fail: the sequence's failure policy would ask for RTL.
            if not self._taken:
                context.clear_all_setpoints()
                context.logger.warn(f"[{self.name}] the mode is {mode or 'unknown'}, not GUIDED: taken by the pilot or a failsafe. The mission commands nothing")
                self._taken = True
            return MissionStatus.RUNNING
        pose = self._fresh_pose(context)
        if self._taken:
            # Handed back. The probe does not start stepping again under whoever just let go.
            # Held where it is, facing as it faces now. With no pose of this moment nothing is commanded: the
            # setpoints were cleared at the takeover, and GUIDED with no setpoint holds.
            if pose is not None:
                position = pose.pose.position
                context.set_local_position_setpoint(float(position.x), float(position.y), float(position.z), yaw_deg=_yaw_deg(pose.pose.orientation))
            context.logger.warn(f"[{self.name}] ended early: GUIDED again after a takeover, {self.probe.steps_begun} of {self.plan.steps} steps commanded")
            return MissionStatus.SUCCESS
        if pose is None:
            # No position to watch the drift by: no step is commanded on it.
            if self._yaw_deg is not None:
                context.set_local_velocity_setpoint(0.0, 0.0, 0.0, yaw_deg=self._yaw_deg)
            if context.seconds_since(self._pose_seen) > NO_POSE_LIMIT_S:
                context.logger.error(f"[{self.name}] failed: no local pose of this moment for {NO_POSE_LIMIT_S:.0f} s")
                return MissionStatus.FAILURE
            return MissionStatus.WAITING
        position = pose.pose.position
        outage_s = context.seconds_since(self._pose_seen)
        self._pose_seen = context.now()
        if self._yaw_deg is None:
            self._yaw_deg = _yaw_deg(pose.pose.orientation)  # the heading the probe began with is kept
        elif outage_s > self._sp.MAX_TICK_GAP_S:
            # The steps are timed tick by tick: after a gap the probe does not pick up where it was.
            context.set_local_position_setpoint(float(position.x), float(position.y), float(position.z), yaw_deg=self._yaw_deg)
            context.logger.error(f"[{self.name}] failed: no local pose of this moment for {outage_s:.1f} s in the middle of the probe")
            return MissionStatus.FAILURE
        command = self.probe.update(context.now().nanoseconds * 1.0e-9, float(position.x), float(position.y))
        if command.done or command.failed:
            context.set_local_position_setpoint(float(position.x), float(position.y), float(position.z), yaw_deg=self._yaw_deg)
            if command.failed:
                context.logger.error(f"[{self.name}] failed: {command.reason}")
                return MissionStatus.FAILURE
            context.logger.info(f"[{self.name}] done: {self.plan.steps} steps commanded")
            return MissionStatus.SUCCESS
        stepping = bool(command.east_mps or command.north_mps)
        if stepping != self._stepping:
            context.logger.info(f"[{self.name}] {'step' if stepping else 'rest'}: velocity ({command.east_mps:+.2f}, {command.north_mps:+.2f}) m/s")
            self._stepping = stepping
        context.set_local_velocity_setpoint(command.east_mps, command.north_mps, 0.0, yaw_deg=self._yaw_deg)
        return MissionStatus.RUNNING
