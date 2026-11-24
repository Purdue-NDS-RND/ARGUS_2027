"""The geofence demonstration (OPS-02): fly at the fence so that the flight controller is seen to stop the vehicle.

Behind the mission flag `fence_demo` (default false). With the flag off the
mission is skipped: it returns SUCCESS on its first tick and commands
nothing.

With it on, the run is drone_mission_core/fence_demo.py's: zero velocity for
`settle_s`, then `speed_mps` along the compass bearing `bearing_deg`. The
vehicle must already be flying in GUIDED (a takeoff mission comes first).
The flight controller ends the run, in one of two ways:

  held      Its avoidance stops the vehicle at the fence's margin while the
            velocity is still commanded, and keeps it there for the 30 s the
            mission goes on commanding (what the sim shows). The vehicle is
            still the mission's: it holds where it is, asks for `rtl_mode`
            once, and from the tick the mode changes commands nothing.
  breached  The vehicle goes over and the fence's action takes it: the mode
            is no longer GUIDED, and from that tick the mission commands
            nothing.

The run is measured from where the vehicle is when it begins, and its
lengths (`boundary_m`, `max_run_m`) are from the surveyed takeoff point. So
the mission starts the run only within START_TOLERANCE_M of `start_latitude`,
`start_longitude` by the flight controller's own position; anywhere else, or
with no position, it fails before commanding anything.

The mission never takes the vehicle back. A mode that is not GUIDED and that
it did not ask for is the fence's, a failsafe's or the safety pilot's: the
mission clears its setpoints, asks for no mode and does not fail (a failure
would have the sequence's `abort_and_rtl` ask for RTL over whoever has the
vehicle). It waits. When the vehicle is disarmed in the way-home mode the
demonstration is over, and the mission ends. A vehicle somebody landed in
another mode is the pilot's: the mission stays quiet (stop the launch). If
GUIDED comes back in the air, the run is not started again: the mission
stops the vehicle and fails, and the failure policy asks for the way home.

Every failure is a stop first. The executor clears the setpoints of a
mission that fails, and the flight controller would fly its last velocity
target on for seconds: so the mission commands a position hold and keeps it
for STOP_S before it reports the failure. It fails when the run has gone
`max_run_m` and the vehicle is still under command (the fence did neither),
when the vehicle stops anywhere but at the fence or leaves the run's line
sideways, when the run takes too long, when the clock steps, when there has
been no pose of this moment for 5 s, when the vehicle is not at the takeoff
point, or when the flight controller does not take the RTL asked for after
a hold. The sequence's failure policy then asks for the way home.
"""

from __future__ import annotations

import math
from typing import Any, Optional

from drone_mission_core.features import get_feature_flag
from drone_mission_core.mission_api import BaseMission, MissionStatus
from drone_mission_core.registry import register_mission

FLAG = "fence_demo"  # fence_demo.FLAG (a test holds the two together); named here so that the flag-off path imports nothing new
NO_POSE_LIMIT_S = 5.0
POSE_MAX_AGE_S = 0.5
RTL_WAIT_S = 5.0  # after a hold at the fence: how long the flight controller has to take the RTL asked for
STOP_S = 2.0  # a position hold is commanded this long before a failure is reported: the failure policy clears the setpoints
START_TOLERANCE_M = 10.0  # the run begins no further than this from the surveyed takeoff point its lengths are measured from
HOME_MODES = ("RTL", "SMART_RTL")  # what `rtl_mode` may be: a mode that brings the vehicle home
EARTH_RADIUS_M = 6378137.0


def _yaw_deg(q: Any) -> float:
    """Heading from a pose's quaternion, in the setpoints' convention: degrees anticlockwise from east."""
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def _stamp_s(message: Any) -> Optional[float]:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    return None if stamp is None else float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


@register_mission("fence_demo")
class FenceDemoMission(BaseMission):
    """One run at the flight boundary, ended by the flight controller's fence."""

    def __init__(self, spec) -> None:
        super().__init__(spec)
        self._enabled = bool(get_feature_flag(spec.features, FLAG))
        self.demo: Any = None
        if not self._enabled:
            return
        from drone_mission_core import fence_demo

        self._fd = fence_demo
        config = dict(spec.config or {})
        self._rtl_mode = str(config.pop("rtl_mode", "RTL"))  # the way home after a hold at the fence
        start = (config.pop("start_latitude", None), config.pop("start_longitude", None))
        try:
            if self._rtl_mode not in HOME_MODES:
                raise ValueError(f"rtl_mode must be one of {', '.join(HOME_MODES)} (a mode that brings the vehicle home), got '{self._rtl_mode}'")
            if not all(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) for value in start) or abs(start[0]) > 90.0 or abs(start[1]) > 180.0:
                raise ValueError("start_latitude and start_longitude must be given: the takeoff point the run's lengths are measured from (tools/field_setup.py writes them)")
            self._start = (float(start[0]), float(start[1]))
            self.plan = fence_demo.plan_from_config(config)
        except ValueError as error:
            raise ValueError(f"mission '{spec.name}': {error}") from error

    def on_enter(self, context: Any) -> None:
        if not self._enabled:
            return
        self.demo = self._fd.FenceDemo(self.plan)
        self._yaw_deg: Optional[float] = None
        self._pose_seen = context.now()
        self._running = False
        self._taken = False  # something else has had the vehicle since the mission began: the fence, a failsafe or the pilot
        self._held_since = None  # when the vehicle was found held at the fence, and RTL was asked for
        self._stopping = None  # (reason, since when): a failure that is being stopped for before it is reported
        self._start_checked = False
        self._entered = context.now()
        context.logger.info(
            f"[{self.name}] in {self.plan.settle_s:.0f} s: {self.plan.speed_mps:.1f} m/s on bearing {self.plan.bearing_deg:.0f} until the fence acts; "
            f"the boundary is {self.plan.boundary_m:.0f} m away and the run fails at {self.plan.max_run_m:.0f} m"
        )

    def _fresh_pose(self, context: Any) -> Any:
        """The local pose, while the flight controller is connected and the pose is of this moment; else None."""
        pose, state = context.local_pose, context.mavros_state
        stamp = None if pose is None else _stamp_s(pose)
        now_s = context.now().nanoseconds * 1.0e-9
        if pose is None or state is None or not getattr(state, "connected", False) or stamp is None or not -0.1 <= now_s - stamp <= POSE_MAX_AGE_S:
            return None
        return pose

    def _stop(self, context: Any, pose: Any, reason: str) -> MissionStatus:
        """Begin a failure: the vehicle is told to hold where it is, and the failure is reported STOP_S later."""
        if pose is not None:
            position = pose.pose.position
            yaw = self._yaw_deg if self._yaw_deg is not None else _yaw_deg(pose.pose.orientation)
            context.set_local_position_setpoint(float(position.x), float(position.y), float(position.z), yaw_deg=yaw)
        elif self._yaw_deg is not None:
            context.set_local_velocity_setpoint(0.0, 0.0, 0.0, yaw_deg=self._yaw_deg)
        context.logger.error(f"[{self.name}] stopping: {reason}")
        self._stopping = (reason, context.now())
        return MissionStatus.RUNNING

    def _from_start_m(self, context: Any) -> Optional[float]:
        """How far the vehicle is from the surveyed takeoff point, by the flight controller's position; None with no fix."""
        fix = context.global_gps
        latitude, longitude = getattr(fix, "latitude", None), getattr(fix, "longitude", None)
        if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in (latitude, longitude)):
            return None
        north = math.radians(latitude - self._start[0]) * EARTH_RADIUS_M
        east = math.radians(longitude - self._start[1]) * EARTH_RADIUS_M * math.cos(math.radians(self._start[0]))
        return math.hypot(north, east)

    def update(self, context: Any) -> MissionStatus:
        if not self._enabled:
            context.logger.info(f"[{self.name}] skipped: the {FLAG} flag is off")
            return MissionStatus.SUCCESS
        state = context.mavros_state
        mode = getattr(state, "mode", None)
        if mode != "GUIDED":
            # Not ours to undo: the fence's action, a failsafe, or the safety pilot's switch. Nothing is commanded,
            # no mode is asked for, and the mission does not fail: the sequence's failure policy would ask for RTL.
            if not self._taken:
                context.clear_all_setpoints()
                if self._held_since is not None:
                    context.logger.info(f"[{self.name}] the mode is {mode or 'unknown'}: on the way home after the hold at the fence. The mission commands nothing")
                elif self._running:
                    context.logger.warn(f"[{self.name}] the mode is {mode or 'unknown'} after {self.demo.run_m:.1f} m of the run: the fence acted, or the vehicle was taken. The mission commands nothing")
                else:
                    context.logger.warn(f"[{self.name}] the mode is {mode or 'unknown'}, not GUIDED, before the run began: taken by the pilot or a failsafe. The mission commands nothing")
                self._taken = True
            self._stopping = None  # whoever has the vehicle has it: a failure that was being stopped for is theirs now
            if state is not None and getattr(state, "connected", False) and getattr(state, "armed", True) is False and mode == self._rtl_mode:
                context.logger.info(f"[{self.name}] over: the vehicle is disarmed, in {mode}")
                return MissionStatus.SUCCESS
            return MissionStatus.RUNNING  # in the air under somebody else, or landed by the pilot in a mode of their own: quiet
        pose = self._fresh_pose(context)
        if self._stopping is not None:
            # A failure, being stopped for: the hold stays commanded, and then the failure policy has the vehicle.
            reason, since = self._stopping
            if context.seconds_since(since) >= STOP_S:
                context.logger.error(f"[{self.name}] failed: {reason}")
                return MissionStatus.FAILURE
            return MissionStatus.RUNNING
        if self._held_since is not None and not self._taken:
            # Held at the fence and RTL asked for: the vehicle stays where it is until the flight controller takes it.
            if context.seconds_since(self._held_since) > RTL_WAIT_S:
                return self._stop(context, pose, f"held at the fence, and the flight controller did not take {self._rtl_mode} within {RTL_WAIT_S:.0f} s")
            return MissionStatus.RUNNING
        if self._taken:
            # Handed back in the air. The run does not start again under whoever just let go: the vehicle is
            # stopped where it is, and the failure policy asks for the way home.
            self._taken = False
            return self._stop(context, pose, "GUIDED again after the vehicle was taken: the run is not repeated")
        if pose is None:
            # No position to measure the run by: nothing is flown on it.
            if self._yaw_deg is not None:
                context.set_local_velocity_setpoint(0.0, 0.0, 0.0, yaw_deg=self._yaw_deg)
            if context.seconds_since(self._pose_seen) > NO_POSE_LIMIT_S:
                # Zero velocity has been commanded all this while: there is nothing more to stop.
                context.logger.error(f"[{self.name}] failed: no local pose of this moment for {NO_POSE_LIMIT_S:.0f} s")
                return MissionStatus.FAILURE
            return MissionStatus.WAITING
        position = pose.pose.position
        outage_s = context.seconds_since(self._pose_seen)
        self._pose_seen = context.now()
        if self._yaw_deg is None:
            self._yaw_deg = _yaw_deg(pose.pose.orientation)  # the heading the mission began with is kept: the vehicle does not turn into the run
        elif outage_s > self._fd.MAX_TICK_GAP_S:
            return self._stop(context, pose, f"no local pose of this moment for {outage_s:.1f} s in the middle of the demonstration")
        if not self._start_checked:
            # The run's lengths are from the surveyed takeoff point: it is flown only from there.
            away_m = self._from_start_m(context)
            if away_m is None:
                if context.seconds_since(self._entered) > NO_POSE_LIMIT_S:
                    return self._stop(context, pose, "no position from the flight controller to check the takeoff point by")
                context.set_local_velocity_setpoint(0.0, 0.0, 0.0, yaw_deg=self._yaw_deg)
                return MissionStatus.WAITING
            if away_m > START_TOLERANCE_M:
                return self._stop(context, pose, f"the vehicle is {away_m:.0f} m from the takeoff point the run's lengths are measured from (at most {START_TOLERANCE_M:g} m): the run is not flown")
            self._start_checked = True
        command = self.demo.update(context.now().nanoseconds * 1.0e-9, float(position.x), float(position.y))
        if command.held:
            # The flight controller holds the vehicle at the fence with the velocity still commanded: that is the
            # demonstration. The vehicle is still ours, so the hold and the way home are ours to ask for.
            context.set_local_position_setpoint(float(position.x), float(position.y), float(position.z), yaw_deg=self._yaw_deg)
            context.logger.info(f"[{self.name}] held at the fence: {command.reason}. Asking for {self._rtl_mode}")
            context.request_mode_change(self._rtl_mode)
            self._held_since = context.now()
            return MissionStatus.RUNNING
        if command.failed:
            # Still under command, so the stop is ours to give; the failure policy then asks for the way home.
            return self._stop(context, pose, command.reason)
        if command.running and not self._running:
            context.logger.info(f"[{self.name}] run: velocity ({command.east_mps:+.2f}, {command.north_mps:+.2f}) m/s east, north")
            self._running = True
        context.set_local_velocity_setpoint(command.east_mps, command.north_mps, 0.0, yaw_deg=self._yaw_deg)
        return MissionStatus.RUNNING
