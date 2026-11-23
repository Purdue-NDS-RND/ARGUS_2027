"""The GPS waypoint mission's 2027 behaviours (PLT-13), each behind a mission flag that defaults to false.

`gps_waypoint_mission.py` is the 2026 mission. With none of these flags on it runs as it did; with one on, its
`update()` hands over to `Route2027` here, which flies the same waypoints with the same arrival rule and adds:

  waypoint_timeout_hold   a waypoint not reached within its timeout is given up and the next flown to; a waypoint
                          with a hold is held for it, inside the arrival radius. `waypoint_timeout_s` (all
                          waypoints; none when absent), `waypoint_timeouts_s` and `waypoint_holds_s` (by waypoint
                          name) in the mission's config; `hold_time_s` stays the hold of every waypoint without one.
  route_optimiser         the waypoints are flown in the shortest order from where the vehicle is when the route
                          begins (gps_route.optimise_order).
  flying_start            a run-in point `flying_start_run_in_m` (30 m) before the first waypoint, on the line of
                          the first leg, is flown to first, so that the first waypoint is passed at speed. It is
                          not a waypoint of the course: it is not timed, held at, or listed in the summary.
  time_trial_auto         the route is uploaded as an AUTO mission (pass radius 2 m) and flown by the flight
                          controller; the mission watches the waypoints go by and takes the vehicle back to
                          GUIDED at the end. If the upload cannot be made, is refused, or AUTO is not taken, or
                          the flight controller makes no progress for `auto_stall_s` (20 s), the rest of the
                          route is flown in GUIDED, the 2026 way.

The mission still succeeds when a waypoint was given up: the summary says which, and the rest were flown.
The optimiser and the flying start are for when the organisers allow them (FEATURES.md, open question 16).
"""

from __future__ import annotations

import math
from typing import Any, List, Optional

from drone_mission_core import gps_route
from drone_mission_core.features import get_feature_flag
from drone_mission_core.mission_api import MissionStatus

TIMEOUT_HOLD_FLAG, AUTO_FLAG, OPTIMISER_FLAG, FLYING_START_FLAG = "waypoint_timeout_hold", "time_trial_auto", "route_optimiser", "flying_start"
FLAGS = (TIMEOUT_HOLD_FLAG, AUTO_FLAG, OPTIMISER_FLAG, FLYING_START_FLAG)
DEFAULT_RUN_IN_M = 30.0
MAX_RUN_IN_M = 100.0
AUTO_SERVICE_WAIT_S = 3.0  # how long the mission upload service is waited for
AUTO_ANSWER_WAIT_S = 5.0  # and its answer, and the mode change
DEFAULT_AUTO_STALL_S = 20.0
WAIT, PREPARE, GUIDED_ROUTE, AUTO_UPLOAD, AUTO_MODE, AUTO_FLY, AUTO_END, DONE = "WAIT", "PREPARE", "GUIDED", "AUTO_UPLOAD", "AUTO_MODE", "AUTO_FLY", "AUTO_END", "DONE"


def flags_on(spec: Any) -> List[str]:
    return [flag for flag in FLAGS if get_feature_flag(spec.features, flag)]


def _seconds(value: Any, what: str, allow_none: bool = False, minimum: float = 0.0) -> Optional[float]:
    if value is None and allow_none:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < minimum or (minimum == 0.0 and allow_none and value <= 0.0):
        raise ValueError(f"{what} must be a number of {'more than' if allow_none else 'at least'} {minimum:g}, got {value!r}")
    return float(value)


class Route2027:
    """The 2027 flight of a GPS waypoint mission's route."""

    def __init__(self, mission: Any) -> None:
        self.m = mission
        config = mission.spec.config or {}
        on = flags_on(mission.spec)
        self.timeout_hold, self.auto, self.optimise, self.flying = (flag in on for flag in FLAGS)
        names = {waypoint.name for waypoint in mission._waypoints}
        try:
            self._timeout_s = _seconds(config.get("waypoint_timeout_s"), "waypoint_timeout_s", allow_none=True)
            self._timeouts = {str(name): _seconds(value, f"waypoint_timeouts_s.{name}", allow_none=True) for name, value in (config.get("waypoint_timeouts_s") or {}).items()}
            self._holds = {str(name): _seconds(value, f"waypoint_holds_s.{name}") for name, value in (config.get("waypoint_holds_s") or {}).items()}
            unknown = sorted((set(self._timeouts) | set(self._holds)) - names)
            if unknown:
                raise ValueError(f"waypoint_timeouts_s and waypoint_holds_s name waypoint(s) the route does not have: {', '.join(unknown)}")
            self._run_in_m = _seconds(config.get("flying_start_run_in_m", DEFAULT_RUN_IN_M), "flying_start_run_in_m", minimum=1.0)
            if self._run_in_m > MAX_RUN_IN_M:
                raise ValueError(f"flying_start_run_in_m must be {MAX_RUN_IN_M:g} m or less, got {self._run_in_m:g}")
            self._auto_stall_s = _seconds(config.get("auto_stall_s", DEFAULT_AUTO_STALL_S), "auto_stall_s", minimum=1.0)
        except ValueError as error:
            raise ValueError(f"mission '{mission.name}': {error}") from error
        self.state = WAIT
        self.order: List[int] = list(range(len(mission._waypoints)))  # indices into the mission's waypoints, as flown
        self.index = 0  # the place in `order` being flown to
        self.run_in: Optional[tuple] = None  # (latitude, longitude, height) while it is still to be flown to
        self.reached: List[str] = []
        self.given_up: List[str] = []
        self.flown_by = "GUIDED"
        self.fallback_reason = ""
        self._clock: Optional[gps_route.WaypointClock] = None
        self._since = None
        self._pending: Optional[str] = None  # "ok" or "refused", from a service answer
        self._auto_seq = 0
        self._auto_progress_at = None
        self._mode_asked = False

    # --- helpers -------------------------------------------------------------------------

    def _waypoint(self):
        return self.m._waypoints[self.order[self.index]]

    def _here(self, context: Any):
        gps = context.global_gps
        return None if gps is None else (float(gps.latitude), float(gps.longitude))

    def _inside(self, context: Any, latitude: float, longitude: float, altitude_m: float) -> bool:
        here = self._here(context)
        if here is None or context.local_pose is None:
            return False
        return gps_route.distance_m(here, (latitude, longitude)) <= self.m._arrival_radius_m and abs(context.local_pose.pose.position.z - altitude_m) <= self.m._arrival_alt_tolerance_m

    def _go(self, context: Any, latitude: float, longitude: float, altitude_m: float) -> None:
        context.set_global_position_setpoint(latitude, longitude, altitude_m, yaw_deg=self.m._desired_yaw_deg, lock_yaw=False)

    def _new_clock(self) -> gps_route.WaypointClock:
        waypoint = self._waypoint()
        last = self.index == len(self.order) - 1
        hold = self._holds.get(waypoint.name, self.m._hold_time_s) if self.timeout_hold else self.m._hold_time_s
        if self.m._mode == self.m.MODE_CIRCUIT_TIME_TRIAL and last:
            hold = 0.0  # the circuit's clock stops at the last waypoint: nothing is held there (as in 2026)
        timeout = self._timeouts.get(waypoint.name, self._timeout_s) if self.timeout_hold else None
        return gps_route.WaypointClock(timeout, hold)

    def _log(self, context: Any, text: str, warn: bool = False) -> None:
        (context.logger.warn if warn else context.logger.info)(f"[{self.m.name}] {text}")

    def _to(self, context: Any, state: str) -> None:
        self._log(context, f"Route: {self.state} -> {state}")
        self.state, self._since, self._pending = state, context.now(), None

    def _passed(self, context: Any, timed_out: bool = False) -> None:
        """The waypoint being flown to is over: reached, or given up."""
        waypoint = self._waypoint()
        (self.given_up if timed_out else self.reached).append(waypoint.name)
        if timed_out:
            self._log(context, f"GPS waypoint #{self.order[self.index] + 1} ({waypoint.name}) not reached in its time: given up, on to the next", warn=True)
        else:
            self._log(context, f"Reached GPS waypoint #{self.order[self.index] + 1} ({waypoint.name})")
        if self.m._mode == self.m.MODE_CIRCUIT_TIME_TRIAL:
            if self.index == 0 and self.m._route_start_time is None:
                self.m._route_start_time = context.now()
                self._log(context, "Circuit timer started")
            if self.index == len(self.order) - 1 and self.m._route_end_time is None:
                self.m._route_end_time = context.now()
        self.index += 1
        self._clock = None

    def _finish(self, context: Any) -> MissionStatus:
        self.state = DONE
        self._log(context, f"Route flown by {self.flown_by}: {len(self.reached)} of {len(self.order)} waypoints reached" + (f"; given up: {', '.join(self.given_up)}" if self.given_up else ""))
        self.m._log_summary(context)
        return MissionStatus.SUCCESS

    # --- one tick ------------------------------------------------------------------------

    def update(self, context: Any) -> MissionStatus:
        handler = {WAIT: self._wait, PREPARE: self._prepare, GUIDED_ROUTE: self._guided, AUTO_UPLOAD: self._auto_upload, AUTO_MODE: self._auto_mode, AUTO_FLY: self._auto_fly, AUTO_END: self._auto_end}.get(self.state)
        return MissionStatus.SUCCESS if handler is None else handler(context)

    def _wait(self, context: Any) -> MissionStatus:
        if context.mavros_state is None or not context.mavros_state.connected or not self.m._gps_fix_valid(context) or context.local_pose is None:
            return MissionStatus.WAITING
        self.state = PREPARE
        return MissionStatus.RUNNING

    def _prepare(self, context: Any) -> MissionStatus:
        points = [(waypoint.latitude, waypoint.longitude) for waypoint in self.m._waypoints]
        if self.optimise:
            given = gps_route.route_length_m(points, self._here(context))
            self.order = gps_route.optimise_order(points, self._here(context))
            best = gps_route.route_length_m([points[i] for i in self.order], self._here(context))
            self._log(context, f"Route optimiser: order {'-'.join(str(i + 1) for i in self.order)}, {best:.0f} m against {given:.0f} m as given")
        if self.flying and len(self.order) >= 2:
            first, second = self.m._waypoints[self.order[0]], self.m._waypoints[self.order[1]]
            latitude, longitude = gps_route.run_in_point((first.latitude, first.longitude), (second.latitude, second.longitude), self._run_in_m)
            self.run_in = (latitude, longitude, first.altitude_m)
            self._log(context, f"Flying start: a run-in point {self._run_in_m:g} m before {first.name}")
        self._to(context, AUTO_UPLOAD if self.auto else GUIDED_ROUTE)
        return MissionStatus.RUNNING

    def _guided(self, context: Any) -> MissionStatus:
        if context.global_gps is None or context.local_pose is None:
            return MissionStatus.WAITING
        if self.run_in is not None:
            self._go(context, *self.run_in)
            if self._inside(context, *self.run_in):
                self.run_in = None  # at the run-in point: on to the first waypoint without stopping
            return MissionStatus.RUNNING
        if self.index >= len(self.order):
            return self._finish(context)
        waypoint = self._waypoint()
        self._go(context, waypoint.latitude, waypoint.longitude, waypoint.altitude_m)
        if self._clock is None:
            self._clock = self._new_clock()
        outcome = self._clock.update(context.now().nanoseconds * 1.0e-9, self._inside(context, waypoint.latitude, waypoint.longitude, waypoint.altitude_m))
        if outcome in (gps_route.REACHED, gps_route.TIMED_OUT):
            self._passed(context, timed_out=outcome == gps_route.TIMED_OUT)
            if self.index >= len(self.order):
                return self._finish(context)
        return MissionStatus.RUNNING

    # --- AUTO ----------------------------------------------------------------------------

    def _fall_back(self, context: Any, why: str) -> MissionStatus:
        self.fallback_reason = why
        self._log(context, f"AUTO not flown ({why}): the rest of the route in GUIDED", warn=True)
        self._to(context, GUIDED_ROUTE)
        return MissionStatus.RUNNING

    def _answer(self, what: str):
        def done(future: Any) -> None:
            try:
                response = future.result()
                ok = response is not None and bool(getattr(response, "success", getattr(response, "mode_sent", False)))
            except Exception:  # noqa: BLE001  (a service call that failed is a request not taken)
                ok = False
            if self.state == what:
                self._pending = "ok" if ok else "refused"

        return done

    def _auto_upload(self, context: Any) -> MissionStatus:
        waited = context.seconds_since(self._since)
        if self._pending is None and not self._mode_asked:
            ready = getattr(context, "mission_upload_ready", None)
            if ready is None or not ready():
                if waited >= AUTO_SERVICE_WAIT_S:
                    return self._fall_back(context, "the mission upload service is not there (MAVROS's waypoint plugin is not running)")
                return MissionStatus.RUNNING
            home = self._here(context)
            route = ([self.run_in] if self.run_in is not None else []) + [(w.latitude, w.longitude, w.altitude_m) for w in (self.m._waypoints[i] for i in self.order)]
            context.upload_mission(gps_route.auto_items(home, route), self._answer(AUTO_UPLOAD))
            self._mode_asked = True
            self._since = context.now()
            return MissionStatus.RUNNING
        if self._pending == "refused" or (self._pending is None and waited >= AUTO_ANSWER_WAIT_S):
            return self._fall_back(context, "the mission upload was refused" if self._pending else "the mission upload was not answered")
        if self._pending == "ok":
            self._to(context, AUTO_MODE)
            context.clear_global_position_setpoint()
            context.request_mode_change("AUTO", self._answer(AUTO_MODE))
        return MissionStatus.RUNNING

    def _auto_mode(self, context: Any) -> MissionStatus:
        if getattr(context.mavros_state, "mode", "") == "AUTO":
            self.flown_by = "AUTO"
            self._auto_progress_at = context.now()
            self._to(context, AUTO_FLY)
            return MissionStatus.RUNNING
        if self._pending == "refused" or context.seconds_since(self._since) >= AUTO_ANSWER_WAIT_S:
            return self._fall_back(context, "the flight controller did not take AUTO")
        return MissionStatus.RUNNING

    def _auto_fly(self, context: Any) -> MissionStatus:
        """The flight controller flies the items; the mission counts the waypoints as they are passed."""
        offset = 1 + (1 if self.run_in is not None else 0)  # item 0 is home; a run-in point is item 1
        reached_seq = context.mission_reached_seq()
        while self.index < len(self.order):
            waypoint = self._waypoint()
            here = self._here(context)
            near = here is not None and gps_route.distance_m(here, (waypoint.latitude, waypoint.longitude)) <= gps_route.AUTO_PASS_RADIUS_M
            if (reached_seq is not None and reached_seq >= self.index + offset) or near:
                self._passed(context)
                self._auto_progress_at = context.now()
            else:
                break
        if self.index >= len(self.order):
            self._to(context, AUTO_END)
            context.request_mode_change("GUIDED", self._answer(AUTO_END))
            return MissionStatus.RUNNING
        if getattr(context.mavros_state, "mode", "") != "AUTO":
            return self._to_guided(context, f"the mode left AUTO for {getattr(context.mavros_state, 'mode', '?')}")
        if context.seconds_since(self._auto_progress_at) >= self._auto_stall_s:
            return self._to_guided(context, f"no waypoint passed in {self._auto_stall_s:g} s of AUTO")
        return MissionStatus.RUNNING

    def _to_guided(self, context: Any, why: str) -> MissionStatus:
        """Out of AUTO in mid-route: back to GUIDED, and the waypoints still to come are flown the 2026 way."""
        self.run_in = None
        context.request_mode_change("GUIDED")
        waypoint = self._waypoint()
        self._go(context, waypoint.latitude, waypoint.longitude, waypoint.altitude_m)
        return self._fall_back(context, why)

    def _auto_end(self, context: Any) -> MissionStatus:
        last = self.m._waypoints[self.order[-1]]
        self._go(context, last.latitude, last.longitude, last.altitude_m)  # held at the last waypoint once GUIDED has it
        if getattr(context.mavros_state, "mode", "") == "GUIDED":
            return self._finish(context)
        if context.seconds_since(self._since) >= AUTO_ANSWER_WAIT_S:
            context.request_mode_change("GUIDED", self._answer(AUTO_END))  # asked again: the next mission needs GUIDED
            self._since = context.now()
        return MissionStatus.RUNNING
