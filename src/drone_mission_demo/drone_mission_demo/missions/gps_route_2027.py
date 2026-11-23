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

The mission still succeeds when a waypoint was given up: the summary says which, and the rest were flown. A route
on which no waypoint at all was reached fails.

The mission never takes the vehicle back from somebody else. In AUTO, a mode that is neither AUTO nor GUIDED is the
safety pilot's or a failsafe's: the mission then commands nothing and asks for no mode, and carries on only if
AUTO or GUIDED comes back. It asks for GUIDED only out of the AUTO it asked for itself (at the route's end, when
AUTO stalls, and when the mission is left while AUTO is still flying), three times at most.

With a `flight_boundary` in the mission's config (latitude and longitude corners) the run-in point and a reordered
route must stay 5 m inside it: a run-in point that would not is left out, and an order whose legs would not is not
flown (the given order is). Without one the mission says so: only the flight controller's fence holds them.
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
GUIDED_ASKS = 3  # how often GUIDED is asked for out of our own AUTO before the mission fails
GUIDED_ASK_EVERY_S = 2.0
STALL_PROGRESS_M = 1.0  # AUTO is making progress while the vehicle gets this much nearer the waypoint within auto_stall_s
WAIT, PREPARE, GUIDED_ROUTE, AUTO_UPLOAD, AUTO_MODE, AUTO_FLY, AUTO_PAUSED, AUTO_END, DONE = "WAIT", "PREPARE", "GUIDED", "AUTO_UPLOAD", "AUTO_MODE", "AUTO_FLY", "AUTO_PAUSED", "AUTO_END", "DONE"


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
            self._boundary = None
            if config.get("flight_boundary") is not None:
                corners = config["flight_boundary"]
                if not isinstance(corners, (list, tuple)) or len(corners) < 3 or not all(isinstance(c, (list, tuple)) and len(c) == 2 and all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in c) for c in corners):
                    raise ValueError("flight_boundary must be three or more [latitude, longitude] corners")
                self._boundary = [(float(c[0]), float(c[1])) for c in corners]
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
        self._auto_progress_at = None
        self._auto_nearest_m = math.inf
        self._mode_asked = False
        self._auto_requested = False  # AUTO was asked for by this mission: only then does it ever ask for GUIDED back
        self._guided_asks = 0
        self._guided_asked_at = None
        self._failed_reason = ""

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
        # Two calls, not one that picks its severity: rclpy lets a line of code log at one severity only, and raises
        # at the first warning otherwise (the first flight with a timeout ended there).
        if warn:
            context.logger.warn(f"[{self.m.name}] {text}")
        else:
            context.logger.info(f"[{self.m.name}] {text}")

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

    def _mode(self, context: Any) -> str:
        return str(getattr(context.mavros_state, "mode", "") or "")

    def _fail(self, context: Any, why: str) -> MissionStatus:
        self.state, self._failed_reason = DONE, why
        context.logger.error(f"[{self.m.name}] {why}")
        self.m._log_summary(context)
        return MissionStatus.FAILURE

    def _take_guided_back(self, context: Any) -> Optional[MissionStatus]:
        """Out of the AUTO this mission asked for: ask for GUIDED, again every GUIDED_ASK_EVERY_S, GUIDED_ASKS times at
        most. None while asking; a failure when AUTO will not hand back."""
        if self._guided_asked_at is not None and context.seconds_since(self._guided_asked_at) < GUIDED_ASK_EVERY_S:
            return None
        if self._guided_asks >= GUIDED_ASKS:
            return self._fail(context, f"the flight controller stayed in AUTO after GUIDED was asked for {GUIDED_ASKS} times")
        self._guided_asks += 1
        self._guided_asked_at = context.now()
        context.request_mode_change("GUIDED")
        return None

    def on_exit(self, context: Any) -> None:
        """The mission is left (done, failed or cancelled): an AUTO mission of ours must not fly on by itself."""
        if self._auto_requested and self._mode(context) == "AUTO":
            self._log(context, "Left with the flight controller still in the AUTO it was asked for: asking for GUIDED", warn=True)
            context.request_mode_change("GUIDED")

    def _finish(self, context: Any) -> MissionStatus:
        if not self.reached:
            return self._fail(context, f"no waypoint of the route was reached ({len(self.given_up)} given up)")
        self.state = DONE
        self._log(context, f"Route flown by {self.flown_by}: {len(self.reached)} of {len(self.order)} waypoints reached" + (f"; given up: {', '.join(self.given_up)}" if self.given_up else ""))
        self.m._log_summary(context)
        return MissionStatus.SUCCESS

    # --- one tick ------------------------------------------------------------------------

    def update(self, context: Any) -> MissionStatus:
        handler = {WAIT: self._wait, PREPARE: self._prepare, GUIDED_ROUTE: self._guided, AUTO_UPLOAD: self._auto_upload, AUTO_MODE: self._auto_mode, AUTO_FLY: self._auto_fly, AUTO_PAUSED: self._auto_paused, AUTO_END: self._auto_end}.get(self.state)
        if handler is None:
            return MissionStatus.FAILURE if self._failed_reason else MissionStatus.SUCCESS
        return handler(context)

    def _wait(self, context: Any) -> MissionStatus:
        if context.mavros_state is None or not context.mavros_state.connected or not self.m._gps_fix_valid(context) or context.local_pose is None:
            return MissionStatus.WAITING
        self.state = PREPARE
        return MissionStatus.RUNNING

    def _prepare(self, context: Any) -> MissionStatus:
        points = [(waypoint.latitude, waypoint.longitude) for waypoint in self.m._waypoints]
        if (self.optimise or self.flying) and self._boundary is None:
            self._log(context, "no flight_boundary is given: the reordered route and the run-in point are held only by the flight controller's fence", warn=True)
        if self.optimise and len(points) > gps_route.EXACT_UP_TO:
            # (The search for more waypoints than this is not bounded to one tick of the executor.)
            self._log(context, f"Route optimiser: {len(points)} waypoints is more than the {gps_route.EXACT_UP_TO} it orders in flight: the given order is flown", warn=True)
        elif self.optimise:
            given = gps_route.route_length_m(points, self._here(context))
            order = gps_route.optimise_order(points, self._here(context))
            best = gps_route.route_length_m([points[i] for i in order], self._here(context))
            if gps_route.way_inside_boundary([self._here(context)] + [points[i] for i in order], self._boundary):
                self.order = order
                self._log(context, f"Route optimiser: order {'-'.join(str(i + 1) for i in self.order)}, {best:.0f} m against {given:.0f} m as given")
            else:
                self._log(context, "Route optimiser: the shortest order leaves the flight boundary: the given order is flown", warn=True)
        if self.flying and len(self.order) >= 2:
            first, second = self.m._waypoints[self.order[0]], self.m._waypoints[self.order[1]]
            latitude, longitude = gps_route.run_in_point((first.latitude, first.longitude), (second.latitude, second.longitude), self._run_in_m)
            if gps_route.way_inside_boundary([self._here(context), (latitude, longitude), (first.latitude, first.longitude)], self._boundary):
                self.run_in = (latitude, longitude, first.altitude_m)
                self._log(context, f"Flying start: a run-in point {self._run_in_m:g} m before {first.name}")
            else:
                self._log(context, "Flying start: the run-in point, or the way to it, leaves the flight boundary: no run-in", warn=True)
        self._to(context, AUTO_UPLOAD if self.auto else GUIDED_ROUTE)
        return MissionStatus.RUNNING

    def _guided(self, context: Any) -> MissionStatus:
        if context.global_gps is None or context.local_pose is None:
            return MissionStatus.WAITING
        if self._auto_requested and self._mode(context) == "AUTO":
            # Our own AUTO is still (or, answered late, only now) the mode: nothing of the GUIDED route is flown, or
            # counted, until GUIDED is back.
            failed = self._take_guided_back(context)
            return failed if failed is not None else MissionStatus.RUNNING
        if self.run_in is not None:
            self._go(context, *self.run_in)
            if self._inside(context, *self.run_in):
                self.run_in = None  # at the run-in point: on to the first waypoint without stopping
            return MissionStatus.RUNNING
        if self.index >= len(self.order):
            return self._finish(context)
        waypoint = self._waypoint()
        self._go(context, waypoint.latitude, waypoint.longitude, waypoint.altitude_m)
        if self._mode(context) not in ("GUIDED", ""):
            # Somebody else has the vehicle (the pilot's LOITER, a failsafe): the setpoint stands, as in 2026, and
            # the waypoint's clock does not run. A waypoint is not given up for time the mission did not fly.
            self._clock = None
            return MissionStatus.RUNNING
        if self._clock is None:
            self._clock = self._new_clock()
            if self.timeout_hold and (self._clock.timeout_s is not None or self._clock.hold_s > 0.0):
                self._log(context, f"To {waypoint.name}: hold {self._clock.hold_s:g} s, " + ("no timeout" if self._clock.timeout_s is None else f"given up after {self._clock.timeout_s:g} s"))
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
            self._auto_requested = True
            context.request_mode_change("AUTO", self._answer(AUTO_MODE))
        return MissionStatus.RUNNING

    def _auto_mode(self, context: Any) -> MissionStatus:
        if self._mode(context) == "AUTO":
            self.flown_by = "AUTO"
            self._auto_progress_at, self._auto_nearest_m = context.now(), math.inf
            self._to(context, AUTO_FLY)
            return MissionStatus.RUNNING
        if self._pending == "refused" or context.seconds_since(self._since) >= AUTO_ANSWER_WAIT_S:
            return self._fall_back(context, "the flight controller did not take AUTO")
        return MissionStatus.RUNNING

    def _auto_fly(self, context: Any) -> MissionStatus:
        """The flight controller flies the items; the mission counts the waypoints as they are passed."""
        offset = 1 + (1 if self.run_in is not None else 0)  # item 0 is home; a run-in point is item 1
        reached_seq = context.mission_reached_seq()
        here = self._here(context)
        while self.index < len(self.order):
            waypoint = self._waypoint()
            away = math.inf if here is None else gps_route.distance_m(here, (waypoint.latitude, waypoint.longitude))
            if (reached_seq is not None and reached_seq >= self.index + offset) or away <= gps_route.AUTO_PASS_RADIUS_M:
                self._passed(context)
                self._auto_progress_at, self._auto_nearest_m = context.now(), math.inf
            else:
                if away < self._auto_nearest_m - STALL_PROGRESS_M:  # nearer the waypoint: progress, however long the leg
                    self._auto_progress_at, self._auto_nearest_m = context.now(), away
                break
        if self.index >= len(self.order):
            self._to(context, AUTO_END)
            return MissionStatus.RUNNING
        mode = self._mode(context)
        if mode == "GUIDED":
            self.run_in = None
            return self._fall_back(context, "the mode is GUIDED again")
        if mode != "AUTO":
            # Not ours to undo: the safety pilot's switch, or a failsafe. Nothing is commanded and no mode asked for.
            self._log(context, f"The mode is {mode or 'unknown'}, not the AUTO this mission asked for: taken by the pilot or a failsafe. The mission commands nothing", warn=True)
            context.clear_global_position_setpoint()
            self._to(context, AUTO_PAUSED)
            return MissionStatus.RUNNING
        if context.seconds_since(self._auto_progress_at) >= self._auto_stall_s:
            self.run_in = None
            return self._fall_back(context, f"no progress toward {self._waypoint().name} in {self._auto_stall_s:g} s of AUTO")
        return MissionStatus.RUNNING

    def _auto_paused(self, context: Any) -> MissionStatus:
        """Somebody else has the vehicle. The mission waits, commanding nothing, for AUTO or GUIDED to come back."""
        mode = self._mode(context)
        if mode == "AUTO":
            self._auto_progress_at, self._auto_nearest_m = context.now(), math.inf
            self._to(context, AUTO_FLY)
        elif mode == "GUIDED":
            self.run_in = None
            return self._fall_back(context, "the mode is GUIDED again")
        return MissionStatus.RUNNING

    def _auto_end(self, context: Any) -> MissionStatus:
        mode = self._mode(context)
        if mode == "AUTO":
            failed = self._take_guided_back(context)  # the route is flown: our AUTO is handed back, asked for three times at most
            return failed if failed is not None else MissionStatus.RUNNING
        if mode == "GUIDED":
            last = self.m._waypoints[self.order[-1]]
            self._go(context, last.latitude, last.longitude, last.altitude_m)  # held at the last waypoint
        else:
            self._log(context, f"The route is flown and the mode is {mode or 'unknown'}, not ours: no mode is asked for", warn=True)
        return self._finish(context)
