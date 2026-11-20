"""The localisation survey: a lawnmower over the search area (LOC-3).

Behind the mission flag `survey_search` (default false; needs
`use_height_estimator`). With the flag off the mission is skipped: it
returns SUCCESS on its first tick and commands nothing.

With it on, in order:

  PLAN            the search area's corners (latitude, longitude) become a
                  polygon in the local frame and the coverage planner
                  (LOC-2) lays the tracks for the survey height (LOC-1), the
                  survey speed and the mission's camera, begun at the end
                  nearest the vehicle. With a `flight_boundary` every
                  waypoint is 5 m inside it; a plan that would cross it, or
                  cannot be made, fails the mission here.
  CLIMB           to the first track's start at the survey height above the
                  ground there (`height_agl()`; half a metre more on a
                  height that is not the rangefinder's). Five seconds with
                  no height above the ground fail the mission.
  SET_SPEED       DO_CHANGE_SPEED to the survey speed. Refused, or not
                  answered within 5 s, three times, the mission fails: the
                  tracks are not flown at a speed nobody chose.
  START_PIPELINE  the localisation pipeline is started through its service.
                  With no service after `localisation_wait_s`, or no answer
                  within 5 s, the survey is flown without it, unless
                  `localisation_required`.
  SURVEY          the tracks, one waypoint after another, at one yaw (the
                  plan's, so the image's long side stays across the
                  tracks). Every tick the height guard is asked: RECOVER
                  stops the track where the vehicle is and climbs back;
                  ABORT ends the survey, the climb first. Between the
                  two the height setpoint follows the ground under the
                  vehicle; the flight controller flies a straight line to
                  a waypoint, so along a track that correction arrives only
                  as fast as the track is flown, and the guard is what
                  answers ground that comes close quickly.
  ENDING          the climb back if the guard asked for one, then the
                  pipeline is stopped, then SUCCESS: the tracks were flown,
                  the time budget ran out, or the guard ended the survey
                  low. The one FAILURE from here is a survey ended with no
                  height above the ground at all.

Heights are above the ground under the survey: `ground_below_home_m` says
how far that is below home, for the height estimator and for a barometer
height. A search area corner more than 1 km from the vehicle fails the
mission (coordinates the wrong way round), as does a way to the first track
that leaves the flight boundary. `speed_after_mps`, when given, is asked of
the flight controller once the survey is over.

The time budget (`time_budget_s`, 600 s) counts from the mission's start
and ends the mission in any state. Leaving the mission in mid-survey (the
executor's on_exit) stops the pipeline too.

The configuration is checked when the mission is constructed, so a search
area, height, speed or camera that cannot be flown stops the executor on
the ground.
"""

from __future__ import annotations

import math
from enum import Enum, auto
from typing import Any, List, Optional, Tuple

from drone_mission_core.features import get_feature_flag
from drone_mission_core.mission_api import BaseMission, MissionStatus
from drone_mission_core.registry import register_mission
from drone_mission_core.safety_limits import PHASE_SPEED_CAPS_MPS
from drone_mission_core.survey_search import SEARCH_FLAG

# The survey's own modules, loaded when a mission is built with the flag on (the planner brings numpy): with the
# flag off the executor imports what it imported in 2026.
coverage_planner: Any = None
survey_height: Any = None


def _load_survey_modules() -> None:
    global coverage_planner, survey_height
    from drone_mission_core import coverage_planner as planner
    from drone_mission_core import survey_height as heights

    coverage_planner, survey_height = planner, heights


DEFAULT_TIME_BUDGET_S = 600.0
MAX_GROUND_OFFSET_M = 20.0  # a ground further than this above or below home is a slip of the keyboard, not a field
MAX_SPEED_AFTER_MPS = 15.0  # nor does a sequence ask the flight controller for more than this after the survey
MAX_SURVEY_DISTANCE_M = 1000.0  # a search area corner further than this from the vehicle is a mistake in the coordinates
RANGE_GROUND_KEEP_S = 3.0  # how long the rangefinder's last word on the ground stands against the barometer's
TRANSIT_CHECK_STEP_M = 1.0  # the way to the first track is checked against the flight boundary every this far
WAYPOINT_RADIUS_M = 1.5  # this close to a waypoint is at it
HEIGHT_TOLERANCE_M = 0.5
SPEED_ATTEMPTS = 3
ANSWER_WAIT_S = 5.0  # how long a service call is waited for
STOP_WAIT_S = 3.0  # how long the end waits for the pipeline to say it has stopped
END_CLIMB_S = 30.0  # and for the climb back, when the guard ended the survey
LOG_EVERY_S = 2.0
RANGEFINDER = "rangefinder"


class SearchState(Enum):
    INIT = auto()
    PLAN = auto()
    CLIMB = auto()
    SET_SPEED = auto()
    START_PIPELINE = auto()
    SURVEY = auto()
    ENDING = auto()
    COMPLETE = auto()


def _corners(value: Any, key: str) -> List[Tuple[float, float]]:
    """A list of [latitude, longitude] corners from the config, or ValueError naming the key."""
    try:
        corners = [(float(item[0]), float(item[1])) for item in value]
    except (TypeError, ValueError, IndexError) as error:
        raise ValueError(f"{key} must be a list of [latitude, longitude] corners, got {value!r}") from error
    if len(corners) < 3 or not all(math.isfinite(v) for corner in corners for v in corner):
        raise ValueError(f"{key} needs at least three [latitude, longitude] corners, got {value!r}")
    return corners


def image_top_yaw_deg(mount: Any) -> float:
    """Where the image's top points in the body frame, anticlockwise from forward, from the camera's mount."""
    rotation = mount.rotation_body_from_optical
    forward, left = -float(rotation[0][1]), -float(rotation[1][1])  # the optical frame's -y is the image's top
    return math.degrees(math.atan2(left, forward))


@register_mission("search_pattern")
class SearchPatternMission(BaseMission):
    """Fly the planner's tracks over the search area with the localisation pipeline running."""

    def __init__(self, spec) -> None:
        super().__init__(spec)
        self._enabled = bool(get_feature_flag(spec.features, SEARCH_FLAG))
        self.plan: Any = None  # the coverage planner's CoveragePlan, once planned
        self.camera = None
        if not self._enabled:
            return
        _load_survey_modules()
        config = spec.config or {}
        try:
            self._area = _corners(config.get("search_area"), "search_area")
            self._boundary = None if config.get("flight_boundary") is None else _corners(config["flight_boundary"], "flight_boundary")
            self._survey_height_m = survey_height.survey_height(config)
            self._speed_mps = config.get("survey_speed_mps", coverage_planner.DEFAULT_SPEED_MPS)
            if isinstance(self._speed_mps, bool) or not isinstance(self._speed_mps, (int, float)) or not self._speed_mps > 0.0:
                raise ValueError(f"survey_speed_mps must be a speed above 0, got {self._speed_mps!r}")
            if self._speed_mps > PHASE_SPEED_CAPS_MPS["survey"]:
                raise ValueError(f"survey_speed_mps {self._speed_mps:g} is over the survey's speed cap of {PHASE_SPEED_CAPS_MPS['survey']:g} m/s")
            self._frame_rate_hz = float(config.get("frame_rate_hz", coverage_planner.DEFAULT_FRAME_RATE_HZ))
            self._time_budget_s = config.get("time_budget_s", DEFAULT_TIME_BUDGET_S)
            if isinstance(self._time_budget_s, bool) or not isinstance(self._time_budget_s, (int, float)) or not self._time_budget_s > 0.0:
                raise ValueError(f"time_budget_s must be a time above 0, got {self._time_budget_s!r}")
            self._localisation_required = bool(config.get("localisation_required", False))
            self._localisation_wait_s = float(config.get("localisation_wait_s", 3.0))
            # How far the ground under the survey is below home (negative: above it): where the height estimator
            # expects the ground, and what a barometer height is counted from.
            self._ground_below_home_m = config.get("ground_below_home_m", 0.0)
            if isinstance(self._ground_below_home_m, bool) or not isinstance(self._ground_below_home_m, (int, float)) or not math.isfinite(self._ground_below_home_m):
                raise ValueError(f"ground_below_home_m must be a number of metres, got {self._ground_below_home_m!r}")
            if abs(self._ground_below_home_m) > MAX_GROUND_OFFSET_M:
                raise ValueError(f"ground_below_home_m must be within {MAX_GROUND_OFFSET_M:g} m of home, got {self._ground_below_home_m!r}")
            # The ground speed asked of the flight controller when the survey is over; None leaves the survey's.
            self._speed_after_mps = config.get("speed_after_mps")
            if self._speed_after_mps is not None and (
                isinstance(self._speed_after_mps, bool) or not isinstance(self._speed_after_mps, (int, float)) or not self._speed_after_mps > 0.0
            ):
                raise ValueError(f"speed_after_mps must be a speed above 0 or null, got {self._speed_after_mps!r}")
            if self._speed_after_mps is not None and self._speed_after_mps > MAX_SPEED_AFTER_MPS:
                raise ValueError(f"speed_after_mps must be {MAX_SPEED_AFTER_MPS:g} m/s or less, got {self._speed_after_mps!r}")
        except ValueError as error:
            raise ValueError(f"mission '{spec.name}': {error}") from error
        from drone_mission_core.frame_projection import camera_from_config

        self.camera, mount = camera_from_config(config, spec.name)  # raises for a mode or airframe that cannot be loaded
        self._image_top_yaw_deg = image_top_yaw_deg(mount)

    # --- the mission API -------------------------------------------------------------

    def on_enter(self, context: Any) -> None:
        self.state = SearchState.INIT
        self.plan = None
        self.yaw_deg = 0.0
        self.track_index = 0  # the waypoint being flown to
        self.tracks_flown = 0
        self.recovering = False
        self._started = context.now()
        self._guard: Any = None  # the survey's height guard, made with the plan
        self._speed_restored = False
        self._speed_restore_refused = False
        self._target_up_m: Optional[float] = None
        self._recover_up_m: Optional[float] = None
        self._range_ground_at_s: Optional[float] = None  # when the rangefinder last gave the height, in the survey
        self._hold_point: Optional[Tuple[float, float]] = None
        self._speed_attempts = 0
        self._speed_pending = False
        self._speed_asked_at = None
        self._speed_set = False
        self._pipeline_asked = False  # asked to start: it is asked to stop at the end, whatever it answered
        self._pipeline_started = False
        self._pipeline_pending = False
        self._pipeline_asked_at = None
        self._pipeline_wait_start = None
        self._no_height_since = None
        self._end_reason = ""
        self._end_failed = False
        self._end_climb_to_m: Optional[float] = None
        self._end_start = None
        self._stop_requested_at = None
        self._stop_done = False
        self._last_log_s: dict = {}

    def on_exit(self, context: Any) -> None:
        if not self._enabled:
            return
        if self._pipeline_asked and self._stop_requested_at is None:
            # Left in mid-survey (a failure, a cancel): the pipeline does not go on running after its mission.
            self._stop_requested_at = context.now()
            context.set_localisation_enabled(False, done_callback=self._on_stop)
        self._restore_speed(context)
        context.clear_all_setpoints()

    def update(self, context: Any) -> MissionStatus:
        if not self._enabled:
            context.logger.info(f"[{self.name}] skipped: the {SEARCH_FLAG} flag is off")
            return MissionStatus.SUCCESS
        before_the_survey = self.state in (SearchState.CLIMB, SearchState.SET_SPEED, SearchState.START_PIPELINE)
        if before_the_survey and self._out_of_time(context):
            return self._end(context, f"the time budget of {self._time_budget_s:.0f} s ran out before the first track")
        handler = {
            SearchState.INIT: self._plan,
            SearchState.PLAN: self._plan,
            SearchState.CLIMB: self._climb,
            SearchState.SET_SPEED: self._set_speed,
            SearchState.START_PIPELINE: self._start_pipeline,
            SearchState.SURVEY: self._survey,
            SearchState.ENDING: self._ending,
            SearchState.COMPLETE: lambda _context: MissionStatus.SUCCESS,
        }[self.state]
        return handler(context)

    # --- helpers ---------------------------------------------------------------------

    def _to(self, state: SearchState, context: Any) -> None:
        context.logger.info(f"[{self.name}] State: {self.state.name} -> {state.name}")
        self.state = state

    def _say(self, context: Any, text: str, about: str, warn: bool = False) -> None:
        """Log a line, at most every LOG_EVERY_S for each thing it is `about`."""
        now_s = context.now().nanoseconds * 1.0e-9
        last = self._last_log_s.get(about)
        if last is not None and now_s - last < LOG_EVERY_S:
            return
        self._last_log_s[about] = now_s
        # Two lines, not one with the method chosen: rclpy lets a line of code log at one severity only.
        if warn:
            context.logger.warn(f"[{self.name}] {text}")
        else:
            context.logger.info(f"[{self.name}] {text}")

    def _here(self, context: Any) -> Tuple[float, float, float]:
        position = context.local_pose.pose.position
        return float(position.x), float(position.y), float(position.z)

    def _height(self, context: Any) -> Any:
        return context.height_agl(self._ground_below_home_m)

    def _restore_speed(self, context: Any) -> None:
        """Once, when the survey is over: the speed the sequence wants after it, if it names one and the survey's was set."""
        if self._speed_after_mps is None or self._speed_restored or not (self._speed_set or self._speed_pending):
            return
        self._speed_restored = True
        context.logger.info(f"[{self.name}] survey over: asking the flight controller for {self._speed_after_mps:g} m/s again")
        context.command_change_speed(float(self._speed_after_mps), done_callback=self._on_speed_restored)

    def _on_speed_restored(self, future: Any) -> None:
        try:
            result = future.result()
            self._speed_restore_refused = result is None or not bool(result.success)
        except Exception:  # noqa: BLE001  (a failed service call is a refusal)
            self._speed_restore_refused = True

    def _go(self, context: Any, east: float, north: float, up: float) -> None:
        context.set_local_position_setpoint(east, north, up, yaw_deg=self.yaw_deg)

    def _up_for(self, context: Any, wanted_m: float, margin_on_baro: bool = True) -> Optional[float]:
        """The local-frame height that puts the vehicle `wanted_m` above the ground here, or None with no height.

        On a height that is not the rangefinder's the survey is flown
        BARO_MARGIN_M higher, so that the guard, which counts such a height
        that much lower, is content at it.
        """
        height = self._height(context)
        if height is None:
            return None
        margin = survey_height.BARO_MARGIN_M if (margin_on_baro and height.source != RANGEFINDER) else 0.0
        return self._here(context)[2] + (wanted_m + margin - float(height.value_m))

    def _out_of_time(self, context: Any) -> bool:
        return context.seconds_since(self._started) >= self._time_budget_s

    def _end(self, context: Any, reason: str, failed: bool = False, climb_to_m: Optional[float] = None, warn: bool = True) -> MissionStatus:
        self._end_reason, self._end_failed, self._end_climb_to_m = reason, failed, climb_to_m
        self._end_start = context.now()
        east, north, _up = self._here(context)
        self._hold_point = (east, north)
        if warn:
            context.logger.warn(f"[{self.name}] survey ended early: {reason}")
        self._to(SearchState.ENDING, context)
        return self._ending(context)

    # --- the states ------------------------------------------------------------------

    def _plan(self, context: Any) -> MissionStatus:
        if self._out_of_time(context):
            context.logger.error(f"[{self.name}] no position to plan the survey from within the time budget of {self._time_budget_s:.0f} s")
            return MissionStatus.FAILURE
        if context.local_pose is None or context.global_gps is None:
            return MissionStatus.WAITING
        area = [context.latlon_to_local_enu(latitude, longitude) for latitude, longitude in self._area]
        boundary = None if self._boundary is None else [context.latlon_to_local_enu(latitude, longitude) for latitude, longitude in self._boundary]
        if any(point is None for point in area) or (boundary is not None and any(point is None for point in boundary)):
            return MissionStatus.WAITING  # no fix to place the corners by yet
        east, north, _up = self._here(context)
        furthest = max(math.dist((east, north), corner) for corner in area)
        if furthest > MAX_SURVEY_DISTANCE_M:
            context.logger.error(
                f"[{self.name}] a corner of search_area is {furthest:.0f} m from the vehicle (over {MAX_SURVEY_DISTANCE_M:g} m): "
                "the survey is not flown; are its latitude and longitude the right way round?"
            )
            return MissionStatus.FAILURE
        try:
            plan = coverage_planner.plan(
                area, self._survey_height_m, self.camera, speed_mps=float(self._speed_mps), frame_rate_hz=self._frame_rate_hz, boundary=boundary, start=(east, north)
            )
        except ValueError as error:
            context.logger.error(f"[{self.name}] the survey cannot be planned: {error}")
            return MissionStatus.FAILURE
        if plan.legs_outside_boundary or not plan.waypoints:
            context.logger.error(
                f"[{self.name}] the survey cannot be flown inside the flight boundary: {plan.legs_outside_boundary} leg(s) between tracks cross outside it"
            )
            return MissionStatus.FAILURE
        first = plan.waypoints[0]
        if boundary is None:
            context.logger.warn(f"[{self.name}] no flight_boundary in the sequence: the survey's waypoints are not checked against one")
        else:
            # The planner checks its own waypoints and the legs between them; the way there is the mission's.
            steps = max(1, int(math.ceil(math.dist((east, north), first) / TRANSIT_CHECK_STEP_M)))
            for index in range(steps + 1):
                point = (east + (first[0] - east) * index / steps, north + (first[1] - north) * index / steps)
                if not coverage_planner.inside_with_margin(point, boundary, 0.0):
                    context.logger.error(
                        f"[{self.name}] the way to the first track leaves the flight boundary {math.dist((east, north), point):.0f} m from here: the survey is not flown"
                    )
                    return MissionStatus.FAILURE
        self.plan = plan
        self.yaw_deg = plan.yaw_deg - self._image_top_yaw_deg
        self._guard = survey_height.SurveyHeightGuard(self._survey_height_m)
        note = ""
        if plan.pulled_back_m or plan.tracks_left_out:
            note = f"; kept inside the flight boundary by pulling track ends back {plan.pulled_back_m:.1f} m and leaving out {plan.tracks_left_out} track(s)"
        if plan.time_s > self._time_budget_s:
            note += f"; the tracks alone take {plan.time_s:.0f} s, more than the {self._time_budget_s:.0f} s budget"
        context.logger.info(
            f"[{self.name}] survey planned: {plan.tracks} tracks {plan.spacing_m:.1f} m apart at {self._survey_height_m:.2f} m, "
            f"{plan.length_m:.0f} m and {plan.time_s:.0f} s at {plan.speed_mps:g} m/s, yaw {self.yaw_deg:.0f} deg, "
            f"a marker in about {plan.frames_per_marker:.0f} frames{note}"
        )
        self._to(SearchState.CLIMB, context)
        return MissionStatus.RUNNING

    def _climb(self, context: Any) -> MissionStatus:
        first = self.plan.waypoints[0]
        east, north, up = self._here(context)
        target_up = self._up_for(context, self._survey_height_m)
        if target_up is None:
            if self._no_height_since is None:
                self._no_height_since = context.now()
                self._hold_point = (east, north)
            self._go(context, self._hold_point[0], self._hold_point[1], up)
            if context.seconds_since(self._no_height_since) >= survey_height.NO_HEIGHT_ABORT_S:
                context.logger.error(
                    f"[{self.name}] no height above the ground for {survey_height.NO_HEIGHT_ABORT_S:g} s before the first track: the survey is not flown"
                )
                return MissionStatus.FAILURE
            self._say(context, "no height above the ground: holding before the survey", "height", warn=True)
            return MissionStatus.RUNNING
        self._no_height_since = None
        self._target_up_m = target_up
        self._go(context, first[0], first[1], target_up)
        if math.dist((east, north), first) <= WAYPOINT_RADIUS_M and abs(up - target_up) <= HEIGHT_TOLERANCE_M:
            self._to(SearchState.SET_SPEED, context)
        return MissionStatus.RUNNING

    def _on_speed(self, future: Any, attempt: int) -> None:
        try:
            result = future.result()
            accepted = result is not None and bool(result.success)
        except Exception:  # noqa: BLE001  (a failed service call is a refusal)
            accepted = False
        if accepted:
            self._speed_set = True  # (also when the answer comes after its wait: the speed is set)
        if accepted or attempt == self._speed_attempts:
            self._speed_pending = False

    def _set_speed(self, context: Any) -> MissionStatus:
        first = self.plan.waypoints[0]
        self._go(context, first[0], first[1], self._target_up_m)
        if self._speed_set:
            self._to(SearchState.START_PIPELINE, context)
            return MissionStatus.RUNNING
        if self._speed_pending:
            if context.seconds_since(self._speed_asked_at) < ANSWER_WAIT_S:
                return MissionStatus.RUNNING
            self._speed_pending = False
            context.logger.warn(f"[{self.name}] the flight controller did not answer the survey speed command within {ANSWER_WAIT_S:g} s")
        if self._speed_attempts >= SPEED_ATTEMPTS:
            context.logger.error(
                f"[{self.name}] the survey speed {self._speed_mps:g} m/s was refused or not answered {SPEED_ATTEMPTS} times: the survey is not flown"
            )
            return MissionStatus.FAILURE
        if not context.command_service_ready():
            return MissionStatus.RUNNING
        self._speed_attempts += 1
        self._speed_pending = True
        self._speed_asked_at = context.now()
        attempt = self._speed_attempts
        context.command_change_speed(float(self._speed_mps), done_callback=lambda future: self._on_speed(future, attempt))
        return MissionStatus.RUNNING

    def _on_pipeline(self, future: Any) -> None:
        self._pipeline_pending = False
        try:
            result = future.result()
            self._pipeline_started = result is not None and bool(result.success)
        except Exception:  # noqa: BLE001
            self._pipeline_started = False

    def _start_pipeline(self, context: Any) -> MissionStatus:
        first = self.plan.waypoints[0]
        self._go(context, first[0], first[1], self._target_up_m)
        if self._pipeline_started:
            context.logger.info(f"[{self.name}] localisation pipeline started")
            self._to(SearchState.SURVEY, context)
            return MissionStatus.RUNNING
        why = "its service is not there"
        if self._pipeline_asked:
            if self._pipeline_pending and context.seconds_since(self._pipeline_asked_at) < ANSWER_WAIT_S:
                return MissionStatus.RUNNING
            why = f"its service did not answer within {ANSWER_WAIT_S:g} s" if self._pipeline_pending else "its service refused to start it"
        else:
            if self._pipeline_wait_start is None:
                self._pipeline_wait_start = context.now()
            if context.localisation_control_ready():
                self._pipeline_asked = True
                self._pipeline_pending = True
                self._pipeline_asked_at = context.now()
                context.set_localisation_enabled(True, done_callback=self._on_pipeline)
                return MissionStatus.RUNNING
            if context.seconds_since(self._pipeline_wait_start) < self._localisation_wait_s:
                return MissionStatus.RUNNING
        if self._localisation_required:
            context.logger.error(f"[{self.name}] the localisation pipeline cannot be started ({why}), and the sequence requires it: the survey is not flown")
            return self._end(context, f"the localisation pipeline cannot be started: {why}", failed=True, warn=False)
        context.logger.warn(f"[{self.name}] flying the survey without the localisation pipeline: {why}")
        self._to(SearchState.SURVEY, context)
        return MissionStatus.RUNNING

    def _survey(self, context: Any) -> MissionStatus:
        east, north, up = self._here(context)
        now_s = context.now().nanoseconds * 1.0e-9
        height = self._height(context)
        answer = self._guard.update(height, now_s)
        if answer.action == survey_height.ABORT:
            climb = None if answer.climb_to_m is None or height is None else up + (answer.climb_to_m - float(height.value_m))
            return self._end(context, answer.reason, failed=answer.climb_to_m is None, climb_to_m=climb)
        if answer.action == survey_height.RECOVER:
            if not self.recovering:
                self.recovering = True
                self._hold_point = (east, north)
                self._recover_up_m = up
                context.logger.warn(f"[{self.name}] survey waiting: {answer.reason}")
            if answer.climb_to_m is not None and height is not None:
                self._recover_up_m = up + (answer.climb_to_m - float(height.value_m))
            # With nothing to climb by this tick (no height, or one the guard does not yet believe), the climb
            # already asked for stands.
            self._go(context, self._hold_point[0], self._hold_point[1], self._recover_up_m)
            if self._out_of_time(context):
                # The budget ends a wait too, with the climb the wait had asked for first.
                return self._end(
                    context, f"the time budget of {self._time_budget_s:.0f} s ran out while the survey was waiting and climbing back", climb_to_m=self._recover_up_m
                )
            self._say(context, f"survey waiting: {answer.reason}", "guard", warn=True)
            return MissionStatus.RUNNING
        if self.recovering:
            self.recovering = False
            # The track goes on from the height the recovery reached, not from the height that needed it.
            self._target_up_m = self._recover_up_m
            context.logger.info(f"[{self.name}] back at the survey height: going on with track {self.track_index // 2 + 1} of {self.plan.tracks}")
        if self._out_of_time(context):
            return self._end(context, f"the time budget of {self._time_budget_s:.0f} s ran out with {self.tracks_flown} of {self.plan.tracks} tracks flown")
        if height is not None and height.source == RANGEFINDER:
            self._range_ground_at_s = now_s
        # The ground's height under the survey, followed slowly: the survey height is above the ground here. For
        # RANGE_GROUND_KEEP_S after the rangefinder last gave the height the setpoint stands: the rangefinder
        # knew where the ground is here, and the barometer, which answers while the vehicle is tilted at a turn
        # or a start, only knows home.
        on_the_rangefinders_ground = (
            height is not None and height.source != RANGEFINDER and self._range_ground_at_s is not None and now_s - self._range_ground_at_s < RANGE_GROUND_KEEP_S
        )
        target_up = None if on_the_rangefinders_ground else self._up_for(context, self._survey_height_m)
        if target_up is not None:
            self._target_up_m += 0.05 * (target_up - self._target_up_m)
        waypoint = self.plan.waypoints[self.track_index]
        self._go(context, waypoint[0], waypoint[1], self._target_up_m)
        if math.dist((east, north), waypoint) <= WAYPOINT_RADIUS_M:
            if self.track_index % 2 == 1:
                self.tracks_flown += 1
            self.track_index += 1
            if self.track_index >= len(self.plan.waypoints):
                context.logger.info(
                    f"[{self.name}] survey finished: {self.tracks_flown} of {self.plan.tracks} tracks flown in "
                    f"{context.seconds_since(self._started):.0f} s, {self._guard.recoveries} climb(s) back to the survey height"
                )
                return self._end(context, "the tracks are flown", warn=False)
        return MissionStatus.RUNNING

    def _on_stop(self, future: Any) -> None:
        self._stop_done = True

    def _ending(self, context: Any) -> MissionStatus:
        east, north, up = self._here(context)
        hold = self._hold_point or (east, north)
        self._restore_speed(context)
        if self._speed_restore_refused:
            self._speed_restore_refused = False
            context.logger.warn(
                f"[{self.name}] the flight controller refused {self._speed_after_mps:g} m/s after the survey: the missions after it fly at the survey's {self._speed_mps:g} m/s"
            )
        if self._end_climb_to_m is not None:
            # The guard ended the survey low: the climb comes before anything else.
            self._go(context, hold[0], hold[1], self._end_climb_to_m)
            if up < self._end_climb_to_m - HEIGHT_TOLERANCE_M and context.seconds_since(self._end_start) < END_CLIMB_S:
                return MissionStatus.RUNNING
            self._end_climb_to_m = None
            self._target_up_m = up
        self._go(context, hold[0], hold[1], self._target_up_m if self._target_up_m is not None else up)
        if self._pipeline_asked and not self._stop_done:
            if self._stop_requested_at is None:
                self._stop_requested_at = context.now()
                context.set_localisation_enabled(False, done_callback=self._on_stop)
                return MissionStatus.RUNNING
            if context.seconds_since(self._stop_requested_at) < STOP_WAIT_S:
                return MissionStatus.RUNNING
            context.logger.warn(f"[{self.name}] the localisation pipeline did not say it had stopped within {STOP_WAIT_S:g} s")
        if self._end_failed:
            context.logger.error(f"[{self.name}] survey failed: {self._end_reason}")
            return MissionStatus.FAILURE
        self._to(SearchState.COMPLETE, context)
        return MissionStatus.SUCCESS
