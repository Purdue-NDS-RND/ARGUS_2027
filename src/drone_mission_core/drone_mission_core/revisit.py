"""Re-visiting uncertain markers after the survey pattern (LOC-4): which tracks, in what order, and the hover over each.

Behind the mission flag `survey_revisit`, which defaults to false and needs
`survey_search` (it is a part of the survey mission, flown when the tracks
are done and before the pipeline is stopped). With the flag off the survey
ends as it did.

A track needs another look when its digit was not read (the vote abstained)
or it has been seen fewer than MIN_SIGHTINGS times. Tracks the tracker has
put outside the field, and tracks it has lost, are left.

Where the vehicle may be sent is not perception's to say. `Bounds` holds the
survey's own search area and the flight boundary, and every point and every
leg is put to it:

  - a point must be inside the search area, or within AREA_SLACK_M of it (a
    marker lies in the area; its track may sit a little outside it), and, when
    the sequence gives a flight boundary, GEOFENCE_MARGIN_M inside that;
  - a leg must stay within LEG_SLACK_M of the search area along its whole
    length, and, with a boundary, inside it by the margin less LEG_MARGIN_GIVE_M
    (the survey planner's own rule for the legs between its tracks).

`choose` orders the visits nearest first, each kept only while its leg and
its hover fit the time that is left. `Revisit` flies the round one `update` a
tick: each leg is checked again from where the vehicle really is when it
starts, the track's place is taken from the tracker's newest word while the
tracker still holds it, and a track the tracker has dropped, a place that no
longer passes the bounds, or a point not reached in time is passed over.
Over each point the vehicle holds for HOVER_S within ARRIVE_RADIUS_M, at the
survey height.

`TrackFeed` and `attach_tracks` are the executor's side: the tracker's tracks
from /loc/tracks, aged by the message's own stamp.

No ROS imports; the standard library only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from .features import get_feature_flag

FLAG = "survey_revisit"
TRACKS_TOPIC = "/loc/tracks"  # argus_localisation.track_messages.TRACKS_TOPIC (a test holds the two together)
HOVER_S = 3.0  # the spec: hover 3 s over each
MIN_SIGHTINGS = 8  # fewer than this is "few sightings": a pass over a marker at the survey's speed gives 10 to 20
UNKNOWN_DIGIT = -1  # argus_localisation.digit_vote.UNKNOWN
ARRIVE_RADIUS_M = 1.0
MAX_CANDIDATES = 40  # the round is planned in one executor tick: no more tracks than this are weighed
MAX_VISITS = 20  # nor more visits than this flown; a field has about ten markers
GEOFENCE_MARGIN_M = 5.0  # as the survey's own tracks keep (safety_limits.GEOFENCE_MARGIN_M)
LEG_MARGIN_GIVE_M = 0.5  # a leg may come this much nearer the boundary than a point (coverage_planner's rule for its legs)
AREA_SLACK_M = 2.0  # a track this far outside the search area is still a marker of it
LEG_SLACK_M = 3.0  # and a leg may pass this far outside it (round a corner of an area that is not convex)
LEG_CHECK_STEP_M = 2.0
LEG_ALLOWANCE = 2.0  # a leg is given this many times its length at the survey speed, and LEG_EXTRA_S, before it is given up
LEG_EXTRA_S = 10.0
TRACKS_FRESH_S = 5.0  # the tracks must be this new, by their own stamp, when the round is planned and while it is flown

Point = Tuple[float, float]


@dataclass(frozen=True)
class TrackView:
    """What the round needs of a track (argus_interfaces/MarkerTrack)."""

    track_id: int
    east_m: float
    north_m: float
    digit: int
    sightings: int
    alive: bool  # tentative, confirmed or coasting: the tracker still holds it
    outside: bool


@dataclass(frozen=True)
class Visit:
    track_id: int
    east_m: float
    north_m: float
    why: str


def why_revisit(track: TrackView, min_sightings: int = MIN_SIGHTINGS) -> Optional[str]:
    """Why a track needs another look, or None."""
    if not track.alive or track.outside or not (math.isfinite(track.east_m) and math.isfinite(track.north_m)):
        return None
    if track.digit == UNKNOWN_DIGIT:
        return f"digit not read ({track.sightings} sightings)"
    if track.sightings < min_sightings:
        return f"{track.sightings} sightings"
    return None


def _signed_inside_m(point: Point, polygon: Sequence[Point]) -> float:
    """How far inside the polygon a point is: its distance to the nearest edge, negative when it is outside."""
    east, north = point
    inside = False
    nearest = math.inf
    for (e1, n1), (e2, n2) in zip(polygon, list(polygon[1:]) + [polygon[0]]):
        if (n1 > north) != (n2 > north) and east < e1 + (north - n1) * (e2 - e1) / (n2 - n1):
            inside = not inside
        length_sq = (e2 - e1) ** 2 + (n2 - n1) ** 2
        along = 0.0 if length_sq == 0 else max(0.0, min(1.0, ((east - e1) * (e2 - e1) + (north - n1) * (n2 - n1)) / length_sq))
        nearest = min(nearest, math.hypot(east - (e1 + along * (e2 - e1)), north - (n1 + along * (n2 - n1))))
    return nearest if inside else -nearest


class Bounds:
    """Where a re-visit may go: the survey's search area, and the flight boundary when the sequence gives one."""

    def __init__(self, area: Sequence[Point], boundary: Optional[Sequence[Point]] = None, margin_m: float = GEOFENCE_MARGIN_M) -> None:
        if len(area) < 3:
            raise ValueError("the search area needs three corners or more")
        self.area = [(float(e), float(n)) for e, n in area]
        self.boundary = None if boundary is None or len(boundary) < 3 else [(float(e), float(n)) for e, n in boundary]
        self.margin_m = float(margin_m)

    def point_problem(self, point: Point) -> Optional[str]:
        """Why the vehicle is not sent to this point, or None."""
        if not (math.isfinite(point[0]) and math.isfinite(point[1])):
            return "its place is not a number"
        outside = -_signed_inside_m(point, self.area)
        if outside > AREA_SLACK_M:
            return f"{outside:.0f} m outside the search area"
        if self.boundary is not None and _signed_inside_m(point, self.boundary) < self.margin_m:
            return f"not {self.margin_m:g} m inside the flight boundary"
        return None

    def leg_problem(self, start: Point, end: Point) -> Optional[str]:
        """Why the straight way from start to end is not flown, or None. Looked at every LEG_CHECK_STEP_M."""
        steps = max(1, int(math.ceil(math.dist(start, end) / LEG_CHECK_STEP_M)))
        for index in range(steps + 1):
            point = (start[0] + (end[0] - start[0]) * index / steps, start[1] + (end[1] - start[1]) * index / steps)
            if -_signed_inside_m(point, self.area) > LEG_SLACK_M:
                return "the straight way to it leaves the search area"
            if self.boundary is not None and _signed_inside_m(point, self.boundary) < self.margin_m - LEG_MARGIN_GIVE_M:
                return "the straight way to it comes too near the flight boundary"
        return None


def choose(tracks: Iterable[TrackView], here: Point, time_left_s: float, speed_mps: float, bounds: Bounds, min_sightings: int = MIN_SIGHTINGS, max_visits: int = MAX_VISITS,
           hover_s: float = HOVER_S) -> Tuple[List[Visit], List[str]]:  # fmt: skip
    """(the visits in the order to fly them, what was left out and why).

    Nearest first from `here`; a visit is kept only while the round so far, with its leg and its hover, fits
    `time_left_s`. The digit that was not read comes before the one seen too few times when both are as near.
    No more than MAX_CANDIDATES tracks are weighed (the unread first, then the least seen).
    """
    left_out: List[str] = []
    wanted = []
    for track in tracks:
        why = why_revisit(track, min_sightings)
        if why is None:
            continue
        problem = bounds.point_problem((track.east_m, track.north_m))
        if problem is not None:
            left_out.append(f"track {track.track_id} ({why}): {problem}")
            continue
        wanted.append((Visit(track.track_id, track.east_m, track.north_m, why), track.sightings))
    wanted.sort(key=lambda item: (not item[0].why.startswith("digit"), item[1], item[0].track_id))
    if len(wanted) > MAX_CANDIDATES:
        left_out.append(f"{len(wanted) - MAX_CANDIDATES} more uncertain track(s): only {MAX_CANDIDATES} are weighed")
    candidates = [visit for visit, _sightings in wanted[:MAX_CANDIDATES]]
    visits: List[Visit] = []
    at, spent = here, 0.0
    speed = max(speed_mps, 0.1)
    while candidates and len(visits) < max_visits:
        nearest = min(candidates, key=lambda visit: (round(math.dist(at, (visit.east_m, visit.north_m)), 1), not visit.why.startswith("digit"), visit.track_id))
        point = (nearest.east_m, nearest.north_m)
        cost = math.dist(at, point) / speed + hover_s
        candidates.remove(nearest)
        if spent + cost > time_left_s:
            left_out.append(f"track {nearest.track_id} ({nearest.why}): no time for it ({cost:.0f} s, {max(0.0, time_left_s - spent):.0f} s left)")
            continue
        problem = bounds.leg_problem(at, point)
        if problem is not None:
            left_out.append(f"track {nearest.track_id} ({nearest.why}): {problem}")
            continue
        visits.append(nearest)
        at, spent = point, spent + cost
    left_out += [f"track {visit.track_id} ({visit.why}): more than {max_visits} visits" for visit in candidates]
    return visits, left_out


@dataclass(frozen=True)
class Step:
    target: Optional[Point]  # where to fly or hold; None when the round is over
    done: bool
    say: Optional[str] = None  # a line for the log when something happened this tick


class Revisit:
    """The round: fly to each visit, hold HOVER_S within ARRIVE_RADIUS_M of it, go on. One `update` a tick."""

    def __init__(self, visits: Sequence[Visit], speed_mps: float, bounds: Bounds, hover_s: float = HOVER_S, radius_m: float = ARRIVE_RADIUS_M) -> None:
        self.visits = list(visits)
        self.speed_mps = max(float(speed_mps), 0.1)
        self.bounds = bounds
        self.hover_s, self.radius_m = float(hover_s), float(radius_m)
        self.index = 0
        self.hovered: List[int] = []  # the tracks hovered over for the full time
        self.passed_over: List[int] = []  # not reached, dropped by the tracker, or no longer inside the bounds
        self._point: Optional[Point] = None  # where the visit being flown is, by the tracker's newest word
        self._leg_point: Optional[Point] = None  # where that visit was when its leg was last checked
        self._leg_started_s: Optional[float] = None
        self._leg_limit_s = 0.0
        self._hover_started_s: Optional[float] = None

    def restart_leg(self) -> None:
        """The vehicle was taken off the leg for a while (the height guard's climb): the leg's time and its check start again."""
        self._leg_started_s = self._hover_started_s = None

    def update(self, here: Point, now_s: float, tracks: Optional[Iterable[TrackView]] = None) -> Step:
        """`tracks`: the tracker's newest tracks, when the caller has them: the visit follows its track's place, and is passed over if the tracker dropped it."""
        said: List[str] = []
        places: Optional[Dict[int, TrackView]] = None if tracks is None else {track.track_id: track for track in tracks}
        while self.index < len(self.visits):
            visit = self.visits[self.index]
            if places is not None:
                track = places.get(visit.track_id)
                if track is None or not track.alive:
                    said.append(self._pass_over(f"track {visit.track_id}: the tracker no longer holds it"))
                    continue
                moved = (track.east_m, track.north_m)
                if self._point is None or math.dist(moved, self._point) > 0.05:
                    problem = self.bounds.point_problem(moved)
                    if problem is not None:
                        said.append(self._pass_over(f"track {visit.track_id}: now {problem}"))
                        continue
                    if self._leg_point is not None and math.dist(moved, self._leg_point) > self.radius_m:
                        self.restart_leg()  # it has moved more than the hover's radius from where the leg was checked to, in one step or many: a new leg, checked again
                    self._point = moved
            if self._point is None:
                self._point = (visit.east_m, visit.north_m)
            away = math.dist(here, self._point)
            if self._leg_started_s is None:
                problem = self.bounds.leg_problem(here, self._point)  # from where the vehicle really is
                if problem is not None:
                    said.append(self._pass_over(f"track {visit.track_id}: {problem} from here"))
                    continue
                self._leg_started_s, self._leg_point = now_s, self._point
                self._leg_limit_s = LEG_ALLOWANCE * away / self.speed_mps + LEG_EXTRA_S
                said.append(f"re-visit {self.index + 1} of {len(self.visits)}: track {visit.track_id} ({visit.why}), {away:.0f} m away")
            if away <= self.radius_m:
                if self._hover_started_s is None:
                    self._hover_started_s = now_s
                elif now_s - self._hover_started_s >= self.hover_s:
                    self.hovered.append(visit.track_id)
                    said.append(f"hovered {self.hover_s:g} s over track {visit.track_id}")
                    self._advance()
                    continue
            else:
                self._hover_started_s = None  # drifted off it: the hover is counted again from the start
                if now_s - self._leg_started_s > self._leg_limit_s:
                    said.append(self._pass_over(f"track {visit.track_id} not reached in {self._leg_limit_s:.0f} s ({away:.1f} m away): going on"))
                    continue
            return Step(self._point, False, "; ".join(said) or None)
        return Step(None, True, "; ".join(said) or None)

    def _advance(self) -> None:
        self.index += 1
        self._point = self._leg_point = self._leg_started_s = self._hover_started_s = None

    def _pass_over(self, say: str) -> str:
        self.passed_over.append(self.visits[self.index].track_id)
        self._advance()
        return say


def views(message: Any, alive_states: Sequence[int]) -> List[TrackView]:
    """The tracks of an argus_interfaces/MarkerTrackArray as TrackViews. `alive_states` are the MarkerTrack states the tracker still holds."""
    return [TrackView(int(track.track_id), float(track.position_enu.x), float(track.position_enu.y), int(track.digit), int(track.sightings), int(track.state) in alive_states, bool(track.outside))
            for track in message.tracks]  # fmt: skip


class TrackFeed:
    """The newest tracks the tracker published. Kept as the message came and read only when asked: the executor's thread does no work for a message nobody reads."""

    def __init__(self, alive_states: Sequence[int]) -> None:
        self._alive = tuple(alive_states)
        self._message: Any = None
        self._stamp_ns = 0

    def store(self, message: Any, arrived_ns: int) -> None:
        """Keep a message. Its age is counted from its own stamp (the frame the tracks are of); from its arrival only when it carries none."""
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        own_ns = 0 if stamp is None else int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        self._message, self._stamp_ns = message, own_ns if own_ns > 0 else int(arrived_ns)

    def newest(self, now_ns: int) -> Optional[Tuple[List[TrackView], float]]:
        """(the tracks, how old they are in seconds), or None before any came. A stamp ahead of the clock is as old as it is ahead: not to be trusted either."""
        if self._message is None:
            return None
        return views(self._message, self._alive), abs(int(now_ns) - self._stamp_ns) * 1.0e-9


def attach_tracks(node: Any, specs: Iterable[Any]) -> Optional[TrackFeed]:
    """The executor's side: a subscription to the tracker's tracks, or None when no mission has the flag on (then nothing is subscribed)."""
    if not any(get_feature_flag(spec.features, FLAG) for spec in specs):
        return None
    from argus_interfaces.msg import MarkerTrack, MarkerTrackArray

    feed = TrackFeed((MarkerTrack.TENTATIVE, MarkerTrack.CONFIRMED, MarkerTrack.COASTING))
    node._marker_tracks_subscription = node.create_subscription(MarkerTrackArray, TRACKS_TOPIC, lambda message: feed.store(message, node.get_clock().now().nanoseconds), 10)
    node.get_logger().info(f"Survey re-visit on ({FLAG}): the tracker's tracks are read from {TRACKS_TOPIC}")
    return feed
