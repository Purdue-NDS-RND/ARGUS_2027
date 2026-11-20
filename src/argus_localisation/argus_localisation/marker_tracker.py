"""The marker tracker (LOC-7): each frame's markers on the ground, into one track a marker. No ROS.

Behind the node flag `features.marker_tracker` (the node makes no tracker
with it off). The pipeline hands it every frame's ground points
(pipeline.GroundDetection); it keeps a track for each `marker` and each
`boundary_marker` it has seen.

  - **A sighting's noise** is the estimator's (FND-06): R = J Sigma J^T from
    projection v2's point, the box's size and the height's source
    (target_estimator.measurement_from_point). A track is a position and
    its 2 x 2 covariance, fused sighting by sighting, never surer than the
    estimator's floor, and growing by the estimator's process noise while
    it is not seen (our own position drifts between two passes).
  - **Assignment** is the Hungarian method on Mahalanobis distance, one
    class at a time, with two gates: 9.21 (chi-squared, 99%) and 1 m
    between the sighting and the track.
  - **A sighting no track took** starts a new track, unless it is within
    1 m of a track of its class: two markers are never that close, so it is
    that marker seen where the gate does not expect it. It is counted, and
    after five in a row the track is moved there (our position has slipped,
    and the track must go with it or it would never be matched again).
  - **States** (MarkerTrack.msg's): TENTATIVE until the third sighting,
    then CONFIRMED. A confirmed track not seen for a second is COASTING and
    is kept for good: it is the survey's result. A tentative track not seen
    for two seconds is LOST and is dropped a second later.
  - **Outside.** Once four boundary markers are confirmed, a marker track
    outside their hull (by more than half a metre) is marked outside and
    kept off the list (`confirmed_ids`). With fewer than four, nothing is;
    with more, the four seen most often are the boundary.
  - A box cut by the edge of the picture is not a sighting (the estimator's
    rule): it is the part of the marker in view. A step in the flight
    controller's position (`shift_all`, an EKF reset the node has seen)
    moves every track with it; a clock that steps back drops what was not
    confirmed and keeps the rest as not seen lately.
  - Every sighting of a track is kept (position and weight, the newest
    MAX_SIGHTINGS_KEPT). With `median_positions` (the node flag
    `features.loc_median_position`, LOC-9) the position a track reports is
    the weighted median of them (final_position.py); the filter's position
    goes on being what sightings are gated against.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from drone_mission_core import target_estimator
from drone_mission_core.assignment import assign_gated

from .final_position import final_position

GATE_NIS = target_estimator.GATE_NIS  # 9.21
GATE_M = 1.0
CONFIRM_SIGHTINGS = 3
COAST_AFTER_S = 1.0
TENTATIVE_LOST_AFTER_S = 2.0
DROP_LOST_AFTER_S = 1.0
RESET_AFTER_REJECTIONS = 5
OUTSIDE_MARGIN_M = 0.5
BOUNDARY_MARKERS = 4
MAX_SIGHTINGS_KEPT = 400
MAX_TRACKS = 200  # a field has tens of markers; beyond this something is making tracks of nothing
MIN_CORNER_APART_M = 2.0  # two boundary tracks closer than this are one corner seen twice
CLOCK_STEP_BACK_S = 0.5  # a frame stamped this far before the newest one seen: the clock has stepped back
TRACKED_CLASSES = ("marker", "boundary_marker")
TENTATIVE, CONFIRMED, COASTING, LOST = 0, 1, 2, 3  # MarkerTrack.msg
S = 1_000_000_000


@dataclass(frozen=True)
class Sighting:
    stamp_ns: int
    east_m: float
    north_m: float
    weight: float  # one over the trace of the sighting's noise: a surer sighting counts for more
    score: float


@dataclass(frozen=True)
class MarkerTrackState:
    """One track as it stands: what MarkerTrack.msg carries."""

    track_id: int
    class_id: str
    state: int
    east_m: float  # the filter's position: what sightings are gated against
    north_m: float
    reported_east_m: float  # the position that is reported: the filter's, or the weighted median of the sightings (LOC-9)
    reported_north_m: float
    covariance: Any  # 2 x 2, metres squared: the filter's
    sightings: int
    outside: bool
    updated_ns: int
    up_m: float = 0.0  # the height of the ground the marker was put on, in the local frame
    digit: int = -1  # unknown until the digit reader's votes are in (LOC-8)
    digit_log_prob: Tuple[float, ...] = (0.0,) * 10


class _Track:
    def __init__(self, track_id: int, measurement: Any, noise: np.ndarray, stamp_ns: int) -> None:
        self.track_id, self.class_id = track_id, measurement.class_id
        self.state = TENTATIVE
        self.x = np.array([measurement.east_m, measurement.north_m], dtype=float)
        self.p = target_estimator._floored(noise)
        self.updated_ns = stamp_ns
        self.last_hit_ns = stamp_ns
        self.hits = 0
        self.rejections = 0
        self.lost_since_ns: Optional[int] = None
        self.up_m = float(getattr(measurement, "up_m", 0.0))
        self.sightings: List[Sighting] = []
        self.median: Optional[Tuple[float, float]] = None  # of the sightings as they stand, once asked for; None when they have changed
        self._note(measurement, noise, stamp_ns)

    def _note(self, measurement: Any, noise: np.ndarray, stamp_ns: int) -> None:
        self.hits += 1
        self.rejections = 0
        self.up_m = float(getattr(measurement, "up_m", self.up_m))
        self.last_hit_ns = max(self.last_hit_ns, stamp_ns)
        self.sightings.append(Sighting(stamp_ns, float(measurement.east_m), float(measurement.north_m), 1.0 / max(float(np.trace(noise)), 1.0e-9), float(measurement.score)))
        del self.sightings[:-MAX_SIGHTINGS_KEPT]
        self.median = None
        if self.state in (TENTATIVE, LOST) and self.hits >= CONFIRM_SIGHTINGS:
            self.state, self.lost_since_ns = CONFIRMED, None
        elif self.state == LOST:
            self.state, self.lost_since_ns = TENTATIVE, None
        elif self.state == COASTING:
            self.state = CONFIRMED

    def covariance_at(self, stamp_ns: int) -> np.ndarray:
        ahead_s = max(0, stamp_ns - self.updated_ns) * 1.0e-9
        return self.p + np.eye(2) * target_estimator.PROCESS_NOISE_M2_PER_S * ahead_s

    def shift(self, d_east_m: float, d_north_m: float) -> None:
        """Step the track and every sighting kept."""
        self.x = self.x + np.array([d_east_m, d_north_m])
        self.sightings = [Sighting(s.stamp_ns, s.east_m + d_east_m, s.north_m + d_north_m, s.weight, s.score) for s in self.sightings]
        self.median = None

    def fuse(self, measurement: Any, noise: np.ndarray, stamp_ns: int) -> None:
        prior = self.covariance_at(stamp_ns)
        gain = prior @ np.linalg.inv(prior + noise)
        self.x = self.x + gain @ (np.array([measurement.east_m, measurement.north_m]) - self.x)
        self.p = target_estimator._floored((np.eye(2) - gain) @ prior)
        self.updated_ns = max(self.updated_ns, stamp_ns)
        self._note(measurement, noise, stamp_ns)

    def move_to(self, measurement: Any, noise: np.ndarray, stamp_ns: int) -> None:
        """Our own position has slipped: the track goes where the marker is now seen, as sure as that one sighting.

        The sightings kept were made before the slip: they step with the
        track, as they do for a reset the node saw (shift), so the position
        reported from them is where the marker is seen now.
        """
        self.shift(float(measurement.east_m) - float(self.x[0]), float(measurement.north_m) - float(self.x[1]))
        self.x = np.array([measurement.east_m, measurement.north_m], dtype=float)
        self.p = target_estimator._floored(noise)
        self.updated_ns = max(self.updated_ns, stamp_ns)
        self._note(measurement, noise, stamp_ns)


def _hull(points: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """The convex hull, anticlockwise (Andrew's monotone chain)."""
    points = sorted(set(points))
    if len(points) < 3:
        return list(points)

    def half(sequence):
        chain: List[Tuple[float, float]] = []
        for point in sequence:
            while len(chain) >= 2 and (chain[-1][0] - chain[-2][0]) * (point[1] - chain[-2][1]) - (chain[-1][1] - chain[-2][1]) * (point[0] - chain[-2][0]) <= 0.0:
                chain.pop()
            chain.append(point)
        return chain[:-1]

    return half(points) + half(points[::-1])


def outside_hull(point: Tuple[float, float], hull: Sequence[Tuple[float, float]], margin_m: float = OUTSIDE_MARGIN_M) -> bool:
    """Whether a point is outside an anticlockwise convex hull by more than the margin."""
    if len(hull) < 3:
        return False
    for (ax, ay), (bx, by) in zip(hull, list(hull[1:]) + [hull[0]]):
        length = math.hypot(bx - ax, by - ay)
        if length <= 0.0:
            continue
        inward = ((bx - ax) * (point[1] - ay) - (by - ay) * (point[0] - ax)) / length  # positive on the hull's inside
        if inward < -margin_m:
            return True
    return False


class MarkerTracker:
    def __init__(self, median_positions: bool = False) -> None:
        self.median_positions = bool(median_positions)  # report the weighted median of a track's sightings, not the filter's position
        self._tracks: Dict[int, _Track] = {}
        self._next_id = 1
        self.rejected = 0  # sightings within a metre of a track that the gate did not take
        self.unusable = 0  # sightings with no usable position or noise
        self.cut = 0  # boxes cut by the edge of the picture, which are not sightings
        self.refused_full = 0  # sightings that would have started a track beyond MAX_TRACKS
        self._newest_ns: Optional[int] = None

    # --- one frame -------------------------------------------------------------------

    def update(self, stamp_ns: int, ground: Sequence[Any]) -> None:
        """One frame's ground points (pipeline.GroundDetection): each has class_id, score, the box and projection v2's point."""
        stamp_ns = int(stamp_ns)
        if self._newest_ns is not None and self._newest_ns - stamp_ns > CLOCK_STEP_BACK_S * S:
            self.clock_stepped_back(stamp_ns)
        self._newest_ns = stamp_ns if self._newest_ns is None else max(self._newest_ns, stamp_ns)
        for class_id in TRACKED_CLASSES:
            found = []
            for item in ground:
                if item.class_id != class_id:
                    continue
                if getattr(item, "cut", False):
                    # The part of the marker in view, not the marker: its centre is wrong the same way frame after frame.
                    self.cut += 1
                    continue
                try:
                    measurement = target_estimator.measurement_from_point(class_id, item.point, item.detection.width, item.detection.height, item.score)
                    noise = target_estimator._usable(measurement)
                except (AttributeError, TypeError, ValueError):
                    noise = None
                if noise is None:
                    self.unusable += 1
                    continue
                found.append((measurement, noise))
            self._update_class(class_id, found, stamp_ns)
        self._age(stamp_ns)

    def _update_class(self, class_id: str, found: List[Tuple[Any, np.ndarray]], stamp_ns: int) -> None:
        tracks = [track for track in self._tracks.values() if track.class_id == class_id]
        cost = np.full((len(found), len(tracks)), np.inf)
        apart = np.full((len(found), len(tracks)), np.inf)
        for i, (measurement, noise) in enumerate(found):
            z = np.array([measurement.east_m, measurement.north_m])
            for j, track in enumerate(tracks):
                difference = z - track.x
                apart[i, j] = float(np.hypot(*difference))
                if apart[i, j] <= GATE_M:
                    cost[i, j] = float(difference @ np.linalg.inv(track.covariance_at(stamp_ns) + noise) @ difference)
        taken = set()
        for i, j in assign_gated(cost, GATE_NIS):
            tracks[j].fuse(*found[i], stamp_ns)
            taken.add(i)
        for i, (measurement, noise) in enumerate(found):
            if i in taken:
                continue
            near = int(np.argmin(apart[i])) if len(tracks) else -1
            if near >= 0 and apart[i, near] <= GATE_M:
                # Two markers are never a metre apart: this is that marker, seen where its track does not expect it.
                track = tracks[near]
                track.rejections += 1
                if track.rejections >= RESET_AFTER_REJECTIONS:
                    track.move_to(measurement, noise, stamp_ns)
                else:
                    self.rejected += 1
                continue
            if len(self._tracks) >= MAX_TRACKS:
                self.refused_full += 1
                continue
            self._tracks[self._next_id] = _Track(self._next_id, measurement, noise, stamp_ns)
            self._next_id += 1

    def _age(self, now_ns: int) -> None:
        for track in list(self._tracks.values()):
            unseen_s = (now_ns - track.last_hit_ns) * 1.0e-9
            if track.state == CONFIRMED and unseen_s > COAST_AFTER_S:
                track.state = COASTING
            elif track.state == TENTATIVE and unseen_s > TENTATIVE_LOST_AFTER_S:
                track.state, track.lost_since_ns = LOST, now_ns
            elif track.state == LOST and track.lost_since_ns is not None and (now_ns - track.lost_since_ns) * 1.0e-9 >= DROP_LOST_AFTER_S:
                del self._tracks[track.track_id]

    def shift_all(self, d_east_m: float, d_north_m: float) -> None:
        """The flight controller's position estimate stepped by this much (an EKF reset): every track, and every sighting kept, steps with it."""
        step = np.array([float(d_east_m), float(d_north_m)])
        if not np.all(np.isfinite(step)):
            raise ValueError("a position reset must be two finite numbers")
        for track in self._tracks.values():
            track.shift(float(step[0]), float(step[1]))

    def clock_stepped_back(self, now_ns: int) -> None:
        """The clock the frames are stamped on has stepped back to `now_ns`.

        A track's last sighting would otherwise lie in the future, and it
        would read as just seen for as long as the step was. Confirmed
        tracks are the survey's result and are kept, as not seen lately;
        the others are dropped.
        """
        now_ns = int(now_ns)
        for track in list(self._tracks.values()):
            if track.state not in (CONFIRMED, COASTING):
                del self._tracks[track.track_id]
                continue
            track.state, track.rejections = COASTING, 0
            track.last_hit_ns = now_ns - int(COAST_AFTER_S * S) - 1
            track.updated_ns = min(track.updated_ns, now_ns)
        self._newest_ns = now_ns

    # --- what it holds ---------------------------------------------------------------

    def _boundary_hull(self) -> List[Tuple[float, float]]:
        confirmed = [t for t in self._tracks.values() if t.class_id == "boundary_marker" and t.state in (CONFIRMED, COASTING)]
        if len(confirmed) < BOUNDARY_MARKERS:
            return []
        # There are four boundary markers. With more tracks than that confirmed, the four seen most often are they.
        best = sorted(confirmed, key=lambda t: (-t.hits, t.track_id))[:BOUNDARY_MARKERS]
        corners = [self._reported(t) for t in best]
        hull = _hull(corners)
        apart = min(math.dist(a, b) for index, a in enumerate(corners) for b in corners[index + 1 :])
        # Four corners, each a corner of the hull and none a second sighting of another: anything else is not the boundary yet.
        return hull if len(hull) == BOUNDARY_MARKERS and apart >= MIN_CORNER_APART_M else []

    def _reported(self, track: _Track) -> Tuple[float, float]:
        if self.median_positions:
            if track.median is None:  # worked out when the sightings have changed, not every time it is asked for
                track.median = final_position(track.sightings)
            if track.median is not None:
                return track.median
        return float(track.x[0]), float(track.x[1])

    def tracks(self) -> List[MarkerTrackState]:
        """Every track, by id. Whether a marker is outside is decided on the position that is reported."""
        hull = self._boundary_hull()
        states = []
        for track in sorted(self._tracks.values(), key=lambda item: item.track_id):
            reported = self._reported(track)
            states.append(
                MarkerTrackState(
                    track_id=track.track_id,
                    class_id=track.class_id,
                    state=track.state,
                    east_m=float(track.x[0]),
                    north_m=float(track.x[1]),
                    reported_east_m=reported[0],
                    reported_north_m=reported[1],
                    covariance=track.p.copy(),
                    sightings=track.hits,
                    outside=track.class_id == "marker" and outside_hull(reported, hull),
                    updated_ns=track.updated_ns,
                    up_m=track.up_m,
                )
            )
        return states

    def track_count(self) -> int:
        """Marker tracks that are not lost (the boundary markers are not counted)."""
        return sum(1 for track in self._tracks.values() if track.class_id == "marker" and track.state != LOST)

    def confirmed_ids(self) -> List[int]:
        """The marker tracks that are on the list: confirmed (seen lately or not) and not outside the boundary."""
        return [state.track_id for state in self.tracks() if state.class_id == "marker" and state.state in (CONFIRMED, COASTING) and not state.outside]

    def sightings_of(self, track_id: int) -> List[Sighting]:
        track = self._tracks.get(int(track_id))
        return [] if track is None else list(track.sightings)
