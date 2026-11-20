"""The target estimator (FND-06): one small Kalman filter per target on the ground. numpy only, no ROS.

Every target is a point that does not move: its state is (east, north) in
MAVROS's local frame. What moves is our own idea of where we are, so the
filter lets its certainty leak away slowly (process noise) rather than model
any motion.

One frame's detections arrive together, already projected to the ground by
projection v2 (FND-05), each with its measurement noise R = J Sigma J^T: the
projection's Jacobians times the pixel, attitude and height noise
(`measurement_noise`). `TargetEstimator.update` then

  1. gives the detections of each class to that class's tracks with the
     Hungarian method on the Mahalanobis distance, a pair being allowed only
     inside the gate (normalised innovation squared at most 9.21);
  2. updates each track with its detection;
  3. deals with what is left over. A detection near a track that got nothing
     this frame is that target seen outside its gate: it is counted, and the
     fifth in a row moves the track there (the target really is there). A
     detection far from every track is a new target and starts a TENTATIVE
     track.

Track life, on the stamps of the frames and the time asked about:

  TENTATIVE -> CONFIRMED   3 hits within 1 s
  CONFIRMED -> COASTING    0.3 s without a hit (a hit brings it straight back)
  COASTING  -> LOST        2 s without a hit
  LOST                     kept 30 s so it can be found again under the same
                           id; a hit makes it TENTATIVE and it must confirm again

What is read out (`estimates`, `primary`) is given for the actuation time,
now + 0.08 s. A static target has not moved by then; only its covariance has
grown by the process noise.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .assignment import assign_gated

# The spec's numbers (FEATURES.md, FND-06).
GATE_NIS = 9.21  # chi-squared, 2 degrees of freedom, 99%
PROCESS_NOISE_M2_PER_S = 0.0025  # absorbs slow drift in our own position
COVARIANCE_FLOOR_M2 = 0.02**2  # the filter is never surer than 2 cm
RESET_AFTER_REJECTIONS = 5
CONFIRM_HITS = 3
CONFIRM_WINDOW_S = 1.0
COAST_AFTER_S = 0.3
LOST_AFTER_S = 2.0
KEEP_LOST_S = 30.0
MAX_TRACKS_PER_CLASS = 8
ACTUATION_DELAY_S = 0.08  # detection to flight-controller response; PLT-05 measures it on the bench
PIXEL_SIGMA_PX = 2.0
ATTITUDE_SIGMA_RAD = math.radians(1.0)
HEIGHT_SIGMA_M = {"rangefinder": 0.05, "home": 0.5}  # "home" is the baro height above the take-off point
# Height above the ground of the plane each class is seen in. "handle" is the
# bucket's handle, for the detector that will report it (REC lane).
PLANE_HEIGHT_M = {"bullseye": 0.0, "green_x": 0.0, "marker": 0.0, "boundary_marker": 0.0, "bucket": 0.2, "handle": 0.3}

# Chosen here; the spec leaves them open (see PROGRESS.md, Decisions).
SMALL_BOX_PX = 32.0  # a box narrower than this has a less certain centre: sigma grows as 32 / side
MAX_PIXEL_SIGMA_GROWTH = 4.0  # up to 8 px
CUT_BOX_MARGIN_PX = 2.0  # a box reaching this close to the edge of the image is cut off by it
SAME_TARGET_RADIUS_M = 2.0  # a detection this close to a track is that target, inside the gate or not
TENTATIVE_DROP_S = 1.0  # a track that never confirmed is forgotten after this long without a hit

TENTATIVE, CONFIRMED, COASTING, LOST = "TENTATIVE", "CONFIRMED", "COASTING", "LOST"
STATES = (TENTATIVE, CONFIRMED, COASTING, LOST)
_RANK = {CONFIRMED: 0, COASTING: 1, TENTATIVE: 2, LOST: 3}  # the order `primary` prefers


def plane_height_m(class_id: str) -> float:
    """Height above the ground of the plane a class is projected onto."""
    if class_id not in PLANE_HEIGHT_M:
        raise ValueError(f"class {class_id!r} has no plane height (known: {', '.join(sorted(PLANE_HEIGHT_M))})")
    return PLANE_HEIGHT_M[class_id]


def box_is_cut(cx: float, cy: float, box_width: float, box_height: float, image_width: float, image_height: float) -> bool:
    """Whether a box, taken as centred on (cx, cy), reaches the edge of the image: the target is partly out of view.

    Such a box is the visible part of the target, not the target, and its
    centre can be wrong by up to half the target's size, the same way frame
    after frame. It is not a measurement of where the target is, and the feed
    does not give it to the filter.
    """
    half_width, half_height = 0.5 * float(box_width), 0.5 * float(box_height)
    return (
        cx - half_width <= CUT_BOX_MARGIN_PX
        or cy - half_height <= CUT_BOX_MARGIN_PX
        or cx + half_width >= float(image_width) - CUT_BOX_MARGIN_PX
        or cy + half_height >= float(image_height) - CUT_BOX_MARGIN_PX
    )


def pixel_sigma_px(box_width: float, box_height: float) -> float:
    """Pixel noise of a box's centre: 2 px, more for a box under 32 px on its short side, at most 8 px."""
    side = max(min(float(box_width), float(box_height)), 1.0e-6)
    return PIXEL_SIGMA_PX * min(max(SMALL_BOX_PX / side, 1.0), MAX_PIXEL_SIGMA_GROWTH)


def measurement_noise(point: Any, box_width: float, box_height: float, height_source: str, pixel_sigma: Optional[float] = None) -> np.ndarray:
    """R = J Sigma J^T for a projected detection: `point` is projection v2's GroundPoint, with its Jacobians.

    `pixel_sigma` is the detector's own figure for its centre's noise, in
    pixels, when it has one; otherwise the box's size decides.
    """
    pixel = (pixel_sigma_px(box_width, box_height) if pixel_sigma is None else max(float(pixel_sigma), 0.1)) ** 2
    height = HEIGHT_SIGMA_M.get(height_source, HEIGHT_SIGMA_M["home"]) ** 2  # an unknown source is trusted least
    d_pixel, d_attitude, d_height = np.asarray(point.d_pixel), np.asarray(point.d_attitude), np.asarray(point.d_height)
    return pixel * d_pixel @ d_pixel.T + ATTITUDE_SIGMA_RAD**2 * d_attitude @ d_attitude.T + height * np.outer(d_height, d_height)


@dataclass(frozen=True)
class Measurement:
    """One detection on the ground."""

    class_id: str
    east_m: float
    north_m: float
    covariance: Any  # 2x2, metres squared: R
    score: float = 1.0
    up_m: float = 0.0  # height of the plane it was projected onto, in the local frame


def measurement_from_point(
    class_id: str, point: Any, box_width: float, box_height: float, score: float, height_source: Optional[str] = None, pixel_sigma_px: Optional[float] = None
) -> Measurement:
    """The measurement for a detection that projection v2 put at `point`."""
    source = height_source if height_source is not None else getattr(point, "height_source", "")
    return Measurement(
        class_id=class_id,
        east_m=float(point.east_m),
        north_m=float(point.north_m),
        covariance=measurement_noise(point, box_width, box_height, source, pixel_sigma_px),
        score=float(score),
        up_m=float(point.up_m),
    )


@dataclass(frozen=True)
class TargetEstimate:
    """One track as it is read out, for the time `for_ns`."""

    track_id: int
    class_id: str
    state: str
    east_m: float
    north_m: float
    up_m: float
    covariance: Tuple[Tuple[float, float], Tuple[float, float]]  # at for_ns
    sigma_m: float  # the larger standard deviation of the covariance
    age_s: float  # from the last hit's frame to the time the question was asked
    hits: int
    score: float  # the detector's confidence in the last hit
    last_hit_ns: int
    for_ns: int


@dataclass
class FrameReport:
    """What one frame's detections did."""

    accepted: int = 0  # inside a track's gate, and used
    rejected: int = 0  # that target, but outside its gate
    resets: int = 0  # tracks moved after five rejections in a row
    started: int = 0  # new tracks
    ignored: int = 0  # a second detection beside a track that already had one
    untracked: int = 0  # a new target with the class already at its limit
    invalid: int = 0  # not a usable measurement
    # The normalised innovation squared of every measurement that was set against a track, accepted or
    # rejected: what a replay judges the noise model by (the mean should be near 2, about 1% above the gate).
    nis: List[float] = field(default_factory=list)


class _Track:
    def __init__(self, track_id: int, measurement: Measurement, noise: np.ndarray, stamp_ns: int) -> None:
        self.track_id = track_id
        self.class_id = measurement.class_id
        self.state = TENTATIVE
        self.x = np.array([measurement.east_m, measurement.north_m], dtype=float)
        self.p = _floored(noise)
        self.updated_ns = stamp_ns  # the time the covariance stands for
        self.last_hit_ns = stamp_ns
        self.recent_hits: Deque[int] = deque([stamp_ns], maxlen=CONFIRM_HITS)
        self.hits = 1
        self.rejections = 0
        self.ever_confirmed = False
        self.lost_since_ns: Optional[int] = None
        self.up_m = measurement.up_m
        self.score = measurement.score

    def covariance_at(self, stamp_ns: int) -> np.ndarray:
        ahead_s = max(0, stamp_ns - self.updated_ns) * 1.0e-9
        return self.p + np.eye(2) * PROCESS_NOISE_M2_PER_S * ahead_s

    def hit(self, measurement: Measurement, stamp_ns: int) -> None:
        self.rejections = 0
        self.hits += 1
        self.up_m, self.score = measurement.up_m, measurement.score
        self.updated_ns = max(self.updated_ns, stamp_ns)
        if self.state == LOST:
            # Found again: the same track, but it has to prove itself again.
            self.state = TENTATIVE
            self.lost_since_ns = None
            self.recent_hits.clear()
        if stamp_ns >= self.last_hit_ns:
            self.last_hit_ns = stamp_ns
        if stamp_ns not in self.recent_hits:
            # A frame counts once toward confirming, however many detectors saw the target in it: two
            # detectors working on the same pixels are one sighting, not two.
            self.recent_hits.append(stamp_ns)
        if self.state == COASTING:
            self.state = CONFIRMED
        elif self.state == TENTATIVE and len(self.recent_hits) == CONFIRM_HITS:
            if max(self.recent_hits) - min(self.recent_hits) <= int(CONFIRM_WINDOW_S * 1.0e9):
                self.state = CONFIRMED
                self.ever_confirmed = True


def _floored(covariance: np.ndarray) -> np.ndarray:
    """The covariance, symmetric, with no direction surer than the floor."""
    symmetric = 0.5 * (covariance + covariance.T)
    values, vectors = np.linalg.eigh(symmetric)
    return (vectors * np.maximum(values, COVARIANCE_FLOOR_M2)) @ vectors.T


def _usable(measurement: Measurement) -> Optional[np.ndarray]:
    """The measurement's noise as a 2x2 matrix, or None if the measurement cannot be used."""
    try:
        noise = np.array(measurement.covariance, dtype=float)
        position_ok = math.isfinite(float(measurement.east_m)) and math.isfinite(float(measurement.north_m))
    except (TypeError, ValueError):
        return None
    if not position_ok or noise.shape != (2, 2) or not np.all(np.isfinite(noise)) or not isinstance(measurement.class_id, str):
        return None
    noise = 0.5 * (noise + noise.T)
    if np.linalg.eigvalsh(noise).min() < 0.0:
        return None
    return noise


class TargetEstimator:
    """Every track of every class. `update` takes one frame's measurements; `estimates` and `primary` read out."""

    def __init__(self) -> None:
        self._tracks: List[_Track] = []
        self._next_id = 1

    # --- measurements in ---------------------------------------------------------------

    def update(self, measurements: Sequence[Measurement], stamp_ns: int) -> FrameReport:
        """Take the detections of the frame captured at `stamp_ns`."""
        report = FrameReport()
        stamp_ns = int(stamp_ns)
        # A track silent for long enough before this frame has coasted or been
        # lost by then, whether or not anyone asked in between.
        self.advance(stamp_ns)
        by_class: Dict[str, List[Tuple[Measurement, np.ndarray]]] = {}
        for measurement in measurements:
            noise = _usable(measurement)
            if noise is None:
                report.invalid += 1
            else:
                by_class.setdefault(measurement.class_id, []).append((measurement, noise))
        for class_id, found in by_class.items():
            self._update_class(class_id, found, stamp_ns, report)
        return report

    def _update_class(self, class_id: str, found: List[Tuple[Measurement, np.ndarray]], stamp_ns: int, report: FrameReport) -> None:
        tracks = [track for track in self._tracks if track.class_id == class_id]
        predicted = [track.covariance_at(stamp_ns) for track in tracks]
        cost = np.full((len(tracks), len(found)), np.inf)
        for row, track in enumerate(tracks):
            for column, (measurement, noise) in enumerate(found):
                innovation = np.array([measurement.east_m, measurement.north_m]) - track.x
                cost[row, column] = float(innovation @ np.linalg.solve(predicted[row] + noise, innovation))
        pairs = assign_gated(cost, GATE_NIS)
        taken_rows = {row for row, _ in pairs}
        taken_columns = {column for _, column in pairs}
        for row, column in pairs:
            track, (measurement, noise) = tracks[row], found[column]
            prior = predicted[row]
            gain = prior @ np.linalg.inv(prior + noise)
            track.x = track.x + gain @ (np.array([measurement.east_m, measurement.north_m]) - track.x)
            keep = np.eye(2) - gain
            track.p = _floored(keep @ prior @ keep.T + gain @ noise @ gain.T)  # Joseph form: stays symmetric and positive
            track.hit(measurement, stamp_ns)
            report.accepted += 1
            report.nis.append(float(cost[row, column]))

        # What is left over, nearest pairs first.
        left = [column for column in range(len(found)) if column not in taken_columns]
        distance = {
            (row, column): math.hypot(found[column][0].east_m - track.x[0], found[column][0].north_m - track.x[1])
            for row, track in enumerate(tracks)
            for column in left
        }
        rejected_rows: set = set()
        new_tracks: List[_Track] = []
        for column in sorted(left, key=lambda c: min((distance[(row, c)] for row in range(len(tracks))), default=math.inf)):
            measurement, noise = found[column]
            near = sorted((distance[(row, column)], row) for row in range(len(tracks)) if distance[(row, column)] <= SAME_TARGET_RADIUS_M)
            free = [row for _, row in near if row not in taken_rows and row not in rejected_rows]
            if free:
                # That target, seen outside its gate.
                track = tracks[free[0]]
                rejected_rows.add(free[0])
                track.rejections += 1
                report.rejected += 1
                report.nis.append(float(cost[free[0], column]))
                if track.rejections >= RESET_AFTER_REJECTIONS:
                    track.x = np.array([measurement.east_m, measurement.north_m], dtype=float)
                    track.p = _floored(noise)
                    track.updated_ns = stamp_ns
                    track.hit(measurement, stamp_ns)
                    report.resets += 1
            elif near or any(math.hypot(measurement.east_m - other.x[0], measurement.north_m - other.x[1]) <= SAME_TARGET_RADIUS_M for other in new_tracks):
                report.ignored += 1  # beside a track that already has its detection for this frame
            elif self._make_room(class_id, stamp_ns):
                track = _Track(self._next_id, measurement, noise, stamp_ns)
                self._next_id += 1
                self._tracks.append(track)
                new_tracks.append(track)
                report.started += 1
            else:
                report.untracked += 1

    def _make_room(self, class_id: str, stamp_ns: int) -> bool:
        """True if the class can take one more track, after giving up its longest-lost or its stalest unconfirmed one if it is full."""
        tracks = [track for track in self._tracks if track.class_id == class_id]
        if len(tracks) < MAX_TRACKS_PER_CLASS:
            return True
        lost = [track for track in tracks if track.state == LOST]
        unproven = [track for track in tracks if track.state == TENTATIVE and not track.ever_confirmed and track.last_hit_ns < stamp_ns]
        if lost:
            victim = min(lost, key=lambda track: (track.lost_since_ns or 0, track.track_id))
        elif unproven:
            victim = min(unproven, key=lambda track: (track.last_hit_ns, track.track_id))
        else:
            return False
        self._tracks.remove(victim)
        return True

    # --- time passing ------------------------------------------------------------------

    def advance(self, now_ns: int) -> None:
        """Move every track through its life for the time `now_ns`."""
        now_ns = int(now_ns)
        kept = []
        for track in self._tracks:
            silent_s = (now_ns - track.last_hit_ns) * 1.0e-9
            if track.state == TENTATIVE and silent_s > TENTATIVE_DROP_S:
                if not track.ever_confirmed:
                    continue  # never proved itself: forgotten
                track.state, track.lost_since_ns = LOST, now_ns  # found again once, then not: lost again
            if track.state == CONFIRMED and silent_s > COAST_AFTER_S:
                track.state = COASTING
            if track.state == COASTING and silent_s > LOST_AFTER_S:
                track.state = LOST
                track.lost_since_ns = track.last_hit_ns + int(LOST_AFTER_S * 1.0e9)
                track.rejections = 0
            if track.state == LOST and (now_ns - (track.lost_since_ns or now_ns)) * 1.0e-9 > KEEP_LOST_S:
                continue
            kept.append(track)
        self._tracks = kept

    def clock_stepped_back(self, now_ns: int) -> None:
        """The clock the stamps are on has stepped back to `now_ns`: nothing known is fresh any more.

        A track's last hit would otherwise lie in the future and the track
        would read as just seen for as long as the step was. Every track that
        ever confirmed becomes LOST as of now (it can be found again and must
        confirm again); the others are dropped.
        """
        now_ns = int(now_ns)
        kept = []
        for track in self._tracks:
            if not track.ever_confirmed:
                continue
            track.state, track.lost_since_ns, track.rejections = LOST, now_ns, 0
            track.last_hit_ns = now_ns - int(LOST_AFTER_S * 1.0e9)
            track.updated_ns = min(track.updated_ns, now_ns)
            track.recent_hits.clear()
            kept.append(track)
        self._tracks = kept

    def shift_all(self, d_east_m: float, d_north_m: float) -> None:
        """The flight controller's position estimate stepped by this much (an EKF reset): every track steps with it."""
        step = np.array([float(d_east_m), float(d_north_m)])
        if not np.all(np.isfinite(step)):
            raise ValueError("a position reset must be two finite numbers")
        for track in self._tracks:
            track.x = track.x + step

    # --- reading out -------------------------------------------------------------------

    def estimates(self, now_ns: int, class_id: Optional[str] = None, for_ns: Optional[int] = None) -> List[TargetEstimate]:
        """Every track (of one class, if given) for the time `for_ns`: by default the actuation time, now + 0.08 s."""
        now_ns = int(now_ns)
        self.advance(now_ns)
        for_ns = now_ns + int(ACTUATION_DELAY_S * 1.0e9) if for_ns is None else int(for_ns)
        result = []
        for track in self._tracks:
            if class_id is not None and track.class_id != class_id:
                continue
            covariance = track.covariance_at(for_ns)
            result.append(
                TargetEstimate(
                    track_id=track.track_id,
                    class_id=track.class_id,
                    state=track.state,
                    east_m=float(track.x[0]),
                    north_m=float(track.x[1]),
                    up_m=float(track.up_m),
                    covariance=((float(covariance[0, 0]), float(covariance[0, 1])), (float(covariance[1, 0]), float(covariance[1, 1]))),
                    sigma_m=float(math.sqrt(max(np.linalg.eigvalsh(covariance).max(), 0.0))),
                    age_s=(now_ns - track.last_hit_ns) * 1.0e-9,
                    hits=track.hits,
                    score=float(track.score),
                    last_hit_ns=track.last_hit_ns,
                    for_ns=for_ns,
                )
            )
        return result

    def primary(self, class_id: str, now_ns: int, near: Optional[Sequence[float]] = None, for_ns: Optional[int] = None) -> Optional[TargetEstimate]:
        """The track of a class a mission should act on, or None if the class has none.

        CONFIRMED before COASTING before TENTATIVE before LOST. Within one
        state: the track nearest `near` (east, north) when a point is given,
        otherwise the one with the most hits.
        """
        candidates = self.estimates(now_ns, class_id, for_ns)
        if not candidates:
            return None

        def order(estimate: TargetEstimate):
            if near is not None:
                return (_RANK[estimate.state], math.hypot(estimate.east_m - float(near[0]), estimate.north_m - float(near[1])), estimate.track_id)
            return (_RANK[estimate.state], -estimate.hits, estimate.track_id)

        return min(candidates, key=order)
