"""The weighted vote over a track's digit readings (LOC-8). No ROS.

Behind the node flag `features.digit_vote` (with the tracker's and the digit
reader's). One reading of a marker is one opinion; a track is seen about 25
times in a survey. Each reading (digit_reader.DigitReading) adds its ten
log-probabilities to its track's, weighted by how much the sighting is worth:

  weight = size x sharpness x view x detector

  size      the marker's side in the crop over FULL_SIDE_PX, at most 1
  sharpness s / (s + SHARPNESS_HALF), s the variance of the patch's Laplacian
  view      the cosine squared of the angle between the ray to the marker and
            straight down: a marker seen from the side is foreshortened
  detector  the detector's confidence in the box

A reading's log-probabilities have a floor (MIN_LOG_PROBABILITY): no one
reading, however sure, rules a digit out. A reading that says nothing (all
ten equal) is not a vote.

A track's score for a digit is its summed log-probability set against
"nothing can be said of this track" (every reading uniform):

  score(track, digit) = sum of w * log p(digit)  -  sum of w * log 0.1

It is positive when the readings favour the digit and grows with their
number. A track has a digit when its best score beats both nothing (0) and
its second-best score by MIN_VOTE_MARGIN, which is odds of 9 to 1.

IDs are taken to be unique (`unique_ids`, the node's `digit_ids_unique`): no
two markers carry one digit. Then the digits are given out by one assignment
over tracks and digits (the Hungarian method on the scores, each track free
to take no digit), so that of two tracks claiming a digit the one with the
evidence has it and the other takes what else its readings allow. A track
is given its digit only if taking it away would cost the whole assignment
MIN_VOTE_MARGIN: two tracks with the same claim to one digit, or two "6 or
9" tracks, are left unknown, not split by a coin. If the rules do not make
IDs unique this step is switched off, and each track answers for itself.

The assignment is solved again when votes have changed, and with
`solve_period_s` no more often than that: between solves the last answer
stands (a track that has joined or left the tracks asked about is answered
at once). `last_solve_ms` is what the last solve took.

The standard library and numpy; the Hungarian method is drone_mission_core's.
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
from drone_mission_core.assignment import hungarian

FLAG = "digit_vote"
UNKNOWN = -1
FULL_SIDE_PX = 100.0  # a marker this many pixels a side in its crop is as big as a reading needs
SHARPNESS_HALF = 500.0  # the patch's Laplacian variance at which a reading counts half (sharp 4K patches: 2500 to 3500; blurred by 3 px: 300)
MIN_LOG_PROBABILITY = math.log(1.0e-3)  # a reading's floor for any digit
MIN_VOTE_CONFIDENCE = 0.9
MIN_VOTE_MARGIN = math.log(MIN_VOTE_CONFIDENCE / (1.0 - MIN_VOTE_CONFIDENCE))  # 2.2: odds of 9 to 1
LOG_UNIFORM = math.log(0.1)
MAX_ASSIGNED_TRACKS = 20  # the assignment is over at most this many tracks, those with the strongest claims
NODE_SOLVE_PERIOD_S = 0.5  # in the node the digits are given out again at most this often: the vote shares the detector's thread
_FORBIDDEN = 1.0e9


def _number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def sighting_weight(side_px: Any, sharpness: Any, view_angle_rad: Any, detector_score: Any) -> float:
    """What a reading is worth, 0 to 1. Nothing, when any of the four is not a usable number."""
    if not all(_number(value) for value in (side_px, sharpness, view_angle_rad, detector_score)):
        return 0.0
    if side_px <= 0.0 or sharpness <= 0.0 or detector_score <= 0.0:
        return 0.0
    overhead = math.cos(float(view_angle_rad))
    if overhead <= 0.0:
        return 0.0
    size = min(1.0, float(side_px) / FULL_SIDE_PX)
    sharp = float(sharpness) / (float(sharpness) + SHARPNESS_HALF)
    return size * sharp * overhead * overhead * min(1.0, float(detector_score))


def view_angle(point: Any) -> Optional[float]:
    """The angle between the ray from the lens to a ground point (projection v2's GroundPoint) and straight down, radians."""
    try:
        height = float(point.camera_enu[2]) - float(point.up_m)
        distance = float(point.range_m)
    except (AttributeError, IndexError, TypeError, ValueError):
        return None
    if not (math.isfinite(height) and math.isfinite(distance)) or distance <= 0.0 or height <= 0.0:
        return None
    return math.acos(min(1.0, height / distance))


class DigitVotes:
    def __init__(self, unique_ids: bool = True, solve_period_s: float = 0.0, clock: Any = time.monotonic) -> None:
        self.unique_ids = bool(unique_ids)
        self.solve_period_s = max(0.0, float(solve_period_s))  # the answer is worked out again no more often than this
        self.solves = 0
        self.last_solve_ms = 0.0
        self._clock = clock
        self._solved_at: Optional[float] = None
        self._sums: Dict[int, np.ndarray] = {}
        self._weights: Dict[int, float] = {}
        self._counts: Dict[int, int] = {}
        self._version = 0
        self._answer: Optional[Tuple[Any, Dict[int, int], List[int]]] = None

    # --- taking votes ---------------------------------------------------------------

    def add(self, track_id: int, log_probabilities: Sequence[float], weight: float) -> bool:
        """One reading for a track. False when it is not a vote: no weight, not ten numbers, or nothing said."""
        if not _number(weight) or weight <= 0.0:
            return False
        try:
            scores = np.asarray(log_probabilities, dtype=float)
        except (TypeError, ValueError):
            return False
        if scores.shape != (10,) or np.isnan(scores).any():
            return False
        scores = np.clip(scores, MIN_LOG_PROBABILITY, 0.0)
        if float(scores.max() - scores.min()) < 1.0e-9:
            return False  # all ten equal: the reader learnt nothing
        track_id = int(track_id)
        self._sums[track_id] = self._sums.get(track_id, np.zeros(10)) + float(weight) * scores
        self._weights[track_id] = self._weights.get(track_id, 0.0) + float(weight)
        self._counts[track_id] = self._counts.get(track_id, 0) + 1
        self._version += 1
        return True

    def add_sums(self, track_id: int, sums: Sequence[float], weight: float, readings: int) -> bool:
        """Votes already summed elsewhere (another pass over the same markers, LOC-12): the ten weighted sums, their total weight and how many readings they were.

        They are added as they stand: `sums` is what `log_probabilities` gave where they were taken. False when they are not a vote.
        """
        if not _number(weight) or weight <= 0.0 or int(readings) <= 0:
            return False
        try:
            scores = np.asarray(sums, dtype=float)
        except (TypeError, ValueError):
            return False
        if scores.shape != (10,) or not np.isfinite(scores).all():
            return False
        track_id = int(track_id)
        self._sums[track_id] = self._sums.get(track_id, np.zeros(10)) + scores
        self._weights[track_id] = self._weights.get(track_id, 0.0) + float(weight)
        self._counts[track_id] = self._counts.get(track_id, 0) + int(readings)
        self._version += 1
        return True

    def add_frame(self, assignment: Mapping[int, int], ground: Sequence[Any], crops: Sequence[Any], readings: Sequence[Any]) -> int:
        """A frame's readings to the tracks their boxes went to; how many were votes.

        `assignment` is the tracker's for this frame: index into `ground` to
        track id. A reading belongs to the ground point cut from the same box
        (`crop.detection is item.detection`).
        """
        by_box = {id(crop.detection): reading for crop, reading in zip(crops, readings)}
        added = 0
        for index, track_id in assignment.items():
            if not 0 <= int(index) < len(ground):
                continue
            item = ground[int(index)]
            reading = by_box.get(id(getattr(item, "detection", None)))
            if reading is None:
                continue
            angle = view_angle(getattr(item, "point", None))
            weight = sighting_weight(getattr(reading, "side_px", None), getattr(reading, "sharpness", None), angle, getattr(item, "score", None))
            added += self.add(track_id, getattr(reading, "log_probabilities", ()), weight)
        return added

    def forget(self, track_ids: Iterable[int]) -> None:
        for track_id in track_ids:
            for store in (self._sums, self._weights, self._counts):
                store.pop(int(track_id), None)
        self._version += 1

    # --- what a track's votes say ---------------------------------------------------

    def log_probabilities(self, track_id: int) -> Tuple[float, ...]:
        """The track's ten summed, weighted log-probabilities; zeros for a track nothing was read on."""
        return tuple(float(value) for value in self._sums.get(int(track_id), np.zeros(10)))

    def readings(self, track_id: int) -> int:
        return self._counts.get(int(track_id), 0)

    def weight(self, track_id: int) -> float:
        return self._weights.get(int(track_id), 0.0)

    def confidence(self, track_id: int) -> float:
        """The share of the track's vote that its best digit has, 0 to 1 (0.1 with no votes)."""
        sums = self._sums.get(int(track_id))
        if sums is None:
            return 0.1
        shifted = np.exp(sums - sums.max())
        return float(shifted.max() / shifted.sum())

    def tracks(self) -> List[int]:
        return sorted(self._sums)

    def _scores(self, track_id: int) -> np.ndarray:
        return self._sums[track_id] - self._weights[track_id] * LOG_UNIFORM

    def _alone(self, track_id: int) -> int:
        """The track's digit by its own votes: the best score, if it beats nothing and the second best by the margin."""
        scores = self._scores(track_id)
        order = np.argsort(scores)
        best, second = float(scores[order[-1]]), float(scores[order[-2]])
        return int(order[-1]) if best - max(0.0, second) >= MIN_VOTE_MARGIN else UNKNOWN

    # --- the answer -----------------------------------------------------------------

    def digits(self, only: Optional[Iterable[int]] = None) -> Dict[int, int]:
        """Each voted track's digit, or UNKNOWN. With `only`, those tracks alone take part (and are answered for)."""
        key = (self._version, None if only is None else tuple(sorted(int(track) for track in only)))
        now = float(self._clock())
        if self._answer is not None and self._answer[0] != key and self._answer[0][1] == key[1]:
            # Only the votes have changed since the last solve: within the period the last answer stands.
            if self._solved_at is not None and 0.0 <= now - self._solved_at < self.solve_period_s:
                return dict(self._answer[1])
        if self._answer is None or self._answer[0] != key:
            started = time.perf_counter()
            taking_part = [track for track in self.tracks() if key[1] is None or track in set(key[1])]
            digits, contested = self._assigned(taking_part) if self.unique_ids else ({track: self._alone(track) for track in taking_part}, [])
            self._answer = (key, digits, contested)
            self._solved_at = now
            self.solves += 1
            self.last_solve_ms = (time.perf_counter() - started) * 1000.0
        return dict(self._answer[1])

    def settle(self) -> None:
        """The next answer is worked out afresh, whatever the period: for the end of a survey, when the list is closed."""
        self._solved_at = None

    def contested(self) -> List[int]:
        """Tracks whose own votes are sure of a digit that the last assignment could not give them."""
        return [] if self._answer is None else list(self._answer[2])

    def _assigned(self, tracks: List[int]) -> Tuple[Dict[int, int], List[int]]:
        digits = {track: UNKNOWN for track in tracks}
        if not tracks:
            return digits, []
        # The assignment is over the tracks with the strongest claims; a field has ten digits.
        claims = sorted(tracks, key=lambda track: -float(self._scores(track).max()))[:MAX_ASSIGNED_TRACKS]
        count = len(claims)
        # Columns: the ten digits, then one "no digit" for each track, which costs nothing.
        cost = np.zeros((count, 10 + count))
        for row, track in enumerate(claims):
            cost[row, :10] = -self._scores(track)
        pairs = hungarian(cost)
        total = sum(float(cost[row, column]) for row, column in pairs)
        for row, column in pairs:
            if column >= 10:
                continue
            without = cost.copy()
            without[row, column] = _FORBIDDEN
            other = sum(float(without[r, c]) for r, c in hungarian(without))
            if other - total >= MIN_VOTE_MARGIN:
                digits[claims[row]] = int(column)
        contested = sorted(track for track in claims if digits[track] == UNKNOWN and self._alone(track) != UNKNOWN)
        return digits, contested
