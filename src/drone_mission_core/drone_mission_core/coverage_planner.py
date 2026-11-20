"""The survey's coverage planner (LOC-2): a lawnmower of waypoints over the search area.

  plan(polygon, height, camera)      the survey: every part of the polygon in at least two frames
  plan_wide_pass(...)                the two-phase search's first pass: everything once, tracks further apart
  plan_revisit(points, ...)          its second: tight passes only over where the first pass saw something
  coverage(plan, polygon)            the share of the polygon seen by one frame and by two (the check)

The polygon is the search area's corners in the local frame, east and north
in metres, in either order round. The camera looks straight down with the
image's long side across the tracks, so the swath is as wide as the picture
and the tracks are as few as can be. `yaw_deg` is the direction the image's
top must point for that (the track's heading): for a camera mounted with the
image's top forward it is the vehicle's yaw, and a mount turned in the
airframe (the robot description's `image_yaw_deg`) is the caller's to take
off it.

No survey is planned under the rules' floor (7.62 m) or over the survey's
speed cap (4 m/s), and a plan carries the height it was made for. Given the
flight boundary, every waypoint is kept 5 m or more inside it by pulling
track ends back, and the plan says so (`boundary_checked`); a leg that still
leaves it, across a notch in the boundary, is counted
(`legs_outside_boundary`) for the caller to refuse. Without a boundary
nothing is checked against one.

Tracks. The footprint is `height * width / fx` across by `height * height /
fy` along. Tracks are at most 0.8 W - 0.85 m apart (W the footprint's width
across the track; 0.85 m keeps a whole marker inside at least one pass), run
along the direction in which the polygon is narrowest across, and the outer
two stand W/2 - 0.85 m in from the polygon's edges, so a marker on the edge
is whole inside the swath. Each track runs as far as the polygon reaches
anywhere within its swath, not only where the track's own line crosses it,
so a corner between two tracks is not cut off. They are flown there and
back in turn.

Frames. A frame is taken every `speed / frame_rate` metres. At 4 m/s and
15 Hz from 30 ft a marker is in about 30 frames.

No ROS; numpy and the standard library only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, List, Optional, Sequence, Tuple

import numpy as np

from .safety_limits import GEOFENCE_MARGIN_M, PHASE_SPEED_CAPS_MPS, SURVEY_MIN_HEIGHT_M

MARKER_MARGIN_M = 0.85  # keeps a whole marker inside at least one pass
OVERLAP_FACTOR = 0.8  # of the footprint's width, before the marker margin
DEFAULT_SPEED_MPS = 4.0
DEFAULT_FRAME_RATE_HZ = 15.0
BOUNDARY_STEP_M = 0.25  # the polygon's edge is walked this finely to find how far a swath reaches
PULL_BACK_STEP_M = 0.25  # a track end outside the flight boundary's margin is pulled back by this much at a time
REVISIT_HALF_LENGTH_M = 6.0  # a revisit pass runs this far either side of what it revisits

Point = Tuple[float, float]


@dataclass(frozen=True)
class CoveragePlan:
    waypoints: Tuple[Point, ...]  # east, north; flown in order
    tracks: int
    spacing_m: float  # between neighbouring tracks
    track_heading_deg: float  # the direction the first track is flown, anticlockwise from east
    yaw_deg: float  # the direction the image's top must point for its long side to lie across the tracks (see the module's note on the mount)
    footprint_across_m: float
    footprint_along_m: float
    speed_mps: float
    frame_rate_hz: float
    length_m: float
    time_s: float
    frames_per_marker: float  # how many frames a point under a track is in
    height_m: float = 0.0  # the height above the ground the plan was made for: the footprint is this height's
    pulled_back_m: float = 0.0  # how far track ends were pulled back, in all, to stay inside the flight boundary
    tracks_left_out: int = 0  # tracks that could not be flown inside the flight boundary at all
    boundary_checked: bool = False  # a flight boundary was given: False means nothing here was checked against one
    legs_outside_boundary: int = 0  # legs between waypoints that leave the boundary's margin (a notch in it): the caller must not fly these as they are


@dataclass(frozen=True)
class Coverage:
    at_least_one: float  # share of the polygon's area in one frame or more
    at_least_two: float  # in two or more
    samples: int


def footprint(camera: Any, height_m: float) -> Tuple[float, float]:
    """(across, along) in metres on flat ground from `height_m`: the image's width and height through the camera's focal lengths."""
    return float(height_m) * float(camera.width) / float(camera.fx), float(height_m) * float(camera.height) / float(camera.fy)


def track_spacing(footprint_across_m: float) -> float:
    """The most that neighbouring tracks may be apart: 0.8 W - 0.85 m."""
    return OVERLAP_FACTOR * float(footprint_across_m) - MARKER_MARGIN_M


def _polygon(polygon: Sequence[Sequence[float]]) -> np.ndarray:
    try:
        points = np.asarray([(float(p[0]), float(p[1])) for p in polygon], dtype=float)
    except (TypeError, ValueError, IndexError) as error:
        raise ValueError(f"the search area's corners must be (east, north) pairs: not a number in {polygon!r}") from error
    if len(points) < 3:
        raise ValueError(f"the search area needs at least three corners, got {len(points)}")
    if not np.all(np.isfinite(points)):
        raise ValueError("a corner of the search area is not a number")
    x, y = points[:, 0], points[:, 1]
    area = 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))
    if area < 1.0e-6:
        raise ValueError("the search area has no area: its corners are in a line")
    return points


def _boundary(points: np.ndarray) -> np.ndarray:
    """The polygon's edge, walked every BOUNDARY_STEP_M, corners included."""
    out = []
    for a, b in zip(points, np.roll(points, -1, axis=0)):
        steps = max(1, int(math.ceil(float(np.linalg.norm(b - a)) / BOUNDARY_STEP_M)))
        out.append(a + np.outer(np.linspace(0.0, 1.0, steps, endpoint=False), b - a))
    return np.vstack(out)


def _narrowest_heading(points: np.ndarray) -> float:
    """The track direction (radians from east) across which the polygon is narrowest: one of its edges' directions."""
    best, best_width = 0.0, math.inf
    for a, b in zip(points, np.roll(points, -1, axis=0)):
        edge = b - a
        if float(np.linalg.norm(edge)) < 1.0e-9:
            continue
        heading = math.atan2(edge[1], edge[0]) % math.pi
        across = -points[:, 0] * math.sin(heading) + points[:, 1] * math.cos(heading)
        width = float(across.max() - across.min())
        if width < best_width - 1.0e-9:
            best, best_width = heading, width
    return best


def _checked(height_m: float, speed_mps: float, frame_rate_hz: float, heading: Optional[float] = None) -> None:
    for name, value in (("height", height_m), ("speed", speed_mps), ("frame rate", frame_rate_hz)):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"the survey's {name} must be a number above 0, got {value!r}")
    if height_m < SURVEY_MIN_HEIGHT_M:
        raise ValueError(f"no survey is planned under the rules' floor of {SURVEY_MIN_HEIGHT_M:g} m: the height is {height_m:g} m")
    if speed_mps > PHASE_SPEED_CAPS_MPS["survey"]:
        raise ValueError(f"the survey's speed cap is {PHASE_SPEED_CAPS_MPS['survey']:g} m/s: {speed_mps:g} m/s is over it")
    if heading is not None and (isinstance(heading, bool) or not isinstance(heading, (int, float)) or not math.isfinite(heading)):
        raise ValueError(f"the tracks' heading must be a number of radians, got {heading!r}")


def _edge_distance(point: np.ndarray, polygon: np.ndarray) -> float:
    """How far a point is from the polygon's edge."""
    a, b = polygon, np.roll(polygon, -1, axis=0)
    edge = b - a
    lengths = np.maximum(np.einsum("ij,ij->i", edge, edge), 1.0e-18)
    along = np.clip(np.einsum("ij,ij->i", point - a, edge) / lengths, 0.0, 1.0)
    return float(np.min(np.linalg.norm(a + along[:, None] * edge - point, axis=1)))


def _edge_distances(points: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    """How far each point is from the polygon's edge: _edge_distance for every point at once.

    The survey mission plans inside one executor tick, and a track is
    checked every quarter metre, so the points are not walked in Python.
    """
    a, b = polygon, np.roll(polygon, -1, axis=0)
    edge = b - a
    lengths = np.maximum(np.einsum("ij,ij->i", edge, edge), 1.0e-18)
    from_a = points[:, None, :] - a[None, :, :]
    along = np.clip(np.einsum("nej,ej->ne", from_a, edge) / lengths, 0.0, 1.0)
    nearest = a[None, :, :] + along[:, :, None] * edge[None, :, :]
    return np.min(np.linalg.norm(nearest - points[:, None, :], axis=2), axis=1)


def _within(points: np.ndarray, corners: np.ndarray, margin_m: float) -> np.ndarray:
    """Which points are inside the boundary and at least `margin_m` from its edge."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    return np.asarray(_inside(points, corners), dtype=bool) & (_edge_distances(points, corners) >= margin_m - 1.0e-9)


def inside_with_margin(point: Sequence[float], boundary: Sequence[Sequence[float]], margin_m: float) -> bool:
    """Whether a point is inside the boundary and at least `margin_m` from its edge."""
    return bool(_within(np.asarray([[float(point[0]), float(point[1])]]), _polygon(boundary), float(margin_m))[0])


def _confine(waypoints: List[Point], boundary: Optional[Sequence[Sequence[float]]], margin_m: float) -> Tuple[List[Point], float, int, int]:
    """Each track's two ends pulled back along the track until inside the boundary's margin.

    (waypoints, metres pulled back, tracks left out, legs that still leave the margin). Only the ends can be
    moved: a leg that crosses a notch in the boundary is counted, for the caller to refuse.
    """
    if boundary is None:
        return waypoints, 0.0, 0, 0
    if isinstance(margin_m, bool) or not isinstance(margin_m, (int, float)) or not math.isfinite(margin_m) or margin_m < GEOFENCE_MARGIN_M:
        raise ValueError(f"the flight boundary's margin is {GEOFENCE_MARGIN_M:g} m or more (safety_limits), got {margin_m!r}")
    corners = _polygon(boundary)
    kept: List[Point] = []
    pulled, left_out = 0.0, 0
    for start, end in zip(waypoints[0::2], waypoints[1::2]):
        a, b = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
        length = float(np.linalg.norm(b - a))
        steps = np.append(np.arange(0.0, length, PULL_BACK_STEP_M), length)
        unit = (b - a) / length if length > 1.0e-9 else np.zeros(2)
        fine = np.flatnonzero(_within(a + np.outer(steps, unit), corners, float(margin_m)))
        if not len(fine):
            left_out += 1
            continue
        first, last = float(steps[fine[0]]), float(steps[fine[-1]])
        pulled += first + (length - last)
        kept += [tuple(float(v) for v in a + unit * first), tuple(float(v) for v in a + unit * last)]
    if not kept:
        raise ValueError("no track of the survey can be flown inside the flight boundary with its margin")
    outside = 0
    for a, b in zip(kept, kept[1:]):
        a, b = np.asarray(a), np.asarray(b)
        count = max(2, int(math.ceil(float(np.linalg.norm(b - a)) / 1.0)) + 1)
        if not np.all(_within(a + np.outer(np.linspace(0.0, 1.0, count), b - a), corners, float(margin_m) - 0.5)):
            outside += 1
    return kept, pulled, left_out, outside


def _lawnmower(
    points: np.ndarray,
    height_m: float,
    camera: Any,
    speed_mps: float,
    frame_rate_hz: float,
    spacing_limit_m: float,
    heading: Optional[float],
    boundary: Optional[Sequence[Sequence[float]]] = None,
    boundary_margin_m: float = GEOFENCE_MARGIN_M,
    start: Optional[Sequence[float]] = None,
) -> CoveragePlan:
    across_m, along_m = footprint(camera, height_m)
    if spacing_limit_m <= 0.0 or across_m / 2.0 <= MARKER_MARGIN_M:
        raise ValueError(
            f"from {height_m:g} m the footprint is {across_m:.2f} m across: too narrow for a track spacing with the {MARKER_MARGIN_M:g} m marker margin"
        )
    heading = _narrowest_heading(points) if heading is None else float(heading)
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    edge = _boundary(points)
    along = edge[:, 0] * cos_h + edge[:, 1] * sin_h
    across = -edge[:, 0] * sin_h + edge[:, 1] * cos_h
    low, high = float(across.min()), float(across.max())
    inset = across_m / 2.0 - MARKER_MARGIN_M  # the outer tracks stand this far in: the edge is 0.85 m inside the swath
    if high - low <= 2.0 * inset:
        offsets = [0.5 * (low + high)]
    else:
        first, last = low + inset, high - inset
        gaps = max(1, int(math.ceil((last - first) / spacing_limit_m - 1.0e-9)))
        offsets = [first + (last - first) * index / gaps for index in range(gaps + 1)]
    ends = []
    for offset in offsets:
        inside = np.abs(across - offset) <= across_m / 2.0  # the polygon's edge within this track's swath
        ends.append((float(along[inside].min()), float(along[inside].max())))

    def flown(reverse_tracks: bool, flip: bool) -> List[Point]:
        """The tracks there and back in turn, from either side of the area and either end of the first track."""
        order = list(zip(offsets, ends))[::-1] if reverse_tracks else list(zip(offsets, ends))
        points: List[Point] = []
        for index, (offset, (low_end, high_end)) in enumerate(order):
            first, second = (high_end, low_end) if (index % 2 == 1) != flip else (low_end, high_end)
            for a in (first, second):
                points.append((a * cos_h - offset * sin_h, a * sin_h + offset * cos_h))
        return points

    waypoints = flown(False, False)
    if start is not None:
        try:
            begin = (float(start[0]), float(start[1]))
        except (TypeError, ValueError, IndexError) as error:
            raise ValueError(f"the start point must be (east, north), got {start!r}") from error
        if not all(math.isfinite(value) for value in begin):
            raise ValueError(f"the start point is not a number: {start!r}")
        waypoints = min((flown(r, f) for r in (False, True) for f in (False, True)), key=lambda points: math.dist(points[0], begin))
    waypoints, pulled_back, left_out, legs_outside = _confine(waypoints, boundary, boundary_margin_m)
    length = sum(math.dist(a, b) for a, b in zip(waypoints, waypoints[1:]))
    spacing = offsets[1] - offsets[0] if len(offsets) > 1 else 0.0
    return CoveragePlan(
        height_m=float(height_m),
        pulled_back_m=pulled_back,
        tracks_left_out=left_out,
        boundary_checked=boundary is not None,
        legs_outside_boundary=legs_outside,
        waypoints=tuple(waypoints),
        tracks=len(offsets) - left_out,
        spacing_m=spacing,
        track_heading_deg=math.degrees(heading),
        yaw_deg=math.degrees(heading),
        footprint_across_m=across_m,
        footprint_along_m=along_m,
        speed_mps=float(speed_mps),
        frame_rate_hz=float(frame_rate_hz),
        length_m=length,
        time_s=length / float(speed_mps),
        frames_per_marker=along_m / (float(speed_mps) / float(frame_rate_hz)),
    )


def plan(
    polygon: Sequence[Sequence[float]],
    height_m: float,
    camera: Any,
    speed_mps: float = DEFAULT_SPEED_MPS,
    frame_rate_hz: float = DEFAULT_FRAME_RATE_HZ,
    track_heading_rad: Optional[float] = None,
    boundary: Optional[Sequence[Sequence[float]]] = None,
    boundary_margin_m: float = GEOFENCE_MARGIN_M,
    start: Optional[Sequence[float]] = None,
) -> CoveragePlan:
    """The survey of a polygon: tracks at most 0.8 W - 0.85 m apart.

    `track_heading_rad` fixes the tracks' direction; left out, the fewest
    tracks. `start` is where the vehicle is: the tracks are then begun at
    the end nearest it (for planning again in mid-survey). `boundary` is the flight boundary's corners: with it, every
    waypoint is at least `boundary_margin_m` (5 m) inside it, track ends
    being pulled back along their tracks as far as that takes
    (`pulled_back_m`, `tracks_left_out`). What that costs in coverage is
    for coverage() to say.
    """
    _checked(height_m, speed_mps, frame_rate_hz, track_heading_rad)
    points = _polygon(polygon)
    across_m, _along = footprint(camera, height_m)
    if track_spacing(across_m) <= 0.0:
        raise ValueError(f"from {height_m:g} m the footprint is {across_m:.2f} m across: no track spacing is left after the {MARKER_MARGIN_M:g} m marker margin")
    return _lawnmower(points, height_m, camera, speed_mps, frame_rate_hz, track_spacing(across_m), track_heading_rad, boundary, boundary_margin_m, start)


def plan_wide_pass(
    polygon: Sequence[Sequence[float]],
    height_m: float,
    camera: Any,
    speed_mps: float = DEFAULT_SPEED_MPS,
    frame_rate_hz: float = DEFAULT_FRAME_RATE_HZ,
    track_heading_rad: Optional[float] = None,
    boundary: Optional[Sequence[Sequence[float]]] = None,
    boundary_margin_m: float = GEOFENCE_MARGIN_M,
) -> CoveragePlan:
    """The two-phase search's first pass: tracks W - 0.85 m apart, so everything is in one pass's swath and nothing need be in two."""
    _checked(height_m, speed_mps, frame_rate_hz, track_heading_rad)
    points = _polygon(polygon)
    across_m, _along = footprint(camera, height_m)
    return _lawnmower(points, height_m, camera, speed_mps, frame_rate_hz, across_m - MARKER_MARGIN_M, track_heading_rad, boundary, boundary_margin_m)


def plan_revisit(
    tentative: Sequence[Sequence[float]],
    height_m: float,
    camera: Any,
    speed_mps: float = DEFAULT_SPEED_MPS,
    frame_rate_hz: float = DEFAULT_FRAME_RATE_HZ,
    track_heading_rad: float = 0.0,
    boundary: Optional[Sequence[Sequence[float]]] = None,
    boundary_margin_m: float = GEOFENCE_MARGIN_M,
) -> CoveragePlan:
    """The two-phase search's second pass: one short pass over each place the first pass saw something, nearest first.

    Places within half a swath of each other across the tracks, and within a
    pass's length along them, share a pass. Each pass runs
    REVISIT_HALF_LENGTH_M beyond what it revisits at both ends.
    """
    _checked(height_m, speed_mps, frame_rate_hz, track_heading_rad)
    across_m, along_m = footprint(camera, height_m)
    cos_h, sin_h = math.cos(track_heading_rad), math.sin(track_heading_rad)
    try:
        seen = [(float(p[0]), float(p[1])) for p in tentative]
    except (TypeError, ValueError, IndexError) as error:
        raise ValueError(f"a place to revisit is not a number: {tentative!r}") from error
    if not all(math.isfinite(value) for point in seen for value in point):
        raise ValueError(f"a place to revisit is not a number: {tentative!r}")
    places = sorted((east * cos_h + north * sin_h, -east * sin_h + north * cos_h) for east, north in seen)
    # A place joins a pass only if the pass, flown down the middle of its places, still has every one of them
    # inside its swath by the marker margin: the places of one pass are never further apart across than that allows.
    reach = 2.0 * (across_m / 2.0 - MARKER_MARGIN_M)
    groups: List[List[Tuple[float, float]]] = []
    for along, across in places:
        for group in groups:
            lowest, highest = min(item[1] for item in group), max(item[1] for item in group)
            if max(highest, across) - min(lowest, across) <= reach and along - max(item[0] for item in group) <= 2.0 * REVISIT_HALF_LENGTH_M:
                group.append((along, across))
                break
        else:
            groups.append([(along, across)])
    waypoints: List[Point] = []
    for index, group in enumerate(groups):
        offset = 0.5 * (min(item[1] for item in group) + max(item[1] for item in group))
        start, end = min(item[0] for item in group) - REVISIT_HALF_LENGTH_M, max(item[0] for item in group) + REVISIT_HALF_LENGTH_M
        if index % 2 == 1:
            start, end = end, start
        for a in (start, end):
            waypoints.append((a * cos_h - offset * sin_h, a * sin_h + offset * cos_h))
    waypoints, pulled_back, left_out, legs_outside = _confine(waypoints, boundary, boundary_margin_m)
    length = sum(math.dist(a, b) for a, b in zip(waypoints, waypoints[1:]))
    return CoveragePlan(
        height_m=float(height_m),
        pulled_back_m=pulled_back,
        tracks_left_out=left_out,
        boundary_checked=boundary is not None,
        legs_outside_boundary=legs_outside,
        waypoints=tuple(waypoints),
        tracks=len(groups) - left_out,
        spacing_m=0.0,
        track_heading_deg=math.degrees(track_heading_rad),
        yaw_deg=math.degrees(track_heading_rad),
        footprint_across_m=across_m,
        footprint_along_m=along_m,
        speed_mps=float(speed_mps),
        frame_rate_hz=float(frame_rate_hz),
        length_m=length,
        time_s=length / float(speed_mps),
        frames_per_marker=along_m / (float(speed_mps) / float(frame_rate_hz)),
    )


def _frame_centres(plan_: CoveragePlan) -> np.ndarray:
    """Where a frame is taken along the whole path, turns included: every speed / frame rate metres."""
    step = plan_.speed_mps / plan_.frame_rate_hz
    centres, carried = [], 0.0
    for a, b in zip(plan_.waypoints, plan_.waypoints[1:]):
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        leg = float(np.linalg.norm(b - a))
        if leg < 1.0e-9:
            continue
        distances = np.arange(carried, leg, step)
        centres.append(a + np.outer(distances / leg, b - a))
        carried = (carried - leg) % step
    return np.vstack(centres) if centres else np.empty((0, 2))


def _frames_over(plan_: CoveragePlan, points: np.ndarray) -> np.ndarray:
    """For each point, how many frames it is in: the footprint is held with its long side across the tracks all the way."""
    centres = _frame_centres(plan_)
    counts = np.zeros(len(points), dtype=int)
    if not len(centres):
        return counts
    heading = math.radians(plan_.yaw_deg)
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    for start in range(0, len(centres), 250):  # in slices, so the check's memory stays small
        delta = points[:, None, :] - centres[None, start : start + 250, :]
        along = delta[:, :, 0] * cos_h + delta[:, :, 1] * sin_h
        across = -delta[:, :, 0] * sin_h + delta[:, :, 1] * cos_h
        counts += np.count_nonzero((np.abs(along) <= plan_.footprint_along_m / 2.0) & (np.abs(across) <= plan_.footprint_across_m / 2.0), axis=1)
    return counts


def frames_seeing(plan_: CoveragePlan, point: Sequence[float]) -> int:
    """How many of the plan's frames a ground point is in."""
    return int(_frames_over(plan_, np.asarray([[float(point[0]), float(point[1])]]))[0])


def _inside(points: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    """Which points are inside the polygon (even-odd rule)."""
    inside = np.zeros(len(points), dtype=bool)
    x, y = points[:, 0], points[:, 1]
    for (x1, y1), (x2, y2) in zip(polygon, np.roll(polygon, -1, axis=0)):
        crosses = (y1 > y) != (y2 > y)
        with np.errstate(divide="ignore", invalid="ignore"):
            at = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
        inside ^= crosses & (x < at)
    return inside


def coverage(plan_: CoveragePlan, polygon: Sequence[Sequence[float]], rng: Optional[np.random.Generator] = None, samples: int = 4000) -> Coverage:
    """The share of the polygon's area that is in at least one frame, and in at least two, by random points inside it.

    A check for tests, tools and planning on the ground: it counts every
    frame against every sample, which is far too much for a mission's tick.
    """
    corners = _polygon(polygon)
    rng = np.random.default_rng(0) if rng is None else rng
    low, high = corners.min(axis=0), corners.max(axis=0)
    kept: List[np.ndarray] = []
    total = 0
    while total < samples:
        candidates = rng.uniform(low, high, size=(samples, 2))
        chosen = candidates[_inside(candidates, corners)]
        kept.append(chosen)
        total += len(chosen)
    points = np.vstack(kept)[:samples]
    counts = _frames_over(plan_, points)
    return Coverage(at_least_one=float(np.mean(counts >= 1)), at_least_two=float(np.mean(counts >= 2)), samples=len(points))
