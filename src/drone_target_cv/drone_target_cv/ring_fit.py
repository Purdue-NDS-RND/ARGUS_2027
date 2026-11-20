"""The near-field bullseye centre finder (CEN-13): concentric rings fitted to the target's red and white edges. No ROS imports.

Near the ground the bullseye is larger than the picture. A box around its
red pixels is then a box around the part in view, and its centre is not the
target's. The edges between red and white still say where the centre is:
each is a circle about it, so the line square to an edge at any point of it
passes through the centre. `fit_rings` finds the point most of those lines
pass through, sorts the edge points into rings by their distance from it,
and refines the point so that every ring is as round about it as can be.

It needs at least two rings (one red disc is not a bullseye), with the
centre itself inside the picture: a centre worked out from arcs alone, far
outside the frame, is a guess. Points on the picture's own border are left
out, since there the red stops because the picture does.

The vehicle's tilt turns the circles into ellipses whose centres are not
the target's, and a fit of circles to them is wrong by pixels or fails. So
the edge points are first moved to where a camera looking straight down
from the same place would have seen them (`nadir_homography`, from the
camera matrix and the camera's attitude at the frame's time, which the
node has); there the rings are circles about one point again. The centre
found is moved back into the picture's own pixels. Without an attitude the
fit is made on the picture as it is, and declines when the rings are too
far out of round.

In rendered views up to 10 degrees of tilt, with blur and noise, the centre
was within 2 px (test_ring_fit.py), and PIXEL_SIGMA_PX, the noise the
estimator is told this detector has, is set from that.

`to_detection` gives the result as a `bullseye` under the detection
contract (FND-02). Its box is the innermost ring fitted, about the centre:
a box that is inside the picture even when the target is not.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from drone_mission_core.detection_contract import Detection

DETECTOR_NAME = "ring_fit"
# One standard deviation of the centre, in pixels, for the estimator's measurement noise.
PIXEL_SIGMA_PX = 2.0


@dataclass(frozen=True)
class RingFitConfig:
    # Red, as the 2026 detector's node sets it (target_cv.py): two hue bands, with floors on saturation and value.
    hsv_red1_h_min: int = 0
    hsv_red1_h_max: int = 8
    hsv_red2_h_min: int = 168
    hsv_red2_h_max: int = 180
    hsv_s_min: int = 80
    hsv_v_min: int = 45
    border_px: int = 3  # edge points this close to the picture's border are the picture's edge, not a ring's
    min_edge_points: int = 120  # fewer points than this are not looked at
    max_sample_points: int = 1500  # the search for the centre uses at most this many, evenly taken
    search_tries: int = 300
    inlier_px: float = 2.5  # a point agrees with a centre when its normal passes this close to it
    min_ring_points: int = 30
    min_ring_arc_deg: float = 25.0  # a ring must be seen over at least this much of its turn
    min_rings: int = 2
    min_ring_spacing_px: float = 4.0  # rings closer than this are too fine to be told apart: the target is too far
    max_rms_px: float = 2.0
    min_inlier_fraction: float = 0.6


@dataclass(frozen=True)
class RingFit:
    center: Tuple[float, float]  # pixels of the image given
    radii_px: Tuple[float, ...]  # of the rings fitted, smallest first
    rings: int
    rms_px: float  # how far the ring points are from their rings
    inlier_fraction: float  # the share of the edge points that belong to the rings
    points: int


def red_mask(image_bgr: np.ndarray, config: RingFitConfig) -> np.ndarray:
    """255 where the picture is the target's red, 0 elsewhere."""
    hsv = cv2.cvtColor(cv2.GaussianBlur(image_bgr, (3, 3), 0), cv2.COLOR_BGR2HSV)
    low = cv2.inRange(hsv, (config.hsv_red1_h_min, config.hsv_s_min, config.hsv_v_min), (config.hsv_red1_h_max, 255, 255))
    high = cv2.inRange(hsv, (config.hsv_red2_h_min, config.hsv_s_min, config.hsv_v_min), (config.hsv_red2_h_max, 255, 255))
    kernel = np.ones((3, 3), np.uint8)
    return cv2.morphologyEx(cv2.morphologyEx(cv2.bitwise_or(low, high), cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)


def _edge_points(mask: np.ndarray, config: RingFitConfig) -> Tuple[np.ndarray, np.ndarray]:
    """(points, unit normals) on the red regions' outlines, without those on the picture's border."""
    contours, _hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return np.empty((0, 2)), np.empty((0, 2))
    points = np.vstack([contour.reshape(-1, 2) for contour in contours]).astype(np.float64)
    height, width = mask.shape
    b = config.border_px
    inside = (points[:, 0] >= b) & (points[:, 0] < width - b) & (points[:, 1] >= b) & (points[:, 1] < height - b)
    points = points[inside]
    if len(points) == 0:
        return points, np.empty((0, 2))
    soft = cv2.GaussianBlur(mask.astype(np.float32) / 255.0, (0, 0), 1.5)
    gx = cv2.Sobel(soft, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(soft, cv2.CV_32F, 0, 1, ksize=3)
    xs, ys = points[:, 0].astype(int), points[:, 1].astype(int)
    normals = np.column_stack([gx[ys, xs], gy[ys, xs]]).astype(np.float64)
    length = np.hypot(normals[:, 0], normals[:, 1])
    keep = length > 1.0e-3
    return points[keep], normals[keep] / length[keep, None]


def nadir_homography(camera_matrix: np.ndarray, world_from_optical: np.ndarray) -> np.ndarray:
    """The 3x3 matrix that moves a pixel to where a camera looking straight down from the same place would see it.

    `camera_matrix` is K. `world_from_optical` is the rotation that turns a
    vector in the camera's optical frame (x right, y down, z along the lens)
    into the world frame (east, north, up). The straight-down camera keeps
    the real one's heading, so the picture is not turned, only levelled.
    Raises ValueError for a camera that is not looking downward at all.
    """
    k = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    r = np.asarray(world_from_optical, dtype=np.float64).reshape(3, 3)
    lens = r[:, 2]
    if lens[2] > -0.5:
        raise ValueError("the camera is more than 60 degrees from straight down: its picture cannot be levelled")
    down = np.array([0.0, 0.0, -1.0])
    right = r[:, 0] - np.dot(r[:, 0], down) * down  # the picture's right, laid flat
    right /= np.linalg.norm(right)
    level = np.column_stack([right, np.cross(down, right), down])  # world from the straight-down camera
    return k @ level.T @ r @ np.linalg.inv(k)


def _apply(homography: np.ndarray, points: np.ndarray) -> np.ndarray:
    moved = np.column_stack([points, np.ones(len(points))]) @ homography.T
    return moved[:, :2] / moved[:, 2:3]


def _search_centre(points: np.ndarray, normals: np.ndarray, size: Tuple[int, int], config: RingFitConfig, in_picture=None) -> Optional[np.ndarray]:
    """The point inside the picture that the most normals pass near, or None. `in_picture` says so for levelled points."""
    step = max(1, len(points) // config.max_sample_points)
    p, n = points[::step], normals[::step]
    rng = np.random.default_rng(0)  # the same picture gives the same answer
    width, height = size
    best, best_count = None, 0
    for _ in range(config.search_tries):
        i, j = rng.integers(0, len(p), size=2)
        cross = n[i, 0] * n[j, 1] - n[i, 1] * n[j, 0]
        if abs(cross) < 0.2:  # nearly parallel normals meet nowhere useful
            continue
        d = p[j] - p[i]
        t = (d[0] * n[j, 1] - d[1] * n[j, 0]) / cross
        candidate = p[i] + t * n[i]
        if not (in_picture(candidate) if in_picture is not None else (0.0 <= candidate[0] < width and 0.0 <= candidate[1] < height)):
            continue
        offset = candidate - p
        count = int(np.count_nonzero(np.abs(offset[:, 0] * n[:, 1] - offset[:, 1] * n[:, 0]) <= config.inlier_px))
        if count > best_count:
            best, best_count = candidate, count
    return best


def _centre_from_normals(points: np.ndarray, normals: np.ndarray) -> Optional[np.ndarray]:
    """The point nearest, in the least-squares sense, to every point's normal line."""
    projector = np.eye(2)[None, :, :] - normals[:, :, None] * normals[:, None, :]
    a = projector.sum(axis=0)
    if abs(np.linalg.det(a)) < 1.0e-6:
        return None
    return np.linalg.solve(a, np.einsum("nij,nj->i", projector, points))


def _rings(radius: np.ndarray, angle: np.ndarray, config: RingFitConfig) -> Tuple[np.ndarray, list]:
    """(the ring each point belongs to, or -1; the rings' radii) from the points' distances to the centre."""
    order = np.argsort(radius)
    order = order[np.isfinite(radius[order])]  # a point given no distance is not being looked at yet
    sorted_radius = radius[order]
    breaks = np.nonzero(np.diff(sorted_radius) > 1.5)[0] + 1
    label = np.full(len(radius), -1)
    radii = []
    for group in np.split(order, breaks):
        if len(group) < config.min_ring_points:
            continue
        seen = np.zeros(72, bool)  # the turn in 5 degree steps
        seen[(np.degrees(angle[group]) % 360.0 / 5.0).astype(int) % 72] = True
        if seen.sum() * 5.0 < config.min_ring_arc_deg:
            continue
        label[group] = len(radii)
        radii.append(float(np.median(radius[group])))
    return label, radii


def fit_rings(image_bgr: np.ndarray, config: Optional[RingFitConfig] = None, level: Optional[np.ndarray] = None) -> Optional[RingFit]:
    """The bullseye's centre from its rings, or None when the picture does not show at least two rings about a point inside it.

    `level` is the homography that levels the picture (`nadir_homography`);
    with it the fit is made on the levelled edge points and the centre is
    given back in the picture's own pixels. The radii are the levelled ones.
    """
    config = config or RingFitConfig()
    if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        return None
    height, width = image_bgr.shape[:2]
    points, normals = _edge_points(red_mask(image_bgr, config), config)
    if len(points) < config.min_edge_points:
        return None
    back = None
    if level is not None:
        level = np.asarray(level, dtype=np.float64).reshape(3, 3)
        try:
            back = np.linalg.inv(level)
        except np.linalg.LinAlgError:
            return None
        # A normal is square to the edge: move a short step along the edge with the point, and turn that.
        along = np.column_stack([-normals[:, 1], normals[:, 0]])
        moved = _apply(level, points)
        step = _apply(level, points + along) - moved
        length = np.hypot(step[:, 0], step[:, 1])
        keep = np.isfinite(moved).all(axis=1) & (length > 1.0e-6)
        points = moved[keep]
        normals = np.column_stack([step[keep, 1], -step[keep, 0]]) / length[keep, None]
        if len(points) < config.min_edge_points:
            return None

    def in_picture(point: np.ndarray) -> bool:
        if back is not None:
            point = _apply(back, point[None, :])[0]
        return bool(0.0 <= point[0] < width and 0.0 <= point[1] < height)

    centre = _search_centre(points, normals, (width, height), config, in_picture if back is not None else None)
    if centre is None:
        return None
    offset = centre - points
    agree = np.abs(offset[:, 0] * normals[:, 1] - offset[:, 1] * normals[:, 0]) <= config.inlier_px
    if agree.sum() < config.min_edge_points:
        return None
    refined = _centre_from_normals(points[agree], normals[agree])
    if refined is not None:
        centre = refined

    label, radii, rms = None, [], math.inf
    for _ in range(4):  # the rings, then the centre that makes them roundest, a few times over
        delta = points - centre
        radius = np.hypot(delta[:, 0], delta[:, 1])
        label, radii = _rings(radius, np.arctan2(delta[:, 1], delta[:, 0]), config)
        on_ring = label >= 0
        if len(radii) < config.min_rings or on_ring.sum() < config.min_edge_points:
            return None
        ring_radius = np.asarray(radii)[label[on_ring]]
        unit = delta[on_ring] / np.maximum(radius[on_ring], 1.0e-9)[:, None]
        residual = radius[on_ring] - ring_radius
        rms = float(np.sqrt(np.mean(residual * residual)))
        # Gauss-Newton on the centre: moving it by s changes each point's distance by -unit . s.
        step, *_rest = np.linalg.lstsq(-unit, -residual, rcond=None)
        centre = centre + step
        if float(np.hypot(*step)) < 0.01:
            break

    radii_sorted = sorted(radii)
    spacing = min(b - a for a, b in zip(radii_sorted, radii_sorted[1:]))
    inlier_fraction = float(np.count_nonzero(label >= 0)) / float(len(points))
    if spacing < config.min_ring_spacing_px or rms > config.max_rms_px or inlier_fraction < config.min_inlier_fraction or not in_picture(centre):
        return None
    if back is not None:
        centre = _apply(back, centre[None, :])[0]
    return RingFit(
        center=(float(centre[0]), float(centre[1])),
        radii_px=tuple(radii_sorted),
        rings=len(radii_sorted),
        rms_px=rms,
        inlier_fraction=inlier_fraction,
        points=int(len(points)),
    )


def to_detection(fit: Optional[RingFit]) -> Optional[Detection]:
    """The fit as a `bullseye` under the detection contract, or None for no fit.

    The box is the innermost ring fitted, about the centre. The score is the
    share of the edge points that belong to the rings, less for a fit on
    two rings only.
    """
    if fit is None:
        return None
    side = 2.0 * min(fit.radii_px)
    return Detection(
        class_id="bullseye",
        score=max(0.01, min(1.0, fit.inlier_fraction * min(1.0, fit.rings / 3.0))),
        cx=fit.center[0],
        cy=fit.center[1],
        width=side,
        height=side,
    )
