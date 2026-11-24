"""The green X and the bucket on it, found in one picture (REC-1). No ROS.

Classical vision, no trained model: behind the node flag `features.recovery_cv`
(recovery_cv_node.py). `detect(image)` returns what it finds:

  - **The X.** Green pixels (hue, after CLAHE on the brightness so that a
    shadow across the mat does not split an arm) in white: the mat. Grass
    is green too and the arms' tips touch the mat's edge, so the X is not
    a green blob of its own. It is looked for as the green that white
    shuts in (the whole mat in the picture), and failing that inside each
    big white patch's hull (the mat running out of the picture, its arms
    reaching the edge). It is an X when two lines fit it: the first by RANSAC over
    its pixels, the second by RANSAC over what the first left, each then
    refined with a Huber fit. The centre is where the two lines cross, so a
    bucket standing on the crossing does not move it. The X's angle is an
    arm's direction, which repeats every 90 degrees.
  - **Not an X.** Grass and a green tarp are green with no white round
    them: the mat between the arms must be white (`white_surround`). One
    bar, two lines far from square to each other, or "arms" as wide as
    they are long (a green disc) is not an X.
  - **Confidence** is the product of four shares, each 0 to 1: the white
    surround, how square the arms are to each other, the share of the
    blob's pixels on the two arms, and how alike the arms' widths are.
    Under MIN_SCORE nothing is reported.
  - **The bucket.** On the crossing, a blob that is neither the mat's white
    nor the arms' green, of about a bucket's size against the arm width;
    its centre from an ellipse fit. Its handle, when the bucket is large
    enough in the picture to show one, is the longest line through it.
  - **The near field.** Under about 3 m the mat fills the picture and its
    arms run out of every side: no X can be fitted. The bucket is then
    looked for by itself (`_near_bucket`): in a picture that is mostly the
    mat's green and white, a coloured, round, solid blob with mat all the
    way round it, standing on the arms' crossing and a bucket's size
    against the arm's width. No `green_x` is reported from there, only the
    `bucket` and its handle.

Sizes are taken from the picture itself (the arm's width is the ruler), so
the detector needs no height; the mat's proportions are the sim's and the
synthetic tool's (drone_utils.sdf_objects.build_x_mat_sdf). A big picture is
shrunk first, and everything reported is in the pixels of the picture given.

numpy and cv2 only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import numpy as np

MIN_SCORE = 0.5
WORK_WIDTH_PX = 640  # a wider picture is shrunk to this before anything is looked for
GREEN_HUE = (35, 90)  # OpenCV's 0 to 179
GREEN_MIN_SATURATION = 70
GREEN_MIN_VALUE = 40
WHITE_MAX_SATURATION = 45  # grass in sun is pale but not this pale
WHITE_OVER_ARM = 1.15  # the mat is at least this much brighter than the arms
WHITE_MIN_VALUE = 150  # and at least this bright as the camera gave it, to be a patch worth looking into
MIN_MAT_SHARE = 0.004  # of the picture's pixels: a smaller white patch is not a mat
MIN_PIECE_SHARE = 0.12  # of the largest white piece: smaller ones are not part of the mat
MAX_CLOSE_PX = 161  # the widest gap the white is closed over, in the shrunk picture: an arm's width from 2 m
MIN_BLOB_SHARE = 0.0008  # of the picture's pixels: less green than this inside a mat is not looked at
MIN_ARM_SHARE = 0.18  # each arm has at least this share of the blob's pixels
MIN_INLIER_FRACTION = 0.8
MIN_ARM_LENGTH_OVER_WIDTH = 3.0  # of what shows of an arm (a whole arm is 13 times its width)
ARM_ANGLE_TOLERANCE_DEG = 25.0  # arms this far from square score nothing (a tilted mat is a few degrees off)
MIN_WHITE_SURROUND = 0.6
RANSAC_ITERATIONS = 120
MAX_FIT_POINTS = 2500
# The mat (build_x_mat_sdf): 5 m square, arms 0.5 m wide on the diagonals, stopping 0.25 m short of the corners.
ARM_LENGTH_OVER_WIDTH = 2.0 * (2.5 * math.sqrt(2.0) - 0.25) / 0.5  # 13.1: an arm's length, tip to tip, in arm widths
BUCKET_OVER_ARM_WIDTH = 0.2 / 0.5  # the bucket's diameter in arm widths
BUCKET_MIN_SOLIDITY = 0.6  # the blob's area over its hull's
BUCKET_MAX_ELONGATION = 2.0  # its length over its width
QUARTER_CLOSE_OVER_PX = 41  # a wider closing of the white is done on a quarter-size mask: the cost of a frame stays bounded
MIN_HANDLE_BUCKET_PX = 24.0  # a bucket narrower than this in the picture shows no handle
# The near field: under about 3 m the mat fills the picture, the arms run out of it on every side and no X can be
# fitted. The bucket is then looked for by itself, on a picture that is plainly the mat seen from close by.
NEAR_MIN_GREEN_SHARE = 0.12  # of the picture is the arms' green,
NEAR_MIN_WHITE_SHARE = 0.03  # some of it is still the mat's white,
NEAR_MIN_MAT_SHARE = 0.6  # and green and white together are most of it
NEAR_MIN_SATURATION = 90  # the bucket is coloured: not the mat's white, not a shadow's grey
NEAR_MIN_DIAMETER_SHARE = 0.03  # of the picture's width: a bucket from 3 m is about 0.04 of it, and larger from nearer
NEAR_MAX_DIAMETER_SHARE = 0.6
NEAR_MIN_RING = 0.8  # of the ring round the blob is the mat's green or white: the bucket stands on the mat
NEAR_CLOSE_SHARE = 0.025  # of the picture's width: a gap this wide across the blob (the handle's bar) is closed over
# The near bucket is held to the X as the far one is (`_bucket`): it stands on the arms' crossing and is a bucket's
# size against the arm's width. With no X fitted, the crossing is read off the green itself.
NEAR_RING_OVER_INSCRIBED = 1.1  # a circle this much wider than the largest one that fits in the arms round the blob
NEAR_RING_SAMPLES = 90
NEAR_MIN_RING_IN_PICTURE = 0.6  # of that circle must be in the picture to count the arms on it
NEAR_ARMS = (3, 5)  # green stretches on that circle: four at a crossing (two along one arm, none or one on grass or a tarp)


@dataclass(frozen=True)
class XDetection:
    cx: float  # where the arms cross, pixels of the picture given
    cy: float
    angle_rad: float  # an arm's direction, 0 to pi/2 (the X looks the same turned a quarter)
    span_px: float  # an arm's length, tip to tip
    arm_width_px: float
    score: float
    parts: Dict[str, float] = field(default_factory=dict)  # white_surround, arm_angle, inlier_fraction, arm_width


@dataclass(frozen=True)
class BucketDetection:
    cx: float
    cy: float
    width: float  # the fitted ellipse's axes, pixels
    height: float
    score: float
    handle: Optional[Tuple[Tuple[float, float], Tuple[float, float]]] = None  # its two ends, when it shows


@dataclass(frozen=True)
class Result:
    x: Optional[XDetection] = None
    bucket: Optional[BucketDetection] = None


def _ransac_line(points: np.ndarray, band: float, rng: np.random.Generator) -> Optional[np.ndarray]:
    """The inliers (a boolean mask) of the line through two of the points that has the most points within `band` of it."""
    count = len(points)
    if count < 8:
        return None
    pairs = rng.integers(0, count, size=(RANSAC_ITERATIONS, 2))
    a, b = points[pairs[:, 0]], points[pairs[:, 1]]
    along = b - a
    length = np.hypot(along[:, 0], along[:, 1])
    usable = length > 2.0 * band
    if not usable.any():
        return None
    a, along, length = a[usable], along[usable], length[usable]
    normal = np.stack([-along[:, 1], along[:, 0]], axis=1) / length[:, None]
    # Distance of every point from every candidate line.
    distance = np.abs(points @ normal.T - np.einsum("ij,ij->i", a, normal)[None, :])
    inside = distance <= band
    best = int(np.argmax(inside.sum(axis=0)))
    return inside[:, best]


def _huber_line(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """(a point on the line, its unit direction) of a Huber fit through the points."""
    import cv2

    vx, vy, x0, y0 = cv2.fitLine(points.astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(-1)
    return np.array([float(x0), float(y0)]), np.array([float(vx), float(vy)])


def _distance_to(points: np.ndarray, origin: np.ndarray, direction: np.ndarray) -> np.ndarray:
    offset = points - origin
    return np.abs(offset[:, 0] * direction[1] - offset[:, 1] * direction[0])


def _fit_x(points: np.ndarray, band: float, rng: np.random.Generator):
    """Two arms through a blob's pixels: ((origin, direction), (origin, direction), share of pixels on either, the arms' shares) or None."""
    first = _ransac_line(points, band, rng)
    if first is None or first.mean() < MIN_ARM_SHARE:
        return None
    one = _huber_line(points[first])
    rest = points[_distance_to(points, *one) > band]
    if len(rest) < MIN_ARM_SHARE * len(points):
        return None
    second = _ransac_line(rest, band, rng)
    if second is None or second.sum() < MIN_ARM_SHARE * len(points):
        return None
    two = _huber_line(rest[second])
    # Each arm again, on every pixel near it (the crossing's pixels belong to both).
    one = _huber_line(points[_distance_to(points, *one) <= band])
    two = _huber_line(points[_distance_to(points, *two) <= band])
    near_one, near_two = _distance_to(points, *one) <= band, _distance_to(points, *two) <= band
    return one, two, float((near_one | near_two).mean()), (float(near_one.mean()), float(near_two.mean()))


def _crossing(one, two) -> Optional[np.ndarray]:
    (p, d), (q, e) = one, two
    cross = d[0] * e[1] - d[1] * e[0]
    if abs(cross) < 1.0e-6:
        return None
    t = ((q[0] - p[0]) * e[1] - (q[1] - p[1]) * e[0]) / cross
    return p + t * d


def _arm_width(points: np.ndarray, arm, other, band: float) -> float:
    """An arm's width in pixels: from the spread of its pixels across it, away from the crossing."""
    origin, direction = arm
    near = _distance_to(points, origin, direction) <= band
    clear = _distance_to(points, *other) > 2.0 * band  # not where the other arm adds its own pixels
    chosen = points[near & clear]
    if len(chosen) < 20:
        chosen = points[near]
    offset = chosen - origin
    across = offset[:, 0] * direction[1] - offset[:, 1] * direction[0]
    return float(np.std(across) * math.sqrt(12.0))  # a bar's pixels are spread evenly across it


def _white_surround(hsv: np.ndarray, centre: np.ndarray, one, two, width: float, arm_value: float) -> float:
    """The share of the mat between the arms that is white: sampled along the four lines half way between the arms."""
    height, wide = hsv.shape[:2]
    bisectors = []
    for sign in (1.0, -1.0):
        direction = one[1] + sign * two[1]
        norm = float(np.hypot(*direction))
        if norm < 1.0e-6:
            return 0.0
        bisectors += [direction / norm, -direction / norm]
    white = total = 0
    for direction in bisectors:
        for reach in np.linspace(0.9, 4.0, 12):  # in arm widths: clear of a bucket on the crossing, inside the mat
            x, y = centre + direction * reach * width
            column, row = int(round(x)), int(round(y))
            if not (0 <= column < wide and 0 <= row < height):
                continue
            total += 1
            _hue, saturation, value = (int(v) for v in hsv[row, column])
            white += saturation <= WHITE_MAX_SATURATION and value >= WHITE_OVER_ARM * arm_value
    if total < 8:  # under a sixth of the samples are in the picture: not enough to say
        return 0.0
    return white / total


def _bucket(image: np.ndarray, hsv: np.ndarray, green: np.ndarray, centre: np.ndarray, width: float, arm_value: float) -> Optional[BucketDetection]:
    """A blob on the crossing that is neither mat nor arm, about a bucket's size against the arm's width."""
    import cv2

    height, wide = green.shape
    reach = int(math.ceil(1.6 * width))
    x0, x1 = max(0, int(centre[0]) - reach), min(wide, int(centre[0]) + reach + 1)
    y0, y1 = max(0, int(centre[1]) - reach), min(height, int(centre[1]) + reach + 1)
    if x1 - x0 < 5 or y1 - y0 < 5:
        return None
    window = hsv[y0:y1, x0:x1]
    white = (window[:, :, 1] <= WHITE_MAX_SATURATION) & (window[:, :, 2] >= WHITE_OVER_ARM * arm_value)
    other = (~white & ~(green[y0:y1, x0:x1] > 0)).astype(np.uint8)
    other = cv2.morphologyEx(other, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(other, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    expected = BUCKET_OVER_ARM_WIDTH * width
    best = None
    for contour in contours:
        area = float(cv2.contourArea(contour))
        diameter = 2.0 * math.sqrt(area / math.pi) if area > 0.0 else 0.0
        if not 0.5 * expected <= diameter <= 2.0 * expected:
            continue
        moments = cv2.moments(contour)
        middle = np.array([moments["m10"] / moments["m00"] + x0, moments["m01"] / moments["m00"] + y0])
        if float(np.hypot(*(middle - centre))) > 0.75 * width:
            continue
        if best is None or area > best[0]:
            best = (area, contour, middle, diameter)
    if best is None:
        return None
    area, contour, middle, diameter = best
    axes = (diameter, diameter)
    if len(contour) >= 5:
        (ex, ey), (major, minor), _angle = cv2.fitEllipse(contour)
        if 0.5 * diameter <= minor <= major <= 2.0 * diameter:
            middle, axes = np.array([ex + x0, ey + y0]), (float(major), float(minor))
    # A bucket from above is round and solid. A shadow across the crossing, or a hook hanging under the camera,
    # is long or ragged: a blob more than twice as long as wide, or filling under 0.6 of its own hull, is not one.
    hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
    (_rx, _ry), (rect_w, rect_h), _turn = cv2.minAreaRect(contour)
    if hull_area <= 0.0 or area / hull_area < BUCKET_MIN_SOLIDITY or max(rect_w, rect_h) > BUCKET_MAX_ELONGATION * max(1.0, min(rect_w, rect_h)):
        return None
    size_score = min(diameter, expected) / max(diameter, expected)
    handle = _handle(image, middle, max(axes)) if max(axes) >= MIN_HANDLE_BUCKET_PX else None
    return BucketDetection(float(middle[0]), float(middle[1]), float(axes[0]), float(axes[1]), float(size_score), handle)


def _green_stretches(green: np.ndarray, middle: np.ndarray, radius: float) -> Optional[int]:
    """How many separate stretches of green a circle about `middle` passes through, or None when too little of the circle is in the picture.

    A stretch cut by the picture's edge counts as one. Short breaks (a shadow's edge, a pixel of noise) do not split a stretch.
    """
    height, wide = green.shape
    samples = []
    for index in range(NEAR_RING_SAMPLES):
        angle = 2.0 * math.pi * index / NEAR_RING_SAMPLES
        x, y = int(round(middle[0] + radius * math.cos(angle))), int(round(middle[1] + radius * math.sin(angle)))
        samples.append(None if not (0 <= x < wide and 0 <= y < height) else bool(green[y, x]))
    inside = [value for value in samples if value is not None]
    if len(inside) < NEAR_MIN_RING_IN_PICTURE * NEAR_RING_SAMPLES:
        return None
    # Close breaks of one sample (4 degrees), then count the runs of green round the circle; the picture's edge ends a run.
    marks = [value is True for value in samples]
    closed = [marks[index] or (marks[index - 1] and marks[(index + 1) % len(marks)] and samples[index] is not None) for index in range(len(marks))]
    if all(closed):
        return 0  # green all the way round: grass, or a tarp
    return sum(1 for index in range(len(closed)) if closed[index] and not closed[index - 1])


def _near_bucket(image: np.ndarray, hsv: np.ndarray, green: np.ndarray, white: np.ndarray) -> Optional[BucketDetection]:
    """The bucket by itself, when the mat fills the picture and no X can be fitted: a coloured round blob standing on the mat.

    Only a picture that is mostly the mat's green and white is looked at, and the blob is held to the X as the far
    bucket is: it must stand on the arms' crossing (a circle round it cuts four arms) and be a bucket's size
    against the arm's width, both read off the green itself. An orange thing on grass, a red target ringed in
    white, or a coloured thing lying along one arm is not a bucket.
    """
    import cv2

    height, wide = green.shape
    pixels = float(height * wide)
    green_share, white_share = float(np.count_nonzero(green)) / pixels, float(np.count_nonzero(white)) / pixels
    if green_share < NEAR_MIN_GREEN_SHARE or white_share < NEAR_MIN_WHITE_SHARE or green_share + white_share < NEAR_MIN_MAT_SHARE:
        return None
    mat = ((green > 0) | (white > 0)).astype(np.uint8)
    other = ((mat == 0) & (hsv[:, :, 1] >= NEAR_MIN_SATURATION) & (hsv[:, :, 2] >= GREEN_MIN_VALUE)).astype(np.uint8)
    other = cv2.morphologyEx(other, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    # The handle's bar (and a hook's rod) lies across the bucket, nearer the camera and so wider than it is: the two
    # halves it cuts the bucket into are closed into one blob again.
    gap = max(3, int(round(NEAR_CLOSE_SHARE * wide)) | 1)
    other = cv2.morphologyEx(other, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (gap, gap)))
    contours, _ = cv2.findContours(other, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for contour in contours:
        area = float(cv2.contourArea(contour))
        diameter = 2.0 * math.sqrt(area / math.pi) if area > 0.0 else 0.0
        if not NEAR_MIN_DIAMETER_SHARE * wide <= diameter <= NEAR_MAX_DIAMETER_SHARE * wide:
            continue
        hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
        (_rx, _ry), (rect_w, rect_h), _turn = cv2.minAreaRect(contour)
        if hull_area <= 0.0 or area / hull_area < BUCKET_MIN_SOLIDITY or max(rect_w, rect_h) > BUCKET_MAX_ELONGATION * max(1.0, min(rect_w, rect_h)):
            continue
        # The ring round it, from 1.15 to 1.5 of its radius: mat all the way round (a ring cut by the picture's edge counts what shows).
        moments = cv2.moments(contour)
        middle = np.array([moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]])
        ring = np.zeros((height, wide), np.uint8)
        cv2.circle(ring, (int(round(middle[0])), int(round(middle[1]))), int(math.ceil(0.75 * diameter)), 1, -1)
        cv2.circle(ring, (int(round(middle[0])), int(round(middle[1]))), int(math.ceil(0.575 * diameter)), 0, -1)
        around = int(np.count_nonzero(ring))
        if around < 20:
            continue
        ring_share = float(np.count_nonzero(ring & mat)) / around
        if ring_share < NEAR_MIN_RING:
            continue
        # On the crossing, and a bucket's size against the arm: the arms with the blob's own hole filled, and the
        # largest circle about the blob's middle that stays inside them. At a crossing of arms w wide that circle
        # is 1.41 w across, and a circle a little wider cuts four arms; along one arm it cuts two, on grass none.
        blob = np.zeros((height, wide), np.uint8)
        cv2.drawContours(blob, [contour], -1, 1, -1)
        # (the seam between the blob and the green, a pixel or two of neither where the colours meet, is closed over)
        arms = cv2.morphologyEx(((green > 0) | (blob > 0)).astype(np.uint8), cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (gap, gap)))
        row, col = min(height - 1, max(0, int(round(middle[1])))), min(wide - 1, max(0, int(round(middle[0]))))
        inscribed = float(cv2.distanceTransform(arms, cv2.DIST_L2, 5)[row, col])
        arm_width = inscribed * math.sqrt(2.0)
        if arm_width <= 0.0 or not 0.5 * BUCKET_OVER_ARM_WIDTH <= diameter / arm_width <= 2.0 * BUCKET_OVER_ARM_WIDTH:
            continue
        stretches = _green_stretches(arms, middle, NEAR_RING_OVER_INSCRIBED * inscribed)
        if stretches is None or not NEAR_ARMS[0] <= stretches <= NEAR_ARMS[1]:
            continue
        if best is None or area > best[0]:
            best = (area, contour, middle, diameter, ring_share * min(1.0, area / hull_area))
    if best is None:
        return None
    _area, contour, middle, diameter, score = best
    axes = (diameter, diameter)
    if len(contour) >= 5:
        (ex, ey), (major, minor), _angle = cv2.fitEllipse(contour)
        if 0.5 * diameter <= minor <= major <= 2.0 * diameter:
            middle, axes = np.array([ex, ey]), (float(major), float(minor))
    handle = _handle(image, middle, max(axes)) if max(axes) >= MIN_HANDLE_BUCKET_PX else None
    return BucketDetection(float(middle[0]), float(middle[1]), float(axes[0]), float(axes[1]), float(score), handle)


def _handle(image: np.ndarray, middle: np.ndarray, diameter: float):
    """The longest straight line through the bucket's middle, as its two ends, or None."""
    import cv2

    reach = int(math.ceil(0.75 * diameter))
    x0, y0 = max(0, int(middle[0]) - reach), max(0, int(middle[1]) - reach)
    patch = cv2.cvtColor(image[y0 : int(middle[1]) + reach + 1, x0 : int(middle[0]) + reach + 1], cv2.COLOR_BGR2GRAY)
    if patch.shape[0] < 8 or patch.shape[1] < 8:
        return None
    edges = cv2.Canny(patch, 60, 160)
    lines = cv2.HoughLinesP(edges, 1, math.pi / 180.0, threshold=max(8, int(0.3 * diameter)), minLineLength=0.6 * diameter, maxLineGap=3)
    if lines is None:
        return None
    local = middle - np.array([x0, y0])
    best = None
    for x1, y1, x2, y2 in lines.reshape(-1, 4).astype(float):
        length = math.hypot(x2 - x1, y2 - y1)
        direction = np.array([x2 - x1, y2 - y1]) / length
        if _distance_to(local[None, :], np.array([x1, y1]), direction)[0] > 0.2 * diameter:
            continue  # not through the middle: the rim, or the arm's edge
        if best is None or length > best[0]:
            best = (length, (x1 + x0, y1 + y0), (x2 + x0, y2 + y0))
    return None if best is None else (best[1], best[2])


def _judge(arms: np.ndarray, hsv: np.ndarray, scale: float, rng: np.random.Generator):
    """Whether the green pixels of `arms` (a boolean mask) are an X: (XDetection, crossing, arm width, arm brightness) or None."""
    area = int(arms.sum())
    if area < MIN_BLOB_SHARE * arms.size:
        return None
    rows, columns = np.nonzero(arms)
    points = np.stack([columns, rows], axis=1).astype(float)
    if len(points) > MAX_FIT_POINTS:
        points = points[rng.choice(len(points), MAX_FIT_POINTS, replace=False)]
    # A first measure of the arm's width, to gate the fits with: two bars of this length have this area.
    extent = 2.0 * float(np.percentile(np.hypot(*(points - points.mean(axis=0)).T), 98))
    rough_width = area / (2.0 * max(extent, 1.0))
    band = max(1.5, 0.75 * rough_width)
    fitted = _fit_x(points, band, rng)
    if fitted is None:
        return None
    one, two, inlier_fraction, _shares = fitted
    centre = _crossing(one, two)
    if centre is None or inlier_fraction < MIN_INLIER_FRACTION:
        return None
    between = math.degrees(math.acos(min(1.0, abs(float(one[1] @ two[1])))))  # 0 to 90
    arm_angle = max(0.0, 1.0 - (90.0 - between) / ARM_ANGLE_TOLERANCE_DEG)
    widths = (_arm_width(points, one, two, band), _arm_width(points, two, one, band))
    if min(widths) < 1.0:
        return None
    width = 0.5 * (widths[0] + widths[1])
    # An arm is a bar, several times longer than it is wide in whatever part of it shows. A green disc (grass
    # through a hole in something white) has two "arms" as wide as they are long.
    for origin, direction in (one, two):
        near = points[_distance_to(points, origin, direction) <= band]
        along = (near - origin) @ direction
        if len(near) < 8 or float(along.max() - along.min()) < MIN_ARM_LENGTH_OVER_WIDTH * width:
            return None
    arm_width = min(widths) / max(widths)
    arm_value = float(np.median(hsv[:, :, 2][arms]))
    surround = _white_surround(hsv, centre, one, two, width, arm_value)
    if surround < MIN_WHITE_SURROUND:
        return None
    parts = {"white_surround": float(surround), "arm_angle": float(arm_angle), "inlier_fraction": float(inlier_fraction), "arm_width": float(arm_width)}
    score = float(np.prod(list(parts.values())))
    if score < MIN_SCORE:
        return None
    # The X's angle: both arms' directions, folded to the quarter turn the X repeats in, and averaged. (On a
    # tilted mat the arms are not square in the picture, and one arm alone is a degree or two off.)
    folded = [4.0 * math.atan2(direction[1], direction[0]) for direction in (one[1], two[1])]
    angle = (math.atan2(sum(math.sin(a) for a in folded), sum(math.cos(a) for a in folded)) / 4.0) % (math.pi / 2.0)
    found = XDetection(float(centre[0] / scale), float(centre[1] / scale), float(angle), float(ARM_LENGTH_OVER_WIDTH * width / scale), float(width / scale), score, parts)
    return found, centre, width, arm_value


def _enclosed_green(green: np.ndarray, white: np.ndarray) -> list:
    """Green blobs shut in by white: green that cannot be reached from the picture's edge without crossing white.

    This is the X with the whole mat in the picture, whatever else is white in it (a runway's lines, another
    mat). The arms' tips touch the mat's edge, and a pixel or two of blended edge would let the grass in there:
    the white is first grown by four pixels, which seals the tips, and what is outside is then grown back by as much.
    """
    import cv2

    grow = np.ones((9, 9), np.uint8)
    barrier = cv2.dilate(white, grow)
    count, labels = cv2.connectedComponents((barrier == 0).astype(np.uint8), connectivity=4)
    edge = np.unique(np.concatenate([labels[0], labels[-1], labels[:, 0], labels[:, -1]]))
    outside = ((barrier == 0) & np.isin(labels, edge)).astype(np.uint8)
    outside = cv2.dilate(outside, grow)
    arms = ((green > 0) & (outside == 0)).astype(np.uint8)
    blobs, blob_labels, stats, _ = cv2.connectedComponentsWithStats(arms, connectivity=8)
    order = sorted(range(1, blobs), key=lambda label: -int(stats[label, cv2.CC_STAT_AREA]))[:3]
    return [blob_labels == label for label in order]


def _green_in_mats(green: np.ndarray, white: np.ndarray) -> list:
    """Green inside each big white patch's hull: the X when the mat runs out of the picture and its arms reach the edge.

    The arms cut the mat's white into pieces. The pieces of a size with the largest are closed over a gap of
    about an arm's width (a third of a piece's size) into one patch; pale specks of grass round the mat, closed
    in with it, would bring their grass inside, so smaller pieces are left out.
    """
    import cv2

    pieces, piece_labels, piece_stats, _ = cv2.connectedComponentsWithStats(white, connectivity=8)
    if pieces < 2:
        return []
    largest = float(piece_stats[1:, cv2.CC_STAT_AREA].max())
    kept = [label for label in range(1, pieces) if piece_stats[label, cv2.CC_STAT_AREA] >= MIN_PIECE_SHARE * largest]
    mats = np.isin(piece_labels, kept).astype(np.uint8)
    gap = int(min(MAX_CLOSE_PX, max(5, round(0.35 * math.sqrt(largest))))) | 1
    if gap > QUARTER_CLOSE_OVER_PX:
        # A wide gap (the mat close in, or a picture that is mostly pale ground) on a quarter-size mask: the same
        # patch to a few pixels, at a sixteenth of the work.
        small = cv2.resize(mats, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
        small = cv2.morphologyEx(small, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, ((gap // 4) | 1, (gap // 4) | 1)))
        mats = cv2.resize(small, (mats.shape[1], mats.shape[0]), interpolation=cv2.INTER_NEAREST) | mats
    else:
        mats = cv2.morphologyEx(mats, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (gap, gap)))
    count, labels, stats, _centroids = cv2.connectedComponentsWithStats(mats, connectivity=8)
    found = []
    for label in sorted(range(1, count), key=lambda item: -int(stats[item, cv2.CC_STAT_AREA]))[:3]:
        if int(stats[label, cv2.CC_STAT_AREA]) < MIN_MAT_SHARE * white.size:
            break
        contours, _ = cv2.findContours((labels == label).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        inside = np.zeros_like(white)
        cv2.fillConvexPoly(inside, cv2.convexHull(np.vstack(contours)), 1)
        found.append((green > 0) & (inside > 0))
    return found


def detect(image: np.ndarray, seed: int = 0) -> Result:
    """The X and the bucket in a BGR picture, in its own pixels. `seed` fixes the RANSAC's draws: the same picture, the same answer."""
    import cv2

    if image is None or image.ndim != 3 or image.shape[0] < 16 or image.shape[1] < 16:
        return Result()
    scale = min(1.0, WORK_WIDTH_PX / float(image.shape[1]))
    work = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale < 1.0 else image
    hsv = cv2.cvtColor(work, cv2.COLOR_BGR2HSV)
    value = hsv[:, :, 2].copy()  # the brightness as the camera gave it, for what is white
    # CLAHE on the brightness: an arm in shadow stays one arm.
    hsv[:, :, 2] = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(hsv[:, :, 2])
    green = cv2.inRange(hsv, (GREEN_HUE[0], GREEN_MIN_SATURATION, GREEN_MIN_VALUE), (GREEN_HUE[1], 255, 255))
    green = cv2.morphologyEx(green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    # The mat: grass is green too, so the X is not a blob of its own in the green. It is green in white.
    white = ((hsv[:, :, 1] <= WHITE_MAX_SATURATION) & (value >= WHITE_MIN_VALUE)).astype(np.uint8)
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    rng = np.random.default_rng(seed)
    best = None
    # First the green that white shuts in (the whole mat in the picture); failing that, the green inside each
    # big white patch (the mat running out of the picture).
    for candidates in (_enclosed_green, _green_in_mats):
        for arms in candidates(green, white):
            judged = _judge(arms, hsv, scale, rng)
            if judged is not None and (best is None or judged[0].score > best[0].score):
                best = judged
        if best is not None:
            break
    if best is None:
        # No X: from close by the mat fills the picture and its arms have no ends. The bucket is looked for by itself.
        found, bucket = None, _near_bucket(work, hsv, green, white)
    else:
        found, centre, width, arm_value = best
        bucket = _bucket(work, hsv, green, centre, width, arm_value)
    if bucket is not None and scale < 1.0:
        handle = None if bucket.handle is None else tuple((x / scale, y / scale) for x, y in bucket.handle)
        bucket = BucketDetection(bucket.cx / scale, bucket.cy / scale, bucket.width / scale, bucket.height / scale, bucket.score, handle)
    return Result(found, bucket)


def to_detections(result: Result) -> tuple:
    """What was found, on the detection contract: a `green_x` and a `bucket`, in the picture's pixels.

    The X's box is centred where the arms cross and holds both arms at the X's angle. The bucket's is its
    fitted ellipse's. Each is put on the ground at its own class's plane height by whoever reads it.
    """
    from drone_mission_core import detection_contract as contract

    found = []
    if result.x is not None:
        side = result.x.span_px * max(abs(math.cos(result.x.angle_rad)), abs(math.sin(result.x.angle_rad)))
        found.append(contract.Detection("green_x", min(1.0, result.x.score), result.x.cx, result.x.cy, side, side))
    if result.bucket is not None:
        found.append(contract.Detection("bucket", min(1.0, result.bucket.score), result.bucket.cx, result.bucket.cy, result.bucket.width, result.bucket.height))
    return tuple(found)
