"""Keyframe homing (REC-7b, P2): where the takeoff point is in the picture now, from pictures kept during the takeoff climb.

During the climb straight up from the takeoff point the downward camera's
pictures are kept at a few heights (`KeyframeHoming.offer`), each with the
pixel the takeoff point was at in it. On the way back a picture is matched
against the kept one of the nearest height (`find`): ORB features, matched
by their descriptors with the ratio test, and a homography fitted to the
matches by RANSAC. The takeoff point's pixel carried through that homography
is where the takeoff point is in the picture now. It goes out on the
detection contract as a `takeoff_point` (a class of its own: the mission must
know it is the takeoff point and not a marker somewhere else), and
the target estimator and the mission take it from there.

What is believed: at least MIN_INLIERS matches agreeing with the fitted
homography, a homography that is a view of flat ground from above (no
mirror image, little perspective), a change of size that goes with the two
heights when both are known, and a takeoff point that falls inside the
picture. Anything else is "not found": no answer is better than a wrong one,
for this is a place a bucket is set down on.

No ROS imports.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, List, Optional, Sequence, Tuple

import cv2
import numpy as np

HEIGHTS_M = (4.0, 7.0, 10.0, 14.0)  # a picture is kept as the climb passes each
HEIGHT_BAND_M = 0.75  # ... within this of it
FEATURES = 800
MIN_KEYPOINTS = 60  # a picture with fewer has too little in it to be found again
RATIO = 0.75  # the ratio test: the best match must be this much better than the second
MIN_MATCHES = 20
MIN_INLIERS = 15
REPROJECTION_PX = 3.0
MAX_PERSPECTIVE = 2.0e-4  # the homography's two perspective terms, per pixel: flat ground seen from above has next to none
SCALE_SLACK = 1.4  # the homography's change of size against the heights' ratio
MAX_SIDE_PX = 640  # pictures are matched no larger than this across
SCORE_AT_INLIERS = 60.0  # the score is the inliers over this, up to 0.9
CLASS_ID = "takeoff_point"
STILL_S = 2.0  # on the ground (the flight controller's landed state) and within STILL_M of one place for this long: that place is the takeoff point
STILL_M = 0.2
CLIMB_RADIUS_M = 1.5  # pictures are kept while the vehicle is within this of the takeoff point, until it first leaves
SEARCH_RADIUS_M = 10.0  # the takeoff point is looked for only within this of where it was: the spec's homing is the last ten metres, and a match costs the detector's thread tenths of a second


@dataclass
class Keyframe:
    height_m: float
    home_px: Tuple[float, float]  # the takeoff point in this picture, in the picture's own (full-size) pixels
    points: np.ndarray  # N x 2, the features' places in full-size pixels
    descriptors: np.ndarray


@dataclass(frozen=True)
class Sighting:
    u: float  # the takeoff point in the picture now, full-size pixels
    v: float
    inliers: int
    scale: float  # how many times larger the ground looks now than in the keyframe
    keyframe_height_m: float
    score: float


def _grey(image: np.ndarray) -> Tuple[np.ndarray, float]:
    """The picture in grey and no larger than MAX_SIDE_PX across, and how many full-size pixels one of its pixels is."""
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = np.ascontiguousarray(gray, dtype=np.uint8)
    longest = max(gray.shape[:2])
    if longest <= MAX_SIDE_PX:
        return gray, 1.0
    factor = longest / float(MAX_SIDE_PX)
    return cv2.resize(gray, (int(round(gray.shape[1] / factor)), int(round(gray.shape[0] / factor))), interpolation=cv2.INTER_AREA), factor


class KeyframeHoming:
    def __init__(self, heights_m: Sequence[float] = HEIGHTS_M) -> None:
        self.heights_m = tuple(float(height) for height in heights_m)
        self.keyframes: List[Keyframe] = []
        self._orb = cv2.ORB_create(nfeatures=FEATURES)
        self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.last_refusal: Optional[str] = None

    def _features(self, image: np.ndarray) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        gray, factor = _grey(image)
        keypoints, descriptors = self._orb.detectAndCompute(gray, None)
        points = np.array([keypoint.pt for keypoint in keypoints], dtype=np.float32).reshape(-1, 2) * factor
        return points, descriptors

    def wanted(self, height_m: float) -> Optional[float]:
        """The height of the list a picture taken at `height_m` would be kept for, or None (none near, or already kept)."""
        if not math.isfinite(height_m):
            return None
        for wanted in self.heights_m:
            if abs(height_m - wanted) <= HEIGHT_BAND_M and not any(abs(keyframe.height_m - wanted) <= HEIGHT_BAND_M for keyframe in self.keyframes):
                return wanted
        return None

    def offer(self, image: np.ndarray, height_m: float, home_px: Sequence[float]) -> bool:
        """A picture from the takeoff climb, with the pixel the takeoff point is at in it. True when it was kept."""
        if self.wanted(height_m) is None or image is None or image.size == 0:
            return False
        if not (0.0 <= home_px[0] < image.shape[1] and 0.0 <= home_px[1] < image.shape[0]):
            return False  # the takeoff point is not in the picture: not a picture of the takeoff point
        points, descriptors = self._features(image)
        if descriptors is None or len(points) < MIN_KEYPOINTS:
            self.last_refusal = f"a picture at {height_m:.1f} m has {len(points)} features, under {MIN_KEYPOINTS}: not kept"
            return False
        self.keyframes.append(Keyframe(float(height_m), (float(home_px[0]), float(home_px[1])), points, descriptors))
        return True

    def clear(self) -> None:
        """A new takeoff: the pictures of the last one are of another place."""
        self.keyframes = []

    def find(self, image: np.ndarray, height_m: Optional[float] = None) -> Optional[Sighting]:
        """The takeoff point in this picture, or None (`last_refusal` says why)."""
        self.last_refusal = None
        if not self.keyframes:
            self.last_refusal = "no picture was kept during the takeoff climb"
            return None
        points, descriptors = self._features(image)
        if descriptors is None or len(points) < MIN_MATCHES:
            self.last_refusal = f"the picture has {len(points)} features: nothing to match"
            return None
        known = height_m is not None and math.isfinite(height_m) and height_m > 0.5
        # With a height, the two kept nearest it (the features carry over about a factor of two in size); without one, every one of them.
        order = sorted(self.keyframes, key=lambda keyframe: abs(math.log(height_m / keyframe.height_m))) [:2] if known else list(self.keyframes)
        for keyframe in order:
            sighting = self._match(keyframe, points, descriptors, image.shape, height_m if known else None)
            if sighting is not None:
                self.last_refusal = None
                return sighting
        return None

    def _match(self, keyframe: Keyframe, points: np.ndarray, descriptors: np.ndarray, shape: Any, height_m: Optional[float]) -> Optional[Sighting]:
        pairs = self._matcher.knnMatch(keyframe.descriptors, descriptors, k=2)
        good = [pair[0] for pair in pairs if len(pair) == 2 and pair[0].distance < RATIO * pair[1].distance]
        if len(good) < MIN_MATCHES:
            self.last_refusal = f"{len(good)} features matched the picture kept at {keyframe.height_m:.1f} m, under {MIN_MATCHES}"
            return None
        before = np.float32([keyframe.points[match.queryIdx] for match in good]).reshape(-1, 1, 2)
        now = np.float32([points[match.trainIdx] for match in good]).reshape(-1, 1, 2)
        homography, mask = cv2.findHomography(before, now, cv2.RANSAC, REPROJECTION_PX)
        inliers = 0 if mask is None else int(mask.sum())
        if homography is None or inliers < MIN_INLIERS:
            self.last_refusal = f"{inliers} matches agree on one view of the picture kept at {keyframe.height_m:.1f} m, under {MIN_INLIERS}"
            return None
        homography = homography / homography[2, 2]
        determinant = float(np.linalg.det(homography[:2, :2]))
        if determinant <= 0.0 or max(abs(float(homography[2, 0])), abs(float(homography[2, 1]))) > MAX_PERSPECTIVE:
            self.last_refusal = "the matches fit a view that flat ground seen from above does not give"
            return None
        scale = math.sqrt(determinant)
        if height_m is not None:
            expected = keyframe.height_m / height_m
            if not expected / SCALE_SLACK <= scale <= expected * SCALE_SLACK:
                self.last_refusal = f"the ground looks {scale:.2f} times its size at {keyframe.height_m:.1f} m, and the heights say {expected:.2f}"
                return None
        home = cv2.perspectiveTransform(np.float32([[keyframe.home_px]]), homography)[0, 0]
        if not (0.0 <= float(home[0]) < shape[1] and 0.0 <= float(home[1]) < shape[0]):
            self.last_refusal = "the takeoff point is outside the picture"
            return None
        return Sighting(float(home[0]), float(home[1]), inliers, scale, keyframe.height_m, min(0.9, inliers / SCORE_AT_INLIERS))


class Climb:
    """Follows the vehicle: where the takeoff point is, and whether a picture now is one of the climb's or one to find the takeoff point in.

    The takeoff point is where the vehicle stands when the flight controller says it is on the ground (its landed
    state), kept up to date for as long as it stands there, so that the place is the one it leaves the ground
    from. Without the landed state nothing is decided: no height or stillness is taken for "on the ground".
    A landing after a flight is a new takeoff point, and the pictures of the last climb are of another place.
    """

    def __init__(self) -> None:
        self.home: Optional[Tuple[float, float, float]] = None  # east, north, up of the takeoff point
        self.left = False  # the vehicle has been further than CLIMB_RADIUS_M from it since the takeoff
        self.takeoffs = 0
        self._still: Optional[Tuple[float, float, float]] = None  # (since, east, north)
        self._airborne = False

    def update(self, t_s: float, position: Sequence[float], landed: Optional[bool]) -> bool:
        """The vehicle's position now, and whether the flight controller says it is on the ground (None: not known).
        True when a new takeoff point was just taken (the pictures of the last are to be dropped)."""
        east, north, up = (float(value) for value in position)
        if landed is None or not all(math.isfinite(value) for value in (t_s, east, north, up)):
            return False
        if not landed:
            self._still = None
            if self.home is not None:
                self._airborne = True
                if math.hypot(east - self.home[0], north - self.home[1]) > CLIMB_RADIUS_M:
                    self.left = True
            return False
        if self._still is None or math.hypot(east - self._still[1], north - self._still[2]) > STILL_M:
            self._still = (float(t_s), east, north)
        if t_s - self._still[0] < STILL_S:
            return False
        new = self.home is None or self._airborne
        self.home, self.left, self._airborne = (east, north, up), False, False  # (while it stands, the place is kept up to date)
        self.takeoffs += 1 if new else 0
        return new

    def height_m(self, position: Sequence[float]) -> Optional[float]:
        return None if self.home is None else float(position[2]) - self.home[2]

    def keeping(self, position: Sequence[float]) -> bool:
        """Whether a picture from here is one of the takeoff climb's."""
        return self.home is not None and not self.left and math.hypot(position[0] - self.home[0], position[1] - self.home[1]) <= CLIMB_RADIUS_M

    def searching(self, position: Sequence[float]) -> bool:
        """Whether the takeoff point is to be looked for in a picture from here: back within SEARCH_RADIUS_M after having left."""
        return self.home is not None and self.left and math.hypot(position[0] - self.home[0], position[1] - self.home[1]) <= SEARCH_RADIUS_M
