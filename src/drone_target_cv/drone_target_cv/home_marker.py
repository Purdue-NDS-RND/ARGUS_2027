"""The home marker (REC-7b, P1): an AprilTag lying at the return point, found in the downward camera's frame.

The tag is of the 36h11 family, 0.8 m across its black border (the spec's
size), with one id that the node is told (`home_marker_id`, 0 unless set).
`find(image, tag_id)` gives where its centre is in the picture, or None:
another id, or a tag whose code does not decode, is not the home marker.
What is found goes out on the detection contract as a `home_marker`, and the
target estimator puts it on the ground and tracks it like any other target;
the recovery mission takes that track as the place to set the bucket down
(recovery_return.HomingFix) inside the last ten metres.

OpenCV's aruco module reads the tag (its AprilTag dictionaries are in the
sim's image; whether the aircraft's OpenCV has them is to be checked there).
`available()` says
whether this OpenCV has it; without it `find` raises, and the node that
hosts it says so and leaves the marker out.

`cells(tag_id)` is the tag as a grid, black border included (8 x 8 for this
family): what a printed tag or a simulated one is made from.

No ROS imports.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, List, Optional, Tuple

import cv2
import numpy as np

FAMILY = "36h11"
SIZE_M = 0.8  # across the black border
DEFAULT_ID = 0
CELLS = 8  # across, border included
MIN_SIDE_PX = 16.0  # two pixels a cell: under that a decoded tag is not believed
MAX_SIDE_RATIO = 1.6  # the longest side over the shortest: a tag lying on the ground under a camera looking down is close to a square
SCORE = 0.9  # a decoded tag of the right id (the family corrects errors and rejects what is not a tag)
CLASS_ID = "home_marker"  # detection_contract.CLASSES (a test holds the two together)


@dataclass(frozen=True)
class Sighting:
    u: float  # the tag's centre, in pixels
    v: float
    side_px: float  # the mean length of its four sides
    corners: Tuple[Tuple[float, float], ...]  # four, in the order the tag's own corners are numbered


def available() -> bool:
    aruco = getattr(cv2, "aruco", None)
    return aruco is not None and hasattr(aruco, "DICT_APRILTAG_36h11")


def _dictionary() -> Any:
    if not available():
        raise RuntimeError("this OpenCV has no aruco module with the AprilTag 36h11 dictionary: the home marker cannot be read")
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)


MIN_MARKER_DISTANCE_RATE = 0.01  # a tag's one-cell white margin on darker ground is a second square just outside the tag: with the
# library's own 0.05 the outer one is kept and the tag inside it is dropped (found on made-up pictures: a tag square to the picture was never read)


def _detect(gray: np.ndarray) -> Tuple[Any, Any]:
    aruco, dictionary = cv2.aruco, _dictionary()
    new_api = hasattr(aruco, "ArucoDetector")  # OpenCV 4.7 and later
    parameters = aruco.DetectorParameters() if new_api else aruco.DetectorParameters_create()
    parameters.minMarkerDistanceRate = MIN_MARKER_DISTANCE_RATE
    if new_api:
        corners, ids, _rejected = aruco.ArucoDetector(dictionary, parameters).detectMarkers(gray)
    else:
        corners, ids, _rejected = aruco.detectMarkers(gray, dictionary, parameters=parameters)
    return corners, ids


def find(image: np.ndarray, tag_id: int = DEFAULT_ID) -> Optional[Sighting]:
    """The home marker in a picture (grey or BGR), or None. With more than one of the id in view, the largest."""
    if image is None or image.ndim not in (2, 3) or image.size == 0:
        return None
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids = _detect(np.ascontiguousarray(gray, dtype=np.uint8))
    best: Optional[Sighting] = None
    for quad, found_id in zip(corners or (), () if ids is None else ids.reshape(-1)):
        if int(found_id) != int(tag_id):
            continue
        points = np.asarray(quad, dtype=float).reshape(4, 2)
        sides = [float(np.linalg.norm(points[index] - points[(index + 1) % 4])) for index in range(4)]
        if min(sides) < MIN_SIDE_PX or max(sides) > MAX_SIDE_RATIO * min(sides):
            continue
        sighting = Sighting(float(points[:, 0].mean()), float(points[:, 1].mean()), float(np.mean(sides)), tuple((float(x), float(y)) for x, y in points))
        if best is None or sighting.side_px > best.side_px:
            best = sighting
    return best


def box(sighting: Sighting) -> Tuple[float, float, float, float]:
    """(centre u, centre v, width, height) of the sighting's box, for the detection contract: centred on the tag's centre."""
    xs, ys = [corner[0] for corner in sighting.corners], [corner[1] for corner in sighting.corners]
    return sighting.u, sighting.v, max(xs) - min(xs), max(ys) - min(ys)


def cells(tag_id: int = DEFAULT_ID) -> List[List[int]]:
    """The tag as CELLS x CELLS cells, row by row from its top left: 1 is white, 0 is black. The outer ring is the black border."""
    aruco, dictionary = cv2.aruco, _dictionary()
    draw = getattr(aruco, "generateImageMarker", None) or aruco.drawMarker
    picture = draw(dictionary, int(tag_id), CELLS, borderBits=1)
    return [[1 if value > 127 else 0 for value in row] for row in np.asarray(picture)]


def picture(tag_id: int = DEFAULT_ID, cell_px: int = 20, quiet_cells: int = 1) -> np.ndarray:
    """The tag as a grey picture with a white margin: for printing, and for tests."""
    grid = np.array(cells(tag_id), dtype=np.uint8) * 255
    grid = np.pad(grid, quiet_cells, constant_values=255)
    return np.kron(grid, np.ones((int(cell_px), int(cell_px)), dtype=np.uint8))
