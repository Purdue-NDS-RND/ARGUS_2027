"""The digit reader (LOC-6): from a crop of a numbered marker to its digit, or to "I do not know". No ROS.

Behind the node flag `features.digit_reader`. For each crop the pipeline cuts
(pipeline.Crop: the full-resolution pixels round a marker's box):

  1. **Corners.** The plate is whiter than anything in the field and its
     triangles blacker: the pixels outside the greys of the crop's rim are
     the marker, and the four corners of their outline are the plate's.
  2. **Warp.** The corners, taken through the camera model to undistorted
     coordinates when the reader has one, give the homography to a 64 x 64
     square (PATCH_PX): the marker as seen from straight above.
  3. **Which way up.** The X's arms settle the quarter turn: the black
     triangles are top and bottom. The half turn is the marker's own cue
     when its layout has one (last year's plate has its digit in the
     right-hand white triangle). The 2027 marker, with its digit centred on
     the X, has none in this repo's model (the font and its 6/9 mark are an
     open question to the organisers).
  4. **The digit.** A `DigitClassifier` looks at the patch in all four
     quarter turns. With a cue, the turn the cue gives is the reading. With
     none, the two ways up the X allows are compared: a digit that reads
     one way only is that digit; a digit that reads the same both ways is
     that digit; a 6 or a 9 (each is the other upside down) is not
     answered, and the reading's probabilities are split between the two
     for the vote over the track (LOC-8). A quarter-turned reading surer
     than both is the X's arms and the digit disagreeing: not answered.
  5. **Abstaining.** No four corners, a marker under MIN_SIDE_PX across, no
     clear black triangles, or a best probability under `min_confidence`:
     the reading has no digit and says why. A wrong ID costs points and a
     marker not read is seen again (LOC-4), so the reader would sooner not
     answer.

Only `marker` crops are read (a boundary marker has no digit), at most
MAX_CROPS_PER_FRAME a frame, the most confident boxes first, within a time
budget, and no crop over MAX_CROP_SIDE_PX a side: the reader shares its
thread with the detector.

A `DigitClassifier` is anything with `name` and
`log_probabilities(patch) -> ten numbers`, the patch a 64 x 64 grey image
of the marker with its black triangles top and bottom. Two are here:

  - `TemplateDigitClassifier`, a stand-in for the trained network while
    training data is on hold. It knows the seven-segment digit of this
    repo's marker models (drone_utils.sdf_objects) and reads which segments
    are dark. It is behind its own node flag,
    `features.template_digit_classifier`.
  - `OnnxDigitClassifier`, the small CNN the spec asks for, run by ONNX
    Runtime on the CPU. Nothing here trains it; `load_onnx` refuses when
    there is no model file or no ONNX Runtime.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Optional, Sequence, Tuple

import numpy as np

PATCH_PX = 64
MIN_SIDE_PX = 24.0  # a marker narrower than this in the crop has a digit a few pixels high: not read
MIN_CONFIDENCE = 0.9
FIELD_GREY_MARGIN = 10.0  # a pixel this much lighter or darker than every grey the crop's rim has is the marker's
MAX_CROPS_PER_FRAME = 6
MAX_CROP_SIDE_PX = 512  # a crop bigger than this is not a 0.6 m marker from the survey's height (338 px at the 25 ft floor), and is not worked on
READ_BUDGET_S = 0.03  # the time a frame's crops may take before the rest wait for another frame
WARP_GRID = 9  # the lens is asked about this many points a side of the patch; between them it bends by a fraction of a pixel
SIX_NINE = (6, 9)
MIN_SEGMENT_MATCH = 0.5  # the stand-in calls nothing a digit unless every one of its seven segments is as that digit has it

DIGIT_SEGMENTS = {
    0: (1, 1, 1, 1, 1, 1, 0),
    1: (0, 1, 1, 0, 0, 0, 0),
    2: (1, 1, 0, 1, 1, 0, 1),
    3: (1, 1, 1, 1, 0, 0, 1),
    4: (0, 1, 1, 0, 0, 1, 1),
    5: (1, 0, 1, 1, 0, 1, 1),
    6: (1, 0, 1, 1, 1, 1, 1),
    7: (1, 1, 1, 0, 0, 0, 0),
    8: (1, 1, 1, 1, 1, 1, 1),
    9: (1, 1, 1, 1, 0, 1, 1),
}


def _grid() -> Tuple[np.ndarray, np.ndarray]:
    """The patch's pixel centres in marker units: (x right, y up), each from -1 to 1."""
    centres = (np.arange(PATCH_PX) + 0.5) / (PATCH_PX / 2.0) - 1.0
    return np.meshgrid(centres, -centres)


_X, _Y = _grid()
# The outer parts of the four triangles, clear of their edges, of the plate's rim and of anything at the crossing.
_INSIDE = (np.abs(_X) < 0.92) & (np.abs(_Y) < 0.92)
TOP = _INSIDE & (_Y > 0.55) & (np.abs(_X) < 0.95 * _Y - 0.12)
BOTTOM = _INSIDE & (_Y < -0.55) & (np.abs(_X) < 0.95 * -_Y - 0.12)
RIGHT = _INSIDE & (_X > 0.45) & (np.abs(_Y) < (_X - 0.05) / 0.95 - 0.12)
LEFT = _INSIDE & (_X < -0.45) & (np.abs(_Y) < (-_X - 0.05) / 0.95 - 0.12)
# Where the plate is white whatever the digit: the outer ends of the two white triangles.
WHITE_ENDS = _INSIDE & (np.abs(_X) > 0.86) & (np.abs(_Y) < 0.55)


@dataclass(frozen=True)
class MarkerLayout:
    """Where a marker model has its seven-segment digit, in units of half the marker's side (x right, y up)."""

    name: str
    digit_cx: float
    digit_cy: float
    digit_size: float  # the digit's overall height, as drone_utils.sdf_objects._build_digit_sdf takes it
    side_cue: bool  # the digit is off to one side of the X, which says which way up the marker is

    def segment_rects(self) -> Tuple[Tuple[float, float, float, float], ...]:
        """Centre x, centre y, width, height of segments A to G."""
        cx, cy, size = self.digit_cx, self.digit_cy, self.digit_size
        seg_len, seg_w, v_len = size * 0.4, size * 0.12, size * 0.35
        top_y, mid_y, bot_y = cy + size * 0.35, cy, cy - size * 0.35
        left_x, right_x = cx - seg_len * 0.4, cx + seg_len * 0.4
        return (
            (cx, top_y, seg_len, seg_w),
            (right_x, (top_y + mid_y) / 2, seg_w, v_len),
            (right_x, (mid_y + bot_y) / 2, seg_w, v_len),
            (cx, bot_y, seg_len, seg_w),
            (left_x, (mid_y + bot_y) / 2, seg_w, v_len),
            (left_x, (top_y + mid_y) / 2, seg_w, v_len),
            (cx, mid_y, seg_len, seg_w),
        )

    def segment_masks(self) -> Tuple[np.ndarray, ...]:
        """Where each segment is looked for: short of its ends, where its neighbours cross it, and a little wider
        than it is, so that a patch a pixel out of place still has the segment inside."""
        seg_w = self.digit_size * 0.12
        masks = []
        for cx, cy, width, height in self.segment_rects():
            if width >= height:  # a bar across
                masks.append((np.abs(_X - cx) <= width / 2.0 - 1.1 * seg_w) & (np.abs(_Y - cy) <= 0.8 * height))
            else:
                masks.append((np.abs(_X - cx) <= 0.8 * width) & (np.abs(_Y - cy) <= height / 2.0 - 1.1 * seg_w))
        return tuple(masks)

    def hole_masks(self) -> Tuple[np.ndarray, np.ndarray]:
        """The two openings of an 8: light in every digit, whatever is lit round them."""
        size = self.digit_size
        half_w, half_h = size * 0.4 * 0.35 / 2.0, size * 0.35 * 0.45 / 2.0
        return tuple((np.abs(_X - self.digit_cx) <= half_w) & (np.abs(_Y - (self.digit_cy + sign * size * 0.175)) <= half_h) for sign in (1.0, -1.0))


# The 2027 marker (build_marker_sdf): the digit centred on the X, half the marker high, no underline.
LAYOUT_2027 = MarkerLayout("x_2027", 0.0, 0.0, 1.0, side_cue=False)
# Last year's plate (build_bw_target_sdf, and tools/synth_targets.py's NumberedMarker): the digit in the right-hand white triangle.
LAYOUT_2026 = MarkerLayout("plate_2026", 0.65, -0.05, 0.84, side_cue=True)
LAYOUTS = {layout.name: layout for layout in (LAYOUT_2027, LAYOUT_2026)}


@dataclass(frozen=True)
class DigitReading:
    digit: Optional[int]  # None: the reader abstains
    confidence: float  # the best digit's probability, 0 to 1 (also when it abstains)
    log_probabilities: Tuple[float, ...]  # ten, for the vote over a track's sightings (LOC-8); all equal when nothing was read
    reason: str = ""  # why it abstained
    quarter_turns: int = 0  # how the patch was turned to stand the marker upright
    side_px: float = 0.0  # the marker's side in the crop, pixels
    sharpness: float = 0.0  # variance of the patch's Laplacian: higher is sharper
    corners: Optional[Tuple[Tuple[float, float], ...]] = None  # the plate's corners in the crop's pixels

    @property
    def abstained(self) -> bool:
        return self.digit is None


UNIFORM = tuple([math.log(0.1)] * 10)


def log_softmax(scores: Sequence[float]) -> np.ndarray:
    scores = np.asarray(scores, dtype=float)
    shifted = scores - scores.max()
    return shifted - math.log(float(np.exp(shifted).sum()))


# --- the classifiers ------------------------------------------------------------------


def patch_levels(patch: np.ndarray) -> Tuple[float, float]:
    """(white, black) grey levels of an upright patch: the white triangles' outer ends and the black triangles' medians."""
    return float(np.median(patch[WHITE_ENDS])), float(np.median(patch[TOP | BOTTOM]))


class TemplateDigitClassifier:
    """The stand-in: which of the seven segments are dark, against each digit's pattern. Not a trained model."""

    name = "template (stand-in)"

    def __init__(self, layout: MarkerLayout = LAYOUT_2027, sharpen: float = 3.0) -> None:
        self.layout = layout
        self._segments, self._holes = layout.segment_masks(), layout.hole_masks()
        self._sharpen = float(sharpen)

    def log_probabilities(self, patch: np.ndarray) -> np.ndarray:
        patch = np.asarray(patch, dtype=float)
        white, black = patch_levels(patch)
        if white - black < 20.0:
            return np.array(UNIFORM)  # no plate to read against
        darkness = np.clip((white - patch) / (white - black), 0.0, 1.0)
        if max(float(darkness[mask].mean()) for mask in self._holes) > 0.5:
            return np.array(UNIFORM)  # what is where the digit should be is not a digit
        # A segment that is there fills half of its (wider) mask: the darker half of the mask's pixels says whether it is.
        lit = np.array([float(np.sort(darkness[mask])[mask.sum() // 2 :].mean()) for mask in self._segments])
        scores, fits = [], []
        for digit in range(10):
            pattern = np.array(DIGIT_SEGMENTS[digit], dtype=float)
            match = pattern * lit + (1.0 - pattern) * (1.0 - lit)
            fits.append(float(match.min()))
            scores.append(self._sharpen * float(np.log(np.clip(match, 0.03, 1.0)).sum()))
        if max(fits) < MIN_SEGMENT_MATCH:
            return np.array(UNIFORM)  # no digit has all seven segments as the patch has them: a plate with no digit, or one upside down
        return log_softmax(scores)


def check_stand_in_sim_only(use_sim_time: bool, sim_only: bool) -> None:
    """Raise unless this is the sim. The template classifier knows the sim's marker model and nothing about a real
    marker: like the oracle detector, it needs the node on sim time and its sim-only parameter set."""
    if not use_sim_time or not sim_only:
        raise DigitClassifierUnavailable(
            "the template digit classifier is a stand-in for the sim's marker model and runs only in the sim: it needs use_sim_time true and the "
            f"node's sim_only parameter true (use_sim_time is {bool(use_sim_time)}, sim_only is {bool(sim_only)})"
        )


class DigitClassifierUnavailable(RuntimeError):
    """The trained digit classifier cannot run here. The message says why."""


class OnnxDigitClassifier:
    """The trained network: one 1 x 1 x 64 x 64 float image in (0 to 1), ten scores out. Run by ONNX Runtime on the CPU."""

    name = "onnx"

    def __init__(self, session: Any) -> None:
        self._session = session
        self._input = session.get_inputs()[0].name

    def log_probabilities(self, patch: np.ndarray) -> np.ndarray:
        image = (np.asarray(patch, dtype=np.float32) / 255.0)[None, None, :, :]
        scores = np.asarray(self._session.run(None, {self._input: image})[0], dtype=float).reshape(-1)
        if scores.shape != (10,):
            raise DigitClassifierUnavailable(f"the digit model gave {scores.size} scores for one marker, not ten")
        return log_softmax(scores)


def load_onnx(path: Any) -> OnnxDigitClassifier:
    """The classifier for a model file. Raises DigitClassifierUnavailable with no file or no ONNX Runtime."""
    if not str(path).strip():
        raise DigitClassifierUnavailable("the digit reader needs digit_model_path: the ONNX file of the trained classifier (none is built here)")
    file = Path(str(path)).expanduser()
    if not file.is_file():
        raise DigitClassifierUnavailable(f"digit_model_path '{file}' is not a file")
    try:
        import onnxruntime
    except ImportError as error:
        raise DigitClassifierUnavailable(f"the digit classifier runs in ONNX Runtime, which is not installed here ({error})") from error
    classifier = OnnxDigitClassifier(onnxruntime.InferenceSession(str(file), providers=["CPUExecutionProvider"]))
    classifier.log_probabilities(np.zeros((PATCH_PX, PATCH_PX), dtype=np.uint8))  # one blank marker now: a model that cannot run is refused at the start
    return classifier


# --- from a crop to an upright patch ----------------------------------------------------


def _corners_from_sides(outline: np.ndarray, quad: np.ndarray) -> np.ndarray:
    """The quad's corners again, as the crossings of lines fitted to the outline's points along each side.

    A corner taken from the outline is one whole pixel of it; a side has
    tens of points, and where two fitted sides cross is the corner to a
    fraction of a pixel. A side with too few points keeps its own corners.
    """
    import cv2

    lines = []
    for index in range(4):
        a, b = quad[index], quad[(index + 1) % 4]
        along = b - a
        length = float(np.linalg.norm(along))
        if length < 8.0:
            return quad
        along = along / length
        offset = outline - a
        position = offset @ along
        distance = np.abs(offset[:, 0] * along[1] - offset[:, 1] * along[0])
        # (The quad's own side can lie a few pixels inside the plate's: its corners are the outline's blunted tips.)
        on_side = outline[(distance < max(3.0, 0.06 * length)) & (position > 0.15 * length) & (position < 0.85 * length)]
        if len(on_side) < 6:
            return quad
        vx, vy, x0, y0 = cv2.fitLine(on_side.astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01).reshape(-1)
        lines.append((float(vx), float(vy), float(x0), float(y0)))
    corners = []
    for index in range(4):
        (vx1, vy1, x1, y1), (vx2, vy2, x2, y2) = lines[index - 1], lines[index]  # the side before the corner, and the one after
        cross = vx1 * vy2 - vy1 * vx2
        if abs(cross) < 1.0e-6:
            return quad
        t = ((x2 - x1) * vy2 - (y2 - y1) * vx2) / cross
        corners.append((x1 + t * vx1, y1 + t * vy1))
    corners = np.array(corners)
    return corners if float(np.linalg.norm(corners - quad, axis=1).max()) <= 0.08 * float(np.linalg.norm(quad[0] - quad[2])) else quad


def find_corners(crop: np.ndarray) -> Optional[np.ndarray]:
    """The plate's four corners in the crop's pixels, clockwise from the one nearest the top left. None if no four are found.

    The plate is whiter than anything in the field and its triangles
    blacker: the pixels outside the range of greys the crop's rim has (the
    field) are the marker. Its outline's hull gives four corners (the hull
    points nearest the corners of the smallest rectangle round it), and
    lines fitted along the sides give them to a fraction of a pixel.
    """
    import cv2

    height, width = crop.shape[:2]
    if height < 8 or width < 8:
        return None
    grey = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY).astype(float)
    rim = max(1, int(round(0.04 * min(height, width))))
    border = np.concatenate([grey[:rim].ravel(), grey[-rim:].ravel(), grey[:, :rim].ravel(), grey[:, -rim:].ravel()])
    low, high = np.percentile(border, 0.5) - FIELD_GREY_MARGIN, np.percentile(border, 99.5) + FIELD_GREY_MARGIN
    marker = ((grey < low) | (grey > high)).astype(np.uint8)
    mask = cv2.morphologyEx(marker, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    outline = max(contours, key=cv2.contourArea)
    if cv2.contourArea(outline) < 0.05 * height * width:
        return None
    hull = cv2.convexHull(outline).reshape(-1, 2).astype(float)
    box = cv2.boxPoints(cv2.minAreaRect(hull.astype(np.float32))).astype(float)
    picked = [int(np.argmin(np.linalg.norm(hull - corner, axis=1))) for corner in box]
    if len(set(picked)) < 4:
        return None
    quad = hull[picked]
    centre = quad.mean(axis=0)
    quad = quad[np.argsort(np.arctan2(quad[:, 1] - centre[1], quad[:, 0] - centre[0]))]  # clockwise in the image (y down)
    quad = _corners_from_sides(hull_points_between(hull), quad)
    return np.roll(quad, -int(np.argmin(quad.sum(axis=1))), axis=0)


def hull_points_between(hull: np.ndarray, step_px: float = 1.0) -> np.ndarray:
    """The hull's outline as points about a pixel apart (a hull keeps only its vertices; a side needs points along it)."""
    points = []
    for a, b in zip(hull, np.roll(hull, -1, axis=0)):
        count = max(1, int(np.linalg.norm(b - a) / step_px))
        points.extend(a + (b - a) * (index / count) for index in range(count))
    return np.array(points)


def warp_to_patch(crop: np.ndarray, corners: np.ndarray, origin: Tuple[int, int] = (0, 0), camera: Any = None) -> np.ndarray:
    """The marker as a 64 x 64 grey square. With a camera model the homography is found in undistorted coordinates."""
    import cv2

    square = np.array([[0.0, 0.0], [PATCH_PX, 0.0], [PATCH_PX, PATCH_PX], [0.0, PATCH_PX]], dtype=np.float32)
    grey = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY) if crop.ndim == 3 else crop
    # A marker much bigger than the patch is smoothed first: one sample for several pixels would pick up their noise.
    shrink = float(np.mean([np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)])) / PATCH_PX
    if shrink > 1.5:
        grey = cv2.GaussianBlur(grey, (0, 0), 0.4 * shrink)
    if camera is None:
        # The patch's pixel (i, j) is the square's point (i + 0.5, j + 0.5): its centre, as the masks take it.
        to_patch = np.array([[1.0, 0.0, -0.5], [0.0, 1.0, -0.5], [0.0, 0.0, 1.0]]) @ cv2.getPerspectiveTransform(corners.astype(np.float32), square)
        return cv2.warpPerspective(grey, to_patch, (PATCH_PX, PATCH_PX), flags=cv2.INTER_LINEAR)
    # The plate is flat, so its corners are a square's only once the lens's bending is taken out. The square's
    # grid is put back through the lens to the crop's own pixels, and the patch is sampled there.
    undistorted = np.array([camera.undistort((u + origin[0], v + origin[1]))[0] for u, v in corners], dtype=np.float32)
    to_plane = cv2.getPerspectiveTransform(square, undistorted)
    # The lens is asked about a coarse grid of the patch (WARP_GRID a side); between its points the map is
    # interpolated. Over one marker the lens bends by a fraction of a pixel between neighbouring points.
    stops = np.linspace(0.0, float(PATCH_PX), WARP_GRID, dtype=np.float32)
    grid = np.stack(np.meshgrid(stops, stops), axis=-1).reshape(-1, 1, 2)
    rays = cv2.perspectiveTransform(grid, to_plane).reshape(-1, 2)
    coarse = np.array([camera.pixel_of((float(x), float(y), 1.0)) for x, y in rays], dtype=np.float32) - np.array(origin, dtype=np.float32)
    coarse = coarse.reshape(WARP_GRID, WARP_GRID, 2)
    # From the coarse grid (at the patch's edges and between) to every pixel centre.
    step = PATCH_PX / float(WARP_GRID - 1)
    centres = (np.arange(PATCH_PX, dtype=np.float32) + 0.5) / step
    at_x, at_y = np.meshgrid(centres, centres)
    maps = cv2.remap(coarse, at_x, at_y, cv2.INTER_LINEAR)
    return cv2.remap(grey, maps[:, :, 0].copy(), maps[:, :, 1].copy(), cv2.INTER_LINEAR)


def upright(patch: np.ndarray, layout: MarkerLayout = LAYOUT_2027) -> Tuple[Optional[np.ndarray], int, bool, str]:
    """(the patch with its black triangles top and bottom, quarter turns made, whether the half turn is settled, why not).

    The X's arms settle the quarter turn. The half turn is settled only by
    a layout with its digit to one side (`side_cue`), and then only if one
    white triangle clearly has the digit. With no clear black pair there is
    no marker to turn: (None, 0, False, reason).
    """
    patch = np.asarray(patch, dtype=float)
    spread = float(np.percentile(patch, 97) - np.percentile(patch, 3))
    if spread < 40.0:
        return None, 0, False, "the patch has no black and white in it"
    vertical, sideways = float(patch[TOP | BOTTOM].mean()), float(patch[LEFT | RIGHT].mean())
    if abs(vertical - sideways) < 0.25 * spread:
        return None, 0, False, "no black triangles to stand the marker by"
    turns = 0
    if sideways < vertical:  # the black pair is left and right: a quarter turn
        patch, turns = np.rot90(patch), 1
    if not layout.side_cue:
        return patch, turns, False, ""
    middle = (float(np.median(patch[TOP | BOTTOM])) + float(np.percentile(patch[LEFT | RIGHT], 90))) / 2.0
    right, left = float((patch[RIGHT] < middle).mean()), float((patch[LEFT] < middle).mean())
    clear = max(right, left) >= 0.05 and max(right, left) >= 2.5 * min(right, left) + 0.01
    if left > right:  # the digit is on the left: a half turn
        patch, turns = np.rot90(patch, 2), turns + 2
    return patch, turns % 4, clear, "" if clear else "the digit's side of the marker is not clear"


def six_or_nine() -> Tuple[float, ...]:
    """Ten log-probabilities that say: a 6 or a 9, equally, and nothing else."""
    rest = 1.0e-6
    return tuple(math.log(0.5 - 4.0 * rest) if digit in SIX_NINE else math.log(rest) for digit in range(10))


class DigitReader:
    def __init__(
        self,
        classifier: Any,
        camera: Any = None,
        min_confidence: float = MIN_CONFIDENCE,
        layout: MarkerLayout = LAYOUT_2027,
        budget_s: float = READ_BUDGET_S,
    ) -> None:
        if isinstance(min_confidence, bool) or not isinstance(min_confidence, (int, float)) or not 0.5 <= float(min_confidence) <= 1.0:
            raise ValueError(f"the digit reader's min_confidence is a probability from 0.5 to 1, got {min_confidence!r}")
        self.classifier, self.camera, self.min_confidence, self.layout = classifier, camera, float(min_confidence), layout
        self.budget_s = float(budget_s)
        self.read_count = 0  # crops answered with a digit
        self.abstained_count = 0  # crops looked at and not answered
        self.skipped_count = 0  # crops not looked at: over the frame's limit of crops or of time
        self.faults = 0  # crops whose reading raised
        self.last_ms = 0.0  # what the last frame's crops took

    def _abstain(self, reason: str, **known: Any) -> DigitReading:
        self.abstained_count += 1
        return DigitReading(None, float(known.pop("confidence", 0.0)), tuple(known.pop("log_probabilities", UNIFORM)), reason, **known)

    def read(self, crop: Any) -> DigitReading:
        """One crop (`image`, and `origin` in the full frame when there is a camera model) to a reading."""
        import cv2

        image = np.asarray(crop.image)
        if max(image.shape[:2]) > MAX_CROP_SIDE_PX:
            return self._abstain(f"the crop is {image.shape[1]} x {image.shape[0]} px: too big to be a marker from the survey's height")
        corners = find_corners(image)
        if corners is None:
            return self._abstain("no marker outline in the crop")
        sides = [float(np.linalg.norm(corners[i] - corners[(i + 1) % 4])) for i in range(4)]
        side = float(np.mean(sides))
        found = tuple((float(x), float(y)) for x, y in corners)
        if min(sides) < MIN_SIDE_PX or max(sides) > 2.0 * min(sides):
            return self._abstain(f"the marker is {min(sides):.0f} to {max(sides):.0f} px a side: too small or not a square", side_px=side, corners=found)
        patch = warp_to_patch(image, corners, tuple(getattr(crop, "origin", (0, 0))), self.camera)
        sharpness = float(cv2.Laplacian(patch, cv2.CV_64F).var())
        stood, turns, settled, why = upright(patch, self.layout)
        known = {"side_px": side, "sharpness": sharpness, "corners": found}
        if stood is None:
            return self._abstain(why, **known)
        # The classifier sees all four quarter turns of the marker as it stands by its X.
        by_turn = [np.asarray(self.classifier.log_probabilities(np.ascontiguousarray(np.rot90(stood, k)).astype(np.uint8)), dtype=float) for k in range(4)]
        best = [int(np.argmax(scores)) for scores in by_turn]
        confidence = [float(np.exp(scores.max())) for scores in by_turn]
        sure = [value >= self.min_confidence for value in confidence]
        if settled:
            chosen = 0
        else:
            # The X allows two ways up. A quarter-turned reading surer than both of them is the arms and the digit disagreeing.
            if max(confidence[1], confidence[3]) >= self.min_confidence and not (sure[0] or sure[2]):
                return self._abstain("the X's arms and the digit disagree about which way up the marker is", confidence=max(confidence), **known)
            if sure[0] and sure[2] and best[0] != best[2]:
                if {best[0], best[2]} == set(SIX_NINE):
                    return self._abstain("a 6 or a 9: the marker does not say which way up it is", confidence=0.5, log_probabilities=six_or_nine(), **known)
                return self._abstain(f"a {best[0]} one way up and a {best[2]} the other", confidence=max(confidence[0], confidence[2]), **known)
            chosen = 0 if confidence[0] >= confidence[2] else 2
            # A 6 one way up is a 9 the other, whether or not the other way's reading came out sure.
            if sure[chosen] and best[chosen] in SIX_NINE:
                return self._abstain("a 6 or a 9: the marker does not say which way up it is", confidence=0.5, log_probabilities=six_or_nine(), **known)
        scores = by_turn[chosen]
        if not settled:
            # A 6 this way up is a 9 the other: with nothing to say which way up, the reading gives the two the
            # same chance, whatever it answers. Otherwise the noise between the two ways up would add up, over
            # a track's sightings, to a vote for one of them.
            scores = scores.copy()
            scores[list(SIX_NINE)] = math.log(0.5 * float(np.exp(scores[SIX_NINE[0]]) + np.exp(scores[SIX_NINE[1]])) + 1.0e-300)
        known["quarter_turns"] = (turns + chosen) % 4
        if confidence[chosen] < self.min_confidence:
            reason = why or f"the best digit has probability {confidence[chosen]:.2f}, under {self.min_confidence:g}"
            return self._abstain(reason, confidence=confidence[chosen], log_probabilities=tuple(float(v) for v in scores), **known)
        self.read_count += 1
        return DigitReading(best[chosen], confidence[chosen], tuple(float(v) for v in scores), "", **known)

    def read_all(self, crops: Sequence[Any]) -> List[DigitReading]:
        """A frame's crops, in their order. Only numbered markers are read; at most MAX_CROPS_PER_FRAME, the most
        confident boxes first; and when the frame's time is spent the rest are left for another frame. A crop
        whose reading raises is that crop's fault alone."""
        started = time.perf_counter()
        readings: List[Optional[DigitReading]] = [None] * len(crops)
        wanted = []
        for index, crop in enumerate(crops):
            detection = getattr(crop, "detection", None)
            if detection is not None and getattr(detection, "class_id", "marker") != "marker":
                readings[index] = DigitReading(None, 0.0, UNIFORM, "a boundary marker has no digit")
            else:
                wanted.append(index)
        wanted.sort(key=lambda index: -float(getattr(getattr(crops[index], "detection", None), "score", 0.0)))
        for position, index in enumerate(wanted):
            if position >= MAX_CROPS_PER_FRAME:
                self.skipped_count += 1
                readings[index] = DigitReading(None, 0.0, UNIFORM, f"more markers in the frame than the {MAX_CROPS_PER_FRAME} that are read")
            elif position > 0 and time.perf_counter() - started > self.budget_s:
                self.skipped_count += 1
                readings[index] = DigitReading(None, 0.0, UNIFORM, "no time left in this frame")
            else:
                try:
                    readings[index] = self.read(crops[index])
                except Exception as error:  # noqa: BLE001  (one crop's fault must not cost the frame's other readings)
                    self.faults += 1
                    readings[index] = DigitReading(None, 0.0, UNIFORM, f"the reading failed ({type(error).__name__}: {error})")
        self.last_ms = (time.perf_counter() - started) * 1000.0
        return [reading for reading in readings if reading is not None]

    __call__ = read_all
