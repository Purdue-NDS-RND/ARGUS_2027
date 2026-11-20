"""One frame through the localisation pipeline (LOC-5). No ROS.

  1. the whole frame is shrunk to 1280 x 720 (shrink.shrink_for_detector);
  2. the detector looks at it (`detect(image, stamp_ns)`, boxes in that
     image's pixels) and boxes under MIN_SCORE (0.25) are dropped;
  3. the boxes are scaled back to the full frame's pixels, which is what
     the detection contract carries;
  4. for each marker a crop is cut from the full frame, with a margin, for
     the digit reader (LOC-6): the frame itself never leaves the process;
  5. each box's centre is projected to the ground with the pose of the
     frame's capture time (projection v2, FND-05);
  6. the tracker (LOC-7) is handed the frame's ground points, and the digit
     reader its crops, when there is one.

With `tiling` the detector is given overlapping 1280 x 720 windows of the
full frame instead of the shrunk frame (tiling.py), and what the windows
find is merged.

A detector is anything with `name` and `detect(image, stamp_ns)`. The sim's
oracle (oracle.py) is one; the trained engine (PLT-04) will be another.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from drone_mission_core import detection_contract as contract
from drone_mission_core import target_estimator

from . import shrink
from . import tiling as tiling_module

MIN_SCORE = 0.25
CROP_CLASSES = ("marker", "boundary_marker")
CROP_MARGIN = 0.15  # of the box's side, on each side


MAX_GROUND_OFFSET_M = 20.0  # a site's ground further than this from home's height is a slip of the keyboard


class SiteGround:
    """The ground's height for the projector: the rangefinder's when it is believed, otherwise where the site says the ground is.

    `ground_below_home_m` is how far the ground under the survey is below
    home (negative: above it), the survey mission's own setting. It is also
    where the rangefinder is expected to find the ground.
    """

    def __init__(self, ground_height: Any, ground_below_home_m: Any = 0.0) -> None:
        if isinstance(ground_below_home_m, bool) or not isinstance(ground_below_home_m, (int, float)) or not abs(ground_below_home_m) <= MAX_GROUND_OFFSET_M:
            raise ValueError(f"ground_below_home_m must be a number of metres within {MAX_GROUND_OFFSET_M:g} m of home, got {ground_below_home_m!r}")
        self._ground_height, self._site_up_m = ground_height, -float(ground_below_home_m)

    @property
    def last_refusal(self) -> Any:
        return getattr(self._ground_height, "last_refusal", None)

    def ground_up(self, stamp_ns: int) -> Tuple[float, str]:
        up_m, source = self._ground_height.ground_up(int(stamp_ns), expected_ground_up_m=self._site_up_m)
        return (float(up_m), source) if source == "rangefinder" else (self._site_up_m, "home")


@dataclass(frozen=True)
class GroundDetection:
    class_id: str
    score: float
    east_m: float
    north_m: float
    detection: contract.Detection  # the box it came from, full-frame pixels
    point: Any  # projection v2's GroundPoint, with its error terms
    cut: bool = False  # the box reaches the edge of the picture: the part of the object in view, not the object


@dataclass(frozen=True)
class Crop:
    detection: contract.Detection
    image: np.ndarray  # a view of the full frame
    origin: Tuple[int, int]  # the crop's top-left pixel in the full frame


@dataclass
class FrameResult:
    stamp_ns: int
    seq: int
    detections: contract.DetectionSet
    ground: List[GroundDetection] = field(default_factory=list)
    crops: List[Crop] = field(default_factory=list)
    refused: int = 0  # boxes that could not be put on the ground
    refusal: str = ""  # why the last of them could not
    fault: str = ""  # a fault in the tracker or the digit reader: the frame's boxes still stand
    timings_ms: Dict[str, float] = field(default_factory=dict)


def cut_crop(image: np.ndarray, detection: contract.Detection, margin: float = CROP_MARGIN) -> Crop:
    """The box with a margin round it, cut to the frame."""
    half_w, half_h = detection.width * (0.5 + margin), detection.height * (0.5 + margin)
    x0, x1 = max(0, int(round(detection.cx - half_w))), min(image.shape[1], int(round(detection.cx + half_w)))
    y0, y1 = max(0, int(round(detection.cy - half_h))), min(image.shape[0], int(round(detection.cy + half_h)))
    return Crop(detection, image[y0:y1, x0:x1], (x0, y0))


class LocPipeline:
    def __init__(
        self,
        detector: Any,
        projector: Any,
        frame_id: str,
        full_size: Tuple[int, int],
        tracker: Any = None,
        digit_reader: Optional[Callable[[Sequence[Crop]], Any]] = None,
        tiling: bool = False,
        min_score: float = MIN_SCORE,
        pose_at: Optional[Callable[[int], Any]] = None,
    ) -> None:
        self.detector, self.projector, self.tracker, self.digit_reader = detector, projector, tracker, digit_reader
        self.frame_id = str(frame_id)
        self.full_size = (int(full_size[0]), int(full_size[1]))
        self.tiling = bool(tiling)
        self.min_score = float(min_score)
        self._pose_at = pose_at  # the pose history's own lookup, to know whether a frame's pose has arrived yet

    def pose_ready(self, stamp_ns: int) -> bool:
        """Whether the pose of a frame's capture time is there to place its boxes with. True when the pipeline was given no history to ask."""
        return self._pose_at is None or self._pose_at(int(stamp_ns)) is not None

    def _detect(self, image: np.ndarray, stamp_ns: int, timings: Dict[str, float]) -> List[contract.Detection]:
        if self.tiling:
            timings["shrink"] = 0.0
            started = time.perf_counter()
            found: List[contract.Detection] = []
            for window in tiling_module.windows_for(image.shape, shrink.DETECTOR_SIZE):
                found += [tiling_module.from_tile(d, window) for d in self.detector.detect(tiling_module.cut(image, window), stamp_ns) if d.score >= self.min_score]
            merged = tiling_module.merge(found)
            timings["detect"] = (time.perf_counter() - started) * 1000.0
            return merged
        started = time.perf_counter()
        small = shrink.shrink_for_detector(image)
        timings["shrink"] = (time.perf_counter() - started) * 1000.0
        started = time.perf_counter()
        found = [shrink.to_full_resolution(d, self.full_size) for d in self.detector.detect(small, stamp_ns) if d.score >= self.min_score]
        timings["detect"] = (time.perf_counter() - started) * 1000.0
        return found

    def process(self, frame: Any) -> FrameResult:
        """Everything for one frame (`image`, `capture_ns`, `seq`)."""
        image, stamp_ns = frame.image, int(frame.capture_ns)
        if (image.shape[1], image.shape[0]) != self.full_size:
            raise ValueError(f"the frame is {image.shape[1]} x {image.shape[0]}, not the calibrated mode's {self.full_size[0]} x {self.full_size[1]}: its pixels would not be the contract's")
        timings: Dict[str, float] = {}
        found = self._detect(image, stamp_ns, timings)
        frame_id = str(getattr(frame, "frame_id", "") or self.frame_id)  # the image's own frame when it names one
        result = FrameResult(stamp_ns, int(getattr(frame, "seq", 0)), contract.DetectionSet(stamp_ns, frame_id, tuple(found)), timings_ms=timings)
        result.crops = [cut_crop(image, d) for d in found if d.class_id in CROP_CLASSES]
        started = time.perf_counter()
        for detection in found:
            point = self.projector.project_pixel_to_ground((detection.cx, detection.cy), stamp_ns)
            if point is None:
                result.refused += 1
                result.refusal = str(self.projector.last_refusal or "the box could not be put on the ground")
                continue
            cut = target_estimator.box_is_cut(detection.cx, detection.cy, detection.width, detection.height, self.full_size[0], self.full_size[1])
            result.ground.append(GroundDetection(detection.class_id, detection.score, float(point.east_m), float(point.north_m), detection, point, cut))
        timings["project"] = (time.perf_counter() - started) * 1000.0
        # What follows the boxes must not cost the frame its boxes: a fault there is said, and the result stands.
        try:
            if self.tracker is not None:
                self.tracker.update(stamp_ns, result.ground)
            if self.digit_reader is not None and result.crops:
                self.digit_reader(result.crops)
        except Exception as error:  # noqa: BLE001
            result.fault = f"{type(error).__name__}: {error}"
        return result

