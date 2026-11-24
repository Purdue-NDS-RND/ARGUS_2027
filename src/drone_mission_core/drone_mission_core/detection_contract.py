"""The one detection contract (FND-02). No ROS imports.

Every detector, whatever it is built from, reports the same way:

  - a `vision_msgs/Detection2DArray` on /perception/<detector>/detections;
  - `header.stamp` is the capture time of the image it came from and
    `header.frame_id` that camera's optical frame;
  - each detection has one hypothesis whose class id is one of CLASSES, a
    string, with the detector's confidence (0 to 1) as its score;
  - the box centre and size are in pixels of the full-resolution calibrated
    image for the sensor mode. A detector that ran on a shrunk frame scales
    back up before it publishes (`scale_to_full_resolution`).

`DetectionSet` and `Detection` are the plain form; `encode` and `decode` go
to and from the message. They take the message classes as an argument, so
this module needs no ROS and the same code is tested with stand-ins.

The 2026 topic /drone_package_drop/target_detection carries a point, not a
box: x and y from -1 to 1 across the image and the red area in pixels, or
(-1, -1, 0) for "nothing found". `to_legacy_point` makes that from a
detection, for the adapter that keeps the 2026 missions fed.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Any, Iterable, Optional, Tuple

CLASSES = ("bullseye", "green_x", "marker", "boundary_marker", "bucket", "home_marker", "takeoff_point")  # the last two (REC-7b): a home marker as it is seen, and the takeoff point as the climb's keyframes show it
_DETECTOR_NAME = re.compile(r"[a-z][a-z0-9_]*")


class ContractError(ValueError):
    """Something that is not a detection under the contract. The message says what."""


@dataclass(frozen=True)
class Detection:
    class_id: str  # one of CLASSES
    score: float  # the detector's confidence, 0 to 1
    cx: float  # box centre, pixels of the full-resolution calibrated image
    cy: float
    width: float  # box size, same pixels
    height: float


@dataclass(frozen=True)
class DetectionSet:
    stamp_ns: int  # capture time of the image, whole nanoseconds
    frame_id: str  # the camera's optical frame
    detections: Tuple[Detection, ...] = ()


def topic_for(detector: str) -> str:
    """The topic a detector publishes on."""
    if not isinstance(detector, str) or not _DETECTOR_NAME.fullmatch(detector):
        raise ContractError(f"detector name {detector!r} must be lower case letters, digits and underscores")
    return f"/perception/{detector}/detections"


def _check(detection: Detection) -> None:
    if detection.class_id not in CLASSES:
        raise ContractError(f"class {detection.class_id!r} is not in the contract ({', '.join(CLASSES)})")
    if not (isinstance(detection.score, (int, float)) and 0.0 <= detection.score <= 1.0):
        raise ContractError(f"score {detection.score!r} must be the detector's confidence, from 0 to 1")
    if not (math.isfinite(detection.cx) and math.isfinite(detection.cy)):
        raise ContractError("the box centre must be finite pixel coordinates")
    if not (math.isfinite(detection.width) and math.isfinite(detection.height) and detection.width > 0.0 and detection.height > 0.0):
        raise ContractError("the box size must be above zero in both directions")


def _check_set(detection_set: DetectionSet) -> None:
    if isinstance(detection_set.stamp_ns, bool) or not isinstance(detection_set.stamp_ns, int) or detection_set.stamp_ns < 0:
        raise ContractError(f"stamp {detection_set.stamp_ns!r} must be the image's capture time in whole nanoseconds")
    if not detection_set.frame_id:
        raise ContractError("the frame id must be the camera's optical frame, not empty")
    for detection in detection_set.detections:
        _check(detection)


def encode(detection_set: DetectionSet, messages: Any) -> Any:
    """A Detection2DArray for the set. `messages` gives Detection2DArray, Detection2D and ObjectHypothesisWithPose (vision_msgs.msg)."""
    _check_set(detection_set)
    seconds, nanoseconds = divmod(detection_set.stamp_ns, 1_000_000_000)

    def stamp(header: Any) -> None:
        header.stamp.sec, header.stamp.nanosec = seconds, nanoseconds
        header.frame_id = detection_set.frame_id

    array = messages.Detection2DArray()
    stamp(array.header)
    for detection in detection_set.detections:
        message = messages.Detection2D()
        stamp(message.header)
        hypothesis = messages.ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = detection.class_id
        hypothesis.hypothesis.score = float(detection.score)
        message.results.append(hypothesis)
        message.bbox.center.position.x = float(detection.cx)
        message.bbox.center.position.y = float(detection.cy)
        message.bbox.center.theta = 0.0
        message.bbox.size_x = float(detection.width)
        message.bbox.size_y = float(detection.height)
        array.detections.append(message)
    return array


def decode(array: Any) -> DetectionSet:
    """The set in a Detection2DArray. A message that does not follow the contract is refused."""
    detections = []
    for message in array.detections:
        if len(message.results) != 1:
            raise ContractError(f"a detection carries one hypothesis under the contract, this one has {len(message.results)} hypotheses")
        hypothesis = message.results[0].hypothesis
        detections.append(
            Detection(
                class_id=str(hypothesis.class_id),
                score=float(hypothesis.score),
                cx=float(message.bbox.center.position.x),
                cy=float(message.bbox.center.position.y),
                width=float(message.bbox.size_x),
                height=float(message.bbox.size_y),
            )
        )
    result = DetectionSet(
        stamp_ns=int(array.header.stamp.sec) * 1_000_000_000 + int(array.header.stamp.nanosec),
        frame_id=str(array.header.frame_id),
        detections=tuple(detections),
    )
    _check_set(result)
    return result


def scale_to_full_resolution(detection: Detection, ran_on: Tuple[int, int], full: Tuple[int, int], letterbox: bool = False) -> Detection:
    """A detection found on a `ran_on` (width, height) frame, in the pixels of the `full` frame it was shrunk from.

    Without `letterbox` the two sizes must have the same shape. With it the
    frame was scaled to fit inside `ran_on` and padded equally on both sides,
    as detectors that need a size divisible by 32 do (1280 x 736 for 16:9).
    """
    ran_w, ran_h = ran_on
    full_w, full_h = full
    if min(ran_w, ran_h, full_w, full_h) <= 0:
        raise ContractError(f"image sizes must be above zero, got {ran_on} and {full}")
    if letterbox:
        scale = min(ran_w / full_w, ran_h / full_h)
        pad_x, pad_y = (ran_w - full_w * scale) / 2.0, (ran_h - full_h * scale) / 2.0
        scale_x = scale_y = 1.0 / scale
    else:
        if abs(ran_w * full_h - ran_h * full_w) > 1.0e-3 * ran_w * full_h:
            raise ContractError(f"{ran_w} x {ran_h} is not the shape of {full_w} x {full_h}: say letterbox=True if the frame was padded")
        pad_x = pad_y = 0.0
        scale_x, scale_y = full_w / ran_w, full_h / ran_h
    return Detection(
        class_id=detection.class_id,
        score=detection.score,
        cx=(detection.cx - pad_x) * scale_x,
        cy=(detection.cy - pad_y) * scale_y,
        width=detection.width * scale_x,
        height=detection.height * scale_y,
    )


def best_of_class(detections: Iterable[Detection], class_id: str) -> Optional[Detection]:
    """The most confident detection of a class, or None if there is none."""
    if class_id not in CLASSES:
        raise ContractError(f"class {class_id!r} is not in the contract ({', '.join(CLASSES)})")
    candidates = [detection for detection in detections if detection.class_id == class_id]
    return max(candidates, key=lambda detection: detection.score) if candidates else None


def to_legacy_point(detection: Optional[Detection], image_size: Tuple[int, int]) -> Tuple[float, float, float]:
    """(x, y, area) as the 2026 target-detection topic carries it: x and y from -1 to 1 across the image, area in pixels.

    None gives the 2026 "nothing found" point, (-1, -1, 0).
    """
    if detection is None:
        return (-1.0, -1.0, 0.0)
    width, height = image_size
    return (detection.cx / width * 2.0 - 1.0, detection.cy / height * 2.0 - 1.0, detection.width * detection.height)
