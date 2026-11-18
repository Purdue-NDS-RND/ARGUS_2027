"""The 2026 red-target detector's result as a detection under the contract (FND-02). No ROS imports."""

from __future__ import annotations

from typing import Any, Optional, Tuple

import cv2

from drone_mission_core.detection_contract import Detection, DetectionSet, best_of_class, to_legacy_point


def bullseye_detection(result: Any) -> Optional[Detection]:
    """A `bullseye` detection for an accepted result, or None when the detector found nothing.

    The centre is the detector's own centre. The box is the extent of the red
    pixels of the target it chose (a square of the cluster's area if it kept
    no mask).
    The detector does not shrink its input, so these are the pixels of the
    image it was given: the full-resolution image when the driver sends that
    (the 2027 camera driver), a reduced one from the 2026 grabbers.
    """
    if result.center is None:
        return None
    red = getattr(result, "selected_red_mask", None)
    points = None if red is None else cv2.findNonZero(red)
    if points is not None:
        _x, _y, width, height = cv2.boundingRect(points)  # the red pixels of the target it chose
    else:
        width = height = max(1.0, float(result.cluster_area_px)) ** 0.5
    return Detection(
        class_id="bullseye",
        score=min(1.0, max(0.0, float(result.confidence))),
        cx=float(result.center[0]),
        cy=float(result.center[1]),
        width=float(max(1, width)),
        height=float(max(1, height)),
    )


def to_detection_set(result: Any, image_stamp: Any, frame_id: str) -> Optional[DetectionSet]:
    """What one frame gave, for the contract topic: the image's stamp, its frame, and the bullseye if there is one.

    None when the image has no stamp or no frame id: the contract asks for
    the capture time and the camera's optical frame, and a guess at either
    would be worse than not publishing.
    """
    if image_stamp is None or not (image_stamp.sec or image_stamp.nanosec) or not frame_id:
        return None
    detection = bullseye_detection(result)
    return DetectionSet(
        stamp_ns=int(image_stamp.sec) * 1_000_000_000 + int(image_stamp.nanosec),
        frame_id=str(frame_id),
        detections=() if detection is None else (detection,),
    )


ADAPTER_MIN_SCORE = 0.25  # the 2026 detector passes nothing under this confidence to the missions (min_detection_confidence)
ADAPTER_MAX_AHEAD_S = 0.1  # a stamp further ahead of the clock than this is from another time base


def legacy_messages(
    detection_set: DetectionSet,
    image_size: Tuple[int, int],
    min_score: float = ADAPTER_MIN_SCORE,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """The two 2026 messages for a contract set: the target point, and the image size point (width, height, 0).

    The target is the most confident `bullseye` whose score is at least
    `min_score` and whose centre is inside the image; anything else is the
    2026 "nothing found".
    """
    width, height = image_size
    usable = [
        detection
        for detection in detection_set.detections
        if detection.score >= min_score and 0.0 <= detection.cx < width and 0.0 <= detection.cy < height
    ]
    bullseye = best_of_class(usable, "bullseye")
    return to_legacy_point(bullseye, image_size), (float(width), float(height), 0.0)


def stamp_is_usable(stamp_ns: int, now_ns: int, max_ahead_s: float = ADAPTER_MAX_AHEAD_S) -> bool:
    """Whether a contract stamp can be given to the 2026 missions as a detection time.

    The missions only ask whether a detection is under two seconds old, so one
    stamped ahead of their clock would never go stale. An old stamp is passed
    on: the missions' own timeout deals with it.
    """
    return int(stamp_ns) - int(now_ns) <= int(max_ahead_s * 1.0e9)
