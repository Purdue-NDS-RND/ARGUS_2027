"""The one shrink between a full frame and the detector (LOC-5). No ROS.

`shrink_for_detector` is the function the flight node calls and the function
training images are made with (DAT-06), so that the detector flies on the
pixels it was trained on: the whole frame resampled with INTER_AREA to
1280 x 720. For a 3840 x 2160 frame that is exactly a third, and labels
scale by exactly a third (`label_scale`).

The 2027 engine is to have a static 1280 x 736 input (a height that is a
multiple of 32): `pad_for_engine` puts eight grey rows above and below, and
`from_engine` takes a box found in the padded frame back to the shrunk one.
They are for a detector that feeds that engine itself (PLT-04); the engine
detector here (engine.py) leaves the fitting to Ultralytics.
`to_full_resolution` takes it from there to the full frame's pixels, which
is what the detection contract (FND-02) carries.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Tuple

import numpy as np

from drone_mission_core import detection_contract as contract

DETECTOR_SIZE = (1280, 720)  # width, height: what shrink_for_detector gives
ENGINE_SIZE = (1280, 736)  # the engine's static input
PAD_ROWS = (ENGINE_SIZE[1] - DETECTOR_SIZE[1]) // 2
PAD_VALUE = 114  # the grey YOLO letterboxes with


def _check(image: np.ndarray) -> Tuple[int, int]:
    if not isinstance(image, np.ndarray) or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("the detector takes a colour frame: height x width x 3")
    height, width = image.shape[:2]
    if width * DETECTOR_SIZE[1] != height * DETECTOR_SIZE[0]:
        raise ValueError(f"the detector's shrink takes a 16:9 frame, got {width} x {height}")
    return width, height


def shrink_for_detector(image: np.ndarray) -> np.ndarray:
    """The whole frame at 1280 x 720 (INTER_AREA). A frame already that size is returned as it is."""
    width, height = _check(image)
    if (width, height) == DETECTOR_SIZE:
        return image
    import cv2

    return cv2.resize(image, DETECTOR_SIZE, interpolation=cv2.INTER_AREA)


def label_scale(full_size: Tuple[int, int]) -> float:
    """What a label's pixel coordinates in the full frame are multiplied by to be the shrunk frame's (1/3 for 4K)."""
    return DETECTOR_SIZE[0] / float(full_size[0])


def pad_for_engine(small: np.ndarray) -> np.ndarray:
    """The shrunk frame with PAD_ROWS grey rows above and below: the engine's 1280 x 736."""
    if small.shape[:2] != (DETECTOR_SIZE[1], DETECTOR_SIZE[0]):
        raise ValueError(f"the engine's frame is made from the 1280 x 720 one, got {small.shape[1]} x {small.shape[0]}")
    padded = np.full((ENGINE_SIZE[1], ENGINE_SIZE[0], 3), PAD_VALUE, dtype=small.dtype)
    padded[PAD_ROWS : PAD_ROWS + DETECTOR_SIZE[1]] = small
    return padded


def from_engine(detection: contract.Detection) -> contract.Detection:
    """A box found in the engine's padded frame, in the shrunk frame's pixels."""
    return replace(detection, cy=detection.cy - PAD_ROWS)


def to_full_resolution(detection: contract.Detection, full_size: Tuple[int, int]) -> contract.Detection:
    """A box in the shrunk frame's pixels, in the full frame's (the contract's)."""
    return contract.scale_to_full_resolution(detection, DETECTOR_SIZE, (int(full_size[0]), int(full_size[1])))
