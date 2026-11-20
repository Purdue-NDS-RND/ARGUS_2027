"""Tiles of the full frame for the detector (SAHI-style), and the merge of what the tiles find (LOC-5). No ROS.

Behind the node flag `features.loc_tiling`, off by default: the pipeline
normally shrinks the whole frame once. With tiling the detector sees the
frame at its own resolution in overlapping 1280 x 720 windows (16 of them
for a 4K frame at 20% overlap), which finds smaller things and costs 16
inferences a frame.
"""

from __future__ import annotations

import math
from dataclasses import replace
from typing import Iterable, List, Sequence, Tuple

from drone_mission_core import detection_contract as contract

Window = Tuple[int, int, int, int]  # x, y, width, height in the full frame


def _starts(full: int, tile: int, overlap: float) -> List[int]:
    if tile >= full:
        return [0]
    count = max(2, int(math.ceil((full - tile * overlap) / (tile * (1.0 - overlap)))))
    return [int(round(index * (full - tile) / (count - 1))) for index in range(count)]


def tile_windows(full_size: Tuple[int, int], tile_size: Tuple[int, int], overlap: float = 0.2) -> List[Window]:
    """Windows that cover the frame, each `tile_size`, neighbours sharing at least `overlap` of a tile."""
    if not 0.0 <= overlap < 1.0:
        raise ValueError(f"overlap is a share of a tile from 0 up to 1, got {overlap!r}")
    width, height = min(tile_size[0], full_size[0]), min(tile_size[1], full_size[1])
    return [(x, y, width, height) for y in _starts(full_size[1], height, overlap) for x in _starts(full_size[0], width, overlap)]


def from_tile(detection: contract.Detection, window: Window) -> contract.Detection:
    """A box found in a tile, in the full frame's pixels."""
    return replace(detection, cx=detection.cx + window[0], cy=detection.cy + window[1])


def _iou(a: contract.Detection, b: contract.Detection) -> float:
    left, right = max(a.cx - a.width / 2, b.cx - b.width / 2), min(a.cx + a.width / 2, b.cx + b.width / 2)
    top, bottom = max(a.cy - a.height / 2, b.cy - b.height / 2), min(a.cy + a.height / 2, b.cy + b.height / 2)
    shared = max(0.0, right - left) * max(0.0, bottom - top)
    union = a.width * a.height + b.width * b.height - shared
    return shared / union if union > 0.0 else 0.0


def merge(detections: Iterable[contract.Detection], iou: float = 0.5) -> List[contract.Detection]:
    """One box for each thing two tiles both found: of boxes of one class that overlap, the most confident stays."""
    kept: List[contract.Detection] = []
    for candidate in sorted(detections, key=lambda item: -item.score):
        if not any(other.class_id == candidate.class_id and _iou(other, candidate) >= iou for other in kept):
            kept.append(candidate)
    return kept


def cut(image, window: Window):
    """The window's pixels (a view, not a copy)."""
    x, y, width, height = window
    return image[y : y + height, x : x + width]


def windows_for(image_shape: Sequence[int], tile_size: Tuple[int, int], overlap: float = 0.2) -> List[Window]:
    return tile_windows((int(image_shape[1]), int(image_shape[0])), tile_size, overlap)
