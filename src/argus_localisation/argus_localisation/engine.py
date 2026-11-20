"""The trained detector's engine as a pipeline detector (LOC-5). No ROS; Ultralytics is imported only when an engine is loaded.

`detector: engine` runs a TensorRT engine the way the 2026 node did
(vision_pipeline/yolo_node.py): through Ultralytics' `YOLO(<file>.engine)`,
on the Jetson, where Ultralytics and TensorRT are installed. Which engine is
the node's `engine_path`; nothing here builds or trains one (PLT-04 and the
DAT lane, on hold). Until PLT-04 gives the 2027 engine (1280 x 736, the
contract's classes) the file is last year's, whose one class is the
black-and-white marker: `engine_classes` says what each of the engine's
class numbers is on the detection contract.

On a machine without Ultralytics, with no engine file, or with an engine
that does not run a blank frame when it is loaded, `load_engine` refuses
with the reason and the node does not start.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence, Tuple

from drone_mission_core import detection_contract as contract


class EngineUnavailable(RuntimeError):
    """The engine detector cannot run here. The message says why."""


def _number(value: Any) -> float:
    """A float from a plain number or from a tensor on any device."""
    if hasattr(value, "cpu"):
        value = value.cpu().numpy()
    return float(value)


class EngineDetector:
    name = "engine"

    def __init__(self, model: Any, classes: Sequence[str], confidence: float = 0.25) -> None:
        classes = [str(item) for item in classes]
        unknown = [item for item in classes if item not in contract.CLASSES]
        if not classes or unknown:
            raise EngineUnavailable(
                f"engine_classes must name a class of the detection contract ({', '.join(contract.CLASSES)}) for each of the engine's class numbers, got {classes}"
            )
        self._model, self._classes, self._confidence = model, classes, float(confidence)
        self.unknown_classes = 0  # boxes of a class number the engine_classes list does not reach

    def detect(self, image: Any, stamp_ns: int) -> Tuple[contract.Detection, ...]:
        """The engine's boxes for an image, in that image's pixels."""
        results = self._model(image, conf=self._confidence, verbose=False)
        found = []
        for box in results[0].boxes:
            x1, y1, x2, y2 = (_number(value) for value in box.xyxy[0])
            number = int(_number(box.cls[0]))
            if not 0 <= number < len(self._classes):
                self.unknown_classes += 1
                continue
            found.append(contract.Detection(self._classes[number], _number(box.conf[0]), (x1 + x2) / 2.0, (y1 + y2) / 2.0, x2 - x1, y2 - y1))
        return tuple(found)


def load_engine(path: Any, classes: Sequence[str], confidence: float = 0.25) -> EngineDetector:
    """The detector for an engine file. Raises EngineUnavailable when there is no file or no Ultralytics to run it."""
    if not str(path).strip():
        raise EngineUnavailable("detector 'engine' needs engine_path: the TensorRT engine file to run (the node builds none)")
    file = Path(str(path)).expanduser()
    if not file.is_file():
        raise EngineUnavailable(f"engine_path '{file}' is not a file")
    try:
        from ultralytics import YOLO
    except ImportError as error:
        raise EngineUnavailable(f"detector 'engine' runs through Ultralytics, which is not installed here ({error}): it runs on the Jetson") from error
    detector = EngineDetector(YOLO(str(file), task="detect"), classes, confidence)
    # One blank frame now: an engine built for another TensorRT or GPU loads as a file and fails at its first frame,
    # and a good one takes seconds over it. Both belong at the start, on the ground, not at the survey's first track.
    try:
        import numpy as np

        from .shrink import DETECTOR_SIZE

        detector.detect(np.zeros((DETECTOR_SIZE[1], DETECTOR_SIZE[0], 3), dtype=np.uint8), 0)
    except Exception as error:  # noqa: BLE001  (whatever the engine's runtime raises)
        raise EngineUnavailable(f"the engine at '{file}' did not run a frame ({type(error).__name__}: {error})") from error
    return detector
