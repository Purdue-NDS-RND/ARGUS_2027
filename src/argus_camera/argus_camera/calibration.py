"""Per-mode camera calibration and the camera_info that goes with raw frames. No ROS imports.

Each sensor mode has its own file: K and D as measured in that mode. A file
for another resolution is refused, never scaled. The driver publishes frames
as the sensor gives them, so camera_info carries that K and D, an identity R,
and P equal to K: nothing is rectified on the vehicle, and projection
undistorts the few points it needs (FND-05).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import yaml

from .modes import SensorMode


class CalibrationError(ValueError):
    """A calibration that cannot be used for the mode. The message says why."""


@dataclass(frozen=True)
class Calibration:
    width: int
    height: int
    k: Tuple[float, ...]  # 3x3, row-major
    d: Tuple[float, ...]
    distortion_model: str
    measured: bool  # False: a stand-in until this mode is calibrated on the bench
    basis: str  # where the numbers come from


def load_calibration(path) -> Calibration:
    path = Path(path)
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise CalibrationError(f"{path}: {error}") from error
    if not isinstance(data, dict):
        raise CalibrationError(f"{path}: expected a mapping")
    missing = [key for key in ("width", "height", "k", "d", "measured") if key not in data]
    if missing:
        raise CalibrationError(f"{path}: missing {', '.join(missing)}")
    k, d = data["k"], data["d"]
    if not isinstance(k, list) or len(k) != 9:
        raise CalibrationError(f"{path}: k must be the nine numbers of the camera matrix, row by row")
    if not isinstance(d, list) or not isinstance(data["measured"], bool):
        raise CalibrationError(f"{path}: d must be a list and measured true or false")
    return Calibration(
        width=int(data["width"]),
        height=int(data["height"]),
        k=tuple(float(value) for value in k),
        d=tuple(float(value) for value in d),
        distortion_model=str(data.get("distortion_model", "plumb_bob")),
        measured=data["measured"],
        basis=str(data.get("basis", "")),
    )


def for_mode(mode: SensorMode, config_dir) -> Calibration:
    """The calibration of one sensor mode. Its image size must be the mode's."""
    if mode.calibration is None:
        raise CalibrationError(f"sensor mode {mode.name} has no calibration yet: calibrate the camera in this mode and add its file to modes.yaml")
    calibration = load_calibration(Path(config_dir) / mode.calibration)
    if (calibration.width, calibration.height) != (mode.width, mode.height):
        raise CalibrationError(
            f"{mode.calibration} is for {calibration.width} x {calibration.height} but mode {mode.name} is "
            f"{mode.width} x {mode.height}: a calibration is measured in its own mode, not scaled from another"
        )
    return calibration


def camera_info_fields(calibration: Calibration) -> dict:
    """The fields of sensor_msgs/CameraInfo for raw frames in this calibration."""
    k = list(calibration.k)
    return {
        "width": calibration.width,
        "height": calibration.height,
        "distortion_model": calibration.distortion_model,
        "d": list(calibration.d),
        "k": k,
        "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "p": [k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0],
    }
