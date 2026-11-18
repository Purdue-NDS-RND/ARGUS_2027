"""Sensor modes: which camera, which driver, what size and rate, which calibration. No ROS imports.

A mode is one way of running one sensor (the IMX519 at 4K, the same sensor at
720p60). Each has its own calibration file, measured in that mode, because a
sensor mode may crop or bin and a matrix scaled from another mode is then
wrong. The modes are in config/modes.yaml.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

import yaml

DRIVERS = ("csi", "v4l2", "ros_topic")
MODE_KEYS = {"camera", "driver", "width", "height", "fps", "calibration", "use", "sensor_id", "device", "pixel_format", "in_process_only", "exposure_time_s", "wb_mode"}


class ModeError(ValueError):
    """A modes file or a mode name that cannot be used. The message says what is wrong."""


@dataclass(frozen=True)
class SensorMode:
    name: str
    camera: str  # which camera on the airframe (a key of robot.yaml's cameras)
    frame_id: str  # that camera's optical frame
    driver: str  # csi (nvarguscamerasrc), v4l2 (USB), or ros_topic (the sim's bridged camera)
    width: int
    height: int
    fps: float
    calibration: Optional[str]  # file under config/, or None if this mode has not been calibrated
    use: str = ""
    sensor_id: int = 0  # csi
    device: str = ""  # v4l2
    pixel_format: str = ""  # v4l2 FOURCC, e.g. MJPG
    in_process_only: bool = False  # True: read through FrameSource in the process that uses it, never published as topics
    exposure_time_s: Optional[float] = None  # csi: a fixed exposure; None leaves the sensor on auto
    wb_mode: Optional[int] = None  # csi: nvarguscamerasrc's wbmode; None leaves it on auto


def load_modes(path) -> Dict[str, SensorMode]:
    path = Path(path)
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ModeError(f"{path}: {error}") from error
    if not isinstance(data, dict) or not isinstance(data.get("cameras"), dict) or not isinstance(data.get("modes"), dict):
        raise ModeError(f"{path}: expected 'cameras' and 'modes' mappings")
    cameras = data["cameras"]
    modes: Dict[str, SensorMode] = {}
    for name, entry in data["modes"].items():
        where = f"{path}: mode {name}"
        if not isinstance(entry, dict):
            raise ModeError(f"{where}: expected a mapping")
        unknown = sorted(str(key) for key in set(entry) - MODE_KEYS)
        if unknown:
            raise ModeError(f"{where}: unknown key(s) {', '.join(unknown)}")
        camera = entry.get("camera")
        if camera not in cameras or not isinstance(cameras[camera], dict) or not cameras[camera].get("frame_id"):
            raise ModeError(f"{where}: camera '{camera}' is not one of the cameras with a frame_id ({', '.join(cameras)})")
        if entry.get("driver") not in DRIVERS:
            raise ModeError(f"{where}: driver must be one of {', '.join(DRIVERS)}, got {entry.get('driver')!r}")
        for key in ("width", "height"):
            if not isinstance(entry.get(key), int) or entry[key] <= 0:
                raise ModeError(f"{where}: {key} must be a whole number of pixels above 0")
        fps = entry.get("fps")
        if isinstance(fps, bool) or not isinstance(fps, (int, float)) or fps <= 0:
            raise ModeError(f"{where}: fps must be above 0")
        if entry["driver"] == "v4l2" and not entry.get("device"):
            raise ModeError(f"{where}: a v4l2 mode needs a device")
        exposure, wb_mode = entry.get("exposure_time_s"), entry.get("wb_mode")
        if exposure is not None and (isinstance(exposure, bool) or not isinstance(exposure, (int, float)) or not 0.0 < exposure < 1.0):
            raise ModeError(f"{where}: exposure_time_s must be a time in seconds between 0 and 1")
        if wb_mode is not None and (isinstance(wb_mode, bool) or not isinstance(wb_mode, int) or not 0 <= wb_mode <= 9):
            raise ModeError(f"{where}: wb_mode must be a whole number from 0 to 9")
        modes[str(name)] = SensorMode(
            name=str(name),
            camera=str(camera),
            frame_id=str(cameras[camera]["frame_id"]),
            driver=str(entry["driver"]),
            width=entry["width"],
            height=entry["height"],
            fps=float(fps),
            calibration=None if entry.get("calibration") is None else str(entry["calibration"]),
            use=str(entry.get("use", "")),
            sensor_id=int(entry.get("sensor_id", 0)),
            device=str(entry.get("device", "")),
            pixel_format=str(entry.get("pixel_format", "")),
            in_process_only=entry.get("in_process_only", False) is True,
            exposure_time_s=None if exposure is None else float(exposure),
            wb_mode=wb_mode,
        )
    return modes


def get_mode(modes: Dict[str, SensorMode], name: str) -> SensorMode:
    if name not in modes:
        raise ModeError(f"no sensor mode '{name}' (modes: {', '.join(sorted(modes))})")
    return modes[name]
