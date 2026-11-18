"""The one description of where the cameras and the payload are on the airframe. No ROS imports.

config/robot.yaml is the only place 2027 code reads a camera mount or the
payload offset from (FND-05, CEN-01). The 2026 copies (mission_logger's
mount_* parameters, the sequences' camera_yaw_offset_deg) stay for the 2026
path and are not read by anything new.

Frames. Body: x forward, y left, z up, origin at the flight controller.
Optical: x right in the image, y down in the image, z out of the lens.
Every camera here looks straight down; `image_yaw_deg` turns the image about
the lens axis: 0 puts the top of the image toward the nose, 180 toward the
tail.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Tuple

import numpy as np
import yaml

CAMERA_KEYS = {"optical_frame", "position_m", "image_yaw_deg", "source"}
AIRFRAME_KEYS = {"description", "verified", "cameras", "payload_offset_m", "payload_source"}


class RobotDescriptionError(ValueError):
    """A robot description that cannot be used. The message names the key."""


@dataclass(frozen=True)
class CameraMount:
    name: str
    optical_frame: str
    position_m: Tuple[float, float, float]  # the lens in the body frame: forward, left, up
    image_yaw_deg: float = 0.0
    source: str = ""

    def rotation_body_from_optical(self) -> np.ndarray:
        """The 3x3 matrix that turns a vector in the optical frame into the body frame."""
        # Looking down with the image top toward the nose: right -> -y, image down -> -x, lens -> -z.
        down = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
        yaw = math.radians(self.image_yaw_deg)
        cos, sin = math.cos(yaw), math.sin(yaw)
        about_lens = np.array([[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]])  # in the optical frame
        return down @ about_lens


@dataclass(frozen=True)
class Airframe:
    name: str
    verified: bool  # True once someone has measured it on the vehicle
    cameras: Dict[str, CameraMount] = field(default_factory=dict)
    payload_offset_m: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # the release point in the body frame
    description: str = ""


def get_airframe(airframes: Dict[str, Airframe], name: str, allow_unverified: bool = False) -> Airframe:
    """One airframe by name. An airframe nobody has measured is only given to a caller that asks for it knowingly."""
    if name not in airframes:
        raise RobotDescriptionError(f"no airframe '{name}' (airframes: {', '.join(sorted(airframes))})")
    airframe = airframes[name]
    if not airframe.verified and not allow_unverified:
        raise RobotDescriptionError(
            f"airframe '{name}' is not verified: its mounts have not been measured on the vehicle. "
            "Measure them and set verified: true, or pass allow_unverified=True and say so in the log"
        )
    return airframe


def _vector(value, where: str) -> Tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3 or any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value):
        raise RobotDescriptionError(f"{where} must be three numbers: forward, left, up, in metres")
    return (float(value[0]), float(value[1]), float(value[2]))


def load(path) -> Dict[str, Airframe]:
    path = Path(path)
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise RobotDescriptionError(f"{path}: {error}") from error
    if not isinstance(data, dict) or not isinstance(data.get("airframes"), dict):
        raise RobotDescriptionError(f"{path}: expected an 'airframes' mapping")
    airframes: Dict[str, Airframe] = {}
    for name, entry in data["airframes"].items():
        where = f"{path}: airframe {name}"
        if not isinstance(entry, dict):
            raise RobotDescriptionError(f"{where}: expected a mapping")
        unknown = sorted(str(key) for key in set(entry) - AIRFRAME_KEYS)
        if unknown:
            raise RobotDescriptionError(f"{where}: unknown key(s) {', '.join(unknown)}")
        for key in ("verified", "cameras", "payload_offset_m"):
            if key not in entry:
                raise RobotDescriptionError(f"{where}: missing {key}")
        if not isinstance(entry["verified"], bool) or not isinstance(entry["cameras"], dict):
            raise RobotDescriptionError(f"{where}: verified must be true or false and cameras a mapping")
        cameras: Dict[str, CameraMount] = {}
        for camera_name, camera in entry["cameras"].items():
            camera_where = f"{where}: camera {camera_name}"
            if not isinstance(camera, dict):
                raise RobotDescriptionError(f"{camera_where}: expected a mapping")
            extra = sorted(str(key) for key in set(camera) - CAMERA_KEYS)
            if extra:
                raise RobotDescriptionError(f"{camera_where}: unknown key(s) {', '.join(extra)}")
            for key in ("optical_frame", "position_m"):
                if key not in camera:
                    raise RobotDescriptionError(f"{camera_where}: missing {key}")
            cameras[str(camera_name)] = CameraMount(
                name=str(camera_name),
                optical_frame=str(camera["optical_frame"]),
                position_m=_vector(camera["position_m"], f"{camera_where}: position_m"),
                image_yaw_deg=float(camera.get("image_yaw_deg", 0.0)),
                source=str(camera.get("source", "")),
            )
        airframes[str(name)] = Airframe(
            name=str(name),
            verified=entry["verified"],
            cameras=cameras,
            payload_offset_m=_vector(entry["payload_offset_m"], f"{where}: payload_offset_m"),
            description=str(entry.get("description", "")),
        )
    return airframes
