"""Projection v2 for the mission logger (FND-05): a detection's pixel to latitude and longitude. No ROS imports.

Behind the logger's node flag `features.projection_v2`. With it on, this
replaces the logger's 2026 raycast: the pose is the one at the frame's
stamp from a pose history (pose_buffer.py), the camera is the sensor mode's
own calibration and the mount comes from the one robot description
(drone_mission_core.frame_projection.load_camera).

The pixels it is given must be those of the mode's raw frames. A source that
has already undistorted its frames publishes another camera matrix (the 2026
image_grabber does), and undoing the lens twice would put every point in the
wrong place, so `camera_info_problem` compares the stream's camera_info with
the mode's calibration and the logger projects nothing when they differ.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

from drone_mission_core.frame_projection import load_camera
from drone_mission_core.pose_buffer import PoseBuffer
from drone_mission_core.projection_v2 import GroundProjector

R_EARTH_M = 6378137.0  # as mission_logger.py


def enu_to_latlon(east_m: float, north_m: float, home_latitude: float, home_longitude: float) -> Tuple[float, float]:
    """A point east and north of home, in degrees: the logger's own flat-earth step."""
    latitude = home_latitude + (north_m / R_EARTH_M) * (180.0 / math.pi)
    longitude = home_longitude + (east_m / (R_EARTH_M * math.cos(math.radians(home_latitude)))) * (180.0 / math.pi)
    return latitude, longitude


class LoggerProjection:
    """The logger's pose history and projector for one camera mode on one airframe."""

    def __init__(self, camera_mode: str, airframe: str, config_dir=None) -> None:
        if not camera_mode or not airframe:
            raise ValueError(
                "features.projection_v2 is on: set the camera_mode and airframe parameters "
                "(the sensor mode the frames come from, and the airframe in argus_camera's robot.yaml)"
            )
        self.camera, self.mount = load_camera(camera_mode, airframe, config_dir)  # raises on a stand-in calibration or an unmeasured airframe
        self._poses = PoseBuffer()
        self._projector = GroundProjector(self.camera, self.mount, self._poses.pose_at)
        self.last_refusal: Optional[str] = None

    def camera_info_problem(self, width: int, height: int, k: Sequence[float], d: Sequence[float]) -> Optional[str]:
        """Why a stream with this camera_info is not this mode's raw frames, or None if it is."""
        camera = self.camera
        if (int(width), int(height)) != (camera.width, camera.height):
            return f"the frames are {int(width)} x {int(height)} but sensor mode {camera.name} is calibrated at {camera.width} x {camera.height}"
        k = [float(value) for value in k]
        theirs = (k[0], k[4], k[2], k[5], k[1]) if len(k) == 9 else ()
        ours = (camera.fx, camera.fy, camera.cx, camera.cy, camera.skew)
        distortion = tuple(float(value) for value in d)[:5]
        distortion += (0.0,) * (5 - len(distortion))
        same_k = len(theirs) == 5 and all(abs(a - b) <= 1.0e-3 for a, b in zip(theirs, ours))
        same_d = all(abs(a - b) <= 1.0e-6 for a, b in zip(distortion, camera.d))
        if not (same_k and same_d):
            return (
                f"the stream's camera_info is not sensor mode {camera.name}'s own calibration "
                f"(fx {theirs[0] if theirs else float('nan'):.2f} against {camera.fx:.2f}): these are not that mode's raw frames "
                "(already undistorted, or another mode), so projection v2 would undo the lens wrongly"
            )
        return None

    def add_pose(self, stamp_ns: int, position: Sequence[float], orientation: Sequence[float]) -> None:
        self._poses.add_pose(int(stamp_ns), tuple(float(value) for value in position), tuple(float(value) for value in orientation))

    def latlon(
        self,
        u: float,
        v: float,
        stamp_ns: int,
        image_size: Sequence[int],
        home_latitude: float,
        home_longitude: float,
        ground_altitude_m: float = 0.0,
    ) -> Optional[Tuple[float, float]]:
        """(latitude, longitude) of the ground point under a pixel of the frame stamped `stamp_ns`, or None (`last_refusal` says why)."""
        if (int(image_size[0]), int(image_size[1])) != (self.camera.width, self.camera.height):
            self.last_refusal = (
                f"the frames are {int(image_size[0])} x {int(image_size[1])} but sensor mode {self.camera.name} is calibrated at "
                f"{self.camera.width} x {self.camera.height}"
            )
            return None
        point = self._projector.project_pixel_to_ground((float(u), float(v)), int(stamp_ns), plane_height_m=float(ground_altitude_m))
        if point is None:
            self.last_refusal = self._projector.last_refusal
            return None
        self.last_refusal = None
        return enu_to_latlon(point.east_m, point.north_m, home_latitude, home_longitude)
