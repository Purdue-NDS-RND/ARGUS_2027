"""Projection v2 (FND-05): a pixel to a point on the ground, with the pose the vehicle had when the frame was taken. No ROS imports.

Beside tracking_projection.py, which is the 2026 projection and is not
changed. Three objects:

  CameraModel   K and D of one sensor mode, calibrated in that mode (never
                scaled from another mode's calibration).
  CameraMount   where the lens is on the body and how the image is turned,
                from the one robot description (argus_camera, config/robot.yaml).
  GroundProjector.project_pixel_to_ground(pixel, stamp)

Steps: undistort the pixel; make the ray in the optical frame; turn it
through the mount into the body frame and through the body's attitude at
the capture time into the world; start it at the lens (the lever arm); meet
the ground plane. The answer is the point (east, north, up, in MAVROS's
local frame) and how it moves with an error in the pixel, the attitude and
the height, which is what a filter needs to weigh the measurement.

Frames. Pixel: u right, v down, in the full-resolution image of the mode.
Optical: x right in the image, y down, z out of the lens. Body: x forward,
y left, z up. World: east, north, up, the frame of
/mavros/local_position/pose, whose orientation turns body into world.

There is no fallback to the latest pose: with no pose for the frame's stamp
there is no projection, and `last_refusal` says why.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, replace
from typing import Any, Callable, Deque, Optional, Sequence, Tuple

import numpy as np

Vector = Tuple[float, float, float]

MIN_DOWNWARD = 0.05  # a ray flatter than about 3 degrees below the horizon does not meet the ground usefully
UNDISTORT_STEPS = 25
UNDISTORT_TOLERANCE = 1.0e-12

# Where the ground is (GroundHeight). New values, not 2026 tuning.
RANGE_MAX_AGE_S = 0.5  # a rangefinder reading older than this, against the frame's stamp, is not used
RANGE_MAX_TILT_DEG = 25.0  # tilted further, the beam is not measuring the height under the vehicle
RANGE_EDGE_MARGIN_M = 0.05  # a reading this close to the sensor's minimum or maximum is the sensor saying "out of range"
RANGE_HISTORY = 64  # readings kept: over a second's worth at the 20 Hz the stream keeper asks for
RANGE_VOTES = 5  # the ground is the median of what this many readings nearest the frame say
RANGE_MIN_VOTES = 3  # with fewer usable readings the rangefinder is not believed
RANGE_MAX_SPREAD_M = 0.5  # readings that disagree about the ground by more than this are not believed
RANGE_MAX_GROUND_OFFSET_M = 5.0  # nor is a ground further than this above or below home (something in the beam, or no return)


class ProjectionError(ValueError):
    """A camera model, mount or input that cannot be projected with. The message says why."""


@dataclass(frozen=True)
class CameraModel:
    """The pinhole camera of one sensor mode with plumb-bob distortion (k1, k2, p1, p2, k3)."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    d: Tuple[float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0)
    skew: float = 0.0
    name: str = ""
    measured: bool = True  # False: a stand-in until this mode is calibrated on the bench

    def __post_init__(self) -> None:
        values = (self.fx, self.fy, self.cx, self.cy, self.skew, *self.d)
        if len(self.d) != 5 or not all(math.isfinite(float(value)) for value in values):
            raise ProjectionError(f"camera model {self.name or '?'}: K and the five distortion numbers must be finite")
        if self.width <= 0 or self.height <= 0 or self.fx <= 0.0 or self.fy <= 0.0:
            raise ProjectionError(f"camera model {self.name or '?'}: image size and focal lengths must be above zero")

    @classmethod
    def from_calibration(cls, calibration: Any, name: str = "") -> "CameraModel":
        """From a per-mode calibration (argus_camera.calibration.Calibration, or anything with its fields)."""
        model = str(getattr(calibration, "distortion_model", "plumb_bob"))
        d = tuple(float(value) for value in calibration.d)
        if model != "plumb_bob" or len(d) not in (0, 4, 5):
            raise ProjectionError(f"camera model {name or '?'}: only plumb_bob with 4 or 5 numbers is supported, got {model} with {len(d)}")
        k = [float(value) for value in calibration.k]
        if len(k) != 9 or k[3] != 0.0 or k[6] != 0.0 or k[7] != 0.0 or k[8] != 1.0:
            raise ProjectionError(f"camera model {name or '?'}: k must be a camera matrix, row by row")
        return cls(
            width=int(calibration.width),
            height=int(calibration.height),
            fx=k[0],
            fy=k[4],
            cx=k[2],
            cy=k[5],
            skew=k[1],
            d=(d + (0.0,) * 5)[:5],
            name=name,
            measured=bool(getattr(calibration, "measured", True)),
        )

    def _distort(self, x: float, y: float) -> Tuple[np.ndarray, np.ndarray]:
        """The distorted normalised point for an ideal one, and the 2x2 derivative of the first by the second."""
        k1, k2, p1, p2, k3 = self.d
        r2 = x * x + y * y
        radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
        d_radial = k1 + r2 * (2.0 * k2 + r2 * 3.0 * k3)  # by r2
        xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        jacobian = np.array(
            [
                [radial + 2.0 * x * x * d_radial + 2.0 * p1 * y + 6.0 * p2 * x, 2.0 * x * y * d_radial + 2.0 * p1 * x + 2.0 * p2 * y],
                [2.0 * x * y * d_radial + 2.0 * p1 * x + 2.0 * p2 * y, radial + 2.0 * y * y * d_radial + 6.0 * p1 * y + 2.0 * p2 * x],
            ]
        )
        return np.array([xd, yd]), jacobian

    def pixel_of(self, ray_optical: Sequence[float]) -> Tuple[float, float]:
        """Where a ray in the optical frame lands in the image (the forward model: lens, then K)."""
        x, y, z = (float(value) for value in ray_optical)
        if z <= 0.0:
            raise ProjectionError("a ray behind the lens has no pixel")
        (xd, yd), _jacobian = self._distort(x / z, y / z)
        return (self.fx * xd + self.skew * yd + self.cx, self.fy * yd + self.cy)

    def undistort(self, pixel: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
        """The ideal normalised point (x, y) of a pixel, so the ray is (x, y, 1), and its 2x2 derivative by the pixel."""
        u, v = float(pixel[0]), float(pixel[1])
        if not (math.isfinite(u) and math.isfinite(v)):
            raise ProjectionError("the pixel must be finite")
        yd = (v - self.cy) / self.fy
        xd = (u - self.cx - self.skew * yd) / self.fx
        target = np.array([xd, yd])
        point = target.copy()
        for _step in range(UNDISTORT_STEPS):  # Newton's method on the lens model
            value, jacobian = self._distort(float(point[0]), float(point[1]))
            if abs(np.linalg.det(jacobian)) < 1.0e-9:
                raise ProjectionError(f"pixel ({u:.1f}, {v:.1f}) is outside what the lens model of {self.name or 'this camera'} can undo")
            step = np.linalg.solve(jacobian, target - value)
            point = point + step
            if float(np.max(np.abs(step))) < UNDISTORT_TOLERANCE:
                break
        else:
            raise ProjectionError(f"pixel ({u:.1f}, {v:.1f}) could not be undistorted with the lens model of {self.name or 'this camera'}")
        _value, jacobian = self._distort(float(point[0]), float(point[1]))
        by_distorted = np.linalg.inv(jacobian)
        distorted_by_pixel = np.array([[1.0 / self.fx, -self.skew / (self.fx * self.fy)], [0.0, 1.0 / self.fy]])
        return point, by_distorted @ distorted_by_pixel


@dataclass(frozen=True)
class CameraMount:
    """Where the lens is on the body (forward, left, up, metres) and the matrix that turns an optical-frame vector into the body frame."""

    position_m: Vector
    rotation_body_from_optical: Tuple[Vector, Vector, Vector]
    optical_frame: str = ""

    def __post_init__(self) -> None:
        rotation = np.array(self.rotation_body_from_optical, dtype=float)
        if rotation.shape != (3, 3) or not np.allclose(rotation @ rotation.T, np.eye(3), atol=1.0e-6) or np.linalg.det(rotation) < 0.0:
            raise ProjectionError("the mount's rotation must be a proper 3x3 rotation matrix")
        if len(self.position_m) != 3 or not all(math.isfinite(float(value)) for value in self.position_m):
            raise ProjectionError("the mount's position must be three finite numbers: forward, left, up")

    @classmethod
    def from_description(cls, mount: Any) -> "CameraMount":
        """From the robot description's mount (argus_camera.robot_description.CameraMount)."""
        rotation = np.array(mount.rotation_body_from_optical(), dtype=float)
        return cls(
            position_m=tuple(float(value) for value in mount.position_m),
            rotation_body_from_optical=tuple(tuple(float(value) for value in row) for row in rotation),
            optical_frame=str(getattr(mount, "optical_frame", "")),
        )


@dataclass(frozen=True)
class GroundPoint:
    """Where a pixel's ray meets the ground, and how that point moves with each error."""

    east_m: float
    north_m: float
    up_m: float  # the ground plane's height, in the local frame
    d_pixel: np.ndarray  # 2x2: metres east and north per pixel right and down
    d_attitude: np.ndarray  # 2x3: metres east and north per radian of attitude error about east, north and up
    d_height: np.ndarray  # 2: metres east and north per metre of error in the lens's height above the plane
    camera_enu: Vector  # where the lens was
    range_m: float  # from the lens to the point
    stamp_ns: int = 0
    pose_extrapolated: bool = False
    height_source: str = ""  # "rangefinder" or "home"


def rotation_from_quaternion(orientation: Sequence[float]) -> np.ndarray:
    """Body to world, from a quaternion (x, y, z, w)."""
    x, y, z, w = (float(value) for value in orientation)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1.0e-9:
        raise ProjectionError("the orientation is not a quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def _cross_matrix(vector: np.ndarray) -> np.ndarray:
    x, y, z = (float(value) for value in vector)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def project(
    camera: CameraModel,
    mount: CameraMount,
    pixel: Sequence[float],
    position: Sequence[float],
    orientation: Sequence[float],
    ground_up_m: float = 0.0,
) -> Optional[GroundPoint]:
    """The ground point for a pixel seen from a known pose. None when the ray does not come down to the plane.

    `position` (east, north, up) and `orientation` (x, y, z, w) are the body's
    in the local frame, at the frame's capture time. `ground_up_m` is the
    plane's height in the same frame (0 is home's height).
    """
    normalised, normalised_by_pixel = camera.undistort(pixel)
    ray_optical = np.array([normalised[0], normalised[1], 1.0])
    body_from_optical = np.array(mount.rotation_body_from_optical, dtype=float)
    world_from_body = rotation_from_quaternion(orientation)
    world_from_optical = world_from_body @ body_from_optical
    ray = world_from_optical @ ray_optical
    arm = world_from_body @ np.array(mount.position_m, dtype=float)  # the lever arm, in the world
    lens = np.array([float(value) for value in position]) + arm
    height = float(lens[2]) - float(ground_up_m)
    if not np.all(np.isfinite(lens)) or not math.isfinite(height):
        raise ProjectionError("the position and the ground height must be finite")
    if height <= 0.0 or ray[2] / float(np.linalg.norm(ray)) > -MIN_DOWNWARD:
        return None  # the lens is not above the plane, or the ray does not come down to it
    along = height / -ray[2]
    point = lens + along * ray

    up = np.array([0.0, 0.0, 1.0])
    slide = np.eye(3) - np.outer(ray, up) / ray[2]  # how the point moves when the lens moves; a move along the ray changes nothing
    by_ray = along * slide
    d_pixel = (by_ray @ world_from_optical[:, :2] @ normalised_by_pixel)[:2, :]
    # A small rotation w (east, north, up; radians) of the body turns every body vector b into b + w x b.
    d_attitude = (-(slide @ _cross_matrix(arm)) - by_ray @ _cross_matrix(ray))[:2, :]
    d_height = (-ray / ray[2])[:2]
    return GroundPoint(
        east_m=float(point[0]),
        north_m=float(point[1]),
        up_m=float(ground_up_m),
        d_pixel=d_pixel,
        d_attitude=d_attitude,
        d_height=d_height,
        camera_enu=(float(lens[0]), float(lens[1]), float(lens[2])),
        range_m=float(along * np.linalg.norm(ray)),
    )


def pixel_of_ground_point(
    camera: CameraModel,
    mount: CameraMount,
    point_enu: Sequence[float],
    position: Sequence[float],
    orientation: Sequence[float],
) -> Tuple[float, float]:
    """The pixel a world point is seen at from a pose: the projection the other way, for checks and for a simulated detector."""
    world_from_body = rotation_from_quaternion(orientation)
    body_from_optical = np.array(mount.rotation_body_from_optical, dtype=float)
    lens = np.array([float(value) for value in position]) + world_from_body @ np.array(mount.position_m, dtype=float)
    ray_optical = (world_from_body @ body_from_optical).T @ (np.array([float(value) for value in point_enu]) - lens)
    return camera.pixel_of(ray_optical)


class GroundHeight:
    """Where the ground is, as a height in the local frame: under the rangefinder when it is healthy, at home's height otherwise.

    The rangefinder points down along the body's z axis and is taken to be at
    the body's origin. Its reading is the distance along the beam, so the
    height above the ground is the reading times the cosine of the tilt, and
    the ground's own height is the vehicle's height at the moment of the
    reading minus that. The ground does not move, so a reading taken a little
    before or after the frame still says where the ground is, however fast
    the vehicle is climbing or descending.

    Healthy means: at least three readings within `max_age_s` of the frame,
    none at the sensor's limits, each with a pose known for its own moment
    and the vehicle tilted no more than `max_tilt_deg` then, that agree about
    the ground within half a metre and put it within five metres of home's
    height. The ground is the median of what the five nearest say, so one odd
    reading does not move it, and a beam that is blocked (a ground far above
    home) or returns nothing is not believed at all.
    """

    def __init__(self, pose_at: Callable[[int], Any], max_age_s: float = RANGE_MAX_AGE_S, max_tilt_deg: float = RANGE_MAX_TILT_DEG) -> None:
        self._pose_at = pose_at
        self._max_age_ns = int(max_age_s * 1.0e9)
        self._min_cos = math.cos(math.radians(max_tilt_deg))
        self._readings: Deque[Tuple[int, float]] = deque(maxlen=RANGE_HISTORY)  # (stamp, range) of usable readings

    def add_range(self, stamp_ns: int, range_m: float, min_m: float, max_m: float) -> None:
        """One rangefinder reading. One at the sensor's limits is the sensor saying "out of range" and is not kept."""
        range_m = float(range_m)
        if math.isfinite(range_m) and float(min_m) + RANGE_EDGE_MARGIN_M < range_m < float(max_m) - RANGE_EDGE_MARGIN_M:
            self._readings.append((int(stamp_ns), range_m))

    def ground_up(self, stamp_ns: int) -> Tuple[float, str]:
        """(the ground's height in the local frame, "rangefinder" or "home") for a frame taken at `stamp_ns`."""
        votes = []
        for read_ns, range_m in sorted(self._readings, key=lambda reading: abs(reading[0] - int(stamp_ns))):
            if abs(read_ns - int(stamp_ns)) > self._max_age_ns or len(votes) == RANGE_VOTES:
                break  # sorted by distance in time: the rest are further still
            pose = self._pose_at(read_ns)
            if pose is None:
                continue
            cos_tilt = float(rotation_from_quaternion(pose.orientation)[2, 2])
            if cos_tilt >= self._min_cos:
                votes.append(float(pose.position[2]) - range_m * cos_tilt)
        if len(votes) < RANGE_MIN_VOTES or max(votes) - min(votes) > RANGE_MAX_SPREAD_M:
            return 0.0, "home"
        ground = sorted(votes)[len(votes) // 2]
        if abs(ground) > RANGE_MAX_GROUND_OFFSET_M:
            return 0.0, "home"
        return ground, "rangefinder"


class GroundProjector:
    """project_pixel_to_ground(pixel, stamp): the pose at the frame's stamp from a pose history, the ground from a GroundHeight."""

    def __init__(
        self,
        camera: CameraModel,
        mount: CameraMount,
        pose_at: Callable[[int], Any],
        ground_height: Optional[GroundHeight] = None,
    ) -> None:
        self.camera = camera
        self.mount = mount
        self._pose_at = pose_at  # PoseBuffer.pose_at: None when it has no pose for that time
        self._ground_height = ground_height
        self.last_refusal: Optional[str] = None

    def project_pixel_to_ground(self, pixel: Sequence[float], stamp_ns: int, plane_height_m: float = 0.0) -> Optional[GroundPoint]:
        """The ground point for a pixel of the frame captured at `stamp_ns`, or None (`last_refusal` says why).

        `plane_height_m` is the height above the ground of the plane the
        object's top is in (0 for a target painted on the ground).
        """
        self.last_refusal = None
        pose = self._pose_at(int(stamp_ns))
        if pose is None:
            self.last_refusal = "no pose for the frame's capture time"
            return None
        if self._ground_height is None:
            ground_up, source = 0.0, "home"
        else:
            ground_up, source = self._ground_height.ground_up(int(stamp_ns))
        try:
            point = project(self.camera, self.mount, pixel, pose.position, pose.orientation, ground_up + float(plane_height_m))
        except ProjectionError as error:
            self.last_refusal = str(error)
            return None
        if point is None:
            self.last_refusal = "the ray does not come down to the ground"
            return None
        return replace(point, stamp_ns=int(stamp_ns), pose_extrapolated=bool(getattr(pose, "extrapolated", False)), height_source=source)
