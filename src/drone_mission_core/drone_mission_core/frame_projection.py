"""Projection v2 for a mission (FND-05): which camera and mount, and the ground error a tracking mission steers on. No ROS imports.

Behind the mission flag `use_frame_time_pose`. A mission that tracks a
target names the sensor mode its detector runs on (`camera_mode`) and the
airframe (`airframe`) in its config. `load_camera` turns those into the
mode's own calibration and the mount from the one robot description
(argus_camera's config), and refuses a calibration that is a stand-in or an
airframe nobody has measured, so the executor does not start on numbers
that are guesses.

`TargetOffset` is what replaces the 2026 latest-pose projection: the
target's point on the ground comes from the pose at the frame's capture
time, and the error to steer on is that point minus where the vehicle is
now. A target seen 150 ms ago while the vehicle was moving or tilted is
still where it was.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping, Optional, Sequence, Tuple

from .projection_v2 import CameraModel, CameraMount, GroundPoint, GroundProjector, ProjectionError

CAMERA_MODE_KEY = "camera_mode"
AIRFRAME_KEY = "airframe"


class FrameProjectionError(ValueError):
    """A camera or airframe that projection v2 cannot be flown with. The message says what to do."""


SIM_DRIVER = "ros_topic"  # argus_camera's driver for the sim's bridged cameras (modes.yaml)
SIM_AIRFRAME_PREFIX = "sim_"  # and how robot.yaml names the sim's vehicles


def load_camera(mode_name: str, airframe_name: str, config_dir: Optional[Path] = None) -> Tuple[CameraModel, CameraMount]:
    """The camera model of a sensor mode and the mount of its camera on an airframe, from argus_camera's config."""
    try:
        from argus_camera import calibration, modes, robot_description
        from argus_camera.bench import default_config_dir
    except ImportError as error:
        raise FrameProjectionError(f"projection v2 reads the camera's calibration and mount from the argus_camera package, which cannot be imported ({error})") from error
    config_dir = Path(config_dir) if config_dir is not None else default_config_dir()
    try:
        known = modes.load_modes(config_dir / "modes.yaml")
        if mode_name not in known:
            raise FrameProjectionError(f"{CAMERA_MODE_KEY} '{mode_name}' is not a sensor mode (modes: {', '.join(sorted(known))})")
        mode = known[mode_name]
        measured = calibration.for_mode(mode, config_dir)
        if not measured.measured:
            raise FrameProjectionError(f"sensor mode {mode_name} is not calibrated yet (its file is a stand-in): calibrate the camera in this mode first")
        airframe = robot_description.get_airframe(robot_description.load(config_dir / "robot.yaml"), airframe_name)
        # A sim camera goes with a sim vehicle and a real one with the aircraft, never across. The sim's cameras are
        # ideal pinholes, some at the aircraft's own image sizes; the sim vehicle's mount is not the aircraft's.
        sim_mode, sim_airframe = mode.driver == SIM_DRIVER, airframe_name.startswith(SIM_AIRFRAME_PREFIX)
        if sim_mode != sim_airframe:
            raise FrameProjectionError(
                f"sensor mode {mode_name} is {'a sim camera' if sim_mode else 'a real camera'} (driver {mode.driver}) and airframe {airframe_name} is "
                f"{'a sim vehicle' if sim_airframe else 'not a sim vehicle'} (their names start with '{SIM_AIRFRAME_PREFIX}'): name a sensor mode and an airframe of the same kind"
            )
        if mode.camera not in airframe.cameras:
            raise FrameProjectionError(f"airframe {airframe_name} has no camera '{mode.camera}', which sensor mode {mode_name} is on")
        return CameraModel.from_calibration(measured, name=mode_name), CameraMount.from_description(airframe.cameras[mode.camera])
    except FrameProjectionError:
        raise
    except (ValueError, ProjectionError) as error:  # ModeError, CalibrationError, RobotDescriptionError
        raise FrameProjectionError(str(error)) from error


def camera_from_config(config: Mapping[str, Any], mission_name: str, config_dir: Optional[Path] = None) -> Tuple[CameraModel, CameraMount]:
    """`load_camera` for a mission's config block; both keys are needed once the flag is on."""
    missing = [key for key in (CAMERA_MODE_KEY, AIRFRAME_KEY) if not str(config.get(key, "") or "").strip()]
    if missing:
        raise FrameProjectionError(
            f"mission '{mission_name}': use_frame_time_pose is on, so its config must name {' and '.join(missing)} "
            "(the sensor mode the detector's images come from, and the airframe in argus_camera's robot.yaml)"
        )
    return load_camera(str(config[CAMERA_MODE_KEY]).strip(), str(config[AIRFRAME_KEY]).strip(), config_dir)


class TargetOffset:
    """The ground error to a detected target: its point at the frame's capture time, minus where the vehicle is now."""

    def __init__(self, projector: GroundProjector) -> None:
        self._projector = projector
        self.last_refusal: Optional[str] = None
        self.last_point: Optional[GroundPoint] = None

    def offset(
        self,
        x_norm: float,
        y_norm: float,
        image_size: Sequence[int],
        stamp_ns: int,
        vehicle_position: Sequence[float],
    ) -> Optional[Tuple[float, float]]:
        """(east, north) in metres from the vehicle to the target, or None (`last_refusal` says why).

        `x_norm` and `y_norm` are the 2026 detection point, -1 to 1 across the
        image; `image_size` is the size of the image the detector saw.
        """
        camera = self._projector.camera
        self.last_point = None
        if (int(image_size[0]), int(image_size[1])) != (camera.width, camera.height):
            self.last_refusal = (
                f"the detector's image is {int(image_size[0])} x {int(image_size[1])} but sensor mode {camera.name} is calibrated at "
                f"{camera.width} x {camera.height}: a calibration is not scaled to another size"
            )
            return None
        pixel = (0.5 * (float(x_norm) + 1.0) * camera.width, 0.5 * (float(y_norm) + 1.0) * camera.height)
        point = self._projector.project_pixel_to_ground(pixel, int(stamp_ns))
        if point is None:
            self.last_refusal = self._projector.last_refusal
            return None
        self.last_refusal = None
        self.last_point = point
        return (point.east_m - float(vehicle_position[0]), point.north_m - float(vehicle_position[1]))
