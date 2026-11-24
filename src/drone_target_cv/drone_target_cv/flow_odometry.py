"""Optical-flow odometry from the downward flow camera (REC-5). No ROS.

Behind the node flag `features.flow_odometry` (flow_odometry_node.py). One
sample is made from two frames, up to FRAMES_APART apart: the newest and the
oldest held frame whose picture can still be followed to it. A long time
between the two is what makes the velocity fine (a tenth of a pixel is
0.13 m/s at 15 m over 50 ms, and a quarter of that over 200 ms); a fast, low
flight moves the picture too far in that time, and the next older frame is
tried. The steps:

  1. each frame is a 320 x 200 greyscale picture (a larger one is shrunk);
  2. the shift of the picture between the two is found by phase correlation
     (cv2.phaseCorrelate under a Hann window): one shift for the whole
     picture, or, when the hook or the bucket is in view and would be
     followed instead of the ground, the shift the eight
     patches round the picture's border agree on;
  3. the part of the shift that is the camera turning, not moving, is taken
     out with the roll and pitch the vehicle turned through between the two
     frames (from the pose history at the frames' own times);
  4. what is left is scaled by the height above the ground to metres a
     second, in the vehicle's frame (forward, right) and, with the heading,
     east and north.

Quality is 0 to 255. It is 0, and the sample says why, when the picture has
no texture to follow (a bare surface), when the correlation's peak is weak,
when the border patches disagree, when the height is not known or is outside
the band the camera can resolve, or when the frames are too far apart in
time. A sample with quality 0 carries no velocity.

Conventions. The camera looks straight down. How its picture sits on the
vehicle is the mount's rotation (`body_from_optical`, from the airframe's
entry in robot.yaml); with the usual mount the picture's top is towards the nose:
picture x is the vehicle's right, picture y its back. Roll and
pitch are the aerospace ones (roll: right side down; pitch: nose up), in
radians. Heading is the ENU yaw (from east, anticlockwise). A positive
`forward_mps` is towards the nose.

numpy and cv2 only.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import cv2
import numpy as np

FLAG = "flow_odometry"
WIDTH, HEIGHT = 320, 200  # the picture the flow is measured on (the spec's)
FRAMES_APART = 4  # at 120 Hz: 33 ms between the two pictures
MIN_TEXTURE = 4.0  # the picture's standard deviation in grey levels; under this there is nothing to follow
MIN_RESPONSE = 0.08  # cv2.phaseCorrelate's peak, 0 to 1; under this the shift is not believed
FULL_RESPONSE = 0.5  # and from here up the quality is full
MAX_SHIFT_SHARE = 0.25  # of the picture's size: a larger shift between the two frames is beyond what correlation finds
MIN_HEIGHT_M, MAX_HEIGHT_M = 0.3, 40.0
MAX_DT_S = 0.5
MAX_PATCH_SPREAD_PX = 1.5  # a patch's shift is within this of the patches' middle, or it is not counted
MIN_PATCHES = 5  # of the eight border patches must have texture and agree: a majority, so that what agrees is the ground


@dataclass(frozen=True)
class Attitude:
    roll_rad: float
    pitch_rad: float
    yaw_rad: float  # ENU heading, from east, anticlockwise


@dataclass(frozen=True)
class FlowSample:
    stamp_ns: int  # the newer frame's
    dt_s: float
    quality: int  # 0 to 255
    why: str  # empty when the quality is above 0
    shift_px: Tuple[float, float] = (0.0, 0.0)  # of the picture's content, x right and y down, over dt
    rotation_px: Tuple[float, float] = (0.0, 0.0)  # the part of it that was the camera turning
    forward_mps: float = 0.0
    right_mps: float = 0.0
    east_mps: float = 0.0
    north_mps: float = 0.0
    height_m: float = 0.0
    patches: int = 0  # how many corner patches agreed; 0 for the whole picture


def prepare(image: np.ndarray) -> np.ndarray:
    """A camera frame as the 320 x 200 float32 greyscale picture the flow is measured on."""
    if image.ndim == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if image.shape[1] != WIDTH or image.shape[0] != HEIGHT:
        image = cv2.resize(image, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
    return image.astype(np.float32)


_WINDOWS: dict = {}


def _window(shape: Tuple[int, int]) -> np.ndarray:
    if shape not in _WINDOWS:
        _WINDOWS[shape] = cv2.createHanningWindow((shape[1], shape[0]), cv2.CV_32F)
    return _WINDOWS[shape]


def shift_between(older: np.ndarray, newer: np.ndarray) -> Tuple[float, float, float]:
    """(dx, dy, response): how far the picture's content moved from `older` to `newer`, in pixels, and how sure the correlation is."""
    (dx, dy), response = cv2.phaseCorrelate(older, newer, _window(older.shape))
    return float(dx), float(dy), float(response)


def _patches(shape: Tuple[int, int]) -> List[Tuple[slice, slice]]:
    """The picture cut three by three, without the middle: the eight patches round its border.

    What hangs under the vehicle (the hook, the bucket on it) is in the middle of the picture and may reach into the
    patches beside it; the ground is in all eight. The patches' middle is not moved by the few the load is in.
    """
    rows, cols = shape[0] // 3, shape[1] // 3
    return [(slice(row * rows, (row + 1) * rows), slice(col * cols, (col + 1) * cols)) for row in range(3) for col in range(3) if (row, col) != (1, 1)]


def measure(older: np.ndarray, newer: np.ndarray, corners_only: bool = False) -> Tuple[Optional[Tuple[float, float]], float, str, int]:
    """(the shift or None, the response, why not, how many patches): the whole picture, or the corner patches' middle."""
    if not corners_only:
        if float(newer.std()) < MIN_TEXTURE or float(older.std()) < MIN_TEXTURE:
            return None, 0.0, "no texture in the picture (a bare surface)", 0
        dx, dy, response = shift_between(older, newer)
        if response < MIN_RESPONSE:
            return None, response, f"the correlation is weak ({response:.2f})", 0
        if abs(dx) > MAX_SHIFT_SHARE * older.shape[1] or abs(dy) > MAX_SHIFT_SHARE * older.shape[0]:
            return None, response, f"the picture moved {math.hypot(dx, dy):.0f} px between the frames: too far to follow", 0
        return (dx, dy), response, "", 0
    found = []
    middle_x, middle_y = (older.shape[1] - 1) / 2.0, (older.shape[0] - 1) / 2.0
    for rows, cols in _patches(older.shape):
        one, two = np.ascontiguousarray(older[rows, cols]), np.ascontiguousarray(newer[rows, cols])
        if float(one.std()) < MIN_TEXTURE or float(two.std()) < MIN_TEXTURE:
            continue
        dx, dy, response = shift_between(one, two)
        if response >= MIN_RESPONSE and abs(dx) <= MAX_SHIFT_SHARE * one.shape[1] and abs(dy) <= MAX_SHIFT_SHARE * one.shape[0]:
            found.append((dx, dy, response, (cols.start + cols.stop - 1) / 2.0 - middle_x, (rows.start + rows.stop - 1) / 2.0 - middle_y))
    if len(found) < MIN_PATCHES:
        return None, 0.0, f"only {len(found)} of the eight border patches have texture to follow", len(found)
    fitted = fit_motion(found)
    if fitted is None:
        spread = max(math.hypot(item[0] - np.median([f[0] for f in found]), item[1] - np.median([f[1] for f in found])) for item in found)
        return None, 0.0, f"the border patches disagree by {spread:.1f} px", 0
    shift, agreeing = fitted
    return shift, float(np.median([item[2] for item in agreeing])), "", len(agreeing)


def fit_motion(found: List[Tuple[float, float, float, float, float]]) -> Optional[Tuple[Tuple[float, float], list]]:
    """The picture's shift at its middle from the patches' shifts: ((dx, dy), the patches that agree), or None.

    A patch's shift is the shift of the whole picture, plus the picture growing or shrinking about its middle (the
    vehicle coming down or going up: a patch moves outward or inward by its distance from the middle), plus the
    picture turning about its middle (the vehicle yawing). All three are fitted together, by least squares, to
    patches given as (dx, dy, response, x from the middle, y from the middle). A patch more than
    MAX_PATCH_SPREAD_PX from the motion most patches agree with (it is on the hook or the bucket, or it was matched
    wrongly) is left out; fewer than MIN_PATCHES agreeing is no answer. Without the growing in the fit, a descent
    biased the shift by pixels: the patches on one side "agreed" and the others did not.
    """
    if len(found) < MIN_PATCHES:
        return None

    def solve(patches):
        rows, values = [], []
        for dx, dy, _response, x, y in patches:
            rows += [[1.0, 0.0, x, -y], [0.0, 1.0, y, x]]
            values += [dx, dy]
        solution, *_ = np.linalg.lstsq(np.asarray(rows), np.asarray(values), rcond=None)
        return tuple(float(value) for value in solution)

    def error(patch, motion) -> float:
        dx, dy, _response, x, y = patch
        tx, ty, grow, turn = motion
        return math.hypot(dx - (tx + grow * x - turn * y), dy - (ty + grow * y + turn * x))

    # Every pair of patches gives one motion exactly; the motion most patches agree with is the ground's. The load
    # under the vehicle moves as one thing too, so a majority is asked for: MIN_PATCHES of the eight.
    best: list = []
    for first in range(len(found)):
        for second in range(first + 1, len(found)):
            motion = solve([found[first], found[second]])
            agreeing = [patch for patch in found if error(patch, motion) <= MAX_PATCH_SPREAD_PX]
            if len(agreeing) > len(best):
                best = agreeing
    if len(best) < MIN_PATCHES:
        return None
    motion = solve(best)
    agreeing = [patch for patch in found if error(patch, motion) <= MAX_PATCH_SPREAD_PX]
    if len(agreeing) < MIN_PATCHES:
        return None
    motion = solve(agreeing)
    return (motion[0], motion[1]), agreeing


def quality_of(response: float) -> int:
    """0 to 255 from the correlation's response: 1 at MIN_RESPONSE, 255 from FULL_RESPONSE up."""
    if response < MIN_RESPONSE:
        return 0
    return int(round(1 + 254 * min(1.0, (response - MIN_RESPONSE) / (FULL_RESPONSE - MIN_RESPONSE))))


# The mount every picture in this module's text assumes: the camera looks straight down with the picture's top to
# the nose. Columns are the picture's x (the vehicle's right), the picture's y (its back) and the lens axis (down),
# each in the body's forward-left-up; rows are forward, left, up. An airframe's own mount replaces it.
NOSE_UP_MOUNT = ((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
MAX_OFF_NADIR = 0.02  # the lens axis must be the body's down to within this (1 degree): the flow is of a camera that looks down


def check_mount(body_from_optical) -> Tuple[Tuple[float, float, float], ...]:
    """The mount's rotation (optical frame to the body's forward-left-up) as tuples, refused unless the camera looks straight down."""
    rows = tuple(tuple(float(value) for value in row) for row in body_from_optical)
    if len(rows) != 3 or any(len(row) != 3 for row in rows):
        raise ValueError("the mount's rotation must be three rows of three")
    if abs(rows[2][2] + 1.0) > MAX_OFF_NADIR or abs(rows[0][2]) > 0.2 or abs(rows[1][2]) > 0.2:
        raise ValueError("the flow camera must look straight down: its mount's lens axis is not the body's down")
    return rows


def rotation_shift_px(older: Attitude, newer: Attitude, focal_px: float, body_from_optical=NOSE_UP_MOUNT) -> Tuple[float, float]:
    """The shift of the picture's content that the vehicle's turning alone makes, in pixels, for a camera mounted as `body_from_optical` says.

    The vehicle turned by (roll, minus pitch, yaw) about its forward, left and up axes (pitch is nose up, which is a
    turn about the right). In the camera's own axes that is a turn about the picture's x and y, and a camera that
    turns about its x moves the picture's content along y, and about its y along minus x. With the nose-up mount:
    nose up moves the content towards the tail (+y), right side down moves it right (+x). Turning about the lens
    axis moves nothing at the picture's middle.
    """
    turned_body = (newer.roll_rad - older.roll_rad, -(newer.pitch_rad - older.pitch_rad), 0.0)
    about_x = sum(body_from_optical[row][0] * turned_body[row] for row in range(3))
    about_y = sum(body_from_optical[row][1] * turned_body[row] for row in range(3))
    return -focal_px * about_y, focal_px * about_x


def velocity(shift_px: Tuple[float, float], rotation_px: Tuple[float, float], height_m: float, focal_px: float, dt_s: float, body_from_optical=NOSE_UP_MOUNT) -> Tuple[float, float]:
    """(forward, right) in metres a second from the shift with the turning taken out, for a camera mounted as `body_from_optical` says.

    The ground's picture moves against the camera: the camera's velocity along the picture's x and y is minus the
    shift, scaled by the height. Those two axes are then the mount's to place on the vehicle. With the nose-up
    mount: content moving down the picture is the vehicle going forward, content moving right is it going left.
    """
    scale = height_m / (focal_px * dt_s)
    along_x, along_y = -(shift_px[0] - rotation_px[0]) * scale, -(shift_px[1] - rotation_px[1]) * scale
    forward = body_from_optical[0][0] * along_x + body_from_optical[0][1] * along_y
    left = body_from_optical[1][0] * along_x + body_from_optical[1][1] * along_y
    return forward, -left


def to_east_north(forward_mps: float, right_mps: float, yaw_rad: float) -> Tuple[float, float]:
    return forward_mps * math.cos(yaw_rad) + right_mps * math.sin(yaw_rad), forward_mps * math.sin(yaw_rad) - right_mps * math.cos(yaw_rad)


class FlowOdometry:
    """Frames in, one sample out for each frame once `frames_apart` frames are held. `focal_px` is of the 320-wide picture."""

    def __init__(self, focal_px: float, frames_apart: int = FRAMES_APART, body_from_optical=NOSE_UP_MOUNT) -> None:
        if not focal_px > 0 or frames_apart < 1:
            raise ValueError("the focal length must be above zero and the frames at least one apart")
        self.mount = check_mount(body_from_optical)  # how the camera sits on the vehicle; refused unless it looks straight down
        self.focal_px = float(focal_px)
        self.frames_apart = int(frames_apart)
        self._held: Deque[Tuple[int, np.ndarray, Optional[Attitude]]] = deque(maxlen=self.frames_apart)

    def reset(self) -> None:
        self._held.clear()

    def add(self, image: np.ndarray, stamp_ns: int, attitude: Optional[Attitude], height_m: Optional[float], corners_only: bool = False) -> Optional[FlowSample]:
        """Take a frame with the attitude of its own moment and the height above the ground; None until enough frames are held."""
        picture = prepare(image)
        if self._held and stamp_ns <= self._held[-1][0]:
            self._held.clear()  # the clock stepped back, or a frame came twice: start again
        full = len(self._held) == self.frames_apart
        earlier = list(self._held)  # oldest first
        self._held.append((int(stamp_ns), picture, attitude))
        if not full:
            return None
        newest_dt_s = (stamp_ns - earlier[-1][0]) * 1.0e-9

        def nothing(why: str, dt_s: float = newest_dt_s, **more) -> FlowSample:
            return FlowSample(int(stamp_ns), dt_s, 0, why, **more)

        if newest_dt_s > MAX_DT_S:
            return nothing(f"the frames are {newest_dt_s:.2f} s apart")
        if height_m is None or not math.isfinite(height_m) or not MIN_HEIGHT_M <= height_m <= MAX_HEIGHT_M:
            return nothing("no height above the ground to scale the flow with" if height_m is None else f"the height {height_m:.1f} m is outside {MIN_HEIGHT_M:g} to {MAX_HEIGHT_M:g} m")
        if attitude is None:
            return nothing("no attitude at the frames' times to take the turning out with")
        # The oldest held frame first: the longest time between the two pictures that can still be followed.
        failed: Optional[FlowSample] = None
        for older_ns, older_picture, older_attitude in earlier:
            dt_s = (stamp_ns - older_ns) * 1.0e-9
            if dt_s > MAX_DT_S:
                continue
            if older_attitude is None:
                failed = failed or nothing("no attitude at the frames' times to take the turning out with", dt_s)
                continue
            shift, response, why, patches = measure(older_picture, picture, corners_only)
            if shift is None:
                failed = nothing(why, dt_s, patches=patches, height_m=float(height_m))  # the reason kept is the nearest frame's: the best chance there was
                continue
            rotation = rotation_shift_px(older_attitude, attitude, self.focal_px, self.mount)
            forward, right = velocity(shift, rotation, float(height_m), self.focal_px, dt_s, self.mount)
            east, north = to_east_north(forward, right, attitude.yaw_rad)
            return FlowSample(int(stamp_ns), dt_s, quality_of(response), "", shift, rotation, forward, right, east, north, float(height_m), patches)
        return failed if failed is not None else nothing(f"the frames are {newest_dt_s:.2f} s apart")


def attitude_from_quaternion(orientation: Tuple[float, float, float, float]) -> Attitude:
    """A MAVROS orientation (x, y, z, w; the body's forward-left-up in east-north-up) as roll, pitch (nose up) and heading."""
    x, y, z, w = orientation
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    nose_down = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))  # about the body's left: positive is nose down
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return Attitude(roll, -nose_down, yaw)


BARO_MAX_QUALITY = 127  # a sample scaled by a height that is not the rangefinder's is worth no more than this


def with_height_source(sample: FlowSample, height_source: str) -> FlowSample:
    """A sample with its quality held down when the height it was scaled by is not the rangefinder's.

    Above the rangefinder's range the flow is scaled by the vehicle's height in the local frame, which is above home
    and not above the ground under it: right over level ground, and wrong by the slope elsewhere.
    """
    if sample.quality == 0 or height_source == "rangefinder" or sample.quality <= BARO_MAX_QUALITY:
        return sample
    return FlowSample(sample.stamp_ns, sample.dt_s, BARO_MAX_QUALITY, sample.why, sample.shift_px, sample.rotation_px, sample.forward_mps, sample.right_mps, sample.east_mps, sample.north_mps,
                      sample.height_m, sample.patches)  # fmt: skip


def mavlink_fields(sample: FlowSample, focal_px: float) -> dict:
    """The sample as MAVLink's OPTICAL_FLOW carries one (mavros_msgs/OpticalFlow).

    flow: the raw shift of the picture's content between the two frames, in pixels of the 320-wide picture and in
    the picture's axes (x right, y towards the tail); flow_comp_m: the velocity over the ground in metres a second,
    forward and left (the body frame ROS uses), turning taken out; flow_rate: the raw flow as an angle a second.
    A sample with no quality is sent with zeros and quality 0, so that the receiver knows the sensor is alive and
    has nothing to say.

    NOT CHECKED against MAVROS's optical_flow plugin or ArduPilot's MAVLink flow backend: which frame the plugin
    turns these fields from, and how the flight controller scales `flow` (it carries no time between the frames,
    and that time varies here). The node therefore sends nothing unless `send_optical_flow` is set.
    """
    if sample.quality == 0:
        return {"flow": (0.0, 0.0), "flow_comp_m": (0.0, 0.0), "flow_rate": (0.0, 0.0), "quality": 0, "ground_distance": float(sample.height_m) if sample.height_m else -1.0}
    rate = (sample.shift_px[0] / (focal_px * sample.dt_s), sample.shift_px[1] / (focal_px * sample.dt_s))
    return {"flow": sample.shift_px, "flow_comp_m": (sample.forward_mps, -sample.right_mps), "flow_rate": rate, "quality": sample.quality, "ground_distance": sample.height_m}
