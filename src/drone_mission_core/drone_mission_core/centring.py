"""Centring on the target estimator, aimed at the payload (CEN-01). No ROS imports.

Behind the mission flag `centre_on_estimator`, which defaults to false and
needs `use_target_estimator`. With the flag off the drop and delivery
missions steer as in 2026, on the latest raw detection, and nothing here is
used.

With it on, the error a tracking mission steers on is

    target - R(yaw) . payload offset - vehicle

in the local frame (east, north): `target` is the estimator's track of the
bullseye (`MissionContext.primary_target`), given for the time a command
takes effect; `payload offset` is the release point in the body frame
(forward, left, up) from the robot description; `yaw` is the vehicle's
heading. Steering that error to zero puts the payload, not the flight
controller or the camera, over the target.

The estimator already filters, so the 2026 low-pass filter and deadband are
not applied to this error. A track that is CONFIRMED or COASTING is steered
on, but only a track the context's descend rule says yes to (CONFIRMED, no
older than 0.5 s, FND-07) lets the mission descend or count itself centred:
nothing is committed to, a fixed column or a release, on a point that is
only remembered. A track that is only TENTATIVE, or LOST, or no track at
all, is no target: the mission's 2026 loss handling takes over.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional, Sequence, Tuple

CENTRE_FLAG = "centre_on_estimator"
TARGET_CLASS = "bullseye"
STEERABLE_STATES = ("CONFIRMED", "COASTING")


def yaw_of(orientation: Sequence[float]) -> float:
    """The heading of the vehicle's nose from east, counter-clockwise, in radians, from a quaternion (x, y, z, w)."""
    x, y, z, w = (float(value) for value in orientation)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def aim_error(target_en: Sequence[float], vehicle_en: Sequence[float], yaw_rad: float, payload_offset_m: Sequence[float]) -> Tuple[float, float]:
    """(east, north) from where the vehicle is to where it must be for the payload to be over the target.

    `payload_offset_m` is the release point in the body frame: forward,
    left, up of the flight controller. Only forward and left matter here.
    """
    forward, left = float(payload_offset_m[0]), float(payload_offset_m[1])
    cos, sin = math.cos(float(yaw_rad)), math.sin(float(yaw_rad))
    offset_east, offset_north = forward * cos - left * sin, forward * sin + left * cos
    return (
        float(target_en[0]) - offset_east - float(vehicle_en[0]),
        float(target_en[1]) - offset_north - float(vehicle_en[1]),
    )


@dataclass(frozen=True)
class CentringSolution:
    """What a mission steers on this tick."""

    east_m: float
    north_m: float
    error_m: float
    track_id: int
    state: str
    may_descend: bool
    why_not: str  # why the descent is not permitted; empty when it is
    sigma_m: float = math.nan  # the track's own standard deviation, for the release gates (CEN-04)


class EstimatorCentring:
    """solve(context): the error to the estimator's track of the target, for the payload, or None when there is no track to steer on.

    One approach flies to one track. The first track taken is the class's
    primary one (nearest the mission's own target); after that the mission
    keeps to it by its id. If that track is lost, the answer is "no target"
    even when another track of the class is there to be had: another target,
    or a false alarm, is never taken over in mid-descent. `forget()`, which
    the mission calls when its loss handling starts looking again, lets the
    next track be taken.
    """

    def __init__(self, payload_offset_m: Sequence[float] = (0.0, 0.0, 0.0), class_id: str = TARGET_CLASS) -> None:
        self.payload_offset_m = tuple(float(value) for value in payload_offset_m)
        self.class_id = class_id
        self.track_id: Optional[int] = None
        self.last_refusal: str = ""

    def forget(self) -> None:
        """Give up the track being flown to: the next steerable primary track is taken."""
        self.track_id = None

    def _track(self, context: Any, near_en: Optional[Sequence[float]]) -> Any:
        """The track to steer on this tick, or None (`last_refusal` says why)."""
        if self.track_id is None:
            near = None if near_en is None else (float(near_en[0]), float(near_en[1]))
            target = context.primary_target(self.class_id, near=near)
            if target is None:
                self.last_refusal = f"the estimator has no {self.class_id} track"
                return None
            if target.state not in STEERABLE_STATES:
                self.last_refusal = f"the {self.class_id} track is {target.state}"
                return None
            self.track_id = int(target.track_id)
            return target
        target = context.tracked_target(self.class_id, self.track_id)
        if target is not None and target.state in STEERABLE_STATES:
            return target
        self.last_refusal = f"{self.class_id} track {self.track_id} is {'gone' if target is None else target.state}"
        other = context.primary_target(self.class_id, near=None if target is None else target.position[:2])
        if other is not None and int(other.track_id) != self.track_id and other.state in STEERABLE_STATES:
            self.last_refusal += f"; track {other.track_id} is not taken over until the target has been looked for again"
        return None

    def solve(self, context: Any, near_en: Optional[Sequence[float]] = None) -> Optional[CentringSolution]:
        """The solution for this tick, or None (`last_refusal` says why).

        `near_en` is where the mission expects its target (its GPS target in
        the local frame): the first track taken is the one nearest it.
        """
        pose = getattr(context, "local_pose", None)
        if pose is None:
            self.last_refusal = "no pose"
            return None
        target = self._track(context, near_en)
        if target is None:
            return None
        position, orientation = pose.pose.position, pose.pose.orientation
        values = (target.position[0], target.position[1], position.x, position.y, orientation.x, orientation.y, orientation.z, orientation.w)
        if not all(math.isfinite(float(value)) for value in values):
            self.last_refusal = "the track or the pose is not a number"
            return None
        yaw = yaw_of((orientation.x, orientation.y, orientation.z, orientation.w))
        east, north = aim_error(target.position[:2], (position.x, position.y), yaw, self.payload_offset_m)
        permission = context.may_descend(self.class_id, track_id=target.track_id)
        self.last_refusal = ""
        return CentringSolution(
            east_m=east,
            north_m=north,
            error_m=math.hypot(east, north),
            track_id=int(target.track_id),
            state=str(target.state),
            may_descend=bool(permission),
            why_not="" if permission else str(getattr(permission, "reason", "")),
            sigma_m=float(getattr(target, "sigma_m", math.nan)),
        )
