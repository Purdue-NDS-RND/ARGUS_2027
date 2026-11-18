"""What a mission reads of the target estimator, and the rules for acting on it (FND-07). No ROS imports.

`MissionContext` (mission_context.py) is the thin side: it finds the
executor's estimator, pose and GPS and calls the functions here. The rules
live in this one place, not in each mission:

  - descending needs a CONFIRMED track no older than 0.5 s;
  - releasing needs a CONFIRMED track no older than 0.3 s.

A mission that has chosen a track asks about that track, by its id: the
answer is then never about another target of the same class.

"Old" is the age of the last frame that hit the track, against the clock
now. A track that is COASTING, LOST or only TENTATIVE never permits either,
however recent, and no track permits nothing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional, Tuple

DESCEND_MAX_AGE_S = 0.5
RELEASE_MAX_AGE_S = 0.3
ACTUATION_DELAY_S = 0.08  # as target_estimator.ACTUATION_DELAY_S (a test holds the two together)
ESTIMATOR_OFF = "the target estimator is off (use_target_estimator)"

# How old the things a height or a position is made from may be (chosen here; see PROGRESS.md, Decisions).
POSE_MAX_AGE_S = 0.5  # a height from a pose older than this is not given: the vehicle has moved since
FIX_MAX_AGE_S = 2.0  # nor a position from a GPS fix older than this
FIX_POSE_MAX_APART_S = 0.25  # the fix and the pose it is paired with must be of the same moment, within this
CONFIRMED = "CONFIRMED"  # as target_estimator.CONFIRMED; not imported, so this module needs no numpy
STAMP_AHEAD_TOLERANCE_S = 0.05  # a hit may be stamped this far ahead of the clock (the feed drops anything further)

# Metres per degree of latitude as ArduPilot counts them (LOCATION_SCALING_FACTOR,
# 0.011131884502145034 m per 1e-7 degree). The local frame the tracks are in is
# ArduPilot's, so a latitude and longitude are put into it with its own scale.
METRES_PER_DEGREE = 111318.84502145034

RANGEFINDER, BARO = "rangefinder", "baro"


@dataclass(frozen=True)
class PrimaryTarget:
    """The track of a class a mission acts on, as the estimator gives it for the actuation time."""

    class_id: str
    track_id: int
    state: str  # TENTATIVE, CONFIRMED, COASTING or LOST
    position: Tuple[float, float, float]  # east, north, up in MAVROS's local frame, metres
    covariance: Tuple[Tuple[float, float], Tuple[float, float]]  # of east and north, metres squared
    sigma_m: float  # the larger standard deviation of the covariance
    age_s: float  # since the frame of the last hit
    for_ns: int  # the time the position and covariance are given for


@dataclass(frozen=True)
class Permission:
    """Whether a mission may do something on a track, and if not, why. True or false in an `if`."""

    allowed: bool
    reason: str = ""  # empty when allowed
    track_id: Optional[int] = None  # the track the answer is about, when there was one

    def __bool__(self) -> bool:
        return self.allowed


@dataclass(frozen=True)
class HeightAgl:
    value_m: float
    source: str  # "rangefinder" (above the ground it measured) or "baro" (above home: not the ground, where the ground is not at home's height)
    pose_age_s: float = 0.0  # how old the pose the height was read from is


def from_estimate(estimate: Any) -> PrimaryTarget:
    """The mission's view of one of the estimator's `TargetEstimate`s."""
    return PrimaryTarget(
        class_id=str(estimate.class_id),
        track_id=int(estimate.track_id),
        state=str(estimate.state),
        position=(float(estimate.east_m), float(estimate.north_m), float(estimate.up_m)),
        covariance=tuple(tuple(float(value) for value in row) for row in estimate.covariance),
        sigma_m=float(estimate.sigma_m),
        age_s=float(estimate.age_s),
        for_ns=int(estimate.for_ns),
    )


def _permission(target: Optional[PrimaryTarget], max_age_s: float, what: str) -> Permission:
    if target is None:
        return Permission(False, f"no track to {what} on")
    if target.state != CONFIRMED:
        return Permission(False, f"the {target.class_id} track is {target.state}, not CONFIRMED", target.track_id)
    if not math.isfinite(target.age_s) or target.age_s < -STAMP_AHEAD_TOLERANCE_S:
        return Permission(False, f"the {target.class_id} track's age ({target.age_s:.2f} s) is not a time since its last hit", target.track_id)
    if target.age_s > max_age_s:
        return Permission(False, f"the {target.class_id} track is {target.age_s:.2f} s old (limit {max_age_s:g} s to {what})", target.track_id)
    return Permission(True, "", target.track_id)


def within(stamp_ns: Optional[int], now_ns: int, max_age_s: float) -> bool:
    """Whether something stamped `stamp_ns` is no more than `max_age_s` from `now_ns`, either way (a stamp well ahead of the clock is as unusable as an old one)."""
    return stamp_ns is not None and abs(int(now_ns) - int(stamp_ns)) <= int(max_age_s * 1.0e9)


def descent_permission(target: Optional[PrimaryTarget]) -> Permission:
    """Descending needs a CONFIRMED track no older than 0.5 s."""
    return _permission(target, DESCEND_MAX_AGE_S, "descend")


def release_permission(target: Optional[PrimaryTarget]) -> Permission:
    """Releasing needs a CONFIRMED track no older than 0.3 s."""
    return _permission(target, RELEASE_MAX_AGE_S, "release")


def height_agl(vehicle_up_m: float, ground: Optional[Tuple[float, str]], pose_age_s: float = 0.0) -> Optional[HeightAgl]:
    """Height above the ground and where it came from.

    `vehicle_up_m` is the vehicle's height in the local frame (above home).
    `ground` is what projection v2's GroundHeight says: (the ground's height
    in the same frame, "rangefinder" or "home"), or None when there is none.
    With a healthy rangefinder the height is above the ground it measured;
    otherwise it is the height above home, which is the barometer's.
    """
    if not math.isfinite(float(vehicle_up_m)):
        return None
    if ground is not None and ground[1] == RANGEFINDER and math.isfinite(float(ground[0])):
        return HeightAgl(float(vehicle_up_m) - float(ground[0]), RANGEFINDER, float(pose_age_s))
    return HeightAgl(float(vehicle_up_m), BARO, float(pose_age_s))


def latlon_to_local_enu(
    latitude: float, longitude: float, here_latitude: float, here_longitude: float, here_east_m: float, here_north_m: float
) -> Optional[Tuple[float, float]]:
    """(east, north) in the local frame of a latitude and longitude, given where the vehicle is in both.

    The offset from the vehicle's own latitude and longitude is added to the
    vehicle's local position, so the local frame's origin is never needed.
    None when a number is not finite or not a latitude or longitude.
    """
    values = (latitude, longitude, here_latitude, here_longitude, here_east_m, here_north_m)
    try:
        values = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    latitude, longitude, here_latitude, here_longitude, here_east_m, here_north_m = values
    if abs(latitude) > 90.0 or abs(here_latitude) > 90.0 or abs(longitude) > 180.0 or abs(here_longitude) > 180.0:
        return None
    d_longitude = (longitude - here_longitude + 180.0) % 360.0 - 180.0  # the short way round
    scale = math.cos(math.radians(0.5 * (latitude + here_latitude)))  # as ArduPilot: at the middle latitude
    north = (latitude - here_latitude) * METRES_PER_DEGREE
    east = d_longitude * METRES_PER_DEGREE * scale
    return (here_east_m + east, here_north_m + north)
