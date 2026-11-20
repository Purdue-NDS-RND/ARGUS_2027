"""Smart re-acquisition (CEN-06): how high to climb to look for a lost target again.

Behind the mission flag `smart_reacquire`, which defaults to false. In 2026
a mission that lost its target held for `target_loss_grace_s`, climbed
`not_found_ascent_m` (a metre) from where it was, and looked again. With
the flag on it climbs to the height from which the camera sees everywhere
the target can be: three standard deviations of the track round its last
estimate, plus a margin,

    h = (3 sigma + 2.5 m) / k

where k is half the camera footprint's width per metre of height, on the
picture's short side (0.463 for the drop camera mode: 180 px / 389 px). It
holds there, over the last estimate, until the track is CONFIRMED again,
for at most HOLD_S; then the mission's failure policy acts.

The vehicle never comes down to look: already above h, it stays where it
is. It never climbs over the mission's recovery ceiling. It moves at most
MAX_SHIFT_M sideways toward the estimate: what is left of the distance is
added to what the footprint has to cover, so a far estimate is looked for
from higher, not flown to.

The mission side is drone_mission_demo/missions/smart_reacquire.py. No ROS
here; the standard library only.
"""

from __future__ import annotations

import math
from typing import Any, Tuple

REACQUIRE_FLAG = "smart_reacquire"

SIGMAS = 3.0
MARGIN_M = 2.5
HOLD_S = 10.0  # at the height, waiting for the track to be CONFIRMED again
HALF_FOOTPRINT_PER_M = 0.463  # the drop camera mode's; used when the mission has no calibrated camera

MAX_SHIFT_M = MARGIN_M  # the vehicle moves at most this far sideways to look: the rest is looked for from higher
ARRIVAL_SHORT_M = 0.1  # the look is from the height: this far under it at most

# A climb that does not arrive is given up: this long, plus the climb at the slowest rate it could be flown.
CLIMB_BASE_S = 20.0
SLOWEST_CLIMB_MPS = 0.5


def _number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def half_footprint_per_m(camera: Any) -> float:
    """Half the footprint's short side per metre of height, from a calibrated camera model; the spec's number without one."""
    try:
        sides = (float(camera.width) / (2.0 * float(camera.fx)), float(camera.height) / (2.0 * float(camera.fy)))
    except (AttributeError, TypeError, ValueError, ZeroDivisionError):
        return HALF_FOOTPRINT_PER_M
    short = min(sides)
    return short if math.isfinite(short) and short > 0.0 else HALF_FOOTPRINT_PER_M


def look_point(vehicle_en: Any, estimate_en: Any) -> Tuple[Tuple[float, float], float]:
    """Where to look from, and how far the estimate is from there: the estimate itself within MAX_SHIFT_M of the vehicle, MAX_SHIFT_M toward it otherwise."""
    east, north = float(estimate_en[0]) - float(vehicle_en[0]), float(estimate_en[1]) - float(vehicle_en[1])
    distance = math.hypot(east, north)
    if distance <= MAX_SHIFT_M:
        return (float(estimate_en[0]), float(estimate_en[1])), 0.0
    share = MAX_SHIFT_M / distance
    return (float(vehicle_en[0]) + share * east, float(vehicle_en[1]) + share * north), distance - MAX_SHIFT_M


def wanted_height(sigma_m: Any, half_footprint_per_m: float = HALF_FOOTPRINT_PER_M, offset_m: float = 0.0) -> float:
    """(3 sigma + margin + offset) / k: the height above the ground that shows it all, whatever the vehicle can reach."""
    if not _number(sigma_m) or sigma_m < 0.0:
        raise ValueError(f"the track's sigma must be a number of metres, 0 or more, got {sigma_m!r}")
    if not _number(offset_m) or offset_m < 0.0:
        raise ValueError(f"the offset of the estimate from the look point must be a number of metres, 0 or more, got {offset_m!r}")
    return (SIGMAS * float(sigma_m) + MARGIN_M + float(offset_m)) / float(half_footprint_per_m)


def reacquire_height(
    sigma_m: Any, floor_m: float, ceiling_m: float, half_footprint_per_m: float = HALF_FOOTPRINT_PER_M, offset_m: float = 0.0
) -> float:
    """The height to look from: (3 sigma + margin + offset) / k, not under `floor_m` (where the vehicle is) and not over `ceiling_m`.

    `offset_m` is how far the estimate is from the point looked from. A
    vehicle already over the ceiling stays where it is.
    """
    return max(float(floor_m), min(wanted_height(sigma_m, half_footprint_per_m, offset_m), float(ceiling_m)))


def climb_limit_s(climb_m: float) -> float:
    """How long a climb of this many metres may take before it is given up."""
    climb = float(climb_m) if _number(climb_m) and climb_m > 0.0 else 0.0
    return CLIMB_BASE_S + climb / SLOWEST_CLIMB_MPS
