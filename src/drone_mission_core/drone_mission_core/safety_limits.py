"""Rule-derived safety limits shared by every mission. Constants, not config.

The 2027 rules (Rev 1) take points, or the whole mission, for a drop released
below 6 ft, a survey flown below 25 ft, or a package or bucket released above
6 ft. Those numbers, and our own margins on top of them, are here as code
constants so that a tuning edit on the field cannot move them: nothing in
this module reads a parameter, a file or the environment.

A 2027 mission imports its limits from here and passes every height or speed
it is about to command through the functions below. The June 2026 missions
are not changed and do not import this module.

Heights are metres above the ground under the vehicle, which is what the
rules measure. A caller passes height above ground (FND-07's `height_agl()`),
never altitude above home: over ground higher than home the two differ, and
the limit would be wrong by that much.

`check_sequence` checks the heights a sequence file configures. The executor
runs it when a sequence is loaded, but refuses to start on a violation only
when the mission flag `enforce_safety_limits` is on; with the flag off a
sequence loads exactly as it did in 2026. That check reads the configured
numbers, which the 2026 missions fly as altitude above home: it shows that a
sequence asks for a legal height over ground level with home, and nothing
about the ground at the target or what the aircraft does once it is there.
"""

from __future__ import annotations

import math
import numbers
from typing import Any, Dict, Iterable, List

from .features import get_feature_flag

FEET_TO_METERS = 0.3048

# --- From the rules -----------------------------------------------------------

# Drop: the package is released at or above 6 ft, and going below 6 ft costs
# 6 points. 6 ft is 1.8288 m; 1.83 rounds it up, the safe way for a floor.
DROP_MIN_RELEASE_HEIGHT_M = 1.83
# Delivery and recovery: a release above 6 ft scores zero. This one is a
# ceiling, so 1.8288 m is rounded down.
SETDOWN_MAX_RELEASE_HEIGHT_M = 1.82
# Localisation: below 25 ft costs 3 points, below 20 ft the mission scores zero.
SURVEY_MIN_HEIGHT_M = 7.62
SURVEY_ZERO_SCORE_HEIGHT_M = 6.10

# --- Our own margins ----------------------------------------------------------

# Drop: below this the drone climbs back before anything else, and no drop
# height command goes under it, which keeps 1.83 m out of reach (CEN-04).
RELEASE_GATE_FLOOR_M = 2.0
# Recovery: a swinging bucket is released at or below this, or on the ground (REC-4).
RECOVERY_MAX_RELEASE_HEIGHT_M = 1.5
# Commanded positions keep this far inside the flight boundary: ArduPilot's
# own 2 m fence margin, plus 3 m for GPS error and overshoot at a waypoint.
GEOFENCE_MARGIN_M = 5.0

# The fastest a mission may command the vehicle in each phase, in m/s.
PHASE_SPEED_CAPS_MPS: Dict[str, float] = {
    "centring": 1.0,  # horizontal correction over a target (CEN-02)
    "survey": 4.0,  # lawnmower legs of the localisation survey (LOC-2)
    "carrying": 3.0,  # with the bucket on the hook (REC-4)
    "final_descent": 0.5,  # the last metres onto a handle or a release height (REC-3)
    "set_down": 0.12,  # the last 0.6 m of a delivery, package on the hook (CEN-05)
}

FLAG_ENFORCE = "enforce_safety_limits"

# Defaults of package_drop_mission.py, used when a sequence leaves a key out.
# A test compares them with that file, so they cannot drift apart unnoticed.
_DROP_DEFAULT_ALTITUDE_M = 5.0
_DROP_DEFAULT_TOLERANCE_M = 0.5


def _is_number(value: Any) -> bool:
    """A real, finite number: not a bool, not NaN, not infinity.

    Any real number type counts, including the NumPy floats that vision code
    produces; otherwise a valid float32 height would read as "not allowed".
    """
    return isinstance(value, numbers.Real) and not isinstance(value, bool) and bool(math.isfinite(value))


def _require_height(value: Any, what: str) -> float:
    # Raising is the safe failure: in a mission's update() it becomes a
    # FAILURE and the failure policy takes over. Returning the floor instead
    # would turn a broken estimate into a commanded descent.
    if not _is_number(value):
        raise ValueError(f"{what} is {value!r}, not a finite height")
    return float(value)


# --- What a command may ask for -----------------------------------------------


def drop_height_command(requested_m: float) -> float:
    """The height above ground to command for a drop: never under the release gate floor.

    Raises ValueError when the request is not a finite number.
    """
    return max(_require_height(requested_m, "the requested drop height"), RELEASE_GATE_FLOOR_M)


def survey_height_command(requested_m: float) -> float:
    """The height above ground to command for a survey: never under 25 ft.

    Raises ValueError when the request is not a finite number.
    """
    return max(_require_height(requested_m, "the requested survey height"), SURVEY_MIN_HEIGHT_M)


def drop_release_allowed(height_m: float) -> bool:
    """A drop may release only at or above our floor, which is above the rule's 6 ft."""
    return _is_number(height_m) and float(height_m) >= RELEASE_GATE_FLOOR_M


def set_down_release_allowed(height_m: float, landed: bool = False) -> bool:
    """A delivery may release on the ground, or at or below 6 ft.

    `landed` must be the flight controller's current landed state, not a
    guess or an old reading: it overrides the height.
    """
    return bool(landed) or (_is_number(height_m) and float(height_m) <= SETDOWN_MAX_RELEASE_HEIGHT_M)


def recovery_release_allowed(height_m: float, landed: bool = False) -> bool:
    """A bucket may be released on the ground, or at or below our own 1.5 m.

    `landed` must be the flight controller's current landed state, not a
    guess or an old reading: it overrides the height.
    """
    return bool(landed) or (_is_number(height_m) and float(height_m) <= RECOVERY_MAX_RELEASE_HEIGHT_M)


def capped_speed(phase: str, speed_mps: float) -> float:
    """A commanded speed limited to the phase's cap, keeping its sign. Not a number gives 0."""
    if phase not in PHASE_SPEED_CAPS_MPS:
        raise KeyError(f"no speed cap for phase '{phase}' (phases: {', '.join(PHASE_SPEED_CAPS_MPS)})")
    if not _is_number(speed_mps):
        return 0.0
    cap = PHASE_SPEED_CAPS_MPS[phase]
    return max(-cap, min(cap, float(speed_mps)))


# --- A sequence's configured heights ------------------------------------------


def check_mission(spec: Any) -> List[str]:
    """What in one mission's configuration breaks a limit. Empty when nothing does.

    For `package_drop` this is the lowest height at which the 2026 mission
    starts its drop: the configured altitude minus its tolerance. The mission
    releases after a hover with no further height check, so a passing
    configuration is necessary for a legal drop, not proof of one.
    """
    problems: List[str] = []
    config = spec.config or {}
    if spec.type_name == "package_drop":
        altitude = config.get("drop_altitude_m", _DROP_DEFAULT_ALTITUDE_M)
        tolerance = config.get("drop_altitude_tolerance_m", _DROP_DEFAULT_TOLERANCE_M)
        for key, value in (("drop_altitude_m", altitude), ("drop_altitude_tolerance_m", tolerance)):
            if not _is_number(value):
                problems.append(f"{spec.name} ({spec.type_name}): {key} is {value!r}, not a number")
        if not problems:
            lowest = float(altitude) - abs(float(tolerance))
            # A hundredth of a millimetre of slack, so 2.33 - 0.5 is not refused for float rounding.
            if lowest < DROP_MIN_RELEASE_HEIGHT_M - 1.0e-9:
                problems.append(
                    f"{spec.name} ({spec.type_name}): drop_altitude_m {float(altitude):.2f} minus its tolerance {abs(float(tolerance)):.2f} "
                    f"allows a release at {lowest:.2f} m, below the {DROP_MIN_RELEASE_HEIGHT_M:.2f} m (6 ft) rule"
                )
    return problems


def check_sequence(specs: Iterable[Any]) -> List[str]:
    """Every limit a sequence's missions break, as one line each."""
    return [problem for spec in specs for problem in check_mission(spec)]


def enforced_violations(specs: Iterable[Any]) -> List[str]:
    """The violations that stop the executor from starting: none unless the flag is on."""
    specs = list(specs)
    if not any(get_feature_flag(spec.features, FLAG_ENFORCE) is True for spec in specs):
        return []
    return check_sequence(specs)
