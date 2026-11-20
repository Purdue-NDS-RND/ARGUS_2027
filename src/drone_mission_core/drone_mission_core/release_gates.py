"""The drop's release gates (CEN-04): what must hold, together and for half a second, before the payload leaves. No ROS imports.

Behind the mission flag `drop_release_gates`, which defaults to false and
needs `centre_on_estimator`. With the flag off the drop releases as in 2026,
from a fixed GPS column after a timed hover, and nothing here is used.

With it on, the drop's last state descends on the estimator to the release
height and lets go only when all of these hold at once, for 0.5 s without a
break:

  error    the payload's aim point within 0.10 m of the target's track
  sigma    the track's own standard deviation at most 0.08 m
  speed    the vehicle moving no faster than 0.15 m/s over the ground
  tilt     the vehicle tilted no more than 6 degrees
  track    the context's release rule says yes: a CONFIRMED track, hit no
           more than 0.3 s ago (FND-07)
  height   inside the release band for the sensor that gave it:
           2.4 m +/- 0.2 on the rangefinder, 3.0 m +/- 0.2 on the barometer,
           whose height is trusted less and so kept further above the
           rules' 1.83 m floor

What is not known does not pass: no speed, no height, a number that is not a
number. The numbers are the spec's and are constants here: they are what a
release is judged by, not something to tune on the field.

If the gates have not held within 25 s the mission looks for the target
again, once; after a second 25 s it releases anyway if `last_resort_allowed`
says so (a track the release rule accepts, within 0.5 m, at a legal
height), because a release slightly off the centre scores and no release
scores nothing. Otherwise it gives up and the sequence moves on.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from .safety_limits import RELEASE_GATE_FLOOR_M, drop_release_allowed

GATES_FLAG = "drop_release_gates"

MAX_ERROR_M = 0.10
MAX_SIGMA_M = 0.08
MAX_SPEED_MPS = 0.15
MAX_TILT_DEG = 6.0
HOLD_S = 0.5
RANGEFINDER_RELEASE_M = 2.4
BARO_RELEASE_M = 3.0
RELEASE_BAND_M = 0.2
ALIGN_TIMEOUT_S = 25.0
LAST_RESORT_MAX_ERROR_M = 0.5
# Two readings further apart than this are not one unbroken stretch: the executor's tick is 0.05 s.
MAX_TICK_GAP_S = 0.2
_RANGEFINDER = "rangefinder"


def release_band(height_source: str) -> Tuple[float, float, float]:
    """(lowest, aimed-for, highest) release height above the ground for a height from this source."""
    target = RANGEFINDER_RELEASE_M if height_source == _RANGEFINDER else BARO_RELEASE_M  # anything else is trusted as little as the barometer
    return max(target - RELEASE_BAND_M, RELEASE_GATE_FLOOR_M), target, target + RELEASE_BAND_M


@dataclass(frozen=True)
class GateReading:
    """One tick's measurements of everything the gates look at."""

    error_m: Optional[float]  # the payload's aim point to the track, horizontally
    sigma_m: Optional[float]  # the track's standard deviation
    speed_mps: Optional[float]  # the vehicle's speed over the ground
    tilt_deg: Optional[float]
    may_release: bool  # the context's release rule for the track being flown to
    height_m: Optional[float]  # above the ground
    height_source: str  # "rangefinder" or "baro"


def _number(value: Optional[float]) -> bool:
    return value is not None and not isinstance(value, bool) and math.isfinite(value)


def failing_gates(reading: GateReading) -> Tuple[str, ...]:
    """The names of the gates that do not hold for this reading, in the order of the module text. Empty when all hold."""
    failing = []
    if not _number(reading.error_m) or reading.error_m > MAX_ERROR_M:
        failing.append("error")
    if not _number(reading.sigma_m) or reading.sigma_m > MAX_SIGMA_M:
        failing.append("sigma")
    if not _number(reading.speed_mps) or reading.speed_mps > MAX_SPEED_MPS:
        failing.append("speed")
    if not _number(reading.tilt_deg) or reading.tilt_deg > MAX_TILT_DEG:
        failing.append("tilt")
    if reading.may_release is not True:
        failing.append("track")
    low, _target, high = release_band(reading.height_source)
    if not _number(reading.height_m) or not low <= reading.height_m <= high:
        failing.append("height")
    return tuple(failing)


def lowest_release_m(height_source: str) -> float:
    """The lowest height anything is released at, by the sensor that gives the height.

    On the rangefinder, our 2.0 m floor. On anything else the bottom of its
    band (2.8 m): the barometer's height is above home, not above the ground
    under the vehicle, and that error is the reason its band is higher.
    """
    return RELEASE_GATE_FLOOR_M if height_source == _RANGEFINDER else release_band(height_source)[0]


def release_height_allowed(height_m: Optional[float], height_source: str) -> bool:
    """Whether a release at this height, from this source, is above the lowest release height."""
    return _number(height_m) and drop_release_allowed(height_m) and height_m >= lowest_release_m(height_source)


def last_resort_allowed(reading: GateReading) -> bool:
    """Whether to release anyway after the last timeout: a track the release rule accepts, within 0.5 m, and not under the lowest release height."""
    return (
        reading.may_release is True
        and _number(reading.error_m)
        and reading.error_m <= LAST_RESORT_MAX_ERROR_M
        and release_height_allowed(reading.height_m, reading.height_source)
    )


class GateHold:
    """update(): True once every gate has held, tick after tick, for HOLD_S."""

    def __init__(self) -> None:
        self._since_s: Optional[float] = None
        self._last_s: Optional[float] = None
        self.held_s = 0.0
        self.last_failing: Tuple[str, ...] = ()

    def reset(self) -> None:
        self._since_s = self._last_s = None
        self.held_s = 0.0

    def update(self, reading: GateReading, now_s: float) -> bool:
        """This tick's reading at time `now_s` (any clock that counts seconds)."""
        self.last_failing = failing_gates(reading)
        broken = self._last_s is not None and not 0.0 <= now_s - self._last_s <= MAX_TICK_GAP_S
        self._last_s = now_s
        if self.last_failing:
            self._since_s, self.held_s = None, 0.0
            return False
        if self._since_s is None or broken:
            self._since_s = now_s
        self.held_s = now_s - self._since_s
        return self.held_s >= HOLD_S - 1.0e-9
