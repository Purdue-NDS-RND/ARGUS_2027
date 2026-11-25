"""The release offset model (CEN-08): where a dropped package lands against where it was let go, and the aim point shifted by it.

Behind the mission flag `release_offset_model`, which defaults to false and
needs `centre_on_estimator` and `drop_release_gates`. With the flag off nothing here is used.

A package let go in wind is carried downwind while it falls. The drop
mission centres the release point over the target, so in wind the package
lands downwind of it. The model says by how much, and the aim point is
moved upwind by the same.

What the model goes by is the vehicle's own lean. A multicopter holding
still in wind leans into it, and the tangent of that lean is the wind's
drag on the vehicle over its weight. The wind's drag on the package over
the package's weight grows with the wind in the same way, so the two keep a
fixed ratio, whatever the wind's speed: no wind speed is measured or
needed. A package falling from a height h for the time that takes
(t squared is 2 h / g) under a sideways acceleration of g x K x lean drifts

    drift = K x h x lean          (downwind: against the lean)

where `lean` is the horizontal part of the vehicle's "up" over its vertical
part (east, north), low-passed over LEAN_TAU_S so that a gust or the
controller's own corrections do not move the aim point, and K is one
number for one package on one vehicle. K is fitted from drops (`fit`, and
tools/release_offset_fit.py): from the sim's first, then from the field's.

The aim point is the target less the drift: the target plus K x h x lean,
upwind. The shift is never more than `max_shift_m`, and with K at 0 (its
value until drops have been fitted) nothing is shifted and the lean is only
logged, for the fit.

No ROS imports, standard library only.
"""

from __future__ import annotations

import math
import numbers
from dataclasses import dataclass
from typing import Any, List, Mapping, Optional, Sequence, Tuple

FLAG = "release_offset_model"
LEAN_TAU_S = 2.0
MAX_SHIFT_M = 0.5  # the most the aim point is ever moved
MAX_LEAN = math.tan(math.radians(30.0))  # a lean beyond this is not a hover in wind
K_MAX = 5.0
RESTART_AFTER_S = 1.0


def lean_en(orientation_xyzw: Sequence[float]) -> Optional[Tuple[float, float]]:
    """The vehicle's lean from its attitude (x, y, z, w in the east-north-up frame): the horizontal part of its "up" over the
    vertical part, as (east, north). (0, 0) is level; leaning 10 degrees to the west is (-0.176, 0). None when it is on its side or the
    quaternion is not one."""
    try:
        x, y, z, w = (float(value) for value in orientation_xyzw)
    except (TypeError, ValueError):
        return None
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1.0e-6:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    up_east, up_north, up_up = 2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)  # the body's z axis in the world
    if up_up <= 0.5:
        return None
    return up_east / up_up, up_north / up_up


@dataclass(frozen=True)
class OffsetSettings:
    k: float = 0.0  # 0: no drops fitted yet; nothing is shifted
    max_shift_m: float = MAX_SHIFT_M

    # The drop mission's keys: `release_offset_k`, `release_offset_max_m`.
    @classmethod
    def from_config(cls, config: Mapping[str, Any]) -> "OffsetSettings":
        values = {}
        for key, name, low, high in (("release_offset_k", "k", 0.0, K_MAX), ("release_offset_max_m", "max_shift_m", 0.0, MAX_SHIFT_M)):
            if key not in config:
                continue
            value = config[key]
            if isinstance(value, bool) or not isinstance(value, numbers.Real) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{key} must be a number from {low:g} to {high:g}, got {value!r}")
            values[name] = float(value)
        return cls(**values)


def predicted_drift(k: float, height_m: float, lean: Sequence[float]) -> Tuple[float, float]:
    """Where the package lands against where it was let go, (east, north) in metres: downwind, against the lean."""
    return -k * height_m * float(lean[0]), -k * height_m * float(lean[1])


class ReleaseOffset:
    """update() with the vehicle's attitude every tick; shift() is how far to move the aim point (east, north) for a release from a height."""

    def __init__(self, settings: Optional[OffsetSettings] = None) -> None:
        self.settings = settings or OffsetSettings()
        self.lean: Optional[Tuple[float, float]] = None  # low-passed
        self._last_s: Optional[float] = None

    def reset(self) -> None:
        self.lean, self._last_s = None, None

    def update(self, orientation_xyzw: Optional[Sequence[float]], now_s: float) -> None:
        now = None if orientation_xyzw is None else lean_en(orientation_xyzw)
        if now is None or math.hypot(*now) > MAX_LEAN:
            return  # no attitude, or not a hover: the lean as it was stands (and goes stale by the clock below)
        if self.lean is None or self._last_s is None or now_s - self._last_s > RESTART_AFTER_S or now_s < self._last_s:
            self.lean = now
        else:
            share = min(1.0, (now_s - self._last_s) / LEAN_TAU_S)
            self.lean = (self.lean[0] + share * (now[0] - self.lean[0]), self.lean[1] + share * (now[1] - self.lean[1]))
        self._last_s = now_s

    def fresh(self, now_s: float) -> bool:
        """Whether there is a lean of the last second."""
        return self.lean is not None and self._last_s is not None and 0.0 <= now_s - self._last_s <= RESTART_AFTER_S

    def shift(self, height_m: Optional[float], now_s: float) -> Tuple[float, float]:
        """The aim point's shift for a release from `height_m` above the ground: upwind, the predicted drift undone. (0, 0) with no
        lean of the last second, no height, or K at 0."""
        if self.lean is None or self._last_s is None or now_s - self._last_s > RESTART_AFTER_S or height_m is None or not math.isfinite(height_m) or height_m <= 0.0:
            return 0.0, 0.0
        drift = predicted_drift(self.settings.k, float(height_m), self.lean)
        shift = (-drift[0], -drift[1])
        size = math.hypot(*shift)
        if size > self.settings.max_shift_m > 0.0:
            shift = (shift[0] * self.settings.max_shift_m / size, shift[1] * self.settings.max_shift_m / size)
        elif self.settings.max_shift_m <= 0.0:
            shift = (0.0, 0.0)
        return shift


# --- learning K from drops -----------------------------------------------------------------------------------


@dataclass(frozen=True)
class Drop:
    height_m: float  # the release height above the ground
    lean: Tuple[float, float]  # the vehicle's low-passed lean at the release (east, north)
    drift_m: Tuple[float, float]  # where the package came to rest less where it was let go (east, north)
    name: str = ""


@dataclass(frozen=True)
class Fit:
    k: float
    drops: int
    residual_rms_m: float  # what the model leaves of the drifts, with this K
    unmodelled_rms_m: float  # the drifts themselves: what K = 0 leaves
    lean_max: float  # the largest lean among the drops: a K fitted in still air is fitted to nothing


MIN_DROPS = 3
MIN_LEAN_FOR_FIT = 0.03  # under two degrees of lean in every drop there was no wind to learn from


def fit(drops: Sequence[Drop]) -> Fit:
    """The K that best explains the drops' drifts by height times lean (least squares through zero), and what it leaves."""
    usable: List[Drop] = [drop for drop in drops if drop.height_m > 0.0 and all(math.isfinite(value) for value in (*drop.lean, *drop.drift_m, drop.height_m))]
    if len(usable) < MIN_DROPS:
        raise ValueError(f"{len(usable)} usable drop(s): at least {MIN_DROPS} are needed to fit the release offset")
    lean_max = max(math.hypot(*drop.lean) for drop in usable)
    if lean_max < MIN_LEAN_FOR_FIT:
        raise ValueError(f"the largest lean in these drops is {lean_max:.3f} (under {MIN_LEAN_FOR_FIT:g}): they were made in still air, and say nothing of the wind's push")
    # drift = -K x (h x lean): K = -sum(x . d) / sum(x . x), with x = h x lean
    xd = sum(drop.height_m * (drop.lean[0] * drop.drift_m[0] + drop.lean[1] * drop.drift_m[1]) for drop in usable)
    xx = sum(drop.height_m**2 * (drop.lean[0] ** 2 + drop.lean[1] ** 2) for drop in usable)
    k = max(0.0, min(K_MAX, -xd / xx))  # (a negative K would be a package that flies upwind: the drops do not show a drift with the wind)
    left = [math.hypot(drop.drift_m[0] - predicted_drift(k, drop.height_m, drop.lean)[0], drop.drift_m[1] - predicted_drift(k, drop.height_m, drop.lean)[1]) for drop in usable]
    raw = [math.hypot(*drop.drift_m) for drop in usable]
    rms = lambda values: math.sqrt(sum(value * value for value in values) / len(values))  # noqa: E731
    return Fit(k, len(usable), rms(left), rms(raw), lean_max)
