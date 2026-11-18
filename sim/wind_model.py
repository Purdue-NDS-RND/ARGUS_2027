"""The sim's wind (SIM-03): what the air is doing, and the force it puts on a body. No ROS, no Gazebo.

The wind is a mean speed from a direction, plus gusts along the mean and
across it and vertically, each an Ornstein-Uhlenbeck process (a random
speed that keeps being pulled back to zero over a few seconds), plus an
optional single discrete gust: a (1 - cos) rise and fall, once in the
flight.

The force on a body is our own drag law, applied to the body by Gazebo's
ApplyLinkWrench system (sim/wind.py):

    F = 1/2 rho CdA |v_rel| v_rel,      v_rel = wind - the body's own velocity

so a body moving with the wind feels none, a hovering one is pushed
downwind, and a falling payload drifts. CdA is the drag area: 0.08 m2 for
the Iris until the hover test on the real airframe gives ours (DAT-10).

Directions are compass bearings the wind comes FROM, in degrees: 0 is a
north wind, which blows toward the south. Vectors are east, north, up.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, Optional, Tuple

AIR_DENSITY_KG_M3 = 1.225
IRIS_CDA_M2 = 0.08
GRAVITY_MPS2 = 9.80665
UPDATE_HZ = 20.0  # how often the force is recomputed from the body's true velocity
GUST_TAU_S = (2.0, 5.0)  # the gusts' correlation time is drawn from this range, per flight
CROSS_GUST_SHARE = 0.5  # gusts across the mean wind are this fraction of the gusts along it
VERTICAL_GUST_SHARE = 0.3  # and vertical gusts this fraction
DISCRETE_GUST_AT_S = (20.0, 60.0)  # the discrete gust starts at a time drawn from this range after the wind starts
DISCRETE_GUST_LENGTH_S = 4.0

Vector = Tuple[float, float, float]


def toward(direction_from_deg: float) -> Tuple[float, float]:
    """The unit vector (east, north) the wind blows toward, for the bearing it comes from."""
    bearing = math.radians(float(direction_from_deg))
    return (-math.sin(bearing), -math.cos(bearing))


def drag_force(wind: Vector, velocity: Vector, cda_m2: float, air_density: float = AIR_DENSITY_KG_M3) -> Vector:
    """F = 1/2 rho CdA |v_rel| v_rel, in newtons, for a body moving at `velocity` in air moving at `wind`."""
    relative = tuple(w - v for w, v in zip(wind, velocity))
    speed = math.sqrt(sum(component * component for component in relative))
    scale = 0.5 * air_density * float(cda_m2) * speed
    return (scale * relative[0], scale * relative[1], scale * relative[2])


def hover_tilt_deg(wind_speed_mps: float, mass_kg: float, cda_m2: float = IRIS_CDA_M2) -> float:
    """The tilt a hovering vehicle of this mass holds against a steady wind: atan(F / mg)."""
    force = 0.5 * AIR_DENSITY_KG_M3 * cda_m2 * wind_speed_mps * wind_speed_mps
    return math.degrees(math.atan2(force, mass_kg * GRAVITY_MPS2))


class OrnsteinUhlenbeck:
    """A random speed with standard deviation `sigma` that forgets itself over `tau_s` seconds."""

    def __init__(self, sigma: float, tau_s: float, generator: random.Random) -> None:
        self.sigma, self.tau_s, self._random = float(sigma), float(tau_s), generator
        self.value = 0.0

    def step(self, dt_s: float) -> float:
        if self.sigma <= 0.0 or dt_s <= 0.0:
            return self.value
        keep = math.exp(-dt_s / self.tau_s)
        # The exact update for any step: the variance stays sigma^2 whatever dt is.
        self.value = self.value * keep + self.sigma * math.sqrt(1.0 - keep * keep) * self._random.gauss(0.0, 1.0)
        return self.value


@dataclass(frozen=True)
class WindSettings:
    mean_mps: float = 0.0
    gust_mps: float = 0.0  # standard deviation of the gusts along the mean wind
    discrete_gust_mps: float = 0.0  # peak of the one discrete gust; 0 for none
    direction_deg: float = 0.0  # the bearing the wind comes from

    @classmethod
    def from_manifest(cls, wind: Optional[dict]) -> "WindSettings":
        wind = wind or {}
        return cls(
            mean_mps=float(wind.get("mean_mps", 0.0)),
            gust_mps=float(wind.get("gust_mps", 0.0)),
            discrete_gust_mps=float(wind.get("discrete_gust_mps", 0.0)),
            direction_deg=float(wind.get("direction_deg", 0.0)),
        )

    @property
    def calm(self) -> bool:
        return self.mean_mps == 0.0 and self.gust_mps == 0.0 and self.discrete_gust_mps == 0.0


class Wind:
    """The wind over a flight: `at(t)` gives its velocity (east, north, up) at t seconds, for times that do not go back."""

    def __init__(self, settings: WindSettings, seed: int = 1) -> None:
        self.settings = settings
        generator = random.Random(f"wind-{int(seed)}")
        self.tau_s = generator.uniform(*GUST_TAU_S)
        self.discrete_at_s = generator.uniform(*DISCRETE_GUST_AT_S)
        self._along = OrnsteinUhlenbeck(settings.gust_mps, self.tau_s, generator)
        self._across = OrnsteinUhlenbeck(settings.gust_mps * CROSS_GUST_SHARE, self.tau_s, generator)
        self._vertical = OrnsteinUhlenbeck(settings.gust_mps * VERTICAL_GUST_SHARE, self.tau_s, generator)
        self._last_s: Optional[float] = None

    def discrete_gust(self, t_s: float) -> float:
        """The discrete gust's speed at t: a (1 - cos) pulse of the set peak, once."""
        since = t_s - self.discrete_at_s
        if self.settings.discrete_gust_mps <= 0.0 or not 0.0 <= since <= DISCRETE_GUST_LENGTH_S:
            return 0.0
        return 0.5 * self.settings.discrete_gust_mps * (1.0 - math.cos(2.0 * math.pi * since / DISCRETE_GUST_LENGTH_S))

    def at(self, t_s: float) -> Vector:
        dt = 0.0 if self._last_s is None else max(0.0, t_s - self._last_s)
        self._last_s = t_s if self._last_s is None else max(self._last_s, t_s)
        along = self.settings.mean_mps + self._along.step(dt) + self.discrete_gust(t_s)
        across = self._across.step(dt)
        up = self._vertical.step(dt)
        east, north = toward(self.settings.direction_deg)
        # Across the wind: the mean direction turned a quarter turn to the left.
        return (along * east - across * north, along * north + across * east, up)


# --- giving a changing force to Gazebo ---------------------------------------------------
#
# Gazebo's ApplyLinkWrench system only adds: a second persistent wrench for the
# same body is applied as well as the first, and the only way to take one away
# is to clear them all. So a force that changes at 20 Hz is sent as its change
# since the last update, and every few seconds the pile of changes is cleared
# and the whole force sent once, after the clear has had time to be taken (an
# add that arrived with the clear would be cleared with the rest).

RESET_EVERY = 100  # updates between clearing the pile: 5 s at 20 Hz
SETTLE_S = 0.010  # sim time between the clear and the whole force after it: ten physics steps


class ForcePlan:
    """What to publish for one body so that the force Gazebo applies is the one wanted."""

    def __init__(self, reset_every: int = RESET_EVERY, settle_s: float = SETTLE_S) -> None:
        self._reset_every = int(reset_every)
        self._settle_s = float(settle_s)
        self.applied: Vector = (0.0, 0.0, 0.0)  # the sum of what Gazebo holds for this body
        self.wanted: Vector = (0.0, 0.0, 0.0)  # the force last asked for
        self._updates = 0
        self._cleared_at_s: Optional[float] = None

    @property
    def settling(self) -> bool:
        """True between a clear and the whole force after it: ask again soon, not at the next 20 Hz update."""
        return self._cleared_at_s is not None

    def update(self, wanted: Vector, t_s: float) -> List[Tuple[str, Vector]]:
        """The messages for this update: ("add", force) and ("clear", zero), in the order to publish them."""
        self.wanted = wanted
        if self._cleared_at_s is not None:
            if t_s - self._cleared_at_s < self._settle_s:
                return []  # the clear has not surely been taken yet: an add now could be cleared with the rest
            self._cleared_at_s = None
            self.applied = wanted
            self._updates = 0
            return [("add", wanted)]
        self._updates += 1
        if self._updates >= self._reset_every:
            self._cleared_at_s = t_s
            self.applied = (0.0, 0.0, 0.0)
            return [("clear", (0.0, 0.0, 0.0))]
        change = tuple(w - a for w, a in zip(wanted, self.applied))
        self.applied = wanted
        return [("add", change)] if any(abs(component) > 1.0e-9 for component in change) else []
