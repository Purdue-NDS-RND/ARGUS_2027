"""A track's reported position: the weighted median of its sightings (LOC-9). No ROS.

Behind the node flag `features.loc_median_position`. With it off a track's
reported position is the tracker's filtered one, as LOC-7 built it.

The filter's position is a weighted mean: every sighting moves it, and a
projection made with a wrong attitude or a wrong height moves it for good.
The weighted median, taken east and north separately, is where half the
sightings' weight lies either side: the odd bad projection does not move
it. A sighting's weight is the tracker's (one over the trace of the
sighting's noise), so a sighting from nearly overhead counts for more than
one from the edge of the picture.

What the median cannot help: an error common to every sighting. The
vehicle's own position is one (about 1 to 1.5 m with the u-blox M10, against
0.2 to 0.3 m of projection error from 30 ft), and it dominates the error of
a reported marker.
"""

from __future__ import annotations

import math
from typing import Any, Optional, Sequence, Tuple


def weighted_median(values: Sequence[float], weights: Sequence[float]) -> float:
    """The value with half the weight at or below it. With exactly half below, midway to the next value."""
    if len(values) != len(weights):
        raise ValueError("the weighted median needs a weight for every value")
    pairs = sorted(
        (float(value), float(weight))
        for value, weight in zip(values, weights)
        if math.isfinite(float(value)) and math.isfinite(float(weight)) and float(weight) > 0.0
    )
    if not pairs:
        raise ValueError("the weighted median needs at least one value with a weight above zero")
    half = sum(weight for _value, weight in pairs) / 2.0
    below = 0.0
    for index, (value, weight) in enumerate(pairs):
        below += weight
        if below > half and not math.isclose(below, half, rel_tol=1.0e-12):
            return value
        if math.isclose(below, half, rel_tol=1.0e-12) and index + 1 < len(pairs):
            return (value + pairs[index + 1][0]) / 2.0
    return pairs[-1][0]


def final_position(sightings: Sequence[Any]) -> Optional[Tuple[float, float]]:
    """(east, north) of a track from its sightings (each with east_m, north_m, weight), or None with none usable."""
    try:
        return (
            weighted_median([s.east_m for s in sightings], [s.weight for s in sightings]),
            weighted_median([s.north_m for s in sightings], [s.weight for s in sightings]),
        )
    except ValueError:
        return None
