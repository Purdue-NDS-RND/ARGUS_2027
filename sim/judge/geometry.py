"""Flat-ground geometry for the judges: polygons and distances. No ROS."""

from __future__ import annotations

import math
from typing import Sequence

Point = Sequence[float]


def point_in_polygon(e: float, n: float, polygon: Sequence[Point]) -> bool:
    """Whether (e, n) is inside a simple polygon given as (e, n) corners."""

    inside = False
    count = len(polygon)
    for i in range(count):
        e1, n1 = polygon[i][0], polygon[i][1]
        e2, n2 = polygon[(i + 1) % count][0], polygon[(i + 1) % count][1]
        if (n1 > n) != (n2 > n):
            crossing = e1 + (n - n1) * (e2 - e1) / (n2 - n1)
            if e < crossing:
                inside = not inside
    return inside


def distance_to_segment(e: float, n: float, a: Point, b: Point) -> float:
    ae, an, be, bn = a[0], a[1], b[0], b[1]
    de, dn = be - ae, bn - an
    length_sq = de * de + dn * dn
    if length_sq == 0.0:
        return math.hypot(e - ae, n - an)
    f = max(0.0, min(1.0, ((e - ae) * de + (n - an) * dn) / length_sq))
    return math.hypot(e - (ae + f * de), n - (an + f * dn))


def distance_outside_polygon(e: float, n: float, polygon: Sequence[Point]) -> float:
    """How far (e, n) is outside the polygon, in metres; 0 inside it."""

    if point_in_polygon(e, n, polygon):
        return 0.0
    count = len(polygon)
    return min(distance_to_segment(e, n, polygon[i], polygon[(i + 1) % count]) for i in range(count))
