"""Where the true position left the fence. No ROS."""

from __future__ import annotations

from .geometry import distance_outside_polygon
from .record import VEHICLE, Record


def fence_breaches(record: Record, fence: dict | None, t0: float, t1: float) -> list[dict]:
    """Spans in which the true position was outside the fence, with the worst excursion."""

    if not fence:
        return []
    polygon = fence.get("polygon")
    max_height = fence.get("max_height_m")
    breaches: list[dict] = []
    open_spans: dict[str, dict] = {}
    for pose in record.track(VEHICLE, t0, t1):
        excursions = {
            "lateral": distance_outside_polygon(pose.e, pose.n, polygon) if polygon else 0.0,
            "height": max(0.0, pose.u - max_height) if max_height is not None else 0.0,
        }
        for kind, excursion in excursions.items():
            span = open_spans.get(kind)
            if excursion > 0.0:
                if span is None:
                    span = {"kind": kind, "t_start": pose.t, "t_end": pose.t, "max_excursion_m": 0.0}
                    open_spans[kind] = span
                    breaches.append(span)
                span["t_end"] = pose.t
                span["max_excursion_m"] = max(span["max_excursion_m"], excursion)
            elif span is not None:
                del open_spans[kind]
    return breaches
