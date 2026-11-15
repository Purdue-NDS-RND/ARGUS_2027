#!/usr/bin/env python3
"""The competition-site file: load it, check it, and derive what the sim needs.

One YAML per site in sim/sites/ holds the field's coordinates. From it come

  - the sim world (the base world with its origin moved to the site),
  - SITL's home (start.sh reads it from that world),
  - the geofence plan that sim/fence_upload.py sends to the flight controller,
  - the site part of the judge's truth.json.

No ROS, no Gazebo. Metres are east and north of the site origin, worked out
with ArduPilot's own flat-earth formula (sim/target_placement.py), so a point
is where the autopilot thinks it is.

Usage:
  python3 sim/site_file.py check <site.yaml>
  python3 sim/site_file.py world <site.yaml> --out <world.sdf> [--base <world.sdf>]
  python3 sim/site_file.py truth <site.yaml>
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM_DIR))

from judge.geometry import distance_to_segment, point_in_polygon  # noqa: E402
from target_placement import GeoOrigin, offset_ne_m  # noqa: E402

BASE_WORLD = SIM_DIR / "worlds" / "argus_runway.sdf"

MAX_ALTITUDE_LIMIT_M = 121.92  # 400 ft: no site may allow more
MIN_TAKEOFF_CLEARANCE_M = 10.0  # the takeoff point must be this far inside the fence
MAX_FENCE_MARGIN_M = 10.0
# ArduPilot FENCE_ACTION values that stop the flight. 0 (report only) is refused.
FENCE_ACTIONS = {1: "RTL or land", 2: "land", 3: "SmartRTL, RTL or land", 4: "brake or land", 5: "SmartRTL or land"}

# MAVLink constants for the fence plan (common.xml), so no pymavlink is needed here.
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001
MAV_FRAME_GLOBAL = 0
# FENCE_TYPE bits: 0 maximum height, 1 circle, 2 polygon. Height and polygon, no circle.
FENCE_TYPE_ALT_MAX_AND_POLYGON = 0b101

KEYS = {
    "name", "verified", "origin", "terrain_elevation_m", "runway", "takeoff_point",
    "compass_rose", "flight_boundary", "max_altitude_agl_m", "survey_area", "fence",
}  # fmt: skip


class SiteError(ValueError):
    pass


@dataclass(frozen=True)
class Runway:
    centre: tuple[float, float]  # east, north
    heading_deg: float
    length_m: float
    width_m: float


@dataclass(frozen=True)
class Site:
    name: str
    verified: bool
    origin: GeoOrigin
    runway: Runway
    takeoff: tuple[float, float]
    compass_rose: tuple[float, float]
    boundary: tuple[tuple[float, float], ...]  # east, north
    boundary_geo: tuple[tuple[float, float], ...]  # latitude, longitude
    max_altitude_agl_m: float
    survey_area: tuple[tuple[float, float], ...]
    fence_action: int
    fence_margin_m: float

    def to_enu(self, latitude_deg: float, longitude_deg: float) -> tuple[float, float]:
        north, east = offset_ne_m(self.origin.latitude_deg, self.origin.longitude_deg, latitude_deg, longitude_deg)
        return east, north


@dataclass(frozen=True)
class FencePlan:
    items: list[dict]  # one mission item per polygon vertex
    params: dict[str, float]  # FENCE_* parameters; FENCE_ENABLE is set last


def _number(data: dict, key: str, where: str) -> float:
    if key not in data:
        raise SiteError(f"{where}: missing '{key}'")
    try:
        return float(data[key])
    except (TypeError, ValueError):
        raise SiteError(f"{where}: '{key}' must be a number, got {data[key]!r}") from None


def _geo(data, where: str) -> tuple[float, float]:
    if not isinstance(data, dict):
        raise SiteError(f"{where}: expected latitude_deg and longitude_deg")
    lat, lon = _number(data, "latitude_deg", where), _number(data, "longitude_deg", where)
    if not -90.0 <= lat <= 90.0:
        raise SiteError(f"{where}: latitude {lat} is not between -90 and 90")
    if not -180.0 <= lon <= 180.0:
        raise SiteError(f"{where}: longitude {lon} is not between -180 and 180")
    return lat, lon


def _crosses_itself(polygon) -> bool:
    def side(a, b, c) -> float:
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    count = len(polygon)
    for i in range(count):
        a, b = polygon[i], polygon[(i + 1) % count]
        for j in range(i + 1, count):
            if j == i or (j + 1) % count == i or (i + 1) % count == j:
                continue  # edges that share a corner
            c, d = polygon[j], polygon[(j + 1) % count]
            if side(a, b, c) * side(a, b, d) < 0 and side(c, d, a) * side(c, d, b) < 0:
                return True
    return False


def _distance_to_boundary(point, polygon) -> float:
    count = len(polygon)
    return min(distance_to_segment(point[0], point[1], polygon[i], polygon[(i + 1) % count]) for i in range(count))


def parse_site(data, where: str = "site") -> Site:
    if not isinstance(data, dict):
        raise SiteError(f"{where}: expected a mapping")
    unknown = sorted(set(data) - KEYS)
    if unknown:
        raise SiteError(f"{where}: unknown key(s) {', '.join(unknown)}")
    for key in sorted(KEYS - {"verified"}):
        if key not in data:
            raise SiteError(f"{where}: missing '{key}'")

    lat0, lon0 = _geo(data["origin"], f"{where}: origin")
    origin = GeoOrigin(lat0, lon0, _number(data, "terrain_elevation_m", where), 0.0)

    def enu(value, name: str) -> tuple[float, float]:
        lat, lon = _geo(value, f"{where}: {name}")
        north, east = offset_ne_m(lat0, lon0, lat, lon)
        return east, north

    def polygon(name: str) -> tuple[tuple[tuple[float, float], ...], tuple[tuple[float, float], ...]]:
        vertices = data[name]
        if not isinstance(vertices, list) or len(vertices) < 3:
            raise SiteError(f"{where}: {name} needs at least 3 corners")
        geo = tuple(_geo(vertex, f"{where}: {name}[{i}]") for i, vertex in enumerate(vertices))
        metres = tuple(enu(vertex, f"{name}[{i}]") for i, vertex in enumerate(vertices))
        if _crosses_itself(metres):
            raise SiteError(f"{where}: {name} crosses itself; list the corners in order around the area")
        return metres, geo

    boundary, boundary_geo = polygon("flight_boundary")
    survey_area, _ = polygon("survey_area")

    runway_data = data["runway"]
    if not isinstance(runway_data, dict):
        raise SiteError(f"{where}: runway: expected a mapping")
    runway = Runway(
        centre=enu(runway_data.get("centre"), "runway.centre"),
        heading_deg=_number(runway_data, "heading_deg", f"{where}: runway"),
        length_m=_number(runway_data, "length_m", f"{where}: runway"),
        width_m=_number(runway_data, "width_m", f"{where}: runway"),
    )

    takeoff = enu(data["takeoff_point"], "takeoff_point")
    compass_rose = enu(data["compass_rose"], "compass_rose")

    max_altitude = _number(data, "max_altitude_agl_m", where)
    if max_altitude <= 0.0:
        raise SiteError(f"{where}: max_altitude_agl_m must be above 0, got {max_altitude}")
    if max_altitude > MAX_ALTITUDE_LIMIT_M:
        raise SiteError(f"{where}: max_altitude_agl_m {max_altitude} is above 400 ft ({MAX_ALTITUDE_LIMIT_M} m)")

    fence = data["fence"]
    if not isinstance(fence, dict):
        raise SiteError(f"{where}: fence: expected a mapping")
    action = fence.get("action")
    if action not in FENCE_ACTIONS:
        raise SiteError(f"{where}: fence.action must be one of {sorted(FENCE_ACTIONS)} (a fence that only reports is not allowed), got {action!r}")
    margin = _number(fence, "margin_m", f"{where}: fence")
    if not 0.0 < margin <= MAX_FENCE_MARGIN_M:
        raise SiteError(f"{where}: fence margin {margin} m must be above 0 and at most {MAX_FENCE_MARGIN_M} m")

    for name, point in (("takeoff_point", takeoff), ("compass_rose", compass_rose)):
        if not point_in_polygon(point[0], point[1], boundary):
            raise SiteError(f"{where}: {name} is outside the flight boundary")
    clearance = _distance_to_boundary(takeoff, boundary)
    needed = max(MIN_TAKEOFF_CLEARANCE_M, 2.0 * margin)
    if clearance < needed:
        raise SiteError(f"{where}: takeoff_point is {clearance:.1f} m from the flight boundary; it needs at least {needed:.1f} m")
    for i, corner in enumerate(survey_area):
        if not point_in_polygon(corner[0], corner[1], boundary):
            raise SiteError(f"{where}: survey_area[{i}] is outside the flight boundary")

    return Site(
        name=str(data["name"]),
        verified=bool(data.get("verified", False)),
        origin=origin,
        runway=runway,
        takeoff=takeoff,
        compass_rose=compass_rose,
        boundary=boundary,
        boundary_geo=boundary_geo,
        max_altitude_agl_m=max_altitude,
        survey_area=survey_area,
        fence_action=int(action),
        fence_margin_m=margin,
    )


def load_site(path) -> Site:
    import yaml

    path = Path(path)
    try:
        data = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as error:
        raise SiteError(f"{path}: {error}") from error
    return parse_site(data, str(path))


def truth_block(site: Site) -> dict:
    """The site's part of a run's truth.json (see sim/judge/README.md)."""

    def point(value) -> dict:
        return {"e": value[0], "n": value[1]}

    return {
        "site": site.name,
        "origin": {
            "latitude_deg": site.origin.latitude_deg,
            "longitude_deg": site.origin.longitude_deg,
            "elevation_m": site.origin.elevation_m,
        },
        "takeoff": point(site.takeoff),
        "fence": {"polygon": [list(corner) for corner in site.boundary], "max_height_m": site.max_altitude_agl_m},
        "survey_area": [list(corner) for corner in site.survey_area],
        "return_points": {"takeoff": point(site.takeoff), "compass_rose": point(site.compass_rose)},
    }


def fence_plan(site: Site) -> FencePlan:
    """The fence as ArduPilot takes it: one inclusion polygon, and the FENCE_* parameters."""

    count = len(site.boundary_geo)
    items = [
        {
            "seq": seq,
            "command": MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            "frame": MAV_FRAME_GLOBAL,
            "param1": count,
            "lat_e7": int(round(lat * 1.0e7)),
            "lon_e7": int(round(lon * 1.0e7)),
        }
        for seq, (lat, lon) in enumerate(site.boundary_geo)
    ]
    params = {
        "FENCE_TYPE": FENCE_TYPE_ALT_MAX_AND_POLYGON,
        "FENCE_ALT_MAX": site.max_altitude_agl_m,
        "FENCE_MARGIN": site.fence_margin_m,
        "FENCE_ACTION": site.fence_action,
        "FENCE_ENABLE": 1,  # last, so the fence is never on without its polygon and limits
    }
    return FencePlan(items=items, params=params)


_SPHERICAL = re.compile(r"<spherical_coordinates>.*?</spherical_coordinates>", re.DOTALL)
_WORLD_NAME = re.compile(r'<world name="[^"]*">')
_RUNWAY_POSE = re.compile(r'(<uri>model://runway</uri>\s*<pose degrees="true">)[^<]*(</pose>)')


def site_world(base_sdf: str, site: Site) -> str:
    """The base world moved to the site: its name, origin and runway strip.

    Everything else (physics, plugins, the vehicle at the origin) is the base
    world's, so a site world flies the same vehicle the 2026 world does. The
    runway model also carries the ground plane, so it stays; only its strip is
    laid along the site's runway (the model's strip runs along its +y axis).
    """

    spherical = (
        "<spherical_coordinates>\n"
        f"      <latitude_deg>{site.origin.latitude_deg!r}</latitude_deg>\n"
        f"      <longitude_deg>{site.origin.longitude_deg!r}</longitude_deg>\n"
        f"      <elevation>{site.origin.elevation_m!r}</elevation>\n"
        "      <heading_deg>0</heading_deg>\n"
        "      <surface_model>EARTH_WGS84</surface_model>\n"
        "    </spherical_coordinates>"
    )
    runway_pose = f"{site.runway.centre[0]!r} {site.runway.centre[1]!r} 0 0 0 {-site.runway.heading_deg!r}"
    text = base_sdf
    for pattern, replacement, what in (
        (_WORLD_NAME, f'<world name="{site.name}">', "<world name=...>"),
        (_SPHERICAL, spherical, "<spherical_coordinates>"),
        (_RUNWAY_POSE, rf"\g<1>{runway_pose}\g<2>", "the model://runway include"),
    ):
        text, found = pattern.subn(replacement, text, count=1)
        if not found:
            raise SiteError(f"the base world has no {what}")
    return text


def summary(site: Site) -> str:
    east = [corner[0] for corner in site.boundary]
    north = [corner[1] for corner in site.boundary]
    lines = [
        f"site {site.name}: {'verified' if site.verified else 'NOT VERIFIED (placeholder coordinates)'}",
        f"  origin {site.origin.latitude_deg:.7f}, {site.origin.longitude_deg:.7f}, ground {site.origin.elevation_m:.1f} m AMSL",
        f"  flight boundary: {len(site.boundary)} corners, {min(east):.0f} to {max(east):.0f} m east, {min(north):.0f} to {max(north):.0f} m north",
        f"  maximum height {site.max_altitude_agl_m:.1f} m; fence action {site.fence_action} ({FENCE_ACTIONS[site.fence_action]}), margin {site.fence_margin_m:.1f} m",
        f"  takeoff point {site.takeoff[0]:.1f} E, {site.takeoff[1]:.1f} N; compass rose {site.compass_rose[0]:.1f} E, {site.compass_rose[1]:.1f} N",
        f"  runway {site.runway.length_m:.0f} x {site.runway.width_m:.0f} m, heading {site.runway.heading_deg:.0f} deg",
    ]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check a site file and derive the sim's files from it.")
    parser.add_argument("action", choices=("check", "world", "truth"))
    parser.add_argument("site", type=Path)
    parser.add_argument("--out", type=Path, help="world: where to write the world file")
    parser.add_argument("--base", type=Path, default=BASE_WORLD, help="world: the world to start from")
    args = parser.parse_args(argv)
    try:
        site = load_site(args.site)
        if args.action == "check":
            print(summary(site))
        elif args.action == "truth":
            print(json.dumps(truth_block(site), indent=2))
        else:
            if args.out is None:
                parser.error("world needs --out")
            text = site_world(args.base.read_text(), site)
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.out.write_text(text)
            print(f"wrote {args.out} (world '{site.name}')")
    except (SiteError, OSError) as error:
        print(f"site: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
