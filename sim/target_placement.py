"""Where to put the delivery target in the Gazebo world. No ROS, no Gazebo.

The sim world is ENU: +x east, +y north, origin at the world's
<spherical_coordinates>. sim/container/start.sh starts SITL with its home at
that same origin, so a GPS point maps to world x/y by its north/east offset
from home. The offset uses ArduPilot's own flat-earth formula
(Location::get_distance_NE), the one the autopilot uses to fly to the
mission's target_latitude/target_longitude, so the target lands where the
autopilot will look for it.
"""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass

# ArduPilot AP_Common/Location.h: metres per 1e-7 degree of latitude.
LOCATION_SCALING_FACTOR = 0.011131884502145034


@dataclass(frozen=True)
class GeoOrigin:
    """A world's spherical_coordinates block."""

    latitude_deg: float
    longitude_deg: float
    elevation_m: float
    heading_deg: float


def _to_e7(deg: float) -> int:
    return int(round(float(deg) * 1.0e7))


def longitude_scale(lat_e7: int) -> float:
    """ArduPilot's longitude_scale(): cos(lat), floored at 0.01."""

    scale = math.cos(math.radians(lat_e7 * 1.0e-7))
    return max(scale, 0.01)


def offset_ne_m(
    home_lat_deg: float,
    home_lon_deg: float,
    lat_deg: float,
    lon_deg: float,
) -> tuple[float, float]:
    """North and east metres from home to a point, as ArduPilot computes it."""

    home_lat = _to_e7(home_lat_deg)
    home_lon = _to_e7(home_lon_deg)
    lat = _to_e7(lat_deg)
    lon = _to_e7(lon_deg)

    d_lon = lon - home_lon
    if d_lon > 1800000000:
        d_lon -= 3600000000
    elif d_lon < -1800000000:
        d_lon += 3600000000

    north = (lat - home_lat) * LOCATION_SCALING_FACTOR
    # int(... / 2) truncates toward zero like ArduPilot's int32 arithmetic.
    east = d_lon * LOCATION_SCALING_FACTOR * longitude_scale(int((home_lat + lat) / 2))
    return north, east


def world_xy_for(origin: GeoOrigin, lat_deg: float, lon_deg: float) -> tuple[float, float]:
    """Gazebo world x (east) and y (north) for a GPS point."""

    if abs(origin.heading_deg) > 1.0e-9:
        raise ValueError(
            "world heading_deg must be 0 so that +x is east and +y is north; "
            f"got {origin.heading_deg}"
        )
    north, east = offset_ne_m(origin.latitude_deg, origin.longitude_deg, lat_deg, lon_deg)
    return east, north


def read_world(sdf_text: str) -> tuple[str, GeoOrigin]:
    """The world name and its spherical_coordinates from an SDF world file."""

    root = ET.fromstring(sdf_text)
    world = root.find("world")
    if world is None:
        raise ValueError("no <world> element")
    name = world.get("name")
    if not name:
        raise ValueError("<world> has no name")
    sc = world.find("spherical_coordinates")
    if sc is None:
        raise ValueError(f"world '{name}' has no <spherical_coordinates>")

    def value(tag: str, default: float | None = None) -> float:
        element = sc.find(tag)
        if element is None or element.text is None:
            if default is None:
                raise ValueError(f"<spherical_coordinates> has no <{tag}>")
            return default
        return float(element.text)

    origin = GeoOrigin(
        latitude_deg=value("latitude_deg"),
        longitude_deg=value("longitude_deg"),
        elevation_m=value("elevation", 0.0),
        heading_deg=value("heading_deg", 0.0),
    )
    return name, origin


_CREATE_SERVICE = re.compile(r"^/world/([^/\s]+)/create$")


def worlds_from_service_list(service_list: str) -> list[str]:
    """World names that accept spawns, from the output of `gz service -l`."""

    names = []
    for line in service_list.splitlines():
        match = _CREATE_SERVICE.match(line.strip())
        if match and match.group(1) not in names:
            names.append(match.group(1))
    return names


def sitl_location_arg(origin: GeoOrigin) -> str:
    """The --custom-location value that puts SITL home at the world origin.

    The fourth field is SITL's start heading. With the JSON backend Gazebo
    supplies the attitude, so it only needs to be a valid number.
    """

    return (
        f"{origin.latitude_deg},{origin.longitude_deg},"
        f"{origin.elevation_m},{origin.heading_deg}"
    )
