#!/usr/bin/env python3
"""Field setup (OPS-01): the day's coordinates, entered once, checked, and turned into every file that needs them.

  python3 tools/field_setup.py field/<day>.yaml --check            # check only, write nothing
  python3 tools/field_setup.py field/<day>.yaml                    # check, then write field/<name>/
  python3 tools/field_setup.py field/<day>.yaml --for-flight       # as above, and refuse a day file that is not verified

The day file is the one place a coordinate is typed (field/example_day.yaml
shows it): the site (takeoff point, flight boundary, survey area, fence: what
sim/site_file.py takes), the targets (drop, delivery, the recovery's pickup),
the waypoints, where the recovery returns to (takeoff or compass_rose), and
the compass bearing of the geofence demonstration's run (OPS-02). From it come, under field/<name>/:

  site.yaml                               the site file: the sim flies it, sim/fence_upload.py uploads its fence
  fence.plan, fence.param                 the same fence for a ground station (QGroundControl plan; FENCE_* values)
  waypoints/competition_waypoints.yaml    the waypoint and time-trial route
  sequences/<name>.yaml                   each mission sequence, line for line, with only its coordinates changed
  summary.txt                             every point's distance and bearing from the takeoff point and its distance
                                          from the fence, to hold against the organisers' map

The checks, before anything is written: the site's own (a fence that closes,
does not cross itself and acts; the takeoff point inside it; at most 400 ft);
every target, waypoint and survey corner inside the flight boundary and at
least safety_limits.GEOFENCE_MARGIN_M from it; nothing more than 2 km from the
takeoff point (a swapped latitude and longitude, a dropped digit); every leg
of the route, in the order given, as far from the boundary as its ends must
be; waypoint heights above zero and under the site's ceiling; names that are
unique.

A generated sequence is its source with only the coordinate lines changed,
and one added where a mission can read the day's flight boundary and its
source has no line for it (the route's missions, the recovery): every gain,
height, dwell and flag is the committed file's. The tool checks
that of its own output and refuses to write a file that differs anywhere
else. Fly a generated sequence by its path: tools/go.py --sequence
field/<name>/sequences/package_delivery_live.yaml.

No ROS imports. Exit code 0, 1 when a check fails (each is printed), 2 when
the day file cannot be read.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / "sim", ROOT / "src" / "drone_mission_core", ROOT / "tools"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import site_file  # noqa: E402  (sim/site_file.py: no ROS)
import sim_sequence_copies as lines  # noqa: E402  (the line-for-line editing of a sequence file)
from drone_mission_core import safety_limits  # noqa: E402

CONFIG = ROOT / "src" / "drone_mission_demo" / "config"
FIELD = ROOT / "field"
DAY_KEYS = {"name", "date", "verified", "site", "targets", "waypoints", "recovery_return", "fence_demo"}
FENCE_DEMO_OVERSHOOT_M = 10.0  # the geofence demonstration's run ends this far past the boundary if the fence has done nothing (OPS-02)
FENCE_DEMO_MAX_OBLIQUE_DEG = 15.0  # the run meets the boundary no further than this from square: the avoidance slides a vehicle along a fence it meets at an angle
FENCE_DEMO_MAX_RUN_M = 500.0  # fence_demo.py's own limit on a run
RECOVERY_RETURNS = ("takeoff", "compass_rose")  # where the recovery takes the bucket: the rules allow either
LEG_STEP_M = 1.0  # a route's leg is checked against the fence at points this far apart
TARGETS = ("drop", "delivery", "recovery_pickup")
MAX_FROM_TAKEOFF_M = 2000.0  # further than this from the takeoff point is a typing mistake, not a target
FEET_TO_METERS = 0.3048
COORDINATE_KEYS = ("target_latitude", "target_longitude", "search_area", "flight_boundary", "pickup", "return_target", "bearing_deg", "start_latitude", "start_longitude", "boundary_m", "max_run_m")  # the only settings a generated sequence may differ in
WAYPOINT_FILE_LINE = "waypoint_file: ../waypoints/competition_waypoints.yaml"

# Sequence file -> what it takes from the day file. "target": the target whose coordinates become
# target_latitude and target_longitude; "waypoints": it flies the generated route (by its relative path, which
# the output folder keeps) and is given the day's flight_boundary, which the 2027 route code reads when it
# reorders the route or adds a run-in point (with every flag off nothing reads the key); "survey": search_area
# and flight_boundary; "pickup": the recovery's pickup, its flight_boundary and, when the day file names one,
# its return_target.
# The survey and the recovery have only sim sequences so far: their generated copies carry the day's coordinates
# and the sim's camera and vehicle, for rehearsing the day in the sim, and say so in their first line.
SEQUENCES: Dict[str, Dict[str, Any]] = {
    "package_drop_live.yaml": {"target": "drop"},
    "package_drop_2027.yaml": {"target": "drop"},
    "package_delivery_live.yaml": {"target": "delivery"},
    "package_delivery_2027.yaml": {"target": "delivery"},
    "final_waypoint_drop.yaml": {"target": "drop", "waypoints": True},
    "final_curcuit_delivery.yaml": {"target": "delivery", "waypoints": True},
    "waypoint_navigation_live.yaml": {"waypoints": True},
    "circuit_time_trial_live.yaml": {"waypoints": True},
    "fence_demo.yaml": {"fence_demo": True},  # the readiness video's demonstration: the run's bearing and length (OPS-02)
    "search_pattern_demo.yaml": {"survey": True, "sim_only": True},
    "sim_package_recovery.yaml": {"pickup": "recovery_pickup", "sim_only": True},
}


class FieldError(ValueError):
    """The day file cannot be used as written. The message says why."""


@dataclass(frozen=True)
class Day:
    name: str
    date: str
    verified: bool
    site: Any  # site_file.Site
    site_data: Dict[str, Any]  # the day file's site block, as typed
    targets: Dict[str, Tuple[float, float]]  # name -> (latitude, longitude)
    waypoints: Tuple[Dict[str, Any], ...]  # {"name", "latitude", "longitude", "altitude_agl_ft"}
    recovery_return: Optional[str] = None  # "takeoff", "compass_rose", or None: the sequence's own is left as it is
    fence_demo_bearing_deg: Optional[float] = None  # the geofence demonstration's compass bearing from the takeoff point, or None


def _latlon(item: Any, where: str) -> Tuple[float, float]:
    if not isinstance(item, dict) or set(item) - {"name", "latitude_deg", "longitude_deg", "altitude_agl_ft"}:
        raise FieldError(f"{where}: expected latitude_deg and longitude_deg")
    values = []
    for key, limit in (("latitude_deg", 90.0), ("longitude_deg", 180.0)):
        value = item.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or abs(value) > limit:
            raise FieldError(f"{where}: {key} must be a number of degrees, got {value!r}")
        values.append(float(value))
    return values[0], values[1]


def parse_day(data: Any, where: str = "day file") -> Day:
    """A day file's content, checked for shape. The geometry is check_day's."""
    if not isinstance(data, dict):
        raise FieldError(f"{where}: expected a mapping")
    unknown = sorted(str(key) for key in set(data) - DAY_KEYS)
    if unknown:
        raise FieldError(f"{where}: unknown key(s) {', '.join(unknown)} (a day file has {', '.join(sorted(DAY_KEYS))})")
    for key in ("name", "date", "site"):
        if key not in data:
            raise FieldError(f"{where}: missing '{key}'")
    name, date = str(data["name"]), str(data["date"])
    if not lines.re.fullmatch(r"[a-z0-9][a-z0-9_]*", name):
        raise FieldError(f"{where}: name is lower case, digits and underscores (it names the output folder), got '{name}'")
    if not lines.re.fullmatch(r"\d{4}-\d{2}-\d{2}", date):
        raise FieldError(f"{where}: date is YYYY-MM-DD, got '{date}'")
    verified = data.get("verified", False)
    if not isinstance(verified, bool):
        raise FieldError(f"{where}: verified is true or false")
    site_data = data["site"]
    if not isinstance(site_data, dict) or {"name", "verified"} & set(site_data):
        raise FieldError(f"{where}: site is a mapping of the site's values; its name and verified are the day file's own")
    try:
        site = site_file.parse_site({"name": name, "verified": verified, **site_data}, f"{where}: site")
    except site_file.SiteError as problem:
        raise FieldError(str(problem)) from problem
    targets_data = data.get("targets") or {}
    if not isinstance(targets_data, dict) or set(targets_data) - set(TARGETS):
        raise FieldError(f"{where}: targets has {', '.join(TARGETS)}")
    targets = {target: _latlon(item, f"{where}: targets.{target}") for target, item in targets_data.items()}
    waypoints_data = data.get("waypoints") or []
    if not isinstance(waypoints_data, list):
        raise FieldError(f"{where}: waypoints is a list")
    waypoints = []
    for index, item in enumerate(waypoints_data):
        latitude, longitude = _latlon(item, f"{where}: waypoints[{index}]")
        height = item.get("altitude_agl_ft")
        if isinstance(height, bool) or not isinstance(height, (int, float)) or not math.isfinite(height):
            raise FieldError(f"{where}: waypoints[{index}]: altitude_agl_ft must be a number of feet, got {height!r}")
        waypoints.append({"name": str(item.get("name", f"wp{index + 1}")), "latitude": latitude, "longitude": longitude, "altitude_agl_ft": height})
    recovery_return = data.get("recovery_return")
    if recovery_return is not None and recovery_return not in RECOVERY_RETURNS:
        raise FieldError(f"{where}: recovery_return is {' or '.join(RECOVERY_RETURNS)}, got {recovery_return!r}")
    demo = data.get("fence_demo")
    bearing = None
    if demo is not None:
        bearing = demo.get("bearing_deg") if isinstance(demo, dict) and set(demo) == {"bearing_deg"} else None
        if isinstance(bearing, bool) or not isinstance(bearing, (int, float)) or not 0.0 <= bearing <= 360.0:
            raise FieldError(f"{where}: fence_demo has bearing_deg, the compass bearing of the demonstration's run from the takeoff point (0 to 360)")
        bearing = float(bearing)
    return Day(name, date, verified, site, dict(site_data), targets, tuple(waypoints), recovery_return, bearing)


def load_day(path) -> Day:
    import yaml

    path = Path(path)
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as problem:
        raise FieldError(f"{path}: {problem}") from problem
    return parse_day(data, str(path))


def _point_facts(day: Day, latitude: float, longitude: float) -> Dict[str, float]:
    """A point against the site: metres east and north, from the takeoff point, and inside the fence by how much."""
    east, north = day.site.to_enu(latitude, longitude)
    d_east, d_north = east - day.site.takeoff[0], north - day.site.takeoff[1]
    inside = site_file.point_in_polygon(east, north, day.site.boundary)
    clearance = site_file._distance_to_boundary((east, north), day.site.boundary)
    return {"east_m": east, "north_m": north, "from_takeoff_m": math.hypot(d_east, d_north), "bearing_deg": math.degrees(math.atan2(d_east, d_north)) % 360.0,
            "clearance_m": clearance if inside else -clearance}  # fmt: skip


def points(day: Day) -> List[Tuple[str, float, float]]:
    """Every commanded point of the day, by name: the targets, the waypoints, the survey area's corners."""
    named = [(f"targets.{name}", *latlon) for name, latlon in day.targets.items()]
    named += [(f"waypoint {item['name']}", item["latitude"], item["longitude"]) for item in day.waypoints]
    corners = day.site_data.get("survey_area") or []
    named += [(f"survey_area[{index}]", float(corner["latitude_deg"]), float(corner["longitude_deg"])) for index, corner in enumerate(corners)]
    return named


def distance_to_boundary_along(day: Day, bearing_deg: float) -> Optional[float]:
    """From the takeoff point along a compass bearing: how far to the flight boundary. None if the line never meets it."""
    met = boundary_meeting(day, bearing_deg)
    return None if met is None else met[0]


def boundary_meeting(day: Day, bearing_deg: float) -> Optional[Tuple[float, float]]:
    """Where a compass bearing from the takeoff point meets the flight boundary: (how far, degrees from square to that edge)."""
    start = day.site.takeoff
    d_east, d_north = math.sin(math.radians(bearing_deg)), math.cos(math.radians(bearing_deg))
    nearest = None
    corners = day.site.boundary
    for index, a in enumerate(corners):
        b = corners[(index + 1) % len(corners)]
        edge_east, edge_north = b[0] - a[0], b[1] - a[1]
        cross = d_east * edge_north - d_north * edge_east
        if abs(cross) < 1.0e-12:
            continue  # the edge runs along the bearing
        along = ((a[0] - start[0]) * edge_north - (a[1] - start[1]) * edge_east) / cross
        on_edge = ((a[0] - start[0]) * d_north - (a[1] - start[1]) * d_east) / cross
        if along > 0.0 and -1.0e-9 <= on_edge <= 1.0 + 1.0e-9 and (nearest is None or along < nearest[0]):
            length = math.hypot(edge_east, edge_north)
            # Square to the edge is the bearing along the edge's normal: the sine of the angle from square is the
            # share of the run's direction that lies along the edge.
            along_edge = abs(d_east * edge_east + d_north * edge_north) / length
            nearest = (along, math.degrees(math.asin(min(1.0, along_edge))))
    return nearest


def fence_demo_run_m(day: Day) -> Optional[float]:
    """The demonstration's max_run_m: to the boundary along its bearing, and the overshoot. None with no demonstration in the day file."""
    if day.fence_demo_bearing_deg is None:
        return None
    distance = distance_to_boundary_along(day, day.fence_demo_bearing_deg)
    return None if distance is None else round(distance + FENCE_DEMO_OVERSHOOT_M, 1)


def check_day(day: Day) -> List[str]:
    """What in a day's coordinates breaks a rule or the fence. Empty when the day can be flown as typed."""
    problems = []
    margin = safety_limits.GEOFENCE_MARGIN_M
    for name, latitude, longitude in points(day):
        facts = _point_facts(day, latitude, longitude)
        if facts["from_takeoff_m"] > MAX_FROM_TAKEOFF_M:
            problems.append(f"{name} is {facts['from_takeoff_m']:.0f} m from the takeoff point (more than {MAX_FROM_TAKEOFF_M:g} m): a latitude and longitude swapped, or a digit lost?")
        elif facts["clearance_m"] < 0.0:
            problems.append(f"{name} is outside the flight boundary, by {-facts['clearance_m']:.1f} m")
        elif facts["clearance_m"] < margin:
            problems.append(f"{name} is {facts['clearance_m']:.1f} m inside the flight boundary; a commanded point keeps {margin:g} m (the fence's margin, GPS error and overshoot)")
    # The route's legs, in the order given: two points inside a boundary with a notch in it can have the fence between them.
    for one, other in zip(day.waypoints, day.waypoints[1:]):
        a, b = day.site.to_enu(one["latitude"], one["longitude"]), day.site.to_enu(other["latitude"], other["longitude"])
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        if length > MAX_FROM_TAKEOFF_M:
            continue  # an end is far from the field, and is said above
        worst = None
        for step in range(1, int(length / LEG_STEP_M)):
            east, north = a[0] + (b[0] - a[0]) * step * LEG_STEP_M / length, a[1] + (b[1] - a[1]) * step * LEG_STEP_M / length
            inside = site_file.point_in_polygon(east, north, day.site.boundary)
            clearance = site_file._distance_to_boundary((east, north), day.site.boundary) * (1.0 if inside else -1.0)
            worst = clearance if worst is None else min(worst, clearance)
        ends_ok = all(_point_facts(day, item["latitude"], item["longitude"])["clearance_m"] >= margin for item in (one, other))
        if ends_ok and worst is not None and worst < margin:
            where = "leaves the flight boundary" if worst < 0.0 else f"comes within {worst:.1f} m of the flight boundary"
            problems.append(f"the leg from {one['name']} to {other['name']} {where} (a commanded path keeps {margin:g} m)")
    if day.fence_demo_bearing_deg is not None:
        run = fence_demo_run_m(day)
        if run is None:
            problems.append(f"fence_demo: the bearing {day.fence_demo_bearing_deg:g} from the takeoff point never meets the flight boundary")
        elif boundary_meeting(day, day.fence_demo_bearing_deg)[1] > FENCE_DEMO_MAX_OBLIQUE_DEG:
            oblique = boundary_meeting(day, day.fence_demo_bearing_deg)[1]
            problems.append(f"fence_demo: bearing {day.fence_demo_bearing_deg:g} meets the boundary {oblique:.0f} degrees from square; at most {FENCE_DEMO_MAX_OBLIQUE_DEG:g}: the flight controller's avoidance slides a vehicle along a fence it meets at an angle. Choose the bearing square to the edge")
        elif run > FENCE_DEMO_MAX_RUN_M:
            problems.append(f"fence_demo: the boundary is {run - FENCE_DEMO_OVERSHOOT_M:.0f} m from the takeoff point on bearing {day.fence_demo_bearing_deg:g}; a run may be {FENCE_DEMO_MAX_RUN_M:g} m at most: choose a bearing to a nearer edge")
    names = [item["name"] for item in day.waypoints]
    for name in sorted({name for name in names if names.count(name) > 1}):
        problems.append(f"waypoint {name} is listed more than once")
    ceiling_ft = day.site.max_altitude_agl_m / FEET_TO_METERS
    for item in day.waypoints:
        if item["altitude_agl_ft"] <= 0.0:
            problems.append(f"waypoint {item['name']}: altitude_agl_ft must be above 0, got {item['altitude_agl_ft']:g}")
        elif item["altitude_agl_ft"] > ceiling_ft:
            problems.append(f"waypoint {item['name']}: {item['altitude_agl_ft']:g} ft is above the site's ceiling ({day.site.max_altitude_agl_m:g} m, {ceiling_ft:.0f} ft)")
    return problems


def needed_by(sequence: str, day: Day) -> List[str]:
    """What a sequence needs that the day file does not give. Empty when it can be generated."""
    plan = SEQUENCES[sequence]
    missing = [f"targets.{plan[key]}" for key in ("target", "pickup") if key in plan and plan[key] not in day.targets]
    if plan.get("waypoints") and not day.waypoints:
        missing.append("waypoints")
    if plan.get("fence_demo") and day.fence_demo_bearing_deg is None:
        missing.append("fence_demo")
    return missing


def _generated_header(source: str, day: Day, sha256: str, sim_only: bool = False) -> List[str]:
    header = [f"# GENERATED by tools/field_setup.py from {source} and the day file {day.name} ({day.date}, sha256 {sha256[:12]}): do not edit.",
              "# The committed file line for line, with the day's coordinates and nothing else changed. To change a coordinate,",
              "# change the day file and run the tool again."]  # fmt: skip
    if not day.verified:
        header.append("# NOT VERIFIED: the day file's coordinates have not been checked against the organisers' sheet. Not for flight.")
    if sim_only:
        header.append("# From a sim sequence (the sim's camera and vehicle): for rehearsing the day in the sim, not for the aircraft.")
    return header


def sequence_text(source_text: str, sequence: str, day: Day, sha256: str) -> str:
    """A sequence file's text with the day's coordinates. Raises FieldError when the file has no place for them."""
    plan = SEQUENCES[sequence]
    changed = {key: 0 for key in COORDINATE_KEYS}
    out = _generated_header(sequence, day, sha256, bool(plan.get("sim_only")))
    boundary = "[" + ", ".join(f"[{latitude:.7f}, {longitude:.7f}]" for latitude, longitude in day.site.boundary_geo) + "]"
    boundary_note = "# The day's flight boundary, [latitude, longitude] corners (field_setup.py)."
    for line in source_text.splitlines():
        match = lines._SETTING.match(line)
        key = match["key"] if match and match["value"] is not None else None
        value = None
        if line.strip() == WAYPOINT_FILE_LINE and plan.get("waypoints"):
            # The route's mission gets the day's fence beside its route: a key the source file does not have.
            out += [line, f"{match['indent']}flight_boundary: {boundary} {boundary_note}"]
            changed["flight_boundary"] += 1
            continue
        if key == "pickup" and "pickup" in plan:
            out += [lines._with_value(match, "{latlon: [%.7f, %.7f]}" % day.targets[plan["pickup"]]), f"{match['indent']}flight_boundary: {boundary} {boundary_note}"]
            changed["pickup"] += 1
            changed["flight_boundary"] += 1
            continue
        if key == "return_target" and "pickup" in plan and day.recovery_return is not None:
            rose = day.site_data["compass_rose"]
            value = "takeoff" if day.recovery_return == "takeoff" else "{latlon: [%.7f, %.7f]}" % (rose["latitude_deg"], rose["longitude_deg"])
            out.append(lines._with_value(match, value))
            changed["return_target"] += 1
            continue
        if key == "bearing_deg" and plan.get("fence_demo"):
            value = f"{day.fence_demo_bearing_deg:.1f}"
        elif key in ("start_latitude", "start_longitude") and plan.get("fence_demo"):
            value = f"{day.site_data['takeoff_point']['latitude_deg' if key == 'start_latitude' else 'longitude_deg']:.7f}"
        elif key == "boundary_m" and plan.get("fence_demo"):
            value = f"{fence_demo_run_m(day) - FENCE_DEMO_OVERSHOOT_M:.1f}"
        elif key == "max_run_m" and plan.get("fence_demo"):
            value = f"{fence_demo_run_m(day):.1f}"
        elif key in ("target_latitude", "target_longitude") and "target" in plan:
            value = f"{day.targets[plan['target']][0 if key == 'target_latitude' else 1]:.7f}"
        elif key == "search_area" and plan.get("survey"):
            value = "[" + ", ".join(f"[{corner['latitude_deg']:.7f}, {corner['longitude_deg']:.7f}]" for corner in day.site_data["survey_area"]) + "]"
        elif key == "flight_boundary" and plan.get("survey"):
            value = boundary
        elif key == "flight_boundary" and (plan.get("waypoints") or "pickup" in plan):
            raise FieldError(f"{sequence}: it has a flight_boundary line of its own; the tool adds the day's and would leave two")
        if value is None:
            out.append(line)
        else:
            changed[key] += 1
            out.append(lines._with_value(match, value))
    wanted = (["target_latitude", "target_longitude"] if "target" in plan else []) + (["search_area", "flight_boundary"] if plan.get("survey") else []) + (["pickup", "flight_boundary"] if "pickup" in plan else [])
    wanted += ["return_target"] if "pickup" in plan and day.recovery_return is not None else []
    wanted += ["bearing_deg", "start_latitude", "start_longitude", "boundary_m", "max_run_m"] if plan.get("fence_demo") else []
    for key in wanted:
        if changed[key] != 1:
            raise FieldError(f"{sequence}: expected one {key} line to put the day's coordinates in, found {changed[key]}")
    if plan.get("waypoints") and WAYPOINT_FILE_LINE not in source_text:
        raise FieldError(f"{sequence}: it does not read its route from ../waypoints/competition_waypoints.yaml, so the day's waypoints would not reach it")
    if plan.get("waypoints") and changed["flight_boundary"] != 1:
        raise FieldError(f"{sequence}: expected one waypoint_file line to put the day's flight boundary beside, found {changed['flight_boundary']}")
    return "\n".join(out) + "\n"


def leaf_differences(before: Any, after: Any, path: str = "") -> List[str]:
    """The paths at which two loaded YAML documents differ."""
    if isinstance(before, dict) and isinstance(after, dict):
        found = []
        for key in sorted(set(before) | set(after), key=str):
            found += leaf_differences(before.get(key), after.get(key), f"{path}.{key}" if path else str(key))
        return found
    if isinstance(before, list) and isinstance(after, list) and len(before) == len(after) and all(isinstance(item, dict) for item in before + after):
        found = []
        for index, (one, other) in enumerate(zip(before, after)):
            found += leaf_differences(one, other, f"{path}[{index}]")
        return found
    return [] if before == after else [path]


def only_coordinates_changed(source_text: str, generated_text: str, sequence: str) -> None:
    """Refuse a generated sequence that differs from its source anywhere but in the coordinates it was given."""
    import yaml

    allowed = COORDINATE_KEYS
    # A difference counts as a coordinate when one of the allowed keys is on its path (pickup.latlon is under pickup).
    stray = [path for path in leaf_differences(yaml.safe_load(source_text), yaml.safe_load(generated_text)) if not set(path.replace("[", ".").split(".")) & set(allowed)]
    if stray:
        raise FieldError(f"{sequence}: the generated file differs from the committed one at {', '.join(stray)}: nothing but coordinates may change")


def waypoints_text(day: Day, sha256: str) -> str:
    out = [f"# GENERATED by tools/field_setup.py from the day file {day.name} ({day.date}, sha256 {sha256[:12]}): do not edit.",
           "# The waypoint and time-trial route. Heights are above the ground, in feet, as the organisers give them."]  # fmt: skip
    if not day.verified:
        out.append("# NOT VERIFIED: the day file's coordinates have not been checked against the organisers' sheet. Not for flight.")
    out.append("waypoints:")
    for item in day.waypoints:
        out += [f"  - name: {item['name']}", f"    latitude: {item['latitude']:.7f}", f"    longitude: {item['longitude']:.7f}", f"    altitude_agl_ft: {item['altitude_agl_ft']:g}"]
    return "\n".join(out) + "\n"


def site_text(day: Day, sha256: str) -> str:
    import yaml

    head = f"# GENERATED by tools/field_setup.py from the day file {day.name} ({day.date}, sha256 {sha256[:12]}): do not edit.\n"
    if not day.verified:
        head += "# NOT VERIFIED: not for flight.\n"
    return head + yaml.safe_dump({"name": day.name, "verified": day.verified, **day.site_data}, sort_keys=False)


def fence_plan_text(day: Day) -> str:
    """The fence as a QGroundControl plan file: one inclusion polygon, no mission items."""
    home = [day.site.origin.latitude_deg, day.site.origin.longitude_deg, 0]
    plan = {
        "fileType": "Plan",
        "geoFence": {"circles": [], "polygons": [{"inclusion": True, "polygon": [[latitude, longitude] for latitude, longitude in day.site.boundary_geo], "version": 1}], "version": 2},
        "groundStation": "QGroundControl",
        "mission": {"cruiseSpeed": 15, "firmwareType": 3, "globalPlanAltitudeMode": 1, "hoverSpeed": 5, "items": [], "plannedHomePosition": home, "vehicleType": 2, "version": 2},
        "rallyPoints": {"points": [], "version": 2},
        "version": 1,
    }
    return json.dumps(plan, indent=2) + "\n"


def fence_param_text(day: Day) -> str:
    """The fence's parameters as the site file's plan sets them, FENCE_ENABLE last."""
    params = site_file.fence_plan(day.site).params
    head = f"# The fence's parameters for {day.name} ({day.date}): load after the polygon is uploaded. sim/fence_upload.py sets the same, in this order.\n"
    return head + "".join(f"{name},{value:g}\n" for name, value in params.items())


def summary_text(day: Day) -> str:
    out = [f"{day.name}, {day.date}" + ("" if day.verified else "  (NOT VERIFIED)"), "",
           f"{'point':<28}{'from takeoff':>14}{'bearing':>10}{'east':>9}{'north':>9}{'inside fence by':>17}"]  # fmt: skip
    for name, latitude, longitude in points(day):
        facts = _point_facts(day, latitude, longitude)
        out.append(f"{name:<28}{facts['from_takeoff_m']:>12.1f} m{facts['bearing_deg']:>8.0f} d{facts['east_m']:>7.1f} m{facts['north_m']:>7.1f} m{facts['clearance_m']:>15.1f} m")
    if len(day.waypoints) > 1:
        legs = []
        for one, other in zip(day.waypoints, day.waypoints[1:]):
            a, b = day.site.to_enu(one["latitude"], one["longitude"]), day.site.to_enu(other["latitude"], other["longitude"])
            legs.append(math.hypot(b[0] - a[0], b[1] - a[1]))
        out += ["", "route: " + ", ".join(f"{leg:.0f} m" for leg in legs) + f"; {sum(legs):.0f} m in the order given"]
    if day.fence_demo_bearing_deg is not None and fence_demo_run_m(day) is not None:
        run = fence_demo_run_m(day)
        out += ["", f"fence demonstration: from the takeoff point on bearing {day.fence_demo_bearing_deg:.0f}, {run - FENCE_DEMO_OVERSHOOT_M:.1f} m to the boundary; the run fails at {run:.1f} m"]
    out += ["", site_file.summary(day.site)]
    return "\n".join(out) + "\n"


def generate(day: Day, out_dir: Path, sha256: str) -> Tuple[List[Path], List[str]]:
    """Write the day's files under `out_dir`; (the files written, what was left out and why)."""
    files: Dict[Path, str] = {
        out_dir / "site.yaml": site_text(day, sha256),
        out_dir / "fence.plan": fence_plan_text(day),
        out_dir / "fence.param": fence_param_text(day),
        out_dir / "summary.txt": summary_text(day),
    }
    skipped = []
    if day.waypoints:
        files[out_dir / "waypoints" / "competition_waypoints.yaml"] = waypoints_text(day, sha256)
    for sequence in SEQUENCES:
        missing = needed_by(sequence, day)
        if missing:
            skipped.append(f"{sequence}: the day file has no {', '.join(missing)}")
            continue
        source_text = (CONFIG / "sequences" / sequence).read_text(encoding="utf-8")
        text = sequence_text(source_text, sequence, day, sha256)
        only_coordinates_changed(source_text, text, sequence)
        files[out_dir / "sequences" / sequence] = text
    for path, text in files.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
    return sorted(files), skipped


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Check the day's coordinates and generate the files that need them.")
    parser.add_argument("day_file")
    parser.add_argument("--out", help="the folder to write (default: field/<name>/)")
    parser.add_argument("--check", action="store_true", help="check only; write nothing")
    parser.add_argument("--for-flight", action="store_true", help="refuse a day file that is not marked verified")
    args = parser.parse_args(argv)
    try:
        day = load_day(args.day_file)
        sha256 = hashlib.sha256(Path(args.day_file).read_bytes()).hexdigest()
    except FieldError as problem:
        print(f"field_setup: {problem}", file=sys.stderr)
        return 2
    problems = check_day(day)
    if args.for_flight and not day.verified:
        problems.append("the day file is not verified: check every value against the organisers' sheet and set verified: true")
    print(summary_text(day), end="")
    if problems:
        print("\n" + "\n".join(f"PROBLEM: {problem}" for problem in problems))
        print(f"\n{len(problems)} problem(s): nothing was written")
        return 1
    if args.check:
        print("\nOK: nothing was written (--check)")
        return 0
    try:
        written, skipped = generate(day, Path(args.out) if args.out else FIELD / day.name, sha256)
    except (FieldError, OSError) as problem:
        print(f"field_setup: {problem}", file=sys.stderr)
        return 2
    print("")
    for path in written:
        print(f"wrote {path}")
    for line in skipped:
        print(f"left out {line}")
    if not day.verified:
        print("NOT VERIFIED: these files are for the sim and for checking, not for flight")
    return 0


if __name__ == "__main__":
    sys.exit(main())
