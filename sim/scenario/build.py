#!/usr/bin/env python3
"""Build a scenario into a run folder. No ROS, no Gazebo.

  python3 sim/scenario/build.py sim/scenarios/delivery/baseline.yaml
  python3 sim/scenario/build.py <scenario.yaml> [--seed N] [--variant off|on] [--out DIR] [--dry-run]

The run folder (default sim/.state/runs/<mission>-<name>-s<seed>-<variant>) gets:

  world.sdf      the site world with every object already in it (no spawn race)
  bridge.yaml    the Gazebo-to-ROS bridge that matches the vehicle's sensors
  sitl.parm      SITL parameters for this run
  sequence.yaml  the mission sequence to fly: the shipped one plus this
                 scenario's target coordinates, overrides and (variant on) flags
  params.yaml    the ROS params file, with node flags set in variant on
  truth.json     where everything really is, for the judge
  judge.yaml     which judge scores it, and its settings
  scenario.json  what the runner needs: launch, timeout, wind, GPS events,
                 and whether the sim runs on sim time

Variant "off" leaves every flag false: that is the 2026 behaviour. Flags are
only ever switched on in these generated copies; a flag that no shipped file
declares fails the build, as does a setting or a config key nothing reads.
A scenario with `sim_time: true` runs on Gazebo's clock in variant "on" (the
sim's start script and the mission launch read it from scenario.json); in
variant "off" it runs on wall time like every other.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import random
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parents[1]
REPO = SIM_DIR.parent
sys.path[:0] = [str(SIM_DIR), str(REPO / "src" / "drone_utils")]

import site_file  # noqa: E402
from drone_utils import sdf_objects  # noqa: E402  (no ROS imports: used from the source tree)
from judge import MISSIONS, default_settings  # noqa: E402
import payload_bridge  # noqa: E402
from judge.geometry import point_in_polygon  # noqa: E402
from target_placement import latlon_for  # noqa: E402

if __package__ in (None, ""):
    from scenario.spec import DRIVER_PORT, MIN_REAL_TIME_FACTOR, SENSOR_PROFILES, BuildError, Scenario, load_scenario  # noqa: E402
else:
    from .spec import DRIVER_PORT, MIN_REAL_TIME_FACTOR, SENSOR_PROFILES, BuildError, Scenario, load_scenario  # noqa: E402

CONFIG_DIR = REPO / "src" / "drone_mission_demo" / "config"
LAUNCH_DIR = REPO / "src" / "drone_mission_demo" / "launch"
SITES_DIR = SIM_DIR / "sites"
BASE_WORLD = SIM_DIR / "worlds" / "argus_runway.sdf"
RUNS_DIR = SIM_DIR / ".state" / "runs"
FALLBACK_LAUNCH = "multi_mission_demo.launch.py"
# Nodes the demo launches start only on a launch argument of their own name, which a scenario gives by turning
# the node's flag of the same name on: `node_flags: {ring_fit: {ring_fit: true}}`.
FLAG_STARTED_NODES = ("ring_fit", "loc_node", "recovery_cv")

# What the sim's oracle detector (LOC-5) reports each kind of object as, on the detection contract. A marker with
# no digit is one of the four larger boundary markers. A payload is not an object of the contract.
ORACLE_CLASSES = {"bullseye": "bullseye", "x_mat": "green_x", "bucket": "bucket"}
ORACLE_SIZE_M = {"bullseye": 5.0, "x_mat": 5.0, "bucket": 0.19}
ORACLE_TOP_M = {"bucket": 0.20}


def oracle_objects(objects: list[dict], vehicle_model: str) -> dict:
    """The true objects as the oracle detector takes them: the run's oracle_objects.json."""
    listed = []
    for item in objects:
        kind = item["kind"]
        if kind == "marker":
            class_id = "marker" if item.get("digit") is not None else "boundary_marker"
        elif kind in ORACLE_CLASSES:
            class_id = ORACLE_CLASSES[kind]
        else:
            continue
        size = float(item.get("size_m" if kind != "bucket" else "diameter_m", ORACLE_SIZE_M.get(kind, 0.6)))
        listed.append({"name": item["name"], "class_id": class_id, "size_m": size, "top_m": ORACLE_TOP_M.get(kind, SURFACE_HEIGHT_M.get(kind, 0.0))})
    return {"vehicle": vehicle_model, "objects": listed}

# Height of the surface an object stands on when it is placed on top of another.
SURFACE_HEIGHT_M = {"x_mat": 0.03, "bullseye": 0.04, "marker": 0.02}
PLACEMENT_TRIES = 2000


def default_run_dir(scenario: Scenario, seed: int, variant: str) -> Path:
    return RUNS_DIR / f"{scenario.mission}-{scenario.name}-s{seed}-{variant}"


def _load_yaml(path: Path, what: str):
    import yaml

    if not path.is_file():
        raise BuildError(f"{what} '{path.name}' does not exist in {path.parent}")
    return yaml.safe_load(path.read_text())


# --- placement ----------------------------------------------------------------


def _area(rule, site: site_file.Site, name: str):
    if rule == "survey_area":
        return list(site.survey_area)
    if rule == "flight_boundary":
        return list(site.boundary)
    if isinstance(rule, dict) and set(rule) == {"east_m", "north_m"}:
        (e0, e1), (n0, n1) = rule["east_m"], rule["north_m"]
        return [(e0, n0), (e1, n0), (e1, n1), (e0, n1)]
    raise BuildError(f"object '{name}': area is survey_area, flight_boundary or {{east_m: [a, b], north_m: [c, d]}}")


def place_objects(scenario: Scenario, site: site_file.Site, seed: int) -> list[dict]:
    """Give every object its true position. Random rules follow the seed."""

    rng = random.Random(f"{seed}-placement")
    placed: dict[str, dict] = {}
    for item in scenario.objects:
        name = item["name"]
        entry = {key: value for key, value in item.items() if key not in ("at", "on_top_of")}
        entry.setdefault("heading_deg", 0.0)
        entry["heading_deg"] = float(entry["heading_deg"])
        entry["up"] = 0.0
        if item["kind"] == "marker":
            entry.setdefault("digit", None)
            entry.setdefault("size_m", 0.6 if entry["digit"] is not None else 1.2)
        if item.get("carried") is True:
            # Hanging under the vehicle where it stands: placed by `carry` once the vehicle's pose is known.
            entry["e"], entry["n"] = site.takeoff
        elif "on_top_of" in item:
            support = placed.get(item["on_top_of"])
            if support is None:
                raise BuildError(f"object '{name}' is on top of '{item['on_top_of']}', which is not an object listed before it")
            entry["e"], entry["n"] = support["e"], support["n"]
            entry["up"] = SURFACE_HEIGHT_M.get(support["kind"], 0.0)
        elif "random" in item["at"]:
            rule = item["at"]["random"]
            polygon = _area(rule["area"], site, name)
            separation = float(rule.get("min_separation_m", 0.0))
            margin = float(rule.get("margin_m", 1.0))
            east = [corner[0] for corner in polygon]
            north = [corner[1] for corner in polygon]
            for _ in range(PLACEMENT_TRIES):
                e, n = rng.uniform(min(east), max(east)), rng.uniform(min(north), max(north))
                if not point_in_polygon(e, n, polygon) or site_file._distance_to_boundary((e, n), polygon) < margin:
                    continue
                if any(math.hypot(e - other["e"], n - other["n"]) < separation for other in placed.values()):
                    continue
                entry["e"], entry["n"] = e, n
                break
            else:
                raise BuildError(f"could not place object '{name}': no free spot in its area {separation} m from the others")
        else:
            entry["e"], entry["n"] = float(item["at"]["east_m"]), float(item["at"]["north_m"])
        if not point_in_polygon(entry["e"], entry["n"], site.boundary):
            raise BuildError(f"object '{name}' at {entry['e']:.1f} E, {entry['n']:.1f} N is outside the flight boundary")
        placed[name] = entry
    return list(placed.values())


def coordinate_error(scenario: Scenario, seed: int) -> tuple[float, float]:
    """East and north error in the coordinates the mission is given."""

    rule = scenario.coordinate_error
    if not rule:
        return 0.0, 0.0
    if "east_m" in rule:
        return float(rule["east_m"]), float(rule["north_m"])
    rng = random.Random(f"{seed}-coordinate-error")
    bearing = rng.uniform(0.0, 2.0 * math.pi)
    if "radius_m" in rule:
        radius = float(rule["radius_m"])
    else:
        radius = float(rule["max_radius_m"]) * math.sqrt(rng.random())  # uniform over the disc
    return radius * math.sin(bearing), radius * math.cos(bearing)


# --- the world -----------------------------------------------------------------


def _object_sdf(item: dict) -> str:
    kind, name = item["kind"], item["name"]
    try:
        if kind == "bullseye":
            return sdf_objects.build_red_white_target_sdf(name, size=float(item.get("size_m", 5.0)), num_rings=int(item.get("rings", 8)))
        if kind == "marker":
            return sdf_objects.build_marker_sdf(name, size=float(item["size_m"]), digit=item["digit"])
        if kind == "x_mat":
            return sdf_objects.build_x_mat_sdf(name, size=float(item.get("size_m", 5.0)), arm_width=float(item.get("arm_width_m", 0.5)))
        if kind == "bucket":
            return sdf_objects.build_bucket_sdf(
                name, mass_kg=float(item.get("mass_kg", 1.5)), diameter=float(item.get("diameter_m", 0.19)), height=float(item.get("height_m", 0.20))
            )
        return sdf_objects.build_payload_sdf(name, mass_kg=float(item.get("mass_kg", 0.5)), size=item.get("size_m", (0.15, 0.15, 0.10)))
    except (TypeError, ValueError) as error:
        raise BuildError(f"object '{name}': {error}") from error


# A carried payload hangs from the vehicle by a Gazebo detachable joint to the camera link, which is the vehicle
# model's own link (a joint's link is found by name among a model's direct links). Where it hangs, in the body's
# frame (forward, left, up from the body's origin, to the box's centre): beside the camera, between the legs, clear
# of the ground when the vehicle stands and out of the camera's view.
CARRY_LINK = "downcam_link"
CARRY_OFFSET_BODY_M = (0.0, 0.13, -0.12)
CARRY_MAX_SIZE_M = (0.20, 0.10, 0.10)  # larger than this and it shows in the camera's image, touches a leg or the ground
_VEHICLE_INCLUDE_POSE = re.compile(r'<uri>model://(\w+)</uri>\s*<pose degrees="true">\s*(\S+)\s+(\S+)\s+(\S+)\s+\S+\s+\S+\s+(\S+)\s*</pose>')


def vehicle_pose(world_sdf: str, model: str) -> tuple[float, float, float, float]:
    """(x, y, z, yaw in degrees) of the vehicle's include in a world."""
    for match in _VEHICLE_INCLUDE_POSE.finditer(world_sdf):
        if match.group(1) == model:
            return tuple(float(match.group(index)) for index in (2, 3, 4, 5))
    raise BuildError(f"the base world has no <include> of model://{model} with a pose in degrees")


def carry(item: dict, vehicle: tuple[float, float, float, float]) -> dict:
    """A carried payload's object with its pose under the vehicle: `e`, `n`, `up` of the box's base, and its heading."""
    size = item.get("size_m", (0.15, 0.15, 0.10))
    size = (size, size, size) if isinstance(size, (int, float)) else tuple(float(value) for value in size)
    if any(side > limit + 1.0e-9 for side, limit in zip(size, CARRY_MAX_SIZE_M)):
        raise BuildError(
            f"object '{item['name']}': a carried payload can be at most {CARRY_MAX_SIZE_M[0]} x {CARRY_MAX_SIZE_M[1]} x {CARRY_MAX_SIZE_M[2]} m "
            f"(it hangs between the camera's view and the legs); this one is {size[0]} x {size[1]} x {size[2]} m"
        )
    x, y, z, yaw_deg = vehicle
    yaw = math.radians(yaw_deg)
    forward, left, up = CARRY_OFFSET_BODY_M
    placed = dict(item)
    placed["size_m"] = list(size)
    placed["e"] = x + forward * math.cos(yaw) - left * math.sin(yaw)
    placed["n"] = y + forward * math.sin(yaw) + left * math.cos(yaw)
    placed["up"] = z + up - size[2] / 2.0  # the model's origin is the bottom of the box
    placed["yaw_rad"] = yaw
    return placed


def _joint_to_vehicle(name: str, vehicle_model: str, link: str = CARRY_LINK) -> ET.Element:
    """The detachable joint from a body to the vehicle, made and unmade on the body's own topics (sim/payload_bridge.py).

    `link` is the vehicle's link the body is joined to: the camera link, or the recovery vehicle's hook for a bucket."""
    plugin = ET.Element("plugin", {"filename": "gz-sim-detachable-joint-system", "name": "gz::sim::systems::DetachableJoint"})
    for tag, text in (
        ("parent_link", "link"),
        ("child_model", vehicle_model),
        ("child_link", link),
        ("detach_topic", payload_bridge.joint_topic(name, "detach")),
        ("attach_topic", payload_bridge.joint_topic(name, "attach")),
    ):
        ET.SubElement(plugin, tag).text = text
    return plugin


def _embedded_model(item: dict, vehicle_model: str | None = None, link: str = CARRY_LINK) -> str:
    """The object's model as a child of <world>, with its pose. Heading is a compass heading.

    `vehicle_model` is given for a body that can hang from the vehicle (a
    carried payload, the mission's bucket): its model then has the joint,
    to the vehicle's `link`.
    """
    model = ET.fromstring(_object_sdf(item)).find("model")
    pose = ET.Element("pose")
    yaw = item["yaw_rad"] if "yaw_rad" in item else math.radians(-item["heading_deg"])
    pose.text = f"{item['e']:.4f} {item['n']:.4f} {item['up']:.4f} 0 0 {yaw:.6f}"
    model.insert(0, pose)
    if vehicle_model is not None:
        model.append(_joint_to_vehicle(item["name"], vehicle_model, link))
    return "    " + ET.tostring(model, encoding="unicode") + "\n"


_SUN_DIRECTION = re.compile(r'(<light type="directional" name="sun">.*?<direction>)[^<]*(</direction>)', re.DOTALL)
# Every built run's SITL: flat ground. The rangefinder stays the default parameters' analog one (RNGFND1_TYPE 1),
# which reads the height above that flat ground. SITL's own type 100, which FEATURES.md names, has no data with
# the Gazebo backend (it takes ranges Gazebo would have to send) and fails the pre-arm check: tried under SIM-05.
SITL_BASE_PARAMS = {"SIM_TERRAIN": 0}
RECOVERY_PARM = SIM_DIR / "params" / "recovery.parm"


def read_parm(path: Path) -> dict:
    """A parameter file's `NAME value` lines as {name: number}. Comments and blank lines are skipped."""
    params = {}
    for number, line in enumerate(Path(path).read_text().splitlines(), start=1):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.replace(",", " ").split()
        try:
            params[parts[0]] = float(parts[1])
        except (IndexError, ValueError) as error:
            raise BuildError(f"{path}, line {number}: expected NAME value, got {line!r}") from error
    return params
BASE_VEHICLE_MODEL = "iris_downcam"  # the vehicle the base world includes: the delivery profile's
_VEHICLE_POSE = re.compile(r'<uri>model://iris_downcam</uri>\s*<pose degrees="true">\s*\S+\s+\S+\s+(\S+)')


# Gazebo's system that applies a force to a body on request: what sim/wind.py blows through (SIM-03).
WRENCH_SYSTEM = '    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>\n'


def build_world(
    base_sdf: str, site: site_file.Site, objects: list[dict], sun: dict | None, windy: bool = False, vehicle_model: str | None = None, hung: tuple = (), hook_link: str | None = None
) -> str:
    """The run's world. `hung` names the objects that can hang from the vehicle: their models get the joint to `vehicle_model`.

    A bucket's joint is to `hook_link` when the vehicle has a hook (the recovery vehicle), so that it swings with it."""
    text = site_file.site_world(base_sdf, site)
    if sun is not None:  # otherwise the base world's light, as in the 2026 sim
        elevation, azimuth = math.radians(sun["elevation_deg"]), math.radians(sun["azimuth_deg"])
        # The light travels from the sun: opposite to the direction the sun is seen in.
        direction = (-math.cos(elevation) * math.sin(azimuth), -math.cos(elevation) * math.cos(azimuth), -math.sin(elevation))
        text, found = _SUN_DIRECTION.subn(lambda m: f"{m.group(1)}{direction[0]:.6f} {direction[1]:.6f} {direction[2]:.6f}{m.group(2)}", text, count=1)
        if not found:
            raise BuildError("the base world has no directional light named 'sun'")
    if vehicle_model not in (None, BASE_VEHICLE_MODEL):  # another camera profile: the same Iris, standing where the base world stands it
        if text.count(f"<uri>model://{BASE_VEHICLE_MODEL}</uri>") != 1:
            raise BuildError(f"the base world does not include model://{BASE_VEHICLE_MODEL} exactly once")
        text = text.replace(f"<uri>model://{BASE_VEHICLE_MODEL}</uri>", f"<uri>model://{vehicle_model}</uri>")
    models = "".join(
        _embedded_model(item, vehicle_model if item["name"] in hung else None, hook_link if hook_link and item["kind"] == "bucket" else CARRY_LINK) for item in objects
    )
    head, closing, tail = text.rpartition("</world>")
    if not closing:
        raise BuildError("the base world has no </world>")
    # Only a windy run's world gets the wrench system: a calm world is the base world, as before.
    return f"{head}{models}{WRENCH_SYSTEM if windy else ''}  {closing}{tail}"


def vehicle_ground_offset_m(base_sdf: str) -> float:
    """Height of the vehicle model's origin when it stands on the ground: its pose in the world."""
    match = _VEHICLE_POSE.search(base_sdf)
    if not match:
        raise BuildError("the base world does not include model://iris_downcam with a pose")
    return float(match.group(1))


# --- sequence and params ---------------------------------------------------------


def build_sequence(scenario: Scenario, shipped: dict, objects: list[dict], site: site_file.Site, seed: int, flags_on: bool, run_dir: Path):
    """The sequence to fly, the waypoint file's content (or None) and the truth it implies."""

    sequence = copy.deepcopy(shipped)
    root = sequence.get("mission_sequence", sequence)
    entries = {entry.get("name"): entry for entry in root.get("missions", [])}
    by_name = {item["name"]: item for item in objects}
    truth: dict = {}

    def entry_for(name: str, what: str) -> dict:
        if name not in entries:
            raise BuildError(f"{what}: the sequence {scenario.sequence} has no mission named '{name}' (it has: {', '.join(map(str, entries))})")
        return entries[name]

    for mission_name, overrides in scenario.config.items():
        config = entry_for(mission_name, "config").setdefault("config", {})
        if not isinstance(overrides, dict):
            raise BuildError(f"config: '{mission_name}' must be a mapping of settings")
        for key, value in overrides.items():
            if key not in config:
                raise BuildError(f"config: mission '{mission_name}' has no setting '{key}' in {scenario.sequence}")
            config[key] = value

    error_east, error_north = coordinate_error(scenario, seed)
    lat0, lon0 = site.origin.latitude_deg, site.origin.longitude_deg
    for mission_name, object_name in scenario.targets.items():
        config = entry_for(mission_name, "targets").get("config") or {}
        if "target_latitude" not in config or "target_longitude" not in config:
            raise BuildError(f"targets: mission '{mission_name}' takes no target_latitude and target_longitude")
        if object_name not in by_name:
            raise BuildError(f"targets: there is no object named '{object_name}'")
        target = by_name[object_name]
        config["target_latitude"], config["target_longitude"] = latlon_for(lat0, lon0, target["n"] + error_north, target["e"] + error_east)
        truth.setdefault("target", {"name": object_name, "e": target["e"], "n": target["n"]})

    route = None
    if scenario.waypoints is not None:
        waypoint_entry = entry_for(scenario.waypoints["mission"], "waypoints")
        config = waypoint_entry.get("config") or {}
        # A survey mission plans its own tracks: the scenario's points are what the judge expects it to pass,
        # and no route is written for it. Every other mission is given the points as its waypoint file.
        plans_its_own = waypoint_entry.get("type") == "search_pattern"
        if "waypoint_file" not in config and not plans_its_own:
            raise BuildError(f"waypoints: mission '{scenario.waypoints['mission']}' takes no waypoint_file")
        route, truth["waypoints"] = [], []
        for index, point in enumerate(scenario.waypoints["points"]):
            name = str(point.get("name", f"wp{index + 1}"))
            east, north, height = float(point["east_m"]), float(point["north_m"]), float(point["height_m"])
            if not point_in_polygon(east, north, site.boundary):
                raise BuildError(f"waypoint '{name}' is outside the flight boundary")
            lat, lon = latlon_for(lat0, lon0, north, east)
            route.append({"name": name, "latitude": lat, "longitude": lon, "altitude_m": height})
            truth["waypoints"].append({"name": name, "e": east, "n": north, "u": height})
        if plans_its_own:
            route = None
        else:
            config["waypoint_file"] = str(run_dir / "waypoints.yaml")

    if scenario.survey is not None:
        config = entry_for(scenario.survey["mission"], "survey").setdefault("config", {})
        if "search_area" not in config:
            raise BuildError(f"survey: mission '{scenario.survey['mission']}' takes no search_area")
        corners, truth["survey_area"] = [], []
        for index, corner in enumerate(scenario.survey["area"]):
            east, north = float(corner["east_m"]), float(corner["north_m"])
            if not point_in_polygon(east, north, site.boundary):
                raise BuildError(f"survey: corner {index + 1} of the area is outside the flight boundary")
            corners.append(list(latlon_for(lat0, lon0, north, east)))
            truth["survey_area"].append([east, north])  # as the site's own survey area is written: the judge reads either
        config["search_area"] = corners
        if scenario.survey.get("boundary", False):
            config["flight_boundary"] = [[latitude, longitude] for latitude, longitude in site.boundary_geo]

    features = root.get("features") or {}
    for flag, value in scenario.flags.items():
        if flag not in features:
            raise BuildError(f"flag '{flag}' is not declared under features: in the shipped sequence {scenario.sequence}")
        if not isinstance(value, bool):
            raise BuildError(f"flag '{flag}' must be true or false, got {value!r}")  # the executor refuses anything else
    if flags_on:
        root["features"] = {**features, **scenario.flags}
    return sequence, route, truth, {"east_m": error_east, "north_m": error_north}


def build_params(scenario: Scenario, shipped: dict, flags_on: bool) -> dict:
    params = copy.deepcopy(shipped)
    for node, flags in scenario.node_flags.items():
        declared = ((shipped.get(node) or {}).get("ros__parameters") or {}).get("features") or {}
        for flag, value in (flags or {}).items():
            if flag not in declared:
                raise BuildError(f"node flag '{node}.features.{flag}' is not declared in the shipped params file {scenario.params}")
            if flags_on:
                params[node]["ros__parameters"]["features"][flag] = value
    return params


# --- the run folder ------------------------------------------------------------------


def _driver(scenario: Scenario, site_path: Path, run_dir: Path) -> dict | None:
    """The command the runner starts in place of a mission launch."""
    if scenario.driver is None:
        return None
    settings = {key: value for key, value in scenario.driver.items() if key != "tool"}
    tool = scenario.driver["tool"]
    # The drivers talk MAVLink with pymavlink, which lives in the base image's user site.
    command = ["env", "-u", "PYTHONNOUSERSITE", "python3", str(SIM_DIR / f"{tool}.py"), "--site", str(site_path)]
    if tool == "hover":
        command += ["--height", str(float(settings.get("height_m", 10.0))), "--hold", str(float(settings.get("hold_s", 40.0)))]
    elif tool == "recovery_check":
        # It flies beside the payload bridge, which has the tools port: sim/container/start.sh gives it this one.
        command += ["--run-dir", str(run_dir), "--height", str(float(settings.get("height_m", 5.0))), "--lift", str(float(settings.get("lift_m", 3.0)))]
        command += ["--return-point", str({**default_settings(scenario.mission), **(scenario.judge or {})}.get("return_point", "takeoff"))]  # the judge's
        if "carry_speed_mps" in settings:
            command += ["--carry-speed", str(float(settings["carry_speed_mps"]))]
        if settings.get("shaped") is True:
            command += ["--shaped"]
        if settings.get("attach_check") is True:
            command += ["--attach-check"]
        command += ["--url", f"udpin:127.0.0.1:{DRIVER_PORT}", "--out", str(run_dir / f"{tool}.json")]
        return {"tool": tool, "command": command, "port": DRIVER_PORT}
    else:
        command += ["--speed", str(float(settings.get("speed_mps", 5.0))), "--height", str(float(settings.get("height_m", 15.0)))]
    command += ["--out", str(run_dir / f"{tool}.json")]
    return {"tool": tool, "command": command}


def build_run(scenario_path, out_dir=None, seed: int | None = None, variant: str = "off", dry_run: bool = False, record: bool = False) -> Path:
    import yaml

    if variant not in ("off", "on"):
        raise BuildError(f"variant must be 'off' or 'on', got {variant!r}")
    scenario = load_scenario(scenario_path)
    seed = scenario.seed if seed is None else int(seed)
    flags_on = variant == "on"
    run_dir = Path(out_dir) if out_dir is not None else default_run_dir(scenario, seed, variant)
    run_dir = run_dir.resolve()

    if scenario.mission not in MISSIONS:
        raise BuildError(f"{scenario.path}: no judge for mission '{scenario.mission}' (judges: {', '.join(MISSIONS)})")
    known = default_settings(scenario.mission)
    for name in scenario.judge:
        if name not in known:
            raise BuildError(f"{scenario.path}: the {scenario.mission} judge has no setting '{name}'")

    site_path = SITES_DIR / f"{scenario.site}.yaml"
    if not site_path.is_file():
        raise BuildError(f"{scenario.path}: no site '{scenario.site}' in {SITES_DIR}")
    try:
        site = site_file.load_site(site_path)
    except site_file.SiteError as error:
        raise BuildError(str(error)) from error

    objects = place_objects(scenario, site, seed)
    if scenario.driver is not None:
        # A tool flies the vehicle: there is no mission sequence to generate.
        sequence, params, route, mission_truth, error, launch = None, None, None, {}, {"east_m": 0.0, "north_m": 0.0}, None
    else:
        shipped_sequence = _load_yaml(CONFIG_DIR / "sequences" / scenario.sequence, "sequence")
        shipped_params = _load_yaml(CONFIG_DIR / "params" / scenario.params, "params file")
        launch = scenario.launch or f"{Path(scenario.sequence).stem}.launch.py"
        if scenario.launch is None and not (LAUNCH_DIR / launch).is_file():
            launch = FALLBACK_LAUNCH
        if not (LAUNCH_DIR / launch).is_file():
            raise BuildError(f"{scenario.path}: launch file '{launch}' does not exist in {LAUNCH_DIR}")
        sequence, route, mission_truth, error = build_sequence(scenario, shipped_sequence, objects, site, seed, flags_on, run_dir)
        params = build_params(scenario, shipped_params, flags_on)
    base_world = BASE_WORLD.read_text()
    wind = scenario.wind or {}
    windy = bool(wind.get("mean_mps") or wind.get("gust_mps") or wind.get("discrete_gust_mps"))
    profile = SENSOR_PROFILES[scenario.sensor_profile]
    # A carried payload hangs under the vehicle where the base world stands it; it and the mission's bucket get
    # a detachable joint to the vehicle, which sim/payload_bridge.py makes and unmakes from the servo outputs.
    carried = [item["name"] for item in objects if item.get("carried") is True]
    if len(carried) > 1 or (carried and carried[0] != scenario.payload):
        raise BuildError(f"{scenario.path}: one payload can be carried, and it is the scenario's 'payload' ({', '.join(carried)} are marked carried)")
    if carried:
        stood = vehicle_pose(base_world, BASE_VEHICLE_MODEL)
        objects = [carry(item, stood) if item["name"] == carried[0] else item for item in objects]
    hung = tuple(name for name in (carried[0] if carried else None, scenario.bucket) if name)
    hook = profile.get("hook")
    world = build_world(base_world, site, objects, scenario.sun, windy, profile["model"], hung, hook["link"] if hook else None)

    # The localisation node's oracle detector reports the run's true objects: it is told which and how large.
    oracle = None
    if flags_on and params is not None and (scenario.node_flags.get("loc_node") or {}).get("sim_oracle_detector") is True:
        if not scenario.truth_poses:
            raise BuildError(f"{scenario.path}: the oracle detector reports the true poses on a topic: the scenario needs truth_poses: true")
        oracle = oracle_objects(objects, profile["model"])
        params["loc_node"]["ros__parameters"]["oracle_objects_file"] = str(run_dir / "oracle_objects.json")
        params["loc_node"]["ros__parameters"]["oracle_seed"] = int(seed)
    # The localisation node's judges' list is written in the run folder, where the runner watches for it.
    list_file = scenario.list_file
    if flags_on and params is not None and (scenario.node_flags.get("loc_node") or {}).get("judges_list") is True:
        params["loc_node"]["ros__parameters"]["judges_list_dir"] = str(run_dir / "list")
        if list_file is None:
            list_file = str(run_dir / "list" / "judges_list.csv")
    bridge = None
    driver_tool = None if scenario.driver is None else scenario.driver["tool"]
    if driver_tool == "recovery_check" and not (scenario.bucket and hook):
        raise BuildError(f"{scenario.path}: the recovery check grabs the scenario's bucket with the recovery vehicle's hook: it needs `bucket:` and sensor_profile: recovery")
    if hung:
        if scenario.driver is not None and driver_tool != "recovery_check":
            raise BuildError(f"{scenario.path}: a driver flies over the sim's tools port, which the payload bridge needs: no carried payload or bucket in a driver scenario")
        bridge = {"carried": carried[0] if carried else None, "bucket": scenario.bucket, "capture_half_width_m": payload_bridge.CAPTURE_HALF_WIDTH_M}
        if hook:
            bridge["hook_offset_m"] = list(hook["offset_m"])  # the hook's tip under the vehicle, in place of the bridge's default
        if scenario.grab_refusals:
            bridge["refuse_grabs"] = int(scenario.grab_refusals)  # a latch that does not close, for the mission's attach check to find

    truth = {**site_file.truth_block(site), **mission_truth, "scenario": scenario.name}
    truth["objects"] = [{key: value for key, value in item.items() if key not in ("up", "yaw_rad")} for item in objects]
    truth["markers"] = [
        {"name": item["name"], "id": item["digit"], "e": item["e"], "n": item["n"]}
        for item in objects
        if item["kind"] == "marker" and item["digit"] is not None
    ]
    # Only the object the scenario names is the mission's payload or bucket.
    # Anything else of that kind is just on the field, and the judge must not score it.
    for kind, name, default_mass in (("bucket", scenario.bucket, 1.5), ("payload", scenario.payload, 0.5)):
        if name is not None:
            found = next(item for item in objects if item["name"] == name)
            truth[kind] = {"name": name, "mass_kg": float(found.get("mass_kg", default_mass)), "e": found["e"], "n": found["n"]}

    root = {} if sequence is None else sequence.get("mission_sequence", sequence)
    sim_time = scenario.sim_time and flags_on
    launch_arguments = {"sequence": str(run_dir / "sequence.yaml"), "params": str(run_dir / "params.yaml")}
    if sim_time:
        launch_arguments["use_sim_time"] = "true"
    if flags_on:
        # A node that exists only for its flag is not in the 2026 launch: the launch starts it when asked to.
        for node in FLAG_STARTED_NODES:
            if (scenario.node_flags.get(node) or {}).get(node) is True:
                launch_arguments[node] = "true"
    manifest = {
        "name": scenario.name,
        "mission": scenario.mission,
        "scenario_file": str(Path(scenario.path).resolve()),
        "site": scenario.site,
        "site_file": str(site_path),
        "seed": seed,
        "variant": variant,
        "flags": dict(scenario.flags) if flags_on else {},
        "node_flags": copy.deepcopy(scenario.node_flags) if flags_on else {},
        "sim_time": sim_time,
        "record": bool(record),  # true: the flight's topics are recorded for replay (sim/replay/record_run.py)
        "payload_bridge": bridge,  # what sim/payload_bridge.py lets go and takes hold of; None when there is nothing
        "timeout_s": scenario.timeout_s,
        "launch": None
        if launch is None
        else {
            "package": "drone_mission_demo",
            "file": launch,
            "arguments": launch_arguments,
        },
        "driver": _driver(scenario, site_path, run_dir),
        "vehicle": {
            "model": profile["model"],
            "bridge": str(run_dir / "bridge.yaml"),
            "sensor_profile": scenario.sensor_profile,
            "mass_kg": profile["mass_kg"],
            "cda_m2": profile["cda_m2"],
            "min_rate_hz": profile["min_rate_hz"],  # the frame rate the sensor check holds this profile's camera to, or None
            "min_real_time_factor": MIN_REAL_TIME_FACTOR if profile["min_rate_hz"] is not None else None,
            "flow_camera": profile["flow_camera"],
            "headless_only": profile["headless_only"],  # sim/container/start.sh refuses to open a window on it
        },
        "sensor_check": scenario.sensor_check,  # true: sim/sensor_check.py runs beside the flight, which must pass it
        "truth_poses": scenario.truth_poses,  # true: sim/truth_poses.py publishes true poses during the flight
        "dds_profile": scenario.dds_profile,  # a Fast DDS profile for the container (sim/container/start.sh), or None
        # Text the flight's launch.log must contain (sim/runner/result.py). The lines are the flags-on mission's.
        "expect_log": list(scenario.expect_log) if flags_on else [],
        "target": "target" in truth,  # false: nothing for the projection check to compare with (sim/projection_check.py)
        "expected_missions": [str(entry.get("name")) for entry in root.get("missions", [])],
        "wind": scenario.wind,
        "gps_events": scenario.gps_events,
        "coordinate_error": error,
        "sun": scenario.sun,
        "list_file": list_file,
    }
    if dry_run:
        return run_dir

    header = f"# Generated by sim/scenario/build.py from {Path(scenario.path).name} (seed {seed}, variant {variant}). Do not edit.\n"
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "world.sdf").write_text(world)
    (run_dir / "bridge.yaml").write_text((SIM_DIR / profile["bridge"]).read_text())
    # With something to let go or take hold of, the flight controller gets the sim's gripper and sprayer outputs.
    # A flight of the recovery vehicle gets the recovery parameters (sim/params/recovery.parm); the scenario's own come last.
    recovery = read_parm(RECOVERY_PARM) if scenario.sensor_profile == "recovery" else {}
    sitl = {**SITL_BASE_PARAMS, **(payload_bridge.SITL_PARAMS if bridge else {}), **recovery, **scenario.sitl_params}
    (run_dir / "sitl.parm").write_text("".join(f"{name} {value:g}\n" for name, value in sitl.items()))
    if sequence is not None:
        (run_dir / "sequence.yaml").write_text(header + yaml.safe_dump(sequence, sort_keys=False))
        (run_dir / "params.yaml").write_text(header + yaml.safe_dump(params, sort_keys=False))
    if oracle is not None:
        (run_dir / "oracle_objects.json").write_text(json.dumps(oracle, indent=2) + "\n")
    if route is not None:
        (run_dir / "waypoints.yaml").write_text(header + yaml.safe_dump({"waypoints": route}, sort_keys=False))
    (run_dir / "truth.json").write_text(json.dumps(truth, indent=2) + "\n")
    judge = {"mission": scenario.mission, "settings": dict(scenario.judge), "vehicle_ground_offset_m": vehicle_ground_offset_m(base_world)}
    (run_dir / "judge.yaml").write_text(header + yaml.safe_dump(judge, sort_keys=False))
    (run_dir / "scenario.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return run_dir


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build a scenario into a run folder.")
    parser.add_argument("scenario", type=Path)
    parser.add_argument("--seed", type=int, help="override the scenario's seed")
    parser.add_argument("--variant", choices=("off", "on"), default="off", help="off: every flag false (2026); on: the scenario's flags")
    parser.add_argument("--out", type=Path, help="run folder (default: sim/.state/runs/<mission>-<name>-s<seed>-<variant>)")
    parser.add_argument("--dry-run", action="store_true", help="check the scenario and print the run folder, write nothing")
    parser.add_argument("--record", action="store_true", help="record the flight's topics into <run folder>/bag for replay (SIM-09)")
    args = parser.parse_args(argv)
    try:
        run_dir = build_run(args.scenario, args.out, seed=args.seed, variant=args.variant, dry_run=args.dry_run, record=args.record)
    except BuildError as error:
        print(f"scenario: {error}", file=sys.stderr)
        return 2
    try:
        shown = run_dir.relative_to(REPO)
    except ValueError:
        shown = run_dir
    print(f"{'would build' if args.dry_run else 'built'} {shown}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
