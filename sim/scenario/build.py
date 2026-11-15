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
  scenario.json  what the runner needs: launch, timeout, wind, GPS events

Variant "off" leaves every flag false: that is the 2026 behaviour. Flags are
only ever switched on in these generated copies; a flag that no shipped file
declares fails the build, as does a setting or a config key nothing reads.
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
from judge.geometry import point_in_polygon  # noqa: E402
from target_placement import latlon_for  # noqa: E402

if __package__ in (None, ""):
    from scenario.spec import SENSOR_PROFILES, BuildError, Scenario, load_scenario  # noqa: E402
else:
    from .spec import SENSOR_PROFILES, BuildError, Scenario, load_scenario  # noqa: E402

CONFIG_DIR = REPO / "src" / "drone_mission_demo" / "config"
LAUNCH_DIR = REPO / "src" / "drone_mission_demo" / "launch"
SITES_DIR = SIM_DIR / "sites"
BASE_WORLD = SIM_DIR / "worlds" / "argus_runway.sdf"
RUNS_DIR = SIM_DIR / ".state" / "runs"
FALLBACK_LAUNCH = "multi_mission_demo.launch.py"

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
        if "on_top_of" in item:
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


def _embedded_model(item: dict) -> str:
    """The object's model as a child of <world>, with its pose. Heading is a compass heading."""
    model = ET.fromstring(_object_sdf(item)).find("model")
    pose = ET.Element("pose")
    pose.text = f"{item['e']:.4f} {item['n']:.4f} {item['up']:.4f} 0 0 {math.radians(-item['heading_deg']):.6f}"
    model.insert(0, pose)
    return "    " + ET.tostring(model, encoding="unicode") + "\n"


_SUN_DIRECTION = re.compile(r'(<light type="directional" name="sun">.*?<direction>)[^<]*(</direction>)', re.DOTALL)
_VEHICLE_POSE = re.compile(r'<uri>model://iris_downcam</uri>\s*<pose degrees="true">\s*\S+\s+\S+\s+(\S+)')


def build_world(base_sdf: str, site: site_file.Site, objects: list[dict], sun: dict | None) -> str:
    text = site_file.site_world(base_sdf, site)
    if sun is not None:  # otherwise the base world's light, as in the 2026 sim
        elevation, azimuth = math.radians(sun["elevation_deg"]), math.radians(sun["azimuth_deg"])
        # The light travels from the sun: opposite to the direction the sun is seen in.
        direction = (-math.cos(elevation) * math.sin(azimuth), -math.cos(elevation) * math.cos(azimuth), -math.sin(elevation))
        text, found = _SUN_DIRECTION.subn(lambda m: f"{m.group(1)}{direction[0]:.6f} {direction[1]:.6f} {direction[2]:.6f}{m.group(2)}", text, count=1)
        if not found:
            raise BuildError("the base world has no directional light named 'sun'")
    models = "".join(_embedded_model(item) for item in objects)
    head, closing, tail = text.rpartition("</world>")
    if not closing:
        raise BuildError("the base world has no </world>")
    return f"{head}{models}  {closing}{tail}"


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
        config = entry_for(scenario.waypoints["mission"], "waypoints").get("config") or {}
        if "waypoint_file" not in config:
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
        config["waypoint_file"] = str(run_dir / "waypoints.yaml")

    features = root.get("features") or {}
    for flag in scenario.flags:
        if flag not in features:
            raise BuildError(f"flag '{flag}' is not declared under features: in the shipped sequence {scenario.sequence}")
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
    # fence_check talks MAVLink with pymavlink, which lives in the base image's user site.
    command = ["env", "-u", "PYTHONNOUSERSITE", "python3", str(SIM_DIR / "fence_check.py"), "--site", str(site_path)]
    command += ["--speed", str(float(settings.get("speed_mps", 5.0))), "--height", str(float(settings.get("height_m", 15.0)))]
    command += ["--out", str(run_dir / "fence_check.json")]
    return {"tool": scenario.driver["tool"], "command": command}


def build_run(scenario_path, out_dir=None, seed: int | None = None, variant: str = "off", dry_run: bool = False) -> Path:
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
    world = build_world(base_world, site, objects, scenario.sun)
    profile = SENSOR_PROFILES[scenario.sensor_profile]

    truth = {**site_file.truth_block(site), **mission_truth, "scenario": scenario.name}
    truth["objects"] = [{key: value for key, value in item.items() if key != "up"} for item in objects]
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
        "timeout_s": scenario.timeout_s,
        "launch": None
        if launch is None
        else {
            "package": "drone_mission_demo",
            "file": launch,
            "arguments": {"sequence": str(run_dir / "sequence.yaml"), "params": str(run_dir / "params.yaml")},
        },
        "driver": _driver(scenario, site_path, run_dir),
        "vehicle": {"model": profile["model"], "bridge": str(run_dir / "bridge.yaml"), "sensor_profile": scenario.sensor_profile},
        "expected_missions": [str(entry.get("name")) for entry in root.get("missions", [])],
        "wind": scenario.wind,
        "gps_events": scenario.gps_events,
        "coordinate_error": error,
        "sun": scenario.sun,
        "list_file": scenario.list_file,
    }
    if dry_run:
        return run_dir

    header = f"# Generated by sim/scenario/build.py from {Path(scenario.path).name} (seed {seed}, variant {variant}). Do not edit.\n"
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "world.sdf").write_text(world)
    (run_dir / "bridge.yaml").write_text((SIM_DIR / profile["bridge"]).read_text())
    sitl = {"SIM_TERRAIN": 0, **scenario.sitl_params}
    (run_dir / "sitl.parm").write_text("".join(f"{name} {value:g}\n" for name, value in sitl.items()))
    if sequence is not None:
        (run_dir / "sequence.yaml").write_text(header + yaml.safe_dump(sequence, sort_keys=False))
        (run_dir / "params.yaml").write_text(header + yaml.safe_dump(params, sort_keys=False))
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
    args = parser.parse_args(argv)
    try:
        run_dir = build_run(args.scenario, args.out, seed=args.seed, variant=args.variant, dry_run=args.dry_run)
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
