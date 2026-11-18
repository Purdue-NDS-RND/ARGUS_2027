"""Read and check a scenario file. No ROS, no Gazebo.

Checks that need other files (the site, the shipped sequence, the judge's
settings) happen in build.py; here the scenario's own shape is checked, so a
typo is an error that names the key and never a silently ignored setting.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

KEYS = {
    "description", "mission", "site", "seed", "timeout_s", "sequence", "params", "launch",
    "config", "targets", "payload", "bucket", "coordinate_error", "flags", "node_flags", "objects", "waypoints",
    "wind", "gps_events", "sun", "sensor_profile", "sitl_params", "judge", "list_file", "driver", "sim_time",
}  # fmt: skip
REQUIRED = ("mission", "site")

# Tools that fly the vehicle themselves, in place of a mission launch, with
# the settings each takes. fence_check: sim/fence_check.py.
DRIVERS = {"fence_check": {"speed_mps", "height_m"}, "hover": {"height_m", "hold_s"}}

OBJECT_KINDS = {
    "bullseye": {"size_m", "rings"},
    "marker": {"digit", "size_m"},
    "x_mat": {"size_m", "arm_width_m"},
    "bucket": {"mass_kg", "diameter_m", "height_m"},
    "payload": {"mass_kg", "size_m"},
}
OBJECT_COMMON = {"name", "kind", "at", "on_top_of", "carried", "heading_deg"}

# SIM-03's wind levels: mean speed, gust and one discrete gust, in m/s.
WIND_LEVELS = {
    "calm": (0.0, 0.0, 0.0),
    "moderate": (4.0, 1.0, 0.0),
    "strong": (7.0, 2.0, 3.0),
}
WIND_KEYS = {"level", "mean_mps", "gust_mps", "discrete_gust_mps", "direction_deg"}

# Camera and vehicle variants. SIM-05 adds the survey and still profiles.
# mass_kg: the Iris's links as the sim's models give them (base 1.5, IMU 0.15, four rotors of 0.025, the camera
# 0.005). cda_m2: its drag area for the wind (SIM-03), 0.08 m2 until the hover test on our airframe (DAT-10).
SENSOR_PROFILES = {"delivery": {"model": "iris_downcam", "bridge": "config/bridge.yaml", "mass_kg": 1.755, "cda_m2": 0.08}}

COORDINATE_ERROR_FORMS = ({"east_m", "north_m"}, {"radius_m"}, {"max_radius_m"})


class BuildError(ValueError):
    """A scenario that cannot be built. The message names what is wrong."""


@dataclass(frozen=True)
class Scenario:
    path: Path
    name: str
    mission: str
    site: str
    seed: int
    timeout_s: float
    sequence: str | None  # None when a driver flies the vehicle
    params: str | None
    launch: str | None
    config: dict
    targets: dict
    payload: str | None  # the object the mission carries; other payload-kind objects are just things on the field
    bucket: str | None  # the object recovery must bring back
    coordinate_error: dict
    flags: dict
    node_flags: dict
    objects: list
    waypoints: dict | None
    wind: dict
    gps_events: list
    sun: dict | None  # None: the base world's light, as in the 2026 sim
    sensor_profile: str
    sitl_params: dict
    judge: dict
    list_file: str | None = None  # where the mission writes its judges' list (localisation)
    driver: dict | None = None  # {"tool": ..., settings}: flies the vehicle in place of a mission launch
    sim_time: bool = False  # variant on runs the whole sim on Gazebo's clock (FND-03); off is wall time, as in 2026
    description: str = ""
    extra: dict = field(default_factory=dict)


def _mapping(data: dict, key: str, where: str) -> dict:
    value = data.get(key) or {}
    if not isinstance(value, dict):
        raise BuildError(f"{where}: {key} must be a mapping")
    return value


def _check_object(item, index: int, where: str) -> dict:
    if not isinstance(item, dict):
        raise BuildError(f"{where}: objects[{index}] must be a mapping")
    name, kind = item.get("name"), item.get("kind")
    if not isinstance(name, str) or not name:
        raise BuildError(f"{where}: objects[{index}] has no name")
    if kind not in OBJECT_KINDS:
        raise BuildError(f"{where}: object '{name}' has unknown kind {kind!r} (kinds: {', '.join(sorted(OBJECT_KINDS))})")
    unknown = sorted(str(key) for key in set(item) - OBJECT_COMMON - OBJECT_KINDS[kind])
    if unknown:
        raise BuildError(f"{where}: object '{name}' ({kind}) has unknown key(s) {', '.join(unknown)}")
    places = [key for key in ("at", "on_top_of", "carried") if key in item]
    if len(places) != 1:
        raise BuildError(f"{where}: object '{name}' needs exactly one of 'at', 'on_top_of' and 'carried'")
    if "carried" in item and (item["carried"] is not True or kind != "payload"):
        raise BuildError(f"{where}: object '{name}': only a payload can be carried, and it is 'carried: true'")
    if "at" in item:
        at = item["at"]
        fixed = isinstance(at, dict) and set(at) == {"east_m", "north_m"}
        random = isinstance(at, dict) and set(at) == {"random"} and isinstance(at["random"], dict)
        if random:
            extra = sorted(set(at["random"]) - {"area", "min_separation_m", "margin_m"})
            if extra or "area" not in at["random"]:
                raise BuildError(f"{where}: object '{name}': random placement takes area, min_separation_m and margin_m")
        elif not fixed:
            raise BuildError(f"{where}: object '{name}': 'at' is {{east_m, north_m}} or {{random: {{area: ...}}}}")
    return item


def _role(data: dict, kind: str, objects: list, where: str) -> str | None:
    """The object named as the mission's payload or bucket; it must be one of that kind."""
    name = data.get(kind)
    if name is None:
        return None
    match = next((item for item in objects if item["name"] == name), None)
    if match is None:
        raise BuildError(f"{where}: {kind}: there is no object named '{name}'")
    if match["kind"] != kind:
        raise BuildError(f"{where}: {kind}: object '{name}' is a {match['kind']}, not a {kind}")
    return str(name)


def _check_wind(wind: dict, where: str) -> dict:
    unknown = sorted(set(wind) - WIND_KEYS)
    if unknown:
        raise BuildError(f"{where}: wind has unknown key(s) {', '.join(unknown)}")
    level = wind.get("level", "calm")
    if level not in WIND_LEVELS:
        raise BuildError(f"{where}: unknown wind level '{level}' (levels: {', '.join(WIND_LEVELS)})")
    mean, gust, discrete = WIND_LEVELS[level]
    return {
        "level": level,
        "mean_mps": float(wind.get("mean_mps", mean)),
        "gust_mps": float(wind.get("gust_mps", gust)),
        "discrete_gust_mps": float(wind.get("discrete_gust_mps", discrete)),
        "direction_deg": float(wind.get("direction_deg", 0.0)),
    }


def parse_scenario(data, path: Path) -> Scenario:
    where = str(path)
    if not isinstance(data, dict):
        raise BuildError(f"{where}: expected a mapping")
    unknown = sorted(str(key) for key in set(data) - KEYS)
    if unknown:
        raise BuildError(f"{where}: unknown key(s) {', '.join(unknown)}")
    for key in REQUIRED:
        if not data.get(key):
            raise BuildError(f"{where}: missing '{key}'")

    driver = data.get("driver")
    mission_keys = ("sequence", "params", "launch", "config", "targets", "flags", "node_flags", "waypoints")
    if driver is None:
        for key in ("sequence", "params"):
            if not data.get(key):
                raise BuildError(f"{where}: missing '{key}'")
    else:
        if not isinstance(driver, dict) or driver.get("tool") not in DRIVERS:
            raise BuildError(f"{where}: driver needs a tool, one of: {', '.join(DRIVERS)}")
        extra = sorted(str(key) for key in set(driver) - {"tool"} - DRIVERS[driver["tool"]])
        if extra:
            raise BuildError(f"{where}: driver {driver['tool']} has unknown setting(s) {', '.join(extra)}")
        used = [key for key in mission_keys if data.get(key)]
        if used:
            raise BuildError(f"{where}: a scenario with a driver flies no mission sequence, so it cannot have {', '.join(used)}")

    objects = data.get("objects") or []
    if not isinstance(objects, list):
        raise BuildError(f"{where}: objects must be a list")
    objects = [_check_object(item, index, where) for index, item in enumerate(objects)]
    names = [item["name"] for item in objects]
    for name in names:
        if names.count(name) > 1:
            raise BuildError(f"{where}: two objects are named '{name}'")

    coordinate_error = _mapping(data, "coordinate_error", where)
    if coordinate_error and set(coordinate_error) not in COORDINATE_ERROR_FORMS:
        raise BuildError(f"{where}: coordinate_error is {{east_m, north_m}}, {{radius_m}} or {{max_radius_m}}, got {sorted(coordinate_error)}")

    gps_events = data.get("gps_events") or []
    if not isinstance(gps_events, list):
        raise BuildError(f"{where}: gps_events must be a list")
    for index, event in enumerate(gps_events):
        if not isinstance(event, dict) or "at_s" not in event or "kind" not in event:
            raise BuildError(f"{where}: gps_events[{index}] needs at_s and kind")

    sun = _mapping(data, "sun", where)
    if set(sun) - {"elevation_deg", "azimuth_deg"}:
        raise BuildError(f"{where}: sun takes elevation_deg and azimuth_deg")

    waypoints = data.get("waypoints")
    if waypoints is not None:
        if not isinstance(waypoints, dict) or set(waypoints) != {"mission", "points"} or not isinstance(waypoints["points"], list):
            raise BuildError(f"{where}: waypoints is {{mission: <name in the sequence>, points: [...]}}")
        for index, point in enumerate(waypoints["points"]):
            if not isinstance(point, dict) or not {"east_m", "north_m", "height_m"} <= set(point):
                raise BuildError(f"{where}: waypoints.points[{index}] needs east_m, north_m and height_m")

    sim_time = data.get("sim_time", False)
    if not isinstance(sim_time, bool):
        raise BuildError(f"{where}: sim_time must be true or false, got {sim_time!r}")

    sensor_profile = data.get("sensor_profile", "delivery")
    if sensor_profile not in SENSOR_PROFILES:
        raise BuildError(f"{where}: sensor profile '{sensor_profile}' does not exist (profiles: {', '.join(SENSOR_PROFILES)})")

    return Scenario(
        path=path,
        name=path.stem,
        mission=str(data["mission"]),
        site=str(data["site"]),
        seed=int(data.get("seed", 1)),
        timeout_s=float(data.get("timeout_s", 600.0)),
        sequence=None if driver is not None else str(data["sequence"]),
        params=None if driver is not None else str(data["params"]),
        launch=data.get("launch"),
        config=_mapping(data, "config", where),
        targets=_mapping(data, "targets", where),
        payload=_role(data, "payload", objects, where),
        bucket=_role(data, "bucket", objects, where),
        coordinate_error=coordinate_error,
        flags=_mapping(data, "flags", where),
        node_flags=_mapping(data, "node_flags", where),
        objects=objects,
        waypoints=waypoints,
        wind=_check_wind(_mapping(data, "wind", where), where),
        gps_events=gps_events,
        sun={"elevation_deg": float(sun.get("elevation_deg", 60.0)), "azimuth_deg": float(sun.get("azimuth_deg", 180.0))} if sun else None,
        sensor_profile=sensor_profile,
        sitl_params=_mapping(data, "sitl_params", where),
        judge=_mapping(data, "judge", where),
        list_file=None if data.get("list_file") is None else str(data["list_file"]),
        driver=driver,
        sim_time=sim_time,
        description=str(data.get("description", "")),
    )


def load_scenario(path) -> Scenario:
    import yaml

    path = Path(path)
    try:
        data = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as error:
        raise BuildError(f"{path}: {error}") from error
    return parse_scenario(data, path)
