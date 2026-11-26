"""sim/scenario: scenario files and the builder that turns one into a run folder.

A scenario is one test case. The builder writes everything a run needs: the
world with every object already in it, the sequence the mission flies, SITL
parameters, and the truth and settings the judge scores against. The rules
that matter most: with flags off the mission flies the shipped 2026 sequence
unchanged, a flag is only ever switched on in the generated copy, and the
same seed gives the same world. No ROS needed.
"""

import copy
import hashlib
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

SIM = Path(__file__).resolve().parents[1]
ROOT = SIM.parent
sys.path.insert(0, str(SIM))

import site_file  # noqa: E402
import target_placement as tp  # noqa: E402
from judge.geometry import point_in_polygon  # noqa: E402
from scenario import BuildError, build_run, load_scenario  # noqa: E402
from scenario import build as build_module  # noqa: E402

SCENARIOS = SIM / "scenarios"
SHIPPED = sorted(SCENARIOS.glob("*/*.yaml"))
CONFIG = ROOT / "src" / "drone_mission_demo" / "config"
DELIVERY = SCENARIOS / "delivery" / "baseline.yaml"
CLUTTER = SCENARIOS / "delivery" / "clutter.yaml"
WAYPOINTS = SCENARIOS / "waypoint_navigation" / "site_circuit.yaml"
RUN_FILES = ["world.sdf", "bridge.yaml", "sitl.parm", "sequence.yaml", "params.yaml", "truth.json", "judge.yaml", "scenario.json"]


def read(run: Path, name: str):
    text = (run / name).read_text()
    return json.loads(text) if name.endswith(".json") else yaml.safe_load(text)


def scenario_file(tmp_path: Path, base: Path = DELIVERY, **changes) -> Path:
    data = yaml.safe_load(base.read_text())
    for key, value in changes.items():
        if value is None:
            data.pop(key, None)
        else:
            data[key] = value
    path = tmp_path / "case.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return path


def missions(sequence: dict) -> dict:
    return {entry["name"]: entry for entry in sequence["mission_sequence"]["missions"]}


def models(run: Path) -> dict:
    world = ET.fromstring((run / "world.sdf").read_text()).find("world")
    return {m.get("name"): m for m in world.findall("model")}


def model_pose(model: ET.Element) -> list[float]:
    return [float(v) for v in model.findtext("pose").split()]


def tree_digest(folder: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(folder.rglob("*")):
        if path.is_file() and "__pycache__" not in path.parts:
            digest.update(str(path.relative_to(folder)).encode())
            digest.update(path.read_bytes())
    return digest.hexdigest()


# --- every shipped scenario ----------------------------------------------------


def test_scenarios_are_shipped_one_folder_per_mission():
    assert SHIPPED, "no scenarios in sim/scenarios/<mission>/"
    for path in SHIPPED:
        assert load_scenario(path).mission == path.parent.name


@pytest.mark.parametrize("path", SHIPPED, ids=lambda p: f"{p.parent.name}/{p.stem}")
@pytest.mark.parametrize("variant", ["off", "on"])
def test_every_shipped_scenario_builds_a_complete_run_folder(path, variant, tmp_path):
    run = build_run(path, tmp_path / "run", variant=variant)
    flown_by_a_tool = load_scenario(path).driver is not None
    for name in RUN_FILES:
        if flown_by_a_tool and name in ("sequence.yaml", "params.yaml"):
            assert not (run / name).exists(), name  # no mission sequence is flown
        else:
            assert (run / name).is_file(), name
    ET.fromstring((run / "world.sdf").read_text())
    manifest = read(run, "scenario.json")
    assert manifest["variant"] == variant
    assert manifest["mission"] == path.parent.name


def test_building_never_touches_shipped_config_or_scenarios(tmp_path):
    before = (tree_digest(CONFIG), tree_digest(SCENARIOS), tree_digest(SIM / "sites"), tree_digest(SIM / "worlds"))
    for path in SHIPPED:
        build_run(path, tmp_path / path.stem, variant="on")
    assert (tree_digest(CONFIG), tree_digest(SCENARIOS), tree_digest(SIM / "sites"), tree_digest(SIM / "worlds")) == before


# --- flags off flies the shipped 2026 sequence ----------------------------------


def test_baseline_sequence_is_the_shipped_one_except_for_the_target_coordinates(tmp_path):
    run = build_run(DELIVERY, tmp_path / "run", variant="off")
    built = read(run, "sequence.yaml")
    shipped = yaml.safe_load((CONFIG / "sequences" / "package_delivery_demo.yaml").read_text())
    for sequence in (built, shipped):
        config = missions(sequence)["package_delivery"]["config"]
        config.pop("target_latitude")
        config.pop("target_longitude")
    assert built == shipped
    assert read(run, "params.yaml") == yaml.safe_load((CONFIG / "params" / "sim_target_mission.yaml").read_text())


def test_mission_is_given_the_coordinates_of_its_target_object(tmp_path):
    run = build_run(DELIVERY, tmp_path / "run")
    truth = read(run, "truth.json")
    config = missions(read(run, "sequence.yaml"))["package_delivery"]["config"]
    origin = truth["origin"]
    north, east = tp.offset_ne_m(origin["latitude_deg"], origin["longitude_deg"], config["target_latitude"], config["target_longitude"])
    assert (east, north) == pytest.approx((truth["target"]["e"], truth["target"]["n"]), abs=0.02)
    assert (truth["target"]["e"], truth["target"]["n"]) == pytest.approx((10.0, 15.0))
    assert truth["target"]["name"] == "delivery_target"


# --- the world -------------------------------------------------------------------


def test_world_is_the_site_world_with_every_object_embedded(tmp_path):
    run = build_run(CLUTTER, tmp_path / "run")
    world = ET.fromstring((run / "world.sdf").read_text()).find("world")
    site = site_file.load_site(SIM / "sites" / "apollo_xi.yaml")
    name, origin = tp.read_world((run / "world.sdf").read_text())
    assert name == "apollo_xi"
    assert (origin.latitude_deg, origin.longitude_deg) == pytest.approx((site.origin.latitude_deg, site.origin.longitude_deg))
    assert any(include.findtext("uri") == "model://iris_downcam" for include in world.findall("include"))

    truth = read(run, "truth.json")
    embedded = models(run)
    assert len(truth["objects"]) >= 8
    for item in truth["objects"]:
        model = embedded[item["name"]]
        x, y, z, roll, pitch, yaw = model_pose(model)
        assert (x, y) == pytest.approx((item["e"], item["n"]), abs=1e-3)
        assert (roll, pitch) == (0.0, 0.0)
        assert yaw == pytest.approx(math.radians(-item["heading_deg"]), abs=1e-4)
        assert model.find("link") is not None


def test_clutter_scenario_has_one_of_every_kind_of_object(tmp_path):
    run = build_run(CLUTTER, tmp_path / "run")
    truth = read(run, "truth.json")
    kinds = {item["kind"] for item in truth["objects"]}
    assert kinds == {"bullseye", "marker", "x_mat", "bucket", "payload"}
    embedded = models(run)
    # Markers: the numbered ones are 0.6 m with their digit; boundary markers are 1.2 m without one.
    numbered = [m for m in truth["markers"]]
    assert sorted(m["id"] for m in numbered) == [1, 4, 7]
    for marker in numbered:
        base = embedded[marker["name"]].find("link[@name='base']/visual/geometry/box/size").text.split()
        assert float(base[0]) == pytest.approx(0.6)
    boundary = [item for item in truth["objects"] if item["kind"] == "marker" and item.get("digit") is None]
    assert len(boundary) == 4
    assert all(embedded[item["name"]].find("link[@name='digit_field']") is None for item in boundary)
    # The bucket is dynamic, has the scenario's mass, and stands on the X mat.
    listed = {item["kind"]: item for item in truth["objects"] if item["kind"] in ("bucket", "x_mat", "payload")}
    bucket = embedded[listed["bucket"]["name"]]
    assert bucket.findtext("static") == "false"
    assert float(bucket.findtext("link/inertial/mass")) == pytest.approx(listed["bucket"]["mass_kg"]) == pytest.approx(1.5)
    mat = listed["x_mat"]
    assert (listed["bucket"]["e"], listed["bucket"]["n"]) == pytest.approx((mat["e"], mat["n"]))
    assert model_pose(bucket)[2] > model_pose(embedded[mat["name"]])[2]  # on the mat, not inside it
    assert float(embedded[listed["payload"]["name"]].findtext("link/inertial/mass")) == pytest.approx(0.5)


def score_a_landing_on_the_target(run: Path) -> dict:
    """Score a flight that lands on the target and fakes the release, with the shipped judge."""
    lines = [{"t": 0.0, "event": "armed"}, {"t": 100.0, "event": "disarmed"}, {"t": 60.0, "event": "release", "fake": True}]
    knots = [(0, 0, 0, 0), (10, 0, 0, 15), (30, 10.0, 15.0, 15), (55, 10.0, 15.0, 0), (65, 10.0, 15.0, 0), (75, 10.0, 15.0, 12), (90, 0, 0, 12), (99, 0, 0, 0)]
    for (t0, e0, n0, u0), (t1, e1, n1, u1) in zip(knots, knots[1:]):
        for i in range(20):
            f = i / 20
            lines.append({"t": t0 + (t1 - t0) * f, "pose": "vehicle", "e": e0 + (e1 - e0) * f, "n": n0 + (n1 - n0) * f, "u": 0.195 + u0 + (u1 - u0) * f})
    (run / "trace.jsonl").write_text("".join(json.dumps(line) + "\n" for line in lines))
    result = subprocess.run([sys.executable, str(SIM / "judge" / "cli.py"), str(run)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    return read(run, "score.json")


def test_things_lying_on_the_field_are_not_the_missions_payload_or_bucket(tmp_path):
    """The clutter scenario's loose package and bucket must not earn or lose the mission points."""
    clutter = build_run(CLUTTER, tmp_path / "clutter")
    truth = read(clutter, "truth.json")
    assert "payload" not in truth and "bucket" not in truth
    baseline = build_run(DELIVERY, tmp_path / "baseline")
    with_clutter, without = score_a_landing_on_the_target(clutter), score_a_landing_on_the_target(baseline)
    assert with_clutter["points"] == pytest.approx(without["points"])
    assert with_clutter["details"]["weight_bonus"] == 0.0


def test_scenario_names_its_payload_and_bucket(tmp_path):
    run = build_run(scenario_file(tmp_path, base=CLUTTER, payload="package", bucket="bucket"), tmp_path / "run")
    truth = read(run, "truth.json")
    assert truth["payload"] == {"name": "package", "mass_kg": 0.5, "e": -12.0, "n": 8.0}
    assert truth["bucket"] == {"name": "bucket", "mass_kg": 1.5, "e": -30.0, "n": 40.0}
    assert score_a_landing_on_the_target(run)["details"]["weight_bonus"] == pytest.approx(0.25)


@pytest.mark.parametrize("changes, message", [({"payload": "bucket"}, "not a payload"), ({"bucket": "nothing"}, "nothing"), ({"payload": "delivery_target"}, "bullseye")])
def test_payload_or_bucket_that_is_not_one_is_refused(changes, message, tmp_path):
    with pytest.raises(BuildError, match=message):
        build_run(scenario_file(tmp_path, base=CLUTTER, **changes), tmp_path / "run")


def test_world_keeps_the_2026_light_unless_the_scenario_sets_the_sun(tmp_path):
    base = ET.fromstring((SIM / "worlds" / "argus_runway.sdf").read_text()).find("world/light[@name='sun']/direction").text
    world = ET.fromstring((build_run(DELIVERY, tmp_path / "run") / "world.sdf").read_text()).find("world")
    assert world.find("light[@name='sun']/direction").text == base


def test_sun_direction_follows_the_scenario(tmp_path):
    run = build_run(scenario_file(tmp_path, sun={"elevation_deg": 30.0, "azimuth_deg": 90.0}), tmp_path / "run")
    world = ET.fromstring((run / "world.sdf").read_text()).find("world")
    direction = [float(v) for v in world.find("light[@name='sun']/direction").text.split()]
    # Sun in the east, 30 degrees up: its light travels west and down.
    assert direction == pytest.approx([-math.cos(math.radians(30)), 0.0, -math.sin(math.radians(30))], abs=1e-6)


# --- placement --------------------------------------------------------------------


def placements(run: Path) -> dict:
    return {item["name"]: (item["e"], item["n"]) for item in read(run, "truth.json")["objects"]}


def test_random_placement_is_inside_its_area_apart_and_repeatable(tmp_path):
    first = build_run(CLUTTER, tmp_path / "a", seed=3)
    again = build_run(CLUTTER, tmp_path / "b", seed=3)
    other = build_run(CLUTTER, tmp_path / "c", seed=4)
    assert placements(first) == placements(again)
    assert placements(first) != placements(other)
    assert (first / "world.sdf").read_text() == (again / "world.sdf").read_text()

    truth = read(first, "truth.json")
    markers = truth["markers"]
    for marker in markers:
        assert point_in_polygon(marker["e"], marker["n"], truth["survey_area"])
    for i, a in enumerate(markers):
        for b in markers[i + 1 :]:
            assert math.hypot(a["e"] - b["e"], a["n"] - b["n"]) >= 8.0  # the scenario's min_separation_m
    assert read(first, "scenario.json")["seed"] == 3


def test_seed_defaults_to_the_scenario_file_and_can_be_overridden(tmp_path):
    assert read(build_run(CLUTTER, tmp_path / "a"), "scenario.json")["seed"] == load_scenario(CLUTTER).seed
    assert read(build_run(CLUTTER, tmp_path / "b", seed=99), "scenario.json")["seed"] == 99


def test_placement_that_cannot_fit_is_an_error(tmp_path):
    crowd = [
        {"name": f"m{i}", "kind": "marker", "digit": i % 10, "at": {"random": {"area": "survey_area", "min_separation_m": 60.0}}}
        for i in range(6)
    ]
    path = scenario_file(tmp_path, objects=yaml.safe_load(DELIVERY.read_text())["objects"] + crowd)
    with pytest.raises(BuildError, match="could not place"):
        build_run(path, tmp_path / "run")


def test_object_outside_the_flight_boundary_is_an_error(tmp_path):
    objects = yaml.safe_load(DELIVERY.read_text())["objects"]
    objects[0]["at"] = {"east_m": 400.0, "north_m": 0.0}
    with pytest.raises(BuildError, match="outside the flight boundary"):
        build_run(scenario_file(tmp_path, objects=objects), tmp_path / "run")


# --- error in the coordinates the mission is given -----------------------------------


def given_offset(run: Path) -> tuple[float, float]:
    truth = read(run, "truth.json")
    config = missions(read(run, "sequence.yaml"))["package_delivery"]["config"]
    north, east = tp.offset_ne_m(truth["origin"]["latitude_deg"], truth["origin"]["longitude_deg"], config["target_latitude"], config["target_longitude"])
    return east - truth["target"]["e"], north - truth["target"]["n"]


def test_fixed_coordinate_error_moves_what_the_mission_is_told_not_the_object(tmp_path):
    run = build_run(scenario_file(tmp_path, coordinate_error={"east_m": 3.0, "north_m": -4.0}), tmp_path / "run")
    assert given_offset(run) == pytest.approx((3.0, -4.0), abs=0.02)
    assert placements(run)["delivery_target"] == pytest.approx((10.0, 15.0))
    assert read(run, "scenario.json")["coordinate_error"] == pytest.approx({"east_m": 3.0, "north_m": -4.0})


def test_radius_coordinate_error_is_exactly_that_far_in_a_seeded_direction(tmp_path):
    path = scenario_file(tmp_path, coordinate_error={"radius_m": 6.0})
    offsets = [given_offset(build_run(path, tmp_path / f"s{seed}", seed=seed)) for seed in (1, 1, 2)]
    assert all(math.hypot(e, n) == pytest.approx(6.0, abs=0.02) for e, n in offsets)
    assert offsets[0] == pytest.approx(offsets[1])
    assert offsets[0] != pytest.approx(offsets[2], abs=0.05)


def test_random_coordinate_error_is_within_its_radius_and_follows_the_seed(tmp_path):
    path = scenario_file(tmp_path, coordinate_error={"max_radius_m": 8.0})
    offsets = [given_offset(build_run(path, tmp_path / f"s{seed}", seed=seed)) for seed in (1, 1, 2, 3, 4)]
    assert offsets[0] == pytest.approx(offsets[1])
    assert len({(round(e, 2), round(n, 2)) for e, n in offsets}) == 4
    assert all(math.hypot(e, n) <= 8.0 + 0.02 for e, n in offsets)
    assert any(math.hypot(e, n) > 1.0 for e, n in offsets)


# --- flags -----------------------------------------------------------------------------


def test_flags_reach_only_the_generated_sequence_and_only_in_the_on_variant(tmp_path):
    path = scenario_file(tmp_path, flags={"use_frame_time_pose": True})
    shipped_before = (CONFIG / "sequences" / "package_delivery_demo.yaml").read_bytes()
    off = build_run(path, tmp_path / "off", variant="off")
    on = build_run(path, tmp_path / "on", variant="on")
    shipped_flags = yaml.safe_load(shipped_before)["mission_sequence"]["features"]
    assert shipped_flags["use_frame_time_pose"] is False and not any(shipped_flags.values())
    assert read(off, "sequence.yaml")["mission_sequence"]["features"] == shipped_flags
    assert read(on, "sequence.yaml")["mission_sequence"]["features"] == {**shipped_flags, "use_frame_time_pose": True}
    assert (CONFIG / "sequences" / "package_delivery_demo.yaml").read_bytes() == shipped_before
    assert read(on, "scenario.json")["flags"] == {"use_frame_time_pose": True}
    assert read(off, "scenario.json")["flags"] == {}


def test_flag_that_no_shipped_sequence_has_fails_the_build(tmp_path):
    with pytest.raises(BuildError, match="use_warp_drive"):
        build_run(scenario_file(tmp_path, flags={"use_warp_drive": True}), tmp_path / "run", variant="off")


def test_flag_value_that_is_not_true_or_false_fails_the_build(tmp_path):
    for value in (1, "true"):
        with pytest.raises(BuildError, match="true or false"):
            build_run(scenario_file(tmp_path, flags={"use_frame_time_pose": value}), tmp_path / "run", variant="on")


def test_node_flags_are_checked_against_the_shipped_params_and_set_in_the_copy(tmp_path, monkeypatch):
    with pytest.raises(BuildError, match="target_cv.*ring_fit"):
        build_run(scenario_file(tmp_path, node_flags={"target_cv": {"ring_fit": True}}), tmp_path / "bad")

    config = tmp_path / "config"
    for folder in ("sequences", "params", "waypoints"):
        (config / folder).mkdir(parents=True)
        for source in (CONFIG / folder).glob("*.yaml"):
            (config / folder / source.name).write_text(source.read_text())
    params = yaml.safe_load((config / "params" / "sim_target_mission.yaml").read_text())
    params["target_cv"]["ros__parameters"]["features"] = {"ring_fit": False}
    (config / "params" / "sim_target_mission.yaml").write_text(yaml.safe_dump(params))
    monkeypatch.setattr(build_module, "CONFIG_DIR", config)

    path = scenario_file(tmp_path, node_flags={"target_cv": {"ring_fit": True}})
    off = read(build_run(path, tmp_path / "off", variant="off"), "params.yaml")
    on = read(build_run(path, tmp_path / "on", variant="on"), "params.yaml")
    assert off["target_cv"]["ros__parameters"]["features"] == {"ring_fit": False}
    assert on["target_cv"]["ros__parameters"]["features"] == {"ring_fit": True}
    assert "features" not in on  # never at the top level of a ROS params file


def test_a_scenario_may_give_a_node_other_values_for_parameters_the_shipped_file_holds_with_its_flags_on(tmp_path):
    """PLT-11's scenario gives the health monitor a topic nobody publishes. Flags off is the shipped file, whatever the scenario says."""
    ghost = ["camera:camera/image:sensor_msgs/msg/Image:30", "ghost:/argus/nobody:std_msgs/msg/String:5"]
    path = scenario_file(tmp_path, node_flags={"health_monitor": {"health_monitor": True}}, node_params={"health_monitor": {"rates": ghost, "temp_limit_c": 70.0}})
    assert yaml.safe_load((CONFIG / "params" / "sim_target_mission.yaml").read_text())["health_monitor"]["ros__parameters"]["rates"] == ghost[:1]  # the shipped list: the camera only
    shipped = yaml.safe_load((CONFIG / "params" / "sim_target_mission.yaml").read_text())["health_monitor"]["ros__parameters"]
    on = read(build_run(path, tmp_path / "on", variant="on"), "params.yaml")["health_monitor"]["ros__parameters"]
    off = read(build_run(path, tmp_path / "off", variant="off"), "params.yaml")["health_monitor"]["ros__parameters"]
    assert on["rates"] == ghost and on["temp_limit_c"] == 70.0 and on["features"] == {"health_monitor": True} and on["send_statustext"] is shipped["send_statustext"]
    assert off == shipped and shipped["features"] == {"health_monitor": False}
    # The launch is asked for the node only with its flag on.
    assert read(tmp_path / "on", "scenario.json")["launch"]["arguments"]["health_monitor"] == "true"
    assert "health_monitor" not in read(tmp_path / "off", "scenario.json")["launch"]["arguments"]
    for wrong, said in (({"health_monitor": {"no_such_setting": 1}}, "health_monitor.no_such_setting"), ({"no_such_node": {"rates": []}}, "no_such_node.rates"),
                        ({"health_monitor": {"features": {"health_monitor": True}}}, "flags are set with node_flags")):  # fmt: skip
        with pytest.raises(BuildError, match=said):
            build_run(scenario_file(tmp_path, node_params=wrong), tmp_path / "bad")


# --- config overrides ---------------------------------------------------------------------


def test_a_node_that_exists_only_for_its_flag_is_started_only_when_the_flag_is_on(tmp_path):
    """The ring fit (CEN-13) is a node of its own: the 2026 launch does not start it, and a scenario's flag does."""
    scenario = SCENARIOS / "drop" / "hold_2m_ring_fit.yaml"
    on = read(build_run(scenario, tmp_path / "on", variant="on"), "scenario.json")
    off = read(build_run(scenario, tmp_path / "off", variant="off"), "scenario.json")
    assert on["launch"]["arguments"]["ring_fit"] == "true" and "ring_fit" not in off["launch"]["arguments"]
    assert read(tmp_path / "on", "params.yaml")["ring_fit"]["ros__parameters"]["features"]["ring_fit"] is True
    assert read(tmp_path / "off", "params.yaml")["ring_fit"]["ros__parameters"]["features"]["ring_fit"] is False
    assert on["flags"]["use_ring_fit"] is True
    # A scenario that does not ask for it never passes the argument, flags on or not.
    plain = read(build_run(SCENARIOS / "drop" / "hold_2m.yaml", tmp_path / "plain", variant="on"), "scenario.json")
    assert "ring_fit" not in plain["launch"]["arguments"]
    for launch in ("package_drop_demo.launch.py", "package_delivery_demo.launch.py"):
        text = (ROOT / "src" / "drone_mission_demo" / "launch" / launch).read_text()
        import re

        assert re.search(r'DeclareLaunchArgument\(\s*"ring_fit",\s*default_value="false"', text)  # off unless asked for
        assert 'condition=IfCondition(LaunchConfiguration("ring_fit"))' in text


def test_config_overrides_change_the_named_mission_only(tmp_path):
    path = scenario_file(tmp_path, config={"package_delivery": {"delivery_dwell_s": 2.0}})
    built = missions(read(build_run(path, tmp_path / "run"), "sequence.yaml"))
    assert built["package_delivery"]["config"]["delivery_dwell_s"] == 2.0
    assert built["takeoff"]["config"] == {"target_altitude_m": 15.0}


@pytest.mark.parametrize(
    "override, message",
    [({"package_delivry": {"delivery_dwell_s": 2.0}}, "package_delivry"), ({"package_delivery": {"delivery_dwel_s": 2.0}}, "delivery_dwel_s")],
)
def test_config_override_with_a_typo_fails_the_build(override, message, tmp_path):
    with pytest.raises(BuildError, match=message):
        build_run(scenario_file(tmp_path, config=override), tmp_path / "run")


# --- waypoints ---------------------------------------------------------------------------------


def test_waypoint_scenario_writes_the_route_and_points_the_mission_at_it(tmp_path):
    run = build_run(WAYPOINTS, tmp_path / "run")
    truth = read(run, "truth.json")
    assert len(truth["waypoints"]) == 7
    config = missions(read(run, "sequence.yaml"))["waypoint_navigation"]["config"]
    route_file = Path(config["waypoint_file"])
    assert route_file.is_absolute() and route_file.parent == run
    route = yaml.safe_load(route_file.read_text())["waypoints"]
    assert [wp["name"] for wp in route] == [wp["name"] for wp in truth["waypoints"]]
    origin = truth["origin"]
    for given, true in zip(route, truth["waypoints"]):
        north, east = tp.offset_ne_m(origin["latitude_deg"], origin["longitude_deg"], given["latitude"], given["longitude"])
        assert (east, north) == pytest.approx((true["e"], true["n"]), abs=0.02)
        assert given["altitude_m"] == pytest.approx(true["u"])
        assert point_in_polygon(true["e"], true["n"], truth["fence"]["polygon"])


# --- what the judge and the runner get ---------------------------------------------------------------


def test_judge_file_and_truth_are_what_the_judge_reads(tmp_path):
    run = build_run(scenario_file(tmp_path, judge={"max_points": 20.0}), tmp_path / "run")
    judge = read(run, "judge.yaml")
    assert judge["mission"] == "delivery"
    assert judge["settings"] == {"max_points": 20.0}
    assert judge["vehicle_ground_offset_m"] == pytest.approx(0.195)  # where the vehicle model rests in the world
    truth = read(run, "truth.json")
    assert {"origin", "takeoff", "fence", "survey_area", "return_points", "target", "objects"} <= set(truth)

    score = score_a_landing_on_the_target(run)
    assert score["details"]["distance_m"] == pytest.approx(0.0, abs=0.01)
    assert score["base_points"] == pytest.approx(20.0)
    assert score["common"]["fence_breaches"] == []


def test_judge_setting_the_judge_does_not_know_fails_the_build(tmp_path):
    with pytest.raises(BuildError, match="max_pionts"):
        build_run(scenario_file(tmp_path, judge={"max_pionts": 20.0}), tmp_path / "run")


def test_manifest_tells_the_runner_how_to_fly_it(tmp_path):
    path = scenario_file(
        tmp_path,
        wind={"level": "moderate", "direction_deg": 270.0},
        gps_events=[{"at_s": 40.0, "kind": "glitch", "north_m": 5.0}],
        sitl_params={"SIM_GPS1_NOISE": 0.5},
        timeout_s=420.0,
    )
    run = build_run(path, tmp_path / "run")
    manifest = read(run, "scenario.json")
    assert manifest["site_file"] == str(SIM / "sites" / "apollo_xi.yaml")
    assert manifest["launch"] == {
        "package": "drone_mission_demo",
        "file": "package_delivery_demo.launch.py",
        "arguments": {"sequence": str(run / "sequence.yaml"), "params": str(run / "params.yaml")},
    }
    assert manifest["vehicle"] == {
        "model": "iris_downcam", "bridge": str(run / "bridge.yaml"), "sensor_profile": "delivery", "mass_kg": 1.755, "cda_m2": 0.08,
        "min_rate_hz": None, "min_real_time_factor": None, "flow_camera": False, "headless_only": False,
    }  # fmt: skip
    assert manifest["sensor_check"] is False and manifest["truth_poses"] is False
    assert manifest["timeout_s"] == 420.0
    assert manifest["wind"] == {"level": "moderate", "mean_mps": 4.0, "gust_mps": 1.0, "discrete_gust_mps": 0.0, "direction_deg": 270.0}
    assert manifest["gps_events"] == [{"at_s": 40.0, "kind": "glitch", "north_m": 5.0}]
    assert manifest["expected_missions"] == ["takeoff", "package_delivery", "rtl"]
    assert manifest["list_file"] is None
    sitl = (run / "sitl.parm").read_text().split("\n")
    assert "SIM_TERRAIN 0" in sitl and "SIM_GPS1_NOISE 0.5" in sitl
    assert (run / "bridge.yaml").read_text() == (SIM / "config" / "bridge.yaml").read_text()


# --- a scenario flown by a tool instead of a mission ---------------------------------------------------


GEOFENCE = SCENARIOS / "geofence" / "breach.yaml"


def test_driver_scenario_tells_the_runner_which_command_flies_it(tmp_path):
    run = build_run(GEOFENCE, tmp_path / "run")
    manifest = read(run, "scenario.json")
    assert manifest["launch"] is None and manifest["expected_missions"] == []
    command = manifest["driver"]["command"]
    assert manifest["driver"]["tool"] == "fence_check"
    assert command[:4] == ["env", "-u", "PYTHONNOUSERSITE", "python3"]  # pymavlink is in the base image's user site
    assert command[4] == str(SIM / "fence_check.py")
    assert command[command.index("--site") + 1] == str(SIM / "sites" / "apollo_xi.yaml")
    assert command[command.index("--speed") + 1] == "5.0"
    assert command[command.index("--out") + 1] == str(run / "fence_check.json")
    assert read(run, "judge.yaml")["mission"] == "geofence"
    assert read(run, "truth.json")["fence"]["polygon"]
    assert read(build_run(DELIVERY, tmp_path / "mission"), "scenario.json")["driver"] is None


@pytest.mark.parametrize(
    "changes, message",
    [
        ({"driver": {"tool": "autopilot_whisperer"}}, "fence_check"),
        ({"driver": {"tool": "fence_check", "sped_mps": 5.0}}, "sped_mps"),
        ({"sequence": "package_delivery_demo.yaml"}, "flies no mission sequence"),
        ({"flags": {"use_frame_time_pose": True}}, "flags"),
    ],
)
def test_bad_driver_scenario_is_refused(changes, message, tmp_path):
    with pytest.raises(BuildError, match=message):
        build_run(scenario_file(tmp_path, base=GEOFENCE, **changes), tmp_path / "run")


def test_mission_scenario_still_needs_its_sequence_and_params(tmp_path):
    for key in ("sequence", "params"):
        with pytest.raises(BuildError, match=key):
            build_run(scenario_file(tmp_path, **{key: None}), tmp_path / "run")


# --- a bad scenario is refused ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "changes, message",
    [
        ({"mision": "delivery"}, "mision"),
        ({"mission": "juggling"}, "juggling"),
        ({"site": "atlantis"}, "atlantis"),
        ({"sequence": "nope.yaml"}, "nope.yaml"),
        ({"sensor_profile": "thermal"}, "thermal"),
        ({"sensor_check": "yes"}, "sensor_check must be true or false"),
        ({"wind": {"level": "hurricane"}}, "hurricane"),
        ({"targets": {"package_delivery": "nothing_here"}}, "nothing_here"),
        ({"targets": {"takeoff": "delivery_target"}}, "target_latitude"),
        ({"objects": [{"name": "a", "kind": "piano", "at": {"east_m": 1.0, "north_m": 1.0}}]}, "piano"),
        ({"gps_events": [{"kind": "glitch"}]}, "at_s"),
        ({"coordinate_error": {"radius": 3.0}}, "coordinate_error"),
    ],
)
def test_bad_scenario_is_refused(changes, message, tmp_path):
    with pytest.raises(BuildError, match=message):
        build_run(scenario_file(tmp_path, **changes), tmp_path / "run")


def test_two_objects_with_one_name_are_refused(tmp_path):
    objects = yaml.safe_load(DELIVERY.read_text())["objects"]
    objects.append(copy.deepcopy(objects[0]))
    with pytest.raises(BuildError, match="delivery_target"):
        build_run(scenario_file(tmp_path, objects=objects), tmp_path / "run")


# --- the command line ---------------------------------------------------------------------------------------


def run_cli(*args):
    return subprocess.run([sys.executable, str(SIM / "scenario" / "build.py"), *map(str, args)], capture_output=True, text=True)


def test_cli_builds_a_run_folder_and_prints_where(tmp_path):
    result = run_cli(DELIVERY, "--out", tmp_path / "run", "--seed", 5, "--variant", "on")
    assert result.returncode == 0, result.stderr
    assert str(tmp_path / "run") in result.stdout
    assert read(tmp_path / "run", "scenario.json")["seed"] == 5


def test_cli_default_run_folder_is_under_sim_state(tmp_path):
    result = run_cli(DELIVERY, "--dry-run")
    assert result.returncode == 0, result.stderr
    assert "sim/.state/runs/delivery-baseline-s" in result.stdout


def test_cli_refuses_a_bad_scenario(tmp_path):
    result = run_cli(scenario_file(tmp_path, mission="juggling"), "--out", tmp_path / "run")
    assert result.returncode == 2
    assert "juggling" in result.stderr
    assert not (tmp_path / "run" / "world.sdf").exists()


def test_the_gated_drop_aims_the_carried_parcel_not_the_vehicle_at_the_target():
    """The sim hangs a carried payload 0.13 m to the left of the vehicle's centre. A drop that centres the vehicle
    leaves the package that far off (0.13 to 0.14 m in five flights); the airframe row sim_iris_parcel gives the
    missions the parcel's place, and the gated drop's scenario names it."""
    from scenario import build, load_scenario

    rows = yaml.safe_load((ROOT / "src" / "argus_camera" / "config" / "robot.yaml").read_text())["airframes"]
    parcel, plain = rows["sim_iris_parcel"], rows["sim_iris_downcam"]
    assert tuple(parcel["payload_offset_m"]) == build.CARRY_OFFSET_BODY_M and parcel["verified"] is True
    assert parcel["cameras"] == plain["cameras"]  # the same vehicle and camera: only where the payload hangs is added
    gated = yaml.safe_load((SIM / "scenarios" / "drop" / "gated.yaml").read_text())
    assert gated["config"]["package_drop"] == {"fake_drop": False, "airframe": "sim_iris_parcel"}
    assert any(item.get("carried") is True for item in gated["objects"])


def test_a_scenario_can_ask_for_the_large_frame_dds_profile_and_only_then_is_it_used(tmp_path):
    """SIM-05's survey camera loses frames on Fast DDS's built-in transports. A scenario that carries large frames
    asks for `dds_profile: large_frames`; every other scenario, the flags-off baseline and G0 among them, runs as before."""
    from scenario import build, load_scenario
    from scenario.spec import BuildError

    survey = load_scenario(SIM / "scenarios" / "hover" / "survey_camera.yaml")
    assert survey.dds_profile == "large_frames"
    assert json.loads((build.build_run(survey.path, tmp_path / "survey") / "scenario.json").read_text())["dds_profile"] == "large_frames"
    for name in ("delivery/baseline", "delivery/set_down", "drop/gated"):
        plain = load_scenario(SIM / "scenarios" / f"{name}.yaml")
        assert plain.dds_profile is None
        assert json.loads((build.build_run(plain.path, tmp_path / name.replace("/", "-")) / "scenario.json").read_text())["dds_profile"] is None
    assert (SIM / "config" / "fastdds_large_frames.xml").is_file()
    ET.parse(SIM / "config" / "fastdds_large_frames.xml")  # well-formed
    bad = tmp_path / "bad.yaml"
    bad.write_text((SIM / "scenarios" / "hover" / "survey_camera.yaml").read_text().replace("dds_profile: large_frames", "dds_profile: huge"))
    with pytest.raises(BuildError, match="dds_profile"):
        load_scenario(bad)
    # The container side: start.sh hands the profile to the camera bridge when the run asks for one.
    start = (SIM / "container" / "start.sh").read_text()
    env = (SIM / "container" / "env.sh").read_text()
    # Only the camera bridge, which publishes the frames, is given the profile. Every process of the container on
    # a 256 MB segment overran the container's 1 GB of shared memory as soon as a flight had more than three
    # processes on ROS: the rest lost shared memory, and a flight with the localisation node never armed (LOC-5).
    assert "dds_profile" in start and 'env "FASTRTPS_DEFAULT_PROFILES_FILE=$DDS_PROFILE_FILE" ros2 run ros_gz_bridge parameter_bridge' in start
    assert "export FASTRTPS_DEFAULT_PROFILES_FILE" not in start and "FASTRTPS_DEFAULT_PROFILES_FILE" not in env


def test_a_survey_scenario_gives_the_mission_its_area_and_the_judge_the_track_ends(tmp_path):
    """LOC-3: the scenario says where the search area is in metres from the take-off point. The build turns that
    into the mission's corners (latitude, longitude) and, when asked, hands it the site's flight boundary; the
    track ends the scenario expects go to the judge only, since the mission plans its own tracks."""
    from scenario.spec import BuildError

    path = SIM / "scenarios" / "waypoint_navigation" / "survey.yaml"
    scenario = load_scenario(path)
    assert scenario.survey["mission"] == "search_pattern" and len(scenario.survey["area"]) == 4
    assert scenario.flags["survey_search"] is True and scenario.dds_profile == "large_frames"
    run = build_run(path, tmp_path / "run", variant="on")
    sequence = yaml.safe_load((run / "sequence.yaml").read_text())["mission_sequence"]
    config = next(entry for entry in sequence["missions"] if entry["name"] == "search_pattern")["config"]
    truth = json.loads((run / "truth.json").read_text())
    assert len(config["search_area"]) == 4 and len(config["flight_boundary"]) == 4
    # The corners come back to the scenario's metres through the site's own origin.
    site = site_file.load_site(SIM / "sites" / "apollo_xi.yaml")
    for (latitude, longitude), corner in zip(config["search_area"], scenario.survey["area"]):
        assert site.to_enu(latitude, longitude) == pytest.approx((corner["east_m"], corner["north_m"]), abs=0.05)
    assert truth["survey_area"] == [[c["east_m"], c["north_m"]] for c in scenario.survey["area"]]  # the form the localisation judge reads
    assert [w["name"] for w in truth["waypoints"]] == [p["name"] for p in scenario.waypoints["points"]] and not (run / "waypoints.yaml").exists()
    assert sequence["features"]["survey_search"] is True
    off = yaml.safe_load((build_run(path, tmp_path / "off", variant="off") / "sequence.yaml").read_text())["mission_sequence"]
    assert off["features"]["survey_search"] is False
    # An area outside the flight boundary, or for a mission that takes none, is refused.
    text = path.read_text()
    outside = tmp_path / "outside.yaml"
    outside.write_text(text.replace("{east_m: 80.0, north_m: 50.0}", "{east_m: 800.0, north_m: 50.0}"))
    with pytest.raises(BuildError, match="outside the flight boundary"):
        build_run(outside, tmp_path / "x")
    wrong = tmp_path / "wrong.yaml"
    assert "survey:\n  mission: search_pattern\n" in text
    wrong.write_text(text.replace("survey:\n  mission: search_pattern\n", "survey:\n  mission: takeoff\n"))
    with pytest.raises(BuildError, match="search_area"):
        build_run(wrong, tmp_path / "y")


def test_the_two_dip_scenarios_are_the_survey_with_the_rangefinder_reading_low_for_a_while(tmp_path):
    """LOC-1's two flights: the rangefinder reads 0.75 of the true range (8.0 m at the survey height: under 8.22 m)
    and 0.68 of it (7.25 m: under 7.77 m), for eight seconds in the middle of the survey."""
    survey = load_scenario(SIM / "scenarios" / "waypoint_navigation" / "survey.yaml")
    for name, scale in (("survey_dip", 0.75), ("survey_abort", 0.68)):
        path = SIM / "scenarios" / "waypoint_navigation" / f"{name}.yaml"
        scenario = load_scenario(path)
        assert scenario.gps_events == [{"at_s": 40.0, "kind": "rangefinder", "scale": scale, "for_s": 8.0}]
        assert scenario.survey == survey.survey and scenario.flags == survey.flags and scenario.sequence == survey.sequence
        manifest = json.loads((build_run(path, tmp_path / name, variant="on") / "scenario.json").read_text())
        assert manifest["gps_events"] == scenario.gps_events and manifest["target"] is False
    assert 0.75 * 10.668 < 8.22 and 0.75 * 10.668 > 7.77 and 0.68 * 10.668 < 7.77


def test_the_survey_scenarios_expect_their_own_lines_in_the_log_with_the_flags_on_only(tmp_path):
    from scenario.spec import BuildError

    expected = {
        "survey": ["survey finished: 3 of 3 tracks flown"],
        "survey_dip": ["is under 8.22 m: climbing back", "back at the survey height", "survey finished: 3 of 3 tracks flown"],
        "survey_abort": ["survey ended early:", "is under 7.77 m: the survey is over"],
        "survey_budget": ["the time budget of 35 s ran out"],
    }
    for name, lines in expected.items():
        path = SIM / "scenarios" / "waypoint_navigation" / f"{name}.yaml"
        assert load_scenario(path).expect_log == lines
        on = json.loads((build_run(path, tmp_path / f"{name}-on", variant="on") / "scenario.json").read_text())
        off = json.loads((build_run(path, tmp_path / f"{name}-off", variant="off") / "scenario.json").read_text())
        assert on["expect_log"] == lines and off["expect_log"] == []  # flags off, the mission is skipped and says none of it
    budget = load_scenario(SIM / "scenarios" / "waypoint_navigation" / "survey_budget.yaml")
    assert budget.config == {"search_pattern": {"time_budget_s": 35.0}}
    text = (SIM / "scenarios" / "waypoint_navigation" / "survey.yaml").read_text()
    for bad in ("expect_log: finished", "expect_log: [3]", "expect_log: ['']"):
        broken = tmp_path / "broken.yaml"
        broken.write_text(text.replace('expect_log:\n  - "survey finished: 3 of 3 tracks flown"', bad))
        assert bad in broken.read_text()
        with pytest.raises(BuildError, match="expect_log"):
            build_run(broken, tmp_path / "broken")


# --- the recovery vehicle and its check flight (REC-8) ---


def test_the_hook_check_scenario_joins_the_bucket_to_the_hook_and_flies_the_check_beside_the_bridge(tmp_path):
    import payload_bridge
    from scenario.build import RECOVERY_PARM, read_parm
    from scenario.spec import DRIVER_PORT

    run = build_run(SCENARIOS / "recovery" / "hook_check.yaml", tmp_path / "run", variant="on")
    manifest = json.loads((run / "scenario.json").read_text())
    assert manifest["mission"] == "recovery" and manifest["vehicle"]["model"] == "iris_recovery" and manifest["vehicle"]["flow_camera"] is True
    # The bucket's joint is to the hook, which swings; the world stands the recovery vehicle where the base world stands its own.
    world = (run / "world.sdf").read_text()
    assert "<uri>model://iris_recovery</uri>" in world and "<child_link>hook_link</child_link>" in world and world.count("DetachableJoint") == 1
    # The bridge is told where the hook hangs; the envelope is the bridge's own.
    bridge = manifest["payload_bridge"]
    assert bridge == {"carried": None, "bucket": "bucket", "capture_half_width_m": 0.10, "hook_offset_m": [0.0, 0.0, -0.55]} and payload_bridge.wanted(manifest)
    # The driver flies beside the bridge, on a MAVLink port of its own.
    driver = manifest["driver"]
    assert driver["tool"] == "recovery_check" and driver["port"] == DRIVER_PORT == 14554 and "udpin:127.0.0.1:14554" in driver["command"]
    assert driver["command"][driver["command"].index("--run-dir") + 1] == str(run)
    # The run's SITL parameters: the recovery file's, with the gripper as the bridge reads it.
    recovery = read_parm(RECOVERY_PARM)
    sitl = dict(line.split() for line in (run / "sitl.parm").read_text().splitlines())
    assert all(float(sitl[name]) == value for name, value in recovery.items())
    assert {name: recovery[name] for name in ("GRIP_ENABLE", "GRIP_TYPE", "GRIP_RELEASE", "GRIP_GRAB", "GRIP_NEUTRAL", "SERVO7_FUNCTION")} == {
        name: float(value) for name, value in payload_bridge.SITL_PARAMS.items() if name.startswith(("GRIP", "SERVO7"))
    }
    assert int(recovery["FS_OPTIONS"]) & (1 << 5) == 0 and recovery["EK3_SRC_OPTIONS"] == 0 and recovery["EK3_SRC2_POSXY"] == 0 and recovery["GPS2_TYPE"] == 0
    truth = json.loads((run / "truth.json").read_text())
    assert truth["bucket"] == {"name": "bucket", "mass_kg": 1.0, "e": 12.0, "n": 8.0} and "takeoff" in truth["return_points"]


def test_only_the_recovery_check_may_fly_beside_the_bridge_and_only_with_a_bucket_and_a_hook(tmp_path):
    source = (SCENARIOS / "recovery" / "hook_check.yaml").read_text()
    cases = {
        "no_hook.yaml": (source.replace("sensor_profile: recovery\n", ""), "sensor_profile: recovery"),
        "no_bucket.yaml": (source.replace("bucket: bucket\n", ""), "needs `bucket:`"),
        "hover.yaml": (source.replace("tool: recovery_check\n  height_m: 5.0\n  lift_m: 3.0", "tool: hover\n  height_m: 5.0"), "no carried payload or bucket in a driver scenario"),
        "refusals.yaml": (source.replace("bucket: bucket\n", "bucket: bucket\ngrab_refusals: -1\n"), "grab_refusals is a whole number"),
    }
    for name, (text, why) in cases.items():
        broken = tmp_path / name
        broken.write_text(text)
        with pytest.raises(BuildError, match=why):
            build_run(broken, tmp_path / name.split(".")[0], variant="on")
    # A delivery on the delivery vehicle has no hook: its payload and its bucket stay joined to the camera link, as before.
    release = build_run(SCENARIOS / "delivery" / "release.yaml", tmp_path / "release", variant="on")
    assert "hook_link" not in (release / "world.sdf").read_text() and "hook_offset_m" not in json.loads((release / "scenario.json").read_text())["payload_bridge"]
    assert "EK3_SRC2_POSXY" not in (release / "sitl.parm").read_text()


def test_a_recovery_scenario_can_have_gps_faults_and_refused_grabs_for_the_runs_the_mission_will_fly(tmp_path):
    """REC-8's runs R3 to R6 need the fault injector (SIM-04) and a latch that does not close, on the recovery vehicle.
    The mission that flies them is REC-2's; what the sim gives them is built and checked here."""
    source = (SCENARIOS / "recovery" / "hook_check.yaml").read_text()
    faulty = tmp_path / "faulty.yaml"
    faulty.write_text(source.replace("bucket: bucket\n", "bucket: bucket\ngrab_refusals: 2\ngps_events:\n  - {at_s: 25.0, kind: glitch, north_m: 10.0, for_s: 5.0}\n"))
    run = build_run(faulty, tmp_path / "run", variant="on")
    manifest = json.loads((run / "scenario.json").read_text())
    assert manifest["payload_bridge"]["refuse_grabs"] == 2 and manifest["vehicle"]["model"] == "iris_recovery"
    assert len(manifest["gps_events"]) == 1 and manifest["gps_events"][0]["kind"] == "glitch"
    plain = json.loads((build_run(SCENARIOS / "recovery" / "hook_check.yaml", tmp_path / "plain", variant="on") / "scenario.json").read_text())
    assert "refuse_grabs" not in plain["payload_bridge"] and not plain["gps_events"]
    no_bucket = tmp_path / "no_bucket.yaml"
    no_bucket.write_text((SCENARIOS / "hover" / "wind_5.yaml").read_text() + "grab_refusals: 1\n")
    with pytest.raises(BuildError, match="grab_refusals needs a `bucket:`"):
        build_run(no_bucket, tmp_path / "nb", variant="on")


# --- the failsafe checks of the flight-controller baseline (PLT-08) ----------------------------------------


FAILSAFES = {"battery": "failsafe_battery.yaml", "radio": "failsafe_radio.yaml", "ground_station": "failsafe_ground_station.yaml"}
REPO = SIM.parent
BASELINE_FILE = REPO / "platform" / "fcu" / "argus_2027.param"


def baseline_params():
    sys.path.insert(0, str(REPO / "tools"))
    from fc_param_check import read_params

    return read_params(BASELINE_FILE)


@pytest.mark.parametrize("failsafe, file", sorted(FAILSAFES.items()))
def test_a_failsafe_scenario_starts_sitl_on_the_baseline_and_is_flown_by_the_failsafe_check(failsafe, file, tmp_path):
    run = build_run(SCENARIOS / "hover" / file, tmp_path / "run")
    manifest = read(run, "scenario.json")
    command = manifest["driver"]["command"]
    assert manifest["driver"]["tool"] == "failsafe_check" and command[4] == str(SIM / "failsafe_check.py")
    assert command[command.index("--failsafe") + 1] == failsafe
    assert command[command.index("--params") + 1] == str(BASELINE_FILE)
    assert command[command.index("--out") + 1] == str(run / "failsafe_check.json")
    # SITL starts with the baseline's own values: every parameter of the baseline that SITL can take.
    sitl = {line.split()[0]: float(line.split()[1]) for line in (run / "sitl.parm").read_text().splitlines()}
    baseline = baseline_params()
    not_for_sitl = {name for name in baseline if name.startswith(("FENCE_", "RNGFND1_", "SERIAL"))}  # the site's upload sets the fence; SITL's rangefinder is its own, and so are its serial ports
    assert not_for_sitl == {"FENCE_ENABLE", "FENCE_TYPE", "FENCE_ACTION", "FENCE_ALT_MAX", "FENCE_MARGIN", "RNGFND1_TYPE", "RNGFND1_ORIENT", "RNGFND1_MIN_CM", "RNGFND1_MAX_CM", "SERIAL4_PROTOCOL", "SERIAL4_BAUD"}
    for name in sorted(set(baseline) - not_for_sitl):
        assert sitl.get(name) == baseline[name], f"{file}: sitl_params has {name} {sitl.get(name)}, the baseline has {baseline[name]}"
    # A battery above the low line from the start: SITL's own 12.6 V would trip the failsafe on the ground.
    assert sitl["SIM_BATT_VOLTAGE"] > baseline["BATT_LOW_VOLT"] + 2.0


def test_the_baselines_ceiling_is_the_site_files():
    site = site_file.load_site(SIM / "sites" / "apollo_xi.yaml")
    plan = site_file.fence_plan(site)
    baseline = baseline_params()
    for name in ("FENCE_TYPE", "FENCE_ACTION", "FENCE_ALT_MAX", "FENCE_MARGIN"):
        assert plan.params[name] == baseline[name], name


def test_a_failsafe_check_needs_a_failsafe_it_knows(tmp_path):
    base = SCENARIOS / "hover" / "failsafe_radio.yaml"
    with pytest.raises(BuildError, match="failsafe"):
        build_run(scenario_file(tmp_path, base=base, driver={"tool": "failsafe_check", "failsafe": "ekf"}), tmp_path / "run")
    with pytest.raises(BuildError, match="failsafe"):
        build_run(scenario_file(tmp_path, base=base, driver={"tool": "failsafe_check"}), tmp_path / "run2")


# --- flight-controller scripts, forbidden log lines and the watchdog check (PLT-09) -------------------------


WATCHDOG_SCRIPT = REPO / "platform" / "fcu" / "scripts" / "argus_watchdog.lua"


def test_a_scenario_with_fc_scripts_has_sitl_run_them_flags_on_or_off(tmp_path):
    for variant in ("off", "on"):
        run = build_run(SCENARIOS / "delivery" / "watchdog.yaml", tmp_path / variant, variant=variant)
        manifest = read(run, "scenario.json")
        assert manifest["fc_scripts"] == [str(WATCHDOG_SCRIPT)]
        sitl = {line.split()[0]: float(line.split()[1]) for line in (run / "sitl.parm").read_text().splitlines()}
        assert sitl["SCR_ENABLE"] == 1.0
        assert manifest["flags"] == {}  # every flag off: the 2026 delivery, with the script on the flight controller
    # Without them nothing is loaded and scripting stays off.
    plain = build_run(DELIVERY, tmp_path / "plain")
    assert read(plain, "scenario.json")["fc_scripts"] == [] and "SCR_ENABLE" not in (plain / "sitl.parm").read_text()
    # The start script copies what the manifest names into SITL's scripts folder, and empties the folder first.
    start = (SIM / "container" / "start.sh").read_text()
    assert 'rm -rf -- "${SITL_DIR:?}/scripts"' in start and 'cp -- "${fc_scripts[@]}" "$SITL_DIR/scripts/"' in start
    assert start.index('rm -rf -- "${SITL_DIR:?}/scripts"') < start.index("sim_vehicle.py")


@pytest.mark.parametrize("scripts, message", [(["platform/fcu/scripts/none.lua"], "not a file"), (["platform/fcu/argus_2027.param"], "lua"), ("argus_watchdog.lua", "lua")])
def test_fc_scripts_that_are_not_lua_files_of_the_repo_are_refused(scripts, message, tmp_path):
    with pytest.raises(BuildError, match=message):
        build_run(scenario_file(tmp_path, fc_scripts=scripts), tmp_path / "run")


def test_the_watchdog_scenario_is_flown_by_its_check_with_the_script_loaded(tmp_path):
    run = build_run(SCENARIOS / "hover" / "watchdog.yaml", tmp_path / "run")
    manifest = read(run, "scenario.json")
    command = manifest["driver"]["command"]
    assert manifest["driver"]["tool"] == "watchdog_check" and command[4] == str(SIM / "watchdog_check.py")
    assert command[command.index("--out") + 1] == str(run / "watchdog_check.json")
    assert manifest["fc_scripts"] == [str(WATCHDOG_SCRIPT)] and "SCR_ENABLE 1" in (run / "sitl.parm").read_text()


def test_forbid_log_is_carried_with_the_flags_on_and_must_be_text(tmp_path):
    on = build_run(SCENARIOS / "delivery" / "actuation_lock.yaml", tmp_path / "on", variant="on")
    assert read(on, "scenario.json")["forbid_log"] == ["Actuation lock: the gripper command was not sent", "TAKEOVER"]
    assert read(on, "scenario.json")["flags"]["takeover_latch"] is True  # a healthy flight with the latch on must not be latched out
    assert read(on, "scenario.json")["flags"]["actuation_lock"] is True and read(on, "scenario.json")["flags"]["delivery_soft_set_down"] is True
    off = build_run(SCENARIOS / "delivery" / "actuation_lock.yaml", tmp_path / "off", variant="off")
    assert read(off, "scenario.json")["forbid_log"] == []  # the lines are the flags-on flight's
    with pytest.raises(BuildError, match="forbid_log"):
        build_run(scenario_file(tmp_path, forbid_log=[3]), tmp_path / "bad")
