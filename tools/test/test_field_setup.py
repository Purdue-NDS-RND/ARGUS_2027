"""OPS-01: the field setup tool (tools/field_setup.py): one day file, checked, and every file that needs its coordinates. No ROS."""

import json
import sys
from pathlib import Path

import pytest
import yaml

TOOLS = Path(__file__).resolve().parents[1]
REPO = TOOLS.parent
sys.path.insert(0, str(TOOLS))

import field_setup as fs  # noqa: E402
from field_setup import FieldError  # noqa: E402

EXAMPLE = REPO / "field" / "example_day.yaml"
SEQUENCES = REPO / "src" / "drone_mission_demo" / "config" / "sequences"
COORDINATE_KEYS = fs.COORDINATE_KEYS


def example():
    return yaml.safe_load(EXAMPLE.read_text())


def day(**changes):
    data = example()
    for key, value in changes.items():
        if value is None:
            data.pop(key, None)
        else:
            data[key] = value
    return fs.parse_day(data)


def moved(target, east_m, north_m):
    """The example day with one target put at metres east and north of the origin."""
    sys.path.insert(0, str(REPO / "sim"))
    import target_placement

    data = example()
    origin = data["site"]["origin"]
    latitude, longitude = target_placement.latlon_for(origin["latitude_deg"], origin["longitude_deg"], north_m, east_m)
    data["targets"][target] = {"latitude_deg": latitude, "longitude_deg": longitude}
    return fs.parse_day(data)


def test_the_example_day_is_the_placeholder_site_and_passes_every_check():
    sys.path.insert(0, str(REPO / "sim"))
    import site_file

    made = fs.load_day(EXAMPLE)
    assert fs.check_day(made) == [] and made.verified is False
    site = site_file.load_site(REPO / "sim" / "sites" / "apollo_xi.yaml")
    for field in ("origin", "runway", "takeoff", "compass_rose", "boundary", "boundary_geo", "max_altitude_agl_m", "survey_area", "fence_action", "fence_margin_m"):
        assert getattr(made.site, field) == getattr(site, field), field
    assert set(made.targets) == set(fs.TARGETS) and len(made.waypoints) == 7


def test_everything_that_needs_a_coordinate_is_generated_from_the_day_file(tmp_path):
    sys.path.insert(0, str(REPO / "sim"))
    import site_file

    made = fs.load_day(EXAMPLE)
    written, skipped = fs.generate(made, tmp_path, "ab" * 32)
    assert skipped == []
    names = {str(path.relative_to(tmp_path)) for path in written}
    assert names == {"site.yaml", "fence.plan", "fence.param", "summary.txt", "waypoints/competition_waypoints.yaml", *(f"sequences/{name}" for name in fs.SEQUENCES)}
    # The site file is one the sim and the fence upload take, and it is the day's site.
    site = site_file.load_site(tmp_path / "site.yaml")
    assert site.boundary_geo == made.site.boundary_geo and site.takeoff == made.site.takeoff and site.name == "example_day" and site.verified is False
    # The fence for a ground station is the same polygon and the same parameters, FENCE_ENABLE last.
    plan = json.loads((tmp_path / "fence.plan").read_text())
    assert [tuple(corner) for corner in plan["geoFence"]["polygons"][0]["polygon"]] == list(made.site.boundary_geo) and plan["geoFence"]["polygons"][0]["inclusion"] is True
    assert plan["geoFence"]["circles"] == [] and plan["mission"]["items"] == []
    params = [line.split(",") for line in (tmp_path / "fence.param").read_text().splitlines() if not line.startswith("#")]
    assert {name: float(value) for name, value in params} == {name: float(value) for name, value in site_file.fence_plan(made.site).params.items()} and params[-1][0] == "FENCE_ENABLE"
    # The route is in the committed route's format, with the day's points in the day's order.
    route = yaml.safe_load((tmp_path / "waypoints" / "competition_waypoints.yaml").read_text())["waypoints"]
    committed = yaml.safe_load((REPO / "src/drone_mission_demo/config/waypoints/competition_waypoints.yaml").read_text())["waypoints"]
    assert [set(item) for item in route] == [set(committed[0])] * 7 and [item["name"] for item in route] == [f"wp{n}" for n in range(1, 8)]
    assert route[1]["altitude_agl_ft"] == 150 and abs(route[0]["latitude"] - made.waypoints[0]["latitude"]) < 1e-7
    assert "NOT VERIFIED" in (tmp_path / "site.yaml").read_text() and "NOT VERIFIED" in (tmp_path / "summary.txt").read_text()


@pytest.mark.parametrize("sequence", sorted(fs.SEQUENCES))
def test_a_generated_sequence_is_its_source_with_only_its_coordinate_lines_changed(sequence, tmp_path):
    made = fs.load_day(EXAMPLE)
    fs.generate(made, tmp_path, "ab" * 32)
    source = (SEQUENCES / sequence).read_text().splitlines()
    generated = [line for line in (tmp_path / "sequences" / sequence).read_text().splitlines()]
    header = [line for line in generated if line.startswith("# GENERATED") or line.startswith("# The committed file") or line.startswith("# change the day file") or line.startswith("# NOT VERIFIED") or line.startswith("# From a sim sequence")]
    body = generated[len(header):]
    assert generated[: len(header)] == header and "do not edit" in header[0] and any("NOT VERIFIED" in line for line in header)
    plan = fs.SEQUENCES[sequence]
    # One line is added where a mission can read the day's flight boundary and the source has no line for it.
    added = [line for line in body if line.strip().startswith("flight_boundary:") and line not in source and not plan.get("survey")]
    assert len(added) == (1 if plan.get("waypoints") or "pickup" in plan else 0)
    body = [line for line in body if line not in added]
    assert len(body) == len(source)
    differing = [(old, new) for old, new in zip(source, body) if old != new]
    expected = (2 if "target" in plan else 0) + (2 if plan.get("survey") else 0) + (1 if "pickup" in plan else 0)
    assert len(differing) == expected, differing
    for old, new in differing:
        key = old.split(":", 1)[0].strip()
        assert key in COORDINATE_KEYS and new.split(":", 1)[0].strip() == key
        assert old.split("#", 1)[1:] == new.split("#", 1)[1:]  # the line's comment is kept
    loaded = yaml.safe_load("\n".join(body))
    if "target" in plan:
        configs = [mission["config"] for mission in loaded["mission_sequence"]["missions"] if "target_latitude" in (mission.get("config") or {})]
        if not configs:  # the drop's sequences nest their mission's config one level down
            configs = [found for found in _configs(loaded) if "target_latitude" in found]
        assert len(configs) == 1 and (round(configs[0]["target_latitude"], 7), round(configs[0]["target_longitude"], 7)) == tuple(round(v, 7) for v in made.targets[plan["target"]])
    assert bool(plan.get("sim_only")) == any("From a sim sequence" in line for line in header)


def _configs(node):
    if isinstance(node, dict):
        yield node
        for value in node.values():
            yield from _configs(value)
    elif isinstance(node, list):
        for value in node:
            yield from _configs(value)


def test_the_days_fence_reaches_every_mission_that_reads_one(tmp_path):
    """The spec audit: the route's missions and the recovery read flight_boundary too, and their sources have no line for it."""
    made = fs.load_day(EXAMPLE)
    fs.generate(made, tmp_path, "ab" * 32)
    fence = [list(round(v, 7) for v in corner) for corner in made.site.boundary_geo]
    for sequence, plan in fs.SEQUENCES.items():
        loaded = yaml.safe_load((tmp_path / "sequences" / sequence).read_text())
        holders = [found for found in _configs(loaded) if "flight_boundary" in found]
        if plan.get("waypoints"):
            assert len(holders) == 1 and "waypoint_file" in holders[0] and holders[0]["flight_boundary"] == fence, sequence
        elif "pickup" in plan or plan.get("survey"):
            assert len(holders) == 1 and holders[0]["flight_boundary"] == fence, sequence
        else:
            assert holders == [], sequence  # the drop and the delivery read no boundary
    # The form is the one the 2027 route code and the recovery take: three or more [latitude, longitude] corners.
    assert len(fence) >= 3 and all(len(corner) == 2 for corner in fence)


def test_the_recovery_returns_where_the_day_file_says(tmp_path):
    data = example()
    data["recovery_return"] = "compass_rose"
    made = fs.parse_day(data)
    fs.generate(made, tmp_path / "rose", "ab" * 32)
    recovery = next(found for found in _configs(yaml.safe_load((tmp_path / "rose" / "sequences" / "sim_package_recovery.yaml").read_text())) if "pickup" in found)
    rose = data["site"]["compass_rose"]
    assert recovery["return_target"] == {"latlon": [rose["latitude_deg"], rose["longitude_deg"]]}
    data["recovery_return"] = "takeoff"
    fs.generate(fs.parse_day(data), tmp_path / "takeoff", "ab" * 32)
    assert "return_target: takeoff " in (tmp_path / "takeoff" / "sequences" / "sim_package_recovery.yaml").read_text()
    del data["recovery_return"]  # not said: the sequence's own line stays
    fs.generate(fs.parse_day(data), tmp_path / "own", "ab" * 32)
    own = next(line for line in (SEQUENCES / "sim_package_recovery.yaml").read_text().splitlines() if "return_target:" in line)
    assert own in (tmp_path / "own" / "sequences" / "sim_package_recovery.yaml").read_text()
    data["recovery_return"] = "the hangar"
    with pytest.raises(FieldError, match="recovery_return is takeoff or compass_rose"):
        fs.parse_day(data)


def test_a_leg_that_crosses_a_notch_in_the_fence_is_said_though_both_its_ends_are_inside():
    sys.path.insert(0, str(REPO / "sim"))
    import target_placement

    data = example()
    origin = data["site"]["origin"]

    def at(east_m, north_m):
        latitude, longitude = target_placement.latlon_for(origin["latitude_deg"], origin["longitude_deg"], north_m, east_m)
        return {"latitude_deg": latitude, "longitude_deg": longitude}

    # The placeholder rectangle with a notch cut into its northern edge, between 40 m west and 20 m west, down to 100 m north.
    data["site"]["flight_boundary"] = [at(-150, -60), at(150, -60), at(150, 180), at(-20, 180), at(-20, 100), at(-40, 100), at(-40, 180), at(-150, 180)]
    data["waypoints"] = [{"name": "west", **at(-80, 150), "altitude_agl_ft": 100}, {"name": "east", **at(20, 150), "altitude_agl_ft": 100}, {"name": "south", **at(20, 40), "altitude_agl_ft": 100}]
    problems = fs.check_day(fs.parse_day(data))
    assert problems == ["the leg from west to east leaves the flight boundary (a commanded path keeps 5 m)"]
    data["waypoints"][0] = {"name": "west", **at(-80, 97), "altitude_agl_ft": 100}
    data["waypoints"][1] = {"name": "east", **at(20, 97), "altitude_agl_ft": 100}
    problems = fs.check_day(fs.parse_day(data))
    assert len(problems) == 1 and "the leg from west to east comes within 3.0 m of the flight boundary" in problems[0]  # under the notch's foot
    data["waypoints"][0] = {"name": "west", **at(-80, 90), "altitude_agl_ft": 100}
    data["waypoints"][1] = {"name": "east", **at(20, 90), "altitude_agl_ft": 100}
    assert fs.check_day(fs.parse_day(data)) == []


def test_the_survey_and_the_recovery_get_the_days_area_boundary_and_pickup(tmp_path):
    made = fs.load_day(EXAMPLE)
    fs.generate(made, tmp_path, "ab" * 32)
    survey = next(found for found in _configs(yaml.safe_load((tmp_path / "sequences" / "search_pattern_demo.yaml").read_text())) if "search_area" in found)
    assert [tuple(corner) for corner in survey["flight_boundary"]] == [tuple(round(v, 7) for v in corner) for corner in made.site.boundary_geo]
    assert len(survey["search_area"]) == 4 and abs(survey["search_area"][0][0] - made.site_data["survey_area"][0]["latitude_deg"]) < 1e-7
    recovery = next(found for found in _configs(yaml.safe_load((tmp_path / "sequences" / "sim_package_recovery.yaml").read_text())) if "pickup" in found)
    assert tuple(recovery["pickup"]["latlon"]) == tuple(round(v, 7) for v in made.targets["recovery_pickup"])


@pytest.mark.parametrize(
    "east_m, north_m, word",
    [
        (160.0, 60.0, "targets.drop is outside the flight boundary, by 10.0 m"),
        (147.0, 60.0, "targets.drop is 3.0 m inside the flight boundary; a commanded point keeps 5 m"),
        (2500.0, 60.0, "targets.drop is 2501 m from the takeoff point"),
    ],
)
def test_a_target_outside_the_fence_too_near_it_or_far_from_the_field_is_said(east_m, north_m, word):
    problems = fs.check_day(moved("drop", east_m, north_m))
    assert len(problems) == 1 and word in problems[0]
    assert fs.check_day(moved("drop", 144.9, 60.0)) == []  # 5.1 m inside: allowed


def test_a_swapped_latitude_and_longitude_is_caught_as_far_from_the_field():
    data = example()
    target = data["targets"]["delivery"]
    data["targets"]["delivery"] = {"latitude_deg": 34.18315, "longitude_deg": -118.4757187}  # two digits of the latitude transposed
    problems = fs.check_day(fs.parse_day(data))
    assert len(problems) == 1 and "targets.delivery" in problems[0] and ("outside the flight boundary" in problems[0] or "from the takeoff point" in problems[0])
    data["targets"]["delivery"] = {"latitude_deg": target["longitude_deg"], "longitude_deg": target["latitude_deg"]}
    with pytest.raises(FieldError, match="latitude_deg must be a number of degrees"):  # -118 is not a latitude at all
        fs.parse_day(data)


def test_waypoints_are_held_to_the_fence_the_ceiling_and_unique_names():
    data = example()
    data["waypoints"][1]["altitude_agl_ft"] = 450
    data["waypoints"][2]["altitude_agl_ft"] = 0
    data["waypoints"][3]["name"] = "wp1"
    data["waypoints"][4]["longitude_deg"] = -118.4700
    problems = fs.check_day(fs.parse_day(data))
    text = " | ".join(problems)
    assert len(problems) == 4 and "waypoint wp2: 450 ft is above the site's ceiling (120 m, 394 ft)" in text and "waypoint wp3: altitude_agl_ft must be above 0" in text
    assert "waypoint wp1 is listed more than once" in text and "waypoint wp5 is outside the flight boundary" in text


def test_a_survey_corner_too_near_the_fence_is_said():
    data = example()
    data["site"]["survey_area"][1] = {"latitude_deg": 34.1755390, "longitude_deg": -118.4808930}  # 2 m inside the eastern edge
    problems = fs.check_day(fs.parse_day(data))
    assert len(problems) == 1 and "survey_area[1] is 2.0 m inside the flight boundary" in problems[0]


@pytest.mark.parametrize(
    "change, word",
    [
        ({"name": "Day One"}, "name is lower case"),
        ({"date": "4 June"}, "date is YYYY-MM-DD"),
        ({"site": None}, "missing 'site'"),
        ({"verified": "yes"}, "verified is true or false"),
        ({"targets": {"landing": {"latitude_deg": 1.0, "longitude_deg": 2.0}}}, "targets has drop, delivery, recovery_pickup"),
        ({"targets": {"drop": {"latitude_deg": "34.1", "longitude_deg": -118.0}}}, "latitude_deg must be a number"),
        ({"waypoints": [{"latitude_deg": 34.175, "longitude_deg": -118.4825}]}, "altitude_agl_ft must be a number of feet"),
        ({"extra": 1}, "unknown key"),
    ],
)
def test_a_day_file_that_is_not_one_is_refused_with_the_reason(change, word):
    with pytest.raises(FieldError, match=word):
        day(**change)


def test_the_sites_own_checks_reach_the_day_file():
    data = example()
    data["site"]["max_altitude_agl_m"] = 150.0
    with pytest.raises(FieldError, match="above 400 ft"):
        fs.parse_day(data)
    data = example()
    data["site"]["flight_boundary"] = data["site"]["flight_boundary"][:2]
    with pytest.raises(FieldError, match="at least 3 corners"):
        fs.parse_day(data)
    data = example()
    data["site"]["name"] = "other"
    with pytest.raises(FieldError, match="name and verified are the day file's own"):
        fs.parse_day(data)


def test_a_sequence_with_no_place_for_the_coordinates_or_another_route_is_refused():
    made = fs.load_day(EXAMPLE)
    source = (SEQUENCES / "package_delivery_live.yaml").read_text()
    with pytest.raises(FieldError, match="expected one target_latitude line.*found 0"):
        fs.sequence_text(source.replace("target_latitude:", "target_lat:"), "package_delivery_live.yaml", made, "ab" * 32)
    route = (SEQUENCES / "waypoint_navigation_live.yaml").read_text()
    with pytest.raises(FieldError, match="does not read its route"):
        fs.sequence_text(route.replace("competition_waypoints.yaml", "other_route.yaml"), "waypoint_navigation_live.yaml", made, "ab" * 32)


def test_a_generated_file_that_changed_anything_but_coordinates_is_refused():
    made = fs.load_day(EXAMPLE)
    source = (SEQUENCES / "package_delivery_live.yaml").read_text()
    good = fs.sequence_text(source, "package_delivery_live.yaml", made, "ab" * 32)
    fs.only_coordinates_changed(source, good, "package_delivery_live.yaml")
    with pytest.raises(FieldError, match=r"differs from the committed one at .*transit_altitude_m: nothing but coordinates may change"):
        fs.only_coordinates_changed(source, good.replace("transit_altitude_m: 15.0", "transit_altitude_m: 25.0", 1), "package_delivery_live.yaml")


def run(tmp_path, data, *arguments):
    path = tmp_path / "day.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return fs.main([str(path), "--out", str(tmp_path / "out"), *arguments])


def test_the_command_checks_then_writes_and_writes_nothing_when_a_check_fails(tmp_path, capsys):
    assert run(tmp_path, example(), "--check") == 0 and not (tmp_path / "out").exists()
    assert "OK: nothing was written (--check)" in capsys.readouterr().out
    assert run(tmp_path, example()) == 0 and (tmp_path / "out" / "sequences" / "package_delivery_live.yaml").is_file()
    out = capsys.readouterr().out
    assert "targets.delivery" in out and "NOT VERIFIED: these files are for the sim and for checking, not for flight" in out
    bad = example()
    bad["waypoints"][0]["altitude_agl_ft"] = 900
    (tmp_path / "bad").mkdir()
    assert run(tmp_path / "bad", bad) == 1
    out = capsys.readouterr().out
    assert "PROBLEM: waypoint wp1: 900 ft is above the site's ceiling" in out and "1 problem(s): nothing was written" in out and not (tmp_path / "bad" / "out").exists()


def test_for_flight_needs_a_verified_day_file_and_an_unreadable_one_is_exit_2(tmp_path, capsys):
    assert run(tmp_path, example(), "--for-flight") == 1
    assert "the day file is not verified" in capsys.readouterr().out and not (tmp_path / "out").exists()
    verified = example()
    verified["verified"] = True
    assert run(tmp_path, verified, "--for-flight") == 0
    text = (tmp_path / "out" / "sequences" / "package_drop_live.yaml").read_text()
    assert "NOT VERIFIED" not in text and "NOT VERIFIED" not in capsys.readouterr().out
    assert fs.main([str(tmp_path / "nowhere.yaml")]) == 2 and "nowhere.yaml" in capsys.readouterr().err


def test_a_day_with_no_delivery_target_leaves_the_delivery_sequences_out_and_says_so(tmp_path, capsys):
    data = example()
    del data["targets"]["delivery"]
    data["waypoints"] = []
    assert run(tmp_path, data) == 0
    out = capsys.readouterr().out
    assert "left out package_delivery_live.yaml: the day file has no targets.delivery" in out and "left out waypoint_navigation_live.yaml: the day file has no waypoints" in out
    assert "left out final_waypoint_drop.yaml: the day file has no waypoints" in out
    assert sorted(path.name for path in (tmp_path / "out" / "sequences").iterdir()) == ["package_drop_2027.yaml", "package_drop_live.yaml", "search_pattern_demo.yaml", "sim_package_recovery.yaml"]
    assert not (tmp_path / "out" / "waypoints").exists()


def test_every_sequence_with_a_target_or_the_competition_route_is_in_the_tools_table():
    """A new live sequence with coordinates in it must be added to SEQUENCES, or its coordinates would be typed by hand."""
    marks = ("target_latitude:", fs.WAYPOINT_FILE_LINE, "search_area:", "pickup:", "latlon:", "flight_boundary:", "latitude:")  # every way a sequence holds a coordinate
    seen = 0
    for path in sorted(SEQUENCES.glob("*.yaml")):
        text = "\n".join(line.split("#", 1)[0] for line in path.read_text().splitlines())
        has_coordinates = any(mark in text for mark in marks)
        # The sim's copies of the drop and the delivery, and the 2026 demos of them, fly the sim's own target (tools/sim_sequence_copies.py).
        sims_own = path.name.startswith("sim_package_d") or path.name in ("package_delivery_demo.yaml", "package_drop_demo.yaml")
        if has_coordinates and not sims_own:
            seen += 1
            assert path.name in fs.SEQUENCES, path.name
    assert seen == len(fs.SEQUENCES)  # and nothing is in the table that holds no coordinate
