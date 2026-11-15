"""sim/site.py: the competition-site file, its world and its geofence plan.

The site file is the one place the field's coordinates live: the sim world,
SITL's home, the fence uploaded to the flight controller and the judge's
truth all come from it. A site that puts the takeoff point outside its own
fence would refuse to arm, so the loader checks the geometry. No ROS needed.
"""

import copy
import json
import math
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import site_file  # noqa: E402
import target_placement as tp  # noqa: E402
from judge.geometry import point_in_polygon  # noqa: E402

APOLLO = SIM / "sites" / "apollo_xi.yaml"
BASE_WORLD = SIM / "worlds" / "argus_runway.sdf"


def raw():
    return yaml.safe_load(APOLLO.read_text())


def load(data, tmp_path):
    path = tmp_path / "site.yaml"
    path.write_text(yaml.safe_dump(data))
    return site_file.load_site(path)


# --- the shipped site -------------------------------------------------------


def test_apollo_xi_loads_and_says_it_is_not_verified():
    site = site_file.load_site(APOLLO)
    assert site.name == "apollo_xi"
    assert site.verified is False  # coordinates still to check against the field map
    assert 34.0 < site.origin.latitude_deg < 34.4 and -118.7 < site.origin.longitude_deg < -118.3
    assert site.origin.heading_deg == 0.0
    assert site.origin.elevation_m > 100.0


def test_apollo_xi_geometry_is_consistent():
    site = site_file.load_site(APOLLO)
    assert len(site.boundary) >= 4
    assert point_in_polygon(*site.takeoff, site.boundary)
    assert point_in_polygon(*site.compass_rose, site.boundary)
    assert all(point_in_polygon(e, n, site.boundary) for e, n in site.survey_area)
    assert math.hypot(*site.takeoff) < 1.0  # the vehicle spawns at the world origin
    assert 30.0 <= site.max_altitude_agl_m <= 122.0  # never above 400 ft


def test_boundary_is_in_metres_from_the_origin_as_ardupilot_computes_them():
    site = site_file.load_site(APOLLO)
    first = raw()["flight_boundary"][0]
    north, east = tp.offset_ne_m(
        site.origin.latitude_deg, site.origin.longitude_deg, first["latitude_deg"], first["longitude_deg"]
    )
    assert site.boundary[0] == pytest.approx((east, north))
    assert site.to_enu(first["latitude_deg"], first["longitude_deg"]) == pytest.approx((east, north))


# --- what the judge and the scenario builder take from it -------------------


def test_truth_block_has_what_the_judges_read():
    site = site_file.load_site(APOLLO)
    truth = site_file.truth_block(site)
    assert truth["origin"] == {
        "latitude_deg": site.origin.latitude_deg,
        "longitude_deg": site.origin.longitude_deg,
        "elevation_m": site.origin.elevation_m,
    }
    assert truth["takeoff"] == {"e": site.takeoff[0], "n": site.takeoff[1]}
    assert truth["fence"]["max_height_m"] == site.max_altitude_agl_m
    assert [tuple(p) for p in truth["fence"]["polygon"]] == [tuple(p) for p in site.boundary]
    assert set(truth["return_points"]) == {"takeoff", "compass_rose"}
    assert len(truth["survey_area"]) == len(site.survey_area)
    json.dumps(truth)  # plain data only


def test_truth_block_feeds_the_judge_fence_report():
    from judge import judge_run
    from judge.record import Record

    site = site_file.load_site(APOLLO)
    east_edge = max(e for e, _ in site.boundary)
    lines = [{"t": 0.0, "event": "armed"}, {"t": 100.0, "event": "disarmed"}]
    knots = [(0, 0, 0, 0), (10, 0, 0, 20), (40, east_edge + 6.0, 0, 20), (70, 0, 0, 20), (90, 0, 0, 0)]
    for (t0, e0, n0, u0), (t1, e1, n1, u1) in zip(knots, knots[1:]):
        for i in range(60):
            f = i / 60
            lines.append({"t": t0 + (t1 - t0) * f, "pose": "vehicle", "e": e0 + (e1 - e0) * f, "n": n0 + (n1 - n0) * f, "u": u0 + (u1 - u0) * f})
    truth = {**site_file.truth_block(site), "waypoints": [{"name": "wp1", "e": 20.0, "n": 0.0, "u": 20.0}]}
    score = judge_run("waypoint_navigation", Record.from_lines(lines), truth)
    breaches = score["common"]["fence_breaches"]
    assert len(breaches) == 1 and breaches[0]["kind"] == "lateral"
    assert breaches[0]["max_excursion_m"] == pytest.approx(6.0, abs=0.3)


# --- the fence plan ----------------------------------------------------------


def test_fence_plan_is_one_inclusion_polygon_with_every_vertex():
    site = site_file.load_site(APOLLO)
    plan = site_file.fence_plan(site)
    boundary = raw()["flight_boundary"]
    assert len(plan.items) == len(boundary)
    for seq, (item, vertex) in enumerate(zip(plan.items, boundary)):
        assert item["seq"] == seq
        assert item["command"] == 5001  # MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        assert item["frame"] == 0  # MAV_FRAME_GLOBAL
        assert item["param1"] == len(boundary)
        assert item["lat_e7"] == round(vertex["latitude_deg"] * 1e7)
        assert item["lon_e7"] == round(vertex["longitude_deg"] * 1e7)


def test_fence_plan_turns_on_the_polygon_and_height_fence_with_rtl():
    site = site_file.load_site(APOLLO)
    params = site_file.fence_plan(site).params
    assert params["FENCE_ENABLE"] == 1
    assert params["FENCE_TYPE"] == 5  # bit 0 maximum height, bit 2 polygon; no circle
    assert params["FENCE_ACTION"] == 1  # RTL, or land if RTL is not possible
    assert params["FENCE_ALT_MAX"] == site.max_altitude_agl_m
    assert params["FENCE_MARGIN"] == site.fence_margin_m


# --- the site world ----------------------------------------------------------


def test_site_world_moves_the_origin_and_keeps_the_vehicle_at_it():
    site = site_file.load_site(APOLLO)
    text = site_file.site_world(BASE_WORLD.read_text(), site)
    name, origin = tp.read_world(text)
    assert name == "apollo_xi"
    assert origin.latitude_deg == pytest.approx(site.origin.latitude_deg)
    assert origin.longitude_deg == pytest.approx(site.origin.longitude_deg)
    assert origin.elevation_m == pytest.approx(site.origin.elevation_m)
    assert origin.heading_deg == 0.0
    assert "model://iris_downcam" in text
    assert "<pose degrees=\"true\">0 0 0.195 0 0 90</pose>" in text  # the vehicle is untouched
    # SITL home comes from the world, so it is the site origin too.
    assert tp.sitl_location_arg(origin).startswith(f"{site.origin.latitude_deg},{site.origin.longitude_deg},")


def test_site_world_lays_the_runway_strip_along_the_site_runway():
    data = raw()
    site = site_file.load_site(APOLLO)
    text = site_file.site_world(BASE_WORLD.read_text(), site)
    import re

    pose = re.search(r"<uri>model://runway</uri>\s*<pose degrees=\"true\">([^<]+)</pose>", text)
    assert pose, "the runway include (the ground plane) must stay in the world"
    x, y, z, roll, pitch, yaw = (float(v) for v in pose.group(1).split())
    assert (x, y) == pytest.approx(site.runway.centre)
    assert (z, roll, pitch) == (0.0, 0.0, 0.0)
    # The model's strip runs along its +y axis; compass heading is clockwise from north.
    assert yaw == pytest.approx(-data["runway"]["heading_deg"])


def test_base_world_is_not_changed_by_loading_a_site():
    before = BASE_WORLD.read_text()
    site_file.site_world(before, site_file.load_site(APOLLO))
    assert BASE_WORLD.read_text() == before
    assert tp.read_world(before)[0] == "argus_runway"


# --- a bad site file is refused, and the message names the problem -----------


def broken(change):
    data = raw()
    change(data)
    return data


def shift(point, d_lat=0.0, d_lon=0.0):
    return {"latitude_deg": point["latitude_deg"] + d_lat, "longitude_deg": point["longitude_deg"] + d_lon}


@pytest.mark.parametrize(
    "change, message",
    [
        (lambda d: d.pop("flight_boundary"), "flight_boundary"),
        (lambda d: d.update(flight_boundary=d["flight_boundary"][:2]), "at least 3"),
        (lambda d: d.update(flight_boundary=[d["flight_boundary"][i] for i in (0, 2, 1, 3)]), "crosses itself"),
        (lambda d: d.update(takeoff_point=shift(d["origin"], d_lat=0.01)), "takeoff_point"),
        (lambda d: d.update(compass_rose=shift(d["origin"], d_lon=0.02)), "compass_rose"),
        (lambda d: d.update(survey_area=[shift(p, d_lat=0.01) for p in d["survey_area"]]), "survey_area"),
        (lambda d: d.update(max_altitude_agl_m=0.0), "max_altitude_agl_m"),
        (lambda d: d.update(max_altitude_agl_m=150.0), "400 ft"),
        (lambda d: d["origin"].update(latitude_deg=95.0), "latitude"),
        (lambda d: d.update(terain_elevation_m=3.0), "terain_elevation_m"),
        (lambda d: d["fence"].update(action=0), "fence.action"),
        (lambda d: d["fence"].update(margin_m=40.0), "margin"),
    ],
)
def test_bad_site_is_refused(change, message, tmp_path):
    with pytest.raises(site_file.SiteError, match=message):
        load(broken(change), tmp_path)


def test_takeoff_point_must_be_clear_of_the_fence(tmp_path):
    data = raw()
    site = site_file.load_site(APOLLO)
    east_edge = max(e for e, _ in site.boundary)
    # 3 m inside the east edge: inside the fence, but the fence margin plus GPS error would breach on the ground.
    lon_per_m = 1.0 / (tp.LOCATION_SCALING_FACTOR * 1e7 * math.cos(math.radians(site.origin.latitude_deg)))
    data["takeoff_point"] = shift(data["origin"], d_lon=(east_edge - 3.0) * lon_per_m)
    with pytest.raises(site_file.SiteError, match="takeoff_point.*from the flight boundary"):
        load(data, tmp_path)


def test_original_site_data_survives_the_broken_copies(tmp_path):
    assert load(copy.deepcopy(raw()), tmp_path).name == "apollo_xi"


# --- the command line ---------------------------------------------------------


def run_cli(*args):
    return subprocess.run([sys.executable, str(SIM / "site_file.py"), *map(str, args)], capture_output=True, text=True)


def test_cli_check_prints_a_summary_and_warns_that_the_site_is_unverified():
    result = run_cli("check", APOLLO)
    assert result.returncode == 0, result.stderr
    assert "apollo_xi" in result.stdout
    assert "NOT VERIFIED" in result.stdout


def test_cli_world_writes_a_world_file(tmp_path):
    out = tmp_path / "worlds" / "apollo_xi.sdf"
    result = run_cli("world", APOLLO, "--out", out)
    assert result.returncode == 0, result.stderr
    assert tp.read_world(out.read_text())[0] == "apollo_xi"


def test_cli_truth_prints_json():
    result = run_cli("truth", APOLLO)
    assert result.returncode == 0, result.stderr
    assert "fence" in json.loads(result.stdout)


def test_cli_refuses_a_bad_site(tmp_path):
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(broken(lambda d: d.pop("origin"))))
    result = run_cli("check", path)
    assert result.returncode == 2
    assert "origin" in result.stderr


# --- the sim start-up scripts -------------------------------------------------


@pytest.mark.parametrize("script", ["container/start.sh", "run_sim.sh"])
def test_start_up_scripts_parse(script):
    assert subprocess.run(["bash", "-n", str(SIM / script)], capture_output=True, text=True).returncode == 0


def test_start_up_only_touches_the_fence_and_the_world_when_a_site_is_set():
    """With no site the sim must start as it did in 2026: same world, no fence, no wipe."""
    lines = (SIM / "container" / "start.sh").read_text().splitlines()
    text = "\n".join(line for line in lines if not line.lstrip().startswith("#")) + "\n"
    blocks = []  # the bodies of `if [[ -n $SITE_FILE ... ]]` blocks
    for start in [i for i in range(len(text)) if text.startswith("if [[ -n $SITE_FILE", i)]:
        blocks.append(text[start : text.index("\nfi\n", start)])
    guarded = "\n".join(blocks)
    for site_only in ("fence_upload.py", "site_file.py", "SIM_TERRAIN", "$TOOLS_PORT\"", "--add-param-file"):
        assert site_only in guarded, site_only
        assert text.count(site_only) == guarded.count(site_only), f"{site_only} is used outside a site check"
    assert 'WORLD_FILE="$ARGUS_SIM/worlds/argus_runway.sdf"' in text


# --- when SITL's stored parameters are wiped -----------------------------------


def wipe_needed(sitl_dir, site="", wipe_params="0"):
    script = 'source "$1"; sitl_wipe_needed "$2" "$3" "$4"'
    result = subprocess.run(
        ["bash", "-c", script, "wipe", str(SIM / "container" / "sitl_storage.sh"), str(sitl_dir), site, wipe_params],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    return result.stdout.strip()


def mark(sitl_dir, site=""):
    script = 'source "$1"; sitl_mark_storage "$2" "$3"'
    subprocess.run(["bash", "-c", script, "mark", str(SIM / "container" / "sitl_storage.sh"), str(sitl_dir), site], check=True)


def test_plain_runs_do_not_wipe_sitl_storage(tmp_path):
    """The 2026 behaviour: a plain start keeps SITL's stored parameters."""
    assert wipe_needed(tmp_path) == "0"  # first start ever
    mark(tmp_path)
    assert wipe_needed(tmp_path) == "0"
    assert wipe_needed(tmp_path, wipe_params="1") == "1"  # unless asked


def test_site_runs_always_wipe_sitl_storage(tmp_path):
    assert wipe_needed(tmp_path, site=str(APOLLO)) == "1"
    mark(tmp_path, str(APOLLO))
    assert wipe_needed(tmp_path, site=str(APOLLO)) == "1"


def test_first_plain_run_after_a_site_run_wipes_once(tmp_path):
    mark(tmp_path, str(APOLLO))
    assert wipe_needed(tmp_path) == "1"  # the site's fence must not follow the vehicle
    assert wipe_needed(tmp_path) == "1"  # still, if that start was interrupted before SITL came up
    mark(tmp_path)  # start.sh marks a plain run once SITL is up
    assert wipe_needed(tmp_path) == "0"


def test_start_up_wipes_only_when_the_rule_says_so():
    lines = (SIM / "container" / "start.sh").read_text().splitlines()
    code = [line.strip() for line in lines if not line.lstrip().startswith("#")]
    assert 'wipe=$(sitl_wipe_needed "$SITL_DIR" "$SITE_FILE" "${WIPE_PARAMS:-0}")' in code
    at = code.index("sitl_args+=(--wipe-eeprom)")
    assert code[at - 1] == "if ((wipe)); then" and code[at + 1] == "fi"
    assert sum("--wipe-eeprom" in line for line in code) == 1
    # A plain run clears the mark only after SITL is up; a site run sets it before SITL starts.
    plain_mark = code.index('sitl_mark_storage "$SITL_DIR" ""  # SITL is up, so any wipe has happened')
    assert plain_mark > next(i for i, line in enumerate(code) if "MAVROS connected." in line)
    assert code.index('sitl_mark_storage "$SITL_DIR" "$SITE_FILE"') < next(i for i, line in enumerate(code) if line.startswith("start sitl "))
