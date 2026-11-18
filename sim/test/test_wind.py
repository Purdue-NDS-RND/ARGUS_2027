"""The wind tool, the hover driver and their place in the scenario runner (SIM-03). No ROS, no Gazebo."""

import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import hover  # noqa: E402
import wind  # noqa: E402
import wind_model  # noqa: E402
from judge import default_settings  # noqa: E402
from runner import events, result  # noqa: E402
from scenario.build import BuildError, build_run  # noqa: E402
from wind_model import WindSettings  # noqa: E402

from test_runner import SCORE, make_run  # noqa: E402  (the run-folder fixture)

HOVER = SIM / "scenarios" / "hover" / "wind_5.yaml"
MODERATE = SIM / "scenarios" / "delivery" / "wind_moderate.yaml"
STRONG = SIM / "scenarios" / "delivery" / "wind_strong.yaml"
BASELINE = SIM / "scenarios" / "delivery" / "baseline.yaml"
MASS = 1.755


def sample(speed=5.0, tilt=None, ground_speed=0.1, height=10.0, mass=MASS):
    force = 0.5 * 1.225 * 0.08 * speed * speed
    expected = math.degrees(math.atan2(force, mass * 9.80665))
    return {"speed_mps": speed, "force_n": force, "ground_speed_mps": ground_speed, "height_m": height, "tilt_deg": expected if tilt is None else tilt}


# --- the tool's arithmetic ---


def test_tilt_is_the_angle_between_the_bodys_up_and_the_worlds():
    assert wind.tilt_deg((0.0, 0.0, 0.0, 1.0)) == 0.0
    half = math.radians(10.0) / 2.0
    assert wind.tilt_deg((math.sin(half), 0.0, 0.0, math.cos(half))) == pytest.approx(10.0)  # rolled ten degrees
    assert wind.tilt_deg((0.0, math.sin(half), 0.0, math.cos(half))) == pytest.approx(10.0)  # pitched ten degrees
    assert wind.tilt_deg((0.0, 0.0, math.sin(1.0), math.cos(1.0))) == pytest.approx(0.0, abs=1e-6)  # yawed: still level


def test_velocity_is_the_move_over_the_last_tenth_of_a_second():
    velocity = wind.Velocity()
    assert velocity.add(10.0, (0.0, 0.0, 5.0)) == (0.0, 0.0, 0.0)
    out = None
    for index in range(1, 40):
        t = 10.0 + index * 0.016
        out = velocity.add(t, (2.0 * (t - 10.0), -1.0 * (t - 10.0), 5.0))
    assert out == pytest.approx((2.0, -1.0, 0.0), abs=1e-9)
    still = wind.Velocity()
    still.add(1.0, (1.0, 1.0, 1.0))
    assert still.add(1.0, (9.0, 9.0, 9.0)) == (0.0, 0.0, 0.0)  # two poses of one instant: no velocity from them


def test_the_wind_acts_on_the_vehicle_and_on_every_payload_and_bucket():
    manifest = {"vehicle": {"model": "iris_downcam", "mass_kg": MASS, "cda_m2": 0.08}}
    truth = {"objects": [
        {"name": "parcel", "kind": "payload", "mass_kg": 0.5, "size_m": [0.15, 0.15, 0.10]},
        {"name": "pail", "kind": "bucket", "mass_kg": 1.5, "diameter_m": 0.3, "height_m": 0.35},
        {"name": "target", "kind": "bullseye", "size_m": 5.0},
    ]}  # fmt: skip
    found = wind.bodies(manifest, truth)
    assert set(found) == {"iris_downcam", "parcel", "pail"}  # a mat on the ground is not blown about
    assert found["iris_downcam"] == {"cda_m2": 0.08, "mass_kg": MASS, "vehicle": True}
    assert found["parcel"]["cda_m2"] == pytest.approx(1.05 * 0.15 * 0.15) and found["pail"]["cda_m2"] == pytest.approx(0.3 * 0.35)
    assert wind.body_cda({"kind": "payload", "size_m": 0.2}) == pytest.approx(1.05 * 0.04)
    # An object that names no size is the size the builder builds it: the drag area must be that body's.
    import inspect

    sys.path.insert(0, str(SIM.parent / "src" / "drone_utils"))
    from drone_utils import sdf_objects

    bucket = inspect.signature(sdf_objects.build_bucket_sdf).parameters
    assert (wind.BUCKET_DIAMETER_M, wind.BUCKET_HEIGHT_M) == (0.19, 0.20) and wind.PAYLOAD_SIZE_M == inspect.signature(sdf_objects.build_payload_sdf).parameters["size"].default
    assert wind.body_cda({"kind": "bucket"}) == pytest.approx(0.19 * 0.20) and wind.body_cda({"kind": "payload"}) == pytest.approx(1.05 * 0.15 * 0.15)
    built = (SIM / "scenario" / "build.py").read_text()
    assert 'item.get("diameter_m", 0.19)' in built and 'item.get("height_m", 0.20)' in built and bucket["mass_kg"].default == 1.5
    assert wind.bodies({"vehicle": {"model": "iris_downcam"}}, {})["iris_downcam"]["cda_m2"] == wind_model.IRIS_CDA_M2


def test_a_hover_whose_tilt_is_atan_f_over_mg_meets_the_line_and_one_20_percent_off_does_not():
    settings = WindSettings(mean_mps=5.0, direction_deg=270.0)
    report = wind.summarise([sample() for _ in range(400)], settings, MASS)
    assert report["passed"] and report["hover"]["within_tolerance"] and wind.tilt_problem(report) == ""
    assert report["hover"]["expected_tilt_deg"] == pytest.approx(wind_model.hover_tilt_deg(5.0, MASS)) and report["hover"]["mean_force_n"] == pytest.approx(1.225)
    assert report["wind_speed_mps"] == {"mean": 5.0, "max": 5.0, "min": 5.0} and report["updates"] == 400
    expected = wind_model.hover_tilt_deg(5.0, MASS)
    for factor, ok in ((1.19, True), (0.81, True), (1.21, False), (0.79, False), (0.0, False)):
        report = wind.summarise([sample(tilt=expected * factor) for _ in range(400)], settings, MASS)
        assert report["hover"]["within_tolerance"] is ok, factor
        assert (wind.tilt_problem(report) == "") is ok
    assert "more than 20% apart" in wind.tilt_problem(wind.summarise([sample(tilt=expected * 1.5) for _ in range(400)], settings, MASS))


def test_only_the_hover_counts_toward_the_tilt():
    settings = WindSettings(mean_mps=5.0)
    moving = [sample(tilt=20.0, ground_speed=4.0) for _ in range(300)]  # flying somewhere, tilted for that
    low = [sample(tilt=0.0, height=0.5) for _ in range(300)]  # on the ground
    report = wind.summarise(moving + low + [sample() for _ in range(150)], settings, MASS)
    assert report["hover"]["updates"] == 150 and report["hover"]["within_tolerance"]
    brief = wind.summarise(moving + [sample() for _ in range(99)], settings, MASS)
    assert brief["passed"] and brief["hover"] is None  # the wind blew; there was no hover to judge
    assert "did not hover in the wind for 5 s" in wind.tilt_problem(brief)


def test_a_flight_the_wind_never_reached_says_so(tmp_path):
    report = wind.summarise([], WindSettings(mean_mps=5.0), MASS)
    assert not report["passed"] and report["problems"] == ["the wind never blew: no pose of the vehicle was heard"]
    assert wind.read_wind(tmp_path) is None and wind.tilt_problem(None) == "the run left no wind.json"
    (tmp_path / "wind.json").write_text(json.dumps(report))
    assert wind.read_wind(tmp_path)["passed"] is False
    (tmp_path / "wind.json").write_text("{torn")
    assert wind.read_wind(tmp_path) is None
    assert wind.main([]) == 2


# --- the hover driver ---


class FakeVehicle:
    """A vehicle that takes off, drifts a little and lands a while after it is told to come home."""

    def __init__(self, simulator=True, lands_after_s=30.0, drift_mps=0.02):
        self.simulator, self.lands_after_s, self.drift_mps = simulator, lands_after_s, drift_mps
        self.t, self.armed, self.height, self.home_at = 0.0, False, 0.0, None

    def is_simulator(self):
        return self.simulator

    def now(self):
        return self.t

    def is_armed(self):
        return self.armed

    def takeoff(self, height_m):
        self.armed, self.height, self.t = True, height_m, self.t + 15.0

    def step(self):
        self.t += 0.05
        if self.home_at is not None and self.t - self.home_at >= self.lands_after_s:
            self.armed = False
        return SimpleNamespace(t=self.t, e=self.drift_mps * self.t, n=0.0, u=self.height, mode="GUIDED")

    def return_home(self):
        self.home_at = self.t


def test_the_hover_holds_for_its_time_then_comes_home_and_lands():
    vehicle = FakeVehicle()
    report = hover.run_hover(vehicle, height_m=10.0, hold_s=40.0)
    assert report["passed"] and report["landed"] and report["problem"] == ""
    assert report["held_s"] == pytest.approx(40.0, abs=0.1) and report["max_drift_m"] == pytest.approx(0.02 * 40.0, abs=0.01)
    assert vehicle.home_at == pytest.approx(15.05 + 40.0, abs=0.1)  # told to come home only after the hold


def test_the_hover_refuses_a_real_autopilot_and_reports_a_vehicle_that_does_not_land():
    real = hover.run_hover(FakeVehicle(simulator=False))
    assert not real["passed"] and "not SITL" in real["problem"] and real["held_s"] == 0.0
    stuck = hover.run_hover(FakeVehicle(lands_after_s=1.0e9), hold_s=5.0, land_timeout_s=20.0)
    assert not stuck["passed"] and not stuck["landed"] and "had not landed" in stuck["problem"]


# --- the scenarios and the runner ---


def test_a_windy_scenario_builds_a_world_with_the_wrench_system_and_a_calm_one_does_not(tmp_path):
    windy = build_run(MODERATE, tmp_path / "windy")
    calm = build_run(BASELINE, tmp_path / "calm")
    assert (windy / "world.sdf").read_text().count("gz-sim-apply-link-wrench-system") == 1
    assert "apply-link-wrench" not in (calm / "world.sdf").read_text()
    manifest = json.loads((windy / "scenario.json").read_text())
    assert manifest["wind"] == {"level": "moderate", "mean_mps": 4.0, "gust_mps": 1.0, "discrete_gust_mps": 0.0, "direction_deg": 225.0}
    assert manifest["vehicle"]["mass_kg"] == 1.755 and manifest["vehicle"]["cda_m2"] == 0.08
    assert wind.wanted(manifest) and not wind.wanted(json.loads((calm / "scenario.json").read_text()))
    strong = json.loads((build_run(STRONG, tmp_path / "strong") / "scenario.json").read_text())["wind"]
    assert (strong["mean_mps"], strong["gust_mps"], strong["discrete_gust_mps"]) == (7.0, 2.0, 3.0)
    # The windy delivery flies the very sequence the calm one does.
    assert (windy / "sequence.yaml").read_text().split("\n", 1)[1] == (calm / "sequence.yaml").read_text().split("\n", 1)[1]  # all but the "generated from" line


def test_the_hover_scenario_is_a_driver_flight_in_a_steady_5_m_s_wind(tmp_path):
    run = build_run(HOVER, tmp_path / "hover")
    manifest = json.loads((run / "scenario.json").read_text())
    assert manifest["mission"] == "hover" and manifest["wind"]["mean_mps"] == 5.0 and manifest["wind"]["gust_mps"] == 0.0
    command = manifest["driver"]["command"]
    assert manifest["driver"]["tool"] == "hover" and command[:4] == ["env", "-u", "PYTHONNOUSERSITE", "python3"]
    assert command[4].endswith("sim/hover.py") and command[command.index("--height") + 1] == "10.0" and command[command.index("--hold") + 1] == "40.0"
    assert command[-1] == str(run / "hover.json") and manifest["launch"] is None
    assert default_settings("hover")["max_drift_m"] == 3.0
    bad = tmp_path / "bad.yaml"
    bad.write_text(HOVER.read_text().replace("hold_s: 40.0", "speed_mps: 3.0"))
    with pytest.raises(BuildError, match="driver hover has unknown setting"):
        build_run(bad, tmp_path / "bad")
    # The fence check's command is as it was.
    fence = json.loads((build_run(SIM / "scenarios" / "geofence" / "breach.yaml", tmp_path / "fence") / "scenario.json").read_text())["driver"]["command"]
    assert fence[4].endswith("sim/fence_check.py") and "--speed" in fence and fence[-1].endswith("fence_check.json")


def test_wind_runs_beside_windy_flights_and_only_those():
    windy = {"wind": {"mean_mps": 4.0, "gust_mps": 1.0, "discrete_gust_mps": 0.0}, "gps_events": []}
    calm = {"wind": {"level": "calm", "mean_mps": 0.0, "gust_mps": 0.0, "discrete_gust_mps": 0.0}, "gps_events": []}
    assert events.tools_needed(windy) == {"wind"} and "wind" not in events.tools_needed(calm)
    assert events.unsupported(windy, {"wind"}) == "" and "sim/wind.py" in events.unsupported(windy, set())
    assert '"wind": SIM_DIR / "wind.py"' in (SIM / "runner" / "fly.py").read_text() and (SIM / "wind.py").is_file()


BLEW = {"passed": True, "problems": [], "updates": 900, "settings": {"direction_deg": 270.0}, "wind_speed_mps": {"mean": 5.0, "max": 5.0, "min": 5.0}, "hover": None}
HOVERED = {**BLEW, "hover": {"updates": 700, "mean_force_n": 1.22, "mean_tilt_deg": 4.2, "expected_tilt_deg": 4.07, "mass_kg": 1.755, "within_tolerance": True}}
TILTED = {**BLEW, "hover": {"updates": 700, "mean_force_n": 1.22, "mean_tilt_deg": 6.5, "expected_tilt_deg": 4.07, "mass_kg": 1.755, "within_tolerance": False}}


def windy_run(tmp_path, report, driver=None):
    manifest = {"variant": "off", "wind": {"mean_mps": 5.0, "gust_mps": 0.0, "discrete_gust_mps": 0.0, "direction_deg": 270.0}}
    if driver:
        manifest["driver"] = {"tool": driver, "command": []}
    run = make_run(tmp_path, score=SCORE, manifest=manifest)
    if report is not None:
        (run / "wind.json").write_text(json.dumps(report))
    return run


def test_a_windy_run_counts_only_if_the_wind_blew(tmp_path):
    blown = result.run_result(windy_run(tmp_path / "a", BLEW))
    assert blown["status"] == "OK"
    assert "| Wind | mean 5.0 m/s, 5.0 to 5.0 m/s, from 270 degrees, over 900 updates |" in result.render_report(blown)
    assert "Hover tilt" not in result.render_report(blown)
    absent = result.run_result(windy_run(tmp_path / "b", None))
    assert absent["status"] == "INVALID" and "wind.json" in absent["reason"]
    never = result.run_result(windy_run(tmp_path / "c", {**BLEW, "passed": False, "updates": 0, "wind_speed_mps": None}))
    assert never["status"] == "INVALID"


def test_the_hover_flight_must_also_meet_the_tilt_line(tmp_path):
    passed = result.run_result(windy_run(tmp_path / "a", HOVERED, driver="hover"))
    assert passed["status"] == "OK"
    assert "| Hover tilt in the wind | PASS: 4.20 degrees against atan(F / mg) = 4.07 degrees (force 1.22 N, mass 1.755 kg, limit 20%) |" in result.render_report(passed)
    tilted = result.run_result(windy_run(tmp_path / "b", TILTED, driver="hover"))
    assert tilted["status"] == "FAIL" and "6.50 degrees where atan(F / mg) gives 4.07" in tilted["reason"]
    no_hover = result.run_result(windy_run(tmp_path / "c", BLEW, driver="hover"))
    assert no_hover["status"] == "FAIL" and "did not hover" in no_hover["reason"]
    # A windy mission flight is not held to the tilt line: it is not a hover test.
    assert result.run_result(windy_run(tmp_path / "d", TILTED))["status"] == "OK"


def test_a_calm_run_needs_no_wind_record(tmp_path):
    run = make_run(tmp_path, score=SCORE, manifest={"variant": "off", "wind": {"level": "calm", "mean_mps": 0.0, "gust_mps": 0.0, "discrete_gust_mps": 0.0}})
    summary = result.run_result(run)
    assert summary["status"] == "OK" and summary["wind"] is None and "| Wind |" not in result.render_report(summary)


def test_the_windy_scenarios_are_in_the_nightly_suite():
    import yaml

    nightly = yaml.safe_load((SIM / "scenarios" / "suites.yaml").read_text())["nightly"]["scenarios"]
    assert {"hover/wind_5", "delivery/wind_moderate", "delivery/wind_strong"} <= set(nightly)
