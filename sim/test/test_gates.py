"""sim/gate.py: the acceptance gates G0 to G8 as data, and the check of a gate's runs.

A gate's verdict decides whether a lane's work counts, so the rules are
checked here on hand-made results, on both sides of every limit: a result
must never be rounded into a pass, and a run the sim was too slow to count
must make the gate INVALID, not PASS or FAIL. No ROS needed.
"""

import json
import subprocess
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
ROOT = SIM.parent
sys.path.insert(0, str(SIM))

import gate  # noqa: E402
from runner import suites  # noqa: E402

LEDGER = json.loads((ROOT / "docs" / "build" / "features.json").read_text())


def run(status="OK", scenario="delivery/baseline", seed=1, variant="off", wind="calm", mission="delivery", settings=None, base_points=10.0, **details):
    """One flight's result as the runner leaves it (result.json plus its score and manifest)."""
    return {
        "scenario": scenario, "seed": seed, "variant": variant, "status": status, "reason": "" if status == "OK" else f"it was {status}",
        "points": 10.0, "real_time_factor": 0.97, "headline": "",
        "score": {"mission": mission, "base_points": base_points, "details": details, "settings": settings or {}, "common": {"fence_breaches": [], "failsafes": []}},
        "manifest": {"mission": mission, "wind": {"level": wind}, "variant": variant, "seed": seed},
    }  # fmt: skip


def deliveries(distances, **kwargs):
    return [run(seed=i + 1, distance_m=d, **kwargs) for i, d in enumerate(distances)]


# --- the gates as shipped ----------------------------------------------------------


def test_every_gate_in_the_ledger_is_defined_with_its_pass_line():
    gates = gate.load_gates()
    assert list(gates) == ["G0", "G1", "G2", "G3", "G4", "G5", "G6", "G7", "G8"]
    for name in LEDGER["gates"]:
        assert gate.resolve(name) in gates, name  # G0-W2, G0-W3 and G0-W4 are G0 again
    assert gate.resolve("G0-W3") == "G0"
    for name, definition in gates.items():
        assert definition["proves"] and definition["pass"], name
    assert gates["G0"]["pass"] == "Delivery touchdown ≤ 0.3 m in 5/5"


def test_gate_pass_lines_are_the_ones_in_the_spec():
    spec = (ROOT / "docs" / "build" / "FEATURES.md").read_text()
    for name, definition in gate.load_gates().items():
        row = next(line for line in spec.splitlines() if line.startswith(f"| {name} |"))
        assert definition["pass"] in row, name
        assert definition["proves"] in row, name


def test_g0_flies_the_2026_delivery_five_times_with_flags_off():
    flights = gate.flights("G0")
    assert [(f["scenario"].relative_to(SIM / "scenarios").as_posix(), f["seed"], f["variant"]) for f in flights] == [
        ("delivery/baseline.yaml", seed, "off") for seed in (1, 2, 3, 4, 5)
    ]
    assert gate.flights("G0-W2") == flights


def test_g1_flies_a_hold_at_6_m_and_at_2_m_in_moderate_and_in_strong_wind_with_the_flags_on():
    flights = gate.flights("G1")
    assert sorted(flight["scenario"].name for flight in flights) == ["hold_2m_moderate.yaml", "hold_2m_strong.yaml", "hold_6m_moderate.yaml", "hold_6m_strong.yaml"]
    assert {(flight["seed"], flight["variant"]) for flight in flights} == {(1, "on")}
    import yaml as real_yaml

    for flight in flights:
        scenario = real_yaml.safe_load(flight["scenario"].read_text())
        assert scenario["flags"] == {"use_frame_time_pose": True, "use_target_estimator": True, "centre_on_estimator": True, "centring_retune": True}
        assert scenario["wind"]["level"] in flight["scenario"].name and scenario["judge"]["hold_height_m"] in (6.0, 2.0)


def test_gates_without_scenarios_yet_say_what_they_wait_for():
    gates = gate.load_gates()
    for name in ("G1", "G2", "G3", "G4", "G5", "G6"):
        assert gate.flights(name) == [] or all(f["scenario"].is_file() for f in gate.flights(name))
        if not gates[name].get("scenarios"):
            assert gates[name]["needs"], name
    for name in ("G7", "G8"):
        assert gates[name]["waiting"], name
        assert gate.flights(name) == []


def test_the_runner_knows_each_gate_as_a_suite_and_all_of_them_as_gates():
    assert suites.expand("G0") == gate.flights("G0")
    assert suites.expand("G0-W2") == gate.flights("G0")
    every = suites.expand("gates")
    assert every[: len(gate.flights("G0"))] == gate.flights("G0")
    with pytest.raises(ValueError, match="G7"):
        suites.expand("G7")  # field data: nothing to fly in the sim


# --- G0 ------------------------------------------------------------------------------


def test_g0_passes_on_five_touchdowns_within_30_cm():
    verdict = gate.check("G0", deliveries([0.02, 0.11, 0.30, 0.09, 0.05]))
    assert verdict["verdict"] == "PASS"
    assert all(rule["passed"] for rule in verdict["rules"])


def test_g0_fails_when_one_touchdown_is_just_outside():
    verdict = gate.check("G0", deliveries([0.02, 0.11, 0.31, 0.09, 0.05]))
    assert verdict["verdict"] == "FAIL"
    failed = [rule for rule in verdict["rules"] if not rule["passed"]]
    assert len(failed) == 1 and "0.31" in failed[0]["found"]


def test_g0_fails_on_a_flight_that_did_not_finish_or_released_nothing():
    for bad in (run(status="TIMEOUT", seed=5, distance_m=0.1), run(status="NEVER_ARMED", seed=5), run(status="ERROR", seed=5), run(seed=5, distance_m=None)):
        verdict = gate.check("G0", deliveries([0.02, 0.11, 0.2, 0.09]) + [bad])
        assert verdict["verdict"] == "FAIL", bad["status"]


def test_g0_is_invalid_when_a_needed_flight_was_too_slow_to_count():
    verdict = gate.check("G0", deliveries([0.02, 0.11, 0.2, 0.09]) + [run(status="INVALID", seed=5, distance_m=0.03)])
    assert verdict["verdict"] == "INVALID"
    assert "seed 5" in verdict["reason"]


def test_gate_already_lost_is_fail_even_with_an_invalid_flight_outstanding():
    verdict = gate.check("G0", deliveries([0.02, 0.45, 0.2, 0.09]) + [run(status="INVALID", seed=5, distance_m=0.03)])
    assert verdict["verdict"] == "FAIL"  # flying seed 5 again cannot bring the 0.45 m back


def test_an_invalid_flight_that_was_flown_again_does_not_spoil_the_gate():
    runs = deliveries([0.02, 0.11, 0.2, 0.09]) + [run(status="INVALID", seed=5, distance_m=0.03), run(seed=5, distance_m=0.04)]
    assert gate.check("G0", runs)["verdict"] == "PASS"


def test_a_failed_flight_cannot_be_flown_again_into_a_pass():
    """Re-flying is for runs the sim spoiled, not for runs the vehicle lost."""
    runs = deliveries([0.02, 0.11, 0.2, 0.09]) + [run(seed=5, distance_m=0.6), run(seed=5, distance_m=0.04)]
    verdict = gate.check("G0", runs)
    assert verdict["verdict"] == "FAIL"
    assert "0.6" in json.dumps(verdict)


def test_g0_fails_when_a_flight_is_missing_or_was_flown_with_flags_on():
    assert gate.check("G0", deliveries([0.02, 0.11, 0.2, 0.09]))["verdict"] == "FAIL"
    with_flags = deliveries([0.02, 0.11, 0.2, 0.09]) + [run(seed=5, variant="on", distance_m=0.04)]
    verdict = gate.check("G0", with_flags)
    assert verdict["verdict"] == "FAIL" and "seed 5" in verdict["reason"]


def test_gate_with_nothing_to_fly_yet_cannot_be_checked(monkeypatch):
    # (Every sim gate has its flights now; a gate that names what it needs and no scenarios is still refused.)
    gates = gate.load_gates()
    waiting = {key: value for key, value in gates["G6"].items() if key not in ("scenarios", "seeds")}
    monkeypatch.setattr(gate, "definition_of", lambda name: {**waiting, "needs": "scenarios nobody has written"})
    monkeypatch.setattr(gate, "flights", lambda name: [])
    with pytest.raises(gate.GateError, match="cannot run yet: it needs scenarios nobody has written"):
        gate.check("G6", [])
    monkeypatch.undo()
    with pytest.raises(gate.GateError, match="field"):
        gate.check("G7", [])


# --- the rule kinds --------------------------------------------------------------------


def evaluate(rule, runs):
    return gate.evaluate_rule(rule, runs)


def test_rule_each_needs_every_run_inside_the_limit():
    rule = {"name": "x", "metric": "details.distance_m", "each": {"max": 0.3}}
    assert evaluate(rule, deliveries([0.1, 0.3]))["passed"] is True
    assert evaluate(rule, deliveries([0.1, 0.30001]))["passed"] is False
    assert evaluate(rule, deliveries([0.1, None]))["passed"] is False  # no value is not a pass
    assert evaluate(rule, [])["passed"] is False  # nor is no run


def test_rule_median_maximum_and_p95():
    values = [0.10, 0.12, 0.14, 0.16, 0.90]
    runs = deliveries(values)
    assert evaluate({"name": "m", "metric": "details.distance_m", "median": {"max": 0.15}}, runs)["passed"] is True
    assert evaluate({"name": "m", "metric": "details.distance_m", "median": {"max": 0.13}}, runs)["passed"] is False
    assert evaluate({"name": "m", "metric": "details.distance_m", "maximum": {"max": 1.0}}, runs)["passed"] is True
    assert evaluate({"name": "m", "metric": "details.distance_m", "maximum": {"max": 0.89}}, runs)["passed"] is False
    p95 = evaluate({"name": "m", "metric": "details.distance_m", "p95": {"max": 0.75}}, runs)
    assert p95["passed"] is False and p95["value"] == pytest.approx(0.752)  # linear interpolation between ranks
    assert evaluate({"name": "m", "metric": "details.distance_m", "minimum": {"min": 0.10}}, runs)["passed"] is True
    assert evaluate({"name": "m", "metric": "details.distance_m", "minimum": {"min": 0.11}}, runs)["passed"] is False


def test_rule_where_picks_runs_by_their_scenario():
    runs = deliveries([0.10, 0.12], wind="calm") + deliveries([0.25, 0.28], wind="strong")
    calm = {"name": "calm", "metric": "details.distance_m", "where": {"wind.level": "calm"}, "median": {"max": 0.15}}
    strong = {"name": "strong", "metric": "details.distance_m", "where": {"wind.level": "strong"}, "median": {"max": 0.15}}
    assert evaluate(calm, runs)["passed"] is True
    assert evaluate(strong, runs)["passed"] is False
    assert evaluate({**calm, "where": {"wind.level": "moderate"}}, runs)["passed"] is False  # no such run flown


def test_rule_each_equals_for_flags_that_must_not_happen():
    rule = {"name": "no low release", "metric": "details.released_low", "each": {"equals": False}}
    assert evaluate(rule, [run(released_low=False), run(released_low=False)])["passed"] is True
    assert evaluate(rule, [run(released_low=False), run(released_low=True)])["passed"] is False


def test_rule_count_is_so_many_runs_out_of_those_flown():
    rule = {"name": "8 of 10", "metric": "details.lifted", "count": {"equals": True, "at_least": 8}}
    assert evaluate(rule, [run(lifted=True)] * 8 + [run(lifted=False)] * 2)["passed"] is True
    assert evaluate(rule, [run(lifted=True)] * 7 + [run(lifted=False)] * 3)["passed"] is False
    assert evaluate(rule, [run(lifted=True)] * 7 + [run(status="TIMEOUT")] * 3)["passed"] is False


def test_rule_ratio_adds_up_over_runs():
    rule = {"name": "95% correct", "ratio": {"of": "details.correct", "over": "details.markers", "min": 0.95}}
    assert evaluate(rule, [run(correct=10, markers=10), run(correct=9, markers=10)])["passed"] is True  # 19 of 20
    assert evaluate(rule, [run(correct=10, markers=10), run(correct=8, markers=10)])["passed"] is False  # 18 of 20


def test_rule_pooled_takes_every_value_of_a_list_metric():
    rule = {"name": "p95 error", "metric": "details.errors_m", "pooled": True, "p95": {"max": 2.0}}
    assert evaluate(rule, [run(errors_m=[0.2, 0.4, 0.5]), run(errors_m=[0.3, 1.9])])["passed"] is True
    assert evaluate(rule, [run(errors_m=[0.2, 0.4, 0.5]), run(errors_m=[0.3, 2.6])])["passed"] is False


def test_rule_flown_counts_the_judged_runs_of_a_kind():
    rule = {"name": "five waypoint flights", "where": {"mission": "waypoint_navigation"}, "flown": {"at_least": 5}}
    assert evaluate(rule, [run(mission="waypoint_navigation")] * 5 + [run(mission="time_trial")] * 9)["passed"] is True
    assert evaluate(rule, [run(mission="waypoint_navigation")] * 4 + [run(mission="time_trial")] * 9)["passed"] is False


def test_rule_count_by_fraction_scales_with_the_number_flown():
    rule = {"name": "8 of every 10", "metric": "base_points", "count": {"min": 0.5, "fraction": 0.8}}
    assert evaluate(rule, [run(base_points=15.0)] * 8 + [run(base_points=0.0)] * 2)["passed"] is True
    assert evaluate(rule, [run(base_points=15.0)] * 15 + [run(base_points=0.0)] * 5)["passed"] is False  # 15 of 20 is not 8 of 10
    assert evaluate(rule, [run(base_points=15.0)] * 16 + [run(base_points=0.0)] * 4)["passed"] is True


def test_rule_with_a_typo_is_an_error_not_a_pass():
    with pytest.raises(gate.GateError, match="eech"):
        evaluate({"name": "x", "metric": "details.distance_m", "eech": {"max": 0.3}}, deliveries([0.1]))
    with pytest.raises(gate.GateError, match="mx"):
        evaluate({"name": "x", "metric": "details.distance_m", "each": {"mx": 0.3}}, deliveries([0.1]))


def test_every_shipped_rule_is_well_formed():
    for name, definition in gate.load_gates().items():
        for rule in definition.get("rules", []):
            gate.evaluate_rule(rule, [])  # raises GateError on an unknown kind or limit


# --- reading a suite folder and writing the result file -------------------------------------


def suite_folder(tmp_path, distances, slow_seed=None):
    folder = tmp_path / "suite"
    for seed, distance in enumerate(distances, start=1):
        run_dir = folder / "runs" / f"delivery-baseline-s{seed}-off"
        run_dir.mkdir(parents=True)
        status = "INVALID" if seed == slow_seed else "OK"
        (run_dir / "result.json").write_text(json.dumps({
            "scenario": "delivery/baseline", "seed": seed, "variant": "off", "status": status, "reason": "real-time factor 0.61 is below 0.8" if status == "INVALID" else "",
            "points": 16.2, "real_time_factor": 0.61 if status == "INVALID" else 0.97, "flight_s": 150.0, "headline": f"{distance:.2f} m from the target", "run_dir": str(run_dir),
        }))  # fmt: skip
        (run_dir / "score.json").write_text(json.dumps({"mission": "delivery", "points": 16.2, "details": {"distance_m": distance}, "common": {}}))
        (run_dir / "scenario.json").write_text(json.dumps({"mission": "delivery", "name": "baseline", "seed": seed, "variant": "off", "wind": {"level": "calm"}}))
    return folder


def run_cli(*args):
    return subprocess.run([sys.executable, str(SIM / "gate.py"), *map(str, args)], capture_output=True, text=True)


def test_cli_checks_a_suite_folder_and_writes_the_result_file(tmp_path):
    folder = suite_folder(tmp_path, [0.02, 0.03, 0.02, 0.09, 0.12])
    out = tmp_path / "gate-G0.md"
    done = run_cli("check", "G0", folder, "--write", out, "--command", "sim/run_scenarios.sh G0")
    assert done.returncode == 0, done.stderr
    assert "G0: PASS" in done.stdout
    text = out.read_text()
    for expected in ("# Gate G0", "**PASS**", "Delivery touchdown ≤ 0.3 m in 5/5", "sim/run_scenarios.sh G0", "flags off", "0.12", "| delivery/baseline | 5 |", "Commit"):
        assert expected in text, expected
    assert text.count("| delivery/baseline |") == 5  # every run is in the table


def test_cli_exit_codes_tell_pass_fail_and_invalid_apart(tmp_path):
    assert run_cli("check", "G0", suite_folder(tmp_path / "a", [0.02, 0.03, 0.02, 0.09, 0.4])).returncode == 1
    invalid = run_cli("check", "G0", suite_folder(tmp_path / "b", [0.02, 0.03, 0.02, 0.09, 0.1], slow_seed=3))
    assert invalid.returncode == 3 and "G0: INVALID" in invalid.stdout
    assert run_cli("check", "G7", suite_folder(tmp_path / "c", [0.1])).returncode == 2  # not a sim gate
    assert run_cli("check", "G9", tmp_path).returncode == 2


def test_cli_checks_several_folders_together_so_a_reflown_run_counts(tmp_path):
    first = suite_folder(tmp_path / "first", [0.02, 0.03, 0.02, 0.09, 0.1], slow_seed=3)
    again = suite_folder(tmp_path / "again", [0.5, 0.5, 0.04])  # only seed 3 matters here; make the others absent
    for seed in (1, 2):
        for path in (again / "runs" / f"delivery-baseline-s{seed}-off").iterdir():
            path.unlink()
        (again / "runs" / f"delivery-baseline-s{seed}-off").rmdir()
    done = run_cli("check", "G0", first, again)
    assert done.returncode == 0, done.stdout + done.stderr


def test_cli_lists_the_gates_and_their_flights():
    listed = run_cli("list")
    assert listed.returncode == 0
    assert "G0" in listed.stdout and "5 flights" in listed.stdout and "G7" in listed.stdout and "field" in listed.stdout
    flights = run_cli("flights", "G0-W3")
    assert flights.stdout.strip().splitlines()[0].endswith("delivery/baseline.yaml 1 off")


# --- every shipped rule, on both sides of its limit ----------------------------------------------
#
# G1 to G6 cannot fly until their lanes exist, so this is the only proof that
# their numbers are the ones in the pass lines: (gate, rule name) -> runs that
# just meet the rule, and runs that just miss it.


def wp(mission, hit=7, of=7, tolerance=None, **kwargs):
    return run(mission=mission, hit=hit, of=of, settings={} if tolerance is None else {"tolerance_m": tolerance}, **kwargs)


SHIPPED_RULE_CASES = {
    ("G0", "every touchdown within 0.3 m of the target"): (deliveries([0.1, 0.30]), deliveries([0.1, 0.31])),
    ("G1", "p95 hold error within 0.20 m in moderate wind"): ([run(wind="moderate", hold_error_p95_m=0.20)], [run(wind="moderate", hold_error_p95_m=0.21)]),
    ("G1", "p95 hold error within 0.30 m in strong wind"): ([run(wind="strong", hold_error_p95_m=0.30)], [run(wind="strong", hold_error_p95_m=0.31)]),
    ("G1", "flown in moderate wind"): ([run(wind="moderate")], [run(wind="strong")]),
    ("G1", "flown in strong wind"): ([run(wind="strong")], [run(wind="moderate")]),
    ("G1", "held at 6 m twice (once in each wind)"): ([run(settings={"hold_height_m": 6.0})] * 2, [run(settings={"hold_height_m": 6.0}), run(settings={"hold_height_m": 2.0})]),
    ("G1", "held at 2 m twice (once in each wind)"): ([run(settings={"hold_height_m": 2.0})] * 2, [run(settings={"hold_height_m": 2.0}), run(settings={"hold_height_m": 6.0})]),
    ("G2", "median drop distance within 0.15 m in calm"): (deliveries([0.10, 0.15, 0.90]), deliveries([0.10, 0.16, 0.90])),
    ("G2", "median drop distance within 0.30 m in strong wind"): (deliveries([0.2, 0.30, 0.5], wind="strong"), deliveries([0.2, 0.31, 0.5], wind="strong")),
    ("G2", "no drop further than 1.0 m"): (deliveries([0.1, 1.0]), deliveries([0.1, 1.01])),
    ("G2", "no release below 1.83 m"): ([run(released_low=False)], [run(released_low=True)]),
    ("G2", "no descent below 1.83 m"): ([run(flew_low=False)], [run(flew_low=True)]),
    ("G2", "the judge's height limit is 1.83 m"): ([run(settings={"min_height_m": 1.83})], [run(settings={"min_height_m": 1.5})]),
    ("G2", "flown in calm"): ([run(wind="calm")], [run(wind="strong")]),
    ("G2", "flown in strong wind"): ([run(wind="strong")], [run(wind="calm")]),
    ("G3", "median delivery distance within 0.5 m"): (deliveries([0.2, 0.5, 0.9]), deliveries([0.2, 0.51, 0.9])),
    ("G3", "no release above 1.83 m"): ([run(released_high=False)], [run(released_high=True)]),
    ("G3", "contact impulse under the limit"): ([run(impulse_ok=True)], [run(impulse_ok=None)]),  # not measured is not a pass
    ("G3", "the judge's release limit is 1.83 m"): ([run(settings={"max_release_height_m": 1.83})], [run(settings={"max_release_height_m": 2.5})]),
    ("G4", "at least 95% of the markers listed with the right ID"): ([run(correct=10, markers=10), run(correct=9, markers=10)], [run(correct=10, markers=10), run(correct=8, markers=10)]),
    ("G4", "median position error within 1.0 m"): ([run(errors_m=[0.5, 1.0]), run(errors_m=[1.5])], [run(errors_m=[0.5, 1.1]), run(errors_m=[1.5])]),
    ("G4", "p95 position error within 2.0 m"): ([run(errors_m=[2.0, 2.0])], [run(errors_m=[2.0, 2.1])]),
    ("G4", "never below 7.62 m over the survey area"): ([run(min_height_m=7.62), run(min_height_m=10.7)], [run(min_height_m=7.61), run(min_height_m=10.7)]),
    ("G4", "the list is written before landing"): ([run(list_before_landing=True)], [run(list_before_landing=False)]),
    ("G4", "the judge's height floor is 7.62 m"): ([run(settings={"height_penalty_below_m": 7.62})], [run(settings={"height_penalty_below_m": 7.0})]),
    ("G5", "at least ten flights"): ([run()] * 10, [run()] * 9),
    ("G5", "the bucket is brought back in at least 8 of every 10"): ([run(base_points=15.2)] * 8 + [run(base_points=0.0)] * 2, [run(base_points=15.2)] * 7 + [run(base_points=0.0)] * 3),
    ("G5", "no release above 1.83 m"): ([run(released_high=False)], [run(released_high=True)]),
    ("G5", "the judge's release limit is 1.83 m"): ([run(settings={"max_release_height_m": 1.83})], [run(settings={"max_release_height_m": 3.0})]),
    ("G5", "every flight ends safely (a failure comes home by RTL before the gripper first closes, and lands otherwise)"): ([run(ended_safely=True)], [run(ended_safely=False)]),
    ("G6", "every waypoint of every flight passed within tolerance"): ([wp("waypoint_navigation"), wp("time_trial")], [wp("waypoint_navigation"), wp("time_trial", hit=6)]),
    ("G6", "five waypoint navigation flights"): ([wp("waypoint_navigation")] * 5, [wp("waypoint_navigation")] * 4 + [wp("time_trial")]),
    ("G6", "five time trial flights"): ([wp("time_trial")] * 5, [wp("time_trial")] * 4 + [wp("waypoint_navigation")]),
    ("G6", "flown in moderate wind"): ([wp("time_trial", wind="moderate")], [wp("time_trial", wind="calm")]),
    ("G6", "waypoint navigation judged at 1 m or tighter"): ([wp("waypoint_navigation", tolerance=1.0)], [wp("waypoint_navigation", tolerance=1.5)]),
    ("G6", "time trial judged at 4 m or tighter"): ([wp("time_trial", tolerance=4.0)], [wp("time_trial", tolerance=4.5)]),
}


def shipped_rules():
    return [(name, rule) for name, definition in gate.load_gates().items() for rule in definition.get("rules", [])]


def test_every_shipped_rule_has_a_case_on_both_sides_of_its_limit():
    assert {(name, rule["name"]) for name, rule in shipped_rules()} == set(SHIPPED_RULE_CASES)


@pytest.mark.parametrize("name, rule", shipped_rules(), ids=lambda value: value if isinstance(value, str) else value["name"])
def test_shipped_rule_holds_at_its_limit_and_fails_just_past_it(name, rule):
    meets, misses = SHIPPED_RULE_CASES[(name, rule["name"])]
    assert gate.evaluate_rule(rule, meets)["passed"] is True, gate.evaluate_rule(rule, meets)
    assert gate.evaluate_rule(rule, misses)["passed"] is False, gate.evaluate_rule(rule, misses)


def test_judge_reports_every_setting_and_metric_the_shipped_rules_read():
    """A rule that reads a name no judge writes must be one of the three known to be owed."""
    from judge import MISSIONS, default_settings

    owed = {"details.hold_error_p95_m", "details.ended_safely"}  # CEN and REC lanes (PROGRESS.md, Decisions)
    settings = {f"settings.{key}" for mission in MISSIONS for key in default_settings(mission)}
    source = (SIM / "judge" / "missions.py").read_text()
    for name, rule in shipped_rules():
        paths = [rule[kind][key] for kind in ("ratio",) if kind in rule for key in ("of", "over")] + ([rule["metric"]] if "metric" in rule else [])
        for path in paths:
            if path in owed or path == "base_points":
                continue
            if path.startswith("settings."):
                assert path in settings, (name, path)
            else:
                assert f'"{path.split(".", 1)[1]}"' in source, (name, path)


def test_result_file_names_the_flags_that_were_on_including_node_flags(tmp_path):
    runs = deliveries([0.02, 0.03, 0.02, 0.09, 0.12])
    for item in runs:
        item["manifest"]["flags"] = {"use_estimator": True}
        item["manifest"]["node_flags"] = {"marker_detector": {"sim_oracle_detector": True}}
    text = gate.render_result(gate.check("G0", runs), runs, "sim/run_scenarios.sh G0")
    assert "use_estimator=True" in text
    assert "marker_detector.features.sim_oracle_detector=True" in text
    assert "none on" not in text


def test_gate_g4_flies_the_survey_over_ten_markers_with_the_whole_localisation_chain_on(tmp_path):
    """G4's flights: the spec's own field (digits 0 to 9), placed by the seed, scored by the localisation judge."""
    import json

    sys.path.insert(0, str(SIM))
    from scenario.build import build_run
    from scenario.spec import load_scenario

    flights = gate.flights("G4")
    assert [(flight["scenario"].name, flight["seed"], flight["variant"]) for flight in flights] == [("gate_field.yaml", seed, "on") for seed in (1, 2, 3)]
    scenario = load_scenario(flights[0]["scenario"])
    assert scenario.mission == "localisation" and {name for name, on in scenario.node_flags["loc_node"].items() if on} >= {"sim_oracle_detector", "digit_reader", "template_digit_classifier", "digit_vote", "judges_list"}
    places = {}
    for seed in (1, 2):
        run = build_run(flights[0]["scenario"], tmp_path / f"s{seed}", variant="on", seed=seed)
        truth = json.loads((run / "truth.json").read_text())
        markers = sorted(truth["markers"], key=lambda item: item["id"])
        assert [marker["id"] for marker in markers] == list(range(10))  # every digit, the 6 and the 9 among them
        assert all(23.0 <= marker["e"] <= 77.0 and 13.0 <= marker["n"] <= 47.0 for marker in markers)
        assert min(((a["e"] - b["e"]) ** 2 + (a["n"] - b["n"]) ** 2) ** 0.5 for a in markers for b in markers if a is not b) >= 6.0
        places[seed] = [(marker["e"], marker["n"]) for marker in markers]
        assert json.loads((run / "scenario.json").read_text())["list_file"] == str(run / "list" / "judges_list.csv")
    assert places[1] != places[2]  # a different field for each seed
