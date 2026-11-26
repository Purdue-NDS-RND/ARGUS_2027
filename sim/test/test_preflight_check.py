"""PLT-10: the sim's check of the preflight (sim/preflight_check.py) and its wiring. No ROS."""

import json
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
REPO = SIM.parent
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(REPO / "src" / "drone_utils"))

import preflight_check  # noqa: E402
from drone_utils import preflight as pf  # noqa: E402
from runner import events, result  # noqa: E402
from scenario import BuildError, build_run  # noqa: E402
from scenario import spec as scenario_spec  # noqa: E402

SCENARIOS = SIM / "scenarios"
GO, NO_GO = SCENARIOS / "delivery" / "preflight.yaml", SCENARIOS / "hover" / "preflight_nogo.yaml"


def record(verdict, failed=()):
    return {"verdict": verdict, "failed": list(failed), "checks": [{"name": name, "go": name not in failed, "detail": f"about {name}"} for name in pf.CHECKS]}


def test_the_verdict_and_the_failed_checks_must_be_the_scenarios():
    assert preflight_check.compare({"expect": "GO", "failed": []}, record("GO"))["passed"]
    assert preflight_check.compare({"expect": "NO-GO", "failed": ["params", "battery"]}, record("NO-GO", ["battery", "params"]))["passed"]
    wrong = preflight_check.compare({"expect": "GO", "failed": []}, record("NO-GO", ["gps"]))
    assert not wrong["passed"] and "said NO-GO, the scenario expects GO (gps: about gps)" in wrong["problem"]
    other = preflight_check.compare({"expect": "NO-GO", "failed": ["params", "battery"]}, record("NO-GO", ["params", "battery", "fence"]))
    assert not other["passed"] and "failed battery, fence, params, the scenario expects battery, params" in other["problem"]
    assert "no record" in preflight_check.compare({"expect": "GO"}, None)["problem"]
    # An overridden NO-GO is not what a scenario that expects NO-GO expects.
    assert not preflight_check.compare({"expect": "NO-GO", "failed": ["gps"]}, record("NO-GO OVERRIDDEN", ["gps"]))["passed"]


def test_the_scenarios_checks_are_the_preflights_own_names():
    assert scenario_spec.PREFLIGHT_CHECKS == pf.CHECKS


def test_a_scenario_with_a_preflight_gets_the_baseline_sitl_can_hold_the_profile_and_a_port(tmp_path):
    run = build_run(GO, tmp_path / "go", variant="on")
    manifest = json.loads((run / "scenario.json").read_text())
    wanted = manifest["preflight"]
    assert wanted["expect"] == "GO" and wanted["failed"] == [] and wanted["cameras"] == {"/camera/image": 15.0} and wanted["port"] == 14555
    assert events.tools_needed(manifest) >= {"preflight"} and preflight_check.wanted(manifest)
    assert manifest["sim_time"] is True and manifest["flags"] == {}  # one clock, and the 2026 mission code
    # The baseline it is held to is the committed one, less what SITL cannot take; SITL is started on it.
    sim_baseline = pf.read_param_file(wanted["baseline"])
    committed = pf.read_param_file(REPO / "platform" / "fcu" / "argus_2027.param")
    assert sim_baseline == {name: value for name, value in committed.items() if not name.startswith(("RNGFND1_", "SERIAL"))} and len(sim_baseline) == len(committed) - 6  # the rangefinder and its port
    sitl = {line.split()[0]: float(line.split()[1]) for line in (run / "sitl.parm").read_text().splitlines()}
    for name, value in sim_baseline.items():
        if not name.startswith("FENCE_"):  # the site's upload sets the fence's own, after SITL starts
            assert sitl.get(name) == value, name
    assert sitl["SIM_BATT_VOLTAGE"] / 6.0 >= pf.Limits().min_cell_v
    assert pf.check_profile(wanted["profile"], run / "sequence.yaml") == (True, "")
    args = preflight_check.arguments(manifest, run)
    assert args[args.index("--fc-url") + 1] == "udpin:127.0.0.1:14555" and args[args.index("--sequence") + 1] == str(run / "sequence.yaml")
    assert args[args.index("--camera") + 1] == "/camera/image=15" and args[args.index("--log-dir") + 1] == str(run)
    assert "--sim-time" in args and "--no-camera" not in args  # the run is on sim time, so the preflight's node is too
    off = build_run(GO, tmp_path / "off", variant="off")
    assert "--sim-time" not in preflight_check.arguments(json.loads((off / "scenario.json").read_text()), off)
    # The start script gives SITL that port only when the scenario has a preflight.
    start = (SIM / "container" / "start.sh").read_text()
    assert '(json.load(open(sys.argv[1])).get("preflight") or {}).get("port")' in start
    plain = build_run(SCENARIOS / "delivery" / "baseline.yaml", tmp_path / "plain")
    assert json.loads((plain / "scenario.json").read_text())["preflight"] is None and not (plain / "fc_baseline.param").exists()


def test_a_flight_flown_by_a_tool_is_checked_against_a_sequence_with_no_flags(tmp_path):
    run = build_run(NO_GO, tmp_path / "nogo", variant="on")
    manifest = json.loads((run / "scenario.json").read_text())
    assert manifest["preflight"]["expect"] == "NO-GO" and manifest["preflight"]["failed"] == ["params", "battery"]
    args = preflight_check.arguments(manifest, run)
    sequence = Path(args[args.index("--sequence") + 1])
    assert sequence.name == "no_sequence.yaml" and pf.check_profile(manifest["preflight"]["profile"], sequence) == (True, "")
    # SITL is left as it always was: the battery it reports is under the preflight's line, and no failsafe is set.
    assert "SIM_BATT_VOLTAGE" not in (run / "sitl.parm").read_text() and "FS_GCS_ENABLE" not in (run / "sitl.parm").read_text()


@pytest.mark.parametrize(
    "preflight, word",
    [
        ({"expect": "MAYBE"}, "GO or NO-GO"),
        ({"expect": "NO-GO"}, "at least one with NO-GO"),
        ({"expect": "GO", "failed": ["gps"]}, "none with GO"),
        ({"expect": "NO-GO", "failed": ["weather"]}, "preflight.failed"),
        ({"expect": "GO", "cameras": {"camera": 30}}, "preflight.cameras"),
        ({"expect": "GO", "strict": True}, "strict"),
    ],
)
def test_a_preflight_block_that_does_not_say_what_must_be_said_is_refused(preflight, word, tmp_path):
    import yaml

    data = yaml.safe_load(GO.read_text())
    data["preflight"] = preflight
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump(data))
    with pytest.raises(BuildError, match=word):
        build_run(bad, tmp_path / "run")


def test_a_run_with_a_preflight_counts_only_if_the_check_passed(tmp_path):
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import test_runner

    manifest = {"preflight": {"expect": "GO", "failed": [], "cameras": {}}}
    run = test_runner.make_run(tmp_path, score=test_runner.SCORE, manifest=manifest)
    assert result.run_result(run)["status"] == "INVALID"
    (run / "preflight_check.json").write_text(json.dumps({"passed": False, "problem": "the preflight said NO-GO, the scenario expects GO (battery: 12.6 V)"}))
    summary = result.run_result(run)
    assert summary["status"] == "FAIL" and "preflight check failed: the preflight said NO-GO" in summary["reason"]
    (run / "preflight_check.json").write_text(json.dumps({"passed": True, "problem": "", "verdict": "GO", "failed": []}))
    summary = result.run_result(run)
    assert summary["status"] == "OK" and summary["preflight_check"]["verdict"] == "GO"
    assert result.run_result(test_runner.make_run(tmp_path / "plain", score=test_runner.SCORE))["preflight_check"] is None


# --- PLT-07: a flight started by the GO command -------------------------------------------

GO_FLIGHT, GO_ESTIMATOR, GO_NOGO = (SCENARIOS / "delivery" / f"{name}.yaml" for name in ("go", "go_estimator", "go_nogo"))


def option(command, name):
    return command[command.index(name) + 1]


def test_a_go_scenario_is_started_by_the_go_command_with_a_committed_profile(tmp_path):
    import yaml

    run = build_run(GO_ESTIMATOR, tmp_path / "run", variant="on")
    manifest = json.loads((run / "scenario.json").read_text())
    command = manifest["go"]["command"]
    assert command[:2] == ["python3", str(REPO / "tools" / "go.py")] and command[-1] == "--sim"
    assert option(command, "--profile") == str(REPO / "profiles" / "delivery_estimator.yaml") == manifest["preflight"]["profile"]
    assert option(command, "--sequence") == str(run / "sequence.yaml") and option(command, "--params") == str(run / "params.yaml")
    assert option(command, "--detector") == "target_cv" and option(command, "--camera") == "/camera/image=15"
    assert option(command, "--baseline") == str(run / "fc_baseline.param") and option(command, "--fc-url") == "udpin:127.0.0.1:14555" and option(command, "--log-dir") == str(run)
    # The judge and the side tools go by the scenario's flags; the run's own files keep every flag false, as shipped files do.
    assert manifest["flags"] == {"use_frame_time_pose": True, "use_target_estimator": True} and manifest["node_flags"]["target_cv"]["stamp_from_image"] is True
    features = yaml.safe_load((run / "sequence.yaml").read_text())["mission_sequence"]["features"]
    assert not any(value is True for value in features.values())
    node_features = yaml.safe_load((run / "params.yaml").read_text())["target_cv"]["ros__parameters"]["features"]
    assert node_features["stamp_from_image"] is False and node_features["detection_contract"] is False
    # The GO command runs the preflight itself: the side tool is not started beside it.
    assert "preflight" not in events.tools_needed(manifest) and "estimator_check" in events.tools_needed(manifest)


def test_the_go_command_plans_the_run_the_builder_made(tmp_path):
    """The builder's command is one the real GO command accepts, and it turns on what the scenario says is on."""
    sys.path.insert(0, str(REPO / "tools"))
    import go

    run = build_run(GO_ESTIMATOR, tmp_path / "run", variant="on")
    manifest = json.loads((run / "scenario.json").read_text())
    plan = go.plan(go.build_parser().parse_args(manifest["go"]["command"][2:]))
    assert "feature_overrides:=use_frame_time_pose,use_target_estimator" in plan.launch
    assert "node_flags:=target_cv.stamp_from_image,target_cv.detection_contract" in plan.launch and "use_sim_time:=true" in plan.launch
    plain = build_run(GO_FLIGHT, tmp_path / "plain", variant="on")
    plan = go.plan(go.build_parser().parse_args(json.loads((plain / "scenario.json").read_text())["go"]["command"][2:]))
    assert not any(item.startswith(("feature_overrides:=", "node_flags:=")) for item in plan.launch) and "detector:=target_cv" in plan.launch


def test_variant_off_of_a_go_scenario_flies_the_2026_profile(tmp_path):
    run = build_run(GO_ESTIMATOR, tmp_path / "run", variant="off")
    manifest = json.loads((run / "scenario.json").read_text())
    assert option(manifest["go"]["command"], "--profile") == str(REPO / "profiles" / "comp_2026.yaml") and manifest["flags"] == {} and "--sim" not in manifest["go"]["command"]


def test_a_profile_that_is_not_the_scenarios_flags_is_refused(tmp_path):
    other = tmp_path / "go_estimator.yaml"
    other.write_text(GO_ESTIMATOR.read_text().replace("profile: delivery_estimator", "profile: comp_2026"))
    with pytest.raises(BuildError, match="does not turn on exactly"):
        build_run(other, tmp_path / "run", variant="on")


@pytest.mark.parametrize(
    "go, word",
    [("{profile: no_such_profile}", "no profiles/no_such_profile.yaml"), ("{detector: target_cv}", "preflight.go has profile"), ("{profile: comp_2026, detector: yolo}", "preflight.go has profile"), ("true", "preflight.go has profile")],
)
def test_a_go_block_that_names_no_profile_or_an_unknown_detector_is_refused(go, word, tmp_path):
    other = tmp_path / "go.yaml"
    text = GO_FLIGHT.read_text()
    start = text.index("  go:\n")
    other.write_text(text[:start] + f"  go: {go}\n")
    with pytest.raises(BuildError, match=word):
        build_run(other, tmp_path / "run", variant="on")


def test_the_go_commands_own_preflight_is_read_from_its_log(tmp_path):
    manifest = {"preflight": {"expect": "NO-GO", "failed": ["params", "battery"]}}
    report = preflight_check.check_log(manifest, tmp_path)
    assert not report["passed"] and "preflight.jsonl" in report["problem"]
    (tmp_path / "preflight.jsonl").write_text(json.dumps(record("GO")) + "\n" + json.dumps(record("NO-GO", ["battery", "params"])) + "\n")
    report = preflight_check.check_log(manifest, tmp_path)
    assert report["passed"] and report["verdict"] == "NO-GO"  # the last record is this flight's
    assert preflight_check.read_preflight_check(tmp_path)["passed"] is True


def test_a_flight_the_go_command_did_not_start_is_right_only_where_no_go_is_expected(tmp_path):
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import test_runner

    def not_started(folder, expect, exit_code, passed=True):
        manifest = {"preflight": {"expect": expect, "failed": ["params"] if expect == "NO-GO" else [], "cameras": {}}, "go": {"command": ["python3", "tools/go.py"]}}
        run = test_runner.make_run(tmp_path / folder, outcome="NOT_STARTED", manifest=manifest)
        (run / "flight.json").write_text(json.dumps({"outcome": "NOT_STARTED", "t_launch": 5.0, "t_end": 40.0, "launch_exit_code": exit_code}))
        (run / "preflight_check.json").write_text(json.dumps({"passed": passed, "problem": "" if passed else "the preflight failed gps, the scenario expects params"}))
        return result.run_result(run)

    assert not_started("a", "NO-GO", 1)["status"] == "OK"
    wrong = not_started("b", "GO", 1)
    assert wrong["status"] == "FAIL" and "started no flight" in wrong["reason"]
    assert not_started("c", "NO-GO", 2)["status"] == "FAIL"  # the command could not plan the flight: not a NO-GO of the preflight
    assert not_started("d", "NO-GO", 1, passed=False)["status"] == "FAIL"  # NO-GO for the wrong checks


def test_the_three_go_scenarios_build_and_say_what_they_expect(tmp_path):
    expects = {}
    for path in (GO_FLIGHT, GO_ESTIMATOR, GO_NOGO):
        manifest = json.loads((build_run(path, tmp_path / path.stem, variant="on") / "scenario.json").read_text())
        expects[path.stem] = (manifest["preflight"]["expect"], manifest["preflight"]["failed"], manifest["sim_time"])
    assert expects == {"go": ("GO", [], True), "go_estimator": ("GO", [], True), "go_nogo": ("NO-GO", ["params", "battery"], True)}


def test_a_go_command_that_ended_without_arming_is_not_started_after_the_vehicle_was_watched():
    from runner.watch import NOT_STARTED, NotStartedWatch

    now = [100.0]
    watch = NotStartedWatch(lambda: now[0], watch_s=10.0)
    assert watch.check(command_ended=False, ever_armed=False) is None  # the preflight is still listening
    now[0] = 108.0
    assert watch.check(command_ended=True, ever_armed=False) is None  # ended: watched from here
    now[0] = 117.9
    assert watch.check(command_ended=True, ever_armed=False) is None
    now[0] = 118.0
    assert watch.check(command_ended=True, ever_armed=False) == NOT_STARTED
    # A vehicle that armed is a flight, whatever became of the command: the end watch judges it.
    assert watch.check(command_ended=True, ever_armed=True) is None
