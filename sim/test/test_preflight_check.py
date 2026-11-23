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
    assert sim_baseline == {name: value for name, value in committed.items() if not name.startswith("RNGFND1_")} and len(sim_baseline) == len(committed) - 4
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
