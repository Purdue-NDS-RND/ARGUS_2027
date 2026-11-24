"""PLT-12: the sim's check of an attempt's bundle (sim/attempt_check.py) and its wiring. No ROS."""

import json
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
REPO = SIM.parent
sys.path.insert(0, str(SIM))

import attempt_check  # noqa: E402
from runner import events, result  # noqa: E402
from scenario import BuildError, build_run  # noqa: E402
from scenario import spec as scenario_spec  # noqa: E402

ATTEMPT = SIM / "scenarios" / "delivery" / "attempt.yaml"
COUNTS = {"/mavros/local_position/pose": 4000, "/mavros/state": 200}


def manifest(**changes):
    whole = {
        "complete": True, "owed": [], "missing": {}, "git": {"hash": "f6ed068"}, "profile": {"name": "comp_2026", "flags": []},
        "parts": {"bag": {"topics": {"/mavros/local_position/pose": "geometry_msgs/msg/PoseStamped", "/mavros/state": "mavros_msgs/msg/State"}}},
    }  # fmt: skip
    return {**whole, **changes}


@pytest.fixture()
def logs(tmp_path):
    wrote = bytes(range(256)) * 800  # 204800 bytes
    (tmp_path / "pulled.BIN").write_bytes(wrote)
    (tmp_path / "disk.BIN").write_bytes(wrote + b"a little more, after the pull")
    return tmp_path / "pulled.BIN", tmp_path / "disk.BIN"


def test_a_whole_bundle_passes(logs):
    assert attempt_check.judge(manifest(), [], COUNTS, *logs) == {"passed": True, "problems": []}


def test_each_way_a_bundle_is_not_whole_is_said(logs, tmp_path):
    pulled, disk = logs

    def problems(**kwargs):
        given = {"manifest": manifest(), "verify_problems": [], "counts": COUNTS, "pulled": pulled, "on_disk": disk, **kwargs}
        verdict = attempt_check.judge(given["manifest"], given["verify_problems"], given["counts"], given["pulled"], given["on_disk"])
        assert verdict["passed"] is False
        return " | ".join(verdict["problems"])

    assert "no bundle was made" in problems(manifest=None)
    assert "incomplete: dataflash (not given)" in problems(manifest=manifest(complete=False, owed=["dataflash"], missing={"dataflash": "not given"}))
    assert "bag/bag_0.mcap is gone" in problems(verify_problems=["bag: bag/bag_0.mcap is gone"])
    assert "no messages on /mavros/local_position/pose" in problems(counts={"/mavros/state": 200})
    with_images = manifest()
    with_images["parts"]["bag"]["topics"]["/camera/image"] = "sensor_msgs/msg/Image"
    assert "holds images: /camera/image" in problems(manifest=with_images)
    assert "no dataflash log was pulled" in problems(pulled=None)
    assert "was not found to compare" in problems(on_disk=tmp_path / "nowhere.BIN")
    assert "no git hash" in problems(manifest=manifest(git={"hash": None}))
    assert "no flag profile" in problems(manifest=manifest(profile=None))
    # The profile in the manifest must be the one the GO command flew, and the bundler's warnings about the config count.
    wrong = attempt_check.judge(manifest(), [], COUNTS, pulled, disk, flown_profile="delivery_estimator")
    assert wrong["passed"] is False and "is comp_2026, and the flight flew delivery_estimator" in wrong["problems"][0]
    assert attempt_check.judge(manifest(), [], COUNTS, pulled, disk, flown_profile="comp_2026")["passed"] is True
    assert "names other.yaml" in problems(manifest=manifest(warnings=["the profile bundled is a.yaml; the flight's own record names other.yaml"]))
    assert attempt_check.judge(manifest(warnings=["the bag is sqlite3, not MCAP: the recorder found no MCAP plugin where it ran"]), [], COUNTS, pulled, disk)["passed"] is True
    # The pulled log must be SITL's own bytes, and nearly all of them.
    other = tmp_path / "other.BIN"
    other.write_bytes(b"\xff" + pulled.read_bytes()[1:])  # one byte wrong
    assert "is not what SITL wrote" in problems(pulled=other)
    short = tmp_path / "short.BIN"
    short.write_bytes(pulled.read_bytes()[:150_000])
    assert "was not pulled whole" in problems(pulled=short)
    tiny = tmp_path / "tiny.BIN"
    tiny.write_bytes(pulled.read_bytes()[:5000])
    assert "not a flight's log" in problems(pulled=tiny)


def test_the_attempt_scenario_records_through_the_go_command_and_gets_a_port_for_the_pull(tmp_path):
    run = build_run(ATTEMPT, tmp_path / "run", variant="on")
    scenario = json.loads((run / "scenario.json").read_text())
    assert scenario["attempt"] == {"slot": "sim", "mission": "delivery", "port": scenario_spec.ATTEMPT_PORT} and scenario_spec.ATTEMPT_PORT != scenario_spec.PREFLIGHT_PORT
    command = scenario["go"]["command"]
    assert command[command.index("--record") + 1] == str(run) == command[command.index("--log-dir") + 1]  # one folder for the flight
    assert scenario["record"] is False and "record" not in events.tools_needed(scenario)  # the GO command records; the runner's recorder is not started beside it
    # The start script gives SITL a MAVLink output on that port.
    start = (SIM / "container" / "start.sh").read_text()
    assert '(json.load(open(sys.argv[1])).get("attempt") or {}).get("port")' in start and 'sitl_args+=(--out "udp:127.0.0.1:$attempt_port")' in start
    plain = json.loads((build_run(SIM / "scenarios" / "delivery" / "go.yaml", tmp_path / "plain", variant="on") / "scenario.json").read_text())
    assert plain["attempt"] is None and "--record" not in plain["go"]["command"]


def test_the_missions_an_attempt_can_be_of_are_the_bundlers():
    sys.path.insert(0, str(REPO / "tools"))
    import bundle_attempt

    assert set(scenario_spec.ATTEMPT_MISSIONS) == set(bundle_attempt.EXPECTED)


@pytest.mark.parametrize("slot, word", [("Slot 2", "preflight.go.attempt is the attempt's slot"), ("true", "preflight.go.attempt is the attempt's slot")])
def test_a_slot_that_is_not_a_short_name_is_refused(slot, word, tmp_path):
    other = tmp_path / "attempt.yaml"
    other.write_text(ATTEMPT.read_text().replace("attempt: sim ", f"attempt: {slot} "))
    with pytest.raises(BuildError, match=word):
        build_run(other, tmp_path / "run", variant="on")


def test_a_run_with_an_attempt_counts_only_if_the_bundle_is_whole(tmp_path):
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import test_runner

    scenario = {"attempt": {"slot": "sim", "mission": "delivery", "port": 14556}}
    run = test_runner.make_run(tmp_path, score=test_runner.SCORE, manifest=scenario)
    assert result.run_result(run)["status"] == "INVALID"
    (run / "attempt_check.json").write_text(json.dumps({"passed": False, "problems": ["the bundle is incomplete: dataflash (not given)"]}))
    summary = result.run_result(run)
    assert summary["status"] == "FAIL" and "not whole: the bundle is incomplete: dataflash" in summary["reason"]
    (run / "attempt_check.json").write_text(json.dumps({"passed": True, "problems": []}))
    assert result.run_result(run)["status"] == "OK"
    assert result.run_result(test_runner.make_run(tmp_path / "plain", score=test_runner.SCORE))["attempt_check"] is None


def test_the_readiness_demonstration_is_flown_by_the_mission_stack_judged_as_a_fence_flight_and_bundled(tmp_path):
    """OPS-02: the fence_demo mission through the GO command, the geofence judge on the true position, an attempt's bundle after."""
    import yaml

    run = build_run(SIM / "scenarios" / "geofence" / "readiness_demo.yaml", tmp_path / "run", variant="on")
    scenario = json.loads((run / "scenario.json").read_text())
    assert scenario["mission"] == "geofence" and scenario["flags"] == {"fence_demo": True} and scenario["driver"] is None  # no fence_check tool: the mission flies it
    command = scenario["go"]["command"]
    assert command[command.index("--profile") + 1] == str(REPO / "profiles" / "fence_demo.yaml") and command[command.index("--detector") + 1] == "none" and "--no-camera" in command
    assert scenario["attempt"] == {"slot": "readiness", "mission": "geofence", "port": scenario_spec.ATTEMPT_PORT}
    assert scenario["preflight"]["expect"] == "GO"
    sequence = yaml.safe_load((run / "sequence.yaml").read_text())["mission_sequence"]
    assert sequence["features"]["fence_demo"] is False  # the run's file keeps the flag false: the profile turns it on
    assert [item["type"] for item in sequence["missions"]] == ["takeoff", "fence_demo", "rtl"]
    # Variant off flies the 2026 profile: the mission is skipped, and nothing flies at the fence.
    off = json.loads((build_run(SIM / "scenarios" / "geofence" / "readiness_demo.yaml", tmp_path / "off", variant="off") / "scenario.json").read_text())
    assert off["flags"] == {} and off["go"]["command"][off["go"]["command"].index("--profile") + 1] == str(REPO / "profiles" / "comp_2026.yaml")


def test_a_fence_demonstration_the_judge_did_not_pass_fails_the_run(tmp_path):
    """Found flying it: with no tool's exit code behind it, a run whose fence judge said "never reached the fence" counted as OK."""
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import test_runner

    def demonstration(folder, passed, notes=()):
        scenario = {"mission": "geofence", "go": {"command": ["python3", "tools/go.py"]}}
        score = {**test_runner.SCORE, "mission": "geofence", "points": 1.0 if passed else 0.0, "details": {"passed": passed, "held": None}, "notes": list(notes)}
        return result.run_result(test_runner.make_run(tmp_path / folder, score=score, manifest=scenario))

    assert demonstration("a", True)["status"] == "OK"
    failed = demonstration("b", False, ["geofence: the vehicle never reached the fence"])
    assert failed["status"] == "FAIL" and "the fence did not answer the demonstration: geofence: the vehicle never reached the fence" in failed["reason"]


def test_the_readiness_kits_verdict_is_part_of_a_fence_demonstrations_pass(tmp_path):
    """The spec audit: the scenario passed whether or not the kit said READY."""
    sys.path.insert(0, str(REPO / "tools" / "test"))
    import test_readiness_kit as kit_test

    ready = kit_test.make_bundle(tmp_path / "ready", events=kit_test.held_events())
    verdict = attempt_check.readiness_verdict(ready)
    assert verdict == {"ready": True, "problems": []} and "Verdict: **READY**" in (ready / "readiness.md").read_text()
    no_fence = kit_test.make_bundle(tmp_path / "not", events=[kit_test.mode(60.0, 4), kit_test.mode(200.0, 6, 2)])
    verdict = attempt_check.readiness_verdict(no_fence)
    assert verdict["ready"] is False and len(verdict["problems"]) == 1 and verdict["problems"][0].startswith("the readiness kit says NOT READY: the log has no fence breach")
    assert attempt_check.readiness_verdict(tmp_path / "nowhere")["problems"][0].startswith("the readiness kit could not read the bundle")
    assert 'attempt["mission"] == "geofence"' in (SIM / "attempt_check.py").read_text()


# --- CEN-14: the report's release confirmation, held against what the sim's payload bridge did


def _report(verdict, **more):
    found = {"verdict": verdict, "left_by": [], "missing": [], "lines": ["command: ..."], **more}
    return "# Post-flight report\n", {"audit": {"clean": True, "release_confirmation": found}}


def test_the_sims_truth_about_a_release_is_the_payload_bridges_own_event(tmp_path):
    assert attempt_check.release_truth(tmp_path) is False  # no bridge ran: nothing was carried
    (tmp_path / "payload_bridge.jsonl").write_text(json.dumps({"t": 96.5, "event": "servo", "pwm": 1900}) + "\n")
    assert attempt_check.release_truth(tmp_path) is False  # carried, and never let go
    (tmp_path / "payload_bridge.jsonl").write_text(json.dumps({"t": 96.5, "event": "servo"}) + "\n" + json.dumps({"t": 96.9, "event": "payload_released", "pwm": 1900}) + "\n")
    assert attempt_check.release_truth(tmp_path) is True
    import payload_bridge

    assert '"payload_released"' in Path(payload_bridge.__file__).read_text()  # the event's name is the bridge's


@pytest.mark.parametrize(
    "left, verdict, more, problem",
    [
        (True, "CONFIRMED", {"left_by": ["step"]}, None),
        (False, "NOT CONFIRMED", {"missing": ["nothing shows the payload leaving"]}, None),
        (False, "NO RELEASE", {}, None),  # a fake drop: nothing commanded, nothing left
        (True, "NOT CONFIRMED", {"missing": ["nothing shows the payload leaving"]}, "the sim let the payload go, and the report says RELEASE NOT CONFIRMED: nothing shows the payload leaving"),
        (True, "NO RELEASE", {}, "the sim let the payload go, and the report says RELEASE NO RELEASE"),
        (False, "CONFIRMED", {"left_by": ["flight"]}, "nothing left the vehicle in the sim, and the report says RELEASE CONFIRMED (by flight)"),
    ],
)
def test_the_reports_release_confirmation_must_be_what_the_sim_did(tmp_path, monkeypatch, left, verdict, more, problem):
    import post_flight

    monkeypatch.setattr(post_flight, "make_report", lambda folder, companion: _report(verdict, **more))
    found = attempt_check.release_verdict(tmp_path, left)
    assert found["verdict"] == verdict and found["left"] is left and found["problems"] == ([problem] if problem else [])
    # The report is written into the bundle, as a person running the tool would leave it.
    assert (tmp_path / "report.md").is_file() and json.loads((tmp_path / "release_confirmation.json").read_text())["verdict"] == verdict
    assert json.loads((tmp_path / "report.json").read_text())["audit"]["release_confirmation"]["verdict"] == verdict


def test_a_bundle_the_report_cannot_read_fails_the_release_check(tmp_path):
    found = attempt_check.release_verdict(tmp_path, True)  # no manifest: not a bundle
    assert found["verdict"] is None and "the post-flight report could not be written" in found["problems"][0]


@pytest.mark.parametrize("name, carried", [("delivery/attempt_release", True), ("delivery/attempt_release_empty", False), ("drop/attempt_release", True), ("drop/attempt_release_empty", False)])
def test_the_release_scenarios_fly_a_real_release_as_an_attempt_with_no_flag_on(tmp_path, name, carried):
    run = build_run(SIM / "scenarios" / f"{name}.yaml", tmp_path / "run", variant="on")
    scenario = json.loads((tmp_path / "run" / "scenario.json").read_text())
    assert scenario["attempt"]["mission"] == name.split("/")[0] and scenario["attempt"]["slot"] == ("release" if carried else "empty")
    assert (scenario.get("payload_bridge") is not None) is carried and Path(scenario["go"]["profile"]).stem == "comp_2026"
    sequence = (tmp_path / "run" / "sequence.yaml").read_text()
    assert "fake_drop: false" in sequence and ": true" not in sequence.split("missions:")[0]  # the release is real, and no flag is on
    assert run is not None
