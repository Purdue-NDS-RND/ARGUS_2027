"""The sim's payload bridge (SIM-06): its rules, the carried payload's joint in the world, and its place in the runner. No MAVLink, no Gazebo."""

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import payload_bridge as pb  # noqa: E402
from payload_bridge import Bridge  # noqa: E402
from runner import events, result  # noqa: E402
from scenario.build import BuildError, build_run, carry, vehicle_pose  # noqa: E402

from test_runner import SCORE, make_run  # noqa: E402  (the run-folder fixture)

RELEASE = SIM / "scenarios" / "drop" / "release.yaml"
BASELINE = SIM / "scenarios" / "drop" / "baseline.yaml"
NEUTRAL, GRAB, LET_GO = pb.GRIPPER_NEUTRAL_PWM, pb.GRIPPER_GRAB_PWM, pb.GRIPPER_RELEASE_PWM


def gripper(pwm):
    return {pb.GRIPPER_SERVO: pwm}


def sprayer(pwm):
    return {pb.SPRAYER_SERVO: pwm}


# --- letting a carried payload go ---


def test_the_gripper_output_arriving_at_its_release_value_lets_the_payload_go_once():
    bridge = Bridge(carried="parcel")
    bridge.on_position("parcel", (1.0, 2.0, 4.0))
    assert bridge.on_servos(10.0, gripper(NEUTRAL)) == []  # where the servo already was: nothing commanded yet
    assert bridge.on_servos(10.1, gripper(NEUTRAL)) == []
    assert bridge.on_servos(10.2, gripper(LET_GO)) == [("detach", "parcel")]
    assert bridge.released and bridge.events[-1] == {"t": 10.2, "event": "payload_released", "name": "parcel", "output": "gripper", "pwm": LET_GO, "at": (1.0, 2.0, 4.0)}
    assert bridge.on_servos(10.3, gripper(LET_GO)) == []  # still at the release value: not a second release
    assert bridge.on_servos(10.4, gripper(NEUTRAL)) == [] and bridge.on_servos(10.5, gripper(LET_GO)) == []  # nothing left to let go
    assert [item["event"] for item in bridge.events] == ["payload_released", "release_with_nothing_held"]


def test_the_sprayer_coming_on_lets_the_payload_go_as_the_2026_drop_commands_it():
    bridge = Bridge(carried="parcel")
    assert bridge.on_servos(5.0, sprayer(1100)) == []
    assert bridge.on_servos(5.1, sprayer(1900)) == [("detach", "parcel")]
    assert bridge.events[-1]["output"] == "sprayer" and bridge.on_servos(5.2, sprayer(1900)) == []


def test_an_output_that_was_already_at_its_release_value_when_the_bridge_started_releases_nothing():
    bridge = Bridge(carried="parcel")
    assert bridge.on_servos(1.0, gripper(LET_GO)) == [] and bridge.on_servos(1.1, gripper(LET_GO)) == []
    assert bridge.on_servos(1.0, sprayer(1900)) == [] and not bridge.released
    # No output at all (PWM 0: the servo has no function yet) is not an output.
    assert bridge.on_servos(1.2, {pb.GRIPPER_SERVO: 0, 3: 1500}) == [] and bridge.events == []


def test_values_near_the_set_ones_count_and_values_between_do_not():
    for pwm, released in ((1100, True), (1249, True), (1251, False), (1500, False)):
        bridge = Bridge(carried="parcel")
        bridge.on_servos(0.0, gripper(NEUTRAL))
        assert bool(bridge.on_servos(0.1, gripper(pwm))) is released, pwm
    assert (pb.CAPTURE_HALF_WIDTH_M, pb.CAPTURE_HALF_HEIGHT_M, pb.GRIPPER_SERVO, pb.SPRAYER_SERVO) == (0.10, 0.15, 7, 8)


# --- taking hold of a bucket ---


def hovering_over(bucket_at=(5.0, 5.0, 0.0), hook_miss=(0.0, 0.0, 0.0)):
    """A bridge with the vehicle's hook `hook_miss` away from the handle of a bucket."""
    bridge = Bridge(bucket="pail", hook_offset_m=(0.0, 0.0, -0.30), handle_height_m=0.32)
    bridge.vehicle = "iris"
    assert bridge.start() == [("detach", "pail")] and bridge.start() == []  # Gazebo makes the joint when the world loads: it is unmade once
    bridge.on_position("pail", bucket_at)
    handle = (bucket_at[0], bucket_at[1], bucket_at[2] + 0.32)
    bridge.on_position("iris", (handle[0] + hook_miss[0], handle[1] + hook_miss[1], handle[2] + hook_miss[2] + 0.30))
    bridge.on_servos(20.0, gripper(NEUTRAL))
    return bridge


def test_a_grab_with_the_hook_on_the_handle_attaches_the_bucket():
    bridge = hovering_over(hook_miss=(0.06, -0.05, 0.10))  # 7.8 cm to the side, 10 cm above
    assert bridge.on_servos(20.1, gripper(GRAB)) == [("attach", "pail")]
    grabbed = bridge.events[-1]
    assert grabbed["event"] == "bucket_grabbed" and grabbed["horizontal_m"] == pytest.approx(0.078, abs=0.001) and grabbed["vertical_m"] == pytest.approx(0.10)
    assert bridge.holding and bridge.on_servos(20.2, gripper(GRAB)) == []
    # And the release value sets it down again.
    assert bridge.on_servos(30.0, gripper(LET_GO)) == [("detach", "pail")]
    assert bridge.events[-1]["event"] == "bucket_released" and not bridge.holding


@pytest.mark.parametrize("miss, key", [((0.11, 0.0, 0.0), "horizontal_m"), ((0.0, -0.105, 0.0), "horizontal_m"), ((0.0, 0.0, 0.16), "vertical_m"), ((0.0, 0.0, -0.16), "vertical_m")])
def test_a_grab_outside_the_capture_box_attaches_nothing_and_logs_grab_miss(miss, key):
    bridge = hovering_over(hook_miss=miss)
    assert bridge.on_servos(20.1, gripper(GRAB)) == []
    missed = bridge.events[-1]
    assert missed["event"] == "grab_miss" and missed[key] > (0.10 if key == "horizontal_m" else 0.15) and not bridge.holding
    # Closing again, this time on the handle, takes it.
    bridge.on_servos(21.0, gripper(NEUTRAL))
    bridge.on_position("iris", (5.0, 5.0, 0.62))
    assert bridge.on_servos(21.1, gripper(GRAB)) == [("attach", "pail")]


def test_the_capture_box_is_a_parameter_and_an_unknown_position_is_a_miss():
    wide = Bridge(bucket="pail", capture_half_width_m=0.25)
    wide.vehicle = "iris"
    wide.on_position("pail", (0.0, 0.0, 0.0))
    wide.on_position("iris", (0.2, 0.0, 0.62))
    wide.on_servos(0.0, gripper(NEUTRAL))
    assert wide.on_servos(0.1, gripper(GRAB)) == [("attach", "pail")]
    blind = Bridge(bucket="pail")
    blind.on_servos(0.0, gripper(NEUTRAL))
    assert blind.on_servos(0.1, gripper(GRAB)) == [] and blind.events[-1]["event"] == "grab_miss" and "not known" in blind.events[-1]["why"]
    nothing = Bridge(carried="parcel")
    nothing.on_servos(0.0, gripper(NEUTRAL))
    assert nothing.on_servos(0.1, gripper(GRAB)) == [] and nothing.events[-1]["event"] == "grab_with_nothing_to_take"


def test_the_summary_and_the_servo_message():
    bridge = hovering_over()
    bridge.on_servos(20.1, gripper(GRAB))
    summary = bridge.summary()
    assert summary["ran"] and summary["holding"] and (summary["grabs"], summary["grab_misses"], summary["releases"]) == (1, 0, 0)
    message = SimpleNamespace(servo1_raw=1500, servo7_raw=1100, servo8_raw=0, servo16_raw=1900)
    assert pb.servo_outputs(message) == {1: 1500, 7: 1100, 16: 1900}
    assert pb.joint_topic("parcel", "detach") == "/payload/parcel/detach"
    assert pb.main([]) == 2


def test_the_sims_actuator_parameters():
    params = pb.SITL_PARAMS
    assert (params["GRIP_ENABLE"], params["GRIP_TYPE"], params["SERVO7_FUNCTION"]) == (1, 1, 28)
    assert (params["GRIP_RELEASE"], params["GRIP_GRAB"], params["GRIP_NEUTRAL"]) == (LET_GO, GRAB, NEUTRAL)
    assert (params["SPRAY_ENABLE"], params["SERVO8_FUNCTION"], params["SPRAY_SPEED_MIN"]) == (1, 22, 0)


# --- the scenario builder ---


def test_a_carried_payload_hangs_under_the_vehicle_on_a_detachable_joint(tmp_path):
    run = build_run(RELEASE, tmp_path / "run")
    world = ET.fromstring((run / "world.sdf").read_text()).find("world")
    parcel = next(model for model in world.findall("model") if model.get("name") == "parcel")
    x, y, z, _roll, _pitch, yaw = (float(value) for value in parcel.find("pose").text.split())
    stood = vehicle_pose((SIM / "worlds" / "argus_runway.sdf").read_text(), "iris_downcam")
    assert stood == (0.0, 0.0, 0.195, 90.0)
    # 13 cm to the vehicle's left (the vehicle faces north, so west), its base 2.5 cm off the ground, turned with the vehicle.
    assert (x, y, z) == pytest.approx((-0.13, 0.0, 0.025), abs=1e-4) and yaw == pytest.approx(math.pi / 2.0, abs=1e-5)
    plugin = parcel.find("plugin")
    assert plugin.get("filename") == "gz-sim-detachable-joint-system"
    assert {child.tag: child.text for child in plugin} == {
        "parent_link": "link", "child_model": "iris_downcam", "child_link": "downcam_link",
        "detach_topic": "/payload/parcel/detach", "attach_topic": "/payload/parcel/attach",
    }  # fmt: skip
    assert 'link name="downcam_link"' in (SIM / "models" / "iris_downcam" / "model.sdf").read_text()
    target = next(model for model in world.findall("model") if model.get("name") == "drop_target")
    assert target.find("plugin") is None  # only what hangs from the vehicle has the joint
    manifest = json.loads((run / "scenario.json").read_text())
    assert manifest["payload_bridge"] == {"carried": "parcel", "bucket": None, "capture_half_width_m": 0.10} and pb.wanted(manifest)
    truth = json.loads((run / "truth.json").read_text())
    assert truth["payload"]["name"] == "parcel" and truth["payload"]["mass_kg"] == 0.5
    parameters = dict(line.split() for line in (run / "sitl.parm").read_text().splitlines())
    assert parameters["GRIP_ENABLE"] == "1" and parameters["SERVO8_FUNCTION"] == "22" and parameters["SIM_TERRAIN"] == "0"
    sequence = (run / "sequence.yaml").read_text()
    assert "fake_drop: false" in sequence


def test_a_scenario_with_nothing_to_carry_is_built_as_before(tmp_path):
    run = build_run(BASELINE, tmp_path / "run")
    manifest = json.loads((run / "scenario.json").read_text())
    assert manifest["payload_bridge"] is None and not pb.wanted(manifest)
    assert "detachable" not in (run / "world.sdf").read_text()
    assert (run / "sitl.parm").read_text() == "SIM_TERRAIN 0\n"
    assert "fake_drop: true" in (run / "sequence.yaml").read_text()


def test_what_cannot_be_carried_is_refused(tmp_path):
    text = RELEASE.read_text()
    for change, problem in (
        (("size_m: [0.10, 0.10, 0.10]", "size_m: [0.15, 0.15, 0.10]"), "a carried payload can be at most"),
        (("payload: parcel\n", ""), "one payload can be carried, and it is the scenario's 'payload'"),
        (("    carried: true", "    carried: true\n    at: {east_m: 1.0, north_m: 1.0}"), "exactly one of 'at', 'on_top_of' and 'carried'"),
        (("    carried: true", "    carried: yes please"), "only a payload can be carried"),
    ):
        bad = tmp_path / f"bad{abs(hash(problem))}.yaml"
        bad.write_text(text.replace(*change))
        with pytest.raises(BuildError, match=problem):
            build_run(bad, tmp_path / f"run{abs(hash(problem))}")
    placed = carry({"name": "p", "size_m": 0.1}, (10.0, 5.0, 0.195, 0.0))  # a vehicle facing east: left is north
    assert (placed["e"], placed["n"], placed["up"]) == pytest.approx((10.0, 5.13, 0.025))


# --- the runner ---


def test_the_bridge_runs_beside_flights_with_something_on_a_joint_and_only_those(tmp_path):
    carrying = json.loads((build_run(RELEASE, tmp_path / "a") / "scenario.json").read_text())
    plain = json.loads((build_run(BASELINE, tmp_path / "b") / "scenario.json").read_text())
    assert "payload_bridge" in events.tools_needed(carrying) and "payload_bridge" not in events.tools_needed(plain)
    assert "sim/payload_bridge.py" in events.unsupported(carrying, set())
    assert '"payload_bridge": SIM_DIR / "payload_bridge.py"' in (SIM / "runner" / "fly.py").read_text()


HEARD = {"ran": True, "carried": "parcel", "bucket": None, "released": True, "holding": False, "releases": 1, "grabs": 0, "grab_misses": 0, "servo_messages": 4000, "events": []}


def carrying_run(tmp_path, report):
    run = make_run(tmp_path, score=SCORE, manifest={"variant": "off", "payload_bridge": {"carried": "parcel", "bucket": None}})
    if report is not None:
        (run / "payload_bridge.json").write_text(json.dumps(report))
    return run


def test_a_run_with_a_payload_counts_only_if_the_bridge_heard_the_servos(tmp_path):
    heard = result.run_result(carrying_run(tmp_path / "a", HEARD))
    assert heard["status"] == "OK" and "| Payload bridge | parcel released (4000 servo messages heard) |" in result.render_report(heard)
    held = result.run_result(carrying_run(tmp_path / "b", {**HEARD, "released": False, "releases": 0}))
    assert held["status"] == "OK" and "parcel not released" in result.render_report(held)  # the judge scores that: nothing released
    absent = result.run_result(carrying_run(tmp_path / "c", None))
    assert absent["status"] == "INVALID" and "payload_bridge.json" in absent["reason"]
    deaf = result.run_result(carrying_run(tmp_path / "d", {**HEARD, "servo_messages": 0}))
    assert deaf["status"] == "INVALID"
    plain = result.run_result(make_run(tmp_path / "e", score=SCORE, manifest={"variant": "off", "payload_bridge": None}))
    assert plain["status"] == "OK" and plain["payload_bridge"] is None and "Payload bridge" not in result.render_report(plain)
    assert pb.read_bridge(tmp_path / "nowhere") is None


# --- the mission's bucket: a joint that Gazebo makes at start and the bridge unmakes ---

DELIVERY_RELEASE = SIM / "scenarios" / "delivery" / "release.yaml"


def test_the_missions_bucket_gets_a_joint_too_and_the_bridge_is_told_to_unmake_it(tmp_path):
    run = build_run(DELIVERY_RELEASE, tmp_path / "run")
    world = ET.fromstring((run / "world.sdf").read_text()).find("world")
    pail = next(model for model in world.findall("model") if model.get("name") == "pail")
    plugin = pail.find("plugin")
    assert plugin.get("filename") == "gz-sim-detachable-joint-system"
    assert plugin.find("child_model").text == "iris_downcam" and plugin.find("detach_topic").text == "/payload/pail/detach"
    assert [float(value) for value in pail.find("pose").text.split()[:3]] == pytest.approx([-8.0, 0.0, 0.0])  # where the scenario put it
    manifest = json.loads((run / "scenario.json").read_text())
    assert manifest["payload_bridge"] == {"carried": "parcel", "bucket": "pail", "capture_half_width_m": 0.10}
    bridge = Bridge(carried=manifest["payload_bridge"]["carried"], bucket=manifest["payload_bridge"]["bucket"])
    assert bridge.start() == [("detach", "pail")]  # the carried package's joint is left made
    assert "GRIP_ENABLE 1" in (run / "sitl.parm").read_text() and "fake_drop: false" in (run / "sequence.yaml").read_text()
    # A driver flies over the tools port the bridge listens on: the two cannot share a scenario.
    hover_with_bucket = tmp_path / "bad.yaml"
    hover_with_bucket.write_text(
        (SIM / "scenarios" / "hover" / "wind_5.yaml").read_text() + "bucket: pail\nobjects:\n  - name: pail\n    kind: bucket\n    at: {east_m: 5.0, north_m: 5.0}\n"
    )
    with pytest.raises(BuildError, match="no carried payload or bucket in a driver scenario"):
        build_run(hover_with_bucket, tmp_path / "bad")


# --- a release the mission commanded and the bridge never carried out ---


def with_release(run, fake):
    """Add the mission's release line to a run's trace, as the runner writes it."""
    lines = [json.loads(line) for line in (run / "trace.jsonl").read_text().splitlines()]
    middle = sorted(line["t"] for line in lines)[len(lines) // 2]
    lines.append({"t": middle, "event": "release", "fake": fake, "what": "Fake drop complete" if fake else "Closing sprayer for payload drop"})
    (run / "trace.jsonl").write_text("".join(json.dumps(line) + "\n" for line in sorted(lines, key=lambda line: line["t"])))
    return run


def test_a_commanded_release_that_never_detached_fails_the_run(tmp_path):
    held = {**HEARD, "released": False, "releases": 0}
    # A fake drop with the package still on the vehicle: as it should be.
    assert result.run_result(with_release(carrying_run(tmp_path / "a", held), fake=True))["status"] == "OK"
    failed = result.run_result(with_release(carrying_run(tmp_path / "b", held), fake=False))
    assert failed["status"] == "FAIL" and "commanded a release but the payload bridge never let parcel go" in failed["reason"]
    assert result.run_result(with_release(carrying_run(tmp_path / "c", HEARD), fake=False))["status"] == "OK"
    # The suite's count line names the failed run (it used to leave FAIL out).
    table = result.suite_table([failed, result.run_result(with_release(carrying_run(tmp_path / "d", HEARD), fake=False))])
    assert table.rstrip().endswith("2 runs: 1 OK, 1 FAIL")


# --- the impulse a released body lands with ---


def fall(watcher, name, t0, height, dt=1.0 / 60.0, settle_s=1.5, bounce=0.0):
    """Feed a free fall from `height` and what follows; returns the events and the speed at the ground."""
    found, t, z, v = [], t0, height, 0.0
    impact_speed = None
    while t < t0 + 6.0:
        t += dt
        v -= 9.80665 * dt
        z += v * dt
        if z <= 0.0:
            impact_speed = impact_speed if impact_speed is not None else -v
            z, v = 0.0, -v * bounce if abs(v) * bounce > 0.3 else 0.0
        event = watcher.update(name, t, (3.0, 4.0, z))
        if event:
            found.append(event)
    return found, impact_speed


def test_a_dropped_package_reports_one_contact_with_its_mass_times_its_change_of_velocity():
    watcher = events.LandingWatcher({"parcel": 0.5})
    for index in range(60):  # carried, hovering at 4 m: nothing to report
        assert watcher.update("parcel", 10.0 + index / 60.0, (3.0, 4.0, 4.0)) is None
    watcher.released(11.0)
    found, impact_speed = fall(watcher, "parcel", 11.0, 4.0)
    (contact,) = found
    assert impact_speed == pytest.approx(math.sqrt(2 * 9.80665 * 4.0), rel=0.03)  # 8.9 m/s
    assert contact["event"] == "contact" and contact["name"] == "parcel"
    assert contact["velocity_change_mps"] == pytest.approx(impact_speed, rel=0.05) and contact["impulse_ns"] == pytest.approx(0.5 * impact_speed, rel=0.05)


def test_a_bounce_reports_the_hardest_contact_and_a_gentle_set_down_reports_almost_none():
    bouncing = events.LandingWatcher({"parcel": 0.5})
    bouncing.update("parcel", 0.0, (3.0, 4.0, 2.0))
    bouncing.released(0.0)
    found, impact_speed = fall(bouncing, "parcel", 0.0, 2.0, bounce=0.5)
    (contact,) = found
    assert contact["velocity_change_mps"] == pytest.approx(1.5 * impact_speed, rel=0.08)  # down at v, up at v/2: the first is the hardest
    gentle = events.LandingWatcher({"parcel": 0.5})
    found = []
    for index in range(400):  # lowered at 0.25 m/s for two seconds, then on the ground
        t = index / 60.0
        z = max(0.0, 0.5 - 0.25 * t)
        if index == 100:
            gentle.released(t)
        event = gentle.update("parcel", t, (3.0, 4.0, z))
        if event:
            found.append(event)
    (contact,) = found
    assert contact["impulse_ns"] == pytest.approx(0.5 * 0.25, abs=0.02)  # it stopped from 0.25 m/s: an eighth of a newton second


def test_nothing_is_reported_before_a_release_or_for_a_body_nobody_weighed():
    watcher = events.LandingWatcher({"parcel": 0.5})
    found, _ = fall(watcher, "parcel", 0.0, 3.0)
    assert found == []  # it fell, but no release was commanded: not this watcher's business
    assert watcher.update("stranger", 1.0, (0.0, 0.0, 0.0)) is None
    two = events.LandingWatcher({"parcel": 0.5, "pail": 1.5})
    two.released(0.0, name="pail")
    assert fall(two, "parcel", 0.0, 3.0)[0] == [] and len(fall(two, "pail", 0.0, 3.0)[0]) == 1


def test_the_recorder_is_given_the_masses_and_told_of_real_releases():
    fly = (SIM / "runner" / "fly.py").read_text()
    assert "self.landing = events.LandingWatcher(masses_kg or {})" in fly and "recorder.landing.released(recorder.now(), name=released_body)" in fly
    assert 'released_body = (truth.get("payload") or truth.get("bucket") or {}).get("name")' in fly
    assert 'if item["event"] == "release" and not item.get("fake"):' in fly


# --- REC-8: the recovery vehicle's hook, and a grab that is refused (run R6) ---


def test_a_bridge_told_to_refuse_grabs_attaches_nothing_until_they_are_used_up():
    """A latch that did not close: the hook is on the handle, the gripper output says grab, and nothing is attached.
    The mission's attach check has to find that out for itself; the bridge only records it."""
    bridge = Bridge(bucket="pail", hook_offset_m=(0.0, 0.0, -0.30), handle_height_m=0.32, refuse_grabs=2)
    bridge.vehicle = "iris"
    bridge.start()
    bridge.on_position("pail", (5.0, 5.0, 0.0))
    bridge.on_position("iris", (5.0, 5.0, 0.62))
    bridge.on_servos(20.0, gripper(NEUTRAL))
    for attempt, left in ((1, 1), (2, 0)):
        assert bridge.on_servos(20.0 + attempt, gripper(GRAB)) == [] and not bridge.holding
        assert bridge.events[-1]["event"] == "grab_refused" and bridge.events[-1]["refusals_left"] == left
        bridge.on_servos(20.5 + attempt, gripper(NEUTRAL))
    assert bridge.on_servos(23.0, gripper(GRAB)) == [("attach", "pail")] and bridge.holding  # the third try takes
    summary = bridge.summary()
    assert (summary["grabs"], summary["grabs_refused"], summary["grab_misses"]) == (1, 2, 0)
    # A grab outside the envelope is a miss, not a refusal: it does not use one up.
    away = Bridge(bucket="pail", hook_offset_m=(0.0, 0.0, -0.30), handle_height_m=0.32, refuse_grabs=1)
    away.vehicle = "iris"
    away.start()
    away.on_position("pail", (5.0, 5.0, 0.0))
    away.on_position("iris", (5.5, 5.0, 0.62))
    away.on_servos(20.0, gripper(NEUTRAL))
    away.on_servos(21.0, gripper(GRAB))
    assert away.events[-1]["event"] == "grab_miss" and away.refusals_left == 1
    assert Bridge(bucket="pail").refusals_left == 0  # by default every grab in the envelope takes


def test_the_recovery_vehicles_hook_is_where_the_built_run_says_and_a_grab_beside_the_handle_misses(tmp_path):
    """Through a built recovery run: the bridge's settings put the hook's tip 0.55 m under the vehicle, the envelope
    is the bridge's own 0.10 m, and a hook 0.12 m to the side takes nothing."""
    from scenario.build import build_run

    run = build_run(SIM / "scenarios" / "recovery" / "hook_check.yaml", tmp_path / "run", variant="on")
    settings = pb.settings_from(json.loads((run / "scenario.json").read_text()))
    assert settings["hook_offset_m"] == [0.0, 0.0, -0.55] and settings["capture_half_width_m"] == pb.CAPTURE_HALF_WIDTH_M == 0.10 and "refuse_grabs" not in settings

    def grab(vehicle_at):
        bridge = Bridge(bucket=settings["bucket"], hook_offset_m=tuple(settings["hook_offset_m"]), capture_half_width_m=settings["capture_half_width_m"])
        bridge.vehicle = "iris_recovery"
        bridge.start()
        bridge.on_position("bucket", (12.0, 8.0, 0.0))
        bridge.on_position("iris_recovery", vehicle_at)
        bridge.on_servos(20.0, gripper(NEUTRAL))
        return bridge.on_servos(21.0, gripper(GRAB)), bridge.events[-1]

    actions, event = grab((12.0, 8.0, 0.32 + 0.55))
    assert actions == [("attach", "bucket")] and event["event"] == "bucket_grabbed"
    actions, event = grab((12.12, 8.0, 0.32 + 0.55))
    assert actions == [] and event["event"] == "grab_miss" and event["horizontal_m"] == pytest.approx(0.12)
    actions, event = grab((12.0, 8.0, 0.32 + 0.30))  # the delivery vehicle's hook height: 0.25 m too low for this hook
    assert actions == [] and event["event"] == "grab_miss" and event["vertical_m"] == pytest.approx(0.25)


def test_a_bucket_the_bridge_let_go_is_a_release_for_the_judge_unless_the_mission_already_said_so():
    """The flight-safety review: the recovery judge holds a release to 1.83 m only at a `release` event, and those
    came only from a mission's log line. A scripted flight has none; the bridge saw the release itself."""
    lines = [
        json.dumps({"t": 52.036, "event": "bucket_grabbed", "name": "bucket", "pwm": 1900}),
        json.dumps({"t": 80.285, "event": "bucket_released", "name": "bucket", "output": "gripper", "pwm": 1100}),
        "not json",
        json.dumps({"t": 90.0, "event": "payload_released", "name": "parcel"}),  # a carried payload's release is the mission's to tell of, as before
    ]
    ((t, event),) = events.from_bridge_log(lines)
    assert t == 80.285 and event == {"event": "release", "fake": False, "what": "payload bridge: bucket let go", "source": "payload_bridge"}
    assert events.from_bridge_log(lines, known_release_times=[79.0]) == []  # the mission's own log line told of it a second before
    assert len(events.from_bridge_log(lines, known_release_times=[60.0])) == 1 and events.from_bridge_log([]) == []
