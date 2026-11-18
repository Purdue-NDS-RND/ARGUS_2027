"""The sim's sensors (SIM-05): camera profiles from one template, true poses on a topic, and the sensor check. No ROS, no Gazebo."""

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
REPO = SIM.parent
sys.path.insert(0, str(SIM))

import sensor_check  # noqa: E402
import sensor_profiles  # noqa: E402
import truth_poses  # noqa: E402
from runner import events, result  # noqa: E402
from scenario.build import BuildError, build_run  # noqa: E402
from scenario.spec import SENSOR_PROFILES  # noqa: E402

from test_runner import SCORE, make_run  # noqa: E402  (the run-folder fixture)

SURVEY = SIM / "scenarios" / "hover" / "survey_camera.yaml"
BASELINE = SIM / "scenarios" / "delivery" / "baseline.yaml"


# --- the profiles ---


def test_the_profiles_are_the_specs():
    profiles = sensor_profiles.PROFILES
    assert {name: (p["width"], p["height"], p["rate_hz"]) for name, p in profiles.items()} == {
        "delivery": (640, 360, 30), "survey": (1280, 720, 10), "still4k": (3840, 2160, 1), "flow": (640, 360, 30),
    }  # fmt: skip
    assert sensor_profiles.FLOW_CAMERA == {"width": 640, "height": 400, "rate_hz": 60}
    assert [name for name, p in profiles.items() if p["headless_only"]] == ["still4k"] and [name for name, p in profiles.items() if p["flow"]] == ["flow"]
    # One field of view for every downward camera: the IMX519's as calibrated.
    assert sensor_profiles.HORIZONTAL_FOV_RAD == pytest.approx(2.0 * math.atan(sensor_profiles.IMX519_WIDTH_PX / 2.0 / sensor_profiles.IMX519_FX_PX), abs=5e-6)
    assert SENSOR_PROFILES["survey"]["model"] == "iris_survey" and SENSOR_PROFILES["delivery"] == {
        "model": "iris_downcam", "bridge": "config/bridge.yaml", "mass_kg": 1.755, "cda_m2": 0.08, "min_rate_hz": None, "flow_camera": False, "headless_only": False,
    }  # fmt: skip
    assert SENSOR_PROFILES["survey"]["min_rate_hz"] == 8.0 and SENSOR_PROFILES["flow"]["flow_camera"] is True


def test_every_generated_file_is_what_the_template_gives_and_todays_model_is_the_delivery_profile():
    assert sensor_profiles.stale() == []
    assert sensor_profiles.main(["--check"]) == 0
    files = sensor_profiles.generated_files()
    assert files[SIM / "models" / "iris_downcam" / "model.sdf"] == (SIM / "models" / "iris_downcam" / "model.sdf").read_text()
    assert files[SIM / "config" / "bridge.yaml"] == (SIM / "config" / "bridge.yaml").read_text()
    assert len(files) == 4 * 3 + 3  # a model, its config and a bridge file each; a calibration for survey, still4k and the flow camera


@pytest.mark.parametrize("name", ["delivery", "survey", "still4k", "flow"])
def test_a_profiles_model_has_its_camera(name):
    profile = sensor_profiles.PROFILES[name]
    model = ET.fromstring((SIM / "models" / profile["model"] / "model.sdf").read_text()).find("model")
    assert model.get("name") == profile["model"]
    cameras = {sensor.get("name"): sensor for sensor in model.iter("sensor")}
    down = cameras["downcam"]
    assert (int(down.find("camera/image/width").text), int(down.find("camera/image/height").text), int(down.find("update_rate").text)) == (profile["width"], profile["height"], profile["rate_hz"])
    assert float(down.find("camera/horizontal_fov").text) == sensor_profiles.HORIZONTAL_FOV_RAD and down.find("topic").text == "downcam/image"
    assert model.find("link[@name='downcam_link']") is not None  # what a carried payload's joint holds on to
    if profile["flow"]:
        flow = cameras["flowcam"]
        assert (int(flow.find("camera/image/width").text), int(flow.find("camera/image/height").text), int(flow.find("update_rate").text)) == (640, 400, 60)
        assert flow.find("camera/image/format").text == "L8" and flow.find("topic").text == "flowcam/image"
    else:
        assert set(cameras) == {"downcam"}
    bridge = (SIM / profile["bridge"]).read_text()
    assert ("/argus/camera/flow/image_raw" in bridge) is profile["flow"] and '"/camera/image"' in bridge


def test_the_calibrations_are_ideal_pinholes_of_the_same_field_of_view():
    import yaml

    for name in ("survey", "still4k"):
        profile = sensor_profiles.PROFILES[name]
        data = yaml.safe_load((REPO / "src" / "argus_camera" / "config" / "calibration" / f"sim_{name}.yaml").read_text())
        fx = profile["width"] / (2.0 * math.tan(sensor_profiles.HORIZONTAL_FOV_RAD / 2.0))
        assert (data["width"], data["height"], data["measured"]) == (profile["width"], profile["height"], True)
        assert data["k"] == pytest.approx([fx, 0.0, profile["width"] / 2.0, 0.0, fx, profile["height"] / 2.0, 0.0, 0.0, 1.0], abs=1e-4)
        assert data["d"] == [0.0] * 5
    delivery = yaml.safe_load((REPO / "src" / "argus_camera" / "config" / "calibration" / "sim_delivery.yaml").read_text())
    assert delivery["k"][0] == pytest.approx(sensor_profiles.focal_px(640), abs=1e-4)  # the hand-written one agrees
    flow = yaml.safe_load((REPO / "src" / "argus_camera" / "config" / "calibration" / "sim_flow.yaml").read_text())
    assert (flow["width"], flow["height"]) == (640, 400) and flow["k"][0] == pytest.approx(320.0 / math.tan(sensor_profiles.FLOW_HORIZONTAL_FOV_RAD / 2.0), abs=1e-4)
    modes = yaml.safe_load((REPO / "src" / "argus_camera" / "config" / "modes.yaml").read_text())["modes"]
    assert (modes["sim_survey"]["width"], modes["sim_survey"]["height"], modes["sim_survey"]["calibration"]) == (1280, 720, "calibration/sim_survey.yaml")
    assert (modes["sim_still4k"]["width"], modes["sim_still4k"]["fps"]) == (3840, 1)
    assert (modes["sim_flow"]["camera"], modes["sim_flow"]["width"], modes["sim_flow"]["height"], modes["sim_flow"]["fps"]) == ("flow", 640, 400, 60)


def test_a_scenarios_profile_picks_the_model_the_bridge_file_and_the_manifest(tmp_path):
    run = build_run(SURVEY, tmp_path / "survey")
    world = (run / "world.sdf").read_text()
    assert "<uri>model://iris_survey</uri>" in world and "model://iris_downcam" not in world
    manifest = json.loads((run / "scenario.json").read_text())
    assert manifest["vehicle"]["model"] == "iris_survey" and manifest["vehicle"]["sensor_profile"] == "survey" and manifest["vehicle"]["min_rate_hz"] == 8.0
    assert manifest["sensor_check"] is True and manifest["truth_poses"] is True
    assert (run / "bridge.yaml").read_text() == (SIM / "config" / "bridge_survey.yaml").read_text()
    plain = build_run(BASELINE, tmp_path / "plain")
    assert "<uri>model://iris_downcam</uri>" in (plain / "world.sdf").read_text()
    plain_manifest = json.loads((plain / "scenario.json").read_text())
    assert plain_manifest["sensor_check"] is False and plain_manifest["truth_poses"] is False and plain_manifest["vehicle"]["model"] == "iris_downcam"
    bad = tmp_path / "bad.yaml"
    bad.write_text(SURVEY.read_text().replace("sensor_profile: survey", "sensor_profile: thermal"))
    with pytest.raises(BuildError, match="sensor_profile"):
        build_run(bad, tmp_path / "bad")


def test_the_4k_profile_is_refused_a_window(tmp_path):
    import subprocess

    run = build_run(SIM / "scenarios" / "hover" / "still4k_camera.yaml", tmp_path / "still")
    manifest = json.loads((run / "scenario.json").read_text())
    assert manifest["vehicle"]["headless_only"] is True and manifest["vehicle"]["model"] == "iris_still4k" and manifest["vehicle"]["min_rate_hz"] is None
    start = (SIM / "container" / "start.sh").read_text()
    line = next(line for line in start.splitlines() if "headless_only" in line)
    assert line.strip().startswith("if [[ ${HEADLESS:-0} != 1 ]] && python3 -c") and "this run's camera profile is headless only" in start
    # The line's own test, run as the start script runs it: true for this run folder, false for a delivery one.
    probe = line.strip()[len("if [[ ${HEADLESS:-0} != 1 ]] && ") : -len("; then")]
    plain = build_run(BASELINE, tmp_path / "plain")
    assert subprocess.run(["bash", "-c", probe], env={"RUN_DIR": str(run), "PATH": "/usr/bin:/bin"}).returncode == 0
    assert subprocess.run(["bash", "-c", probe], env={"RUN_DIR": str(plain), "PATH": "/usr/bin:/bin"}).returncode == 1


# --- true poses ---


def test_true_poses_go_out_for_the_vehicle_and_the_scenarios_objects_at_30_hz_of_sim_time():
    names = truth_poses.names_of({"vehicle": {"model": "iris_survey"}}, {"objects": [{"name": "target"}, {"name": "pail"}]})
    assert names == ["iris_survey", "target", "pail"] and truth_poses.TOPIC == "/sim/truth/poses" and truth_poses.RATE_HZ == 30.0
    selection = truth_poses.Selection(names)
    assert selection.due(1.0) is None  # nothing known yet
    pose = (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    selection.add("iris_survey", pose)
    selection.add("a_stranger", pose)
    selection.add("pail", (5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0))
    assert selection.due(1.0) == [("iris_survey", pose), ("pail", (5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0))]  # in the list's order, strangers left out
    assert selection.due(1.02) is None and selection.due(1.0 + 1.0 / 30.0) is not None  # a thirtieth of a second apart
    sent = sum(1 for index in range(1, 601) if selection.due(2.0 + index / 60.0) is not None)
    assert sent == pytest.approx(300, abs=2)  # poses at 60 Hz for ten seconds: half are published
    assert truth_poses.wanted({"truth_poses": True}) and not truth_poses.wanted({}) and truth_poses.main([]) == 2
    # The node spins, so that it answers for its use_sim_time parameter (sim/time_check.py asks every node on a sim-time run).
    assert "rclpy.spin_once(node, timeout_sec=0.2)" in (SIM / "truth_poses.py").read_text()


# --- the sensor check ---


def hover_truth(seconds=40.0, height=10.0, start_z=0.195):
    truth = sensor_check.TrueHeight()
    t = 0.0
    while t <= seconds + 20.0:
        if t < 10.0:
            z = start_z + height * t / 10.0
        elif t < 10.0 + seconds:
            z = start_z + height
        else:
            z = start_z + height * max(0.0, 1.0 - (t - 10.0 - seconds) / 10.0)
        truth.add(t, (0.0, 0.0, z), (0.0, 0.0, 0.0, 1.0))
        t += 1.0 / 60.0
    return truth


def test_frame_rate_is_frames_per_second_of_sim_time_and_the_slowest_window():
    steady = [index / 10.0 for index in range(601)]  # 10 Hz for a minute
    rate = sensor_check.frame_rate(steady)
    assert rate["frames"] == 601 and rate["rate_hz"] == pytest.approx(10.0) and rate["slowest_window_hz"] == pytest.approx(10.0)
    stalled = [t for t in steady if not 20.0 <= t < 24.0]  # four seconds with no frames
    assert sensor_check.frame_rate(stalled)["slowest_window_hz"] == pytest.approx(2.0) and sensor_check.frame_rate(stalled)["rate_hz"] == pytest.approx(560 / 60.0, abs=0.02)
    assert sensor_check.frame_rate([]) is None and sensor_check.frame_rate([1.0]) is None
    assert sensor_check.TRUTH_TOPIC == truth_poses.TOPIC and truth_poses.RATE_HZ > sensor_check.TRUTH_MIN_RATE_HZ


def test_the_rangefinder_is_judged_against_how_far_the_vehicle_has_risen_and_only_in_a_hover():
    truth = hover_truth()
    readings = [(t / 20.0, max(0.0, truth.at(t / 20.0)["height_m"] + 0.03)) for t in range(10, 1100) if truth.at(t / 20.0) is not None]
    check = sensor_check.range_check(readings, truth)
    assert check["passed"] and check["p95_m"] == pytest.approx(0.03, abs=1e-6) and 700 < check["in_hover"] < 820  # the 40 s hold at 20 Hz
    assert truth.at(20.0)["height_m"] == pytest.approx(10.0) and truth.start_z == 0.195  # not the height of the model's origin
    off = sensor_check.range_check([(t, r + 0.15) for t, r in readings], truth)
    assert not off["passed"] and off["p95_m"] == pytest.approx(0.18, abs=1e-6)
    few = sensor_check.range_check(readings[220:260], truth)
    assert few["in_hover"] == 40 and not few["passed"] and few["p95_m"] is None
    # Readings during the climb are wrong by the climb's own speed and are not counted.
    climbing = sensor_check.range_check([(t / 20.0, 0.0) for t in range(20, 180)], truth)
    assert climbing["in_hover"] == 0


def test_the_verdict():
    truth = hover_truth()
    readings = [(t / 20.0, truth.at(t / 20.0)["height_m"]) for t in range(10, 1100)]
    frames = [index / 9.0 for index in range(540)]  # 9 Hz for a minute
    good = sensor_check.build_report(frames, [], readings, truth, min_rate_hz=8.0, flow_expected=False)
    assert good["passed"] and good["problems"] == [] and good["camera"]["rate_hz"] == pytest.approx(9.0)
    lines = sensor_check.summary_lines(good)
    assert lines[0].startswith("camera: 9.0 frames a second of sim time over 540 frames") and "(at least 8 needed)" in lines[0]
    assert lines[1].startswith("rangefinder against the true height in a hover: PASS, median 0.000 m")
    slow = sensor_check.build_report([index / 7.0 for index in range(420)], [], readings, truth, 8.0, False)
    assert not slow["passed"] and "7.0 frames a second of sim time (at least 8 needed)" in slow["problems"][0]
    assert sensor_check.build_report([index / 7.0 for index in range(420)], [], readings, truth, None, False)["passed"]  # no floor on this profile
    no_flow = sensor_check.build_report(frames, [], readings, truth, 8.0, flow_expected=True)
    assert not no_flow["passed"] and no_flow["problems"] == ["no flow-camera frames were heard"]
    with_flow = sensor_check.build_report(frames, [index / 60.0 for index in range(3600)], readings, truth, None, True)
    assert with_flow["passed"] and sensor_check.summary_lines(with_flow)[1].startswith("flow camera: 60.0 frames a second")
    # True poses, when the scenario publishes them, must be heard at about their 30 Hz.
    heard = sensor_check.build_report(frames, [], readings, truth, 8.0, False, truth_stamps=[index / 30.0 for index in range(1800)])
    assert heard["passed"] and heard["truth_poses"]["rate_hz"] == pytest.approx(30.0)
    assert "true poses on /sim/truth/poses: 30.0 messages a second of sim time over 1800" in sensor_check.summary_lines(heard)
    silent = sensor_check.build_report(frames, [], readings, truth, 8.0, False, truth_stamps=[])
    assert silent["problems"] == ["true poses were asked for on /sim/truth/poses and nothing was heard (at least 25 needed)"]
    slow_truth = sensor_check.build_report(frames, [], readings, truth, 8.0, False, truth_stamps=[index / 10.0 for index in range(600)])
    assert not slow_truth["passed"] and "10.0 messages a second" in slow_truth["problems"][0]
    assert good["truth_poses"] is None  # not asked for: not judged
    blind = sensor_check.build_report([], [], [], sensor_check.TrueHeight(), 8.0, False)
    assert blind["problems"] == ["no camera frames were heard", "only 0 rangefinder readings in a hover (at least 50 needed)"]
    assert sensor_check.main([]) == 2


def serialised_image_head(sec, nanosec, frame_id, height, width, little=True):
    """The first bytes of a sensor_msgs/Image as ROS serialises it."""
    import struct

    order = "<" if little else ">"
    text = frame_id.encode() + b"\x00"
    body = struct.pack(order + "iII", sec, nanosec, len(text)) + text
    body += b"\x00" * (-len(body) % 4)
    return (b"\x00\x01\x00\x00" if little else b"\x00\x00\x00\x00") + body + struct.pack(order + "II", height, width) + b"rgb8 and the rest"


def test_a_frames_stamp_and_size_are_read_from_its_first_bytes():
    assert sensor_check.image_header(serialised_image_head(12, 500_000_000, "downcam_link", 720, 1280)) == (12.5, 1280, 720)
    assert sensor_check.image_header(serialised_image_head(3, 0, "iris_still4k::downcam_link::downcam", 2160, 3840)) == (3.0, 3840, 2160)
    assert sensor_check.image_header(serialised_image_head(7, 250_000_000, "", 360, 640, little=False)) == (7.25, 640, 360)
    assert sensor_check.image_header(b"\x00\x01\x00\x00\x01\x02") is None and sensor_check.image_header(b"") is None


def test_the_rate_floor_is_held_against_the_frames_that_arrived():
    truth = hover_truth()
    readings = [(t / 20.0, truth.at(t / 20.0)["height_m"]) for t in range(10, 1100)]
    info = [index / 10.0 for index in range(600)]  # camera_info at the sensor's 10 Hz
    arrived = sensor_check.build_report(info, [], readings, truth, 8.0, False, image_stamps=[index / 9.5 for index in range(570)], image_size=(1280, 720))
    assert arrived["passed"] and arrived["images"]["rate_hz"] == pytest.approx(9.5) and arrived["image_size"] == [1280, 720]
    assert sensor_check.summary_lines(arrived)[0].startswith("camera: 9.5 frames of 1280 x 720 a second of sim time arrived, 570 in all")
    # camera_info at 10 Hz says nothing if only six frames a second reach ROS.
    lost = sensor_check.build_report(info, [], readings, truth, 8.0, False, image_stamps=[index / 6.0 for index in range(360)], image_size=(1280, 720))
    assert not lost["passed"] and "6.0 frames a second of sim time (at least 8 needed)" in lost["problems"][0]
    none = sensor_check.build_report(info, [], readings, truth, 8.0, False, image_stamps=[])
    assert none["problems"] == ["no camera frames were heard"]


def test_true_poses_must_name_the_vehicle_and_every_object():
    truth = hover_truth()
    readings = [(t / 20.0, truth.at(t / 20.0)["height_m"]) for t in range(10, 1100)]
    frames = [index / 10.0 for index in range(600)]
    stamps = [index / 30.0 for index in range(1800)]
    heard = sensor_check.build_report(frames, [], readings, truth, None, False, stamps, truth_names=["target", "iris_survey", "pail"], truth_expected=["iris_survey", "target", "pail"])
    assert heard["passed"] and heard["truth_poses"]["names"] == ["iris_survey", "pail", "target"]
    assert sensor_check.summary_lines(heard)[1].endswith("over 1800, for iris_survey, pail, target")
    missing = sensor_check.build_report(frames, [], readings, truth, None, False, stamps, truth_names=["iris_survey"], truth_expected=["iris_survey", "target", "pail"])
    assert missing["problems"] == ["no true pose was heard for: pail, target"]
    # Not asked for: names are not judged.
    assert sensor_check.build_report(frames, [], readings, truth, None, False, None, truth_expected=["iris_survey", "target"])["passed"]


def test_rangefinder_stamp_ages_are_reported_on_sim_time_runs():
    assert sensor_check.age_summary([]) is None
    ages = sensor_check.age_summary([0.010] * 90 + [0.050] * 10)
    assert ages["count"] == 100 and ages["median_ms"] == pytest.approx(10.0) and ages["p95_ms"] == pytest.approx(50.0)
    truth = hover_truth()
    readings = [(t / 20.0, truth.at(t / 20.0)["height_m"]) for t in range(10, 1100)]
    report = sensor_check.build_report([index / 10.0 for index in range(600)], [], readings, truth, None, False, range_ages_s=[0.012] * 200)
    assert report["rangefinder"]["stamp_age"]["median_ms"] == pytest.approx(12.0)
    assert sensor_check.summary_lines(report)[-1].endswith("readings 12 ms old by their stamps at the median, 12 ms at p95")
    assert "raw=True" in (SIM / "sensor_check.py").read_text()  # frames are not unpacked


def test_on_sim_time_a_reading_sits_at_its_own_stamp_and_stamps_off_the_clock_fail_the_check():
    assert sensor_check.place_reading(100.0, 99.996) == (99.996, pytest.approx(0.004))  # on the clock: placed at its stamp
    assert sensor_check.place_reading(100.0, 100.0) == (100.0, 0.0)
    assert sensor_check.place_reading(100.0, 100.2) == (100.0, None)  # stamped ahead of the clock: placed when heard, counted
    assert sensor_check.place_reading(100.0, 98.5) == (100.0, None)  # over a second behind
    assert sensor_check.place_reading(100.0, 1794983472.6) == (100.0, None)  # a wall-clock stamp
    assert sensor_check.stamp_problem(0, []) is None and sensor_check.stamp_problem(1000, [0.002] * 1000) is None
    assert sensor_check.stamp_problem(1000, [0.002] * 960) is None  # 4% off the clock is let by
    assert "60 of 1000 rangefinder readings were stamped off the sim clock" in sensor_check.stamp_problem(1000, [0.002] * 940)
    assert "all" not in sensor_check.stamp_problem(1000, []) and "1000 of 1000" in sensor_check.stamp_problem(1000, [])
    assert "300 ms old by their stamps at the 95th percentile (limit 100 ms)" in sensor_check.stamp_problem(1000, [0.3] * 1000)
    truth = hover_truth()
    readings = [(t / 20.0, truth.at(t / 20.0)["height_m"]) for t in range(10, 1100)]
    frames = [index / 10.0 for index in range(600)]
    late = sensor_check.build_report(frames, [], readings, truth, None, False, range_ages_s=[0.3] * len(readings), stamps_judged=True)
    assert not late["passed"] and "300 ms old" in late["problems"][0]
    unstamped = sensor_check.build_report(frames, [], readings, truth, None, False, range_ages_s=[], stamps_judged=True)
    assert not unstamped["passed"] and "stamped off the sim clock" in unstamped["problems"][0]
    assert sensor_check.build_report(frames, [], readings, truth, None, False, range_ages_s=[], stamps_judged=False)["passed"]  # a wall-time run: not judged
    assert sensor_check.build_report(frames, [], readings, truth, None, False, range_ages_s=[0.002] * len(readings), stamps_judged=True)["passed"]


def test_report_file_round_trip(tmp_path):
    assert sensor_check.read_sensor_check(tmp_path) is None
    (tmp_path / "sensor_check.json").write_text(json.dumps({"passed": True, "problems": []}))
    assert sensor_check.read_sensor_check(tmp_path)["passed"] is True


# --- the runner ---


def test_the_tools_run_beside_flights_that_ask_for_them():
    asked = {"wind": {}, "gps_events": [], "sensor_check": True, "truth_poses": True}
    assert events.tools_needed(asked) == {"sensor_check", "truth_poses"} and events.tools_needed({"wind": {}, "gps_events": []}) == set()
    assert "sim/sensor_check.py" in events.unsupported(asked, {"truth_poses"}) and "sim/truth_poses.py" in events.unsupported(asked, {"sensor_check"})
    fly = (SIM / "runner" / "fly.py").read_text()
    assert '"sensor_check": SIM_DIR / "sensor_check.py"' in fly and '"truth_poses": SIM_DIR / "truth_poses.py"' in fly


PASSED = {"passed": True, "problems": [], "min_rate_hz": 8.0, "camera": {"frames": 520, "rate_hz": 9.6, "slowest_window_hz": 9.2}, "flow_camera": None,
          "rangefinder": {"readings": 900, "in_hover": 700, "limit_m": 0.1, "median_m": 0.01, "p95_m": 0.03, "max_m": 0.05, "passed": True}}  # fmt: skip


def checked_run(tmp_path, report):
    run = make_run(tmp_path, score=SCORE, manifest={"variant": "off", "sensor_check": True})
    if report is not None:
        (run / "sensor_check.json").write_text(json.dumps(report))
    return run


def test_a_run_that_asked_for_the_sensor_check_counts_only_if_it_passed(tmp_path):
    passed = result.run_result(checked_run(tmp_path / "a", PASSED))
    assert passed["status"] == "OK"
    text = result.render_report(passed)
    assert "| Sensors | camera: 9.6 frames a second of sim time over 520 frames, slowest 5 s 9.2 Hz (at least 8 needed) |" in text
    assert "| Sensors | rangefinder against the true height in a hover: PASS, median 0.010 m, p95 0.030 m over 700 readings (limit 0.1 m) |" in text
    failed = result.run_result(checked_run(tmp_path / "b", {**PASSED, "passed": False, "problems": ["the camera gave 6.0 frames a second of sim time (at least 8 needed)"]}))
    assert failed["status"] == "FAIL" and "6.0 frames a second" in failed["reason"]
    absent = result.run_result(checked_run(tmp_path / "c", None))
    assert absent["status"] == "INVALID" and "sensor_check.json" in absent["reason"]
    plain = result.run_result(make_run(tmp_path / "d", score=SCORE, manifest={"variant": "off"}))
    assert plain["status"] == "OK" and "| Sensors |" not in result.render_report(plain)


def test_a_profile_with_a_rate_floor_also_holds_the_run_to_its_real_time_factor(tmp_path, monkeypatch):
    survey = json.loads((build_run(SURVEY, tmp_path / "built") / "scenario.json").read_text())
    assert survey["vehicle"]["min_real_time_factor"] == 0.8 and survey["vehicle"]["min_rate_hz"] == 8.0
    manifest = {"variant": "off", "sensor_check": True, "vehicle": {"min_real_time_factor": 0.8}}
    run = make_run(tmp_path / "a", score=SCORE, manifest=manifest)
    (run / "sensor_check.json").write_text(json.dumps(PASSED))
    assert result.run_result(run)["status"] == "OK"  # the fixture's flight ran at real time
    monkeypatch.setattr(result, "real_time_factor", lambda *_: 0.85)
    assert result.run_result(run)["status"] == "OK"
    monkeypatch.setattr(result, "real_time_factor", lambda *_: 0.70)
    slow = result.run_result(run)
    assert slow["status"] == "FAIL" and "real-time factor was 0.70 (at least 0.8 needed with this camera profile)" in slow["reason"]
    monkeypatch.setattr(result, "real_time_factor", lambda *_: 0.82)
    no_floor = make_run(tmp_path / "b", score=SCORE, manifest={"variant": "off", "sensor_check": True, "vehicle": {"min_real_time_factor": None}})
    (no_floor / "sensor_check.json").write_text(json.dumps(PASSED))
    assert result.run_result(no_floor)["status"] == "OK"
    monkeypatch.setattr(result, "real_time_factor", lambda *_: None)
    unknown = result.run_result(run)
    assert unknown["status"] == "FAIL" and "real-time factor was unknown (at least 0.8 needed with this camera profile)" in unknown["reason"]


# --- MAVROS in the sim ---


def test_the_sims_mavros_plugin_list_is_the_flights_plus_the_four_the_spec_names():
    import yaml

    flight = yaml.safe_load((REPO / "config" / "mavros" / "argus_mavros.yaml").read_text())
    sim = yaml.safe_load((SIM / "config" / "mavros_sim.yaml").read_text())
    assert sim["/**"]["ros__parameters"]["plugin_denylist"] == ["*"]
    extra = set(sim["/**"]["ros__parameters"]["plugin_allowlist"]) - set(flight["/**"]["ros__parameters"]["plugin_allowlist"])
    assert extra == {"optical_flow", "vision_pose", "vision_speed", "distance_sensor"}
    assert set(flight["/**"]["ros__parameters"]["plugin_allowlist"]) <= set(sim["/**"]["ros__parameters"]["plugin_allowlist"])
    assert sim["/**/time"] == flight["/**/time"]  # time sync as the aircraft's
    # Whole-file: the aircraft's settings with the four plugins added give exactly the sim's, so a key added to one and not the other fails here.
    expected = yaml.safe_load((REPO / "config" / "mavros" / "argus_mavros.yaml").read_text())
    expected["/**"]["ros__parameters"]["plugin_allowlist"] += ["optical_flow", "vision_pose", "vision_speed", "distance_sensor"]
    assert sim == expected
    assert "features" not in sim
    start = (SIM / "container" / "start.sh").read_text()
    assert 'config:=$ARGUS_SIM/config/mavros_sim.yaml' in start
