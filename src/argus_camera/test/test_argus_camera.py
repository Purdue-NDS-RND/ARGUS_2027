"""argus_camera's library (PLT-03): sensor modes, per-mode calibration, frame sources, the robot description, the frame log. No ROS."""

import ast
import json
import math
import re
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from argus_camera import bench, calibration, frame_log, frame_source, modes, robot_description
from argus_camera.frame_source import CaptureSource, Frame, FrameSourceError, SyntheticSource, csi_pipeline, open_source

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
CONFIG = PACKAGE / "config"
MODES = modes.load_modes(CONFIG / "modes.yaml")


# --- sensor modes ------------------------------------------------------------------------------------


def test_the_three_modes_in_the_spec_and_the_sim_camera():
    assert {"imx519_4k", "imx519_720p60", "ov9782_flow", "sim_delivery"} <= set(MODES)
    fourk, fast, flow, sim = MODES["imx519_4k"], MODES["imx519_720p60"], MODES["ov9782_flow"], MODES["sim_delivery"]
    assert (fourk.width, fourk.height, fourk.fps, fourk.driver, fourk.camera) == (3840, 2160, 17.0, "csi", "down")
    assert (fast.width, fast.height, fast.fps, fast.driver, fast.camera) == (1280, 720, 60.0, "csi", "down")
    assert (flow.driver, flow.camera, flow.width, flow.height) == ("v4l2", "flow", 1280, 800)
    assert (sim.width, sim.height, sim.fps, sim.driver) == (640, 360, 30.0, "ros_topic")
    assert fourk.sensor_id == fast.sensor_id  # the same sensor, two modes
    # 4K and the flow camera stay in the process that uses them; only 720p and the sim camera become topics.
    assert {name for name, mode in MODES.items() if mode.in_process_only} == {"imx519_4k", "ov9782_flow"}


def test_every_mode_names_a_camera_with_an_optical_frame_and_its_own_calibration_file():
    seen = set()
    for mode in MODES.values():
        assert mode.frame_id.endswith("_optical")
        if mode.calibration is not None:
            assert mode.calibration not in seen, "two modes share a calibration file"
            seen.add(mode.calibration)
            assert (CONFIG / mode.calibration).is_file()


@pytest.mark.parametrize(
    "change, word",
    [
        ({"driver": "firewire"}, "driver"),
        ({"width": 0}, "width"),
        ({"fps": -1}, "fps"),
        ({"camera": "sideways"}, "camera"),
        ({"shutter": 5}, "unknown key"),
    ],
)
def test_a_mode_that_cannot_be_right_is_refused_with_the_reason(tmp_path, change, word):
    data = yaml.safe_load((CONFIG / "modes.yaml").read_text())
    data["modes"]["imx519_720p60"].update(change)
    path = tmp_path / "modes.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(modes.ModeError, match=word):
        modes.load_modes(path)


def test_an_unknown_mode_name_lists_the_ones_there_are():
    with pytest.raises(modes.ModeError, match="imx519_4k"):
        modes.get_mode(MODES, "imx519_8k")


# --- calibration: measured in the mode, never scaled ------------------------------------------------------


def test_the_4k_calibration_is_the_measured_one_from_2026():
    cal = calibration.for_mode(MODES["imx519_4k"], CONFIG)
    old = yaml.safe_load((REPO / "src" / "drone_target_cv" / "config" / "mipi_info.yaml").read_text())
    assert cal.measured and (cal.width, cal.height) == (3840, 2160)
    assert list(cal.k) == pytest.approx(old["camera_matrix"]["data"])
    assert list(cal.d) == pytest.approx(old["distortion_coefficients"]["data"])


def test_the_720p_calibration_says_it_is_not_measured():
    cal = calibration.for_mode(MODES["imx519_720p60"], CONFIG)
    assert not cal.measured and "4K" in cal.basis and (cal.width, cal.height) == (1280, 720)


def test_the_sim_calibration_is_the_sim_cameras_own_pinhole():
    cal = calibration.for_mode(MODES["sim_delivery"], CONFIG)
    sdf = (REPO / "sim" / "models" / "iris_downcam" / "model.sdf").read_text()
    hfov = float(re.search(r"<horizontal_fov>([\d.]+)</horizontal_fov>", sdf).group(1))
    fx = 640.0 / (2.0 * math.tan(hfov / 2.0))
    assert cal.k[0] == pytest.approx(fx, rel=1e-4) and cal.k[4] == pytest.approx(fx, rel=1e-4)
    assert (cal.k[2], cal.k[5]) == (320.0, 180.0) and not any(cal.d)


def test_a_calibration_for_another_resolution_is_refused_not_scaled(tmp_path):
    (tmp_path / "calibration").mkdir()
    (tmp_path / "calibration" / "imx519_720p60.yaml").write_text((CONFIG / "calibration" / "imx519_4k.yaml").read_text())
    with pytest.raises(calibration.CalibrationError, match="3840 x 2160.*1280 x 720"):
        calibration.for_mode(MODES["imx519_720p60"], tmp_path)


def test_a_mode_with_no_calibration_yet_says_so():
    assert MODES["ov9782_flow"].calibration is None
    with pytest.raises(calibration.CalibrationError, match="ov9782_flow has no calibration"):
        calibration.for_mode(MODES["ov9782_flow"], CONFIG)


def test_camera_info_is_the_raw_frames_k_and_d_with_no_rectification():
    cal = calibration.for_mode(MODES["imx519_4k"], CONFIG)
    info = calibration.camera_info_fields(cal)
    assert info["width"] == 3840 and info["height"] == 2160 and info["distortion_model"] == "plumb_bob"
    assert info["k"] == list(cal.k) and info["d"] == list(cal.d)
    assert info["r"] == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    # P is K with a zero fourth column: the frame is published as the sensor gave it, not remapped.
    assert info["p"] == [cal.k[0], cal.k[1], cal.k[2], 0.0, cal.k[3], cal.k[4], cal.k[5], 0.0, cal.k[6], cal.k[7], cal.k[8], 0.0]
    assert info["p"][0] == cal.k[0] != 1906.98572  # not the 2026 file's new-camera-matrix P, which did not match its K and D


def test_a_broken_calibration_file_is_refused(tmp_path):
    (tmp_path / "calibration").mkdir()
    for text, word in ((("k: [1, 2, 3]\nwidth: 3840\nheight: 2160\nd: []\nmeasured: true\n"), "nine"), ("width: 3840\n", "missing")):
        (tmp_path / "calibration" / "imx519_4k.yaml").write_text(text)
        with pytest.raises(calibration.CalibrationError, match=word):
            calibration.for_mode(MODES["imx519_4k"], tmp_path)


# --- frame sources ------------------------------------------------------------------------------------------


class FakeClock:
    def __init__(self, start_ns=1_000_000_000):
        self.now = start_ns

    def __call__(self):
        self.now += 1_000_000
        return self.now


def test_synthetic_source_gives_frames_of_the_modes_size_with_rising_sequence_and_stamps():
    source = SyntheticSource(MODES["imx519_720p60"], clock=FakeClock())
    with source:
        frames = [source.read() for _ in range(3)]
    assert all(isinstance(frame, Frame) for frame in frames)
    assert frames[0].image.shape == (720, 1280, 3) and frames[0].image.dtype == np.uint8
    assert [frame.seq for frame in frames] == [0, 1, 2]
    assert frames[0].capture_ns < frames[1].capture_ns < frames[2].capture_ns
    assert frames[0].mode == "imx519_720p60"
    assert not np.array_equal(frames[0].image, frames[1].image)  # the pattern moves
    assert source.read() is None  # closed


class FakeCapture:
    """What CaptureSource uses of cv2.VideoCapture."""

    def __init__(self, frames):
        self.frames = list(frames)
        self.released = False
        self.opened = True

    def isOpened(self):
        return self.opened

    def read(self):
        if not self.frames:
            return False, None
        return True, self.frames.pop(0)

    def release(self):
        self.released = True


def test_capture_source_passes_frames_through_untouched_and_stamps_them():
    raw = np.arange(720 * 1280 * 3, dtype=np.uint8).reshape(720, 1280, 3)
    capture = FakeCapture([raw.copy(), raw.copy()])
    source = CaptureSource(MODES["imx519_720p60"], opener=lambda mode: capture, clock=FakeClock())
    source.open()
    frame = source.read()
    assert np.array_equal(frame.image, raw) and frame.seq == 0 and frame.capture_ns > 1_000_000_000
    assert source.read().seq == 1
    assert source.read() is None  # the camera gave nothing: no frame, not an old one
    source.close()
    assert capture.released


def test_capture_source_refuses_a_frame_of_the_wrong_size_instead_of_resizing_it():
    capture = FakeCapture([np.zeros((1080, 1920, 3), dtype=np.uint8)])
    source = CaptureSource(MODES["imx519_720p60"], opener=lambda mode: capture, clock=FakeClock())
    source.open()
    with pytest.raises(FrameSourceError, match="1920 x 1080.*1280 x 720"):
        source.read()


def test_capture_source_that_does_not_open_says_which_camera():
    capture = FakeCapture([])
    capture.opened = False
    with pytest.raises(FrameSourceError, match="imx519_4k"):
        CaptureSource(MODES["imx519_4k"], opener=lambda mode: capture).open()


def test_csi_pipeline_asks_the_sensor_for_the_mode_and_does_no_scaling():
    pipeline = csi_pipeline(MODES["imx519_4k"])
    assert "nvarguscamerasrc sensor-id=0" in pipeline
    assert "width=3840, height=2160" in pipeline and "framerate=17/1" in pipeline
    assert pipeline.endswith("appsink drop=1 max-buffers=1")
    assert pipeline.count("width=") == 1  # one size only: nothing downstream resizes
    fast = csi_pipeline(MODES["imx519_720p60"])
    assert "width=1280, height=720" in fast and "framerate=60/1" in fast


def test_open_source_picks_the_driver_and_the_node_owns_ros_topics():
    assert isinstance(open_source(MODES["imx519_4k"], opener=lambda mode: FakeCapture([])), CaptureSource)
    assert isinstance(open_source(MODES["ov9782_flow"], opener=lambda mode: FakeCapture([])), CaptureSource)
    assert isinstance(open_source(MODES["imx519_4k"], synthetic=True), SyntheticSource)
    with pytest.raises(FrameSourceError, match="node"):
        open_source(MODES["sim_delivery"])


# --- the one robot description ----------------------------------------------------------------------------------


def test_robot_description_has_the_aircraft_and_the_sim_vehicle():
    robot = robot_description.load(CONFIG / "robot.yaml")
    assert {"argus_2027", "sim_iris_downcam"} <= set(robot)
    aircraft = robot["argus_2027"]
    assert {"down", "flow"} <= set(aircraft.cameras)
    down = aircraft.cameras["down"]
    assert down.optical_frame == MODES["imx519_4k"].frame_id
    assert len(down.position_m) == 3 and len(aircraft.payload_offset_m) == 3
    assert not aircraft.verified  # nobody has measured it on the airframe yet


def test_camera_frames_in_the_description_match_the_modes():
    robot = robot_description.load(CONFIG / "robot.yaml")
    frames = {camera.optical_frame for airframe in robot.values() for camera in airframe.cameras.values()}
    assert {mode.frame_id for mode in MODES.values()} <= frames


def test_a_camera_looking_straight_down_with_its_image_top_forward():
    robot = robot_description.load(CONFIG / "robot.yaml")
    rotation = robot["sim_iris_downcam"].cameras["down"].rotation_body_from_optical()
    # Optical frame: x right in the image, y down in the image, z out of the lens. Body: x forward, y left, z up.
    assert rotation @ np.array([0.0, 0.0, 1.0]) == pytest.approx([0.0, 0.0, -1.0])  # the lens looks down
    assert rotation @ np.array([0.0, -1.0, 0.0]) == pytest.approx([1.0, 0.0, 0.0])  # image top is the nose
    assert rotation @ np.array([1.0, 0.0, 0.0]) == pytest.approx([0.0, -1.0, 0.0])  # image right is the right side
    assert np.linalg.det(rotation) == pytest.approx(1.0)


def test_image_yaw_turns_the_image_about_the_lens_axis():
    camera = robot_description.CameraMount("down", "x_optical", (0.0, 0.0, 0.0), image_yaw_deg=180.0)
    rotation = camera.rotation_body_from_optical()
    assert rotation @ np.array([0.0, 0.0, 1.0]) == pytest.approx([0.0, 0.0, -1.0])
    assert rotation @ np.array([0.0, -1.0, 0.0]) == pytest.approx([-1.0, 0.0, 0.0])  # image top is the tail


def test_the_sim_vehicles_camera_is_where_the_sim_model_puts_it():
    robot = robot_description.load(CONFIG / "robot.yaml")
    sdf = (REPO / "sim" / "models" / "iris_downcam" / "model.sdf").read_text()
    sensor = sdf[sdf.index('<sensor name="downcam"') :]
    pose = [float(value) for value in re.search(r"<pose>([^<]+)</pose>", sensor).group(1).split()]
    camera = robot["sim_iris_downcam"].cameras["down"]
    assert list(camera.position_m) == pytest.approx(pose[:3])
    assert pose[3:] == pytest.approx([0.0, math.pi / 2.0, 0.0], abs=1e-6)  # pitched straight down, as the description says
    assert robot["sim_iris_downcam"].verified


def test_a_description_with_a_missing_or_extra_key_is_refused(tmp_path):
    data = yaml.safe_load((CONFIG / "robot.yaml").read_text())
    del data["airframes"]["argus_2027"]["cameras"]["down"]["position_m"]
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(robot_description.RobotDescriptionError, match="position_m"):
        robot_description.load(path)


def test_nothing_else_in_src_describes_a_camera_mount_for_2027_code():
    # The 2026 copies stay for the 2026 path (mission_logger's mount_*, the sequences' camera_yaw_offset_deg).
    # No new one may appear: 2027 code reads argus_camera's robot.yaml.
    allowed = {"src/drone_target_cv/config/mipi_params.yaml", "src/vision_pipeline/config/vision_params.yaml"}
    found = set()
    for path in (REPO / "src").rglob("*.yaml"):
        text = path.read_text()
        if re.search(r"^\s*mount_(x|y|z|roll|pitch|yaw)\s*:", text, re.M):
            found.add(path.relative_to(REPO).as_posix())
    assert found <= allowed, found - allowed


# --- frame log core -------------------------------------------------------------------------------------------


def frame(seq, capture_ns, value=0):
    image = np.full((36, 64, 3), value, dtype=np.uint8)
    cv2.putText(image, str(seq), (2, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    return Frame(image=image, seq=seq, capture_ns=capture_ns, mode="sim_delivery")


def test_frame_log_writes_each_frame_losslessly_with_an_index(tmp_path):
    log = frame_log.FrameLog(tmp_path / "frames")
    first = frame(0, 1_000_000_000, 10)
    assert log.write(first) and log.write(frame(1, 1_033_000_000, 20))
    log.close()
    lines = [json.loads(line) for line in (tmp_path / "frames" / "index.jsonl").read_text().splitlines()]
    assert [line["seq"] for line in lines] == [0, 1]
    assert lines[0] == {"seq": 0, "capture_ns": 1_000_000_000, "mode": "sim_delivery", "file": "frame_000000.png", "width": 64, "height": 36}
    stored = cv2.imread(str(tmp_path / "frames" / "frame_000000.png"), cv2.IMREAD_COLOR)
    assert np.array_equal(stored, first.image)


def test_frame_log_can_thin_by_time_and_counts_what_it_skipped(tmp_path):
    log = frame_log.FrameLog(tmp_path / "frames", min_interval_s=0.1)
    written = [log.write(frame(seq, 1_000_000_000 + seq * 33_000_000)) for seq in range(10)]  # 30 Hz for 0.3 s
    log.close()
    assert written == [True, False, False, False, True, False, False, False, True, False]  # 0, 132 and 264 ms
    assert log.written == 3 and log.skipped == 7
    assert len(list((tmp_path / "frames").glob("frame_*.png"))) == 3


def test_frame_log_stops_at_its_size_limit_and_says_so(tmp_path):
    log = frame_log.FrameLog(tmp_path / "frames", max_bytes=1500)
    results = [log.write(frame(seq, seq * 1_000_000_000, seq)) for seq in range(20)]
    log.close()
    assert results[0] and not results[-1] and log.full
    total = sum(path.stat().st_size for path in (tmp_path / "frames").glob("frame_*.png"))
    assert total <= 1500 + 2000  # at most one frame past the limit


def test_frame_log_refuses_a_folder_that_already_has_a_log(tmp_path):
    frame_log.FrameLog(tmp_path / "frames").close()
    with pytest.raises(frame_log.FrameLogError):
        frame_log.FrameLog(tmp_path / "frames")


# --- the bench check -----------------------------------------------------------------------------------------------


def stamps_at(fps, count, start=1_000_000_000):
    return [start + int(round(index * 1.0e9 / fps)) for index in range(count)]


def test_bench_passes_a_camera_at_its_rate_with_a_measured_calibration():
    report = bench.judge(MODES["imx519_4k"], stamps_at(17.0, 171), True)
    assert report["passed"] and report["fps"] == pytest.approx(17.0, abs=0.01) and report["frames"] == 171


@pytest.mark.parametrize(
    "stamps, measured, word",
    [
        (stamps_at(40.0, 400), True, "under 90%"),  # the 60 fps mode at 40
        (stamps_at(60.0, 300) + stamps_at(60.0, 300, start=7_000_000_000), True, "gap"),  # a 1 s stall
        (stamps_at(60.0, 600), False, "not measured"),
        (stamps_at(60.0, 600), None, "no calibration file"),
        ([1_000_000_000], True, "only 1 frame"),
    ],
)
def test_bench_fails_and_says_why(stamps, measured, word):
    report = bench.judge(MODES["imx519_720p60"], stamps, measured)
    assert not report["passed"] and any(word in problem for problem in report["problems"])


def test_bench_measures_a_source_through_the_same_interface_the_node_uses():
    clock = FakeClock()
    clock_step = int(1.0e9 / 60.0)

    def tick():
        clock.now += clock_step
        return clock.now

    source = SyntheticSource(MODES["imx519_720p60"], clock=tick)
    with source:
        stamps = bench.measure(source, 2.0)
    assert 120 <= len(stamps) <= 122
    assert bench.judge(MODES["imx519_720p60"], stamps, True)["passed"]


def test_bench_gives_up_on_a_camera_that_stops():
    source = CaptureSource(MODES["imx519_720p60"], opener=lambda mode: FakeCapture([np.zeros((720, 1280, 3), dtype=np.uint8)] * 3), clock=FakeClock())
    source.open()
    stamps = bench.measure(source, 10.0, sleep=lambda _seconds: None)
    assert len(stamps) == 3


def test_bench_command_runs_on_the_test_pattern(capsys):
    assert bench.main(["--mode", "imx519_720p60", "--seconds", "0.5", "--synthetic"]) == 1  # 720p60 is not measured yet
    out = capsys.readouterr().out
    assert "imx519_720p60: 1280 x 720" in out and "calibration is not measured" in out and "camera bench: FAIL" in out


# --- the package ------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("name", ["modes", "calibration", "frame_source", "robot_description", "frame_log", "bench"])
def test_the_library_has_no_ros_in_it(name):
    tree = ast.parse((PACKAGE / "argus_camera" / f"{name}.py").read_text())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {alias.name.split(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            imported.add((node.module or "").split(".")[0])
    assert imported <= {"__future__", "abc", "argparse", "dataclasses", "json", "math", "pathlib", "sys", "time", "typing", "numpy", "cv2", "yaml"}


def test_the_node_is_a_thin_wrapper_that_keeps_the_legacy_topic_names():
    source = (PACKAGE / "argus_camera" / "camera_node.py").read_text()
    assert '"/camera/image"' in source and '"/camera/camera_info"' in source  # what target_cv and the 2026 path read
    assert "open_source(" in source and "camera_info_fields(" in source
    assert "resize" not in source and "undistort" not in source and "remap" not in source  # raw frames
    assert "if self._mode.in_process_only:\n            raise RuntimeError" in source  # no 4K over topics
    assert 'message.data = array.array("B", frame.image.tobytes())' in source  # plain bytes cost 0.3 s a frame in rclpy
    assert "get_clock().now()" not in source.split("def _publish")[1].split("def ")[0]  # a frame is stamped with its capture time
    setup = (PACKAGE / "setup.py").read_text()
    assert "'camera_node = argus_camera.camera_node:main'" in setup
    assert len(source.splitlines()) < 170
