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

from argus_camera import bench, calibration, frame_log, frame_source, modes, publishing, robot_description
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
        ({"exposure_time_s": 30}, "exposure_time_s"),
        ({"wb_mode": "sunny"}, "wb_mode"),
    ],
)
def test_a_mode_that_cannot_be_right_is_refused_with_the_reason(tmp_path, change, word):
    data = yaml.safe_load((CONFIG / "modes.yaml").read_text())
    data["modes"]["imx519_720p60"].update(change)
    path = tmp_path / "modes.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(modes.ModeError, match=word):
        modes.load_modes(path)


def test_the_imx519_modes_keep_the_exposure_and_white_balance_that_flew_in_2026():
    live = yaml.safe_load((REPO / "src" / "drone_mission_demo" / "config" / "params" / "live_target_mission.yaml").read_text())
    flown = live["mipi_grabber"]["ros__parameters"]
    for name in ("imx519_4k", "imx519_720p60"):
        assert MODES[name].exposure_time_s == pytest.approx(1.0 / flown["shutter_speed"])
        assert MODES[name].wb_mode == flown["wb_mode"]
    assert MODES["sim_delivery"].exposure_time_s is None and MODES["sim_delivery"].wb_mode is None


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
    source = CaptureSource(MODES["imx519_720p60"], FakeClock(), opener=lambda mode: capture)
    source.open()
    frame = source.read()
    assert np.array_equal(frame.image, raw) and frame.seq == 0 and frame.capture_ns > 1_000_000_000
    assert source.read().seq == 1
    assert source.read() is None  # the camera gave nothing: no frame, not an old one
    source.close()
    assert capture.released


def test_capture_source_refuses_a_frame_of_the_wrong_size_instead_of_resizing_it():
    capture = FakeCapture([np.zeros((1080, 1920, 3), dtype=np.uint8)])
    source = CaptureSource(MODES["imx519_720p60"], FakeClock(), opener=lambda mode: capture)
    source.open()
    with pytest.raises(FrameSourceError, match="1920 x 1080.*1280 x 720"):
        source.read()


def test_capture_source_that_does_not_open_says_which_camera():
    capture = FakeCapture([])
    capture.opened = False
    with pytest.raises(FrameSourceError, match="imx519_4k"):
        CaptureSource(MODES["imx519_4k"], FakeClock(), opener=lambda mode: capture).open()


def test_csi_pipeline_asks_the_sensor_for_the_mode_and_does_no_scaling():
    pipeline = csi_pipeline(MODES["imx519_4k"])
    assert "nvarguscamerasrc sensor-id=0" in pipeline
    assert "width=3840, height=2160" in pipeline and "framerate=17/1" in pipeline
    assert pipeline.endswith("appsink drop=1 max-buffers=1")
    assert pipeline.count("width=") == 1  # one size only: nothing downstream resizes
    assert "exposuretimerange='1000000 1000000' wbmode=6 !" in pipeline  # 1/1000 s and the 2026 white balance, in the source element
    auto = csi_pipeline(modes.SensorMode("x", "down", "down_camera_optical", "csi", 1280, 720, 60.0, None))
    assert "exposuretimerange" not in auto and "wbmode" not in auto  # a mode that sets neither leaves the sensor on auto
    fast = csi_pipeline(MODES["imx519_720p60"])
    assert "width=1280, height=720" in fast and "framerate=60/1" in fast


def test_open_source_picks_the_driver_and_the_node_owns_ros_topics():
    clock = FakeClock()
    assert isinstance(open_source(MODES["imx519_4k"], clock=clock, opener=lambda mode: FakeCapture([])), CaptureSource)
    assert isinstance(open_source(MODES["ov9782_flow"], clock=clock, opener=lambda mode: FakeCapture([])), CaptureSource)
    assert isinstance(open_source(MODES["imx519_4k"], clock=clock, synthetic=True), SyntheticSource)
    with pytest.raises(FrameSourceError, match="node"):
        open_source(MODES["sim_delivery"], clock=clock)


def test_a_source_has_no_default_clock():
    # Wall time by default would put frames on a different clock from the poses under sim time.
    with pytest.raises(TypeError):
        open_source(MODES["imx519_4k"])
    with pytest.raises(TypeError):
        SyntheticSource(MODES["imx519_4k"])


def test_the_usb_camera_is_opened_through_v4l2_at_the_modes_size_rate_and_format(monkeypatch):
    calls = {}

    class Recorder:
        def __init__(self, device, backend):
            calls["open"] = (device, backend)
            calls["set"] = {}

        def set(self, prop, value):
            calls["set"][prop] = value

        def isOpened(self):
            return True

    monkeypatch.setattr(frame_source.cv2, "VideoCapture", Recorder)
    source = open_source(MODES["ov9782_flow"], clock=FakeClock())
    source.open()
    assert calls["open"] == ("/dev/video0", cv2.CAP_V4L2)
    assert calls["set"][cv2.CAP_PROP_FRAME_WIDTH] == 1280 and calls["set"][cv2.CAP_PROP_FRAME_HEIGHT] == 800
    assert calls["set"][cv2.CAP_PROP_FPS] == 120.0 and calls["set"][cv2.CAP_PROP_FOURCC] == cv2.VideoWriter_fourcc(*"MJPG")


def test_the_csi_camera_is_opened_with_its_pipeline_through_gstreamer(monkeypatch):
    calls = {}

    class Recorder:
        def __init__(self, pipeline, backend):
            calls["open"] = (pipeline, backend)

        def isOpened(self):
            return True

    monkeypatch.setattr(frame_source.cv2, "VideoCapture", Recorder)
    open_source(MODES["imx519_720p60"], clock=FakeClock()).open()  # the default backend is OpenCV's capture
    assert calls["open"] == (csi_pipeline(MODES["imx519_720p60"]), cv2.CAP_GSTREAMER)


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


def test_an_unverified_airframe_is_only_given_to_a_caller_that_asks_for_it():
    robot = robot_description.load(CONFIG / "robot.yaml")
    assert robot_description.get_airframe(robot, "sim_iris_downcam").verified
    with pytest.raises(robot_description.RobotDescriptionError, match="not verified"):
        robot_description.get_airframe(robot, "argus_2027")
    assert robot_description.get_airframe(robot, "argus_2027", allow_unverified=True).name == "argus_2027"
    with pytest.raises(robot_description.RobotDescriptionError, match="sim_iris_downcam"):
        robot_description.get_airframe(robot, "argus_2028")


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


def test_frame_log_writes_jpeg_at_quality_90_with_an_index(tmp_path):
    log = frame_log.FrameLog(tmp_path / "frames")
    first = frame(0, 1_000_000_000, 10)
    assert log.write(first) and log.write(frame(1, 1_033_000_000, 20))
    log.close()
    lines = [json.loads(line) for line in (tmp_path / "frames" / "index.jsonl").read_text().splitlines()]
    assert [line["seq"] for line in lines] == [0, 1]
    assert lines[0] == {"seq": 0, "capture_ns": 1_000_000_000, "mode": "sim_delivery", "file": "frame_000000.jpg", "width": 64, "height": 36}
    stored = cv2.imread(str(tmp_path / "frames" / "frame_000000.jpg"), cv2.IMREAD_COLOR)
    assert stored.shape == first.image.shape and np.abs(stored.astype(int) - first.image.astype(int)).mean() < 6.0
    assert log._encode_args == [int(cv2.IMWRITE_JPEG_QUALITY), 90]  # LOC-11: JPEG quality 90


def test_frame_log_can_keep_every_pixel_as_png(tmp_path):
    log = frame_log.FrameLog(tmp_path / "frames", image_format="png")
    first = frame(0, 1_000_000_000, 10)
    log.write(first)
    log.close()
    assert np.array_equal(cv2.imread(str(tmp_path / "frames" / "frame_000000.png"), cv2.IMREAD_COLOR), first.image)
    with pytest.raises(frame_log.FrameLogError, match="image_format"):
        frame_log.FrameLog(tmp_path / "other", image_format="bmp")


def test_frame_log_can_thin_by_time_and_counts_what_it_skipped(tmp_path):
    log = frame_log.FrameLog(tmp_path / "frames", min_interval_s=0.1)
    written = [log.write(frame(seq, 1_000_000_000 + seq * 33_000_000)) for seq in range(10)]  # 30 Hz for 0.3 s
    log.close()
    assert written == [True, False, False, False, True, False, False, False, True, False]  # 0, 132 and 264 ms
    assert log.written == 3 and log.skipped == 7
    assert len(list((tmp_path / "frames").glob("frame_*.jpg"))) == 3


def test_frame_log_stops_at_its_size_limit_and_says_so(tmp_path):
    log = frame_log.FrameLog(tmp_path / "frames", image_format="png", max_bytes=1500)
    results = [log.write(frame(seq, seq * 1_000_000_000, seq)) for seq in range(20)]
    log.close()
    assert results[0] and not results[-1] and log.full
    total = sum(path.stat().st_size for path in (tmp_path / "frames").glob("frame_*.png"))
    assert total <= 1500 + 2000  # at most one frame past the limit


def test_frame_log_refuses_a_folder_that_already_has_a_log(tmp_path):
    frame_log.FrameLog(tmp_path / "frames").close()
    with pytest.raises(frame_log.FrameLogError):
        frame_log.FrameLog(tmp_path / "frames")


def test_frame_log_on_worker_threads_writes_everything_it_took(tmp_path):
    log = frame_log.FrameLog(tmp_path / "frames", workers=2, queue_frames=64)
    taken = [log.write(frame(seq, seq * 33_000_000, seq)) for seq in range(40)]
    log.close()  # waits for the queue
    assert all(taken) and log.written == 40 and log.dropped == 0 and log.error is None
    lines = [json.loads(line) for line in (tmp_path / "frames" / "index.jsonl").read_text().splitlines()]
    assert sorted(line["seq"] for line in lines) == list(range(40))
    assert len(list((tmp_path / "frames").glob("frame_*.jpg"))) == 40


def test_frame_log_drops_frames_instead_of_blocking_when_its_workers_fall_behind(tmp_path, monkeypatch):
    import threading

    gate = threading.Event()
    real_store = frame_log.FrameLog._store

    def slow_store(self, item):
        gate.wait(5.0)  # the disk is slow
        real_store(self, item)

    monkeypatch.setattr(frame_log.FrameLog, "_store", slow_store)
    log = frame_log.FrameLog(tmp_path / "frames", workers=1, queue_frames=2)
    taken = [log.write(frame(seq, seq * 33_000_000)) for seq in range(10)]  # returns at once every time
    assert taken.count(True) in (2, 3) and log.dropped == 10 - taken.count(True)
    gate.set()
    log.close()
    assert log.written == taken.count(True)


def test_frame_log_worker_that_cannot_write_stops_the_log_and_keeps_the_reason(tmp_path, monkeypatch):
    monkeypatch.setattr(frame_log.cv2, "imwrite", lambda *args: False)
    log = frame_log.FrameLog(tmp_path / "frames", workers=1)
    log.write(frame(0, 0))
    log.close()
    assert log.full and "could not write" in log.error and log.written == 0
    assert not log.write(frame(1, 1_000_000_000))


# --- what the node publishes where ------------------------------------------------------------------------------


def test_the_nodes_own_topics_and_the_three_2026_names():
    own = publishing.topic_names(MODES["imx519_720p60"], legacy=False)
    assert own["image"] == ["/argus/camera/down/image_raw"] and own["info"] == ["/argus/camera/down/camera_info"]
    assert own["legacy_image"] == own["legacy_compressed"] == own["legacy_info"] == []
    legacy = publishing.topic_names(MODES["imx519_720p60"], legacy=True)
    assert (legacy["legacy_image"], legacy["legacy_compressed"], legacy["legacy_info"]) == (["/camera/image"], ["/camera/image/compressed"], ["/camera/camera_info"])
    # Every topic the 2026 CSI grabber publishes is among them.
    grabber = (REPO / "src" / "drone_target_cv" / "drone_target_cv" / "mipi_grabber.py").read_text()
    published = set(re.findall(r'create_publisher\(\s*\w+,\s*"([^"]+)"', grabber))
    assert published == {"/camera/image", "/camera/image/compressed", "/camera/camera_info"}
    assert published == set(legacy["legacy_image"] + legacy["legacy_compressed"] + legacy["legacy_info"])


def test_the_2026_names_are_served_at_the_2026_rate_and_jpeg_quality():
    live = yaml.safe_load((REPO / "src" / "drone_mission_demo" / "config" / "params" / "live_target_mission.yaml").read_text())
    flown = live["mipi_grabber"]["ros__parameters"]
    assert publishing.LEGACY_RATE_HZ == flown["image_publishing_rate"] == 30.0
    assert publishing.LEGACY_JPEG_QUALITY == flown["compressed_quality"]


def test_a_relay_may_not_take_its_input_from_a_topic_it_publishes():
    legacy = publishing.topic_names(MODES["sim_delivery"], legacy=True)
    own = publishing.topic_names(MODES["sim_delivery"], legacy=False)
    assert publishing.check_input_topic("/camera/image", own) == "/camera/image"
    assert publishing.check_input_topic("camera/image/", own) == "/camera/image"
    for topic in ("/camera/image", "camera/image", "/argus/camera/down/image_raw"):
        with pytest.raises(publishing.TopicError, match="its own output"):
            publishing.check_input_topic(topic, legacy)


def test_another_publisher_on_a_2026_name_is_noticed():
    legacy = publishing.topic_names(MODES["imx519_720p60"], legacy=True)
    alone = {"/camera/image": 1, "/camera/image/compressed": 1, "/camera/camera_info": 1, "/argus/camera/down/image_raw": 1}
    assert publishing.other_publishers(alone, legacy) == []
    assert publishing.other_publishers({**alone, "/camera/image": 2, "/camera/camera_info": 2}, legacy) == ["/camera/camera_info", "/camera/image"]
    own = publishing.topic_names(MODES["imx519_720p60"], legacy=False)
    assert publishing.other_publishers({"/camera/image": 1}, own) == []  # not our topic when the aliases are off


def test_rate_limiter_thins_60_fps_to_30_by_capture_time():
    limiter = publishing.RateLimiter(30.0)
    stamps = [1_000_000_000 + int(round(index * 1.0e9 / 60.0)) for index in range(120)]
    passed = [stamp for stamp in stamps if limiter.allow(stamp)]
    assert len(passed) == 60 and passed[:3] == [stamps[0], stamps[2], stamps[4]]
    slow = publishing.RateLimiter(30.0)
    assert all(slow.allow(1_000_000_000 + index * 50_000_000) for index in range(20))  # a 20 fps source loses nothing
    for source_fps in (25, 33, 40, 45, 50, 55, 60, 90):  # a loop that cannot hold its rate never feeds the 2026 names faster
        limiter = publishing.RateLimiter(30.0)
        stamps = [int(round(index * 1.0e9 / source_fps)) for index in range(source_fps * 10)]
        assert sum(limiter.allow(stamp) for stamp in stamps) / 10.0 <= 30.0 * 1.12, source_fps
    back = publishing.RateLimiter(30.0)
    assert back.allow(5_000_000_000) and back.allow(1_000_000_000)  # the clock was reset: do not wait for it to catch up


def test_jpeg_encoding_and_reading_an_image_message_back():
    image = np.zeros((36, 64, 3), dtype=np.uint8)
    image[:, :32] = (0, 0, 200)  # red on the left, in BGR
    jpeg = publishing.encode_jpeg(image, 20)
    decoded = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == image.shape and decoded[18, 8, 2] > 150 and decoded[18, 56, 2] < 50
    rgb = image[:, :, ::-1].tobytes()
    assert np.array_equal(publishing.image_from_message(rgb, 36, 64, "rgb8"), image)
    assert np.array_equal(publishing.image_from_message(image.tobytes(), 36, 64, "bgr8"), image)
    assert publishing.image_from_message(image.tobytes(), 36, 64, "yuv422") is None
    assert publishing.image_from_message(image.tobytes()[:-3], 36, 64, "bgr8") is None


def test_pacer_holds_the_rate_whatever_the_work_between_frames_takes():
    now = {"t": 100.0}
    slept = []

    def sleep(seconds):
        slept.append(seconds)
        now["t"] += seconds

    pacer = publishing.Pacer(60.0, clock=lambda: now["t"], sleep=sleep)
    for _ in range(120):
        now["t"] += 0.004  # 4 ms of work per frame
        pacer.wait()
    assert now["t"] - 100.0 == pytest.approx(120 / 60.0 + 0.004, abs=0.001)  # 60 a second, not 60 minus the work
    assert all(seconds <= 1 / 60.0 + 1e-9 for seconds in slept)
    now["t"] += 3.0  # a long stall
    pacer.wait()
    assert slept[-1] == pytest.approx(1 / 60.0)  # it starts again from now instead of bursting to catch up


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

    source = SyntheticSource(MODES["imx519_720p60"], tick)
    with source:
        stamps = bench.measure(source, 2.0)
    assert 120 <= len(stamps) <= 122
    assert bench.judge(MODES["imx519_720p60"], stamps, True)["passed"]


def test_bench_records_how_old_each_frame_was_when_it_was_read():
    now = {"t": 5_000_000_000}

    def clock():
        now["t"] += 4_000_000
        return now["t"]

    source = SyntheticSource(MODES["imx519_720p60"], clock)
    ages = []
    with source:
        bench.measure(source, 0.2, now=clock, ages_ns=ages)
    assert ages and all(age == 4_000_000 for age in ages)  # read one clock tick after it was stamped


def test_bench_gives_up_on_a_camera_that_stops():
    source = CaptureSource(MODES["imx519_720p60"], FakeClock(), opener=lambda mode: FakeCapture([np.zeros((720, 1280, 3), dtype=np.uint8)] * 3))
    source.open()
    stamps = bench.measure(source, 10.0, sleep=lambda _seconds: None)
    assert len(stamps) == 3


def test_bench_finds_its_config_in_the_source_tree_and_in_an_installed_package(tmp_path):
    assert bench.default_config_dir() == CONFIG
    # A plain `colcon build` (no symlinks): the module is under site-packages and the config under share.
    module = tmp_path / "install" / "argus_camera" / "lib" / "python3.10" / "site-packages" / "argus_camera" / "bench.py"
    module.parent.mkdir(parents=True)
    module.write_text("")
    share = tmp_path / "install" / "argus_camera" / "share" / "argus_camera" / "config"
    share.mkdir(parents=True)
    (share / "modes.yaml").write_text((CONFIG / "modes.yaml").read_text())
    assert bench.default_config_dir(module) == share
    setup = (PACKAGE / "setup.py").read_text()
    assert "('share/' + package_name + '/config', ['config/modes.yaml', 'config/robot.yaml'])" in setup  # what puts it there


def test_bench_command_runs_on_the_test_pattern(capsys):
    assert bench.main(["--mode", "imx519_720p60", "--seconds", "0.5", "--synthetic"]) == 1  # 720p60 is not measured yet
    out = capsys.readouterr().out
    assert "imx519_720p60: 1280 x 720" in out and "calibration is not measured" in out and "camera bench: FAIL" in out
    assert "source SyntheticSource; frames stamped at the read because their own time was unusable: 0; dropped as over a second old: 0" in out


# --- the package ------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("name", ["modes", "calibration", "frame_source", "robot_description", "frame_log", "bench", "publishing", "capture_clock"])
def test_the_library_has_no_ros_in_it(name):
    tree = ast.parse((PACKAGE / "argus_camera" / f"{name}.py").read_text())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {alias.name.split(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            imported.add((node.module or "").split(".")[0])
    allowed = {"__future__", "abc", "argparse", "dataclasses", "json", "math", "os", "pathlib", "queue", "sys", "threading", "time", "typing", "numpy", "cv2", "yaml"}
    if name == "frame_source":
        allowed.add("gi")  # GStreamer's bindings, imported inside load_gst and only if present
    assert imported <= allowed


def test_the_node_is_a_thin_wrapper_around_the_library():
    source = (PACKAGE / "argus_camera" / "camera_node.py").read_text()
    assert "open_source(self._mode, clock=self._now_ns" in source and "camera_info_fields(" in source
    assert "publishing.topic_names(" in source and "publishing.check_input_topic(" in source and "publishing.other_publishers(" in source
    assert "resize" not in source and "undistort" not in source and "remap" not in source  # raw frames
    assert "if self._mode.in_process_only:\n            raise RuntimeError" in source  # no 4K over topics
    assert 'message.data = array.array("B", frame.image.tobytes())' in source  # plain bytes cost 0.3 s a frame in rclpy
    assert "get_clock().now()" not in source.split("def _publish(")[1]  # a frame is stamped with its capture time
    assert 'self.declare_parameter("legacy_topics", False)' in source  # the 2026 names are asked for, never a default
    watch = source.split("def _watch")[1].split("    def ")[0]
    assert "raise SystemExit(1)" in watch  # a failure stops the node
    assert "publishing.STARTUP_GUARD_S" in watch and "throttle_duration_sec" in watch  # a late second publisher is reported, not fatal
    assert "TEST PATTERN" in source and "no frame from the camera in the last 5 s" in source
    setup = (PACKAGE / "setup.py").read_text()
    assert "'camera_node = argus_camera.camera_node:main'" in setup
    assert "capture time from {self._stamp_source}" in source and "capture_clock.fallbacks" in source  # the log says where stamps come from
    assert 'self.declare_parameter("csi_backend", "opencv")' in source and "csi_backend=str(self.get_parameter" in source
    assert len(source.splitlines()) < 225


def test_the_launch_file_leaves_the_2026_names_off_and_reads_the_sims_bridge():
    tree = ast.parse((PACKAGE / "launch" / "argus_camera.launch.py").read_text())
    arguments = next(node.value for node in ast.walk(tree) if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "arguments")
    defaults = {key.value: value.elts[0].value for key, value in zip(arguments.keys, arguments.values)}
    assert defaults["legacy_topics"] == "false" and defaults["synthetic"] == "false" and defaults["use_sim_time"] == "false"
    bridge = yaml.safe_load((REPO / "sim" / "config" / "bridge.yaml").read_text())
    assert defaults["input_topic"] in {entry["ros_topic_name"] for entry in bridge}  # a topic something publishes


def test_nothing_outside_the_package_starts_the_node():
    # No flag guards argus_camera: what keeps the 2026 path as it was is that nothing shipped starts it.
    import subprocess

    def mentions(pattern):
        return subprocess.run(
            ["git", "grep", "-l", "-E", "-e", pattern, "--", "*.py", "*.sh", "*.yaml", "*.yml", "*.xml", "*.launch", "*.service", ":!src/argus_camera/*"],
            cwd=REPO, capture_output=True, text=True,
        ).stdout.split()  # fmt: skip

    assert "scripts/check.sh" in mentions("argus_camera")  # the search runs and finds a mention there is
    # Other packages read its config (projection v2 reads the calibration and the mount). None starts its node:
    # no launch file includes its launch, names its package for a Node, or runs its executables.
    starters = mentions(r"argus_camera\.launch|package=.argus_camera.|ros2 (run|launch) argus_camera|camera_node|camera_bench")
    # One thing may: the boot service's script (PLT-02), which starts it only when /etc/argus/boot.env names a
    # sensor mode. It ships "off" (platform/jetson/test/test_services.py holds it to that), so the 2026 start-up
    # still opens the camera with its own grabber.
    assert set(starters) <= {"platform/jetson/boot/argus-boot.sh", "platform/jetson/test/test_services.py"}, starters
    readers = [path for path in mentions("argus_camera") if path.startswith("src/") and path.endswith(".py") and "/test/" not in path]  # flight code
    assert set(readers) <= {
        "src/drone_mission_core/drone_mission_core/estimator_feed.py",  # the estimator's camera, through frame_projection (FND-06)
        "src/drone_mission_core/drone_mission_core/frame_projection.py",
        "src/drone_mission_core/drone_mission_core/projection_v2.py",
        "src/drone_target_cv/drone_target_cv/ring_fit_node.py",  # the ring fit's camera, through frame_projection (CEN-13)
        "src/drone_target_cv/drone_target_cv/recovery_cv_node.py",  # the recovery detector's camera, through frame_projection (REC-1)
        "src/argus_localisation/argus_localisation/loc_node.py",  # the localisation node reads frames with the package's FrameSource, inside itself (LOC-5)
        "src/argus_localisation/argus_localisation/frame_logging.py",  # the survey's logging policy writes with the package's one FrameLog (LOC-11)
        "src/vision_pipeline/vision_pipeline/logger_projection.py",
        "src/vision_pipeline/vision_pipeline/mission_logger.py",
    }, readers
