"""Capture timestamps (FND-01): from a GStreamer buffer's PTS or a V4L2 kernel timestamp to the caller's clock. No ROS."""

import ast
from pathlib import Path

import numpy as np
import pytest

from argus_camera import capture_clock, frame_source, modes
from argus_camera.capture_clock import CaptureClock, stamp_from_kernel, stamp_from_pts
from argus_camera.frame_source import CaptureSource, FrameSourceError, GstCsiSource, open_source

PACKAGE = Path(__file__).resolve().parents[1]
MODES = modes.load_modes(PACKAGE / "config" / "modes.yaml")
MS = 1_000_000
S = 1_000_000_000


# --- the arithmetic ----------------------------------------------------------------------------------


def test_pts_plus_base_time_is_when_the_frame_was_taken_on_the_pipeline_clock():
    # The pipeline started at 5000 s on its own clock. A buffer with PTS 12.340 s was captured at 5012.340 s.
    # It is read when the pipeline clock says 5012.395 s (55 ms later) and the ROS clock says 800.000 s.
    stamp = stamp_from_pts(pts_ns=12_340 * MS, base_time_ns=5000 * S, pipeline_now_ns=5012 * S + 395 * MS, clock_now_ns=800 * S)
    assert stamp == 800 * S - 55 * MS


def test_the_measured_camera_offset_is_subtracted():
    # The LED probe finds the sensor exposes 18 ms before the buffer's PTS says (PLT-05).
    stamp = stamp_from_pts(12_340 * MS, 5000 * S, 5012 * S + 395 * MS, 800 * S, offset_ns=18 * MS)
    assert stamp == 800 * S - 55 * MS - 18 * MS


def test_a_kernel_timestamp_is_moved_from_the_monotonic_clock_to_the_callers():
    stamp = stamp_from_kernel(buffer_ns=77 * S + 120 * MS, monotonic_now_ns=77 * S + 131 * MS, clock_now_ns=800 * S)
    assert stamp == 800 * S - 11 * MS
    assert stamp_from_kernel(77 * S, 77 * S + 11 * MS, 800 * S, offset_ns=2 * MS) == 800 * S - 13 * MS


def test_synthetic_60_fps_pts_come_out_evenly_spaced_whatever_the_read_jitter():
    # Frames are captured every 16.667 ms but read with 0 to 9 ms of jitter: the stamps must not carry the jitter.
    clock_offset = 800 * S - 5000 * S  # ROS clock minus pipeline clock
    stamps = []
    for index in range(120):
        pts = index * S // 60
        read_delay = (index * 7) % 10 * MS + 30 * MS
        pipeline_now = 5000 * S + pts + read_delay
        stamps.append(stamp_from_pts(pts, 5000 * S, pipeline_now, pipeline_now + clock_offset))
    gaps = {later - earlier for earlier, later in zip(stamps, stamps[1:])}
    assert gaps <= {S // 60, S // 60 + 1}
    assert stamps[0] == 800 * S


# --- the clock object: sanity and fallback ------------------------------------------------------------------------


class Now:
    def __init__(self, value=800 * S):
        self.value = value

    def __call__(self):
        return self.value


def test_capture_clock_uses_the_pts_and_says_so():
    clock = CaptureClock(Now(), offset_s=0.018)
    stamp, source = clock.from_pts(12_340 * MS, 5000 * S, 5012 * S + 395 * MS)
    assert (stamp, source) == (800 * S - 73 * MS, "pts")
    assert clock.fallbacks == 0


@pytest.mark.parametrize(
    "pts_ns, why",
    [
        (None, "no PTS on the buffer"),
        (-1, "no PTS on the buffer"),
        (12_500 * MS, "from the future"),  # 105 ms after the read
    ],
)
def test_a_pts_that_cannot_be_right_falls_back_to_the_read_time_and_is_counted(pts_ns, why):
    clock = CaptureClock(Now())
    stamp, source = clock.from_pts(pts_ns, 5000 * S, 5012 * S + 395 * MS)
    assert (stamp, source) == (800 * S, "read")
    assert clock.fallbacks == 1 and why in clock.last_problem


def test_a_frame_over_a_second_old_by_its_own_timestamp_is_dropped_not_restamped_as_new():
    # After a stall the pipeline hands over an old frame. Stamped at the read it would look fresh to everything downstream.
    clock = CaptureClock(Now())
    assert clock.from_pts(10_000 * MS, 5000 * S, 5012 * S + 395 * MS) is None  # 2.4 s old
    assert clock.from_kernel(75 * S, 77 * S) is None  # 2 s old
    assert clock.stale == 2 and clock.fallbacks == 0 and "dropped" in clock.last_problem
    assert clock.from_pts(11_500 * MS, 5000 * S, 5012 * S + 395 * MS) == (800 * S - 895 * MS, "pts")  # under a second: passed on with its true age


def test_kernel_timestamps_get_the_same_sanity_check():
    clock = CaptureClock(Now())
    assert clock.from_kernel(77 * S + 120 * MS, 77 * S + 131 * MS) == (800 * S - 11 * MS, "kernel")
    assert clock.from_kernel(0, 77 * S) == (800 * S, "read")  # a driver that does not stamp its buffers
    assert clock.from_kernel(77 * S, 70 * S) == (800 * S, "read")
    assert clock.fallbacks == 2
    assert clock.on_read() == (800 * S, "read") and clock.fallbacks == 2  # a source with nothing better is not a fallback


def test_the_offset_comes_from_the_modes_file_and_is_zero_until_it_is_measured():
    for name in ("imx519_4k", "imx519_720p60", "ov9782_flow", "sim_delivery"):
        assert MODES[name].capture_offset_s == 0.0
    assert "capture_offset_s" in (PACKAGE / "config" / "modes.yaml").read_text()


def modes_with(tmp_path, mode_name, **changes):
    import yaml

    data = yaml.safe_load((PACKAGE / "config" / "modes.yaml").read_text())
    data["modes"][mode_name].update(changes)
    path = tmp_path / "modes.yaml"
    path.write_text(yaml.safe_dump(data))
    return modes.load_modes(path)


def test_an_offset_written_in_the_modes_file_reaches_the_stamp_of_a_csi_frame(tmp_path):
    # What PLT-05 will do: measure 18 ms and write it in modes.yaml. Nothing else may be needed.
    measured = modes_with(tmp_path, "imx519_720p60", capture_offset_s=0.018)["imx519_720p60"]
    assert measured.capture_offset_s == 0.018
    frames = {}
    for label, mode in (("as shipped", MODES["imx519_720p60"]), ("measured", measured)):
        source = GstCsiSource(mode, Now(), gst=FakeGst([(hd_frame(), 12_340 * MS)]))
        source.open()
        frames[label] = source.read()
    assert frames["as shipped"].capture_ns == 800 * S - 55 * MS
    assert frames["measured"].capture_ns == 800 * S - 55 * MS - 18 * MS and frames["measured"].stamp_source == "pts"


def test_an_offset_written_in_the_modes_file_reaches_the_stamp_of_a_usb_frame(tmp_path):
    measured = modes_with(tmp_path, "ov9782_flow", capture_offset_s=0.004)["ov9782_flow"]
    source = CaptureSource(measured, Now(), opener=lambda mode: FakeV4l2([flow_frame()], pos_msec=77_120.0), monotonic=lambda: 77 * S + 131 * MS)
    source.open()
    assert source.read().capture_ns == 800 * S - 11 * MS - 4 * MS


@pytest.mark.parametrize("value", [0.6, -0.6, -0.001, "18ms", True])
def test_an_offset_that_cannot_be_a_camera_delay_is_refused_when_the_modes_are_loaded(tmp_path, value):
    with pytest.raises(modes.ModeError, match="capture_offset_s"):
        modes_with(tmp_path, "imx519_720p60", capture_offset_s=value)


# --- the sources stamp frames with it ------------------------------------------------------------------------------


class FakeV4l2:
    def __init__(self, images, pos_msec):
        self.images, self.pos_msec = list(images), pos_msec

    def isOpened(self):
        return True

    def read(self):
        return (True, self.images.pop(0)) if self.images else (False, None)

    def get(self, prop):
        assert prop == frame_source.cv2.CAP_PROP_POS_MSEC
        return self.pos_msec

    def release(self):
        pass


def flow_frame():
    return np.zeros((800, 1280, 3), dtype=np.uint8)


def test_the_usb_camera_frame_carries_the_kernels_buffer_time():
    capture = FakeV4l2([flow_frame()], pos_msec=77_120.0)
    source = CaptureSource(MODES["ov9782_flow"], Now(), opener=lambda mode: capture, monotonic=lambda: 77 * S + 131 * MS)
    source.open()
    frame = source.read()
    assert frame.capture_ns == 800 * S - 11 * MS and frame.stamp_source == "kernel"


def test_a_stale_usb_frame_is_not_handed_on():
    capture = FakeV4l2([flow_frame(), flow_frame()], pos_msec=75_000.0)  # 2 s before the read
    source = CaptureSource(MODES["ov9782_flow"], Now(), opener=lambda mode: capture, monotonic=lambda: 77 * S)
    source.open()
    assert source.read() is None and source.capture_clock.stale == 1


def test_a_usb_camera_with_no_buffer_time_is_stamped_at_the_read_and_says_so():
    capture = FakeV4l2([flow_frame()], pos_msec=0.0)
    source = CaptureSource(MODES["ov9782_flow"], Now(), opener=lambda mode: capture, monotonic=lambda: 77 * S)
    source.open()
    frame = source.read()
    assert frame.capture_ns == 800 * S and frame.stamp_source == "read" and source.capture_clock.fallbacks == 1


class FakeGst:
    """What GstCsiSource uses of GStreamer: a pipeline with a clock and a base time, and an appsink that hands out samples."""

    SECOND = S
    CLOCK_TIME_NONE = 2**64 - 1

    class State:
        PLAYING, NULL = "playing", "null"

    class StateChangeReturn:
        FAILURE = "failure"

    class MapFlags:
        READ = 1

    class MessageType:
        ERROR = "error"

    def __init__(self, frames, base_time_ns=5000 * S, now_ns=5012 * S + 395 * MS, start_ok=True, settles=True, bus_error=None):
        self.frames, self.base_time_ns, self.now_ns, self.start_ok = list(frames), base_time_ns, now_ns, start_ok
        self.settles, self.bus_error = settles, bus_error
        self.launched = None
        self.states = []

    def get_state(self, timeout_ns):
        return ("success" if self.settles else self.StateChangeReturn.FAILURE, None, None)

    def get_bus(self):
        return self

    def pop_filtered(self, kind):
        assert kind == self.MessageType.ERROR
        error, self.bus_error = self.bus_error, None
        return None if error is None else type("Message", (), {"parse_error": lambda _self: (error, "debug text")})()

    def parse_launch(self, text):
        self.launched = text
        return self

    # the pipeline
    def get_by_name(self, name):
        assert name == "sink"
        return self

    def set_state(self, state):
        self.states.append(state)
        return "success" if self.start_ok else self.StateChangeReturn.FAILURE

    def get_base_time(self):
        return self.base_time_ns

    def get_clock(self):
        return self

    def get_time(self):
        return self.now_ns

    # the appsink
    def emit(self, signal, timeout_ns):
        assert signal == "try-pull-sample"
        if not self.frames:
            return None
        image, pts = self.frames.pop(0)
        return FakeSample(image, pts)


class FakeSample:
    def __init__(self, image, pts):
        self.image, self.pts = image, pts

    def get_buffer(self):
        return self

    def map(self, flags):
        return True, self

    @property
    def data(self):
        return self.image.tobytes()

    def unmap(self, info):
        pass


def hd_frame(value=7):
    return np.full((720, 1280, 3), value, dtype=np.uint8)


def test_the_csi_camera_frame_carries_its_pts_moved_to_the_callers_clock():
    gst = FakeGst([(hd_frame(), 12_340 * MS)])
    source = GstCsiSource(MODES["imx519_720p60"], Now(), gst=gst)
    source.open()
    frame = source.read()
    assert frame.capture_ns == 800 * S - 55 * MS and frame.stamp_source == "pts"
    assert frame.image.shape == (720, 1280, 3) and int(frame.image[0, 0, 0]) == 7
    assert "nvarguscamerasrc sensor-id=0" in gst.launched and gst.launched.endswith("appsink name=sink drop=true max-buffers=1 sync=false")
    assert gst.states == ["playing"]
    assert source.read() is None  # nothing new: no frame, not the old one
    source.close()
    assert gst.states == ["playing", "null"]


def test_a_csi_buffer_without_a_pts_is_stamped_at_the_read_and_counted():
    gst = FakeGst([(hd_frame(), FakeGst.CLOCK_TIME_NONE)])
    source = GstCsiSource(MODES["imx519_720p60"], Now(), gst=gst)
    source.open()
    frame = source.read()
    assert frame.capture_ns == 800 * S and frame.stamp_source == "read" and source.capture_clock.fallbacks == 1


def test_a_csi_pipeline_that_does_not_start_says_which_mode():
    with pytest.raises(FrameSourceError, match="imx519_720p60"):
        GstCsiSource(MODES["imx519_720p60"], Now(), gst=FakeGst([], start_ok=False)).open()


def test_a_csi_pipeline_that_fails_a_moment_after_being_started_is_caught_too():
    # No camera on the port: the change to PLAYING is accepted and then fails.
    gst = FakeGst([], settles=False)
    with pytest.raises(FrameSourceError, match="did not start"):
        GstCsiSource(MODES["imx519_720p60"], Now(), gst=gst).open()
    assert gst.states == ["playing", "null"]  # and the pipeline is taken down again


def test_a_csi_pipeline_that_errors_later_raises_instead_of_going_quiet():
    gst = FakeGst([(hd_frame(), 12_340 * MS)], bus_error="the sensor stopped answering")
    source = GstCsiSource(MODES["imx519_720p60"], Now(), gst=gst)
    source.open()
    assert source.read() is not None  # a frame was there
    with pytest.raises(FrameSourceError, match="the sensor stopped answering"):
        source.read()
    assert source.read() is None  # no error pending, no frame: just nothing yet


def test_a_stale_csi_frame_is_not_handed_on():
    gst = FakeGst([(hd_frame(1), 10_000 * MS), (hd_frame(2), 12_340 * MS)])  # the first is 2.4 s old
    source = GstCsiSource(MODES["imx519_720p60"], Now(), gst=gst)
    source.open()
    assert source.read() is None and source.capture_clock.stale == 1
    frame = source.read()
    assert int(frame.image[0, 0, 0]) == 2 and frame.seq == 0 and frame.stamp_source == "pts"


def test_the_csi_backend_is_chosen_never_guessed_from_what_is_installed(monkeypatch):
    gst = FakeGst([])
    monkeypatch.setattr(frame_source, "load_gst", lambda: gst)
    # The bindings being installed changes nothing by itself: the default is the OpenCV capture.
    default = open_source(MODES["imx519_720p60"], clock=Now())
    assert isinstance(default, CaptureSource) and not isinstance(default, GstCsiSource)
    assert isinstance(open_source(MODES["imx519_720p60"], clock=Now(), csi_backend="gstreamer"), GstCsiSource)
    monkeypatch.setattr(frame_source, "load_gst", lambda: None)  # python3-gi is not installed
    with pytest.raises(FrameSourceError, match="bindings are not installed"):
        open_source(MODES["imx519_720p60"], clock=Now(), csi_backend="gstreamer")
    with pytest.raises(FrameSourceError, match="csi_backend"):
        open_source(MODES["imx519_720p60"], clock=Now(), csi_backend="auto")
    # The USB camera never needs the bindings, whatever is asked.
    monkeypatch.setattr(frame_source, "load_gst", lambda: (_ for _ in ()).throw(AssertionError("not for v4l2")))
    assert isinstance(open_source(MODES["ov9782_flow"], clock=Now(), csi_backend="gstreamer"), CaptureSource)


def test_the_opencv_csi_path_has_no_timestamp_and_says_so(monkeypatch):
    class Capture(FakeV4l2):
        def get(self, prop):
            raise AssertionError("the GStreamer capture's position is not a capture time")

    source = CaptureSource(MODES["imx519_720p60"], Now(), opener=lambda mode: Capture([hd_frame()], 0.0))
    source.open()
    frame = source.read()
    assert frame.capture_ns == 800 * S and frame.stamp_source == "read"
    assert source.capture_clock.fallbacks == 0  # nothing better was on offer, so it is not counted as a failure


def test_the_module_has_no_ros_in_it():
    tree = ast.parse(Path(capture_clock.__file__).read_text())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {alias.name.split(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            imported.add((node.module or "").split(".")[0])
    assert imported <= {"__future__", "typing"}
