"""PLT-05: the LED latency probe (argus_camera/latency_probe.py). No ROS, no GPIO, no camera."""

import json
import random
import re
import threading
import time

import numpy as np
import pytest

from argus_camera import latency_probe as probe
from argus_camera.frame_source import Frame

MS = 1_000_000
PERIOD_S = 1.0 / 60.0


def simulate(stamp_error_s, blinks=120, period_s=PERIOD_S, seed=3, locked=False, drop_every=0, miss_every=0):
    """A camera whose stamps are `stamp_error_s` later than mid-exposure, watching an LED blink.

    Returns (LED-on times, frame stamps, frame levels) as the probe records them:
    a frame is lit when the LED came on before the middle of its exposure.
    """
    rng = random.Random(seed)
    period = round(period_s * 1e9)
    on_times, t = [], 5 * period
    for index in range(blinks):
        gap = 30 * period if locked else rng.randrange(30 * period, 34 * period)
        t += gap
        on_times.append(t if not locked else (t // period) * period + period // 4)
    off_times = [on + 6 * period for on in on_times]
    stamps, levels = [], []
    frames = (on_times[-1] + 20 * period) // period
    for k in range(frames):
        if drop_every and k % drop_every == 0:
            continue
        middle = k * period + period // 2
        lit = any(on <= middle < off for on, off in zip(on_times, off_times))
        stamps.append(middle + round(stamp_error_s * 1e9))
        levels.append(200.0 + rng.uniform(-3, 3) if lit else 20.0 + rng.uniform(-3, 3))
    if miss_every:
        on_times = [on for index, on in enumerate(on_times) if index % miss_every]
    return on_times, stamps, levels


@pytest.mark.parametrize("error_ms", [0.0, 12.0, 41.5, -8.0])
def test_the_offset_is_found_to_within_a_millisecond_and_a_half(error_ms):
    result = probe.led_offset(*simulate(error_ms / 1000.0), period_s=PERIOD_S)
    assert result.valid, result.problems
    assert result.blinks >= 110
    assert result.offset_s * 1000.0 == pytest.approx(error_ms, abs=1.5)
    assert result.stderr_s < 0.001


def test_other_frame_rates_work_the_same():
    result = probe.led_offset(*simulate(0.030, period_s=1.0 / 17.0, blinks=200), period_s=1.0 / 17.0)
    assert result.valid
    assert result.offset_s == pytest.approx(0.030, abs=0.005)


def test_blinks_locked_to_the_frames_are_not_a_measurement():
    result = probe.led_offset(*simulate(0.012, locked=True), period_s=PERIOD_S)
    assert not result.valid
    assert any("locked" in problem for problem in result.problems)


def test_too_few_blinks_are_not_a_measurement():
    result = probe.led_offset(*simulate(0.012, blinks=10), period_s=PERIOD_S)
    assert not result.valid
    assert any("blinks" in problem for problem in result.problems)


def test_an_led_that_never_lights_in_the_picture_is_not_a_measurement():
    on_times, stamps, _levels = simulate(0.012)
    result = probe.led_offset(on_times, stamps, [20.0] * len(stamps), period_s=PERIOD_S)
    assert not result.valid and result.blinks == 0


def test_dropped_frames_are_set_aside_not_averaged_in():
    clean = probe.led_offset(*simulate(0.012), period_s=PERIOD_S)
    dropped = probe.led_offset(*simulate(0.012, drop_every=7), period_s=PERIOD_S)
    assert dropped.valid
    assert dropped.rejected > 0
    assert dropped.offset_s == pytest.approx(clean.offset_s, abs=0.002)


def test_a_blink_the_log_missed_does_not_shift_the_pairing():
    result = probe.led_offset(*simulate(0.012, miss_every=9), period_s=PERIOD_S)
    assert result.valid
    assert result.offset_s * 1000.0 == pytest.approx(12.0, abs=1.5)


def test_the_residual_passes_at_5_ms_and_says_what_to_set_otherwise():
    small = probe.led_offset(*simulate(0.003), period_s=PERIOD_S)
    passed, line = probe.judge(small, configured_offset_s=0.040)
    assert passed and "PASS" in line
    large = probe.led_offset(*simulate(0.012), period_s=PERIOD_S)
    passed, line = probe.judge(large, configured_offset_s=0.040)
    assert not passed
    said = float(re.search(r"capture_offset_s: ([0-9.]+)", line).group(1))
    assert said == pytest.approx(0.052, abs=0.0015)  # 40 ms configured and 12 ms more measured
    invalid = probe.led_offset(*simulate(0.012, blinks=5), period_s=PERIOD_S)
    assert not probe.judge(invalid, configured_offset_s=0.0)[0]


def test_the_led_is_found_in_the_picture_and_its_absence_is_said():
    dark = np.full((90, 160, 3), 30, dtype=np.uint8)
    lit = dark.copy()
    lit[40:48, 100:110] = 250
    x0, y0, x1, y1 = probe.find_led_roi(dark, lit)
    assert (x0, y0, x1, y1) == (100, 40, 110, 48)
    assert probe.led_level(lit, (x0, y0, x1, y1)) > 200 > 40 > probe.led_level(dark, (x0, y0, x1, y1))
    assert probe.find_led_roi(dark, dark.copy()) is None


def test_gaps_between_blinks_cover_every_phase_of_a_frame():
    rng = random.Random(1)
    gaps = [probe.next_gap_s(PERIOD_S, rng) for _ in range(500)]
    assert min(gaps) >= probe.MIN_GAP_S
    phases = sorted((gap % PERIOD_S) / PERIOD_S for gap in gaps)
    assert phases[0] < 0.05 and phases[-1] > 0.95
    assert max(b - a for a, b in zip(phases, phases[1:])) < 0.1


class FakeLed:
    def __init__(self):
        self.lock = threading.Lock()
        self.events = []  # (monotonic ns, on?)

    def on(self):
        with self.lock:
            self.events.append((time.monotonic_ns(), True))

    def off(self):
        with self.lock:
            self.events.append((time.monotonic_ns(), False))

    def is_on_at(self, moment_ns):
        state = False
        with self.lock:
            for at, on in self.events:
                if at <= moment_ns:
                    state = on
        return state

    def close(self):
        self.closed = True


class FakeCamera:
    """100 frames a second on the real clock, stamped `error_ns` late, seeing the fake LED."""

    def __init__(self, led, error_ns, period_ns=10 * MS):
        self.led, self.error_ns, self.period_ns = led, error_ns, period_ns
        self.seq = 0
        self.next_ns = (time.monotonic_ns() // period_ns + 2) * period_ns

    def read(self):
        middle = self.next_ns
        self.next_ns += self.period_ns
        delay = (middle + 3 * MS - time.monotonic_ns()) / 1e9  # the frame reaches the reader 3 ms after its exposure
        if delay > 0:
            time.sleep(delay)
        image = np.full((40, 60, 3), 25, dtype=np.uint8)
        if self.led.is_on_at(middle):
            image[10:20, 30:40] = 240
        self.seq += 1
        return Frame(image=image, seq=self.seq, capture_ns=middle + self.error_ns, mode="fake", stamp_source="pts")


def test_the_probe_blinks_a_real_thread_and_measures_a_fake_camera():
    led = FakeLed()
    camera = FakeCamera(led, error_ns=50 * MS)
    record = probe.run_probe(camera, led, clock=time.monotonic_ns, period_s=0.010, blinks=40, on_s=0.05, min_gap_s=0.08, settle_s=0.1)
    assert record["roi"] == [30, 10, 40, 20]
    assert len(record["on_ns"]) == 40
    assert led.events[-1][1] is False  # left off
    result = probe.led_offset(record["on_ns"], record["stamps_ns"], record["levels"], period_s=0.010, max_error_s=0.07)
    assert result.valid, result.problems
    # A loaded test machine sleeps late; 50 ms against 0 is still unmistakable.
    assert result.offset_s == pytest.approx(0.050, abs=0.010)
    assert json.loads(json.dumps(record))["blinks"] == 40


def test_the_probe_stops_when_the_led_is_not_in_view():
    led = FakeLed()
    camera = FakeCamera(led, error_ns=0)
    led.is_on_at = lambda _moment: False  # the camera never sees it
    with pytest.raises(probe.ProbeError, match="LED"):
        probe.run_probe(camera, led, clock=time.monotonic_ns, period_s=0.010, blinks=5, on_s=0.05, min_gap_s=0.08, settle_s=0.1)
    assert led.events[-1][1] is False


def test_the_tool_analyses_a_saved_record(tmp_path, capsys):
    on_times, stamps, levels = simulate(0.012)
    saved = tmp_path / "probe.json"
    saved.write_text(json.dumps({"mode": "imx519_720p60", "period_s": PERIOD_S, "configured_offset_s": 0.0, "on_ns": on_times, "stamps_ns": stamps, "levels": levels}))
    assert probe.main(["--analyse", str(saved)]) == 1
    out = capsys.readouterr().out
    assert float(re.search(r"capture_offset_s: ([0-9.]+)", out).group(1)) == pytest.approx(0.012, abs=0.0015)
    assert "latency probe: FAIL" in out
    saved.write_text(json.dumps({"mode": "imx519_720p60", "period_s": PERIOD_S, "configured_offset_s": 0.012, "on_ns": on_times, "stamps_ns": [s - 12 * MS for s in stamps], "levels": levels}))
    assert probe.main(["--analyse", str(saved)]) == 0
    assert "latency probe: PASS" in capsys.readouterr().out


def test_without_the_gpio_library_the_tool_says_so(monkeypatch):
    monkeypatch.setattr(probe, "load_gpio", lambda: None)
    with pytest.raises(probe.ProbeError, match="Jetson.GPIO"):
        probe.GpioLed(7)


def offset(value_s):
    return probe.LedOffset(blinks=200, rejected=0, offset_s=value_s, stderr_s=0.0003, spread_s=0.016, valid=True)


def test_the_residual_limit_is_fnd_01s_5_ms():
    assert probe.RESIDUAL_LIMIT_S == 0.005
    for sign in (1.0, -1.0):
        assert probe.judge(offset(sign * 0.0049), configured_offset_s=0.040)[0]
        assert not probe.judge(offset(sign * 0.0051), configured_offset_s=0.040)[0]


def test_an_offset_that_would_have_to_be_negative_is_not_suggested():
    passed, line = probe.judge(offset(-0.008), configured_offset_s=0.0)
    assert not passed
    assert "capture_offset_s:" not in line  # nothing to set: modes.yaml refuses a negative offset
    assert "8.0 ms before the LED" in line
    # Coming down from too large an offset is still a number to set.
    passed, line = probe.judge(offset(-0.008), configured_offset_s=0.020)
    assert not passed and "capture_offset_s: 0.012" in line


class FakeGpio:
    """Jetson.GPIO as far as the probe uses it; the pin's history is what the fake camera looks at."""

    BOARD, OUT, LOW, HIGH = "BOARD", "OUT", 0, 1

    def __init__(self):
        self.calls, self.led = [], FakeLed()

    def setmode(self, mode):
        self.calls.append(("setmode", mode))

    def setup(self, pin, direction, initial=None):
        self.calls.append(("setup", pin, direction, initial))

    def output(self, pin, level):
        self.calls.append(("output", pin, level))
        (self.led.on if level == self.HIGH else self.led.off)()

    def cleanup(self, pin):
        self.calls.append(("cleanup", pin))


def test_the_live_run_drives_the_pin_reads_the_camera_and_saves_the_record(tmp_path, monkeypatch, capsys):
    from argus_camera import frame_source

    gpio = FakeGpio()
    opened = {}

    class Camera(FakeCamera):
        def __enter__(self):
            return self

        def __exit__(self, *_exc):
            opened["closed"] = True

    def open_source(mode, *, clock, csi_backend):
        opened.update(mode=mode.name, backend=csi_backend, clock=clock)
        return Camera(gpio.led, error_ns=0)

    monkeypatch.setattr(probe, "load_gpio", lambda: gpio)
    monkeypatch.setattr(frame_source, "open_source", open_source)
    monkeypatch.setattr(time, "time_ns", time.monotonic_ns)  # the fake camera stamps on the monotonic clock
    saved = tmp_path / "run.json"
    code = probe.main(["--mode", "imx519_720p60", "--pin", "7", "--blinks", "3", "--out", str(saved)])
    out = capsys.readouterr().out
    assert code == 1 and "3 usable blinks of 3: 30 are needed" in out  # three blinks are a run, not a measurement
    assert opened["mode"] == "imx519_720p60" and opened["backend"] == "gstreamer" and opened["closed"]
    assert gpio.calls[:2] == [("setmode", "BOARD"), ("setup", 7, "OUT", 0)]
    assert gpio.calls[-2:] == [("output", 7, 0), ("cleanup", 7)]  # left off, and the pin given back
    assert sum(1 for call in gpio.calls if call == ("output", 7, 1)) == 4  # once to find it in the picture, then three blinks
    record = json.loads(saved.read_text())
    assert record["mode"] == "imx519_720p60" and record["blinks"] == 3 and record["stamp_source"] == "pts"
    assert record["configured_offset_s"] == 0.0 and record["roi"] == [30, 10, 40, 20]
    assert probe.main(["--analyse", str(saved)]) == 1


def test_the_live_run_needs_a_mode_and_a_pin():
    with pytest.raises(SystemExit):
        probe.main(["--mode", "imx519_720p60"])
