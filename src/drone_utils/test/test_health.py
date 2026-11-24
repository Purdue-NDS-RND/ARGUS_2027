"""PLT-11: the companion computer's health and the status texts it gives the ground station (drone_utils/health.py)."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_utils import health as h  # noqa: E402


def fed(rate, hz, start, end):
    t = start
    while t < end:
        rate.add(t)
        t += 1.0 / hz


def run(health, reporter=None, start=0.0, end=60.0, each=None):
    """Ask once a second; returns the findings' texts at the end and every status text sent with its time."""
    sent, findings = [], []
    t = start
    while t <= end:
        if each is not None:
            each(t)
        findings = health.findings(t)
        if reporter is not None:
            out = reporter.next(findings, t)
            if out is not None:
                sent.append((t, *out))
        t += 1.0
    return [finding.text for finding in findings], sent


def test_a_topic_under_four_fifths_of_its_rate_or_silent_is_a_finding_after_five_seconds_and_not_before():
    camera, pose = h.Rate("cam", 30.0), h.Rate("pose", 30.0)
    health = h.Health([camera, pose])

    def each(t):
        fed(pose, 30.0, t, t + 1.0)
        if t < 40.0:
            fed(camera, 30.0 if t < 20.0 else 12.0, t, t + 1.0)  # full rate, then 12 a second, then nothing

    assert run(health, end=19.0, each=each)[0] == []
    assert run(health, start=20.0, end=24.0, each=each)[0] == []  # the window's rate has been under the line for less than five seconds: not yet
    assert run(health, start=25.0, end=39.0, each=each)[0] == ["ARGUS H:RATE cam 12/30Hz"]
    assert run(health, start=40.0, end=52.0, each=each)[0] == ["ARGUS H:SILENT cam"]
    assert h.MIN_RATE_SHARE == 0.8 and h.HOLD_S == 5.0 and h.SILENT_S == 3.0
    # A topic never heard at all is silent too, once the monitor has run a while.
    never = h.Health([h.Rate("lidar", 10.0)], started_s=100.0)
    assert run(never, start=100.0, end=102.0)[0] == [] and run(never, start=103.0, end=112.0)[0] == ["ARGUS H:SILENT lidar"]
    with pytest.raises(ValueError, match="over 0"):
        h.Rate("x", 0.0)


def test_the_computer_is_read_from_the_kernels_files_and_what_cannot_be_read_is_not_known(tmp_path):
    thermal, cpus = tmp_path / "thermal", tmp_path / "cpu"
    for index, (kind, milli) in enumerate((("cpu-thermal", "71500"), ("gpu-thermal", "88250"), ("broken", "x"), ("absent", "-274000"))):
        zone = thermal / f"thermal_zone{index}"
        zone.mkdir(parents=True)
        (zone / "type").write_text(kind + "\n")
        (zone / "temp").write_text(milli + "\n")
    assert h.read_temperatures(thermal) == [("cpu-thermal", 71.5), ("gpu-thermal", 88.25)]
    assert h.read_temperatures(tmp_path / "nowhere") == []
    assert h.read_throttled(cpus) is None  # the kernel does not say
    for index, allowed in enumerate((2035200, 2035200)):
        policy = cpus / f"cpu{index}" / "cpufreq"
        policy.mkdir(parents=True)
        (policy / "scaling_max_freq").write_text(f"{allowed}\n")
        (policy / "cpuinfo_max_freq").write_text("2035200\n")
    assert h.read_throttled(cpus) is False
    (cpus / "cpu1" / "cpufreq" / "scaling_max_freq").write_text("1190400\n")
    assert h.read_throttled(cpus) is True
    gpu = tmp_path / "gpu_load"
    gpu.write_text("987\n")
    assert h.read_gpu_percent([str(tmp_path / "none"), str(gpu)]) == 98.7 and h.read_gpu_percent([str(tmp_path / "none")]) is None
    gpu.write_text("5000\n")
    assert h.read_gpu_percent([str(gpu)]) is None  # not the 0 to 1000 the Jetson gives: not taken for a load
    assert h.cpu_times("cpu  100 0 50 800 50 0 0 0 0 0\ncpu0 1 2 3 4\n") == (150, 1000) and h.cpu_times("intr 5\n") is None
    stat = tmp_path / "stat"
    stat.write_text("cpu  100 0 50 800 50 0 0 0 0 0\n")
    computer = h.Computer(thermal, cpus, stat, [str(tmp_path / "none")])
    first = computer.read()
    assert first.cpu_percent is None and first.gpu_percent is None and first.throttled is True and len(first.temperatures) == 2
    stat.write_text("cpu  190 0 60 800 50 0 0 0 0 0\n")  # 100 more jiffies, all of them busy
    assert computer.read().cpu_percent == pytest.approx(100.0)


def test_heat_throttling_load_and_dropped_frames_are_findings_once_they_last():
    health = h.Health()

    def each(t):
        health.read(h.Reading([("cpu-thermal", 70.0), ("gpu-thermal", 91.4)], True, 97.2, 99.0) if 10.0 <= t < 30.0 else h.Reading([("cpu-thermal", 60.0)], False, 40.0, None))
        health.counted("loc", int(10 * t), int(3 * t) if t >= 10.0 else 0, t)  # from 10 s on, three frames dropped for every ten had

    assert run(health, end=14.0, each=each)[0] == []
    texts, _sent = run(health, start=15.0, end=29.0, each=each)
    assert texts == ["ARGUS H:CPU 97%", "ARGUS H:DROP loc 23%", "ARGUS H:GPU 99%", "ARGUS H:TEMP gpu-thermal 91C", "ARGUS H:THROTTLE cpu"]
    # The computer cools at 30 s: its findings last five more seconds and are gone; the drops go on.
    assert len(run(health, start=30.0, end=34.0, each=each)[0]) == 5 and run(health, start=35.0, end=40.0, each=each)[0] == ["ARGUS H:DROP loc 23%"]
    # A node that starts again counts from nothing: not a negative share, and not a finding by itself.
    again = h.Health()
    again.counted("loc", 1000, 400, 1.0)
    again.counted("loc", 5, 0, 2.0)
    again.counted("loc", 15, 0, 3.0)
    assert again.findings(10.0) == [] and h.Health().findings(0.0) == []
    again.counted("loc", "many", None, 4.0)  # not numbers: not counted
    summary = health.summary(40.0)
    assert summary["cpu_percent"] == 40.0 and summary["gpu_percent"] is None and summary["throttled"] is False and summary["findings"] == ["ARGUS H:DROP loc 23%"]


def test_the_ground_station_gets_one_short_line_a_finding_again_every_thirty_seconds_and_ok_when_all_is_well():
    health, reporter = h.Health(), h.Reporter()

    def each(t):
        hot = 10.0 <= t < 80.0
        health.read(h.Reading([("cpu-thermal", 93.0 if hot else 55.0)], hot, 30.0, None))

    _texts, sent = run(health, reporter, end=120.0, each=each)
    assert [(t, text) for t, _severity, text in sent] == [
        (15.0, "ARGUS H:TEMP cpu-thermal 93C"), (17.0, "ARGUS H:THROTTLE cpu"),  # one line at a time, two seconds apart
        (45.0, "ARGUS H:TEMP cpu-thermal 93C"), (47.0, "ARGUS H:THROTTLE cpu"),  # again thirty seconds later, while it lasts
        (75.0, "ARGUS H:TEMP cpu-thermal 93C"), (77.0, "ARGUS H:THROTTLE cpu"),
        (85.0, "ARGUS H:OK"),  # five seconds after the last of it has gone, once
    ]
    assert [severity for _t, severity, _text in sent] == [h.WARNING] * 6 + [h.NOTICE] and reporter.sent == 7
    assert all(len(text) <= 50 for _t, _severity, text in sent) and h.STATUSTEXT_MAX == 50
    # Nothing wrong from the start: nothing is ever said, not even OK.
    quiet, silent = h.Health(), h.Reporter()
    assert run(quiet, silent, end=120.0)[1] == []
    # A long name does not make a text longer than a STATUSTEXT carries.
    assert len(h.text_of("RATE", "the_recovery_detectors_own_output_topic 3/30Hz")) == 50


def test_the_rates_parameter_is_name_topic_type_and_rate_and_a_wrong_one_is_said_whole():
    assert h.parse_rates(["cam:camera/image:sensor_msgs/msg/Image:30", "pose:/mavros/local_position/pose:geometry_msgs/msg/PoseStamped:30.0"]) == [
        ("cam", "camera/image", "sensor_msgs/msg/Image", 30.0), ("pose", "/mavros/local_position/pose", "geometry_msgs/msg/PoseStamped", 30.0)]  # fmt: skip
    assert h.parse_rates([]) == []
    for wrong in ("cam:camera/image:30", "cam:camera/image:sensor_msgs/msg/Image:fast", ":t:a/msg/B:5", "cam::a/msg/B:5", "cam:t:Image:5", "cam:camera/image:sensor_msgs/msg/Image:0"):
        with pytest.raises(ValueError, match="is not name:topic:type:hz"):
            h.parse_rates([wrong])
    with pytest.raises(ValueError, match="one name"):
        h.parse_rates(["a:x:p/msg/T:1", "a:y:p/msg/T:2"])
    assert h.parse_counts(["loc:/loc/status"]) == [("loc", "/loc/status")] and h.parse_counts([]) == []
    for wrong in ("loc", "loc:", ":/loc/status", "loc:/loc/status:extra"):
        with pytest.raises(ValueError, match="is not name:topic"):
            h.parse_counts([wrong])
