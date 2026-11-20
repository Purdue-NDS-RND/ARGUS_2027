"""The executor's side of the stale-data guard (FND-03), through the real node class.

With `time_health_guard` off the executor creates no monitor and no extra
subscription. With it on, the executor feeds a TimeHealth from the pose, the
target detection and MAVROS's time-sync status, and missions read the report
through MissionContext.time_health(). rclpy is replaced by the stand-ins of
test_executor_feature_overrides.py.
"""

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_executor_feature_overrides import FakeNode, executor_module, start  # noqa: E402, F401  (executor_module is a fixture)

FLAG = "time_health_guard"
POSE_TOPIC = "/mavros/local_position/pose"
DETECTION_TOPIC = "/drone_package_drop/target_detection"
TIMESYNC_TOPIC = "/mavros/timesync_status"


class Clock:
    def __init__(self) -> None:
        self.now_s = 100.0

    def now(self):
        return SimpleNamespace(nanoseconds=int(round(self.now_s * 1e9)))


def stamp(seconds: float):
    whole = int(seconds)
    return SimpleNamespace(sec=whole, nanosec=int(round((seconds - whole) * 1e9)))


def stamped(seconds: float):
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp(seconds)))


@pytest.fixture
def recording(monkeypatch):
    """Make the stand-in node record its subscriptions and run on a clock the test moves."""
    subscriptions, clock = [], Clock()

    def create_subscription(self, message_type, topic, callback, qos):
        subscriptions.append((topic, callback))

    monkeypatch.setattr(FakeNode, "create_subscription", create_subscription, raising=False)
    monkeypatch.setattr(FakeNode, "get_clock", lambda self: clock, raising=False)
    return subscriptions, clock


def callbacks(subscriptions, topic):
    return [callback for name, callback in subscriptions if name == topic]


def test_flag_off_adds_no_monitor_and_no_subscription(executor_module, monkeypatch, recording):
    subscriptions, _clock = recording
    node = start(executor_module, monkeypatch)
    assert node._time_health is None
    assert node._mission_context.time_health() is None
    topics = sorted(topic for topic, _ in subscriptions)
    assert TIMESYNC_TOPIC not in topics
    assert topics.count(POSE_TOPIC) == 1 and topics.count(DETECTION_TOPIC) == 1  # the 2026 ones only
    assert len(topics) == 6


def test_shipped_sequences_with_a_vision_mission_declare_the_flag_off():
    import yaml

    sequences = Path(__file__).resolve().parents[2] / "drone_mission_demo" / "config" / "sequences"
    declaring = []
    for path in sorted(sequences.glob("*.yaml")):
        root = yaml.safe_load(path.read_text())["mission_sequence"]
        vision = any(entry["type"] in ("package_drop", "package_delivery") for entry in root["missions"])
        if vision:
            assert root["features"].get(FLAG) is False, path.name
            declaring.append(path.name)
    assert len(declaring) == 8  # six 2026 files and the two generated sim copies of the live ones (CEN-11)


def test_flag_on_feeds_the_monitor_and_missions_read_it(executor_module, monkeypatch, recording):
    subscriptions, clock = recording
    node = start(executor_module, monkeypatch, feature_overrides=FLAG)
    assert node._time_health is not None
    for topic in (POSE_TOPIC, DETECTION_TOPIC):
        assert len(callbacks(subscriptions, topic)) == 2  # the executor's own, and the monitor's
    assert len(callbacks(subscriptions, TIMESYNC_TOPIC)) == 1

    context = node._mission_context
    first = context.time_health()
    assert first is not None and not first.healthy and len(first.reasons) == 3  # nothing heard yet

    def feed(pose_s, frame_s, offset_ns=5_000_000):
        for callback in callbacks(subscriptions, POSE_TOPIC):
            callback(stamped(pose_s))
        for callback in callbacks(subscriptions, DETECTION_TOPIC):
            callback(stamped(frame_s))
        callbacks(subscriptions, TIMESYNC_TOPIC)[0](SimpleNamespace(estimated_offset_ns=offset_ns, round_trip_time_ms=2.0))

    feed(99.98, 99.95)
    report = context.time_health()
    assert report.healthy, report.reasons
    assert report.pose_age_s == pytest.approx(0.02) and report.frame_age_s == pytest.approx(0.05)
    assert report.offset_ms == pytest.approx(5.0) and report.round_trip_ms == pytest.approx(2.0)

    clock.now_s = 100.3  # nothing new for 300 ms
    report = context.time_health()
    assert not report.healthy
    assert any("pose" in reason for reason in report.reasons) and any("frame" in reason for reason in report.reasons)

    feed(100.29, 100.28, offset_ns=5_000_000 + 30_000_000)  # fresh again, but the offset stepped by 30 ms
    report = context.time_health()
    assert not report.healthy and any("offset jumped" in reason for reason in report.reasons)

    clock.now_s = 101.4
    feed(101.39, 101.38, offset_ns=35_000_000)
    assert context.time_health().healthy


def test_flag_on_still_gives_the_executor_its_own_pose_and_detection(executor_module, monkeypatch, recording):
    subscriptions, _clock = recording
    node = start(executor_module, monkeypatch, feature_overrides=FLAG)
    pose, detection = stamped(99.99), stamped(99.99)
    for callback in callbacks(subscriptions, POSE_TOPIC):
        callback(pose)
    for callback in callbacks(subscriptions, DETECTION_TOPIC):
        callback(detection)
    assert node._local_pose is pose and node._target_detection is detection
    assert node.declared["loop_rate_hz"] == 20.0
