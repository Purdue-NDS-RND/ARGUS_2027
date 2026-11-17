"""The executor's side of the pose history buffer (FND-04), through the real node class.

With `use_frame_time_pose` off the executor keeps no pose history and adds no
subscription. With it on, a PoseBuffer is fed from MAVROS's pose and
velocity. rclpy is replaced by the stand-ins of test_executor_feature_overrides.py.
"""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_executor_feature_overrides import FakeNode, executor_module, start  # noqa: E402, F401  (executor_module is a fixture)

FLAG = "use_frame_time_pose"
POSE_TOPIC = "/mavros/local_position/pose"
VELOCITY_TOPIC = "/mavros/local_position/velocity_local"
MS = 1_000_000


def stamp(ns: int):
    return SimpleNamespace(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def pose_message(ns: int, east: float, yaw_deg: float = 0.0):
    half = math.radians(yaw_deg) / 2.0
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp(ns)),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=east, y=2.0, z=10.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half)),
        ),
    )


def velocity_message(ns: int, east_mps: float):
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp(ns)), twist=SimpleNamespace(linear=SimpleNamespace(x=east_mps, y=0.0, z=0.0)))


@pytest.fixture
def subscriptions(monkeypatch):
    made = []
    monkeypatch.setattr(FakeNode, "create_subscription", lambda self, message_type, topic, callback, qos: made.append((topic, callback)), raising=False)
    return made


def callbacks(subscriptions, topic):
    return [callback for name, callback in subscriptions if name == topic]


def test_flag_off_keeps_no_pose_history_and_adds_no_subscription(executor_module, monkeypatch, subscriptions):
    node = start(executor_module, monkeypatch)
    assert node._pose_buffer is None
    topics = [topic for topic, _ in subscriptions]
    assert VELOCITY_TOPIC not in topics and topics.count(POSE_TOPIC) == 1 and len(topics) == 6


def test_every_shipped_sequence_declares_the_flag_off():
    import yaml

    sequences = sorted((Path(__file__).resolve().parents[2] / "drone_mission_demo" / "config" / "sequences").glob("*.yaml"))
    assert len(sequences) == 10
    for path in sequences:
        assert yaml.safe_load(path.read_text())["mission_sequence"]["features"][FLAG] is False, path.name


def test_flag_on_feeds_the_buffer_from_mavros_and_it_answers_for_a_past_instant(executor_module, monkeypatch, subscriptions):
    node = start(executor_module, monkeypatch, feature_overrides=FLAG)
    buffer = node._pose_buffer
    assert buffer is not None and len(buffer) == 0
    assert len(callbacks(subscriptions, POSE_TOPIC)) == 2 and len(callbacks(subscriptions, VELOCITY_TOPIC)) == 1

    base = 200_000_000_000
    for index in range(11):  # 30 Hz, moving east at 1 m/s and yawing 30 deg/s
        ns = base + index * 33 * MS
        t = index * 0.033
        for callback in callbacks(subscriptions, POSE_TOPIC):
            callback(pose_message(ns, east=t, yaw_deg=30.0 * t))
        callbacks(subscriptions, VELOCITY_TOPIC)[0](velocity_message(ns, 1.0))
    assert len(buffer) == 11

    pose = buffer.pose_at(base + 150 * MS)  # between samples, 180 ms before the newest
    assert pose.position == pytest.approx((0.150, 2.0, 10.0), abs=1e-6)
    yaw = math.degrees(2.0 * math.atan2(pose.orientation[2], pose.orientation[3]))
    assert yaw == pytest.approx(4.5, abs=0.01)
    ahead = buffer.pose_at(base + 330 * MS + 20 * MS)  # 20 ms past the newest: carried on by the velocity
    assert ahead.extrapolated and ahead.position[0] == pytest.approx(0.350, abs=1e-6)
    assert buffer.pose_at(base + 330 * MS + 40 * MS) is None
    # The executor's own latest pose is still the 2026 one, untouched.
    assert node._local_pose.pose.position.x == pytest.approx(0.330)


def test_flag_on_leaves_the_tick_and_the_sequence_alone(executor_module, monkeypatch, subscriptions):
    node = start(executor_module, monkeypatch, feature_overrides=FLAG)
    assert node.declared["loop_rate_hz"] == 20.0
    assert [mission.name for mission in node._sequence] == ["takeoff", "package_delivery", "rtl"]
