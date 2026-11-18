"""The executor's side of the target estimator (FND-06), through the real node class.

With `use_target_estimator` off the executor makes nothing new. With it on
(and `use_frame_time_pose`, which it needs) the executor listens to the
detection contract, runs the estimator in-process and publishes the tracks.
rclpy is replaced by the stand-ins of test_executor_feature_overrides.py.
"""

import math
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from stand_in_messages import FakeMessages as Messages3D  # noqa: E402
from test_executor_feature_overrides import FakeNode, executor_module, start  # noqa: E402, F401  (executor_module is a fixture)

ROOT = Path(__file__).resolve().parents[3]
FLAGS = "use_frame_time_pose,use_target_estimator"
POSE_TOPIC = "/mavros/local_position/pose"
VELOCITY_TOPIC = "/mavros/local_position/velocity_local"
CONTRACT_TOPIC = "/perception/target_cv/detections"
IMAGE_SIZE_TOPIC = "/drone_package_drop/image_size"
TARGETS_TOPIC = "/estimator/targets"
MS = 1_000_000
S = 1_000_000_000
BASE = 300 * S
HEIGHT = 15.0


def stamp(ns: int):
    return SimpleNamespace(sec=ns // S, nanosec=ns % S)


def pose_message(ns: int, east: float = 0.0, north: float = 0.0):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp(ns)),
        pose=SimpleNamespace(position=SimpleNamespace(x=east, y=north, z=HEIGHT), orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)),
    )


def velocity_message(ns: int):
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp(ns)), twist=SimpleNamespace(linear=SimpleNamespace(x=0.0, y=0.0, z=0.0)))


def size_message(width: int, height: int):
    return SimpleNamespace(point=SimpleNamespace(x=float(width), y=float(height), z=0.0))


def say_image_size(wiring, width: int = 640, height: int = 360):
    """The 2026 detector's image size, to the estimator's own subscription (the last one made on that topic)."""
    wiring.callbacks(IMAGE_SIZE_TOPIC)[-1](size_message(width, height))


def contract_message(ns: int, pixel, class_id="bullseye", score=0.9, hypotheses=1):
    """A vision_msgs/Detection2DArray as the contract has it, with one detection."""
    header = SimpleNamespace(stamp=stamp(ns), frame_id="down_camera_optical")
    result = SimpleNamespace(hypothesis=SimpleNamespace(class_id=class_id, score=score))
    box = SimpleNamespace(center=SimpleNamespace(position=SimpleNamespace(x=pixel[0], y=pixel[1]), theta=0.0), size_x=60.0, size_y=60.0)
    return SimpleNamespace(header=header, detections=[SimpleNamespace(header=header, results=[result] * hypotheses, bbox=box)])


class Messages(Messages3D):
    Detection2DArray = object  # only named, as the subscription's type


class Wiring:
    """What the node made: subscriptions, publishers and timers, and a clock the test sets."""

    def __init__(self):
        self.subscriptions, self.publishers, self.timers = [], {}, []
        self.now_ns = BASE

    def callbacks(self, topic):
        return [callback for name, callback in self.subscriptions if name == topic]


@pytest.fixture
def wiring(monkeypatch):
    made = Wiring()

    def create_publisher(self, message_type, topic, qos):
        made.publishers[topic] = SimpleNamespace(sent=[])
        made.publishers[topic].publish = made.publishers[topic].sent.append
        return made.publishers[topic]

    monkeypatch.setattr(FakeNode, "create_subscription", lambda self, message_type, topic, callback, qos: made.subscriptions.append((topic, callback)), raising=False)
    monkeypatch.setattr(FakeNode, "create_publisher", create_publisher, raising=False)
    monkeypatch.setattr(FakeNode, "create_timer", lambda self, period, callback: made.timers.append((period, callback)), raising=False)
    monkeypatch.setattr(FakeNode, "get_clock", lambda self: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=made.now_ns)), raising=False)
    monkeypatch.setitem(sys.modules, "vision_msgs", types.SimpleNamespace(msg=Messages))
    monkeypatch.setitem(sys.modules, "vision_msgs.msg", Messages)
    return made


def test_flag_off_makes_nothing_new(executor_module, monkeypatch, wiring):
    node = start(executor_module, monkeypatch)
    assert node._target_estimator is None
    topics = [topic for topic, _ in wiring.subscriptions]
    assert len(topics) == 6 and CONTRACT_TOPIC not in topics
    assert TARGETS_TOPIC not in wiring.publishers and len(wiring.publishers) == 4
    assert len(wiring.timers) == 1 and wiring.timers[0][0] == pytest.approx(1.0 / 20.0)  # the 20 Hz tick, alone
    assert "estimator_detectors" not in node.declared
    # Nor is any of the estimator's numpy code imported: flags off, the executor loads what it loaded in 2026.
    for module in ("target_estimator", "assignment", "projection_v2", "frame_projection"):
        assert f"drone_mission_core.{module}" not in sys.modules, module


def test_every_shipped_sequence_declares_the_flag_off():
    import yaml

    sequences = sorted((ROOT / "src" / "drone_mission_demo" / "config" / "sequences").glob("*.yaml"))
    assert len(sequences) == 10
    for path in sequences:
        assert yaml.safe_load(path.read_text())["mission_sequence"]["features"]["use_target_estimator"] is False, path.name


def test_the_estimator_needs_the_pose_history(executor_module, monkeypatch, wiring):
    with pytest.raises(ValueError, match="use_target_estimator requires use_frame_time_pose"):
        start(executor_module, monkeypatch, feature_overrides="use_target_estimator")


def test_flag_on_runs_the_estimator_in_the_executor_from_the_contract_topic_to_the_targets_topic(executor_module, monkeypatch, wiring):
    node = start(executor_module, monkeypatch, feature_overrides=FLAGS)
    feed = node._target_estimator
    assert feed is not None and node.declared["estimator_detectors"] == ["target_cv"]
    assert len(wiring.callbacks(CONTRACT_TOPIC)) == 1 and len(wiring.callbacks(POSE_TOPIC)) == 3
    periods = sorted(period for period, _ in wiring.timers)
    assert node.declared["loop_rate_hz"] == 20.0 and periods == pytest.approx([1.0 / 20.0, 0.1])  # the 20 Hz tick, and the estimator's own 10 Hz
    (on_timer,) = [callback for period, callback in wiring.timers if period == pytest.approx(0.1)]

    from drone_mission_core.frame_projection import load_camera
    from drone_mission_core.projection_v2 import pixel_of_ground_point

    camera, mount = load_camera("sim_delivery", "sim_iris_downcam")  # the camera package_delivery_demo.yaml names
    target = (2.0, -1.5, 0.0)

    def fly(until_ns):
        while wiring.now_ns <= until_ns:
            for callback in wiring.callbacks(POSE_TOPIC):
                callback(pose_message(wiring.now_ns))
            wiring.callbacks(VELOCITY_TOPIC)[0](velocity_message(wiring.now_ns))
            wiring.now_ns += 33 * MS

    fly(BASE + 200 * MS)
    say_image_size(wiring)
    for index in range(3):  # three frames, each handled 40 ms after it was taken
        frame_ns = BASE + (100 + index * 100) * MS
        fly(frame_ns + 40 * MS)
        pixel = pixel_of_ground_point(camera, mount, target, (0.0, 0.0, HEIGHT), (0.0, 0.0, 0.0, 1.0))
        wiring.callbacks(CONTRACT_TOPIC)[0](contract_message(frame_ns, pixel))

    track = feed.primary("bullseye", wiring.now_ns)
    assert track.state == "CONFIRMED" and (track.east_m, track.north_m) == pytest.approx(target[:2], abs=0.01)
    assert track.sigma_m == pytest.approx(HEIGHT * math.radians(1.0) / math.sqrt(3.0), rel=0.2)  # three hits of about 0.26 m each

    on_timer()
    (array,) = wiring.publishers[TARGETS_TOPIC].sent
    (detection,) = array.detections
    assert detection.id == f"bullseye/{track.track_id}/CONFIRMED" and array.header.frame_id == "map"
    position = detection.results[0].pose.pose.position
    assert (position.x, position.y) == pytest.approx(target[:2], abs=0.01)
    # The executor's own 2026 state is as it was: the latest pose, and no target point from this topic.
    assert node._local_pose.pose.position.z == HEIGHT and node._target_detection is None


def test_a_message_that_is_not_on_the_contract_is_dropped_and_said_so(executor_module, monkeypatch, wiring):
    node = start(executor_module, monkeypatch, feature_overrides=FLAGS)
    wiring.callbacks(CONTRACT_TOPIC)[0](contract_message(BASE, (320.0, 180.0), class_id="red_thing"))
    wiring.callbacks(CONTRACT_TOPIC)[0](contract_message(BASE, (320.0, 180.0), hypotheses=2))
    assert node._target_estimator.counters["dropped: not on the detection contract"] == 2
    assert node._target_estimator.pending == 0 and node.logger.error.call_count == 2


def test_a_fault_inside_the_estimator_is_counted_and_never_reaches_the_executor(executor_module, monkeypatch, wiring):
    node = start(executor_module, monkeypatch, feature_overrides=FLAGS)
    feed = node._target_estimator

    def broken(*_args, **_kwargs):
        raise RuntimeError("a bug in the estimator")

    monkeypatch.setattr(feed, "step", broken)
    monkeypatch.setattr(feed, "add_pose", broken)
    say_image_size(wiring)
    wiring.callbacks(CONTRACT_TOPIC)[0](contract_message(BASE, (320.0, 180.0)))  # none of these may raise
    wiring.callbacks(POSE_TOPIC)[2](pose_message(BASE))
    (on_timer,) = [callback for period, callback in wiring.timers if period == pytest.approx(0.1)]
    on_timer()
    assert feed.counters["failed: handling a detection message"] == 1 and feed.counters["failed: handling a pose"] == 1
    assert feed.counters["failed: publishing the tracks"] == 1 and wiring.publishers[TARGETS_TOPIC].sent == []
    assert "a bug in the estimator" in str(node.logger.error.call_args_list[0])


def test_the_2026_detector_is_used_only_while_its_image_is_known_to_be_the_calibrated_size(executor_module, monkeypatch, wiring):
    node = start(executor_module, monkeypatch, feature_overrides=FLAGS)
    feed = node._target_estimator
    assert len(wiring.callbacks(IMAGE_SIZE_TOPIC)) == 2  # the executor's own, and the estimator's
    wiring.callbacks(CONTRACT_TOPIC)[0](contract_message(BASE, (320.0, 180.0)))
    assert feed.pending == 0 and feed.counters["dropped: the detector has not said what size its image is"] == 1
    say_image_size(wiring, 1280, 720)  # sim_delivery is calibrated at 640 x 360
    wiring.callbacks(CONTRACT_TOPIC)[0](contract_message(BASE, (320.0, 180.0)))
    assert feed.pending == 0 and feed.counters["dropped: the detector's image is not the calibrated mode's size"] == 1
    assert "1280 x 720" in str(node.logger.error.call_args_list[-1])
    say_image_size(wiring)
    node._image_size = None  # a mission clearing its own copy between missions does not blind the estimator
    wiring.callbacks(CONTRACT_TOPIC)[0](contract_message(BASE, (320.0, 180.0)))
    assert feed.pending == 1  # taken, and waiting for its pose


def test_the_executor_does_not_start_when_no_mission_names_a_camera(executor_module, monkeypatch, wiring):
    waypoints_only = ROOT / "src" / "drone_mission_demo" / "config" / "sequences" / "square_only.yaml"
    with pytest.raises(ValueError, match="use_target_estimator is on but no mission's config names camera_mode and airframe"):
        start(executor_module, monkeypatch, sequence_file=str(waypoints_only), feature_overrides=FLAGS)
    assert start(executor_module, monkeypatch, sequence_file=str(waypoints_only))._target_estimator is None
