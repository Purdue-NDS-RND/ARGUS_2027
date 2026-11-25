"""CEN-09: the estimator's target as ArduPilot's LANDING_TARGET (precision_loiter.py)."""

import math
import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_mission_core import precision_loiter as pl  # noqa: E402

LEVEL_EAST = (0.0, 0.0, 0.0, 1.0)  # nose to the east
LEVEL_NORTH = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))  # nose to the north


def los_as_ardupilot_reads_it(message):
    """(forward, right, down) of the line of sight, as the flight controller's companion backend makes it from the two angles."""
    return -math.tan(message.angle_y_rad), math.tan(message.angle_x_rad), 1.0


def test_a_target_ahead_and_to_the_right_is_ahead_and_to_the_right_in_the_message():
    # Nose east, 6 m up, the target 1.2 m east (ahead) and 0.6 m south (to the right).
    message = pl.landing_target((1.2, -0.6, 0.0), (0.0, 0.0, 6.0), LEVEL_EAST)
    assert (message.forward_m, message.right_m, message.down_m) == pytest.approx((1.2, 0.6, 6.0))
    assert message.angle_x_rad == pytest.approx(math.atan2(0.6, 6.0)) and message.angle_y_rad == pytest.approx(-math.atan2(1.2, 6.0))
    assert message.distance_m == pytest.approx(math.sqrt(1.2**2 + 0.6**2 + 36.0))
    forward, right, down = los_as_ardupilot_reads_it(message)
    assert (forward * 6.0, right * 6.0) == pytest.approx((1.2, 0.6))  # read back the flight controller's way: the same place
    # The same target with the nose to the north: now it is to the right (east) and behind (south).
    message = pl.landing_target((1.2, -0.6, 0.0), (0.0, 0.0, 6.0), LEVEL_NORTH)
    assert (message.forward_m, message.right_m) == pytest.approx((-0.6, 1.2))


def test_the_bodys_tilt_is_in_the_message_so_that_the_flight_controller_can_take_it_out_again():
    # Directly over the target and pitched nose down by 10 degrees (a turn about the body's left axis, the world's north with the nose east):
    # the target is then ahead of the body's own "down" by 10 degrees. The flight controller turns the line of sight back with its attitude.
    pitch = math.radians(10.0)
    nose_down = (0.0, math.sin(pitch / 2.0), 0.0, math.cos(pitch / 2.0))
    message = pl.landing_target((0.0, 0.0, 0.0), (0.0, 0.0, 5.0), nose_down)
    assert math.atan2(message.forward_m, message.down_m) == pytest.approx(-pitch, abs=1e-9) or math.atan2(message.forward_m, message.down_m) == pytest.approx(pitch, abs=1e-9)
    assert message.right_m == pytest.approx(0.0, abs=1e-9) and message.distance_m == pytest.approx(5.0)
    # Whichever way the pitch's sign reads, turning the body vector back by the same attitude gives straight down in the world.
    x, y, z, w = nose_down
    fwd, left, up = message.forward_m, -message.right_m, -message.down_m
    # rotate (fwd, left, up) by the attitude: about the y axis by `pitch`
    east = fwd * math.cos(pitch) + up * math.sin(pitch)
    upward = -fwd * math.sin(pitch) + up * math.cos(pitch)
    assert (east, left, upward) == pytest.approx((0.0, 0.0, -5.0), abs=1e-9)


@pytest.mark.parametrize(
    "target, vehicle, attitude, age, why",
    [
        ((0.0, 0.0, 0.0), (0.0, 0.0, 6.0), LEVEL_EAST, 0.8, "a track older than half a second"),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.2), LEVEL_EAST, 0.0, "not far enough under the vehicle"),
        ((0.0, 0.0, 6.5), (0.0, 0.0, 6.0), LEVEL_EAST, 0.0, "above the vehicle"),
        ((20.0, 0.0, 0.0), (0.0, 0.0, 6.0), LEVEL_EAST, 0.0, "far off to the side: over 60 degrees from straight down"),
        ((float("nan"), 0.0, 0.0), (0.0, 0.0, 6.0), LEVEL_EAST, 0.0, "not a number"),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 6.0), (0.0, 0.0, 0.0, 0.0), 0.0, "no attitude"),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 6.0), LEVEL_EAST, -1.0, "an age that is not one"),
    ],
)
def test_nothing_is_sent_for_a_target_that_is_old_not_under_the_vehicle_or_not_numbers(target, vehicle, attitude, age, why):
    assert pl.landing_target(target, vehicle, attitude, age) is None, why
    assert pl.FLAG == "precision_loiter_bridge" and pl.MAV_FRAME_BODY_FRD == 12 and pl.MAX_AGE_S == 0.5


# --- the bridge on the executor --------------------------------------------------------------------------------


class _Spec:
    def __init__(self, features):
        self.features = features


class _Node:
    """What attach_precision_loiter needs of the executor: a logger, a clock, a publisher and a timer."""

    def __init__(self):
        self.published, self.timers, self.said = [], [], []
        self.now_ns = 10_000_000_000
        self._mavros_state = types.SimpleNamespace(mode="LOITER")

    def get_logger(self):
        node = self

        class Logger:
            def warn(self, text, **_):
                node.said.append(text)

            error = info = warn

        return Logger()

    def get_clock(self):
        node = self

        class Stamp:
            nanoseconds = node.now_ns

            def to_msg(self):
                return types.SimpleNamespace(sec=node.now_ns // 10**9, nanosec=node.now_ns % 10**9)

        return types.SimpleNamespace(now=lambda: Stamp())

    def create_publisher(self, _type, topic, _depth):
        node = self
        self.topic = topic
        return types.SimpleNamespace(publish=node.published.append)

    def create_timer(self, period, callback):
        self.timers.append((period, callback))


class _Feed:
    def __init__(self, estimate):
        self.estimate = estimate

    def primary(self, class_id, _now_ns, near=None):
        return self.estimate if class_id == "bullseye" else None


class _Poses:
    def __init__(self, pose, stamp_ns=9_950_000_000):
        self.pose, self.stamp_ns, self.asked = pose, stamp_ns, []

    def pose_at(self, ns):
        self.asked.append(ns)
        return self.pose if ns == self.stamp_ns else None  # (the real one would extrapolate; the bridge asks only for the newest sample itself)

    def newest_ns(self):
        return self.stamp_ns


def _with_fake_messages(monkeypatch, track):
    """mavros_msgs and the estimator's track reader stood in for, so the bridge runs without ROS."""

    class Message:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=0), frame_id="")
            self.pose = types.SimpleNamespace(position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0), orientation=types.SimpleNamespace(w=0.0))

    package, messages = types.ModuleType("mavros_msgs"), types.ModuleType("mavros_msgs.msg")
    messages.LandingTarget = Message
    monkeypatch.setitem(sys.modules, "mavros_msgs", package)
    monkeypatch.setitem(sys.modules, "mavros_msgs.msg", messages)
    from drone_mission_core import target_access

    monkeypatch.setattr(target_access, "from_estimate", lambda _estimate: track)


def test_flag_off_the_bridge_makes_nothing_and_flag_on_it_sends_only_a_confirmed_track_under_the_vehicle(monkeypatch):
    node = _Node()
    assert pl.attach_precision_loiter(node, [_Spec({"use_target_estimator": True})], _Feed(object()), _Poses(None)) is None
    assert node.timers == [] and node.published == [] and not hasattr(node, "topic")
    on = [_Spec({"use_frame_time_pose": True, "use_target_estimator": True, "precision_loiter_bridge": True})]
    # The flag on with no estimator running: said, and nothing made.
    assert pl.attach_precision_loiter(node, on, None, None) is None and "not running" in node.said[-1] and node.timers == []

    track = types.SimpleNamespace(state="CONFIRMED", position=(1.2, -0.6, 0.0), age_s=0.1)
    _with_fake_messages(monkeypatch, track)
    pose = types.SimpleNamespace(position=(0.0, 0.0, 6.0), orientation=LEVEL_EAST)
    poses = _Poses(pose)
    counts = pl.attach_precision_loiter(node, on, _Feed(object()), poses)
    assert node.topic == "/mavros/landing_target/raw" and node.timers[0][0] == pytest.approx(0.1) and "benchmark" in node.said[-1]
    node.timers[0][1]()
    message = node.published[-1]
    assert (message.frame, message.type, message.target_num) == (12, 3, 0) and message.distance == pytest.approx(math.sqrt(1.2**2 + 0.6**2 + 36.0))
    assert message.angle == pytest.approx([math.atan2(0.6, 6.0), -math.atan2(1.2, 6.0)])
    # The position in the body's forward-left-up axes, which MAVROS turns into the flight controller's forward-right-down.
    assert (message.pose.position.x, message.pose.position.y, message.pose.position.z) == pytest.approx((1.2, -0.6, -6.0))
    # A track that is only remembered, an old one, and none at all: nothing is sent, so the flight controller loses the target when we have.
    for state, age in (("COASTING", 0.1), ("CONFIRMED", 0.9), ("LOST", 0.1)):
        track.state, track.age_s = state, age
        node.timers[0][1]()
    assert len(node.published) == 1 and counts["sent"] == 1 and counts["no confirmed track"] == 2 and counts["not under the vehicle"] == 1
    # The message carries the time of the pose it was made from, and that pose is the newest sample itself.
    assert (message.header.stamp.sec, message.header.stamp.nanosec) == (9, 950_000_000) and set(poses.asked) == {9_950_000_000}
    # A pose over 0.2 s old (the stream stalled), or one from the future: nothing is sent on old geometry.
    track.state, track.age_s = "CONFIRMED", 0.1
    for stamp in (10_000_000_000 - 250_000_000, 10_000_000_000 + 50_000_000):
        poses.stamp_ns = stamp
        node.timers[0][1]()
    assert len(node.published) == 1 and counts["no pose"] == 2
    poses.stamp_ns = 9_950_000_000
    # Any mode but GUIDED and LOITER (a return, a landing, the pilot's): nothing is sent, so no landing is drawn onto the target.
    for mode in ("RTL", "LAND", "ALT_HOLD", None):
        node._mavros_state.mode = mode
        node.timers[0][1]()
    assert len(node.published) == 1 and counts["another mode"] == 4
    node._mavros_state.mode = "GUIDED"
    node.timers[0][1]()
    assert len(node.published) == 2 and pl.SENT_IN_MODES == ("GUIDED", "LOITER") and pl.MAX_POSE_AGE_S == 0.2
