"""The delivery's soft set-down (CEN-05): the contact rule and the last descent's rate, with no ROS.

Contact is three things at once, held for 0.3 s: the rangefinder at or below
the package's height, no vertical speed, and the throttle fallen from what
it was while the vehicle was still coming down.
"""

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_mission_core import set_down as sd  # noqa: E402
from drone_mission_core.features import FLAG_REQUIRES  # noqa: E402

PACKAGE_M = 0.10
DESCENDING = sd.ContactReading(range_m=0.40, range_age_s=0.05, vertical_speed_mps=-0.12, throttle=0.40, throttle_age_s=0.05)
ON_THE_GROUND = sd.ContactReading(range_m=0.02, range_age_s=0.05, vertical_speed_mps=0.0, throttle=0.30, throttle_age_s=0.05)


def but(reading, **changes):
    return sd.ContactReading(**{**reading.__dict__, **changes})


def come_down(detector, seconds=3.0, start_s=10.0, throttle=0.40):
    """Feed the descent that gives the detector its picture of the throttle while descending. The time it ended."""
    now = start_s
    for _ in range(int(round(seconds / 0.05))):
        assert detector.update(but(DESCENDING, throttle=throttle), now) is False
        now += 0.05
    return now


def test_the_numbers_are_the_specs():
    assert (sd.HOLD_BELOW_M, sd.SET_DOWN_RATE_MPS, sd.CONTACT_HOLD_S, sd.AFTER_RELEASE_WAIT_S) == (0.6, 0.12, 0.3, 1.0)
    assert sd.SET_DOWN_FLAG == "delivery_soft_set_down"
    # The estimator's track, the centring on it, and the height estimator, whose rangefinder the contact rule reads.
    assert FLAG_REQUIRES[sd.SET_DOWN_FLAG] == ("use_target_estimator", "centre_on_estimator", "use_height_estimator")


def test_the_descent_slows_to_the_set_down_rate_at_the_height_where_the_position_is_held():
    assert sd.descent_rate(6.0, 0.4) == pytest.approx(0.4)  # the mission's own rate while high
    assert sd.descent_rate(1.0, 0.4) == pytest.approx(0.12 + 0.3 * 0.4)  # slower as 0.6 m comes near
    assert sd.descent_rate(0.6, 0.4) == pytest.approx(0.12)
    assert sd.descent_rate(0.2, 0.4) == pytest.approx(0.12)
    assert sd.descent_rate(6.0, 0.05) == pytest.approx(0.05)  # never faster than the mission's rate
    # A height that is not the rangefinder's is taken to be half a metre lower than it says.
    assert sd.working_height(1.1, from_rangefinder=True) == pytest.approx(1.1)
    assert sd.working_height(1.1, from_rangefinder=False) == pytest.approx(0.6)
    assert sd.SIGNAL_MAX_AGE_S < sd.CONTACT_HOLD_S


def test_contact_is_the_three_signals_together_for_three_tenths_of_a_second():
    detector = sd.ContactDetector(PACKAGE_M)
    now = come_down(detector)
    assert detector.baseline == pytest.approx(0.40)
    answers = []
    for _ in range(8):
        answers.append(detector.update(ON_THE_GROUND, now))
        now += 0.05
    assert answers == [False] * 6 + [True, True]  # 0.3 s after the first tick with all three
    assert detector.missing == ()


@pytest.mark.parametrize(
    "reading, missing",
    [
        (but(ON_THE_GROUND, range_m=0.25), "range"),  # still above the package's height
        (but(ON_THE_GROUND, range_m=None), "range"),  # no rangefinder heard
        (but(ON_THE_GROUND, range_age_s=0.25), "range"),  # or not lately: under the contact hold, so 0.3 s is never one sample
        (but(ON_THE_GROUND, vertical_speed_mps=-0.12), "speed"),  # still coming down
        (but(ON_THE_GROUND, vertical_speed_mps=None), "speed"),
        (but(ON_THE_GROUND, throttle=0.38), "throttle"),  # 5% under the descent's: not a drop
        (but(ON_THE_GROUND, throttle=None), "throttle"),
        (but(ON_THE_GROUND, throttle_age_s=0.25), "throttle"),
    ],
)
def test_any_one_signal_missing_is_no_contact(reading, missing):
    detector = sd.ContactDetector(PACKAGE_M)
    now = come_down(detector)
    for _ in range(20):
        assert detector.update(reading, now) is False
        now += 0.05
    assert missing in detector.missing


def test_the_rangefinder_reading_under_its_own_minimum_counts_as_at_the_package():
    # On the ground the sim's rangefinder reads 0.01 m and a TFmini reads under its 0.1 m: both are "at or below".
    detector = sd.ContactDetector(PACKAGE_M)
    now = come_down(detector)
    assert [detector.update(but(ON_THE_GROUND, range_m=0.0), now + 0.05 * i) for i in range(7)][-1] is True


def test_a_break_in_the_signals_or_in_the_ticks_starts_the_three_tenths_again():
    detector = sd.ContactDetector(PACKAGE_M)
    now = come_down(detector)
    for i in range(5):
        detector.update(ON_THE_GROUND, now + 0.05 * i)
    assert detector.update(but(ON_THE_GROUND, vertical_speed_mps=0.2), now + 0.25) is False  # a bounce
    assert [detector.update(ON_THE_GROUND, now + 0.30 + 0.05 * i) for i in range(7)] == [False] * 6 + [True]
    late = sd.ContactDetector(PACKAGE_M)
    now = come_down(late)
    late.update(ON_THE_GROUND, now)
    assert late.update(ON_THE_GROUND, now + 0.5) is False  # the ticks stopped for half a second: not one stretch


def test_without_a_throttle_seen_while_descending_there_is_no_contact():
    """The drop is judged against the throttle while the vehicle was coming down. A vehicle that was never seen
    descending (the feed started late) has nothing to compare with, and a low throttle alone proves nothing."""
    detector = sd.ContactDetector(PACKAGE_M)
    now = 10.0
    for _ in range(20):
        assert detector.update(ON_THE_GROUND, now) is False
        now += 0.05
    assert detector.baseline is None and "throttle" in detector.missing


def test_the_throttle_while_standing_still_does_not_become_the_baseline():
    # On the ground the throttle runs down to idle. If those samples moved the baseline, the drop would vanish.
    detector = sd.ContactDetector(PACKAGE_M)
    now = come_down(detector)
    for i in range(60):
        detector.update(but(ON_THE_GROUND, range_m=0.25, throttle=0.40 - 0.005 * i), now)  # no contact: range not there
        now += 0.05
    assert detector.baseline == pytest.approx(0.40)


def test_the_package_height_must_be_a_real_height():
    for bad in (0.0, -0.1, float("nan"), None, "tall", True):
        with pytest.raises(ValueError, match="package_height_m"):
            sd.ContactDetector(bad)
    with pytest.raises(ValueError, match="package_height_m"):
        sd.ContactDetector(0.9)  # above the height at which the set-down begins: every reading there would be "at the package"


def test_the_throttle_feed_keeps_the_newest_value_and_its_age():
    feed = sd.ThrottleFeed()
    assert feed.newest(5_000_000_000) is None
    feed.add(4_900_000_000, 0.42)
    feed.add(4_950_000_000, float("nan"))  # not a number: not kept
    feed.add(4_960_000_000, 1.7)  # nor outside 0 to 1
    value, age_s = feed.newest(5_000_000_000)
    assert value == pytest.approx(0.42) and age_s == pytest.approx(0.1)


class Node:
    def __init__(self):
        self.subscriptions = []
        self.lines = []

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscriptions.append((topic, callback))

    def get_logger(self):
        return SimpleNamespace(info=self.lines.append, error=self.lines.append)


def spec(**features):
    return SimpleNamespace(features=features)


def test_the_executor_listens_for_the_throttle_only_with_the_flag_on(monkeypatch):
    import types

    node = Node()
    assert sd.attach_throttle(node, [spec(), spec(use_target_estimator=True)]) is None and node.subscriptions == []
    # The ROS message packages, as far as attach_throttle imports them.
    monkeypatch.setitem(sys.modules, "rclpy", types.ModuleType("rclpy"))
    qos = types.ModuleType("rclpy.qos")
    qos.qos_profile_sensor_data = object()
    monkeypatch.setitem(sys.modules, "rclpy.qos", qos)
    monkeypatch.setitem(sys.modules, "mavros_msgs", types.ModuleType("mavros_msgs"))
    messages = types.ModuleType("mavros_msgs.msg")
    messages.VfrHud = type("VfrHud", (), {})
    monkeypatch.setitem(sys.modules, "mavros_msgs.msg", messages)
    feed = sd.attach_throttle(node, [spec(), spec(delivery_soft_set_down=True)])
    assert [topic for topic, _ in node.subscriptions] == ["/mavros/vfr_hud"] and "delivery_soft_set_down" in node.lines[-1]
    node.subscriptions[0][1](SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=7, nanosec=500_000_000)), throttle=0.37))
    assert feed.newest(7_600_000_000) == (pytest.approx(0.37), pytest.approx(0.1))
