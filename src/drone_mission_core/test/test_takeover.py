"""PLT-09: the takeover latch's rule (drone_mission_core/takeover.py). No ROS."""

import pytest

from drone_mission_core import takeover
from drone_mission_core.takeover import TakeoverLatch


def flying(latch, t=100.0):
    """Autonomy has taken off: GUIDED was asked for (the take-off service's) and seen, and the vehicle is armed."""
    latch.observe("STABILIZE", False, t - 5.0)
    latch.commanded("GUIDED", t - 3.0)
    latch.observe("GUIDED", False, t - 2.0)
    latch.observe("GUIDED", True, t)
    assert not latch.latched
    return latch


def test_the_flag_has_one_name_and_reads_from_the_sequence():
    assert takeover.FLAG == "takeover_latch"
    spec = lambda **features: type("Spec", (), {"features": features})()  # noqa: E731
    assert not takeover.latch_requested([spec(), spec(takeover_latch=False)])
    assert takeover.latch_requested([spec(), spec(takeover_latch=True)])


@pytest.mark.parametrize("mode", ["LOITER", "STABILIZE", "ALT_HOLD", "RTL", "LAND", "BRAKE"])
def test_a_mode_autonomy_did_not_ask_for_is_a_takeover_and_it_stays_one(mode):
    latch = flying(TakeoverLatch())
    assert latch.observe(mode, True, 110.0) is True
    assert latch.latched and "GUIDED" in latch.reason and mode in latch.reason
    # Going back to GUIDED by itself does not hand the vehicle back: only a re-arm by hand does.
    assert latch.observe("GUIDED", True, 111.0) is True
    assert latch.observe("GUIDED", True, 500.0) is True


def test_a_mode_autonomy_asked_for_is_not_a_takeover():
    latch = flying(TakeoverLatch())
    latch.commanded("RTL", 110.0)
    assert latch.observe("RTL", True, 110.4) is False
    latch.commanded("LAND", 120.0)
    assert latch.observe("LAND", True, 129.9) is False  # late, but inside the ten seconds
    assert not latch.latched


def test_a_mode_asked_for_long_ago_is_no_longer_ours():
    latch = flying(TakeoverLatch())
    latch.commanded("RTL", 110.0)
    assert latch.observe("GUIDED", True, 111.0) is False  # nothing changed: the request was not taken
    assert latch.observe("RTL", True, 110.0 + takeover.OURS_WITHIN_S + 0.1) is True  # that is somebody else's RTL (a failsafe's, the pilot's)


def test_the_same_mode_seen_again_is_nothing_and_no_state_teaches_nothing():
    latch = flying(TakeoverLatch())
    for t in range(200):
        assert latch.observe("GUIDED", True, 100.0 + t * 0.05) is False
    assert latch.observe(None, True, 120.0) is False and latch.observe("", True, 120.1) is False  # MAVROS between heartbeats
    assert latch.observe("GUIDED", True, 120.2) is False  # and GUIDED again is still the mode it was


def test_modes_set_before_autonomy_has_the_vehicle_or_on_the_ground_are_the_crews_own():
    latch = TakeoverLatch()
    for mode in ("STABILIZE", "LOITER", "GUIDED", "STABILIZE"):  # the pilot's pre-flight checks, armed or not
        assert latch.observe(mode, True, 10.0) is False
    latch = flying(TakeoverLatch())
    assert latch.observe("STABILIZE", False, 300.0) is False  # after landing, disarmed
    assert not latch.latched


def test_a_takeover_in_the_tick_the_vehicle_arms_is_still_one():
    latch = TakeoverLatch()
    latch.commanded("GUIDED", 1.0)
    latch.observe("GUIDED", True, 2.0)
    assert latch.observe("LOITER", True, 2.05) is True


def test_rearming_by_hand_needs_guided_and_gives_the_vehicle_back():
    latch = flying(TakeoverLatch())
    assert latch.rearm("GUIDED", 105.0) == (False, "autonomy is not latched out: there is nothing to re-arm")
    latch.observe("LOITER", True, 110.0)
    ok, why = latch.rearm("LOITER", 120.0)
    assert not ok and "GUIDED" in why and latch.latched
    ok, why = latch.rearm(None, 121.0)
    assert not ok and latch.latched
    ok, why = latch.rearm("GUIDED", 125.0)
    assert ok and not latch.latched and latch.takeovers == 1
    assert latch.observe("GUIDED", True, 125.05) is False
    # And the next one is a takeover again.
    assert latch.observe("ALT_HOLD", True, 130.0) is True and latch.takeovers == 2
