"""PLT-09: the actuation lock's rule (drone_mission_core/actuation_lock.py). No ROS."""

import pytest

from drone_mission_core import actuation_lock as al
from drone_mission_core.actuation_lock import ActuationLock


def spec(type_name, **features):
    return type("Spec", (), {"type_name": type_name, "name": type_name, "features": features})()


def test_the_flag_has_one_name():
    assert al.FLAG == "actuation_lock"
    assert not al.lock_requested([spec("takeoff"), spec("package_drop", actuation_lock=False)])
    assert al.lock_requested([spec("takeoff"), spec("package_drop", actuation_lock=True)])


def test_nothing_is_commanded_without_a_permit():
    lock = ActuationLock()
    for actuator in al.ACTUATORS:
        allowed, why = lock.allows(actuator, 10.0)
        assert not allowed and "no release state has permitted" in why
    assert [r[0] for r in lock.refused] == list(al.ACTUATORS)


@pytest.mark.parametrize("state", ["DROP_PAYLOAD", "GROUND_DWELL", "ENGAGE", "SET_DOWN"])
def test_a_release_state_whose_gates_hold_may_command_its_actuator_on_that_tick(state):
    lock = ActuationLock()
    assert lock.permit("gripper", state, True, 10.0) is True
    assert lock.allows("gripper", 10.0) == (True, "")
    # The permit was for that command: a second one needs its own.
    assert lock.allows("gripper", 10.01)[0] is False


@pytest.mark.parametrize("state", ["TRACK_AND_DESCEND", "FINAL_ALIGN", "TRANSIT_TO_TARGET", "CONE_DESCENT", "RETURN", "CLIMB", "COMPLETE", "", None])
def test_a_state_that_is_not_a_release_state_cannot_permit(state):
    lock = ActuationLock()
    assert lock.permit("gripper", state, True, 10.0) is False
    allowed, why = lock.allows("gripper", 10.0)
    assert not allowed and "not a release state" in why


def test_gates_that_do_not_hold_cannot_permit():
    lock = ActuationLock()
    assert lock.permit("sprayer", "DROP_PAYLOAD", False, 10.0, "the track is 0.4 s old") is False
    allowed, why = lock.allows("sprayer", 10.0)
    assert not allowed and "gates do not hold" in why and "0.4 s old" in why
    # Anything but True is "do not hold": a rule that returned None, or a truthy string, has not said yes.
    for said in (None, 1, "yes"):
        assert lock.permit("sprayer", "DROP_PAYLOAD", said, 11.0) is False


def test_a_permit_is_for_its_own_actuator_and_goes_stale():
    lock = ActuationLock()
    lock.permit("gripper", "GROUND_DWELL", True, 10.0)
    assert lock.allows("sprayer", 10.0)[0] is False  # the gripper's permit is not the sprayer's
    assert lock.allows("gripper", 10.0 + al.PERMIT_LIFE_S + 0.01)[0] is False  # given two ticks ago: the gates may have moved
    lock.permit("gripper", "GROUND_DWELL", True, 20.0)
    assert lock.allows("gripper", 20.0 + al.PERMIT_LIFE_S)[0] is True
    lock.permit("gripper", "GROUND_DWELL", True, 30.0)
    assert lock.allows("gripper", 29.0)[0] is False  # a clock that went backwards


def test_a_later_refusal_takes_an_earlier_permit_away():
    lock = ActuationLock()
    lock.permit("gripper", "SET_DOWN", True, 10.0)
    lock.permit("gripper", "SET_DOWN", False, 10.05, "the rangefinder is not healthy")
    assert lock.allows("gripper", 10.05)[0] is False


def test_an_actuator_the_lock_does_not_know_is_refused():
    lock = ActuationLock()
    with pytest.raises(ValueError, match="winch"):
        lock.permit("winch", "DROP_PAYLOAD", True, 1.0)
    assert lock.allows("winch", 1.0)[0] is False


def test_every_actuating_mission_of_a_locked_sequence_needs_its_gated_release_path():
    on = {"actuation_lock": True}
    assert al.lock_violations([spec("takeoff", **on), spec("rtl", **on), spec("search_pattern", **on)]) == []
    assert al.lock_violations([spec("package_drop", actuation_lock=False)]) == []  # the lock off: 2026's release, untouched
    problems = al.lock_violations([spec("package_drop", **on), spec("package_delivery", **on), spec("package_recovery", **on)])
    assert len(problems) == 3
    assert "drop_release_gates" in problems[0] and "delivery_soft_set_down" in problems[1] and "package_recovery" in problems[2]
    assert al.lock_violations([spec("package_drop", **on, drop_release_gates=True), spec("package_delivery", **on, delivery_soft_set_down=True), spec("package_recovery", **on, package_recovery=True)]) == []
    assert set(al.GATED_RELEASE_FLAG) == {"package_drop", "package_delivery", "package_recovery"}
