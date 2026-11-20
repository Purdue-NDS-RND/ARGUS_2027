"""The retuned centring controller (CEN-02): the arithmetic of one step, and what its settings may be. No ROS, no numpy."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_mission_core import centring_controller as cc  # noqa: E402
from drone_mission_core import safety_limits  # noqa: E402

TICK = 0.05


def settled(controller, east, north, height=6.0, ticks=400):
    """The command after the slew limit has had time to let it through."""
    t = 0.0
    command = (0.0, 0.0)
    for _ in range(ticks):
        t += TICK
        command = controller.step(east, north, height, t)
    return command


def test_the_defaults_are_the_specs():
    gains = cc.CentringGains()
    assert (gains.kp, gains.soft_deadband_m, gains.slew_mps2) == (0.6, 0.03, 0.5)
    assert (gains.speed_cap_per_m_height, gains.speed_cap_min_mps, gains.speed_cap_max_mps) == (0.08, 0.25, 1.0)
    assert gains.lead_s == 0.5  # not the spec's: added after the first Gazebo holds in strong wind
    assert cc.RETUNE_FLAG == "centring_retune"


def test_speed_cap_is_eight_hundredths_of_the_height_between_a_quarter_and_one_metre_a_second():
    gains = cc.CentringGains()
    assert cc.speed_cap(2.4, gains) == 0.25  # 0.19 would be too slow to hold against wind: the floor
    assert cc.speed_cap(6.0, gains) == pytest.approx(0.48)
    assert cc.speed_cap(10.0, gains) == pytest.approx(0.80)
    assert cc.speed_cap(15.0, gains) == 1.0 and cc.speed_cap(40.0, gains) == 1.0
    # A height that is not known is the lowest cap, not the highest.
    for unknown in (None, float("nan"), float("inf"), -3.0):
        assert cc.speed_cap(unknown, gains) == 0.25
    # And never over the rule-derived cap for centring, whatever a sequence asks for.
    fast = cc.CentringGains(speed_cap_max_mps=5.0, speed_cap_per_m_height=1.0)
    assert cc.speed_cap(15.0, fast) == safety_limits.PHASE_SPEED_CAPS_MPS["centring"] == 1.0


def test_soft_deadband_takes_three_centimetres_off_the_error_and_keeps_its_direction():
    assert cc.soft_deadband(0.02, 0.01, 0.03) == (0.0, 0.0)
    east, north = cc.soft_deadband(0.3, 0.4, 0.03)  # 0.5 m at 53 degrees
    assert math.hypot(east, north) == pytest.approx(0.47) and east / north == pytest.approx(0.75)
    # Soft: just outside the band the output is small, not a step (the 2026 deadband jumps from 0 to the full error).
    east, north = cc.soft_deadband(0.031, 0.0, 0.03)
    assert east == pytest.approx(0.001) and north == 0.0


def test_command_is_kp_times_the_error_outside_the_deadband():
    east, north = settled(cc.CentringController(), 0.33, 0.0)
    assert (east, north) == pytest.approx((0.6 * 0.30, 0.0))
    east, north = settled(cc.CentringController(), 0.0, -0.23)
    assert (east, north) == pytest.approx((0.0, 0.6 * -0.20))
    assert settled(cc.CentringController(), 0.02, 0.02) == (0.0, 0.0)


def test_command_is_capped_by_height_and_keeps_its_direction():
    east, north = settled(cc.CentringController(), 3.0, 4.0, height=6.0)  # 5 m away at 6 m: cap 0.48 m/s
    assert math.hypot(east, north) == pytest.approx(0.48) and east / north == pytest.approx(0.75)
    low = settled(cc.CentringController(), 3.0, 4.0, height=2.0)
    assert math.hypot(*low) == pytest.approx(0.25)
    limited = cc.CentringController()
    t, command = 0.0, (0.0, 0.0)
    for _ in range(400):
        t += TICK
        command = limited.step(3.0, 4.0, 15.0, t, speed_limit_mps=0.3)  # a caller's own, lower limit
    assert math.hypot(*command) == pytest.approx(0.3)


def test_command_changes_by_no_more_than_half_a_metre_per_second_squared():
    controller = cc.CentringController()
    t, previous = 0.0, (0.0, 0.0)
    speeds = []
    for _ in range(40):  # 2 s toward a target 5 m east at 15 m (cap 1.0 m/s)
        t += TICK
        command = controller.step(5.0, 0.0, 15.0, t)
        assert math.hypot(command[0] - previous[0], command[1] - previous[1]) <= 0.5 * TICK + 1e-9
        previous = command
        speeds.append(command[0])
    assert speeds[0] == pytest.approx(0.025) and speeds[19] == pytest.approx(0.5) and speeds[39] == pytest.approx(1.0)
    # The target jumps to the other side: the command comes down and reverses at the same rate, it does not flip.
    for index in range(80):
        t += TICK
        command = controller.step(-5.0, 0.0, 15.0, t)
        assert abs(command[0] - previous[0]) <= 0.5 * TICK + 1e-9
        previous = command
    assert previous[0] == pytest.approx(-1.0)


def test_the_slew_limit_uses_the_real_time_between_steps_and_a_long_gap_starts_again_from_rest():
    controller = cc.CentringController()
    assert controller.step(5.0, 0.0, 15.0, 10.00) == pytest.approx((0.025, 0.0))  # the first step counts as one tick
    assert controller.step(5.0, 0.0, 15.0, 10.10) == pytest.approx((0.075, 0.0))  # a late tick: 0.1 s of slew
    assert controller.step(5.0, 0.0, 15.0, 10.10) == pytest.approx((0.075, 0.0))  # no time has passed: no change
    # Over half a second without a step (the target was lost and the mission held): what was commanded then is no guide now.
    assert controller.step(5.0, 0.0, 15.0, 11.00) == pytest.approx((0.025, 0.0))
    controller.step(5.0, 0.0, 15.0, 11.05)
    controller.reset()
    assert controller.step(5.0, 0.0, 15.0, 11.10) == pytest.approx((0.025, 0.0))


def test_the_lead_takes_the_error_for_where_the_vehicle_will_be_half_a_second_on():
    def command(velocity, lead_s=0.5, east=0.33, north=0.0):
        controller = cc.CentringController(cc.CentringGains(lead_s=lead_s))
        t, out = 0.0, (0.0, 0.0)
        for _ in range(400):
            t += TICK
            out = controller.step(east, north, 6.0, t, velocity_en_mps=velocity)
        return out

    assert command(None) == pytest.approx((0.6 * 0.30, 0.0))  # no velocity reported: the spec's control
    # Moving toward the target at 0.2 m/s: in half a second the error will be 0.23 m, so less is asked for now.
    assert command((0.2, 0.0)) == pytest.approx((0.6 * (0.33 - 0.10 - 0.03), 0.0))
    # Being blown away from it at 0.2 m/s: more is asked for, before the error has grown.
    assert command((-0.2, 0.0)) == pytest.approx((0.6 * (0.33 + 0.10 - 0.03), 0.0))
    # Over the target and drifting north: a command south, though the error itself is inside the deadband.
    assert command((0.0, 0.3), east=0.0) == pytest.approx((0.0, 0.6 * -(0.15 - 0.03)))
    # A lead of zero is the spec's control whatever the velocity; a velocity that is not a number is not used.
    assert command((0.2, 0.0), lead_s=0.0) == pytest.approx((0.6 * 0.30, 0.0))
    assert command((float("nan"), 0.0)) == pytest.approx((0.6 * 0.30, 0.0))
    assert command((0.2, 0.0, -0.4)) == pytest.approx((0.6 * 0.20, 0.0))  # east, north, up as the context gives it: up is not used


def test_without_the_lead_the_gain_is_held_to_what_was_flown_without_it():
    # The range of gains up to 1.0 is only known to be steady with the lead. When the lead cannot be applied
    # (no velocity reported), a higher gain is brought down to 0.6 for that step.
    assert cc.KP_MAX_WITHOUT_LEAD == 0.6
    keen = cc.CentringGains(kp=1.0)
    with_lead = cc.CentringController(keen)
    without = cc.CentringController(keen)
    t = 0.0
    for _ in range(400):
        t += TICK
        led = with_lead.step(0.33, 0.0, 6.0, t, velocity_en_mps=(0.0, 0.0))
        bare = without.step(0.33, 0.0, 6.0, t, velocity_en_mps=None)
    assert led == pytest.approx((1.0 * 0.30, 0.0)) and bare == pytest.approx((0.6 * 0.30, 0.0))
    assert with_lead.lead_applied is True and without.lead_applied is False
    # A sequence that switches the lead off may not ask for more gain than that either.
    with pytest.raises(ValueError, match="centring_kp.*centring_lead_s"):
        cc.CentringGains.from_config({"centring_kp": 0.9, "centring_lead_s": 0.0})
    assert cc.CentringGains.from_config({"centring_kp": 0.6, "centring_lead_s": 0.0}).lead_s == 0.0
    # Nor with a lead shorter than the one the higher gains were flown with: a little lead is not the lead.
    assert cc.LEAD_FOR_KP_MAX_S == 0.5
    with pytest.raises(ValueError, match="centring_kp.*centring_lead_s"):
        cc.CentringGains.from_config({"centring_kp": 1.0, "centring_lead_s": 0.05})
    with pytest.raises(ValueError, match="centring_kp.*centring_lead_s"):
        cc.CentringGains.from_config({"centring_kp": 0.7, "centring_lead_s": 0.4})
    assert cc.CentringGains.from_config({"centring_kp": 1.0, "centring_lead_s": 0.5}).kp == 1.0
    assert cc.CentringGains.from_config({"centring_kp": 0.6, "centring_lead_s": 0.2}).lead_s == 0.2
    spec_only = cc.CentringController(cc.CentringGains(lead_s=0.0))
    spec_only.step(0.33, 0.0, 6.0, 1.0, velocity_en_mps=(0.1, 0.0))
    assert spec_only.lead_applied is True  # a lead of zero is not a lead that went missing


def test_an_error_that_is_not_a_number_commands_a_stop_through_the_slew_limit():
    controller = cc.CentringController()
    t = 0.0
    for _ in range(40):
        t += TICK
        controller.step(5.0, 0.0, 15.0, t)
    t += TICK
    command = controller.step(float("nan"), 0.0, 15.0, t)
    assert command == pytest.approx((1.0 - 0.025, 0.0))  # toward zero, never NaN to the flight controller


def test_gains_come_from_new_keys_of_the_missions_config_and_bad_ones_are_refused():
    assert cc.CentringGains.from_config({}) == cc.CentringGains()
    gains = cc.CentringGains.from_config(
        {
            "centring_kp": 0.4,
            "centring_soft_deadband_m": 0.05,
            "centring_speed_cap_per_m_height": 0.1,
            "centring_speed_cap_min_mps": 0.2,
            "centring_speed_cap_max_mps": 0.8,
            "centring_slew_mps2": 0.3,
            "centring_lead_s": 0.0,
            "centering_gain_mps_per_m": 0.15,  # the 2026 keys are not read
            "centering_deadband_m": 0.4,
        }
    )
    assert gains == cc.CentringGains(0.4, 0.05, 0.1, 0.2, 0.8, 0.3, 0.0)
    for key, bad in (
        ("centring_kp", 0.0),
        ("centring_kp", -0.6),
        ("centring_kp", float("nan")),
        ("centring_kp", "fast"),
        ("centring_kp", True),
        ("centring_soft_deadband_m", -0.01),
        ("centring_slew_mps2", 0.0),
        ("centring_speed_cap_min_mps", 0.0),
        ("centring_speed_cap_per_m_height", -0.08),
        ("centring_lead_s", -0.1),
        ("centring_lead_s", 2.5),
    ):
        with pytest.raises(ValueError, match=key):
            cc.CentringGains.from_config({key: bad})
    with pytest.raises(ValueError, match="centring_speed_cap_min_mps.*centring_speed_cap_max_mps"):
        cc.CentringGains.from_config({"centring_speed_cap_min_mps": 0.9, "centring_speed_cap_max_mps": 0.5})
    # A gain this high would have the loop ring with one tick of delay: refused, not flown.
    with pytest.raises(ValueError, match="centring_kp"):
        cc.CentringGains.from_config({"centring_kp": 1.2})
    assert cc.KP_MAX == 1.0 and cc.CentringGains.from_config({"centring_kp": 1.0}).kp == 1.0
