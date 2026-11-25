"""CEN-07: wind compensation on the retuned centring controller's command (wind_compensation.py), alone and in the simulated loop."""

import math
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(REPO / "src" / "drone_mission_core"), str(REPO / "tools")]

import cen_loop_sim as sim  # noqa: E402
from drone_mission_core import wind_compensation as wc  # noqa: E402
from drone_mission_core.features import FLAG_REQUIRES  # noqa: E402


def test_the_flag_needs_the_retuned_controller_and_the_gains_are_checked():
    assert wc.FLAG == "wind_compensation" and FLAG_REQUIRES[wc.FLAG] == ("centring_retune",)
    gains = wc.WindGains.from_config({"wind_ki": 0.2, "wind_observer_tau_s": 5.0, "centring_kp": 0.6})
    assert (gains.ki, gains.observer_tau_s, gains.integral_limit_mps, gains.observer_limit_mps, gains.integral_zone_m, gains.delay_s) == (0.2, 5.0, 0.5, 0.8, 0.6, 0.4)
    for key, value, said in (("wind_ki", 0.9, "wind_ki must be from 0 to 0.5"), ("wind_observer_tau_s", 0.2, "wind_observer_tau_s must be from 1 to 20"), ("wind_integral_limit_mps", True, "must be a number"),
                             ("wind_observer_limit_mps", float("nan"), "must be a number"), ("wind_delay_s", "soon", "must be a number")):  # fmt: skip
        with pytest.raises(ValueError, match=said):
            wc.WindGains.from_config({key: value})


def test_the_observer_learns_the_push_from_what_the_vehicle_did_against_what_it_was_asked():
    wind = wc.WindCompensation()
    out = (0.0, 0.0)
    for tick in range(400):  # 20 s hovering over the target: asked to stand still, and moving 0.3 m/s east
        out = wind.step((0.0, 0.0), (0.0, 0.0), (0.3 + out[0], out[1]), tick * 0.05, 0.48)
    assert wind.push == pytest.approx((0.3, 0.0), abs=0.02) and out == pytest.approx((-0.3, 0.0), abs=0.02)  # the push, taken off the command
    # No velocity reported: the observer stands where it was, and the integral alone goes on.
    before = wind.push
    for tick in range(400, 440):
        wind.step((0.0, 0.0), (0.2, 0.0), None, tick * 0.05, 0.48)
    assert wind.push == before and wind.integral[0] == pytest.approx(0.10 * 0.2 * 2.0, rel=0.05)


def test_the_integral_sums_the_error_only_close_in_and_both_terms_and_their_sum_are_limited():
    wind = wc.WindCompensation()
    for tick in range(200):  # far from the target: on the way there, not blown off it
        wind.step((0.4, 0.0), (3.0, 0.0), None, tick * 0.05, 0.48)
    assert wind.integral == (0.0, 0.0)
    for tick in range(200, 400):  # close in, and still flying in at half a metre a second: an error being flown off, not the wind's
        wind.step((0.2, 0.0), (0.5, 0.0), (0.5, 0.0), tick * 0.05, 0.9)
    assert wind.integral == (0.0, 0.0) and wc.WindGains().integral_max_speed_mps == 0.2
    wind.reset()
    for tick in range(400, 4000):  # half a metre off for three minutes, the controller asking little: the integral fills to its own limit
        out = wind.step((0.05, 0.0), (0.5, 0.0), None, tick * 0.05, 0.9)
    assert wind.integral == pytest.approx((0.5, 0.0)) and out == pytest.approx((0.55, 0.0))
    # At the cap nothing more is summed (it would be an overshoot stored up for the arrival), and the sum is held to the cap.
    capped = wc.WindCompensation()
    for tick in range(4000):
        out = capped.step((0.4, 0.0), (0.5, 0.0), None, tick * 0.05, 0.48)
    assert capped.integral[0] < 0.09 and math.hypot(*out) <= 0.48 + 1e-9
    gusty = wc.WindCompensation()
    for tick in range(2000):
        gusty.step((0.0, 0.0), (0.0, 0.0), (5.0, -5.0), tick * 0.05, 0.48)  # a velocity report gone wild
    assert math.hypot(*gusty.push) == pytest.approx(0.8) and math.hypot(*gusty.step((0.0, 0.0), (0.0, 0.0), (5.0, -5.0), 100.0, 0.48)) <= 0.48 + 1e-9
    # An error or a command that is not a number: the command goes through as it is and nothing is learnt.
    nan = float("nan")
    assert gusty.step((0.1, 0.2), (nan, 0.0), (0.0, 0.0), 100.05, 0.48) == (0.1, 0.2)


def test_with_the_controllers_slew_limit_the_command_sent_never_changes_faster_than_it():
    """CEN-02's slew limit (0.5 m/s in a second) is on the command the flight controller is sent, the two terms included."""
    wind, previous, worst = wc.WindCompensation(), None, 0.0
    for tick in range(600):
        # a velocity report that jumps between wild values every second, and a controller's command that steps with it
        wild = (5.0, -5.0) if (tick // 20) % 2 else (-4.0, 3.0)
        base = (0.3, 0.0) if (tick // 30) % 2 else (-0.3, 0.2)
        out = wind.step(base, (0.3, 0.1), wild, tick * 0.05, 0.48, 0.5)
        if previous is not None:
            worst = max(worst, math.hypot(out[0] - previous[0], out[1] - previous[1]) / 0.05)
        previous = out
        assert math.hypot(*out) <= 0.48 + 1e-9
    assert 0.4 < worst <= 0.5 + 1e-6
    # Without the limit given (the module by itself), the same reports move the command faster: the limit is what holds it.
    free, previous, fastest = wc.WindCompensation(), None, 0.0
    for tick in range(600):
        wild = (5.0, -5.0) if (tick // 20) % 2 else (-4.0, 3.0)
        out = free.step((0.3, 0.0) if (tick // 30) % 2 else (-0.3, 0.2), (0.3, 0.1), wild, tick * 0.05, 0.48)
        fastest = max(fastest, 0.0 if previous is None else math.hypot(out[0] - previous[0], out[1] - previous[1]) / 0.05)
        previous = out
    assert fastest > 1.0
    # In the loop, as the mission applies it: no faster than the controller alone, wind or none.
    for steady in ((0.0, 0.0), (0.3, 0.0)):
        flown = sim.simulate(sim.controller_2027_wind(), height_m=6.0, seconds=60.0, gust_sigma_mps=sim.GUST_SIGMA_MPS["strong"], steady_push_mps=steady)
        assert flown.max_command_change_mps2 <= 0.5 + 1e-6


def test_what_was_learnt_is_dropped_on_a_reset_and_after_three_ticks_without_a_step_and_nothing_is_learnt_from_a_remembered_track():
    def hover(wind, ticks, start=0):
        out = (0.0, 0.0)
        for tick in range(start, start + ticks):  # a vehicle that flies what it is asked plus 0.3 m/s of wind, 0.3 m from the target
            out = wind.step((0.0, 0.0), (0.3, 0.0), (0.3 + out[0], out[1]), tick * 0.05, 0.48)
        return out

    wind = wc.WindCompensation()
    hover(wind, 400)
    assert wind.push[0] > 0.2 and wind.integral[0] > 0.05
    wind.reset()
    assert wind.push == wind.integral == (0.0, 0.0)
    hover(wind, 400)
    first_after_gap = wind.step((0.2, 0.0), (0.3, 0.0), (0.0, 0.0), 400 * 0.05 + 2.0, 0.48)  # the target was lost for two seconds
    # Four ticks without a step (the mission held still on a track that dropped out) is enough; one late tick is not.
    hover(wind, 400, start=500)
    learnt = wind.push
    wind.step((0.0, 0.0), (0.3, 0.0), (0.3, 0.0), 900 * 0.05 + 0.05, 0.48)
    assert wind.push[0] == pytest.approx(learnt[0], abs=0.02) and wc.RESTART_AFTER_S == 0.15
    wind.step((0.0, 0.0), (0.3, 0.0), (0.3, 0.0), 901 * 0.05 + 0.05 + 0.2, 0.48)
    assert wind.push[0] < 0.01
    # A track that is only remembered teaches nothing: neither term moves, and what was learnt before still acts.
    seen = wc.WindCompensation()
    hover(seen, 400)
    push, integral = seen.push, seen.integral
    for tick in range(400, 430):
        out = seen.step((0.0, 0.0), (0.5, 0.2), (2.0, -2.0), tick * 0.05, 0.48, learn=False)
    assert seen.push == push and seen.integral == integral and out == pytest.approx((integral[0] - push[0], integral[1] - push[1]))
    assert first_after_gap == pytest.approx((0.2 + 0.10 * 0.3 * 0.05, 0.0), abs=1e-6) and wind.push == (0.0, 0.0)


def test_in_the_loop_a_steady_wind_no_longer_holds_the_vehicle_downwind_and_calm_air_is_not_made_worse():
    """The spec: "so the drone stops sitting downwind of the target"."""
    plain = sim.simulate(sim.controller_2027(), height_m=6.0, seconds=60.0, steady_push_mps=(0.3, 0.0))
    held = sim.simulate(sim.controller_2027_wind(), height_m=6.0, seconds=60.0, steady_push_mps=(0.3, 0.0))
    assert plain.hold_p95_m == pytest.approx(0.3 / 0.6, abs=0.1)  # the push over the gain: half a metre downwind
    assert held.hold_p95_m < 0.10 and held.settle_s is not None and held.settle_s < 20.0
    for steady in ((0.15, 0.1), (-0.15, 0.12)):
        # (two minutes: against the wind at 3 m the approach is at the cap less the push, a tenth of a metre a second)
        assert sim.simulate(sim.controller_2027_wind(), height_m=3.0, seconds=120.0, steady_push_mps=steady).hold_p95_m < 0.12
    # The sum is held to the controller's speed cap (0.25 m/s at 3 m): a push the flight controller leaves that is larger than
    # the cap cannot be held by any command inside it, with compensation or without. The cap is not raised for the wind.
    blown = sim.simulate(sim.controller_2027_wind(), height_m=3.0, seconds=60.0, steady_push_mps=(-0.4, 0.2))
    assert blown.hold_p95_m > 5.0 and blown.max_command_mps <= 0.25 + 1e-6
    # With gusts on top of it, the steady part is still taken out (the gusts are not).
    gusty_plain = sim.simulate(sim.controller_2027(), height_m=6.0, seconds=120.0, gust_sigma_mps=sim.GUST_SIGMA_MPS["strong"], steady_push_mps=(0.3, 0.0))
    gusty_held = sim.simulate(sim.controller_2027_wind(), height_m=6.0, seconds=120.0, gust_sigma_mps=sim.GUST_SIGMA_MPS["strong"], steady_push_mps=(0.3, 0.0))
    assert gusty_held.hold_p95_m < 0.7 * gusty_plain.hold_p95_m
    # In calm air and in gusts with no steady part, it is no worse than without, and a step in still settles without ringing.
    for wind_name in ("calm", "moderate", "strong"):
        without = sim.simulate(sim.controller_2027(), height_m=6.0, seconds=120.0, gust_sigma_mps=sim.GUST_SIGMA_MPS[wind_name])
        with_it = sim.simulate(sim.controller_2027_wind(), height_m=6.0, seconds=120.0, gust_sigma_mps=sim.GUST_SIGMA_MPS[wind_name])
        assert with_it.hold_p95_m <= without.hold_p95_m + 0.03, wind_name
    step = sim.simulate(sim.controller_2027_wind(), height_m=6.0, seconds=40.0, noise_sigma_m=0.0)
    assert step.overshoot_m < 0.10 and step.settle_s is not None and step.settle_s < 12.0
