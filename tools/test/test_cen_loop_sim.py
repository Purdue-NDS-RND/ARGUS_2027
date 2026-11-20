"""tools/cen_loop_sim.py: the centring loop simulated offline (CEN-02), so a gain is tried before it is flown. No ROS."""

import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools" / "cen_loop_sim.py"
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "src" / "drone_mission_core"))

import cen_loop_sim as sim  # noqa: E402
from drone_mission_core.centring_controller import KP_MAX, CentringController, CentringGains  # noqa: E402


def run(*args):
    return subprocess.run([sys.executable, str(TOOL), *map(str, args)], capture_output=True, text=True)


def test_the_loop_is_the_executors_twenty_hertz_with_one_tick_of_delay_and_the_real_controller():
    assert sim.TICK_S == 0.05 and sim.DELAY_TICKS == 1
    assert isinstance(sim.controller_2027().controller, CentringController)
    # One tick of delay: the first command is made from no estimate at all, the second from the first tick's.
    result = sim.simulate(sim.controller_2027(), height_m=6.0, initial_error_m=(2.0, 0.0), seconds=1.0, noise_sigma_m=0.0, gust_sigma_mps=0.0)
    assert result.commands[0] == (0.0, 0.0) and result.commands[1][0] > 0.0
    undelayed = sim.simulate(sim.controller_2027(), height_m=6.0, initial_error_m=(2.0, 0.0), seconds=1.0, noise_sigma_m=0.0, gust_sigma_mps=0.0, delay_ticks=0)
    assert undelayed.commands[0][0] > 0.0


def test_the_estimators_noise_is_in_the_loop():
    def hold(noise):
        return sim.simulate(sim.controller_2027(), height_m=6.0, initial_error_m=(0.0, 0.0), seconds=30.0, noise_sigma_m=noise, gust_sigma_mps=0.0)

    assert hold(0.0).hold_p95_m == 0.0 and hold(0.0).max_command_mps == 0.0  # nothing to chase: the vehicle stays put
    noisy = hold(0.10)
    assert noisy.hold_p95_m > 0.01 and noisy.max_command_mps > 0.0  # the controller chases what the estimator says
    assert hold(0.03).hold_p95_m < noisy.hold_p95_m


def test_the_specs_gains_centre_from_two_metres_without_overshoot_and_hold_within_five_centimetres():
    for height in (15.0, 6.0, 2.4):
        result = sim.simulate(sim.controller_2027(), height_m=height, initial_error_m=(2.0, 0.0), seconds=40.0, noise_sigma_m=0.03, gust_sigma_mps=0.0, seed=1)
        assert result.settle_s is not None and result.settle_s < 20.0, height
        assert result.overshoot_m < 0.05, height
        assert result.hold_p95_m < 0.05, height
        assert result.max_command_mps <= max(0.25, min(1.0, 0.08 * height)) + 1e-9, height
        assert result.max_command_change_mps2 <= 0.5 + 1e-6, height


def test_the_2026_live_tuning_stops_inside_its_deadband_and_the_retune_does_not():
    live = sim.simulate(sim.controller_2026(gain=0.15, deadband_m=0.4, alpha=0.12, max_speed_mps=0.45), height_m=6.0, initial_error_m=(2.0, 1.0), seconds=60.0, noise_sigma_m=0.03, gust_sigma_mps=0.0)
    retuned = sim.simulate(sim.controller_2027(), height_m=6.0, initial_error_m=(2.0, 1.0), seconds=60.0, noise_sigma_m=0.03, gust_sigma_mps=0.0)
    assert 0.15 < live.hold_p95_m <= 0.6  # it parks wherever inside the 0.4 m deadband it stopped
    assert retuned.hold_p95_m < 0.05 < live.hold_p95_m


def test_gusts_get_through_in_proportion_and_a_higher_gain_lets_less_through():
    def hold(wind, kp=0.6, height=6.0):
        gains = CentringGains(kp=kp)
        return sim.simulate(sim.controller_2027(gains), height_m=height, initial_error_m=(0.0, 0.0), seconds=120.0, gust_sigma_mps=sim.GUST_SIGMA_MPS[wind], seed=3).hold_p95_m

    assert hold("calm") < 0.05 < hold("moderate") < hold("strong")
    assert hold("strong", kp=1.0) < hold("strong", kp=0.6) < hold("strong", kp=0.3)
    # Near the ground the cap is 0.25 m/s. Gusts of this size reach it now and then, and it costs the hold next to nothing.
    low = sim.simulate(sim.controller_2027(), height_m=2.4, initial_error_m=(0.0, 0.0), seconds=120.0, gust_sigma_mps=sim.GUST_SIGMA_MPS["strong"], seed=3)
    assert low.max_command_mps <= 0.25 + 1e-9 and low.hold_p95_m == pytest.approx(hold("strong", height=6.0), abs=0.01)


def test_a_gain_over_the_limit_rings_and_the_lead_is_what_keeps_the_limit_where_it_is():
    wild = sim.simulate(sim.controller_2027(CentringGains(kp=6.0)), height_m=15.0, initial_error_m=(0.8, 0.0), seconds=40.0, noise_sigma_m=0.0, gust_sigma_mps=0.0)
    assert wild.overshoot_m > 0.15 and wild.rings is True
    spec = sim.simulate(sim.controller_2027(), height_m=15.0, initial_error_m=(0.8, 0.0), seconds=40.0, noise_sigma_m=0.0, gust_sigma_mps=0.0)
    assert spec.rings is False and spec.overshoot_m < 0.02
    noisy = sim.simulate(sim.controller_2027(), height_m=15.0, initial_error_m=(0.8, 0.0), seconds=40.0, noise_sigma_m=0.03, gust_sigma_mps=0.0)
    assert noisy.rings is False  # the estimator's noise crossing the target by a centimetre is not ringing
    rows = sim.sweep_kp([0.3, 0.6, KP_MAX, 1.5, 2.5, 4.0], height_m=15.0)
    by_kp = {row["kp"]: row for row in rows}
    assert KP_MAX == 1.0
    assert by_kp[0.6]["overshoot_m"] < 0.02 and by_kp[KP_MAX]["overshoot_m"] < 0.05 and not by_kp[KP_MAX]["rings"]
    assert by_kp[2.5]["rings"] and by_kp[4.0]["overshoot_m"] > by_kp[2.5]["overshoot_m"] > by_kp[1.5]["overshoot_m"]
    assert by_kp[0.3]["settle_s"] > by_kp[0.6]["settle_s"]  # and too low a gain is slow
    # Without the lead the same vehicle overshoots at the spec's gain and rings at the limit.
    def no_lead(kp):
        return sim.simulate(sim.controller_2027(CentringGains(kp=kp, lead_s=0.0)), height_m=15.0, initial_error_m=(0.8, 0.0), seconds=40.0, noise_sigma_m=0.0, gust_sigma_mps=0.0)

    assert no_lead(0.6).overshoot_m > 0.03 > spec.overshoot_m
    assert no_lead(0.6).rings is False and no_lead(0.6).overshoot_m < 0.06  # the gain allowed without the lead: it overshoots a little and settles
    assert no_lead(KP_MAX).rings is True


def test_the_same_seed_gives_the_same_flight_and_another_seed_another():
    def go(seed):
        return sim.simulate(sim.controller_2027(), height_m=6.0, initial_error_m=(1.0, 0.0), seconds=20.0, noise_sigma_m=0.03, gust_sigma_mps=0.2, seed=seed)

    assert go(5).errors == go(5).errors and go(5).errors != go(6).errors


def test_the_command_line_prints_the_comparison_and_a_sweep(tmp_path):
    done = run()
    assert done.returncode == 0, done.stderr
    assert "2027 retune" in done.stdout and "2026 live" in done.stdout and "2026 demo" in done.stdout
    assert "hold p95" in done.stdout and "strong" in done.stdout
    out = tmp_path / "loop.json"
    swept = run("--sweep-kp", "0.3,0.6,1.5,4.0", "--height", "15", "--out", out)
    assert swept.returncode == 0, swept.stderr
    assert "kp 0.60" in swept.stdout and "kp 4.00" in swept.stdout and "rings" in swept.stdout
    assert [row["kp"] for row in json.loads(out.read_text())["sweep"]] == [0.3, 0.6, 1.5, 4.0]
    one = run("--kp", "0.9", "--height", "2.4", "--wind", "strong", "--seconds", "30")
    assert one.returncode == 0 and "kp 0.90" in one.stdout
    assert run("--kp", "0").returncode == 2 and run("--kp", "1.4").returncode == 2  # what a sequence would be refused
