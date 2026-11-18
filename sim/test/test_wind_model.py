"""The sim's wind (SIM-03): the wind itself, the drag force and how it is given to Gazebo. No ROS, no Gazebo."""

import math
import statistics
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import wind_model  # noqa: E402
from scenario.spec import WIND_LEVELS  # noqa: E402
from wind_model import ForcePlan, Wind, WindSettings  # noqa: E402


def test_the_drag_force_is_half_rho_cda_v_squared_along_the_relative_wind():
    force = wind_model.drag_force((5.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.08)
    assert force == pytest.approx((0.5 * 1.225 * 0.08 * 25.0, 0.0, 0.0))  # 1.225 N at 5 m/s
    assert wind_model.drag_force((5.0, 0.0, 0.0), (5.0, 0.0, 0.0), 0.08) == (0.0, 0.0, 0.0)  # moving with the air: none
    # A vehicle flying into still air is pushed back, and a falling payload is slowed and carried downwind.
    assert wind_model.drag_force((0.0, 0.0, 0.0), (0.0, 4.0, 0.0), 0.08)[1] == pytest.approx(-0.5 * 1.225 * 0.08 * 16.0)
    east, _north, up = wind_model.drag_force((3.0, 0.0, 0.0), (0.0, 0.0, -4.0), 0.02)
    assert east == pytest.approx(0.5 * 1.225 * 0.02 * 5.0 * 3.0) and up == pytest.approx(0.5 * 1.225 * 0.02 * 5.0 * 4.0)
    assert (wind_model.IRIS_CDA_M2, wind_model.UPDATE_HZ, wind_model.GUST_TAU_S) == (0.08, 20.0, (2.0, 5.0))


def test_the_hover_tilt_is_atan_of_force_over_weight():
    assert wind_model.hover_tilt_deg(5.0, 1.755) == pytest.approx(math.degrees(math.atan(1.225 / (1.755 * 9.80665))))
    assert 4.0 < wind_model.hover_tilt_deg(5.0, 1.755) < 4.2 and wind_model.hover_tilt_deg(0.0, 1.755) == 0.0


def test_a_bearing_is_where_the_wind_comes_from():
    assert wind_model.toward(0.0) == pytest.approx((0.0, -1.0))  # a north wind blows south
    assert wind_model.toward(90.0) == pytest.approx((-1.0, 0.0), abs=1e-12)  # an east wind blows west
    assert wind_model.toward(225.0) == pytest.approx((math.sqrt(0.5), math.sqrt(0.5)))


def test_a_steady_wind_is_steady_and_calm_is_nothing():
    steady = Wind(WindSettings(mean_mps=5.0, direction_deg=270.0), seed=3)  # a west wind: blowing east
    assert all(steady.at(t * 0.05) == pytest.approx((5.0, 0.0, 0.0), abs=1e-12) for t in range(200))
    assert WindSettings().calm and Wind(WindSettings()).at(12.0) == (0.0, 0.0, 0.0)
    assert not WindSettings(gust_mps=0.5).calm


def test_gusts_have_the_set_spread_forget_themselves_over_seconds_and_average_to_the_mean():
    wind = Wind(WindSettings(mean_mps=4.0, gust_mps=1.0, direction_deg=180.0), seed=7)  # a south wind: blowing north
    samples = [wind.at(index * 0.05) for index in range(120_000)]  # 100 minutes at 20 Hz
    north = [sample[1] for sample in samples]
    assert statistics.fmean(north) == pytest.approx(4.0, abs=0.1) and statistics.pstdev(north) == pytest.approx(1.0, abs=0.08)
    assert statistics.pstdev([sample[0] for sample in samples]) == pytest.approx(0.5, abs=0.05)  # across the wind: half
    assert statistics.pstdev([sample[2] for sample in samples]) == pytest.approx(0.3, abs=0.03)  # vertical: three tenths
    assert 2.0 <= wind.tau_s <= 5.0
    # One correlation time apart the gusts are still alike (about 1/e); ten apart they are not.
    gust = [value - 4.0 for value in north]
    lag = round(wind.tau_s / 0.05)

    def correlation(shift):
        return statistics.fmean(a * b for a, b in zip(gust, gust[shift:])) / statistics.pvariance(gust)

    assert correlation(lag) == pytest.approx(math.exp(-1.0), abs=0.08) and abs(correlation(10 * lag)) < 0.08


def test_the_same_seed_gives_the_same_wind_and_another_seed_another():
    settings = WindSettings(mean_mps=7.0, gust_mps=2.0, discrete_gust_mps=3.0, direction_deg=30.0)
    one, same, other = Wind(settings, seed=1), Wind(settings, seed=1), Wind(settings, seed=2)
    first = [one.at(index * 0.05) for index in range(400)]
    assert first == [same.at(index * 0.05) for index in range(400)]
    assert first != [other.at(index * 0.05) for index in range(400)]


def test_the_step_size_does_not_change_the_gusts_spread():
    coarse = Wind(WindSettings(gust_mps=1.0), seed=5)
    values = [coarse.at(index * 1.0)[1] for index in range(40_000)]  # one sample a second
    assert statistics.pstdev(values) == pytest.approx(1.0, abs=0.05)


def test_the_discrete_gust_comes_once_rises_to_its_peak_and_is_gone():
    wind = Wind(WindSettings(mean_mps=7.0, discrete_gust_mps=3.0, direction_deg=270.0), seed=4)
    start = wind.discrete_at_s
    assert 20.0 <= start <= 60.0
    assert wind.discrete_gust(start - 0.01) == 0.0 and wind.discrete_gust(start + 4.01) == 0.0
    assert wind.discrete_gust(start + 2.0) == pytest.approx(3.0) and wind.discrete_gust(start + 1.0) == pytest.approx(1.5)
    speeds = [wind.at(index * 0.05)[0] for index in range(int(80 / 0.05))]
    assert max(speeds) == pytest.approx(10.0, abs=0.01) and speeds[0] == pytest.approx(7.0) and speeds[-1] == pytest.approx(7.0)


def test_the_levels_are_the_specs_and_read_from_a_scenarios_wind():
    assert WIND_LEVELS == {"calm": (0.0, 0.0, 0.0), "moderate": (4.0, 1.0, 0.0), "strong": (7.0, 2.0, 3.0)}
    settings = WindSettings.from_manifest({"level": "strong", "mean_mps": 7.0, "gust_mps": 2.0, "discrete_gust_mps": 3.0, "direction_deg": 45.0})
    assert settings == WindSettings(7.0, 2.0, 3.0, 45.0) and WindSettings.from_manifest(None).calm


# --- giving a changing force to a system that only adds ---


def test_a_changing_force_is_sent_as_its_change_so_that_what_gazebo_holds_adds_up_to_it():
    plan = ForcePlan(reset_every=1000)
    held = [0.0, 0.0, 0.0]
    for index in range(50):
        wanted = (1.0 + 0.1 * index, -0.5, 0.01 * index)
        for kind, force in plan.update(wanted, index * 0.05):
            assert kind == "add"
            held = [h + f for h, f in zip(held, force)]
        assert held == pytest.approx(wanted, abs=1e-9)
    assert plan.update(wanted, 2.5) == []  # nothing changed: nothing sent


def test_the_pile_is_cleared_every_so_often_and_the_whole_force_sent_once_the_clear_has_been_taken():
    plan = ForcePlan(reset_every=3, settle_s=0.010)
    held, adds = [0.0, 0.0, 0.0], 0
    log = []
    for index in range(40):
        t = index * 0.005  # called at 200 Hz here, to see the settling
        wanted = (2.0 + index * 0.01, 0.0, 0.0)
        for kind, force in plan.update(wanted, t):
            log.append(kind)
            if kind == "clear":
                held, adds = [0.0, 0.0, 0.0], 0
            else:
                held = [h + f for h, f in zip(held, force)]
                adds += 1
        if log and log[-1] != "clear" and held != [0.0, 0.0, 0.0]:
            assert held[0] == pytest.approx(wanted[0], abs=1e-9)
        assert adds <= 3  # the pile never grows past the reset
    assert log.count("clear") >= 5
    for position, kind in enumerate(log[:-1]):
        if kind == "clear":
            assert log[position + 1] == "add"  # never two clears running, and an add after each


def test_no_add_is_sent_until_the_clear_has_had_time_to_be_taken():
    plan = ForcePlan(reset_every=1, settle_s=0.010)
    assert plan.update((1.0, 0.0, 0.0), 0.0) == [("clear", (0.0, 0.0, 0.0))]
    assert plan.update((1.0, 0.0, 0.0), 0.005) == []
    assert plan.update((1.5, 0.0, 0.0), 0.010) == [("add", (1.5, 0.0, 0.0))] and plan.applied == (1.5, 0.0, 0.0)
