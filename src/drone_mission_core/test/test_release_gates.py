"""The drop's release gates (CEN-04): the arithmetic. No ROS, no numpy."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_mission_core import release_gates as rg  # noqa: E402
from drone_mission_core import safety_limits  # noqa: E402

GOOD = rg.GateReading(error_m=0.05, sigma_m=0.04, speed_mps=0.05, tilt_deg=2.0, may_release=True, height_m=2.4, height_source="rangefinder")


def but(**changes):
    return rg.GateReading(**{**GOOD.__dict__, **changes})


def test_the_numbers_are_the_specs():
    assert (rg.MAX_ERROR_M, rg.MAX_SIGMA_M, rg.MAX_SPEED_MPS, rg.MAX_TILT_DEG, rg.HOLD_S) == (0.10, 0.08, 0.15, 6.0, 0.5)
    assert (rg.RANGEFINDER_RELEASE_M, rg.BARO_RELEASE_M, rg.RELEASE_BAND_M) == (2.4, 3.0, 0.2)
    assert (rg.ALIGN_TIMEOUT_S, rg.LAST_RESORT_MAX_ERROR_M) == (25.0, 0.5)
    assert rg.GATES_FLAG == "drop_release_gates"


def test_the_release_height_depends_on_what_measured_it_and_is_never_under_our_floor():
    assert rg.release_band("rangefinder") == pytest.approx((2.2, 2.4, 2.6))
    assert rg.release_band("baro") == pytest.approx((2.8, 3.0, 3.2))  # a larger margin over 1.83 m when the height is the barometer's
    assert rg.release_band("anything else") == rg.release_band("baro")  # an unknown source is trusted least
    for source in ("rangefinder", "baro"):
        assert rg.release_band(source)[0] >= safety_limits.RELEASE_GATE_FLOOR_M > safety_limits.DROP_MIN_RELEASE_HEIGHT_M


def test_every_gate_must_hold():
    assert rg.failing_gates(GOOD) == ()
    assert rg.failing_gates(but(error_m=0.11)) == ("error",) and rg.failing_gates(but(error_m=0.10)) == ()
    assert rg.failing_gates(but(sigma_m=0.09)) == ("sigma",)
    assert rg.failing_gates(but(speed_mps=0.16)) == ("speed",)
    assert rg.failing_gates(but(tilt_deg=6.5)) == ("tilt",)
    assert rg.failing_gates(but(may_release=False)) == ("track",)
    assert rg.failing_gates(but(height_m=2.61)) == ("height",) and rg.failing_gates(but(height_m=2.19)) == ("height",)
    assert rg.failing_gates(but(height_m=2.4, height_source="baro")) == ("height",)  # 2.4 m is too low to trust on the barometer
    assert rg.failing_gates(but(height_m=3.0, height_source="baro")) == ()
    assert rg.failing_gates(but(error_m=0.3, speed_mps=0.4)) == ("error", "speed")
    # What is not known does not pass: no speed, no height, a number that is not one.
    assert rg.failing_gates(but(speed_mps=None)) == ("speed",)
    assert rg.failing_gates(but(height_m=None)) == ("height",)
    assert rg.failing_gates(but(error_m=float("nan"))) == ("error",) and rg.failing_gates(but(error_m=float("inf"))) == ("error",)
    assert rg.failing_gates(but(sigma_m=float("nan"), tilt_deg=float("nan"))) == ("sigma", "tilt")


def ticks(hold, reading, start_s, count):
    """Feed the hold `count` executor ticks of 50 ms from `start_s`; the answers."""
    return [hold.update(reading, start_s + 0.05 * index) for index in range(count)]


def test_the_gates_must_hold_together_for_half_a_second_without_a_break():
    hold = rg.GateHold()
    answers = ticks(hold, GOOD, 10.0, 11)  # 10.00 to 10.50
    assert answers[:10] == [False] * 10 and answers[10] is True and hold.held_s == pytest.approx(0.5)
    hold = rg.GateHold()
    ticks(hold, GOOD, 10.0, 7)  # 0.30 s of holding
    assert hold.update(but(tilt_deg=7.0), 10.35) is False and hold.last_failing == ("tilt",) and hold.held_s == 0.0
    answers = ticks(hold, GOOD, 10.40, 11)
    assert answers[:10] == [False] * 10 and answers[10] is True  # half a second from when they held again
    hold.reset()
    assert hold.update(GOOD, 11.0) is False and hold.held_s == 0.0
    # A gap in the ticks is not half a second of gates holding: the count starts again after it.
    late = rg.GateHold()
    late.update(GOOD, 20.0)
    assert late.update(GOOD, 20.7) is False and late.held_s == 0.0
    assert ticks(late, GOOD, 20.75, 9) == [False] * 9 and late.update(GOOD, 21.2) is True


def test_the_last_resort_releases_only_on_a_track_within_half_a_metre_and_at_a_legal_height():
    assert rg.last_resort_allowed(but(error_m=0.5)) is True
    assert rg.last_resort_allowed(but(error_m=0.51)) is False
    assert rg.last_resort_allowed(but(error_m=0.3, may_release=False)) is False  # not on a track that has not been seen
    assert rg.last_resort_allowed(but(error_m=0.3, height_m=1.95)) is False  # never under our floor
    assert rg.last_resort_allowed(but(error_m=0.3, height_m=None)) is False
    assert rg.last_resort_allowed(but(error_m=0.3, speed_mps=0.4, tilt_deg=9.0, height_m=3.5)) is True  # off the other gates, but a release scores
    # On the barometer the lowest release is the bottom of its band, not the floor: its error is why its band is higher.
    assert rg.last_resort_allowed(but(error_m=0.3, height_m=2.5, height_source="baro")) is False
    assert rg.last_resort_allowed(but(error_m=0.3, height_m=2.8, height_source="baro")) is True
    assert rg.last_resort_allowed(but(error_m=0.3, height_m=2.05)) is True  # the rangefinder's height is taken down to the floor


def test_the_lowest_release_height_depends_on_the_sensor_that_gives_the_height():
    assert rg.lowest_release_m("rangefinder") == safety_limits.RELEASE_GATE_FLOOR_M == 2.0
    assert rg.lowest_release_m("baro") == pytest.approx(2.8)
    assert rg.lowest_release_m("anything else") == pytest.approx(2.8)  # trusted as little as the barometer
