"""CEN-08: the release offset model (release_offset.py): the lean from the attitude, the aim point's shift, and K fitted from drops."""

import math
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_mission_core import release_offset as ro  # noqa: E402
from drone_mission_core.features import FLAG_REQUIRES  # noqa: E402


def leaning(east_deg=0.0, north_deg=0.0):
    """The attitude of a vehicle whose "up" leans `east_deg` towards the east and `north_deg` towards the north (small angles)."""
    # Leaning east is a turn about the north axis (y); leaning north is a turn about the east axis (x) the other way.
    pitch, roll = math.radians(east_deg), math.radians(-north_deg)
    return (math.sin(roll / 2.0) * math.cos(pitch / 2.0), math.cos(roll / 2.0) * math.sin(pitch / 2.0), -math.sin(roll / 2.0) * math.sin(pitch / 2.0), math.cos(roll / 2.0) * math.cos(pitch / 2.0))


def test_the_lean_is_the_horizontal_part_of_the_vehicles_up_over_its_vertical_part():
    assert ro.lean_en((0.0, 0.0, 0.0, 1.0)) == (0.0, 0.0)
    east, north = ro.lean_en(leaning(east_deg=10.0))
    assert east == pytest.approx(math.tan(math.radians(10.0)), rel=1e-6) and north == pytest.approx(0.0, abs=1e-9)
    east, north = ro.lean_en(leaning(north_deg=-6.0))
    assert north == pytest.approx(-math.tan(math.radians(6.0)), rel=1e-6) and east == pytest.approx(0.0, abs=1e-9)
    # A turn about the vertical (the heading) leans nothing.
    assert ro.lean_en((0.0, 0.0, math.sin(0.7), math.cos(0.7))) == pytest.approx((0.0, 0.0), abs=1e-12)
    for not_one in ((0.0, 0.0, 0.0, 0.0), (float("nan"), 0.0, 0.0, 1.0), "level", leaning(east_deg=80.0)):
        assert ro.lean_en(not_one) is None


def test_the_aim_point_is_moved_upwind_by_the_drift_the_model_predicts_and_never_far():
    assert ro.FLAG == "release_offset_model" and FLAG_REQUIRES[ro.FLAG] == ("centre_on_estimator", "drop_release_gates")
    assert ro.predicted_drift(4.0, 2.5, (-0.10, 0.0)) == pytest.approx((1.0, 0.0))  # leaning west into the wind: the package drifts east
    model = ro.ReleaseOffset(ro.OffsetSettings(k=1.2))
    assert model.shift(2.5, 0.0) == (0.0, 0.0)  # no lean heard yet
    for tick in range(200):  # ten seconds leaning 6 degrees west (a wind from the west)
        model.update(leaning(east_deg=-6.0), tick * 0.05)
    east, north = model.shift(2.5, 10.0)
    assert east == pytest.approx(-1.2 * 2.5 * math.tan(math.radians(6.0)), rel=0.01) and north == pytest.approx(0.0, abs=1e-6)  # 0.32 m west: upwind
    # A gust of two seconds moves the aim point on, and by less than the gust's own lean would.
    gusty = ro.ReleaseOffset(ro.OffsetSettings(k=0.6))
    for tick in range(240):
        gusty.update(leaning(east_deg=-6.0 if tick < 200 else -14.0), tick * 0.05)
    steady, full = 0.6 * 2.5 * math.tan(math.radians(6.0)), 0.6 * 2.5 * math.tan(math.radians(14.0))
    assert steady < abs(gusty.shift(2.5, 12.0)[0]) < 0.85 * full and ro.LEAN_TAU_S == 2.0
    # Never more than half a metre, whatever the lean and the height.
    strong = ro.ReleaseOffset(ro.OffsetSettings(k=6.0))
    for tick in range(200):
        strong.update(leaning(east_deg=-12.0, north_deg=9.0), tick * 0.05)
    assert math.hypot(*strong.shift(4.0, 10.0)) == pytest.approx(0.5) and ro.MAX_SHIFT_M == 0.5
    # No height, a lean that is a second old, a lean that is no hover, or K at 0: nothing is shifted.
    assert strong.shift(None, 10.0) == strong.shift(float("nan"), 10.0) == strong.shift(4.0, 12.5) == (0.0, 0.0)
    wild = ro.ReleaseOffset(ro.OffsetSettings(k=6.0))
    wild.update(leaning(east_deg=40.0), 0.0)
    assert wild.lean is None and wild.shift(3.0, 0.0) == (0.0, 0.0)
    unfitted = ro.ReleaseOffset()
    unfitted.update(leaning(east_deg=-6.0), 0.0)
    assert unfitted.settings.k == 0.0 and unfitted.lean is not None and unfitted.shift(2.5, 0.0) == (0.0, 0.0)  # the lean is there for the log
    strong.reset()
    assert strong.lean is None


def test_the_settings_are_the_missions_keys_and_a_wrong_one_is_refused():
    assert ro.OffsetSettings.from_config({}) == ro.OffsetSettings(0.0, 0.5)
    assert ro.OffsetSettings.from_config({"release_offset_k": 1.5, "release_offset_max_m": 0.3, "drop_altitude_m": 2.4}) == ro.OffsetSettings(1.5, 0.3)
    for key, value in (("release_offset_k", -1.0), ("release_offset_k", 50.0), ("release_offset_k", True), ("release_offset_max_m", 0.8), ("release_offset_max_m", "far")):
        with pytest.raises(ValueError, match=f"{key} must be a number from"):
            ro.OffsetSettings.from_config({key: value})


def test_k_is_fitted_from_drops_and_drops_in_still_air_are_not_fitted_to():
    rng = random.Random(4)
    truth = 1.4
    drops = []
    for index in range(12):
        height, lean = rng.uniform(2.0, 4.0), (rng.uniform(-0.2, 0.05), rng.uniform(-0.1, 0.1))
        drift = ro.predicted_drift(truth, height, lean)
        drops.append(ro.Drop(height, lean, (drift[0] + rng.gauss(0.0, 0.03), drift[1] + rng.gauss(0.0, 0.03)), f"drop {index}"))
    found = ro.fit(drops)
    assert found.k == pytest.approx(truth, rel=0.1) and found.drops == 12 and found.residual_rms_m < 0.06 < found.unmodelled_rms_m and found.lean_max > 0.1
    with pytest.raises(ValueError, match="at least 3 are needed"):
        ro.fit(drops[:2])
    still = [ro.Drop(2.5, (0.005, -0.004), (0.02, 0.01)), ro.Drop(2.5, (0.0, 0.01), (-0.03, 0.0)), ro.Drop(2.5, (-0.01, 0.0), (0.0, 0.02))]
    with pytest.raises(ValueError, match="made in still air"):
        ro.fit(still)
    # Drops that drifted upwind (a package that cannot be): K is not made negative for them.
    upwind = [ro.Drop(2.5, (-0.1, 0.0), (-0.3, 0.0)), ro.Drop(3.0, (-0.12, 0.0), (-0.4, 0.0)), ro.Drop(2.0, (-0.08, 0.02), (-0.2, 0.0))]
    assert ro.fit(upwind).k == 0.0
    # A drop that is not numbers is left out of the count.
    assert ro.fit(drops + [ro.Drop(float("nan"), (0.0, 0.0), (0.0, 0.0))]).drops == 12
