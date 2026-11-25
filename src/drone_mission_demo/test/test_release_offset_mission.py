"""CEN-08: the release offset model in the drop mission (flag release_offset_model)."""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml  # noqa: F401  (the real one, before the stand-ins put an empty module in its place)

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

import test_centring_missions as missions  # noqa: E402  (installs the ROS stand-ins, then imports the missions)

from drone_mission_core import release_offset, target_access  # noqa: E402
from drone_mission_core.features import FLAG_REQUIRES  # noqa: E402

FLAG = "release_offset_model"
GATES = "drop_release_gates"
OFFSET_ON = {**missions.RETUNE_ON, "use_height_estimator": True, GATES: True, FLAG: True}


def leaning(context, east=0.0, north=0.0):
    """Tilt the stand-in vehicle so that its "up" leans by (east, north): the tangent of the lean, as release_offset.lean_en reads it."""
    size = math.hypot(east, north)
    angle = math.atan(size)
    axis = (0.0, 0.0) if size == 0.0 else (-north / size, east / size)  # up tips toward (east, north) by a turn about this horizontal axis
    q = (axis[0] * math.sin(angle / 2.0), axis[1] * math.sin(angle / 2.0), 0.0, math.cos(angle / 2.0))
    assert release_offset.lean_en(q) == pytest.approx((east, north), abs=1e-9)
    context.local_pose.pose.orientation = SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3])


def over_the_target(features, lean=(0.0, 0.0), **config):
    context = missions.Context(vehicle=(0.0, 0.0, 6.0))
    context.height = target_access.HeightAgl(6.0, "rangefinder")
    context.velocity = (0.0, 0.0, 0.0)
    said = []
    context.logger.info = said.append
    leaning(context, *lean)
    mission = missions.make("drop", context, features=features, **config)
    context.see(0.0, 0.0)
    command = missions.settle(context, mission, ticks=200)
    return context, mission, command, said


def test_the_flag_is_known_needs_the_estimators_centring_and_is_declared_false_in_every_drop_sequence():
    assert release_offset.FLAG == FLAG and FLAG_REQUIRES[FLAG] == ("centre_on_estimator", GATES)
    import yaml as real_yaml

    declaring = []
    for path in sorted((missions.SRC / "drone_mission_demo" / "config" / "sequences").glob("*.yaml")):
        root = real_yaml.safe_load(path.read_text())["mission_sequence"]
        if any(entry["type"] == "package_drop" for entry in root["missions"]) and "drop_release_gates" in root["features"]:
            assert root["features"].get(FLAG) is False, path.name
            assert all(entry.get("config", {}).get("release_offset_k", 0.0) == 0.0 for entry in root["missions"]), path.name  # no K is shipped: none is fitted for the aircraft
            declaring.append(path.name)
    assert len(declaring) == 6


def test_flag_off_or_k_zero_nothing_is_shifted_and_with_k_the_aim_point_moves_upwind_by_no_more_than_the_limit():
    lean = (-0.10, 0.0)  # leaning west, into a wind from the west: the package will be carried east
    _c, plain_mission, plain, _said = over_the_target({**OFFSET_ON, FLAG: False}, lean, release_offset_k=2.0)
    assert plain_mission._release_offset is None  # the flag off: the key is not read
    # Without the gated release (the 2026 column lets go from a height and at a time the model was not following): not made.
    assert over_the_target({**OFFSET_ON, GATES: False}, lean, release_offset_k=2.0)[1]._release_offset is None
    _c, zero_mission, zero, _said = over_the_target(OFFSET_ON, lean)
    assert zero_mission._release_offset is not None and zero_mission._release_offset.settings.k == 0.0
    assert zero[:2] == pytest.approx(plain[:2], abs=1e-9)  # K 0 (no drops fitted): the lean is followed and nothing moves
    # K 0.5 from 6 m with a lean of 0.1: 0.3 m of drift to the east, so the aim point is 0.3 m west (upwind) of the target.
    _c, mission, shifted, _said = over_the_target(OFFSET_ON, lean, release_offset_k=0.5)
    assert mission._release_offset.lean == pytest.approx(lean, abs=1e-6)
    assert shifted[0] == pytest.approx(plain[0] - 0.6 * (0.30 - 0.03), abs=0.01) and shifted[1] == pytest.approx(plain[1], abs=1e-6)
    # A K that asks for more than the limit: half a metre and no more (the cap at 6 m is 0.48 m/s; 0.6 x 0.47 is under it).
    _c, _m, far, _said = over_the_target(OFFSET_ON, lean, release_offset_k=5.0)
    assert far[0] == pytest.approx(plain[0] - 0.6 * (0.50 - 0.03), abs=0.01)
    _c, _m, held, _said = over_the_target(OFFSET_ON, lean, release_offset_k=5.0, release_offset_max_m=0.2)
    assert held[0] == pytest.approx(plain[0] - 0.6 * (0.20 - 0.03), abs=0.01)
    # No height above the ground of this moment (the pose is old): nothing is shifted and the lean is not followed.
    context, mission, _command, _said = over_the_target(OFFSET_ON, lean, release_offset_k=0.5)
    context.height = None
    learnt = mission._release_offset.lean
    leaning(context, 0.2, 0.2)
    for _ in range(40):
        context.tick(mission)
    assert mission._release_offset.lean == learnt and mission._release_offset.shift(None, 0.0) == (0.0, 0.0)
    # The delivery sets its package down: the flag makes nothing there.
    context = missions.Context(vehicle=(0.0, 0.0, 6.0))
    assert missions.make("delivery", context, features=OFFSET_ON)._release_offset is None


def test_the_release_is_logged_once_with_the_height_the_lean_and_the_shift_in_the_line_the_fit_tool_reads():
    import release_offset_fit

    context, mission, _command, said = over_the_target(OFFSET_ON, (-0.10, 0.04), release_offset_k=0.5)
    mission._log_release_offset(context)
    mission._log_release_offset(context)
    lines = [text for text in said if "release offset:" in text]
    assert len(lines) == 1
    found = release_offset_fit.LOG_LINE.search(lines[0])
    assert found and [float(value) for value in found.groups()] == pytest.approx([6.0, -0.10, 0.04], abs=1e-3)
    assert "aim shifted (-0.300, 0.120) m (K 0.5)" in lines[0]
    # The flag off: nothing is said.
    context, mission, _command, said = over_the_target({**OFFSET_ON, FLAG: False}, (-0.10, 0.04))
    mission._log_release_offset(context)
    assert not [text for text in said if "release offset" in text]


def test_a_k_that_is_not_one_stops_the_mission_being_made():
    for value, why in ((-1.0, "from 0 to 5"), (50.0, "from 0 to 5"), (True, "must be a number"), ("big", "must be a number")):
        with pytest.raises(ValueError, match=why):
            missions.make("drop", missions.Context(), features=OFFSET_ON, release_offset_k=value)
    with pytest.raises(ValueError, match="release_offset_max_m must be a number from 0 to 0.5"):
        missions.make("drop", missions.Context(), features=OFFSET_ON, release_offset_max_m=1.0)
