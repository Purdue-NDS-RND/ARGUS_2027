"""Recovery's executor aborts by landing (REC-7): an RTL refused without a position is asked for again for ever."""

import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]
PARAMS = REPO / "src" / "drone_mission_demo" / "config" / "params"
sys.path.insert(0, str(REPO / "src" / "drone_mission_core"))

from drone_mission_core import recovery_return  # noqa: E402


def test_the_recovery_params_abort_by_landing_and_nothing_else_about_the_executor_moves():
    recovery = yaml.safe_load((PARAMS / "recovery_mission.yaml").read_text())
    base = yaml.safe_load((PARAMS / "mission_params.yaml").read_text())
    assert set(recovery) == set(base) == {"simple_takeoff_service", "mission_executor"}  # no top-level `features:`, no other node
    executor = recovery["mission_executor"]["ros__parameters"]
    assert executor["abort_rtl_mode"] == recovery_return.RECOVERY_ABORT_MODE == "LAND"
    # One value differs from the 2026 file, and it is that one: the 20 Hz tick and the take-off service's tuned
    # values (the 5 s settle after arming among them) are 2026's, so a launch that passes only this file takes off
    # as 2026 did. (The flight-safety review: without the block the service fell back to its code's defaults.)
    assert executor == {**base["mission_executor"]["ros__parameters"], "abort_rtl_mode": "LAND"} and executor["loop_rate_hz"] == 20.0
    assert recovery["simple_takeoff_service"] == base["simple_takeoff_service"]
    assert recovery["simple_takeoff_service"]["ros__parameters"]["post_arm_settle_s"] == 5.0


def test_every_other_mission_keeps_the_2026_abort_mode():
    for file in sorted(PARAMS.glob("*.yaml")):
        if file.name == "recovery_mission.yaml":
            continue
        executor = (yaml.safe_load(file.read_text()) or {}).get("mission_executor", {}).get("ros__parameters", {})
        assert executor.get("abort_rtl_mode", "RTL") == "RTL", file.name
    declared = (REPO / "src" / "drone_mission_core" / "drone_mission_core" / "mission_executor.py").read_text()
    assert 'self.declare_parameter("abort_rtl_mode", "RTL")' in declared  # the executor's own default is 2026's
