"""drone_mission_core/safety_limits.py: the rule-derived limits every mission shares.

A release below 6 ft, a survey below 25 ft or a bucket let go above 6 ft
costs points or the whole mission, so these numbers live in one module as
constants, where a field edit to a YAML file cannot reach them. The tests
pin each number to the rule it comes from and check every shipped mission's
configured heights against them. No ROS needed.
"""

import ast
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "drone_mission_core"))
for module_name in list(sys.modules):
    if module_name == "drone_mission_core" or module_name.startswith("drone_mission_core."):
        del sys.modules[module_name]

from drone_mission_core import safety_limits as limits  # noqa: E402
from drone_mission_core.mission_api import MissionSpec  # noqa: E402
from drone_mission_core.registry import load_sequence_file  # noqa: E402

MODULE = ROOT / "src" / "drone_mission_core" / "drone_mission_core" / "safety_limits.py"
SEQUENCES = sorted((ROOT / "src").glob("*/config/sequences/*.yaml"))


def spec(type_name, **config):
    return MissionSpec(type_name=type_name, name=type_name, config=config, base_dir=Path("."))


# --- the numbers, and the rules they come from --------------------------------------


def test_rule_heights_are_the_rules_feet_in_metres():
    assert limits.DROP_MIN_RELEASE_HEIGHT_M == 1.83  # 6 ft is 1.8288 m; rounded up, the safe way for "at or above"
    assert limits.DROP_MIN_RELEASE_HEIGHT_M >= 6 * limits.FEET_TO_METERS
    assert limits.SETDOWN_MAX_RELEASE_HEIGHT_M == 1.82  # delivery and recovery: a release above 6 ft scores zero
    assert limits.SETDOWN_MAX_RELEASE_HEIGHT_M <= 6 * limits.FEET_TO_METERS  # a ceiling: rounded down
    assert limits.SURVEY_MIN_HEIGHT_M == 7.62 == pytest.approx(25 * limits.FEET_TO_METERS)
    assert limits.SURVEY_ZERO_SCORE_HEIGHT_M == 6.10
    assert limits.SURVEY_ZERO_SCORE_HEIGHT_M >= 20 * limits.FEET_TO_METERS  # 6.096 m, rounded up
    assert limits.FEET_TO_METERS == 0.3048


def test_our_own_margins_sit_on_the_safe_side_of_the_rules():
    assert limits.RELEASE_GATE_FLOOR_M == 2.0
    assert limits.RELEASE_GATE_FLOOR_M > limits.DROP_MIN_RELEASE_HEIGHT_M
    assert limits.RECOVERY_MAX_RELEASE_HEIGHT_M == 1.5
    assert limits.RECOVERY_MAX_RELEASE_HEIGHT_M < limits.SETDOWN_MAX_RELEASE_HEIGHT_M
    assert limits.SURVEY_MIN_HEIGHT_M > limits.SURVEY_ZERO_SCORE_HEIGHT_M
    assert limits.GEOFENCE_MARGIN_M == 5.0  # the autopilot's 2 m fence margin plus 3 m; the lead confirms this number


def test_phase_speed_caps():
    assert limits.PHASE_SPEED_CAPS_MPS == {"centring": 1.0, "survey": 4.0, "carrying": 3.0, "final_descent": 0.5, "set_down": 0.12}
    assert all(cap > 0.0 for cap in limits.PHASE_SPEED_CAPS_MPS.values())


def test_the_sim_judge_and_the_limits_agree():
    """The judge is the independent referee and keeps its own copy; the two must not drift apart."""
    sys.path.insert(0, str(ROOT / "sim"))
    from judge import default_settings
    from judge.missions import SIX_FEET_M, TWENTY_FEET_M, TWENTY_FIVE_FEET_M

    assert (SIX_FEET_M, TWENTY_FEET_M, TWENTY_FIVE_FEET_M) == (
        limits.DROP_MIN_RELEASE_HEIGHT_M,
        limits.SURVEY_ZERO_SCORE_HEIGHT_M,
        limits.SURVEY_MIN_HEIGHT_M,
    )
    # Our ceiling is never looser than the judge's, and within a centimetre of it.
    judged = default_settings("delivery")["max_release_height_m"]
    assert judged - 0.011 <= limits.SETDOWN_MAX_RELEASE_HEIGHT_M <= judged


# --- constants, not config -------------------------------------------------------------


def test_the_module_reads_nothing_it_could_be_tuned_through():
    tree = ast.parse(MODULE.read_text(encoding="utf-8"))
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {alias.name.split(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom):
            imported.add("relative" if node.level else (node.module or "").split(".")[0])
    assert imported <= {"__future__", "typing", "math", "numbers", "relative"}, imported  # no yaml, os, rclpy: nothing to read a value from
    source = MODULE.read_text(encoding="utf-8")
    for forbidden in ("os.environ", "getenv", "declare_parameter(", "get_parameter(", "open("):
        assert forbidden not in source, forbidden


def test_every_limit_is_a_literal_number_in_the_source():
    tree = ast.parse(MODULE.read_text(encoding="utf-8"))
    values = {}
    for node in tree.body:
        if isinstance(node, (ast.Assign, ast.AnnAssign)):
            target = node.targets[0] if isinstance(node, ast.Assign) else node.target
            if isinstance(target, ast.Name) and target.id.endswith(("_M", "_MPS")) and target.id.isupper():
                literals = node.value.values if isinstance(node.value, ast.Dict) else [node.value]
                for literal in literals:  # a number written in the source, not computed or looked up
                    assert isinstance(literal, ast.Constant) and isinstance(literal.value, float), target.id
                values[target.id] = node.value
    assert {"DROP_MIN_RELEASE_HEIGHT_M", "SETDOWN_MAX_RELEASE_HEIGHT_M", "SURVEY_MIN_HEIGHT_M", "RELEASE_GATE_FLOOR_M", "GEOFENCE_MARGIN_M"} <= set(values)


def test_no_config_file_tries_to_set_a_limit():
    import yaml

    names = {name.lower() for name in vars(limits) if name.isupper()}

    def keys(node):
        if isinstance(node, dict):
            for key, value in node.items():
                yield str(key)
                yield from keys(value)
        elif isinstance(node, list):
            for item in node:
                yield from keys(item)

    for path in sorted((ROOT / "src").glob("*/config/**/*.yaml")):
        for document in yaml.safe_load_all(path.read_text(encoding="utf-8")):
            clash = names & set(keys(document))
            assert not clash, f"{path.relative_to(ROOT)} has a key named like a safety limit: {clash}"


# --- what a command may ask for ------------------------------------------------------------


def test_drop_height_command_cannot_go_under_the_release_gate_floor():
    assert limits.drop_height_command(2.4) == 2.4
    assert limits.drop_height_command(2.0) == 2.0
    assert limits.drop_height_command(1.9) == 2.0  # 1.83 m is unreachable by command
    assert limits.drop_height_command(-3.0) == 2.0
    assert limits.drop_height_command(40) == 40.0


@pytest.mark.parametrize("broken", [float("nan"), float("inf"), float("-inf"), None, "2.4", True])
def test_a_broken_height_request_raises_so_the_failure_policy_fires(broken):
    """Returning the floor would turn a broken estimate into a commanded descent."""
    with pytest.raises(ValueError, match="not a finite height"):
        limits.drop_height_command(broken)
    with pytest.raises(ValueError, match="not a finite height"):
        limits.survey_height_command(broken)


def test_survey_height_command_cannot_go_under_25_ft():
    assert limits.survey_height_command(10.67) == 10.67
    assert limits.survey_height_command(7.0) == 7.62


def test_release_permissions():
    assert limits.drop_release_allowed(2.4) is True
    assert limits.drop_release_allowed(2.0) is True
    assert limits.drop_release_allowed(1.99) is False  # under our floor, though still above the rule
    assert limits.drop_release_allowed(float("nan")) is False
    assert limits.drop_release_allowed(float("inf")) is False  # not a reading
    assert limits.set_down_release_allowed(1.82) is True
    assert limits.set_down_release_allowed(1.83) is False  # 6 ft is 1.8288 m
    assert limits.set_down_release_allowed(float("nan")) is False
    assert limits.set_down_release_allowed(float("-inf")) is False
    assert limits.set_down_release_allowed(5.0, landed=True) is True  # on the ground the reading does not matter
    assert limits.recovery_release_allowed(1.5) is True
    assert limits.recovery_release_allowed(1.6) is False  # our own tighter limit for a swinging bucket
    assert limits.recovery_release_allowed(9.9, landed=True) is True


def test_numpy_numbers_from_vision_code_are_numbers_too():
    """A valid float32 height must not read as "not allowed" or raise as "not a height"."""
    import numpy as np

    assert limits.drop_release_allowed(np.float32(2.4)) is True
    assert limits.drop_release_allowed(np.float64(1.9)) is False
    assert limits.set_down_release_allowed(np.float32(1.5)) is True
    assert limits.recovery_release_allowed(np.float32(1.4)) is True
    assert limits.drop_height_command(np.float32(3.0)) == pytest.approx(3.0)
    assert isinstance(limits.drop_height_command(np.float32(3.0)), float)
    assert limits.survey_height_command(np.int64(5)) == 7.62
    assert limits.capped_speed("centring", np.float32(2.0)) == 1.0
    assert limits.drop_release_allowed(np.float32("nan")) is False
    with pytest.raises(ValueError):
        limits.drop_height_command(np.float32("inf"))
    assert limits.check_mission(spec("package_drop", drop_altitude_m=np.float64(6.0), drop_altitude_tolerance_m=np.float32(3.0))) == []


def test_speed_is_capped_per_phase_in_either_direction():
    assert limits.capped_speed("centring", 0.45) == 0.45
    assert limits.capped_speed("centring", 2.5) == 1.0
    assert limits.capped_speed("centring", -2.5) == -1.0
    assert limits.capped_speed("set_down", 0.35) == 0.12
    assert limits.capped_speed("carrying", float("nan")) == 0.0
    assert limits.capped_speed("carrying", float("inf")) == 0.0  # not a speed: hold
    with pytest.raises(KeyError, match="taxiing"):
        limits.capped_speed("taxiing", 1.0)


# --- every mission's configured heights, checked when the sequence is loaded ------------------


def test_drop_height_that_allows_a_release_below_6_ft_is_a_violation():
    assert limits.check_mission(spec("package_drop", drop_altitude_m=6.0, drop_altitude_tolerance_m=3.0)) == []
    assert limits.check_mission(spec("package_drop", drop_altitude_m=2.33, drop_altitude_tolerance_m=0.5)) == []  # 1.83 exactly
    low = limits.check_mission(spec("package_drop", drop_altitude_m=2.3, drop_altitude_tolerance_m=0.5))
    assert len(low) == 1 and "1.80" in low[0] and "1.83" in low[0] and "package_drop" in low[0]
    assert len(limits.check_mission(spec("package_drop", drop_altitude_m=1.5, drop_altitude_tolerance_m=0.0))) == 1


def test_drop_mission_defaults_are_checked_when_the_config_leaves_them_out():
    # package_drop_mission.py defaults: drop_altitude_m 5.0, drop_altitude_tolerance_m 0.5.
    assert limits.check_mission(spec("package_drop")) == []
    assert len(limits.check_mission(spec("package_drop", drop_altitude_tolerance_m=3.5))) == 1  # 5.0 - 3.5 = 1.5


def test_the_copied_drop_defaults_are_the_missions_own():
    """check_mission models package_drop_mission.py; if its defaults change, this copy must follow."""
    import re

    mission = (ROOT / "src" / "drone_mission_demo" / "drone_mission_demo" / "missions" / "package_drop_mission.py").read_text()
    altitude = re.search(r'config\.get\(\s*"drop_altitude_m",\s*([0-9.]+)\s*\)', mission)
    tolerance = re.search(r'config\.get\(\s*"drop_altitude_tolerance_m",\s*([0-9.]+)\s*\)', mission)
    assert altitude and tolerance, "package_drop_mission.py no longer reads these keys this way"
    assert float(altitude.group(1)) == limits._DROP_DEFAULT_ALTITUDE_M
    assert float(tolerance.group(1)) == limits._DROP_DEFAULT_TOLERANCE_M
    # ...and it still starts its drop when within the tolerance of the drop altitude.
    assert "self._drop_altitude_m) <= self._drop_altitude_tolerance_m" in mission


def test_no_mission_repeats_a_rule_height_instead_of_importing_it():
    """The limits live in one module: a mission that writes 1.83 itself has its own, tunable, copy."""
    rule_heights = {limits.DROP_MIN_RELEASE_HEIGHT_M, limits.SETDOWN_MAX_RELEASE_HEIGHT_M, limits.SURVEY_MIN_HEIGHT_M, limits.SURVEY_ZERO_SCORE_HEIGHT_M, 1.8288, 6.096}
    repeated = []
    for path in sorted((ROOT / "src").glob("*/**/*.py")):
        if "/test/" in path.as_posix() or path == MODULE:
            continue
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            if isinstance(node, ast.Constant) and isinstance(node.value, float) and node.value in rule_heights:
                repeated.append(f"{path.relative_to(ROOT)}:{node.lineno}: {node.value}")
    assert repeated == [], "import the limit from drone_mission_core.safety_limits"


def test_drop_config_that_is_not_a_number_is_a_violation_not_a_crash():
    problems = limits.check_mission(spec("package_drop", drop_altitude_m="six"))
    assert len(problems) == 1 and "drop_altitude_m" in problems[0]


def test_missions_with_no_rule_height_have_nothing_to_check():
    for type_name in ("takeoff", "rtl", "gps_waypoint", "local_waypoint", "package_delivery"):
        assert limits.check_mission(spec(type_name, target_altitude_m=0.5)) == []


@pytest.mark.parametrize("path", SEQUENCES, ids=lambda p: p.relative_to(ROOT / "src").as_posix())
def test_every_shipped_sequence_is_inside_the_limits(path):
    specs = load_sequence_file(str(path))
    assert limits.check_sequence(specs) == []


def test_shipped_sequences_include_every_drop_mission():
    drops = [s for path in SEQUENCES for s in load_sequence_file(str(path)) if s.type_name == "package_drop"]
    assert len(drops) >= 3 and len(SEQUENCES) >= 11
    lowest = min(float(s.config["drop_altitude_m"]) - float(s.config["drop_altitude_tolerance_m"]) for s in drops)
    assert lowest >= limits.DROP_MIN_RELEASE_HEIGHT_M


# --- enforcement at load time is behind a flag -------------------------------------------------


def test_violations_are_only_enforced_when_the_flag_is_on():
    bad = spec("package_drop", drop_altitude_m=2.0, drop_altitude_tolerance_m=0.5)
    off = MissionSpec(type_name=bad.type_name, name=bad.name, config=bad.config, base_dir=bad.base_dir, features={"enforce_safety_limits": False})
    on = MissionSpec(type_name=bad.type_name, name=bad.name, config=bad.config, base_dir=bad.base_dir, features={"enforce_safety_limits": True})
    assert limits.check_sequence([off]) != []  # the check always sees it
    assert limits.enforced_violations([off]) == []  # ...but with the flag off the sequence loads as it did in 2026
    assert limits.enforced_violations([bad]) == []  # no flags at all is off
    assert len(limits.enforced_violations([on])) == 1


@pytest.mark.parametrize("path", sorted((ROOT / "src" / "drone_mission_demo" / "config" / "sequences").glob("*.yaml")), ids=lambda p: p.name)
def test_every_mission_sequence_declares_the_flag_off(path):
    for item in load_sequence_file(str(path)):
        assert item.features.get("enforce_safety_limits") is False
