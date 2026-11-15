"""All flags off must equal comp-2026: the rules that keep it so, checked over the whole tree.

Three ways a flag could come on, or be read as on, without anyone deciding
it: a config file that ships it true, a flag read with a default other than
False, and a read that goes around get_feature_flag. Each is checked here on
every package's config and source, not just today's. No ROS needed.
"""

import ast
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"
CONFIG_FILES = sorted(path for path in SRC.glob("*/config/**/*") if path.suffix in (".yaml", ".yml"))
SOURCE_FILES = sorted(
    path
    for path in SRC.glob("*/**/*.py")
    if "/test/" not in path.as_posix() and "/build/" not in path.as_posix() and path.name != "setup.py"
)
FEATURES_PY = SRC / "drone_mission_core" / "drone_mission_core" / "features.py"


# --- config: every flag ships false -----------------------------------------------


def enabled_flags(node, path=()):
    """(path, value) for every value under a features: mapping that is not False."""
    if isinstance(node, dict):
        for key, value in node.items():
            here = path + (str(key),)
            if key == "features":
                if isinstance(value, dict):
                    for flag, flag_value in value.items():
                        if flag_value is not False:
                            yield here + (str(flag),), flag_value
                elif value not in (None, {}):
                    yield here, value
            elif isinstance(key, str) and key.startswith("features.") and value is not False:
                yield here, value  # a node flag written as a dotted parameter name
            else:
                yield from enabled_flags(value, here)
    elif isinstance(node, list):
        for index, item in enumerate(node):
            yield from enabled_flags(item, path + (str(index),))


def test_config_files_are_found_in_every_package():
    packages = {path.relative_to(SRC).parts[0] for path in CONFIG_FILES}
    assert {"drone_mission_demo", "drone_live_tests", "drone_target_cv"} <= packages
    assert len(CONFIG_FILES) >= 15


@pytest.mark.parametrize("path", CONFIG_FILES, ids=lambda p: p.relative_to(SRC).as_posix())
def test_no_config_file_ships_a_flag_on(path):
    documents = list(yaml.safe_load_all(path.read_text(encoding="utf-8")))
    found = [f"{'.'.join(where)} = {value!r}" for document in documents for where, value in enabled_flags(document)]
    assert found == [], "feature flags ship false; turn them on with feature_overrides or a profile"


@pytest.mark.parametrize("path", CONFIG_FILES, ids=lambda p: p.relative_to(SRC).as_posix())
def test_no_params_file_has_features_at_the_top_level(path):
    for document in yaml.safe_load_all(path.read_text(encoding="utf-8")):
        if isinstance(document, dict) and any(isinstance(v, dict) and "ros__parameters" in v for v in document.values()):
            assert "features" not in document, "a top-level features: key stops a ROS params file from parsing"


def test_the_scan_catches_flags_that_are_on():
    data = {
        "mission_sequence": {"features": {"a": False, "b": True}},
        "node": {"ros__parameters": {"features": {"c": "yes"}, "features.d": True, "features.e": False}},
        "list": [{"features": {"f": 1}}],
    }
    assert sorted(".".join(where) for where, _ in enabled_flags(data)) == [
        "list.0.features.f",
        "mission_sequence.features.b",
        "node.ros__parameters.features.c",
        "node.ros__parameters.features.d",
    ]


# --- source: flags are read one way, and default to False ------------------------------


def flag_read_problems(source: str, name: str = "<source>") -> list:
    """Ways a file reads a flag that could make it true by default, or go around get_feature_flag."""
    problems = []

    def is_features(node) -> bool:
        return (isinstance(node, ast.Attribute) and node.attr == "features") or (isinstance(node, ast.Name) and node.id == "features")

    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Call):
            called = node.func.attr if isinstance(node.func, ast.Attribute) else getattr(node.func, "id", "")
            if called == "get_feature_flag" and (len(node.args) != 2 or node.keywords):
                problems.append(f"{name}:{node.lineno}: get_feature_flag takes the flags and a name, nothing else")
            if called == "get" and isinstance(node.func, ast.Attribute) and is_features(node.func.value):
                problems.append(f"{name}:{node.lineno}: flags are read with get_feature_flag, not .get()")
            if called == "declare_parameter" and node.args and isinstance(node.args[0], ast.Constant):
                parameter = node.args[0].value
                if isinstance(parameter, str) and parameter.startswith("features."):
                    default = node.args[1] if len(node.args) > 1 else None
                    if not (isinstance(default, ast.Constant) and default.value is False):
                        problems.append(f"{name}:{node.lineno}: node flag {parameter} must be declared with default False")
        elif isinstance(node, ast.Subscript) and is_features(node.value):
            problems.append(f"{name}:{node.lineno}: flags are read with get_feature_flag, not a dictionary lookup")
    return problems


def test_source_files_are_found_in_every_package():
    assert len(SOURCE_FILES) >= 30
    assert FEATURES_PY in SOURCE_FILES


@pytest.mark.parametrize("path", SOURCE_FILES, ids=lambda p: p.relative_to(SRC).as_posix())
def test_flags_are_only_read_through_get_feature_flag_and_node_flags_default_false(path):
    source = path.read_text(encoding="utf-8")
    problems = flag_read_problems(source, path.relative_to(ROOT).as_posix())
    if path == FEATURES_PY:  # the one place allowed to look inside the mapping
        problems = [problem for problem in problems if "not .get()" not in problem]
    assert problems == []


def test_the_source_rules_catch_each_kind_of_bad_read():
    bad = '''
value = get_feature_flag(self.spec.features, "x", True)
other = get_feature_flag(features=self.spec.features, name="x")
third = self.spec.features.get("x", True)
fourth = self.spec.features["x"]
fifth = features["x"]
self.declare_parameter("features.ring_fit", True)
self.declare_parameter("features.oracle")
'''
    assert len(flag_read_problems(bad)) == 7
    good = '''
value = get_feature_flag(self.spec.features, "x")
self.declare_parameter("features.ring_fit", False)
self.declare_parameter("loop_rate_hz", 20.0)
config = spec.config.get("fake_drop", False)
'''
    assert flag_read_problems(good) == []


def test_get_feature_flag_itself_defaults_to_false():
    tree = ast.parse(FEATURES_PY.read_text(encoding="utf-8"))
    function = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == "get_feature_flag")
    assert [arg.arg for arg in function.args.args] == ["features", "name"]
    assert not function.args.defaults and not function.args.kwonlyargs and function.args.vararg is None and function.args.kwarg is None
    returns = [node.value for node in ast.walk(function) if isinstance(node, ast.Return)]
    assert len(returns) == 2
    for value in returns:
        if isinstance(value, ast.Constant):
            assert value.value is False
        else:  # features.get(name, False)
            assert isinstance(value, ast.Call) and value.func.attr == "get" and len(value.args) == 2
            assert isinstance(value.args[1], ast.Constant) and value.args[1].value is False

    sys.path.insert(0, str(SRC / "drone_mission_core"))
    from drone_mission_core.features import get_feature_flag

    assert get_feature_flag(None, "anything") is False and get_feature_flag({}, "anything") is False


# --- launch files: every launch that starts the executor takes feature_overrides ----------------

LAUNCH_FILES = sorted(SRC.glob("*/launch/*.launch.py"))


def executor_launch_problems(source: str, name: str = "<launch>") -> list:
    """What is missing for a launch file that starts mission_executor to pass feature_overrides to it."""
    tree = ast.parse(source)
    starts_executor = any(
        isinstance(node, ast.keyword) and node.arg == "executable" and getattr(node.value, "value", None) == "mission_executor"
        for node in ast.walk(tree)
    )
    if not starts_executor:
        return []
    problems = []

    def call_name(node) -> str:
        return getattr(node.func, "id", getattr(node.func, "attr", "")) if isinstance(node, ast.Call) else ""

    declared = [
        node
        for node in ast.walk(tree)
        if call_name(node) == "DeclareLaunchArgument" and node.args and getattr(node.args[0], "value", None) == "feature_overrides"
    ]
    if len(declared) != 1:
        problems.append(f"{name}: declares the feature_overrides launch argument {len(declared)} times, not once")
    else:
        default = next((kw.value for kw in declared[0].keywords if kw.arg == "default_value"), None)
        if not (isinstance(default, ast.Constant) and default.value == ""):
            problems.append(f"{name}: the feature_overrides argument must default to an empty string (no flag on)")

    passed = False
    for node in ast.walk(tree):
        if call_name(node) != "Node" or not any(kw.arg == "executable" and getattr(kw.value, "value", None) == "mission_executor" for kw in node.keywords):
            continue
        for entry in ast.walk(node):
            if isinstance(entry, ast.Dict):
                for key, value in zip(entry.keys, entry.values):
                    if getattr(key, "value", None) != "feature_overrides":
                        continue
                    as_string = call_name(value) == "ParameterValue" and any(kw.arg == "value_type" and getattr(kw.value, "id", "") == "str" for kw in value.keywords)
                    from_argument = any(
                        call_name(inner) == "LaunchConfiguration" and inner.args and getattr(inner.args[0], "value", None) == "feature_overrides"
                        for inner in ast.walk(value)
                    )
                    passed = as_string and from_argument
    if not passed:
        problems.append(f"{name}: mission_executor is not given feature_overrides as a string from the launch argument")
    return problems


def test_launch_files_that_start_the_executor_are_found():
    starting = [path for path in LAUNCH_FILES if 'executable="mission_executor"' in path.read_text(encoding="utf-8")]
    assert len(starting) >= 12


@pytest.mark.parametrize("path", LAUNCH_FILES, ids=lambda p: p.relative_to(SRC).as_posix())
def test_every_launch_that_starts_the_executor_passes_feature_overrides(path):
    assert executor_launch_problems(path.read_text(encoding="utf-8"), path.relative_to(ROOT).as_posix()) == []


def test_the_launch_rule_catches_a_launch_file_without_the_argument():
    good = (SRC / "drone_mission_demo" / "launch" / "package_delivery_demo.launch.py").read_text(encoding="utf-8")
    assert executor_launch_problems(good) == []
    without_argument = good.replace('"feature_overrides",\n                default_value="",', '"something_else",\n                default_value="",')
    assert len(executor_launch_problems(without_argument)) == 1
    on_by_default = good.replace('"feature_overrides",\n                default_value="",', '"feature_overrides",\n                default_value="use_frame_time_pose",')
    assert any("empty string" in problem for problem in executor_launch_problems(on_by_default))
    not_passed = good.replace('"feature_overrides": ParameterValue(', '"other_parameter": ParameterValue(')
    assert any("not given feature_overrides" in problem for problem in executor_launch_problems(not_passed))
    untyped = good.replace("value_type=str", "value_type=None")
    assert any("as a string" in problem for problem in executor_launch_problems(untyped))
    assert executor_launch_problems("from launch_ros.actions import Node\nNode(package='x', executable='target_cv')\n") == []
