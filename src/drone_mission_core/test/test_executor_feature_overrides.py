"""The mission executor's start-up with `feature_overrides`, driven through the real node class.

rclpy and the message packages are replaced by stand-ins (as the mission
tests do), so this runs without ROS. It checks the one thing the executor
adds to the flag plumbing: it reads the launch parameter, applies it to every
mission of the sequence, and does not start when the flags cannot be flown.
"""

import sys
import types
from pathlib import Path
from unittest import mock

import pytest

ROOT = Path(__file__).resolve().parents[3]
SEQUENCE = ROOT / "src" / "drone_mission_demo" / "config" / "sequences" / "package_delivery_demo.yaml"


class FakeParameterValue:
    def __init__(self, value):
        self.string_value = value if isinstance(value, str) else ""
        self.string_array_value = value if isinstance(value, list) else []
        self.double_value = float(value) if isinstance(value, (int, float)) else 0.0


class FakeNode:
    """What MissionExecutorNode uses of rclpy.node.Node, with parameters set by the test."""

    overrides: dict = {}

    def __init__(self, name):
        self.name = name
        self.declared = {}
        self.logger = mock.MagicMock()

    def declare_parameter(self, name, default=None):
        self.declared[name] = default

    def get_parameter(self, name):
        assert name in self.declared, f"{name} read before it was declared"
        value = self.overrides.get(name, self.declared[name])
        return types.SimpleNamespace(get_parameter_value=lambda: FakeParameterValue(value))

    def get_logger(self):
        return self.logger

    def __getattr__(self, name):  # create_publisher, create_subscription, create_client, create_timer
        if name.startswith("create_"):
            return mock.MagicMock()
        raise AttributeError(name)


@pytest.fixture
def executor_module(monkeypatch):
    stand_ins = {
        "rclpy": mock.MagicMock(),
        "rclpy.node": types.SimpleNamespace(Node=FakeNode),
        "rclpy.parameter": mock.MagicMock(),
        "rclpy.qos": mock.MagicMock(),
        "rclpy.time": mock.MagicMock(),
        "rclpy.task": mock.MagicMock(),
        "rclpy.impl": mock.MagicMock(),
        "rclpy.impl.rcutils_logger": mock.MagicMock(),
        "geometry_msgs": mock.MagicMock(),
        "geometry_msgs.msg": mock.MagicMock(),
        "mavros_msgs": mock.MagicMock(),
        "mavros_msgs.msg": mock.MagicMock(),
        "mavros_msgs.srv": mock.MagicMock(),
        "sensor_msgs": mock.MagicMock(),
        "sensor_msgs.msg": mock.MagicMock(),
        "std_srvs": mock.MagicMock(),
        "std_srvs.srv": mock.MagicMock(),
    }
    for name, module in stand_ins.items():
        monkeypatch.setitem(sys.modules, name, module)
    for name in [n for n in sys.modules if n.split(".")[0] in ("drone_mission_core", "drone_mission_demo")]:
        monkeypatch.delitem(sys.modules, name)
    monkeypatch.syspath_prepend(str(ROOT / "src" / "argus_camera"))  # use_frame_time_pose reads the camera's calibration and mount from it
    monkeypatch.syspath_prepend(str(ROOT / "src" / "drone_mission_demo"))
    monkeypatch.syspath_prepend(str(ROOT / "src" / "drone_mission_core"))
    from drone_mission_core import mission_executor

    return mission_executor


def start(executor_module, monkeypatch, **parameters):
    monkeypatch.setattr(
        FakeNode,
        "overrides",
        {"sequence_file": str(SEQUENCE), "mission_modules": ["drone_mission_demo.missions"], **parameters},
    )
    return executor_module.MissionExecutorNode()


def flags(node):
    return [mission.spec.features for mission in node._sequence]


def shipped_flags(**changed):
    """The flags package_delivery_demo.yaml declares (every one false), with any changes, once per mission."""
    import yaml

    declared = yaml.safe_load(SEQUENCE.read_text())["mission_sequence"]["features"]
    assert declared["use_frame_time_pose"] is False and not any(declared.values())
    return [{**declared, **changed}] * 3


def test_executor_declares_the_parameter_empty_and_flies_the_sequence_as_shipped(executor_module, monkeypatch):
    node = start(executor_module, monkeypatch)
    assert node.declared["feature_overrides"] == ""
    assert [mission.name for mission in node._sequence] == ["takeoff", "package_delivery", "rtl"]
    assert flags(node) == shipped_flags()  # every flag as the YAML has it: off
    assert not any("Feature overrides" in str(call) for call in node.logger.info.call_args_list)
    assert node.declared["loop_rate_hz"] == 20.0  # the tick is untouched


def test_executor_applies_overrides_to_every_mission_and_logs_them(executor_module, monkeypatch):
    node = start(executor_module, monkeypatch, feature_overrides="use_frame_time_pose")
    assert flags(node) == shipped_flags(use_frame_time_pose=True)
    assert any("Feature overrides: use_frame_time_pose" in str(call) for call in node.logger.info.call_args_list)


@pytest.mark.parametrize("text", ["use_warp_drive", "use_frame_time_pose=perhaps", "drop_release_gates"])
def test_executor_does_not_start_with_flags_it_cannot_fly(executor_module, monkeypatch, text):
    with pytest.raises(ValueError):
        start(executor_module, monkeypatch, feature_overrides=text)


def test_executor_does_not_start_with_a_dependency_missing(executor_module, monkeypatch, tmp_path):
    from drone_mission_core import features

    monkeypatch.setattr(features, "FLAG_REQUIRES", {"use_frame_time_pose": ("time_base_ok",)})
    with pytest.raises(ValueError, match="use_frame_time_pose requires time_base_ok"):
        start(executor_module, monkeypatch, feature_overrides="use_frame_time_pose")
    assert flags(start(executor_module, monkeypatch)) == shipped_flags()  # off: nothing to require


# --- safety limits at load time (FND-09), behind the enforce_safety_limits flag ---------------


def low_drop_sequence(tmp_path, drop_altitude_m):
    """package_drop_demo.yaml with another drop height; its tolerance stays 0.5 m."""
    import yaml

    source = SEQUENCE.parent / "package_drop_demo.yaml"
    data = yaml.safe_load(source.read_text())
    for mission in data["mission_sequence"]["missions"]:
        if mission["type"] == "package_drop":
            assert mission["config"]["drop_altitude_tolerance_m"] == 0.5
            mission["config"]["drop_altitude_m"] = drop_altitude_m
    path = tmp_path / "drop.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return str(path)


def test_low_drop_height_still_loads_with_the_flag_off_as_it_did_in_2026(executor_module, monkeypatch, tmp_path):
    node = start(executor_module, monkeypatch, sequence_file=low_drop_sequence(tmp_path, 2.0))
    assert [mission.name for mission in node._sequence] == ["takeoff", "package_drop", "rtl"]


def test_low_drop_height_stops_the_executor_when_limits_are_enforced(executor_module, monkeypatch, tmp_path):
    with pytest.raises(ValueError, match="1.50 m, below the 1.83 m"):
        start(executor_module, monkeypatch, sequence_file=low_drop_sequence(tmp_path, 2.0), feature_overrides="enforce_safety_limits")


def test_legal_drop_height_starts_with_limits_enforced(executor_module, monkeypatch, tmp_path):
    node = start(executor_module, monkeypatch, sequence_file=low_drop_sequence(tmp_path, 4.0), feature_overrides="enforce_safety_limits")
    assert all(features["enforce_safety_limits"] is True for features in flags(node))


@pytest.mark.parametrize("name", ["package_drop_demo.yaml", "package_drop_live.yaml", "final_waypoint_drop.yaml", "package_delivery_live.yaml"])
def test_shipped_sequences_start_with_limits_enforced(executor_module, monkeypatch, name):
    node = start(executor_module, monkeypatch, sequence_file=str(SEQUENCE.parent / name), feature_overrides="enforce_safety_limits")
    assert node._sequence
