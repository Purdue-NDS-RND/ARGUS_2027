"""PLT-09: the take-off service drops a pending take-off when the executor's takeover latch tells it to.

The node needs ROS to run. Its cancel callback and the first lines of its timer do not: they are called here on a
node object made without rclpy (the ROS modules are stand-ins), with the fields those lines read.
"""

import sys
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture
def service(monkeypatch):
    class Node:
        pass

    stand_ins = {
        "rclpy": mock.MagicMock(),
        "rclpy.node": SimpleNamespace(Node=Node),
        "rclpy.qos": mock.MagicMock(),
        "geometry_msgs": mock.MagicMock(),
        "geometry_msgs.msg": mock.MagicMock(),
        "sensor_msgs": mock.MagicMock(),
        "sensor_msgs.msg": mock.MagicMock(),
        "mavros_msgs": mock.MagicMock(),
        "mavros_msgs.msg": mock.MagicMock(),
        "mavros_msgs.srv": mock.MagicMock(),
        "std_srvs": mock.MagicMock(),
        "std_srvs.srv": mock.MagicMock(),
    }
    for name, module in stand_ins.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.delitem(sys.modules, "drone_utils.simple_takeoff_service", raising=False)
    monkeypatch.syspath_prepend(str(PACKAGE))
    from drone_utils import simple_takeoff_service

    node = object.__new__(simple_takeoff_service.SimpleTakeoffService)
    node.logger = mock.MagicMock()
    node.get_logger = lambda: node.logger
    node._pending_altitude_m, node._takeoff_sent = None, False
    node._state = SimpleNamespace(connected=True, mode="LOITER", armed=True)
    return node


def response():
    return SimpleNamespace(success=None, message="")


def test_a_pending_take_off_is_dropped_and_the_timer_then_asks_for_nothing(service):
    service._pending_altitude_m = 10.0  # accepted; GUIDED, the arming and the take-off still to come
    answer = service._on_cancel_service(None, response())
    assert answer.success is True and answer.message == "pending take-off cancelled"
    assert service._pending_altitude_m is None
    # With nothing pending the timer returns at its first line: it reads no parameter and sends no request
    # (a missing client or parameter would raise here: the object has none).
    service._on_timer()
    assert service.logger.warn.call_count == 1


def test_with_nothing_pending_or_the_take_off_already_sent_the_cancel_says_so(service):
    answer = service._on_cancel_service(None, response())
    assert answer.success is True and answer.message == "no take-off was pending"
    service._pending_altitude_m, service._takeoff_sent = 10.0, True  # the flight controller has the take-off: not undone here
    answer = service._on_cancel_service(None, response())
    assert answer.message == "no take-off was pending" and service.logger.warn.call_count == 0


def test_the_service_is_behind_a_node_flag_that_every_params_file_has_false():
    import yaml

    text = (PACKAGE / "drone_utils" / "simple_takeoff_service.py").read_text()
    assert 'self.declare_parameter("features.takeoff_cancel", False)' in text
    # Created only inside the flag's branch: with the flag off the node offers what it offered in 2026.
    branch = text.index('if self.get_parameter("features.takeoff_cancel").get_parameter_value().bool_value:')
    assert branch < text.index('"drone_utils/cancel_takeoff",\n                self._on_cancel_service') and text.count("self.create_service(") == 2
    params = sorted((PACKAGE.parent / "drone_mission_demo" / "config" / "params").glob("*.yaml"))
    blocks = [yaml.safe_load(file.read_text()) for file in params]
    with_node = [(file.name, block) for file, block in zip(params, blocks) if "simple_takeoff_service" in block]
    assert len(with_node) == 7
    for name, block in with_node:
        assert "features" not in block, f"{name}: features at the top level stops every node of the launch"
        assert block["simple_takeoff_service"]["ros__parameters"]["features"] == {"takeoff_cancel": False}, name


def test_the_service_is_named_as_the_executor_calls_it():
    text = (PACKAGE / "drone_utils" / "simple_takeoff_service.py").read_text()
    sys.path.insert(0, str(PACKAGE.parent / "drone_mission_core"))
    from drone_mission_core import takeover

    assert takeover.TAKEOFF_CANCEL_SERVICE == "drone_utils/cancel_takeoff"
    assert "<exec_depend>std_srvs</exec_depend>" in (PACKAGE / "package.xml").read_text()
