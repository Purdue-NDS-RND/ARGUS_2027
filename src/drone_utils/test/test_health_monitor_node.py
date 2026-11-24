"""PLT-11: the health monitor through its real node class, with its flag off and on. rclpy and the message packages are stand-ins: no ROS."""

import json
import sys
import types
from pathlib import Path
from unittest import mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_utils import health  # noqa: E402

HEALTH_TOPIC, STATUSTEXT_TOPIC = "/argus/health", "/mavros/statustext/send"


class Publisher:
    def __init__(self):
        self.sent = []

    def publish(self, message):
        self.sent.append(message)


class FakeNode:
    overrides: dict = {}
    now_s = 1000.0

    def __init__(self, name):
        self.name, self.declared, self.publishers, self.subscriptions, self.raw, self.timers, self.logger = name, {}, {}, {}, {}, [], mock.MagicMock()

    def declare_parameter(self, name, default):
        self.declared[name] = self.overrides.get(name, default)

    def get_parameter(self, name):
        value = self.declared[name]
        return types.SimpleNamespace(value=value, get_parameter_value=lambda: types.SimpleNamespace(bool_value=value, string_value=value, double_value=value))

    def create_publisher(self, message_type, topic, qos):
        self.publishers[topic] = Publisher()
        return self.publishers[topic]

    def create_subscription(self, message_type, topic, callback, qos, raw=False):
        self.subscriptions[topic], self.raw[topic] = callback, raw
        return topic

    def create_timer(self, period, callback):
        self.timers.append((period, callback))

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=int(FakeNode.now_s * 1.0e9)))


class Message:
    def __init__(self):
        self.data, self.severity, self.text = "", 0, ""


def get_message(type_name):
    if type_name == "no_such/msg/Thing":
        raise ModuleNotFoundError("No module named 'no_such'")
    return type_name


@pytest.fixture
def node_with(monkeypatch):
    def make(reading=None, **parameters):
        for name, module in {
            "rclpy": mock.MagicMock(), "rclpy.node": types.SimpleNamespace(Node=FakeNode), "rclpy.qos": types.SimpleNamespace(qos_profile_sensor_data="sensor data"),
            "mavros_msgs": mock.MagicMock(), "mavros_msgs.msg": types.SimpleNamespace(StatusText=Message), "std_msgs": mock.MagicMock(), "std_msgs.msg": types.SimpleNamespace(String=Message),
            "rosidl_runtime_py": mock.MagicMock(), "rosidl_runtime_py.utilities": types.SimpleNamespace(get_message=get_message),
        }.items():  # fmt: skip
            monkeypatch.setitem(sys.modules, name, module)
        monkeypatch.delitem(sys.modules, "drone_utils.health_monitor_node", raising=False)
        monkeypatch.setattr(FakeNode, "overrides", parameters)
        monkeypatch.setattr(FakeNode, "now_s", 1000.0)
        monkeypatch.setattr(health.Computer, "read", lambda self: reading or health.Reading([("cpu-thermal", 51.0)], False, 23.0, None))
        from drone_utils import health_monitor_node

        return health_monitor_node.HealthMonitorNode()

    return make


def second(node, heard=()):
    """One second passes: the topics in `heard` give their messages, and the node's timer comes round."""
    for step in range(30):
        FakeNode.now_s += 1.0 / 30.0
        for topic in heard:
            node.subscriptions[topic](b"raw bytes")
    for _period, callback in node.timers:
        callback()


def test_with_the_flag_off_the_node_makes_nothing(node_with):
    node = node_with()
    assert node.declared == {"features.health_monitor": False}  # with the flag off nothing else is even declared
    assert node.publishers == {} and node.subscriptions == {} and node.timers == [] and node.health is None
    assert "off" in node.logger.info.call_args[0][0]


def test_with_the_flag_on_rates_are_counted_raw_the_computer_is_read_and_a_silent_topic_reaches_the_ground_station(node_with):
    node = node_with(**{"features.health_monitor": True, "rates": ["cam:camera/image:sensor_msgs/msg/Image:30", "pose:/mavros/local_position/pose:geometry_msgs/msg/PoseStamped:30"], "frame_counts": ["loc:/loc/status"]})
    assert node.logger.error.call_args is None
    assert set(node.subscriptions) == {"camera/image", "/mavros/local_position/pose", "/loc/status"} and node.raw["camera/image"] is True and node.raw["/loc/status"] is False
    assert set(node.publishers) == {HEALTH_TOPIC, STATUSTEXT_TOPIC} and [period for period, _callback in node.timers] == [1.0]
    for _ in range(12):
        second(node, heard=("camera/image", "/mavros/local_position/pose"))
    said = json.loads(node.publishers[HEALTH_TOPIC].sent[-1].data)
    assert len(node.publishers[HEALTH_TOPIC].sent) == 12 and said["findings"] == [] and said["cpu_percent"] == 23.0 and said["gpu_percent"] is None
    assert said["rates"]["cam"] == {"hz": pytest.approx(30.0, abs=0.5), "expected_hz": 30.0} and said["temperatures_c"] == {"cpu-thermal": 51.0}
    # One line when it starts watching, so that silence afterwards means "nothing wrong" and not "no monitor".
    (started,) = node.publishers[STATUSTEXT_TOPIC].sent
    assert (started.severity, started.text) == (health.NOTICE, "ARGUS H:ON 2 rate(s) watched")
    node.publishers[STATUSTEXT_TOPIC].sent.clear()
    # Every thirty seconds the log has everything as it stands, throttling and what is not known included.
    assert any("hottest zone 51 C; CPU 23.0%; GPU not known; throttled no; 0 finding(s)" in call.args[0] for call in node.logger.info.call_args_list)
    # The camera stops; the pose goes on. Three seconds silent and five more of that: one line to the ground station.
    for _ in range(7):
        second(node, heard=("/mavros/local_position/pose",))
    assert node.publishers[STATUSTEXT_TOPIC].sent == []
    for _ in range(3):
        second(node, heard=("/mavros/local_position/pose",))
    (line,) = node.publishers[STATUSTEXT_TOPIC].sent
    assert (line.severity, line.text) == (health.WARNING, "ARGUS H:SILENT cam") and any("Health: ARGUS H:SILENT cam" in call.args[0] for call in node.logger.warn.call_args_list)
    # It comes back: five seconds later the finding is over, and the ground station is told so, once.
    for _ in range(12):
        second(node, heard=("camera/image", "/mavros/local_position/pose"))
    assert [message.text for message in node.publishers[STATUSTEXT_TOPIC].sent] == ["ARGUS H:SILENT cam", "ARGUS H:OK"]
    # A node's frame counts: a third of its frames dropped is a finding; a status that is not one changes nothing.
    for step in range(12):
        node.subscriptions["/loc/status"](types.SimpleNamespace(data=json.dumps({"frames": 20 * step, "dropped": 10 * step})))
        node.subscriptions["/loc/status"](types.SimpleNamespace(data="not json"))
        node.subscriptions["/loc/status"](types.SimpleNamespace(data="[1, 2]"))
        second(node, heard=("camera/image", "/mavros/local_position/pose"))
    assert node.publishers[STATUSTEXT_TOPIC].sent[-1].text == "ARGUS H:DROP loc 33%"


def test_heat_goes_to_the_log_and_the_health_topic_and_not_to_the_link_when_sending_is_off(node_with):
    node = node_with(reading=health.Reading([("gpu-thermal", 91.0)], True, 30.0, 55.0), **{"features.health_monitor": True, "send_statustext": False, "temp_limit_c": 90.0})
    assert set(node.publishers) == {HEALTH_TOPIC}
    for _ in range(8):
        second(node)
    assert json.loads(node.publishers[HEALTH_TOPIC].sent[-1].data)["findings"] == ["ARGUS H:TEMP gpu-thermal 91C", "ARGUS H:THROTTLE cpu"]
    assert any("Health: ARGUS H:TEMP gpu-thermal 91C (not sent: send_statustext is false)" in call.args[0] for call in node.logger.warn.call_args_list)
    cooler = node_with(reading=health.Reading([("gpu-thermal", 91.0)], False, 30.0, 55.0), **{"features.health_monitor": True, "temp_limit_c": 95.0})
    for _ in range(8):
        second(cooler)
    assert [message.text for message in cooler.publishers[STATUSTEXT_TOPIC].sent] == ["ARGUS H:ON 0 rate(s) watched"]  # under the limit it was given: only its start


@pytest.mark.parametrize("parameters, said", [({"rates": ["cam:camera/image:30"]}, "is not name:topic:type:hz"), ({"rates": ["cam:camera/image:no_such/msg/Thing:30"]}, "No module named"), ({"frame_counts": ["loc"]}, "is not name:topic")])
def test_a_setting_that_is_wrong_stops_the_monitor_with_a_reason_and_nothing_is_half_made(node_with, parameters, said):
    node = node_with(**{"features.health_monitor": True, **parameters})
    assert said in node.logger.error.call_args[0][0] and "nothing is watched" in node.logger.error.call_args[0][0]
    assert node.health is None and node.timers == [] and node.subscriptions == {}
    for _period, callback in node.timers:
        callback()


def test_a_value_of_the_wrong_kind_in_the_params_file_is_a_monitor_that_says_so_not_a_node_that_ends(node_with, monkeypatch):
    """The flight-safety review: the declarations sat outside the try, and a typed declare that refuses a value ended the node at its start."""

    def refusing(self, name, default):
        if name == "temp_limit_c":
            raise TypeError("temp_limit_c: expected a double, got the integer 85")
        self.declared[name] = self.overrides.get(name, default)

    monkeypatch.setattr(FakeNode, "declare_parameter", refusing)
    node = node_with(**{"features.health_monitor": True})
    assert "expected a double" in node.logger.error.call_args[0][0] and "nothing is watched" in node.logger.error.call_args[0][0]
    assert node.health is None and node.timers == []


def test_a_reading_that_fails_is_said_and_the_next_second_is_read(node_with, monkeypatch):
    node = node_with(**{"features.health_monitor": True})
    second(node)
    monkeypatch.setattr(health.Computer, "read", lambda self: (_ for _ in ()).throw(OSError("the thermal zone went away")))
    second(node)
    assert "a reading failed (the thermal zone went away)" in node.logger.error.call_args[0][0] and len(node.publishers[HEALTH_TOPIC].sent) == 1
    monkeypatch.setattr(health.Computer, "read", lambda self: health.Reading())
    second(node)
    assert len(node.publishers[HEALTH_TOPIC].sent) == 2
