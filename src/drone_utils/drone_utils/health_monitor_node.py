"""The health monitor as a node (PLT-11): a thin wrapper around health.py.

Behind the node flag `features.health_monitor`, which defaults to false. With
the flag off the node makes no subscription, no publisher and no timer.

With it on, once a second it reads the computer (temperatures, throttling,
CPU and GPU load), looks at the rates of the topics its `rates` parameter
names (each "name:topic:type:hz"; the messages are only counted, never
decoded) and at the frame counts of the nodes its `frame_counts` parameter
names (each "name:topic", a std_msgs/String whose JSON says `frames` and
`dropped`, as /loc/status does), and publishes all of it as JSON on
/argus/health. What has been wrong for five seconds goes to the ground
station as a short STATUSTEXT through MAVROS (`statustext_topic`), one line
at a time, again every thirty seconds while it lasts, and `ARGUS H:OK` when
it is over (health.Reporter). One line, `ARGUS H:ON`, goes out when the
monitor starts watching: a ground station that has had no line at all has
no monitor. `send_statustext: false` keeps it all off the link and in the
log and on /argus/health only.

The node flies nothing and commands nothing: it tells.
"""

from __future__ import annotations

import json
from typing import Any, List, Optional

import rclpy
from rclpy.node import Node

from drone_utils import health

HEALTH_TOPIC = "/argus/health"
STATUSTEXT_TOPIC = "/mavros/statustext/send"
PERIOD_S = 1.0
LOG_EVERY_S = 30.0


class HealthMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("health_monitor")
        self.declare_parameter(f"features.{health.FLAG}", False)
        self.health: Optional[health.Health] = None
        self.reporter: Optional[health.Reporter] = None
        self._outputs: dict = {}
        self._last_log_s: Optional[float] = None
        self._said_on = False
        if not bool(self.get_parameter(f"features.{health.FLAG}").get_parameter_value().bool_value):
            self.get_logger().info(f"Health monitor off (features.{health.FLAG}): nothing subscribed, nothing published")
            return
        try:
            # Declared here, inside the try: a value of the wrong kind in the params file (85 for 85.0) is then a monitor
            # that says why it is not watching, not a node that ends at its start.
            self.declare_parameter("rates", [""])  # (a list of strings; the empty string stands for "none": ROS cannot declare an empty list's type)
            self.declare_parameter("frame_counts", [""])
            self.declare_parameter("statustext_topic", STATUSTEXT_TOPIC)
            self.declare_parameter("send_statustext", True)
            self.declare_parameter("temp_limit_c", health.TEMP_LIMIT_C)
            self.declare_parameter("cpu_limit_percent", health.CPU_LIMIT_PERCENT)
            self.declare_parameter("gpu_limit_percent", health.GPU_LIMIT_PERCENT)
            self.declare_parameter("drop_limit_share", health.DROP_LIMIT_SHARE)
            self._start()
        except Exception as error:  # noqa: BLE001  (a monitor that cannot start must say so and stay out of the way)
            self.health, self._outputs = None, {}
            self.get_logger().error(f"features.{health.FLAG} is on but the health monitor could not start ({error}): nothing is watched")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _strings(self, name: str) -> List[str]:
        return [str(item) for item in (self.get_parameter(name).value or []) if str(item).strip()]

    def _start(self) -> None:
        from mavros_msgs.msg import StatusText
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.utilities import get_message
        from std_msgs.msg import String

        self._String, self._StatusText = String, StatusText
        rates = health.parse_rates(self._strings("rates"))
        counts = health.parse_counts(self._strings("frame_counts"))
        limits = health.Limits(
            temp_c=float(self.get_parameter("temp_limit_c").value),
            cpu_percent=float(self.get_parameter("cpu_limit_percent").value),
            gpu_percent=float(self.get_parameter("gpu_limit_percent").value),
            drop_share=float(self.get_parameter("drop_limit_share").value),
        )
        monitor = health.Health([health.Rate(name, hz) for name, _topic, _type, hz in rates], limits, started_s=self._now())
        types = [(name, topic, get_message(type_name)) for name, topic, type_name, _hz in rates]  # (a type that does not exist stops the start here)
        self._computer = health.Computer()
        self._send = bool(self.get_parameter("send_statustext").get_parameter_value().bool_value)
        self._outputs[HEALTH_TOPIC] = self.create_publisher(String, HEALTH_TOPIC, 10)
        if self._send:
            self._outputs["statustext"] = self.create_publisher(StatusText, str(self.get_parameter("statustext_topic").get_parameter_value().string_value), 10)
        for name, topic, message_type in types:
            # raw: a message is only counted, so it is not worth decoding (a 4K frame least of all)
            self.create_subscription(message_type, topic, lambda _message, name=name: monitor.heard(name, self._now()), qos_profile_sensor_data, raw=True)
        for name, topic in counts:
            self.create_subscription(String, topic, lambda message, name=name: self._on_counts(name, message), 10)
        self.health, self.reporter = monitor, health.Reporter()
        self.create_timer(PERIOD_S, self._tick)
        self.get_logger().info(
            f"Health monitor on (features.{health.FLAG}): "
            + (", ".join(f"{name} at {hz:g} Hz" for name, _topic, _type, hz in rates) or "no topic's rate")
            + (f"; frames of {', '.join(name for name, _topic in counts)}" if counts else "")
            + f"; the computer every {PERIOD_S:g} s; " + ("findings to the ground station as STATUSTEXT" if self._send else "nothing sent to the ground station (send_statustext is false)")
        )

    def _on_counts(self, name: str, message: Any) -> None:
        try:
            said = json.loads(message.data)
            self.health.counted(name, said["frames"], said["dropped"], self._now())
        except (ValueError, KeyError, TypeError, AttributeError):
            return  # not a status that says frames and dropped: nothing of it is counted

    def _tick(self) -> None:
        if self.health is None:
            return
        try:
            now = self._now()
            self.health.read(self._computer.read())
            findings = self.health.findings(now)
            summary = self.health.summary(now)
            status = self._String()
            status.data = json.dumps(summary)
            self._outputs[HEALTH_TOPIC].publish(status)
            out = self.reporter.next(findings, now)
            if not self._said_on:
                # Once, on the first second: so that a quiet ground station can be told from a monitor that never started.
                self._said_on = True
                out = out or (health.NOTICE, health.text_of("ON", f"{len(self.health.rates)} rate(s) watched"))
            if out is not None:
                severity, text = out
                self.get_logger().warn(f"Health: {text}" + ("" if self._send else " (not sent: send_statustext is false)"))
                if self._send:
                    message = self._StatusText()
                    message.severity, message.text = int(severity), text
                    self._outputs["statustext"].publish(message)
            if self._last_log_s is None or now - self._last_log_s >= LOG_EVERY_S:
                self._last_log_s = now
                rates = ", ".join(f"{name} {value['hz']:g}/{value['expected_hz']:g} Hz" for name, value in summary["rates"].items()) or "no rates"
                hottest = max(summary["temperatures_c"].values(), default=None)
                self.get_logger().info(
                    f"Health: {rates}; hottest zone {'not known' if hottest is None else f'{hottest:g} C'}; CPU {'not known' if summary['cpu_percent'] is None else str(summary['cpu_percent']) + '%'}; "
                    f"GPU {'not known' if summary['gpu_percent'] is None else str(summary['gpu_percent']) + '%'}; "
                    f"throttled {'not known' if summary['throttled'] is None else 'yes' if summary['throttled'] else 'no'}; {len(findings)} finding(s)"
                )
        except Exception as error:  # noqa: BLE001  (a reading that fails is said, and the next second is another reading)
            self.get_logger().error(f"Health monitor: a reading failed ({error})", throttle_duration_sec=10.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HealthMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
