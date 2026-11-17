"""Stream keeper node (PLT-06): keeps the MAVLink message rates ARGUS needs. Thin wrapper around stream_keeper.py.

Started by config/mavros/argus_mavros.launch.py with stream_keeper:=true; it
is not part of the 2026 start-up. It watches /mavros/state and the flight
controller's clock on /mavros/timesync_status, and sends each request, one
at a time, as MAV_CMD_SET_MESSAGE_INTERVAL through /mavros/cmd/command.
"""

from __future__ import annotations

import rclpy
from mavros_msgs.msg import State, TimesyncStatus
from mavros_msgs.srv import CommandLong
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from drone_utils.stream_keeper import SET_MESSAGE_INTERVAL, StreamKeeper, interval_us


class StreamKeeperNode(Node):
    def __init__(self) -> None:
        super().__init__("stream_keeper")
        self._keeper = StreamKeeper()
        self._reboots_logged = 0
        self._holding_logged = False
        self._failure_logged = False
        self._client = self.create_client(CommandLong, "/mavros/cmd/command")
        self.create_subscription(State, "/mavros/state", lambda m: self._keeper.on_state(m.connected, self._now()), 10)
        self.create_subscription(
            TimesyncStatus,
            "/mavros/timesync_status",
            lambda m: self._keeper.on_fcu_time(m.remote_timestamp_ns, self._now()),
            qos_profile_sensor_data,
        )
        self.create_timer(0.2, self._tick)
        rates = ", ".join(f"{name} {rate:g} Hz" for name, rate in self._keeper.rates.values())
        self.get_logger().info(f"Stream keeper: holding {rates}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _tick(self) -> None:
        if not self._client.service_is_ready():
            return
        if self._keeper.reboots > self._reboots_logged:
            self._reboots_logged = self._keeper.reboots
            self._holding_logged = False
            self.get_logger().warn("Flight controller restarted or reconnected: requesting the message rates again")
        for message_id, rate in self._keeper.due(self._now()):
            request = CommandLong.Request()
            request.broadcast = False
            request.command = SET_MESSAGE_INTERVAL
            request.confirmation = 0
            request.param1 = float(message_id)
            request.param2 = float(interval_us(rate))
            future = self._client.call_async(request)
            future.add_done_callback(lambda done, message_id=message_id: self._on_answer(message_id, done))
        if self._keeper.failures and not self._failure_logged:
            self._failure_logged = True
            self._holding_logged = False
            name = self._keeper.rates.get(self._keeper.last_failed, ("?", 0.0))[0]
            self.get_logger().warn(f"Stream keeper: the request for {name} was refused or not answered; the others go first, then it is tried again")
        if self._keeper.holding() and not self._holding_logged:
            self._holding_logged = True
            self._failure_logged = False
            self.get_logger().info("Stream keeper: the flight controller accepted every rate")

    def _on_answer(self, message_id: int, future) -> None:
        try:
            result = future.result()
            success = bool(result is not None and result.success)
        except Exception as error:  # noqa: BLE001  (a failed call is a refused request: it is asked again)
            self.get_logger().warn(f"SET_MESSAGE_INTERVAL for message {message_id} failed: {error}")
            success = False
        self._keeper.on_result(message_id, success, self._now())


def main() -> None:
    rclpy.init()
    node = StreamKeeperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
