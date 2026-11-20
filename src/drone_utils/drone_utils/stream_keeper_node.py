"""Stream keeper node (PLT-06): keeps the MAVLink message rates ARGUS needs. Thin wrapper around stream_keeper.py.

Started by config/mavros/argus_mavros.launch.py with stream_keeper:=true; it
is not part of the 2026 start-up. It watches /mavros/state, the flight
controller's clock on /mavros/timesync_status, the rate of each topic the
messages become, and the mission executor's log for release and actuation
states. It sends each request, one at a time, as MAV_CMD_SET_MESSAGE_INTERVAL
through /mavros/cmd/command.
"""

from __future__ import annotations

import rclpy
from mavros_msgs.msg import State, TimesyncStatus
from mavros_msgs.srv import CommandLong
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

from drone_utils.stream_keeper import SET_MESSAGE_INTERVAL, ActuationWatch, StreamKeeper, interval_us

# MAVLink message id: the MAVROS topic it becomes, and that topic's type. Counted to measure the rate.
TOPICS = {
    32: ("/mavros/local_position/pose", "geometry_msgs/msg/PoseStamped"),
    30: ("/mavros/imu/data", "sensor_msgs/msg/Imu"),
    173: ("/mavros/rangefinder/rangefinder", "sensor_msgs/msg/Range"),
    24: ("/mavros/gpsstatus/gps1/raw", "mavros_msgs/msg/GPSRAW"),
    147: ("/mavros/battery", "sensor_msgs/msg/BatteryState"),
    33: ("/mavros/global_position/global", "sensor_msgs/msg/NavSatFix"),
    245: ("/mavros/extended_state", "mavros_msgs/msg/ExtendedState"),
    1: ("/mavros/sys_status", "mavros_msgs/msg/SysStatus"),
    74: ("/mavros/vfr_hud", "mavros_msgs/msg/VfrHud"),
}


class StreamKeeperNode(Node):
    def __init__(self) -> None:
        super().__init__("stream_keeper")
        self._keeper = StreamKeeper()
        self._watch = ActuationWatch()
        self._logged = {"reboots": 0, "drops": 0, "failures": False, "holding": False, "hold": False, "given_up": 0}
        self._client = self.create_client(CommandLong, "/mavros/cmd/command")
        self.create_subscription(State, "/mavros/state", lambda m: self._keeper.on_state(m.connected, self._now()), 10)
        self.create_subscription(
            TimesyncStatus,
            "/mavros/timesync_status",
            lambda m: self._keeper.on_fcu_time(m.remote_timestamp_ns, self._now()),
            qos_profile_sensor_data,
        )
        for message_id, (topic, type_name) in TOPICS.items():
            # raw: the message is only counted, so it is not worth decoding
            self.create_subscription(get_message(type_name), topic, lambda _m, i=message_id: self._keeper.on_message(i), qos_profile_sensor_data, raw=True)
        self.create_subscription(Log, "/rosout", self._on_log, 100)
        self.create_timer(0.2, self._tick)
        rates = ", ".join(f"{name} {rate:g} Hz" for name, rate in self._keeper.rates.values())
        self.get_logger().info(f"Stream keeper: holding {rates}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_log(self, message: Log) -> None:
        if message.name == "mission_executor":
            self._watch.on_log(message.msg, self._now())
            self._keeper.hold = self._watch.active

    def _tick(self) -> None:
        if not self._client.service_is_ready():
            return
        keeper, logged, log = self._keeper, self._logged, self.get_logger()
        keeper.hold = self._watch.holding(self._now())  # a hold lapses if the executor goes silent
        for message_id, rate in keeper.due(self._now()):
            request = CommandLong.Request()
            request.broadcast = False
            request.command = SET_MESSAGE_INTERVAL
            request.confirmation = 0
            request.param1 = float(message_id)
            request.param2 = float(interval_us(rate))
            future = self._client.call_async(request)
            future.add_done_callback(lambda done, message_id=message_id: self._on_answer(message_id, done))
        if keeper.reboots > logged["reboots"]:
            logged.update(reboots=keeper.reboots, holding=False)
            log.warn("Flight controller restarted or reconnected: requesting the message rates again")
        if keeper.rate_drops > logged["drops"]:
            logged.update(drops=keeper.rate_drops, holding=False)
            low = ", ".join(f"{keeper.rates[i][0]} {hz:.1f} Hz" for i, hz in keeper.measured_hz.items() if hz < 0.8 * keeper.rates[i][1])
            log.warn(f"Stream keeper: a rate dropped ({low}); requesting it again")
        if keeper.hold != logged["hold"]:
            logged["hold"] = keeper.hold
            log.info(f"Stream keeper: {'quiet while the mission is in ' + str(self._watch.state) if keeper.hold else 'mission out of its actuation state'}")
        if bool(keeper.failures) and not logged["failures"]:
            name = keeper.rates.get(keeper.last_failed, ("?", 0.0))[0]
            log.warn(f"Stream keeper: the request for {name} was refused or not answered; the others go first, then it is tried again")
        logged["failures"] = bool(keeper.failures)
        if len(keeper.given_up) > logged["given_up"]:
            log.warn("Stream keeper: no longer asking for " + ", ".join(keeper.rates[i][0] for i in sorted(keeper.given_up)) + " (accepted but not arriving)")
        logged["given_up"] = len(keeper.given_up)
        if keeper.holding() and not logged["holding"]:
            logged["holding"] = True
            log.info("Stream keeper: the flight controller accepted every rate")

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
