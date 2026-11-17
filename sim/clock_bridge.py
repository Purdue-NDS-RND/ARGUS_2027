#!/usr/bin/env python3
"""Bridge Gazebo's clock to the ROS /clock topic, thinned to one message every 4 ms of sim time.

  python3 sim/clock_bridge.py            (started by sim/container/start.sh on sim time)

Runs inside the sim container. Gazebo publishes its clock at every physics
step, 1000 times per sim second. ros_gz_bridge passes all of them on, and
every ROS node with use_sim_time handles every one: MAVROS alone is about
sixty nodes. Measured on a club laptop, that took MAVROS to 280% of a core,
each Python node to 60%, and the sim to half of real time. At 250 messages per
sim second the clock still resolves 4 ms, which is finer than anything that
reads it here (the 20 Hz executor, the 100 ms and 250 ms stale-data limits,
the 20 ms time check).

The thinning rule is a plain class with no ROS or Gazebo in it.
"""

from __future__ import annotations

import sys

STEP_NS = 4_000_000  # sim time between /clock messages: 250 per sim second
GZ_TOPIC = "/clock"


class ClockThrottle:
    """Decides which of Gazebo's clock messages go out on /clock."""

    def __init__(self, step_ns: int = STEP_NS) -> None:
        self._step_ns = int(step_ns)
        self._last_ns: int | None = None

    def forward(self, sim_ns: int) -> bool:
        """True for the first message, then for each one a full step later, and for any that goes back in time (a reset)."""
        if self._last_ns is None or sim_ns - self._last_ns >= self._step_ns or sim_ns < self._last_ns:
            self._last_ns = sim_ns
            return True
        return False


def main() -> int:
    import rclpy
    from gz.msgs10.clock_pb2 import Clock as GzClock
    from gz.transport13 import Node as GzNode
    from rclpy.parameter import Parameter
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    node = rclpy.create_node("argus_clock_bridge", parameter_overrides=[Parameter("use_sim_time", value=True)])
    publisher = node.create_publisher(Clock, "/clock", 10)
    throttle = ClockThrottle()

    def on_clock(message) -> None:
        sim_ns = message.sim.sec * 1_000_000_000 + message.sim.nsec
        if throttle.forward(sim_ns):
            out = Clock()
            out.clock.sec = message.sim.sec
            out.clock.nanosec = message.sim.nsec
            publisher.publish(out)

    gz_node = GzNode()
    if not gz_node.subscribe(GzClock, GZ_TOPIC, on_clock):
        print(f"clock bridge: could not subscribe to Gazebo's {GZ_TOPIC}", file=sys.stderr)
        return 1
    node.get_logger().info(f"Gazebo {GZ_TOPIC} -> ROS /clock, one message per {STEP_NS / 1e6:g} ms of sim time")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
