#!/usr/bin/env python3
"""The recorder's taps (SIM-09): one small message in the bag for every frame seen, and the sim's clock.

  python3 sim/replay/taps.py [--image /camera/image ...] [--sim-clock]

Images are not recorded. For each image topic named, this node listens to
it and publishes the header of every frame it hears (std_msgs/Header: the
frame's own stamp and frame id) on /replay/frame_seen/<the topic>. The
recorder stores that with the time it arrived, which is when a subscriber
on this machine had the frame. The frame's bytes are never unpacked: the
header is cut from the front of the message as it arrived.

With --sim-clock (a sim run whose nodes are on wall time), Gazebo's clock is
published on /replay/sim_clock every 4 ms of sim time, so that a frame's
stamp, which is Gazebo's, can be set against the recorder's clock.

The cutting and the argument list are plain functions with no ROS in them.
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path
from typing import List, Optional

SIM_DIR = Path(__file__).resolve().parents[1]
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))

from replay import topics  # noqa: E402


def header_bytes(message: bytes) -> bytes:
    """The CDR bytes of the std_msgs/Header that a serialised message starts with (as every stamped message does).

    A CDR message is a 4-byte header and then its fields: for a
    std_msgs/Header, the stamp's two 4-byte numbers and the frame id as its
    length and its bytes. Those bytes, with the same 4-byte start, are a
    whole serialised std_msgs/Header.
    """
    if len(message) < 16 or message[0] != 0 or message[1] not in (0, 1):
        raise ValueError("not a serialised message that starts with a header")
    (length,) = struct.unpack_from("<I" if message[1] == 1 else ">I", message, 12)
    end = 16 + length
    if end > len(message):
        raise ValueError("the message ends inside its header's frame id")
    return bytes(message[:end])


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish a header for every frame seen on an image topic, and the sim's clock.")
    parser.add_argument("--image", action="append", default=None, help=f"an image topic to tap (repeatable; default {', '.join(topics.TAPPED_IMAGES)})")
    parser.add_argument("--no-images", action="store_true", help="tap no image topic (the sim clock only)")
    parser.add_argument("--sim-clock", action="store_true", help=f"publish Gazebo's clock on {topics.SIM_CLOCK_TAP}")
    parser.add_argument("--use-sim-time", action="store_true", help="run this node on /clock, as the nodes it records beside do")
    args = parser.parse_args(argv)
    args.image = [] if args.no_images else (list(args.image) if args.image else list(topics.TAPPED_IMAGES))
    return args


def run(args: argparse.Namespace) -> int:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header

    rclpy.init()
    node = rclpy.create_node("argus_replay_taps", parameter_overrides=[Parameter("use_sim_time", value=bool(args.use_sim_time))])
    seen = {topic: 0 for topic in args.image}

    def tap(topic: str):
        publisher = node.create_publisher(Header, topics.frame_tap_topic(topic), 50)

        def on_frame(raw: bytes) -> None:
            try:
                publisher.publish(header_bytes(raw))  # serialised already: published as it is
                seen[topic] += 1
            except ValueError:
                pass

        return on_frame

    for topic in args.image:
        node.create_subscription(Image, topic, tap(topic), qos_profile_sensor_data, raw=True)

    gz_node = None
    if args.sim_clock:
        from gz.msgs10.clock_pb2 import Clock as GzClock
        from gz.transport13 import Node as GzNode

        from clock_bridge import ClockThrottle

        clock_publisher = node.create_publisher(Clock, topics.SIM_CLOCK_TAP, 50)
        throttle = ClockThrottle()

        def on_clock(message) -> None:
            sim_ns = message.sim.sec * 1_000_000_000 + message.sim.nsec
            if throttle.forward(sim_ns):
                out = Clock()
                out.clock.sec, out.clock.nanosec = divmod(sim_ns, 1_000_000_000)
                clock_publisher.publish(out)

        gz_node = GzNode()
        if not gz_node.subscribe(GzClock, "/clock", on_clock):
            node.get_logger().error("could not subscribe to Gazebo's /clock: no sim clock in this recording")

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    print("frames seen: " + ", ".join(f"{topic} {count}" for topic, count in seen.items()), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(run(parse_args()))
