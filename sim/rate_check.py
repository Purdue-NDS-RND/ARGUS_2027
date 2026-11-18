#!/usr/bin/env python3
"""Measure the rates MAVROS publishes at, in sim time, against what the stream keeper asks for (PLT-06).

  sim/run_sim.sh --exec python3 sim/rate_check.py [--duration S]
  sim/run_sim.sh --exec python3 sim/rate_check.py <run folder>

Runs inside the sim container. It counts messages on the topics below for
--duration seconds of sim time (default 20), read from Gazebo's clock, and
prints each rate with PASS or FAIL. Given a folder it counts until it is
interrupted and writes <folder>/rate_check.json: sim/runner/fly.py starts it
that way beside every flight on sim time, where the stream keeper is running.
The rate is taken over windows of 5 s of sim time and a topic is judged on its
median window: the stream keeper stays quiet while a mission is in a release
or actuation state (the takeoff, a drop), so for those seconds a rate may sit
where the takeoff service left it. A topic passes when its median window is
at 90% of its rate or more and no more than a quarter of its windows are
under that. A topic with no messages at all is "silent": that fails, unless
the topic is optional (a sensor the vehicle may not carry).

The arithmetic and the verdict are plain functions with no ROS in them.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "drone_utils"))
from drone_utils.stream_keeper import MESSAGE_RATES_HZ  # noqa: E402  (no ROS in it)

# topic: (message type, the MAVLink message it is made from, optional)
TOPICS = {
    "/mavros/local_position/pose": ("geometry_msgs/msg/PoseStamped", "LOCAL_POSITION_NED", False),
    "/mavros/imu/data": ("sensor_msgs/msg/Imu", "ATTITUDE", False),
    "/mavros/global_position/global": ("sensor_msgs/msg/NavSatFix", "GLOBAL_POSITION_INT", False),
    "/mavros/gpsstatus/gps1/raw": ("mavros_msgs/msg/GPSRAW", "GPS_RAW_INT", False),
    "/mavros/battery": ("sensor_msgs/msg/BatteryState", "BATTERY_STATUS", False),
    "/mavros/sys_status": ("mavros_msgs/msg/SysStatus", "SYS_STATUS", False),
    "/mavros/extended_state": ("mavros_msgs/msg/ExtendedState", "EXTENDED_SYS_STATE", False),
    "/mavros/rangefinder/rangefinder": ("sensor_msgs/msg/Range", "RANGEFINDER", True),
}
PASS_FRACTION = 0.9
WINDOW_S = 5.0
MAX_LOW_SHARE = 0.25  # of a topic's windows that may be under the pass line (the keeper's quiet spells)


def expected_hz(mavlink_name: str) -> float:
    return next(rate for name, rate in MESSAGE_RATES_HZ.values() if name == mavlink_name)


def median(values) -> float:
    ordered = sorted(values)
    middle = len(ordered) // 2
    return ordered[middle] if len(ordered) % 2 else 0.5 * (ordered[middle - 1] + ordered[middle])


def judge(counts: dict, span_s: float, windows: dict | None = None) -> dict:
    """Rates and verdicts from message counts over span_s seconds of sim time.

    `windows` maps a topic to its rate in each full 5 s window; given, a topic
    is judged on the median of them, and its lowest window is reported.
    """
    rows, problems = {}, []
    if not span_s > 0.0:
        return {"passed": False, "problems": ["the sim clock did not advance"], "topics": {}}
    for topic, (_type, mavlink_name, optional) in TOPICS.items():
        want = expected_hz(mavlink_name)
        rates = (windows or {}).get(topic) or []
        got = median(rates) if rates else counts.get(topic, 0) / span_s
        low = sum(1 for rate in rates if rate < PASS_FRACTION * want)
        if counts.get(topic, 0) == 0:
            status = "silent"
            if not optional:
                problems.append(f"{topic} is silent ({mavlink_name}, {want:g} Hz asked)")
        elif rates and low > MAX_LOW_SHARE * len(rates):
            status = "FAIL"
            problems.append(f"{topic} was under {PASS_FRACTION:.0%} of the {want:g} Hz asked in {low} of its {len(rates)} five-second windows ({mavlink_name})")
        elif got >= PASS_FRACTION * want:
            status = "PASS"
        else:
            status = "FAIL"
            problems.append(f"{topic} is at {got:.1f} Hz, under {PASS_FRACTION:.0%} of the {want:g} Hz asked ({mavlink_name})")
        rows[topic] = {"mavlink": mavlink_name, "asked_hz": want, "measured_hz": got, "count": counts.get(topic, 0), "status": status}
        if rates:
            rows[topic].update(lowest_window_hz=min(rates), mean_hz=counts.get(topic, 0) / span_s, windows=len(rates), low_windows=low)
    return {"passed": not problems, "problems": problems, "span_sim_s": span_s, "topics": rows}


def read_rate_check(folder) -> dict | None:
    """The report a run folder holds, or None if there is none that can be read."""
    try:
        report = json.loads((Path(folder) / "rate_check.json").read_text())
    except (OSError, json.JSONDecodeError):
        return None
    return report if isinstance(report, dict) and isinstance(report.get("passed"), bool) else None


def listen(duration_s: float | None) -> dict:
    """Count for duration_s of sim time, or until interrupted if it is None."""
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from gz.msgs10.clock_pb2 import Clock as GzClock
    from gz.transport13 import Node as GzNode
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = rclpy.create_node("argus_rate_check")
    clock = {"now": None}

    def on_clock(message) -> None:
        clock["now"] = message.sim.sec + message.sim.nsec * 1.0e-9

    gz_node = GzNode()
    if not gz_node.subscribe(GzClock, "/clock", on_clock):
        raise RuntimeError("could not subscribe to Gazebo's /clock")
    counts = {topic: 0 for topic in TOPICS}
    counting = {"on": False}

    def counter(topic):
        def on_message(_message) -> None:
            if counting["on"]:
                counts[topic] += 1

        return on_message

    for topic, (type_name, _mavlink, _optional) in TOPICS.items():
        node.create_subscription(get_message(type_name), topic, counter(topic), qos_profile_sensor_data)
    deadline = time.monotonic() + 30.0
    while clock["now"] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if clock["now"] is None:
        return {"passed": False, "problems": ["no clock from Gazebo within 30 s"], "topics": {}}
    end_settle = time.monotonic() + 2.0  # let the subscriptions match before counting
    while time.monotonic() < end_settle:
        rclpy.spin_once(node, timeout_sec=0.05)
    start = clock["now"]
    counting["on"] = True
    windows = {topic: [] for topic in TOPICS}
    window_start, window_counts = start, dict(counts)
    try:
        while rclpy.ok() and (duration_s is None or clock["now"] - start < duration_s):
            rclpy.spin_once(node, timeout_sec=0.05)
            elapsed = clock["now"] - window_start
            if elapsed >= WINDOW_S:
                for topic in TOPICS:
                    windows[topic].append((counts[topic] - window_counts[topic]) / elapsed)
                window_start, window_counts = clock["now"], dict(counts)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # the runner interrupts it when the flight ends
    counting["on"] = False
    return judge(counts, clock["now"] - start, windows)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Measure MAVROS topic rates in sim time against the stream keeper's rates.")
    parser.add_argument("folder", type=Path, nargs="?", help="count until interrupted and write rate_check.json here (a run folder)")
    parser.add_argument("--duration", type=float, default=20.0, help="sim seconds to count for, when no folder is given")
    args = parser.parse_args(argv)
    report = listen(None if args.folder is not None else args.duration)
    for topic, row in report["topics"].items():
        print(f"{row['status']:6} {topic}: {row['measured_hz']:.1f} Hz ({row['mavlink']}, {row['asked_hz']:g} Hz asked)")
    for problem in report["problems"]:
        print(f"  {problem}")
    if args.folder is not None:
        (args.folder / "rate_check.json").write_text(json.dumps(report, indent=2) + "\n")
    print("rate check: PASS" if report["passed"] else "rate check: FAIL", flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)  # Gazebo's subscriber thread is still running; see sim/time_check.py
