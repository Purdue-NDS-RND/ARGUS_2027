#!/usr/bin/env python3
"""Check that the sim is on one clock: MAVROS pose stamps against the sim clock, and every node on sim time.

  sim/run_sim.sh --exec python3 sim/time_check.py <folder> [--duration S]
  sim/run_sim.sh --exec python3 sim/time_check.py --wait-synced S

Runs inside the sim container, in a sim started on sim time (SIM_TIME=1, or a
scenario with sim_time: true). It listens until it is interrupted, or for
--duration seconds of sim time, then writes <folder>/time_check.json and
prints PASS or FAIL. sim/runner/fly.py starts it beside every sim-time flight.

The clock it compares with is Gazebo's own, read straight from Gazebo: the
clock that sim/clock_bridge.py copies to /clock, one message in four. A pose
stamp is made by MAVROS from /clock, so a /clock that strayed from Gazebo's
would show up here as stamps that disagree. Reading /clock itself in this
script would put 250 Python callbacks a second in front of every pose, and
the delay that adds was measured as a 95th percentile of 23 ms where the
stamps themselves were good.

It passes (FND-03) when
  - the 95th percentile of |sim clock - pose stamp| over the listening time
    is at most 20 ms, with at least 100 poses,
  - at least 100 of those stamps are the flight controller's own time moved
    onto the ROS clock by MAVROS's time sync, not the time MAVROS received
    the message (which would agree with /clock whatever the link did),
  - MAVROS's time synchroniser is running at the rate the ARGUS config asks for,
  - every ROS node it can see has use_sim_time true.

MAVROS only applies its offset once the synchroniser has converged (500
samples, 50 s at 10 Hz, and longer in the sim: see docs/results). The sim's
start script waits for that with --wait-synced, which exits 0 as soon as
20 poses in a row carry synchronised stamps, or 1 after S wall seconds.

The arithmetic and the verdict are plain functions with no ROS in them.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "drone_mission_core"))
from drone_mission_core.time_health import is_synced  # noqa: E402  (no ROS in it; the missions' guard uses the same rule)

LIMIT_MS = 20.0
MIN_POSES = 100
TIMESYNC_RATE_HZ = 10.0  # what config/mavros/argus_mavros.yaml sets
MIN_TIMESYNC_SAMPLES = 20
POSE_TOPIC = "/mavros/local_position/pose"
TIMESYNC_TOPIC = "/mavros/timesync_status"
TIMESYNC_NODE = "/mavros/time"
SWEEP_EVERY_S = 10.0  # wall seconds between looks at the nodes' parameters
# Nodes that are not part of the system under test: `ros2 launch` puts a
# helper node in the graph that takes no parameters from the launch file and
# stamps nothing, and the ros2 command line has a hidden daemon.
# Two more answer no parameter request: tf2's listener helper inside MAVROS is
# never spun, and MAVROS's /mavros/param serves the flight controller's
# parameters in place of its own.
# And the rate check (sim/rate_check.py), which like this script is on the
# wall clock on purpose and takes its time from Gazebo.
IGNORED_NODE_PREFIXES = ("/launch_ros_", "/_", "/transform_listener_impl_", "/mavros/param", "/argus_rate_check")
SYNCED_IN_A_ROW = 20  # poses, for --wait-synced


def percentile(values, fraction: float) -> float:
    """The value at `fraction` (0 to 1) of the sorted values, by the nearest-rank rule."""
    ordered = sorted(values)
    if not ordered:
        raise ValueError("no values")
    rank = max(1, math.ceil(fraction * len(ordered)))
    return ordered[min(rank, len(ordered)) - 1]


class StampAgreement:
    """Differences between the clock when a message arrived and the stamp it carries."""

    def __init__(self) -> None:
        self.deltas_s: list[float] = []

    def add(self, clock_s: float, stamp_s: float) -> None:
        self.deltas_s.append(clock_s - stamp_s)

    def summary(self) -> dict:
        if not self.deltas_s:
            return {"count": 0, "median_ms": None, "p95_abs_ms": None, "max_abs_ms": None}
        absolute = [abs(delta) for delta in self.deltas_s]
        return {
            "count": len(self.deltas_s),
            "median_ms": percentile(self.deltas_s, 0.5) * 1000.0,
            "p95_abs_ms": percentile(absolute, 0.95) * 1000.0,
            "max_abs_ms": max(absolute) * 1000.0,
        }


class TimesyncLog:
    """MAVROS's /mavros/timesync_status samples: how many, and how good."""

    def __init__(self) -> None:
        self.round_trips_ms: list[float] = []
        self.offsets_ns: list[int] = []

    def add(self, estimated_offset_ns: int, round_trip_ms: float) -> None:
        self.offsets_ns.append(int(estimated_offset_ns))
        self.round_trips_ms.append(float(round_trip_ms))

    def summary(self) -> dict:
        count = len(self.offsets_ns)
        jumps = [abs(b - a) / 1.0e6 for a, b in zip(self.offsets_ns, self.offsets_ns[1:])]
        return {
            "count": count,
            "median_round_trip_ms": percentile(self.round_trips_ms, 0.5) if count else None,
            "largest_offset_step_ms": max(jumps) if jumps else None,
        }


def ignored(node_name: str) -> bool:
    return node_name.startswith(IGNORED_NODE_PREFIXES)


def verdict(pose: dict, synced: dict, timesync: dict, nodes: dict, timesync_rate_param, limit_ms: float = LIMIT_MS) -> dict:
    """PASS or FAIL with every reason, from the summaries.

    `pose` is every pose heard and `synced` the ones whose stamp is
    synchronised flight-controller time (is_synced). `nodes` maps a node's full name to its use_sim_time (True, False, or None
    when the node did not answer). `timesync_rate_param` is MAVROS's
    timesync_rate as the running node reports it, or None if it did not answer.
    """
    problems = []
    if pose["count"] < MIN_POSES:
        problems.append(f"only {pose['count']} poses on {POSE_TOPIC} (at least {MIN_POSES} are needed)")
    elif pose["p95_abs_ms"] > limit_ms:
        problems.append(f"pose stamps are {pose['p95_abs_ms']:.1f} ms from the sim clock at the 95th percentile (limit {limit_ms:.0f} ms)")
    if synced["count"] < MIN_POSES:
        problems.append(
            f"only {synced['count']} of {pose['count']} pose stamps are synchronised flight-controller time "
            f"(at least {MIN_POSES} are needed): MAVROS's time sync had not converged, so the stamps are arrival times"
        )
    elif synced["p95_abs_ms"] > limit_ms:
        problems.append(f"synchronised pose stamps are {synced['p95_abs_ms']:.1f} ms from the sim clock at the 95th percentile (limit {limit_ms:.0f} ms)")
    if timesync_rate_param is None:
        problems.append(f"{TIMESYNC_NODE} did not report its timesync_rate")
    elif abs(float(timesync_rate_param) - TIMESYNC_RATE_HZ) > 1.0e-6:
        problems.append(f"{TIMESYNC_NODE} timesync_rate is {timesync_rate_param}, not {TIMESYNC_RATE_HZ:g} Hz")
    if timesync["count"] < MIN_TIMESYNC_SAMPLES:
        problems.append(f"only {timesync['count']} samples on {TIMESYNC_TOPIC}: MAVROS's time synchroniser is not running")
    checked = {name: value for name, value in nodes.items() if not ignored(name)}
    if not checked:
        problems.append("no ROS nodes were found to check")
    for name in sorted(checked):
        if checked[name] is None:
            problems.append(f"node {name} did not report use_sim_time")
        elif checked[name] is not True:
            problems.append(f"node {name} is on wall time (use_sim_time is false)")
    return {"passed": not problems, "problems": problems}


def build_report(agreement: StampAgreement, synced: StampAgreement, timesync: TimesyncLog, nodes: dict, timesync_rate_param, listened_s: float) -> dict:
    pose, pose_synced, sync = agreement.summary(), synced.summary(), timesync.summary()
    report = verdict(pose, pose_synced, sync, nodes, timesync_rate_param)
    report.update(
        limit_ms=LIMIT_MS,
        listened_sim_s=listened_s,
        pose=pose,
        pose_synced=pose_synced,
        timesync=sync,
        timesync_rate_param=timesync_rate_param,
        nodes=dict(sorted(nodes.items())),
    )
    return report


def read_time_check(folder) -> dict | None:
    """The report a run folder holds, or None if there is none that can be read."""
    try:
        report = json.loads((Path(folder) / "time_check.json").read_text())
    except (OSError, json.JSONDecodeError):
        return None
    return report if isinstance(report, dict) and isinstance(report.get("passed"), bool) else None


# --- the ROS side ---------------------------------------------------------------------


def _stamp_ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class _Sweep:
    """One round of asking every node for use_sim_time (and MAVROS for timesync_rate)."""

    def __init__(self, node) -> None:
        from rcl_interfaces.srv import GetParameters

        self.node = node
        self.started = time.monotonic()
        self.pending = {}
        for name, namespace in node.get_node_names_and_namespaces():
            full = f"{namespace.rstrip('/')}/{name}"
            if full == node.get_fully_qualified_name() or ignored(full):
                continue
            client = node.create_client(GetParameters, f"{full}/get_parameters")
            if not client.service_is_ready():
                self.pending[full] = (client, None)
                continue
            names = ["use_sim_time"] + (["timesync_rate"] if full == TIMESYNC_NODE else [])
            self.pending[full] = (client, client.call_async(GetParameters.Request(names=names)))

    def collect(self, nodes: dict, found: dict, timeout_s: float = 5.0) -> bool:
        """Move finished answers into `nodes`; True once the sweep is over."""
        expired = time.monotonic() - self.started > timeout_s
        for full in list(self.pending):
            client, future = self.pending[full]
            if future is not None and future.done():
                values = future.result().values if future.result() is not None else []
                nodes[full] = bool(values[0].bool_value) if values and values[0].type == 1 else None
                if full == TIMESYNC_NODE and len(values) > 1 and values[1].type == 3:
                    found["timesync_rate"] = values[1].double_value
            elif expired:
                nodes.setdefault(full, None)
            else:
                continue
            self.node.destroy_client(client)
            del self.pending[full]
        return not self.pending


def listen(folder: Path | None, duration_s: float | None, wait_synced_s: float | None = None) -> dict:
    import collections

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from gz.msgs10.clock_pb2 import Clock as GzClock
    from gz.transport13 import Node as GzNode
    from mavros_msgs.msg import TimesyncStatus
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    # On the wall clock on purpose: a node on sim time handles every /clock
    # message, and this one must hear a pose with nothing queued ahead of it.
    # It uses no ROS time, and it leaves itself out of the node check.
    node = rclpy.create_node("argus_time_check")
    agreement, synced, timesync = StampAgreement(), StampAgreement(), TimesyncLog()
    state = {"clock": None, "first_clock": None, "synced_in_a_row": 0}
    recent_offsets = collections.deque(maxlen=4)  # a pose may be stamped with the offset before the newest
    nodes: dict = {}
    found: dict = {}

    def on_clock(message) -> None:  # on Gazebo's transport thread
        state["clock"] = message.sim.sec * 1_000_000_000 + message.sim.nsec
        if state["first_clock"] is None:
            state["first_clock"] = state["clock"]

    def on_pose(message) -> None:
        if state["clock"] is None:
            return
        stamp = _stamp_ns(message.header.stamp)
        agreement.add(state["clock"] * 1.0e-9, stamp * 1.0e-9)
        if is_synced(stamp, recent_offsets):
            synced.add(state["clock"] * 1.0e-9, stamp * 1.0e-9)
            state["synced_in_a_row"] += 1
        else:
            state["synced_in_a_row"] = 0

    def on_timesync(message) -> None:
        recent_offsets.append(int(message.estimated_offset_ns))
        timesync.add(message.estimated_offset_ns, message.round_trip_time_ms)

    gz_node = GzNode()
    if not gz_node.subscribe(GzClock, "/clock", on_clock):
        raise RuntimeError("could not subscribe to Gazebo's /clock")
    node.create_subscription(PoseStamped, POSE_TOPIC, on_pose, qos_profile_sensor_data)
    node.create_subscription(TimesyncStatus, TIMESYNC_TOPIC, on_timesync, qos_profile_sensor_data)

    def listened() -> float:
        return 0.0 if state["first_clock"] is None else (state["clock"] - state["first_clock"]) * 1.0e-9

    if wait_synced_s is not None:
        deadline = time.monotonic() + wait_synced_s
        while state["synced_in_a_row"] < SYNCED_IN_A_ROW and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        result = {"passed": state["synced_in_a_row"] >= SYNCED_IN_A_ROW, "poses": agreement.summary()["count"]}
    else:
        from rclpy.executors import ExternalShutdownException

        sweep, next_sweep = None, time.monotonic() + 3.0
        try:
            while rclpy.ok() and (duration_s is None or listened() < duration_s):
                rclpy.spin_once(node, timeout_sec=0.05)
                if sweep is None and time.monotonic() >= next_sweep:
                    sweep = _Sweep(node)
                if sweep is not None and sweep.collect(nodes, found):
                    sweep, next_sweep = None, time.monotonic() + SWEEP_EVERY_S
        except (KeyboardInterrupt, ExternalShutdownException):
            pass  # the runner interrupts it when the flight ends
        result = build_report(agreement, synced, timesync, nodes, found.get("timesync_rate"), listened())
        (folder / "time_check.json").write_text(json.dumps(result, indent=2) + "\n")
    node.destroy_node()
    rclpy.try_shutdown()
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check pose stamps against /clock and that every node is on sim time.")
    parser.add_argument("folder", type=Path, nargs="?", help="where time_check.json is written (a run folder)")
    parser.add_argument("--duration", type=float, help="sim seconds to listen; default: until interrupted")
    parser.add_argument("--wait-synced", type=float, metavar="S", help="only wait, up to S wall seconds, for MAVROS's stamps to be synchronised")
    args = parser.parse_args(argv)
    if args.wait_synced is not None:
        result = listen(None, None, wait_synced_s=args.wait_synced)
        print("time sync: applied" if result["passed"] else f"time sync: not applied after {args.wait_synced:.0f} s ({result['poses']} poses heard)", flush=True)
        return 0 if result["passed"] else 1
    if args.folder is None:
        parser.error("a folder is needed unless --wait-synced is given")
    report = listen(args.folder.resolve(), args.duration)
    for label, pose in (("all", report["pose"]), ("synchronised", report["pose_synced"])):
        if pose["count"]:
            print(f"pose stamps against the sim clock, {label}: {pose['count']} poses, median {pose['median_ms']:+.1f} ms, p95 {pose['p95_abs_ms']:.1f} ms, max {pose['max_abs_ms']:.1f} ms")
    for problem in report["problems"]:
        print(f"  {problem}")
    print("time check: PASS" if report["passed"] else "time check: FAIL", flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    code = main()
    # Leave without tearing the interpreter down, as sim/runner/fly.py does:
    # Gazebo's subscriber thread is still delivering clock messages, and one
    # arriving during teardown aborts the process. The report is on disk.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
