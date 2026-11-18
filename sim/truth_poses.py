#!/usr/bin/env python3
"""Gazebo's true poses on a ROS topic, /sim/truth/poses (SIM-05). Runs inside the sim container.

  python3 sim/truth_poses.py <run folder>

Started by sim/runner/fly.py beside a flight whose scenario asks for it
(`truth_poses: true`), and interrupted when the flight ends. It publishes
the true pose of the vehicle and of every object of the scenario as one
tf2_msgs/TFMessage, 30 times a second of sim time: each transform's
child_frame_id is the model's name, its frame_id the world's, its stamp the
sim time of the pose. Nothing on the aircraft has such a topic: it is for
the sim's own checks and for the oracle detector (LOC-5), which refuses to
run outside the sim.

Which poses go out and when is decided by the plain class `Selection`.
"""

from __future__ import annotations

import json
import os
import signal
import sys
import threading
from pathlib import Path
from typing import Dict, List, Optional, Tuple

TOPIC = "/sim/truth/poses"
RATE_HZ = 30.0

Pose = Tuple[float, float, float, float, float, float, float]  # x, y, z, then the quaternion x, y, z, w


def names_of(manifest: dict, truth: dict) -> List[str]:
    """The models whose poses are published: the vehicle first, then the scenario's objects."""
    return [manifest["vehicle"]["model"]] + [item["name"] for item in truth.get("objects", [])]


def wanted(manifest: dict) -> bool:
    return manifest.get("truth_poses") is True


class Selection:
    """Keeps the newest pose of each wanted model and says when a message is due."""

    def __init__(self, names: List[str], rate_hz: float = RATE_HZ) -> None:
        self.names = list(names)
        self._period_s = 1.0 / float(rate_hz)
        self._poses: Dict[str, Pose] = {}
        self._last_s: Optional[float] = None

    def add(self, name: str, pose: Pose) -> None:
        if name in self.names:
            self._poses[name] = pose

    def due(self, t_s: float) -> Optional[List[Tuple[str, Pose]]]:
        """The poses to publish at sim time `t_s`, in the order of `names`, or None if it is not time yet."""
        if not self._poses or (self._last_s is not None and 0.0 <= t_s - self._last_s < self._period_s - 1.0e-9):
            return None
        self._last_s = t_s
        return [(name, self._poses[name]) for name in self.names if name in self._poses]


def run(run_dir: Path) -> int:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node
    from rclpy.parameter import Parameter
    from tf2_msgs.msg import TFMessage

    manifest = json.loads((run_dir / "scenario.json").read_text())
    truth = json.loads((run_dir / "truth.json").read_text())
    world = manifest["site"]
    selection = Selection(names_of(manifest, truth))
    rclpy.init()
    node = rclpy.create_node("argus_sim_truth", parameter_overrides=[Parameter("use_sim_time", value=manifest.get("sim_time") is True)])
    publisher = node.create_publisher(TFMessage, TOPIC, 10)
    lock = threading.Lock()
    sent = [0]

    def on_poses(message) -> None:
        t_s = message.header.stamp.sec + message.header.stamp.nsec * 1.0e-9
        with lock:
            for pose in message.pose:
                selection.add(pose.name, (pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
            poses = selection.due(t_s)
            if poses is None:
                return
            out = TFMessage()
            for name, (x, y, z, qx, qy, qz, qw) in poses:
                transform = TransformStamped()
                transform.header.stamp.sec, transform.header.stamp.nanosec = message.header.stamp.sec, message.header.stamp.nsec
                transform.header.frame_id, transform.child_frame_id = world, name
                transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = x, y, z
                rotation = transform.transform.rotation
                rotation.x, rotation.y, rotation.z, rotation.w = qx, qy, qz, qw
                out.transforms.append(transform)
            publisher.publish(out)
            sent[0] += 1

    gz_node = Node()
    ok = gz_node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", on_poses)
    ok = gz_node.subscribe(Pose_V, f"/world/{world}/pose/info", on_poses) and ok  # objects standing still are only in here
    if not ok:
        print(f"truth_poses: could not subscribe to the poses of Gazebo world '{world}'", file=sys.stderr)
        return 1
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    print(f"truth poses: {', '.join(selection.names)} on {TOPIC}", flush=True)
    while not stopping.is_set():
        rclpy.spin_once(node, timeout_sec=0.2)  # the node answers for its parameters: sim/time_check.py asks every node for use_sim_time
    print(f"truth poses: {sent[0]} messages", flush=True)
    return 0 if sent[0] else 1


def main(argv: Optional[List[str]] = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__)
        return 2
    return run(Path(argv[0]))


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
