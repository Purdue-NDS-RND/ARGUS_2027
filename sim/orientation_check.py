#!/usr/bin/env python3
"""Check, with numbers, that the sim camera steers the mission the right way.

A mirrored or rotated camera can look fine in the Gazebo window and still
steer the drone away from the target. This flies the real delivery mission
with its target set to home, so the drone climbs straight up over the spawn
point, and puts a red target a few metres east (or north) of it. Then it
records the first detection and the first centring velocity setpoint:

  target east : detection right of centre (x_norm > 0), setpoint east  (v_east > 0)
  target north: detection above centre   (y_norm < 0), setpoint north (v_north > 0)

The vehicle spawns facing north, so image top is north.

Start the sim without the mission target and run one direction per sim start:
  SPAWN_TARGET=0 sim/run_sim.sh
  sim/run_sim.sh --exec python3 sim/orientation_check.py --direction east
  sim/run_sim.sh --down     # then start again for --direction north

The functions above main() have no ROS imports; sim/test/test_orientation_check.py
tests them.
"""

from __future__ import annotations

import argparse
import copy
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
REPO = SIM_DIR.parent
SEQUENCE = REPO / "src" / "drone_mission_demo" / "config" / "sequences" / "package_delivery_demo.yaml"
WORLD = SIM_DIR / "worlds" / "argus_runway.sdf"
TARGET_NAME = "orientation_target"


def check_sequence(sequence: dict, home_lat: float, home_lon: float) -> dict:
    """The demo sequence with only the delivery target moved to home."""

    out = copy.deepcopy(sequence)
    found = 0
    for mission in out["mission_sequence"]["missions"]:
        config = mission.get("config") or {}
        if "target_latitude" in config and "target_longitude" in config:
            config["target_latitude"] = float(home_lat)
            config["target_longitude"] = float(home_lon)
            found += 1
    if found != 1:
        raise ValueError(f"expected one mission with a target, found {found}")
    return out


def target_offset(direction: str, distance_m: float) -> tuple[float, float]:
    """World (east, north) of the check target."""

    if direction == "east":
        return float(distance_m), 0.0
    if direction == "north":
        return 0.0, float(distance_m)
    raise ValueError(f"direction must be east or north, got {direction!r}")


@dataclass(frozen=True)
class Verdict:
    ok: bool
    detection_ok: bool
    setpoint_ok: bool
    expected: str


def verdict(direction: str, x_norm: float, y_norm: float, v_east: float, v_north: float) -> Verdict:
    """Does the detection and first centring setpoint point at the target?

    The off-axis component must be the smaller one, so a camera rotated 90
    degrees (east target seen above centre) fails even if one sign happens
    to match.
    """

    if direction == "east":
        det = x_norm > 0.0 and abs(x_norm) > abs(y_norm)
        sp = v_east > 0.0 and abs(v_east) > abs(v_north)
        expected = "x_norm > 0 (right of centre), v_east > 0"
    elif direction == "north":
        det = y_norm < 0.0 and abs(y_norm) > abs(x_norm)
        sp = v_north > 0.0 and abs(v_north) > abs(v_east)
        expected = "y_norm < 0 (above centre), v_north > 0"
    else:
        raise ValueError(f"direction must be east or north, got {direction!r}")
    return Verdict(det and sp, det, sp, expected)


def yaw_enu_deg(qx: float, qy: float, qz: float, qw: float) -> float:
    """ENU yaw in degrees (0 = facing east, 90 = facing north)."""

    return math.degrees(math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))


def main() -> int:  # pragma: no cover - needs the running sim
    import rclpy
    import yaml
    from geometry_msgs.msg import PointStamped, PoseStamped
    from mavros_msgs.msg import PositionTarget
    from rclpy.qos import qos_profile_sensor_data

    sys.path.insert(0, str(SIM_DIR))
    from target_placement import read_world

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--direction", choices=("east", "north"), required=True)
    parser.add_argument("--distance", type=float, default=4.0, help="metres from home")
    parser.add_argument("--timeout", type=float, default=420.0, help="seconds")
    args = parser.parse_args()

    state_dir = Path(os.environ.get("ARGUS_STATE", SIM_DIR / ".state"))
    state_dir.mkdir(parents=True, exist_ok=True)
    _, origin = read_world(WORLD.read_text())

    sequence = check_sequence(
        yaml.safe_load(SEQUENCE.read_text()), origin.latitude_deg, origin.longitude_deg
    )
    seq_file = state_dir / f"orientation_{args.direction}.yaml"
    seq_file.write_text(yaml.safe_dump(sequence, sort_keys=False))

    east, north = target_offset(args.direction, args.distance)
    subprocess.run(
        [sys.executable, str(SIM_DIR / "spawn_target.py"), "--name", TARGET_NAME,
         "--east", str(east), "--north", str(north)],
        check=True,
    )

    log_path = state_dir / f"orientation_{args.direction}.launch.log"
    log = open(log_path, "w")
    launch = subprocess.Popen(
        ["ros2", "launch", "drone_mission_demo", "package_delivery_demo.launch.py",
         f"sequence:={seq_file}", "log_session:=false"],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
    )
    print(f"Mission launched with {seq_file.name} (log: {log_path.name}); target at "
          f"east={east:.1f} m, north={north:.1f} m. Waiting for the first centring setpoint...")

    rclpy.init()
    node = rclpy.create_node("orientation_check")
    seen: dict = {"detection": None, "first_detection": None, "pose": None, "setpoint": None}

    def on_detection(msg: PointStamped) -> None:
        if msg.point.x == -1.0 and msg.point.y == -1.0:
            return
        seen["detection"] = msg
        if seen["first_detection"] is None:
            seen["first_detection"] = (msg, seen["pose"])

    def on_pose(msg: PoseStamped) -> None:
        seen["pose"] = msg

    def on_setpoint(msg: PositionTarget) -> None:
        velocity_only = msg.type_mask & PositionTarget.IGNORE_PX
        if seen["setpoint"] is None and velocity_only and seen["detection"] is not None:
            if math.hypot(msg.velocity.x, msg.velocity.y) > 0.01:
                seen["setpoint"] = (msg, seen["detection"], seen["pose"])

    node.create_subscription(PointStamped, "/drone_package_drop/target_detection", on_detection, 10)
    node.create_subscription(PoseStamped, "/mavros/local_position/pose", on_pose, qos_profile_sensor_data)
    node.create_subscription(PositionTarget, "/mavros/setpoint_raw/local", on_setpoint, 10)

    deadline = time.monotonic() + args.timeout
    try:
        while seen["setpoint"] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if launch.poll() is not None:
                print(f"ERROR: the mission launch exited early; see {log_path}", file=sys.stderr)
                return 1
    finally:
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
        log.close()

    if seen["setpoint"] is None:
        print(f"ERROR: no centring setpoint within {args.timeout:.0f} s; see {log_path}", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    def pose_text(pose) -> str:
        if pose is None:
            return "n/a"
        p, q = pose.pose.position, pose.pose.orientation
        return (f"alt {p.z:.2f} m, local E {p.x:.2f} N {p.y:.2f} m, "
                f"yaw {yaw_enu_deg(q.x, q.y, q.z, q.w):.1f} deg ENU")

    first_det, first_pose = seen["first_detection"]
    sp, det, pose = seen["setpoint"]
    v = verdict(args.direction, det.point.x, det.point.y, sp.velocity.x, sp.velocity.y)

    print(f"\nTarget {args.distance:.1f} m {args.direction} of home")
    print(f"  first detection:        x_norm {first_det.point.x:+.3f}, y_norm {first_det.point.y:+.3f}"
          f"  ({pose_text(first_pose)})")
    print(f"  detection at setpoint:  x_norm {det.point.x:+.3f}, y_norm {det.point.y:+.3f}")
    print(f"  first centring setpoint: v_east {sp.velocity.x:+.3f} m/s, v_north {sp.velocity.y:+.3f} m/s,"
          f" v_up {sp.velocity.z:+.3f} m/s  ({pose_text(pose)})")
    print(f"  expected: {v.expected}")
    print(f"  detection {'OK' if v.detection_ok else 'WRONG'}, setpoint {'OK' if v.setpoint_ok else 'WRONG'}"
          f" -> {'PASS' if v.ok else 'FAIL'}")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if v.ok else 1


if __name__ == "__main__":
    sys.exit(main())
