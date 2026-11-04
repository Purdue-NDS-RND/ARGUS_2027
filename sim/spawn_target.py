#!/usr/bin/env python3
"""Spawn the red delivery target into the running Gazebo world.

By default the target goes at target_latitude/target_longitude from the
package_delivery mission in package_delivery_demo.yaml, placed relative to
the sim's home (the world origin; see sim/target_placement.py).

    python3 sim/spawn_target.py                      # the mission's target
    python3 sim/spawn_target.py --east 4 --north 0   # 4 m east of home
    python3 sim/spawn_target.py --remove             # delete it again

Needs `gz` on PATH and Gazebo running (sim/run_sim.sh --exec ... does both).
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
REPO = SIM_DIR.parent
sys.path.insert(0, str(SIM_DIR))
# sdf_objects.py has no ROS imports, so use it straight from the source tree.
sys.path.insert(0, str(REPO / "src" / "drone_utils"))

import yaml  # noqa: E402

from drone_utils.sdf_objects import build_red_white_target_sdf, spawn_sdf  # noqa: E402
from target_placement import read_world, world_xy_for, worlds_from_service_list  # noqa: E402

DEFAULT_WORLD = SIM_DIR / "worlds" / "argus_runway.sdf"
DEFAULT_SEQUENCE = (
    REPO / "src" / "drone_mission_demo" / "config" / "sequences" / "package_delivery_demo.yaml"
)


def mission_target(sequence_file: Path) -> tuple[float, float]:
    data = yaml.safe_load(sequence_file.read_text())
    for mission in data["mission_sequence"]["missions"]:
        config = mission.get("config") or {}
        if "target_latitude" in config and "target_longitude" in config:
            return float(config["target_latitude"]), float(config["target_longitude"])
    raise SystemExit(f"no mission with target_latitude/target_longitude in {sequence_file}")


def running_worlds(timeout_s: float) -> list[str]:
    deadline = time.monotonic() + timeout_s
    while True:
        result = subprocess.run(
            ["gz", "service", "-l"], capture_output=True, text=True, timeout=10
        )
        worlds = worlds_from_service_list(result.stdout)
        if worlds or time.monotonic() > deadline:
            return worlds
        time.sleep(1.0)


def remove_model(world: str, name: str) -> bool:
    result = subprocess.run(
        [
            "gz", "service", "-s", f"/world/{world}/remove",
            "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000", "--req", f'name: "{name}" type: MODEL',
        ],
        capture_output=True, text=True, timeout=10,
    )
    return result.returncode == 0 and "true" in result.stdout.lower()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--world-file", type=Path, default=DEFAULT_WORLD)
    parser.add_argument("--sequence", type=Path, default=DEFAULT_SEQUENCE)
    parser.add_argument("--name", default="delivery_target")
    parser.add_argument("--size", type=float, default=5.0, help="target diameter, m")
    parser.add_argument("--east", type=float, help="metres east of home (overrides the YAML)")
    parser.add_argument("--north", type=float, help="metres north of home (overrides the YAML)")
    parser.add_argument("--remove", action="store_true", help="remove the model instead")
    parser.add_argument("--wait", type=float, default=30.0, help="seconds to wait for Gazebo")
    args = parser.parse_args()

    world_name, origin = read_world(args.world_file.read_text())
    worlds = running_worlds(args.wait)
    if world_name not in worlds:
        print(
            f"ERROR: Gazebo is not running world '{world_name}' "
            f"(running: {worlds or 'none'}).",
            file=sys.stderr,
        )
        return 1

    if args.remove:
        ok = remove_model(world_name, args.name)
        print(f"{'Removed' if ok else 'Could not remove'} '{args.name}' from '{world_name}'")
        return 0 if ok else 1

    if args.east is not None or args.north is not None:
        x, y = (args.east or 0.0), (args.north or 0.0)
        source = "command line"
    else:
        lat, lon = mission_target(args.sequence)
        x, y = world_xy_for(origin, lat, lon)
        source = f"{args.sequence.name} ({lat:.7f}, {lon:.7f})"

    sdf = build_red_white_target_sdf(args.name, size=args.size)
    for _ in range(3):
        if spawn_sdf(sdf, args.name, x=x, y=y, z=0.0, world_name=world_name):
            print(
                f"Spawned '{args.name}' in '{world_name}' at x={x:.2f} m east, "
                f"y={y:.2f} m north of home, from {source}"
            )
            return 0
        time.sleep(2.0)
    print(f"ERROR: Gazebo refused to spawn '{args.name}' in '{world_name}'", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
