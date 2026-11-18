#!/usr/bin/env python3
"""Time one step of the target estimator's feed (FND-06) on this machine. No ROS.

  python3 tools/estimator_step_cost.py [--steps 200]

The estimator runs inside the mission executor, on the same thread as the
20 Hz tick, so a step's cost is time the tick waits. Two cases are timed with
the code the executor runs (projection v2 with the rangefinder's ground, the
filter, the assignment): the ordinary one, a frame with one bullseye, and the
worst the feed's bounds allow, four frames of sixteen detections each with
sixteen tracks alive. Run it on the flight computer to know what the bounds
cost there.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
for package in ("drone_mission_core", "argus_camera"):
    sys.path.insert(0, str(REPO / "src" / package))

MS, T0 = 1_000_000, 500_000_000_000
CLASSES = ("bullseye", "green_x", "marker", "boundary_marker", "bucket")


def measure(detections_per_frame: int, frames_per_step: int, steps: int) -> dict:
    """Step times in milliseconds for `steps` steps, each with `frames_per_step` new frames of `detections_per_frame` detections."""
    from drone_mission_core.detection_contract import Detection, DetectionSet
    from drone_mission_core.estimator_feed import EstimatorFeed
    from drone_mission_core.frame_projection import load_camera
    from drone_mission_core.pose_buffer import PoseBuffer
    from drone_mission_core.projection_v2 import GroundHeight, GroundProjector

    camera, mount = load_camera("sim_delivery", "sim_iris_downcam")
    poses = PoseBuffer()
    ground = GroundHeight(poses.pose_at)
    feed = EstimatorFeed(GroundProjector(camera, mount, poses.pose_at, ground), poses)
    last_ns = T0 + (200 + steps * 20) * MS
    stamp = T0
    while stamp <= last_ns:  # a hover at 15 m, the rangefinder healthy
        poses.add_pose(stamp, (0.0, 0.0, 15.0), (0.0, 0.0, 0.0, 1.0))
        poses.add_velocity(stamp, (0.0, 0.0, 0.0))
        ground.add_range(stamp, 15.0, 0.1, 40.0)
        stamp += 33 * MS
    found = tuple(
        Detection(CLASSES[index % len(CLASSES)], 0.9, 60.0 + 34.0 * (index % 16), 60.0 + 70.0 * (index // 4 % 4), 24.0, 24.0) for index in range(detections_per_frame)
    )
    times = []
    for index in range(steps):
        base = T0 + (200 + index * 20) * MS
        for frame in range(frames_per_step):
            feed.add_detections(DetectionSet(base + frame * MS, "down_camera_optical", found))
        start = time.perf_counter()
        feed.step(base + 10 * MS)
        times.append((time.perf_counter() - start) * 1000.0)
    ordered = sorted(times)
    return {
        "steps": steps,
        "median_ms": statistics.median(ordered),
        "p95_ms": ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))],
        "max_ms": ordered[-1],
        "tracks": len(feed.estimates(base + 10 * MS)),
        "measurements": feed.counters["measurements"],
    }


def main(argv: list[str] | None = None) -> int:
    from drone_mission_core import estimator_feed

    parser = argparse.ArgumentParser(description="Time one step of the target estimator's feed on this machine.")
    parser.add_argument("--steps", type=int, default=200, help="steps to time in each case (default 200)")
    args = parser.parse_args(argv)
    cases = (
        ("one bullseye in one frame", 1, 1),
        (
            f"the worst the bounds allow ({estimator_feed.MAX_FRAMES_PER_STEP} frames of {estimator_feed.MAX_DETECTIONS_PER_FRAME} detections)",
            estimator_feed.MAX_DETECTIONS_PER_FRAME,
            estimator_feed.MAX_FRAMES_PER_STEP,
        ),
    )
    for label, per_frame, frames in cases:
        result = measure(per_frame, frames, max(1, args.steps))
        print(
            f"{label}: median {result['median_ms']:.2f} ms, p95 {result['p95_ms']:.2f} ms, max {result['max_ms']:.2f} ms "
            f"over {result['steps']} steps, {result['tracks']} tracks (the tick is 50 ms)"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
