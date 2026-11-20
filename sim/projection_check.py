#!/usr/bin/env python3
"""Check projection v2 against the truth in a sim flight (FND-05). Runs inside the sim container.

  sim/run_sim.sh --exec python3 sim/projection_check.py <run folder>

Started by sim/runner/fly.py beside every flight whose scenario turns the
mission flag use_frame_time_pose on, and interrupted when the flight ends.
It listens to what the mission listens to (the detector's target point and
image size, MAVROS's pose, velocity and rangefinder), projects every
detection of the target to the ground with the same code the mission uses
(projection_v2.py: the pose at the frame's stamp, the sim camera's own
calibration and mount), and compares the point with where the scenario put
the target (truth.json), in the frame of MAVROS's local position, whose
origin is the take-off point.

Pass line (FEATURES.md, FND-05): projected bullseye against the truth within
0.3 m at 15 m. Judged on the 95th percentile of the detections taken between
13.5 and 16.5 m in frames that, by the truth, had the whole target in the
picture, of which there must be at least 20. A frame with the target partly
out of view is counted apart and not judged: there the detector reports the
middle of the part it sees, up to half the target away, and the projection
has no part in that. (With no target size in the truth, every frame is
judged.) The run folder gets projection_check.json; sim/runner/result.py
reads it.

For comparison the report also has the same pixels projected with the newest
pose at the moment the detection arrived, which is what 2026 did.

The arithmetic and the verdict are plain functions with no ROS in them.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Optional

REPO = Path(__file__).resolve().parents[1]
for package in ("drone_mission_core", "argus_camera"):
    sys.path.insert(0, str(REPO / "src" / package))

LIMIT_M = 0.3
HEIGHT_M = 15.0
HEIGHT_BAND_M = 1.5
MIN_IN_BAND = 20
FLAG = "use_frame_time_pose"
DETECTION_TOPIC = "/drone_package_drop/target_detection"
IMAGE_SIZE_TOPIC = "/drone_package_drop/image_size"
POSE_TOPIC = "/mavros/local_position/pose"
VELOCITY_TOPIC = "/mavros/local_position/velocity_local"
RANGEFINDER_TOPIC = "/mavros/rangefinder/rangefinder"
DEFAULT_CAMERA_MODE, DEFAULT_AIRFRAME = "sim_delivery", "sim_iris_downcam"


def wanted(manifest: dict) -> bool:
    """Whether a built scenario turns the projection flag on and has a target, so that its flight is checked.

    A scenario with no target (a survey) gives projection v2 nothing to be
    compared with. A manifest that does not say has one, as every scenario
    had before the survey's.
    """
    return (manifest.get("flags") or {}).get(FLAG) is True and manifest.get("target", True) is not False


def target_in_local_frame(truth: dict) -> tuple[float, float]:
    """The scenario's target, east and north of the take-off point: the origin of MAVROS's local position."""
    target, takeoff = truth["target"], truth.get("takeoff") or {"e": 0.0, "n": 0.0}
    return (float(target["e"]) - float(takeoff["e"]), float(target["n"]) - float(takeoff["n"]))


def target_size_from(truth: dict) -> Optional[float]:
    """How wide the scenario's target is, in metres, when the truth says: the size of the object the target is."""
    name = (truth.get("target") or {}).get("name")
    for item in truth.get("objects") or []:
        if name is not None and item.get("name") == name and isinstance(item.get("size_m"), (int, float)):
            return float(item["size_m"])
    return None


def _spread(values: list[float]) -> Optional[dict]:
    if not values:
        return None
    ordered = sorted(values)
    return {
        "count": len(ordered),
        "median_m": ordered[len(ordered) // 2],
        "p95_m": ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)],
        "max_m": ordered[-1],
    }


def build_report(records: list[dict], refusals: dict[str, int], detections: int) -> dict:
    """The verdict for a flight.

    Each record is one detection of the target that was projected:
    `height_m` of the lens above the ground, `error_m` from the truth,
    `latest_pose_error_m` (the same pixel with the newest pose, or None),
    `height_source`, and where in the image it was (`x_norm`, `y_norm`).
    """
    in_band = [r for r in records if abs(r["height_m"] - HEIGHT_M) <= HEIGHT_BAND_M]
    # A frame in which the target was partly out of the picture (by the truth) is not a detection of its centre:
    # the detector reports the middle of the part in view. Such frames are counted apart, not judged.
    band = [r for r in in_band if r.get("whole", True)]
    partly = [r for r in in_band if not r.get("whole", True)]
    at_height = _spread([r["error_m"] for r in band])
    problems = []
    if detections == 0:
        problems.append("no detection of the target was heard")
    elif not records:
        problems.append(f"none of {detections} detections could be projected ({', '.join(sorted(refusals)) or 'no reason recorded'})")
    elif at_height is None or at_height["count"] < MIN_IN_BAND:
        count = 0 if at_height is None else at_height["count"]
        problems.append(f"only {count} projected detections between {HEIGHT_M - HEIGHT_BAND_M:g} and {HEIGHT_M + HEIGHT_BAND_M:g} m (at least {MIN_IN_BAND} needed)")
    elif at_height["p95_m"] > LIMIT_M:
        problems.append(f"projected target is {at_height['p95_m']:.2f} m from the truth at the 95th percentile at {HEIGHT_M:g} m (limit {LIMIT_M} m)")
    sources: dict[str, int] = {}
    for record in records:
        sources[record["height_source"]] = sources.get(record["height_source"], 0) + 1
    band_sources: dict[str, int] = {}
    for record in band:
        band_sources[record["height_source"]] = band_sources.get(record["height_source"], 0) + 1
    worst = sorted(band, key=lambda record: record["error_m"], reverse=True)[:5]
    return {
        "passed": not problems,
        "problems": problems,
        "limit_m": LIMIT_M,
        "detections": detections,
        "projected": len(records),
        "refusals": dict(sorted(refusals.items())),
        "at_15_m": at_height,
        "at_15_m_target_partly_out_of_view": _spread([r["error_m"] for r in partly]),
        "at_15_m_with_the_latest_pose": _spread([r["latest_pose_error_m"] for r in band if r["latest_pose_error_m"] is not None]),
        "all_heights": _spread([r["error_m"] for r in records]),
        "below_6_m": _spread([r["error_m"] for r in records if r["height_m"] < 6.0]),
        "height_sources": sources,
        "height_sources_at_15_m": band_sources,
        # The five largest errors at 15 m, to tell a projection fault from a detection at the image's edge.
        "worst_at_15_m": [
            {key: round(record[key], 3) if isinstance(record[key], float) else record[key] for key in ("error_m", "height_m", "x_norm", "y_norm", "height_source", "stamp_ns")}
            for record in worst
        ],
    }


def read_projection_check(folder) -> dict | None:
    """The report a run folder holds, or None if there is none that can be read."""
    try:
        report = json.loads((Path(folder) / "projection_check.json").read_text())
    except (OSError, json.JSONDecodeError):
        return None
    return report if isinstance(report, dict) and isinstance(report.get("passed"), bool) else None


class Checker:
    """Projects each detection as it arrives and keeps the comparison. No ROS: the node below feeds it."""

    def __init__(self, target_en: tuple[float, float], camera_mode: str = DEFAULT_CAMERA_MODE, airframe: str = DEFAULT_AIRFRAME, target_size_m: Optional[float] = None) -> None:
        from drone_mission_core.frame_projection import TargetOffset, load_camera
        from drone_mission_core.pose_buffer import PoseBuffer
        from drone_mission_core.projection_v2 import GroundHeight, GroundProjector

        self.target = target_en
        self.target_size_m = target_size_m  # None: not known, and every frame counts as showing the whole target
        self.camera, self.mount = load_camera(camera_mode, airframe)
        self.poses = PoseBuffer()
        self.ground = GroundHeight(self.poses.pose_at)
        self.offset = TargetOffset(GroundProjector(self.camera, self.mount, self.poses.pose_at, self.ground))
        self.image_size: Optional[tuple[int, int]] = None
        self.records: list[dict] = []
        self.refusals: dict[str, int] = {}
        self.detections = 0

    def on_detection(self, stamp_ns: int, x_norm: float, y_norm: float) -> None:
        if x_norm == -1.0 and y_norm == -1.0:
            return  # the detector's "nothing found"
        self.detections += 1
        if self.image_size is None:
            self.refusals["the image size was not known yet"] = self.refusals.get("the image size was not known yet", 0) + 1
            return
        if self.offset.offset(x_norm, y_norm, self.image_size, stamp_ns, (0.0, 0.0)) is None:
            reason = (self.offset.last_refusal or "no reason given").split(":")[0]
            self.refusals[reason] = self.refusals.get(reason, 0) + 1
            return
        point = self.offset.last_point
        self.records.append(
            {
                "stamp_ns": stamp_ns,
                "height_m": point.camera_enu[2] - point.up_m,
                "error_m": math.hypot(point.east_m - self.target[0], point.north_m - self.target[1]),
                "latest_pose_error_m": self._with_the_latest_pose(x_norm, y_norm, point.up_m),
                "height_source": point.height_source,
                "x_norm": float(x_norm),
                "y_norm": float(y_norm),
                "whole": self._whole_target_in_view(stamp_ns),
            }
        )

    def _whole_target_in_view(self, stamp_ns: int) -> bool:
        """Whether, by the truth, all of the target was inside the picture in the frame taken at `stamp_ns`."""
        if self.target_size_m is None:
            return True
        from drone_mission_core.projection_v2 import pixel_of_ground_point

        pose = self.poses.pose_at(stamp_ns)
        if pose is None:
            return True
        radius = 0.5 * self.target_size_m
        for step in range(8):  # eight points round the target's edge
            angle = step * math.pi / 4.0
            point = (self.target[0] + radius * math.cos(angle), self.target[1] + radius * math.sin(angle), 0.0)
            pixel = pixel_of_ground_point(self.camera, self.mount, point, pose.position, pose.orientation)
            if pixel is None or not (0.0 <= pixel[0] < self.camera.width and 0.0 <= pixel[1] < self.camera.height):
                return False
        return True

    def _with_the_latest_pose(self, x_norm: float, y_norm: float, ground_up_m: float) -> Optional[float]:
        from drone_mission_core.projection_v2 import project

        newest = self.poses.newest_ns()
        pose = None if newest is None else self.poses.pose_at(newest)
        if pose is None:
            return None
        pixel = (0.5 * (x_norm + 1.0) * self.camera.width, 0.5 * (y_norm + 1.0) * self.camera.height)
        point = project(self.camera, self.mount, pixel, pose.position, pose.orientation, ground_up_m)
        return None if point is None else math.hypot(point.east_m - self.target[0], point.north_m - self.target[1])

    def report(self) -> dict:
        return build_report(self.records, self.refusals, self.detections)


# --- the ROS side ---------------------------------------------------------------------


def _stamp_ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def listen(folder: Path) -> dict:
    """Listen until interrupted, writing projection_check.json into the folder as it goes."""
    import time

    import rclpy
    from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Range

    truth = json.loads((folder / "truth.json").read_text())
    checker = Checker(target_in_local_frame(truth), target_size_m=target_size_from(truth))
    rclpy.init()
    node = rclpy.create_node("argus_projection_check", parameter_overrides=[Parameter("use_sim_time", value=True)])

    def on_pose(message) -> None:
        p, q = message.pose.position, message.pose.orientation
        checker.poses.add_pose(_stamp_ns(message.header.stamp), (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def on_velocity(message) -> None:
        v = message.twist.linear
        checker.poses.add_velocity(_stamp_ns(message.header.stamp), (v.x, v.y, v.z))

    def on_size(message) -> None:
        checker.image_size = (int(message.point.x), int(message.point.y))

    node.create_subscription(PoseStamped, POSE_TOPIC, on_pose, qos_profile_sensor_data)
    node.create_subscription(TwistStamped, VELOCITY_TOPIC, on_velocity, qos_profile_sensor_data)
    node.create_subscription(Range, RANGEFINDER_TOPIC, lambda m: checker.ground.add_range(_stamp_ns(m.header.stamp), m.range, m.min_range, m.max_range), qos_profile_sensor_data)
    node.create_subscription(PointStamped, IMAGE_SIZE_TOPIC, on_size, 10)
    node.create_subscription(PointStamped, DETECTION_TOPIC, lambda m: checker.on_detection(_stamp_ns(m.header.stamp), m.point.x, m.point.y), 50)

    def write() -> dict:
        report = checker.report()
        (folder / "projection_check.json").write_text(json.dumps(report, indent=2) + "\n")
        return report

    last = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - last >= 5.0:
                last = time.monotonic()
                write()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    return write()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Compare projection v2's ground point for the target with the scenario's truth.")
    parser.add_argument("folder", type=Path, help="the run folder: truth.json is read from it and projection_check.json written to it")
    args = parser.parse_args(argv)
    report = listen(args.folder)
    at_height = report["at_15_m"]
    if at_height:
        print(f"projected target against the truth at {HEIGHT_M:g} m: median {at_height['median_m']:.2f} m, p95 {at_height['p95_m']:.2f} m over {at_height['count']} detections")
    partly = report.get("at_15_m_target_partly_out_of_view")
    if partly:
        print(f"  not judged: {partly['count']} detections with the target partly out of the picture (up to {partly['max_m']:.2f} m off)")
    for problem in report["problems"]:
        print(f"  {problem}")
    print("projection check: PASS" if report["passed"] else "projection check: FAIL", flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)  # as sim/time_check.py: leave without tearing the interpreter down
