#!/usr/bin/env python3
"""Check the green X and bucket detector against the truth in a sim flight (REC-1). Runs inside the sim container.

  sim/run_sim.sh --exec python3 sim/recovery_cv_check.py <run folder>

Started by sim/runner/fly.py beside every flight whose scenario turns the
node flag `recovery_cv` on, and interrupted when the flight ends. It listens
to the detector's contract topic and to the true poses (sim/truth_poses.py),
puts the true centre of the X mat and of the bucket's rim through the sim
camera's own calibration and mount at each frame's stamp, and compares the
pixels with what the detector reported for that frame.

Pass lines, judged on the frames in which, by the truth, the whole mat was
in the picture (at least MIN_FRAMES of them):

  - the X is reported in at least FOUND_SHARE of them;
  - its centre is within the spec's line, max(2 px, 1% of the X's span in
    the picture), plus what the truth itself can be out by, at the 95th
    percentile: EXTRA_PX, and the distance the vehicle flies in one of the
    truth's own periods (TRUTH_PERIOD_S), in pixels.

The truth is a pose every 33 ms. At a survey's 4 m/s the vehicle flies
13 cm in that time, nine pixels from 10 m, and a frame's truth can be that
far from where the camera was: that is the truth's error, not the
detector's, and the allowance shrinks to EXTRA_PX as the vehicle slows.
The errors as measured, and those of the frames taken slower than
STILL_MPS, are in the report.

The bucket's numbers are in the report and not judged: from a survey's
height it is a dozen pixels across.

The run folder gets recovery_cv_check.json; sim/runner/result.py reads it.
The arithmetic and the verdict are plain functions with no ROS in them.
"""

from __future__ import annotations

import json
import math
import sys
import threading
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

DETECTIONS_TOPIC = "/perception/recovery_cv/detections"
MIN_FRAMES = 20
STILL_MPS = 1.0  # frames slower than this are also reported by themselves
TRUTH_PERIOD_S = 1.0 / 30.0  # sim/truth_poses.py publishes at 30 Hz
FOUND_SHARE = 0.95
NEAR_BELOW_M = 3.0  # under this height the bucket's own numbers are reported as the near field
CENTRE_PX, CENTRE_SHARE_OF_SPAN = 2.0, 0.01  # the spec's line for the X's centre
EXTRA_PX = 2.0  # over the spec's line with the vehicle still: stamps to the millisecond, a pose a frame
SPEED_OVER_S = 0.1  # the vehicle's speed at a frame is its true movement over this long before it
ARM_SPAN_OVER_MAT = 2.0 * (math.sqrt(2.0) / 2.0 - 0.05)  # arm tip to tip over the mat's side: 6.57 m on the 5 m mat
BUCKET_RIM_M = 0.2


def wanted(manifest: dict) -> bool:
    """Whether a built scenario turns the recovery detector on, so that its flight is checked."""
    return ((manifest.get("node_flags") or {}).get("recovery_cv") or {}).get("recovery_cv") is True


def _percentile(values: Sequence[float], fraction: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    return float(ordered[min(len(ordered) - 1, int(math.ceil(fraction * len(ordered))) - 1)])


def build_report(frames: List[dict]) -> dict:
    """The verdict on a flight's frames.

    Each frame: `whole` (the mat wholly in the picture, by the truth), `span_px` (the X's span there),
    `x_error_px` (None when no X was reported), `bucket_error_px` (None when no bucket was), `speed_mps`
    (the vehicle's true speed; None when it is not known, which counts as moving).
    """
    whole = [frame for frame in frames if frame.get("whole")]
    found = [frame for frame in whole if frame.get("x_error_px") is not None]
    judged = [frame for frame in found if frame.get("speed_mps") is not None]  # a frame whose speed is not known has no allowance to be given
    still = [frame["x_error_px"] for frame in judged if frame["speed_mps"] <= STILL_MPS]
    # How far each centre is over what it is allowed: the spec's line, EXTRA_PX, and one truth period of flight.
    over = [
        frame["x_error_px"] - max(CENTRE_PX, CENTRE_SHARE_OF_SPAN * frame["span_px"]) - EXTRA_PX - frame["speed_mps"] * TRUTH_PERIOD_S * frame.get("px_per_m", 0.0)
        for frame in judged
    ]
    errors = [frame["x_error_px"] for frame in judged]
    buckets = [frame["bucket_error_px"] for frame in whole if frame.get("bucket_error_px") is not None]
    # The near field (REC-5's hand-off): under NEAR_BELOW_M with the bucket's rim in the picture, where no X can be fitted
    # and the bucket is looked for by itself. Reported, not judged: the mission's own rules (a CONFIRMED track) judge it.
    near = [frame for frame in frames if frame.get("bucket_in_view") and frame["height_m"] < NEAR_BELOW_M]
    near_found = [frame for frame in near if frame.get("bucket_error_px") is not None]
    near_errors = [frame["bucket_error_px"] for frame in near_found]
    near_metres = [frame["bucket_error_px"] / frame["px_per_m"] for frame in near_found if frame.get("px_per_m")]
    share = len(found) / len(whole) if whole else 0.0
    over_p95 = _percentile(over, 0.95)
    problems = []
    if len(whole) < MIN_FRAMES:
        problems.append(f"only {len(whole)} frame(s) had the whole mat in the picture (at least {MIN_FRAMES} needed): the flight did not show the detector the X")
    else:
        if share < FOUND_SHARE:
            problems.append(f"the X was reported in {len(found)} of the {len(whole)} frames that had the whole mat in view ({100.0 * share:.0f}%; at least {100.0 * FOUND_SHARE:.0f}% needed)")
        if len(judged) < MIN_FRAMES:
            problems.append(f"only {len(judged)} of the frames with the X reported had a true speed to judge its centre by (at least {MIN_FRAMES} needed)")
        elif over_p95 is not None and over_p95 > 0.0:
            problems.append(f"the X's centre was {over_p95:.1f} px over what it is allowed at the 95th percentile (the spec's line, {EXTRA_PX:g} px, and one truth period of flight)")
    return {
        "passed": not problems,
        "problems": problems,
        "frames": len(frames),
        "frames_with_the_whole_mat": len(whole),
        "x_found": len(found),
        "x_found_share": round(share, 4),
        "x_error_px": {"median": _percentile(errors, 0.5), "p95": _percentile(errors, 0.95), "max": max(errors) if errors else None},
        "x_over_its_allowance_px_p95": over_p95,  # at or under 0 passes
        "x_error_px_slow": {"frames": len(still), "median": _percentile(still, 0.5), "p95": _percentile(still, 0.95)},  # slower than STILL_MPS
        "speeds_mps": {"median": _percentile([frame["speed_mps"] for frame in judged], 0.5), "max": max((frame["speed_mps"] for frame in judged), default=None)},
        # Of the judged frames, those whose truth lay between two true poses (the others had none after them yet,
        # and their truth is up to a period old).
        "truth_between_two_poses": sum(1 for frame in judged if frame.get("truth_between_two_poses")),
        "x_reported_without_the_whole_mat": sum(1 for frame in frames if not frame.get("whole") and frame.get("x_error_px") is not None),
        "bucket_found": len(buckets),
        "bucket_error_px": {"median": _percentile(buckets, 0.5), "p95": _percentile(buckets, 0.95)},
        "heights_m": {"lowest": min((frame["height_m"] for frame in whole), default=None), "highest": max((frame["height_m"] for frame in whole), default=None)},
        "near_field": {"below_m": NEAR_BELOW_M, "frames": len(near), "bucket_found": len(near_found), "bucket_found_share": round(len(near_found) / len(near), 4) if near else None,
                       "x_reported": sum(1 for frame in near if frame.get("x_error_px") is not None),
                       "bucket_error_px": {"median": _percentile(near_errors, 0.5), "p95": _percentile(near_errors, 0.95), "max": max(near_errors) if near_errors else None},
                       "bucket_error_m_at_the_ground": {"median": _percentile(near_metres, 0.5), "p95": _percentile(near_metres, 0.95)},
                       "lowest_m": min((frame["height_m"] for frame in near_found), default=None)},  # fmt: skip
    }


def frame_of(detections: Sequence[Tuple[str, float, float]], truth_x_px, truth_bucket_px, corners_px, size: Tuple[int, int], height_m: float, mat_m: float, speed_mps: Optional[float] = None) -> dict:
    """One frame's comparison. `detections` are (class, cx, cy); the truth pixels are None when behind the camera."""
    width, height = size
    whole = corners_px is not None and all(0.0 <= u < width and 0.0 <= v < height for u, v in corners_px)
    span = ARM_SPAN_OVER_MAT * mat_m * (math.dist(corners_px[0], corners_px[2]) / (math.sqrt(2.0) * mat_m)) if corners_px else 0.0

    def error(class_id, truth):
        seen = [(cx, cy) for name, cx, cy in detections if name == class_id]
        return None if not seen or truth is None else min(math.hypot(cx - truth[0], cy - truth[1]) for cx, cy in seen)

    return {
        "whole": bool(whole),
        "bucket_in_view": truth_bucket_px is not None and 0.0 <= truth_bucket_px[0] < width and 0.0 <= truth_bucket_px[1] < height,
        "span_px": float(span),
        "height_m": float(height_m),
        "speed_mps": None if speed_mps is None else float(speed_mps),
        "px_per_m": float(span) / (ARM_SPAN_OVER_MAT * mat_m) if span else 0.0,
        "x_error_px": error("green_x", truth_x_px),
        "bucket_error_px": error("bucket", truth_bucket_px),
    }


def read_check(folder) -> Optional[dict]:
    try:
        report = json.loads((Path(folder) / "recovery_cv_check.json").read_text())
    except (OSError, json.JSONDecodeError):
        return None
    return report if isinstance(report, dict) and isinstance(report.get("passed"), bool) else None


def summary_lines(report: dict) -> List[str]:
    errors = report.get("x_error_px") or {}
    line = (
        f"{'PASS' if report['passed'] else 'FAIL'}: the X reported in {report.get('x_found', 0)} of {report.get('frames_with_the_whole_mat', 0)} frames with the whole mat in view"
        + ("" if errors.get("median") is None else f", centre error median {errors['median']:.1f} px and p95 {errors['p95']:.1f} px at up to {(report.get('speeds_mps') or {}).get('max') or 0.0:.1f} m/s")
        + f"; the bucket in {report.get('bucket_found', 0)}"
    )
    lines = [line]
    near = report.get("near_field") or {}
    if near.get("frames"):
        errors = near.get("bucket_error_px") or {}
        metres = near.get("bucket_error_m_at_the_ground") or {}
        found = f"near field (under {near['below_m']:g} m, the bucket's rim in view): the bucket reported in {near['bucket_found']} of {near['frames']} frames"
        if errors.get("median") is not None:
            found += f", down to {near['lowest_m']:.2f} m; centre error median {errors['median']:.1f} px and p95 {errors['p95']:.1f} px (about {metres['median']:.3f} m and {metres['p95']:.3f} m)"
        lines.append(found + f"; the X in {near.get('x_reported', 0)} of them")
    return lines + [f"problem: {problem}" for problem in report.get("problems") or []]


def listen(run_dir: Path) -> dict:
    """Listen until interrupted, writing recovery_cv_check.json into the run folder as it goes."""
    import time

    import rclpy
    import yaml
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from tf2_msgs.msg import TFMessage
    from vision_msgs.msg import Detection2DArray

    from argus_localisation import oracle
    from drone_mission_core.frame_projection import load_camera
    from drone_mission_core.projection_v2 import ProjectionError, pixel_of_ground_point

    manifest = json.loads((run_dir / "scenario.json").read_text())
    truth_file = json.loads((run_dir / "truth.json").read_text())
    settings = yaml.safe_load((run_dir / "params.yaml").read_text())["recovery_cv"]["ros__parameters"]
    camera, mount = load_camera(str(settings["camera_mode"]), str(settings["airframe"]))
    objects = {item["kind"]: item for item in truth_file.get("objects", []) if item.get("kind") in ("x_mat", "bucket")}
    if "x_mat" not in objects:
        raise RuntimeError("the scenario has no x_mat on the field: nothing to check the recovery detector against")
    mat_m = float(objects["x_mat"].get("size_m", 5.0))
    history = oracle.TruthHistory(manifest["vehicle"]["model"])
    frames: List[dict] = []
    lock = threading.Lock()
    rclpy.init()
    node = rclpy.create_node("argus_recovery_cv_check", parameter_overrides=[Parameter("use_sim_time", value=manifest.get("sim_time") is True)])

    def on_truth(message) -> None:
        poses, stamp_ns = {}, None
        for transform in message.transforms:
            stamp_ns = int(transform.header.stamp.sec) * 1_000_000_000 + int(transform.header.stamp.nanosec)
            at, turn = transform.transform.translation, transform.transform.rotation
            poses[str(transform.child_frame_id)] = (at.x, at.y, at.z, turn.x, turn.y, turn.z, turn.w)
        if stamp_ns is not None:
            with lock:
                history.add(stamp_ns, poses)

    def pixel(point, position, orientation):
        try:
            return pixel_of_ground_point(camera, mount, point, position, orientation)
        except ProjectionError:
            return None

    def on_detections(message) -> None:
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        with lock:
            state = history.at(stamp_ns)
            before = history.at(stamp_ns - int(SPEED_OVER_S * 1.0e9))
            between = bool(history._stamps) and history._stamps[-1] >= stamp_ns  # a true pose after the frame had come: the truth is between two
        if state is None:
            return  # no truth near this frame's time: it cannot be judged
        position, orientation, poses = state
        speed = None if before is None else float(math.dist(position, before[0])) / SPEED_OVER_S
        mat = poses.get(objects["x_mat"]["name"])
        if mat is None:
            return
        yaw = oracle._yaw(mat[3:7])
        half = mat_m / 2.0
        corners = [pixel((mat[0] + dx * math.cos(yaw) - dy * math.sin(yaw), mat[1] + dx * math.sin(yaw) + dy * math.cos(yaw), mat[2]), position, orientation) for dx, dy in ((-half, -half), (half, -half), (half, half), (-half, half))]
        bucket = poses.get(objects["bucket"]["name"]) if "bucket" in objects else None
        found = [(str(d.results[0].hypothesis.class_id), float(d.bbox.center.position.x), float(d.bbox.center.position.y)) for d in message.detections if d.results]
        frame = frame_of(
            found,
            pixel((mat[0], mat[1], mat[2]), position, orientation),
            None if bucket is None else pixel((bucket[0], bucket[1], bucket[2] + BUCKET_RIM_M), position, orientation),
            None if any(corner is None for corner in corners) else corners,
            (int(camera.width), int(camera.height)),
            float(position[2]) - float(mat[2]),
            mat_m,
            speed,
        )
        frame["truth_between_two_poses"] = between
        with lock:
            frames.append(frame)

    node.create_subscription(TFMessage, oracle.TRUTH_TOPIC, on_truth, qos_profile_sensor_data)
    node.create_subscription(Detection2DArray, DETECTIONS_TOPIC, on_detections, 50)

    def write() -> dict:
        with lock:
            report = build_report(list(frames))
        (run_dir / "recovery_cv_check.json").write_text(json.dumps(report, indent=2) + "\n")
        return report

    last = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() - last > 5.0:
                write()
                last = time.monotonic()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    report = write()
    for line in summary_lines(report):
        print(f"recovery_cv_check: {line}", flush=True)
    return report


def main(argv: Optional[List[str]] = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print("usage: recovery_cv_check.py <run folder>", file=sys.stderr)
        return 2
    report = listen(Path(argv[0]).resolve())
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
