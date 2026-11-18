#!/usr/bin/env python3
"""Check the sim's sensors in a flight (SIM-05): the camera's frame rate and the rangefinder against the true height. Runs inside the sim container.

  python3 sim/sensor_check.py <run folder>

Started by sim/runner/fly.py beside a flight whose scenario asks for it
(`sensor_check: true`), and interrupted when the flight ends. It listens to
the camera's frames as they arrive in ROS (as raw bytes: only each frame's
stamp and size are read), to its camera_info, to MAVROS's rangefinder,
to Gazebo's true pose of the vehicle and, when the scenario publishes true
poses (`truth_poses: true`), to /sim/truth/poses, which must then be there.

Pass lines (FEATURES.md, SIM-05):
  - a camera profile with a floor on its rate (`survey`: 8 Hz) delivers at
    least that, in sim time, over the flight; the real-time factor is the
    runner's own figure and is judged there (0.8);
  - in a hover, the rangefinder is within 0.1 m of the true height: the
    95th percentile of |range - true height| over the readings taken while
    the vehicle was above 2 m, moving slower than 0.3 m/s and tilted less
    than 5 degrees, of which there must be at least 50.

"True height" is how far the vehicle has risen from where it stood: SITL's
rangefinder reads 0 on the ground. The run folder gets sensor_check.json;
sim/runner/result.py reads it. The arithmetic is plain functions and
classes with no ROS or Gazebo in them.
"""

from __future__ import annotations

import bisect
import json
import math
import os
import struct
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

CAMERA_INFO_TOPIC = "/camera/camera_info"
IMAGE_TOPIC = "/camera/image"
FLOW_INFO_TOPIC = "/argus/camera/flow/camera_info"
RANGEFINDER_TOPIC = "/mavros/rangefinder/rangefinder"
RANGE_LIMIT_M = 0.1
HOVER_MIN_HEIGHT_M = 2.0
HOVER_MAX_SPEED_MPS = 0.3
HOVER_MAX_TILT_DEG = 5.0
MIN_HOVER_READINGS = 50
RATE_WINDOW_S = 5.0
TRUTH_TOPIC = "/sim/truth/poses"  # sim/truth_poses.py publishes at 30 Hz of sim time
TRUTH_MIN_RATE_HZ = 25.0


def wanted(manifest: dict) -> bool:
    return manifest.get("sensor_check") is True


def frame_rate(stamps_s: List[float]) -> Optional[dict]:
    """Frames per second of sim time: over the whole span, and the slowest 5 s window."""
    stamps = sorted(set(stamps_s))
    if len(stamps) < 2 or stamps[-1] - stamps[0] <= 0.0:
        return None
    windows = []
    start = stamps[0]
    while start + RATE_WINDOW_S <= stamps[-1]:
        count = bisect.bisect_left(stamps, start + RATE_WINDOW_S) - bisect.bisect_left(stamps, start)
        windows.append(count / RATE_WINDOW_S)
        start += RATE_WINDOW_S
    return {"frames": len(stamps), "rate_hz": (len(stamps) - 1) / (stamps[-1] - stamps[0]), "slowest_window_hz": min(windows) if windows else None}


def image_header(data: bytes) -> Optional[Tuple[float, int, int]]:
    """(stamp in seconds, width, height) of a serialised sensor_msgs/Image, read from its first bytes.

    The check takes the camera's frames as raw bytes, so that a 25 MB frame
    is not unpacked into Python just to be counted. The layout is CDR's:
    four bytes of encapsulation, the stamp (int32 seconds, uint32
    nanoseconds), the frame id (a length, the text, padding to four bytes),
    then height and width.
    """
    try:
        order = "<" if data[1] == 1 else ">"
        sec, nanosec, length = struct.unpack_from(order + "iII", data, 4)
        at = 16 + length
        at += -at % 4
        height, width = struct.unpack_from(order + "II", data, at)
    except (struct.error, IndexError):
        return None
    return sec + nanosec * 1.0e-9, width, height


STAMP_WINDOW_S = 1.0  # a rangefinder stamp this far behind the sim clock, or at all ahead of it, is off the clock
STAMP_MAX_AGE_P95_S = 0.1  # on a sim-time run the readings' stamps must be this close to the sim clock when heard,
STAMP_MAX_OFF_CLOCK = 0.05  # and no more than this share of them off the clock


def place_reading(now_s: float, stamp_s: float) -> Tuple[float, Optional[float]]:
    """(the time a rangefinder reading is placed at, its age when heard) on a sim-time run.

    A reading stamped on the sim clock is placed at its stamp, as flight
    code does when it looks up the pose of a reading. One whose stamp is
    ahead of the clock or more than a second behind is off the clock: it is
    placed at the time it was heard and has no age (None), and
    `stamp_problem` counts it.
    """
    age = now_s - stamp_s
    if 0.0 <= age < STAMP_WINDOW_S:
        return stamp_s, age
    return now_s, None


def stamp_problem(readings: int, ages_s: List[float]) -> Optional[str]:
    """Why a sim-time run's rangefinder stamps cannot be trusted, or None."""
    if readings == 0:
        return None
    off_clock = readings - len(ages_s)
    if off_clock > STAMP_MAX_OFF_CLOCK * readings:
        return f"{off_clock} of {readings} rangefinder readings were stamped off the sim clock (ahead of it, or over {STAMP_WINDOW_S:g} s behind)"
    summary = age_summary(ages_s)
    if summary is not None and summary["p95_ms"] > 1000.0 * STAMP_MAX_AGE_P95_S:
        return f"rangefinder readings were {summary['p95_ms']:.0f} ms old by their stamps at the 95th percentile (limit {1000.0 * STAMP_MAX_AGE_P95_S:.0f} ms)"
    return None


def age_summary(ages_s: List[float]) -> Optional[dict]:
    """How old the rangefinder's readings were when heard, by their own stamps (sim-time runs): median and 95th percentile, in ms."""
    if not ages_s:
        return None
    ordered = sorted(ages_s)
    return {"count": len(ordered), "median_ms": 1000.0 * ordered[len(ordered) // 2], "p95_ms": 1000.0 * ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)]}


class TrueHeight:
    """The vehicle's true height above where it stood, its speed and its tilt, at any sim time it has poses around."""

    def __init__(self) -> None:
        self._t: List[float] = []
        self._rows: List[Tuple[float, float, float, float]] = []  # x, y, z, tilt in degrees
        self.start_z: Optional[float] = None

    def add(self, t_s: float, position, orientation) -> None:
        if self._t and t_s <= self._t[-1]:
            return
        if self.start_z is None:
            self.start_z = float(position[2])
        x, y = float(orientation[0]), float(orientation[1])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))
        self._t.append(t_s)
        self._rows.append((float(position[0]), float(position[1]), float(position[2]), tilt))

    def at(self, t_s: float) -> Optional[dict]:
        """Height, speed and tilt at `t_s`, from the poses either side of it; None outside them or across a gap over 0.5 s."""
        index = bisect.bisect_left(self._t, t_s)
        if index == 0 or index >= len(self._t):
            return None
        t0, t1 = self._t[index - 1], self._t[index]
        if t1 - t0 > 0.5:
            return None
        before, after = self._rows[index - 1], self._rows[index]
        fraction = (t_s - t0) / (t1 - t0)
        height = before[2] + fraction * (after[2] - before[2]) - self.start_z
        speed = math.dist(before[:3], after[:3]) / (t1 - t0)
        return {"height_m": height, "speed_mps": speed, "tilt_deg": max(before[3], after[3])}


def range_check(readings: List[Tuple[float, float]], truth: TrueHeight) -> dict:
    """The rangefinder against the true height, over the readings taken in a hover. Each reading is (sim time, range)."""
    errors = []
    for t_s, range_m in readings:
        state = truth.at(t_s)
        if state is None or not math.isfinite(range_m):
            continue
        if state["height_m"] >= HOVER_MIN_HEIGHT_M and state["speed_mps"] <= HOVER_MAX_SPEED_MPS and state["tilt_deg"] <= HOVER_MAX_TILT_DEG:
            errors.append(abs(range_m - state["height_m"]))
    errors.sort()
    result = {"readings": len(readings), "in_hover": len(errors), "limit_m": RANGE_LIMIT_M, "median_m": None, "p95_m": None, "max_m": None, "passed": False}
    if len(errors) >= MIN_HOVER_READINGS:
        result.update(median_m=errors[len(errors) // 2], p95_m=errors[min(len(errors) - 1, math.ceil(0.95 * len(errors)) - 1)], max_m=errors[-1])
        result["passed"] = result["p95_m"] <= RANGE_LIMIT_M
    return result


def build_report(
    camera_stamps: List[float],
    flow_stamps: List[float],
    readings: List[Tuple[float, float]],
    truth: TrueHeight,
    min_rate_hz: Optional[float],
    flow_expected: bool,
    truth_stamps: Optional[List[float]] = None,
    image_stamps: Optional[List[float]] = None,
    image_size: Optional[Tuple[int, int]] = None,
    truth_names: Optional[List[str]] = None,
    truth_expected: Optional[List[str]] = None,
    range_ages_s: Optional[List[float]] = None,
    stamps_judged: bool = False,
) -> dict:
    """The check's verdict.

    `camera_stamps` are the camera_info stamps and `image_stamps` the stamps
    of the frames that arrived on the image topic (None: the frames were not
    listened for, as in the unit tests of the other rules). The rate floor
    is held against the frames that arrived. `stamps_judged`: a sim-time
    run, where the rangefinder's stamps are held to the sim clock
    (`range_ages_s` are the ages of those on it). `truth_stamps`, `truth_names`:
    what was heard on /sim/truth/poses when the scenario publishes it, and
    `truth_expected` the names that must be among them.
    """
    camera, flow, rangefinder = frame_rate(camera_stamps), frame_rate(flow_stamps), range_check(readings, truth)
    images = None if image_stamps is None else frame_rate(image_stamps)
    truth_topic = None if truth_stamps is None else frame_rate(truth_stamps)
    problems = []
    if truth_stamps is not None and (truth_topic is None or truth_topic["rate_hz"] < TRUTH_MIN_RATE_HZ):
        heard = "nothing" if truth_topic is None else f"{truth_topic['rate_hz']:.1f} messages a second of sim time"
        problems.append(f"true poses were asked for on {TRUTH_TOPIC} and {heard} was heard (at least {TRUTH_MIN_RATE_HZ:g} needed)")
    unheard = sorted(set(truth_expected or []) - set(truth_names or [])) if truth_stamps is not None else []
    if unheard:
        problems.append(f"no true pose was heard for: {', '.join(unheard)}")
    judged = camera if image_stamps is None else images  # the frames themselves, when they were listened for
    if judged is None:
        problems.append("no camera frames were heard")
    elif min_rate_hz is not None and judged["rate_hz"] < min_rate_hz:
        problems.append(f"the camera gave {judged['rate_hz']:.1f} frames a second of sim time (at least {min_rate_hz:g} needed)")
    if flow_expected and flow is None:
        problems.append("no flow-camera frames were heard")
    if rangefinder["in_hover"] < MIN_HOVER_READINGS:
        problems.append(f"only {rangefinder['in_hover']} rangefinder readings in a hover (at least {MIN_HOVER_READINGS} needed)")
    elif not rangefinder["passed"]:
        problems.append(f"the rangefinder is {rangefinder['p95_m']:.2f} m from the true height at the 95th percentile in a hover (limit {RANGE_LIMIT_M} m)")
    rangefinder["stamp_age"] = age_summary(range_ages_s or [])
    if stamps_judged:  # a sim-time run: the readings' own stamps must be on the sim clock
        problem = stamp_problem(len(readings), range_ages_s or [])
        if problem:
            problems.append(problem)
    if truth_topic is not None:
        truth_topic["names"] = sorted(set(truth_names or []))
    return {
        "passed": not problems,
        "problems": problems,
        "camera": camera,
        "images": images,
        "image_size": None if image_size is None else list(image_size),
        "min_rate_hz": min_rate_hz,
        "flow_camera": flow,
        "rangefinder": rangefinder,
        "truth_poses": truth_topic,
    }


def read_sensor_check(folder) -> Optional[dict]:
    try:
        report = json.loads((Path(folder) / "sensor_check.json").read_text())
    except (OSError, json.JSONDecodeError):
        return None
    return report if isinstance(report, dict) and isinstance(report.get("passed"), bool) else None


def summary_lines(report: dict) -> List[str]:
    lines = []
    camera, images, flow, rangefinder = report.get("camera"), report.get("images"), report.get("flow_camera"), report.get("rangefinder") or {}
    floor = "" if report.get("min_rate_hz") is None else f" (at least {report['min_rate_hz']:g} needed)"
    if images:
        size = "" if not report.get("image_size") else f" of {report['image_size'][0]} x {report['image_size'][1]}"
        slowest = "" if images.get("slowest_window_hz") is None else f", slowest 5 s {images['slowest_window_hz']:.1f} Hz"
        lines.append(f"camera: {images['rate_hz']:.1f} frames{size} a second of sim time arrived, {images['frames']} in all{slowest}{floor}")
    elif camera:
        slowest = "" if camera.get("slowest_window_hz") is None else f", slowest 5 s {camera['slowest_window_hz']:.1f} Hz"
        lines.append(f"camera: {camera['rate_hz']:.1f} frames a second of sim time over {camera['frames']} frames{slowest}{floor}")
    if flow:
        lines.append(f"flow camera: {flow['rate_hz']:.1f} frames a second of sim time over {flow['frames']} frames")
    if report.get("truth_poses"):
        names = report["truth_poses"].get("names") or []
        lines.append(
            f"true poses on {TRUTH_TOPIC}: {report['truth_poses']['rate_hz']:.1f} messages a second of sim time over {report['truth_poses']['frames']}"
            + (f", for {', '.join(names)}" if names else "")
        )
    if rangefinder.get("p95_m") is not None:
        age = rangefinder.get("stamp_age")
        lines.append(
            f"rangefinder against the true height in a hover: {'PASS' if rangefinder['passed'] else 'FAIL'}, median {rangefinder['median_m']:.3f} m, "
            f"p95 {rangefinder['p95_m']:.3f} m over {rangefinder['in_hover']} readings (limit {rangefinder['limit_m']} m)"
            + ("" if not age else f"; readings {age['median_ms']:.0f} ms old by their stamps at the median, {age['p95_ms']:.0f} ms at p95")
        )
    return lines


def _stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1.0e-9


def listen(run_dir: Path) -> dict:
    import rclpy
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node as GzNode
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, Range
    from tf2_msgs.msg import TFMessage

    manifest = json.loads((run_dir / "scenario.json").read_text())
    vehicle = manifest["vehicle"]
    truth = TrueHeight()
    lock = threading.Lock()
    camera_stamps: List[float] = []
    flow_stamps: List[float] = []
    readings: List[Tuple[float, float]] = []
    truth_stamps: Optional[List[float]] = [] if manifest.get("truth_poses") is True else None
    truth_names: set = set()
    truth_expected = [vehicle["model"]] + [item["name"] for item in json.loads((run_dir / "truth.json").read_text()).get("objects", [])]
    image_stamps: List[float] = []
    image_size: List[Optional[Tuple[int, int]]] = [None]
    range_ages: List[float] = []
    on_sim_time = manifest.get("sim_time") is True
    sim_now = [0.0]

    def on_poses(message) -> None:
        t_s = message.header.stamp.sec + message.header.stamp.nsec * 1.0e-9
        with lock:
            sim_now[0] = t_s
            for pose in message.pose:
                if pose.name == vehicle["model"]:
                    truth.add(t_s, (pose.position.x, pose.position.y, pose.position.z), (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))

    gz_node = GzNode()
    if not gz_node.subscribe(Pose_V, f"/world/{manifest['site']}/dynamic_pose/info", on_poses):
        raise RuntimeError("could not subscribe to Gazebo's poses")
    rclpy.init()
    node = rclpy.create_node("argus_sensor_check", parameter_overrides=[Parameter("use_sim_time", value=manifest.get("sim_time") is True)])

    def on_range(message) -> None:
        with lock:
            when = sim_now[0]  # on a wall-time run MAVROS's stamp is not on the sim's clock: Gazebo's time now is used
            if on_sim_time:  # on sim time the reading is placed at its own stamp, and how old it was when heard is kept
                when, age = place_reading(sim_now[0], _stamp_s(message.header.stamp))
                if age is not None:
                    range_ages.append(age)
            readings.append((when, float(message.range)))

    def on_image(data: bytes) -> None:
        header = image_header(bytes(data[:300]))
        if header is not None:
            image_stamps.append(header[0])
            image_size[0] = (header[1], header[2])

    def on_truth(message) -> None:
        if message.transforms:
            truth_stamps.append(_stamp_s(message.transforms[0].header.stamp))
            truth_names.update(transform.child_frame_id for transform in message.transforms)

    # The frames themselves, as raw bytes: the rate that counts is of frames that arrived, not of camera_info.
    node.create_subscription(Image, IMAGE_TOPIC, on_image, qos_profile_sensor_data, raw=True)

    node.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, lambda m: camera_stamps.append(_stamp_s(m.header.stamp)), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, FLOW_INFO_TOPIC, lambda m: flow_stamps.append(_stamp_s(m.header.stamp)), qos_profile_sensor_data)
    node.create_subscription(Range, RANGEFINDER_TOPIC, on_range, qos_profile_sensor_data)
    if truth_stamps is not None:
        node.create_subscription(TFMessage, TRUTH_TOPIC, on_truth, 10)

    def write() -> dict:
        with lock:
            report = build_report(
                list(camera_stamps), list(flow_stamps), list(readings), truth, vehicle.get("min_rate_hz"), bool(vehicle.get("flow_camera")),
                None if truth_stamps is None else list(truth_stamps),
                image_stamps=list(image_stamps), image_size=image_size[0], truth_names=sorted(truth_names), truth_expected=truth_expected, range_ages_s=list(range_ages), stamps_judged=on_sim_time,
            )  # fmt: skip
        (run_dir / "sensor_check.json").write_text(json.dumps(report, indent=2) + "\n")
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


def main(argv: Optional[List[str]] = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__)
        return 2
    report = listen(Path(argv[0]))
    for line in summary_lines(report):
        print(line)
    for problem in report["problems"]:
        print(f"  {problem}")
    print("sensor check: PASS" if report["passed"] else "sensor check: FAIL", flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
