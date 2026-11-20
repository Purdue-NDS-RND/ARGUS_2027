#!/usr/bin/env python3
"""Check the height estimator against the true height in a flight (CEN-03). Runs inside the sim container.

  python3 sim/height_check.py <run folder>

The runner starts this beside every flight whose scenario turns
`use_height_estimator` on. It runs the estimator itself
(drone_mission_core/height_estimator.py, the module the executor uses) on
the flight's own topics: MAVROS's pose, into a pose history, and MAVROS's
rangefinder. At every pose it asks for the height above the ground and
keeps the answer, its source and, for a barometer height, the reason. The
true height is the vehicle's in Gazebo, above where it stood before
take-off.

The run passes when

  - where the truth says the rangefinder should work (0.5 to 11 m up, tilted
    10 degrees or less), at least 90% of the heights came from it;
  - the heights that came from the rangefinder are within 0.10 m of the truth
    at the 95th percentile (the line SIM-05 holds the rangefinder itself to);
  - no height came from the rangefinder above 12.5 m, beyond the sensor's
    12 m range, or while the truth says the vehicle was tilted over 17
    degrees (the estimator's limit is 15, by the flight controller's
    attitude).

A flight that never comes into the rangefinder's band has nothing to judge
and passes; the report says so. The barometer heights' error is reported and
not judged: it is the flight controller's height estimate, not ours. The run
folder gets height_check.json.
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading
from pathlib import Path
from typing import List, Optional, Tuple

SIM_DIR = Path(__file__).resolve().parent
REPO = SIM_DIR.parent
sys.path.insert(0, str(SIM_DIR))
sys.path.insert(0, str(REPO / "src" / "drone_mission_core"))

from sensor_check import TrueHeight  # noqa: E402

FLAG = "use_height_estimator"
POSE_TOPIC = "/mavros/local_position/pose"
RANGEFINDER_TOPIC = "/mavros/rangefinder/rangefinder"
RANGE_LIMIT_M = 0.10  # heights from the rangefinder against the truth, 95th percentile
IN_BAND_M = (0.5, 11.0)  # true heights at which the rangefinder must be the source...
IN_BAND_MAX_TILT_DEG = 10.0  # ...while the vehicle is tilted no more than this
IN_BAND_MIN_SHARE = 0.9
ABOVE_SENSOR_M = 12.5  # no rangefinder height above this true height
TILTED_OVER_DEG = 17.0  # nor while the truth says the vehicle is tilted over this (the estimator's limit is 15, by the flight controller's attitude)
MIN_SAMPLES = 50  # fewer samples in the band than this are not judged
REASON_KINDS = (
    ("outside its range", "out of range"),
    ("tilt", "tilt"),
    ("has not changed", "stuck reading"),
    ("old", "stale"),
    ("disagree", "readings disagree"),
    ("expected", "ground not where expected"),
    ("no rangefinder reading", "no reading"),
    ("not a number", "not a number"),
    ("usable rangefinder reading", "too few readings"),
    ("no pose is known", "no pose for the readings"),
)


def wanted(manifest: dict) -> bool:
    """Whether a built scenario turns the height estimator's flag on, so that its flight is checked."""
    return (manifest.get("flags") or {}).get(FLAG) is True


def reason_kind(reason: str) -> str:
    for needle, kind in REASON_KINDS:
        if needle in reason:
            return kind
    return "other"


def _spread(errors: List[float]) -> dict:
    errors = sorted(errors)
    if not errors:
        return {"count": 0, "median_m": None, "p95_m": None, "max_m": None}
    return {"count": len(errors), "median_m": errors[len(errors) // 2], "p95_m": errors[min(len(errors) - 1, math.ceil(0.95 * len(errors)) - 1)], "max_m": errors[-1]}


def build_report(samples: List[Tuple[float, float, str, str]], truth: TrueHeight) -> dict:
    """The verdict over the samples, each (sim time, height, source, reason), against the true height."""
    by_source: dict = {"rangefinder": [], "baro": []}
    reasons: dict = {}
    in_band = in_band_rangefinder = above = above_rangefinder = judged = unknown_source = tilted = tilted_rangefinder = 0
    for t_s, value_m, source, reason in samples:
        state = truth.at(t_s)
        if state is None:
            continue
        judged += 1
        if source not in by_source:
            unknown_source += 1
            continue
        by_source[source].append(abs(value_m - state["height_m"]))
        if source == "baro":
            kind = reason_kind(reason)
            reasons[kind] = reasons.get(kind, 0) + 1
        if IN_BAND_M[0] <= state["height_m"] <= IN_BAND_M[1] and state["tilt_deg"] <= IN_BAND_MAX_TILT_DEG:
            in_band += 1
            in_band_rangefinder += source == "rangefinder"
        if state["height_m"] > ABOVE_SENSOR_M:
            above += 1
            above_rangefinder += source == "rangefinder"
        if state["tilt_deg"] > TILTED_OVER_DEG:
            tilted += 1
            tilted_rangefinder += source == "rangefinder"
    rangefinder, baro = _spread(by_source["rangefinder"]), _spread(by_source["baro"])
    share = None if in_band == 0 else in_band_rangefinder / in_band
    problems = []
    if judged == 0:
        problems.append("no height was compared with the truth: no pose was heard, or none in the time the true poses cover")
    if unknown_source:
        problems.append(f"{unknown_source} heights named a source that is neither rangefinder nor baro")
    if in_band >= MIN_SAMPLES and share < IN_BAND_MIN_SHARE:
        problems.append(
            f"only {100.0 * share:.0f}% of the samples between {IN_BAND_M[0]:g} and {IN_BAND_M[1]:g} m and under {IN_BAND_MAX_TILT_DEG:g} degrees of tilt "
            f"took their height from the rangefinder (at least {100.0 * IN_BAND_MIN_SHARE:.0f}% needed)"
        )
    if rangefinder["count"] >= MIN_SAMPLES and rangefinder["p95_m"] > RANGE_LIMIT_M:
        problems.append(f"heights from the rangefinder are {rangefinder['p95_m']:.2f} m from the true height at the 95th percentile (limit {RANGE_LIMIT_M:g} m)")
    if above_rangefinder:
        problems.append(f"{above_rangefinder} heights came from the rangefinder above {ABOVE_SENSOR_M:g} m, beyond the sensor's range")
    if tilted_rangefinder:
        problems.append(f"{tilted_rangefinder} heights came from the rangefinder while the vehicle was tilted over {TILTED_OVER_DEG:g} degrees")
    rangefinder.update(limit_m=RANGE_LIMIT_M, passed=rangefinder["count"] >= MIN_SAMPLES and rangefinder["p95_m"] <= RANGE_LIMIT_M)
    baro["reasons"] = dict(sorted(reasons.items()))
    return {
        "passed": not problems,
        "problems": problems,
        "samples": judged,
        "rangefinder": rangefinder,
        "baro": baro,
        "in_band": {"count": in_band, "rangefinder_share": share, "min_share": IN_BAND_MIN_SHARE, "from_m": IN_BAND_M[0], "to_m": IN_BAND_M[1]},
        "above_sensor_range": {"count": above, "rangefinder": above_rangefinder},
        "tilted": {"count": tilted, "rangefinder": tilted_rangefinder},
    }


def read_height_check(folder) -> Optional[dict]:
    """The report a run folder holds, or None if there is none that can be read."""
    try:
        report = json.loads((Path(folder) / "height_check.json").read_text())
    except (OSError, json.JSONDecodeError):
        return None
    return report if isinstance(report, dict) and isinstance(report.get("passed"), bool) else None


def summary_lines(report: dict) -> List[str]:
    """Lines for the terminal and the run's report."""
    rangefinder, baro, band, above, tilted = (report.get(key) or {} for key in ("rangefinder", "baro", "in_band", "above_sensor_range", "tilted"))
    lines = []
    if rangefinder.get("count", 0) >= MIN_SAMPLES:
        lines.append(
            f"rangefinder source: {'PASS' if rangefinder.get('passed') else 'FAIL'}, median {rangefinder['median_m']:.2f} m, p95 {rangefinder['p95_m']:.2f} m "
            f"over {rangefinder['count']} heights (limit {rangefinder.get('limit_m', RANGE_LIMIT_M):g} m)"
        )
    else:
        lines.append(f"rangefinder source: {rangefinder.get('count', 0)} heights, nothing to judge")
    if band.get("count"):
        lines.append(f"between {band.get('from_m', IN_BAND_M[0]):g} and {band.get('to_m', IN_BAND_M[1]):g} m: {100.0 * band['rangefinder_share']:.0f}% of {band['count']} heights from the rangefinder")
    if baro.get("count"):
        why = ", ".join(f"{kind} {count}" for kind, count in (baro.get("reasons") or {}).items())
        lines.append(f"baro source: median {baro['median_m']:.2f} m, p95 {baro['p95_m']:.2f} m from the truth over {baro['count']} heights ({why})")
    if above.get("count"):
        lines.append(f"above {ABOVE_SENSOR_M:g} m: {above['count']} heights, {above.get('rangefinder', 0)} from the rangefinder")
    if tilted.get("count"):
        lines.append(f"tilted over {TILTED_OVER_DEG:g} degrees: {tilted['count']} heights, {tilted.get('rangefinder', 0)} from the rangefinder")
    return lines


class Checker:
    """The estimator, fed as the executor feeds it, and what it answered at every pose."""

    def __init__(self) -> None:
        from drone_mission_core.height_estimator import HeightEstimator
        from drone_mission_core.pose_buffer import PoseBuffer

        self._buffer = PoseBuffer()
        self._estimator = HeightEstimator(self._buffer.pose_at)
        self._truth = TrueHeight()
        self._samples: List[Tuple[float, float, str, str]] = []
        self._lock = threading.Lock()

    def on_truth(self, t_s: float, position, orientation) -> None:
        with self._lock:
            self._truth.add(t_s, position, orientation)

    def on_range(self, stamp_ns: int, range_m: float, min_m: float, max_m: float) -> None:
        with self._lock:
            self._estimator.add_range(stamp_ns, range_m, min_m, max_m)

    def on_pose(self, stamp_ns: int, position, orientation, when_s: float) -> None:
        """A pose from MAVROS. `when_s` is the sim time the sample is compared with the truth at."""
        with self._lock:
            self._buffer.add_pose(int(stamp_ns), tuple(float(v) for v in position), tuple(float(v) for v in orientation))
            height = self._estimator.height(position[2], int(stamp_ns), orientation=tuple(float(v) for v in orientation))
            if height is not None:
                self._samples.append((float(when_s), height.value_m, height.source, height.reason))

    def report(self) -> dict:
        with self._lock:
            return build_report(list(self._samples), self._truth)


def listen(run_dir: Path) -> dict:
    """Listen until interrupted, writing height_check.json into the run folder as it goes."""
    import time

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node as GzNode
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Range

    manifest = json.loads((run_dir / "scenario.json").read_text())
    vehicle = manifest["vehicle"]["model"]
    on_sim_time = manifest.get("sim_time") is True
    checker = Checker()
    sim_now = [0.0]

    def on_poses(message) -> None:
        t_s = message.header.stamp.sec + message.header.stamp.nsec * 1.0e-9
        sim_now[0] = t_s
        for pose in message.pose:
            if pose.name == vehicle:
                checker.on_truth(t_s, (pose.position.x, pose.position.y, pose.position.z), (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))

    gz_node = GzNode()
    if not gz_node.subscribe(Pose_V, f"/world/{manifest['site']}/dynamic_pose/info", on_poses):
        raise RuntimeError("could not subscribe to Gazebo's poses")
    rclpy.init()
    node = rclpy.create_node("argus_height_check", parameter_overrides=[Parameter("use_sim_time", value=on_sim_time)])

    def stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def on_pose(message) -> None:
        position, orientation = message.pose.position, message.pose.orientation
        ns = stamp_ns(message.header.stamp)
        # On sim time the pose's own stamp is on Gazebo's clock; on a wall-time run it is not, and Gazebo's time now is used.
        checker.on_pose(ns, (position.x, position.y, position.z), (orientation.x, orientation.y, orientation.z, orientation.w), ns * 1.0e-9 if on_sim_time else sim_now[0])

    node.create_subscription(PoseStamped, POSE_TOPIC, on_pose, qos_profile_sensor_data)
    node.create_subscription(Range, RANGEFINDER_TOPIC, lambda m: checker.on_range(stamp_ns(m.header.stamp), m.range, m.min_range, m.max_range), qos_profile_sensor_data)

    def write() -> dict:
        report = checker.report()
        (run_dir / "height_check.json").write_text(json.dumps(report, indent=2) + "\n")
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
        print(f"height estimator against the true height: {line}")
    for problem in report["problems"]:
        print(f"  {problem}")
    print("height check: PASS" if report["passed"] else "height check: FAIL", flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)  # as sim/time_check.py: leave without tearing the interpreter down
