#!/usr/bin/env python3
"""Check the flow odometry against the true velocity in a flight (REC-5). Runs inside the sim container.

  python3 sim/flow_check.py <run folder>

The runner starts this beside every flight whose scenario turns the node flag
`flow_odometry` on. It listens to the node's own samples
(/argus/flow/status: every sample, with its quality and why not) and to the
vehicle's true pose in Gazebo. A sample is the mean velocity over the time
between its two frames, so it is compared with the true displacement over
that same time, divided by it, east and north.

The run passes when, over the samples with quality whose true height was 5 to
15 m (the spec's band):

  - the error's RMS (the length of the east-north difference) is 0.15 m/s or
    less (the spec's pass line);
  - there are at least MIN_SAMPLES of them: a flight whose ground gave the
    camera nothing to follow has not shown the odometry, and fails with that
    reason, not with a number.

Reported and not judged: the share of samples with quality in the band, the
RMS at other heights, the commonest reasons for no quality, and the error
along and across the true direction of flight (a scale error shows along it).
The run folder gets flow_check.json.
"""

from __future__ import annotations

import bisect
import json
import math
import sys
import threading
from pathlib import Path
from typing import Dict, List, Optional, Tuple

SIM_DIR = Path(__file__).resolve().parent
REPO = SIM_DIR.parent
sys.path.insert(0, str(SIM_DIR))

FLAG = "flow_odometry"
STATUS_TOPIC = "/argus/flow/status"
BAND_M = (5.0, 15.0)
RMS_LIMIT_MPS = 0.15
MIN_SAMPLES = 100
HEIGHT_BANDS = {"under 2 m": (-1.0, 2.0), "2 to 5 m": (2.0, 5.0), "5 to 10 m": (5.0, 10.0), "10 to 15 m": (10.0, 15.0), "over 15 m": (15.0, 1000.0)}  # reported, not judged


def wanted(manifest: dict) -> bool:
    return ((manifest.get("node_flags") or {}).get(FLAG) or {}).get(FLAG) is True


class TrueTrack:
    """The vehicle's true position over sim time, and its mean velocity over any stretch it has poses around."""

    def __init__(self) -> None:
        self._t: List[float] = []
        self._rows: List[Tuple[float, float, float]] = []
        self.start_z: Optional[float] = None

    def add(self, t_s: float, position) -> None:
        if self._t and t_s <= self._t[-1]:
            return
        if self.start_z is None:
            self.start_z = float(position[2])
        self._t.append(float(t_s))
        self._rows.append((float(position[0]), float(position[1]), float(position[2])))

    def _at(self, t_s: float) -> Optional[Tuple[float, float, float]]:
        index = bisect.bisect_left(self._t, t_s)
        if index == 0 or index >= len(self._t):
            return None
        t0, t1 = self._t[index - 1], self._t[index]
        if t1 - t0 > 0.5:
            return None
        fraction = (t_s - t0) / (t1 - t0)
        return tuple(a + fraction * (b - a) for a, b in zip(self._rows[index - 1], self._rows[index]))

    def mean_velocity(self, t_end_s: float, dt_s: float) -> Optional[dict]:
        """East and north metres a second over [t_end - dt, t_end], and the height above the start at its middle."""
        if dt_s <= 0:
            return None
        start, end = self._at(t_end_s - dt_s), self._at(t_end_s)
        if start is None or end is None:
            return None
        return {"east_mps": (end[0] - start[0]) / dt_s, "north_mps": (end[1] - start[1]) / dt_s, "height_m": 0.5 * (start[2] + end[2]) - self.start_z}


def _rms(values: List[float]) -> Optional[float]:
    return None if not values else math.sqrt(sum(value * value for value in values) / len(values))


def build_report(samples: List[dict], truth: TrueTrack) -> dict:
    """The verdict from the node's samples (the status topic's own fields) and the true track."""
    in_band: List[float] = []
    elsewhere: List[float] = []
    along: List[float] = []
    across: List[float] = []
    band_all = band_quality = unmatched = 0
    whys: Dict[str, int] = {}
    bands: Dict[str, Dict[str, list]] = {name: {"all": [], "errors": [], "along": []} for name in HEIGHT_BANDS}
    for sample in samples:
        true = truth.mean_velocity(sample["stamp_ns"] * 1.0e-9, sample["dt_s"])
        if true is None:
            unmatched += 1
            continue
        inside = BAND_M[0] <= true["height_m"] <= BAND_M[1]
        band_all += inside
        band = next((name for name, (low, high) in HEIGHT_BANDS.items() if low <= true["height_m"] < high), None)
        if band is not None:
            bands[band]["all"].append(1)
        if sample["quality"] <= 0:
            whys[sample["why"]] = whys.get(sample["why"], 0) + 1
            continue
        d_east, d_north = sample["east_mps"] - true["east_mps"], sample["north_mps"] - true["north_mps"]
        error = math.hypot(d_east, d_north)
        speed_true = math.hypot(true["east_mps"], true["north_mps"])
        if band is not None:
            bands[band]["errors"].append(error)
            if speed_true > 0.5:
                bands[band]["along"].append((d_east * true["east_mps"] + d_north * true["north_mps"]) / speed_true / speed_true)
        if inside:
            band_quality += 1
            in_band.append(error)
            speed = math.hypot(true["east_mps"], true["north_mps"])
            if speed > 0.5:
                unit = (true["east_mps"] / speed, true["north_mps"] / speed)
                along.append(d_east * unit[0] + d_north * unit[1])
                across.append(-d_east * unit[1] + d_north * unit[0])
        else:
            elsewhere.append(error)
    rms = _rms(in_band)
    problems = []
    if len(in_band) < MIN_SAMPLES:
        problems.append(f"only {len(in_band)} sample(s) with quality at {BAND_M[0]:g} to {BAND_M[1]:g} m (of {band_all} there): the flight did not show the odometry (at least {MIN_SAMPLES} are needed)")
    elif rms > RMS_LIMIT_MPS:
        problems.append(f"the velocity error's RMS at {BAND_M[0]:g} to {BAND_M[1]:g} m is {rms:.3f} m/s, over {RMS_LIMIT_MPS} m/s")
    return {"passed": not problems, "problems": problems, "samples": len(samples), "unmatched": unmatched, "in_band": band_all, "in_band_with_quality": band_quality,
            "quality_share": None if band_all == 0 else band_quality / band_all, "rms_mps": rms, "rms_limit_mps": RMS_LIMIT_MPS, "rms_elsewhere_mps": _rms(elsewhere), "elsewhere": len(elsewhere),
            "along_mean_mps": None if not along else sum(along) / len(along), "across_mean_mps": None if not across else sum(across) / len(across),
            "p95_mps": None if not in_band else sorted(in_band)[min(len(in_band) - 1, math.ceil(0.95 * len(in_band)) - 1)],
            "no_quality": dict(sorted(whys.items(), key=lambda item: -item[1])[:5]),
            "by_height": {name: {"samples": len(found["all"]), "with_quality": len(found["errors"]), "rms_mps": _rms(found["errors"]),
                                 "scale_error": None if not found["along"] else sum(found["along"]) / len(found["along"])} for name, found in bands.items()}}  # fmt: skip


def read_flow_check(folder) -> Optional[dict]:
    try:
        return json.loads((Path(folder) / "flow_check.json").read_text())
    except (OSError, json.JSONDecodeError):
        return None


def summary_lines(report: dict) -> List[str]:
    if report["rms_mps"] is None:
        head = f"flow odometry: no sample with quality at {BAND_M[0]:g} to {BAND_M[1]:g} m ({report['samples']} samples heard)"
    else:
        share = "" if report["quality_share"] is None else f" ({report['quality_share']:.0%} of the samples there had quality)"
        head = f"flow odometry: velocity error RMS {report['rms_mps']:.3f} m/s over {report['in_band_with_quality']} samples at {BAND_M[0]:g} to {BAND_M[1]:g} m{share}; the line is {RMS_LIMIT_MPS} m/s"
    lines = [head]
    if report.get("along_mean_mps") is not None:
        lines.append(f"  mean error along the flight {report['along_mean_mps']:+.3f} m/s, across it {report['across_mean_mps']:+.3f} m/s; 95th percentile {report['p95_mps']:.3f} m/s")
    if report.get("rms_elsewhere_mps") is not None:
        lines.append(f"  at other heights: RMS {report['rms_elsewhere_mps']:.3f} m/s over {report['elsewhere']} samples (not judged)")
    for name, found in (report.get("by_height") or {}).items():
        if found["samples"]:
            rms = "no quality" if found["rms_mps"] is None else f"RMS {found['rms_mps']:.3f} m/s"
            scale = "" if found.get("scale_error") is None else f", speed read {found['scale_error']:+.1%} off along the flight"
            lines.append(f"  {name}: {found['with_quality']} of {found['samples']} samples with quality, {rms}{scale}")
    if report.get("no_quality"):
        lines.append("  no quality: " + "; ".join(f"{why} ({count})" for why, count in report["no_quality"].items()))
    return lines + [f"  FAILED: {problem}" for problem in report["problems"]]


class Checker:
    def __init__(self) -> None:
        self._truth = TrueTrack()
        self._samples: List[dict] = []
        self._lock = threading.Lock()

    def on_truth(self, t_s: float, position) -> None:
        with self._lock:
            self._truth.add(t_s, position)

    def on_status(self, text: str) -> None:
        try:
            sample = json.loads(text)
            kept = {"stamp_ns": int(sample["stamp_ns"]), "dt_s": float(sample["dt_s"]), "quality": int(sample["quality"]), "why": str(sample.get("why", "")),
                    "east_mps": float(sample.get("east_mps", 0.0)), "north_mps": float(sample.get("north_mps", 0.0)), "height_m": float(sample.get("height_m", 0.0)),
                    "height_source": str(sample.get("height_source", "")), "patches": int(sample.get("patches", 0))}  # fmt: skip
        except (ValueError, KeyError, TypeError):
            return
        with self._lock:
            self._samples.append(kept)

    def report(self) -> dict:
        with self._lock:
            return build_report(list(self._samples), self._truth)

    def table(self) -> str:
        """Every sample beside the truth of its moment, as CSV: for reading a flight phase by phase."""
        with self._lock:
            rows = ["t_s,quality,patches,east_mps,north_mps,true_east_mps,true_north_mps,height_m,height_source,true_height_m,why"]
            for sample in self._samples:
                true = self._truth.mean_velocity(sample["stamp_ns"] * 1.0e-9, sample["dt_s"])
                if true is None:
                    continue
                rows.append(f"{sample['stamp_ns'] * 1.0e-9:.3f},{sample['quality']},{sample['patches']},{sample['east_mps']:.3f},{sample['north_mps']:.3f},{true['east_mps']:.3f},{true['north_mps']:.3f},"
                            f"{sample['height_m']:.2f},{sample['height_source']},{true['height_m']:.2f},{sample['why'].replace(',', ';')}")  # fmt: skip
            return "\n".join(rows) + "\n"


def listen(run_dir: Path) -> dict:
    """Listen until interrupted, writing flow_check.json into the run folder as it goes."""
    import time

    import rclpy
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node as GzNode
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    from std_msgs.msg import String

    manifest = json.loads((run_dir / "scenario.json").read_text())
    if manifest.get("sim_time") is not True:
        raise RuntimeError("the flow check needs a run on sim time: the samples' stamps are compared with Gazebo's clock")
    vehicle = manifest["vehicle"]["model"]
    checker = Checker()

    def on_poses(message) -> None:
        t_s = message.header.stamp.sec + message.header.stamp.nsec * 1.0e-9
        for pose in message.pose:
            if pose.name == vehicle:
                checker.on_truth(t_s, (pose.position.x, pose.position.y, pose.position.z))

    gz_node = GzNode()
    if not gz_node.subscribe(Pose_V, f"/world/{manifest['site']}/dynamic_pose/info", on_poses):
        raise RuntimeError("could not subscribe to Gazebo's poses")
    rclpy.init()
    node = rclpy.create_node("argus_flow_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
    node.create_subscription(String, STATUS_TOPIC, lambda message: checker.on_status(message.data), 50)

    def write() -> dict:
        report = checker.report()
        (run_dir / "flow_check.json").write_text(json.dumps(report, indent=2) + "\n")
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
    (run_dir / "flow_check_samples.csv").write_text(checker.table())
    return write()


def main(argv: Optional[List[str]] = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__)
        return 2
    report = listen(Path(argv[0]))
    print("\n".join(summary_lines(report)), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
