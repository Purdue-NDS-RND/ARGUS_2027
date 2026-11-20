#!/usr/bin/env python3
"""Run the height estimator over a recorded flight (CEN-03's bench and field check). No ROS needed.

  python3 tools/height_log.py <bag>
  python3 tools/height_log.py <bag> --tape 2.4 --last 20
  python3 tools/height_log.py <bag> --ground-below-home 1.2 --out height.json

The bag is one made by the recorder (`ros2 launch sim/replay/record.launch.py
out:=<folder>`, or `sim/run_scenarios.sh ... --record`): it holds MAVROS's
pose and rangefinder. The estimator (drone_mission_core/height_estimator.py,
the module the executor uses) is fed both in the order they were recorded
and asked for the height at every pose. Printed:

  - by height, how many heights came from the rangefinder and how many from
    the barometer, and why the barometer was used;
  - the rangefinder's height minus the height above home, where both exist:
    its median is how far the ground under the flight is from home's height
    plus the flight controller's height error, and its spread is how well
    the two agree. This is the number the drop's barometer margin (3.0 m
    against 2.4 m, CEN-04) has to cover;
  - with --tape H: the vehicle was held at a height H measured with a tape
    for the last --last seconds of the recording (default 20). The check
    passes when the heights in that time came from the rangefinder and their
    median is within 0.10 m of H.

Exit code 0, or 1 when a taped check fails or the bag has no poses, 2 when
the bag cannot be read.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import List, Optional

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "sim"))
sys.path.insert(0, str(REPO / "src" / "drone_mission_core"))

from height_check import reason_kind  # noqa: E402  (sim/height_check.py: the same names for the reasons)
from replay import bag, topics  # noqa: E402

TAPE_LIMIT_M = 0.10
TAPE_DEFAULT_LAST_S = 20.0
TAPE_MIN_HEIGHTS = 50
BANDS = ((0.0, 3.0), (3.0, 6.0), (6.0, 9.0), (9.0, 12.0), (12.0, math.inf))


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _band_name(low: float, high: float) -> str:
    return f"{low:g} m and over" if math.isinf(high) else f"{low:g} to {high:g} m"


def _median(values: List[float]) -> float:
    values = sorted(values)
    return values[len(values) // 2]


def analyse(bag_path, ground_below_home_m: float = 0.0, tape_m: Optional[float] = None, last_s: float = TAPE_DEFAULT_LAST_S) -> dict:
    """The report for a recorded flight. Raises bag.BagError for a bag that cannot be read."""
    from drone_mission_core.height_estimator import HeightEstimator
    from drone_mission_core.pose_buffer import PoseBuffer

    buffer = PoseBuffer()
    estimator = HeightEstimator(buffer.pose_at)
    samples = []  # (stamp, height, source, reason, height above home)
    readings = 0
    for message in bag.read_topics(bag_path, (topics.POSE, topics.RANGEFINDER)):
        decoded = message.decode()
        stamp_ns = _stamp_ns(decoded.header.stamp)
        if message.topic == topics.RANGEFINDER:
            readings += 1
            estimator.add_range(stamp_ns, decoded.range, decoded.min_range, decoded.max_range)
            continue
        position, orientation = decoded.pose.position, decoded.pose.orientation
        buffer.add_pose(stamp_ns, (position.x, position.y, position.z), (orientation.x, orientation.y, orientation.z, orientation.w))
        height = estimator.height(position.z, stamp_ns, ground_below_home_m, orientation=(orientation.x, orientation.y, orientation.z, orientation.w))
        if height is not None:
            samples.append((stamp_ns, height.value_m, height.source, height.reason, float(position.z)))

    problems = []
    if not samples:
        problems.append("no pose in the recording gave a height: is /mavros/local_position/pose in it?")
    bands = []
    for low, high in BANDS:
        inside = [sample for sample in samples if low <= sample[1] < high]
        if inside:
            from_rangefinder = sum(1 for sample in inside if sample[2] == "rangefinder")
            bands.append({"band": _band_name(low, high), "heights": len(inside), "rangefinder_share": from_rangefinder / len(inside)})
    reasons: dict = {}
    for sample in samples:
        if sample[2] == "baro":
            kind = reason_kind(sample[3])
            reasons[kind] = reasons.get(kind, 0) + 1
    differences = [sample[1] - sample[4] for sample in samples if sample[2] == "rangefinder"]
    agreement = None
    if differences:
        median = _median(differences)
        spread = sorted(abs(value - median) for value in differences)
        agreement = {"heights": len(differences), "median_m": median, "spread_p95_m": spread[min(len(spread) - 1, math.ceil(0.95 * len(spread)) - 1)]}

    tape = None
    if tape_m is not None:
        end_ns = samples[-1][0] if samples else 0
        held = [sample for sample in samples if sample[0] >= end_ns - int(last_s * 1.0e9)]
        from_rangefinder = [sample[1] for sample in held if sample[2] == "rangefinder"]
        tape = {"tape_m": float(tape_m), "last_s": float(last_s), "limit_m": TAPE_LIMIT_M, "heights": len(from_rangefinder), "on_baro": len(held) - len(from_rangefinder), "median_m": None, "passed": False}
        if len(from_rangefinder) < TAPE_MIN_HEIGHTS:
            problems.append(f"no height from the rangefinder to set against the tape: {len(from_rangefinder)} in the last {last_s:g} s (at least {TAPE_MIN_HEIGHTS} needed), {tape['on_baro']} from the barometer")
        else:
            tape["median_m"] = _median(from_rangefinder)
            tape["passed"] = abs(tape["median_m"] - float(tape_m)) <= TAPE_LIMIT_M and tape["on_baro"] <= 0.1 * len(held)
            if abs(tape["median_m"] - float(tape_m)) > TAPE_LIMIT_M:
                problems.append(f"the rangefinder's height is {abs(tape['median_m'] - float(tape_m)):.2f} m from the taped {tape_m:g} m (limit {TAPE_LIMIT_M:g} m)")
            elif not tape["passed"]:
                problems.append(f"{tape['on_baro']} of {len(held)} heights in the taped hover came from the barometer")
    return {
        "poses": len(samples),
        "range_readings": readings,
        "ground_below_home_m": float(ground_below_home_m),
        "bands": bands,
        "baro_reasons": dict(sorted(reasons.items())),
        "rangefinder_minus_home": agreement,
        "tape": tape,
        "problems": problems,
    }


def lines(report: dict) -> List[str]:
    out = [f"{report['poses']} heights from {report['range_readings']} rangefinder readings"]
    for band in report["bands"]:
        out.append(f"  {band['band']}: {100.0 * band['rangefinder_share']:.0f}% from the rangefinder ({band['heights']} heights)")
    if report["baro_reasons"]:
        out.append("  barometer used because: " + ", ".join(f"{kind} {count}" for kind, count in report["baro_reasons"].items()))
    agreement = report["rangefinder_minus_home"]
    if agreement:
        out.append(f"rangefinder height minus height above home: median {agreement['median_m']:+.2f} m, 95% within {agreement['spread_p95_m']:.2f} m of that ({agreement['heights']} heights)")
    tape = report["tape"]
    if tape is not None and tape["median_m"] is not None:
        out.append(f"taped height {tape['tape_m']:g} m: {'PASS' if tape['passed'] else 'FAIL'}, median {tape['median_m']:.2f} m over the last {tape['last_s']:g} s ({tape['heights']} heights, limit {tape['limit_m']:g} m)")
    elif tape is not None:
        out.append(f"taped height {tape['tape_m']:g} m: FAIL")
    out.extend(f"  {problem}" for problem in report["problems"])
    return out


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Run the height estimator over a recorded flight.")
    parser.add_argument("bag", type=Path, help="a recording made by sim/replay/record.launch.py or --record")
    parser.add_argument("--ground-below-home", type=float, default=0.0, metavar="M", help="how far the ground under the flight is known to be below home (default 0)")
    parser.add_argument("--tape", type=float, metavar="H", help="the height the vehicle was held at for the end of the recording, measured with a tape")
    parser.add_argument("--last", type=float, default=TAPE_DEFAULT_LAST_S, metavar="S", help=f"how many seconds at the end of the recording the taped hover lasted (default {TAPE_DEFAULT_LAST_S:g})")
    parser.add_argument("--out", type=Path, help="also write the report as JSON")
    args = parser.parse_args(argv)
    try:
        report = analyse(args.bag, args.ground_below_home, args.tape, args.last)
    except (bag.BagError, OSError) as error:
        print(f"height_log: {error}", file=sys.stderr)
        return 2
    for line in lines(report):
        print(line)
    if args.out:
        args.out.write_text(json.dumps(report, indent=2) + "\n")
    return 1 if report["problems"] else 0


if __name__ == "__main__":
    sys.exit(main())
