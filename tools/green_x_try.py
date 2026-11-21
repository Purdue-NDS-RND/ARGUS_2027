#!/usr/bin/env python3
"""The green X and bucket detector (REC-1) on synthetic renders: the spec's pass lines. No ROS.

  python3 tools/green_x_try.py --n 300 --negatives 200 --seed 1

Positives: the X mat as the downward camera (1280 x 720, focal length 778 px)
sees it from 3 to 30 m, at any rotation, off centre, tilted up to 8 degrees
each way, blurred, noisy, darker and brighter, on a noisy green field, with a
bucket on every second one. A hit is the X found with its centre within
max(2 px, 1% of the X's span) and its angle within 3 degrees.

Negatives: bullseyes, numbered markers (last year's and this year's) and
bare green-noise fields. Any X reported on one is a false alarm.

The spec's lines: recall at least 98%, no false alarm. Exit code 0 when both
are met, 1 otherwise.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import List, Optional

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
for package in ("drone_target_cv", "drone_mission_core"):
    sys.path.insert(0, str(REPO / "src" / package))

FOCAL_PX = 778.0
SIZE = (1280, 720)
ARM_SPAN_M = 2.0 * (2.5 * math.sqrt(2.0) - 0.25)
MIN_RECALL = 0.98
MAX_ANGLE_DEG = 3.0


def centre_limit_px(height_m: float) -> float:
    return max(2.0, 0.01 * ARM_SPAN_M * FOCAL_PX / height_m)


def angle_error_deg(found_rad: float, rotation_deg: float) -> float:
    """The arms are on the mat's diagonals, 45 degrees from the picture's axes at zero rotation; the period is 90."""
    true = (45.0 - rotation_deg) % 90.0
    return abs((math.degrees(found_rad) - true + 45.0) % 90.0 - 45.0)


def evaluate(count: int = 300, negatives: int = 200, seed: int = 1) -> dict:
    import numpy as np
    import synth_targets

    from drone_target_cv import green_x

    rng = np.random.default_rng(seed)
    hits = found_count = buckets = buckets_found = false_buckets = 0
    worst_centre = worst_angle = 0.0
    misses: List[dict] = []
    took_ms = []
    for index in range(count):
        height_m = float(rng.uniform(3.0, 30.0))
        rotation = float(rng.uniform(0.0, 360.0))
        reach = 0.25 * SIZE[1]
        bucket = bool(index % 2)
        options = synth_targets.RenderOptions(
            width=SIZE[0], height=SIZE[1], mpp=height_m / FOCAL_PX, rotation_deg=rotation,
            offset_px=(float(rng.uniform(-reach, reach)), float(rng.uniform(-reach, reach))),
            tilt_deg=(float(rng.uniform(-8.0, 8.0)), float(rng.uniform(-8.0, 8.0))),
            blur_sigma_px=float(rng.uniform(0.0, 1.0)), noise_sigma=float(rng.uniform(0.0, 4.0)), brightness=float(rng.uniform(0.8, 1.15)),
            background="noise", supersample=2,
        )  # fmt: skip
        image, truth = synth_targets.render(synth_targets.XMat(shape="circle" if index % 5 == 0 else "square", bucket=bucket), options, seed=seed * 100003 + index)
        started = time.perf_counter()
        result = green_x.detect(image)
        took_ms.append((time.perf_counter() - started) * 1000.0)
        buckets += bucket
        buckets_found += bucket and result.bucket is not None
        false_buckets += (not bucket) and result.bucket is not None
        if result.x is None:
            misses.append({"index": index, "height_m": round(height_m, 1), "why": "not found"})
            continue
        found_count += 1
        centre = math.hypot(result.x.cx - truth["center_px"][0], result.x.cy - truth["center_px"][1])
        angle = angle_error_deg(result.x.angle_rad, rotation)
        if centre <= centre_limit_px(height_m) and angle <= MAX_ANGLE_DEG:
            hits += 1
            worst_centre, worst_angle = max(worst_centre, centre / centre_limit_px(height_m)), max(worst_angle, angle)
        else:
            misses.append({"index": index, "height_m": round(height_m, 1), "why": f"centre {centre:.1f} px (limit {centre_limit_px(height_m):.1f}), angle {angle:.1f} deg"})
    false_alarms: List[str] = []
    kinds = ("bullseye", "marker", "marker27", "green noise")
    for index in range(negatives):
        kind = kinds[index % len(kinds)]
        options = synth_targets.RenderOptions(
            width=SIZE[0], height=SIZE[1], mpp=float(rng.uniform(3.0, 30.0)) / FOCAL_PX, rotation_deg=float(rng.uniform(0.0, 360.0)),
            blur_sigma_px=float(rng.uniform(0.0, 1.0)), noise_sigma=float(rng.uniform(0.0, 4.0)), brightness=float(rng.uniform(0.8, 1.15)),
            background="noise", background_color=(int(rng.integers(40, 110)), int(rng.integers(110, 190)), int(rng.integers(40, 110))), supersample=2,
        )  # fmt: skip
        if kind == "green noise":
            image = synth_targets._background(options, np.random.default_rng(seed * 7919 + index))
            image = np.clip(image, 0, 255).astype(np.uint8)
        else:
            target = {"bullseye": synth_targets.Bullseye(), "marker": synth_targets.NumberedMarker(digit=index % 10), "marker27": synth_targets.SurveyMarker(digit=index % 10)}[kind]
            if kind != "bullseye":
                options = synth_targets.RenderOptions(**{**options.__dict__, "mpp": float(rng.uniform(0.002, 0.02))})
            image, _truth = synth_targets.render(target, options, seed=seed * 7919 + index)
        if green_x.detect(image).x is not None:
            false_alarms.append(f"{kind} #{index}")
    return {
        "count": count,
        "found": found_count,
        "hits": hits,
        "recall": hits / count if count else 0.0,
        "worst_centre_share_of_limit": round(worst_centre, 3),
        "worst_angle_deg": round(worst_angle, 2),
        "misses": misses,
        "buckets": buckets,
        "buckets_found": buckets_found,
        "false_buckets": false_buckets,  # a bucket reported on a mat that had none
        "negatives": negatives,
        "false_alarms": false_alarms,
        "ms_a_frame": {"median": round(float(np.median(took_ms)), 2), "p95": round(float(np.percentile(took_ms, 95)), 2)} if took_ms else None,
        "seed": seed,
    }


def verdict(report: dict) -> bool:
    """The spec's two lines. (A bucket reported where there is none is counted and shown; the spec has no line for it.)"""
    return report["recall"] >= MIN_RECALL and not report["false_alarms"]


def summary(report: dict) -> List[str]:
    lines = [
        f"{'PASS' if verdict(report) else 'FAIL'}: recall {100.0 * report['recall']:.1f}% ({report['hits']} of {report['count']} within the centre and angle lines; {report['found']} found at all), "
        f"{len(report['false_alarms'])} false alarm(s) on {report['negatives']} bullseye, marker and green-noise renders; lines {100 * MIN_RECALL:.0f}% and none",
        f"worst hit: centre at {100.0 * report['worst_centre_share_of_limit']:.0f}% of its limit, angle {report['worst_angle_deg']:g} deg (limit {MAX_ANGLE_DEG:g}); seed {report['seed']}",
        f"bucket: found on {report['buckets_found']} of the {report['buckets']} mats that had one, and on {report['false_buckets']} of the {report['count'] - report['buckets']} that had none",
    ]
    if report["misses"]:
        lines.append("misses: " + "; ".join(f"#{miss['index']} at {miss['height_m']} m: {miss['why']}" for miss in report["misses"][:8]))
    if report["false_alarms"]:
        lines.append("false alarms: " + ", ".join(report["false_alarms"][:8]))
    if report["ms_a_frame"]:
        lines.append(f"time a frame: median {report['ms_a_frame']['median']:g} ms, 95th percentile {report['ms_a_frame']['p95']:g} ms (this machine, 1280 x 720)")
    return lines


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Recall and false alarms of the green X detector on synthetic renders.")
    parser.add_argument("--n", type=int, default=300, help="X mats to render")
    parser.add_argument("--negatives", type=int, default=200, help="bullseyes, markers and green-noise fields to render")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--report", type=Path, help="also write the numbers here as JSON")
    args = parser.parse_args(argv)
    report = evaluate(args.n, args.negatives, args.seed)
    for line in summary(report):
        print(line)
    if args.report is not None:
        args.report.write_text(json.dumps(report, indent=2) + "\n")
    return 0 if verdict(report) else 1


if __name__ == "__main__":
    sys.exit(main())
