#!/usr/bin/env python3
"""Try the drone's red-bullseye detector on a webcam, an image, or a generated target.

No ROS needed: this runs the same RedTargetDetector the drone uses, with the
same live settings (src/drone_target_cv/config/target_cv_live.yaml).

Examples:
  python3 tools/bullseye_try.py --make-target bullseye.png    # make a target to show on your phone
  python3 tools/bullseye_try.py --webcam                       # live; q quits, s saves a screenshot
  python3 tools/bullseye_try.py --image photo.jpg              # writes photo_detected.png
  python3 tools/bullseye_try.py --webcam --set hsv_s_min=140   # change a setting and compare
"""

from __future__ import annotations

import argparse
import dataclasses
import pathlib
import sys
import time

import cv2
import numpy as np
import yaml

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "drone_target_cv"))

from drone_target_cv.target_detector import (  # noqa: E402
    RedTargetDetector,
    RedTargetDetectorConfig,
    draw_debug_overlay,
)

LIVE_YAML = REPO / "src" / "drone_target_cv" / "config" / "target_cv_live.yaml"


def build_config(overrides: list[str]) -> RedTargetDetectorConfig:
    fields = {f.name: f for f in dataclasses.fields(RedTargetDetectorConfig)}
    values = {}
    params = yaml.safe_load(LIVE_YAML.read_text())["target_cv"]["ros__parameters"]
    for key, value in params.items():
        if key in fields:
            values[key] = value
    for item in overrides:
        key, _, raw = item.partition("=")
        if key not in fields:
            sys.exit(f"Unknown setting '{key}'. Valid: {', '.join(sorted(fields))}")
        default = fields[key].default
        if isinstance(default, bool):
            values[key] = raw.lower() in ("1", "true", "yes")
        else:
            values[key] = type(default)(raw)
        print(f"override {key} = {values[key]}")
    return RedTargetDetectorConfig(**values)


def make_target(path: str) -> None:
    frame = np.full((800, 800, 3), 255, dtype=np.uint8)
    center, outer = (400, 400), 360
    red, white = (30, 30, 220), (255, 255, 255)
    for frac, color in [(1.0, red), (0.84, white), (0.70, red), (0.56, white),
                        (0.42, red), (0.28, white), (0.16, red), (0.07, white)]:
        cv2.circle(frame, center, int(outer * frac), color, -1, lineType=cv2.LINE_AA)
    cv2.circle(frame, center, 12, red, -1, lineType=cv2.LINE_AA)
    cv2.imwrite(path, frame)
    print(f"wrote {path}")


def run(detector: RedTargetDetector, frame: np.ndarray) -> tuple[np.ndarray, str]:
    start = time.perf_counter()
    result = detector.detect(frame)
    ms = (time.perf_counter() - start) * 1000.0
    if result.accepted:
        h, w = frame.shape[:2]
        x = (result.center[0] - w / 2) / (w / 2)
        y = (result.center[1] - h / 2) / (h / 2)
        line = f"FOUND x={x:+.2f} y={y:+.2f} conf={result.confidence:.2f} {ms:.1f} ms"
    else:
        line = f"none ({result.reject_reason}) {ms:.1f} ms"
    annotated = draw_debug_overlay(frame, result)
    cv2.putText(annotated, line, (10, annotated.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return annotated, line


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--webcam", nargs="?", const=0, type=int, metavar="INDEX")
    src.add_argument("--image")
    src.add_argument("--make-target", metavar="OUT.png")
    parser.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                        help="override a detector setting; repeatable")
    args = parser.parse_args()

    if args.make_target:
        make_target(args.make_target)
        return

    detector = RedTargetDetector(build_config(args.set))

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            sys.exit(f"Could not read {args.image}")
        annotated, line = run(detector, frame)
        out = str(pathlib.Path(args.image).with_name(pathlib.Path(args.image).stem + "_detected.png"))
        cv2.imwrite(out, annotated)
        print(line)
        print(f"wrote {out}")
        return

    cap = cv2.VideoCapture(args.webcam)
    if not cap.isOpened():
        sys.exit(f"Could not open webcam {args.webcam}")
    shots = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        annotated, line = run(detector, frame)
        print(line, end="\r", flush=True)
        cv2.imshow("bullseye_try (q quit, s save)", annotated)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            shots += 1
            name = f"bullseye_shot_{shots}.png"
            cv2.imwrite(name, annotated)
            print(f"\nsaved {name}")
    cap.release()
    cv2.destroyAllWindows()
    print()


if __name__ == "__main__":
    main()
