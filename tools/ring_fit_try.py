#!/usr/bin/env python3
"""Try the near-field ring fit (CEN-13) on a photo or a saved frame of the bullseye. No ROS needed.

  python3 tools/ring_fit_try.py --image frame.png
  python3 tools/ring_fit_try.py --image frame.png --expect 412,236          # compare with a centre you measured
  python3 tools/ring_fit_try.py --image frame.png --tilt 0,-8 --focal-px 389  # a tilted camera, levelled first
  python3 tools/ring_fit_try.py --image frame.png --set hsv_s_min=110        # another colour threshold
  python3 tools/ring_fit_try.py --image frame.png --time 50                  # how long one fit takes on this machine

This is the same fit the ring_fit node runs (src/drone_target_cv/drone_target_cv/ring_fit.py).
It prints the centre, the rings and how well they fit, and the 2026 box
detector's centre beside it, and writes <name>_ring_fit.png with both
marked. For the bench: print the target, hold the camera 1.5 to 4 m from it,
mark the target's centre in the frame (--expect), and the fit should be
within 3 px of it.

--tilt is the tilt of the target's plane about the picture's x and y axes in
degrees, as tools/synth_targets.py has it; with --focal-px (the camera's
focal length in pixels; the principal point is taken as the picture's
middle) the picture is levelled before the fit, as the node does with the
vehicle's attitude.

Exit code 0 when rings were found (and, with --expect, within 3 px), 1 when
not, 2 when the picture cannot be read.
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import pathlib
import sys

import cv2
import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "drone_target_cv"))
sys.path.insert(0, str(REPO / "src" / "drone_mission_core"))

from drone_target_cv import ring_fit  # noqa: E402
from drone_target_cv.target_detector import RedTargetDetector  # noqa: E402

EXPECT_LIMIT_PX = 3.0


def config_from(overrides: list) -> ring_fit.RingFitConfig:
    fields = {field.name: field for field in dataclasses.fields(ring_fit.RingFitConfig)}
    values = {}
    for item in overrides:
        name, _, raw = item.partition("=")
        if name not in fields:
            raise SystemExit(f"--set {item}: no setting '{name}' (settings: {', '.join(sorted(fields))})")
        values[name] = type(fields[name].default)(float(raw))
    return ring_fit.RingFitConfig(**values)


def levelling(tilt_deg, focal_px: float, size) -> np.ndarray:
    """The homography that levels a picture of a plane tilted by `tilt_deg` about the picture's x and y axes."""
    tx, ty = math.radians(tilt_deg[0]), math.radians(tilt_deg[1])
    rx = np.array([[1, 0, 0], [0, math.cos(tx), -math.sin(tx)], [0, math.sin(tx), math.cos(tx)]])
    ry = np.array([[math.cos(ty), 0, math.sin(ty)], [0, 1, 0], [-math.sin(ty), 0, math.cos(ty)]])
    optical_from_ground = np.diag([1.0, -1.0, -1.0]) @ (ry @ rx)
    width, height = size
    k = np.array([[focal_px, 0, (width - 1) / 2], [0, focal_px, (height - 1) / 2], [0, 0, 1]])
    return ring_fit.nadir_homography(k, optical_from_ground.T)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Try the near-field ring fit on a picture of the bullseye.")
    parser.add_argument("--image", type=pathlib.Path, required=True)
    parser.add_argument("--expect", help="x,y of the target's centre in the picture, to compare with")
    parser.add_argument("--tilt", help="tilt of the target's plane about the picture's x and y axes, degrees: a,b")
    parser.add_argument("--focal-px", type=float, help="the camera's focal length in pixels (needed with --tilt)")
    parser.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", help="a RingFitConfig setting")
    parser.add_argument("--time", type=int, metavar="N", help="also fit the picture N times and print the time one fit takes")
    args = parser.parse_args(argv)
    image = cv2.imread(str(args.image))
    if image is None:
        print(f"ring_fit_try: cannot read {args.image}", file=sys.stderr)
        return 2
    level = None
    if args.tilt:
        if not args.focal_px:
            parser.error("--tilt needs --focal-px")
        level = levelling([float(value) for value in args.tilt.split(",")], args.focal_px, (image.shape[1], image.shape[0]))
    fit = ring_fit.fit_rings(image, config_from(args.set), level)
    marked = image.copy()
    box = RedTargetDetector().detect(image)
    if box.center is not None:
        cv2.drawMarker(marked, (int(box.center[0]), int(box.center[1])), (255, 160, 0), cv2.MARKER_TILTED_CROSS, 18, 2)
        print(f"2026 detector: centre at ({box.center[0]}, {box.center[1]}) px, confidence {box.confidence:.2f}")
    else:
        print("2026 detector: no target")
    code = 0
    if fit is None:
        print("ring fit: no rings found (fewer than two rings about a point inside the picture, or too far out of round)")
        code = 1
    else:
        how = " (levelled)" if level is not None else ""
        print(f"ring fit{how}: centre at ({fit.center[0]:.1f}, {fit.center[1]:.1f}) px, {fit.rings} rings, rms {fit.rms_px:.2f} px, {100.0 * fit.inlier_fraction:.0f}% of {fit.points} edge points on them")
        centre = (int(round(fit.center[0])), int(round(fit.center[1])))
        cv2.drawMarker(marked, centre, (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
        if level is None:
            for radius in fit.radii_px:
                cv2.circle(marked, centre, int(round(radius)), (0, 255, 0), 1)
        if args.expect:
            x, y = (float(value) for value in args.expect.split(","))
            off = math.hypot(fit.center[0] - x, fit.center[1] - y)
            passed = off <= EXPECT_LIMIT_PX
            print(f"{off:.1f} px from the expected centre: {'PASS' if passed else 'FAIL'} (limit {EXPECT_LIMIT_PX:g} px)")
            code = 0 if passed else 1
    if args.time:
        import time

        config = config_from(args.set)
        started = time.perf_counter()
        for _ in range(max(1, args.time)):
            ring_fit.fit_rings(image, config, level)
        each_ms = 1000.0 * (time.perf_counter() - started) / max(1, args.time)
        print(f"one fit of this {image.shape[1]} x {image.shape[0]} picture takes {each_ms:.1f} ms ({1000.0 / each_ms:.0f} a second)")
    out = args.image.with_name(args.image.stem + "_ring_fit.png")
    cv2.imwrite(str(out), marked)
    print(f"wrote {out}")
    return code


if __name__ == "__main__":
    sys.exit(main())
