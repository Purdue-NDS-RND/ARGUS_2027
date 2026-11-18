"""Bench check for a camera in one sensor mode: does it give the mode's frames at the mode's rate? No ROS imports.

  ros2 run argus_camera camera_bench --mode imx519_720p60 --seconds 10
  python3 -m argus_camera.bench --mode imx519_4k --seconds 10 [--synthetic]

Run on the Jetson with the camera fitted. It opens the mode through the same
FrameSource the node uses, reads for the given time and reports the frame
size, the rate, the longest gap between frames and whether the mode has a
measured calibration. It passes when the rate is at least 90% of the mode's,
no gap is longer than three frame periods, and the calibration is measured.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Callable, List, Optional

from .calibration import CalibrationError, for_mode
from .frame_source import FrameSource, open_source
from .modes import SensorMode, get_mode, load_modes

MIN_RATE_FRACTION = 0.9
MAX_GAP_PERIODS = 3.0


def measure(source: FrameSource, seconds: float, sleep: Callable[[float], None] = time.sleep) -> List[int]:
    """Capture times, in nanoseconds, of the frames read from an open source over `seconds` of its own clock."""
    stamps: List[int] = []
    misses = 0
    while True:
        frame = source.read()
        if frame is None:
            misses += 1
            if misses > 2000:
                break  # the camera has stopped giving frames
            sleep(0.001)
            continue
        misses = 0
        stamps.append(frame.capture_ns)
        if frame.capture_ns - stamps[0] >= seconds * 1.0e9:
            break
    return stamps


def judge(mode: SensorMode, stamps: List[int], calibration_measured: Optional[bool]) -> dict:
    """Rate, longest gap and verdict from capture times. calibration_measured: True, False, or None for no file."""
    problems = []
    rate = longest = None
    if len(stamps) < 2:
        problems.append(f"only {len(stamps)} frame(s) came from the camera")
    else:
        rate = (len(stamps) - 1) / ((stamps[-1] - stamps[0]) * 1.0e-9)
        longest = max(later - earlier for earlier, later in zip(stamps, stamps[1:])) * 1.0e-9
        if rate < MIN_RATE_FRACTION * mode.fps:
            problems.append(f"{rate:.1f} fps is under {MIN_RATE_FRACTION:.0%} of the mode's {mode.fps:g} fps")
        if longest > MAX_GAP_PERIODS / mode.fps:
            problems.append(f"a gap of {longest * 1000.0:.0f} ms between frames (more than {MAX_GAP_PERIODS:g} frame periods)")
    if calibration_measured is None:
        problems.append("the mode has no calibration file")
    elif not calibration_measured:
        problems.append("the mode's calibration is not measured")
    return {"mode": mode.name, "frames": len(stamps), "fps": rate, "longest_gap_s": longest, "passed": not problems, "problems": problems}


def default_config_dir(module_file=__file__) -> Path:
    """Where modes.yaml is: beside the package in the source tree, or under share/argus_camera in an installed one."""
    here = Path(module_file).resolve()
    source = here.parents[1] / "config"
    if (source / "modes.yaml").is_file():
        return source
    for parent in here.parents:  # <prefix>/lib/python3.x/site-packages/argus_camera/bench.py -> <prefix>/share/argus_camera/config
        installed = parent / "share" / "argus_camera" / "config"
        if (installed / "modes.yaml").is_file():
            return installed
    return source


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Bench check for a camera in one sensor mode.")
    parser.add_argument("--mode", required=True)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--synthetic", action="store_true", help="use the test pattern instead of a camera (tries the tool itself)")
    parser.add_argument("--config", type=Path, default=default_config_dir(), help="folder with modes.yaml (default: the package's own)")
    args = parser.parse_args(argv)
    mode = get_mode(load_modes(args.config / "modes.yaml"), args.mode)
    try:
        measured: Optional[bool] = for_mode(mode, args.config).measured
    except CalibrationError:
        measured = None
    source = open_source(mode, clock=time.time_ns, synthetic=args.synthetic)  # wall clock: only intervals are used
    with source:
        if args.synthetic:
            stamps = []
            end = time.monotonic() + args.seconds
            while time.monotonic() < end:  # the test pattern does not pace itself
                stamps.append(source.read().capture_ns)
                time.sleep(1.0 / mode.fps)
        else:
            stamps = measure(source, args.seconds)
    report = judge(mode, stamps, measured)
    fps = "n/a" if report["fps"] is None else f"{report['fps']:.1f}"
    gap = "n/a" if report["longest_gap_s"] is None else f"{report['longest_gap_s'] * 1000.0:.0f} ms"
    print(f"{mode.name}: {mode.width} x {mode.height}, {report['frames']} frames, {fps} fps (mode {mode.fps:g}), longest gap {gap}")
    for problem in report["problems"]:
        print(f"  {problem}")
    print("camera bench: PASS" if report["passed"] else "camera bench: FAIL")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
