#!/usr/bin/env python3
"""Bench check for the survey's frame logging (LOC-11): can this machine log every frame at the survey's rate? No ROS.

  python3 tools/frame_log_bench.py --out /mnt/nvme/bench_frames                  # 4K at 17 Hz (the 4K mode's rate) for 20 s, 3 workers
  python3 tools/frame_log_bench.py --out /tmp/f --width 1280 --height 720 --rate 10

It feeds test frames of the survey's size to the same policy the localisation
node uses (argus_localisation/frame_logging.py over argus_camera's FrameLog),
at the camera's rate, and reports how many were logged, the megabytes a
second written, the size of a frame, and how many CPU cores the encoding
took. The Orin Nano has no hardware JPEG encoder, so the last number is what
the detector has to share. Run it on the flight computer, writing to the
disk the flight will write to.

The frames are textured (noise over a gradient, changing every frame): a
flat frame compresses to nothing and says little about the disk. A real
survey frame of grass compresses differently again, so the megabytes are a
guide; the spec's figure is about 42 MB a second for 4K at 15 Hz, and the
4K sensor mode gives 17 frames a second (argus_camera's modes.yaml), which
is what "every frame" is and the tool's default.

It also says which JPEG library this machine's OpenCV was built with: the
spec counts on libjpeg-turbo.

Exit code 0 when every frame offered was logged, 1 when any was dropped or
thinned. The frames are left in --out for looking at; it must be a folder
the tool may fill.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

REPO = Path(__file__).resolve().parents[1]
for package in ("argus_localisation", "argus_camera", "drone_mission_core"):
    sys.path.insert(0, str(REPO / "src" / package))

S = 1_000_000_000


def test_frame(width: int, height: int, seq: int, textured: bool = True):
    """A frame of the given size: a gradient with noise over it that changes with `seq`, or a flat grey one."""
    import numpy as np

    if not textured:
        return np.full((height, width, 3), 110, dtype=np.uint8)
    rng = np.random.default_rng(seq)
    ramp = np.linspace(60, 160, width, dtype=np.float32)[None, :, None]
    noise = rng.integers(-35, 36, size=(height, width, 3), dtype=np.int16)
    return np.clip(ramp + noise, 0, 255).astype(np.uint8)


def jpeg_library() -> str:
    """The JPEG line of OpenCV's build information, e.g. "build-libjpeg-turbo (ver 2.1.3-62)"."""
    import cv2

    for line in cv2.getBuildInformation().splitlines():
        if line.strip().startswith("JPEG:"):
            return line.split(":", 1)[1].strip()
    return "not stated by this OpenCV build"


def run(
    out, width: int = 3840, height: int = 2160, rate_hz: float = 17.0, seconds: float = 20.0, workers: int = 3, quality: int = 90, min_free_gb: float = 20.0
) -> dict:
    """Log test frames at the rate for the time, and say what happened."""
    from argus_camera.frame_source import Frame
    from argus_localisation.frame_logging import SurveyFrameLog

    count = int(round(rate_hz * seconds))
    frames = [test_frame(width, height, seq) for seq in range(min(count, 8))]  # made beforehand: making them is not what is timed
    Path(out).mkdir(parents=True, exist_ok=True)  # the bench makes its folder; the flight's must be there already
    log = SurveyFrameLog(out, jpeg_quality=quality, workers=workers, expected_rate_hz=rate_hz, min_free_bytes=min_free_gb * 1.0e9)
    started_wall, started_cpu = time.monotonic(), time.process_time()
    folder = log.start(int(time.time() * S))
    if folder is None:
        raise RuntimeError(f"the frame log could not start: {log.status()['error']}")
    for seq in range(count):
        due = started_wall + seq / rate_hz
        while time.monotonic() < due:
            time.sleep(0.0005)
        log.offer(Frame(image=frames[seq % len(frames)], seq=seq, capture_ns=int(seq * S / rate_hz), mode="bench", stamp_source="bench"))
        if seq and seq % max(1, int(round(rate_hz))) == 0:
            log.review({"dropped": 0, "below_minimum_rate": False, "rate_hz": rate_hz}, int(time.time() * S))  # a detector with nothing wrong
    keep_every = log.keep_every
    log.stop(wait_s=60.0)
    wall_s, cpu_s = time.monotonic() - started_wall, time.process_time() - started_cpu
    status = log.status()
    megabytes = sum(path.stat().st_size for path in Path(folder).glob("frame_*.jpg")) / 1.0e6
    written = int(status["written"])
    return {
        "folder": str(folder),
        "width": width,
        "height": height,
        "rate_hz": rate_hz,
        "workers": workers,
        "quality": quality,
        "offered": count,
        "written": written,
        "dropped": int(status["dropped"]),
        "thinned": int(status["thinned"]),
        "keep_every_at_end": keep_every,
        "every_frame_logged": written == count,
        "wall_s": wall_s,
        "megabytes": megabytes,
        "megabytes_per_second": megabytes / wall_s if wall_s > 0.0 else 0.0,
        "frame_megabytes": megabytes / written if written else 0.0,
        "cpu_cores_used": cpu_s / wall_s if wall_s > 0.0 else 0.0,
        "jpeg_library": jpeg_library(),
    }


def verdict(report: dict) -> Tuple[bool, str]:
    lost = report["offered"] - report["written"]
    if lost == 0:
        return True, "every frame logged"
    return False, f"{lost} of {report['offered']} frames were not logged ({report['dropped']} dropped at the log's queue, {report['thinned']} thinned)"


def summary(report: dict) -> List[str]:
    passed, text = verdict(report)
    return [
        f"{'PASS' if passed else 'FAIL'}: {text}",
        f"{report['written']} of {report['offered']} frames of {report['width']} x {report['height']} at {report['rate_hz']:g} a second, {report['workers']} worker(s), JPEG quality {report['quality']}",
        f"{report['megabytes_per_second']:.1f} MB a second written ({report['frame_megabytes']:.2f} MB a frame); {report['cpu_cores_used']:.2f} CPU cores used over {report['wall_s']:.1f} s",
        f"JPEG library of this OpenCV: {report['jpeg_library']}",
        f"frames in {report['folder']}",
    ]


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Bench check for the survey's frame logging.")
    parser.add_argument("--out", required=True, type=Path, help="folder to log into (on the disk the flight writes to)")
    parser.add_argument("--width", type=int, default=3840)
    parser.add_argument("--height", type=int, default=2160)
    parser.add_argument("--rate", type=float, default=17.0, help="frames a second (17: the 4K sensor mode's)")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--workers", type=int, default=3)
    parser.add_argument("--quality", type=int, default=90)
    parser.add_argument("--min-free-gb", type=float, default=20.0, help="the logging does not start with less than this free on the disk, as in flight")
    parser.add_argument("--report", type=Path, help="also write the numbers here as JSON")
    args = parser.parse_args(argv)
    report = run(args.out, args.width, args.height, args.rate, args.seconds, args.workers, args.quality, args.min_free_gb)
    for line in summary(report):
        print(line)
    if args.report is not None:
        args.report.write_text(json.dumps(report, indent=2) + "\n")
    return 0 if report["every_frame_logged"] else 1


if __name__ == "__main__":
    sys.exit(main())
