#!/usr/bin/env python3
"""Take off, hold a hover, come home (SIM-03's pass flight). Runs inside the sim container.

  env -u PYTHONNOUSERSITE python3 sim/hover.py --site sim/sites/apollo_xi.yaml --height 10 --hold 40

A scenario's driver: it flies the vehicle itself over MAVLink, in place of
a mission. It arms, takes off to `--height`, holds there in GUIDED for
`--hold` seconds while the wind tool (sim/wind.py) blows and measures the
tilt, then returns and lands. Sim only: it refuses an autopilot that is not
SITL, as sim/fence_check.py does.

`run_hover` takes any vehicle with is_simulator, takeoff, step, now,
return_home and is_armed, so the flight's logic is tested without MAVLink.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import List, Optional

SIM_DIR = Path(__file__).resolve().parent
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))

import site_file  # noqa: E402

LAND_TIMEOUT_S = 240.0


def run_hover(vehicle, height_m: float = 10.0, hold_s: float = 40.0, land_timeout_s: float = LAND_TIMEOUT_S) -> dict:
    """The flight. Returns what happened: how long it held, how far it drifted, whether it came down."""
    report = {"height_m": height_m, "hold_s": hold_s, "held_s": 0.0, "max_drift_m": None, "landed": False, "passed": False, "problem": ""}
    if not vehicle.is_simulator():
        report["problem"] = "this autopilot is not SITL: hover arms and takes off, so it runs in the sim only"
        return report
    vehicle.takeoff(height_m)
    start = vehicle.step()
    t0 = vehicle.now()
    drift = 0.0
    while vehicle.now() - t0 < hold_s:
        sample = vehicle.step()
        drift = max(drift, math.hypot(sample.e - start.e, sample.n - start.n))
        if not vehicle.is_armed():
            report["problem"] = "the vehicle disarmed during the hold"
            break
    report["held_s"] = round(vehicle.now() - t0, 2)
    report["max_drift_m"] = round(drift, 3)
    vehicle.return_home()
    deadline = vehicle.now() + land_timeout_s
    while vehicle.is_armed() and vehicle.now() < deadline:
        vehicle.step()
    report["landed"] = not vehicle.is_armed()
    if not report["problem"] and not report["landed"]:
        report["problem"] = f"the vehicle had not landed and disarmed {land_timeout_s:g} s after the hold"
    report["passed"] = not report["problem"]
    return report


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Take off, hold a hover and land, in the sim.")
    parser.add_argument("--site", type=Path, required=True)
    parser.add_argument("--url", default="udpin:127.0.0.1:14552", help="pymavlink connection (the sim's tools port)")
    parser.add_argument("--height", type=float, default=10.0)
    parser.add_argument("--hold", type=float, default=40.0, help="seconds to hold the hover")
    parser.add_argument("--out", type=Path, help="write the report as JSON")
    args = parser.parse_args(argv)
    try:
        site = site_file.load_site(args.site)
    except site_file.SiteError as error:
        print(f"hover: {error}", file=sys.stderr)
        return 2
    try:
        from fence_check import MavlinkVehicle

        report = run_hover(MavlinkVehicle(args.url, site), height_m=args.height, hold_s=args.hold)
    except ImportError:
        print("hover: pymavlink is not installed here (in the sim container, run with PYTHONNOUSERSITE unset)", file=sys.stderr)
        return 2
    except RuntimeError as error:
        print(f"hover: {error}", file=sys.stderr)
        return 2
    if args.out:
        args.out.write_text(json.dumps(report, indent=2) + "\n")
    for key, value in report.items():
        print(f"{key}: {value}")
    print("RESULT: PASS" if report["passed"] else f"RESULT: FAIL ({report['problem']})")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
