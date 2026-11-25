#!/usr/bin/env python3
"""Fit the release offset model's K from drops (CEN-08): how far a package drifts, against the vehicle's lean and the release height.

  python3 tools/release_offset_fit.py --runs sim/.state/suites/drop-gated_wind-*/runs/*       # sim flights flown with release_offset_model on
  python3 tools/release_offset_fit.py --drops field/<day>/drops.csv                           # drops measured on the field

A sim run gives a drop when its mission logged the lean at the release (the
flag on; K may be 0, which shifts nothing and only logs) and its trace has
the package's place at the release and at rest: the drift is the second
less the first. A field drop is a line of a CSV with the header
`name,height_m,lean_east,lean_north,drift_east_m,drift_north_m`: the lean
from the flight's log at the release, the drift measured on the ground from
the point under the release to where the package lies (east, north).

Prints every drop, the K that fits them (drift = K x height x lean,
downwind), what that K leaves of the drifts against what no model leaves,
and the line for the drop mission's config. Exit code 0 with a fit, 1 when
the drops cannot be fitted (too few, or made in still air), 2 when a file
cannot be read.

No ROS imports; the standard library only.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from pathlib import Path
from typing import List, Optional

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "drone_mission_core"))

from drone_mission_core import release_offset as ro  # noqa: E402

LOG_LINE = re.compile(r"release offset: height (-?[\d.]+) m, lean \((-?[\d.]+), (-?[\d.]+)\)")
RELEASE_LINE = "Closing sprayer for payload drop"


def drop_from_run(run: Path) -> ro.Drop:
    """The drop of one sim run's folder. Raises ValueError saying what the run lacks."""
    run = Path(run)
    try:
        log = (run / "launch.log").read_text(errors="replace")
        trace = [json.loads(line) for line in (run / "trace.jsonl").read_text().splitlines() if line.strip()]
    except (OSError, ValueError) as problem:
        raise ValueError(f"{run}: not a run's folder ({problem})") from problem
    before_release = log.split(RELEASE_LINE)[0] if RELEASE_LINE in log else None
    if before_release is None:
        raise ValueError(f"{run.name}: nothing was released")
    said = LOG_LINE.findall(before_release)
    if not said:
        raise ValueError(f"{run.name}: the mission did not log its lean at the release (was release_offset_model on?)")
    height, lean_east, lean_north = (float(value) for value in said[-1])
    release = next((event for event in trace if event.get("event") == "release" and not event.get("fake")), None)
    if release is None:
        raise ValueError(f"{run.name}: the trace has no release")
    names = {event["pose"] for event in trace if "pose" in event and event["pose"] != "vehicle"}
    payload = None
    try:
        payload = json.loads((run / "scenario.json").read_text()).get("payload")
    except (OSError, ValueError):
        pass
    if payload not in names:
        if len(names) != 1:
            raise ValueError(f"{run.name}: the trace does not say which of {sorted(names)} was dropped")
        payload = next(iter(names))
    places = [event for event in trace if event.get("pose") == payload]
    held = [event for event in places if event["t"] <= release["t"]]
    if not held or not places or places[-1]["t"] <= release["t"]:
        raise ValueError(f"{run.name}: the trace has no place for {payload} at the release and after it")
    at_release, at_rest = held[-1], places[-1]
    return ro.Drop(height, (lean_east, lean_north), (at_rest["e"] - at_release["e"], at_rest["n"] - at_release["n"]), run.name)


def drops_from_csv(path: Path) -> List[ro.Drop]:
    with open(path, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    needed = ("name", "height_m", "lean_east", "lean_north", "drift_east_m", "drift_north_m")
    if not rows or any(key not in rows[0] for key in needed):
        raise ValueError(f"{path}: expected the header {','.join(needed)}")
    try:
        return [ro.Drop(float(row["height_m"]), (float(row["lean_east"]), float(row["lean_north"])), (float(row["drift_east_m"]), float(row["drift_north_m"])), str(row["name"])) for row in rows]
    except (TypeError, ValueError) as problem:
        raise ValueError(f"{path}: a row is not numbers ({problem})") from problem


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Fit the release offset model's K from drops.")
    parser.add_argument("--runs", nargs="+", default=[], help="sim run folders flown with release_offset_model on")
    parser.add_argument("--drops", help="a CSV of drops measured on the field")
    args = parser.parse_args(argv)
    drops: List[ro.Drop] = []
    try:
        if args.drops:
            drops += drops_from_csv(Path(args.drops))
    except (OSError, ValueError) as problem:
        print(f"release_offset_fit: {problem}", file=sys.stderr)
        return 2
    for run in args.runs:
        try:
            drops.append(drop_from_run(Path(run)))
        except ValueError as problem:
            print(f"not used: {problem}")
    if not drops:
        print("release_offset_fit: no drops (give --runs or --drops)", file=sys.stderr)
        return 2
    print("| Drop | Height (m) | Lean (east, north) | Drift (east, north) m | Drift (m) |")
    print("| --- | --- | --- | --- | --- |")
    for drop in drops:
        print(f"| {drop.name} | {drop.height_m:.2f} | ({drop.lean[0]:+.3f}, {drop.lean[1]:+.3f}) | ({drop.drift_m[0]:+.2f}, {drop.drift_m[1]:+.2f}) | {math.hypot(*drop.drift_m):.2f} |")
    try:
        found = ro.fit(drops)
    except ValueError as problem:
        print(f"\nno fit: {problem}")
        return 1
    print(f"\nK = {found.k:.2f} from {found.drops} drop(s) (the largest lean {found.lean_max:.3f}). The drifts' RMS is {found.unmodelled_rms_m:.2f} m; with this K the model leaves {found.residual_rms_m:.2f} m.")
    if found.residual_rms_m >= found.unmodelled_rms_m:
        print("The model explains nothing of these drops: the drift is not along the lean. Leave release_offset_k at 0.")
        return 1
    print(f"For the drop mission's config: release_offset_k: {found.k:.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
