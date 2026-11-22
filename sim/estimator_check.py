#!/usr/bin/env python3
"""Check the target estimator against the truth in a sim flight (FND-06). Runs inside the sim container.

  sim/run_sim.sh --exec python3 sim/estimator_check.py <run folder>

Started by sim/runner/fly.py beside every flight whose scenario turns the
mission flag use_target_estimator on, and interrupted when the flight ends.
It listens to what a ground station would: the estimator's tracks on
/estimator/targets and MAVROS's pose for the vehicle's height. Each time the
tracks are published it takes the bullseye track a mission would act on (a
CONFIRMED one, the oldest if there are several) and compares its position
with where the scenario put the target (truth.json), in the frame of
MAVROS's local position, whose origin is the take-off point.

Pass lines (FEATURES.md, FND-06, calm): the 95th percentile of the horizontal
error at most 0.30 m at 15 m, 0.15 m at 6 m and 0.08 m at 2.4 m. Each is
judged on the samples taken while the vehicle was within a band around that
height, of which there must be at least 20. A band the vehicle was never in
is "not flown" and is not judged (a hold at 6 m never reaches 2.4 m), and
neither is one it only passed through, with fewer than 20 samples in it (a
gated drop releases half a second after reaching 2.4 m); a flight in none of
the bands fails. Only samples taken within 5 m of the
target, horizontally, and before the mission's release, count: the climb
after take-off and the landing at home pass through the bands with nothing
to track, and after a release the mission switches its detector off. The run folder gets estimator_check.json;
sim/runner/result.py reads it.

The arithmetic and the verdict are plain functions with no ROS in them.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Optional

from projection_check import target_in_local_frame

# (height, half-width of the band around it, limit on the 95th percentile), all in metres.
LINES = ((15.0, 1.5, 0.30), (6.0, 1.0, 0.15), (2.4, 0.4, 0.08))
MIN_IN_BAND = 20
# A sample counts for a band only while the vehicle is within this of the target, horizontally:
# climbing out after take-off and landing at home pass through the bands with no target to track.
NEAR_TARGET_M = 5.0
FLAG = "use_target_estimator"
CLASS_ID = "bullseye"
STATE_ORDER = ("CONFIRMED", "COASTING", "TENTATIVE", "LOST")
TARGETS_TOPIC = "/estimator/targets"
POSE_TOPIC = "/mavros/local_position/pose"


def wanted(manifest: dict) -> bool:
    """Whether a built scenario turns the estimator's flag on and has a target, so that its flight is checked.

    The check compares the estimator's bullseye track with the scenario's target. A scenario with no such target
    (the recovery mission's is its bucket) gives it nothing to compare, as with the projection check.
    """
    return (manifest.get("flags") or {}).get(FLAG) is True and manifest.get("target", True) is not False


def band_name(height_m: float) -> str:
    return f"at_{height_m:g}_m".replace(".", "_")


def parse_track_id(text: str) -> Optional[tuple[str, int, str]]:
    """("bullseye", 3, "CONFIRMED") from "bullseye/3/CONFIRMED", as the estimator writes a track's id; None for anything else."""
    parts = str(text).split("/")
    if len(parts) != 3 or not parts[1].isdigit():
        return None
    return parts[0], int(parts[1]), parts[2]


def _spread(values: list[float]) -> Optional[dict]:
    if not values:
        return None
    ordered = sorted(values)
    return {
        "count": len(ordered),
        "median_m": ordered[len(ordered) // 2],
        "p95_m": ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)],
        "max_m": ordered[-1],
    }


def build_report(records: list[dict], messages: int) -> dict:
    """The verdict for a flight.

    Each record is one publication of the tracks while the height was known:
    `height_m`, `state` (of the best bullseye track then: CONFIRMED before
    COASTING before TENTATIVE before LOST, or "none"), and, if a bullseye
    track was CONFIRMED, its `error_m` from the truth, its `sigma_m` and its
    `track_id` (None otherwise).
    """
    problems = []
    bands = {}
    if messages == 0:
        problems.append("nothing was heard on /estimator/targets")
    for height_m, half_width_m, limit_m in LINES:
        in_band = [r for r in records if abs(r["height_m"] - height_m) <= half_width_m]
        confirmed = [r for r in in_band if r["error_m"] is not None]
        spread = _spread([r["error_m"] for r in confirmed])
        sigmas = sorted(r["sigma_m"] for r in confirmed)
        band = {
            "limit_m": limit_m,
            "samples": len(in_band),
            "without_a_confirmed_track": len(in_band) - len(confirmed),
            "states": {state: sum(1 for r in in_band if r.get("state") == state) for state in sorted({r.get("state", "none") for r in in_band})},
            "error": spread,
            # What the filter itself said its error was, to set beside what it was.
            "median_sigma_m": sigmas[len(sigmas) // 2] if sigmas else None,
            "within_3_sigma": (sum(1 for r in confirmed if r["error_m"] <= 3.0 * r["sigma_m"]) / len(confirmed)) if confirmed else None,
            "passed": False,
            # A band the vehicle was never in has nothing to judge: a hold at 6 m never reaches 2.4 m.
            "flown": bool(in_band),
            # Fewer samples than can be judged: the vehicle passed through the band, as a gated drop does at 2.4 m.
            "brief": 0 < len(in_band) < MIN_IN_BAND,
        }
        low, high = height_m - half_width_m, height_m + half_width_m
        if messages and len(in_band) < MIN_IN_BAND:
            pass  # never there, or only passing through: nothing to judge
        elif messages and len(confirmed) < MIN_IN_BAND:
            problems.append(f"only {len(confirmed)} samples of a confirmed bullseye track between {low:g} and {high:g} m (at least {MIN_IN_BAND} needed)")
        elif messages and spread["p95_m"] > limit_m:
            problems.append(f"the bullseye track is {spread['p95_m']:.2f} m from the truth at the 95th percentile at {height_m:g} m (limit {limit_m:g} m)")
        elif messages:
            band["passed"] = True
        bands[band_name(height_m)] = band
    if messages and not any(band["flown"] and not band["brief"] for band in bands.values()):
        problems.append(
            "the vehicle was at none of the heights the estimator is judged at ("
            + ", ".join(f"{height_m:g} m" for height_m, _half, _limit in LINES)
            + f") long enough to judge it: {MIN_IN_BAND} samples near the target"
        )
    return {
        "passed": not problems,
        "problems": problems,
        "messages": messages,
        "samples": len(records),
        "confirmed_track_ids": sorted({r["track_id"] for r in records if r["track_id"] is not None}),
        **bands,
    }


def read_estimator_check(folder) -> dict | None:
    """The report a run folder holds, or None if there is none that can be read."""
    try:
        report = json.loads((Path(folder) / "estimator_check.json").read_text())
    except (OSError, json.JSONDecodeError):
        return None
    return report if isinstance(report, dict) and isinstance(report.get("passed"), bool) else None


class Checker:
    """Keeps the comparison as the tracks are published. No ROS: the node below feeds it."""

    def __init__(self, target_en: tuple[float, float]) -> None:
        self.target = target_en
        self.height_m: Optional[float] = None
        self.near_target = True  # until a position says otherwise
        self.released = False  # the mission has let its payload go: nothing after that is counted
        self.records: list[dict] = []
        self.messages = 0

    def on_height(self, up_m: float, east_m: Optional[float] = None, north_m: Optional[float] = None) -> None:
        """The vehicle's height, and with its position whether it is over the target's area."""
        self.height_m = float(up_m)
        if east_m is not None and north_m is not None:
            self.near_target = math.hypot(float(east_m) - self.target[0], float(north_m) - self.target[1]) <= NEAR_TARGET_M

    def on_mission_log(self, text: str) -> None:
        """A line of the mission executor's log: a release ends the counting."""
        from runner.events import from_mission_log

        if any(event.get("event") == "release" for event in from_mission_log(str(text))):
            self.released = True

    def on_targets(self, tracks: list[dict]) -> None:
        """One publication: each track is {"id": "<class>/<n>/<STATE>", "east_m", "north_m", "sigma_m"}."""
        self.messages += 1
        if self.height_m is None or not self.near_target or self.released:
            return
        confirmed, states = [], set()
        for track in tracks:
            parsed = parse_track_id(track.get("id", ""))
            if parsed is None or parsed[0] != CLASS_ID:
                continue
            states.add(parsed[2])
            if parsed[2] == "CONFIRMED":
                confirmed.append((parsed[1], track))
        best = next((state for state in STATE_ORDER if state in states), "none")
        record = {"height_m": self.height_m, "state": best, "error_m": None, "sigma_m": None, "track_id": None}
        if confirmed:
            track_id, track = min(confirmed, key=lambda item: item[0])
            record.update(
                error_m=math.hypot(float(track["east_m"]) - self.target[0], float(track["north_m"]) - self.target[1]),
                sigma_m=float(track["sigma_m"]),
                track_id=track_id,
            )
        self.records.append(record)

    def report(self) -> dict:
        return build_report(self.records, self.messages)


# --- the ROS side ---------------------------------------------------------------------


def listen(folder: Path) -> dict:
    """Listen until interrupted, writing estimator_check.json into the folder as it goes."""
    import time

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from vision_msgs.msg import Detection3DArray

    checker = Checker(target_in_local_frame(json.loads((folder / "truth.json").read_text())))
    rclpy.init()
    node = rclpy.create_node("argus_estimator_check", parameter_overrides=[Parameter("use_sim_time", value=True)])

    def on_targets(message) -> None:
        tracks = []
        for detection in message.detections:
            position = detection.bbox.center.position
            tracks.append({"id": detection.id, "east_m": position.x, "north_m": position.y, "sigma_m": 0.5 * detection.bbox.size.x})
        checker.on_targets(tracks)

    node.create_subscription(PoseStamped, POSE_TOPIC, lambda m: checker.on_height(m.pose.position.z, m.pose.position.x, m.pose.position.y), qos_profile_sensor_data)
    node.create_subscription(Detection3DArray, TARGETS_TOPIC, on_targets, 50)
    # The executor's log, for the release: the mission switches its detector off then and leaves.
    from rcl_interfaces.msg import Log

    node.create_subscription(Log, "/rosout", lambda m: checker.on_mission_log(m.msg), 200)

    def write() -> dict:
        report = checker.report()
        (folder / "estimator_check.json").write_text(json.dumps(report, indent=2) + "\n")
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


def summary_lines(report: dict) -> list[str]:
    """One line per pass line, for the terminal and the run's report."""
    lines = []
    for height_m, _half_width_m, limit_m in LINES:
        band = report.get(band_name(height_m)) or {}
        error = band.get("error")
        if band.get("flown") is False and report.get("messages"):
            lines.append(f"at {height_m:g} m: not flown")
        elif band.get("brief") and report.get("messages"):
            seen = f", p95 {error['p95_m']:.2f} m" if error and error.get("count") else ""
            lines.append(f"at {height_m:g} m: passed through, not judged: {band.get('samples', 0)} samples, {error['count'] if error else 0} with a confirmed track{seen}")
        elif not error:
            lines.append(f"at {height_m:g} m: no confirmed bullseye track in {band.get('samples', 0)} samples")
        else:
            lines.append(
                f"at {height_m:g} m: {'PASS' if band.get('passed') else 'FAIL'}, median {error['median_m']:.2f} m, p95 {error['p95_m']:.2f} m "
                f"over {error['count']} samples (limit {limit_m:g} m)"
            )
    return lines


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Compare the estimator's bullseye track with the scenario's truth.")
    parser.add_argument("folder", type=Path, help="the run folder: truth.json is read from it and estimator_check.json written to it")
    args = parser.parse_args(argv)
    report = listen(args.folder)
    for line in summary_lines(report):
        print(f"estimator against the truth {line}")
    for problem in report["problems"]:
        print(f"  {problem}")
    print("estimator check: PASS" if report["passed"] else "estimator check: FAIL", flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)  # as sim/time_check.py: leave without tearing the interpreter down
