#!/usr/bin/env python3
"""Check the localisation pipeline during a sim flight (LOC-5). Runs inside the sim container.

  python3 sim/loc_check.py <run folder>

Started by sim/runner/fly.py beside every flight whose scenario turns the
node flag `loc_node` on, and interrupted when the flight ends. It listens to
what the node says of itself, /loc/status (a JSON string once a second), and
to its boxes on /loc/detections, and holds the flight to these lines:

  - the pipeline was started during the flight (the survey mission starts
    it through /loc/set_enabled) and was stopped again before the end;
  - while it ran, its rate was at least MIN_RATE_HZ (8 frames a second) at
    the median, counted from the third status after the start, and never
    under SLOWEST_RATE_HZ (6): the node's rate is frames over the last five
    seconds, so a stall shows there;
  - a frame was no older than MAX_FRAME_AGE_MS when its turn came, at the
    95th percentile: the pipeline drops frames, it does not queue them;
  - no frame faulted, and at least MIN_PLACED_SHARE of the boxes were put
    on the ground;
  - every class of object the scenario has on the field (from the run's
    oracle_objects.json) was reported at least once.

With the marker tracker on as well (`marker_tracker` in the node's flags,
LOC-7), its last message on /loc/tracks is held to the scenario's true
markers: every numbered marker inside the four corner markers has exactly
one listed track (confirmed, not marked outside) within TRACK_TOLERANCE_M;
no listed track is without a marker; and a marker outside the corners has
a confirmed track that is marked outside. The distance of each track from
its marker is reported.

The run folder gets loc_check.json; sim/runner/result.py reads it. The
verdict is a plain function with no ROS in it.
"""

from __future__ import annotations

import json
import math
import sys
import threading
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

MIN_RATE_HZ = 8.0
SLOWEST_RATE_HZ = 6.0  # no five seconds of the run may be slower than this: a stall does not hide in the median
MAX_FRAME_AGE_MS = 250.0
MIN_PLACED_SHARE = 0.9
SETTLE_STATUSES = 2  # the first statuses after a start have a rate over too few frames
STATUS_TOPIC = "/loc/status"
DETECTIONS_TOPIC = "/loc/detections"
TRACKS_TOPIC = "/loc/tracks"
TRACK_TOLERANCE_M = 1.0  # a listed track this close to a true marker is that marker's
LISTED_STATES = (1, 2)  # MarkerTrack.msg: CONFIRMED, COASTING


def wanted(manifest: dict) -> bool:
    """Whether a built scenario turns the localisation node on, so that its flight is checked."""
    return ((manifest.get("node_flags") or {}).get("loc_node") or {}).get("loc_node") is True


def expected_classes(run_dir) -> Set[str]:
    """The classes of object the scenario has on the field, from the list the build wrote for the oracle."""
    try:
        data = json.loads((Path(run_dir) / "oracle_objects.json").read_text())
    except (OSError, json.JSONDecodeError):
        return set()
    return {str(item["class_id"]) for item in data.get("objects") or []}


def tracks_wanted(manifest: dict) -> bool:
    """Whether a built scenario turns the marker tracker on, so that its tracks are held to the truth."""
    return wanted(manifest) and ((manifest.get("node_flags") or {}).get("loc_node") or {}).get("marker_tracker") is True


def median_wanted(manifest: dict) -> bool:
    """Whether a built scenario asks for a track's reported position to be the weighted median of its sightings (LOC-9)."""
    return tracks_wanted(manifest) and ((manifest.get("node_flags") or {}).get("loc_node") or {}).get("loc_median_position") is True


def true_markers(run_dir) -> Tuple[List[dict], List[Tuple[float, float]]]:
    """(the numbered markers, the corner markers) of the scenario, east and north of the take-off point: MAVROS's local frame."""
    truth = json.loads((Path(run_dir) / "truth.json").read_text())
    takeoff = truth.get("takeoff") or {"e": 0.0, "n": 0.0}
    markers, corners = [], []
    for item in truth.get("objects") or []:
        if item.get("kind") != "marker":
            continue
        east, north = float(item["e"]) - float(takeoff["e"]), float(item["n"]) - float(takeoff["n"])
        if item.get("digit") is None:
            corners.append((east, north))
        else:
            markers.append({"name": str(item["name"]), "e": east, "n": north})
    return markers, corners


def _inside(point: Tuple[float, float], corners: Sequence[Tuple[float, float]]) -> bool:
    """Whether a point is inside the corner markers (taken in order round their centre). With under three corners everything is."""
    if len(corners) < 3:
        return True
    centre = (sum(c[0] for c in corners) / len(corners), sum(c[1] for c in corners) / len(corners))
    ring = sorted(corners, key=lambda c: math.atan2(c[1] - centre[1], c[0] - centre[0]))
    inside = False
    for (ax, ay), (bx, by) in zip(ring, ring[1:] + ring[:1]):
        if (ay > point[1]) != (by > point[1]) and point[0] < (bx - ax) * (point[1] - ay) / (by - ay) + ax:
            inside = not inside
    return inside


def judge_tracks(tracks: Optional[List[dict]], markers: List[dict], corners: Sequence[Tuple[float, float]]) -> dict:
    """The tracker's last word against the true markers. `tracks`: {id, state, e, n, outside, sightings} each; None if none was heard."""
    problems: List[str] = []
    if tracks is None:
        return {"problems": [f"no message was heard on {TRACKS_TOPIC}: the tracker did not run"], "listed": 0, "markers_inside": 0, "outside_marked": 0, "errors_m": {}, "largest_error_m": None}
    listed = [track for track in tracks if track["state"] in LISTED_STATES and not track["outside"]]
    inside = [marker for marker in markers if _inside((marker["e"], marker["n"]), corners)]
    beyond = [marker for marker in markers if marker not in inside]
    errors: Dict[str, float] = {}
    for marker in inside:
        near = [math.hypot(track["e"] - marker["e"], track["n"] - marker["n"]) for track in listed]
        near = sorted(distance for distance in near if distance <= TRACK_TOLERANCE_M)
        if len(near) != 1:
            problems.append(f"{marker['name']} has {'no' if not near else len(near)} listed track{'s' if len(near) > 1 else ''} within {TRACK_TOLERANCE_M:g} m")
        if near:
            errors[marker["name"]] = near[0]
    for track in listed:
        if not any(math.hypot(track["e"] - marker["e"], track["n"] - marker["n"]) <= TRACK_TOLERANCE_M for marker in inside):
            mine = next((marker for marker in beyond if math.hypot(track["e"] - marker["e"], track["n"] - marker["n"]) <= TRACK_TOLERANCE_M), None)
            if mine is None:
                problems.append(f"a listed track at {track['e']:.1f}, {track['n']:.1f} has no marker within {TRACK_TOLERANCE_M:g} m")
            else:
                problems.append(f"{mine['name']} is outside the boundary markers and its track is on the list")
    outside_marked = 0
    for marker in beyond:
        confirmed = [t for t in tracks if t["state"] in LISTED_STATES and math.hypot(t["e"] - marker["e"], t["n"] - marker["n"]) <= TRACK_TOLERANCE_M]
        if any(track["outside"] for track in confirmed):
            outside_marked += 1
        elif not confirmed:
            problems.append(f"{marker['name']} is outside the boundary markers and no confirmed track of it is marked outside")
    return {
        "problems": problems,
        "listed": len(listed),
        "markers_inside": len(inside),
        "outside_marked": outside_marked,
        "errors_m": {name: round(value, 4) for name, value in errors.items()},
        "largest_error_m": max(errors.values()) if errors else None,
    }


def _percentile(values: Sequence[float], share: float) -> Optional[float]:
    ordered = sorted(values)
    if not ordered:
        return None
    return ordered[min(len(ordered) - 1, max(0, int(round(share * len(ordered) + 0.5)) - 1))]


def build_report(
    statuses: List[Tuple[float, dict]],
    boxes: List[Tuple[float, Iterable[str]]],
    expected: Set[str],
    tracks: Optional[List[dict]] = None,
    markers: Optional[List[dict]] = None,
    corners: Optional[Sequence[Tuple[float, float]]] = None,
    tracks_wanted: Optional[bool] = None,
    median_wanted: bool = False,
) -> dict:
    """The verdict: `statuses` are (sim time, the node's status), `boxes` (sim time, the classes in one message).

    With the tracker on (`tracks_wanted`; taken as on when tracks are
    given) the last /loc/tracks message is judged against the true
    `markers` and `corners` (judge_tracks).
    """
    problems: List[str] = []
    judge_list = (tracks is not None) if tracks_wanted is None else bool(tracks_wanted)
    track_verdict = judge_tracks(tracks, markers or [], corners or []) if judge_list else None
    running = [index for index, (_t, status) in enumerate(statuses) if status.get("enabled")]
    started = bool(running)
    stopped = started and not statuses[-1][1].get("enabled")
    last = statuses[-1][1] if statuses else {}
    settled = [statuses[index][1] for index in running[SETTLE_STATUSES:]]
    rates = [float(status.get("rate_hz") or 0.0) for status in settled]
    ages = [float(status["frame_age_ms"]) for status in settled if status.get("frame_age_ms") is not None]
    classes: Dict[str, int] = {}
    for _t, names in boxes:
        for name in names:
            classes[str(name)] = classes.get(str(name), 0) + 1
    frames, detections, placed, faults = (int(last.get(key) or 0) for key in ("frames", "detections", "placed", "faults"))
    median_rate, slowest = _percentile(rates, 0.5), (min(rates) if rates else None)
    age_p95 = _percentile(ages, 0.95)
    if not statuses:
        problems.append(f"no status was heard on {STATUS_TOPIC}: the node did not run, or its flag was off")
    elif not started:
        problems.append("the pipeline never started: nothing called /loc/set_enabled")
    else:
        if not stopped:
            problems.append("the pipeline was still running at the end of the flight: nothing stopped it")
        if median_rate is None:
            problems.append(f"the pipeline ran for under {SETTLE_STATUSES + 1} statuses, too short to have a rate")
        elif median_rate < MIN_RATE_HZ:
            problems.append(f"the pipeline ran at {median_rate:.1f} frames a second at the median (at least {MIN_RATE_HZ:g} needed)")
        elif slowest is not None and slowest < SLOWEST_RATE_HZ:
            problems.append(f"the pipeline's slowest five seconds ran at {slowest:.1f} frames a second (at least {SLOWEST_RATE_HZ:g} needed): it stalled")
        if age_p95 is not None and age_p95 > MAX_FRAME_AGE_MS:
            problems.append(f"frames were {age_p95:.0f} ms old when their turn came, at the 95th percentile (limit {MAX_FRAME_AGE_MS:g} ms): the pipeline is behind its camera")
        if faults:
            problems.append(f"{faults} frame(s) faulted in the pipeline: {last.get('last_fault') or 'no reason kept'}")
        if detections and placed < MIN_PLACED_SHARE * detections:
            problems.append(f"only {placed} of {detections} boxes were put on the ground (at least {100.0 * MIN_PLACED_SHARE:.0f}% needed): {last.get('last_refusal') or 'no reason kept'}")
        missing = sorted(expected - set(classes))
        if missing:
            problems.append(f"nothing was reported of class {', '.join(missing)}, which the scenario has on the field")
    if track_verdict is not None and statuses and started:
        problems += track_verdict["problems"]
        # In a calm sim the median and the filter's position are centimetres apart: the node's own word is what
        # shows which of them the tracks carried.
        if median_wanted and last.get("track_positions") != "median":
            problems.append(f"the scenario asks for tracks at the weighted median of their sightings, and the node says it reports '{last.get('track_positions')}'")
    return {
        "passed": not problems,
        "problems": problems,
        "tracks": track_verdict,
        "track_positions": last.get("track_positions"),
        "started": started,
        "stopped": stopped,
        "statuses": len(statuses),
        "frames": frames,
        "dropped": int(last.get("dropped") or 0),
        "detections": detections,
        "placed": placed,
        "faults": faults,
        "rate_hz": {"median": median_rate, "slowest": slowest, "min_hz": MIN_RATE_HZ},
        "frame_age_ms": {"p95": age_p95, "limit_ms": MAX_FRAME_AGE_MS},
        "messages": len(boxes),
        "classes": dict(sorted(classes.items())),
        "detector": last.get("detector"),
    }


def read_loc_check(folder) -> Optional[dict]:
    try:
        report = json.loads((Path(folder) / "loc_check.json").read_text())
    except (OSError, json.JSONDecodeError):
        return None
    return report if isinstance(report, dict) and isinstance(report.get("passed"), bool) else None


def summary_lines(report: dict) -> List[str]:
    """Lines for the terminal and the run's report."""
    rate, age = report.get("rate_hz") or {}, report.get("frame_age_ms") or {}
    if rate.get("median") is None:
        lines = [f"{'PASS' if report.get('passed') else 'FAIL'}: {report.get('frames', 0)} frames, no rate"]
    else:
        lines = [
            f"{'PASS' if report.get('passed') else 'FAIL'}: {rate['median']:.1f} frames a second at the median (slowest {rate['slowest']:.1f}) over {report.get('frames', 0)} frames, "
            f"{report.get('dropped', 0)} dropped, detector '{report.get('detector')}'"
        ]
    if age.get("p95") is not None:
        lines.append(f"frames {age['p95']:.0f} ms old at their turn, at the 95th percentile")
    if report.get("classes"):
        lines.append("boxes: " + ", ".join(f"{count} {name}" for name, count in report["classes"].items()) + f"; {report.get('placed', 0)} of {report.get('detections', 0)} put on the ground")
    tracked = report.get("tracks")
    if tracked:
        matched = len(tracked.get("errors_m") or {})
        furthest = "" if tracked.get("largest_error_m") is None else f", the furthest {tracked['largest_error_m']:.2f} m from its marker"
        lines.append(
            f"tracks: {matched} of {tracked.get('markers_inside', 0)} markers inside the boundary have their one listed track{furthest}; "
            f"{tracked.get('listed', 0)} listed, {tracked.get('outside_marked', 0)} marked outside"
            + (f"; positions reported: the {report['track_positions']}" if report.get("track_positions") else "")
        )
    return lines


def listen(run_dir: Path) -> dict:
    """Listen until interrupted, writing loc_check.json into the run folder as it goes."""
    import time

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from vision_msgs.msg import Detection2DArray

    manifest = json.loads((run_dir / "scenario.json").read_text())
    expected = expected_classes(run_dir)
    with_tracks, with_median = tracks_wanted(manifest), median_wanted(manifest)
    markers, corners = true_markers(run_dir) if with_tracks else ([], [])
    statuses: List[Tuple[float, dict]] = []
    boxes: List[Tuple[float, List[str]]] = []
    last_tracks: List[Optional[List[dict]]] = [None]
    lock = threading.Lock()
    rclpy.init()
    node = rclpy.create_node("argus_loc_check", parameter_overrides=[Parameter("use_sim_time", value=manifest.get("sim_time") is True)])

    def now_s() -> float:
        return node.get_clock().now().nanoseconds * 1.0e-9

    def on_status(message) -> None:
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        with lock:
            statuses.append((now_s(), status))

    def on_boxes(message) -> None:
        names = [str(result.hypothesis.class_id) for detection in message.detections for result in detection.results[:1]]
        with lock:
            boxes.append((now_s(), names))

    def on_tracks(message) -> None:
        tracks = [
            {"id": int(t.track_id), "state": int(t.state), "e": float(t.position_enu.x), "n": float(t.position_enu.y), "outside": bool(t.outside), "sightings": int(t.sightings)}
            for t in message.tracks
        ]
        with lock:
            last_tracks[0] = tracks

    if with_tracks:
        from argus_interfaces.msg import MarkerTrackArray

        node.create_subscription(MarkerTrackArray, TRACKS_TOPIC, on_tracks, 10)
    node.create_subscription(String, STATUS_TOPIC, on_status, 10)
    node.create_subscription(Detection2DArray, DETECTIONS_TOPIC, on_boxes, 50)

    def write() -> dict:
        with lock:
            report = build_report(list(statuses), list(boxes), expected, tracks=last_tracks[0], markers=markers, corners=corners, tracks_wanted=with_tracks, median_wanted=with_median)
        (run_dir / "loc_check.json").write_text(json.dumps(report, indent=2) + "\n")
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


def main(argv: Optional[List[str]] = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__)
        return 2
    report = listen(Path(argv[0]))
    for line in summary_lines(report):
        print("localisation pipeline: " + line)
    for problem in report["problems"]:
        print("  " + problem)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
