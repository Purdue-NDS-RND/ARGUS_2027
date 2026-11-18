"""The numbers a replay gives (SIM-09). No ROS imports; plain data in, plain data out.

From a `core.Replay` that has run a bag:

  - the estimator's error over time and at the release (when the truth and
    the release time are given);
  - the mean normalised innovation squared (NIS) and the fraction above the
    gate, 9.21: a filter whose noise model is right has a mean near 2 and
    about 1% above;
  - time to lock: from the first frame with a detection of the class to its
    first CONFIRMED track;
  - lost-state duration: the time the class's track was LOST, to the end of
    the bag; and, separately, how long the class was without a CONFIRMED
    track between its first lock and its last;
  - capture-to-output latency: from a frame's capture stamp to the moment
    its detections were heard;
  - when the bag holds the tracks the flight's own estimator published:
    how far the replayed track is from them.

Flow velocity error and per-marker localisation error belong to code that
is not built yet (the flow lane, LOC-7); they are added with it.
"""

from __future__ import annotations

import bisect
import math
from typing import Any, List, Optional, Sequence

GATE_NIS = 9.21
CLASS_ID = "bullseye"
_ORDER = {"CONFIRMED": 0, "COASTING": 1, "TENTATIVE": 2, "LOST": 3}


def spread(values: Sequence[float], digits: int = 6) -> Optional[dict]:
    """Count, median, 95th percentile and maximum of some numbers, or None for none."""
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    return {
        "count": len(ordered),
        "median": round(ordered[len(ordered) // 2], digits),
        "p95": round(ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)], digits),
        "max": round(ordered[-1], digits),
    }


def primary(tracks: List[dict], class_id: str) -> Optional[dict]:
    """The track of a class a mission would act on, among a tick's tracks: by state, then by hits, then the oldest."""
    candidates = [track for track in tracks if track["class"] == class_id]
    if not candidates:
        return None
    return min(candidates, key=lambda track: (_ORDER.get(track["state"], 9), -track["hits"], track["id"]))


COMPARE_WITHIN_NS = 60_000_000  # a replayed tick is set against the recorded tracks published within this of it


def against_recorded(samples: List[dict], recorded: List[dict], class_id: str) -> Optional[dict]:
    """How far the replayed track is from the one the flight's own estimator published, where both were CONFIRMED at about the same time."""
    if not recorded:
        return None
    times = [sample["t_ns"] for sample in recorded]
    distances, both, only_replayed, only_recorded = [], 0, 0, 0
    for sample in samples:
        index = bisect.bisect_left(times, sample["t_ns"])
        near = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(times)]
        nearest = min(near, key=lambda candidate: abs(times[candidate] - sample["t_ns"]))
        if abs(times[nearest] - sample["t_ns"]) > COMPARE_WITHIN_NS:
            continue
        ours, theirs = primary(sample["tracks"], class_id), primary(recorded[nearest]["tracks"], class_id)
        ours_confirmed = ours is not None and ours["state"] == "CONFIRMED"
        theirs_confirmed = theirs is not None and theirs["state"] == "CONFIRMED"
        if ours_confirmed and theirs_confirmed:
            both += 1
            distances.append(math.hypot(ours["east_m"] - theirs["east_m"], ours["north_m"] - theirs["north_m"]))
        elif ours_confirmed:
            only_replayed += 1
        elif theirs_confirmed:
            only_recorded += 1
    return {"both_confirmed": both, "confirmed_only_in_the_replay": only_replayed, "confirmed_only_in_the_flight": only_recorded, "distance_m": spread(distances)}


def summarise(replay: Any, truth_en: Optional[Sequence[float]] = None, release_ns: Optional[int] = None, class_id: str = CLASS_ID) -> dict:
    samples, frames = replay.samples, replay.frames
    start_ns = samples[0]["t_ns"] if samples else None
    series = []  # (seconds from the first tick, state, error or None)
    for sample in samples:
        track = primary(sample["tracks"], class_id)
        error = None
        if track is not None and truth_en is not None:
            error = math.hypot(track["east_m"] - float(truth_en[0]), track["north_m"] - float(truth_en[1]))
        series.append(((sample["t_ns"] - start_ns) * 1.0e-9, "none" if track is None else track["state"], error, sample["t_ns"]))

    confirmed_errors = [error for _t, state, error, _ns in series if state == "CONFIRMED" and error is not None]
    first_seen_ns = min((frame["stamp_ns"] for frame in frames if frame["detections"]), default=None)
    locked = [t_ns for _t, state, _error, t_ns in series if state == "CONFIRMED"]
    time_to_lock = None if not locked or first_seen_ns is None else round((locked[0] - first_seen_ns) * 1.0e-9, 6)
    unlocked_after = 0.0
    if locked:
        for (_t, state, _error, t_ns), following in zip(series, series[1:]):
            if locked[0] <= t_ns < locked[-1] and state != "CONFIRMED":
                unlocked_after += (following[3] - t_ns) * 1.0e-9
    # Time the class's track spent LOST, to the end of the bag: a loss that is never made good counts in full.
    lost_s = 0.0
    for (_t, state, _error, t_ns), following in zip(series, series[1:]):
        if state == "LOST":
            lost_s += (following[3] - t_ns) * 1.0e-9
    at_release = None
    if release_ns is not None and series:
        nearest = min(series, key=lambda item: abs(item[3] - release_ns))
        at_release = {"state": nearest[1], "error_m": None if nearest[2] is None else round(nearest[2], 6), "tick_offset_s": round((nearest[3] - release_ns) * 1.0e-9, 6)}

    innovations = list(replay.innovations)
    # A frame's stamp is on the camera's clock and "heard" on the recorder's. When they differ (a sim run on wall
    # time) the stamp is first converted through the bag's sim-clock tap; where that cannot be done it is left out.
    to_recorder = getattr(replay, "to_recorder_time", None) or (lambda stamp: stamp)
    latency = [(frame["heard_ns"] - converted) * 1.0e-9 for frame in frames for converted in [to_recorder(frame["stamp_ns"])] if converted is not None]
    states: dict = {}
    for _t, state, _error, _ns in series:
        states[state] = states.get(state, 0) + 1
    return {
        "class": class_id,
        "ticks": len(samples),
        "frames": len(frames),
        "counters": dict(sorted(replay.feed.counters.items())),
        "undecodable": dict(sorted(replay.undecodable.items())),
        "totals": {name: getattr(replay.feed.totals, name) for name in ("accepted", "rejected", "resets", "started", "ignored", "untracked", "invalid")},
        "states": dict(sorted(states.items())),
        "error_m": spread(confirmed_errors),  # of the CONFIRMED primary track, every tick
        "error_over_time": [[round(t, 3), state, None if error is None else round(error, 4)] for t, state, error, _ns in series],
        "error_at_release": at_release,
        "nis": None if not innovations else {
            "count": len(innovations),
            "mean": round(sum(innovations) / len(innovations), 6),
            "fraction_above_gate": round(sum(1 for value in innovations if value > GATE_NIS) / len(innovations), 6),
        },
        "time_to_lock_s": time_to_lock,
        "without_lock_after_first_lock_s": round(unlocked_after, 6) if locked else None,
        "lost_state_s": round(lost_s, 6),
        "capture_to_heard_s": spread(latency),
        "against_the_recorded_tracks": against_recorded(samples, getattr(replay, "recorded", []), class_id),
        "tracks_at_end": samples[-1]["tracks"] if samples else [],
    }


def short_lines(result: dict) -> List[str]:
    """The metrics in a few lines, for a terminal."""
    lines = [f"{result['frames']} frames with detections, {result['ticks']} ticks; states of the {result['class']} track: {result['states']}"]
    if result["error_m"]:
        e = result["error_m"]
        lines.append(f"error of the CONFIRMED {result['class']} track: median {e['median']:.3f} m, p95 {e['p95']:.3f} m, max {e['max']:.3f} m over {e['count']} ticks")
    if result["error_at_release"]:
        lines.append(f"at the release: {result['error_at_release']}")
    if result["nis"]:
        n = result["nis"]
        lines.append(f"NIS: mean {n['mean']:.2f}, {100.0 * n['fraction_above_gate']:.1f}% above {GATE_NIS} over {n['count']} measurements")
    if result["time_to_lock_s"] is not None:
        lines.append(
            f"time to lock {result['time_to_lock_s']:.2f} s; LOST for {result['lost_state_s']:.2f} s; "
            f"without a lock between the first and the last {result['without_lock_after_first_lock_s']:.2f} s"
        )
    compared = result.get("against_the_recorded_tracks")
    if compared and compared["distance_m"]:
        d = compared["distance_m"]
        lines.append(
            f"against the tracks the flight itself published: median {d['median']:.3f} m, max {d['max']:.3f} m apart over {compared['both_confirmed']} ticks; "
            f"CONFIRMED only in the replay {compared['confirmed_only_in_the_replay']}, only in the flight {compared['confirmed_only_in_the_flight']}"
        )
    if result["capture_to_heard_s"]:
        c = result["capture_to_heard_s"]
        lines.append(f"capture to heard: median {1000 * c['median']:.0f} ms, p95 {1000 * c['p95']:.0f} ms, max {1000 * c['max']:.0f} ms over {c['count']} frames")
    return lines
