"""The post-landing second pass (LOC-12): the survey's logged frames looked at again, slowly, and the result merged with the live tracks.

In flight the detector sees each 4K frame shrunk to its own size, or tiled
at whatever rate it can keep, and frames it has no time for are gone. The
frame log (LOC-11) keeps them all. After landing there are minutes to look
again: every logged frame, tiled at full resolution, through the same
pipeline (detector, projection at the frame's own pose, tracker, digit
reader, vote), and what that finds is merged with what the flight found.
The list that goes to the judges is due 30 minutes after the slot; the pass
is given 900 s and no more.

Behind a flag (`features.post_landing_pass`, default false): whether a list
may hold results worked out after landing is the organisers' to rule on
(question 4), and until they do the in-flight list is the list.

What is here, with no ROS in it:

  FlightRecord     the poses and rangefinder readings of the survey, written
                   (by a thread of its own: the node does not wait on the
                   disk) beside the frame log's index (`flight_record.jsonl`): a
                   frame can only be put on the ground again with the pose
                   of its own moment, and the pose history in memory is five
                   seconds long.
  save_live_tracks / load_live_tracks
                   the tracks as the flight left them, with each one's digit
                   votes (`live_tracks.json`).
  SecondPass.run   the replay: the record's poses and ranges fed to the pose
                   history and the ground height in the order of their
                   stamps, each logged frame read and given to the pipeline
                   when its moment is reached. Frames go through in the
                   order they were taken (the tracker ages its tracks by the
                   frames' stamps), and a step of the flight controller's
                   position found in the record moves the tracks made before
                   it, as it did in the flight. The budget is kept by looking at fewer
                   frames, evenly: after every frame the pass asks how many
                   of those left it has time for and steps over the rest.
  merge            a second-pass track within MERGE_GATE_M of a live track
                   of the same class is the same marker: the place is the
                   mean weighted by sightings, the digit votes are summed.
                   A live track nothing matched stays as it is: the pass
                   never takes a marker off the list. A second-pass track
                   nothing matched is added if it is confirmed. The digits
                   are then given out anew over all of them by the vote's
                   own rule (digit_vote.DigitVotes).

The pass writes nothing itself: `tools/second_pass.py` and the node hand
the merged tracks to the judges' list (judges_list.py), which keeps the
list before it under another name.
"""

from __future__ import annotations

import json
import math
import queue
import threading
import time
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

from . import marker_tracker
from .digit_vote import DigitVotes
from .frames import TopicFrame

FLAG = "post_landing_pass"
BUDGET_S = 900.0  # the spec's: within 900 s, so the list still meets the 30 minutes
RESERVE_S = 45.0  # of the budget, kept for the merge and the writing of the list
RECORD_NAME = "flight_record.jsonl"
LIVE_TRACKS_NAME = "live_tracks.json"
INDEX_NAME = "index.jsonl"  # the frame log's own index (FrameLog writes it)
MERGE_GATE_M = 1.5  # two tracks of one class this close are one marker (the tracker's own gate is 1 m; the two passes each carry their own error)
POSE_AHEAD_NS = 200_000_000  # the record is fed this far past a frame's stamp before the frame is looked at: its pose is between two poses
LISTED = (marker_tracker.CONFIRMED, marker_tracker.COASTING)
S = 1_000_000_000
_END = object()  # the record's last word to its own thread


class SecondPassError(RuntimeError):
    """The pass cannot be run on this folder. The message says why."""


# --- the survey's poses and ranges ---------------------------------------------------------------------------


class FlightRecord:
    """Writes the survey's poses, velocities and rangefinder readings beside the frame log, a line each.

    Whoever gives it a line does not wait on the disk: the lines are queued and written by a thread of the
    record's own (the node gives them from its one executor thread, beside the frame log's 4K frames on the same
    disk). `close(then)` ends the record; `then`, if given, is run on that thread after the last line is written
    (the node writes the flight's tracks there). `wait()` waits for the thread, for shutdown and for tests.
    """

    def __init__(self, folder: Any, flush_every: int = 50) -> None:
        self.path = Path(folder) / RECORD_NAME
        self._file = open(self.path, "a", encoding="utf-8")  # (opening it here: a folder that cannot be written is known at once)
        self._flush_every, self.lines, self.failed, self._closed = int(flush_every), 0, "", False
        self._queue: "queue.SimpleQueue[Any]" = queue.SimpleQueue()
        self._thread = threading.Thread(target=self._work, name="flight_record", daemon=True)
        self._thread.start()

    def _work(self) -> None:
        pending = 0
        while True:
            item = self._queue.get()
            if isinstance(item, tuple) and item and item[0] is _END:
                try:
                    self._file.close()
                except OSError as problem:
                    self.failed = self.failed or str(problem)
                if item[1] is not None:
                    try:
                        item[1]()
                    except Exception as problem:  # noqa: BLE001  (what is done at the end must not take the thread down unsaid)
                        self.failed = self.failed or f"{type(problem).__name__}: {problem}"
                return
            if self.failed:
                continue  # a record that has failed takes no more lines, and the queue is still emptied
            try:
                self._file.write(json.dumps(item, separators=(",", ":")) + "\n")
                self.lines += 1
                pending += 1
                if pending >= self._flush_every:
                    self._file.flush()
                    pending = 0
            except (OSError, ValueError) as problem:  # a full disk must not reach the survey: the record stops, and says so
                self.failed = str(problem)

    def _put(self, record: Dict[str, Any]) -> None:
        if not self._closed:
            self._queue.put(record)

    def pose(self, stamp_ns: int, position: Sequence[float], orientation: Sequence[float]) -> None:
        self._put({"k": "pose", "t": int(stamp_ns), "p": [float(value) for value in position], "q": [float(value) for value in orientation]})

    def velocity(self, stamp_ns: int, linear: Sequence[float]) -> None:
        self._put({"k": "vel", "t": int(stamp_ns), "v": [float(value) for value in linear]})

    def range(self, stamp_ns: int, range_m: float, min_m: float, max_m: float) -> None:
        self._put({"k": "range", "t": int(stamp_ns), "r": float(range_m), "min": float(min_m), "max": float(max_m)})

    def close(self, then: Optional[Callable[[], None]] = None, wait_s: Optional[float] = None) -> None:
        """No more lines. The file is closed, and `then` run, on the record's thread; `wait_s` waits that long for it."""
        if not self._closed:
            self._closed = True
            self._queue.put((_END, then))
        if wait_s is not None:
            self.wait(wait_s)

    def wait(self, timeout_s: float = 5.0) -> bool:
        self._thread.join(timeout=timeout_s)
        return not self._thread.is_alive()


def read_record(folder: Any) -> List[Dict[str, Any]]:
    """The record's lines in the order of their stamps. A line that is not whole (the last one of a log cut short) is left out."""
    path = Path(folder) / RECORD_NAME
    if not path.is_file():
        raise SecondPassError(f"{path} is not there: the survey's poses were not recorded (the flag {FLAG} was off in the flight), and a frame cannot be put on the ground without the pose of its moment")
    events = []
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            event = json.loads(line)
            if event.get("k") in ("pose", "vel", "range") and isinstance(event.get("t"), int):
                events.append(event)
        except ValueError:
            continue
    events.sort(key=lambda event: event["t"])
    return events


def read_index(folder: Any) -> List[Dict[str, Any]]:
    """The frame log's frames in the order they were taken: {"seq", "capture_ns", "file", "width", "height"}."""
    path = Path(folder) / INDEX_NAME
    if not path.is_file():
        raise SecondPassError(f"{path} is not there: this is not a frame log's folder")
    frames = []
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            record = json.loads(line)
            if isinstance(record.get("capture_ns"), int) and isinstance(record.get("file"), str):
                frames.append(record)
        except ValueError:
            continue
    frames.sort(key=lambda record: record["capture_ns"])
    return frames


# --- the live tracks, kept for the merge ---------------------------------------------------------------------


@dataclass(frozen=True)
class KeptTrack:
    """A track as it is kept between the passes: MarkerTrackState's fields the list needs, and its digit votes' weight and count."""

    track_id: int
    class_id: str
    state: int
    reported_east_m: float
    reported_north_m: float
    sightings: int
    outside: bool = False
    up_m: float = 0.0
    digit: int = -1
    digit_log_prob: Tuple[float, ...] = (0.0,) * 10
    vote_weight: float = 0.0
    vote_readings: int = 0
    source: str = "live"  # "live", "second pass" or "both"

    @property
    def east_m(self) -> float:
        return self.reported_east_m

    @property
    def north_m(self) -> float:
        return self.reported_north_m


def kept(tracks: Iterable[Any], votes: Any = None, source: str = "live") -> List[KeptTrack]:
    """The tracker's tracks (MarkerTrackState) as kept tracks, with the vote's weight and count for each when there is a vote."""
    out = []
    for track in tracks:
        sums = tuple(float(value) for value in getattr(track, "digit_log_prob", (0.0,) * 10))
        out.append(KeptTrack(
            int(track.track_id), str(track.class_id), int(track.state), float(track.reported_east_m), float(track.reported_north_m), int(track.sightings),
            bool(getattr(track, "outside", False)), float(getattr(track, "up_m", 0.0)), int(getattr(track, "digit", -1)), sums if len(sums) == 10 else (0.0,) * 10,
            0.0 if votes is None else float(votes.weight(track.track_id)), 0 if votes is None else int(votes.readings(track.track_id)), source,
        ))  # fmt: skip
    return out


SETTINGS = {"ground_below_home_m": 0.0, "digit_min_confidence": 0.9, "median_positions": False, "digit_ids_unique": True}  # the node's own defaults


def save_live_tracks(path: Any, tracks: Sequence[KeptTrack], origin: Optional[Tuple[float, float]], settings: Optional[Dict[str, Any]] = None) -> None:
    """The flight's tracks, the local frame's origin (latitude, longitude) the list places them by, and the settings of the
    flight's pipeline that the second pass must use as well (SETTINGS's keys), written whole or not at all."""
    from .judges_list import write_atomic

    unknown = set(settings or {}) - set(SETTINGS)
    if unknown:
        raise ValueError(f"not settings of the pipeline that the second pass takes: {sorted(unknown)}")
    body = {"origin": None if origin is None else [float(origin[0]), float(origin[1])], "settings": {**SETTINGS, **(settings or {})},
            "tracks": [{**{name: getattr(track, name) for name in KeptTrack.__dataclass_fields__ if name != "digit_log_prob"}, "digit_log_prob": list(track.digit_log_prob)} for track in tracks]}  # fmt: skip
    write_atomic(Path(path), (json.dumps(body, indent=1) + "\n").encode("utf-8"))


def load_live_tracks(path: Any) -> Tuple[List[KeptTrack], Optional[Tuple[float, float]], Dict[str, Any]]:
    """(the flight's tracks, the local frame's origin, the flight's pipeline settings)."""
    try:
        body = json.loads(Path(path).read_text(encoding="utf-8"))
        tracks = [KeptTrack(**{**item, "digit_log_prob": tuple(float(value) for value in item["digit_log_prob"])}) for item in body["tracks"]]
        origin = None if body.get("origin") is None else (float(body["origin"][0]), float(body["origin"][1]))
        settings = {**SETTINGS, **{name: value for name, value in (body.get("settings") or {}).items() if name in SETTINGS}}
        settings = {"ground_below_home_m": float(settings["ground_below_home_m"]), "digit_min_confidence": float(settings["digit_min_confidence"]),
                    "median_positions": bool(settings["median_positions"]), "digit_ids_unique": bool(settings["digit_ids_unique"])}  # fmt: skip
    except (OSError, ValueError, KeyError, TypeError) as problem:
        raise SecondPassError(f"{path} is not the flight's tracks: {problem}") from problem
    for track in tracks:
        if len(track.digit_log_prob) != 10 or not all(math.isfinite(value) for value in (track.reported_east_m, track.reported_north_m, *track.digit_log_prob)):
            raise SecondPassError(f"{path}: track {track.track_id} is not whole")
    return tracks, origin, settings


# --- the replay ----------------------------------------------------------------------------------------------


@dataclass
class PassResult:
    frames_logged: int = 0
    frames_looked_at: int = 0
    frames_unreadable: int = 0
    frames_without_pose: int = 0
    widest_step: int = 1  # the most frames stepped over at once to keep the budget, plus one
    elapsed_s: float = 0.0
    stopped_by: str = ""  # "every frame was looked at", "the budget", "nothing to look at"
    position_steps: int = 0  # steps of the flight controller's position found in the record (the tracks were stepped with each)
    detections: int = 0
    faults: List[str] = field(default_factory=list)

    def summary(self) -> str:
        return (f"second pass: {self.frames_looked_at} of {self.frames_logged} logged frame(s) looked at in {self.elapsed_s:.0f} s ({self.stopped_by}"
                + (f"; every {self.widest_step}th at the widest" if self.widest_step > 1 else "") + f"), {self.detections} box(es)"
                + (f", {self.position_steps} step(s) of the flight controller's position followed" if self.position_steps else "")
                + (f", {self.frames_unreadable} frame(s) unreadable" if self.frames_unreadable else "") + (f", {self.frames_without_pose} without a pose" if self.frames_without_pose else ""))  # fmt: skip


class SecondPass:
    """The replay of one frame log through a pipeline whose pose history and ground height are fed from the flight's record.

    `chain` is a pipeline.LocPipeline (made with tiling on); `poses` and `ground` are the PoseBuffer and the
    GroundHeight its projector reads. `read_image(path)` gives the frame's picture (BGR) or None.
    """

    def __init__(self, chain: Any, poses: Any, ground: Any, read_image: Callable[[str], Any], clock: Callable[[], float] = time.monotonic) -> None:
        from drone_mission_core.estimator_feed import PositionResetDetector

        self.chain, self.poses, self.ground, self._read_image, self._clock = chain, poses, ground, read_image, clock
        self._resets = PositionResetDetector()  # as the node has: a step of the reported position that its velocity does not explain
        self.position_steps: List[Tuple[int, float, float]] = []  # (stamp, east, north) of each step found in the record

    def _feed(self, events: List[Dict[str, Any]], start: int, until_ns: int) -> int:
        index = start
        while index < len(events) and events[index]["t"] <= until_ns:
            event = events[index]
            try:
                if event["k"] == "pose":
                    self.poses.add_pose(event["t"], tuple(event["p"]), tuple(event["q"]))
                    # An EKF reset moves everything seen from here on: the tracks made before it go with it, as they did in the flight.
                    previous_ns = self._resets.last_stamp_ns
                    velocity = None if previous_ns is None else (self.poses.velocity_at(previous_ns) or self.poses.velocity_at(event["t"]))
                    step = self._resets.add(event["t"], (event["p"][0], event["p"][1]), velocity)
                    tracker = getattr(self.chain, "tracker", None)
                    if step is not None:
                        self.position_steps.append((int(event["t"]), float(step[0]), float(step[1])))
                        if tracker is not None:
                            tracker.shift_all(*step)
                elif event["k"] == "vel":
                    self.poses.add_velocity(event["t"], tuple(event["v"]))
                elif self.ground is not None:
                    self.ground.add_range(event["t"], event["r"], event["min"], event["max"])
            except (KeyError, TypeError, ValueError):
                pass  # a line of the record that is not whole
            index += 1
        return index

    def run(self, folder: Any, budget_s: float = BUDGET_S, reserve_s: float = RESERVE_S, mode: str = "") -> PassResult:
        """Look at the folder's logged frames again within the budget. The pipeline's tracker holds what was found."""
        started = self._clock()
        folder = Path(folder)
        frames, events = read_index(folder), read_record(folder)
        result = PassResult(frames_logged=len(frames))
        available = float(budget_s) - float(reserve_s)
        if not frames or available <= 0.0:
            result.stopped_by = "nothing to look at" if not frames else "the budget"
            result.elapsed_s = self._clock() - started
            return result
        fed, index, spent_on_frames = 0, 0, 0.0
        while index < len(frames):
            left = available - (self._clock() - started)
            if left <= 0.0 or (result.frames_looked_at and left < spent_on_frames / result.frames_looked_at):
                result.stopped_by = "the budget"
                break
            record = frames[index]
            frame_started = self._clock()
            fed = self._feed(events, fed, int(record["capture_ns"]) + POSE_AHEAD_NS)
            image = self._read_image(str(folder / record["file"]))
            if image is None:
                result.frames_unreadable += 1
            elif not self.chain.pose_ready(int(record["capture_ns"])):
                result.frames_without_pose += 1
            else:
                try:
                    found = self.chain.process(TopicFrame(image, int(record.get("seq", index)), int(record["capture_ns"]), str(record.get("mode", mode)), "frame log"))
                    result.frames_looked_at += 1
                    result.detections += len(found.detections.detections)
                    if found.fault and found.fault not in result.faults and len(result.faults) < 5:
                        result.faults.append(found.fault)
                except Exception as error:  # noqa: BLE001  (one frame that cannot be handled is said, and the pass goes on)
                    result.frames_unreadable += 1
                    if len(result.faults) < 5:
                        result.faults.append(f"{record['file']}: {type(error).__name__}: {error}")
            spent_on_frames += self._clock() - frame_started
            # How many of the frames left there is time for, at what a frame has cost so far: the rest are stepped over, evenly.
            step = 1
            if result.frames_looked_at:
                per_frame = spent_on_frames / result.frames_looked_at
                left = available - (self._clock() - started)
                remaining = len(frames) - index - 1
                affordable = int(left / per_frame) if per_frame > 0.0 else remaining
                if remaining > 0 and affordable < remaining:
                    step = max(1, math.ceil(remaining / max(affordable, 1)))
            result.widest_step = max(result.widest_step, step)
            index += step
        else:
            result.stopped_by = "every frame was looked at" if result.widest_step == 1 else "the budget"
        result.elapsed_s, result.position_steps = self._clock() - started, len(self.position_steps)
        return result


# --- the merge -----------------------------------------------------------------------------------------------


def merge(live: Sequence[KeptTrack], second: Sequence[KeptTrack], unique_ids: bool = True, gate_m: float = MERGE_GATE_M) -> List[KeptTrack]:
    """The flight's tracks and the second pass's as one list, digits given out anew over all of them.

    No live track is dropped or moved onto another marker: a live track with no partner stays where it was, with
    the votes it had. Only confirmed second-pass tracks inside the search area take part.
    """
    candidates = [track for track in second if track.state in LISTED and not track.outside]
    pairs = sorted(
        (math.hypot(one.reported_east_m - other.reported_east_m, one.reported_north_m - other.reported_north_m), one_index, other_index)
        for one_index, one in enumerate(live)
        for other_index, other in enumerate(candidates)
        if one.class_id == other.class_id
    )
    partner: Dict[int, int] = {}
    taken = set()
    for distance, one_index, other_index in pairs:
        if distance > gate_m:
            break
        if one_index in partner or other_index in taken:
            continue
        partner[one_index] = other_index
        taken.add(other_index)
    merged: List[KeptTrack] = []
    for one_index, one in enumerate(live):
        if one_index not in partner:
            merged.append(one)
            continue
        other = candidates[partner[one_index]]
        total = max(one.sightings, 0) + max(other.sightings, 0)
        share = 0.5 if total <= 0 else max(other.sightings, 0) / total
        merged.append(replace(
            one,
            state=one.state if one.state in LISTED else other.state,  # a track the flight had only begun is listed if the second pass confirmed it
            reported_east_m=one.reported_east_m + share * (other.reported_east_m - one.reported_east_m),
            reported_north_m=one.reported_north_m + share * (other.reported_north_m - one.reported_north_m),
            sightings=total,
            outside=one.outside and other.outside,
            digit_log_prob=tuple(a + b for a, b in zip(one.digit_log_prob, other.digit_log_prob)),
            vote_weight=one.vote_weight + other.vote_weight,
            vote_readings=one.vote_readings + other.vote_readings,
            source="both",
        ))  # fmt: skip
    next_id = max([track.track_id for track in live] + [0]) + 1
    for other_index, other in enumerate(candidates):
        if other_index not in taken:
            merged.append(replace(other, track_id=next_id, source="second pass"))
            next_id += 1
    # The digits, by the vote's own rule over the summed votes.
    votes = DigitVotes(unique_ids=unique_ids)
    for track in merged:
        if track.vote_weight > 0.0 and track.vote_readings > 0:
            votes.add_sums(track.track_id, track.digit_log_prob, track.vote_weight, track.vote_readings)
    listed = [track.track_id for track in merged if track.class_id == "marker" and track.state in LISTED and not track.outside]
    digits = votes.digits(only=listed)
    return [replace(track, digit=int(digits.get(track.track_id, -1))) if track.track_id in listed else track for track in merged]


def changes(live: Sequence[KeptTrack], merged: Sequence[KeptTrack]) -> List[str]:
    """What the second pass changed, a line each: for the log, and for the person who decides which list is handed in."""
    before = {track.track_id: track for track in live}
    lines = []
    for track in merged:
        if track.class_id != "marker":
            continue
        was = before.get(track.track_id)
        if was is None:
            lines.append(f"new: a marker at ({track.reported_east_m:.1f}, {track.reported_north_m:.1f}) m, {'digit ' + str(track.digit) if track.digit >= 0 else 'not read'}, seen {track.sightings} time(s) in the second pass")
            continue
        moved = math.hypot(track.reported_east_m - was.reported_east_m, track.reported_north_m - was.reported_north_m)
        if track.digit != was.digit:
            lines.append(f"track {track.track_id}: digit {was.digit if was.digit >= 0 else 'not read'} -> {track.digit if track.digit >= 0 else 'not read'}")
        if moved > 0.25:
            lines.append(f"track {track.track_id}: moved {moved:.2f} m")
    return lines
