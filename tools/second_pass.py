#!/usr/bin/env python3
"""The post-landing second pass (LOC-12): look again at a survey's logged frames, and merge what is found with the flight's tracks.

  python3 tools/second_pass.py <the survey's frame log folder> --camera-mode imx519_4k --airframe argus_2027 \\
      --engine <the detector's engine> [--engine-classes marker boundary_marker] [--digit-model <the digit reader's model>] \\
      --out <the judges' list's folder> [--budget-s 900]

Run on the aircraft's computer after landing, with the localisation node
stopped (the detector's engine is the one it used). The folder is a survey's
frame log (LOC-11) made with the node flag `features.post_landing_pass` on:
beside its frames it holds the survey's poses and ranges
(`flight_record.jsonl`) and the tracks as the flight left them, with the
settings of the flight's own pipeline that the pass uses as well
(`live_tracks.json`).

Every logged frame is given to the same pipeline the flight used, tiled at
full resolution, in the order the frames were taken; within the budget (900
s unless told otherwise) fewer frames are looked at, evenly, rather than the
budget being broken. What is found is merged with the flight's tracks
(second_pass.merge: no marker the flight listed is dropped), and the merged
list is written to `--out` as the judges' list is (CSV, KML, map), the list
that was there kept under another name. What changed is printed, a line a
marker.

Whether a list may hold results worked out after landing is the organisers'
to rule on (question 4). Until they do, the list to hand in is the flight's
own: write the second pass's list to a folder of its own and compare the two
with tools/judges_list_view.py --against.

  python3 tools/second_pass.py <folder> --camera-mode ... --airframe ... --time-only

`--time-only` reads and tiles every logged frame with a detector that finds
nothing, and says what that costs on this computer and how many of the
folder's frames a 900 s pass has time for at a given cost of the detector on
one tile (`--tile-ms`, 25 unless told otherwise): for planning, before there
is an engine to run. It needs the frames, the index and the record, and
writes nothing.

Exit code 0: the pass ran and a list was written. 1: it ran and no list
could be written. 2: it could not run (the message says why).

No ROS imports.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any, List, Optional

REPO = Path(__file__).resolve().parents[1]
for package in ("argus_localisation", "drone_mission_core", "argus_camera"):
    path = str(REPO / "src" / package)
    if path not in sys.path:
        sys.path.insert(0, path)


def build_chain(camera: Any, mount: Any, detector: Any, digit_classifier: Any = None, unique_ids: bool = True, median_positions: bool = False, min_confidence: float = 0.9,
                ground_below_home_m: float = 0.0):  # fmt: skip
    """(the pipeline, its pose history, its ground height, its tracker, its votes): the flight's own chain, tiled, with nothing of the flight in it yet.
    The four settings are the flight's own, kept beside its tracks (second_pass.SETTINGS)."""
    from argus_localisation import digit_reader, digit_vote, marker_tracker, pipeline
    from drone_mission_core.pose_buffer import PoseBuffer
    from drone_mission_core.projection_v2 import GroundHeight, GroundProjector

    poses = PoseBuffer()
    ground = GroundHeight(poses.pose_at)
    tracker = marker_tracker.MarkerTracker(median_positions=median_positions)
    reader, votes = None, None
    if digit_classifier is not None:
        reader = digit_reader.DigitReader(digit_classifier, camera=camera, min_confidence=min_confidence)
        votes = digit_vote.DigitVotes(unique_ids=unique_ids)
        tracker.digit_votes = votes
    projector = GroundProjector(camera, mount, poses.pose_at, pipeline.SiteGround(ground, ground_below_home_m))
    frame_id = str(getattr(mount, "optical_frame", "") or "down_camera_optical")
    chain = pipeline.LocPipeline(detector, projector, frame_id, (int(camera.width), int(camera.height)), tracker=tracker, digit_reader=reader, digit_votes=votes, tiling=True, pose_at=poses.pose_at)
    return chain, poses, ground, tracker, votes


class NoDetector:
    """For --time-only: looks at nothing and counts the pictures it was handed (a frame's tiles)."""

    name = "none (timing only)"

    def __init__(self) -> None:
        self.tiles = 0

    def detect(self, image: Any, stamp_ns: int) -> list:
        self.tiles += 1
        return []


def time_only(folder: Path, camera: Any, mount: Any, budget_s: float, tile_ms: float, clock) -> int:
    import cv2
    from argus_localisation import second_pass as sp

    counting = NoDetector()
    chain, poses, ground, _tracker, _votes = build_chain(camera, mount, counting)
    result = sp.SecondPass(chain, poses, ground, cv2.imread, clock=clock).run(folder, budget_s=budget_s)
    print(result.summary())
    if not result.frames_looked_at:
        print("no frame could be looked at: nothing to time")
        return 1
    per_frame_s = result.elapsed_s / result.frames_looked_at
    tiles = counting.tiles / result.frames_looked_at
    with_detector_s = per_frame_s + tiles * tile_ms / 1000.0
    fits = int((sp.BUDGET_S - sp.RESERVE_S) / with_detector_s)
    print(f"reading and tiling a frame: {1000.0 * per_frame_s:.0f} ms on this computer, {tiles:.0f} tile(s) a frame ({camera.width} x {camera.height})")
    print(f"with a detector at {tile_ms:g} ms a tile: {with_detector_s:.2f} s a frame, so {fits} frame(s) fit in the {sp.BUDGET_S:g} s"
          + (f": all {result.frames_logged} of this survey" if fits >= result.frames_logged else f": every {-(-result.frames_logged // max(fits, 1))}th of this survey's {result.frames_logged}"))  # fmt: skip
    return 0


def main(argv: Optional[List[str]] = None, detector: Any = None, digit_classifier: Any = None, clock=time.monotonic) -> int:
    parser = argparse.ArgumentParser(description="The post-landing second pass over a survey's logged frames (LOC-12).")
    parser.add_argument("folder", help="the survey's frame log folder (frames, index.jsonl, flight_record.jsonl, live_tracks.json)")
    parser.add_argument("--camera-mode", required=True, help="the sensor mode the frames were taken in (argus_camera modes.yaml)")
    parser.add_argument("--airframe", required=True, help="the airframe whose camera mount is used (argus_camera robot.yaml)")
    parser.add_argument("--engine", help="the detector's TensorRT engine, as the flight used it")
    parser.add_argument("--engine-classes", nargs="+", default=["marker"], help="the engine's classes, in its order")
    parser.add_argument("--digit-model", help="the digit reader's model (ONNX); without it no digit is read in the second pass")
    parser.add_argument("--out", help="the folder the merged judges' list is written to")
    parser.add_argument("--time-only", action="store_true", help="time the reading and tiling of the logged frames with a detector that finds nothing; writes nothing")
    parser.add_argument("--tile-ms", type=float, default=25.0, help="for --time-only: what the detector is taken to cost on one tile, in milliseconds")
    parser.add_argument("--budget-s", type=float, default=None, help="seconds the pass may take in all (default 900)")
    args = parser.parse_args(argv)
    started = clock()
    try:
        import cv2
        from argus_localisation import judges_list, pipeline
        from argus_localisation import second_pass as sp
        from drone_mission_core.frame_projection import load_camera

        budget = sp.BUDGET_S if args.budget_s is None else float(args.budget_s)
        if not 0.0 < budget <= sp.BUDGET_S:
            raise sp.SecondPassError(f"--budget-s must be over 0 and no more than {sp.BUDGET_S:g}: the list is due 30 minutes after the slot")
        folder = Path(args.folder)
        camera, mount = load_camera(args.camera_mode, args.airframe)
        if args.time_only:
            if not args.tile_ms > 0.0:
                raise sp.SecondPassError("--tile-ms must be over 0")
            return time_only(folder, camera, mount, budget, args.tile_ms, clock)
        if not args.out:
            raise sp.SecondPassError("--out is needed: the folder the merged list is written to")
        live, origin, settings = sp.load_live_tracks(folder / sp.LIVE_TRACKS_NAME)  # with the settings of the flight's own pipeline
        if detector is None:
            if not args.engine:
                raise sp.SecondPassError("--engine is needed: the detector's engine, as the flight used it")
            from argus_localisation import engine

            detector = engine.load_engine(args.engine, list(args.engine_classes), pipeline.MIN_SCORE)
        if digit_classifier is None and args.digit_model:
            from argus_localisation import digit_reader

            digit_classifier = digit_reader.load_onnx(args.digit_model)
        chain, poses, ground, tracker, votes = build_chain(camera, mount, detector, digit_classifier, unique_ids=settings["digit_ids_unique"], median_positions=settings["median_positions"],
                                                           min_confidence=settings["digit_min_confidence"], ground_below_home_m=settings["ground_below_home_m"])  # fmt: skip
        out = judges_list.JudgesList(args.out)
        if not out.folder.parent.is_dir():
            raise sp.SecondPassError(f"--out is {out.folder}, and {out.folder.parent} is not a folder that exists")
        result = sp.SecondPass(chain, poses, ground, cv2.imread, clock=clock).run(folder, budget_s=budget - (clock() - started), mode=args.camera_mode)
    except Exception as problem:  # noqa: BLE001  (whatever stops the pass before it has a result is said, and the flight's list stands)
        print(f"second_pass: {type(problem).__name__}: {problem}", file=sys.stderr)
        print("The flight's own list stands.", file=sys.stderr)
        return 2
    print(result.summary())
    for fault in result.faults:
        print(f"  fault: {fault}")
    if votes is not None:
        votes.settle()
    merged = sp.merge(live, sp.kept(tracker.tracks(), votes, source="second pass"), unique_ids=settings["digit_ids_unique"])
    rows, unread = judges_list.rows_from_tracks(merged, origin)
    changed = sp.changes(live, merged)
    print(f"merged with the flight's {len([t for t in live if t.class_id == 'marker'])} marker track(s): {len(rows)} row(s) in the list, {len(unread)} marker(s) found and not read" + ("" if origin is not None else " (the flight left no origin for the local frame: no row can be given a latitude and longitude)"))
    for line in changed or ["nothing changed: the list is the flight's"]:
        print(f"  {line}")
    written = out.write(rows, unread, judges_list.boundary_of(merged))
    took = clock() - started
    if not written:
        print(f"second_pass: the list could not be written to {out.folder} ({out.error}). The flight's own list stands.", file=sys.stderr)
        return 1
    print(f"written: {out.csv_path} ({took:.0f} s in all). Compare with the flight's list before handing either in: tools/judges_list_view.py {out.csv_path} --against <the flight's judges_list.csv>")
    return 0


if __name__ == "__main__":
    sys.exit(main())
