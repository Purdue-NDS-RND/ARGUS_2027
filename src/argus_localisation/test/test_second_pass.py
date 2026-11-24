"""LOC-12: the post-landing second pass: the flight's record, the replay within its budget, and the merge with the live tracks."""

import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "argus_localisation"), str(SRC / "drone_mission_core"), str(SRC / "argus_camera")]

from argus_localisation import marker_tracker, pipeline, second_pass as sp  # noqa: E402
from argus_localisation.digit_vote import DigitVotes  # noqa: E402
from drone_mission_core import detection_contract as contract  # noqa: E402
from drone_mission_core import projection_v2  # noqa: E402
from drone_mission_core.frame_projection import load_camera  # noqa: E402
from drone_mission_core.pose_buffer import PoseBuffer  # noqa: E402

S = 1_000_000_000
LEVEL = (0.0, 0.0, 0.0, 1.0)


def track(track_id, east, north, sightings=10, digit=-1, votes=None, weight=0.0, readings=0, state=marker_tracker.CONFIRMED, class_id="marker", outside=False, source="live"):
    sums = (0.0,) * 10 if votes is None else tuple(votes)
    return sp.KeptTrack(track_id, class_id, state, east, north, sightings, outside, 0.0, digit, sums, weight, readings, source)


def reading(digit, sure=0.9, weight=1.0):
    """A digit vote's sums for readings that say `digit` with probability `sure`, of total weight `weight`."""
    other = (1.0 - sure) / 9.0
    return tuple(weight * math.log(sure if index == digit else other) for index in range(10))


# --- the record and the kept tracks --------------------------------------------------------------------------


def test_the_flights_record_comes_back_in_the_order_of_its_stamps_and_a_cut_line_is_left_out(tmp_path):
    record = sp.FlightRecord(tmp_path, flush_every=2)
    record.pose(3 * S, (1.0, 2.0, 30.0), LEVEL)
    record.range(2 * S, 29.5, 0.2, 50.0)
    record.velocity(1 * S, (3.0, 0.0, 0.0))
    done = []
    record.close(then=lambda: done.append(record.lines), wait_s=5.0)  # the file is written and closed on the record's own thread, then this is run there
    assert done == [3] and record.failed == ""
    record.pose(9 * S, (0.0, 0.0, 0.0), LEVEL)  # after the close: nothing, and no error
    # What is done at the end and fails is said, not raised into whoever closed it.
    other = sp.FlightRecord(tmp_path / "..", flush_every=1) if False else None
    failing = sp.FlightRecord(tmp_path)
    failing.close(then=lambda: (_ for _ in ()).throw(OSError("the disk is full")), wait_s=5.0)
    assert failing.failed == "OSError: the disk is full" and other is None
    with open(tmp_path / sp.RECORD_NAME, "a") as handle:
        handle.write('{"k":"pose","t":4000000000,"p":[1,2')  # the power went
    events = sp.read_record(tmp_path)
    assert [(event["k"], event["t"]) for event in events] == [("vel", 1 * S), ("range", 2 * S), ("pose", 3 * S)] and record.lines == 3
    assert events[2]["p"] == [1.0, 2.0, 30.0] and events[1] == {"k": "range", "t": 2 * S, "r": 29.5, "min": 0.2, "max": 50.0}
    with pytest.raises(sp.SecondPassError, match="the survey's poses were not recorded"):
        sp.read_record(tmp_path / "nowhere")
    with pytest.raises(sp.SecondPassError, match="not a frame log's folder"):
        sp.read_index(tmp_path)


def test_the_flights_tracks_are_kept_with_their_votes_and_read_back_whole(tmp_path):
    tracker = marker_tracker.MarkerTracker()
    votes = DigitVotes()
    tracker.digit_votes = votes
    states = [marker_tracker.MarkerTrackState(4, "marker", marker_tracker.CONFIRMED, 10.0, 5.0, 10.2, 5.1, np.eye(2), 12, False, 7 * S, 0.3, 7, reading(7, weight=3.0))]
    votes.add_sums(4, reading(7, weight=3.0), 3.0, 5)
    kept = sp.kept(states, votes)
    assert kept == [sp.KeptTrack(4, "marker", marker_tracker.CONFIRMED, 10.2, 5.1, 12, False, 0.3, 7, reading(7, weight=3.0), 3.0, 5, "live")]
    sp.save_live_tracks(tmp_path / sp.LIVE_TRACKS_NAME, kept, (40.42, -86.92), {"ground_below_home_m": -1.5, "digit_ids_unique": False})
    back, origin, settings = sp.load_live_tracks(tmp_path / sp.LIVE_TRACKS_NAME)
    assert back == kept and origin == (40.42, -86.92)
    # The settings of the flight's pipeline go with the tracks: what was given, and the node's defaults for the rest.
    assert settings == {"ground_below_home_m": -1.5, "digit_min_confidence": 0.9, "median_positions": False, "digit_ids_unique": False}
    with pytest.raises(ValueError, match="not settings of the pipeline"):
        sp.save_live_tracks(tmp_path / "x.json", kept, None, {"tiling": True})
    (tmp_path / "broken.json").write_text('{"tracks": [{"track_id": 1}]}')
    with pytest.raises(sp.SecondPassError, match="is not the flight's tracks"):
        sp.load_live_tracks(tmp_path / "broken.json")


# --- the replay's budget -------------------------------------------------------------------------------------


class CountingChain:
    """A pipeline that only counts: which frames it was given."""

    def __init__(self, no_pose=()):
        self.seen, self.no_pose = [], set(no_pose)

    def pose_ready(self, stamp_ns):
        return stamp_ns not in self.no_pose

    def process(self, frame):
        self.seen.append(frame.seq)
        return pipeline.FrameResult(frame.capture_ns, frame.seq, contract.DetectionSet(frame.capture_ns, "f", (contract.Detection("marker", 0.9, 10.0, 10.0, 8.0, 8.0),)))


def log_of(folder, frames):
    folder.mkdir(parents=True, exist_ok=True)
    (folder / sp.INDEX_NAME).write_text("".join(json.dumps({"seq": index, "capture_ns": (100 + index) * S // 10, "mode": "m", "file": f"frame_{index:06d}.jpg", "width": 4, "height": 4}) + "\n" for index in range(frames)))
    sp.FlightRecord(folder).close(wait_s=5.0)
    return folder


def timed_pass(chain, seconds_a_frame, unreadable=()):
    clock = [0.0]

    def read(path):
        clock[0] += seconds_a_frame
        return None if int(Path(path).stem.split("_")[1]) in unreadable else np.zeros((4, 4, 3), dtype=np.uint8)

    return sp.SecondPass(chain, PoseBuffer(), None, read, clock=lambda: clock[0]), clock


def test_with_time_for_every_frame_every_frame_is_looked_at_in_the_order_it_was_taken(tmp_path):
    chain = CountingChain()
    run, clock = timed_pass(chain, 2.0)
    result = run.run(log_of(tmp_path / "log", 100), budget_s=900.0)
    assert chain.seen == list(range(100)) and result.frames_looked_at == result.frames_logged == 100 and result.stopped_by == "every frame was looked at"
    assert result.elapsed_s == pytest.approx(200.0) and result.widest_step == 1 and result.detections == 100
    assert result.summary() == "second pass: 100 of 100 logged frame(s) looked at in 200 s (every frame was looked at), 100 box(es)"


def test_with_too_little_time_fewer_frames_are_looked_at_evenly_to_the_end_and_the_budget_is_kept(tmp_path):
    """The spec's 900 s: 3000 frames at 2 s each is 6000 s of work."""
    chain = CountingChain()
    run, clock = timed_pass(chain, 2.0)
    result = run.run(log_of(tmp_path / "log", 3000), budget_s=sp.BUDGET_S)
    assert clock[0] <= sp.BUDGET_S - sp.RESERVE_S + 2.0 and result.stopped_by == "the budget"  # the reserve for the merge and the list is left
    assert 380 <= result.frames_looked_at <= 428 and chain.seen == sorted(chain.seen)  # (855 s at 2 s a frame is 427)
    gaps = [later - earlier for earlier, later in zip(chain.seen, chain.seen[1:])]
    assert chain.seen[0] == 0 and chain.seen[-1] >= 2950 and max(gaps[5:]) <= 9 and result.widest_step >= 7  # spread over the whole survey, not its first seventh
    assert "every" in result.summary() and "th at the widest" in result.summary()
    # A frame that costs more than what is left is not begun.
    slow_chain = CountingChain()
    slow, slow_clock = timed_pass(slow_chain, 400.0)
    slow_result = slow.run(log_of(tmp_path / "slow", 10), budget_s=sp.BUDGET_S)
    assert slow_result.frames_looked_at == 2 and slow_clock[0] == 800.0 and slow_result.stopped_by == "the budget"
    none = sp.SecondPass(CountingChain(), PoseBuffer(), None, lambda path: None).run(log_of(tmp_path / "none", 0))
    assert none.stopped_by == "nothing to look at" and none.frames_looked_at == 0


def test_a_frame_that_cannot_be_read_or_has_no_pose_is_counted_and_the_pass_goes_on(tmp_path):
    chain = CountingChain(no_pose={(100 + 3) * S // 10})
    run, _clock = timed_pass(chain, 1.0, unreadable={1, 5})
    result = run.run(log_of(tmp_path / "log", 8), budget_s=900.0)
    assert chain.seen == [0, 2, 4, 6, 7] and result.frames_unreadable == 2 and result.frames_without_pose == 1 and result.frames_looked_at == 5
    assert "2 frame(s) unreadable" in result.summary() and "1 without a pose" in result.summary()

    class Faulty(CountingChain):
        def process(self, frame):
            if frame.seq == 2:
                raise ValueError("a frame of the wrong size")
            return super().process(frame)

    faulty = Faulty()
    run, _clock = timed_pass(faulty, 1.0)
    result = run.run(log_of(tmp_path / "faulty", 4), budget_s=900.0)
    assert faulty.seen == [0, 1, 3] and result.faults == ["frame_000002.jpg: ValueError: a frame of the wrong size"]


# --- the replay through the real pipeline --------------------------------------------------------------------

MARKERS = {"a": (4.0, 0.5), "b": (12.0, -1.0), "c": (20.0, 1.5)}  # east, north of three white plates on dark ground


class PlateDetector:
    """A detector for the test's pictures: every bright blob is a marker. It looks at the pixels it is given, whole frame or tile."""

    name = "plates"

    def detect(self, image, stamp_ns):
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        _count, _labels, stats, centres = cv2.connectedComponentsWithStats((grey > 128).astype(np.uint8))
        return [contract.Detection("marker", 0.9, float(cx), float(cy), float(w), float(h)) for (x, y, w, h, area), (cx, cy) in zip(stats[1:], centres[1:]) if area >= 9]


def survey(folder, camera, mount, frames=40, height=10.0, shift=(0.0, 0.0), suffix=".png", mode="sim_delivery", step_at_tick=None, step=(0.0, 0.0)):
    """A straight flight east over the plates, logged as the node logs one: frames with their index, and the record of poses and ranges.

    With `step_at_tick`, the flight controller's reported position steps by `step` from that tick on (an EKF reset): the
    camera goes on seeing the ground from where the vehicle truly is."""
    folder.mkdir(parents=True, exist_ok=True)
    record = sp.FlightRecord(folder)
    index_lines = []
    start = 1000 * S
    for tick in range(frames * 6 + 12):  # poses at 30 Hz, a range every other one, a frame every sixth
        stamp = start + tick * S // 30
        position = (-2.0 + 0.1 * tick + shift[0], shift[1], height)  # where the vehicle truly is
        stepped = step_at_tick is not None and tick >= step_at_tick
        record.pose(stamp, (position[0] + (step[0] if stepped else 0.0), position[1] + (step[1] if stepped else 0.0), height), LEVEL)
        record.velocity(stamp, (3.0, 0.0, 0.0))
        if tick % 2 == 0:
            record.range(stamp, height, 0.2, 60.0)
        if tick % 6 == 3 and len(index_lines) < frames:
            picture = np.full((camera.height, camera.width, 3), 20, dtype=np.uint8)
            for east, north in MARKERS.values():
                u, v = projection_v2.pixel_of_ground_point(camera, mount, (east, north, 0.0), position, LEVEL)
                side = 0.6 * camera.fx / height
                cv2.rectangle(picture, (int(round(u - side / 2)), int(round(v - side / 2))), (int(round(u + side / 2)), int(round(v + side / 2))), (255, 255, 255), -1)
            name = f"frame_{len(index_lines):06d}{suffix}"
            cv2.imwrite(str(folder / name), picture)
            index_lines.append(json.dumps({"seq": len(index_lines), "capture_ns": stamp, "mode": mode, "file": name, "width": camera.width, "height": camera.height}))
    record.close(wait_s=10.0)
    (folder / sp.INDEX_NAME).write_text("\n".join(index_lines) + "\n")
    return folder


def real_chain(camera, mount):
    poses = PoseBuffer()
    ground = projection_v2.GroundHeight(poses.pose_at)
    tracker = marker_tracker.MarkerTracker()
    projector = projection_v2.GroundProjector(camera, mount, poses.pose_at, pipeline.SiteGround(ground, 0.0))
    chain = pipeline.LocPipeline(PlateDetector(), projector, "down_camera_optical", (camera.width, camera.height), tracker=tracker, tiling=True, pose_at=poses.pose_at)
    return chain, poses, ground, tracker


def test_the_logged_frames_go_through_the_real_pipeline_again_and_the_plates_are_where_they_lie(tmp_path):
    camera, mount = load_camera("sim_delivery", "sim_iris_downcam")
    folder = survey(tmp_path / "log", camera, mount)
    chain, poses, ground, tracker = real_chain(camera, mount)
    result = sp.SecondPass(chain, poses, ground, cv2.imread).run(folder)
    assert result.frames_looked_at == result.frames_logged == 40 and result.frames_without_pose == 0 and result.faults == [] and result.detections > 40
    found = sp.kept(tracker.tracks(), source="second pass")
    listed = [item for item in found if item.state in sp.LISTED]
    assert len(listed) == 3
    for east, north in MARKERS.values():
        nearest = min(listed, key=lambda item: math.hypot(item.reported_east_m - east, item.reported_north_m - north))
        assert math.hypot(nearest.reported_east_m - east, nearest.reported_north_m - north) < 0.15 and nearest.sightings >= 5
    # Without the record the frames cannot be placed, and the pass says so before it begins.
    (folder / sp.RECORD_NAME).rename(folder / "record.moved")
    with pytest.raises(sp.SecondPassError, match="the survey's poses were not recorded"):
        sp.SecondPass(*real_chain(camera, mount)[:3], cv2.imread).run(folder)


def test_a_step_of_the_flight_controllers_position_in_the_record_moves_the_tracks_made_before_it():
    """The spec audit and the flight-safety review: the node steps its tracks with an EKF reset, and the replay did not, so the frames
    before and after one were merged as two places."""
    import tempfile

    camera, mount = load_camera("sim_delivery", "sim_iris_downcam")
    with tempfile.TemporaryDirectory() as scratch:
        folder = survey(Path(scratch) / "log", camera, mount, step_at_tick=120, step=(3.0, -2.0))  # halfway through the survey
        chain, poses, ground, tracker = real_chain(camera, mount)
        run = sp.SecondPass(chain, poses, ground, cv2.imread)
        result = run.run(folder)
    assert result.position_steps == 1 and len(run.position_steps) == 1 and "1 step(s) of the flight controller's position followed" in result.summary()
    assert run.position_steps[0][1:] == pytest.approx((3.0, -2.0), abs=0.15)
    listed = [item for item in sp.kept(tracker.tracks(), source="second pass") if item.state in sp.LISTED]
    assert len(listed) == 3  # three plates, not five: the ones seen before the step are where the frame after it puts them
    for east, north in MARKERS.values():
        nearest = min(listed, key=lambda item: math.hypot(item.reported_east_m - (east + 3.0), item.reported_north_m - (north - 2.0)))
        assert math.hypot(nearest.reported_east_m - (east + 3.0), nearest.reported_north_m - (north - 2.0)) < 0.35


# --- the merge -----------------------------------------------------------------------------------------------


def test_a_second_pass_track_on_a_live_one_is_the_same_marker_and_their_sightings_and_votes_are_summed():
    live = [track(1, 10.0, 5.0, sightings=10, votes=reading(6, sure=0.5, weight=1.0), weight=1.0, readings=2)]  # the flight read "6, perhaps": not enough for a digit
    second = [track(7, 10.6, 5.0, sightings=30, votes=reading(6, sure=0.9, weight=4.0), weight=4.0, readings=8, source="second pass")]
    merged = sp.merge(live, second)
    assert len(merged) == 1 and merged[0].track_id == 1 and merged[0].source == "both" and merged[0].sightings == 40
    assert merged[0].reported_east_m == pytest.approx(10.45) and merged[0].reported_north_m == 5.0  # the mean by sightings: 30 of 40 from the second pass
    assert merged[0].digit == 6 and merged[0].vote_weight == 5.0 and merged[0].vote_readings == 10
    assert sp.changes(live, merged) == ["track 1: digit not read -> 6", "track 1: moved 0.45 m"]


def test_no_live_track_is_dropped_and_only_confirmed_second_pass_tracks_inside_the_area_are_added():
    live = [track(1, 0.0, 0.0, digit=3, votes=reading(3, weight=3.0), weight=3.0, readings=4), track(2, 30.0, 0.0), track(3, 60.0, 0.0, class_id="boundary_marker")]
    second = [
        track(1, 0.2, 0.1, sightings=20, votes=reading(3, weight=2.0), weight=2.0, readings=3),
        track(2, 31.8, 0.0, sightings=20),  # 1.8 m from live track 2: over the gate, another marker
        track(3, 90.0, 0.0, sightings=2, state=marker_tracker.TENTATIVE),  # not confirmed: not added
        track(4, 95.0, 0.0, sightings=9, outside=True),  # outside the search area: not added
        track(5, 60.3, 0.0, sightings=5, class_id="marker"),  # beside the boundary marker, and another class: its own marker
    ]
    merged = sp.merge(live, second)
    by_id = {item.track_id: item for item in merged}
    assert sorted(by_id) == [1, 2, 3, 4, 5] and [by_id[i].source for i in (1, 2, 3, 4, 5)] == ["both", "live", "live", "second pass", "second pass"]
    assert by_id[2] == live[1] and by_id[3] == live[2]  # untouched
    assert (by_id[4].reported_east_m, by_id[5].reported_east_m) == (31.8, 60.3) and by_id[1].digit == 3
    assert sp.changes(live, merged) == ["new: a marker at (31.8, 0.0) m, not read, seen 20 time(s) in the second pass", "new: a marker at (60.3, 0.0) m, not read, seen 5 time(s) in the second pass"]
    # With nothing from the second pass the list is the flight's.
    assert sp.merge(live, []) == [live[0], live[1], live[2]]


def test_the_digits_are_given_out_anew_over_both_passes_one_digit_to_one_marker():
    # The flight gave track 1 the digit 8 on thin evidence; the second pass reads another marker as 8 far more surely, and track 1 as 3.
    live = [track(1, 0.0, 0.0, digit=8, votes=reading(8, sure=0.6, weight=1.5), weight=1.5, readings=2), track(2, 40.0, 0.0)]
    second = [track(1, 0.1, 0.0, votes=reading(3, sure=0.9, weight=6.0), weight=6.0, readings=9), track(2, 40.2, 0.0, votes=reading(8, sure=0.95, weight=6.0), weight=6.0, readings=9)]
    merged = {item.track_id: item for item in sp.merge(live, second)}
    assert merged[1].digit == 3 and merged[2].digit == 8
    # Two markers with the same claim to one digit are both left unread, not split by a coin (the vote's own rule).
    same = [track(1, 0.0, 0.0, votes=reading(5, weight=4.0), weight=4.0, readings=5), track(2, 40.0, 0.0, votes=reading(5, weight=4.0), weight=4.0, readings=5)]
    assert [item.digit for item in sp.merge(same, [])] == [-1, -1]
    assert [item.digit for item in sp.merge(same, [], unique_ids=False)] == [5, 5]
