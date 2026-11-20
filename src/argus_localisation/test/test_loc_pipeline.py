"""The localisation pipeline's ROS-free parts (LOC-5): the shrink, the tiles, the sim's oracle detector, one
frame through the pipeline, and the newest-frame runtime."""

import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "argus_localisation"), str(SRC / "drone_mission_core"), str(SRC / "argus_camera")]

from argus_localisation import engine, oracle, pipeline, runtime, shrink, tiling  # noqa: E402
from drone_mission_core import detection_contract as contract  # noqa: E402
from drone_mission_core.pose_buffer import PoseBuffer  # noqa: E402
from drone_mission_core.projection_v2 import CameraModel, CameraMount, GroundProjector  # noqa: E402

S = 1_000_000_000
FULL = (3840, 2160)
CAMERA = CameraModel(width=3840, height=2160, fx=2334.0, fy=2334.0, cx=1920.0, cy=1080.0, name="test_4k")
MOUNT = CameraMount(position_m=(0.0, 0.0, 0.0), rotation_body_from_optical=((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0)))
LEVEL = (0.0, 0.0, 0.0, 1.0)  # facing east


# --- the shrink ----------------------------------------------------------------------


def test_a_4k_frame_is_shrunk_to_1280_by_720_by_exactly_a_third():
    assert shrink.DETECTOR_SIZE == (1280, 720) and shrink.ENGINE_SIZE == (1280, 736)
    frame = np.zeros((2160, 3840, 3), dtype=np.uint8)
    frame[900:1260, 1500:2340] = (0, 0, 255)
    small = shrink.shrink_for_detector(frame)
    assert small.shape == (720, 1280, 3) and shrink.label_scale(FULL) == pytest.approx(1.0 / 3.0)
    assert tuple(small[360, 640]) == (0, 0, 255) and tuple(small[100, 100]) == (0, 0, 0)
    assert tuple(small[300, 500]) == (0, 0, 255) and tuple(small[299, 499]) == (0, 0, 0)  # the block's corner, to the pixel
    # A frame already that size is the same array, not a copy; a shape that is not 16:9 is refused.
    assert shrink.shrink_for_detector(small) is small
    with pytest.raises(ValueError, match="16:9"):
        shrink.shrink_for_detector(np.zeros((1000, 1280, 3), dtype=np.uint8))


def test_the_engines_frame_is_the_shrunk_one_with_eight_rows_above_and_below_and_boxes_come_back_out():
    small = np.full((720, 1280, 3), 200, dtype=np.uint8)
    padded = shrink.pad_for_engine(small)
    assert padded.shape == (736, 1280, 3) and shrink.PAD_ROWS == 8
    assert (padded[:8] == 114).all() and (padded[-8:] == 114).all() and (padded[8:728] == 200).all()
    found = contract.Detection("marker", 0.9, 640.0, 368.0, 30.0, 30.0)  # the padded frame's centre
    assert shrink.from_engine(found) == contract.Detection("marker", 0.9, 640.0, 360.0, 30.0, 30.0)
    back = shrink.to_full_resolution(shrink.from_engine(found), FULL)
    assert (back.cx, back.cy, back.width, back.height) == pytest.approx((1920.0, 1080.0, 90.0, 90.0))


# --- tiles ---------------------------------------------------------------------------


def test_tiles_cover_the_frame_with_overlap_and_their_boxes_are_merged():
    windows = tiling.tile_windows(FULL, (1280, 720), overlap=0.2)
    assert len(windows) == 16 and windows[0] == (0, 0, 1280, 720) and windows[-1] == (2560, 1440, 1280, 720)
    covered = np.zeros((2160, 3840), dtype=bool)
    for x, y, w, h in windows:
        assert 0 <= x and x + w <= 3840 and 0 <= y and y + h <= 2160
        covered[y : y + h, x : x + w] = True
    assert covered.all()
    # A marker in the overlap of two tiles is found twice: the better one stays.
    in_first = tiling.from_tile(contract.Detection("marker", 0.7, 1100.0, 400.0, 60.0, 60.0), windows[0])
    in_second = tiling.from_tile(contract.Detection("marker", 0.9, 1100.0 - windows[1][0] + 2.0, 400.0, 60.0, 60.0), windows[1])
    other = contract.Detection("marker", 0.6, 3000.0, 1500.0, 60.0, 60.0)
    merged = tiling.merge([in_first, in_second, other])
    assert len(merged) == 2 and merged[0].score == 0.9 and merged[0].cx == pytest.approx(1102.0) and merged[1] == other
    # Boxes of different classes over each other are both kept.
    assert len(tiling.merge([in_first, contract.Detection("bucket", 0.5, 1100.0, 400.0, 60.0, 60.0)])) == 2


# --- the oracle detector (sim only) ------------------------------------------------------


def truth(vehicle=(0.0, 0.0, 10.0), marker=(2.0, 1.0, 0.02), orientation=LEVEL):
    return {"iris": (*vehicle, *orientation), "marker_3": (*marker, 0.0, 0.0, 0.0, 1.0), "parcel": (0.5, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0)}


OBJECTS = {"vehicle": "iris", "objects": [{"name": "marker_3", "class_id": "marker", "size_m": 0.6, "top_m": 0.0}, {"name": "corner", "class_id": "boundary_marker", "size_m": 1.2, "top_m": 0.0}]}


def exact_oracle(**settings):
    world = oracle.TruthHistory("iris")
    return oracle.OracleDetector(CAMERA, MOUNT, oracle.objects_from(OBJECTS), world, seed=1, **{"pixel_sigma_px": 0.0, "miss_rate": 0.0, "false_alarms_per_frame": 0.0, **settings}), world


def test_the_oracle_refuses_to_be_anything_but_a_sim_detector():
    oracle.check_sim_only(use_sim_time=True, sim_only=True)
    for use_sim_time, sim_only in ((False, True), (True, False), (False, False)):
        with pytest.raises(oracle.OracleRefused, match="sim"):
            oracle.check_sim_only(use_sim_time=use_sim_time, sim_only=sim_only)
    assert oracle.FLAG == "sim_oracle_detector" and oracle.TRUTH_TOPIC == "/sim/truth/poses"


def test_the_oracle_reports_the_true_objects_where_the_camera_sees_them():
    detector, world = exact_oracle()
    world.add(10 * S, truth())
    found = detector.detect_full(10 * S)
    assert [d.class_id for d in found] == ["marker"]  # the parcel is not an object of the contract; the corner has no pose
    marker = found[0]
    # Facing east, level, 9.98 m above the marker: 2 m ahead is up the image, 1 m to the left is left in it.
    assert marker.cx == pytest.approx(1920.0 - 2334.0 * 1.0 / 9.98, abs=0.5) and marker.cy == pytest.approx(1080.0 - 2334.0 * 2.0 / 9.98, abs=0.5)
    assert marker.width == pytest.approx(0.6 * 2334.0 / 9.98, rel=0.02) and marker.height == marker.width and marker.score > 0.9
    # Out of the picture: nothing. No truth near the frame's time: nothing, and the reason is kept.
    world.add(11 * S, truth(marker=(60.0, 0.0, 0.02)))
    assert detector.detect_full(11 * S) == ()
    assert detector.detect_full(20 * S) == () and "no true poses" in detector.last_refusal


def test_the_vehicle_between_two_true_poses_is_where_it_was_at_the_frames_time():
    detector, world = exact_oracle()
    world.add(10 * S, truth(vehicle=(0.0, 0.0, 10.0)))
    world.add(10 * S + 40_000_000, truth(vehicle=(0.16, 0.0, 10.0)))  # 4 m/s east
    early, late = detector.detect_full(10 * S)[0], detector.detect_full(10 * S + 40_000_000)[0]
    middle = detector.detect_full(10 * S + 20_000_000)[0]
    assert middle.cy == pytest.approx((early.cy + late.cy) / 2.0, abs=0.2) and late.cy - early.cy == pytest.approx(2334.0 * 0.16 / 9.98, abs=0.5)


def test_the_oracles_noise_misses_and_false_alarms_are_what_it_is_told_and_repeat_with_the_seed():
    def run(seed):
        world = oracle.TruthHistory("iris")
        detector = oracle.OracleDetector(CAMERA, MOUNT, oracle.objects_from(OBJECTS), world, seed=seed, pixel_sigma_px=3.0, miss_rate=0.2, false_alarms_per_frame=0.1)
        out = []
        for index in range(2000):
            world.add(10 * S + index * 100_000_000, truth())
            out.append(detector.detect_full(10 * S + index * 100_000_000))
        return out

    frames = run(7)
    assert frames == run(7) and frames != run(8)
    real = [d for frame in frames for d in frame if d.score >= 0.5]
    false = [d for frame in frames for d in frame if d.score < 0.5]
    assert 0.17 < 1.0 - len(real) / 2000.0 < 0.23  # misses
    assert 150 < len(false) < 250 and {d.class_id for d in false} <= set(contract.CLASSES)  # one frame in ten
    exact = exact_oracle()[0]
    exact._truth.add(10 * S, truth())
    centre = exact.detect_full(10 * S)[0]
    assert np.std([d.cx - centre.cx for d in real]) == pytest.approx(3.0, rel=0.1) and abs(np.mean([d.cy - centre.cy for d in real])) < 0.3
    with pytest.raises(ValueError, match="miss_rate"):
        oracle.OracleDetector(CAMERA, MOUNT, oracle.objects_from(OBJECTS), oracle.TruthHistory("iris"), seed=1, miss_rate=1.5)


def test_the_oracles_objects_come_from_the_runs_file_and_only_contract_classes_are_taken(tmp_path):
    path = tmp_path / "oracle_objects.json"
    path.write_text(json.dumps(OBJECTS))
    vehicle, objects = oracle.load_objects(path)
    assert vehicle == "iris" and [item.name for item in objects] == ["marker_3", "corner"]
    with pytest.raises(ValueError, match="not a class of the detection contract"):
        oracle.objects_from({"vehicle": "iris", "objects": [{"name": "x", "class_id": "payload", "size_m": 0.2, "top_m": 0.1}]})
    with pytest.raises(ValueError, match="oracle_objects_file"):
        oracle.load_objects(tmp_path / "missing.json")


# --- one frame through the pipeline ----------------------------------------------------------


class Poses:
    """A pose history with the vehicle level over the origin at 10 m, facing east."""

    def __init__(self, east=0.0, up=10.0):
        self.buffer = PoseBuffer()
        for index in range(-20, 40):
            self.buffer.add_pose(10 * S + index * 33_000_000, (east, 0.0, up), LEVEL)


class OneMarker:
    """A detector that finds a marker where the test says, in the pixels of the image it is given."""

    name = "test"

    def __init__(self, at=(640.0, 360.0), score=0.9):
        self.at, self.score, self.shapes = at, score, []

    def detect(self, image, stamp_ns):
        self.shapes.append(image.shape)
        return (contract.Detection("marker", self.score, self.at[0], self.at[1], 20.0, 20.0), contract.Detection("bucket", 0.1, 100.0, 100.0, 10.0, 10.0))


def frame_4k(stamp_ns=10 * S, seq=1):
    image = np.zeros((2160, 3840, 3), dtype=np.uint8)
    image[1080 - 30 : 1080 + 30, 1920 - 30 : 1920 + 30] = 255
    return SimpleNamespace(image=image, seq=seq, capture_ns=stamp_ns, mode="test_4k", stamp_source="pts")


def pipeline_with(detector, **settings):
    poses = Poses()
    projector = GroundProjector(CAMERA, MOUNT, poses.buffer.pose_at)
    return pipeline.LocPipeline(detector, projector, frame_id="down_camera_optical", full_size=FULL, **settings)


def test_a_frame_is_shrunk_detected_scaled_back_cropped_and_projected():
    detector = OneMarker(at=(740.0, 260.0))  # 100 px right of and above the small frame's centre
    result = pipeline_with(detector).process(frame_4k())
    assert detector.shapes == [(720, 1280, 3)]  # the detector saw the shrunk frame
    assert result.stamp_ns == 10 * S and result.detections.frame_id == "down_camera_optical"
    assert [d.class_id for d in result.detections.detections] == ["marker"]  # the 0.1 bucket is under the 0.25 confidence
    found = result.detections.detections[0]
    assert (found.cx, found.cy, found.width, found.height) == pytest.approx((2220.0, 780.0, 60.0, 60.0))  # 4K pixels
    # On the ground: 300 px up the image is ahead (east), 300 px right is to the right (south), at 10 m.
    ground = result.ground[0]
    assert (ground.east_m, ground.north_m) == pytest.approx((300.0 * 10.0 / 2334.0, -300.0 * 10.0 / 2334.0), abs=0.01)
    assert ground.class_id == "marker" and ground.detection is found
    # A 4K crop for the digit reader, with a margin round the box, cut from the frame itself.
    crop = result.crops[0]
    assert crop.detection is found and crop.image.shape == (78, 78, 3) and crop.image.base is not None
    assert set(result.timings_ms) == {"shrink", "detect", "project"} and result.refused == 0


def test_the_centre_marker_crop_is_the_marker_and_a_box_at_the_edge_is_cut_to_the_frame():
    result = pipeline_with(OneMarker(at=(640.0, 360.0))).process(frame_4k())
    crop = result.crops[0].image
    assert crop.shape == (78, 78, 3) and (crop[9:69, 9:69] == 255).all() and (crop[:8] == 0).all()
    edge = pipeline_with(OneMarker(at=(2.0, 2.0))).process(frame_4k())
    assert edge.crops[0].image.shape == (45, 45, 3) and edge.crops[0].origin == (0, 0)


def test_a_frame_with_no_pose_gives_its_detections_and_says_none_could_be_placed():
    result = pipeline_with(OneMarker()).process(frame_4k(stamp_ns=500 * S))
    assert len(result.detections.detections) == 1 and result.ground == [] and result.refused == 1 and "pose" in result.refusal


def test_the_tracker_and_the_digit_reader_are_handed_each_frames_ground_points_and_crops():
    seen = {}
    tracker = SimpleNamespace(update=lambda stamp_ns, ground: seen.setdefault("tracker", (stamp_ns, list(ground))))
    reader = lambda crops: seen.setdefault("reader", list(crops))  # noqa: E731
    result = pipeline_with(OneMarker(), tracker=tracker, digit_reader=reader).process(frame_4k())
    assert seen["tracker"] == (10 * S, result.ground) and seen["reader"] == result.crops


def test_with_tiling_on_the_detector_sees_tiles_of_the_full_frame_and_boxes_are_already_full_size():
    class CentreOnly(OneMarker):
        def detect(self, image, stamp_ns):
            self.shapes.append(image.shape)
            bright = np.argwhere(image[:, :, 0] == 255)
            if not len(bright):
                return ()
            (y0, x0), (y1, x1) = bright.min(axis=0), bright.max(axis=0)
            return (contract.Detection("marker", 0.8, (x0 + x1 + 1) / 2.0, (y0 + y1 + 1) / 2.0, float(x1 - x0 + 1), float(y1 - y0 + 1)),)

    detector = CentreOnly()
    result = pipeline_with(detector, tiling=True).process(frame_4k())
    assert len(detector.shapes) == 16 and set(detector.shapes) == {(720, 1280, 3)}
    found = result.detections.detections
    assert len(found) == 1 and (found[0].cx, found[0].cy, found[0].width) == pytest.approx((1920.0, 1080.0, 60.0))


# --- the runtime: the newest frame, never a queue ------------------------------------------------


def runtime_with(detector=None, **settings):
    clock = SimpleNamespace(ns=10 * S)
    run = runtime.LocRuntime(pipeline_with(detector or OneMarker()), lambda: clock.ns, **settings)
    return run, clock


def test_the_pipeline_is_off_until_it_is_started_and_stops_when_told():
    run, clock = runtime_with()
    run.offer(frame_4k())
    assert run.step() is None and run.status()["enabled"] is False and run.status()["frames"] == 0
    assert run.set_enabled(True) == "localisation pipeline started"
    run.offer(frame_4k(seq=2))
    assert run.step() is not None and run.status()["frames"] == 1
    assert run.set_enabled(True) == "localisation pipeline already started"
    assert run.set_enabled(False) == "localisation pipeline stopped after 1 frames"
    run.offer(frame_4k(seq=3))
    assert run.step() is None and run.status()["frames"] == 1
    started, _ = runtime_with(start_enabled=True)
    assert started.status()["enabled"] is True


def test_a_frame_that_waited_behind_a_newer_one_is_dropped_not_queued():
    run, clock = runtime_with(start_enabled=True)
    for seq in (1, 2, 3):
        run.offer(frame_4k(stamp_ns=10 * S + seq * 33_000_000, seq=seq))
    result = run.step()
    assert result.stamp_ns == 10 * S + 3 * 33_000_000 and run.step() is None  # the newest, and nothing behind it
    status = run.status()
    assert status["frames"] == 1 and status["dropped"] == 2


def test_the_status_says_the_rate_the_detections_and_the_frames_age():
    run, clock = runtime_with(start_enabled=True)
    for index in range(50):  # 10 frames a second for 5 s
        clock.ns = 10 * S + index * 100_000_000
        run.offer(frame_4k(stamp_ns=clock.ns - 40_000_000, seq=index))
        run.step()
    status = run.status()
    assert status["rate_hz"] == pytest.approx(10.0, abs=0.3) and status["frames"] == 50 and status["dropped"] == 0
    assert status["detections"] == 50 and status["placed"] >= 0 and status["detector"] == "test"
    assert status["frame_age_ms"] == pytest.approx(40.0, abs=1.0) and status["tracks"] == 0 and status["confirmed_ids"] == []
    assert status["below_minimum_rate"] is False and runtime.MIN_RATE_HZ == 8.0 and runtime.TARGET_RATE_HZ == 15.0
    json.dumps(status)  # it goes out as JSON
    # Five frames a second is under the minimum, and the status says so.
    slow, clock = runtime_with(start_enabled=True)
    for index in range(25):
        clock.ns = 10 * S + index * 200_000_000
        slow.offer(frame_4k(stamp_ns=clock.ns, seq=index))
        slow.step()
    assert slow.status()["rate_hz"] == pytest.approx(5.0, abs=0.3) and slow.status()["below_minimum_rate"] is True
    # A tracker's tracks are counted when there is one.
    tracker = SimpleNamespace(update=lambda stamp_ns, ground: None, track_count=lambda: 4, confirmed_ids=lambda: [3, 7])
    tracked = runtime.LocRuntime(pipeline_with(OneMarker(), tracker=tracker), lambda: 10 * S, start_enabled=True)
    assert tracked.status()["tracks"] == 4 and tracked.status()["confirmed_ids"] == [3, 7]
    assert tracked.status()["track_positions"] == "filter" and status["track_positions"] is None  # which position the tracks report; none without a tracker
    tracker.median_positions = True
    assert tracked.status()["track_positions"] == "median"


def test_a_fault_in_one_frame_is_counted_and_the_next_frame_is_still_processed():
    class Faulty(OneMarker):
        def detect(self, image, stamp_ns):
            if stamp_ns == 10 * S:
                raise RuntimeError("engine fell over")
            return super().detect(image, stamp_ns)

    run, clock = runtime_with(Faulty(), start_enabled=True)
    run.offer(frame_4k(stamp_ns=10 * S))
    assert run.step() is None and run.status()["faults"] == 1 and "engine fell over" in run.status()["last_fault"]
    run.offer(frame_4k(stamp_ns=10 * S + 33_000_000, seq=2))
    assert run.step() is not None and run.status()["frames"] == 1


def test_a_frame_newer_than_the_newest_pose_waits_for_its_pose_and_is_not_placed_without_one():
    """Found in the sim: a frame's turn came 34 ms after its capture, before MAVROS's pose of that time had arrived,
    and a fifth of the boxes could not be put on the ground."""
    poses = PoseBuffer()
    for index in range(-20, 1):
        poses.add_pose(10 * S + index * 33_000_000, (0.0, 0.0, 10.0), LEVEL)  # the newest pose is at 10.000 s
    chain = pipeline.LocPipeline(OneMarker(), GroundProjector(CAMERA, MOUNT, poses.pose_at), "down_camera_optical", FULL, pose_at=poses.pose_at)
    clock = SimpleNamespace(ns=10 * S + 90_000_000)
    run = runtime.LocRuntime(chain, lambda: clock.ns, start_enabled=True)
    late = 10 * S + 60_000_000  # captured 60 ms after the newest pose: more than the pose history will carry on
    assert chain.pose_ready(10 * S) and not chain.pose_ready(late)
    run.offer(frame_4k(stamp_ns=late))
    assert run.step() is None and run.status()["frames"] == 0 and run.status()["waiting_for_pose"] is True
    poses.add_pose(10 * S + 66_000_000, (0.0, 0.0, 10.0), LEVEL)  # the pose after the frame arrives
    result = run.step()
    assert result is not None and len(result.ground) == 1 and result.refused == 0 and run.status()["waiting_for_pose"] is False
    # A pose that never comes: the frame is processed after the wait, its boxes published and counted as not placed.
    assert runtime.POSE_WAIT_S == 0.15
    run.offer(frame_4k(stamp_ns=10 * S + 300_000_000, seq=2))
    clock.ns = 10 * S + 400_000_000
    assert run.step() is None
    clock.ns = 10 * S + 451_000_000
    result = run.step()
    assert result is not None and result.ground == [] and result.refused == 1
    # A newer frame replaces one that is waiting, as it replaces any other.
    run.offer(frame_4k(stamp_ns=10 * S + 500_000_000, seq=3))
    clock.ns = 10 * S + 510_000_000
    assert run.step() is None
    run.offer(frame_4k(stamp_ns=10 * S + 600_000_000, seq=4))
    assert run.status()["dropped"] == 1


# --- the engine detector (the 2026 way of running an engine) ---------------------------------


class Tensor:
    """What Ultralytics hands back: a value on a device."""

    def __init__(self, value):
        self.value = value

    def cpu(self):
        return SimpleNamespace(numpy=lambda: self.value)


class FakeModel:
    def __init__(self, boxes):
        self.boxes, self.calls = boxes, []

    def __call__(self, image, conf, verbose):
        self.calls.append((image.shape, conf, verbose))
        boxes = [SimpleNamespace(xyxy=[[Tensor(v) for v in xyxy]], conf=[Tensor(score)], cls=[Tensor(number)]) for xyxy, score, number in self.boxes]
        return [SimpleNamespace(boxes=boxes)]


def test_the_engines_boxes_become_contract_detections_in_the_images_pixels():
    model = FakeModel([((100.0, 200.0, 140.0, 260.0), 0.8, 0.0), ((10.0, 10.0, 20.0, 20.0), 0.6, 3.0)])
    detector = engine.EngineDetector(model, ["marker"], confidence=0.25)
    found = detector.detect(np.zeros((720, 1280, 3), dtype=np.uint8), 10 * S)
    assert found == (contract.Detection("marker", 0.8, 120.0, 230.0, 40.0, 60.0),) and detector.unknown_classes == 1  # class 3 is not in the list
    assert model.calls == [((720, 1280, 3), 0.25, False)] and detector.name == "engine"
    # Through the pipeline: shrunk, found, and back in 4K pixels.
    result = pipeline_with(engine.EngineDetector(FakeModel([((630.0, 350.0, 650.0, 370.0), 0.9, 0.0)]), ["marker"])).process(frame_4k())
    assert [(d.cx, d.cy, d.width) for d in result.detections.detections] == [pytest.approx((1920.0, 1080.0, 60.0))]
    for bad in ([], ["kite"], ["marker", "payload"]):
        with pytest.raises(engine.EngineUnavailable, match="engine_classes"):
            engine.EngineDetector(model, bad)


def test_an_engine_is_loaded_the_2026_way_and_refused_where_it_cannot_run(tmp_path, monkeypatch):
    with pytest.raises(engine.EngineUnavailable, match="needs engine_path"):
        engine.load_engine("", ["marker"])
    with pytest.raises(engine.EngineUnavailable, match="is not a file"):
        engine.load_engine(tmp_path / "missing.engine", ["marker"])
    file = tmp_path / "yolo.engine"
    file.write_bytes(b"not a real engine")
    monkeypatch.setitem(sys.modules, "ultralytics", None)  # as on a machine without it
    with pytest.raises(engine.EngineUnavailable, match="Ultralytics, which is not installed here"):
        engine.load_engine(file, ["marker"])
    loaded = []
    monkeypatch.setitem(sys.modules, "ultralytics", SimpleNamespace(YOLO=lambda path, task: loaded.append((path, task)) or FakeModel([])))
    detector = engine.load_engine(file, ["marker"], confidence=0.25)
    assert loaded == [(str(file), "detect")] and detector.detect(np.zeros((720, 1280, 3), dtype=np.uint8), 0) == ()


def test_a_pipeline_that_is_on_and_gets_no_frames_is_under_its_rate_too():
    run, clock = runtime_with()
    run.set_enabled(True)
    clock.ns = 10 * S + 4 * S
    assert run.status()["below_minimum_rate"] is False  # not yet a window's worth of time
    clock.ns = 10 * S + 6 * S
    status = run.status()
    assert status["rate_hz"] == 0.0 and status["below_minimum_rate"] is True
    run.set_enabled(False)
    assert run.status()["below_minimum_rate"] is False  # off is not slow


def test_a_fault_outside_a_frame_is_counted_with_its_reason():
    run, clock = runtime_with(start_enabled=True)
    run.note_fault("FrameSourceError: the camera gave a 640 x 480 frame")
    assert run.status()["faults"] == 1 and "640 x 480" in run.status()["last_fault"]


# --- the flight-safety review's points --------------------------------------------------------


def test_frames_that_keep_arriving_cannot_hold_the_pipeline_waiting_for_a_pose_that_never_comes():
    """Each newer frame used to start the wait again: with the pose stopped and the camera running, no frame was
    ever processed."""
    poses = PoseBuffer()
    for index in range(-20, 1):
        poses.add_pose(10 * S + index * 33_000_000, (0.0, 0.0, 10.0), LEVEL)  # and then the pose stops
    chain = pipeline.LocPipeline(OneMarker(), GroundProjector(CAMERA, MOUNT, poses.pose_at), "down_camera_optical", FULL, pose_at=poses.pose_at)
    clock = SimpleNamespace(ns=10 * S)
    run = runtime.LocRuntime(chain, lambda: clock.ns, start_enabled=True)
    results = []
    for tick_index in range(100, 1100):  # the node's turn every 10 ms for ten seconds, a frame every 100 ms
        clock.ns = 10 * S + tick_index * 10_000_000
        if tick_index % 10 == 0:
            run.offer(frame_4k(stamp_ns=clock.ns - 30_000_000, seq=tick_index))  # 30 ms old when it arrives
        results.append(run.step())
    processed = [result for result in results if result is not None]
    assert len(processed) >= 45 and all(result.ground == [] and result.refused == 1 for result in processed)  # published, not placed
    assert run.status()["frames"] == len(processed) and "pose" in run.status()["last_refusal"]
    # The pose comes back, trailing each frame by 40 ms: frames wait for it again and are placed, every one.
    placed_before = run.status()["placed"]
    for tick_index in range(1100, 1300):
        clock.ns = 10 * S + tick_index * 10_000_000
        poses.add_pose(clock.ns - 10_000_000, (0.0, 0.0, 10.0), LEVEL)  # the pose of 10 ms ago arrives now
        if tick_index % 10 == 0:
            run.offer(frame_4k(stamp_ns=clock.ns + 30_000_000, seq=tick_index))  # stamped 40 ms ahead of the newest pose
        run.step()
    assert run.status()["placed"] - placed_before >= 18  # of twenty frames; before, they went out unplaced after one outage


def test_the_rate_is_frames_over_the_time_and_a_pipeline_that_stalls_is_seen_to():
    run, clock = runtime_with(start_enabled=True)
    for index in range(100):  # ten frames a second for ten seconds
        clock.ns = 10 * S + index * 100_000_000
        run.offer(frame_4k(stamp_ns=clock.ns, seq=index))
        run.step()
    assert run.status()["rate_hz"] == pytest.approx(10.0, abs=0.3) and run.status()["below_minimum_rate"] is False
    clock.ns += 2 * S  # then nothing for two seconds: the last five seconds held 30 frames
    assert run.status()["rate_hz"] == pytest.approx(6.0, abs=0.3) and run.status()["below_minimum_rate"] is True
    clock.ns += 4 * S
    assert run.status()["rate_hz"] == 0.0 and run.status()["below_minimum_rate"] is True


def test_the_ground_is_the_rangefinders_when_believed_and_the_sites_when_not():
    class Ranges:
        last_refusal = None

        def __init__(self, answer):
            self.answer, self.expected = answer, []

        def ground_up(self, stamp_ns, expected_ground_up_m=0.0):
            self.expected.append(expected_ground_up_m)
            return self.answer

    believed = Ranges((2.4, "rangefinder"))
    assert pipeline.SiteGround(believed, ground_below_home_m=-2.0).ground_up(10 * S) == (2.4, "rangefinder") and believed.expected == [2.0]
    refused = Ranges((0.0, "home"))
    assert pipeline.SiteGround(refused, ground_below_home_m=-2.0).ground_up(10 * S) == (2.0, "home")  # the field is 2 m above home
    assert pipeline.SiteGround(refused).ground_up(10 * S) == (0.0, "home")
    for bad in (float("nan"), 25.0, "low"):
        with pytest.raises(ValueError, match="ground_below_home_m"):
            pipeline.SiteGround(refused, ground_below_home_m=bad)
    # Through the pipeline: with no rangefinder, a marker 300 px off the centre lands where a ground 2 m up puts it.
    poses = Poses()
    projector = GroundProjector(CAMERA, MOUNT, poses.buffer.pose_at, pipeline.SiteGround(refused, ground_below_home_m=-2.0))
    result = pipeline.LocPipeline(OneMarker(at=(740.0, 260.0)), projector, "down_camera_optical", FULL).process(frame_4k())
    assert result.ground[0].east_m == pytest.approx(300.0 * 8.0 / 2334.0, abs=0.01) and result.ground[0].point.height_source == "home"


def test_an_engine_that_cannot_run_a_frame_is_refused_when_it_is_loaded(tmp_path, monkeypatch):
    """An engine built for another TensorRT loads as a file and fails at its first frame: that frame is asked for
    at the start, on the ground, not at the first track."""
    file = tmp_path / "other_tensorrt.engine"
    file.write_bytes(b"stand-in")

    class Broken:
        def __call__(self, image, conf, verbose):
            raise RuntimeError("TensorRT version mismatch")

    monkeypatch.setitem(sys.modules, "ultralytics", SimpleNamespace(YOLO=lambda path, task: Broken()))
    with pytest.raises(engine.EngineUnavailable, match="did not run a frame.*TensorRT version mismatch"):
        engine.load_engine(file, ["marker"])
    warm = FakeModel([])
    monkeypatch.setitem(sys.modules, "ultralytics", SimpleNamespace(YOLO=lambda path, task: warm))
    engine.load_engine(file, ["marker"])
    assert warm.calls == [((720, 1280, 3), 0.25, False)]  # one blank frame of the detector's size, at the load


def test_a_tracker_that_faults_does_not_cost_the_frame_its_detections_and_cut_boxes_are_marked():
    def broken(stamp_ns, ground):
        raise RuntimeError("tracker fell over")

    chain = pipeline_with(OneMarker(), tracker=SimpleNamespace(update=broken))
    result = chain.process(frame_4k())
    assert len(result.detections.detections) == 1 and len(result.ground) == 1 and "tracker fell over" in result.fault
    run = runtime.LocRuntime(chain, lambda: 10 * S, start_enabled=True)
    run.offer(frame_4k())
    assert run.step() is not None and run.status()["faults"] == 1 and "tracker fell over" in run.status()["last_fault"]
    # A box that reaches the edge of the picture is the part of the marker in view: it is published, and marked.
    whole = pipeline_with(OneMarker(at=(640.0, 360.0))).process(frame_4k())
    edge = pipeline_with(OneMarker(at=(8.0, 360.0))).process(frame_4k())
    assert whole.ground[0].cut is False and edge.ground[0].cut is True


def test_the_digit_readers_time_is_in_the_frames_timings():
    reader = lambda crops: [None for _ in crops]  # noqa: E731
    result = pipeline_with(OneMarker(), digit_reader=reader).process(frame_4k())
    assert set(result.timings_ms) == {"shrink", "detect", "project", "digits"} and result.readings == [None]
    assert "digits" not in pipeline_with(OneMarker()).process(frame_4k()).timings_ms
