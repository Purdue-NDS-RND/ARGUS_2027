"""The estimator's feed (FND-06): detections on the contract in, projected with the pose at their frame's time, tracks out. No ROS."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from stand_in_messages import FakeMessages  # noqa: E402

from drone_mission_core import estimator_feed as ef  # noqa: E402
from drone_mission_core.detection_contract import Detection, DetectionSet  # noqa: E402
from drone_mission_core.estimator_feed import EstimatorFeed, PositionResetDetector  # noqa: E402
from drone_mission_core.pose_buffer import PoseBuffer  # noqa: E402
from drone_mission_core.projection_v2 import CameraModel, CameraMount, GroundProjector, pixel_of_ground_point  # noqa: E402
from drone_mission_core.target_estimator import CONFIRMED, TENTATIVE  # noqa: E402

MS = 1_000_000
S = 1_000_000_000
T0 = 500 * S
CAMERA = CameraModel(width=1280, height=720, fx=1000.0, fy=1000.0, cx=640.0, cy=360.0, name="test")
MOUNT = CameraMount(position_m=(0.1, 0.0, -0.05), rotation_body_from_optical=((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0)))
LEVEL = (0.0, 0.0, 0.0, 1.0)
HEIGHT = 10.0


class Flight:
    """A vehicle flying east at a steady speed, its poses in a PoseBuffer and a feed on top."""

    def __init__(self, speed_mps=0.0):
        self.speed = speed_mps
        self.poses = PoseBuffer()
        self.feed = EstimatorFeed(GroundProjector(CAMERA, MOUNT, self.poses.pose_at), self.poses)
        self.offset = (0.0, 0.0)  # what a position reset has added to the reported position

    def true_position(self, stamp_ns):
        return (self.speed * (stamp_ns - T0) * 1e-9, 0.0, HEIGHT)

    def reported(self, stamp_ns):
        east, north, up = self.true_position(stamp_ns)
        return (east + self.offset[0], north + self.offset[1], up)

    def fly(self, start_ns, end_ns, step_ms=33):
        stamp = start_ns
        while stamp <= end_ns:
            position = self.reported(stamp)
            self.poses.add_pose(stamp, position, LEVEL)
            self.poses.add_velocity(stamp, (self.speed, 0.0, 0.0))
            self.feed.add_pose(stamp, position)
            stamp += step_ms * MS

    def frame(self, stamp_ns, *targets, score=0.9):
        """A detection set for the frame captured at stamp_ns: each target is (class, east, north, height above ground)."""
        detections = []
        for class_id, east, north, up in targets:
            u, v = pixel_of_ground_point(CAMERA, MOUNT, (east, north, up), self.true_position(stamp_ns), LEVEL)
            detections.append(Detection(class_id=class_id, score=score, cx=u, cy=v, width=80.0, height=80.0))
        return DetectionSet(stamp_ns=stamp_ns, frame_id="down_camera_optical", detections=tuple(detections))


def test_a_target_seen_from_a_moving_vehicle_is_put_where_it_is_using_the_pose_at_each_frames_time():
    flight = Flight(speed_mps=5.0)
    flight.fly(T0, T0 + 600 * MS)
    for index in range(3):  # frames 150 ms old when they are handled: the vehicle has moved 0.75 m since
        stamp = T0 + (100 + index * 100) * MS
        flight.feed.add_detections(flight.frame(stamp, ("bullseye", 3.0, 1.0, 0.0)))
        flight.feed.step(stamp + 150 * MS)
    (track,) = flight.feed.estimates(T0 + 450 * MS)
    assert track.state == CONFIRMED and track.class_id == "bullseye"
    assert (track.east_m, track.north_m) == pytest.approx((3.0, 1.0), abs=0.01)
    assert flight.feed.counters["frames"] == 3 and flight.feed.counters["measurements"] == 3


def test_whoever_asks_is_given_each_frames_report_and_the_totals_do_not_keep_the_innovations():
    flight = Flight()
    flight.fly(T0, T0 + 600 * MS)
    reports = []
    flight.feed.on_report = lambda report, stamp_ns: reports.append((stamp_ns, report))
    for index in range(3):
        stamp = T0 + (100 + index * 100) * MS
        flight.feed.add_detections(flight.frame(stamp, ("bullseye", 3.0, 1.0, 0.0)))
        flight.feed.step(stamp + 50 * MS)
    assert [stamp for stamp, _ in reports] == [T0 + 100 * MS, T0 + 200 * MS, T0 + 300 * MS]
    assert [len(report.nis) for _, report in reports] == [0, 1, 1] and all(value < 0.01 for _, report in reports for value in report.nis)
    assert flight.feed.totals.nis == [] and flight.feed.totals.accepted == 2


def test_each_class_is_projected_onto_its_own_height():
    flight = Flight()
    flight.fly(T0, T0 + 400 * MS)
    stamp = T0 + 100 * MS
    flight.feed.add_detections(flight.frame(stamp, ("bucket", 2.0, 3.0, 0.2), ("bullseye", -3.0, 2.0, 0.0)))
    flight.feed.step(stamp + 50 * MS)
    tracks = {t.class_id: t for t in flight.feed.estimates(stamp + 50 * MS)}
    assert (tracks["bucket"].east_m, tracks["bucket"].north_m) == pytest.approx((2.0, 3.0), abs=1e-3)
    assert tracks["bucket"].up_m == pytest.approx(0.2) and tracks["bullseye"].up_m == pytest.approx(0.0)
    # On the ground plane the bucket's rim would have been put about 7 cm further out.
    assert (tracks["bullseye"].east_m, tracks["bullseye"].north_m) == pytest.approx((-3.0, 2.0), abs=1e-3)


def test_a_box_cut_by_the_edge_of_the_image_is_not_a_measurement():
    flight = Flight()
    flight.fly(T0, T0 + 900 * MS)
    feed = flight.feed
    cut = Detection("bullseye", 0.9, 49.0, 360.0, 100.0, 100.0)  # its left side is at the image's edge
    for index in range(6):  # the target coming into view: six frames of its visible part
        feed.add_detections(DetectionSet(T0 + (100 + index * 33) * MS, "down_camera_optical", (cut,)))
        feed.step(T0 + (110 + index * 33) * MS)
    assert feed.estimates(T0 + 300 * MS) == [] and feed.counters["skipped: box cut by the image's edge"] == 6
    for index in range(3):  # then whole
        stamp = T0 + (400 + index * 100) * MS
        feed.add_detections(flight.frame(stamp, ("bullseye", 2.0, 1.0, 0.0)))
        feed.step(stamp + 10 * MS)
    (track,) = feed.estimates(T0 + 610 * MS)
    assert track.state == CONFIRMED and (track.east_m, track.north_m) == pytest.approx((2.0, 1.0), abs=0.01)
    # A target that fills the frame is cut on every side: the track is not kept alive by it, and coasts.
    filling = Detection("bullseye", 0.9, 640.0, 360.0, 1280.0, 720.0)
    for index in range(4):
        feed.add_detections(DetectionSet(T0 + (650 + index * 50) * MS, "down_camera_optical", (filling,)))
    feed.step(T0 + 810 * MS)
    assert feed.estimates(T0 + 950 * MS)[0].state == "COASTING" and feed.totals.accepted == 2


def test_a_detector_with_its_own_noise_model_is_weighed_by_it_and_keeps_a_track_alive_where_a_cut_box_cannot():
    from drone_mission_core import estimator_feed

    assert estimator_feed.DETECTOR_PIXEL_SIGMA_PX == {"ring_fit": 2.0}  # the ring fit's own figure (drone_target_cv/ring_fit.py)
    # The same small box from two detectors: the box detector's centre is trusted less for the box being small,
    # the ring fit's centre is trusted as the ring fit says.
    def variance(pixel_sigma_px):
        flight = Flight()
        flight.fly(T0, T0 + 600 * MS)
        small = Detection("bullseye", 0.9, 640.0, 360.0, 8.0, 8.0)
        seen = []
        flight.feed.on_report = lambda report, stamp: None
        original = flight.feed.estimator.update

        def update(measurements, stamp_ns):
            seen.extend(measurements)
            return original(measurements, stamp_ns)

        flight.feed.estimator.update = update
        flight.feed.add_detections(DetectionSet(T0 + 100 * MS, "down_camera_optical", (small,)), pixel_sigma_px=pixel_sigma_px)
        flight.feed.step(T0 + 110 * MS)
        return float(seen[0].covariance[0][0])

    # An 8 px box is given the box detector's largest noise, 8 px. From 10 m up the pixels are one part of the
    # noise beside the attitude's and the height's, so the variance moves, not by the square of the ratio.
    assert variance(None) > variance(4.0) > variance(2.0)
    assert variance(None) - variance(2.0) == pytest.approx((8.0**2 - 2.0**2) * (HEIGHT / 1000.0) ** 2, rel=0.05)

    # Near the ground: the box is cut by the edge on every frame, the ring fit's detection of the same frames is not.
    flight = Flight()
    flight.fly(T0, T0 + 1200 * MS)
    feed = flight.feed
    for index in range(10):
        stamp = T0 + (100 + index * 50) * MS
        whole = flight.frame(stamp, ("bullseye", 2.0, 1.0, 0.0)).detections[0]
        cut = Detection("bullseye", 0.9, whole.cx, 360.0, 1280.0, 720.0)
        feed.add_detections(DetectionSet(stamp, "down_camera_optical", (cut,)))
        feed.add_detections(DetectionSet(stamp, "down_camera_optical", (Detection("bullseye", 0.8, whole.cx, whole.cy, 60.0, 60.0),)), pixel_sigma_px=2.0)
        feed.step(stamp + 10 * MS)
    (track,) = feed.estimates(T0 + 600 * MS)
    assert track.state == CONFIRMED and (track.east_m, track.north_m) == pytest.approx((2.0, 1.0), abs=0.02)
    assert feed.counters["skipped: box cut by the image's edge"] == 10
    assert feed.totals.started == 1 and feed.totals.accepted == 9  # the first starts the track, the other nine keep it

    # The ring fit's centre is not the centre of its box: near the picture's edge its box (the innermost ring)
    # runs over the edge, and the centre is as good as anywhere else. It is not skipped as a cut box.
    flight = Flight()
    flight.fly(T0, T0 + 1200 * MS)
    feed = flight.feed
    for index in range(4):
        stamp = T0 + (100 + index * 50) * MS
        feed.add_detections(DetectionSet(stamp, "down_camera_optical", (Detection("bullseye", 0.8, 30.0, 360.0, 144.0, 144.0),)), pixel_sigma_px=2.0)
        feed.step(stamp + 10 * MS)
    assert feed.counters.get("skipped: box cut by the image's edge", 0) == 0 and feed.totals.started == 1 and feed.totals.accepted == 3
    assert feed.estimates(T0 + 300 * MS)[0].state == CONFIRMED


def test_one_frame_counts_once_toward_confirming_a_track_however_many_detectors_saw_it():
    """Two detectors on the same red pixels are not two sightings: a track still needs three frames."""
    flight = Flight()
    flight.fly(T0, T0 + 1200 * MS)
    feed = flight.feed
    for index in range(2):
        stamp = T0 + (100 + index * 50) * MS
        whole = flight.frame(stamp, ("bullseye", 2.0, 1.0, 0.0))
        feed.add_detections(whole)  # the box detector
        feed.add_detections(DetectionSet(stamp, "down_camera_optical", whole.detections), pixel_sigma_px=2.0)  # the ring fit, same frame
        feed.step(stamp + 10 * MS)
    assert feed.estimates(T0 + 200 * MS)[0].state == "TENTATIVE"  # four hits, two frames
    third = flight.frame(T0 + 200 * MS, ("bullseye", 2.0, 1.0, 0.0))
    feed.add_detections(third)
    feed.step(T0 + 210 * MS)
    assert feed.estimates(T0 + 220 * MS)[0].state == CONFIRMED


def test_a_frame_that_arrives_before_its_pose_waits_for_it():
    flight = Flight()
    flight.fly(T0, T0 + 100 * MS)
    stamp = T0 + 190 * MS  # 90 ms past the newest pose: too far to carry the pose forward
    flight.feed.add_detections(flight.frame(stamp, ("bullseye", 1.0, 1.0, 0.0)))
    flight.feed.step(stamp + 5 * MS)
    assert flight.feed.estimates(stamp + 5 * MS) == [] and flight.feed.pending == 1
    flight.fly(T0 + 132 * MS, T0 + 300 * MS)
    flight.feed.step(stamp + 60 * MS)
    assert flight.feed.pending == 0
    assert [t.state for t in flight.feed.estimates(stamp + 60 * MS)] == [TENTATIVE]


def test_frames_that_cannot_be_placed_in_time_are_dropped_and_counted():
    flight = Flight()
    flight.fly(T0, T0 + 300 * MS)
    flight.fly(T0 + 700 * MS, T0 + 2000 * MS)  # 0.4 s of pose lost in between
    feed = flight.feed
    feed.add_detections(flight.frame(T0 + 500 * MS, ("bullseye", 1.0, 1.0, 0.0)))  # in the gap
    feed.add_detections(flight.frame(T0 + 1700 * MS, ("bullseye", 1.0, 1.0, 0.0)))  # stamped half a second ahead of the clock
    feed.step(T0 + 1200 * MS)
    feed.add_detections(flight.frame(T0 + 800 * MS, ("bullseye", 1.0, 1.0, 0.0)))  # over a second old by the time it is handled
    feed.step(T0 + 2000 * MS)
    assert feed.estimates(T0 + 2000 * MS) == [] and feed.pending == 0
    assert feed.counters["dropped: no pose for the frame's capture time"] == 1
    assert feed.counters["dropped: frame older than 1 s"] == 1
    assert feed.counters["dropped: stamp ahead of the clock"] == 1


def test_a_detection_that_does_not_come_down_to_the_ground_is_counted_not_tracked():
    flight = Flight()
    flight.fly(T0, T0 + 300 * MS)
    upside_down = (1.0, 0.0, 0.0, 0.0)  # rolled half a turn: the camera looks at the sky
    for stamp in range(T0 + 400 * MS, T0 + 600 * MS, 33 * MS):
        flight.poses.add_pose(stamp, (0.0, 0.0, HEIGHT), upside_down)
    stamp = T0 + 499 * MS
    flight.feed.add_detections(DetectionSet(stamp, "down_camera_optical", (Detection("bullseye", 0.9, 640.0, 360.0, 50.0, 50.0),)))
    flight.feed.step(stamp + 20 * MS)
    assert flight.feed.estimates(stamp + 20 * MS) == []
    assert flight.feed.counters["refused: the ray does not come down to the ground"] == 1


def test_the_queue_of_waiting_frames_is_bounded():
    flight = Flight()
    flight.fly(T0, T0 + 100 * MS)
    for index in range(ef.PENDING_LIMIT + 10):
        flight.feed.add_detections(flight.frame(T0 + 200 * MS + index, ("bullseye", 1.0, 1.0, 0.0)))
    assert flight.feed.pending == ef.PENDING_LIMIT
    assert flight.feed.counters["dropped: too many frames waiting"] == 10


def test_the_work_of_one_step_is_bounded():
    flight = Flight()
    flight.fly(T0, T0 + 900 * MS)
    feed = flight.feed
    for index in range(10):  # ten frames ready at once
        feed.add_detections(flight.frame(T0 + (100 + index * 33) * MS, ("bullseye", 2.0, 1.0, 0.0)))
    feed.step(T0 + 500 * MS)
    assert feed.counters["frames"] == ef.MAX_FRAMES_PER_STEP == 4 and feed.pending == 6  # the oldest four; the rest keep
    feed.step(T0 + 510 * MS)
    feed.step(T0 + 520 * MS)
    assert feed.counters["frames"] == 10 and feed.pending == 0
    # And a frame with a crowd in it: sixteen are kept, the best of every class first.
    crowd = tuple(Detection("marker", 0.5 + 0.01 * index, 100.0 + 40.0 * (index % 20), 100.0 + 60.0 * (index // 20), 20.0, 20.0) for index in range(40))
    lone = Detection("bullseye", 0.3, 900.0, 500.0, 60.0, 60.0)  # less confident than every marker
    feed.add_detections(DetectionSet(T0 + 600 * MS, "down_camera_optical", crowd + (lone,)))
    feed.step(T0 + 610 * MS)
    assert feed.counters["trimmed: more detections in a frame than are handled"] == 1
    assert feed.counters["measurements"] == 10 + ef.MAX_DETECTIONS_PER_FRAME
    assert len(feed.estimates(T0 + 610 * MS, "bullseye")) == 2  # the lone bullseye was not pushed out by the crowd
    kept = ef._most_confident_of_each_class(crowd + (lone,), 16)
    assert lone in kept and sorted(d.score for d in kept if d.class_id == "marker") == pytest.approx([0.75 + 0.01 * index for index in range(15)])


def test_a_frame_that_makes_the_estimator_fail_is_dropped_once_and_nothing_is_replayed(monkeypatch):
    flight = Flight()
    flight.fly(T0, T0 + 900 * MS)
    feed = flight.feed
    feed.add_detections(flight.frame(T0 + 100 * MS, ("bullseye", 2.0, 1.0, 0.0)))
    bad = flight.frame(T0 + 133 * MS, ("bucket", 2.0, 1.0, 0.2))
    feed.add_detections(bad)
    feed.add_detections(flight.frame(T0 + 166 * MS, ("bullseye", 2.0, 1.0, 0.0)))
    real = feed._te.plane_height_m

    def fails_on_bucket(class_id):
        if class_id == "bucket":
            raise RuntimeError("a bug")
        return real(class_id)

    monkeypatch.setattr(feed._te, "plane_height_m", fails_on_bucket)
    for index in range(5):  # stepping again and again must not feed the good frames in again
        feed.step(T0 + (200 + index * 10) * MS)
    assert feed.counters["failed: handling a frame"] == 1 and feed.last_fault == "RuntimeError: a bug"
    assert feed.pending == 0 and feed.counters["frames"] == 2
    (track,) = feed.estimates(T0 + 240 * MS)
    assert track.hits == 2 and track.state == TENTATIVE  # two frames, two hits: not confirmed by a replay


def test_detections_from_an_image_of_another_size_are_refused():
    flight = Flight()
    flight.fly(T0, T0 + 400 * MS)
    frame = flight.frame(T0 + 100 * MS, ("bullseye", 2.0, 1.0, 0.0))
    assert flight.feed.add_detections(frame, image_size=(640, 360)) is False  # the camera is calibrated at 1280 x 720
    assert flight.feed.pending == 0 and flight.feed.counters["dropped: the detector's image is not the calibrated mode's size"] == 1
    assert flight.feed.add_detections(frame, image_size=(1280, 720)) is True and flight.feed.add_detections(frame) is True
    assert flight.feed.pending == 2


def test_when_the_clock_steps_back_nothing_held_is_fresh_any_more():
    flight = Flight()
    flight.fly(T0, T0 + 400 * MS)
    feed = flight.feed
    for index in range(3):
        stamp = T0 + (100 + index * 100) * MS
        feed.add_detections(flight.frame(stamp, ("bullseye", 2.0, 1.0, 0.0)))
        feed.step(stamp + 40 * MS)
    feed.add_detections(flight.frame(T0 + 390 * MS, ("bucket", 1.0, 1.0, 0.2)))  # still waiting on the old clock
    assert feed.estimates(T0 + 350 * MS)[0].state == CONFIRMED
    earlier = T0 - 60 * S  # the clock goes back a minute
    feed.step(earlier)
    (track,) = feed.estimates(earlier)
    assert track.state == "LOST" and track.age_s == pytest.approx(2.0) and feed.pending == 0
    assert feed.counters["clock steps back"] == 1
    assert feed.estimates(earlier + 10 * S)[0].state == "LOST"  # kept, to be found again, not reported as just seen
    feed.step(earlier - 100 * MS)  # a little jitter is not a step
    assert feed.counters["clock steps back"] == 1


# --- position resets -------------------------------------------------------------------


def test_steady_flight_and_hard_manoeuvres_are_not_resets():
    detector = PositionResetDetector()
    position, velocity = [0.0, 0.0, 10.0], [8.0, 0.0, 0.0]
    for index in range(100):
        stamp = T0 + index * 33 * MS
        if index == 50:
            velocity = [-8.0, 6.0, 0.0]  # an instant turn: far more than the vehicle can do
        position = [position[0] + velocity[0] * 0.033, position[1] + velocity[1] * 0.033, 10.0]
        assert detector.add(stamp, position, tuple(velocity)) is None


def test_a_step_the_velocity_does_not_explain_is_a_reset_by_that_step():
    detector = PositionResetDetector()
    assert detector.add(T0, (0.0, 0.0, 10.0), (3.0, 0.0, 0.0)) is None
    assert detector.add(T0 + 33 * MS, (0.099, 0.0, 10.0), (3.0, 0.0, 0.0)) is None
    step = detector.add(T0 + 66 * MS, (0.198 + 1.5, -0.7, 10.0), (3.0, 0.0, 0.0))
    assert step == pytest.approx((1.5, -0.7), abs=1e-6)
    assert detector.add(T0 + 99 * MS, (0.297 + 1.5, -0.7, 10.0), (3.0, 0.0, 0.0)) is None  # once, not again


def test_no_reset_is_called_across_a_gap_or_without_a_velocity():
    detector = PositionResetDetector()
    detector.add(T0, (0.0, 0.0, 10.0), (0.0, 0.0, 0.0))
    assert detector.add(T0 + 500 * MS, (3.0, 0.0, 10.0), (0.0, 0.0, 0.0)) is None  # half a second unseen: it may have flown there
    assert detector.add(T0 + 533 * MS, (6.0, 0.0, 10.0), None) is None  # nothing to judge the step against


def test_after_a_position_reset_the_tracks_have_moved_with_it_and_detections_still_fit():
    flight = Flight()
    flight.fly(T0, T0 + 400 * MS)
    for index in range(3):
        stamp = T0 + (100 + index * 100) * MS
        flight.feed.add_detections(flight.frame(stamp, ("bullseye", 2.0, 1.0, 0.0)))
        flight.feed.step(stamp + 40 * MS)
    late = flight.frame(T0 + 390 * MS, ("bullseye", 2.0, 1.0, 0.0))  # taken before the reset, handled after it

    flight.offset = (1.5, -0.7)  # a GPS glitch: the flight controller now reports itself here
    flight.fly(T0 + 429 * MS, T0 + 900 * MS)
    assert flight.feed.counters["position resets"] == 1
    (track,) = flight.feed.estimates(T0 + 450 * MS)
    assert (track.east_m, track.north_m) == pytest.approx((3.5, 0.3), abs=0.01)

    flight.feed.add_detections(late)
    flight.feed.step(T0 + 460 * MS)
    for index in range(3):  # and frames taken after it, seen from the new reported position
        stamp = T0 + (500 + index * 100) * MS
        flight.feed.add_detections(flight.frame(stamp, ("bullseye", 2.0, 1.0, 0.0)))
        flight.feed.step(stamp + 40 * MS)
    assert flight.feed.totals.accepted == 2 + 1 + 3 and flight.feed.totals.rejected == 0
    (track,) = flight.feed.estimates(T0 + 740 * MS)
    assert track.state == CONFIRMED and (track.east_m, track.north_m) == pytest.approx((3.5, 0.3), abs=0.01)


# --- the forecast ----------------------------------------------------------------------


def test_outputs_are_for_the_actuation_time_and_the_vehicle_is_forecast_to_it_from_the_pose_history():
    flight = Flight(speed_mps=4.0)
    flight.fly(T0, T0 + 330 * MS)
    now = T0 + 335 * MS
    assert flight.feed.actuation_time_ns(now) == now + 80 * MS
    assert flight.feed.ready(now) and not flight.feed.ready(now + 1 * S) and not Flight().feed.ready(now)  # live only with a recent pose
    east, north, up = flight.feed.vehicle_position_at(flight.feed.actuation_time_ns(now))
    assert (east, north, up) == pytest.approx((4.0 * 0.415, 0.0, HEIGHT), abs=1e-6)
    assert flight.feed.vehicle_position_at(T0 + 200 * MS) == pytest.approx((0.8, 0.0, HEIGHT), abs=1e-6)  # a past instant: from the history
    assert flight.feed.vehicle_position_at(now + 1 * S) is None  # too far ahead to say

    flight.feed.add_detections(flight.frame(T0 + 300 * MS, ("bullseye", 2.0, 1.0, 0.0)))
    flight.feed.step(now)
    (track,) = flight.feed.estimates(now)
    assert track.for_ns == now + 80 * MS and track.age_s == pytest.approx(0.035)
    assert (flight.feed.primary("bullseye", now).track_id, flight.feed.primary("bucket", now)) == (track.track_id, None)


# --- the topic for logs and the ground station -----------------------------------------


def test_the_tracks_are_encoded_as_a_detection3darray():
    flight = Flight()
    flight.fly(T0, T0 + 400 * MS)
    for index in range(3):
        stamp = T0 + (100 + index * 100) * MS
        flight.feed.add_detections(flight.frame(stamp, ("bucket", 2.0, 1.0, 0.2), score=0.7))
        flight.feed.step(stamp + 40 * MS)
    now = T0 + 350 * MS
    estimates = flight.feed.estimates(now)
    array = ef.encode_targets(estimates, now, FakeMessages)
    assert (array.header.stamp.sec, array.header.stamp.nanosec, array.header.frame_id) == (500, 350 * MS, "map")
    (detection,) = array.detections
    (result,) = detection.results
    assert detection.id == f"bucket/{estimates[0].track_id}/CONFIRMED"
    assert (result.hypothesis.class_id, result.hypothesis.score) == ("bucket", pytest.approx(0.7))
    position = result.pose.pose.position
    assert (position.x, position.y, position.z) == pytest.approx((2.0, 1.0, 0.2), abs=1e-3)
    covariance = result.pose.covariance
    assert len(covariance) == 36
    assert (covariance[0], covariance[1], covariance[6], covariance[7]) == pytest.approx(
        (estimates[0].covariance[0][0], estimates[0].covariance[0][1], estimates[0].covariance[1][0], estimates[0].covariance[1][1])
    )
    assert covariance[0] > 0.0 and sum(abs(value) for value in covariance) == pytest.approx(abs(covariance[0]) + 2 * abs(covariance[1]) + abs(covariance[7]))
    centre = detection.bbox.center.position
    assert (centre.x, centre.y, centre.z) == (position.x, position.y, position.z)
    # The stamp of each detection is its track's last hit, so a log shows how old the information is.
    assert detection.header.stamp.sec * S + detection.header.stamp.nanosec == estimates[0].last_hit_ns
    assert math.isclose(detection.bbox.size.x, 2.0 * estimates[0].sigma_m) and detection.header.frame_id == "map"
    assert ef.encode_targets([], now, FakeMessages).detections == []
