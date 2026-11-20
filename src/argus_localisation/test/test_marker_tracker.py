"""The marker tracker (LOC-7): sightings on the ground into one track a marker."""

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "argus_localisation"), str(SRC / "drone_mission_core"), str(SRC / "argus_camera")]

from argus_localisation import marker_tracker as mt  # noqa: E402
from argus_localisation.pipeline import GroundDetection  # noqa: E402
from drone_mission_core import detection_contract as contract  # noqa: E402
from drone_mission_core import target_estimator  # noqa: E402
from drone_mission_core.projection_v2 import CameraModel, CameraMount, pixel_of_ground_point, project  # noqa: E402

S = 1_000_000_000
CAMERA = CameraModel(width=3840, height=2160, fx=2334.0, fy=2334.0, cx=1920.0, cy=1080.0, name="test_4k")
MOUNT = CameraMount(position_m=(0.0, 0.0, 0.0), rotation_body_from_optical=((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0)))
LEVEL = (0.0, 0.0, 0.0, 1.0)


def seen(east, north, class_id="marker", score=0.9, vehicle=None, box_px=140.0):
    """A marker at (east, north) as the pipeline hands it over: seen from 10.67 m, nearly overhead, projected by projection v2."""
    position = vehicle or (east - 1.0, north + 0.5, 10.67)
    pixel = pixel_of_ground_point(CAMERA, MOUNT, (east, north, 0.0), position, LEVEL)
    point = project(CAMERA, MOUNT, pixel, position, LEVEL, 0.0)
    detection = contract.Detection(class_id, score, pixel[0], pixel[1], box_px, box_px)
    return GroundDetection(class_id, score, float(point.east_m), float(point.north_m), detection, point)


def track(tracker, frames, *positions, start_s=100.0, dt_s=0.1, class_id="marker"):
    """Hand the tracker `frames` frames, each with a sighting of every position."""
    for index in range(frames):
        tracker.update(int((start_s + index * dt_s) * S), [seen(e, n, class_id) for e, n in positions])
    return int((start_s + frames * dt_s) * S)


def test_the_numbers_are_the_specs():
    assert mt.GATE_NIS == 9.21 == target_estimator.GATE_NIS and mt.GATE_M == 1.0 and mt.CONFIRM_SIGHTINGS == 3
    assert mt.TRACKED_CLASSES == ("marker", "boundary_marker")
    assert (mt.TENTATIVE, mt.CONFIRMED, mt.COASTING, mt.LOST) == (0, 1, 2, 3)  # MarkerTrack.msg's constants


def test_a_marker_is_tentative_until_its_third_sighting():
    tracker = mt.MarkerTracker()
    track(tracker, 1, (35.0, 20.0))
    (one,) = tracker.tracks()
    assert one.state == mt.TENTATIVE and one.sightings == 1 and tracker.confirmed_ids() == []
    track(tracker, 1, (35.0, 20.0), start_s=100.1)
    assert tracker.tracks()[0].state == mt.TENTATIVE
    track(tracker, 1, (35.0, 20.0), start_s=100.2)
    (three,) = tracker.tracks()
    assert three.state == mt.CONFIRMED and three.sightings == 3 and three.track_id == one.track_id
    assert (three.east_m, three.north_m) == pytest.approx((35.0, 20.0), abs=0.01) and three.class_id == "marker"
    assert tracker.confirmed_ids() == [three.track_id] and tracker.track_count() == 1


def test_the_covariance_is_the_estimators_and_narrows_with_sightings():
    tracker = mt.MarkerTracker()
    first = seen(35.0, 20.0)
    tracker.update(100 * S, [first])
    expected = target_estimator.measurement_from_point("marker", first.point, 140.0, 140.0, 0.9).covariance
    assert np.allclose(tracker.tracks()[0].covariance, expected, rtol=1.0e-6)  # R = J Sigma J^T, as FND-06 builds it
    track(tracker, 10, (35.0, 20.0), start_s=100.1)
    later = tracker.tracks()[0].covariance
    assert np.trace(later) < 0.2 * np.trace(expected) and np.linalg.eigvalsh(later).min() >= target_estimator.COVARIANCE_FLOOR_M2 - 1.0e-12


def test_a_tentative_track_never_seen_again_is_lost_and_dropped_and_a_confirmed_one_is_kept_for_good():
    tracker = mt.MarkerTracker()
    track(tracker, 1, (35.0, 20.0))  # a false alarm: once
    track(tracker, 5, (62.0, 31.0), start_s=100.1)
    false_id, real_id = (t.track_id for t in sorted(tracker.tracks(), key=lambda t: t.east_m))
    tracker.update(int(102.5 * S), [])
    states = {t.track_id: t.state for t in tracker.tracks()}
    assert states[false_id] == mt.LOST and states[real_id] == mt.COASTING  # not seen for over a second: coasting, still listed
    tracker.update(int(104.0 * S), [])
    assert [t.track_id for t in tracker.tracks()] == [real_id]
    tracker.update(int(700.0 * S), [])  # ten minutes on: the survey's result is still there
    (kept,) = tracker.tracks()
    assert kept.state == mt.COASTING and tracker.confirmed_ids() == [real_id] and kept.sightings == 5
    track(tracker, 1, (62.0, 31.0), start_s=701.0)
    assert tracker.tracks()[0].state == mt.CONFIRMED and tracker.tracks()[0].track_id == real_id


def test_two_markers_in_a_frame_each_go_to_their_own_track_whatever_order_they_come_in():
    tracker = mt.MarkerTracker()
    for index in range(6):
        pair = [seen(35.0, 20.0), seen(37.5, 20.4)]
        tracker.update(int((100 + index * 0.1) * S), pair if index % 2 else pair[::-1])
    tracks = sorted(tracker.tracks(), key=lambda t: t.east_m)
    assert [(round(t.east_m, 1), round(t.north_m, 1), t.sightings, t.state) for t in tracks] == [(35.0, 20.0, 6, mt.CONFIRMED), (37.5, 20.4, 6, mt.CONFIRMED)]


def test_a_sighting_over_a_metre_from_a_track_is_another_marker_however_unsure_the_track_is():
    tracker = mt.MarkerTracker()
    track(tracker, 1, (35.0, 20.0))  # one sighting: the track's covariance is still wide
    tracker.update(int(100.1 * S), [seen(36.2, 20.0)])
    assert len(tracker.tracks()) == 2  # 1.2 m away: over the 1 m gate, whatever the Mahalanobis distance says


def test_the_one_metre_gate_holds_when_the_other_gate_would_let_the_pair_through():
    """Two minutes unseen, a track's covariance has grown to half a metre a side: a sighting 1.2 m away is inside the
    9.21 gate (about 4), and only the 1 m gate keeps it from being taken for the same marker."""
    tracker = mt.MarkerTracker()
    track(tracker, 5, (35.0, 20.0))
    (first,) = tracker.tracks()
    later = int(225.0 * S)
    wide = first.covariance + np.eye(2) * target_estimator.PROCESS_NOISE_M2_PER_S * 120.0
    sighting = seen(36.2, 20.0)
    noise = target_estimator.measurement_from_point("marker", sighting.point, 140.0, 140.0, 0.9).covariance
    difference = np.array([1.2, 0.0])
    assert float(difference @ np.linalg.inv(wide + noise) @ difference) < mt.GATE_NIS  # the Mahalanobis gate alone would take it
    tracker.update(later, [sighting])
    tracks = sorted(tracker.tracks(), key=lambda t: t.east_m)
    assert len(tracks) == 2 and tracks[0].track_id == first.track_id and tracks[0].sightings == 5
    assert tracks[0].east_m == pytest.approx(first.east_m, abs=1.0e-9) and tracks[1].east_m == pytest.approx(36.2, abs=0.01)


def test_the_boundary_is_the_four_best_seen_boundary_markers_when_a_fifth_has_been_confirmed():
    """A false alarm confirmed as a boundary marker far outside must not widen the boundary."""
    tracker = mt.MarkerTracker()
    track(tracker, 20, *CORNERS, class_id="boundary_marker")
    track(tracker, 3, (120.0, 30.0), start_s=110.0, class_id="boundary_marker")  # three sightings of nothing, 40 m out
    track(tracker, 4, (83.5, 30.0), start_s=120.0)
    (marker,) = [t for t in tracker.tracks() if t.class_id == "marker"]
    assert marker.outside and tracker.confirmed_ids() == []


def test_a_sighting_inside_a_metre_but_outside_the_gate_is_not_a_second_track_and_five_of_them_move_the_track():
    tracker = mt.MarkerTracker()
    track(tracker, 30, (35.0, 20.0))
    (sure,) = tracker.tracks()
    for index in range(4):  # our own position has slipped 0.8 m: over four sigma of a sighting from 35 ft (one degree of attitude is 0.19 m)
        tracker.update(int((103.0 + index * 0.1) * S), [seen(35.8, 20.0)])
        (still,) = tracker.tracks()
        assert (still.east_m, still.sightings) == (pytest.approx(35.0, abs=0.02), 30)
    tracker.update(int(103.4 * S), [seen(35.8, 20.0)])
    (moved,) = tracker.tracks()
    assert moved.track_id == sure.track_id and moved.east_m == pytest.approx(35.8, abs=0.02) and moved.state == mt.CONFIRMED and moved.sightings == 31
    assert tracker.rejected == 4


def test_a_marker_seen_again_on_the_next_track_is_the_same_marker_though_our_position_has_drifted_a_little():
    tracker = mt.MarkerTracker()
    track(tracker, 25, (35.0, 20.0))
    track(tracker, 25, (35.25, 20.15), start_s=125.0)  # twenty seconds later, 0.3 m off
    (one,) = tracker.tracks()
    assert one.sightings == 50 and (one.east_m, one.north_m) == pytest.approx((35.2, 20.1), abs=0.1)


def test_a_marker_and_a_boundary_marker_in_one_place_are_two_tracks():
    tracker = mt.MarkerTracker()
    for index in range(3):
        tracker.update(int((100 + index * 0.1) * S), [seen(20.0, 10.0, "boundary_marker"), seen(20.3, 10.0, "marker"), seen(5.0, 5.0, "bucket")])
    assert sorted((t.class_id, t.state) for t in tracker.tracks()) == [("boundary_marker", mt.CONFIRMED), ("marker", mt.CONFIRMED)]  # the bucket is not the tracker's


CORNERS = ((20.0, 10.0), (80.0, 10.0), (80.0, 50.0), (20.0, 50.0))


def test_a_marker_outside_the_four_boundary_markers_is_marked_outside_and_kept_off_the_list():
    tracker = mt.MarkerTracker()
    track(tracker, 4, (35.0, 20.0), (83.5, 30.0), (20.2, 49.9))
    assert not any(t.outside for t in tracker.tracks())  # no boundary known yet: nothing can be called outside
    track(tracker, 4, *CORNERS[:3], start_s=110.0, class_id="boundary_marker")
    assert not any(t.outside for t in tracker.tracks())  # three corners are not the boundary
    track(tracker, 4, CORNERS[3], start_s=120.0, class_id="boundary_marker")
    markers = {(round(t.east_m, 1), round(t.north_m, 1)): t for t in tracker.tracks() if t.class_id == "marker"}
    assert not markers[(35.0, 20.0)].outside and markers[(83.5, 30.0)].outside
    assert not markers[(20.2, 49.9)].outside  # on the line, by the corner: inside, with the half metre the hull is given
    listed = tracker.confirmed_ids()
    assert markers[(83.5, 30.0)].track_id not in listed and markers[(35.0, 20.0)].track_id in listed and len(listed) == 2
    assert tracker.track_count() == 3 and not any(t.outside for t in tracker.tracks() if t.class_id == "boundary_marker")


def test_the_sightings_of_a_track_are_kept_for_its_final_position():
    tracker = mt.MarkerTracker()
    track(tracker, 5, (35.0, 20.0))
    (one,) = tracker.tracks()
    sightings = tracker.sightings_of(one.track_id)
    assert len(sightings) == 5 and all(s.weight > 0.0 and (s.east_m, s.north_m) == pytest.approx((35.0, 20.0), abs=0.01) for s in sightings)
    assert [s.stamp_ns for s in sightings] == sorted(s.stamp_ns for s in sightings) and tracker.sightings_of(999) == []


def test_a_sighting_that_cannot_be_used_is_counted_and_left_out():
    tracker = mt.MarkerTracker()
    bad = seen(35.0, 20.0)
    broken = GroundDetection("marker", 0.9, float("nan"), 20.0, bad.detection, SimpleNamespace(**{**vars(bad.point), "east_m": float("nan")}))
    tracker.update(100 * S, [broken])
    assert tracker.tracks() == [] and tracker.unusable == 1


# --- the flight-safety review's points --------------------------------------------------------


def test_a_track_carries_the_height_of_the_ground_its_marker_was_put_on():
    tracker = mt.MarkerTracker()
    position = (34.0, 20.5, 12.67)
    pixel = pixel_of_ground_point(CAMERA, MOUNT, (35.0, 20.0, 2.0), position, LEVEL)
    point = project(CAMERA, MOUNT, pixel, position, LEVEL, 2.0)  # the field is 2 m above home
    tracker.update(100 * S, [GroundDetection("marker", 0.9, float(point.east_m), float(point.north_m), contract.Detection("marker", 0.9, pixel[0], pixel[1], 140.0, 140.0), point)])
    assert tracker.tracks()[0].up_m == pytest.approx(2.0) and (tracker.tracks()[0].east_m, tracker.tracks()[0].north_m) == pytest.approx((35.0, 20.0), abs=0.01)


def test_a_step_in_our_own_position_moves_every_track_with_it():
    tracker = mt.MarkerTracker()
    track(tracker, 5, (35.0, 20.0), (62.0, 31.0))
    tracker.shift_all(1.5, -0.4)  # an EKF reset: everything we will now see is 1.5 m east and 0.4 m south of where it was
    assert sorted((round(t.east_m, 2), round(t.north_m, 2)) for t in tracker.tracks()) == [(36.5, 19.6), (63.5, 30.6)]
    track(tracker, 3, (36.5, 19.6), (63.5, 30.6), start_s=101.0)
    assert len(tracker.tracks()) == 2 and all(t.sightings == 8 for t in tracker.tracks())  # the same two markers, not four
    assert all((s.east_m, s.north_m) == pytest.approx((36.5, 19.6), abs=0.02) for s in tracker.sightings_of(tracker.tracks()[0].track_id))  # the sightings kept for LOC-9 moved too
    with pytest.raises(ValueError, match="finite"):
        tracker.shift_all(float("nan"), 0.0)


def test_a_box_cut_by_the_edge_of_the_picture_is_not_a_sighting():
    tracker = mt.MarkerTracker()
    whole = seen(35.0, 20.0)
    cut = GroundDetection("marker", 0.9, whole.east_m, whole.north_m, whole.detection, whole.point, cut=True)
    for index in range(5):
        tracker.update(int((100 + index * 0.1) * S), [cut])
    assert tracker.tracks() == [] and tracker.cut == 5
    tracker.update(int(100.5 * S), [whole])
    assert len(tracker.tracks()) == 1


def test_a_clock_that_steps_back_drops_what_was_not_confirmed_and_keeps_the_list():
    tracker = mt.MarkerTracker()
    track(tracker, 5, (62.0, 31.0), start_s=1000.0)
    track(tracker, 1, (35.0, 20.0), start_s=1000.5)  # one sighting only
    tracker.update(int(200.0 * S), [])  # the clock is 800 s behind where it was
    (kept,) = tracker.tracks()
    assert kept.state == mt.COASTING and kept.sightings == 5 and kept.updated_ns <= 200 * S
    track(tracker, 1, (62.0, 31.0), start_s=200.1)
    assert tracker.tracks()[0].state == mt.CONFIRMED and len(tracker.tracks()) == 1
    track(tracker, 1, (35.0, 20.0), start_s=200.2)
    tracker.update(int(203.0 * S), [])
    assert sorted(t.state for t in tracker.tracks()) == [mt.COASTING, mt.LOST]  # ageing works on the new clock


def test_four_boundary_tracks_that_are_not_four_corners_are_not_a_boundary():
    tracker = mt.MarkerTracker()
    track(tracker, 5, *CORNERS[:3], class_id="boundary_marker")
    track(tracker, 5, (21.5, 10.4), start_s=110.0, class_id="boundary_marker")  # a second track by the first corner: a triangle
    track(tracker, 4, (30.0, 45.0), start_s=120.0)  # a marker in the part of the field the triangle leaves out
    (marker,) = [t for t in tracker.tracks() if t.class_id == "marker"]
    assert not marker.outside and tracker.confirmed_ids() == [marker.track_id]


def test_there_is_a_limit_to_how_many_tracks_are_held():
    assert mt.MAX_TRACKS == 200
    tracker = mt.MarkerTracker()
    for index in range(mt.MAX_TRACKS + 20):
        tracker.update(int((100 + index * 0.001) * S), [seen(10.0 + 3.0 * (index % 50), 10.0 + 3.0 * (index // 50))])
    assert len(tracker.tracks()) == mt.MAX_TRACKS and tracker.refused_full == 20
