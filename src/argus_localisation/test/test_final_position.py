"""A track's reported position: the weighted median of its sightings (LOC-9)."""

import sys
from pathlib import Path

import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "argus_localisation"), str(SRC / "drone_mission_core"), str(SRC / "argus_camera"), str(Path(__file__).resolve().parent)]

from argus_localisation import final_position as fp  # noqa: E402
from argus_localisation import marker_tracker as mt  # noqa: E402
from test_marker_tracker import S, seen, track  # noqa: E402


def test_the_weighted_median_is_the_value_half_the_weight_lies_either_side_of():
    assert fp.weighted_median([1.0, 2.0, 3.0], [1.0, 1.0, 1.0]) == 2.0
    assert fp.weighted_median([3.0, 1.0, 2.0], [1.0, 1.0, 1.0]) == 2.0  # in any order
    assert fp.weighted_median([1.0, 2.0, 3.0], [1.0, 1.0, 5.0]) == 3.0  # the heavy one decides
    assert fp.weighted_median([1.0, 2.0], [1.0, 1.0]) == 1.5  # exactly half either side: between the two
    assert fp.weighted_median([1.0, 2.0, 3.0, 4.0], [1.0, 1.0, 1.0, 1.0]) == 2.5
    assert fp.weighted_median([7.0], [0.3]) == 7.0
    assert fp.weighted_median([1.0, 2.0, 100.0], [1.0, 1.0, 0.9]) == 2.0  # one far value moves it by nothing
    # Values and weights that are not numbers, and weights that are not above zero, are left out.
    assert fp.weighted_median([1.0, float("nan"), 2.0, 3.0, 50.0], [1.0, 1.0, 1.0, 1.0, 0.0]) == 2.0
    for values, weights in (([], []), ([1.0], [0.0]), ([float("nan")], [1.0]), ([1.0, 2.0], [1.0])):
        with pytest.raises(ValueError):
            fp.weighted_median(values, weights)


def test_the_odd_bad_projection_moves_the_mean_and_not_the_median():
    rng = np.random.default_rng(3)
    good = [mt.Sighting(i, 35.0 + rng.normal(0.0, 0.05), 20.0 + rng.normal(0.0, 0.05), 25.0, 0.9) for i in range(24)]
    bad = [mt.Sighting(100 + i, 35.0 + 0.9, 20.0 - 0.7, 25.0, 0.9) for i in range(3)]  # three projections with a wrong attitude
    east, north = fp.final_position(good + bad)
    mean_east = float(np.mean([s.east_m for s in good + bad]))
    assert abs(mean_east - 35.0) > 0.08 and abs(east - 35.0) < 0.04 and abs(north - 20.0) < 0.04
    assert fp.final_position([]) is None
    # A sighting counts by its weight: two sure sightings outvote three unsure ones.
    sure = [mt.Sighting(i, 10.0, 5.0, 100.0, 0.9) for i in range(2)]
    unsure = [mt.Sighting(10 + i, 11.0, 6.0, 10.0, 0.9) for i in range(3)]
    assert fp.final_position(sure + unsure) == (10.0, 5.0)


def test_the_tracker_reports_the_median_only_when_told_to():
    def survey(tracker):
        track(tracker, 22, (35.0, 20.0))
        for index in range(3):  # three sightings 0.6 m off that the gates still took (the first frames after a long gap)
            tracker.update(int((160.0 + index * 0.1) * S), [seen(35.6, 20.0)])
        return tracker.tracks()[0]

    plain = survey(mt.MarkerTracker())
    assert (plain.reported_east_m, plain.reported_north_m) == (plain.east_m, plain.north_m)  # as LOC-7 built it: the filter's position
    median = survey(mt.MarkerTracker(median_positions=True))
    assert median.sightings == 25 and (median.east_m, median.north_m) == pytest.approx((plain.east_m, plain.north_m))  # the filter is the same
    assert abs(median.reported_east_m - 35.0) < 0.01 and abs(plain.east_m - 35.0) > 0.03  # the median is on the marker; the filter was pulled
    assert median.reported_north_m == pytest.approx(20.0, abs=0.01)


def test_whether_a_marker_is_outside_is_decided_on_the_position_that_is_reported():
    corners = ((20.0, 10.0), (80.0, 10.0), (80.0, 50.0), (20.0, 50.0))
    tracker = mt.MarkerTracker(median_positions=True)
    track(tracker, 5, *corners, class_id="boundary_marker")
    track(tracker, 20, (80.3, 30.0), start_s=110.0)  # just inside the half metre the hull is given
    for index in range(3):
        tracker.update(int((170.0 + index * 0.1) * S), [seen(80.9, 30.0)])
    (marker,) = [t for t in tracker.tracks() if t.class_id == "marker"]
    assert marker.reported_east_m == pytest.approx(80.3, abs=0.01) and not marker.outside and tracker.confirmed_ids() == [marker.track_id]


def test_when_a_track_is_moved_for_a_slip_of_our_position_its_sightings_and_its_reported_position_move_with_it():
    """The flight-safety review: the track moved and its reported position did not, under a fresh stamp."""
    tracker = mt.MarkerTracker(median_positions=True)
    track(tracker, 20, (35.0, 20.0))
    track(tracker, 5, (35.8, 20.0), start_s=110.0)  # four refused, and on the fifth the track moves
    (moved,) = tracker.tracks()
    assert moved.east_m == pytest.approx(35.8, abs=0.02) and moved.reported_east_m == pytest.approx(35.8, abs=0.02) and moved.sightings == 21
    assert all(s.east_m == pytest.approx(35.8, abs=0.03) for s in tracker.sightings_of(moved.track_id))  # the twenty made before the slip stepped with it
    track(tracker, 10, (35.8, 20.0), start_s=120.0)
    assert tracker.tracks()[0].reported_east_m == pytest.approx(35.8, abs=0.02)


def test_a_tracks_median_is_worked_out_again_only_when_its_sightings_change(monkeypatch):
    """The node asks for every track after every frame: forty markers with 400 sightings each must not be sorted ten times a second."""
    calls = []
    original = mt.final_position
    monkeypatch.setattr(mt, "final_position", lambda sightings: calls.append(len(sightings)) or original(sightings))
    tracker = mt.MarkerTracker(median_positions=True)
    track(tracker, 5, (35.0, 20.0), (62.0, 31.0))
    tracker.tracks()
    assert calls == [5, 5]  # each track's median, the first time it is asked for
    calls.clear()
    for _ in range(10):
        tracker.tracks()
        tracker.confirmed_ids()
    assert calls == []  # nothing new was seen: nothing is worked out
    tracker.update(int(101.0 * S), [seen(35.0, 20.0)])
    tracker.tracks()
    tracker.tracks()
    assert calls == [6]  # the one track that got a sighting, once
    tracker.shift_all(0.5, 0.0)
    assert sorted(round(t.reported_east_m, 2) for t in tracker.tracks()) == [35.5, 62.5]
