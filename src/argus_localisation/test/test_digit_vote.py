"""The weighted vote over a track's digit readings, and the assignment when two tracks claim one digit (LOC-8)."""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "argus_localisation"), str(SRC / "drone_mission_core")]

from argus_localisation import digit_vote as dv  # noqa: E402


def reading(digit, probability=0.97):
    """Ten log-probabilities with `probability` on one digit and the rest shared out."""
    scores = np.full(10, math.log((1.0 - probability) / 9.0))
    scores[digit] = math.log(probability)
    return tuple(float(v) for v in scores)


def either(a=6, b=9):
    rest = 1.0e-6
    return tuple(math.log(0.5 - 4.0 * rest) if digit in (a, b) else math.log(rest) for digit in range(10))


def test_a_sighting_weighs_more_when_it_is_bigger_sharper_more_overhead_and_surer():
    base = dict(side_px=80.0, sharpness=800.0, view_angle_rad=math.radians(20.0), detector_score=0.7)
    weight = dv.sighting_weight(**base)
    assert 0.0 < weight < 1.0
    for better in ({"side_px": 120.0}, {"sharpness": 3000.0}, {"view_angle_rad": 0.0}, {"detector_score": 0.95}):
        assert dv.sighting_weight(**{**base, **better}) > weight, better
    # Each term is a share from 0 to 1, and the weight is their product.
    assert dv.sighting_weight(side_px=500.0, sharpness=1.0e9, view_angle_rad=0.0, detector_score=1.0) == pytest.approx(1.0, abs=1.0e-3)
    assert dv.sighting_weight(side_px=dv.FULL_SIDE_PX / 2, sharpness=dv.SHARPNESS_HALF, view_angle_rad=math.radians(60.0), detector_score=0.5) == pytest.approx(0.5 * 0.5 * 0.25 * 0.5)
    # A reading with nothing to weigh it by weighs nothing.
    for nonsense in ({"side_px": 0.0}, {"sharpness": float("nan")}, {"view_angle_rad": math.radians(95.0)}, {"detector_score": -1.0}, {"side_px": None}):
        assert dv.sighting_weight(**{**base, **nonsense}) == 0.0, nonsense


def test_a_tracks_vote_is_the_weighted_sum_of_its_readings_log_probabilities():
    votes = dv.DigitVotes()
    votes.add(4, reading(3, 0.9), 0.5)
    votes.add(4, reading(3, 0.6), 0.25)
    votes.add(4, reading(8, 0.7), 1.0)
    expected = 0.5 * np.array(reading(3, 0.9)) + 0.25 * np.array(reading(3, 0.6)) + 1.0 * np.array(reading(8, 0.7))
    assert votes.log_probabilities(4) == pytest.approx(tuple(expected))
    assert votes.readings(4) == 3 and votes.weight(4) == pytest.approx(1.75)
    assert votes.log_probabilities(5) == (0.0,) * 10 and votes.readings(5) == 0  # a track nothing was read on
    # A reading that says nothing (all ten equal), a weight of nothing, and nonsense are not votes.
    votes.add(4, (math.log(0.1),) * 10, 1.0)
    votes.add(4, reading(1), 0.0)
    votes.add(4, (0.0,) * 9, 1.0)
    votes.add(4, tuple([float("nan")] * 10), 1.0)
    assert votes.readings(4) == 3 and votes.log_probabilities(4) == pytest.approx(tuple(expected))


def test_many_right_readings_outvote_a_few_sure_wrong_ones_and_no_reading_is_a_veto():
    votes = dv.DigitVotes(unique_ids=False)
    for _ in range(22):
        votes.add(1, reading(7, 0.95), 0.8)
    for _ in range(3):
        votes.add(1, reading(1, 0.999), 0.9)  # the 7 misread as a 1, surely, three times
    assert votes.digits()[1] == 7
    # A reading that gives a digit no chance at all cannot rule it out by itself: a reading's log-probabilities have a floor.
    veto = [0.0 if digit == 2 else -math.inf for digit in range(10)]
    votes.add(1, veto, 1.0)
    assert votes.digits()[1] == 7 and min(votes.log_probabilities(1)) > -1.0e6
    assert dv.MIN_LOG_PROBABILITY == pytest.approx(math.log(1.0e-3))


def test_a_track_has_no_digit_until_its_vote_is_sure():
    votes = dv.DigitVotes(unique_ids=False)
    assert votes.digits() == {}
    votes.add(1, reading(5, 0.6), 0.3)
    assert votes.digits()[1] == dv.UNKNOWN  # one weak reading
    for _ in range(6):
        votes.add(1, reading(5, 0.6), 0.6)
    assert votes.digits()[1] == 5 and votes.confidence(1) >= dv.MIN_VOTE_CONFIDENCE
    # A track whose readings all say "6 or 9" stays unknown, and its vote says which two.
    for _ in range(20):
        votes.add(2, either(), 0.9)
    assert votes.digits()[2] == dv.UNKNOWN and votes.confidence(2) == pytest.approx(0.5, abs=1.0e-3)
    assert sorted(np.argsort(votes.log_probabilities(2))[-2:]) == [6, 9]


def test_two_tracks_claiming_one_digit_are_settled_by_the_assignment_over_tracks_and_digits():
    def filled(unique_ids):
        votes = dv.DigitVotes(unique_ids=unique_ids)
        for _ in range(20):
            votes.add(1, reading(1, 0.97), 0.9)  # the 1
        for _ in range(12):
            votes.add(2, reading(1, 0.55), 0.5)  # the 7, read as a 1 a little more often than as a 7
            votes.add(2, reading(7, 0.5), 0.5)
        for _ in range(15):
            votes.add(3, reading(4, 0.95), 0.8)  # nobody else's
        return votes

    assert filled(unique_ids=True).digits() == {1: 1, 2: 7, 3: 4}
    # With the step switched off (IDs that may repeat), each track keeps its own vote: here the 7's track is not sure of anything.
    alone = filled(unique_ids=False)
    assert alone.digits()[1] == 1 and alone.digits()[3] == 4 and alone.digits()[2] in (1, dv.UNKNOWN)
    assert int(np.argmax(alone.log_probabilities(2))) == 1


def test_the_assignment_does_not_toss_a_coin():
    # Two tracks with the same evidence for the same digit: neither is given it.
    votes = dv.DigitVotes()
    for track in (1, 2):
        for _ in range(10):
            votes.add(track, reading(3, 0.97), 0.9)
    assert votes.digits() == {1: dv.UNKNOWN, 2: dv.UNKNOWN} and votes.contested() == [1, 2]
    # Two "6 or 9" tracks: one is the 6 and one the 9, and nothing says which.
    votes = dv.DigitVotes()
    for track in (1, 2):
        for _ in range(10):
            votes.add(track, either(), 0.9)
    assert votes.digits() == {1: dv.UNKNOWN, 2: dv.UNKNOWN}
    # A "6 or 9" track beside a track that is surely the 6 (a marker with its own cue): it is the 9.
    votes = dv.DigitVotes()
    for _ in range(10):
        votes.add(1, either(), 0.9)
        votes.add(3, reading(6, 0.97), 0.9)
    assert votes.digits() == {1: 9, 3: 6} and votes.contested() == []
    # Alone, a "6 or 9" track is not settled by there being no other: the field may hold only one of the two.
    votes = dv.DigitVotes()
    for _ in range(10):
        votes.add(1, either(), 0.9)
    assert votes.digits() == {1: dv.UNKNOWN}


def test_more_tracks_than_digits_leave_the_weakest_without_one_and_only_listed_tracks_take_part():
    votes = dv.DigitVotes()
    for digit in range(10):
        for _ in range(8):
            votes.add(digit + 1, reading(digit, 0.97), 0.9)
    votes.add(11, reading(4, 0.6), 0.3)  # an eleventh track with one weak reading
    digits = votes.digits()
    assert [digits[track] for track in range(1, 11)] == list(range(10)) and digits[11] == dv.UNKNOWN
    # Asked about some tracks only (the confirmed markers inside the search area), the others neither get a digit nor take one.
    votes = dv.DigitVotes()
    for _ in range(3):
        votes.add(1, reading(2, 0.97), 0.9)  # a false track that reads as a 2, three times
    for _ in range(3):
        votes.add(2, reading(2, 0.97), 0.9)  # the real 2
    assert votes.digits() == {1: dv.UNKNOWN, 2: dv.UNKNOWN}
    assert votes.digits(only=[2]) == {2: 2}


def test_the_answer_is_worked_out_when_the_votes_have_changed_not_every_time_it_is_asked_for(monkeypatch):
    votes = dv.DigitVotes()
    votes.add(1, reading(2), 0.9)
    calls = []
    real = dv.hungarian
    monkeypatch.setattr(dv, "hungarian", lambda cost: calls.append(1) or real(cost))
    first = votes.digits()
    asked = len(calls)
    assert votes.digits() == first and len(calls) == asked
    votes.add(1, reading(2), 0.9)
    votes.digits()
    assert len(calls) > asked
    votes.forget([1])
    assert votes.digits() == {} and votes.readings(1) == 0


def test_a_frames_readings_go_to_the_tracks_their_boxes_went_to():
    """What the pipeline does with a frame: each reading is weighed by its crop and its view, and added to its box's track."""
    box_a, box_b, box_c = (SimpleNamespace(width=140.0, height=140.0) for _ in range(3))
    overhead = SimpleNamespace(camera_enu=(0.0, 0.0, 10.0), up_m=0.0, range_m=10.0)
    slanted = SimpleNamespace(camera_enu=(0.0, 0.0, 10.0), up_m=0.0, range_m=10.0 / math.cos(math.radians(35.0)))
    ground = [
        SimpleNamespace(class_id="marker", score=0.9, detection=box_a, point=overhead),
        SimpleNamespace(class_id="marker", score=0.9, detection=box_b, point=slanted),
        SimpleNamespace(class_id="marker", score=0.9, detection=box_c, point=overhead),
    ]
    crops = [SimpleNamespace(detection=box) for box in (box_a, box_b, box_c)]
    read = lambda digit: SimpleNamespace(digit=digit, log_probabilities=reading(digit), side_px=130.0, sharpness=2500.0)  # noqa: E731
    readings = [read(3), read(8), read(5)]
    votes = dv.DigitVotes()
    added = votes.add_frame({0: 11, 1: 12}, ground, crops, readings)  # the third box went to no track (cut, or gated out)
    assert added == 2 and votes.readings(11) == 1 and votes.readings(12) == 1 and votes.readings(13) == 0
    assert dv.view_angle(overhead) == pytest.approx(0.0) and dv.view_angle(slanted) == pytest.approx(math.radians(35.0))
    assert votes.weight(11) == pytest.approx(dv.sighting_weight(130.0, 2500.0, 0.0, 0.9)) and votes.weight(12) < votes.weight(11)
    # Fewer readings than crops (a reader that stopped early), or a point with no camera position: no vote, no fault.
    assert dv.DigitVotes().add_frame({0: 11, 1: 12}, ground, crops, readings[:1]) == 1
    ground[0].point = SimpleNamespace()
    assert dv.DigitVotes().add_frame({0: 11}, ground, crops, readings) == 0


def test_a_survey_of_small_blurred_markers_is_read_by_the_vote():
    """The reader and the vote together on drawn markers, 60 px across and blurred: a single reading is often
    no answer, eight sightings a marker are a digit. Last year's plate has a cue for 6 against 9; this year's
    marker has none, and its 6 and 9 stay unknown rather than being split by a coin."""
    sys.path.insert(0, str(SRC.parent / "tools"))
    import synth_targets

    from argus_localisation import digit_reader

    def survey(layout, target, sightings=8):
        rng = np.random.default_rng(4)
        reader = digit_reader.DigitReader(digit_reader.TemplateDigitClassifier(layout), layout=layout)
        votes = dv.DigitVotes()
        for digit in range(10):
            for index in range(sightings):
                options = synth_targets.RenderOptions(
                    width=140,
                    height=140,
                    mpp=0.01,
                    rotation_deg=float(rng.uniform(0.0, 360.0)),
                    tilt_deg=(float(rng.uniform(-10.0, 10.0)), float(rng.uniform(-10.0, 10.0))),
                    blur_sigma_px=float(rng.uniform(0.5, 1.4)),
                    noise_sigma=3.0,
                    background="noise",
                    supersample=2,
                )
                picture, _truth = synth_targets.render(target(digit), options, seed=100 * digit + index)
                read = reader.read(SimpleNamespace(image=picture, origin=(0, 0)))
                votes.add(digit + 1, read.log_probabilities, dv.sighting_weight(read.side_px, read.sharpness, math.radians(float(rng.uniform(0.0, 30.0))), 0.9))
        return reader, votes.digits()

    reader, digits = survey(digit_reader.LAYOUT_2026, lambda digit: synth_targets.NumberedMarker(digit=digit))
    assert reader.abstained_count > 0  # single readings did fail
    assert digits == {digit + 1: digit for digit in range(10)}
    reader, digits = survey(digit_reader.LAYOUT_2027, lambda digit: synth_targets.SurveyMarker(digit=digit))
    assert {track: digit for track, digit in digits.items() if track not in (7, 10)} == {digit + 1: digit for digit in (0, 1, 2, 3, 4, 5, 7, 8)}
    assert digits[7] == dv.UNKNOWN and digits[10] == dv.UNKNOWN


# --- the reviews' points ----------------------------------------------------------------------


def test_between_solves_the_last_answer_stands_for_the_period_the_node_gives():
    """The flight-safety review: the assignment shares the detector's thread, so the node solves it at most twice a second."""
    now = [100.0]
    votes = dv.DigitVotes(solve_period_s=0.5, clock=lambda: now[0])
    for _ in range(5):
        votes.add(1, reading(2), 0.9)
    assert votes.digits(only=[1]) == {1: 2} and votes.solves == 1 and votes.last_solve_ms >= 0.0
    for _ in range(40):
        votes.add(1, reading(8), 0.9)  # the votes turn to an 8
    now[0] = 100.3
    assert votes.digits(only=[1]) == {1: 2} and votes.solves == 1  # within the period: the last answer
    now[0] = 100.6
    assert votes.digits(only=[1]) == {1: 8} and votes.solves == 2
    # A track joining the ones asked about is answered at once, period or not.
    for _ in range(5):
        votes.add(2, reading(4), 0.9)
    now[0] = 100.7
    assert votes.digits(only=[1, 2]) == {1: 8, 2: 4} and votes.solves == 3
    # A clock that steps back does not hold the answer for ever.
    votes.add(2, reading(4), 0.9)
    now[0] = 50.0
    votes.digits(only=[1, 2])
    assert votes.solves == 4
    assert dv.NODE_SOLVE_PERIOD_S == 0.5 and dv.DigitVotes().solve_period_s == 0.0  # by default every change is answered


def test_the_assignment_is_over_the_twenty_strongest_claims():
    assert dv.MAX_ASSIGNED_TRACKS == 20
    votes = dv.DigitVotes()
    for track in range(1, 26):  # 25 tracks; the first 20 have the stronger claims
        strong = track <= 20
        for _ in range(12 if strong else 3):
            votes.add(track, reading(track % 10, 0.97 if strong else 0.9), 0.9)
    digits = votes.digits()
    assert len(digits) == 25 and all(digits[track] == dv.UNKNOWN for track in range(21, 26))  # beyond the cap: no digit, and no part in the others'
    # Tracks 1 to 10 and 11 to 20 claim the ten digits twice over with equal evidence: none is given.
    assert all(digits[track] == dv.UNKNOWN for track in range(1, 21)) and votes.contested() == list(range(1, 21))
    # With one strong claim to each digit, the weak ones change nothing.
    votes = dv.DigitVotes()
    for track in range(1, 11):
        for _ in range(12):
            votes.add(track, reading(track % 10, 0.97), 0.9)
    for track in range(11, 36):
        votes.add(track, reading(track % 10, 0.9), 0.5)
    digits = votes.digits()
    assert [digits[track] for track in range(1, 11)] == [track % 10 for track in range(1, 11)] and all(digits[track] == dv.UNKNOWN for track in range(11, 36))


def test_settling_the_votes_has_the_next_answer_worked_out_afresh_within_the_period():
    """For the end of a survey, when the judges' list is closed: the digits as the votes now stand, not as they stood half a second ago."""
    now = [10.0]
    votes = dv.DigitVotes(solve_period_s=0.5, clock=lambda: now[0])
    for _ in range(5):
        votes.add(1, reading(2), 0.9)
    assert votes.digits(only=[1]) == {1: 2}
    for _ in range(40):
        votes.add(1, reading(8), 0.9)
    now[0] = 10.1
    assert votes.digits(only=[1]) == {1: 2}  # within the period
    votes.settle()
    assert votes.digits(only=[1]) == {1: 8} and votes.solves == 2
