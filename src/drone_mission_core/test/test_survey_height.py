"""The survey's height and its guard (LOC-1). No ROS.

The rules' floor for the survey is 25 ft (7.62 m): below it costs points, below
20 ft the mission scores nothing. The survey flies at 35 ft by default, and a
guard on the height above the ground climbs back under 8.22 m and ends the
survey under 7.77 m.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_mission_core import safety_limits  # noqa: E402
from drone_mission_core import survey_height as sh  # noqa: E402
from drone_mission_core.target_access import HeightAgl  # noqa: E402


def at(height_m, source="rangefinder"):
    return HeightAgl(height_m, source)


def test_the_numbers_are_the_specs():
    assert sh.DEFAULT_HEIGHT_M == pytest.approx(35 * 0.3048) == pytest.approx(10.668)
    assert sh.LOG_JUDGED_HEIGHT_M == pytest.approx(30 * 0.3048) == pytest.approx(9.144)
    assert (sh.RECOVER_BELOW_M, sh.ABORT_BELOW_M) == (8.22, 7.77)
    assert safety_limits.SURVEY_MIN_HEIGHT_M == 7.62 < sh.ABORT_BELOW_M < sh.RECOVER_BELOW_M < sh.LOG_JUDGED_HEIGHT_M


def test_the_survey_flies_at_35_ft_or_at_30_ft_once_the_height_is_known_to_be_judged_from_our_log():
    assert sh.survey_height({}) == pytest.approx(10.668)
    assert sh.survey_height({"survey_height_judged_from_log": True}) == pytest.approx(9.144)
    assert sh.survey_height({"survey_height_judged_from_log": False}) == pytest.approx(10.668)
    assert sh.survey_height({"survey_height_m": 11.0}) == 11.0  # a sequence may fly higher, inside the rangefinder's reach
    assert sh.survey_height({"survey_height_m": 9.5, "survey_height_judged_from_log": True}) == 9.5  # its own number wins


@pytest.mark.parametrize("bad", [8.2, 7.0, 0.0, -1.0, "high", None, True, float("nan"), float("inf")])  # (and see the bounds' own test)
def test_a_survey_height_at_which_the_guard_would_already_be_climbing_is_refused(bad):
    with pytest.raises(ValueError, match="survey_height_m"):
        sh.survey_height({"survey_height_m": bad})
    with pytest.raises(ValueError, match="survey_height_judged_from_log"):
        sh.survey_height({"survey_height_judged_from_log": "yes"})


def watching(height_m=10.668):
    """A guard a survey is under way with: it has been asked once at the survey height."""
    guard = sh.SurveyHeightGuard(height_m)
    assert guard.update(at(height_m), 0.0).action == sh.CONTINUE
    return guard


def test_at_the_survey_height_the_guard_lets_the_survey_go_on():
    guard = sh.SurveyHeightGuard(10.668)
    answer = guard.update(at(10.6), 100.0)
    assert answer.action == sh.CONTINUE and answer.reason == ""
    assert guard.update(at(8.22), 100.05).action == sh.CONTINUE  # at the line is not below it


def test_below_8_22_m_the_survey_waits_and_the_vehicle_climbs_back_to_its_height():
    guard = sh.SurveyHeightGuard(10.668)
    guard.update(at(10.6), 100.0)
    answer = guard.update(at(8.21), 100.05)
    assert answer.action == sh.RECOVER and answer.climb_to_m == pytest.approx(10.668) and "8.21 m" in answer.reason and "8.22 m" in answer.reason
    # It stays in RECOVER until the height is back, not just over the line again: no chatter at 8.22 m.
    assert guard.update(at(8.5), 100.5).action == sh.RECOVER
    assert guard.update(at(10.0), 101.0).action == sh.RECOVER
    assert guard.update(at(10.668 - sh.BACK_WITHIN_M + 0.01), 102.0).action == sh.CONTINUE
    assert guard.recoveries == 1


def test_below_7_77_m_the_survey_is_over_for_good_and_the_climb_comes_before_anything_else():
    guard = watching()
    answer = guard.update(at(7.76), 100.0)
    assert answer.action == sh.ABORT and answer.climb_to_m == pytest.approx(10.668) and "7.76 m" in answer.reason
    assert guard.update(at(10.668), 105.0).action == sh.ABORT  # back at height: still over
    assert guard.aborted and "7.76 m" in guard.update(at(10.668), 106.0).reason  # and it says what ended it


def test_a_height_that_is_not_the_rangefinders_is_taken_to_be_lower_than_it_says():
    """The barometer's height is above home, not above this ground. It counts for BARO_MARGIN_M less."""
    assert watching().update(at(8.6, "baro"), 100.0).action == sh.RECOVER  # 8.6 - 0.5 is under 8.22
    assert watching().update(at(8.8, "baro"), 100.0).action == sh.CONTINUE
    assert watching().update(at(8.2, "baro"), 100.0).action == sh.ABORT  # 7.7


def test_a_recovery_on_a_barometer_height_ends_when_the_vehicle_reaches_the_height_it_was_told_to_climb_to():
    """The climb asked for on a barometer height is the survey height plus the margin, so that the counted
    height comes back to the survey height: a recovery that could never end would hold the survey for good."""
    guard = watching()
    answer = guard.update(at(8.6, "baro"), 100.0)
    assert answer.action == sh.RECOVER and answer.climb_to_m == pytest.approx(10.668 + sh.BARO_MARGIN_M)
    assert guard.update(at(10.668, "baro"), 105.0).action == sh.RECOVER  # at the survey height by the barometer: counted 0.5 m lower
    assert guard.update(at(answer.climb_to_m, "baro"), 110.0).action == sh.CONTINUE
    assert guard.update(at(answer.climb_to_m - 0.25, "baro"), 111.0).action == sh.CONTINUE  # and it stays over
    # On the rangefinder the climb is to the survey height itself.
    assert watching().update(at(8.0), 100.0).climb_to_m == pytest.approx(10.668)
    assert watching().update(at(7.0, "baro"), 100.0).climb_to_m == pytest.approx(10.668 + sh.BARO_MARGIN_M)  # ABORT too


def test_a_survey_height_the_guard_could_not_hold_on_either_sensor_is_refused():
    """Low: at its own height on the barometer (counted 0.5 m lower) the guard must not already be climbing, with
    the 0.3 m of a recovery's way back to spare: 8.22 + 0.3 + 0.5 = 9.02 m. High: the rangefinder reads to 12 m,
    and above its reach the guard would only ever have the barometer."""
    assert sh.MIN_CONFIGURED_HEIGHT_M == pytest.approx(9.02) and sh.MAX_CONFIGURED_HEIGHT_M == 11.5
    for bad in (8.3, 8.6, 9.0, 11.6, 12.0, 30.0):
        with pytest.raises(ValueError, match="survey_height_m"):
            sh.survey_height({"survey_height_m": bad})
    assert sh.survey_height({"survey_height_m": 9.05}) == 9.05 and sh.survey_height({"survey_height_m": 11.5}) == 11.5
    for height in (sh.DEFAULT_HEIGHT_M, sh.LOG_JUDGED_HEIGHT_M, 9.05):
        assert sh.SurveyHeightGuard(height).update(at(height, "baro"), 1.0).action == sh.CONTINUE  # at its own height, on either sensor


def test_the_guard_watches_from_its_first_tick_and_a_dip_on_the_barometer_is_acted_on():
    """There is no "not watching yet": a guard that could be silently off would read the same as one that is content.
    The mission asks it once the survey is under way (asked on the way up, it ends the survey: it fails closed)."""
    assert sh.SurveyHeightGuard(10.668).update(at(3.0), 100.0).action == sh.ABORT
    guard = sh.SurveyHeightGuard(10.668)
    assert guard.update(at(10.668, "baro"), 100.0).action == sh.CONTINUE  # flown at the survey height by the barometer
    assert guard.update(at(8.6, "baro"), 101.0).action == sh.RECOVER
    assert guard.update(at(8.2, "baro"), 102.0).action == sh.ABORT
    assert not hasattr(guard, "watching")


def test_a_time_that_is_not_one_is_refused_and_a_clock_that_steps_back_starts_the_wait_again():
    guard = sh.SurveyHeightGuard(10.668)
    for bad in (float("nan"), float("inf"), None, "now"):
        with pytest.raises(ValueError, match="time"):
            guard.update(at(10.6), bad)
    guard.update(at(10.6), 100.0)
    guard.update(None, 100.0)
    assert guard.update(None, 50.0).action == sh.CONTINUE  # the clock went back: the wait starts from here
    assert guard.update(None, 50.6).action == sh.RECOVER
    assert guard.update(None, 54.9).action == sh.RECOVER and guard.update(None, 55.1).action == sh.ABORT


def test_with_no_height_the_survey_waits_and_after_five_seconds_it_is_over():
    guard = sh.SurveyHeightGuard(10.668)
    guard.update(at(10.6), 100.0)
    assert sh.NO_HEIGHT_GRACE_S == 0.5 and sh.HEIGHT_STEADY_S == 1.0
    assert guard.update(None, 100.05).action == sh.CONTINUE  # a gap under half a second does not stop the survey
    assert guard.update(None, 100.5).action == sh.CONTINUE
    answer = guard.update(None, 100.55)
    assert answer.action == sh.RECOVER and answer.climb_to_m is None and "no height" in answer.reason  # wait: nothing to climb by
    assert guard.update(None, 104.9).action == sh.RECOVER
    over = guard.update(None, 105.1)
    assert over.action == sh.ABORT and over.climb_to_m is None  # still nothing to climb by: the mission's failure policy, not a blind climb
    # A height that comes back in time lets the survey go on, once it has stayed for a second.
    guard = watching()
    guard.update(None, 100.0)
    guard.update(None, 101.0)
    back = guard.update(at(10.6), 102.0)
    assert back.action == sh.RECOVER and back.climb_to_m is None and "stay" in back.reason
    assert guard.update(at(10.6), 102.9).action == sh.RECOVER and guard.update(at(10.6), 103.0).action == sh.CONTINUE
    # A short gap that the survey flew through leaves nothing behind: the next gap has its own half second.
    guard = watching()
    guard.update(None, 100.0)
    assert guard.update(at(10.6), 100.3).action == sh.CONTINUE
    assert guard.update(None, 100.7).action == sh.CONTINUE and guard.update(None, 101.3).action == sh.RECOVER


def test_a_height_that_comes_and_goes_is_no_height():
    """LOC-1's open point (a pose that comes and goes never reached the guard's five seconds): once the survey
    waits for a height, only a height that stays for a second ends the wait, and the five seconds are counted
    from the first tick without one."""
    guard = watching()
    t = 100.0
    actions = []
    while t < 105.2:  # no height for 0.6 s, then a height for 0.4 s, over and over
        height = None if (t - 100.0) % 1.0 < 0.6 else at(10.6)
        actions.append(guard.update(height, round(t, 2)).action)
        t += 0.1
    assert actions[:5] == [sh.CONTINUE] * 5 and set(actions[6:50]) == {sh.RECOVER}  # one wait, not a stop and a start each second
    assert actions[-1] == sh.ABORT and guard.aborted
    over = guard.update(at(10.6), 106.0)
    assert over.action == sh.ABORT and over.climb_to_m is None and "came and went" in over.reason
    # A height that is low is still climbed by while the wait lasts, and one under 7.77 m still ends the survey with its climb.
    guard = watching()
    guard.update(None, 100.0)
    guard.update(None, 100.6)
    low = guard.update(at(8.0), 100.7)
    assert low.action == sh.RECOVER and low.climb_to_m == pytest.approx(10.668)
    end = guard.update(at(7.5), 100.8)
    assert end.action == sh.ABORT and end.climb_to_m == pytest.approx(10.668)
    # In a recovery there is no half second: a tick without a height is a tick with nothing to climb by.
    guard = watching()
    guard.update(at(8.0), 100.0)
    assert guard.update(None, 100.05).action == sh.RECOVER


def test_the_guard_never_asks_for_a_height_under_the_rules_floor():
    for height in (10.668, 9.144, 11.5):
        assert sh.SurveyHeightGuard(height).survey_height_m >= safety_limits.SURVEY_MIN_HEIGHT_M
    with pytest.raises(ValueError, match="survey height"):
        sh.SurveyHeightGuard(8.0)


def test_a_recovery_begun_on_the_rangefinder_is_not_ended_by_the_barometer_which_cannot_see_the_ground():
    """Found in the sim (LOC-3's survey_dip): braking for the recovery tilted the vehicle past the rangefinder's
    15 degrees, the height was the barometer's for a moment, and the barometer, blind to the ground that had
    come close, ended the recovery 0.8 s after it began."""
    assert sh.RANGEFINDER_BACK_WAIT_S == 3.0
    guard = sh.SurveyHeightGuard(10.668)
    assert guard.update(at(10.668), 0.0).action == sh.CONTINUE
    assert guard.update(at(8.05), 1.0).action == sh.RECOVER
    waiting = guard.update(at(11.06, "baro"), 1.5)  # fine by the barometer: not believed yet
    assert waiting.action == sh.RECOVER and waiting.climb_to_m is None and "rangefinder" in waiting.reason
    assert guard.update(at(11.06, "baro"), 3.9).action == sh.RECOVER
    again = guard.update(at(8.4), 4.0)  # the rangefinder is back, and still low
    assert again.action == sh.RECOVER and again.climb_to_m == pytest.approx(10.668)
    assert guard.update(at(11.06, "baro"), 4.5).action == sh.RECOVER  # (and the wait starts again from here)
    assert guard.update(at(10.5), 5.0).action == sh.CONTINUE  # back on the rangefinder: the recovery ends
    assert guard.recoveries == 1


def test_a_rangefinder_that_stays_away_for_three_seconds_leaves_the_recovery_to_the_barometer():
    guard = sh.SurveyHeightGuard(10.668)
    guard.update(at(8.05), 0.0)
    assert guard.update(at(11.4, "baro"), 1.0).action == sh.RECOVER
    assert guard.update(at(11.4, "baro"), 3.9).action == sh.RECOVER
    assert guard.update(at(11.4, "baro"), 4.0).action == sh.CONTINUE  # three seconds of barometer only: judged on it
    # A barometer height that is not back keeps the recovery, with the barometer's climb.
    guard = sh.SurveyHeightGuard(10.668)
    guard.update(at(8.05), 0.0)
    guard.update(at(9.5, "baro"), 1.0)
    late = guard.update(at(9.5, "baro"), 4.0)
    assert late.action == sh.RECOVER and late.climb_to_m == pytest.approx(10.668 + sh.BARO_MARGIN_M)
    # During the wait the barometer can still end the survey, and a recovery begun on the barometer is judged on it at once.
    guard = sh.SurveyHeightGuard(10.668)
    guard.update(at(8.05), 0.0)
    assert guard.update(at(8.0, "baro"), 1.0).action == sh.ABORT
    guard = sh.SurveyHeightGuard(10.668)
    assert guard.update(at(8.5, "baro"), 0.0).action == sh.RECOVER
    assert guard.update(at(11.2, "baro"), 1.0).action == sh.CONTINUE
    # A clock that steps back starts the three seconds again.
    guard = sh.SurveyHeightGuard(10.668)
    guard.update(at(8.05), 10.0)
    guard.update(at(11.4, "baro"), 11.0)
    assert guard.update(at(11.4, "baro"), 5.0).action == sh.RECOVER and guard.update(at(11.4, "baro"), 7.9).action == sh.RECOVER
    assert guard.update(at(11.4, "baro"), 8.0).action == sh.CONTINUE
