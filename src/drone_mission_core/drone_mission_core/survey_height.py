"""The survey's height, and the guard that keeps it above the rules' floor (LOC-1).

The rules' floor for the localisation survey is 25 ft (7.62 m,
safety_limits.SURVEY_MIN_HEIGHT_M): a survey flown below it loses points,
and below 20 ft the mission scores nothing.

Height. survey_height(config) is what a survey mission flies at: 35 ft
(10.668 m) by default; 30 ft (9.144 m), for a sharper view, when the
sequence says the organisers judge the height from our own flight log
(`survey_height_judged_from_log: true`); or the sequence's own
`survey_height_m`, between MIN_CONFIGURED_HEIGHT_M (9.02 m: at its own
height on the barometer the guard must not already be climbing) and
MAX_CONFIGURED_HEIGHT_M (11.5 m: inside the rangefinder's reach, so the
guard has a height above this ground to watch).

Guard. SurveyHeightGuard.update(height, now) is asked every tick of the
survey with `context.height_agl()`. It watches from its first tick: the
mission asks it once the survey is under way, at its height. Asked on the
way up, it ends the survey; it has no "not watching yet" that could be
mistaken for "content":

  CONTINUE  the survey goes on.
  RECOVER   the height is under 8.22 m: the survey waits where it is and the
            vehicle climbs back to the survey height. It stays RECOVER until
            the height is within BACK_WITHIN_M of the survey height, so the
            survey does not start and stop about the line.
  ABORT     the height has been under 7.77 m: the survey is over, for good,
            and the climb back comes before anything else. The mission ends
            with what it has found.

A height that is not the rangefinder's (the barometer's: above home, not
above this ground) counts for BARO_MARGIN_M less than it says, and the climb
asked for on such a height is the survey height plus that margin. With no
height at all the survey waits (RECOVER, with nothing to climb by) and after
NO_HEIGHT_ABORT_S it is over, still with nothing to climb by: that is the
mission's failure policy's to handle, not a blind climb.

The barometer's 0.5 m is an allowance, not knowledge: over ground more than
that above home (beyond the ground offset the mission passes to
`height_agl`) a survey on the barometer alone is lower than it counts.

The mission that flies the survey (LOC-3's search_pattern) acts on the
answer; nothing here commands anything. No ROS; the standard library only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Optional

from .safety_limits import FEET_TO_METERS, SURVEY_MIN_HEIGHT_M, survey_height_command

DEFAULT_HEIGHT_M = 35.0 * FEET_TO_METERS  # 10.668 m
LOG_JUDGED_HEIGHT_M = 30.0 * FEET_TO_METERS  # 9.144 m: only once the organisers confirm the height is judged from our log
RECOVER_BELOW_M = 8.22
ABORT_BELOW_M = 7.77
BACK_WITHIN_M = 0.3  # a recovery is over this close under the survey height
BARO_MARGIN_M = 0.5  # a height that is not the rangefinder's counts for this much less
NO_HEIGHT_ABORT_S = 5.0
# A gap in the height shorter than this does not stop a survey that is not already waiting: half a second ago the
# height was above the guard's lines. Once the survey waits for a height, only one that has stayed this long ends
# the wait, and the five seconds above run from the first tick without one: a height that comes and goes is no height.
NO_HEIGHT_GRACE_S = 0.5
HEIGHT_STEADY_S = 1.0
# A recovery that began on the rangefinder's height is not ended by another source's for this long: the barometer
# cannot see ground that has come close, and the stop for the recovery can tilt the vehicle past the rangefinder's
# limit for a moment. After it the rangefinder is taken to be gone, and the height there is, is judged.
RANGEFINDER_BACK_WAIT_S = 3.0
# A sequence's own survey height: at it, on the barometer, the guard must be content with a recovery's way back to
# spare; and it must be inside the rangefinder's reach (the TFmini reads to 12 m).
MIN_CONFIGURED_HEIGHT_M = RECOVER_BELOW_M + BACK_WITHIN_M + BARO_MARGIN_M  # 9.02 m
MAX_CONFIGURED_HEIGHT_M = 11.5

CONTINUE, RECOVER, ABORT = "continue", "recover", "abort"
RANGEFINDER = "rangefinder"


def _number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _checked_height(height: Any, what: str) -> float:
    if not _number(height) or not MIN_CONFIGURED_HEIGHT_M <= height <= MAX_CONFIGURED_HEIGHT_M:
        raise ValueError(
            f"{what} must be a height from {MIN_CONFIGURED_HEIGHT_M:g} m (under it the guard, which climbs back under "
            f"{RECOVER_BELOW_M:g} m, could not hold it on the barometer) to {MAX_CONFIGURED_HEIGHT_M:g} m (the rangefinder's reach), got {height!r}"
        )
    return survey_height_command(float(height))


def survey_height(config: Mapping[str, Any]) -> float:
    """The height above the ground a survey flies at, from a mission's config."""
    judged = config.get("survey_height_judged_from_log", False)
    if not isinstance(judged, bool):
        raise ValueError(f"survey_height_judged_from_log must be true or false, got {judged!r}")
    if "survey_height_m" not in config:
        return LOG_JUDGED_HEIGHT_M if judged else DEFAULT_HEIGHT_M
    return _checked_height(config["survey_height_m"], "survey_height_m")


@dataclass(frozen=True)
class GuardAnswer:
    action: str  # CONTINUE, RECOVER or ABORT
    climb_to_m: Optional[float]  # the height to climb back to, by the source the height came from; None when there is nothing to climb by
    reason: str  # empty for CONTINUE


class SurveyHeightGuard:
    def __init__(self, survey_height_m: float) -> None:
        self.survey_height_m = _checked_height(survey_height_m, "the survey height")
        self.recoveries = 0
        self.aborted = False
        self._recovering = False
        self._by_rangefinder = False  # this recovery began on, or has since seen, a low rangefinder height
        self._other_since_s: Optional[float] = None  # since when the height has not been the rangefinder's, in such a recovery
        self._abort: Optional[GuardAnswer] = None
        self._no_height_since_s: Optional[float] = None
        self._waiting_for_height = False  # a gap outlasted its grace: the survey waits until a height has stayed
        self._height_since_s: Optional[float] = None  # since when, in that wait, there has been a height on every tick

    def _end(self, climb_to_m: Optional[float], reason: str) -> GuardAnswer:
        self.aborted = True
        self._abort = GuardAnswer(ABORT, climb_to_m, reason)
        return self._abort

    def update(self, height: Any, now_s: float) -> GuardAnswer:
        """One tick. `height` is a HeightAgl (value_m, source) or None; `now_s` the mission's clock in seconds."""
        if not _number(now_s):
            raise ValueError(f"the guard's time must be a number of seconds, got {now_s!r}")
        if self._abort is not None:
            return self._abort
        now = float(now_s)
        if height is None or not _number(getattr(height, "value_m", None)):
            self._height_since_s = None
            if self._no_height_since_s is None or now < self._no_height_since_s:
                self._no_height_since_s = now  # (a clock that stepped back starts the wait again)
            gone_s = now - self._no_height_since_s
            if gone_s >= NO_HEIGHT_ABORT_S:
                return self._end(None, f"no height above the ground for {NO_HEIGHT_ABORT_S:g} s: the survey cannot be kept above {SURVEY_MIN_HEIGHT_M:g} m blind")
            if gone_s < NO_HEIGHT_GRACE_S and not self._waiting_for_height and not self._recovering:
                return GuardAnswer(CONTINUE, None, "")
            self._waiting_for_height = True
            return GuardAnswer(RECOVER, None, "no height above the ground: the survey waits")
        if not self._waiting_for_height:
            self._no_height_since_s = None
        else:
            if self._height_since_s is None or now < self._height_since_s:
                self._height_since_s = now
            if now - self._height_since_s >= HEIGHT_STEADY_S:
                self._waiting_for_height = False
                self._no_height_since_s = None
                self._height_since_s = None
            elif self._no_height_since_s is not None and now - self._no_height_since_s >= NO_HEIGHT_ABORT_S:
                return self._end(
                    None, f"the height above the ground came and went for {NO_HEIGHT_ABORT_S:g} s: the survey cannot be kept above {SURVEY_MIN_HEIGHT_M:g} m on it"
                )
        source = str(getattr(height, "source", ""))
        margin = 0.0 if source == RANGEFINDER else BARO_MARGIN_M
        counted = float(height.value_m) - margin
        # The climb is asked for on the height the mission has: on the barometer that is the survey height
        # plus the margin, so that the counted height comes back to the survey height and the recovery can end.
        climb_to = self.survey_height_m + margin
        said = f"{float(height.value_m):.2f} m on the {source or 'unknown source'}"
        if source != RANGEFINDER:
            said += f" (counted as {counted:.2f} m)"
        if counted < ABORT_BELOW_M:
            return self._end(climb_to, f"{said} is under {ABORT_BELOW_M:g} m: the survey is over, climbing back to {climb_to:.2f} m")
        if source == RANGEFINDER:
            self._other_since_s = None
        if counted < RECOVER_BELOW_M:
            if not self._recovering:
                self._recovering = True
                self._by_rangefinder = False
                self.recoveries += 1
            self._by_rangefinder = self._by_rangefinder or source == RANGEFINDER
            return GuardAnswer(RECOVER, climb_to, f"{said} is under {RECOVER_BELOW_M:g} m: climbing back to {climb_to:.2f} m")
        if self._recovering:
            if self._by_rangefinder and source != RANGEFINDER:
                if self._other_since_s is None or float(now_s) < self._other_since_s:
                    self._other_since_s = float(now_s)
                if float(now_s) - self._other_since_s < RANGEFINDER_BACK_WAIT_S:
                    return GuardAnswer(
                        RECOVER, None, f"{said}, but the rangefinder saw the ground close and is not giving the height: waiting up to {RANGEFINDER_BACK_WAIT_S:g} s for it"
                    )
                self._by_rangefinder = False  # the rangefinder has stayed away: the height there is, is judged
            if counted < self.survey_height_m - BACK_WITHIN_M:
                return GuardAnswer(RECOVER, climb_to, f"{said}: climbing back to {climb_to:.2f} m")
            self._recovering = False
            self._by_rangefinder = False
            self._other_since_s = None
        if self._waiting_for_height:
            return GuardAnswer(RECOVER, None, f"{said}: the height is back, waiting {HEIGHT_STEADY_S:g} s for it to stay")
        return GuardAnswer(CONTINUE, None, "")
