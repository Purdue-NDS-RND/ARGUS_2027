"""GPS health (REC-6): is the GPS to be trusted, by everything else the vehicle knows. No ROS.

Behind the mission flag `ekf_source_switching`. `GpsHealth` is fed what comes
and asked for a rating: TRUSTED, SUSPECT or UNTRUSTED, with the reasons.
The feed hands the flow odometry's samples over only from under
VISION_MAX_HEIGHT_M, where that odometry has been shown good.
`SourceSwitch` is the rule for the flight controller's source set (set 2 on
UNTRUSTED, back to 1 only when the caller allows it and the rating is TRUSTED
again). `GpsHealthFeed` and `attach_gps_health` are the executor's side: the
raw fix, its velocity and quality from MAVROS and the flow odometry's samples.

What makes it UNTRUSTED (any one):

  a jump     the raw GPS position moved, over JUMP_WINDOW_S, by more than
             JUMP_M beyond what was flown in that time. "What was flown" is
             the vision velocity summed over the window when it has quality
             throughout, and the GPS's own velocity summed otherwise (a
             receiver that steps 5 m does not report 5 m/s);
  a drift    the GPS velocity and the vision velocity differ by more than
             DRIFT_MPS over DRIFT_WINDOW_S (vision with quality throughout,
             and the speed steady: while it changes, the receiver's own
             delay looks the same);
  the X      a target that stands still (the recovery's X or bucket, as the
             estimator tracks it) moves by more than STATIC_DRIFT_MPS over
             STATIC_WINDOW_S: the ground does not move, so the position it is
             placed with does;
  the EKF    the flight controller's own estimator says the GPS is glitching
             (its flag), when that is heard.

What makes it SUSPECT (not a reason to switch, a reason to look): a fix worse
than 3D, fewer than MIN_SATELLITES, HDOP over MAX_HDOP, no GPS heard for
STALE_S, the EKF's position or velocity variance over VARIANCE_SUSPECT, or
any of the three disagreements above at half its limit.

UNTRUSTED holds: the rating comes back to TRUSTED only after CLEAN_S with
nothing wrong, not even something SUSPECT. SUSPECT by itself lasts only as
long as what is doubtful does. `clean_for_s` says how long that has been;
going back to the first source set is the mission's choice, not this
module's.

Positions are metres east and north in the local frame (the raw fix placed
with the same origin as MAVROS's pose); velocities are metres a second east
and north. Times are seconds on one clock.

The standard library only.
"""

from __future__ import annotations

import math
import bisect
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Iterable, List, Optional, Tuple

from .features import get_feature_flag

FLAG = "ekf_source_switching"
TRUSTED, SUSPECT, UNTRUSTED = "TRUSTED", "SUSPECT", "UNTRUSTED"

JUMP_M = 2.5  # the spec's pass is a 5 m step: half of it is a jump
JUMP_WINDOW_S = 1.0
DRIFT_MPS = 0.3  # the spec's pass is a 0.5 m/s drift
DRIFT_WINDOW_S = 3.0
DRIFT_MAX_SPEED_CHANGE_MPS = 1.0  # the drift is judged in steady flight: a receiver's fix is a fifth of a second old, and that shows as a drift while the speed changes
STATIC_DRIFT_MPS = 0.3
STATIC_WINDOW_S = 3.0
CLEAN_S = 10.0  # the spec's: back only after 10 s clean
MIN_FIX_TYPE = 3
MIN_SATELLITES = 8
MAX_HDOP = 1.4
STALE_S = 1.0
VARIANCE_SUSPECT = 0.8  # the EKF's position or velocity variance, as its status gives them (1.0 is its own limit)
VISION_MIN_QUALITY = 50
VISION_MAX_GAP_S = 0.3  # vision samples no further apart than this are "throughout"
VISION_MAX_HEIGHT_M = 5.0  # the flow odometry is good to 0.05 m/s under this and to 0.3 to 0.5 m/s above it (REC-5's flights): above it, it is not something to judge a GPS by
HISTORY_S = 6.0
STAMP_STEP_BACK_S = 1.0  # a stamp this far behind the last one is a clock that was set, not a late message: what was kept is dropped
FLOW_SENT_MAX_AGE_S = 1.0  # source set 2 is "fed" while a flow sample with quality was sent to the flight controller this lately


@dataclass(frozen=True)
class Rating:
    state: str
    reasons: Tuple[str, ...]  # what is wrong now; empty when nothing is
    since_s: float  # when the state was entered
    clean_for_s: float  # how long nothing has been wrong (0 while something is)
    detected_at_s: Optional[float] = None  # when UNTRUSTED was last entered


@dataclass
class _Gps:
    t: float
    east: float
    north: float
    v_east: Optional[float]  # None: the receiver's velocity was not heard with this fix
    v_north: Optional[float]


def _mean(values: List[float]) -> float:
    return sum(values) / len(values)


class GpsHealth:
    def __init__(self) -> None:
        self._gps: Deque[_Gps] = deque()
        self._vision: Deque[Tuple[float, float, float]] = deque()  # (t, east m/s, north m/s), with quality only
        self._static: Deque[Tuple[float, float, float]] = deque()
        self._quality: Optional[Tuple[float, int, int, float]] = None  # (t, fix type, satellites, hdop)
        self._ekf: Optional[Tuple[float, bool, float, float]] = None  # (t, glitching, position variance, velocity variance)
        self._state, self._since_s = TRUSTED, 0.0
        self._last_wrong_s: Optional[float] = None
        self._detected_at_s: Optional[float] = None
        self._started_s: Optional[float] = None
        self._window_speed_change = 0.0
        self._newest_fix: Optional[float] = None  # the newest fix's own stamp, and when (by the caller's clock) it was first seen:
        self._newest_fix_seen_s = 0.0  # "no GPS lately" is judged by arrival, the windows by the fixes' own stamps, so the two clocks never meet
        self._no_velocity = False

    # --- what comes in ---------------------------------------------------------------

    def _trim(self, queue: Deque, now_s: float) -> None:
        while queue and (now_s - (queue[0].t if isinstance(queue[0], _Gps) else queue[0][0])) > HISTORY_S:
            queue.popleft()

    def add_gps(self, t_s: float, east_m: float, north_m: float, v_east_mps: Optional[float] = None, v_north_mps: Optional[float] = None) -> None:
        """A raw fix in metres, with the receiver's own velocity of the same moment when there is one (None: not heard)."""
        if not all(math.isfinite(value) for value in (t_s, east_m, north_m)):
            return
        if v_east_mps is None or v_north_mps is None or not (math.isfinite(v_east_mps) and math.isfinite(v_north_mps)):
            v_east_mps = v_north_mps = None
        if self._gps and t_s <= self._gps[-1].t:
            if self._gps[-1].t - t_s <= STAMP_STEP_BACK_S:
                return  # a late or repeated message
            self._gps.clear()  # the stamps stepped back: what was kept cannot be placed against what comes now
        self._gps.append(_Gps(float(t_s), float(east_m), float(north_m), None if v_east_mps is None else float(v_east_mps), None if v_north_mps is None else float(v_north_mps)))
        self._trim(self._gps, t_s)

    def add_fix_quality(self, t_s: float, fix_type: int, satellites: int, hdop: float) -> None:
        self._quality = (float(t_s), int(fix_type), int(satellites), float(hdop))

    def add_vision(self, t_s: float, v_east_mps: float, v_north_mps: float, quality: int) -> None:
        """A flow-odometry sample. One without quality is not kept: a gap in the samples is how "no vision" is known."""
        if quality < VISION_MIN_QUALITY or not all(math.isfinite(value) for value in (t_s, v_east_mps, v_north_mps)):
            return
        if self._vision and t_s <= self._vision[-1][0]:
            if self._vision[-1][0] - t_s <= STAMP_STEP_BACK_S:
                return
            self._vision.clear()
        self._vision.append((float(t_s), float(v_east_mps), float(v_north_mps)))
        self._trim(self._vision, t_s)

    def add_static_target(self, t_s: float, east_m: float, north_m: float) -> None:
        """Where a target that does not move is estimated to be (the recovery's X or the bucket on the ground)."""
        if not all(math.isfinite(value) for value in (t_s, east_m, north_m)) or (self._static and t_s <= self._static[-1][0]):
            return
        self._static.append((float(t_s), float(east_m), float(north_m)))
        self._trim(self._static, t_s)

    def forget_static_target(self) -> None:
        """The target is no longer standing still (it is on the hook), or another one is tracked."""
        self._static.clear()

    def add_ekf(self, t_s: float, gps_glitching: bool, position_variance: float, velocity_variance: float) -> None:
        self._ekf = (float(t_s), bool(gps_glitching), float(position_variance), float(velocity_variance))

    # --- the tests -------------------------------------------------------------------

    def _flown(self, window: List[_Gps]) -> Optional[Tuple[List[Tuple[float, float]], str]]:
        """What was flown from the window's first fix to each of its fixes, and by what that is known: vision when it covers the window throughout, the GPS's own velocity otherwise. None when there is neither."""
        start_s, end_s = window[0].t, window[-1].t
        inside = [sample for sample in self._vision if start_s - VISION_MAX_GAP_S <= sample[0] <= end_s + VISION_MAX_GAP_S]
        covered = (
            len(inside) >= 2
            and inside[0][0] <= start_s + VISION_MAX_GAP_S
            and inside[-1][0] >= end_s - VISION_MAX_GAP_S
            and all(later[0] - earlier[0] <= VISION_MAX_GAP_S for earlier, later in zip(inside, inside[1:]))
        )
        if not covered and any(fix.v_east is None for fix in window):
            return None  # no vision and no velocity of the receiver's: nothing says what was flown, and a jump is not judged against standing still
        samples = inside if covered else [(fix.t, fix.v_east, fix.v_north) for fix in window]
        times = [sample[0] for sample in samples]

        def velocity_at(t_s: float) -> Tuple[float, float]:
            """The velocity at a time, on the line between the samples either side of it (held at the ends)."""
            index = bisect.bisect_left(times, t_s)
            if index <= 0:
                return samples[0][1], samples[0][2]
            if index >= len(samples):
                return samples[-1][1], samples[-1][2]
            earlier, later = samples[index - 1], samples[index]
            share = (t_s - earlier[0]) / (later[0] - earlier[0])
            return earlier[1] + share * (later[1] - earlier[1]), earlier[2] + share * (later[2] - earlier[2])

        # Summed between the fixes' own times, with the samples' times as steps in between (trapezoids).
        flown, east, north = [(0.0, 0.0)], 0.0, 0.0
        for earlier, later in zip(window, window[1:]):
            steps = [earlier.t] + times[bisect.bisect_right(times, earlier.t) : bisect.bisect_left(times, later.t)] + [later.t]
            for a, b in zip(steps, steps[1:]):
                va, vb = velocity_at(a), velocity_at(b)
                east += 0.5 * (va[0] + vb[0]) * (b - a)
                north += 0.5 * (va[1] + vb[1]) * (b - a)
            flown.append((east, north))
        self._window_speed_change = max(math.hypot(a[1] - b[1], a[2] - b[2]) for a in samples for b in (samples[0], samples[-1]))
        return flown, "vision" if covered else "its own velocity"

    def _residuals(self, window_s: float) -> Optional[Tuple[List[Tuple[float, float, float]], str]]:
        """(t, east, north) of the GPS position less what was flown, for each fix of the window; None when the window is not filled.

        The window ends at the newest fix's own stamp, not at the caller's clock: the fixes are stamped by the
        flight controller's time as MAVROS places it, and an offset between the two must not empty the window.
        """
        if not self._gps:
            return None
        end_s = self._gps[-1].t
        window = [fix for fix in self._gps if fix.t >= end_s - window_s - 1e-9]
        if len(window) < 4 or window[-1].t - window[0].t < 0.7 * window_s:
            return None
        found = self._flown(window)
        if found is None:
            self._no_velocity = True
            return None
        flown, by = found
        return [(fix.t, fix.east - window[0].east - gone[0], fix.north - window[0].north - gone[1]) for fix, gone in zip(window, flown)], by

    def _jump_m(self) -> Optional[Tuple[float, str]]:
        """How far the GPS position moved beyond what was flown over the jump window (its last third against its first), and by what that was measured."""
        found = self._residuals(JUMP_WINDOW_S)
        if found is None:
            return None
        residuals, by = found
        third = max(1, len(residuals) // 3)
        first, last = residuals[:third], residuals[-third:]
        return math.hypot(_mean([r[1] for r in last]) - _mean([r[1] for r in first]), _mean([r[2] for r in last]) - _mean([r[2] for r in first])), by

    def _drift_mps(self) -> Optional[float]:
        """The rate at which the GPS position leaves what vision says was flown, over the drift window: a line fitted through the difference. None without vision throughout."""
        found = self._residuals(DRIFT_WINDOW_S)
        if found is None or found[1] != "vision":
            return None
        if self._window_speed_change > DRIFT_MAX_SPEED_CHANGE_MPS:
            return None  # speeding up or slowing down: the receiver's own delay looks like a drift then, and is not judged
        residuals = found[0]
        mean_t = _mean([r[0] for r in residuals])
        spread = sum((r[0] - mean_t) ** 2 for r in residuals)
        if spread <= 0.0:
            return None
        east = sum((r[0] - mean_t) * r[1] for r in residuals) / spread
        north = sum((r[0] - mean_t) * r[2] for r in residuals) / spread
        return math.hypot(east, north)

    def _static_mps(self, now_s: float) -> Optional[float]:
        window = [sample for sample in self._static if sample[0] >= now_s - STATIC_WINDOW_S - 1e-9]
        if len(window) < 3 or window[-1][0] - window[0][0] < 0.8 * STATIC_WINDOW_S:
            return None
        # The middle of the first third against the middle of the last: an estimate jitters, and a drift goes one way.
        third = max(1, len(window) // 3)
        start = (_mean([s[1] for s in window[:third]]), _mean([s[2] for s in window[:third]]), _mean([s[0] for s in window[:third]]))
        end = (_mean([s[1] for s in window[-third:]]), _mean([s[2] for s in window[-third:]]), _mean([s[0] for s in window[-third:]]))
        return math.hypot(end[0] - start[0], end[1] - start[1]) / max(end[2] - start[2], 1e-6)

    def rating(self, now_s: float) -> Rating:
        """The rating now. Call it every tick: UNTRUSTED is entered here, and the clean time is counted from here."""
        if self._started_s is None:
            self._started_s = self._since_s = float(now_s)
        untrusted: List[str] = []
        suspect: List[str] = []
        newest = self._gps[-1].t if self._gps else None
        if newest != self._newest_fix:
            self._newest_fix, self._newest_fix_seen_s = newest, float(now_s)
        fresh = newest is not None and now_s - self._newest_fix_seen_s <= STALE_S  # a fix came lately: old fixes are not judged again and again
        self._no_velocity = False
        jump = self._jump_m() if fresh else None
        if self._no_velocity:
            suspect.append("no GPS velocity and no vision to judge a jump by")
        if jump is not None:
            if jump[0] > JUMP_M:
                untrusted.append(f"the GPS position moved {jump[0]:.1f} m beyond what was flown in {JUMP_WINDOW_S:g} s (by {jump[1]})")
            elif jump[0] > 0.5 * JUMP_M:
                suspect.append(f"the GPS position moved {jump[0]:.1f} m beyond what was flown in {JUMP_WINDOW_S:g} s (by {jump[1]})")
        drift = self._drift_mps() if fresh else None
        if drift is not None:
            if drift > DRIFT_MPS:
                untrusted.append(f"the GPS position drifts {drift:.2f} m/s against vision")
            elif drift > 0.5 * DRIFT_MPS:
                suspect.append(f"the GPS position drifts {drift:.2f} m/s against vision")
        static = self._static_mps(now_s)
        if static is not None:
            if static > STATIC_DRIFT_MPS:
                untrusted.append(f"a target that stands still is moving {static:.2f} m/s")
            elif static > 0.5 * STATIC_DRIFT_MPS:
                suspect.append(f"a target that stands still is moving {static:.2f} m/s")
        if self._ekf is not None and now_s - self._ekf[0] <= STALE_S:
            if self._ekf[1]:
                untrusted.append("the flight controller's estimator says the GPS is glitching")
            if max(self._ekf[2], self._ekf[3]) > VARIANCE_SUSPECT:
                suspect.append(f"the estimator's variance is {max(self._ekf[2], self._ekf[3]):.2f}")
        if not fresh:
            suspect.append(f"no GPS position for {STALE_S:g} s" if self._gps else "no GPS position yet")
        if self._quality is not None and now_s - self._quality[0] <= 5.0:
            _t, fix_type, satellites, hdop = self._quality
            if fix_type < MIN_FIX_TYPE:
                suspect.append(f"the fix is type {fix_type}, under a 3D fix")
            if satellites < MIN_SATELLITES:
                suspect.append(f"{satellites} satellites, under {MIN_SATELLITES}")
            if math.isfinite(hdop) and hdop > MAX_HDOP:
                suspect.append(f"HDOP {hdop:.1f}, over {MAX_HDOP:g}")
        wrong = untrusted + suspect
        if wrong:
            self._last_wrong_s = float(now_s)
        clean_for = float(now_s) - (self._last_wrong_s if self._last_wrong_s is not None else self._started_s)
        state = self._state
        if untrusted:
            state = UNTRUSTED
        elif self._state == UNTRUSTED:
            state = UNTRUSTED if clean_for < CLEAN_S else TRUSTED  # it holds until nothing has been wrong for CLEAN_S
        else:
            state = SUSPECT if suspect else TRUSTED  # SUSPECT is for as long as something is doubtful; only UNTRUSTED holds
        if state != self._state:
            if state == UNTRUSTED:
                self._detected_at_s = float(now_s)
            self._state, self._since_s = state, float(now_s)
        return Rating(self._state, tuple(wrong), self._since_s, 0.0 if wrong else max(0.0, clean_for), self._detected_at_s)


# --- the source set ------------------------------------------------------------------------------------------

SET_EKF_SOURCE_SET = 42007  # MAV_CMD_SET_EKF_SOURCE_SET: param1 is the source set, 1 to 3
GPS_SET, NO_GPS_SET = 1, 2
COMMAND_RETRY_S = 1.0  # an unanswered or refused command is sent again this often
COMMAND_TRIES = 5  # ... this many times; after that which set the flight controller is on is not known, and it is asked more slowly
COMMAND_SLOW_RETRY_S = 5.0


class SourceSwitch:
    """Which source set the flight controller is on, as far as its acknowledgements say (no message reports it), and when to ask for another.

    The first thing asked for is set 1, whatever the rating: a flight controller left on set 2 by an earlier run
    is put back before anything is flown on GPS, and `confirmed` says an answer has been heard.

    UNTRUSTED asks for set 2 only when the caller says that set is being fed (`no_gps_set_fed`: optical flow is
    reaching the flight controller). Unfed, set 2 is dead reckoning and the command would still be acknowledged:
    it is not sent, the flight controller stays on set 1, and the caller flies without the GPS's word.
    Set 1 is asked for again only when the rating is TRUSTED (which the monitor gives back only after CLEAN_S
    clean) and the caller says it may be (`may_go_back`): that is the mission's choice.

    A command is sent again every COMMAND_RETRY_S until it is acknowledged. After COMMAND_TRIES without an answer
    the set is not known (`acknowledged` None: a command may have been taken and its answer lost), that is said
    once, and the command goes on being sent every COMMAND_SLOW_RETRY_S.
    """

    def __init__(self) -> None:
        self.acknowledged: Optional[int] = GPS_SET  # what the flight controller starts on, until it answers; None: not known
        self.confirmed = False  # an accepted answer has been heard
        self.wanted = GPS_SET
        self.failed = False
        self._asked_s: Optional[float] = None
        self._tries = 0
        self._declined = False
        self.switched_at_s: Optional[float] = None  # when set 2 was last acknowledged

    def update(self, rating: Rating, now_s: float, may_go_back: bool = False, no_gps_set_fed: bool = False) -> Tuple[Optional[int], Optional[str]]:
        """(the source set to command now, or None; a line for the log, or None). Call every tick."""
        wanted, say = self.wanted, None
        why = "; ".join(rating.reasons) if rating.reasons else f"GPS clean for {rating.clean_for_s:.0f} s"
        if rating.state == UNTRUSTED:
            if no_gps_set_fed or self.wanted == NO_GPS_SET:
                wanted = NO_GPS_SET
            elif not self._declined:
                self._declined = True
                say = f"GPS UNTRUSTED: source set {NO_GPS_SET} is not asked for, nothing is feeding it (no optical flow is being sent to the flight controller); it stays on set {GPS_SET} ({why})"
        else:
            self._declined = False
            if rating.state == TRUSTED and may_go_back and (self.acknowledged != GPS_SET or self.wanted != GPS_SET):
                wanted = GPS_SET
        if wanted != self.wanted:
            if self._tries > 0 and self.acknowledged != self.wanted:
                self.acknowledged = None  # commands for the other set went out unanswered: one may have been taken, so the set is not known
            self.wanted, self._tries, self._asked_s, self.failed = wanted, 0, None, False
            say = f"GPS {rating.state}: asking the flight controller for source set {wanted} ({why})"
        if self.wanted == self.acknowledged and self.confirmed:
            return None, say
        if self._asked_s is not None and now_s - self._asked_s < (COMMAND_SLOW_RETRY_S if self.failed else COMMAND_RETRY_S):
            return None, say
        if self._tries == 0 and say is None:
            say = f"making sure the flight controller is on source set {self.wanted}"  # the first command of a run, or one after an answer for another set
        if self._tries >= COMMAND_TRIES and not self.failed:
            self.failed, self.acknowledged = True, None
            say = f"source set {self.wanted} was not acknowledged after {COMMAND_TRIES} tries: which set the flight controller is on is not known; asking every {COMMAND_SLOW_RETRY_S:g} s"
        self._tries, self._asked_s = self._tries + 1, float(now_s)
        return self.wanted, say

    def answered(self, source_set: int, accepted: bool, now_s: float) -> Optional[str]:
        """The flight controller's answer to a command for `source_set`. Returns a line for the log when something is learnt.

        An accepted answer is where the flight controller now is, whether or not that set is still the one wanted:
        a fault that arrives between asking for set 1 and its answer leaves the flight controller on set 1, and the
        next `update` asks for set 2 again because the acknowledged set is not the wanted one. A refusal says
        nothing about where it is.
        """
        if not accepted:
            return None
        news = source_set != self.acknowledged or not self.confirmed
        self.acknowledged, self.confirmed, self.failed = int(source_set), True, False
        if source_set == NO_GPS_SET and news:
            self.switched_at_s = float(now_s)
        if source_set != self.wanted:
            self._tries, self._asked_s = 0, None  # the wish changed while this was on its way: ask for the wanted set at once
            return f"source set {source_set} acknowledged by the flight controller, and set {self.wanted} is the one wanted now: asking again"
        return f"source set {source_set} acknowledged by the flight controller" if news else None

    def leave(self, rating_state: str, now_s: float) -> Tuple[Optional[int], Optional[str]]:
        """The mission ends. (a last command, or None; a line for the log, or None).

        Whatever flies next flies on GPS without asking this monitor, so a flight controller that is off set 1
        (or not known to be on it) is asked for set 1, unless the GPS is UNTRUSTED right now: then it is left
        where it is and that is said.
        """
        if self.acknowledged == GPS_SET and self.wanted == GPS_SET and self.confirmed:
            return None, None
        where = "a source set that is not known" if self.acknowledged is None else f"source set {self.acknowledged}"
        if rating_state == UNTRUSTED:
            return None, f"the mission ends with the flight controller on {where} and the GPS UNTRUSTED: it is left there, and nothing after this should be flown on GPS"
        self.wanted, self._tries, self._asked_s = GPS_SET, 1, float(now_s)
        return GPS_SET, f"the mission ends with the flight controller on {where}: asking for source set {GPS_SET} (once: nothing asks again after this)"


# --- the executor's side -------------------------------------------------------------------------------------

RAW_FIX_TOPIC = "/mavros/global_position/raw/fix"
RAW_VELOCITY_TOPIC = "/mavros/global_position/raw/gps_vel"
RAW_QUALITY_TOPIC = "/mavros/gpsstatus/gps1/raw"
FLOW_STATUS_TOPIC = "/argus/flow/status"  # drone_target_cv/flow_odometry_node.py STATUS_TOPIC (a test holds the two together)
METRES_PER_DEGREE = 111_319.49


class GpsHealthFeed:
    """The monitor fed from messages' own values: the raw fix placed in metres from the first fix heard, its velocity, its quality, the flow's samples."""

    def __init__(self) -> None:
        self.health = GpsHealth()
        self._origin: Optional[Tuple[float, float]] = None
        self._velocity: Optional[Tuple[float, float, float]] = None  # (t, east, north)
        self._flow_sent_s: Optional[float] = None  # the stamp of the last flow sample with quality that was sent to the flight controller

    def on_fix(self, t_s: float, latitude: float, longitude: float, has_fix: bool = True) -> None:
        if not has_fix or not (math.isfinite(latitude) and math.isfinite(longitude)) or (latitude == 0.0 and longitude == 0.0):
            return
        if self._origin is None:
            self._origin = (float(latitude), float(longitude))
        north = (latitude - self._origin[0]) * METRES_PER_DEGREE
        east = (longitude - self._origin[1]) * METRES_PER_DEGREE * math.cos(math.radians(self._origin[0]))
        # The receiver's own velocity of about the same moment; none heard lately is "no velocity", and a jump is then judged by vision or not at all.
        velocity = (None, None) if self._velocity is None or abs(t_s - self._velocity[0]) > STALE_S else (self._velocity[1], self._velocity[2])
        self.health.add_gps(t_s, east, north, velocity[0], velocity[1])

    def on_velocity(self, t_s: float, v_east_mps: float, v_north_mps: float) -> None:
        if math.isfinite(v_east_mps) and math.isfinite(v_north_mps):
            self._velocity = (float(t_s), float(v_east_mps), float(v_north_mps))

    def on_quality(self, t_s: float, fix_type: int, satellites: int, eph_cm: int) -> None:
        """GPS_RAW_INT's fix type, satellites and eph (HDOP times 100; 65535 is "not known")."""
        self.health.add_fix_quality(t_s, int(fix_type), int(satellites), math.nan if int(eph_cm) >= 65535 else int(eph_cm) / 100.0)

    def on_flow_status(self, text: str) -> None:
        """A flow odometry sample (its status topic's JSON)."""
        import json

        try:
            sample = json.loads(text)
            if not isinstance(sample, dict):
                return  # not the flow odometry's status: nothing of it is used, and nothing is raised into the executor's spin
            if sample.get("sent") is True and int(sample["quality"]) >= VISION_MIN_QUALITY:
                self._flow_sent_s = int(sample["stamp_ns"]) * 1.0e-9
            if float(sample["height_m"]) > VISION_MAX_HEIGHT_M:
                return  # too high for the flow to be the judge: a gap in the samples, and the tests fall back as they do without vision
            self.health.add_vision(int(sample["stamp_ns"]) * 1.0e-9, float(sample["east_mps"]), float(sample["north_mps"]), int(sample["quality"]))
        except (ValueError, KeyError, TypeError, AttributeError):
            return

    def rating(self, now_s: float) -> Rating:
        return self.health.rating(now_s)

    def flow_sent(self, now_s: float) -> bool:
        """Whether the flow odometry is sending the flight controller flow with quality now: what source set 2 flies on.

        It is the companion's side only: that the flight controller's set 2 takes it (EK3_SRC2_VELXY 5, FLOW_TYPE 5)
        is the ground check's (tools/fc_param_check.py). Both stamps are the companion's own clock.
        """
        return self._flow_sent_s is not None and abs(now_s - self._flow_sent_s) <= FLOW_SENT_MAX_AGE_S


def attach_gps_health(node: Any, specs: Iterable[Any]) -> Optional[GpsHealthFeed]:
    """The monitor fed by `node`'s own subscriptions, or None when no mission has the flag on (then nothing is subscribed)."""
    if not any(get_feature_flag(spec.features, FLAG) for spec in specs):
        return None
    from geometry_msgs.msg import TwistStamped
    from mavros_msgs.msg import GPSRAW
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String

    feed = GpsHealthFeed()

    def seconds(stamp: Any) -> float:
        return int(stamp.sec) + int(stamp.nanosec) * 1.0e-9

    node._gps_health_subscriptions = [
        node.create_subscription(NavSatFix, RAW_FIX_TOPIC, lambda m: feed.on_fix(seconds(m.header.stamp), m.latitude, m.longitude, getattr(m.status, "status", 0) >= 0), qos_profile_sensor_data),
        node.create_subscription(TwistStamped, RAW_VELOCITY_TOPIC, lambda m: feed.on_velocity(seconds(m.header.stamp), m.twist.linear.x, m.twist.linear.y), qos_profile_sensor_data),
        node.create_subscription(GPSRAW, RAW_QUALITY_TOPIC, lambda m: feed.on_quality(seconds(m.header.stamp), m.fix_type, m.satellites_visible, m.eph), qos_profile_sensor_data),
        node.create_subscription(String, FLOW_STATUS_TOPIC, lambda m: feed.on_flow_status(m.data), 20),
    ]
    node.get_logger().info(f"GPS health on ({FLAG}): {RAW_FIX_TOPIC}, {RAW_VELOCITY_TOPIC}, {RAW_QUALITY_TOPIC} and the flow odometry's {FLOW_STATUS_TOPIC}")
    return feed
