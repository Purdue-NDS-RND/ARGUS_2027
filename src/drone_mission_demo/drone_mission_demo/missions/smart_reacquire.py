"""Smart re-acquisition (CEN-06): what a mission does in TARGET_NOT_FOUND with the `smart_reacquire` flag on.

In place of the 2026 recovery (a climb of `not_found_ascent_m` from where
the vehicle is, on a GPS point): the vehicle goes to the height from which
the camera sees three sigma of the lost track plus 2.5 m
(drone_mission_core/reacquire.py), over the track's last estimate, and holds
there until a track of the target is CONFIRMED again, for at most 10 s.

  - The track is the one the mission's centring was flying to, with its
    estimate and sigma as the estimator has them now (a LOST track is kept
    for a while, moves with a position reset, and its sigma grows). Nothing
    is remembered by the mission itself.
  - The height is above the ground under the vehicle (the mission's
    `height_agl`), not above home. The vehicle moves at most 2.5 m sideways
    toward the estimate; what is left of the distance is added to what the
    footprint has to cover, so a far estimate is looked for from higher.
  - The mission does not wait its own `target_loss_grace_s` once the
    estimator says the track is LOST or gone: the estimator's two seconds
    are the grace (track_is_lost, which the mission's grace asks).
  - The 10 s count from arriving: no more than 0.1 m under the height and
    within ARRIVAL_RADIUS_M of the look point. A vehicle still within the
    mission's `recovery_alt_tolerance_m` of the height when the climb's time
    is up takes its look from there. A track CONFIRMED again before that does not end the climb:
    the look is from the height. A climb that does not arrive in
    reacquire.climb_limit_s fails.
  - "CONFIRMED again": the class's primary track, nearest the last estimate,
    is CONFIRMED and has been hit within the time a descent asks. It may be
    the same track found again or a new one; the centring takes it.
  - Each look is one of the mission's `max_recovery_attempts`, and the
    height never goes over `max_recovery_altitude_m` or under the vehicle's
    own.
  - With no track to look for (nothing was ever steered on, or the
    estimator has dropped it), there is no estimate to look round: update()
    returns None and the mission flies its 2026 recovery.

The answer is the base mission's own RecoveryCommandResult, so the missions'
TARGET_NOT_FOUND handlers are the 2026 ones: "reached" sends them to
ACQUIRE_TARGET, "failed" to the failure policy.

No ROS here: only the mission and its context are used.
"""

from __future__ import annotations

import math
from typing import Any, Optional, Tuple

from drone_mission_core import reacquire

ARRIVAL_RADIUS_M = 1.0  # this close to the last estimate, horizontally, is over it
CONFIRMED, LOST = "CONFIRMED", "LOST"


class SmartReacquire:
    def __init__(self, mission: Any, half_footprint_per_m: float) -> None:
        self._mission = mission
        self._half_footprint_per_m = float(half_footprint_per_m)
        self._point: Optional[Tuple[float, float]] = None  # the last estimate, east and north
        self._started_s: Optional[float] = None
        self._arrived_s: Optional[float] = None
        self._climb_limit_s = reacquire.CLIMB_BASE_S

    def track_is_lost(self, context: Any) -> bool:
        """Whether the track the mission was flying to is LOST by the estimator's own rule, or gone."""
        centring = self._mission._estimator_centring
        if centring.track_id is None:
            return False  # never had one: the mission's own grace applies
        track = context.tracked_target(centring.class_id, centring.track_id)
        return track is None or track.state == LOST

    def _last_estimate(self, context: Any) -> Optional[Tuple[float, float, float]]:
        """(east, north, sigma) of the track to look for, as the estimator has it now, or None when it has none.

        The track the mission was flying to. With none being flown to (the
        centring let its track go at the last look and has not taken another),
        the class's primary track nearest the last look point, whatever its
        state. Nothing remembered by the mission is used: the estimator's
        tracks move with a position reset and their sigma grows while unseen.
        """
        centring = self._mission._estimator_centring
        if centring.track_id is not None:
            track = context.tracked_target(centring.class_id, centring.track_id)
        elif self._point is not None:
            track = context.primary_target(centring.class_id, near=self._point)
        else:
            track = None  # nothing was ever steered on
        if track is None:
            return None
        values = (track.position[0], track.position[1], track.sigma_m)
        if not all(math.isfinite(float(value)) for value in values):
            return None
        return float(track.position[0]), float(track.position[1]), max(0.0, float(track.sigma_m))

    def _start(self, context: Any, now_s: float, estimate: Tuple[float, float, float]):
        from .red_bullseye_mission_base import RecoveryCommandResult

        mission = self._mission
        if mission._recovery_attempts >= mission._max_recovery_attempts:
            context.logger.error(f"[{mission.name}] Target recovery exceeded {mission._max_recovery_attempts} attempts")
            return RecoveryCommandResult(failed=True, reached_altitude=False)
        east, north, sigma = estimate
        position = context.local_pose.pose.position
        up = float(position.z)
        # The footprint is on the ground under the vehicle: the height asked is above that ground, by the
        # mission's height above it (the rangefinder's, or the barometer's with the known ground offset).
        # With no such height, the height above home stands in.
        height_now = context.height_agl(getattr(mission, "_ground_below_home_m", 0.0))
        above_ground = up if height_now is None else float(height_now.value_m)
        point, left_m = reacquire.look_point((float(position.x), float(position.y)), (east, north))
        ceiling = mission._max_recovery_altitude_m
        wanted = reacquire.wanted_height(sigma, self._half_footprint_per_m, left_m)
        look_from = reacquire.reacquire_height(
            sigma, floor_m=above_ground, ceiling_m=above_ground + max(0.0, ceiling - up), half_footprint_per_m=self._half_footprint_per_m, offset_m=left_m
        )
        target_up = up + (look_from - above_ground)
        mission._estimator_centring.forget()  # the next track taken is the one found from up there
        mission._recovery_target_altitude = target_up
        mission._recovery_hold_position = (context.global_gps.latitude, context.global_gps.longitude)
        mission._recovery_attempts += 1
        self._point, self._started_s, self._arrived_s = point, now_s, None
        self._climb_limit_s = reacquire.climb_limit_s(target_up - up)
        in_view = f"{reacquire.SIGMAS:g} x {sigma:.2f} m + {reacquire.MARGIN_M:g} m"
        where = "over its last estimate"
        if left_m > 0.0:
            in_view += f" + {left_m:.1f} m"
            where = f"{reacquire.MAX_SHIFT_M:g} m toward its last estimate, which is {left_m:.1f} m further"
        note = ""
        if look_from < wanted:
            note = f"; {wanted:.1f} m would show it all, the ceiling is {ceiling:.1f} m above home"
        elif look_from > wanted:
            note = f"; already above the {wanted:.1f} m that shows it all"
        context.logger.info(
            f"[{mission.name}] Target lost, looking again from {look_from:.1f} m above the ground {where} "
            f"({in_view} in view{note}; attempt {mission._recovery_attempts}/{mission._max_recovery_attempts})"
        )
        return None

    def update(self, context: Any):
        """One tick of TARGET_NOT_FOUND: the setpoint, and the base mission's RecoveryCommandResult. None: fly the 2026 recovery."""
        from .red_bullseye_mission_base import HOLD_YAW_DEG, RecoveryCommandResult

        mission = self._mission
        now_s = context.now().nanoseconds * 1.0e-9
        if mission._recovery_target_altitude is None or self._point is None:
            estimate = self._last_estimate(context)
            if estimate is None:
                self._point = None
                return None
            refused = self._start(context, now_s, estimate)
            if refused is not None:
                return refused

        height = mission._recovery_target_altitude
        context.set_local_position_setpoint(self._point[0], self._point[1], height, yaw_deg=HOLD_YAW_DEG)
        position = context.local_pose.pose.position
        over = math.hypot(float(position.x) - self._point[0], float(position.y) - self._point[1]) <= ARRIVAL_RADIUS_M
        # From the height itself: the mission's climb tolerance would take the look from half a metre under it.
        at_height = height - reacquire.ARRIVAL_SHORT_M <= float(position.z) <= height + mission._recovery_alt_tolerance_m
        if self._arrived_s is None and over and at_height:
            self._arrived_s = now_s
            context.logger.info(
                f"[{mission.name}] at the height to look from ({float(position.z):.1f} m above home): holding up to {reacquire.HOLD_S:g} s "
                "for the track to be CONFIRMED again"
            )
        if self._arrived_s is None and now_s - self._started_s >= self._climb_limit_s:
            if over and abs(float(position.z) - height) <= mission._recovery_alt_tolerance_m:
                # The climb's time is up and the vehicle has settled a little low: the look is taken from here.
                self._arrived_s = now_s
                context.logger.warn(
                    f"[{mission.name}] looking from {float(position.z):.1f} m above home, {height - float(position.z):.1f} m short of the height: "
                    f"holding up to {reacquire.HOLD_S:g} s for the track to be CONFIRMED again"
                )
        if self._arrived_s is None:
            if now_s - self._started_s >= self._climb_limit_s:
                context.logger.error(
                    f"[{mission.name}] did not reach the height to look from ({height:.1f} m above home) within {self._climb_limit_s:.0f} s; "
                    "giving up the look"
                )
                return RecoveryCommandResult(failed=True, reached_altitude=False)
            return RecoveryCommandResult(failed=False, reached_altitude=False)

        centring = mission._estimator_centring
        track = context.primary_target(centring.class_id, near=self._point)
        if track is not None and track.state == CONFIRMED and context.may_descend(centring.class_id, track_id=track.track_id):
            context.logger.info(
                f"[{mission.name}] {centring.class_id} track {track.track_id} CONFIRMED again after {now_s - self._arrived_s:.1f} s at the height"
            )
            return RecoveryCommandResult(failed=False, reached_altitude=True)
        if now_s - self._arrived_s >= reacquire.HOLD_S:
            state = "no track" if track is None else f"track {track.track_id} is {track.state}"
            context.logger.error(f"[{mission.name}] the target was not CONFIRMED again within {reacquire.HOLD_S:g} s at the height ({state})")
            return RecoveryCommandResult(failed=True, reached_altitude=False)
        return RecoveryCommandResult(failed=False, reached_altitude=False)
