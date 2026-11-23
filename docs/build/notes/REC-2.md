# REC-2: `package_recovery` mission

Flags: `package_recovery` (mission flag; off: the mission is skipped), `swing_shaping`, `recovery_gps_untrusted` · Status: PARTIAL · Commits: `git log --oneline --grep '^REC-2:' platform/build-2027`

## What it solves

The recovery had rules (the cone, the grip, the return) and nothing that flew by them. This is the mission: from the pickup's coordinates to the bucket set down at the return point, with no human input.

## Architecture

- `drone_mission_core/recovery_flight.py` (new, no ROS): `RecoveryFlight(plan).update(Readings) -> Command`, 15 states and three side states; `plan_from_config`, `expanding_square`, `HomeHeight`.
- `drone_mission_demo/missions/package_recovery_mission.py` (new): the context read into `Readings`, the `Command` handed back (local or global setpoint, gripper with its attempt, LAND).
- Executor, behind the flag: the throttle feed and `/mavros/global_position/rel_alt` (home's height, `context.home_up_m()`). Estimator: a 3 px sigma for `recovery_cv`.
- Config: `sequences/sim_package_recovery.yaml` (every flag false), `params/sim_recovery_mission.yaml`, `launch/sim_package_recovery.launch.py`.
- Sim: scenarios `recovery/pickup_oracle` and `recovery/pickup_detector`; the scenario build places a recovery mission's pickup.

## Flow

Before: no mission. After, one tick in CONE_DESCENT: the estimator's bucket track and the height above the ground → the hook's error and height → `ConeDescent.update` → DESCEND walks the commanded height down at the cone's rate (HOLD holds; LOST → LOST_TARGET; ABORT → ABORT_CLIMB) → a local position setpoint.

## Code

- One tick, the limits and the dispatch: `src/drone_mission_core/drone_mission_core/recovery_flight.py` `RecoveryFlight.update`; the cone and the engage: `_cone_descent`, `_engaging`; the way back and the set-down: `_return`, `_setting_down`.
- The context in and the command out: `src/drone_mission_demo/drone_mission_demo/missions/package_recovery_mission.py` `_read`, `_apply`.

## Evidence

```
$ pytest src/drone_mission_core/test/test_recovery_flight.py        -> 23 passed
$ pytest src/drone_mission_demo/test/test_package_recovery_mission.py -> 7 passed
$ sim/run_scenarios.sh recovery/pickup_oracle --seeds 1 --variants on
recovery-pickup_oracle-s1-on OK  points 19.99  (the bucket 0.007 m from the takeoff point, 1 descent)
$ sim/run_scenarios.sh recovery/pickup_detector --seeds 1 --variants on
recovery-pickup_detector-s1-on OK  points 0.0  (3 descents ended by the cone, mission FAILURE, nothing grabbed)
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

The spec's four tests are in `test_recovery_flight.py`: the state walk, no descent below 0.8 m after a loss, no global setpoint when GPS is untrusted, and 10,000 random set-downs. Four deliberate breaks of the flight each failed a test. `docs/results/2026-11-22-recovery-mission.md`.

## Spec check

- flight-guardian, round 1: BLOCKER, four: the cone and the engage went by a height above home when the rangefinder was not believed; a sweep engage was flown as a descent; a failure over the bucket ended in a landing on it; the mission's landing request was sent once and never confirmed. All four fixed in the flight, each with a test, with its six concerns (a hook of unknown load retried as empty, descents on a coasting track, the X's track taken for the handle, RETURN's limit, no flight boundary, unknown settings dropped).
- spec-auditor, round 1: INCOMPLETE: the search's speed and height unasserted, the lost climb's rule untested, the landing unconfirmed, no connection or pose-age check, and the lidar note unanswered. Fixed or recorded under Decisions.
- Round 2, spec-auditor: every flagged row fixed or recorded; INCOMPLETE only on the Sim proof that is owed.
- Round 2, flight-guardian: CONCERNS, the four blockers and five of six concerns resolved. Acted on after the round, not reviewed again: a landing is not asked for again once LAND has been seen (a pilot who leaves it is not fought); the mission warns when no flight boundary is given. Left for the lead: with the gripper closed, a fault lands in place, onto or with the bucket.

## Not done

- A pickup on a real detector (REC-1 below 3 m), a live sequence, REC-8's runs R1 to R6: "Needs a person".
- The five flags with nothing behind them (REC-5, REC-6, REC-7b, the hook camera) are refused when on.

## Change of 23 November 2026: the abort

By the lead's note ("RTL until the bucket is attached, LAND once it is"), the mission now tells the executor which mode a failure of the sequence asks for, each tick: `flight.abort_mode()` (`drone_mission_core/recovery_flight.py`) is RTL while the gripper has never closed and the vehicle has a pose and a GPS fix in a mission that trusts GPS, and LAND otherwise; the mission passes a change on with `context.set_abort_mode`, never gives a landing back when it leaves, and does not undo one an earlier mission left. The mission's own carrying fault still lands itself; the named mode covers what the mission cannot (an exception, the 120 s limit on a landing not seen, and whatever the executor asks after the mission has failed).

Tests: `test_recovery_flight.py` (the mode in every state, an attach check that said empty, each kind of failure, a failure after the release, no pose, no fix, GPS not trusted), `test_package_recovery_mission.py` (what the executor is told and when, leaving in mid-carry, an inherited landing), `test_executor_abort_mode.py` (the executor asks for the named mode, and for the parameter when none is named).

Sim (`docs/results/2026-11-23-recovery-abort.md`): `recovery/pickup_detector` (an empty-hook failure) aborted by RTL and came home; the new `recovery/carrying_fault` landed with the bucket on the hook and the abort stayed LAND; `recovery/pickup_oracle` still brings the bucket back (19.99 points). None of the three has the executor itself asking for LAND in the air: that is unit-tested.

Reviews. spec-auditor, round 1 (on the first build, which gave LAND back at the release): INCOMPLETE: an RTL after the release comes down on the bucket at the takeoff point; no flight showed either half; four comments still said the old thing. flight-guardian, round 1: BLOCKER: a failure for want of a position asked for RTL, which needs one, for ever. Both fixed by the rule as it stands. flight-guardian, round 2: CONCERNS, the blocker closed. Fixed after it, unit-tested, not reviewed again: LAND is named on entry when it is already known (GPS not trusted), and the abort goes back to RTL only after two seconds of pose and fix. Left as concerns, for the lead ("Needs a person"): a named LAND lasts for the rest of the sequence; a mission can now change what `abort_and_rtl` asks for (LAND only); an RTL refused in spite of a fix is still asked for again for ever (2026's executor).
