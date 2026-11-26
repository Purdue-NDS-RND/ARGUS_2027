# REC-7: Return and set-down (GPS path)

Flag: none of its own (rules with no caller in a mission yet; the recovery mission, REC-2, is the first). In the sim: the scenario's driver settings `return_frame`, `return_target`, `set_down_rule` · Status: PARTIAL · Commits: `git log --oneline --grep '^REC-7:' platform/build-2027`

## What it solves

With the bucket on the hook, where to take it, in which frame to fly there, and how to put it down without dropping it: the 2026 code returned by RTL and released on a timer after "landed".

## Architecture

- `drone_mission_core/recovery_return.py` (new; standard library and this package's limits): `parse_return_target` (`takeoff`, `{latlon}`, `{marker, near}`), `Anchor` (one place in both frames), `return_goal` (a global setpoint with GPS trusted, a local one otherwise; a homing fix within 10 m), `global_height_m`, `setpoint_along`, `BucketSetDown` and `SetDown` (LOWER, SET_DOWN, RELEASE, CONFIRM; LAND when it cannot go on).
- `config/params/recovery_mission.yaml` (new): 2026's `mission_params.yaml`, value for value. (It had `abort_rtl_mode: "LAND"` until 23 November 2026: see "Change of 23 November 2026" below.)
- `recovery_grip.py`: `new_attempt()` (the gripper commands of both modules share one numbering).
- Sim: `sim/recovery_check.py` flies the way back and the set-down by these rules when asked; scenarios `recovery/return_set_down` (global, takeoff) and `recovery/return_local` (local, published point).

## Flow

The bucket lifted, 14.4 m from the takeoff point:

- Before (REC-8's scripted flight): a local point walked to the takeoff, a lowering to 5 cm, the gripper opened.
- After: `return_goal` → GPS trusted, so a global setpoint at the takeoff's own latitude and longitude, walked along the shaped leg (untrusted: the same point in the local frame) → LOWER to 0.5 m over the touch height → SET_DOWN at 0.12 m/s until the throttle has come half way back to the empty hover's for 0.3 s → the release guard (a fresh rangefinder height at or under 1.5 m) → gripper open, acknowledged → CONFIRM: up 0.5 m, the throttle still empty.

## Code

- The goal and its frame: `src/drone_mission_core/drone_mission_core/recovery_return.py:199`; the walked setpoint: `:249`; a global setpoint's height: `:236`.
- The set-down felt: `:281`; one tick of the set-down: `:386`; the acknowledgement and the retries: `:437`; the confirming climb: `:460`.
- In the check flight: `sim/recovery_check.py:192` (the way back), `:216` (the set-down).

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_recovery_return.py
15 passed in 1.14s
$ sim/run_scenarios.sh recovery/return_local --seeds 1 --variants on
recovery-return_local-s1-on OK  rtf 0.91  points 19.97
return {'target': 'latlon', 'frame': 'local', ... 'setpoints': {'global': 0, 'local': 220}}
set_down phases lower, set_down, release, confirm; released at 0.66 m of rangefinder height; confirmed
```

`docs/results/2026-11-21-recovery-return.md` has both flights' tables and the six failed flights before them. The unit tests include 5,000 random cases in which an untrusted GPS never gives a global setpoint and 10,000 random set-downs in which the gripper never opens above 1.5 m or on a stale or unhealthy height in the air.

## Spec check

- spec-auditor, round 1: INCOMPLETE on two rows fixable here: the local return had no flight, and a published point was not driven through any entry point. Both now fly in `recovery/return_local`. The confirming climb got the lowering's second of grace for a late height.
- flight-guardian, round 1: CONCERNS, no blocker. Fixed: the params file now carries 2026's take-off block (without it the take-off service fell back to its code's defaults); a published point over 1000 m from the takeoff is refused (a swapped latitude and longitude); `global_height_m` and the docstring hand the height-frame difference to the mission; the tool ages readings on the autopilot's clock. Recorded for the lead: LAND as the abort for the whole recovery sequence.
- Round 2, spec-auditor: INCOMPLETE only on what is owed to REC-2 and REC-7b (a marker target, a launch that passes the params file); the two rows fixable here are DONE. Two stale phrases in the progress file corrected.
- Round 2, flight-guardian: CONCERNS, no blocker; four of five resolved. Open then, for the lead: LAND as the abort for the whole recovery sequence (answered on 23 November 2026: see below). Left as flown: the sim tool rounds a reading stamped just ahead of its clock to age 0 (about 5 ms at this real-time factor), and its clock code has no unit test. After the round, a docstring only: read the height offset after arming.

## Not done

- A flight by a mission: REC-2. With it: the flags (GPS-untrusted mode, homing), who says GPS is trusted (REC-6), a launch that passes the params file.
- A marker target and homing in a flight: REC-7b gives the fixes.
- A flight with GPS really faulted (REC-8's runs): the local flight here takes GPS as untrusted by a setting.
- The lowering has no time limit of its own and trusts the rangefinder alone; the mission's state timeout is its bound.
- The sim's hook is a rigid rod; the share of throttle that says "set down" needs a flight with the real suspension.

## Change of 23 November 2026: the abort

The spec's "Recovery uses `abort_rtl_mode: LAND`" was replaced by the lead's note: RTL until the bucket is attached, LAND once it is. As built (`recovery_return.py`, "Abort"; `abort_mode`, `ABORT_RTL`, `ABORT_LAND`; `RECOVERY_ABORT_MODE` is gone): LAND from the gripper's first close for the rest of the mission, LAND whenever there is no pose, no GPS fix or `recovery_gps_untrusted` is on, RTL otherwise. Both recovery params files are back to `abort_rtl_mode: RTL` (`recovery_mission.yaml` is `mission_params.yaml` value for value), and the executor's abort asks for a mode a mission has named (`MissionContext.set_abort_mode`, LAND or nothing).

What REC-7 was written to avoid, an RTL refused for want of a position and asked for again for ever, does not come back: without a position the mission names LAND. The two places where the build is not the note's letter are under "Decisions" and "Needs a person" for the lead. The older results files (`docs/results/2026-11-21-recovery-return.md`, `2026-11-22-recovery-mission.md`, `2026-11-22-gate-G5.md`) describe the abort as it was when those flights were made.

Sim and reviews: `docs/results/2026-11-23-recovery-abort.md` and the same section of `docs/build/notes/REC-2.md`.

## After the build (2026-11-25)

The lead accepted the abort as built (RTL until the gripper first closes; LAND after that, and whenever there is no position or no trusted GPS), and `docs/build/FEATURES.md` now says it in its three places: decision 14, REC-7's last line and gate G5's pass line (`sim/scenarios/gates.yaml` with it). "Decisions", 25 November.
