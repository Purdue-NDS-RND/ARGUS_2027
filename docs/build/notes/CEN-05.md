# CEN-05: Delivery: soft set-down on the estimator

Flag: `delivery_soft_set_down` (default false; needs `use_target_estimator`, `centre_on_estimator`, `use_height_estimator`) · Status: DONE · Commits: `git log --oneline --grep '^CEN-05:' platform/build-2027`

## What it solves

In 2026 the delivery stopped looking at the target at the handoff height, came down a frozen GPS column at 0.25 m/s, waited for the flight controller to say "landed" (which it says 2.75 s after the legs touch), and then stood five seconds on the ground. With the flag on the camera steers down to 0.6 m, the last 0.6 m is at 0.12 m/s on a held point, and the release follows contact by one tick. This fixes the cause of both the placement error and the time on the ground.

## Architecture

- `drone_mission_core/set_down.py` (new): the contact rule (`ContactDetector`), the descent's rate, and the executor's throttle feed (`attach_throttle`, subscribes only with the flag on).
- `drone_mission_demo/missions/delivery_set_down.py` (new): the FINAL_SET_DOWN state's work.
- `PackageDeliveryState.FINAL_SET_DOWN` added to the delivery mission's states.
- `MissionContext.throttle()` and `MissionContext.rangefinder_range()` (new; None with their flags off).
- New topic used: `/mavros/vfr_hud`. `vfr_hud` joins the plugin list in `config/mavros/argus_mavros.yaml` and the sim's copy; VFR_HUD at 20 Hz joins the stream keeper's rates and the sim's rate check.
- New mission config key `package_height_m` (read only with the flag on, no default).

## Flow

A delivery, centred over the target at the handoff height:

- Before (flag off, 2026): TRACK_AND_DESCEND → FINAL_FIXED_COLUMN_DESCENT (GPS point frozen, down at `final_descent_rate_mps`, "landed" plus 0.5 s) → GROUND_DWELL (5 s, release) → GUIDED_RELAUNCH.
- After (flag on): TRACK_AND_DESCEND → FINAL_SET_DOWN (steer on the track, easing to 0.12 m/s; when low and centred, a held point and 0.12 m/s; contact = rangefinder at the package's height, no vertical speed, throttle 15% down, for 0.3 s, or the flight controller's "landed") → GROUND_DWELL (release at once if under 1.82 m or landed, then 1 s) → GUIDED_RELAUNCH.

## Code

- The contact rule: `src/drone_mission_core/drone_mission_core/set_down.py:140`; what it takes as the descent's throttle (only samples while coming down): `:128`.
- One tick of FINAL_SET_DOWN: `src/drone_mission_demo/drone_mission_demo/missions/delivery_set_down.py:135`; the hold, taken only while low and centred on a track: `:182`; the held point, the lowered setpoint, contact and the 20 s limit: `:203`.
- The release on contact, its ceiling, and the one second: `src/drone_mission_demo/drone_mission_demo/missions/package_delivery_mission.py:404`; the hand-over, also from under the 2026 window: `:247`; `package_height_m` checked when the sequence is built: `:44`.
- The two readings a mission can now ask for: `src/drone_mission_core/drone_mission_core/mission_context.py:242`, `:254`.

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider src/drone_mission_demo/test/test_delivery_set_down.py
27 passed in 0.60s
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_set_down.py
18 passed in 0.03s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh delivery/set_down --seeds 1 --variants off,on     # first version; the off flight is the 2026 comparison
| delivery/set_down | 1 | OK | 16.36 | 0.17 m from the target | OK | 16.54 | 0.02 m from the target |
$ sim/run_scenarios.sh delivery/set_down --seeds 1 --variants on
| delivery/set_down | 1 | not run |  |  | OK | 16.55 | 0.01 m from the target |
$ sim/run_scenarios.sh delivery/set_down_floor --seeds 1 --variants on
| delivery/set_down_floor | 1 | not run |  |  | OK | 16.52 | 0.03 m from the target |
```

`docs/results/2026-11-20-soft-set-down.md`: touched at 0.12 m/s (2026: 0.25), released 0.8 s after the legs touched (2.6 s), 2.6 s on the ground (7.5 s), package 0.010 m from the centre (0.165 m).

## Spec check

- spec-auditor, round 1: INCOMPLETE on two rows (a band between the 0.75 m projection floor and the 0.6 m hold with nothing to steer on; the ground reached early on a barometer height). Round 2: COMPLETE.
- flight-guardian, round 1: BLOCKER (the same band: on level ground the set-down could not be reached with a track; the first flight passed by 0.05 m) and seven concerns. All acted on: the hold only while centred on a track, and at the floor; "landed" as contact before the hold; the contact rule only on a rangefinder hold; signals no older than 0.2 s; the 1.82 m ceiling at the release; 20 s and 10 s limits; the hand-over from under the window; `package_height_m` checked at build. Round 2: no blocker, the earlier findings resolved, two lower concerns: the ceiling took a barometer height as it is (fixed after the round: it counts 0.5 m higher), and the floor trigger is on the height above home, so over ground more than about 1.5 m below home the hold starts too high for its 20 s (recorded under Decisions as a known limit, fails cleanly).
- Beyond the spec, each under Decisions: the thresholds of the contact rule; the held GPS point for "holds horizontal position"; the `vfr_hud` plugin and its rate; `package_height_m`.

## Not done

- On the aircraft, three person steps before the flag can be turned on (all under "Needs a person"): `package_height_m` is unmeasured (the flag cannot fly without it, by design), the throttle's behaviour on a real touchdown is unrecorded, and the contact impulse is OPS-07's rig.
- Wind and several seeds: CEN-12's A/B harness and gate G3.
- A track lost between the handoff and 0.6 m goes to the 2026 recovery climb until CEN-06.
