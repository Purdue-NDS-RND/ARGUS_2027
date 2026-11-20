# CEN-04: Drop: camera to release, with gates

Flag: `drop_release_gates` (default false; needs `use_target_estimator`, `centre_on_estimator`, `use_height_estimator`) · Status: DONE · Commits: `git log --oneline --grep '^CEN-04:' platform/build-2027`

## What it solves

In 2026 the drop stopped looking at the target at 8 m: it froze a GPS point, came down a fixed column to the drop height and released after a timed hover, so the release was only as good as the GPS hold. With the flag on the camera steers all the way down, and the package goes only when the aim, the track, the speed, the tilt and the height are all good at once. This fixes the cause.

## Architecture

- `drone_mission_core/release_gates.py` (new, no ROS): the gates' numbers, the release band for each height sensor, the half-second hold, the last-resort rule.
- `drone_mission_demo/missions/drop_final_align.py` (new): the FINAL_ALIGN state's work, one answer a tick (running, release, look again, give up).
- `PackageDropState.FINAL_ALIGN` added to the drop mission's states. No new node, topic or message.
- `argus_camera/config/robot.yaml`: airframe row `sim_iris_parcel` (the sim's Iris with the place its carried package hangs).

## Flow

A drop, centred over the target at 8 m:

- Before (flag off, 2026): TRACK_AND_DESCEND → FINAL_FIXED_DROP_COLUMN (GPS point frozen, descend to `drop_altitude_m`) → DROP_PAYLOAD (hover `drop_hover_dwell_s`, release).
- After (flag on): TRACK_AND_DESCEND → FINAL_ALIGN (steer on the estimator's track; descend to 2.4 m on the rangefinder or 3.0 m on the barometer; under 2.0 m climb first) → the six gates hold for 0.5 s → DROP_PAYLOAD (release at once, where it is). After 25 s without them: look again once; after a second 25 s: release if within 0.5 m, else fail and let the failure policy move on.

## Code

- The gates and their hold: `src/drone_mission_core/drone_mission_core/release_gates.py:81` (`failing_gates`), `:125` (`GateHold`: a failed gate or a gap between ticks starts the 0.5 s again).
- One tick of FINAL_ALIGN, in order: no height → hold; under the floor → climb only; no track → the mission's loss grace, then recovery; steer and descend to the middle of the band; read the gates: `src/drone_mission_demo/drone_mission_demo/missions/drop_final_align.py:97`.
- The timeouts and the last resort, including "no recovery attempt left": `drop_final_align.py:171`; the lowest release height by sensor: `release_gates.py:100`.
- The release itself, checked again at the moment of the command, never sent on gates that held earlier: `src/drone_mission_demo/drone_mission_demo/missions/package_drop_mission.py:330`.
- Where the mission switches on the flag: `package_drop_mission.py:184`, `:207`, `:373`.

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider src/drone_mission_demo/test/test_drop_release_gates.py
21 passed in 0.62s
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_release_gates.py
6 passed in 0.01s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh drop/gated --seeds 1 --variants on
| drop/gated | 1 | not run |  |  | OK | 17.47 | 0.03 m from the target |
$ sim/run_scenarios.sh drop/gated_track_loss --seeds 1 --variants on
| drop/gated_track_loss | 1 | not run |  |  | OK | 0.00 | nothing released |
$ sim/run_scenarios.sh drop/gated_timeout --seeds 1 --variants on
| drop/gated_timeout | 1 | not run |  |  | OK | 17.46 | 0.04 m from the target |
$ sim/run_scenarios.sh drop/gated_floor --seeds 1 --variants on
| drop/gated_floor | 1 | not run |  |  | OK | 17.48 | 0.02 m from the target |
```

Numbers and the missions' own log lines: `docs/results/2026-11-20-gated-drop.md`. The real 0.5 kg package was released from 2.545 m and came to rest 0.027 m from the target.

## Spec check

- spec-auditor, round 1: INCOMPLETE (sim proof of the floor, the timeout and the track loss; the timeout with no recovery attempt left; records owed). Each was done: three scenarios flown, the edge handled and tested. Round 2: every flagged spec row DONE; one record gap (which code two of the flights flew), now stated in the results file.
- flight-guardian, round 1: CONCERNS, no blocker. All three acted on: the flag now needs `use_height_estimator`; the last resort and the release-moment check use the lowest release height for the sensor (2.8 m on the barometer); a release command is not sent on gates that held earlier. Round 2: NO FINDINGS. Its observation that a return from the release state kept the gates' half second running was acted on (it now starts afresh). Left as observed: with the flag on nothing stops a sequence setting `drop_column_handoff_altitude_m` under the release band (FINAL_ALIGN then climbs back, as `drop/gated_floor` shows).
- Beyond the spec, each under Decisions: the descent's rule (to the middle of the band, slowing by 0.3 m/s per metre above it); the last resort's extra conditions; the baro band's width; the three cases the spec does not name.
- The sim found one bug no unit test had: a log helper that rclpy refuses ended the mission (fixed, with a test that holds the logger to rclpy's rule).
- Two sim checks of earlier features were corrected so that a correct flight is not failed (projection: frames with the whole target in view; estimator: a band only passed through, and nothing after the release). Both are under Decisions for the lead to reverse.

## Not done

- Wind: gate G2 flies the drop in calm and strong wind over several seeds.
- The 25 s count from the 8 m handoff, and the calm descent uses 18 to 19 s of them (Decisions: the lead may want them counted from reaching the band).
- "One re-acquisition" is the 2026 recovery climb until CEN-06.
- The real aircraft: its release point in `robot.yaml` (`argus_2027`) is unmeasured, and its rangefinder is not fitted (CEN-03's person steps). The flag stays off until CEN-12's A/B says otherwise.
