# REC-8: Recovery simulation

Flag: none (sim only: a vehicle variant, a parameter file, scenario keys and a scripted check; no flight code) · Status: PARTIAL · Commits: `git log --oneline --grep '^REC-8:' platform/build-2027`

## What it solves

The sim had a bucket and a bridge that could join it to the vehicle, and nothing to fly a recovery with: no hook, no vehicle with both cameras, no recovery parameters, and no flight that had ever taken hold of the bucket. Now it has all four, proven together by one scripted flight, ahead of the mission.

## Architecture

- `sim/sensor_profiles.py`: profile `recovery` → model `iris_recovery` (generated): the delivery camera, a 320 x 200, 20 Hz flow camera, and a hook (a 0.50 m rod on a damped two-axis joint under the body). A camera mode `sim_flow_recovery` with its calibration.
- `sim/scenario/build.py`: on that vehicle a bucket's detachable joint is to the hook, the bridge is told where the hook hangs, and the run gets `sim/params/recovery.parm`. New scenario key `grab_refusals` (run R6).
- `sim/recovery_check.py` (new): a driver that grabs, lifts, carries and sets down, on a MAVLink port of its own (14554) beside the payload bridge. Scenario `recovery/hook_check`.
- `sim/payload_bridge.py`: refused grabs. `sim/runner`: a bucket the bridge lets go is a `release` in the trace, so the judge sees it whoever flew.

## Flow

The gripper output goes to "grab" with the hook at the handle:

- Before: the bucket was joined rigidly to the vehicle's camera link, 0.30 m under a vehicle that had no hook; no flight had done it.
- After: bridge checks the hook's rest point (0.55 m under the vehicle) against the handle → joins the bucket to `hook_link` → the bucket hangs and swings on the damped joint → carried → gripper to "release" → bridge unmakes the joint and the trace gets a `release` → the judge scores lift, release height and where the bucket rests.

## Code

- The hook and the vehicle: `sim/sensor_profiles.py:50` (the flow camera), `:161` (`HOOK`), `:206` (where its tip hangs).
- The build: the joint to the hook `sim/scenario/build.py:244`, the recovery parameters `:282`, refused grabs `:547`; the bridge's refusal `sim/payload_bridge.py:174`.
- The check flight `sim/recovery_check.py:51`; the bridge's release into the trace `sim/runner/events.py:25`.

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider sim/test/test_scenario.py sim/test/test_payload_bridge.py sim/test/test_recovery_check.py sim/test/test_sensors.py sim/test/test_runner.py
277 passed in 10.09s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh recovery/hook_check --seeds 1 --variants on
| recovery/hook_check | 1 | not run |  |  | OK | 19.99 | ... |
```

`docs/results/2026-11-20-recovery-sim.md`: real-time factor 0.91; grab 0.006 m from the handle in the bridge's own envelope; the bucket lifted 2.17 m, carried 14.4 m at 1 m/s swinging at most 0.10 m, let go 0.05 m up, at rest 0.004 m from the takeoff point; 19.99 points. Four flights: the first three found the wrong height frame in the check, a flow camera too heavy for the laptop, and a bucket that brings the vehicle down when carried fast.

## Spec check

- spec-auditor, one round: INCOMPLETE. R1 to R6 are not flown and cannot be until the mission exists: the row is PARTIAL for that. Acted on: the envelope being checked on the hook's rest point is now said (Decisions); `grab_refusals` gives R6 its refused attach; a recovery scenario with `gps_events` is built in a test (R3 to R5); the widened envelope is gone from the scenario; a miss is tested through a built run. The fixes after the round were not re-reviewed.
- flight-guardian, one round: CONCERNS, no blocker. Acted on: the check keeps the sim's clock and does not count a stale position; the recovery flow camera has its own camera mode; the judge sees a bucket's release from the bridge; the tool carries to the judge's return point.

## Not done

- Runs R1 to R6: with REC-2's mission (and REC-1, REC-3, REC-4, REC-7; REC-5 and REC-6 for R3 to R5). Their scenarios are written then, on `grab_refusals` and `gps_events`.
- The hook's real numbers and mechanism: the mechanical team (REC-9).
- The bridge reads the hook's rest point, not the swinging link's pose.
- The mission needs REC-4's swing shaping before it carries: flight 2 is why.
