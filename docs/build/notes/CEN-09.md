# CEN-09: ArduPilot precision-loiter bridge (benchmark only)

Flag: `precision_loiter_bridge` (mission flag, default false; needs `use_target_estimator`) · Status: PARTIAL · Commits: `git log --oneline --grep '^CEN-09:' platform/build-2027`

## What it solves

ArduPilot can hold a multicopter over a target by itself when it is told where the target is. The bridge tells it, from our estimator, so that its loop can be flown against ours. **Only the bridge is built; the comparison has not been flown.**

## Architecture

- `drone_mission_core/precision_loiter.py` (new): `landing_target()` (no ROS) turns the track and the vehicle's pose into the message's numbers; `attach_precision_loiter()` is the executor's thin part: a publisher of `mavros_msgs/LandingTarget` on `/mavros/landing_target/raw` and a 10 Hz timer.
- The executor attaches it beside the estimator. With the flag off nothing is made: no publisher, no timer.
- No MAVROS plugin is added anywhere. The topic has no listener until `landing_target` is on a plugin list with `listen_lt: true`; the aircraft's list is the lead's.

## Flow

- Flag off: as before.
- Flag on: estimator's CONFIRMED bullseye track + the pose of now → the target in the body's axes → `LandingTarget` (frame BODY_FRD, the two angles, the distance, and the position, which MAVROS marks valid) → MAVROS → LANDING_TARGET → ArduPilot's PLND. Nothing is sent for a track that is COASTING, LOST, older than 0.5 s, not under the vehicle or more than 60 degrees off its "down"; nor from a pose over 0.2 s old; nor in any mode but GUIDED and LOITER, so a return or a landing is never drawn onto the target. The message carries the time of the pose it was made from.

## Code

- The message's numbers: `src/drone_mission_core/drone_mission_core/precision_loiter.py:78`.
- The executor's part, and what is not sent: `src/drone_mission_core/drone_mission_core/precision_loiter.py:100`; attached at `src/drone_mission_core/drone_mission_core/mission_executor.py:184`.

## Evidence

```
$ pytest src/drone_mission_core/test/test_precision_loiter.py     (as scripts/check.sh runs it)
10 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

What MAVROS does with the message was read from its source (`mavros_extras/src/plugins/landing_target.cpp`, branch `ros2`): the raw topic is listened to only with `listen_lt` true; the angles and the distance go through as they are; a BODY_FRD position is turned from forward-left-up to forward-right-down and marked valid. How ArduPilot reads the two angles is from memory of its source, not checked: the first benchmark flight is the check.

## Spec check

spec-auditor: INCOMPLETE, and it stays so: the comparison in the A/B harness and the PLND parameters are MISSING (not flown, nothing in the repo sets them), and how ArduPilot reads the message is unverified. The bridge itself, the flag, the flags-off path and the lead's note on the aircraft's plugin list are DONE. It also pointed out that the sim's own MAVROS list could take the plugin: true, and said so below. The row is closed PARTIAL on that verdict, not DONE.

flight-guardian: BLOCKER on round one (geometry from a pose of any age, stamped as now). Fixed: only the newest pose sample, at most 0.2 s old, and the message carries that pose's time; and nothing is sent outside GUIDED and LOITER. Round two: the blocker cleared, CONCERNS left: the mode is the last one MAVROS reported, so the gate can close up to a state message late, and what the flight controller does with a target it already holds is its PLND parameters'; ArduPilot's side of the frame chain is from memory; the track is the primary bullseye, not a mission's own.

## Not done

- The comparison in the A/B harness (why PARTIAL). It was not flown, and needs things the sim does not have: the `landing_target` plugin on the sim's MAVROS list with `listen_lt: true`; `PLND_ENABLED 1` and `PLND_TYPE 1` in the scenario's parameters; the vehicle in LOITER with precision loiter switched on (an RC switch with option 39, or `MAV_CMD_DO_AUX_FUNCTION` 39), and in LOITER a throttle stick at the middle, which SITL's is not unless something stands in for the pilot; and a measure of the hold that does not end at a release, which the judge's does. A hold mission that asks for LOITER and switches precision loiter on was drafted and left out of the commit: it has never flown.
- The parameter names and the option number are from memory and are to be checked against ArduPilot's documentation for 4.6.
- Only the bullseye class is sent, and it is the estimator's primary bullseye track: with two bullseyes in view that need not be the one a mission is centring on. The benchmark is for one target.
- How long the flight controller goes on acting on its own prediction after the messages stop is its own timeout, not ours; the benchmark's PLND parameters are to be chosen with that in mind.
- Who: the lead decides whether the benchmark is worth flying; whoever flies it does the sim work above first.
