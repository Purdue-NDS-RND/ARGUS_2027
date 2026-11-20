# CEN-06: Smart re-acquisition

Flag: `smart_reacquire` (default false; needs `use_target_estimator`, `centre_on_estimator`) · Status: DONE · Commits: `git log --oneline --grep '^CEN-06:' platform/build-2027`

## What it solves

In 2026 a mission that lost its target hovered for a 4 s grace time, climbed one metre from wherever it was, and looked again, up to four times. The climb had nothing to do with how well the target's place was known. With the flag on the mission climbs to the height from which the camera sees everywhere the target can be, over where it was last estimated, and waits there at most 10 s. This fixes the cause.

## Architecture

- `drone_mission_core/reacquire.py` (new, no ROS): the height rule, the look point, the limits.
- `drone_mission_demo/missions/smart_reacquire.py` (new): what TARGET_NOT_FOUND does with the flag on.
- The switch is in the base mission's recovery, so the drop's and the delivery's state machines are unchanged. No new node, topic, message or config key.

## Flow

A drop descending on its track at 3 m; the detector stops seeing the target:

- Before (flag off, 2026): track COASTING, then LOST after 2 s → hold 4 s (`target_loss_grace_s`) → TARGET_NOT_FOUND: climb 1 m on the vehicle's own GPS point → ACQUIRE_TARGET.
- After (flag on): track LOST after 2 s → TARGET_NOT_FOUND at once: local position setpoint over the track's last estimate (2.5 m sideways at most) at (3 sigma + 2.5 m) / k above the ground → on arriving, up to 10 s for a CONFIRMED, freshly hit track → ACQUIRE_TARGET; or the mission fails and its failure policy acts.

## Code

- The height: `src/drone_mission_core/drone_mission_core/reacquire.py:71` and `:80`; the look point, 2.5 m toward the estimate at most: `:61`.
- Starting a look (the estimate as the estimator has it now, the height above the ground, the attempt count): `src/drone_mission_demo/drone_mission_demo/missions/smart_reacquire.py:92`; one tick (the setpoint, arriving, "CONFIRMED again", the 10 s): `:136`.
- The two places the base mission switches on the flag: no second grace once the estimator says LOST, `src/drone_mission_demo/drone_mission_demo/missions/red_bullseye_mission_base.py:598`; the recovery itself, `:643`.

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider src/drone_mission_demo/test/test_smart_reacquire.py
18 passed in 0.50s
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_reacquire.py
11 passed in 0.02s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh drop/reacquire --seeds 1 --variants on
| drop/reacquire | 1 | not run |  |  | OK | 0.00 | nothing released |
```

`docs/results/2026-11-20-smart-reacquire.md`: four losses at 2.9 m, each looked for at once from 5.9 m above the ground over the last estimate, confirmed again on arriving, descending again 5 s after LOST (2026 recovery on the same target: 7 s, from one metre up).

## Spec check

- spec-auditor, round 1: INCOMPLETE on two rows: the gated drop's timeout entering the look was unproven (now a mission test: one look from the height, and the second timeout is not another), and the height was above home, unrecorded (now above the ground under the vehicle). Round 2: COMPLETE.
- flight-guardian, round 1: CONCERNS, no blocker. Acted on: the look point had no bound on its distance from the vehicle (now 2.5 m sideways at most, the rest looked for from higher); a remembered estimate with no age (removed: only the estimator's own tracks are used); the height datum (as above). Reported under Decisions, not changed: the look is from under the 7.62 m of the survey rule, as the spec's formula gives. Round 2: three of the four resolved, no new blocker; the fourth (the survey rule) stays open for the lead, with a second note that one look can now reach the 40 m recovery ceiling.
- Beyond the spec, each under Decisions: no second grace time; the 10 s from arriving; what "CONFIRMED again" is; the attempt count and ceiling kept; the 2026 recovery when there is no track to look for.

## Not done

- A loss and recovery that ends in a release: the sim scenario's target cannot be kept below 3 m, so every look ends in another loss. Gate G2's flights in wind are where it would show.
- The 10 s running out and the climb that does not arrive are mission tests, not flights.
