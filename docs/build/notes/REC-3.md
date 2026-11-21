# REC-3: Cone descent

Flag: none of its own (a rule with no caller yet; the recovery mission's flag, REC-2, is what will turn it on) · Status: PARTIAL · Commits: `git log --oneline --grep '^REC-3:' platform/build-2027`

## What it solves

Coming down on a 0.2 m handle with a hook, the sideways error that is safe at 3 m is a miss at 0.3 m. The cone says how much error each height allows and what to do when there is more: wait, climb, or give the attempt up. The 2026 descents had one tolerance for every height.

## Architecture

- `drone_mission_core/cone_descent.py` (new, standard library only): `allowed_error_m`, `descent_rate_mps`, `hook_error_m` and `hook_height_above_handle_m` (the error and the height are the hook's, not the camera's), and `ConeDescent.update(hook_height_m, error_m, detection_age_s, now_s)` → `ConeAnswer(action, rate_mps, cone_m, reason)` with five actions: DESCEND, HOLD, CLIMB, LOST, ABORT.
- No node, topic, config key or mission change. Nothing calls it yet.

## Flow

One tick of a descent with the hook 2 m above the handle and 0.9 m to the side:

- Before: no such rule.
- After: the cone at 2 m is 0.76 m → 0.9 m is outside it but within 1.5 times → HOLD, and the three seconds start → the error comes down to 0.7 m → DESCEND at 0.32 m/s → at 0.6 m of height the cone is 0.28 m and a detection older than 0.2 s means HOLD, and 1.5 s of that means LOST → inside 0.08 m at the handle.

## Code

- The cone's width and the rate: `src/drone_mission_core/drone_mission_core/cone_descent.py:82`, `:102`; the hook's error and height: `:114`, `:136`.
- One tick: what a descent may rest on, the bounded wait, the stall: `:185`.

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_cone_descent.py
13 passed in 0.08s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

The tests hold every number of the spec (1.0 m at 3 m, 0.48 m at 0.8 m, 0.8 of the capture half-width below 0.4 m; 0.5 to 0.1 m/s; 1.5 times; 20% in 3 s, with a case that fails without the progress rule; 0.8 m and 0.2 s), and fly a point down the cone from 0.9 m out at 3 m to inside the capture width at the handle.

## Spec check

- spec-auditor, round 1: INCOMPLETE on two rows it could fix here: the 20% rule had no test that failed without it, and the hook-point error was a sentence, not code. Both built and tested.
- flight-guardian, round 1: CONCERNS, no blocker (nothing calls the rule). All five acted on in the rule itself: no descent on an old detection at any height; a wait for data ends in LOST after 1.5 s; no climb where it cannot widen the cone; rates through the speed cap; a missing height is a hold, and an abort stands whatever is known.
- Round 2, both: no blocker. Each found the same new fault in round 1's fix: with the age asked before the cone, a detection that comes and goes kept restarting both clocks and the cone held for ever. Fixed with a test (the age is asked only of a descent; the stall's clock runs through unseen moments). Also after round 2, not re-reviewed: the age limit above 0.8 m is the repo's 0.5 s, not 1.0 s; the helpers give None, not an exception, for data that did not come. Left to the mission: the ceiling of an abort's climb.

## Not done

- The Sim proof: with the recovery mission (REC-2), which is what flies by it.
- With REC-1 as built, no detection exists below about 3 m of vehicle height, so a descent by this cone ends in LOST with the hook about 2 m above the handle: "Needs a person".
- An abort's climb has a rate and no ceiling here: the mission's ABORT_CLIMB gives it one.
