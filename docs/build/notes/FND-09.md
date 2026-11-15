# FND-09: Safety limits module

Flag: `enforce_safety_limits` (mission flag, default false, declared in all ten mission sequences). It only switches on the load-time check; the constants and helper functions have no flag · Status: PARTIAL · Commits: `git log --oneline --grep '^FND-09:' platform/build-2027`

## What it solves

The rule limits (6 ft for a drop release, 25 ft for the survey, 6 ft for a set-down release) existed only as numbers in YAML files and in people's heads; the reviewer agent repeats them by hand. Now they are code constants in one module that reads no parameter or file, with functions a mission passes its height and speed commands through, and a check of the heights a sequence configures. It fixes the cause for new code. The June 2026 missions are untouched and do not use it.

## Architecture

- New ROS-free module `src/drone_mission_core/drone_mission_core/safety_limits.py`: the constants, `drop_height_command`, `survey_height_command`, `drop_release_allowed`, `set_down_release_allowed`, `recovery_release_allowed`, `capped_speed`, `check_mission`, `check_sequence`, `enforced_violations`.
- `mission_executor.py`: after loading the sequence it raises if `enforced_violations` returns anything, which only happens with the flag on.
- The ten sequences in `src/drone_mission_demo/config/sequences/` gain `enforce_safety_limits: false` in their `features:` block (a new key; no value changed).

No new node, topic or dependency.

## Flow

Before: a sequence with `drop_altitude_m: 2.0` and `drop_altitude_tolerance_m: 0.5` loaded and could release at 1.5 m, below the rule.

After, flag off: exactly the same (tested through the real executor class). After, with `feature_overrides:=enforce_safety_limits`: the executor raises `enforce_safety_limits is on and the sequence breaks a safety limit: package_drop (package_drop): drop_altitude_m 2.00 minus its tolerance 0.50 allows a release at 1.50 m, below the 1.83 m (6 ft) rule` in its constructor and never starts.

## Code

- `safety_limits.py`, the constants: the limits, each a literal with the rule or spec section it comes from. Heights are above the ground under the vehicle, not above home. A test reads the syntax tree and fails if one stops being a literal number or the module starts importing anything it could be tuned through.
- `drop_height_command` and `survey_height_command`: a commanded height never goes under 2.0 m (drop) or 7.62 m (survey); a request that is not a finite number raises, so the mission fails and its failure policy acts.
- `capped_speed`: per-phase speed caps, sign kept, not-a-number gives 0 (a hold).
- `check_mission`: a drop height minus its tolerance must not allow a release below 1.83 m. It models when the 2026 mission starts its drop; a pass is necessary for a legal drop, not proof of one.
- `mission_executor.py:158-165`: the flagged refusal at start-up.

## Evidence

```
$ PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_safety_limits.py
47 passed in 0.28s
$ python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_executor_feature_overrides.py
13 passed in 0.30s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

`test_safety_limits.py` was run before the module existed and failed on import; its "declares the flag off" cases failed until the sequences had the key. The tuning gate passes: adding a key to a sequence is not a changed value.

In ROS 2 Humble (the sim image), the scratch launch script from FND-08, output in `sim/.state/fnd09_launch_all.log`: all twelve executor launch files start with nothing on the line, and

```
package_drop_demo.launch.py              feature_overrides:=enforce_safety_limits   executor started: yes  Feature overrides: enforce_safety_limits
package_drop_live.launch.py              feature_overrides:=enforce_safety_limits   executor started: yes  Feature overrides: enforce_safety_limits
final_waypoint_drop.launch.py            feature_overrides:=enforce_safety_limits   executor started: yes  Feature overrides: enforce_safety_limits
package_delivery_demo.launch.py          feature_overrides:=enforce_safety_limits   executor started: yes  Feature overrides: enforce_safety_limits
```

And a drop sequence edited to `drop_altitude_m: 2.0` (tolerance 0.5), in real ROS (`sim/.state/fnd09_refusal.log`):

```
== ros2 launch drone_mission_demo package_drop_demo.launch.py sequence:=<drop_altitude_m 2.0, tolerance 0.5>
[INFO] [mission_executor]: Mission executor initialized with 3 mission(s)
== ros2 launch drone_mission_demo package_drop_demo.launch.py sequence:=<drop_altitude_m 2.0, tolerance 0.5> feature_overrides:=enforce_safety_limits
[INFO] [mission_executor]: Feature overrides: enforce_safety_limits
ValueError: enforce_safety_limits is on and the sequence breaks a safety limit: package_drop (package_drop): drop_altitude_m 2.00 minus its tolerance 0.50 allows a release at 1.50 m, below the 1.83 m (6 ft) rule
```

Flags off in the sim, after the executor and sequence change (the review fixes that followed touch only helper functions nothing calls yet, and were not flown again): `sim/run_scenarios.sh smoke` gave `OK delivery/baseline seed 1 off: 16.23 points, 0.01 m from the target`.

## Spec check

spec-auditor: INCOMPLETE in both rounds, with PARTIAL confirmed as the right status. Round 1 asked for the geofence margin to be pinned and its reason corrected, a guard against missions repeating the numbers, and plainer words about what the load-time check does and does not do; round 2 found those done and nothing left for the session. What keeps it INCOMPLETE is the spec's second bullet, the flight-guardian edit, which is the lead's.

flight-guardian: CONCERNS in both rounds, no blocker. Fixed from round 1: the set-down ceiling rounded the wrong way (now 1.82 m), a not-a-number height became the floor (now raises), the height datum was unstated, and the copied drop defaults were unpinned. Fixed after round 2, not reviewed again: the helpers rejected NumPy floats. Left for the features that add the first callers:

- **REC-4:** `landed=True` overrides the height, and the only landed source today (`vehicle_is_landed()`) has no age check.
- **FND-07:** the module asks for height above ground, and `height_agl()` does not exist until then; a mission written earlier has only altitude above home.
- **Every 2027 mission:** the release helpers answer "not allowed" and `capped_speed` answers 0 on a bad input without raising, so a caller needs its own timeout or the mission stalls quietly.

## Not done

- **The flight-guardian still repeats the numbers** (the spec's second bullet, and why this row is PARTIAL). Its invariant F5 lives in `.claude/agents/flight-guardian.md`, which the session may not edit; the edit is written out under "Needs a person".
- **As shipped, no sequence is checked at start-up.** `enforce_safety_limits` is false everywhere and no profile turns it on; the lead decides when it goes into a flight profile. The check runs in the tests on every shipped sequence.
- **No mission imports the module yet.** The 2026 missions are the fallback and stay as they are; CEN-04, LOC-1, REC-4 and the other 2027 missions import it as they are written. A test fails if a file under `src` writes 1.83, 1.82, 7.62 or 6.10 itself.
- **Only the drop height is checked at load time**, against the 1.83 m rule. The survey and recovery heights join `check_mission` with LOC-1 and REC-4.
- **The geofence margin (5.0 m) and the speed caps are the session's picks**, used by nothing yet; the lead confirms them.
- No `package_drop` flight was flown on this tree; the drop mission's code is unchanged.
