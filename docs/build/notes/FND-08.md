# FND-08: Feature-flag plumbing and profiles

Flag: this is the plumbing for flags; it adds none of its own. The new launch argument `feature_overrides` defaults to empty, which changes nothing · Status: DONE · Commits: `git log --oneline --grep '^FND-08:' platform/build-2027`

## What it solves

A flag could only be turned on by editing a sequence YAML under `src/*/config`, which is the tuned, CI-guarded config; and nothing stopped a flag being on without the flag it depends on. This adds a launch-line switch, a dependency table the executor enforces, profiles outside `src`, and tests that keep every flag off by default across the whole tree. It fixes the cause for mission flags; node flags still wait for PLT-07.

## Architecture

- `drone_mission_core/features.py` (ROS-free; `get_feature_flag` unchanged): `FLAG_REQUIRES`, `parse_feature_overrides`, `resolve_features`, `apply_feature_overrides`, `load_profile`, `FeatureFlagError`.
- `drone_mission_core/mission_executor.py`: new string parameter `feature_overrides` (default `""`), applied once when the sequence is loaded.
- The twelve launch files that start the executor: new launch argument `feature_overrides` (default `""`), passed to the executor as a string.
- `profiles/comp_2026.yaml`, `profiles/README.md`; `tools/flag_profile.py` prints a profile as a `feature_overrides` value.

No new node, topic or dependency.

## Flow

Before: the executor loaded the sequence and each mission got the `features:` block as written.

After, with nothing on the launch line: the same; `apply_feature_overrides(specs, "")` returns specs equal to the loaded ones (tested on all ten shipped sequences). With `feature_overrides:=use_frame_time_pose`: the executor logs `Feature overrides: use_frame_time_pose` and every mission's flags have it true. With a flag the sequence does not declare, a value that is not true or false, or a flag whose requirement is off: `FeatureFlagError` in the constructor, so the executor does not start and no setpoint is ever published.

## Code

- `features.py:91` `resolve_features`: an override must name a declared flag; every flag must be a real boolean; then `missing_requirements` (`:75`) checks `FLAG_REQUIRES`, counting anything other than false as on.
- `features.py:45` `parse_feature_overrides`: `a,b` or `a=true,b=false`; anything else is refused.
- `mission_executor.py:147-156`: read the parameter, apply it, log it when not empty.
- `features.py:127` `load_profile`: an enable-list, checked for names, repeats and dependencies.
- `tools/flag_profile.py`: on any failure it prints `PROFILE_ERROR`, which the executor refuses, so a launch line built from its output does not quietly start with every flag off.

## Evidence

```
$ PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_feature_plumbing.py src/drone_mission_core/test/test_feature_flag_rules.py
186 passed in 0.55s
$ python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_executor_feature_overrides.py tools/test/test_flag_profile.py
14 passed in 0.83s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

`test_feature_plumbing.py` was run before the code existed and failed on import. `test_executor_feature_overrides.py` drives the real `MissionExecutorNode` with rclpy stood in.

In ROS 2 Humble (the sim image, no sim running), every launch file that starts the executor was launched with nothing on the line and with an override; a scratch script, output kept in `sim/.state/fnd08_launch_all.log`:

```
circuit_time_trial_live.launch.py        (no argument)                              executor started: yes  none logged
circuit_time_trial_live.launch.py        feature_overrides:=use_frame_time_pose     executor started: yes  Feature overrides: use_frame_time_pose
...the same two lines for the other ten drone_mission_demo launch files...
live_landing_test.launch.py              (no argument)                              executor started: yes  none logged
live_landing_test.launch.py              feature_overrides:=use_frame_time_pose     executor started: no   REFUSED: FeatureFlagError: feature_overrides names flag(s) this sequence does not declare: use_frame_time_pose
package_delivery_demo.launch.py          feature_overrides:=use_warp_drive          executor started: no   REFUSED: FeatureFlagError: feature_overrides names flag(s) this sequence does not declare: use_warp_drive
package_delivery_demo.launch.py          feature_overrides:=PROFILE_ERROR           executor started: no   REFUSED: FeatureFlagError: feature_overrides: 'PROFILE_ERROR' is not a flag name
```

(`live_landing_test`'s sequence declares no flags, so any override is refused there.) A test now reads every launch file's syntax tree and fails if one that starts the executor does not declare the argument with an empty default and pass it as a string.

Flags off in the sim, after the change: `sim/run_scenarios.sh smoke` on the final code gave `OK delivery/baseline seed 1 off: 16.17 points, 0.07 m from the target` (an earlier run, before the review fixes, gave 0.04 m). The executor's log has no "Feature overrides" line.

## Spec check

spec-auditor: INCOMPLETE in round 1 (nothing in the tree proved the launch-file leg, a flag set to `1` or `"true"` slipped the dependency check, and the mission-flags-only scope was not recorded); COMPLETE in round 2.

flight-guardian: CONCERNS in both rounds, no blocker. Round 1: the empty default through a real launch was unproven (since proven in ROS and by a sim flight), the dependency check failed open on a truthy value (fixed), and a failed profile left the documented launch line with an empty string, so the aircraft would fly with every flag off while the operator believed otherwise. Round 2: that last concern still held for failures other than a bad profile. After round 2 the tool prints the refused word on every failure path and the README shows a two-step form with `|| exit`; this last change was not reviewed a third time. What cannot be closed here: if the tool does not run at all, an empty `feature_overrides` flies flags off, which is the safe fallback but not what the operator asked for. PLT-07's bringup launch, which takes the profile itself, removes the shell from the path.

## Not done

- **Node flags.** `feature_overrides`, profiles and `FLAG_REQUIRES` cover mission flags only. Node flags (`features.<flag>` ROS parameters) still need the params YAML on the aircraft until PLT-07's bringup launch routes them (Decisions).
- **A profile by name.** PLT-07 owns the launch that takes a profile; today `tools/flag_profile.py` feeds the launch line.
- **`FLAG_REQUIRES` has one entry**, with flag names that FND-06 and CEN-04 still have to declare (or rename).
- No 2027 flag is read by any mission yet, so no override changes anything in the air.
