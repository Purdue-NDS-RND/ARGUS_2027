# REC-9: Flight-controller profile and mechanical interface

Flag: none (a parameter file and a checking tool; nothing flies differently) · Status: PARTIAL · Commits: `git log --oneline --grep '^REC-9:' platform/build-2027`

## What it solves

In 2026 the flight controller flew with the gripper disabled, no rangefinder, `EK3_SRC_OPTIONS` 1 and a second GPS configured that was not fitted. Recovery cannot fly on that. This row is the code part: the parameters recovery needs, written down, and a tool that says whether a parameter file (the profile, or what is actually on the aircraft) has them.

## Architecture

- `docs/fc/recovery.parm` (new): 22 parameters over the aircraft's baseline, each group with its reason.
- `tools/fc_param_check.py` (new, standard library only): reads a parameter file in the forms ground stations write (Mission Planner's, MAVProxy's, QGroundControl's five columns), applies eight rules, and with `--profile` compares a dump with the profile and requires it to be a whole one.
- No node, topic or flight code.

## Flow

On the bench, before a recovery flight:

- Before: nothing said what the flight controller had to be set to, and nothing checked it.
- After: load `docs/fc/recovery.parm` → save the aircraft's parameters to a file → `python3 tools/fc_param_check.py <dump> --profile docs/fc/recovery.parm` → a line a rule and a line for each value that differs → PASS or FAIL (exit code 0 or 1).

## Code

- Reading a parameter file: `tools/fc_param_check.py:54`.
- The eight rules, in the spec's order: `:92` (gripper) to `:160` (radio switches); the table of them `:166`.
- A dump against the profile: `:187`.

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider tools/test/test_fc_param_check.py
25 passed in 0.03s
$ python3 tools/fc_param_check.py docs/fc/recovery.parm
ok   gripper: holds
ok   failsafe: holds
ok   rangefinder: holds
ok   ekf_source_options: holds
ok   source_set_2: holds
ok   second_gps: holds
ok   fence: holds
ok   radio_switches: holds
PASS: docs/fc/recovery.parm (22 parameters)
```

Each rule is shown to fail on what it is for (21 cases: the gripper on a motor's output, bit 5 of `FS_OPTIONS` set, the 2026 values of `EK3_SRC_OPTIONS` and `GPS2_TYPE`, a flow type the companion's `OPTICAL_FLOW` would be ignored with, a switch mapped to the gripper, and so on). A dump in QGroundControl's form passes through the tool; a file that is not a whole dump does not pass as one. The sim's `sim/params/recovery.parm` is held to the same values where the two are the same setting.

## Spec check

- spec-auditor, round 1: INCOMPLETE on two rows: the tool could not read QGroundControl's saved parameters, which is what the bench will produce, and the source-set rule took any flow type where REC-5 needs MAVLink's. Round 2: both DONE; the overlap with PLT-08's baseline is recorded under "Needs a person".
- No flight-guardian review: no flight code and no config under `src/` changed.

## Not done

- Everything the spec calls "blocking inputs from the mechanical team": the mechanism, its output and PWMs, suspension length, envelope, switch wiring, hover throttle with a load.
- The profile on a real flight controller: the bench run of the checker against the aircraft's dump, which is also what checks the parameter numbers themselves and the reading of a real QGroundControl file.
- The baseline it sits on is PLT-08's.
