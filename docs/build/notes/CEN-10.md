# CEN-10: 2027 sequences for drop and delivery (and CEN-11's sim copies of them)

Flag: none of its own; the two files declare every 2027 mission flag, all false · Status: DONE · Commits: `git log --oneline --grep '^CEN-10:' platform/build-2027`

## What it solves

The 2027 features read settings the 2026 live files do not have (the retuned controller's gains, the camera's name, the package's height), and the live files must stay the fallback. The two new files are where a 2027 flight is configured: the live file's values, plus the 2027 settings, with every flag off until a profile turns one on. This is configuration, not a behaviour change.

## Architecture

- `src/drone_mission_demo/config/sequences/package_drop_2027.yaml`, `package_delivery_2027.yaml` (new).
- Their generated sim copies `sim_package_drop_2027.yaml`, `sim_package_delivery_2027.yaml`; `tools/sim_sequence_copies.py` has the two files in `COPIES` and one more changed setting (`package_height_m`).
- One new mission setting, read only with `centre_on_estimator` on: `estimator_centering_tolerance_m`.
- Scenarios `drop/live_tuning_2027`, `delivery/live_tuning_2027`.

## Flow

A drop configured from `package_drop_2027.yaml`:

- Every flag off (as the file stands): the 2026 live flight, value for value (`centering_tolerance_m` 0.50 m, the fixed column, the hover).
- With the 2027 flags on (a profile): the same heights and rates, the gated release, and "centred" is 0.20 m from the estimator's track (`estimator_centering_tolerance_m`), not 0.50 m from the raw detection.

## Code

- The estimator's own tolerance, checked when the sequence is built (finite, above 0, at most 1 m): `src/drone_mission_demo/drone_mission_demo/missions/red_bullseye_mission_base.py:113`; where it takes the place of the 2026 one: `:167`.
- The copy tool's sixth changed setting and the two new pairs: `tools/sim_sequence_copies.py:33`, `:44`.
- The test that holds every 2026 value in a 2027 file equal to its live file's: `src/drone_mission_demo/test/test_sequences_2027.py` (`test_with_every_flag_off_a_2027_file_is_its_2026_live_file`).

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider src/drone_mission_demo/test/test_sequences_2027.py
9 passed in 0.32s
$ python3 tools/sim_sequence_copies.py --check
4 sim copies up to date
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh drop/live_tuning_2027 --seeds 1 --variants on
| drop/live_tuning_2027 | 1 | not run |  |  | OK | 17.47 | 0.03 m from the target |
$ sim/run_scenarios.sh delivery/live_tuning_2027 --seeds 1 --variants on
| delivery/live_tuning_2027 | 1 | not run |  |  | OK | 16.21 | 0.03 m from the target |
```

`docs/results/2026-11-20-sequences-2027.md`: with the live files' rates and heights and eight of each file's ten flags on (not `time_health_guard` or `enforce_safety_limits`), the drop released on its gates 0.026 m from the target and the delivery set down 0.029 m from it.

## Spec check

- spec-auditor, round 1: every spec line met; INCOMPLETE on three small items (the new tolerance was checked in the air, not at build; a copy-tool assertion loosened by a branch that never applied; the record said "every flag on" when eight of ten were). Round 2: the first two DONE; one sentence of the corrected record was itself wrong (it said `time_health_guard` has a scenario of its own; none flies it on yet) and is corrected.
- flight-guardian, round 1: two concerns on the new tolerance (an infinite value would have counted a remembered track as centred; a bad value was found in the air). Both fixed: finite, above 0, at most 1 m, checked when the sequence is built. Round 2: NO FINDINGS.
- Beyond the spec, each under Decisions: `estimator_centering_tolerance_m`; the copy tool's `package_height_m`.

## Not done

- On the aircraft neither file builds with a 2027 flag on until the camera mode is calibrated and the `argus_2027` airframe measured; the delivery also needs `package_height_m`. All under "Needs a person".
- Wind and several seeds on these files: CEN-12's A/B harness.
- Not in the files, because no sequence setting is read for them: the release gates' numbers, the set-down's and the re-acquisition's are constants in code (each feature's Decisions row says why).
- The competition-day `final_*` files have no 2027 twin: the spec names the drop and the delivery only.
