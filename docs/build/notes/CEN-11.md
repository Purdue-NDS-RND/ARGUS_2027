# CEN-11: Sim copies of the live configs

Flag: none (two new sequence files, a generator and two sim scenarios; no flight code and no existing file's value changes, and the copies list every flag false) · Status: PARTIAL (the 2026 files are done; the 2027 files do not exist until CEN-10, which adds their copies) · Commits: `git log --oneline --grep '^CEN-11:' platform/build-2027`

## What it solves

The sim flew the demo sequences, whose tuning is not the aircraft's (gain 0.25 against 0.15, deadband 0.15 m against 0.4 to 0.5 m, drop at 4 m against 6 m). So a sim result said little about what the live files do. Now each live file has a sim copy that differs from it only where the sim must, and a scenario that flies it.

## Architecture

- `tools/sim_sequence_copies.py` (new, ROS-free): makes `sim_<live file>` from a live sequence, line for line, changing only `target_latitude`, `target_longitude` (the sim's default), `fake_drop` (true), `camera_mode` and `airframe` (the sim's, for projection v2) and the note above the coordinates. It reads the result back and refuses to write a copy that differs anywhere else. `--check` (the default) fails on a stale copy; `--write` regenerates.
- `src/drone_mission_demo/config/sequences/sim_package_drop_live.yaml`, `sim_package_delivery_live.yaml` (new, generated, committed).
- `sim/scenarios/delivery/live_tuning.yaml`, `sim/scenarios/drop/live_tuning.yaml` (new), both in the nightly suite.

## Flow

Before: `sim/run_scenarios.sh delivery/baseline` → `package_delivery_demo.yaml` → the demo's tuning.

After: `sim/run_scenarios.sh delivery/live_tuning` → `sim_package_delivery_live.yaml` (the live file's every gain, height, deadband and dwell) → the builder sets the target's coordinates → the 2026 demo launch flies it with a fake release. Changing a value in the live file without `--write` fails `test_sim_sequence_copies.py`.

## Code

- `tools/sim_sequence_copies.py:30`, `COPIES`: live file → sim copy. CEN-10 adds the two 2027 files here.
- `tools/sim_sequence_copies.py:75`, `sim_copy_text`: the line-by-line copy; a live file with no `target_latitude` or no `fake_drop` is refused.
- `tools/sim_sequence_copies.py:109`, `_verify`: both files parsed and compared; anything but the five settings differing is an error.
- The scenarios name `launch:` because a sequence with no launch of its own falls back to `multi_mission_demo.launch.py`, which starts no detector.
- Four FND tests counted the shipped sequence files (10, or 6 with a vision mission) and listed which name a camera; they now expect the two copies too.

## Evidence

```
$ python3 -m pytest -q tools/test/test_sim_sequence_copies.py
19 passed in 0.41s
$ python3 tools/sim_sequence_copies.py --check
2 sim copies up to date
$ sim/run_scenarios.sh delivery/live_tuning --seeds 1 --variants off
| delivery/live_tuning | 1 | OK | 15.71 | 0.43 m from the target | not run |  |  |
$ sim/run_scenarios.sh drop/live_tuning --seeds 1 --variants off
| drop/live_tuning | 1 | OK | 17.16 | 0.27 m from the target | not run |  |  |
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh
OK
```

Numbers and what the live tuning does differently: `docs/results/2026-11-20-live-tuning.md`.

## Spec check

spec-auditor: INCOMPLETE on one row, as expected: the 2027 live files' copies cannot exist before CEN-10, so the row is PARTIAL and a test now fails if a `_live` or `_2027` sequence with a tracking mission has no copy. Its two deviations are recorded under Decisions: which files "the 2026 live files" are, and the sim camera's two settings in a copy. Not re-run (the fixes were a test and records).

flight-guardian: NO FINDINGS. Three observations taken up: the generator now refuses a live file in which any mission with a target lacks its own `fake_drop`; the drop scenario's comment said the live file stops steering at 8 m (it is 9 m, and the column is never reached); and the 2026 drop's unchecked release on a timer is noted in the results file.

## Not done

- The 2027 live files do not exist until CEN-10; their copies are one line each in `COPIES` then.
- The competition-day files (`final_curcuit_delivery.yaml`, `final_waypoint_drop.yaml`) are not copied: they start with a waypoint leg, and their tracking settings equal the `_live` files' except two delivery values (see the results file).
