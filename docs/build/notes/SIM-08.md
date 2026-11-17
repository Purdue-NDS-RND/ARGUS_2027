# SIM-08: Judges that score like the organisers

Flag: none (sim tooling, no flight behaviour) · Status: DONE · Commits: `git log --oneline --grep '^SIM-08:' platform/build-2027`

## What it solves

The sim could fly a mission but nothing said how many points the flight was worth, so every run needed a person to watch it. This adds a judge per mission that scores a recorded run from Gazebo's true positions, never the EKF, using the scoring table in FEATURES.md. It fixes the cause: every later sim gate reads its pass or fail from these scores.

## Architecture

New ROS-free package `sim/judge/`:

- `record.py`: the run record, true poses and events on sim time, read from `trace.jsonl`.
- `missions.py`: one judge per mission (waypoint navigation, time trial, localisation, drop, delivery, recovery).
- `score.py`: default settings, the checks every mission gets (flights, fence breaches, failsafes, autonomy bonus), and `judge_run`.
- `hungarian.py` (numpy) and `geometry.py`: list matching and fence polygons.
- `cli.py`: `python3 sim/judge/cli.py <run folder>` reads `judge.yaml`, `truth.json`, `trace.jsonl` and writes `score.json`.
- `README.md`: the run-folder files and the `truth.json` keys, which SIM-01 and SIM-07 write to.

It reuses `sim/target_placement.py` to turn the list's latitude and longitude into metres. No new dependency, node or topic.

## Flow

Before: a delivery run ended with a log; someone read the touchdown distance off it by hand.

After: the run folder holds `trace.jsonl`. `cli.py` finds the first arm-to-disarm flight that left the ground, takes the vehicle's true position at the `release` event (a fake release), measures 0.10 m to the target in `truth.json`, and writes `score.json` with `13 − 0.10`, the weight bonus, the 25% autonomy bonus, and the flight, fence and failsafe report.

## Code

- `sim/judge/missions.py:94` `pass_in_order`: waypoints count only in the given order, measured on the straight leg between pose samples so a low sample rate cannot skip a waypoint.
- `sim/judge/missions.py:201` `score_list`: the Hungarian method picks the pairing of list rows to true markers that loses the fewest points, with distance as the tie-break; rows that pair with nothing are wrong on both counts.
- `sim/judge/missions.py:299` `judge_drop` and `:326` `judge_delivery`: distance from the payload at rest, or from the vehicle at a fake release; the 1.83 m rules on release and flight height.
- `sim/judge/record.py:143` `flights`: a flight is an arm-to-disarm span in which the vehicle left the ground, so delivery's touchdown and relaunch is still one flight.
- `sim/judge/score.py:90` `judge_run`: merges settings (an unknown name is an error), scores only the first flight, and adds the bonus only when that flight has no `gcs_command` or `rc_override` event.

## Evidence

```
$ python3 -m pytest -q -p no:cacheprovider sim/test/test_judge.py sim/test/test_judge_cli.py
84 passed in 1.13s

$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -3
Not run (need ROS): src/drone_live_tests/test/test_live_landing_test_mission.py, src/drone_utils/test/test_stream_contract.py, src/drone_utils/test/test_{copyright,flake8,pep257}.py

OK
```

One rule broken on purpose (`low_release_penalty` 6 → 5 in `score.py`): `test_drop_release_below_six_feet_costs_six_and_being_below_six_feet_costs_six` failed; restored.

The tests were written before the judges but first run after them, so "seen failing first" rests on that one deliberate break, not on a red first run.

## Spec check

spec-auditor: INCOMPLETE in round 1 (thresholds not pinned by tests, time-trial points and open scoring choices not recorded), COMPLETE in round 2 after adding near-threshold tests and nine Decisions rows. flight-guardian did not run: no flight code or config changed.

Choices the spec left open are in PROGRESS.md under Decisions. One needs the lead's eye: localisation penalties are outright (−0.5, −0.5, −1), not deductions from a marker's point. The autonomy bonus applies only to localisation, drop, delivery and recovery, as the rules say; waypoint navigation and the time trial get none (fixed 2026-11-17).

## Not done

- Nothing writes `trace.jsonl` yet. SIM-07 owes the recorder, and the copy of the judges' list as it stood at landing; SIM-01 owes `truth.json` and `judge.yaml`.
- Points the rules have not published (per waypoint, per marker, recovery per metre, time-trial reference times) are settings with default guesses.
