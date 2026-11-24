# OPS-03: Post-flight report and autonomy audit

Flag: none (a tool run on an attempt's bundle after the flight; it reads, and writes two files into the bundle) · Status: DONE · Commits: `git log --oneline --grep '^OPS-03:' platform/build-2027`

## What it solves

The autonomy bonus is 25% on four missions and is lost to one human input. After a flight nobody could show, from a record the organisers would accept, that there had been none; and what the flight scored was a guess until the judges said.

## Architecture

- `tools/post_flight.py` (new, no ROS): reads the bundle PLT-12 makes and writes `report.md` and `report.json` into it. Everything is from the flight controller's own dataflash log (read with `tools/dataflash_params.py`) and the bundle's config.
- It uses the sim's judges (`sim/judge`, SIM-08) for the score estimate: the log's track and events are turned into the lines a judge takes.
- `tools/bundle_attempt.py`: the route file a sequence names goes into the bundle (`config/waypoints.yaml`), so a waypoint flight can be scored.

## Flow

- Before: "I don't think anyone touched the sticks."
- After: `python3 tools/post_flight.py attempts/<attempt>` → the attempt (mission, commit, profile, preflight) → what happened (arming, each mode with its reason, errors, the flight controller's own text, time, height, battery) → the audit of the armed window → the score estimate → `NO HUMAN INPUT FOUND` (exit 0) or `HUMAN INPUT: n finding(s)` (exit 1).

## Code

- The log, read once: `tools/post_flight.py:121`. The window (the armed period the vehicle flew in): `:187`.
- The audit: `:257`; the modes the stack asks for, against which a MAVLink mode change is held: `:360`.
- The first command that lets a payload go: `:196`. The log as a judge's record: `:325`. The estimate by mission: `:386`.

## Evidence

```
$ pytest tools/test/test_post_flight.py tools/test/test_bundle_attempt.py   (as scripts/check.sh runs them)
23 passed, 19 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ python3 tools/post_flight.py <a sim delivery attempt's bundle> | tail -1
NO HUMAN INPUT FOUND
$ python3 tools/post_flight.py <the fence demonstration's bundle> | grep -A3 '^## Score'
## Score estimate

By the sim's geofence judge on the log's own track and the fence polygon the log holds:
- 1.00 points (1.00 and 0.00 autonomy bonus).     details: held 1.88 m inside the fence for 37.1 s
```

The second is a cross-check that cost nothing: the sim's judge on Gazebo's true positions gave the same flight 1.87 m for 37.2 s (`docs/results/2026-11-23-readiness-demo.md`), so the judge fed from the flight controller's log agrees with the truth to a centimetre. Both real logs are sim flights: armed in GUIDED by the companion (system 1, component 191), RTL asked over MAVLink, four commands, all the companion's. The tests were written after the tool; each kind of human input is a test on a log built byte by byte.

## Spec check

- spec-auditor, round 1: INCOMPLETE on ten rows. Fixed: the drop and the delivery go through the real judges, with their 6 ft rules, from the first command that lets go (a grab and the drop servo's return are not releases); the rangefinder's reading at the release is in the report; a log with several armed periods is audited for the one that flew; an arm from the transmitter is found; a MAVLink mode change to a mode the stack never asks for is a finding, and the verdict counts the ones it cannot attribute; parameter writes are listed; the usage line. Recorded under "Decisions": the window, what is not an input, what this log cannot attribute.
- spec-auditor, round 2: INCOMPLETE on one row, then closed and not audited a third time: the stack's own AUTO (time trial) and LAND (recovery abort) are set in code and would have read as human input. The expected modes are now the ones the code asks for, held to the source by a test, and three flights go through the command (LAND and AUTO clean, LOITER found).
- No flight-guardian review: no mission, executor or detector code and no config under `src/` changed.

## Not done

- A real aircraft's log has not been through it. The stick rule in particular has seen only SITL's steady RC values ("Needs a person").
- A mode changed over MAVLink to a mode the stack asks for cannot be attributed from this log: an operator pressing RTL in the ground station looks like the mission's own RTL. The report lists every such change and says so; the ground station's log settles it. The same holds for a position target sent as a plain message.
- The drop and the delivery are scored from where the vehicle was when it let go, not from where the payload came down, which no log holds. The sim's delivery attempt released nothing (its sequence is a sim copy with `fake_drop`), so that path has run only on built logs.
- The localisation and the recovery are not estimated: their judges need the markers' and the bucket's true places.
- The release-verification result FEATURES.md says goes into the report (the payload seen to leave) is not in it: no feature produces that result yet.
