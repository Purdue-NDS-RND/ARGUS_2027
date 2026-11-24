# PLT-12: Logging and attempt bundle

Flag: none (three tools a person runs around a flight; `go` records only when given `--record`) · Status: PARTIAL · Commits: `git log --oneline --grep '^PLT-12:' platform/build-2027`

## What it solves

After a 2026 flight the evidence was in four places: a bag if someone had started one, the SD card, the session logger's folder and whatever the repo was at. This puts one attempt in one folder, `attempts/<date>_<slot>_<mission>/`, with a manifest that says what is there, what is missing, which commit and which flags flew.

## Architecture

- `tools/bundle_attempt.py` (new, no ROS): copies or moves the parts into the folder and writes `manifest.json`: every file with its size and sha256, the git commit, the flag profile, what the preflight last said, and what the mission still owes. `--verify` holds a folder to its manifest. `--from-run` takes the parts from a flight's log folder.
- `tools/dataflash_pull.py` (new): pulls a log from the flight controller over MAVLink (list, data in 90-byte packets, gaps asked for again). The protocol takes any pymavlink-like link, so it is tested with a stand-in.
- `tools/go.py`: `--record FOLDER` records the flight's bag to `FOLDER/bag` from GO to the end of the launch, with the recorder's own topic list (SIM-09's: no images), sends the preflight's log to the same folder, and writes `FOLDER/go.json`: the profile, sequence and params it flew.
- Sim: `preflight.go.attempt: <slot>` in a scenario; `sim/attempt_check.py` (new) pulls SITL's log after the landing, makes the bundle and checks it; scenario `delivery/attempt`.
- `attempts/` is in `.gitignore`.

## Flow

- Before: land → find the bag, pull the SD card, remember the commit.
- After: `go ... --record <folder>` (the bag and `preflight.jsonl` land in `<folder>`) → land, disarm → `dataflash_pull.py --fc-url <link> --out <folder>/dataflash` → `bundle_attempt.py --slot 2 --mission delivery --from-run <folder>` (the config comes from `go.json`) → `attempts/2027-05-21_2_delivery/` with `bag/`, `dataflash/`, `config/`, `preflight.jsonl`, `manifest.json` (and `frames/`, `judges_list.csv`, `session/` when given) → `COMPLETE`, or `INCOMPLETE: <parts>` with the bundle still made.

## Code

- The bundle and its manifest: `tools/bundle_attempt.py:200`; a bag's topics and storage from its metadata, images refused: `:112`; the folder against its manifest: `:317`; a flight's log folder, with the GO command's own record: `:338`.
- The pull: `tools/dataflash_pull.py:58` (the list at `:37`).
- The recording from GO to the end of the launch, and `go.json`: `tools/go.py:339` (the recorder's command at `:196`).
- The sim's verdict on a bundle: `sim/attempt_check.py:60`.

## Evidence

```
$ pytest tools/test/test_bundle_attempt.py tools/test/test_dataflash_pull.py sim/test/test_attempt_check.py tools/test/test_go.py   (as scripts/check.sh runs them)
18 passed, 6 passed, 7 passed, 32 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh delivery/attempt --variants on
   OK          delivery/attempt seed 1 on: 16.18 points, 0.06 m from the target
pulled 00000324.BIN: 8.20 MB
[   in] bag          9.2 MB
[   in] dataflash    8.2 MB
[   in] config       0.0 MB
[   in] preflight    0.0 MB
WARNING: the bag is sqlite3, not MCAP: the recorder found no MCAP plugin where it ran
COMPLETE
ATTEMPT CHECK: PASS
```

The pulled log is byte for byte the file SITL wrote; the bag has 14 topics, 72 432 messages and no image: `docs/results/2026-11-23-attempt-bundle.md`. The bundler's and the pull's tests were written with their modules outside the repo and first run there; the sim's and `go`'s were written with the wiring.

## Spec check

- spec-auditor, round 1: INCOMPLETE on five points. Fixed: the bundle takes its profile, sequence and params from `go.json`, which the GO command writes in the flight's folder, the manifest keeps the profile's own name, and config that is not what the flight's own records name is a warning (the sim's check fails on one); an unclosed bag no longer stops the bundle; the manifest records the bag's storage. Recorded under "Decisions" and here as owed: MCAP is unproven in the sim (sqlite3 there), and frames and the judges' list have not been in a sim bundle. No second round: the fixes are tested rules, and the attempt was flown again on them.
- No flight-guardian review (see "Not done").

## Not done

- The aircraft (why PARTIAL): the recorder beside a real flight, the pull over a real link, a bundle of a real attempt ("Needs a person").
- MCAP in the sim: the sim's image has no MCAP plugin, so the sim's bag is sqlite3. The recorder asks for MCAP where it is installed; the Jetson's package list has it.
- Frames: only the survey writes a frame log, so only the localisation owes frames. Frames of the other missions go in when a frame log is given; nothing writes one for them yet.
- No survey has been flown as an attempt: frames and the judges' list in a bundle are unit-tested only.
- `go --record` is not a flag in a profile: recording is asked for per flight on the command line.
- No flight-guardian review: no mission, executor or detector code and no config under `src/` changed. The recorder is a new listener beside the flight, and its cost on the Jetson is unmeasured ("Needs a person").
