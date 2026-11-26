# PLT-10: Preflight go/no-go

Flag: none (a command a person or the `go` launcher runs before a flight; it starts nothing and no mission reads it) · Status: PARTIAL · Commits: `git log --oneline --grep '^PLT-10:' platform/build-2027`

## What it solves

In 2026 "ready" was a person's reading of several terminals, and `preflight_checks.sh` checked only that the installed config files matched the source. This is one command that looks at the aircraft and says GO or NO-GO, check by check, and logs what it found and who overrode it.

## Architecture

- `drone_utils/preflight.py` (new, no ROS): `Snapshot` (what was read), `Limits`, eleven checks in the spec's order, `verdict` (with the override), the log record, and the parameter and profile helpers.
- `drone_utils/preflight_node.py` (new; console script `preflight`): listens for 5 s on `/mavros/sys_status`, `/mavros/gpsstatus/gps1/raw`, `/mavros/rangefinder/rangefinder`, `/mavros/battery`, `/mavros/timesync_status` and each camera topic named, fills the snapshot, prints, logs to `<log dir>/preflight.jsonl`.
- Sim: `sim/preflight_check.py` (new side tool) runs the real command before a flight whose scenario has a `preflight:` block and holds it to what the scenario expects; scenarios `delivery/preflight` (GO) and `hover/preflight_nogo` (NO-GO).
- No topic or message is added. Of the four MAVLink messages it reads, one that is not arriving is asked for once, at the stream keeper's rate, from a disarmed flight controller only.

## Flow

- Before: MAVROS up, a look at the ground station, launch.
- After: `ros2 run drone_utils preflight ...` → eleven lines, each `[   GO]` or `[NO-GO]` with what was read → `GO`, `NO-GO: <checks>`, or with `--override "<reason>"`, `NO-GO OVERRIDDEN by <user>: ...` → exit code 0 or 1 → a line in `preflight.jsonl`. A check that had nothing to read is NO-GO.

## Code

- The checks and their order: `src/drone_utils/drone_utils/preflight.py:276`; the verdict and the override: `:289`; the log an override must reach: `:319`; the flight controller's values against the baseline's: `:349`; a preflight that could not look: `:362`.
- What is listened to: `src/drone_utils/drone_utils/preflight_node.py:123`; what may be asked of the flight controller, and when: `:54` and `:61`; the parts that need no ROS (a dump, the disk, the clock, the engine, the profile): `:242`; the print, the log and the exit code: `:297`.
- The sim's comparison: `sim/preflight_check.py:39`.

## Evidence

```
$ pytest src/drone_utils/test/test_preflight.py sim/test/test_preflight_check.py   (as scripts/check.sh runs them)
63 passed, 11 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh delivery/preflight --variants on     ->  OK   GO, eleven checks; the delivery 16.21 points at 0.03 m (the baseline's score)
$ sim/run_scenarios.sh hover/preflight_nogo --variants on   ->  OK   NO-GO: params, battery (SITL's own parameters and 12.6 V), exactly the two expected
```

The two outputs, what the sim does not show (the time sync reads 0.0 ms on the sim's clock; the preflight runs beside the flight, not before it) and the four flights that went wrong on the way: `docs/results/2026-11-23-preflight.md`. The rule tests were written before `preflight.py`; the command's and the sim tool's tests with their modules.

## Spec check

- spec-auditor, round 1: INCOMPLETE on six rows, all acted on. The command no longer lowers a message rate (it asked for the rangefinder at 10 Hz where the missions fly on 20): what has not been heard after 1.5 s is asked for once at the stream keeper's rates, and nothing else. With no camera named the camera check is NO-GO unless `--no-camera` says so. The rangefinder's line needs the flight controller's word that the sensor has data, and the fence's line needs stored fence points where the parameters were read. A preflight that could not look is printed and logged as NO-GO. The bench steps say what must fail. No second round.
- flight-guardian, round 1: CONCERNS, no blocker, nine rows, most the same as the audit's. Fixed besides: nothing is asked of a flight controller that is armed or whose state is not known; an empty baseline is NO-GO; the profile's line lists what will be on in flight, a flag the sequence itself has true included; the largest time-sync step is bounded (100 ms) beside the 90th percentile. Recorded under "Decisions" and left: the engine and the companion's clock pass by saying so when nothing is given to check them against; a dump's age is its file's modification time; the readings are up to 2 s old when the verdict is printed, and the parameter read can add seconds. No second round: the fixes are tested rules.

## Not done

- Bench (why PARTIAL): the command against the real flight controller, GPS, rangefinder and cameras ("Needs a person").
- How parameters are read on the aircraft is the lead's choice: a daily dump works today; MAVROS's param plugin is used when it is loaded, and is not in the aircraft's plugin list.
- The engine check has no registry to check against (DAT-04 is on hold): it compares a file's sha256 with one given on the command line.
- Nothing calls the preflight yet: the `go` command (PLT-07) does, and blocks the start on NO-GO.
- `preflight_checks.sh` (2026: installed configs against source) is untouched and is not one of the eleven checks.

## After the build (2026-11-25)

- A twelfth check, `actuators` (`drone_utils/preflight.py`, `_actuators` and `sequence_actuators`): the preflight reads what the sequence's missions command (`package_drop` the sprayer, `package_delivery` and `package_recovery` the gripper; a drop or delivery with `fake_drop: true` nothing) and is NO-GO when the flight controller reports `SPRAY_ENABLE` not 1 or no `SERVOn_FUNCTION 22` for a drop, or `GRIP_ENABLE` not 1 for a delivery or a recovery. The reason: ArduPilot 4.6.3 acknowledges `MAV_CMD_DO_SPRAYER` with the sprayer disabled, so the mission cannot tell. The settings are read from where the parameters are (`--fc-url`, MAVROS's param plugin, or the dump), and only when the sequence commands an actuator. A sequence that was not read, or settings that were not, are NO-GO.
- Evidence: `pytest src/drone_utils/test/test_preflight.py` -> 78 passed (67 of them failed before the check existed). The 2026 aircraft's own settings (`SPRAY_ENABLE 0`, `GRIP_ENABLE 0`) refuse `package_drop_live.yaml` and `package_delivery_live.yaml` and let `waypoint_navigation_live.yaml` go.
- Not verified: the read through MAVROS's param plugin (it needs ROS; the MAVLink and dump paths are tested), and the check against the real flight controller.
