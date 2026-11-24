# PLT-07: Launch, profiles and GO launcher

Flag: none of its own. It is how every other flag is turned on for a flight: with the profile `comp_2026` it starts the 2026 nodes with every flag false · Status: PARTIAL · Commits: `git log --oneline --grep '^PLT-07:' platform/build-2027`

## What it solves

In 2026 a flight was one of fourteen launch files, and a 2027 flag was turned on by typing its name on the launch line (mission flags) or by editing a params file (node flags). This is one launch for every flight and one command in front of it: the profile is checked against the files it will be flown with, the preflight must say GO, and only then is anything started.

## Architecture

- `drone_mission_core/bringup.py` (new, no ROS): a profile's `enable:`, `nodes:` and `start:` blocks turned into the launch's arguments, the list of nodes to start, and the `features.<flag>` overrides of each node; node flags checked against the params file.
- `drone_mission_demo/launch/argus_bringup.launch.py` (new): arguments `sequence`, `params`, `feature_overrides`, `node_flags`, `camera`, `detector`, `log_session`, `use_sim_time`. It starts what `bringup.py` says. The fourteen 2026 launch files are unchanged.
- `tools/go.py` (new): plan (no ROS) → the 2026 grabber, if the flight uses one → preflight (`ros2 run drone_utils preflight`, PLT-10) → `GO` and the launch, or `NO-GO` and nothing.
- `features.load_profile` allows the two new profile blocks. `profiles/delivery_estimator.yaml` (new) is a worked example with node flags.
- Sim: a scenario's `preflight.go` block has the GO command start the flight with a committed profile (`sim/scenario/spec.py`, `build.py`), the run's own sequence and params files keeping every flag false; `sim/runner/fly.py` ends a flight the command did not start as `NOT_STARTED` after watching the vehicle for 10 s. Scenarios `delivery/go`, `delivery/go_estimator`, `delivery/go_nogo`.
- `preflight_node.py` finds a pymavlink that is installed for the user where the environment hides user packages (the sim's image).

## Flow

- Before: `ros2 launch drone_mission_demo package_delivery_live.launch.py feature_overrides:=...`, after a person's look at the aircraft.
- After: `python3 tools/go.py --profile <name> --sequence <file> --params <file> ...` → the profile against the sequence and the params file (a flight that cannot be planned: `NO-GO`, exit code 2) → the 2026 grabber, if the flight uses one, started and given five seconds → the preflight's eleven lines → `NO-GO`, exit code 1, the grabber stopped, nothing else started; or `GO` → `ros2 launch drone_mission_demo argus_bringup.launch.py sequence:=... params:=... feature_overrides:=<mission flags> node_flags:=<node.flag,...>` → the executor gets the mission flags, each node its flags in a params file keyed by its name.

## Code

- What a profile starts and turns on: `src/drone_mission_core/drone_mission_core/bringup.py:165`; the nodes of a flight: `:96`; a node's flags as a params file keyed by its name, and why: `:117`; the rules a flight is held to: `:144` and `flight_problems` below it.
- The launch: `src/drone_mission_demo/launch/argus_bringup.launch.py:61`.
- The commands from the arguments: `tools/go.py:111`; a named file must be the installed one and the tree's: `:90`; the grabber, the preflight, then the launch: `:265`; waiting on the launch: `:206`; ending a grabber: `:244`.
- The sim's GO flight: `sim/scenario/build.py:448`; a flight that was not started: `sim/runner/watch.py:68` and `sim/runner/result.py:219`.

## Evidence

```
$ pytest tools/test/test_go.py src/drone_mission_core/test/test_bringup.py sim/test/test_preflight_check.py tools/test/test_flag_profile.py   (as scripts/check.sh runs them)
30 passed, 29 passed, 23 passed, 9 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh delivery/go --variants on             ->  OK   GO; the 2026 nodes, no flag; 16.21 points at 0.03 m, the baseline's score
$ sim/run_scenarios.sh delivery/go_estimator --variants on   ->  OK   GO; mission and node flags from the profile; 1766 of 1767 detections on frame stamps; estimator and projection checks pass
$ sim/run_scenarios.sh delivery/go_nogo --variants on        ->  OK   NO-GO: params, battery; exit code 1; nothing started, never armed
```

All three are of the final code. The sim found two faults the unit tests had not: `ros2 launch` refuses an empty `name:=`, and node flags given as a parameter dictionary lost to the params file under `use_sim_time` (the first flags-on flight flew with the detector's flags off). Both, and a grabber that would have outlived its NO-GO, are in `docs/results/2026-11-23-go-launcher.md`. The tests were written with their modules (drafted together outside the repo), not before them; the ones for the faults above were written after each was found.

## Spec check

- spec-auditor, round 1: INCOMPLETE. The preflight counted the camera's frames before the launch had started the 2026 grabber, so the aircraft's documented command could never say GO: `go` now starts the grabber first and stops it on NO-GO. Recorded under "Decisions": the launch takes flags and `go` takes the profile; `go` does not start MAVROS (a boot service since PLT-02).
- spec-auditor, round 2: INCOMPLETE on two points, both then closed and not audited a third time. The flights' results were placeholders (now filled, all of the final code). Stopping the `ros2 run` wrapper left the node running: confirmed on Humble, and `go` now ends the grabber's whole process group (tested with real processes, and against `ros2 run` in the sim's image).
- flight-guardian, round 1: CONCERNS, no blocker, six rows. Fixed: the camera order; a mission flag that reads frame times without the detector's stamp flag is refused, and `flag_profile.py` refuses a profile with node flags; `go` waits on the launch and passes a SIGTERM on; a file named is the installed one and must be the tree's; a drop or delivery with no detector is refused; the launch run by hand checks its node flags.
- flight-guardian, round 2: CONCERNS, no blocker, four rows, each narrowed from round 1. Fixed after the round, unit-tested, not reviewed again: the launch and `go` share one check that reads `feature_overrides` as the executor does and counts a flag the sequence itself has true; a file in the tree and not in the install is refused; mission types are read as the registry reads them; `--no-camera` is refused for a flight that starts the detector; a SIGTERM during the preflight still stops the grabber. Left: a waypoint flight through `go` gets a session logger the 2026 waypoint launch did not start (`--no-session-log`), and `go` checks the flight with the tree's `drone_mission_core` while the nodes run the installed build.

## Not done

- The aircraft (why PARTIAL): `go` has been run only in the sim's container. The bench run, props off, is under "Needs a person".
- Profiles for the competition's missions: which flags fly is the lead's choice. One example profile ships.
- The 2026 launch files are not retired or rewritten onto the bringup; they still work as they did.
- `go` does not start MAVROS or the 2027 camera driver: both are boot services (PLT-02). In the sim the sim's start script starts MAVROS.
- A 2026 grabber started by `go` has never run with a real camera, and the sim's recovery flights with the oracle stand-in still need their own launch file (it remaps the oracle's topic for the estimator).
- `go` checks a flight with the checked-out tree's `drone_mission_core`; the nodes run the installed build. A named sequence or params file is held to both; the code is not.
- The slot clock the launcher is to show is OPS-05's.
