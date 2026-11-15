# SIM-07: Headless scenario runner

Flag: none (sim tooling). It flies each scenario with flags off (`--variants off`, the 2026 behaviour) or on · Status: DONE · Commits: `git log --oneline --grep '^SIM-07:' platform/build-2027`

## What it solves

Every sim flight needed a person: start the sim, wait, launch the mission, watch, read the distance off a log. `sim/run_scenarios.sh` does all of it unattended and scores each flight from Gazebo's true positions, so later features can prove themselves with a command. It fixes the cause, not a symptom: nothing in the loop waits for a keyboard.

## Architecture

- `sim/run_scenarios.sh` (host): for each flight of a suite or scenario, build the run folder, start a fresh container detached, wait for its READY file, fly, audit, judge, report, tear down; then a suite table.
- `sim/runner/` (ROS-free unless noted): `helpers.py` (the tools run beside a flight, the judges'-list capture), `suites.py` (what a suite name stands for), `events.py` (log lines and autopilot text to trace events, ground-station commands), `trace.py` (the trace writer, real-time factor), `watch.py` (when the flight is over, in sim time), `result.py` (status, `report.md`, the suite table), `gcs_audit.py` (human input from the MAVLink log; needs pymavlink), `fly.py` (the in-container wrapper: Gazebo Python bindings for true poses and sim time, rclpy for arm state, mission log and autopilot text).
- `sim/scenarios/suites.yaml`: `smoke`, `nightly`, `gates`.
- `sim/run_sim.sh --detach`; `start.sh` writes `<run folder>/READY`.
- Scenario key `driver` and scenario `geofence/breach`; judge `geofence` in `sim/judge` (see "Spec check").

No new dependency: Gazebo's Python bindings and pymavlink are already in the sim image.

## Flow

Before: `sim/run_sim.sh` in one terminal, `ros2 launch ...` in another, eyes on the log.

After: `sim/run_scenarios.sh smoke` builds `delivery/baseline`, starts the container, sees READY after about 40 s, and runs `fly.py`. That records 19 true vehicle poses a second and a clock mark every 5 s of sim time, launches the 2026 delivery launch file with the generated sequence, writes `armed`, `release` ("Fake delivery complete") and `disarmed` events, and stops 5 s of sim time after the disarm. `gcs_audit.py` finds no ground-station command, the judge writes `score.json` (0.02 m from the target), and `result.py` writes `report.md` and the table. About 3.5 minutes, exit code 0.

## Code

- `sim/runner/watch.py:16` `EndWatch`: flight over, timed out or never armed, all on sim time; "on the ground" is the autopilot's landed state or, when that is not streamed, the true height.
- `sim/runner/fly.py:44` `Recorder`: true poses from `dynamic_pose/info` at 20 Hz, everything else once a second from `pose/info`, sim time from the world clock.
- `sim/runner/result.py:73` `run_result`: OK, TIMEOUT, NEVER_ARMED, INVALID (real-time factor below 0.8 while armed, or a scenario the sim cannot fly faithfully) or ERROR.
- `sim/runner/events.py:81` `human_inputs` and `:110` `unsupported`: which MAVLink senders and messages are a person; and the refusal to fly a windy or GPS-fault scenario calm.
- `sim/run_scenarios.sh` `fly_one`: a fresh container before every flight and always torn down after, whatever failed.

## Evidence

```
$ python3 -m pytest -q -p no:cacheprovider sim/test/test_runner.py sim/test/test_scenario.py sim/test/test_judge.py sim/test/test_judge_cli.py
212 passed in 7.76s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ for i in 1 2 3; do sim/run_scenarios.sh smoke; echo "exit code $?"; done
   OK          delivery/baseline seed 1 off: 16.22 points, 0.02 m from the target
exit code 0
   OK          delivery/baseline seed 1 off: 16.22 points, 0.03 m from the target
exit code 0
   OK          delivery/baseline seed 1 off: 16.22 points, 0.02 m from the target
exit code 0
$ sim/run_scenarios.sh geofence/breach
   OK          geofence/breach seed 1 off: 1.00 points, response -0.02 s from the line, 1.07 m outside
```

One more smoke flight after the last refactor of `fly.py`: OK, 0.09 m, 16.14 points.

Full output and the earlier runs: [docs/results/2026-11-15-scenario-runner.md](../../results/2026-11-15-scenario-runner.md). `test_runner.py` was run before `sim/runner` existed and failed (`ModuleNotFoundError`).

Three bugs were found by the live runs, not by the tests, and fixed: a QoS mismatch that left the autopilot's text unrecorded, an abort on exit in one run of three, and the end-of-flight check never seeing "on the ground" for a flight not started by the 2026 takeoff service.

## Spec check

spec-auditor: INCOMPLETE in round 1. The pass line was met, but three steps of a run were only partly proven: starting and stopping the tools beside a flight and the judges'-list copy sat untested inside the ROS wrapper, and the path where the sim never gets READY had never run. It also found that the sleep lock broke when the script was called by a relative path, and that the geofence judge let a prompt breach report excuse a late turn back. COMPLETE in round 2: both pieces moved into `sim/runner/helpers.py` with tests, the host script is now tested end to end against a stand-in sim (which found a missing teardown), the lock re-runs the script by full path, and the judge bounds the mode change.

flight-guardian did not run: no mission, executor, target CV or `config/` YAML changed.

Beyond the spec, recorded under Decisions: the scenario `driver` key, the `geofence/breach` scenario and the `geofence` judge (committed as SIM-08 and SIM-02 follow-ups), which give SIM-02's pass line its scored run from true positions.

## Not done

- `fly.py` itself (the rclpy and Gazebo wrapper) has no unit test; it is proven by the live runs above. Everything it decides is in tested modules.
- Wind and GPS-event scenarios are reported INVALID until `sim/wind.py` (SIM-03) and `sim/inject.py` (SIM-04) exist. Starting and stopping such tools is tested with stand-in scripts; the live run is owed to those features.
- The judges'-list capture is tested on files only; no mission writes a list until the localisation lane.
- `contact` events (the delivery impulse) are not recorded; SIM-06 owes them.
- Only `delivery/baseline` and `geofence/breach` have flown through the runner, one flight per suite. A multi-flight suite and a sim that never gets READY have run only against a stand-in sim in the tests; G0 (five flights) is next, in SIM-11.
- The INVALID rule for a slow sim is covered by unit tests only; no run was that slow.
- True poses are not on a ROS topic; SIM-05 bridges them to `/sim/truth/poses` for the oracle detector.
- The ground-station audit has a positive control with our own tool's system id, not with a real ground station.
