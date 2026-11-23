# PLT-05: Latency probe (LED)

Flag: `command_step_probe` (mission flag, default false) for the one part that flies; the LED probe and the bag analysis are tools · Status: PARTIAL · Commits: `git log --oneline --grep '^PLT-05:' platform/build-2027`

## What it solves

Two delays the 2027 stack assumes and nobody has measured: how late a camera's capture stamps are (`capture_offset_s`, 0 in every mode today) and how long the flight controller takes to answer a command (FND-06 forecasts over 0.08 s). This is the tooling that measures both, and the 150 ms capture-to-command line; the measurements on the aircraft are a person's.

## Architecture

- `argus_camera/latency_probe.py` (new, numpy only; console script `latency_probe`): blinks an LED on a Jetson pin from its own thread, finds the first lit frame of each blink, and gives how late the stamps are. `--analyse` reads a saved run.
- `sim/replay/command_delay.py` (new, on SIM-09's bag reader): from a flight's bag, capture to command (a frame's stamp to the first setpoint after its detection was heard; line 150 ms at p95) and the actuation delay (a command step to the start of the tilt).
- `drone_mission_core/step_probe.py` (new, ROS-free) and the mission `command_step_probe` (`missions/step_probe_mission.py`): velocity steps in a hover, there and back. A delivery's own commands have no steps to time.
- `config/sequences/command_step_probe.yaml` (new; takeoff, probe, RTL; every flag false) and the scenario `sim/scenarios/hover/step_probe.yaml`.
- No new topic or message. No 2026 file changed except the missions' import list.

## Flow

- Camera offset, on the bench: LED on at a random moment → the first frame whose stamp shows it lit → over 200 blinks those frames follow the LED by half a frame period on average if the stamps are right → the excess is the offset → set `capture_offset_s`, run again, residual within 5 ms.
- Actuation delay: `command_step_probe` on → 3 s at zero velocity, 1 m/s for 1 s, six times, alternating → the bag → each edge of the command to the start of the tilt.

## Code

- The estimate, with dropped frames set aside and locked blinks refused: `src/argus_camera/argus_camera/latency_probe.py:90`; the 5 ms verdict and what to set: `:132`.
- The blinker's thread (the blinks must not follow the frame reads): `src/argus_camera/argus_camera/latency_probe.py:186`.
- Capture to command: `sim/replay/command_delay.py:118`. The start of the tilt, carried back along a fitted line from where it clears the noise: `:155` and `:183`.
- The steps' rule: `src/drone_mission_core/drone_mission_core/step_probe.py:86`. The mission, which stops commanding when the mode is not GUIDED: `src/drone_mission_demo/drone_mission_demo/missions/step_probe_mission.py:90`.

## Evidence

```
$ pytest src/argus_camera/test/test_latency_probe.py sim/test/test_command_delay.py src/drone_mission_core/test/test_step_probe.py src/drone_mission_demo/test/test_step_probe_mission.py   (as scripts/check.sh runs them)
21 passed, 13 passed, 22 passed, 17 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh hover/step_probe --variants on --record
   OK          hover/step_probe seed 1 on: 1.00 points, held within 1.03 m for 60 s
$ python3 sim/replay/command_delay.py sim/.state/suites/hover-step_probe-20261123T133104Z/runs/hover-step_probe-s1-on/bag
actuation delay: median 79 ms, max 96 ms over 12 of 12 command steps (FND-06 forecasts over 80 ms)
```

Sim numbers, with two recorded delivery flights' capture to command (p95 102 and 92 ms against 150): `docs/results/2026-11-23-latency-probe.md`. The mission and rule tests failed before their modules existed; the LED and bag tests were written with their modules and first run against them.

## Spec check

- spec-auditor, round 1: INCOMPLETE on five rows, all fixed: the two-leg reading of the second test is under "Decisions"; the probe no longer suggests a negative `capture_offset_s` (which `modes.yaml` refuses) and says what an early stamp means; a test pins the 5 ms limit; a test runs the live branch (`GpioLed`, the camera, the saved record) on a stand-in GPIO module; the bag tool counts and prints the detections it left out for want of a setpoint within 200 ms. No second round: tests and wording.
- flight-guardian, round 1: BLOCKER. On a mode that was not GUIDED the mission returned FAILURE, and the sequence's `abort_and_rtl` would have had the executor ask for RTL over the safety pilot or a failsafe. Two concerns: the pose's age was not checked, and the steps were timed off a clock that could step. Fixed: a takeover makes the mission go quiet and wait (no failure, no mode asked for), and GUIDED again ends it holding; steps are commanded only on a pose under 0.5 s old from a connected flight controller; a clock that steps ends the probe.
- flight-guardian, round 2: CONCERNS; the three rows resolved, one new concern: the hand-back hold could go out at the context's default heading (a turn to north). Fixed after the round and not reviewed again: the hold takes the pose's own heading, and with no pose nothing is commanded; the test vehicle now faces 30 degrees so the default would show. A pose that returns after more than a second now fails with that reason instead of "the clock moved".
- The second sim flight was made after round 1's fixes; the hand-back fix after round 2 is covered by unit tests only.

## Not done

- Bench (why PARTIAL): the LED on a pin, the probe run in each sensor mode, `capture_offset_s` set in `modes.yaml` from it and the residual within 5 ms. `Jetson.GPIO` and the GStreamer backend have not been run by this session.
- Field: `command_step_probe` flown on the aircraft with the bag recorded, for the aircraft's actuation delay; a delivery flight's bag for the aircraft's capture to command.
- FND-06's 0.08 s is not changed by this row: it is a tuned value, and the sim's 79 ms agrees with it.
- A flags-off flight cannot be followed frame to command (2026 detections carry the detector's clock); the tool says how many it could not follow.
