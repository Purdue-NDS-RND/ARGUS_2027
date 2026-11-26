# PLT-08: Flight-controller baseline: fence, failsafes, gripper

Flag: none (a parameter file a person loads; nothing in the repo loads it, and the sim uses it only in three scenarios that ask for it) · Status: PARTIAL · Commits: `git log --oneline --grep '^PLT-08:' platform/build-2027`

## What it solves

In June 2026 the flight controller flew with the fence off, no failsafe with an action except the EKF's, the gripper disabled and a second GPS configured that was not fitted. Nobody kept the parameter file. This row writes down what 2027 needs, recovers what 2026 flew with from the flight's own log, and checks both. It fixes the cause on paper; the cause is fixed on the aircraft when a person loads the file.

## Architecture

- `platform/fcu/argus_2027.param` (new): 29 parameters with their reasons: fence, battery, radio, ground-station and EKF failsafes, `FS_OPTIONS` 16, the gripper on output 7, the rangefinder, `EK3_SRC_OPTIONS 0`, `GPS2_TYPE 0`.
- `platform/fcu/comp_2026.param` (new, generated): the 1055 parameters of the June 2026 flight, from `LOGS/00000031.BIN`.
- `tools/dataflash_params.py` (new, standard library): reads the PARM messages out of a dataflash log.
- `tools/fc_param_check.py` (extended): `--rules baseline` (eleven rules) and `--diff OTHER`.
- `sim/failsafe_check.py` (new) with three scenarios, `sim/scenarios/hover/failsafe_{battery,radio,ground_station}.yaml`; wired through `sim/scenario/spec.py`, `build.py` and `sim/runner/result.py`.
- `platform/fcu/README.md`: what to confirm before loading, and the bench and field steps.
- No node, topic or flight code.

## Flow

- Before: nothing said what the flight controller should be set to, and 2026's settings were known only to whoever set them.
- After, on the bench: save the aircraft's parameters → load `argus_2027.param` → upload the fence from the site file → save again → `fc_param_check.py <dump> --rules baseline --profile platform/fcu/argus_2027.param` → PASS → each failsafe tried with the safety pilot.
- In the sim: a failsafe scenario starts SITL on the baseline's values → `failsafe_check.py` reads them back, takes off, flies 15 m away, closes the gripper, takes away what the failsafe watches → the mode the baseline asks for must begin in time, with the gripper's output still at its grab value.

## Code

- Reading a dataflash log by its own FMT messages: `tools/dataflash_params.py:50`.
- The baseline's rules: `tools/fc_param_check.py:229` (the battery's at `:193`, the fence with its ceiling at `:182`); what a file changes in another: `:265`.
- The failsafe check: what is tripped and what must follow, from the baseline's own values: `sim/failsafe_check.py:71`; the verdict of a step: `:96`; the flight: `:116`.
- A failsafe flight counts only with its check passed: `sim/runner/result.py:283`.

## Evidence

```
$ python3 tools/fc_param_check.py platform/fcu/argus_2027.param --rules baseline | tail -1
PASS: platform/fcu/argus_2027.param (29 parameters)
$ python3 tools/fc_param_check.py platform/fcu/argus_2027.param --rules baseline --diff platform/fcu/comp_2026.param | grep -c '^diff [A-Z]'
21
$ pytest tools/test/test_fc_param_check.py tools/test/test_dataflash_params.py sim/test/test_failsafe_check.py   (as scripts/check.sh runs them)
52 passed, 9 passed, 15 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh hover/failsafe_radio            ->  OK   GUIDED -> RTL 1.0 s after the receiver went silent (limit 5 s)
$ sim/run_scenarios.sh hover/failsafe_ground_station   ->  OK   GUIDED -> RTL 4.7 s after the heartbeat stopped counting (limit 10 s)
$ sim/run_scenarios.sh hover/failsafe_battery          ->  OK   low: GUIDED -> RTL in 10.3 s; critical: RTL -> LAND in 10.2 s (limit 15 s each)
```

In all three the gripper's output stayed at its grab value: `docs/results/2026-11-23-fc-baseline-failsafes.md`. The 2026 set fails exactly eight of the eleven baseline rules (fence, battery, radio, ground station, gripper, rangefinder, EKF source options, second GPS), and passes them all with the baseline loaded over it: a test holds both. The failsafe check's tests ran first against a draft outside the repo; in the repo they were first run with the tool already there.

## Spec check

- spec-auditor, round 1: INCOMPLETE on three rows. Fixed: the failsafe check now needs the gripper's output seen at its grab value after the grab and at every step (an output that never moved used to read as "held"; two tests), and it counts anything but the grab value as let go; the EKF failsafe being set by parameter only, and the review of `FS_EKF_ACTION` being an argument that no flight has shown, are under "Decisions" and reworded in the parameter file; a wrong command in the tool's own text. No second round: one tested rule and wording.
- The three flights were made before the gripper rule was tightened; their reports show the output at 1900 after every trigger, which the tightened rule passes. `hover/failsafe_radio` was flown again with it: OK, RTL 1.0 s after the receiver went silent, the output at 1900 after the grab and 1900 after the trigger.
- No flight-guardian review: no mission, executor or detector code and no config under `src/` changed.

## Not done

- Field (why PARTIAL): loading the file on the aircraft, the fence upload from a verified site file, and each failsafe tried with the safety pilot ("Needs a person"; the steps are in `platform/fcu/README.md`).
- Five values are the session's working values, marked "to confirm" in the file: the battery thresholds (a 6S pack by the 2026 logs), the gripper's output and PWMs, the rangefinder's serial port, whether the ground-station failsafe fits the competition's rule, and the fence's ceiling (the site file's placeholder).
- `mav.parm` from the 2026 team is still not collected; the log's 1055 parameters stand in for it, and `--diff` compares against whichever arrives.
- The EKF failsafe is not tripped in the sim: its action (Land) is 2026's and unchanged. The review the spec asks for is written in the parameter file beside it.
- MAVROS's `rc_io` plugin (servo outputs in recordings), which SIM-06 left for this row, is not added: a plugin in the list runs with every flag off, so it is the lead's to add ("Needs a person").

## After the build (2026-11-25)

- The drop's release: the drop mission sends `MAV_CMD_DO_SPRAYER`, which ArduPilot 4.6.3 acknowledges with the sprayer disabled, so the file as first written left the aircraft's drop doing nothing. It now sets `SPRAY_ENABLE 1`, `SPRAY_SPEED_MIN 0`, `SPRAY_PUMP_MIN 100` and `SERVO8_FUNCTION 22`, the sim's set (`sim/payload_bridge.py`), and the baseline's rules have `sprayer` (`tools/fc_param_check.py`, `_sprayer`). Output 8 is the sim's number: the mechanical team's replaces it. 33 parameters, twelve rules; the 2026 set fails nine of them.
- The rangefinder: `RNGFND1_MAX_CM` 1200 -> 600 (ArduPilot's TFmini page gives about 6 m outdoors), here and in `docs/fc/recovery.parm`; `SERIAL4_PROTOCOL 9` and `SERIAL4_BAUD 115` added, marked `PORT TO CONFIRM (lead)` (4 is the GPS2 connector, unused in 2026). The baseline's `rangefinder` rule now wants a serial port at 115 for a TFmini (`_rangefinder_with_port`). 35 parameters.
- The load is two passes (`platform/fcu/README.md`): `GRIP_ENABLE`, `RNGFND1_TYPE`, `SPRAY_ENABLE`, `FLOW_TYPE`, `PLND_ENABLED` and `SCR_ENABLE` hide the rest of their groups until they are set and the flight controller rebooted (the 2026 log's parameters show each switch at 0 with nothing else of its group). `fc_param_check <dump> --profile ...` prints `FAIL hidden: <switch> is <value> and the rest of <group>* is not in the file` for any group still hidden (`hidden_groups`).

```
$ python3 tools/fc_param_check.py platform/fcu/argus_2027.param --rules baseline | tail -1
PASS: platform/fcu/argus_2027.param (35 parameters)
$ pytest tools/test/test_fc_param_check.py
63 passed
```
