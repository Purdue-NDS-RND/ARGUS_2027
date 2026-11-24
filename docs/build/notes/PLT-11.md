# PLT-11: Health monitor and status to the ground station

Flag: `features.health_monitor` (node flag of the new node `health_monitor`, default false) · Status: PARTIAL · Commits: `git log --oneline --grep '^PLT-11:' platform/build-2027`

## What it solves

A camera that has slowed, a computer that is throttling, a node dropping frames: in 2026 the first anyone knew of these was the flight going wrong, or the log afterwards. Now something watches them and says so on the ground station in one short line, while there is still time to do something about it.

## Architecture

- `drone_utils/health.py` (new, no ROS, standard library only): `Rate`, `Computer` (thermal zones, throttling, CPU load, the Jetson's GPU load), `Health` (what has been wrong for five seconds), `Reporter` (which status text goes out now).
- `drone_utils/health_monitor_node.py` (new): the node `health_monitor`. Publishes `/argus/health` (JSON, once a second) and `mavros_msgs/StatusText` on `/mavros/statustext/send`. Subscribes, raw, to the topics its `rates` parameter names, and to the status topics its `frame_counts` parameter names.
- Started only when asked: `bringup.FLAG_STARTED_NODES`, the delivery demo launch's `health_monitor:=true`, the entry point in `drone_utils/setup.py`. Its settings are in `sim_target_mission.yaml`.
- Sim: a scenario may now give a node other values for parameters the shipped file holds (`node_params`), and the scenario `delivery/health_monitor`.

## Flow

- Before: the camera slows to 12 frames a second → nothing says so.
- After (flag on): the camera's messages are counted → under four fifths of its rate for five seconds → `ARGUS H:RATE camera 12/30Hz` on the ground station, again every thirty seconds while it lasts, `ARGUS H:OK` when the last finding has gone.

## Code

- What is a finding, and for how long: `src/drone_utils/drone_utils/health.py:237`, `:267`.
- One line at a time to the ground station: `src/drone_utils/drone_utils/health.py:298`.
- Reading the computer: `src/drone_utils/drone_utils/health.py:109`, `:123`, `:151`.
- The node's second: `src/drone_utils/drone_utils/health_monitor_node.py:116`.

## Evidence

```
$ pytest src/drone_utils/test/test_health.py src/drone_utils/test/test_health_monitor_node.py   (as scripts/check.sh runs them)
5 passed, 8 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh delivery/health_monitor --variants on
   OK          delivery/health_monitor seed 1 on: 16.21 points, 0.03 m from the target
Health: ARGUS H:ON 3 rate(s) watched
Health: ARGUS H:SILENT ghost
$ docker run --rm -v "$PWD:$PWD" -w "$PWD" argus-sim:humble python3 sim/statustext_from_tlog.py sim/.state/logs/latest/mavlink.tlog --prefix "ARGUS H"
14 status text(s) starting with 'ARGUS H' in the telemetry log
(1, 191, 5, 'ARGUS H:ON 3 rate(s) watched')
(1, 191, 4, 'ARGUS H:SILENT ghost')
```

`docs/results/2026-11-24-health-monitor.md` has the flight. The tests were written with the code. The flights also said three true things nobody planted: the laptop at 92 °C, the pose at 4 a second in a 2026 start-up, the camera twice under four fifths of its rate. `sim/statustext_from_tlog.py` (new) is the reader of the telemetry log.

## Spec check

spec-auditor, two rounds: INCOMPLETE, as a PARTIAL row is. Rates, temperatures, CPU load, the codes and their reaching the link DONE at the sim's level (round one asked for the telemetry-log reader to be in the commit, and for the throttle reading to be shown in a run: both are). Still PARTIAL and recorded: the GPU's load and the Jetson's files (never read there), dropped frames in a flight, a quiet flight and an `ARGUS H:OK` in a flight. flight-guardian, two rounds: CONCERNS both times, no blocker. Closed: a standing warning from the shipped list (the pose is out of it), a monitor that cannot start looking like a healthy one (`ARGUS H:ON`; declarations inside the try), the shipped params file in ROS's own parser. Left: the cost of its camera subscription is not measured, and a monitor that dies after `ON` is silent like a healthy one.

## Not done

- On the Jetson (why PARTIAL): its thermal zones, its throttling and its GPU's load file have not been read by this code; which topics and rates to watch on the aircraft is not set in any flight params file (only the sim's).
- Nobody has looked at QGroundControl showing a line: the flight's telemetry log holds them.
- A scenario cannot give the monitor `frame_counts` (the shipped block does not hold the key, and the builder only changes keys it holds): the owed survey with `loc:/loc/status` needs that key in the survey's params file first.
- Dropped frames are counted from a node's own status (`/loc/status`); the grabbers and the detector do not say theirs, so theirs are not watched. No survey was flown with the monitor: dropped frames are unit tests only.
- What its subscription costs the camera's stream is not measured; a quiet flight (nothing wrong, nothing said after `ARGUS H:ON`) and an `ARGUS H:OK` in a flight are not flown.
- Throttling is "the allowed top frequency is under what the CPU can do": a Jetson power mode that caps the frequency would read as throttled for a whole flight. Not checked on a Jetson.
- The limits (85 °C, 95%, 98%, a tenth of the frames, four fifths of a rate) are the session's.
