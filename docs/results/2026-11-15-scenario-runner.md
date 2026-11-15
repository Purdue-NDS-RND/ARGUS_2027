# Headless scenario runner: first unattended runs (2026-11-15)

The first runs of `sim/run_scenarios.sh` (SIM-07): the 2026 delivery mission flown, recorded and scored with nobody at the keyboard, and the geofence check as a scored scenario. Every number is from Gazebo's true positions.

| Run | Scenario | Status | Result | Points | Flight, sim time | Real-time factor while armed |
| --- | --- | --- | --- | --- | --- | --- |
| Smoke 1 of 3 | `delivery/baseline`, flags off | OK | 0.02 m from the target | 16.22 | 151 s | 0.97 |
| Smoke 2 of 3 | `delivery/baseline`, flags off | OK | 0.03 m from the target | 16.22 | 150 s | 0.97 |
| Smoke 3 of 3 | `delivery/baseline`, flags off | OK | 0.02 m from the target | 16.22 | 149 s | 0.97 |
| Geofence | `geofence/breach` | OK | response 0.02 s before the true crossing, 1.07 m outside | 1.00 | 135 s | 1.00 |

**The SIM-07 pass line is met:** the smoke suite ran three times in a row, started by one script with no input, each ending `OK` with exit code 0 (19:06:50 to 19:17:28 UTC, about 3.5 minutes per suite).

**The SIM-02 pass line, scored from true positions:** flying east through the fence at 5 m/s, the autopilot reported the breach and switched to RTL 0.02 s before the true position crossed the line, and the vehicle went 1.07 m outside (limits: 2 s and 10 m). `sim/fence_check.py`, reading the autopilot's own position in the same flight, measured 1.07 m too.

Delivery points are 13 − distance, plus the 25% autonomy bonus: no ground-station command or RC override was found in the MAVLink log during any flight.

## Setup

- **Machine:** club drive (Lenovo 82BG laptop), headless, lid lock held by the script.
- **Sim:** `argus-sim:humble` on `ardupilot-gz:4.6.3`, a fresh container per flight, SITL storage wiped, site `apollo_xi` (placeholder coordinates) with its geofence on.
- **Mission:** `delivery/baseline`: the shipped `package_delivery_demo.yaml` sequence and `sim_target_mission.yaml` params, flags off, `fake_drop: true`, bullseye 10 m east and 15 m north of the takeoff point. The only generated change is the target's latitude and longitude.
- **Code:** branch `platform/build-2027`, the SIM-07 commit.

## Commands and output

```
$ for i in 1 2 3; do sim/run_scenarios.sh smoke; echo "exit code $?"; done
== Suite 'smoke': 1 flight(s); results in sim/.state/suites/smoke-20261115T190650Z

== Flight 1 of 1: delivery-baseline-s1-off
   waiting for the sim to be READY (up to 900 s)...
   READY after 39 s; flying...
   flight: DONE
   OK          delivery/baseline seed 1 off: 16.22 points, 0.02 m from the target

# Suite smoke, 2026-11-15

| Scenario | Seed | 2026 (flags off) | Points | Result | 2027 (flags on) | Points | Result |
| --- | --- | --- | --- | --- | --- | --- | --- |
| delivery/baseline | 1 | OK | 16.22 | 0.02 m from the target | not run |  |  |

1 runs: 1 OK

Summary: sim/.state/suites/smoke-20261115T190650Z/summary.md
exit code 0
...
   READY after 43 s; flying...
   flight: DONE
   OK          delivery/baseline seed 1 off: 16.22 points, 0.03 m from the target
exit code 0
...
   OK          delivery/baseline seed 1 off: 16.22 points, 0.02 m from the target
exit code 0
```

One run's `report.md`:

```
# delivery/baseline, seed 1, flags off

**OK**

| Points | 16.22 (12.98 before the autonomy bonus) |
| Result | 0.02 m from the target |
| Flight time, arm to disarm (sim time) | 149 s |
| Real-time factor while armed | 0.97 |
| Fence breaches (true position) | 0 |
| Failsafes | 0 |
| Human input during the flight | 0 |
```

The geofence scenario:

```
$ sim/run_scenarios.sh geofence/breach
   READY after 40 s; flying...
   flight: DONE
   OK          geofence/breach seed 1 off: 1.00 points, response -0.02 s from the line, 1.07 m outside
```

Its `score.json` details: `t_cross 102.944` (true position leaves the polygon), `t_reported 102.921` ("Polygon fence breached"), `t_mode 102.92` (RTL), `response_delay_s -0.024`, `max_excursion_m 1.075`. The trace has 19 true-pose samples a second for the vehicle.

## Can the audit see a command at all?

Every flight's audit found no human input, so it needs a positive control. In the geofence flight `fence_check.py` commands the vehicle through MAVProxy's tools port, the same kind of port a ground station connects to, as system id 245. Reading that flight's MAVLink log with 245 counted as a ground station:

```
ground station (system id 255) -> commands in the log: 0 ; inside the flight (sim 29.6 to 164.6 s): 0 {}
positive control: our tool on the tools port (system id 245) -> commands in the log: 701 ; inside the flight (sim 29.6 to 164.6 s): 689 {'COMMAND_LONG 22': 1, 'SET_POSITION_TARGET_LOCAL_NED': 688}
   first three inside the flight: [(30.0, 'COMMAND_LONG 22'), (40.2, 'SET_POSITION_TARGET_LOCAL_NED'), (50.6, 'SET_POSITION_TARGET_LOCAL_NED')]
```

So a command sent through a MAVProxy output port is in the log with its sender's id, and the clock marks place it inside the flight. This was a one-off script over `sim/.state/logs/20261115T190343Z/mavlink.tlog` using `runner.gcs_audit.read_tlog`, `runner.events.human_inputs(..., ground_stations={245})` and `runner.trace.sim_time_at`. A real ground station (QGroundControl, id 255) sending a command during a flight has not been tried.

## Earlier runs the same day, and what they changed

| Run | Result | What it showed |
| --- | --- | --- |
| First smoke | OK, 0.12 m, 16.10 points | The whole chain worked on the first try |
| Three smokes in a row | OK: 0.04, 0.03, 0.03 m | Two bugs in the logs, neither changing a score: the subscription to the autopilot's text messages had the wrong QoS and received nothing, and the flight script aborted on exit in one run of three (a Gazebo callback during interpreter teardown). Both fixed; the runs above are with the fixes |
| First geofence run | TIMEOUT, scored 1.12 m outside | The flight had ended, but the runner did not see "on the ground": the landed state is only streamed when the 2026 takeoff service asks for it. Now the true height also counts |

After the audit's fixes (tools and list handling moved out of the flight script), one more smoke flight: OK, 0.09 m from the target, 16.14 points.

Touchdown distance over the eight delivery flights of the day: 0.02 to 0.12 m (the 2026-11-04 baseline, at the old world, measured 0.03 to 0.11 m by hand).

## Not measured

- Flags on: no 2027 flag does anything yet, so every run is flags off.
- Wind and GPS events: a scenario that asks for them is reported INVALID until SIM-03 and SIM-04 exist.
- `delivery/clutter`, `drop/baseline` and `waypoint_navigation/site_circuit` have not been flown through the runner yet.
- A sim that fails to get READY, a suite of more than one flight, the tools beside a flight and the judges'-list copy have only run against a stand-in sim in the unit tests.
- A run slow enough to be INVALID (real-time factor below 0.8) did not happen; that rule is covered by unit tests only.
