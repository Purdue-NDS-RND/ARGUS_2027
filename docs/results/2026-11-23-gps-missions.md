# The GPS missions' 2027 behaviours in the sim (PLT-13)

2026-11-23 (UTC). The seven-point circuit at the 2027 site (`waypoint_navigation/site_circuit`'s points, about 600 m round), flown by the GPS waypoint mission with PLT-13's flags. Calm air, wall time (real-time factor 0.99 to 1.00). The AUTO time trial and the timeout flight tabled are the ones flown after the reviews' fixes.

## Commands

```
sim/run_scenarios.sh waypoint_navigation/circuit_hold --seeds 1 --variants on
sim/run_scenarios.sh time_trial/circuit_auto --seeds 1 --variants on
sim/run_scenarios.sh time_trial/circuit_auto --seeds 1 --variants off
sim/run_scenarios.sh time_trial/circuit_flying --seeds 1 --variants on
sim/run_scenarios.sh waypoint_navigation/circuit_timeout --seeds 1 --variants on
```

Numbers from each run's `result.json` and `score.json` (`sim/.state/suites/waypoint_navigation-circuit_hold-20261123T004323Z`, `time_trial-circuit_auto-20261123T010959Z` (flags on), `time_trial-circuit_auto-20261123T005126Z` (flags off), `time_trial-circuit_flying-20261123T005443Z`, `waypoint_navigation-circuit_timeout-20261123T012159Z`).

## Waypoint navigation

| | `circuit_hold`: 3 s hold, 90 s timeout at every waypoint | `circuit_timeout`: wp2 held 6 s by name, wp4 given 3 s |
| --- | --- | --- |
| Status | OK | OK (the three log lines the scenario expects were there) |
| Judge (1 m, three dimensions) | 7 of 7 waypoints, 7 points | 6 of 7, 6 points: wp4 was to be given up |
| Closest to each waypoint (m) | 0.01, 0.02, 0.01, 0.01, 0.02, 0.01, 0.00 | 0.01, 0.01, 0.01, 21.57, 0.01, 0.01, 0.00 |
| Route | flown in GUIDED, 7 of 7 reached, none given up | "To wp2: hold 6 s"; wp4 given up 3 s after it became the waypoint; on to wp5; 6 of 7 reached |

Both with the flag `waypoint_timeout_hold` on.

## Time trial

| | 2026, flags off (GUIDED) | `time_trial_auto` on | `time_trial_auto` and `flying_start` on |
| --- | --- | --- | --- |
| Status | OK | OK | OK |
| Judge (4 m, three dimensions) | 7 of 7 | 7 of 7 | 7 of 7 |
| Time from the first waypoint to the last (the judge's) | 71.34 s | 62.955 s (62.964 s in the flight before the fixes) | 63.51 s |
| Closest to each waypoint (m) | 0.30, 0.80, 0.83, 0.44, 0.72, 0.75, 0.18 | 0.94, 1.93, 1.97, 1.99, 1.97, 1.97, 0.15 | 0.53, 1.93, 1.98, 1.99, 1.97, 1.97, 0.16 |
| How it was flown | a GUIDED setpoint per waypoint, 4 m arrival radius | uploaded, AUTO taken, every item reached, back to GUIDED | the same, with a run-in point 30 m before the first waypoint as the first item |

## What it shows

- The uploaded AUTO mission is 8.4 s (12%) faster round this circuit than the 2026 GUIDED flight, and passes each waypoint within the flight controller's 2 m: inside the judge's 4 m.
- The flying start gained nothing here (0.55 s slower, within what one flight can tell): the first leg is 40 m from the takeoff point and the vehicle was already moving when it reached the first waypoint.
- With a 3 s hold the waypoint mission is within 2 cm of every waypoint in three dimensions.

## What the flights found

The first flight of `circuit_timeout` (`waypoint_navigation-circuit_timeout-20261123T010700Z`) ended at wp4's timeout with the mission raising "Logger severity cannot be changed between calls": the route's log helper chose info or warn at one line of code, which rclpy refuses, and the first warning of the flight was the giving up. The helper now logs from two lines, and a test runs every path of the route against a logger that enforces rclpy's rule. The flying-start flight was flown before the reviews' fixes and has not been flown again.

## Not shown

- AUTO's fallbacks (no upload service, upload refused or unanswered, AUTO not taken, AUTO stalling) in a flight: unit-tested.
- The route optimiser in a flight: the sim's judge scores the waypoints in the order given. On the 2026 course the optimiser finds 3-2-1-4-7-6-5, 934 m against 1278 m (a test holds it).
- Wind: gate G6 flies these in moderate wind.
- AUTO on the aircraft: its MAVROS does not load the waypoint plugin the upload needs, so with the flag on it would fly in GUIDED until the plugin is allowed.
