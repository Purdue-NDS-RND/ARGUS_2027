# PLT-13: GPS missions

Flags: `waypoint_timeout_hold`, `time_trial_auto`, `route_optimiser`, `flying_start` (mission flags of the GPS waypoint mission, all false) · Status: PARTIAL · Commits: `git log --oneline --grep '^PLT-13:' platform/build-2027`

## What it solves

The 2026 GPS mission waits for ever at a waypoint it cannot reach, and flies the time trial one GUIDED setpoint at a time, stopping its turn at each 4 m radius. These add a timeout and a hold per waypoint, the time trial as an AUTO mission the flight controller flies itself, and (for when the organisers allow them) the shortest order and a flying start.

## Architecture

- `drone_mission_core/gps_route.py` (new, no ROS): `WaypointClock`, `optimise_order`, `run_in_point`, `auto_items`, and the executor's thin side `attach_mission_upload` (MAVROS `/mavros/mission/push`, `/mavros/mission/reached`).
- `drone_mission_demo/missions/gps_route_2027.py` (new): `Route2027`, the route flown with any of the flags on.
- `gps_waypoint_mission.py`: a switch to `Route2027` when a flag is on. Executor and context: the upload, behind `time_trial_auto`.
- The two live sequences declare the flags false and carry the new settings. Sim: `waypoint_navigation/circuit_hold[_wind]`, `waypoint_navigation/circuit_timeout`, `time_trial/circuit_auto[_wind]`, `time_trial/circuit_flying`.

## Flow

The time trial, before: GUIDED setpoint at waypoint 1 → within 4 m → setpoint at waypoint 2 → ... (71.3 s). After, `time_trial_auto` on: the route uploaded as mission items → AUTO → the flight controller flies through each waypoint's 2 m → the mission counts them from `/mavros/mission/reached` → GUIDED again (63.0 s). Any step not taken: the rest in GUIDED.

## Code

- The rules: `src/drone_mission_core/drone_mission_core/gps_route.py` (`WaypointClock.update`, `optimise_order`, `auto_items`).
- The route: `src/drone_mission_demo/drone_mission_demo/missions/gps_route_2027.py` (`_guided`, `_auto_upload`, `_auto_fly`, `_fall_back`).
- The switch: `src/drone_mission_demo/drone_mission_demo/missions/gps_waypoint_mission.py` (`on_enter`, `update`).

## Evidence

```
$ pytest src/drone_mission_core/test/test_gps_route.py              -> 6 passed
$ pytest src/drone_mission_demo/test/test_gps_route_2027.py         -> 21 passed
$ sim/run_scenarios.sh waypoint_navigation/circuit_hold --seeds 1 --variants on   -> OK, 7 of 7 within 0.02 m
$ sim/run_scenarios.sh waypoint_navigation/circuit_timeout --seeds 1 --variants on -> OK, 6 of 7: wp2 held 6 s, wp4 given up at 3 s
$ sim/run_scenarios.sh time_trial/circuit_auto --seeds 1 --variants on            -> OK, 7 of 7, 62.96 s (AUTO)
$ sim/run_scenarios.sh time_trial/circuit_auto --seeds 1 --variants off           -> OK, 7 of 7, 71.34 s (2026, GUIDED)
$ sim/run_scenarios.sh time_trial/circuit_flying --seeds 1 --variants on          -> OK, 7 of 7, 63.51 s (flown before the reviews' fixes; not flown again)
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

`docs/results/2026-11-23-gps-missions.md`.

## Spec check

- flight-guardian, round 1: BLOCKER, two: with AUTO on, a mode taken by the pilot or a failsafe was answered with a GUIDED request; and the optimiser tried every order inside one tick (seconds at nine waypoints). Both fixed with tests, with its concerns: an AUTO mission left flying when its mission ends, GUIDED flown without the mode confirmed, a run-in point or reordered legs outside the course, a route with nothing reached called a success, the stall time against long legs.
- spec-auditor, round 1: INCOMPLETE: the timeout and a by-name hold had not flown (now `waypoint_navigation/circuit_timeout`), the "mode leaves AUTO" branch untested (now tested and changed), and the MAVROS plugin left out against PLT-06's line (recorded, the lead's).
- Round 2, spec-auditor: the timeout and the by-name hold are DONE (flown); INCOMPLETE only on what has not flown (AUTO's fallbacks, the optimiser) and the plugin, which is the lead's.
- Round 2, flight-guardian: CONCERNS, both blockers resolved. Acted on after the round, not reviewed again: the optimiser orders at most 8 waypoints in flight (milliseconds) and flies the given order beyond that; a waypoint's clock does not run while the mode is somebody else's. Left: leaving the mission asks for GUIDED once and is not confirmed, and an executor that dies leaves an uploaded AUTO mission flying; the flight boundary is optional.

## Not done

- AUTO on the aircraft waits for the lead to allow MAVROS's waypoint plugin; until then the flag flies GUIDED.
- AUTO's fallbacks and the optimiser have not flown in the sim (unit-tested; the judge scores the waypoints in the order given). The flying start was flown before the reviews' fixes and not again.
