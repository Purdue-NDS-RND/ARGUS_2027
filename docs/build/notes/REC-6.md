# REC-6: GPS health monitor and EKF source switching

Flag: `ekf_source_switching` (mission flag, default false; needs `package_recovery`) · Status: PARTIAL · Commits: `git log --oneline --grep '^REC-6:' platform/build-2027`

## What it solves

"GPS may not always be trustworthy" (the rules). The recovery flew on whatever the GPS said as long as there was a fix: a receiver that stepped 10 m took the vehicle with it. Now something watches the GPS against what else the vehicle knows; when it cannot be trusted the mission stops flying on it, and tells the flight controller to stop using it once there is something else for it to use.

## Architecture

- `drone_mission_core/gps_health.py` (new, no ROS): `GpsHealth` (the rating: TRUSTED, SUSPECT, UNTRUSTED, with reasons), `SourceSwitch` (which source set the flight controller is on by its acknowledgements, and when to ask for another), `GpsHealthFeed` and `attach_gps_health` (the executor's side: the raw fix, its velocity and quality from MAVROS, the flow odometry's samples; subscribed only with the flag on).
- `MissionContext`: `gps_health()`, `no_gps_source_fed()`, `note_static_target()`, `command_ekf_source_set()` (MAV_CMD_SET_EKF_SOURCE_SET).
- `recovery_flight.py`: `Readings.gps_trusted`: False is flown as "no GPS" (no global setpoint, the abort a landing, the way back in the local frame). `ekf_source_switching` is no longer a refused flag.
- `package_recovery_mission.py`: with the flag on, asks the monitor every tick, tells it where the standing bucket is, runs the switch, and hands the flight its word.
- `flow_odometry_node.py`: its status says whether the sample was sent to the flight controller (`sent`).
- Scenario `recovery/gps_step` (the spec's run R3, as far as it can be flown).

## Flow

- Before: a fix → trusted → global setpoints, RTL as the abort.
- After: at the start, set 1 is asked for and the mission holds until it is acknowledged (five tries at most; unanswered, the flight is flown without GPS). Every tick, the rating → on UNTRUSTED the flight flies as without GPS; `source set 2` is asked for only if flow is being sent to the flight controller (again each second until acknowledged; after five, the set is "not known" and nothing is flown on GPS) → set 1 again only when the rating is TRUSTED (ten clean seconds) and the mission is in TRANSIT, ACQUIRE, SEARCH or RETURN → a mission that ends off set 1 asks for it as it ends, or says why not.

## Code

- The jump and the drift, each measured on the GPS position less what was flown: `src/drone_mission_core/drone_mission_core/gps_health.py:226`, `:236`; the rating and its hold: `:262`.
- The source set by acknowledgements, fed or not, known or not: `:335`. The feed: `:437`.
- The mission's side: `src/drone_mission_demo/drone_mission_demo/missions/package_recovery_mission.py:219`, and its end `:175`. The flight rule taking the monitor's word: `src/drone_mission_core/drone_mission_core/recovery_flight.py:421`.

## Evidence

```
$ pytest test_gps_health.py test_package_recovery_mission.py test_recovery_flight.py   (as scripts/check.sh runs them)
20 passed, 23 passed, 28 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh recovery/gps_step --variants on
   OK          recovery/gps_step seed 1 on: 19.99 points, 0.01 m from the return point
66.532 s  source set 1 acknowledged by the flight controller     [asked at 66.528 s; the mission held until then]
94.301 s  GPS UNTRUSTED (the GPS position moved 3.3 m beyond what was flown in 1 s (by its own velocity))     [the step was set at 94.195 s]
94.302 s  GPS UNTRUSTED: source set 2 is not asked for, nothing is feeding it (...); it stays on set 1
```

`docs/results/2026-11-24-gps-health.md` has the three pass lines and all four flights, two of them on the rule before the review (set 2 asked and acknowledged in 0.02 s). The rule's tests were written with the rule; the height limit on the flow's word came from a flight, and the fed rule, the unknown set and the mission's end from the reviews.

## Spec check

spec-auditor, two rounds: INCOMPLETE, as a PARTIAL row is. Round one found two defects, both fixed (a fault arriving between the command for set 1 and its answer was lost; the flag was declared twice in the sequence file). Round two: the acknowledged set and the flag DONE; the switch itself a recorded DEVIATION (asked for only while fed); the estimator's flags, the drift in a flight, R3's eight of ten, the failsafe ordering and the thirty real minutes PARTIAL and recorded as owed. flight-guardian, two rounds: BLOCKER (a switch to a set nothing feeds; a mission ending off set 1), then CONCERNS after the rework, listed under "Not done".

## Not done

- The switch as the spec means it (why PARTIAL): source set 2 is asked for only when flow is being sent to the flight controller, and none is (REC-5: OPTICAL_FLOW is off until its units are checked). On the committed rule the sim never goes to set 2; that the command is taken was flown before the rule.
- The spec's "EKF glitch flags and variances": the rule takes them; nothing feeds them (MAVROS's estimator status is not among the aircraft's plugins).
- The drift line has unit tests only: the sim has no GPS drift event. R3 was flown once per rule, not ten times. The flight PLT-08 left to REC-6 (the switch before the estimator's failsafe) is not flown.
- The thirty clean minutes are made-up noise.
- A step that stays cannot be seen once it has stayed ten seconds: the mission then flies on GPS again.
- Only the recovery mission asks the monitor. The other missions fly on GPS as before.
- Left open by the flight-safety review's second round (concerns, recorded for the lead): "fed" is the companion saying it sends flow, not the flight controller saying it gets it; once on set 2, a flow that stops changes nothing; a mission that ends UNTRUSTED leaves the flight controller on set 2 with only a log line, and its one command for set 1 at the end is not repeated.
