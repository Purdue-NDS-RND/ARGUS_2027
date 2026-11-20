# LOC-3: `search_pattern` mission

Flag: `survey_search` (default false; needs `use_height_estimator`) · Status: DONE · Commits: `git log --oneline --grep '^LOC-3:' platform/build-2027`

## What it solves

Nothing flew the localisation survey: LOC-1's height guard and LOC-2's planner were modules with no caller. This is the mission that flies the planner's tracks over the search area, at the survey height, with the guard watching, inside a time budget. It is the missing piece, not a workaround.

## Architecture

- `drone_mission_demo/missions/search_pattern_mission.py` (new): mission type `search_pattern`. With the flag off it is skipped (SUCCESS on its first tick, nothing commanded, none of its modules loaded).
- `drone_mission_core/survey_search.py` (new): the flag, and the executor's client for the localisation pipeline's service `/loc/set_enabled` (std_srvs/SetBool), made only with the flag on. LOC-5's node is to serve it.
- `MissionContext`: `command_change_speed` (MAV_CMD_DO_CHANGE_SPEED through the existing command client), `localisation_control_ready`, `set_localisation_enabled`.
- `survey_height.py` (LOC-1's guard) changed by what the sim flights found: see "Spec check".
- New sequence `search_pattern_demo.yaml` (take-off, survey, RTL; every flag false).
- Sim: scenario keys `survey` (the search area in metres) and `expect_log` (lines the mission's log must have); fault kind `rangefinder` in `sim/inject.py`; scenarios `waypoint_navigation/survey`, `survey_dip`, `survey_abort`, `survey_budget`.

## Flow

A sequence with `search_pattern` after take-off, flag on:

- Before: no such mission; a sequence naming it did not build.
- After: PLAN (corners to the local frame, `coverage_planner.plan`, inside the flight boundary) → CLIMB (first track's start, survey height above the ground) → SET_SPEED (DO_CHANGE_SPEED 4 m/s) → START_PIPELINE (`/loc/set_enabled` true, or flown without it) → SURVEY (the tracks at one yaw; the guard asked every tick: RECOVER holds and climbs, ABORT ends) → ENDING (the climb if one is owed, the pipeline stopped) → SUCCESS.

## Code

- The plan, and what fails before the vehicle moves (a corner over 1 km away, a leg or the way to the first track outside the boundary): `search_pattern_mission.py:302`.
- One tick of the survey: the guard's answer, the hold and climb, the ground followed between its lines: `search_pattern_mission.py:458`; the track going on from the recovered height `:487`; the rangefinder's ground kept against the barometer for 3 s `:497`.
- The end: the owed climb first, then the pipeline's stop, then the status: `search_pattern_mission.py:520`; leaving in mid-survey `:208`.
- The guard's two new rules: a recovery begun on the rangefinder is not ended by the barometer (`survey_height.py:182`); a gap in the height (`survey_height.py:141`).
- The executor's side: `survey_search.py:26`, `mission_context.py:388` and `:411`.

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider src/drone_mission_demo/test/test_search_pattern_mission.py
43 passed in 0.63s
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_survey_search.py src/drone_mission_core/test/test_survey_height.py
28 passed in 0.31s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh waypoint_navigation/survey --seeds 1 --variants off,on
| waypoint_navigation/survey | 1 | OK | 0.00 | 0 of 6 waypoints | OK | 6.00 | 6 of 6 waypoints |
$ sim/run_scenarios.sh waypoint_navigation/survey_dip --seeds 1 --variants on
| waypoint_navigation/survey_dip | 1 | not run |  |  | OK | 6.00 | 6 of 6 waypoints |
$ sim/run_scenarios.sh waypoint_navigation/survey_abort --seeds 1 --variants on
| waypoint_navigation/survey_abort | 1 | not run |  |  | OK | 2.00 | 2 of 2 waypoints |
$ sim/run_scenarios.sh waypoint_navigation/survey_budget --seeds 1 --variants on
| waypoint_navigation/survey_budget | 1 | not run |  |  | OK | 2.00 | 2 of 2 waypoints |
```

`docs/results/2026-11-20-survey-height-guard.md`: three tracks flown at 10.67 m true (lowest 10.66 m), 3.99 m/s, heading held to 0.0 rad, 67 s; a dip to 8.0 m held and climbed out of in 3 s with the survey finished 5 s late; a dip to 7.25 m ended the survey after its climb; a 35 s budget ended it in the second track.

## Spec check

- spec-auditor, round 1: INCOMPLETE (six rows PARTIAL, one DEVIATION, two MISSING): the real service path and the yaw for any but east-west tracks were untested; the budget did not end a recovery; "a height that comes and goes" was undecided. Round 2: every row DONE but one test's place in `scripts/check.sh` (moved after the round) and re-planning from boundary markers, which LOC-3's spec does not ask for and is the lead's ("Needs a person").
- flight-guardian, round 1: eight concerns, no blocker. Round 2: five resolved (the stale setpoint after a recovery, the transit and far corners, the planner loaded with flags off, the plan inside one tick, the budget in a recovery); three recorded with a setting built for each and a person row (height above home on the barometer: `ground_below_home_m`; the optional boundary; the survey's speed left set: `speed_after_mps`); two new concerns about those settings' bounds, bounded after the round and not re-reviewed.
- Changed in LOC-1's guard because of the flights (Decisions): a recovery begun on the rangefinder waits up to 3 s for the rangefinder before the barometer may end it; a gap in the height under 0.5 s is flown through, a longer one waits for a height that stays 1 s, and 5 s of coming and going ends the survey.
- Beyond the spec, each under Decisions: the flag; the service's name and what its absence does; the three speed attempts; ground following; the 1 km check; the sensor-made dip.

## Not done

- The pipeline started and stopped in flight: `/loc/set_enabled` is LOC-5's to serve ("Needs a person": the lead adds it to LOC-5's spec).
- Re-planning from the four boundary markers (lead).
- Before the flag is turned on for a competition sequence: `search_area`, `flight_boundary`, `ground_below_home_m`, `speed_after_mps` ("Needs a person").
- A dip the rangefinder sees in the world needs the Gazebo lidar (SIM-05, P1). The 7.62 m floor is a verdict only from gate G4's localisation judge on.
- One flight per scenario, calm air.
