# The survey mission and its height guard in the sim (LOC-3, LOC-1)

2026-11-20, on `8fb0023` plus the LOC-3 change (uncommitted when flown; it is the commit this file comes with).
Sim: ArduPilot SITL 4.6.3 and Gazebo, site `apollo_xi`, the survey camera (1280 x 720 at 10 Hz,
`dds_profile: large_frames`), calm air, sim time, seed 1. Flags on: `use_frame_time_pose`, `use_height_estimator`,
`survey_search`. One flight of each scenario; the plain survey also with every flag off.

After these flights two settings got bounds and a refused speed restore got a log line (`ground_below_home_m`,
`speed_after_mps`); no scenario sets either, and they were not flown again.

## Commands

```bash
sim/run_scenarios.sh waypoint_navigation/survey --seeds 1 --variants off,on
sim/run_scenarios.sh waypoint_navigation/survey_dip --seeds 1 --variants on
sim/run_scenarios.sh waypoint_navigation/survey_abort --seeds 1 --variants on
sim/run_scenarios.sh waypoint_navigation/survey_budget --seeds 1 --variants on
```

The runner's row for each (flags off, then flags on):

```
| waypoint_navigation/survey | 1 | OK | 0.00 | 0 of 6 waypoints | OK | 6.00 | 6 of 6 waypoints |
| waypoint_navigation/survey_dip | 1 | not run |  |  | OK | 6.00 | 6 of 6 waypoints |
| waypoint_navigation/survey_abort | 1 | not run |  |  | OK | 2.00 | 2 of 2 waypoints |
| waypoint_navigation/survey_budget | 1 | not run |  |  | OK | 2.00 | 2 of 2 waypoints |
```

A run is OK only if the mission's log has the lines its scenario lists under `expect_log` (the guard's own, the
budget's own): each report says `Lines expected in the mission's log | n of n found`. `survey_dip` was flown
twice; the first run's flight was the same but its time check could not read one node's `use_sim_time`
(INVALID, seen in 2 of about 170 runs this session), so it was flown again.

True heights and positions are the vehicle's poses in each run's `trace.jsonl` (`u` is the model's origin, 0.2 m
above the ground when it stands, so 0.2 m is taken off below), read with:

```bash
python3 - <run folder> <<'PY'
import json, sys
d = sys.argv[1]
poses = [p for p in map(json.loads, open(d + "/trace.jsonl")) if p.get("pose") == "vehicle" and "u" in p]
on = [e["t"] for e in map(json.loads, open(d + "/gps_events.jsonl")) if e.get("event") == "rangefinder" and e["what"] == "on"]
for dt in (-0.5, 1, 2, 3, 4, 6, 8, 10, 12, 20, 30):
    p = min(poses, key=lambda p: abs(p["t"] - (on[0] + dt)))
    print(dt, round(p["u"] - 0.2, 2), p["e"], p["n"], p["yaw"])
PY
grep "search_pattern\]" <run folder>/launch.log
```

## The survey (`survey`)

A 60 x 40 m search area inside the site's flight boundary. The mission's own lines:

```
[search_pattern] survey planned: 3 tracks 12.1 m apart at 10.67 m, 204 m and 51 s at 4 m/s, yaw 0 deg, a marker in about 25 frames
[search_pattern] flying the survey without the localisation pipeline: its service is not there
[search_pattern] survey finished: 3 of 3 tracks flown in 67 s, 0 climb(s) back to the survey height
```

| | |
| --- | --- |
| Track ends passed in order, within 2.5 m | 6 of 6; closest approach 0.00 to 0.29 m |
| True height over the area (1,200 poses) | median 10.67 m, lowest 10.66 m (the survey height is 10.668 m) |
| Speed along the second track, 70 m to 30 m east | 3.99 m/s (4 m/s asked for with DO_CHANGE_SPEED) |
| Heading over the area | 0.0 rad from east on every pose (the tracks run east and west; the image's long side is across them) |
| Height estimator beside the flight | PASS: rangefinder heights p95 0.02 m from the truth |
| Time | 67 s from the mission's start (10 s to the first track, 60 s from its start to the last track's end), against the plan's 51 s for the tracks alone |
| Flags off | the mission says `skipped: the survey_search flag is off`; the vehicle takes off and returns; 0 of 6 track ends |

The localisation pipeline's service does not exist yet (LOC-5), so the survey was flown without it.

## The guard

The sim's rangefinder is SITL's own, over flat ground: nothing in the Gazebo world changes its reading. The dip is
therefore made in the sensor: 40 s after arming, for 8 s, the rangefinder reads a fraction of the true range
(`gps_events: [{kind: rangefinder, scale}]`, SITL's `SIM_SONAR_SCALE`), as it would over ground that had risen. In
both flights the fault began as the vehicle left the first track for the second.

### A dip under 8.22 m: wait, climb back, go on (`survey_dip`, scale 0.75)

```
[search_pattern] survey waiting: 8.00 m on the rangefinder is under 8.22 m: climbing back to 10.67 m
[search_pattern] survey waiting: 9.15 m on the rangefinder: climbing back to 10.67 m
[search_pattern] back at the survey height: going on with track 2 of 3
[search_pattern] survey finished: 3 of 3 tracks flown in 72 s, 1 climb(s) back to the survey height
```

| Seconds after the fault began | True height, m | East, north, m |
| --- | --- | --- |
| -0.5 | 10.66 | 80.1, 19.4 |
| 1 | 10.81 | 80.2, 24.8 |
| 2 | 11.83 | 80.2, 26.3 |
| 3 | 13.05 | 80.2, 25.5 |
| 4 | 13.83 | 80.2, 23.3 |
| 6 | 14.08 | 80.1, 24.1 |
| 8 (the fault ends) | 14.19 | 80.0, 29.4 |
| 10 | 14.18 | 75.1, 30.1 |
| 12 | 14.14 | 67.2, 30.2 |
| 20 | 12.05 | 35.3, 30.1 |
| 30 | 10.66 | 23.2, 42.2 |

The vehicle stopped (it ran on about 4 m while braking, and came back), climbed 3.2 m in 3 s to where the
rangefinder read the survey height again (14.2 m true is 10.65 m at 0.75), held that through the rest of the
fault, and flew all three tracks; the survey took 5 s longer. After the fault the height came down along the
second track, not at once: ArduPilot flies a straight line to a position target, so a height correction on a 60 m
track is spread along it. The height estimator beside the flight passed (p95 0.05 m), with the 270 heights of the
fault's time counted and not judged.

### A dip under 7.77 m: the survey ends, the climb first (`survey_abort`, scale 0.68)

```
[search_pattern] survey ended early: 7.25 m on the rangefinder is under 7.77 m: the survey is over, climbing back to 10.67 m
[search_pattern] State: SURVEY -> ENDING
[search_pattern] State: ENDING -> COMPLETE
[rtl] Requesting mode RTL
```

| Seconds after the fault began | True height, m | East, north, m |
| --- | --- | --- |
| -0.5 | 10.66 | 80.1, 19.3 |
| 1 | 10.79 | 80.2, 24.7 |
| 2 | 11.87 | 80.2, 26.3 |
| 3 | 13.18 | 80.2, 25.5 |
| 4 | 13.98 | 80.2, 23.2 |
| 6 | 14.92 | 80.1, 22.1 |
| 10 | 14.98 | 67.7, 18.4 |
| 20 | 14.98 | 0.1, 0.0 |

The mission climbed the 3.4 m it was short by (to 14.0 m true) where it was, then ended with SUCCESS; the sequence
went on to RTL (which climbs to its own 15 m) and landed. Tracks two and three were not flown.

## The time budget (`survey_budget`, 35 s)

```
[search_pattern] survey planned: 3 tracks 12.1 m apart at 10.67 m, 204 m and 51 s at 4 m/s, yaw 0 deg, a marker in about 25 frames; the tracks alone take 51 s, more than the 35 s budget
[search_pattern] survey ended early: the time budget of 35 s ran out with 1 of 3 tracks flown
[search_pattern] State: SURVEY -> ENDING
[search_pattern] State: ENDING -> COMPLETE
```

The survey began 10 s after the mission did; the budget ended it 19 m into the second track (61.2 m east, 30.2 m
north) with SUCCESS, and the sequence returned and landed. True height over the area: median 10.67 m, lowest
10.65 m.

## What the dip flights found

Three faults, each seen in a `survey_dip` flight and fixed before the flights above (Decisions, 2026-11-20):

1. **The recovery ended after 0.76 s** (first flight). The stop tilted the vehicle past the rangefinder's
   15 degrees; the height estimator, as built, gave the barometer's height for that moment; and the barometer,
   which had not seen the ground come close, was at the survey height.

   ```
   [1795193482.549516493] [search_pattern] survey waiting: 8.05 m on the rangefinder is under 8.22 m: climbing back to 10.67 m
   [1795193483.312211460] [search_pattern] back at the survey height: going on with track 2 of 3
   ```

   The survey went on with the rangefinder reading about 8.2 to 9.6 m for the rest of the fault. The guard now
   keeps a recovery that began on the rangefinder until the rangefinder says the height is back, or has been away
   for 3 s.
2. **The height sagged after the recovery** (second and third flights: 13.87 m true at 4 s, 12.08 m and 12.71 m at
   8 s, the fault still on). Two causes: the first setpoint after a recovery came from the height before it (the
   flight-safety review found this one), and a moment on the barometer as the vehicle set off again pulled the
   setpoint toward the survey height above home. The track now goes on from the height the recovery reached, and
   the rangefinder's ground stands against the barometer's for 3 s. In the flight above the height is 14.08 to
   14.19 m from 6 s to 10 s.
3. **The projection check made the first survey flight INVALID**: it runs beside every flight with
   `use_frame_time_pose` on and compares projection v2 with the target's true place, and a survey has no target.
   It now runs only for a scenario that has one.

## Not shown

- A dip in the world (terrain, an obstacle): the sim's rangefinder cannot see one. It needs the Gazebo lidar
  (SIM-05's P1 part).
- The true height against the rules' 7.62 m as a verdict: these scenarios are judged on their track ends. The
  localisation judge takes the lowest true height over the survey area; gate G4 is where it is used.
- The localisation pipeline started and stopped in flight: its service is LOC-5's.
- Wind, other seeds, a concave boundary, a refused speed command, a height that comes and goes, a turned camera
  mount: mission tests only (`src/drone_mission_demo/test/test_search_pattern_mission.py`).
