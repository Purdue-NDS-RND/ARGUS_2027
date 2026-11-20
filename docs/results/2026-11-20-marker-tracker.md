# The marker tracker in the sim (LOC-7)

2026-11-20, on `538d733` plus the LOC-7 change (uncommitted when flown; it is the commit this file comes with).
Sim: ArduPilot SITL 4.6.3 and Gazebo, site `apollo_xi`, the survey camera (1280 x 720 at 10 Hz), calm air, seed 1.

## Command

```bash
sim/run_scenarios.sh waypoint_navigation/survey_pipeline --seeds 1 --variants off,on
```

```
| waypoint_navigation/survey_pipeline | 1 | OK | 0.00 | 0 of 6 waypoints | OK | 6.00 | 6 of 6 waypoints |
```

The survey of a 60 x 40 m area with the localisation node beside it. On the field: three numbered markers inside
the four corner markers, and a fourth (`marker_9`) 3.5 m beyond the east corners. Flags on: `use_frame_time_pose`,
`use_height_estimator`, `survey_search` (mission); `loc_node`, `sim_oracle_detector`, `marker_tracker` (node). The
detector is the sim's oracle (true markers through the camera model, 2 px of noise, 5% misses, a false alarm
every fifty frames): no trained detector was run. With every flag off the node is not started and the survey
mission is skipped.

## The tracks against the truth

`sim/loc_check.py` beside the flight takes the tracker's last message on `/loc/tracks` and the scenario's true
markers (the run's `loc_check.json`, key `tracks`, and the report's row):

| Marker | True place, east and north of take-off | Its listed track | Distance |
| --- | --- | --- | --- |
| `marker_3` | 35.0, 20.0 | one | 0.037 m |
| `marker_7` | 62.0, 31.0 | one | 0.029 m |
| `marker_1` | 48.0, 44.0 | one | 0.032 m |
| `marker_9` (outside the corners) | 83.5, 30.0 | a confirmed track, marked outside, not listed | |

| | This flight | Line |
| --- | --- | --- |
| Listed tracks (confirmed, not outside) | 3 | one within 1 m of each marker inside the corners, none without a marker |
| Furthest listed track from its marker | 0.037 m | 1 m |
| Markers outside the corners, tracked and marked outside | 1 of 1 | each |
| Frames processed, dropped, faulted | 572, 0, 0 | |
| Rate, median and slowest five seconds | 10.0 and 9.83 a second | at least 8 and 6 |
| Boxes published and put on the ground | 212 (114 `marker`, 98 `boundary_marker`), all 212 placed | at least 90% placed |

```
| Localisation pipeline | PASS: 10.0 frames a second at the median (slowest 9.8) over 572 frames, 0 dropped, detector 'oracle'; frames 32 ms old at their turn, at the 95th percentile; boxes: 98 boundary_marker, 114 marker; 212 of 212 put on the ground; tracks: 3 of 3 markers inside the boundary have their one listed track, the furthest 0.04 m from its marker; 3 listed, 1 marked outside |
```

The flight before the review's changes (no cut-box rule, no position-step handling) gave the same list: three
listed tracks, the furthest 0.052 m from its marker, one marked outside, over 571 frames and 213 boxes.

## How to read the 0.04 m

It is the tracker's position against the marker's true place in the vehicle's local frame, in a sim whose
position estimate is within a few centimetres of the truth and whose detector is an oracle that finds its pixels
with the camera model the pipeline places them with. It shows that the tracker makes one track a marker and
puts it where the sightings are; it does not show the error budget of the aircraft, where the GNSS error (about
1 to 1.5 m) dominates (LOC-9's spec).

## Not shown

- A box cut by the edge of the picture (the oracle never clips one), a false alarm that repeats (the oracle's do
  not), a step in the flight controller's position, a clock step: tests only
  (`src/argus_localisation/test/test_marker_tracker.py`, `test_loc_node.py`).
- The corner markers are 1.2 m wide and lie 7.92 m from the nearest track, 0.25 m inside the swath's edge. With
  cut boxes no longer counted, 0.2 m of sideways error or a degree of roll takes a corner's sightings away, and
  without four corners nothing is marked outside. In this flight all four were confirmed.
- Wind, other seeds, a second pass over a marker after our position has drifted (the tracker's process noise and
  its 1 m rule are tested, not flown).
