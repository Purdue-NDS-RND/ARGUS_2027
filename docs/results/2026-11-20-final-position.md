# The reported position of a marker: the weighted median in the sim (LOC-9)

2026-11-20, on `b4d99fb` plus the LOC-9 change (uncommitted when flown; it is the commit this file comes with).
Sim: ArduPilot SITL 4.6.3 and Gazebo, site `apollo_xi`, the survey camera (1280 x 720 at 10 Hz), calm air, seed 1.

## Command

```bash
sim/run_scenarios.sh waypoint_navigation/survey_pipeline --seeds 1 --variants off,on   # the first flight
sim/run_scenarios.sh waypoint_navigation/survey_pipeline --seeds 1 --variants on       # on the final code
```

```
| waypoint_navigation/survey_pipeline | 1 | OK | 0.00 | 0 of 6 waypoints | OK | 6.00 | 6 of 6 waypoints |
| waypoint_navigation/survey_pipeline | 1 | not run |  |  | OK | 6.00 | 6 of 6 waypoints |
```

The survey with the localisation node, the oracle detector and the marker tracker
(`docs/results/2026-11-20-marker-tracker.md`), now with `loc_median_position` on among the node's flags: the
position each track carries on `/loc/tracks` is the weighted median of its sightings. That the flag was on in the
flight is the node's own word, which the check holds the flight to: the run's `params.yaml` has
`loc_median_position: true`, `/loc/status` said `track_positions: median`, and the report's row ends "positions
reported: the median".

## The reported positions against the truth

From the run's `loc_check.json` (key `tracks`): the distance of each listed track's reported position from its
marker's true place.

| Marker | Filter's position reported (LOC-7's flight) | Median reported, first flight | Median reported, final code |
| --- | --- | --- | --- |
| `marker_3` | 0.037 m | 0.054 m | 0.020 m |
| `marker_7` | 0.029 m | 0.021 m | 0.022 m |
| `marker_1` | 0.032 m | 0.030 m | 0.020 m |
| Listed tracks, and the marker outside the corners | 3, marked outside | 3, marked outside | 3, marked outside |

```
localisation pipeline: tracks: 3 of 3 markers inside the boundary have their one listed track, the furthest 0.02 m from its marker; 3 listed, 1 marked outside; positions reported: the median
```

These are three different flights, each with its own draw of the oracle's 2 px noise: the differences between the
columns are that noise, not the median against the filter. In a calm sim with no bad projection the two are
centimetres apart either way, and the check's 1 m line cannot tell them apart. What the flight shows is that the
median is what goes out, end to end, and that it is on the marker.

## What shows the median does its job

Tests, not this flight (`src/argus_localisation/test/test_final_position.py`):

- 24 sightings with 5 cm of noise and 3 projections 0.9 m east and 0.7 m south of the marker: the mean is more
  than 0.08 m off, the weighted median under 0.04 m.
- Through the tracker, 22 sightings and then 3 that are 0.6 m off and inside the gates: the filter's position is
  pulled more than 0.03 m, the reported median stays within 0.01 m.
- Two sure sightings outvote three unsure ones: the weight is one over the trace of a sighting's noise.

## The error budget

The spec's budget is about the aircraft and cannot be shown here.

| Term | The spec | This sim |
| --- | --- | --- |
| Projection error, a sighting | 0.2 to 0.3 m from 30 ft | from 35 ft with a position estimate right to centimetres and an oracle for a detector: a track's reported position is 0.02 to 0.05 m from the truth |
| Absolute GNSS error | 1 to 1.5 m with the u-blox M10; it dominates | none: the sim's position is the truth to centimetres |

The GNSS error is common to every sighting of a marker, so the median does not reduce it; the median and the
filter work on the projection term only. The check's 1 m line would not fail if the projection error doubled.
The standing decision is the spec's: no RTK unless the organisers' tolerance is tighter than the GNSS error.

## Not shown

- A bad projection in flight, a position slip, a marker seen on two passes: tests.
- Wind, other seeds, a trained detector.
