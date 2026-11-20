# The localisation pipeline in the sim, and the camera frames after the shared-memory fix (LOC-5, SIM-05)

2026-11-20, on `c8bbddd` plus the LOC-5 change (uncommitted when flown; it is the commit this file comes with).
Sim: ArduPilot SITL 4.6.3 and Gazebo, site `apollo_xi`, the survey camera (1280 x 720 at 10 Hz,
`dds_profile: large_frames`), calm air, seed 1.

## The pipeline beside the survey

```bash
sim/run_scenarios.sh waypoint_navigation/survey_pipeline --seeds 1 --variants off,on   # the flags-off flight, and one flags-on
sim/run_scenarios.sh waypoint_navigation/survey_pipeline --seeds 1 --variants on       # flags on again, on the final code
```

```
| waypoint_navigation/survey_pipeline | 1 | OK | 0.00 | 0 of 6 waypoints | OK | 6.00 | 6 of 6 waypoints |
| waypoint_navigation/survey_pipeline | 1 | not run |  |  | OK | 6.00 | 6 of 6 waypoints |
```

The survey of `waypoint_navigation/survey` over a field with three numbered markers and the four larger corner
markers. Flags on: `use_frame_time_pose`, `use_height_estimator`, `survey_search` (mission) and `loc_node`,
`sim_oracle_detector` (node), on sim time. The detector is the sim's oracle: it reports the true markers through
the camera model with 2 px of noise, 5% misses and a false alarm every fifty frames. No trained detector was run.

The two nodes' own lines, from the last flight's `launch.log`:

```
Sim oracle detector on (features.sim_oracle_detector): reporting 7 true object(s) from /sim/truth/poses, not looking at the picture
Localisation node on (features.loc_node): sim_survey on sim_iris_downcam, frames from the topic camera/image, detector 'oracle', waiting for /loc/set_enabled
/loc/set_enabled: localisation pipeline started
[search_pattern] localisation pipeline started
[search_pattern] survey finished: 3 of 3 tracks flown in 64 s, 0 climb(s) back to the survey height
/loc/set_enabled: localisation pipeline stopped after 571 frames
```

What `sim/loc_check.py` heard on `/loc/status` and `/loc/detections` beside the flight (the run's
`loc_check.json`, and the report's row):

| | Last flight (final code) | Line |
| --- | --- | --- |
| Started by the mission, and stopped at the survey's end | yes | both |
| Frames processed | 571 | |
| Frames dropped | 0 | |
| Rate, median of the node's status | 10.0 a second | at least 8 |
| Rate, slowest five seconds | 9.83 a second | at least 6 |
| A frame's age at its turn, 95th percentile | 33 ms | at most 250 ms |
| Frames that faulted | 0 | 0 |
| Boxes published | 177: 96 `boundary_marker`, 81 `marker` | each class on the field at least once |
| Boxes put on the ground | 177 of 177 | at least 90% |
| Messages on `/loc/detections` | 571, one a frame | |

```
| Localisation pipeline | PASS: 10.0 frames a second at the median (slowest 9.8) over 571 frames, 0 dropped, detector 'oracle'; frames 33 ms old at their turn, at the 95th percentile; boxes: 96 boundary_marker, 81 marker; 177 of 177 put on the ground |
| Lines expected in the mission's log | 3 of 3 found |
```

The rate is the camera's: the sim's survey camera gives 10 frames a second and the oracle takes no time, so this
shows the pipeline keeps up with its camera and drops nothing, not what a detector costs. The vehicle flew the
survey as without the node (6 of 6 track ends, closest approach 0.05 to 0.28 m; height estimator beside the
flight p95 0.02 m).

With every flag off the node is not started (no `loc_node` line in the launch log), the survey mission is
skipped and the vehicle takes off and returns: OK, 0 of 6 track ends.

### The flights before, and what each found

| Flight | Result | What it found |
| --- | --- | --- |
| 1 | NEVER_ARMED | The sim itself did not come up: with three more processes on ROS, the processes on the 256 MB shared-memory segment overran the container's 1 GB. Fixed in the sim (below) |
| 2 | FAIL: 138 of 173 boxes placed | A frame's turn came 34 ms after its capture, before MAVROS's pose of that moment: "no pose for the frame's capture time". The pipeline now waits up to 0.15 s for a frame's pose |
| 3 | OK: 175 of 175 placed, 10.0 a second, 571 frames, 42 ms | |
| 4 (flags off and on) | OK both: 176 of 176 placed, 10.0 a second (slowest 9.81), 34 ms | After the first review round's changes |
| 5 | OK (the table above) | After the second round's |

## The camera frames with the profile on the bridge alone

The `large_frames` Fast DDS profile used to be given to every process in the container; it is now given to the
camera bridge only. The bar from the lead's wave 3 note 3 (the survey camera at 8 Hz or more), flown again:

```bash
sim/run_scenarios.sh hover/survey_camera --seeds 1 --variants off
sim/run_scenarios.sh hover/still4k_camera --seeds 1 --variants off
```

```
| hover/survey_camera | 1 | OK | 1.00 | held within 0.01 m for 73 s | not run |  |  |
| hover/still4k_camera | 1 | OK | 1.00 | held within 0.01 m for 73 s | not run |  |  |
camera: 10.0 frames of 1280 x 720 a second of sim time arrived, 892 in all, slowest 5 s 10.0 Hz (at least 8 needed)
camera: 1.0 frames of 3840 x 2160 a second of sim time arrived, 90 in all, slowest 5 s 1.0 Hz
```

| | Every process on the profile (`docs/results/2026-11-20-survey-frames.md`) | The bridge alone |
| --- | --- | --- |
| Survey camera, frames arriving | 10.0 Hz, 901 frames | 10.0 Hz, 892 frames |
| 4K still camera, frames arriving | 1.0 Hz, 89 frames | 1.0 Hz, 90 frames |
| Processes whose log has "Failed to create segment" | three in the survey-camera hover (`fly.log`, `sensor_check.log`, `truth_poses.log`) | none, in both hovers and in the pipeline flights |

`hover/survey_camera` flown with its flags on (sim time) fails a different line of the sensor check, as it did on
18 November: more than 5% of rangefinder stamps ahead of the check's clock (1,459 of 1,773; the camera in that
flight: 10.0 Hz, 887 frames). It is recorded under Decisions and is not the frame rate.

## Not shown

- A trained detector: none exists. `detector: engine` has run against a stand-in model in tests only.
- 4K frames through the node, the camera's own `FrameSource`, and the rate on the Jetson: the bench.
- Where the boxes fall on the ground against the truth: there is no tracker's list yet (LOC-7, gate G4). The
  oracle finds its pixels with the camera model the pipeline places them with, so these flights do not show that
  the camera's geometry is right.
- Wind, other seeds.
