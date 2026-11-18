# SIM-05: Sensors

Flag: none (sim only; a scenario picks a camera with `sensor_profile:`, and the default is today's vehicle, byte for byte) · Status: PARTIAL · Commits: `git log --oneline --grep '^SIM-05:' platform/build-2027`

## What it solves

The sim had one camera, 640 x 360 at 30 Hz, written by hand, no flow camera, no true poses on a topic, and nothing that checked a sensor against the truth. Localisation needs a wider and a 4K view, the flow lane a flow camera, and the oracle detector true poses. This generates the vehicle's camera variants from one template and adds the checks.

## Architecture

- `sim/sensor_profiles.py` (new, standard library only) and `sim/models/iris_camera.sdf.template`: four profiles, `delivery`, `survey`, `still4k`, `flow`. It writes each profile's model (`sim/models/iris_<profile>/`), bridge file (`sim/config/bridge_<profile>.yaml`) and the camera driver's calibration (`src/argus_camera/config/calibration/sim_<profile>.yaml`). The files are committed; `--check` and a test fail when one is out of date.
- `src/argus_camera/config/modes.yaml`: modes `sim_survey`, `sim_still4k`, `sim_flow`.
- `sim/sensor_check.py` (new): a side tool for scenarios with `sensor_check: true`. The rate of camera frames that arrive in ROS, rangefinder against the true height in a hover, and the true-pose topic and its names when it is asked for. Writes `sensor_check.json`.
- `drone_mission_core/frame_projection.py`: `load_camera` refuses a sim sensor mode on an airframe that is not the sim's, and a real one on the sim's (only reached with a projection flag on).
- `sim/truth_poses.py` (new): a side tool for scenarios with `truth_poses: true`. Publishes `/sim/truth/poses` (`tf2_msgs/TFMessage`, 30 Hz of sim time).
- `sim/config/mavros_sim.yaml` (new): the flight's twelve MAVROS plugins plus `optical_flow`, `vision_pose`, `vision_speed`, `distance_sensor`. `sim/container/start.sh` uses it on sim-time runs, and refuses a window on a headless-only profile.
- Scenarios `hover/survey_camera`, `hover/still4k_camera` (both in the nightly suite) and `hover/flow_camera`.

## Flow

Before: every scenario flew `iris_downcam`; `sensor_profile: survey` was refused.

After: `sensor_profile: survey` → the builder swaps the world's vehicle for `iris_survey` and copies its bridge file → the runner starts `sensor_check.py` → arriving frames are counted by their stamps, each rangefinder reading is compared with the rise of the vehicle's true pose → `sensor_check.json` → the run is FAIL if the rate is under 8 Hz, the real-time factor under 0.8, or the rangefinder over 0.1 m off at p95.

## Code

- `sensor_profiles.py:47`, `PROFILES`, `:149`, `render_model`, `:226`, `generated_files`: the profiles and everything generated from them.
- `sensor_check.py:101`, `TrueHeight`, and `:134`, `range_check`: the true height is the rise from where the vehicle stood (SITL's rangefinder reads 0 on the ground), and only hover readings count.
- `scenario/spec.py:54`, `SENSOR_PROFILES`, and `scenario/build.py:274`: the profile picks the model the world includes and the manifest's `min_rate_hz`.
- `runner/result.py:200`: the sensor check's verdict, and the real-time-factor floor of a profile that has a rate floor.
- `container/start.sh:56` and `:259`: no window on a headless-only profile; MAVROS's sim plugin list.

## Evidence

```
$ python3 -m pytest -q -p no:cacheprovider sim/test/test_sensors.py
23 passed in 0.21s
$ python3 sim/sensor_profiles.py --check; echo $?
0
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

Sim ([docs/results/2026-11-18-sim-sensors.md](../../results/2026-11-18-sim-sensors.md)):

```
$ sim/run_scenarios.sh hover/survey_camera --seeds 1 --variants on
| hover/survey_camera | 1 | not run |  |  | FAIL | 1.00 | held within 0.01 m for 73 s |
| Real-time factor while armed | 0.99 |
| Sensors | camera: 5.5 frames of 1280 x 720 a second of sim time arrived, 485 in all, slowest 5 s 4.0 Hz (at least 8 needed) |
| Sensors | true poses on /sim/truth/poses: 26.9 messages a second of sim time over 2395, for field_parcel, field_target, iris_survey |
| Sensors | rangefinder against the true height in a hover: PASS, median 0.004 m, p95 0.006 m over 1040 readings (limit 0.1 m); readings 1 ms old by their stamps at the median, 4 ms at p95 |
$ sim/run_scenarios.sh hover/still4k_camera --seeds 1 --variants off
| hover/still4k_camera | 1 | OK | 1.00 | held within 0.01 m for 73 s | not run |  |  |
$ sim/run_scenarios.sh hover/flow_camera --seeds 1 --variants off
| hover/flow_camera | 1 | INVALID | 1.00 | held within 0.00 m for 57 s | not run |  |  |
```

The rangefinder line is met. The survey line is not: the camera makes 10.0 frames a second at a real-time factor of 0.99, and 5.5 a second arrive at a subscriber. The flow flight is INVALID for its real-time factor (0.58).

## Spec check

spec-auditor, round one: INCOMPLETE, PARTIAL the right status. Fixed after it: the frame rate is now counted from frames that arrive (which turned the survey line from a pass on a proxy into a measured fail); true poses of objects were flown and their names are required; tests for the real-time-factor floor. Recorded with a reason and an owner: the P1 lidar (REC-2), the flow profile's real-time factor (lead). Deviations recorded: rangefinder type 1 in place of 100 (type 100 cannot arm), the MAVROS list on sim-time runs only, true poses from a side tool.

spec-auditor, round two (those rows only): the fail is reported faithfully, nothing hidden; close as PARTIAL with the unmet line first. Wording about where the frames are lost was corrected after it (it was not located).

flight-guardian, two rounds: CONCERNS, no blocker. After round two, not reviewed again: the camera guard works both ways, and a sim-time run fails on rangefinder stamps off the sim clock (unit-tested, not flown). Fixed: `load_camera` refuses a sim sensor mode on a non-sim airframe (the new sim calibrations are at the aircraft's image sizes); the test now compares the sim's MAVROS file whole against the aircraft's; on sim-time runs the rangefinder check uses the readings' stamps and reports their age.

## Not done

Why the row is PARTIAL, and who has what is left:

- **Survey frames arriving at 8 Hz**: the pass line is open, and where the missing frames go is not located. How large frames travel in the sim is the lead's choice, with the LOC lane (under "Needs a person").
- **The flow profile in a flight that counts** (real-time factor 0.58 on a club laptop): the lead's choice of machine or a cheaper camera. Its field of view and mount are stand-ins until the real camera is calibrated.
- **The Gazebo one-beam lidar** (P1): with REC-2, which needs a range to the bucket; reason under Decisions.
- A mission flown with the survey or 4K camera needs its `camera_mode` set to `sim_survey` or `sim_still4k`; the builder does not tie the two together yet (LOC lane, with its first scenario).
- Nothing sends through the four added MAVROS plugins yet.
- One flight of each scenario, one seed.
