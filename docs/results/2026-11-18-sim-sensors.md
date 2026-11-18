# The sim's sensors (SIM-05)

Measured on 18 Nov 2026 (UTC) on a club laptop, on `platform/build-2027` at `504d4ab` plus the SIM-05 change. Sim image `argus-sim:humble`, headless.

**Result: one pass line met, one not.**

- **Rangefinder: PASS.** In a hover MAVROS's rangefinder was 0.006 m from the true height at the 95th percentile (0.1 m allowed).
- **`survey` at 8 Hz or more with a real-time factor of 0.8 or more: NOT MET as a subscriber sees it.** The camera (1280 x 720) makes 10.0 frames a second of sim time at a real-time factor of 0.99, but only 5.5 a second arrive at a ROS subscriber with the sensor-data quality of service the camera node and the detector use. Where the other 45% are lost (Gazebo to the bridge, or the bridge to the subscriber) was not located.
- The flow profile holds this laptop to a real-time factor of 0.58, so a flight with it does not count here.

## The pass flight

```
$ sim/run_scenarios.sh hover/survey_camera --seeds 1 --variants on
| hover/survey_camera | 1 | not run |  |  | FAIL | 1.00 | held within 0.01 m for 73 s |
**FAIL**: the sensor check failed: the camera gave 5.5 frames a second of sim time (at least 8 needed)
| Real-time factor while armed | 0.99 |
| Sensors | camera: 5.5 frames of 1280 x 720 a second of sim time arrived, 485 in all, slowest 5 s 4.0 Hz (at least 8 needed) |
| Sensors | true poses on /sim/truth/poses: 26.9 messages a second of sim time over 2395, for field_parcel, field_target, iris_survey |
| Sensors | rangefinder against the true height in a hover: PASS, median 0.004 m, p95 0.006 m over 1040 readings (limit 0.1 m); readings 1 ms old by their stamps at the median, 4 ms at p95 |
```

`sim/sensor_check.py` ran beside the flight (takeoff to 10 m, a 40 s hold, return; a bullseye and a package stand on the field).

| | Count over the flight | Rate, sim time |
| --- | --- | --- |
| `camera_info` messages (one per frame the camera made) | 893 | 10.0 Hz, slowest 5 s 10.0 Hz |
| Frames that arrived on `/camera/image` (best effort, as raw bytes) | 485 | 5.5 Hz, slowest 5 s 4.0 Hz |

An earlier flight of the same scenario, judged on `camera_info` alone, was reported OK at 10.0 Hz. That count is of what Gazebo scheduled, not of what a node receives, and the check now holds the floor against arrivals. A frame here is 2.8 MB. The check's subscriber is the only one counted, so the run shows that 893 frames were made and 485 reached it, and no more than that. Possibly related, not shown to be the same thing: the 2026 detector puts out about 22 detection messages a second from the delivery camera's 30 frames (`docs/results/2026-11-04-sim-baseline.md`), which was put down to its own per-frame work. Not tried: a second subscriber, a reliable subscription, or larger transport buffers in the sim container. Under "Needs a person" in `docs/build/PROGRESS.md`.

The rangefinder is compared with how far the vehicle's true pose has risen from where it stood, over the readings taken above 2 m, slower than 0.3 m/s and tilted less than 5 degrees; the largest of the 1040 was 0.015 m off. On this sim-time run each reading is placed at its own stamp; the stamps were 1 ms behind Gazebo's clock at the median when heard (4 ms at p95). After this flight the check was given a rule for that (a sim-time run fails if more than 5% of readings are stamped off the sim clock, or their age passes 100 ms at p95), which is unit-tested and has not been flown.

True poses: 2395 messages named the vehicle, the bullseye (which never moves) and the package.

The flight was on sim time, so MAVROS ran with the sim's plugin list (`sim/config/mavros_sim.yaml`). Its log shows 16 plugins created and initialised, the flight's twelve and `optical_flow`, `vision_pose`, `vision_speed`, `distance_sensor`, and every other plugin ignored. The time check and the rate check of a sim-time run both passed.

## The other profiles

| Flight | Status | Downward camera made (sim time) | Frames that arrived | Also | Real-time factor | Rangefinder p95 |
| --- | --- | --- | --- | --- | --- | --- |
| `hover/survey_camera`, on | FAIL: frames arriving | 10.0 Hz, 893 frames | 5.5 Hz, 485 of 1280 x 720 | true poses at 26.9 Hz | 0.99 | 0.006 m |
| `hover/still4k_camera`, off | OK (no floor set) | 1.0 Hz, 90 frames | 0.6 Hz, 54 of 3840 x 2160 | | 1.00 | 0.006 m |
| `hover/flow_camera`, off | INVALID: real-time factor | 30.3 Hz, 2228 frames | not counted in that flight | flow camera made 62.5 Hz, 4595 frames | 0.58 | 0.006 m |

The flow flight was flown before the check counted arrivals. Both of its cameras make their frames at their rates in sim time (Gazebo steps in whole milliseconds, so 60 Hz comes out as one frame every 16 ms). But rendering 90 frames per sim second is more than this laptop does in real time, and the runner does not count a flight under a real-time factor of 0.8. The scenario is kept out of the nightly suite for that reason.

## The rangefinder is not SITL's type 100

FEATURES.md names SITL's rangefinder type 100. With it (`RNGFND1_TYPE 100` in the run's parameters) the vehicle did not arm:

```
$ sim/run_scenarios.sh hover/survey_camera --seeds 1 --variants on     # with RNGFND1_TYPE 100
   WARNING: pre-arm checks still failing after 600s. Last FCU messages:
   FCU: PreArm: Rangefinder 1: No Data
```

Type 100 reads ranges the simulator's physics backend supplies, and with the Gazebo backend nothing supplies them. The default parameters' analog rangefinder (`RNGFND1_TYPE 1`) reads the height above the flat ground through a simulated voltage, which is what the numbers above are from: it sees only flat terrain, as the spec wants of this step. The builder leaves it as it is.

## Not shown

- **The survey camera's frames arriving at 8 Hz.** The open pass line, above.
- **The flow profile in a flight that counts.** It needs a faster machine or a cheaper camera.
- **The Gazebo one-beam lidar** that would see the bucket (P1 in the spec) is not built; the reason is under Decisions.
- **Messages through the four added MAVROS plugins.** They load; nothing sends optical flow, a vision pose or a distance yet.
- One flight of each scenario, one seed.

## Commands

```bash
python3 sim/sensor_profiles.py --check
python3 -m pytest -q sim/test/test_sensors.py
sim/run_scenarios.sh hover/survey_camera --seeds 1 --variants on
sim/run_scenarios.sh hover/still4k_camera --seeds 1 --variants off
sim/run_scenarios.sh hover/flow_camera --seeds 1 --variants off
```

Run folders (not committed): `sim/.state/suites/hover-survey_camera-20261118T065525Z/`, `sim/.state/suites/hover-still4k_camera-20261118T065808Z/`, `sim/.state/suites/hover-flow_camera-20261118T063951Z/`. The earlier survey flight judged on `camera_info`: `hover-survey_camera-20261118T063707Z/`. The type 100 attempt: `hover-survey_camera-20261118T062204Z/`.
