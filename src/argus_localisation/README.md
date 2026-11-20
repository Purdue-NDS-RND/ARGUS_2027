# argus_localisation

The localisation pipeline (LOC-5): the node that turns the survey's frames into marker detections, and the plain modules it wraps.

Everything here is behind node flags that default to false. As shipped, the node does nothing.

| Flag (`features.` under `loc_node`) | What it turns on |
| --- | --- |
| `loc_node` | The pipeline: frames in, `/loc/detections` and `/loc/status` out, `/loc/set_enabled` served. |
| `sim_oracle_detector` | The sim's stand-in detector (`detector: oracle`). Refuses to run outside the sim. |
| `loc_tiling` | The detector sees overlapping windows of the full frame instead of the shrunk frame. |

## Per frame

1. `shrink.shrink_for_detector`: the whole frame to 1280 x 720 with INTER_AREA (exactly a third of a 4K frame). Training images are to be made with the same function.
2. The detector looks at it. Boxes under a confidence of 0.25 are dropped.
3. Boxes are scaled back to the full frame's pixels: that is what the detection contract (FND-02) carries.
4. A crop of each marker is cut from the full frame for the digit reader (LOC-6).
5. Each box's centre is projected to the ground with the pose of the frame's capture time (projection v2, FND-05).
6. The tracker (LOC-7) is handed the ground points, the digit reader the crops, when there is one.

The pipeline is off until `/loc/set_enabled` (std_srvs/SetBool) starts it; the survey mission (`search_pattern`) starts it at its first track and stops it at its end. Only the newest frame is ever processed: a frame that waited behind a newer one is dropped and counted, never queued. The target is 15 frames a second and the minimum 8; `/loc/status` says when the rate is under it.

## Frames

A real camera mode is read by argus_camera's `FrameSource` inside the node, on a thread of its own, so full frames never cross a ROS topic. The sim's camera modes come from `image_topic`, taken with a queue of one.

## Detectors

- `detector: engine` runs the TensorRT engine at `engine_path` the way the 2026 node did, through Ultralytics, on the Jetson (`engine.py`). `engine_classes` says what each of the engine's class numbers is on the detection contract; last year's engine has one class, the marker. The node builds no engine: with no file, or on a machine without Ultralytics, it says so and does not start. Last year's engine was trained on slices of the frame, so it wants `features.loc_tiling`; the 2027 engine (1280 x 736, static) waits for training data (PLT-04). Ultralytics fits the 1280 x 720 frame to the engine's input itself; `shrink.pad_for_engine` and `shrink.from_engine` make and undo the 1280 x 736 frame for a detector that feeds such an engine directly, and nothing calls them yet.
- `detector: oracle` is for the sim only. It does not look at the picture: it puts the scenario's true objects (`/sim/truth/poses`) through the camera model at the frame's capture time and reports them with pixel noise, misses and false alarms from a seeded generator. It needs `features.sim_oracle_detector`, `use_sim_time` and `sim_only: true`, and refuses `features.loc_tiling`.

## Topics

| | |
| --- | --- |
| `/loc/detections` | `vision_msgs/Detection2DArray` on the detection contract: the frame's capture time, full-frame pixels, class `marker`, `boundary_marker`, ... |
| `/loc/status` | `std_msgs/String` of JSON, once a second: `enabled`, `frames`, `rate_hz`, `dropped`, `detections`, `placed`, `refused`, `faults`, `frame_age_ms`, `below_minimum_rate`, `tracks`, `confirmed_ids`, `timings_ms`. |
| `/loc/set_enabled` | `std_srvs/SetBool`: true starts the pipeline, false stops it. |

## Running it

In the sim, a scenario turns the node on (`sim/scenarios/waypoint_navigation/survey_pipeline.yaml`):

```bash
sim/run_scenarios.sh waypoint_navigation/survey_pipeline --seeds 1 --variants on
```

`sim/loc_check.py` runs beside that flight and holds the pipeline to its rate, to frames no older than 250 ms at their turn, and to having reported every kind of object on the field.

## Tests

```bash
python3 -m pytest -q src/argus_localisation/test
```
