# argus_localisation

The localisation pipeline (LOC-5): the node that turns the survey's frames into marker detections, and the plain modules it wraps.

Everything here is behind node flags that default to false. As shipped, the node does nothing.

| Flag (`features.` under `loc_node`) | What it turns on |
| --- | --- |
| `loc_node` | The pipeline: frames in, `/loc/detections` and `/loc/status` out, `/loc/set_enabled` served. |
| `sim_oracle_detector` | The sim's stand-in detector (`detector: oracle`). Refuses to run outside the sim. |
| `loc_tiling` | The detector sees overlapping windows of the full frame instead of the shrunk frame. |
| `marker_tracker` | The marker tracker runs on every frame's ground points and `/loc/tracks` is published. |
| `loc_median_position` | A track's published position is the weighted median of its sightings (needs `marker_tracker`). |
| `survey_frame_log` | Every frame of the survey is logged as JPEG under `frame_log_dir`, giving way to the detector. |
| `digit_reader` | The digit of every marker crop is read, or the reader abstains. |
| `template_digit_classifier` | The reader's stand-in classifier for the sim's marker model (needs `digit_reader`; sim only). Not a trained model. |

## Per frame

1. `shrink.shrink_for_detector`: the whole frame to 1280 x 720 with INTER_AREA (exactly a third of a 4K frame). Training images are to be made with the same function.
2. The detector looks at it. Boxes under a confidence of 0.25 are dropped.
3. Boxes are scaled back to the full frame's pixels: that is what the detection contract (FND-02) carries.
4. A crop of each marker is cut from the full frame for the digit reader (LOC-6).
5. Each box's centre is projected to the ground with the pose of the frame's capture time (projection v2, FND-05).
6. The tracker (LOC-7) is handed the ground points, the digit reader the crops, when there is one.

The pipeline is off until `/loc/set_enabled` (std_srvs/SetBool) starts it; the survey mission (`search_pattern`) starts it at its first track and stops it at its end. Only the newest frame is ever processed: a frame that waited behind a newer one is dropped and counted, never queued. The target is 15 frames a second and the minimum 8; `/loc/status` says when the rate is under it.

## The marker tracker

`marker_tracker.py` (LOC-7) keeps one track for each `marker` and each `boundary_marker`. A sighting's noise is the target estimator's (R = J Sigma J^T from projection v2's point); sightings are assigned to tracks with the Hungarian method on Mahalanobis distance, gated at 9.21 and at 1 m. A track is TENTATIVE until its third sighting, then CONFIRMED; a confirmed track that is no longer in view is COASTING and is kept for good. A sighting within 1 m of a track that the gate refuses is not a second track: after five in a row the track moves to it. Once the four boundary markers are confirmed, a marker track outside their hull (by more than 0.5 m) is marked outside and kept off the list. Each track keeps its sightings for the final position (LOC-9).

## The digit reader

`digit_reader.py` (LOC-6) takes each `marker` crop to a digit or to no answer: the plate's corners (the pixels outside the greys of the crop's rim), a homography in undistorted coordinates to a 64 x 64 patch, the X's arms for the quarter turn, and a classifier on the patch's four quarter turns. It abstains under 24 px a side, without a clear marker, or under `digit_min_confidence` (0.9).

- **Which way up.** This year's marker (`LAYOUT_2027`, the digit centred on the X) has no cue for the half turn, so the two ways up the arms allow are compared: a digit that reads one way only, or the same both ways, is answered; a 6 or a 9 is not, and its reading says "6 or 9" for the vote over the track. Last year's plate (`LAYOUT_2026`) has its digit to one side, which settles it.
- **The classifier** is the ONNX model at `digit_model_path`, run on the CPU; none is shipped. `features.template_digit_classifier` selects a stand-in that reads the seven-segment digit of the sim's marker model; it starts only in the sim (sim time and `sim_only`).
- **Beside the detector.** At most 6 crops a frame, the most confident boxes first, within 30 ms; the time is `digits` in the frame's timings and `digits.last_ms` in `/loc/status`, with the counts of crops read, abstained on and skipped.
- **If it cannot start** (no model, no ONNX Runtime, a bad setting) the node says so, `/loc/status` has `digits.error`, and boxes and tracks go on without digits.

`tools/digit_reader_eval.py` measures precision, coverage and the time a crop takes, on synthetic markers or on a folder of labelled crops (`--crops`), with the stand-in or an ONNX model (`--model`).

## Logging the survey's frames

With `features.survey_frame_log` the node logs every frame the camera gives while the pipeline is on, as JPEG at quality 90, into a new folder for each survey under `frame_log_dir` (`frame_logging.py`, LOC-11; the writer is argus_camera's `FrameLog`, encoding on `frame_log_workers` threads). The encoding threads run at a lowered priority. The logger gives way to the detector: when the pipeline runs under nine tenths of the rate it should have (its target, or its camera's rate when that is lower), under its minimum rate, or the log's own queue overflows, it keeps every second frame, then every fourth, down to one in sixteen, and steps back up after ten seconds with nothing wrong. `frame_log_dir` must exist; the logging stops with under `frame_log_min_free_gb` free on the disk or when one survey's log reaches `frame_log_max_gb`, and stopping the pipeline does not wait for the frames still queued. `/loc/status` carries its counts under `frame_log`. `tools/frame_log_bench.py` measures what the logging costs on a machine.

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
| `/loc/tracks` | `argus_interfaces/MarkerTrackArray` after every frame, with `features.marker_tracker`: the `marker` tracks, in the local ENU frame (`map`). `digit` is -1 until the digit reader exists. |
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
