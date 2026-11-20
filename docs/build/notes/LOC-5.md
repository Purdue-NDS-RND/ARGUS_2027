# LOC-5: `argus_localisation` package and `loc_node`

Flags: `features.loc_node`, `features.sim_oracle_detector`, `features.loc_tiling` (node flags, all default false) · Status: PARTIAL · Commits: `git log --oneline --grep '^LOC-5:' platform/build-2027`

## What it solves

Nothing turned the survey's frames into marker detections on the 2027 contract: last year's `yolo_node` took images over a topic, one queue behind the camera, with boxes in the pixels of whatever it was given. This is the node the survey mission starts and stops, built so that a full frame never crosses a topic and a late frame is dropped, not queued. It is the pipeline itself, with the sim's oracle standing in for the detector that cannot be trained yet.

## Architecture

- New package `src/argus_localisation`. ROS-free: `shrink.py` (`shrink_for_detector`), `pipeline.py` (one frame: shrink, detect, scale back, crop, project; `SiteGround`), `runtime.py` (on and off, newest frame only, the status), `oracle.py` (the sim's stand-in detector), `engine.py` (an engine file run through Ultralytics, as in 2026), `tiling.py`, `frames.py`. The node: `loc_node.py`.
- Topics: `/loc/detections` (the FND-02 contract), `/loc/status` (JSON on `std_msgs/String`). Service: `/loc/set_enabled` (std_srvs/SetBool), which LOC-3's mission calls. In: MAVROS's pose, velocity and rangefinder; the sim's image topic (queue of one) or the camera's own `FrameSource`; `/sim/truth/poses` for the oracle.
- `drone_mission_demo`: `launch/search_pattern_demo.launch.py` (starts the node on `loc_node:=true`), `config/params/sim_survey_mission.yaml`.
- Sim: `sim/loc_check.py` beside every flight with the node on; the scenario build writes the oracle its objects; scenario `waypoint_navigation/survey_pipeline`. The `large_frames` DDS profile now goes to the camera bridge alone (SIM-05, its own commit).

## Flow

One frame of the survey, node flag on, pipeline started:

- Before (2026): image topic → `yolo_node` slices and infers every queued frame → boxes in slice-merged pixels on its own topic.
- After: newest frame only (a waiting frame is replaced and counted) → wait up to 0.15 s for the pose of its capture time → shrink to 1280 x 720 → detector → boxes back to full-frame pixels → crops of markers for the digit reader → each centre onto the ground (projection v2; the rangefinder's ground or the site's) → `/loc/detections`; once a second `/loc/status`.

## Code

- One frame: `src/argus_localisation/argus_localisation/pipeline.py:145`; the crops `:95`; the ground the boxes are put on `:44`.
- Newest frame, the wait for its pose, faults counted: `runtime.py:85`; the rate as frames over the last 5 s and the low-rate flag: `:129`, `:140`.
- The node's start, everything that can refuse before anything is made on the ROS side: `loc_node.py:152` (the camera opened at `:193`, the service made last at `:218`); the camera's own thread `:257`.
- The oracle: sim-only refusal `oracle.py:54`; the true objects through the camera model `:152`.
- The engine detector and its blank frame at load: `engine.py:63`. The shrink: `shrink.py:42`.

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider src/argus_localisation/test
46 passed in 2.00s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh waypoint_navigation/survey_pipeline --seeds 1 --variants on
| waypoint_navigation/survey_pipeline | 1 | not run |  |  | OK | 6.00 | 6 of 6 waypoints |
| Localisation pipeline | PASS: 10.0 frames a second at the median (slowest 9.8) over 571 frames, 0 dropped, detector 'oracle'; frames 33 ms old at their turn, at the 95th percentile; boxes: 96 boundary_marker, 81 marker; 177 of 177 put on the ground |
```

`docs/results/2026-11-20-loc-pipeline.md`: the mission started and stopped the pipeline; 571 frames at the camera's 10 a second, none dropped, none faulted, every box placed; flags off, the node is not started.

## Spec check

- spec-auditor, round 1: INCOMPLETE (the `FrameSource` path untested, tiling wrong with the oracle, no results file, the low-rate flag blind to no frames). Round 2: those DONE but the results file (written after the round) and the engine row, which is PARTIAL and before the lead.
- flight-guardian, round 1: seven concerns, no blocker. Round 2: five resolved (the pose wait that newer frames restarted, the rate blind to a stall, a service offered by a node that had not started, the camera thread dying silently, the oracle with tiling), one resolved in code with a person step (the site's ground), one recorded (the oracle shares its camera model with the projection); three new, all changed after the round and not re-reviewed (an engine that cannot run refused at load, a parameter of the wrong kind, each frame's own wait).
- Different from the spec's words, each under Decisions: the shrink is to 1280 x 720 (the dataset rule), with the 736 padding as helpers nothing calls yet; `/loc/status` is JSON on a String; `detector: engine` runs an existing engine file the 2026 way, on the ledger's note, which the lead is asked to confirm; the `GetCrops` service is not served.

## Not done

- **Bench (why PARTIAL).** A real engine on the Jetson, 4K frames from the camera's own `FrameSource`, the 15 Hz target and 8 Hz minimum there: "Needs a person".
- The 2027 engine: PLT-04 and training data, on hold.
- The tracker (LOC-7) and the digit reader (LOC-6) plug into `LocPipeline`; until then the status says 0 tracks.
- `ground_below_home_m` is set twice, in the sequence and in the node's params ("Needs a person").
- Ground accuracy against the truth: gate G4, once there is a list.
