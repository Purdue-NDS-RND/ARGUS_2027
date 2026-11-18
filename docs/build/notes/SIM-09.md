# SIM-09: Replay harness (record on aircraft, replay anywhere)

Flag: none (sim and ground tooling; the recorder is started on purpose, `--record` or the launch file) · Status: PARTIAL · Commits: `git log --oneline --grep '^SIM-09:' platform/build-2027`

## What it solves

There was no way to run the flight code on a flight that had already happened: every question about the estimator or about timing needed a new sim flight, and a field flight could not be examined at all. This records a flight's topics and replays them, without ROS and deterministically (`core`) or through `ros2 bag play` (`ros`). It also answers the question left open in wave 1, where the 260 to 275 ms frame age comes from.

## Architecture

- `sim/replay/` (new): `topics.py` (the one list of what is recorded), `msgdefs.py` and `cdr.py` (message layouts and a CDR codec, no ROS), `bag.py` (rosbag2's sqlite3 and MCAP, read and written without ROS), `core.py` (the `core` replay), `metrics.py`, `latency.py` (frame age by stage), `record.launch.py`, `taps.py` (one header per frame seen, and the sim's clock), `record_run.py` (the sim runner's recorder), `play.launch.py` (the `ros` mode), `check_defs.py` (the layouts against ROS itself).
- `sim/run_scenarios.sh --record`, through `scenario/build.py` (`record` in the run's manifest) and the runner's side tools: any scenario's run folder gets a `bag/`.
- `TargetEstimator`'s `FrameReport` carries the normalised innovation of each measurement (`nis`), and `EstimatorFeed.on_report` hands each frame's report to whoever asks: the replay's hook. Nothing reads either in flight.
- `drone_utils/package.xml`: `ros2bag` and `rosbag2_storage_mcap` as run-time dependencies, so the next image build and the Jetson's `rosdep` install them.
- New topics, published only by the recorder's taps: `/replay/frame_seen/<image topic>` (`std_msgs/Header`) and `/replay/sim_clock`.

## Flow

Record: `ros2 bag record` on the list + the taps → a bag with no images in it.

`core`: the bag's messages in recorded order → decoded in plain Python → poses, velocities and rangefinder readings into the same `PoseBuffer` and `GroundHeight` the executor uses, contract detections into the same `EstimatorFeed`, at their recorded arrival times → the tracks read every 100 ms of bag time → metrics. No clock is read, so the same bag gives the same bytes.

Frame age: for each frame, its stamp → `camera_info` heard → the image heard (tap) → the detector's detection published → heard: four stages that add up to the age.

## Code

- `core.py:42`, `Replay`, and `:167`, `run`: one handler per topic, the 10 Hz ticks of bag time between messages, the 2026 detector replayed under the executor's own rule about its image size.
- `cdr.py:106` and `:164`, `decode` and `encode`: alignment counted from the end of the 4-byte CDR header; ROS's trailing padding is read past.
- `bag.py:144`, `_read_mcap`: records front to back, schemas feed the layouts, chunks uncompressed (zstd and lz4 only if their modules are there, with a message saying how to record without).
- `latency.py:130`, `breakdown`: which frame a detection came from (its own stamp, or the annotated copy published just before it), the four stages, and the detector's stage split into waiting and working, by height.
- `taps.py:35`, `header_bytes`: the header is cut from the front of the serialised image, so a 691 kB frame is never unpacked.
- `metrics.py:84`, `summarise`, and `:59`, `against_recorded`.

## Evidence

```
$ for f in cdr bag core latency record; do python3 -m pytest -q -p no:cacheprovider sim/test/test_replay_$f.py | tail -1; done
10 passed in 0.01s
11 passed in 0.05s
13 passed in 1.22s
8 passed in 0.27s
11 passed in 0.21s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

`test_the_same_bag_gives_the_same_bytes_every_time_and_in_either_storage` is the wave's exit line ("replay deterministic") in CI, on a bag the test generates.

On recorded sim flights ([docs/results/2026-11-18-replay.md](../../results/2026-11-18-replay.md)):

```
$ python3 sim/replay/check_defs.py                         (in a container of the sim image)
33 of 33 layouts match ROS; 0 differ; 0 not installed
$ python3 sim/replay/core.py <run folder>/bag --truth-file <run folder>/truth.json
error of the CONFIRMED bullseye track: median 0.010 m, p95 0.044 m, max 0.167 m over 677 ticks
NIS: mean 0.09, 0.0% above 9.21 over 1311 measurements
against the tracks the flight itself published: median 0.001 m, max 0.066 m apart over 672 ticks; CONFIRMED only in the replay 5, only in the flight 3
```

A second run wrote the same bytes. `play.launch.py` gave one `/clock`, original stamps and the flight's outputs under `/recorded`, after checking no MAVROS was live.

The frame age, on one `delivery/baseline` flight with every flag off ([docs/results/2026-11-18-frame-age-by-stage.md](../../results/2026-11-18-frame-age-by-stage.md)): 35 ms at the median, 228 ms at the 95th percentile; render 0.7 ms, bridge 14 ms, detector 19 ms (213 ms at the 95th percentile), publish 0.2 ms. The tail is frames waiting in the 2026 detector's queue, below 6 m.

## Not done

Why the row is PARTIAL, and who has what is left:

- **MCAP has not been recorded or read on a real bag.** The sim image has no MCAP plugin and the session did not rebuild the image (it would re-run the image's package installs just before the flags-off gate). The recorder falls back to sqlite3, which is what every bag in the session is; the MCAP reader has read only the replay's own writer's files. The lead rebuilds the image and repeats one recording (under "Needs a person").
- **Servo outputs are not in the recorded list.** MAVROS publishes them from a plugin the 2027 plugin list does not load. SIM-06's payload bridge is the first thing that reads servo outputs; the topic joins the list with it.
- **Flow frames, flow velocity error and per-marker localisation error** wait for the flow code, the flow camera (SIM-05) and the marker tracker (LOC-7). `core` replays the estimator, the only one of the three that exists.
- **4K frames as JPEG files with an index** are not written by anything yet: `argus_camera`'s `FrameLog` is the writer and nothing constructs it outside its tests. The `ros` mode replays the bag's topics, not frames: wiring the frame log to the driver and a frame player belong with the first field bag (G7).
- **On the aircraft the launch file taps no image topic unless told to**, because a tap costs the 2026 grabber and detector something that has not been measured on the Jetson (under "Needs a person"). Without taps a field bag has `camera_info` and detections but not the frames' arrival times.
- **The `ros` mode has not had a node under test run against it**, and `record.launch.py` has not run on the Jetson.
- **The error at the release** is computed for a time the caller gives (`--release-ns`); nothing yet reads the release time out of the bag (no servo outputs, and the fake drop is a log line).

## Spec check

spec-auditor, two rounds. Round one: PARTIAL is the right status, with eight things either to fix or to say. Fixed: camera info under the 2027 driver's names; lost-state duration counts a loss that is never made good; a test puts measurements over the gate through the NIS metric and ties the gate to the estimator's; latency on a wall-time bag goes through the sim clock. Said where the lead will see them (Decisions, "Needs a person", the results file): frames on disk, flow frames, the error at the release, the sim recording through `record_run.py`, the optional `zstandard` and `lz4` imports. Round two: the row can be closed as PARTIAL. After it the launch file was run under a real `ros2 launch`.

flight-guardian, two rounds, CONCERNS both times, no blocker; nothing in the change runs in a flight unless the recorder is started. Fixed after round one: on the aircraft the launch file taps no image topic unless told to; the player never puts recorded setpoints on MAVROS's topics and does not start beside a live MAVROS. Fixed after round two, not reviewed again: the player also refuses when it cannot tell whether a MAVROS is live, and the setpoint topics it moves aside are found by name. Left standing:

- **A frame tap on the aircraft costs something unmeasured.** It is opt-in and the bench step to measure it is under "Needs a person".
- **The player's check is made once, before it starts**, and `beside_live_mavros:=true` skips it. The rule is in the README: a bag is never played on the aircraft's network.
- **`ros2 bag record` itself on the Jetson** (CPU, disk) has not been measured.
