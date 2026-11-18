# FND-06: Target estimator (Kalman per target)

Flag: `use_target_estimator` (mission flag, default false, declared in all ten sequences; needs `use_frame_time_pose`) · Status: DONE · Commits: `git log --oneline --grep '^FND-06:' platform/build-2027`

## What it solves

In 2026 a mission steered on the newest raw detection: one frame's pixel, projected with whatever pose was newest, with no memory and no idea how good it was. This is the cause-level fix: one estimator inside the executor turns every detection into a ground point with the pose of its own frame (FND-05), keeps a small Kalman filter per target, and says where each target is, how sure it is and how old that knowledge is. Nothing flies on it yet: FND-07 gives missions the calls and CEN-01 makes the drop and delivery read it.

## Architecture

- `drone_mission_core/target_estimator.py` (new, numpy only): `TargetEstimator` (tracks, gate, track life, forecast), `measurement_noise` (R = J Σ Jᵀ), the per-class plane heights, `box_is_cut`.
- `drone_mission_core/assignment.py` (new, numpy only): the Hungarian method and `assign_gated`; LOC-7 will use the same code.
- `drone_mission_core/estimator_feed.py` (new): `EstimatorFeed` (no ROS: contract detections in, projection with the pose at the frame's time, position-reset detection, tracks out) and `attach_target_estimator`, the thin ROS side the executor calls.
- `PoseBuffer.predict_position` (new method): the vehicle's position up to 0.2 s ahead, for the actuation time.
- Executor, flag on: subscribes to `/perception/<detector>/detections` for each name in the new parameter `estimator_detectors` (default `target_cv`), once more to MAVROS's pose and to the 2026 detector's image size; publishes `vision_msgs/Detection3DArray` on `/estimator/targets` at 10 Hz from its own timer. Flag off: none of it exists.
- `FLAG_REQUIRES`: `use_target_estimator` requires `use_frame_time_pose`. `drone_mission_core` now depends on `vision_msgs` at run time.
- Sim: `sim/estimator_check.py` beside every flight with the flag on, the scenario `delivery/estimator` (in the nightly suite), three report rows.
- `tools/estimator_step_cost.py` (new): times one step of the feed on whatever machine it is run on.

## Flow

Before, and after with the flag off: detection point → mission, each tick, nothing kept.

After, flag on: `Detection2DArray` on the contract → decoded → waits until the pose history covers the frame's stamp → each box not cut by the image's edge is projected onto its class's plane with the pose at that stamp → R from the projection's Jacobians (2 px, 1°, 0.05 m or 0.5 m) → Hungarian assignment to the class's tracks on Mahalanobis distance, gate 9.21 → Kalman update, covariance floored at (0.02 m)² → TENTATIVE, CONFIRMED, COASTING, LOST by the frames' stamps → read out for now + 0.08 s → `/estimator/targets`.

## Code

- `target_estimator.py:264`, `_update_class`: the cost matrix, the gated assignment, the Joseph-form update, then the leftovers: a detection within 2 m of a track that got nothing is that target outside its gate (the fifth in a row moves the track), one beside a track that already has its detection is ignored, anything else starts a track.
- `target_estimator.py:194`, `_Track.hit`, and `:339`, `advance`: the track's life. A lost track that is seen again keeps its id and must confirm again.
- `estimator_feed.py:198`, `EstimatorFeed.step`, and `:239`, `_handle`: frames wait for their pose, are dropped when too old, ahead of the clock or in a gap of the pose history; at most four are handled per step and each leaves the queue first; cut boxes are skipped; frames taken before a position reset are shifted by it.
- `estimator_feed.py:86`, `PositionResetDetector`: a step of 0.5 m or more between two consecutive poses that the reported velocity does not explain.
- `estimator_feed.py:355`, `attach_target_estimator`: every callback is guarded, so a fault inside the estimator is counted and logged and never reaches the executor's tick.
- `assignment.py:19`, `hungarian`: shortest augmenting paths with potentials, checked against a search of every assignment.

## Evidence

```
$ PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_assignment.py
12 passed in 0.14s
$ PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_target_estimator.py
31 passed in 3.85s
$ PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_estimator_feed.py
17 passed in 0.09s
$ python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_executor_target_estimator.py
8 passed in 0.50s
$ python3 -m pytest -q -p no:cacheprovider sim/test/test_estimator_check.py
13 passed in 0.11s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

The tests were written first and failed for want of the modules. `test_executor_target_estimator.py` drives the real executor class with the flags on, from a `Detection2DArray` on the contract topic to a CONFIRMED track on `/estimator/targets`, and with the flags off checks that nothing was added (six subscriptions, four publishers, one timer). `test_p95_error_meets_the_acceptance_line_on_noisy_projections` runs the three acceptance heights on generated measurements with the noise the filter is told.

Sim, flags on, three flights of `sim/run_scenarios.sh delivery/estimator --seeds 1 --variants on`, the third on the code as committed ([docs/results/2026-11-18-target-estimator.md](../../results/2026-11-18-target-estimator.md)):

```
| delivery/estimator | 1 | not run |  |  | OK | 16.18 | 0.06 m from the target |
| Estimator's bullseye track against the truth | at 15 m: PASS, median 0.04 m, p95 0.10 m over 122 samples (limit 0.3 m) |
| Estimator's bullseye track against the truth | at 6 m: PASS, median 0.01 m, p95 0.02 m over 125 samples (limit 0.15 m) |
| Estimator's bullseye track against the truth | at 2.4 m: PASS, median 0.00 m, p95 0.01 m over 46 samples (limit 0.08 m) |
$ python3 tools/estimator_step_cost.py
one bullseye in one frame: median 0.22 ms, p95 0.23 ms, max 0.34 ms over 200 steps, 1 tracks (the tick is 50 ms)
the worst the bounds allow (4 frames of 16 detections): median 12.18 ms, p95 12.48 ms, max 13.70 ms over 200 steps, 16 tracks (the tick is 50 ms)
```

An earlier flight, on the 5 m bullseye and before cut boxes were skipped, failed the 15 m line (0.63 m): the results file says why and what changed.

## Spec check

spec-auditor, two rounds. Round one: INCOMPLETE on two rows, every other row DONE. The sim result was not yet in `docs/results/` and the check's judging rule was not recorded (both done since), and the flags-off executor had started importing numpy through `estimator_feed.py` (the imports are lazy again, and the flags-off test now checks that none of the estimator's modules is loaded). Round two: that row DONE; the acceptance row still PARTIAL for one reason, that the recorded flight was older than the last code changes. Flights 2 and 3 in the results file answer that; flight 3 is the code as committed. Not reviewed a third time.

flight-guardian, two rounds, CONCERNS both times, no blocker. Fixed after round one: the numpy import; work per step bounded (16 detections a frame, 4 frames a step); a failing frame leaves the queue once instead of being replayed into the filter; a clock stepping back makes every track LOST instead of "just seen"; the 2026 detector's detections refused when its image is not the calibrated size. Fixed after round two, not reviewed again: the per-frame trim goes class by class, so a crowd of markers cannot push out the one bullseye; the estimator keeps its own copy of the 2026 detector's image size and uses nothing from it until the size is known. Left standing:

- **The cost of a step on the flight computer is not measured.** 0.2 ms ordinarily and 12 ms at the bounds on a laptop; the tool to measure it on the Jetson is there and the step is under "Needs a person".
- **`primary()` returns a COASTING, TENTATIVE or LOST track when that is the best there is.** By design (CEN-06 flies back to a LOST one). Nothing reads it yet; FND-07 puts the descend and release rules in one place so that no mission has to remember to check.
- **Five rejections move a CONFIRMED track by up to 2 m without a state change.** That is the spec's rule; a consumer that cares sees it in the position.
- **Only the 2026 detector's image size can be checked.** Another detector publishing pixels of a reduced image would be believed. The contract says full-resolution pixels; nothing in a contract message says what size they are from.
- After a clock step back of under 0.5 s a track reads fresher than it is by that much, and for up to one step (0.1 s) after a larger one.

What differs from the spec, each recorded under Decisions: the module is `drone_mission_core/target_estimator.py` (the package's own name for "core"); a position reset is detected from the pose stream because the flight controller reports none through MAVROS; cut boxes are not used; the sim line is judged at the 95th percentile of CONFIRMED samples in a band around each height, on a 1.5 m bullseye.

## Not done

- **No mission reads the estimator.** FND-07 adds the context calls and the descend and release rules; CEN-01 makes the drop and delivery steer on it. `primary()` returns a COASTING or LOST track when that is all there is (CEN-06 needs the LOST one), so the state and age must be checked by whoever acts on it: that is FND-07's job, in one place.
- **A box cut by the image's edge is not used**, so with a box detector the track is lost once the target stops fitting in the image (about 6.5 m for the 5 m sim bullseye in the 640 x 360 camera). Below that the centre comes from the ring fit (CEN-13). The sim line was measured on a 1.5 m bullseye.
- **Position resets are found from a step in the pose stream** (ArduPilot reports none through MAVROS) and have run only in unit tests. SIM-04's GPS glitches will exercise them in a flight.
- **Not flyable on the aircraft yet**, for FND-05's reasons (the airframe's mount is not measured, the aircraft sequences name no camera) and one more: the detector must publish on the contract (`features.detection_contract`) in the pixels of the calibrated mode. The 2026 detector's image size is checked; another detector's is not, and a detection's `frame_id` is not checked against the mount.
- **The run-time cost on the Jetson is not measured.** The work per step is bounded (16 detections a frame, 4 frames a step) but the 20 Hz tick's jitter with the estimator on has not been timed on the flight computer; PLT-05's bench timing is the place.
- **The 0.08 s actuation delay is the spec's figure**, to be replaced by PLT-05's measurement.
- One flight, one seed, calm. Wind (SIM-03) and a moving vehicle (the survey) are still to come.
