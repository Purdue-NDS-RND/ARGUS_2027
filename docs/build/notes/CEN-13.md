# CEN-13: Near-field bullseye centre finder

Flags: `features.ring_fit` (node flag on the new `ring_fit` node, default false) and `use_ring_fit` (mission flag, default false; needs `use_target_estimator`) · Status: PARTIAL (the bench check with the printed target is a person step) · Commits: `git log --oneline --grep '^CEN-13:' platform/build-2027`

## What it solves

Near the ground the bullseye is larger than the picture, and a box around its red pixels is a box around the part in view. The estimator rightly refuses such a box, so the track coasts and is lost just where the drop needs it most. The rings' edges still say where the centre is; this fits them.

## Architecture

- `drone_target_cv/ring_fit.py` (new, no ROS): `fit_rings` (red mask → edge points and their normals → the point most normals pass through → rings by distance → the centre that makes them roundest), `nadir_homography` (levels a tilted camera's view), `to_detection` (a `bullseye` under the contract).
- `drone_target_cv/ring_fit_node.py` (new node `ring_fit`): images with a queue of one, MAVROS's pose and `camera_info` in; `/perception/ring_fit/detections` out. Started by the demo launches only with `ring_fit:=true`.
- `estimator_feed.py`: with `use_ring_fit` on the estimator also listens to the ring fit, and weighs its detections by the ring fit's own 2 px (`DETECTOR_PIXEL_SIGMA_PX`), not by their box.
- `tools/ring_fit_try.py` (new): the fit on a saved frame, for the bench.
- Sim: scenario `drop/hold_2m_ring_fit` (nightly); the scenario builder passes `ring_fit:=true` to the launch when the node's flag is on.

## Flow

Before: at 2 m in wind, target_cv's box touches the picture's edge → the estimator skips it → the track is COASTING, then LOST → the mission holds, then climbs to look again.

After (both flags on): the same frame → `ring_fit` levels its edge points with the vehicle's attitude at the frame's time, fits the rings, publishes the centre with the innermost ring as its box → the estimator takes it as a measurement of the same bullseye track → the track stays CONFIRMED.

## Code

- `ring_fit.py`, `fit_rings`: the fit; declines unless two rings or more share a centre inside the picture.
- `ring_fit.py`, `nadir_homography`: K, the camera's attitude → the levelling homography.
- `ring_fit_node.py`, `_process_frame`: a frame is looked at only with a pose within 0.1 s of its stamp, below 4 m, in the calibrated mode's size; it is levelled, fitted and published, and the node's counts go to the log every ten seconds.
- `target_estimator.py`, `hit`: a frame counts once toward confirming a track however many detectors saw it.
- `estimator_feed.py`, `add_detections` and `_handle`: a detector's own pixel noise carried with its frame; such a detection is not put through the cut-box rule.
- `sim/scenario/build.py`, `FLAG_STARTED_NODES`: the launch argument for a node that exists only for its flag.

## Evidence

```
$ python3 -m pytest -q src/drone_target_cv/test/test_ring_fit.py src/drone_target_cv/test/test_ring_fit_node.py
33 passed in 8.47s
$ python3 -m pytest -q src/drone_mission_core/test/test_estimator_feed.py src/drone_mission_core/test/test_executor_target_estimator.py
28 passed in 0.54s
$ python3 -m pytest -q tools/test/test_ring_fit_try.py
4 passed in 2.52s
$ sim/run_scenarios.sh drop/hold_2m_ring_fit --seeds 1 --variants on
| drop/hold_2m_ring_fit | 1 | not run |  |  | OK | 17.37 | 0.10 m from the target |
estimator against the truth at 2.4 m: PASS, median 0.02 m, p95 0.04 m over 514 samples (limit 0.08 m)
Ring fit: 4413 frames so far, looked at 1391, rings found in 1100; not looked at: no pose of the frame's moment 3, above the height limit 3018
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh
OK
```

Rendered views: 60 of 60 found, median under 0.6 px, worst under 3 px. The 2 m hold in strong wind on the 1.5 m target: 0.200 m at p95 and no lost track, against 0.31 m and 30 to 40 losses without the ring fit; rings found in 1100 of the 1391 frames looked at. `docs/results/2026-11-20-ring-fit.md`.

## Spec check

spec-auditor, two rounds. Round one found that the estimator's cut-box rule also dropped ring-fit centres near the picture's edge, the case the feature is for (fixed: a detection with its own noise model is not put through that rule), that unlevelled fits were published at the same 2 px (fixed: no pose of the frame's moment, no fit), that the Gazebo flight showed the ring fit only indirectly (the node now logs its counts, quoted in the results), and that the bench procedure fitted tilted frames unlevelled (corrected, with a timing command). Round two: all closed but the bench itself, a person step, so the row is PARTIAL. "Sim renders" is read as `tools/synth_targets.py` views (Decisions).

flight-guardian, two rounds, CONCERNS, no blocker. Round one: two detectors on one frame confirmed a track in two frames (fixed: a frame counts once); the ring fit's pixels were never checked against the calibrated size (fixed: the node loads the calibrated camera and refuses other sizes); the height gate used a pose of any age (fixed); a detectors parameter could bring the ring fit in with its flag off (fixed). Round two: one residual, recorded: the node's `camera_mode` and `airframe` are its own parameters and nothing compares them with the missions'.

## Not done

- Bench with the printed target, and the fit's speed on the Jetson at 1280 x 720 (Needs a person).
- The aircraft's launch and params do not have the node yet (PLT-07); its camera parameters must then match the sequence's, which nothing checks.
- The sim scenario is in no gate and passes without the ring fit: its evidence is read by a person.
- The height gate is height above home; the estimator takes both sources at any height the node publishes.
