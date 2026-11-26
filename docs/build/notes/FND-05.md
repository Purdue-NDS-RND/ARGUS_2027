# FND-05: Projection v2 (pixel to ground)

Flags: `use_frame_time_pose` (mission flag, default false, already declared in all ten sequences) for the drop and delivery missions; `features.projection_v2` (node flag on `mission_logger`, default false) · Status: DONE · Commits: `git log --oneline --grep '^FND-05:' platform/build-2027`

## What it solves

In 2026 a detection was projected with whatever pose was newest when it arrived, a camera matrix scaled from the 4K calibration, no lens distortion and the camera assumed to be at the flight controller; the mission logger's raycast sent every ray upward and logged nothing. This is the cause-level fix for the geometry: one projection that uses the pose at the frame's capture time, the sensor mode's own calibration, the camera's real mount, and says how uncertain its answer is.

## Architecture

- `drone_mission_core/projection_v2.py` (new, numpy only): `CameraModel`, `CameraMount`, `project`, `GroundProjector.project_pixel_to_ground(pixel, stamp_ns, plane_height_m)`, `GroundHeight`, `GroundPoint` (the point and its three Jacobians). `tracking_projection.py` is not touched.
- `drone_mission_core/frame_projection.py` (new): `load_camera(mode, airframe)` from `argus_camera`'s config, and `TargetOffset`, the error a mission steers on.
- Executor, flag on: one more subscription, `/mavros/rangefinder/rangefinder`, feeding a `GroundHeight`. `MissionContext.pose_at(stamp_ns)` and `ground_projector(camera, mount)` (None with the flag off).
- New mission config keys, read only with the flag on: `camera_mode` and `airframe` (set in the two sim sequences) and `frame_time_max_age_s` (default 0.5).
- `vision_pipeline/logger_projection.py` (new) and three parameters on `mission_logger`: `features.projection_v2`, `camera_mode`, `airframe`. With the flag on the logger only projects frames whose `camera_info` is that mode's own K and D.
- Sim: `sim/projection_check.py` beside every flight with the flag on, the scenario `delivery/frame_time`, a new run status FAIL.
- New dependencies: `drone_mission_core` on `argus_camera` and numpy, `vision_pipeline` on `drone_mission_core`; all imported only with a flag on.

## Flow

Before, and after with the flag off: detection point → newest pose, scaled K → offset from the vehicle → velocity.

After, flag on: detection point and its frame's stamp → pixel in the mode's image → undistorted ray → through the mount → through the body's pose at that stamp (pose history) → from the lens, down to the ground (rangefinder's ground if healthy, home's height otherwise) → the target's point → minus the vehicle's position now → the same deadband, low-pass and velocity as 2026. No pose for that stamp, or a frame over 0.5 s old: no tracking solution.

## Code

- `projection_v2.py:216`, `project`: the ray, the lever arm, the plane, and the Jacobians by pixel, attitude and height.
- `projection_v2.py:127`, `CameraModel.undistort`: Newton's method on the plumb-bob model's own derivative; it also gives the pixel Jacobian exactly.
- `projection_v2.py:280`, `GroundHeight`: the ground's own height from rangefinder readings and the pose at each reading's moment, so climbing between reading and frame does not move the ground. The median of up to five, not believed with fewer than three, when they disagree by 0.5 m, or more than 5 m from home's height.
- `frame_projection.py:33`, `load_camera`: refuses a stand-in calibration, an unverified airframe, or a mode whose camera the airframe does not carry.
- `red_bullseye_mission_base.py:70`, `:301` and `:362`: the camera is loaded when the mission is built (the executor then does not start on a bad one); the one branch in `_get_tracking_solution`; `_frame_time_projection`, with its frame-age limit.
- `mission_logger.py:60` and `:580`, `logger_projection.py:49`: the logger's flag, `_project_v2_to_gps`, and `camera_info_problem`.

## Evidence

```
$ PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_projection_v2.py
28 passed in 0.20s
$ PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_frame_projection.py
18 passed in 0.40s
$ python3 -m pytest -q -p no:cacheprovider src/drone_mission_demo/test/test_frame_time_missions.py
16 passed in 0.30s
$ PYTHONPATH=src/vision_pipeline python3 -m pytest -q -p no:cacheprovider src/vision_pipeline/test
12 passed, 1 warning in 0.28s
$ python3 -m pytest -q -p no:cacheprovider sim/test/test_projection_check.py
10 passed in 0.27s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

The spec's three unit tests: `test_the_nadir_pixel_lands_directly_below_the_camera_after_the_lever_arm`, `test_synthetic_ground_points_round_trip_within_a_centimetre` (worst case under a micrometre, with the real distortion numbers) and `test_the_jacobian_matches_a_numerical_derivative`. The mission test was broken once on purpose (the pose 150 ms later than the frame's): the two flag-on steering tests failed, and it was put back.

Sim, flags on, three flights of `sim/run_scenarios.sh delivery/frame_time --variants on`, the third on the committed code ([docs/results/2026-11-18-projection-v2.md](../../results/2026-11-18-projection-v2.md)):

```
| delivery/frame_time | 1 | not run |  |  | OK | 16.18 | 0.05 m from the target |
| Projected target against the truth at 15 m | PASS: median 0.09 m, p95 0.25 m over 287 detections (limit 0.3 m); with the latest pose instead, p95 0.24 m |
```

0.22, 0.22 and 0.25 m at the 95th percentile against the 0.3 m line. The largest errors (2.3 m) are five frames at the top edge of the image as the bullseye comes into view: the detector's centre of a half-seen target, not the projection.

## Spec check

spec-auditor, one round: INCOMPLETE on two rows. Fixed after it, not reviewed again: the logger refuses frames that are not the mode's raw frames (it would otherwise have undone the lens twice on the 2026 grabber's frames), a test that drives the real logger node with its flag on, the flag declared in the second params file, the second and third flights recorded and the outlier explained.

flight-guardian, one round: CONCERNS, no blocker. Fixed after it, not reviewed again: the 0.5 s frame-age limit (a stopped detector could otherwise have let the mission centre on a remembered point), and the rangefinder's ground as a checked median instead of one reading. Left standing:

- **The mission cannot tell whether a detection's stamp is its frame's.** That needs the detector's `features.stamp_from_image` on and one clock; nothing ties the two flags together until PLT-07's profiles do. Wrong pairings fall back to about the 2026 behaviour or to holding.
- **A blocked rangefinder beam that reads plausibly** (within 5 m of home's height and steady) is believed. MAVROS's topic carries no health field and always says 0 to 1000 m.
- **A refused tick resets the mission's low-pass filter** (as any lost-target tick did in 2026), so a stream of frames near the age limit would alternate between holding and unfiltered steering. Not seen in the sim: no refusals in three flights.

What differs from the spec, each recorded under Decisions: `project_pixel_to_ground` is a method with an optional plane height; "healthy" for the rangefinder is defined here; the sim line is judged at the 95th percentile.

## Not done

- **Not flyable on the aircraft yet.** The four aircraft sequences name no camera, `argus_2027` is `verified: false`, and the detector's images must come from `argus_camera` in a calibrated mode with `features.stamp_from_image` on. With the flag on and any of that missing the executor does not start, or the mission gets no tracking solution. The person steps are under "Needs a person".
- **The frame-time pose showed no gain in the sim flight**, because the 2026 delivery hovers while it looks. Unit tests show the gain on a moving vehicle; the survey missions will show it in the sim.
- **Nothing uses the Jacobian yet.** FND-06's estimator turns it into measurement noise.
- **The logger's flag has not run in ROS**: it needs the YOLO node, which needs a model (training-data hold). Its logic is unit-tested without ROS.
- **The rangefinder is taken to be at the flight controller**, and a failed sensor that still reports a plausible number is not detected.

## After the build (2026-11-25): the fallback's imports

- **What was wrong.** The June 2026 files imported 2027 modules when they were loaded: `red_bullseye_mission_base.py` six of them, the drop and delivery missions three, `mission_context.py` two, `mission_executor.py` fourteen, and `missions/__init__.py` the four 2027 missions. A 2027 module that could not be imported on the Jetson would have stopped the executor, and every 2026 mission with it, with every flag off.
- **Now.** With every flag off, none of those files imports a 2027 module. A flag's modules are imported where that flag is on and only while the sequence is built, on the ground: in a mission's constructor, and in the executor's `_load_sequence` and `_attach_2027`, which run when any flag is on in the sequence. `mission_context.py` reaches `target_access.py` through `_target_access()`, from methods only flag-on code calls. The package offers each 2027 mission through `registry.import_optional_mission`: one that cannot be imported is recorded in `registry.IMPORT_FAILURES`, said by the executor when it starts, and a sequence that names it does not start.
- **Flag names.** A 2026 mission cannot take a flag's name from the flag's module without importing it, so it carries the name (`CENTRE_FLAG = "centre_on_estimator"` and seven more); a test holds each to its module's constant.
- **Code.** The executor's rule: `mission_executor.py`, `any_flag_on` and `_attach_2027`. The base's constructor: `red_bullseye_mission_base.py`, `__init__` (the time guard's class is kept there and the guard made in `_initialize_common_vision_state`). The optional missions: `missions/__init__.py`, `registry.py` `import_optional_mission`.
- **Evidence.** `src/drone_mission_demo/test/test_fallback_imports.py` (17 tests, twelve of them in a fresh interpreter where every 2027 module of the two packages, `numpy`, `cv2` and the `argus_` packages refuse to import): the tag's own two mission test files pass as they are; every committed sequence of 2026 mission types is built; the 2026 delivery is flown from transit to relaunch at 20 Hz with the gripper commanded once; the executor starts and ticks three 2026 sequences with nothing attached; with a flag on, the executor and the mission refuse to be built. 13 of the first 15 failed before the change. Gate G0 was run after it (`docs/results/`).
- **Review.** flight-guardian, one round: CONCERNS, no blocker, two rows, both acted on. A failed optional import now takes back whatever the module had registered before it failed (a type left registered on a half-loaded module would have let a sequence start and fail in the air; not reachable in the four 2027 missions as written, whose class is their last statement), with a test. And the test that keeps a 2026 mission's 2027 imports in its constructor no longer lets them into `_load_...` helpers, which are called from `on_enter`. It checked the flags-off equivalence of the executor's gate, the time guard and the context's lazy imports and found nothing; it could not read the `comp-2026` tree itself or run anything. No spec-auditor round: the spec has no line for this.
