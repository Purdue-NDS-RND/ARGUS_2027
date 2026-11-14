# ARGUS 2027: Feature Specification

Working copy of the ARGUS 2027 Full Feature Brief (12 November 2026) for the build session. `docs/build/SESSION.md` says how the session uses it and `docs/build/features.json` tracks the status of every feature. If this file and the lead disagree, the lead wins.

## Summary

This brief lists every feature ARGUS needs to go from the June 2026 code to a competition-ready system for C-UASC 2027 (4 to 6 June 2027, Apollo XI Field, Encino), written so that each feature can be built and checked on its own. It is the build session's work list; how the session runs is in `docs/build/SESSION.md`.

**What "done" means.** If the competition were next week: every P0 feature merged behind its flag; sim gates G0 to G8 green on main; the flags-on profile flown on the aircraft through a GO from the preflight check; and the flags-off profile still flying exactly as `comp-2026`, as the fallback.

### Scope

| Area | Features | P0 | P1 | P2 |
| --- | --- | --- | --- | --- |
| [Foundations (FND)](#foundations-fnd) | 9 | 9 | 0 | 0 |
| [Drop and delivery (CEN)](#drop-and-delivery-cen) | 14 | 10 | 1 | 3 |
| [Localisation (LOC)](#localisation-loc) | 14 | 11 | 3 | 0 |
| [Recovery (REC)](#recovery-rec) | 10 | 7 | 3 | 0 |
| [Platform (PLT)](#platform-and-flight-operations-plt) | 14 | 12 | 2 | 0 |
| [Data and models (DAT)](#data-training-and-models-dat) | 10 | 7 | 2 | 1 |
| [Simulation (SIM)](#simulation-and-test-infrastructure-sim) | 12 | 11 | 0 | 1 |
| [Rules and operations (OPS)](#rules-compliance-and-competition-day-operations-ops) | 9 | 7 | 2 | 0 |
| **Total** | **92** | **74** | **13** | **5** |

P0 is needed to compete well; P1 adds points or margin; P2 is cut first. [Build order](#build-order-and-dependencies) puts them in six waves; [tasks for people, hardware and the field](#tasks-for-people-hardware-and-the-field) lists the hardware, field and people tasks; [open questions](#open-questions) lists 23 questions for the organisers, each with a default.

### Rules for every feature

- Behind a feature flag that defaults to false. All flags off flies exactly as `comp-2026`, and a test enforces it.
- Logic in ROS-free modules behind thin nodes, with tests registered in `scripts/check.sh`.
- Every PR passes `scripts/pr_checks.py`: a test with every code change, no changed values in `src/*/config` without a `[tuning: …]` waiver, no secrets, no local machine details.
- Every mission lane ends at a scored, unattended sim gate before any field time is spent on it.

### Decisions taken in this brief

"Earlier" means decided in previous planning and confirmed here; "New" means decided while writing this brief.

| # | Decision | Why | Where | Status |
| --- | --- | --- | --- | --- |
| 1 | Detect on the whole 4K frame shrunk to 1280 × 736; read digits from 4K crops; tiling only behind a flag and in the post-landing pass | Matches flight scale; 2026 tiling cost about 460 ms per frame | LOC-5, DAT-01 | Earlier |
| 2 | One target estimator, run inside the mission executor; missions read only the estimator | Fixes the 2026 time misalignment in one tested place | FND-06 | Earlier |
| 3 | Gate measurements at NIS 9.21 (χ², 99%), not 3 | A gate of 3 rejects about 22% of good measurements | FND-06, LOC-7 | New |
| 4 | The camera driver publishes raw frames; only points are undistorted | Removes the 2026 K and D mismatch and saves CPU at 4K | PLT-03, FND-05 | New |
| 5 | 4K frames stay in one process; drop and delivery use 720p at 60 fps | No 4K images over ROS topics | PLT-03, LOC-5 | New |
| 6 | One of each: frame logger, robot description, model registry, fault-injection tool, payload bridge | The area drafts had duplicates; three or four mount descriptions already disagree | PLT-03, DAT-04, SIM-04, SIM-06 | New |
| 7 | New `argus_interfaces` package for marker tracks and timing messages | Custom messages need an ament\_cmake package | LOC-7 | New |
| 8 | Survey at 35 ft by default (30 ft if height is judged from our log); 25 ft floor as a code constant | Keeps clear of the −3 and zero-score bands | LOC-1, FND-09 | New |
| 9 | Buy a TFmini Plus or TFmini-S rangefinder, the only new purchase; RTK only if the tolerance demands it | Release height and recovery need true height above ground | CEN-03 | Earlier |
| 10 | Drop: camera to release at 2.4 m (3.0 m on baro) with gates; after two timeouts, release if the error is ≤ 0.5 m | An off-centre drop scores; no drop scores nothing | CEN-04 | New |
| 11 | Ring-fit bullseye centre finder below 4 m | At 2.4 m the target fills the frame and a box centre is poor | CEN-13 | New |
| 12 | Delivery descends on the estimator to touchdown at 0.12 m/s; no GPS column | 2026 spent 80 s on a GPS column from 11 m | CEN-05 | New |
| 13 | Recovery without trusted GPS sends `OPTICAL_FLOW` (external navigation as the alternative) and switches EKF sources from the companion, never by RC | Speed-only input cannot hold position in ArduPilot 4.6.3; an RC switch is human input and voids the bonus | REC-5, REC-6 | New, replaces the speed-only plan |
| 14 | Recovery never releases on a carrying fault: it lands with the bucket; its abort mode is LAND | The rules allow landing with the bucket; a refused RTL is retried forever today | REC-2, REC-7 | New |
| 15 | Time trial as an AUTO mission with GUIDED fallback; route optimiser and flying start behind flags | Both depend on organiser rulings | PLT-13 | New |
| 16 | Flag profiles are enable-lists outside `src/*/config` | Keeps the tuning gate intact | FND-08, PLT-07 | Earlier |
| 17 | Dataset class names equal the detection contract names | One vocabulary from labels to judges | DAT-01, FND-02 | New |
| 18 | The digit reader may abstain: ≥ 99% precision at ≥ 90% coverage; abstained markers are re-visited | A wrong ID costs points; a re-visit costs seconds | LOC-6, DAT-08, LOC-4 | New |
| 19 | One CUDA process per flight; the digit CNN runs on CPU | 8 GB shared memory on the Orin Nano | PLT-04 | Earlier |
| 20 | Wind compensation is the first P2 cut | The estimator and retune carry most of the gain | CEN-07 | New |
| 21 | Field ground truth by stakes and tape measure, not RTK | No unnecessary hardware | DAT-10 | New |
| 22 | Drop gate G2 uses the A/B numbers: median ≤ 0.15 m calm, ≤ 0.30 m strong | One standard for "better than 2026" | SIM-11, CEN-12 | New |
| 23 | Slot plan: localisation and drop, then time trial and delivery, then recovery | Best expected score under the current rules | OPS-04 | Earlier, pending organiser question 1 |
| 24 | Code freeze around 19 May 2027, tag `comp-2027-rc1`, 2026 fallback workspace kept ready | Field changes only through tuning files | OPS-06 | New |
| 25 | The session deletes no 2026 code; retiring it needs the lead's approval | The 2026 path is the fallback | LOC-14 | New |
| 26 | The build session holds off on everything that needs training data (the DAT lane); runtime code is built against stand-ins: a sim-only oracle detector, the classical detectors, and a template-matching digit classifier | No 2027 training data exists yet; nothing downstream of the model has to wait for it | DAT, LOC-5, LOC-6 | New (lead's call, 12 Nov) |

## Architecture

Every detector reports into one target estimator; every mission reads only the estimator; the 2026 mission framework, MAVROS and ArduPilot carry the commands out unchanged. Localisation runs a second branch (digit reader and marker tracker) that produces the judges' list instead of steering.

```
IMX519 camera driver: 4K for markers, 720p for the bullseye, capture time on every frame
   |                         |
   v                         v
Colour detectors          YOLO26 detector (5 classes, 1280 input, TensorRT FP16)
(bullseye kept, green X,     |                      |
 ring fit; flags-off path)   |                      v
   |                         |               Digit reader (4K crop, flattened, 0-9 or unsure)
   v                         v                      |
Target estimator (Kalman per target: pose at       v
capture time, NIS gate, forecast to actuation)   Marker tracker (match, vote, median) -> judges' list
   |
   v
Mission executor, 20 Hz (kept): drop, delivery, recovery, search pattern, waypoint, time trial
   |
   v
MAVROS (kept): setpoints out; pose, IMU, rangefinder in   <---  optical flow from the OV9782
   ^                                                           (global shutter, up to 120 fps)
   v
ArduPilot: EKF3 (GPS, or flow when GPS fails), GUIDED, fence, RTL

Shared services used by everything: pose buffer (pose at any instant), health monitor,
preflight go/no-go, frame logger, post-landing pass, latency probe.
```

The shared services are used by everything: the pose buffer gives any node the drone's pose at a frame's capture time; the health monitor and preflight test decide go or no-go; the frame logger and post-landing pass serve localisation; the latency probe measures the whole chain. With every 2027 flag off, the colour detector, the June 2026 centring loop and the original sequences run exactly as flown.

## Foundations (FND)

These nine pieces are shared by every mission and must exist before any mission work starts. They fix the root cause of the 2026 drift: frames, detections and poses were never lined up in time, so the drone steered toward where the target was a third of a second ago.

Sizes used throughout: **S** = one module plus tests; **M** = several files or one node; **L** = a new package or a change that crosses packages. "Proven in" is the lowest level at which the acceptance test can pass: Unit (ROS-free tests in `scripts/check.sh`), Sim (Gazebo + SITL), Bench (Jetson on the table), Field.

| ID | Feature | Priority | Size | Depends on | Proven in |
| --- | --- | --- | --- | --- | --- |
| FND-01 | Capture timestamps on every frame | P0 | M | PLT-03 (PLT-05 later supplies the measured offset) | Sim, Bench |
| FND-02 | One detection message contract | P0 | S | FND-01 | Unit |
| FND-03 | Time base: MAVROS timesync, sim clock, stale-data guard | P0 | M | none | Sim |
| FND-04 | Pose history buffer | P0 | S | FND-03 | Unit |
| FND-05 | Projection v2 (pixel to ground) | P0 | M | FND-04, PLT-03 mount | Unit, Sim |
| FND-06 | Target estimator (Kalman per target) | P0 | L | FND-02, FND-05 | Unit, Sim |
| FND-07 | MissionContext additions | P0 | S | FND-04, FND-06 | Unit |
| FND-08 | Feature-flag plumbing and profiles | P0 | S | none | Unit |
| FND-09 | Safety limits module | P0 | S | none | Unit |

### FND-01 Capture timestamps

- **Problem.** In 2026 the camera node stamped each frame when it published it, and the detector stamped results with `now()`. The real capture-to-result delay (40 to 120 ms, varying) was invisible.
- **Behaviour.** Every `sensor_msgs/Image` carries the sensor capture time in the ROS clock. A new `cv/capture_clock.py` converts the GStreamer buffer PTS plus pipeline base time into ROS time, then subtracts a fixed per-camera offset measured by the LED latency probe (PLT-05) and stored in the camera config. `nvarguscamerasrc` does not pass the sensor's own timestamp through, so PTS plus measured offset is the method for the IMX519; for the OV9782 (V4L2) the kernel buffer timestamp is used directly.
- **Every downstream message copies `header.stamp` from its source image.** No node may stamp vision output with `now()`.
- **Tests.** Unit: clock conversion with synthetic PTS. Sim: frame stamp vs Gazebo render time within one frame period. Bench: LED probe residual ≤ 5 ms after offset correction.

### FND-02 Detection contract

- Every detector publishes `vision_msgs/Detection2DArray` on `/perception/<detector>/detections`; `header.stamp` = image capture stamp, `header.frame_id` = the camera's optical frame.
- Class ids are strings from one list: `bullseye`, `green_x`, `marker`, `boundary_marker`, `bucket`. Score is the detector confidence. Box centre and size are in pixels of the full-resolution calibrated image for that sensor mode (a detector that ran on a shrunk frame scales back up before publishing).
- The legacy `/drone_package_drop/target_detection` topic stays alive through a small adapter so the flags-off path is untouched.
- **Tests.** Unit: round-trip encode and decode; scaling from 1280 to 4K pixel space.

### FND-03 Time base

- **MAVROS timesync.** Set the `sys_time` plugin's `timesync_rate` to 10 Hz in a new `config/mavros/argus_mavros.yaml` (the MAVROS 2 default is 0, which leaves pose stamps at receive time). PLT-06 owns this file; FND-03 owns the requirement.
- **Sim clock.** Bridge `/clock` from Gazebo and set `use_sim_time:=true` on every node in the sim launch; today the sim mixes wall time and sim time, which would hide timing bugs.
- **Stale-data guard.** `core/time_health.py` watches timesync offset and round-trip time, pose age and frame age. It reports UNHEALTHY when pose age exceeds 100 ms, frame age exceeds 250 ms, or the timesync offset jumps by more than 10 ms. Missions treat UNHEALTHY as "no target" (hold, never descend).
- **Tests.** Unit: health logic. Sim: pose stamps agree with `/clock` within 20 ms.

### FND-04 Pose history buffer

- `core/pose_buffer.py` keeps 5 s of `/mavros/local_position/pose` and velocity.
- `pose_at(t)` interpolates position linearly and orientation by SLERP between the two bracketing samples. It refuses (returns None) if the bracketing gap exceeds 0.12 s, and extrapolates at most 30 ms past the newest sample using velocity. It never falls back to the latest pose.
- **Tests.** Unit: synthetic 1 m/s and 30 °/s trajectories reconstructed to ≤ 1 cm and ≤ 0.2°; gap and extrapolation refusals.

### FND-05 Projection v2

- Adds `projection_v2.py` beside `tracking_projection.py`, which stays unchanged for the flags-off path, built around three objects: `CameraModel` (K and distortion D for each sensor mode, calibrated per mode, not scaled from the 4K calibration), `CameraMount` (lever arm and rotation from the single robot description owned by PLT-03), and `project_pixel_to_ground(pixel, stamp)`.
- Steps: undistort the pixel, form the ray in the camera frame, rotate through the mount and the body pose at the capture time (FND-04), intersect with the ground plane at the known ground height (rangefinder AGL when healthy, otherwise home altitude), return the ENU point and the Jacobian of that point with respect to pixel, attitude and height errors.
- With its flag on, it replaces the `mission_logger` raycast, which in 2026 sent every ray upward, and the latest-pose projection in the drop and delivery paths.
- **Tests.** Unit: nadir pixel lands directly below the camera after the lever arm; synthetic points round-trip ≤ 1 cm; Jacobian matches a numerical derivative. Sim: projected bullseye vs truth ≤ 0.3 m at 15 m.

### FND-06 Target estimator

- `core/target_estimator.py`. Each target is a static ground point (East, North) with its own two-state Kalman filter. Process noise q = 0.0025 m²/s absorbs slow drift in our own position error. Measurement noise R = J Σ Jᵀ from FND-05, where Σ holds pixel noise (about 2 px σ, growing for small boxes), attitude noise (about 1°) and height noise (0.05 m on rangefinder, 0.5 m on baro).
- **Gate.** A measurement is accepted when its normalised innovation squared is ≤ 9.21 (χ², 2 degrees of freedom, 99%). A covariance floor of (0.02 m)² stops the filter becoming overconfident. Five consecutive gated-out measurements reset the track to the new position (the target was really there).
- **Track life.** TENTATIVE → CONFIRMED after 3 hits within 1 s → COASTING after 0.3 s without a hit → LOST after 2 s; lost tracks are kept 30 s for re-acquisition. At most 8 tracks per class. Several detections in one frame are assigned to tracks by the Hungarian method on Mahalanobis distance (the same code LOC-7 uses).
- **Forecast.** Outputs are given for the actuation time, now + 0.08 s (the command delay measured by PLT-05). For a static target this adds no motion, only the matching growth in covariance; the drone's own predicted position at that time comes from the pose buffer.
- **Where it runs.** In-process inside the mission executor, so missions read it without a ROS hop. It also publishes `vision_msgs/Detection3DArray` on `/estimator/targets` for logs and the ground station.
- **Acceptance (sim, calm).** p95 horizontal error ≤ 0.30 m at 15 m, ≤ 0.15 m at 6 m, ≤ 0.08 m at 2.4 m.
- **Plane height per class.** Each class projects onto its own height above ground (bullseye and X at 0, bucket rim about 0.2 m, handle about 0.3 m), passed to FND-05.
- **EKF resets.** When the flight controller reports a position reset (a GPS glitch, or switching navigation sources), every track is shifted by the same reset so the gate does not reject every detection that follows.

### FND-07 MissionContext additions

- New calls: `estimator_ready()`, `primary_target(cls)` returning position, covariance, age and track state, `pose_at(t)`, `height_agl()` returning value and source (rangefinder or baro), `actuation_time_ns()`, `latlon_to_local_enu(lat, lon)`.
- Rules enforced here, not in each mission: descending needs a CONFIRMED track no older than 0.5 s; releasing needs a track no older than 0.3 s.
- **Tests.** Unit, with a fake context.

### FND-08 Feature-flag plumbing

- `FLAG_REQUIRES` lists dependencies between flags (for example, release gates require the estimator); the executor refuses to start when a dependency is missing.
- A `feature_overrides` launch parameter turns flags on without editing `src/*/config` YAML, which keeps the CI rule intact. Flag profiles are enable-lists in `profiles/*.yaml` outside `src` (PLT-07 loads them).
- A check scans every `src/*/config` file, and an AST test confirms every `get_feature_flag` default is False, so all flags off still equals the `comp-2026` behaviour.

### SETUP-1 Shared messages (`argus_interfaces`)

- A new ament_cmake package, `argus_interfaces`, so later features have custom messages to build on. A build test (or a parse test of the `.msg`/`.srv` files where ROS isn't available) proves it.
- `msg/MarkerTrack.msg`: `std_msgs/Header header`, `uint32 track_id`, `uint8 state` (constants TENTATIVE=0, CONFIRMED=1, COASTING=2, LOST=3), `geometry_msgs/Point position_enu`, `float64[4] covariance_en` (2 × 2, row-major), `float32[10] digit_log_prob`, `int8 digit` (−1 when unknown), `uint32 sightings`, `bool outside`.
- `msg/MarkerTrackArray.msg`: `std_msgs/Header header`, `MarkerTrack[] tracks`.
- `srv/GetCrops.srv`: request `builtin_interfaces/Time stamp`, `sensor_msgs/RegionOfInterest[] rois`; response `sensor_msgs/Image[] crops`, `bool ok`, `string message`.
- `msg/StageTiming.msg`: `std_msgs/Header header`, `string stage`, `float32 latency_ms`.
- Later features (LOC-5, LOC-7, PLT-05) may add fields; that is recorded under "Decisions", not a stop.

### FND-09 Safety limits

- `core/safety_limits.py` holds the rule-derived constants every mission imports: release height ≥ 1.83 m (6 ft), survey height ≥ 7.62 m (25 ft), our own release gate floor 2.0 m, phase speed caps, geofence margin. Values here are constants, not config, so they cannot be tuned away on the field.
- The flight-guardian checks F1 to F10 reference this module instead of repeating numbers.
- **Tests.** Unit: every mission's configured heights are checked against these limits at load time.

## Drop and delivery (CEN)

In 2026 the live drop stopped trusting the camera at about 9 m and the live delivery at about 11 m; delivery then took 80 s to descend on a GPS column. In 2027 the camera, through the estimator, steers all the way to release or touchdown, with a rangefinder for height and explicit gates that must hold before the payload leaves.

| ID | Feature | Priority | Size | Depends on | Proven in |
| --- | --- | --- | --- | --- | --- |
| CEN-01 | Centring reads the estimator, aims the payload | P0 | M | FND-06, FND-07 | Sim |
| CEN-02 | Controller retune and offline loop simulator | P0 | M | CEN-01 | Unit, Sim |
| CEN-03 | Rangefinder and height estimator | P0 | M | hardware buy | Sim, Field |
| CEN-04 | Drop: camera to release with gates | P0 | L | CEN-01..03, FND-09 | Sim |
| CEN-05 | Delivery: soft set-down on the estimator | P0 | M | CEN-01..03 | Sim |
| CEN-06 | Smart re-acquisition | P0 | S | FND-06 | Sim |
| CEN-10 | 2027 sequences for drop and delivery | P0 | S | CEN-04, CEN-05 | Sim |
| CEN-11 | Sim copies of the live configs | P0 | S | none | Sim |
| CEN-12 | A/B harness (flags off vs on) | P0 | M | SIM-03, SIM-06, SIM-07 | Sim |
| CEN-13 | Near-field bullseye centre finder | P0 | M | FND-02 | Sim, Bench |
| CEN-14 | Release confirmation | P1 | S | CEN-04 | Sim, Field |
| CEN-07 | Wind compensation (PI plus disturbance observer) | P2 | M | CEN-02 | Sim |
| CEN-08 | Release offset model | P2 | S | CEN-04, OPS-07 | Sim, Field |
| CEN-09 | ArduPilot precision-loiter bridge (benchmark only) | P2 | S | FND-06 | Sim |

P2 items are the first cut if time runs short; CEN-07 goes first.

### CEN-01 Centring on the estimator

- `TRACK_AND_DESCEND` in `package_drop_mission.py` (and the delivery equivalent) takes its error from `ctx.primary_target('bullseye')` instead of the latest raw detection.
- The aim point is the payload, not the camera: setpoint = target − R(yaw) · payload offset, where the offset comes from the robot description (PLT-03).
- The 2026 EMA filter (α 0.12, about 0.37 s of lag) and hard deadband (0.4 m) are bypassed when the flag is on; the estimator already filters.

### CEN-02 Controller retune

- Proportional velocity control with a soft deadband of 0.03 m, gain Kp 0.6, speed cap clamp(0.08 · h, 0.25, 1.0) m/s where h is height above ground, and a slew limit of 0.5 m/s².
- `tools/cen_loop_sim.py` simulates the loop offline (estimator noise, one executor tick of delay, 20 Hz) so gains are chosen before Gazebo and checked in Gazebo after.
- Gains live in the 2027 sequence files, not in the 2026 ones, so no `[tuning: …]` waiver is needed for the change.

### CEN-03 Rangefinder and height estimator

- Buy a downward Benewake TFmini Plus or TFmini-S (cheap, 0.1 to 12 m). ArduPilot: `RNGFND1_TYPE 20`, `RNGFND1_ORIENT 25` (down); SITL uses type 100.
- `core/height_estimator.py` gives height above ground: rangefinder when in range, signal is good and tilt is under 15° (tilt-corrected); otherwise baro height above home plus the known ground offset. It always reports which source it used, and FND-07's `height_agl()` passes that on.
- Release height depends on the source (CEN-04), so the source matters as much as the number.

### CEN-04 Drop: camera to release

- New `FINAL_ALIGN` state replaces `FINAL_FIXED_DROP_COLUMN` when the flag is on. The drone descends on the estimator to the release height: 2.4 m ± 0.2 on the rangefinder, 3.0 m on baro (larger margin above the 1.83 m rule floor).
- **Release gates, all held together for 0.5 s:** horizontal error ≤ 0.10 m, estimator σ ≤ 0.08 m, horizontal speed ≤ 0.15 m/s, tilt ≤ 6°, track age ≤ 0.3 s, height inside the band.
- **Floor.** Below 2.0 m the drone climbs back into the band before anything else; FND-09 makes 1.83 m unreachable by command.
- **Timeout.** 25 s in `FINAL_ALIGN` without the gates holding triggers one re-acquisition (CEN-06). After a second timeout the drone releases at the current estimate if the error is ≤ 0.5 m, otherwise it moves on to the next mission. (Decision made in this brief: a release slightly off-centre scores; no release scores nothing.)
- **Tests.** Sim: gates, floor climb, timeout path, track loss at 3 m. The CEN-12 harness decides whether the flag can be on for the competition.

### CEN-05 Delivery soft set-down

- Same estimator-based descent as the drop, continued to the ground: below 0.6 m the drone holds horizontal position (no corrections that would drag the package) and descends at 0.12 m/s.
- **Contact detection:** rangefinder at or below the package height, vertical speed near zero, and throttle output dropping, held for 0.3 s. Then release, wait 1 s, climb.
- The impulse limit is still TBD in the rules; OPS-07's rig measures our contact impulse. Heavier packages earn +0.25 per 500 g, so the package weight is a team decision once the impulse limit is known.

### CEN-06 Smart re-acquisition

- When the track is LOST, the drone climbs to the height at which the camera footprint covers the 3σ uncertainty plus a 2.5 m margin: h = (3σ + 2.5) / 0.463, where 0.463 is half the footprint width per metre of height for the drop camera mode. It holds there until the track is CONFIRMED again, for at most 10 s, then follows the mission's failure policy.

### CEN-10 and CEN-11 Sequences and sim configs

- New `package_drop_2027.yaml` and `package_delivery_2027.yaml` sequences listing the 2027 flags (all false) and their gains; the 2026 live files stay untouched.
- Sim copies of the 2026 and 2027 live files (sim coordinates, `fake_drop: true`) so the exact live tuning can be flown in Gazebo.

### CEN-12 A/B harness

- `sim/ab/run_ab.py` flies the same seeded scenarios with flags off and on and reports landing or release error, time and failures.
- **Pass for turning the flag on:** drop median error ≤ 0.15 m in calm air and ≤ 0.30 m at 7 m/s wind, no floor violations, no more failures than flags off.

### CEN-13 Near-field bullseye centre finder

- At 2.4 m the drop camera sees only about 2.2 m of ground, so the bullseye fills the frame and a YOLO box stops being a good centre. Below 4 m, a classical ring fit (edges, then concentric circle or ellipse fit) gives the centre; it publishes on the FND-02 contract as `bullseye` with its own noise model, and the estimator takes both sources.
- **Tests.** Sim renders at 1.5 to 4 m; bench with the printed target.

### CEN-14 Release confirmation

- After the release command, check the gripper command was acknowledged and that the payload has left (downward camera frame difference, or the rangefinder seeing the payload fall away). The result goes in the attempt bundle (PLT-12) and the post-flight report.

### CEN-07, CEN-08, CEN-09 (P2)

- **CEN-07 Wind.** Add an integral term and a disturbance observer that estimates the steady wind push from commanded vs achieved velocity, so the drone stops sitting downwind of the target.
- **CEN-08 Release offset.** Learn the payload's horizontal drift from release height and wind (sim, then field drops) and shift the aim point by it.
- **CEN-09 Precision loiter.** Feed the estimator to ArduPilot's precision landing through `/mavros/landing_target/raw` (PLND parameters) and compare it with our own loop in the A/B harness. Benchmark only; it is not on the competition path unless it clearly wins.

## Localisation (LOC)

The drone flies a lawnmower pattern at or above 25 ft, finds every numbered marker, reads its digit and writes the judges' list with a latitude and longitude for each. In 2026 the list was typed by hand because the live raycast pointed every ray upward. In 2027 the list is produced on board, from many views of each marker, and is ready when the drone lands.

| ID | Feature | Priority | Size | Depends on | Proven in |
| --- | --- | --- | --- | --- | --- |
| LOC-1 | Survey height floor and guard | P0 | S | FND-09, CEN-03 | Unit, Sim |
| LOC-2 | Coverage planner | P0 | M | none | Unit |
| LOC-3 | `search_pattern` mission | P0 | M | LOC-1, LOC-2 | Sim |
| LOC-5 | `argus_localisation` package and `loc_node` | P0 | L | PLT-03, PLT-04, FND-02 | Sim, Bench |
| LOC-6 | Digit reader | P0 | M | LOC-5, DAT-08 | Unit, Bench |
| LOC-7 | Marker tracker | P0 | M | FND-05, FND-06 | Unit, Sim |
| LOC-8 | Weighted digit vote and global ID assignment | P0 | S | LOC-6, LOC-7 | Unit |
| LOC-9 | Final position by weighted median | P0 | S | LOC-7 | Unit, Sim |
| LOC-10 | Judges' list output | P0 | S | LOC-8, LOC-9 | Unit, Sim |
| LOC-11 | Frame logging policy | P0 | S | PLT-03 | Bench |
| LOC-13 | Surveyed-field validation | P0 | M | all above | Field |
| LOC-4 | Re-visit uncertain markers | P1 | S | LOC-3, LOC-8 | Sim |
| LOC-12 | Post-landing second pass | P1 | M | LOC-11 | Bench |
| LOC-14 | Retire 2026 localisation pieces | P1 | S | lead approval | none |

### LOC-1 Survey height

- Default survey height 35 ft (10.67 m). If the organisers confirm the height is judged from our flight log, drop to 30 ft (9.14 m) for a sharper view. The rule floor is 25 ft: below it costs 3 points, below 20 ft the mission scores zero.
- The guard watches `height_agl()` during the survey: below 8.22 m it climbs back (RECOVER); below 7.77 m it stops the survey and climbs before anything else (ABORT). 7.62 m (25 ft) is the FND-09 constant.

### LOC-2 Coverage planner

- Input: the search area polygon (organiser coordinates, refined by the four larger boundary markers once seen), survey height, speed, camera mode. Output: a lawnmower list of ENU waypoints.
- Track spacing = 0.8 × W − 0.85 m, where W is the cross-track footprint width and 0.85 m keeps a whole marker inside at least one pass. At 30 ft the 4K footprint is 15.0 × 8.45 m. Default speed 4 m/s, which still gives each marker about 30 frames at 15 Hz.
- Option: two-phase search (fast wide pass, then a tighter pass only where the first pass found tentative tracks).
- **Tests.** Unit: coverage of random polygons ≥ 99% of area seen by at least two frames.

### LOC-3 `search_pattern` mission

- `missions/search_pattern_mission.py` in the existing framework (`on_enter`/`update`/`on_exit`, `@register_mission`). Flies the planner's waypoints with `DO_CHANGE_SPEED`, holds a constant yaw so the footprint stays aligned with the tracks, and starts and stops the localisation pipeline through a service.
- Time budget 600 s by default; when it runs out the mission ends cleanly with whatever has been found.

### LOC-5 `argus_localisation` package

- New package with `loc_node`. The node embeds the camera's `FrameSource` (PLT-03), so 4K frames never cross a ROS topic.
- **Pipeline per frame:** shrink the whole 4K frame to 1280 × 736, run the YOLO26n TensorRT FP16 engine (static shape, confidence 0.25), scale boxes back to 4K, cut 4K crops for the digit reader, project to the ground (FND-05), update the tracker.
- Rate target 15 Hz, minimum 8 Hz; if the node falls behind it drops frames, never queues them.
- Publishes `/loc/detections` (FND-02 contract) and `/loc/status` (frames processed, rate, tracks, confirmed IDs). Tiling (SAHI) stays available behind a flag, off by default.
- Owns `shrink_for_detector()` and, while training data is on hold, the sim-only oracle detector described under "Data, training and models".

### LOC-6 Digit reader

- From each 4K crop: find the marker's corners, compute the homography in undistorted coordinates, warp to a 64 × 64 square, run a small CNN in ONNX Runtime on the CPU (keeps the GPU for detection) on all four rotations, since the marker's orientation is unknown.
- The 6 and 9 confusion is broken by the marker's own orientation cue if the rules' marker has one, otherwise by the rotation that best matches the X arms.
- **Pass:** ≥ 99% precision while answering ≥ 90% of crops on a held-out field day; markers the reader abstains on are re-visited (LOC-4). DAT-08 builds the set.

### LOC-7 Marker tracker

- Same projection and covariance as the estimator. Detections are assigned to tracks with the Hungarian method on Mahalanobis distance, gated at 9.21 and at 1 m absolute.
- TENTATIVE → CONFIRMED after 3 sightings. Tracks outside the hull of the four boundary markers are marked OUTSIDE and kept off the list.
- Messages: `MarkerTrack` and `MarkerTrackArray` in the new `argus_interfaces` package.

### LOC-8 Weighted vote

- Each sighting adds its digit probabilities to its track, weighted by crop size, sharpness, view angle and detector confidence (log-probabilities summed).
- If two tracks claim the same digit, a final assignment over tracks and digits (Hungarian on vote scores) settles it, assuming the rules make IDs unique; if they do not, this step is switched off.

### LOC-9 Final position

- The reported position is the weighted median of the track's projected sightings, which ignores the odd bad projection.
- Error budget: absolute GNSS error (about 1 to 1.5 m with the u-blox M10) dominates; projection error at 30 ft is about 0.2 to 0.3 m. RTK is only worth buying if the organisers' tolerance is tighter than the GNSS error.

### LOC-10 Judges' list

- `ID,Latitude,Longitude` CSV with 7 decimal places, rewritten atomically (temp file then rename) on every change, so a crash never leaves a half file. Also a KML and a map image for the judges.
- A `STATUSTEXT` downlink reports the count found, so the ground station sees progress. The list is due within 30 minutes after the slot.

### LOC-11 Frame logging policy

- The single frame logger lives in `argus_camera` (PLT-03); localisation sets the policy: every 4K frame during the survey, JPEG quality 90, about 42 MB/s to the 512 GB NVMe.
- The Orin Nano has no hardware JPEG encoder, so encoding runs on CPU worker threads (libjpeg-turbo). If the CPU budget is exceeded, the logger drops its rate before the detector does.

### LOC-13 Surveyed-field validation

- Lay 10 markers at measured positions on a field and fly the full mission. **Pass:** 10 of 10 found, every digit correct, relative position RMS ≤ 0.3 m after removing the common GNSS offset.

### LOC-4, LOC-12, LOC-14 (P1)

- **LOC-4 Re-visit.** After the pattern, fly back over tracks with low digit confidence or few sightings and hover 3 s over each, at survey height.
- **LOC-12 Post-landing pass.** After landing, run tiled detection over the logged 4K frames and merge with the live tracks, within 900 s so the list still meets the 30-minute deadline. Behind a flag until the organisers rule on whether processing after landing is allowed.
- **LOC-14 Retire.** `offline_photogrammetry.py`, the `mission_logger` raycast, and the `LOGS/*.BIN` files and engine kept in git; each removal needs the lead's approval.

### Problems found while writing this section

- The synthetic and Gazebo marker models put the digit in a different place from the rules' marker (the rules centre the digit in the X). DAT and SIM fix both.
- `image_grabber` publishes the new camera matrix after undistortion together with the original distortion coefficients, which double-corrects downstream. FND-05 and PLT-03 replace this.

## Recovery (REC)

Recovery is new for 2027 and has no 2026 code to build on. The drone flies to given coordinates, finds a green X with a 1 to 2 kg bucket on it, hooks the handle, verifies it is carrying the bucket, flies back and sets it down at the return point, all without human input (16 points plus the 4-point autonomy bonus). The rules warn that GPS may not always be trustworthy, so the mission must also work on vision-based navigation.

| ID | Feature | Priority | Size | Depends on | Proven in |
| --- | --- | --- | --- | --- | --- |
| REC-1 | Green X and bucket detector | P0 | M | FND-02, PLT-03 | Unit, Sim |
| REC-2 | `package_recovery` mission | P0 | L | REC-1, 3, 4, 7 | Unit, Sim |
| REC-3 | Cone descent with stall abort | P0 | S | REC-1, FND-06 | Unit, Sim |
| REC-4 | Engage, lift, attach check, swing shaping, release guard | P0 | M | REC-9 | Unit, Sim |
| REC-7 | Return and set-down (GPS path) | P0 | M | FND, PLT | Sim |
| REC-8 | Recovery simulation | P0 | L | SIM-01..04 | Sim |
| REC-9 | Flight-controller profile and mechanical interface | P0 | S | mechanical team | Field |
| REC-5 | Optical-flow odometry on the OV9782 | P1 (P0 if GPS degradation is confirmed) | L | PLT-03, FND-04 | Sim, Field |
| REC-6 | GPS health monitor and EKF source switching | P1 | M | REC-5 | Sim |
| REC-7b | Visual homing: marker (P1), keyframes (P2) | P1/P2 | M | REC-7 | Sim |

### Findings that changed the design

- **Speed-only vision will not hold position in ArduPilot 4.6.3.** The plan was to send `VISION_SPEED_ESTIMATE` into EKF3. In 4.6.3 relative navigation only starts from optical flow or body odometry; with velocity alone, the EKF drops both position-valid flags after 10 s, and the message's own covariance is ignored. **Decision:** the primary output is the MAVLink `OPTICAL_FLOW` message (`FLOW_TYPE 5`), with external-navigation position plus velocity as the alternative; speed-only is kept only to bridge gaps under 10 s. Simulation run R3 picks between the two.
- **The 2026 parameter set is readable from the dataflash logs in the repo** (ArduCopter 4.6.3, Pixhawk 6C, quad X). Rangefinder, flow, gripper, external navigation, precision landing and fence are all off; servo outputs 5 to 8 are unassigned; `EK3_SRC_OPTIONS` is 1 where ArduPilot's GPS/non-GPS guide says 0; `GPS2_TYPE` is 2 with no second GPS fitted. These feed PLT-08.
- **Existing hold code assumes good GPS.** The bullseye and delivery missions hold by freezing the current GPS latitude and longitude. Recovery uses local ENU setpoints only.
- **No RC switch for source selection.** ArduPilot's usual method uses a radio switch, which is human input and would void the bonus. The companion switches source sets with `MAV_CMD_SET_EKF_SOURCE_SET` (42007) and tracks the acknowledged set itself, since no message reports it.

### REC-1 Green X and bucket detector

- Classical detector in a new node `recovery_cv`, publishing on the FND-02 contract with classes `green_x` and `bucket` (plus a handle line). YOLO classes for both are a second source once DAT has real images.
- **X:** scale from height (about 778 px focal length at 1280 width); green mask after CLAHE; a white surround test rejects grass and green tarps; the two arms are fitted as lines (RANSAC, then Huber refinement), and the centre is their intersection, which stays correct when the bucket covers the middle. Orientation has a 90° period. Confidence combines white surround, arm angle, inlier fraction and arm width.
- **Bucket:** a blob of the expected size near the X centre; centre from an ellipse fit; handle as the longest line through it. Each class projects onto its own plane height (X at 0, rim at about 0.2 m, handle at about 0.3 m); projecting the handle onto the ground at 1 m height would overstate its offset by 43%.
- Below about 2 m the X arms leave the IMX519 view, so the OV9782 takes over; if the hook tip is visible it also gives the hook-to-handle pixel offset directly.
- **Pass (synthetic renders, 3 to 30 m):** centre error ≤ max(2 px, 1% of the X span), angle ≤ 3°, recall ≥ 98%, zero hits on 200 bullseye, marker and green-noise renders.

### REC-2 `package_recovery` mission

- New mission on `BaseMission` (not the GPS-holding bullseye base). States: connection and navigation checks → TRANSIT at 15 m → ACQUIRE → SEARCH (expanding square at 20 m, 4 m/s, about 35 s for a 25 m radius, shared generator with LOC-2) → APPROACH to 6 m → CENTRE 3 m above the handle → CONE\_DESCENT → ENGAGE, LIFT, VERIFY\_ATTACH → CLIMB and RETURN → LOWER, SET\_DOWN, RELEASE, CONFIRM → CLEAR (climb and move 2 m sideways before landing) → COMPLETE.
- **Side states:** ABORT\_CLIMB back to CENTRE (at most 3 attempts); LOST\_TARGET (hold 1.5 s, then climb until the footprint covers the 3σ estimate, at most 10 m); CARRYING\_FAULT never releases and lands with the bucket, which the rules allow.
- All new flags default to false: telemetry, GPS-untrusted mode, source switching, visual attach check, swing shaping, marker homing, keyframe homing.
- **Tests:** state walk; no descent below 0.8 m after a loss; untrusted mode never sends a global setpoint; a 10,000-run fuzz showing the release never fires above 1.83 m or with an unhealthy rangefinder while airborne.

### REC-3 Cone descent

- The allowed horizontal error shrinks with hook height above the handle: 1.0 m at 3 m and above, narrowing to 0.48 m at 0.8 m, then to 0.8 × the mechanism's capture half-width below 0.4 m (after the NimbRo MBZIRC approach).
- Inside the cone the drone descends (0.1 to 0.5 m/s, slower when low); up to 1.5 times outside it holds; further out it climbs. Error is measured from the hook point, not the image centre. If the error does not shrink by 20% in 3 s, it aborts and climbs. Below 0.8 m, descending needs a detection no older than 0.2 s.

### REC-4 Engage, lift, attach check, release guard

- **Engage:** plunge (descend past the handle, close the gripper, require an ACK with result 0) or sweep across the handle, depending on the mechanism.
- **Lift and verify:** climb to suspension length + 0.5 m and hold 1.5 s. Attached means hover throttle has risen by the expected ratio (about 1.125 for 1 kg on a 4 kg aircraft), the bucket stays fixed in the hook camera, and the latch switch (if fitted) is closed. Not attached → back to CENTRE. Throttle falling back during the return means the bucket was lost.
- **Swing shaping (flag):** velocity ramps last whole pendulum periods, leaving no residual swing; speed capped at 3 m/s.
- **Release guard:** release only when landed, or when the rangefinder is healthy, height ≤ min(1.5 m, 1.83 m) and set-down is detected (throttle drop as the bucket takes its own weight). 1.83 m is a code constant. With no healthy rangefinder the drone lands first. A release above 6 ft scores zero.

### REC-7 Return and set-down

- Return target is `takeoff`, a published `latlon`, or a `marker`. With GPS trusted it flies a global setpoint; untrusted, a local one in the EKF frame (drift about 3 to 5% of distance flown). Within 10 m, optional visual homing: a 0.8 m AprilTag at takeoff if teams may place one (P1), or keyframes saved during the takeoff climb matched by ORB and a homography (P2).
- Recovery uses `abort_rtl_mode: LAND`, because today a refused RTL is retried forever.

### REC-5 and REC-6 Flying without trusted GPS (P1)

- **REC-5 Flow odometry.** The OV9782 at 120 Hz, decoded to 320 × 200 greyscale, phase correlation between frames four apart (corner patches when the hook or bucket is in view), rotation removed using the pose buffer, scaled by rangefinder height. Quality per sample; zero quality on bare surfaces. Sent as `OPTICAL_FLOW` (primary) or external-navigation position and velocity (alternative). Camera and gyro timing must agree within 2 ms. **Pass:** replay against GPS velocity, RMS < 0.15 m/s at 5 to 15 m.
- **REC-6 GPS health.** A monitor rates GPS as TRUSTED, SUSPECT or UNTRUSTED from fix quality, EKF glitch flags and variances, disagreement with vision velocity, position jumps against integrated vision, and apparent drift of the static X. On UNTRUSTED the mission switches to source set 2 before ArduPilot's own EKF failsafe lands the aircraft. Switching back happens only after 10 s clean and only when the mission chooses. **Pass:** a 5 m step detected within 1.5 s, 0.5 m/s drift within 5 s, no false alarms in 30 minutes of clean data.

### REC-8 Recovery simulation

- X mat and bucket models come from the SIM-01 builders and are placed by scenario files. Recovery adds a new `iris_recovery` vehicle with a downward camera, a flow camera, a hook on a damped joint and a detachable joint to the bucket; SIM-06's `payload_bridge.py` attaches when the grab command arrives with the hook inside the capture envelope. SITL parameters in `sim/params/recovery.parm`; GPS glitches, jamming, drift and dropouts come from SIM-04's `inject.py`.

| Run | Setup | Pass |
| --- | --- | --- |
| R1 | Nominal, bucket 0.15 m off centre, 10 runs | ≥ 9/10; no release above 1.83 m in any log |
| R2 | Given coordinates 15 m off | X found within 90 s; ≥ 8/10 |
| R3 | 10 m GPS step during descent | Detected within 1.5 s; source set 2 acknowledged; ≥ 8/10 |
| R4 | 0.5 m/s GPS drift for 60 s | Detected within 5 s; lands within 3 m of home |
| R5 | GPS off during return | Within 3 m of home (1 m with marker) |
| R6 | Attach refused | Detected within 3 s; retries; never reports success |

### REC-9 Flight-controller profile and mechanical interface

- `docs/fc/recovery.parm`, checked by `tools/fc_param_check.py`: gripper on a spare servo output, `FS_OPTIONS` bit 5 clear, rangefinder, `EK3_SRC_OPTIONS 0`, source set 2 per REC-5, `GPS2_TYPE 0`, fence on, and no radio switch mapped to the gripper or the source set.
- **Blocking inputs from the mechanical team:** the mechanism (a servo-latched hook is preferred over a claw, since gripping is where MBZIRC teams lost most points), capture half-widths, whether yaw must align with the handle, body positions of hook, cameras and rangefinder, suspension length, a latch that holds when power is lost, the contact switch wiring, and hover throttle with 1 and 2 kg.

## Platform and flight operations (PLT)

Everything between the missions and the hardware: the Jetson, the cameras, the models, MAVROS, the flight controller's own safety settings, and the launch and logging that make each attempt repeatable. The biggest 2026 gap here is safety: the flight controller flew with no fence, no failsafes and the gripper disabled.

| ID | Feature | Priority | Size | Depends on | Proven in |
| --- | --- | --- | --- | --- | --- |
| PLT-01 | Jetson bring-up script | P0 | M | none | Bench |
| PLT-02 | Boot services and time sync | P0 | S | PLT-01 | Bench |
| PLT-03 | Unified camera driver `argus_camera` | P0 | L | none (built together with FND-01) | Sim, Bench |
| PLT-04 | Model deployment and engine builder | P0 | M | DAT-03, DAT-04 | Bench |
| PLT-05 | Latency probe (LED) | P0 | M | PLT-03 | Bench |
| PLT-06 | MAVROS config and stream keeper | P0 | S | FND-03 | Sim |
| PLT-07 | Launch, profiles and GO launcher | P0 | M | FND-08 | Sim |
| PLT-08 | Flight-controller baseline: fence, failsafes, gripper | P0 | M | mav.parm | Sim, Field |
| PLT-09 | Takeover latch, actuation lock, Lua watchdog | P0 | M | PLT-08 | Sim |
| PLT-10 | Preflight go/no-go | P0 | M | PLT-06, PLT-08 | Sim, Bench |
| PLT-12 | Logging and attempt bundle | P0 | M | PLT-03 | Sim |
| PLT-13 | GPS missions: waypoint and time trial | P0 | M | sim/gps-missions | Sim |
| PLT-11 | Health monitor and status to the ground station | P1 | S | PLT-06 | Sim |
| PLT-14 | Ground station tools | P1 | S | PLT-12 | Bench |

### PLT-01 and PLT-02 Jetson

- `platform/jetson/setup.sh` brings a fresh JetPack 6.2 Orin Nano Super to flight state: MAXN SUPER power mode, `jetson_clocks` at boot, fan curve, NVMe mounted for logs, ROS 2 Humble, ONNX Runtime, udev rule giving the flight controller a fixed name `/dev/argus/pixhawk`. A checklist confirms each step; a disk image is kept as backup.
- systemd units start MAVROS and the camera driver at boot; missions start only through the GO launcher. Chrony keeps the Jetson and the ground laptop on one clock so logs line up.

### PLT-03 Unified camera driver

- One package, `argus_camera`, with a library (`FrameSource`) and a node. It drives the IMX519 (CSI, `nvarguscamerasrc`), the OV9782 (USB, V4L2) and the 12 MP spare through one interface.
- **Sensor modes:** 4K at 17 fps for localisation; 720p at 60 fps for drop and delivery; the OV9782 for optical flow. Each mode has its own calibration file (K and D measured in that mode), published consistently in `camera_info`.
- **4K stays in one process.** `loc_node` embeds `FrameSource`; drop and delivery use 720p topics; the optical-flow code reads the OV9782 in-process. Legacy topic names stay as aliases so the 2026 path still runs.
- **One robot description** for camera mounts and the payload offset. Today three or four copies disagree; FND-05 and CEN-01 read only this one.
- Capture timestamps (FND-01) and the frame logger core live here.
- **Raw frames.** The driver publishes frames as the sensor gives them, with that mode's true K and D in `camera_info`. Nothing remaps whole frames on the Jetson; projection undistorts the few points it needs (FND-05). This removes the 2026 mismatch between the published camera matrix and distortion, and saves CPU at 4K.

### PLT-04 Model deployment

- TensorRT engines are built on the Jetson from ONNX by DAT-03's `build_engine.sh` (engines are device-specific, so none are committed). The model registry (DAT-04) records each model's hashes, input shape (736 × 1280), classes and training run; the node refuses an engine whose hash or TensorRT version does not match.
- FP16 only: INT8 is not used for the NMS-free YOLO head. One CUDA process per flight, a GPU memory budget of 5.5 GB, and the digit CNN on CPU through ONNX Runtime.

### PLT-05 Latency probe

- An LED on a Jetson GPIO sits in the camera's view. The probe blinks it and finds the frame where it lights, giving the fixed camera offset that FND-01 subtracts. A second test times detection → setpoint → flight controller response, giving the actuation delay (about 0.08 s) that FND-06 forecasts to.
- Target: capture to command ≤ 150 ms at p95.

### PLT-06 MAVROS config

- `config/mavros/argus_mavros.yaml`: timesync at 10 Hz, only the plugins we use. Message rates are requested with `SET_MESSAGE_INTERVAL` (pose 30 Hz, attitude 50 Hz, rangefinder 20 Hz, GPS 5 Hz, battery 2 Hz) and re-requested whenever the flight controller reboots.

### PLT-07 Launch and GO launcher

- One bringup launch takes a profile (an enable-list of flags, FND-08) and a mission sequence. The `go` command runs the preflight (PLT-10), starts everything and prints GO or NO-GO. The same command runs in sim and on the drone.

### PLT-08 Flight-controller baseline

- A committed parameter file, `platform/fcu/argus_2027.param`, diffed against last year's `mav.parm` (still to be collected from the 2026 team).
- Fence on (polygon from the organiser coordinates, maximum altitude, breach action RTL or land). Failsafes on for battery, RC loss, GCS loss and EKF. Gripper enabled. `FS_OPTIONS` bit 5 (release gripper on failsafe) must be 0, so a failsafe never drops the payload. Rangefinder parameters from CEN-03.
- The geofence and readiness video is due May 5 (OPS-02).
- The 2026 values are readable from the dataflash logs already in the repo, so the baseline does not wait on `mav.parm`. Changes from 2026: set `EK3_SRC_OPTIONS` to 0, set `GPS2_TYPE` to 0 (no second GPS is fitted), assign the gripper to a spare servo output (5 to 8 are unassigned), and review `FS_EKF_ACTION` (Land) against recovery's source switching.

### PLT-09 Takeover latch, actuation lock, watchdog

- **Takeover latch.** Any mode change we did not command means the safety pilot has taken over: the executor stops sending setpoints and stays out until autonomy is re-armed by hand.
- **Actuation lock.** The gripper and drop servo can only be commanded from a release state with its gates satisfied.
- **Watchdog.** A Lua script on the flight controller switches to LOITER if companion setpoints stop for more than 1 s while in GUIDED.

### PLT-10 Preflight go/no-go

- Checks: parameter file hash, fence loaded, 3D GPS fix and HDOP, EKF healthy, rangefinder reading, camera rates, engine loaded and matching the manifest, timesync healthy, disk space, battery, flag profile. NO-GO blocks the start unless overridden by hand, and the override is logged.

### PLT-12 Logging and attempt bundle

- Each attempt produces one folder `attempts/<date>_<slot>_<mission>/` with a manifest: MCAP bag of key topics (no images), the flight controller's dataflash log pulled after landing, the frames, a config snapshot, git hash, flag profile and the judges' list. `tools/bundle_attempt.py` assembles it.

### PLT-13 GPS missions

- The GPS mission harness (`sim/gps/`) was merged on 11 November (PR #10), and all three GPS missions pass in the sim. The work below builds on it.
- **Waypoint mission:** per-waypoint timeout and hold.
- **Time trial:** the primary path uploads an AUTO mission with an internal pass radius of 2 m; GUIDED is the fallback. A route optimiser and a flying start are built behind flags and switched on only if the organisers allow them ("Open questions", question 16): on the 2026 course the best order (3-2-1-4-7-6-5) is 934 m against 1278 m in the given order.

### PLT-11 and PLT-14 (P1)

- **Health monitor.** Rates, temperatures and throttling, CPU and GPU load, dropped frames; short `STATUSTEXT` codes to QGroundControl.
- **Ground station tools.** QGroundControl setup, a judges' list viewer, attempt bundle download, all in the Humble container that runs on the club's Jazzy laptops.

## Data, training and models (DAT)

The 2026 detector was trained on one class, from undistorted frames, through a notebook with a broken validation split and an API key in it; the v2.1 weights that flew are not in the repo. In 2027 one dataset definition matches what the aircraft sees, training runs from a script in the repo, and every engine that flies can be traced to its data, config and evaluation.

**On hold for the build session.** There is no 2027 training data yet, so the whole DAT lane waits: no dataset conversion, no training runs, no digit-reader training. Runtime code that uses a model is still built, against stand-ins: an oracle detector in sim that reports the true objects with noise (sim only, behind a flag), the classical detectors that need no training (bullseye colour, green X, ring fit), and a template-matching digit classifier for synthetic markers. Each stand-in is swapped for the trained model once DAT produces one. `shrink_for_detector()` moves to LOC-5, because the flight node needs it regardless.

| ID | Feature | Priority | Size | Depends on | Proven in |
| --- | --- | --- | --- | --- | --- |
| DAT-01 | Dataset specification and label schema | P0 | S | FND-02 | Unit |
| DAT-02 | Convert the 2026 data | P0 | M | DAT-01 | Unit |
| DAT-03 | Training script in the repo, with FP16 export | P0 | M | DAT-01 | Unit, Bench |
| DAT-04 | Model registry | P0 | S | DAT-03 | Unit |
| DAT-05 | Evaluation protocol (including the tiled pass) | P0 | M | DAT-01..04, SIM-09 | Unit |
| DAT-08 | Digit-reader dataset | P0 | M | LOC-6, DAT-10 | Unit |
| DAT-10 | Field data day protocol | P0 | S per day | PLT-12 | Field |
| DAT-06 | Synthetic data, phase 1 (composites at flight scale) | P1 | M | DAT-01 | Unit |
| DAT-09 | SAM 3 auto-labelling, off the aircraft | P1 | M | DAT-01 | Unit |
| DAT-07 | Synthetic data, phase 2 (BlenderProc renders) | P2 | L | DAT-06 | Unit |

### Findings that shape this section

- **2026 training frames are undistorted.** `image_grabber` remaps every frame (alpha 0, cropped field of view) before saving, so the Roboflow set is rectified. The 2027 test sets must match whatever the 2027 driver publishes.
- **Notebook defects:** the Roboflow export URL contains the API key (rotate it); `min_area_ratio` 0.1 produced fragment labels; every class forced to 0; the validation split is broken; paths point at v0.04; no seed and no export step.
- **Synthetic images are at the wrong scale.** `synth_targets.py` sizes targets at 30 to 80% of image height, so a 0.6 m marker is 216 to 576 px tall, against 38 to 61 px in flight.
- `.gitignore` excludes `datasets/`, `*.mcap` and `*.engine`, so manifests cannot live in a folder called `datasets/` and test fixtures are generated inside tests.

### DAT-01 Dataset specification

- **Classes,** using the FND-02 names: `marker` (numbered, 0.6 m), `boundary_marker` (1.2 m), `bullseye`, `green_x`, `bucket`. Whether the model trains both marker classes or one merged class split afterwards by projected ground size (above 0.9 m = boundary) is settled by DAT-05.
- **Box rules:** markers by printed area only; bullseye and X by the whole mat; bucket by rim plus handle; cut-off targets labelled when at least 30% is visible, with `truncated: true`.
- **Image scale:** masters are full 3840 × 2160 frames; training images come from the same `shrink_for_detector()` function the flight node calls (INTER\_AREA to 1280 × 720, labels scaled by exactly 1/3). No tiles.
- **Format:** COCO JSON master, with per-image flight id, capture stamp, height, pose, `rectified`, K, source (`real`, `synth_p1`, `synth_blender`, `sam3`) and label status; per object: digit, rotation, four corners in 4K pixels. Ultralytics folders are generated from it.
- **Splits by flight, never by frame.** Whole field days are held out for test; about 15% of training-day flights for validation. Near-duplicate images across splits (perceptual hash distance ≤ 6) fail the build.
- **Versioning:** `tools/training/manifests/det-<YYYY>.<n>.yaml` lists Drive paths with sha256, class map, split membership and counts. Images stay on the team Drive.

### DAT-02 Convert the 2026 data

- `tools/training/convert_roboflow.py`: download the full-frame COCO export (key from an environment variable or Colab secrets, never the repo), map classes by name, recover each image's flight from its original file name and a perceptual-hash match against the Drive originals (timestamps fail: the 2026 Jetson clock read 1969), join height and pose from the telemetry CSVs, shrink, and write manifest `det-2027.1`.
- The notebook is archived as `tools/training/legacy/` with keys and URLs removed.
- The 2026 labels contain markers only; bullseye, X and bucket labels come from DAT-09 and DAT-10.

### DAT-03 Training script and export

- `python tools/training/train.py --config tools/training/configs/det_y26n_1280.yaml [--smoke]`, run locally or from a thin Colab notebook that clones the repo at a given commit.
- Config: YOLO26n (compared against YOLO26s), imgsz 1280, 250 epochs with patience 50, batch 16, seed 0 and deterministic, rotation augmentation 180° (nadir view, any heading), both flips, scale 0.5 (covers the size range between 25 and 40 ft). Optimiser and mosaic are ablated.
- Each run records git sha (dirty tree refused), config and manifest hashes, library versions and GPU in `meta.json` on Drive. Ultralytics is pinned to 8.4.12.
- Export writes ONNX at (736, 1280); `build_engine.sh` builds the FP16 engine on the Jetson. A parity check on 300 validation images allows at most 0.01 mAP50 loss between `.pt` and engine.

### DAT-04 Model registry

- `tools/training/registry.yaml` (metadata only): for each model id, its classes, manifest hash, run, versions, weights and ONNX locations, every engine built (device, JetPack, TensorRT version, power mode, sha256), its evaluation report, and status (`candidate`, `promoted`, `retired`; one promoted per task).
- With node flag `features.model_registry` on, the detector loads by model id and checks sha256 and TensorRT version before starting; a mismatch is an error on the ground, not in the air. With the flag off, the 2026 lookup is unchanged. PLT-04 uses this registry; there is no second manifest.

### DAT-05 Evaluation protocol

- `tools/training/eval.py --model … --manifest … --split test --mode full|tiled|merged`. All predictions are mapped into 4K full-frame coordinates, so every mode is scored against the same ground truth.
- Metrics: per-class AP and operating-point precision and recall; recall by object size and by height; false positives per frame by background; mission-level fraction of markers seen in at least 3 frames, digits correct after voting, ground position error; Jetson latency p50 and p95 with power mode recorded.
- **Promotion rule:** marker recall ≥ 0.95 at 25 to 40 ft, ≤ 0.05 false positives per frame, bullseye and X recall ≥ 0.99 at 7.6 to 30 m, p95 latency ≤ 50 ms at 736 × 1280 FP16.
- The tiled pass is compared on the same flights at detection level and at list level (in-flight list vs merged list).

### DAT-08 Digit-reader dataset

- One sample is the 64 × 64 patch LOC-6's own pipeline produces, plus the 4K source crop. Labels: digit 0 to 9 or `unreadable`, and the number of 90° turns to upright. No flip augmentation, so 6 and 9 stay distinct.
- At least 300 real crops per digit and 1,000 `unreadable`, plus 2,000 synthetic per digit; test on a held-out day.
- **Pass:** ≥ 99% precision while answering ≥ 90% of crops (the reader may abstain; LOC-4 re-visits those markers), zero 6↔9 confusions.

### DAT-10 Field data day

- Layout: grass and asphalt zones; ten 0.6 m markers (digits 0 to 9, random headings, two shaded); four 1.2 m markers at the corners of about 30 × 20 m; a 5 m bullseye; a 5 m X with the bucket; distractors.
- Ground truth without new hardware: two stakes logged by drone GPS averaged over 10 minutes, every target tape-measured from the stakes.
- Flights: lawnmower at 25, 30, 35, 40 and 50 ft; a 360° heading sweep; vertical profiles over the bullseye and X from 30 m to 2 m (logged as non-localisation flights); a 60 s hover in steady wind (drag area for SIM-03); the LED latency probe.
- Set the Jetson's clock before the first flight; copy everything to Drive with a sha256 manifest the same day; assign each day to train or test before looking at results.

### DAT-06, DAT-07, DAT-09 (P1 and P2)

- **DAT-06 Composites.** `synth_targets.py --scale deploy` draws height per class, renders full 4K scenes with lens distortion, motion blur, rolling-shutter shear, exposure jitter and shadows on real field backgrounds. About 6,000 scenes. Adopted only if real held-out recall improves by at least 2 points.
- **DAT-07 BlenderProc.** Physically rendered grass, asphalt, cloth targets and buckets at our camera intrinsics; used only if DAT-06 gains less than 2 points.
- **DAT-09 SAM 3.** Text-prompted auto-labels on a Colab GPU, filtered by expected size at the logged height, then reviewed by a person; only reviewed labels enter train or val, and test sets are fully hand-checked. Request access to the SAM 3 weights now.
- The Ultralytics licence is AGPL-3.0; note it in the repo.

## Simulation and test infrastructure (SIM)

The sim already flies the 2026 delivery to 0.03 to 0.11 m. What it cannot do yet is test 2027: there is no wind that works, no GPS faults, no payload physics, no markers or X mat, no scoring, and every run needs a person at the keyboard. This section turns the sim into an unattended test bench that scores each run the way the judges will, so the build session can prove its own work.

| ID | Feature | Priority | Size | Depends on |
| --- | --- | --- | --- | --- |
| SIM-01 | Scenario files, world builder, object models | P0 | M | SIM-02 |
| SIM-02 | Competition-site world, home and geofence | P0 | S | none |
| SIM-03 | Wind through applied drag force | P0 | M | SIM-01 |
| SIM-04 | GPS faults and wrong target coordinates | P0 | S | SIM-02 |
| SIM-05 | Sensors: camera profiles, rangefinder, flow camera, true pose | P0 / P1 | M | SIM-01, PLT-03 |
| SIM-06 | Payload, gripper and release physics | P0 | M | SIM-01 |
| SIM-07 | Headless scenario runner | P0 | M | SIM-01, SIM-08 |
| SIM-08 | Judges that score like the organisers | P0 | M | none |
| SIM-09 | Replay harness (record on aircraft, replay anywhere) | P0 | M | FND-01 |
| SIM-10 | CI tiers and nightly runs | P0 | S | SIM-07 |
| SIM-11 | Acceptance gates G0 to G8 | P0 | S | SIM-07, SIM-08 |
| SIM-12 | Gaussian-splat twin of the test field | P2 | L | SIM-09 |

This table has no "Proven in" column: a SIM feature is proven in sim when its Pass line names a sim run, otherwise by unit tests. `docs/build/features.json` records the level for each.

### Findings that shape this section

- `SIM_WIND_*` does nothing when Gazebo runs the physics, and Gazebo's WindEffects is reported to have no visible effect on multicopters in Harmonic. Wind must be applied as a force.
- `start.sh` passes `--no-extra-ports`, so the parked GPS branch's parameter tool (SITL serial 2, TCP 5763) cannot work in the main sim.
- The sim runs at 0.4 to 0.98 of real time on club laptops while our nodes use the wall clock; a laptop suspend once cut a run short.
- CI only requires tests for `src/` and `tools/`, so `sim/` code can merge untested, and there is no secret scan.
- The sim MAVROS uses the default plugin list, which blocks the plugins recovery needs (optical flow, vision pose and speed, distance sensor).

### SIM-01 Scenarios and world builder

- One YAML per test case in `sim/scenarios/<mission>/`: mission, site, seed, sequence file, config overrides, flags, objects (markers with size, digit and heading; X mat; bucket with mass), random placement rules, error in the given coordinates, wind, GPS events, sun, sensor profile, judge settings.
- `sim/scenario/build.py` (ROS-free) writes a run folder: the world with every object embedded (no spawn race), the vehicle variant and matching bridge file, SITL parameters, the run's sequence, `truth.json` and `judge.yaml`.
- `sdf_objects.py` gains marker (0.6 and 1.2 m, digit centred in the X as the rules show), green X mat, bucket (dynamic, handle upright, mass as a parameter) and payload builders. Recovery's models (REC-8) use these builders.
- Flags from a scenario reach only the generated sequence under `sim/.state`, so shipped YAML stays false; a flag name that does not exist in shipped config fails the build.

### SIM-02 Competition site

- `sim/sites/apollo_xi.yaml`: origin (exact coordinates still to verify from the field map), terrain elevation (a wrong home altitude made the SITL rangefinder read 18.5 m on the ground), runway, flight boundary, maximum altitude, takeoff point and compass rose, and a placeholder survey area.
- The geofence polygon is uploaded by `sim/fence_upload.py`; the real flight controller uses the same file (PLT-08). Scenario runs always wipe SITL's stored parameters.
- **Pass:** a waypoint 20 m outside the fence triggers a breach and RTL within 2 s, with at most 10 m excursion.

### SIM-03 Wind

- Our own drag force F = ½ ρ CdA |v\_rel| v\_rel through Gazebo's ApplyLinkWrench, driven at 20 Hz from true vehicle velocity; mean wind plus Ornstein–Uhlenbeck gusts (τ 2 to 5 s), optional discrete gusts, vertical gusts. CdA 0.08 m² for the Iris until DAT-10's hover gives ours. Payloads have their own drag, so drop drift is simulated too.
- Levels: calm; moderate 4 m/s with 1 m/s gusts; strong 7 m/s with 2 m/s gusts and one 3 m/s discrete gust.
- **Pass:** hovering at 5 m/s wind, logged tilt within 20% of atan(F / mg).

### SIM-04 GPS faults

- Two separate effects: an error in the coordinates the mission is given (recovery's "GPS may not be trustworthy"), and vehicle GPS faults (glitch steps, noise, satellite count, jamming, dropouts) set through SITL parameters at scheduled sim times by one tool, `sim/inject.py` (REC-8 uses it; there is no separate fault script).
- The two drafts of this brief disagree on 4.6.3's parameter names (`SIM_GPS_GLITCH_*` vs `SIM_GPS1_GLTCH_*`), so `inject.py` reads the names from the running SITL at the start and aborts if they are missing.
- **Pass:** a 5 m north glitch moves the raw GPS fix 5 ± 0.5 m within 1 s.

### SIM-05 Sensors

- Camera profiles generated from one template: `delivery` 640 × 360 at 30 Hz (today's), `survey` 1280 × 720 at 10 Hz, `still4k` 3840 × 2160 at 1 Hz (headless only, for digit crops); field of view matches the IMX519 calibration.
- A global-shutter flow camera at 640 × 400, 60 Hz (the real 120 Hz is too heavy for club laptops).
- Rangefinder: SITL type 100 now (sees only flat terrain); a Gazebo one-beam lidar that sees the bucket, relayed to MAVROS, as P1.
- MAVROS in the sim gets an explicit plugin list including optical flow, vision pose and speed, and distance sensor. True poses are bridged to `/sim/truth/poses`.
- **Pass:** `survey` at ≥ 8 Hz headless with real-time factor ≥ 0.8; the rangefinder within 0.1 m of true height in a hover.

### SIM-06 Payload and gripper

- Payloads hang from Gazebo detachable joints. One `sim/payload_bridge.py` watches servo outputs: a release PWM detaches; a grab PWM attaches the bucket only if the hook is within the capture half-width (parameter, default 0.10 m) and 0.15 m vertically of the handle, otherwise it logs `grab_miss`. The sprayer and gripper parameters copy the aircraft's.
- With `fake_drop: true` no command is sent and the judge scores the vehicle position at the fake-release log line.

### SIM-07 Headless runner

- `sim/run_scenarios.sh <scenario|suite> [--seeds 1,2,3] [--variants off,on]`, suites `smoke`, `nightly`, `gates`. The runner keeps the laptop awake for the whole suite itself (`inhibit_sleep` from `sim/gps/common.sh`, as `sim/gps/fly_all.sh` does). Each run: build, start a fresh container detached, wait for a READY file (900 s timeout), fly with recorder and injectors, end on disarm or timeout, judge, tear down.
- All durations in sim time. Until every node runs on sim time (FND-03), a run whose real-time factor averages below 0.8 is INVALID.
- Output: `score.json` and `report.md` per run, and a suite table of flags off (2026) against flags on.
- **Pass:** the smoke suite runs three times in a row with nobody at the keyboard.

### SIM-08 Judges

| Mission | How the judge scores it (from true positions, never the EKF) |
| --- | --- |
| Waypoint navigation | Each of 7 waypoints in order within 1.0 m (3D) |
| Time trial | Every waypoint passed within 4 m in order; sim time between first and last; 12 or 8 points by rank against reference times |
| Localisation | Our CSV matched to truth by the Hungarian method; tolerance a parameter (default 2.0 m pending the rules); −0.5 wrong ID, −0.5 wrong location, −1 both; minimum height 6.10 to 7.62 m costs −3, below 6.10 m scores 0; CSV must exist before landing; merged list scored separately |
| Drop | 14 − distance in metres; −6 if released below 1.83 m; −6 if the vehicle goes below 1.83 m |
| Delivery | 13 − distance; 0 if released above 1.83 m; contact impulse logged against a limit parameter; +0.25 per 500 g |
| Recovery | Bucket lifted above 0.5 m; 0 if released above 1.83 m; distance to the return point (takeoff or compass rose, a parameter) |

Every mission: single flight, geofence breaches and failsafes reported, +25% autonomy if the log shows no ground-station commands.

### SIM-09 Replay harness

- `sim/replay/record.launch.py`, used in sim and on the aircraft, records MCAP bags of MAVROS pose, IMU, GPS, state, rangefinder, setpoints and servo outputs plus camera info, detections, estimator outputs and flow frames; 4K frames stay as JPEG files with an index.
- Two modes: `ros` republishes with original stamps and `/clock` through the real nodes; `core` reads the bag in pure Python and calls the estimator, tracker and flow code directly, deterministically, in CI on bags generated by the tests.
- Metrics: estimator error over time and at release, mean NIS and fraction above 9.21, time to lock, lost-state duration, capture-to-output latency, flow velocity error, per-marker localisation error.

### SIM-10 CI tiers

| Tier | Where and when | What runs |
| --- | --- | --- |
| A | GitHub Actions, every PR | `check.sh` plus every ROS-free test: scenario build, judges, wind, GPS conversions, payload bridge, replay `core`, dataset and registry tools, model parity |
| B | Nightly, on a club machine that never sleeps | Fresh worktree of main → `nightly` suite, three seeds, flags off and on → report posted to a pinned GitHub issue |
| C | Per PR, by the author | `smoke` suite for PRs touching missions, CV or sim; report pasted in "How I tested it" |
| D | Weekly, GPU desktop | `still4k`, splat replay, long gate suites |

- `pr_checks.py` counts `sim/` as code; the hygiene gate gains a secret-pattern rule. Tier B is not a self-hosted Actions runner, because the repo is public.

### SIM-11 Acceptance gates

| Gate | What it proves | Pass |
| --- | --- | --- |
| G0 | Sim baseline, flags off | Delivery touchdown ≤ 0.3 m in 5/5 |
| G1 | Estimator and centring | Hold at 6 m and 2 m: p95 true error ≤ 0.20 m in moderate wind, ≤ 0.30 m in strong |
| G2 | Drop | Median ≤ 0.15 m calm and ≤ 0.30 m strong, max ≤ 1.0 m; no release or descent below 1.83 m |
| G3 | Delivery | Median ≤ 0.5 m; release ≤ 1.83 m; impulse under the limit |
| G4 | Localisation | ≥ 95% listed and IDs correct; error median ≤ 1.0 m, p95 ≤ 2.0 m; never below 7.62 m; CSV before landing |
| G5 | Recovery | ≥ 8/10 with up to 8 m coordinate error and a 5 m glitch mid-descent; no release above 1.83 m; every failure ends in a climb and RTL |
| G6 | GPS missions | Waypoint within 1 m and time trial within 4 m, 5/5 including moderate wind |
| G7 | Field replay | Deterministic `core` replay; detector meets the DAT-05 promotion rule |
| G8 | Readiness (May 5) | G0 to G6 green on main within 7 days of the deadline, plus one real flight |

### SIM-12 Gaussian-splat twin (P2)

- Capture the Purdue test field in 4K, train a splat offline, render along recorded trajectories at our camera intrinsics with synthetic targets composited, and replay through the detector and estimator. Offline only; first to cut.

## Rules compliance and competition-day operations (OPS)

This section ties the software to the 2027 rules (Rev 1, 16 September 2026) and to what happens on the field from 4 to 6 June 2027. Most of it is tooling that saves minutes in a slot and proof that the autonomy bonus was earned.

### Rules that constrain the software

| Rule (Rev 1) | What the software does | Where |
| --- | --- | --- |
| Best 4 missions count | Slot plan concentrates on the four with the highest expected points | OPS-04 |
| +25% autonomy bonus on localisation, drop, delivery, recovery, only with no human input | No RC mapping touches the payload or navigation source; takeover latch; post-flight autonomy audit | PLT-09, REC-9, OPS-03 |
| Localisation: below 25 ft costs 3 points, below 20 ft scores 0 | Hard floor constant, RECOVER and ABORT guard, default 35 ft | FND-09, LOC-1 |
| Localisation: four larger objects mark the area | `boundary_marker` class; tracks outside the hull are kept off the list | DAT-01, LOC-7 |
| Localisation list due within 30 minutes after the slot | List written on board before landing; post-landing pass limited to 900 s | LOC-10, LOC-12 |
| Drop: release at or above 6 ft; going below 6 ft costs 6 points | Release at 2.4 m (rangefinder) or 3.0 m (baro); climb below 2.0 m; actuation lock | CEN-04, PLT-09 |
| Delivery: impulse limit to be announced; +0.25 per 500 g | Soft set-down at 0.12 m/s; impulse measured on a rig | CEN-05, OPS-07 |
| Recovery: 1 to 2 kg bucket; GPS may not always be trustworthy; return to compass rose or takeoff point | Attach check by throttle ratio; flow odometry and source switching; return target is a parameter | REC-4, REC-5, REC-6, REC-7 |
| Time trial: 12 points fastest, 8 slowest | AUTO mission with route optimiser, flying start | PLT-13 |
| Geofence and readiness video due 5 May 2027 | Fence on in the baseline; readiness kit; gate G8 | PLT-08, OPS-02, SIM-11 |

### Features

| ID | Feature | Priority | Size | Depends on |
| --- | --- | --- | --- | --- |
| OPS-01 | Field setup tool | P0 | M | PLT-08, SIM-02 |
| OPS-02 | Readiness kit (May 5 video and logs) | P0 | S | PLT-08, PLT-12 |
| OPS-03 | Post-flight report and autonomy audit | P0 | M | PLT-12, SIM-08 |
| OPS-04 | Scoring model and slot planner | P0 | S | SIM-08 |
| OPS-05 | Slot time budget and clock | P1 | S | PLT-07 |
| OPS-06 | Code freeze, field tuning and rollback | P0 | S | PLT-07 |
| OPS-07 | Impulse rig (delivery) | P1 | S | hardware |
| OPS-08 | Full site rehearsal | P0 | S | everything |
| OPS-09 | Required documents: SOP, emergency procedures, checklists | P0 | S | PLT-10 |

OPS features are proven by unit tests unless noted (OPS-02 in sim, OPS-08 in the field); `docs/build/features.json` records the level for each.

- **OPS-01 Field setup.** The day's coordinates (survey area, targets, waypoints, fence, takeoff point) are entered once in one file; the tool checks them against the rules and the fence and generates the mission sequences, the fence upload and the site file. No coordinates are typed into YAML by hand on the day.
- **OPS-02 Readiness kit.** A scripted geofence demonstration flight, the logs and the video checklist for the 5 May submission.
- **OPS-03 Post-flight report.** From the attempt bundle: what happened, a score estimate using the SIM-08 judges on the real log where possible, and an autonomy audit that scans the flight log for any RC override or ground-station command inside the autonomous window. The audit is the evidence for each bonus claim.
- **OPS-04 Scoring model.** A Monte Carlo over mission success probabilities from sim and field results. Its main finding: if missions may be combined in one flight, the expected score rises from about 39 to about 57 points. Draft slot plan: slot 1 localisation and drop, slot 2 time trial and delivery, slot 3 recovery. This depends on organiser question 1 ("Open questions").
- **OPS-05 Slot clock.** Per-slot timeline (setup, preflight, flight, list deadline) shown by the GO launcher and on the ground station.
- **OPS-06 Freeze and rollback.** Code freeze around 19 May 2027, tag `comp-2027-rc1`. On the field, changes only through tuning files with a `[tuning: …]` waiver. A fallback workspace `~/ARGUS_2026_fallback` with the `comp-2026` build can be launched in minutes if 2027 code misbehaves.
- **OPS-07 Impulse rig.** A plate with a load cell or accelerometer that measures the delivery contact impulse, so the descent speed can be set against the limit once it is announced.
- **OPS-08 Rehearsal.** A full competition day at the test field on the real timeline, all slots, all paperwork.
- **OPS-09 Documents.** Standard operating procedure, emergency procedures and checklists, generated where possible from the preflight checks so they cannot drift apart.

## Build order and dependencies

The test bench comes first, so that every later feature is proven by an unattended, scored sim run rather than by someone watching Gazebo. Then the shared time and geometry, then the estimator, then the four mission lanes side by side. Hardware work waits until the logic has passed in sim.

Each wave is "finish this before relying on it", not "stop all other work": the data lane and the ROS-free parts of later waves can start early.

| Wave | Contents, in order | Done when |
| --- | --- | --- |
| 0 Test bench | SIM-08 judges → SIM-02 site → SIM-01 scenarios → SIM-07 runner → SIM-10 tier A; FND-08, FND-09; create `argus_interfaces`. The GPS mission harness that PLT-13 builds on is already on main (PR #10) | The smoke suite runs three times unattended; G0 green; CI requires tests for `sim/` and scans for secrets |
| 1 Time and geometry | FND-03 and PLT-06 → PLT-03 (driver core) and FND-01 → FND-02 → FND-04 → FND-05; the DAT lane waits for training data | Pose stamps agree with `/clock` within 20 ms; projection tests pass |
| 2 Estimator and sim physics | FND-06 → FND-07; SIM-09 replay `core`; SIM-03 wind; SIM-06 payloads; SIM-05 sensors | FND-06 acceptance met; replay deterministic; G0-W2 (flags off) green |
| 3 Mission lanes | **Drop and delivery:** CEN-03 → CEN-01 → CEN-02 → CEN-13 → CEN-04 → CEN-05 → CEN-06 → CEN-10, 11 → CEN-12. **Localisation:** LOC-1 → LOC-2 → LOC-3 → LOC-5 → LOC-7 → LOC-9 → LOC-11, then LOC-6 → LOC-8 → LOC-10, with a stand-in digit classifier until DAT-08 has real crops. **Recovery:** REC-8 → REC-1 → REC-3 → REC-4 → REC-7 → REC-2. **GPS missions:** PLT-13 | G1 (after CEN-02) and G2 to G6 green with flags on; G0-W3 (flags off) green |
| 4 Aircraft and field | PLT-01, 02 → PLT-04, 05 → PLT-08 → PLT-09 → PLT-10 → PLT-07 → PLT-12; OPS-01, 03, 04, 06, 09; DAT-04, DAT-05 on the Jetson | Latency probe and engine parity pass on the bench; preflight says GO on the aircraft; G7; G8 by 5 May |
| 5 If time allows | P1 items, then P2 | Each behind its flag, with an A/B run showing no regression |

**Critical paths.** Drop: FND-03 → FND-01 → FND-04 → FND-05 → FND-06 → CEN-04 → G2. Localisation: PLT-03 (`FrameSource`) → LOC-5 → LOC-7 → LOC-10 → G4, with the digit reader waiting on field data. Recovery: SIM-06 and REC-8 → REC-1 → REC-2 → G5, with the mechanism (REC-9) as the outside dependency.

**What can run side by side.** The four wave-3 lanes share only the estimator and the sim, so they can be built in parallel once wave 2 is done. The DAT lane is on hold until training data exists; once it starts, everything in it except the field days and engine builds is ROS-free and can run alongside any wave.

## Tasks for people, hardware and the field

The long session can write, test and prove in sim everything in waves 0 to 3 and the software half of wave 4. The items below need hands, hardware, a field or a decision from a person. Most are short, but several block a lane, so they should start now rather than after the session.

### People and permissions

| Task | Blocks | Who |
| --- | --- | --- |
| Rotate the Roboflow API key that sits in the 2026 notebook | DAT-02 (and it is a live leak) | Roboflow workspace owner |
| Collect from the 2026 team: the `.pt` weights and ONNX behind the v2.1 engine, `mav.parm`, the full-frame Roboflow export, the Drive flight folders | DAT-02, DAT-05 baseline, PLT-08 diff | Autonomy lead |
| Post the organiser questions in "Open questions" on Discord | Defaults in LOC, REC, OPS | Autonomy lead |
| Approve retiring 2026 pieces (LOC-14) and review and merge every PR | Merges | Team lead |
| Choose the delivery package weight once the impulse limit is known | CEN-05, OPS-07 | Team |
| Request access to the SAM 3 weights on Hugging Face | DAT-09 | Anyone |
| Film and submit the readiness video by 5 May 2027 | G8 | Team |

### Hardware and builds

- **Buy the downward rangefinder** (TFmini Plus or TFmini-S), mount it clear of the hook line and the bucket's swing, wire it to the flight controller. This is the only new purchase in the brief. (CEN-03, REC-9)
- **Recovery mechanism** from the mechanical team: servo-latched hook on the flight controller's gripper output, contact switch, capture half-widths, body positions of hook, cameras and rangefinder, suspension length, hover throttle with 1 and 2 kg. (REC-9; blocks REC-3 and REC-4 tuning)
- **LED latency probe**: one LED on a Jetson GPIO, placed in the camera's view. (PLT-05)
- **Impulse rig**: a plate with a load cell or accelerometer. (OPS-07, P1)
- **Targets**: 0.6 m numbered markers in the rules' design, 1.2 m boundary markers, a full-size X mat replica, a 3 L bucket with 1 and 2 kg loads, the 2026 bullseye. (DAT-10, LOC-13, REC field tests)
- Confirm the 512 GB NVMe is fitted for frame logging. (LOC-11)

### Bench and aircraft

- Run the Jetson setup script on the real Orin and keep a disk image. (PLT-01)
- Calibrate each camera in each sensor mode with a checkerboard (IMX519 4K and 720p, OV9782), and measure the mounts for the robot description. (PLT-03, FND-05)
- Build the TensorRT engines on the Jetson and run the parity and latency checks. (DAT-03, DAT-05, PLT-04)
- Load the baseline parameters, upload the fence, and test each failsafe on the aircraft with the safety pilot. (PLT-08, PLT-09)
- A club machine that never sleeps, for nightly sim runs, with a GitHub token limited to issues. (SIM-10)

### Field days

- At least two data days before the training cutoff, at different sun angles (DAT-10), including the hover in steady wind for the drag area (SIM-03) and the vertical profiles over the bullseye and X.
- Surveyed-field validation of localisation (LOC-13).
- Calibrate the recovery attach check with 1 and 2 kg dummies, and the release confirmation thresholds (REC-4, CEN-14).
- Field drops to fit the release offset model (CEN-08, P2).
- A full competition-day rehearsal (OPS-08).
- A safety pilot for every flight.

## Open questions

None of these blocks the session: each has a default the software uses until it is answered, and each answer becomes a parameter or a flag change rather than new code.

### For the organisers

| # | Question | Default until answered | Affects |
| --- | --- | --- | --- |
| 1 | May several missions be flown in one flight? | Not allowed; both slot plans are kept ready | OPS-04 (about 57 vs 39 expected points) |
| 2 | How many slots, and how long is each? | Three slots | OPS-04, OPS-05 |
| 3 | When does the autonomous window start and end (does a manual takeoff or landing void the bonus)? | Takeoff to landing fully autonomous | PLT-07, OPS-03 |
| 4 | May the localisation list include results from processing after landing? | No; in-flight list only, post-landing pass off | LOC-12 |
| 5 | What is the localisation position tolerance? | 2.0 m in the judge; no RTK | LOC-9, SIM-08 |
| 6 | How is "below 25 ft" judged: from our log or by an observer? | Fly at 35 ft | LOC-1 |
| 7 | What do the four larger boundary markers look like, and are they listed? | Detected as `boundary_marker`, kept off the list | DAT-01, LOC-7 |
| 8 | Marker design: font, digit size, and how 6 and 9 are told apart | Both seven-segment and font styles trained; the reader may abstain | DAT-08, LOC-6 |
| 9 | Delivery impulse limit, and how it is measured | Contact at 0.12 m/s; measured on our rig | CEN-05, OPS-07 |
| 10 | Weight bonus: maximum weight and what counts | Fixed package weight | CEN-05 |
| 11 | Recovery return point: compass rose or takeoff point? May teams place a marker there? | Takeoff point; marker homing off | REC-7 |
| 12 | Bucket size, colour and handle height; X arm width and colour | Ø 0.19 m bucket, 0.5 m arms, as parameters | REC-1, SIM-01 |
| 13 | What does "GPS may not always be trustworthy" mean: wrong coordinates, degraded GPS, or both? | Both: search for coordinate error (P0), flow odometry for degraded GPS (P1) | REC-2, REC-5, REC-6 |
| 14 | Exact takeoff location for rotorcraft ("close to the centre of the runway") | Home set at arming | PLT-13, REC-7 |
| 15 | Waypoint navigation: 2D or 3D tolerance? | 3D, 1 m | SIM-08 |
| 16 | Time trial: start and stop definition; may waypoints be reordered; is a flying start allowed? | Given order, standing start | PLT-13 |
| 17 | When are fence, survey-area and target coordinates given, and in what format and datum? | Decimal degrees, WGS 84, the morning of the slot | OPS-01 |
| 18 | What must the readiness video show? | The rules' text, a fence breach demonstration | OPS-02 |
| 19 | How is the localisation list handed in (file, USB, paper)? | CSV on USB plus a printed map | LOC-10 |
| 20 | How is the 6 ft release height checked? | Our rangefinder log is kept as evidence | CEN-04, OPS-03 |
| 21 | Does a second attempt replace the first, or does the best count? | Replaces | OPS-04 |
| 22 | Which radio frequencies and powers are allowed for telemetry? | Current radios | PLT-14 |
| 23 | Will scores be published after each slot? | No plan change | OPS-04 |

### For the team

| Question | Owner | Default |
| --- | --- | --- |
| Recovery mechanism: hook with servo latch or claw; plunge or sweep | Mechanical | Servo-latched hook, plunge |
| Delivery package weight | Team | Decided after question 9 |
| Survey height: 30 or 35 ft | Autonomy | 35 ft until question 6 is answered |
| Recovery flow output: `OPTICAL_FLOW` or external navigation | Autonomy | Decided by sim run R3 |
| Detector size: YOLO26n or YOLO26s | Autonomy | YOLO26n unless DAT-05 shows s meets the latency budget with better recall |
| RTK GPS | Team lead | Not bought unless question 5's tolerance is tighter than about 1.5 m |
