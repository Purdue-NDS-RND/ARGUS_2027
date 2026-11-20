# drone_mission_core

Reusable timer-driven mission framework for ROS2 drone missions in this workspace.

This package does not define mission behavior itself. It provides the common executor, mission lifecycle interfaces, sequence loading, and shared mission context used by higher-level mission packages such as `drone_mission_demo`.

## What It Contains

### `mission_executor`

Main executor node that:
- loads a mission sequence from YAML
- imports mission modules so they can register mission types
- instantiates missions one at a time
- owns shared MAVROS subscriptions, publishers, and service clients
- advances the active mission on a timer
- applies configurable mission failure handling (`ABORT_AND_RTL` or `CONTINUE_TO_NEXT`)

The executor is intentionally non-blocking and follows the same ROS2 style already used elsewhere in the repo: subscriptions + async service calls + `create_timer()`.

### Mission API

Defined in `drone_mission_core/mission_api.py`:
- `MissionStatus`
- `MissionFailurePolicy`
- `MissionSpec`
- `BaseMission`

The intended lifecycle is:
1. `on_enter(context)`
2. repeated `update(context)` calls
3. `on_exit(context)` on completion or failure

### Mission Context

Defined in `drone_mission_core/mission_context.py`.

`MissionContext` is the executor-owned facade given to every mission. It currently provides:
- MAVROS state access
- local pose access
- global GPS access
- target detection and image size access for vision-guided missions
- target-detection cache clearing and target-CV enable/disable requests
- clock/logger access
- takeoff requests through `drone_utils/takeoff`
- local position setpoint management
- global GPS setpoint management
- local velocity setpoint management
- gripper / sprayer actuator requests through MAVROS `CommandLong`
- mode changes such as `RTL`

### Mission Registry

Defined in `drone_mission_core/registry.py`.

Responsibilities:
- register mission classes via `@register_mission("type_name")`
- import mission modules dynamically
- load mission sequence YAML
- instantiate missions from `type` values

## Topics and Services

### Subscribed

| Topic | Type | Purpose |
|---|---|---|
| `/mavros/state` | `mavros_msgs/State` | FCU connection and mode |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | Local ENU position feedback |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | GPS telemetry |
| `/drone_package_drop/target_detection` | `geometry_msgs/PointStamped` | Center-relative normalized target offsets in `[-1, 1]` for the new vision-guided missions |
| `/drone_package_drop/image_size` | `geometry_msgs/PointStamped` | Camera image dimensions retained for observability/debugging |

### Published

| Topic | Type | Purpose |
|---|---|---|
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | Managed local position setpoint keepalive |
| `/mavros/setpoint_raw/global` | `mavros_msgs/GlobalPositionTarget` | Managed global GPS setpoint keepalive |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | Managed local velocity setpoint keepalive |

### Service Clients

| Service | Type | Purpose |
|---|---|---|
| `drone_utils/takeoff` | `mavros_msgs/CommandTOL` | Shared takeoff sequence |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | Mode changes such as `RTL` |
| `/mavros/cmd/command` | `mavros_msgs/CommandLong` | Shared gripper / sprayer actuator commands |
| `/drone_package_drop/set_target_cv_enabled` | `std_srvs/SetBool` | Enable or disable target detection work |

## Sequence YAML Shape

The executor expects a YAML file with a structure like:

```yaml
mission_sequence:
  on_failure: abort_and_rtl
  features:
    use_frame_time_pose: false
  missions:
    - type: takeoff
      name: takeoff
      config:
        target_altitude_m: 10.0

    - type: local_waypoint
      name: fly_square
      config:
        pattern_file: ../patterns/square.yaml
        waypoint_altitude_m: 20.0

    - type: rtl
      name: rtl
      config:
        rtl_mode: "RTL"
```

Supported failure policies:
- `abort_and_rtl`
- `continue_to_next`

### Feature flags

Mission flags live in the optional `features:` block above. The loader copies it into
every mission's `spec.features`; read a flag with
`drone_mission_core.features.get_feature_flag(self.spec.features, "use_frame_time_pose")`,
which returns `False` when the flag or the block is missing. Shipped YAML keeps every
flag `false` (`src/drone_mission_demo/test/test_feature_flags_config.py` enforces it).

`time_health_guard` (declared, off, in the six sequences that fly a drop or delivery
mission) turns on the stale-data guard: the executor watches pose age, frame age and
MAVROS's time-sync offset (`time_health.py`, limits 100 ms, 250 ms and a 10 ms jump), and a
vision mission treats UNHEALTHY as "no target": it holds, may climb to look again, and
never descends. A blind column descent that stays held for `time_guard_hold_timeout_s`
(default 10 s) fails the mission, so its failure policy acts. Once UNHEALTHY the guard
holds until the time base has been healthy for a full second. What it needs to read
HEALTHY:

- MAVROS time sync running and converged (`/mavros/timesync_status`, and pose stamps that
  are synchronised flight-controller time; for about a minute after MAVROS starts they
  are arrival times and the guard says so).
- Poses at 20 Hz or more: the pose-age limit is 100 ms.
- The detector's node flag `features.stamp_from_image` on, for the frame age to be the
  frame's. With it off the detector stamps its output with its own clock, and the guard
  only notices a camera or detector that has stopped.

`use_frame_time_pose` (declared, off, in every sequence) makes the executor keep five
seconds of pose history (`pose_buffer.py`) and where the ground is by the rangefinder,
and makes a drop or delivery mission find its target with projection v2
(`projection_v2.py`): the pose at the frame's capture time, the sensor mode's own
calibration and the camera's mount, in place of the 2026 latest-pose projection. The
error steered on is the target's ground point minus the vehicle's position now; a frame
over `frame_time_max_age_s` old (default 0.5 s), or one with no pose in the history,
gives no tracking solution. What it needs:

- `camera_mode` and `airframe` in the tracking mission's config (a sensor mode and an
  airframe from `argus_camera`'s `modes.yaml` and `robot.yaml`). Without them, or with a
  calibration that is a stand-in or an airframe that is not verified, the executor does
  not start. Only the two sim sequences name them so far.
- The detector's node flag `features.stamp_from_image` on, and the detector and MAVROS on
  one clock, so that a detection's stamp is its frame's. Nothing checks this pairing: with
  the detector's flag off the pose used is the one at publish time, as good as 2026 and no
  better.
- The detector's images in the size of that sensor mode: a calibration is not scaled.

`use_target_estimator` (declared, off, in every sequence; needs `use_frame_time_pose`)
runs the target estimator inside the executor (`target_estimator.py`, fed by
`estimator_feed.py`). It listens to the detection contract
(`/perception/<detector>/detections`, detectors named by the executor parameter
`estimator_detectors`, default `target_cv`), projects every detection with projection v2
onto its class's plane with the pose at the frame's capture time, and keeps one two-state
Kalman filter per target: gate at a normalised innovation squared of 9.21, TENTATIVE →
CONFIRMED after three hits in a second → COASTING after 0.3 s → LOST after 2 s, kept 30 s.
The tracks are published at 10 Hz as `vision_msgs/Detection3DArray` on
`/estimator/targets` (id `<class>/<track>/<STATE>`). No mission reads the estimator yet;
the context calls arrive with FND-07 and the missions with CEN-01. What it needs:

- Exactly one camera named by the sequence's missions (`camera_mode` and `airframe`), or
  the executor does not start.
- A detector publishing on the contract: for the 2026 bullseye detector, its node flags
  `features.detection_contract` and `features.stamp_from_image` on.

A mission reads the estimator through its context (`MissionContext`, rules in
`target_access.py`), never the feed itself:

| Call | What it gives |
| --- | --- |
| `estimator_ready()` | True when the estimator is running and the pose history is live. False with the flag off |
| `primary_target(cls, near=None)` (`near` by keyword) | The class's track to act on: `position` (east, north, up), `covariance`, `sigma_m`, `age_s`, `state`, for the actuation time. None with the flag off or no track. It may be COASTING, TENTATIVE or LOST |
| `may_descend(cls, near=None, track_id=None)` (both by keyword) | True only on a CONFIRMED track no older than 0.5 s. Otherwise false, with `.reason`. Pass the `track_id` of the track being flown to, or the answer is about whichever track of the class is primary now |
| `may_release(cls, near=None, track_id=None)` | True only on a CONFIRMED track no older than 0.3 s. Otherwise false, with `.reason`. `track_id` as above |
| `tracked_target(cls, track_id)` | The track with that id, whichever track is the class's primary one now; None when it is gone or the flag is off |
| `vehicle_velocity()` | The vehicle's newest velocity (east, north, up) from the pose history; None when it is over 0.2 s old or `use_frame_time_pose` is off |
| `pose_at(stamp_ns)` | The vehicle's pose at a past instant, from the pose history (`use_frame_time_pose`) |
| `height_agl(ground_below_home_m=0.0)` | `value_m` and `source`: `rangefinder` when its ground is healthy, else `baro`, which is the height above home plus `ground_below_home_m` (how far the ground here is known to be below home; 0 when nothing is known, and then it is not the height above the ground wherever the ground is not at home's height). None when the newest pose is over 0.5 s old; `pose_age_s` says how old it was. Needs no flag; with `use_height_estimator` on the answer is the height estimator's (below) |
| `actuation_time_ns()` | Now + 0.08 s, the time the estimator's outputs are given for |
| `latlon_to_local_enu(lat, lon)` | (east, north) in MAVROS's local frame, from the GPS fix and the local position of the same moment. None without a fix under 2 s old and a pose of its moment (ask again next tick). Needs no flag |

`centre_on_estimator` (declared, off, in the eight sequences that fly a drop or delivery;
needs `use_target_estimator`) makes the drop and delivery missions centre on the
estimator's bullseye track instead of the latest raw detection (`centring.py`, CEN-01).
The error steered on is `target − R(yaw) · payload offset − vehicle`: the payload's
release point (`payload_offset_m` of the mission's `airframe` in argus_camera's
`robot.yaml`, forward, left, up of the flight controller) is put over the target, not the
camera. The 2026 low-pass filter and deadband are not applied; the 2026 gain, speed cap,
tolerance and dwell still are, and so do the 2026 limits `min_projection_altitude_m` and
`max_projection_distance_m`. A CONFIRMED or COASTING track is steered on, but only while
`may_descend` says yes for that track does the mission descend or count itself centred, so
it never starts its fixed column or the dwell before a release on a point it only
remembers. A TENTATIVE or LOST track, or none, is a lost target, for acquisition as well
as tracking, whatever the raw detector says: the mission's 2026 grace time and recovery
climb follow. One approach flies to one track: the first taken is the one nearest the
mission's own GPS target, it is kept to by its id (`tracked_target`), and if it is lost
another track of the class is not taken over until the recovery climb has started. From
the fixed column down, the 2026 code is unchanged and asks nothing of the estimator. An
airframe that is not `verified` stops the executor at start-up.

`centring_retune` (declared, off, in the same eight sequences; needs
`centre_on_estimator`) replaces the 2026 gain and speed cap in that centring with the 2027
controller (`centring_controller.py`, CEN-02): the error less a soft deadband of 0.03 m,
times Kp 0.6, capped at clamp(0.08 × h, 0.25, 1.0) m/s where h is `height_agl()` (the
lowest cap when the height is not known, and never over `safety_limits`' cap for
centring), and changed by no more than 0.5 m/s². Before that the error is given a lead,
which the spec does not have: it is taken for where the vehicle will be 0.5 s on at the
velocity `vehicle_velocity()` reports, which damps the loop's swing against the vehicle's
slow answer to a velocity command. A height from the barometer is taken as 2 m lower than
it says for the cap, since it is above home and not above the ground. On a lost target
the command is zero at once, as in 2026, and the next one starts from rest. The numbers
are the defaults in code; a 2027 sequence may set its own under `centring_kp`,
`centring_soft_deadband_m`, `centring_speed_cap_per_m_height`,
`centring_speed_cap_min_mps`, `centring_speed_cap_max_mps`, `centring_slew_mps2` and
`centring_lead_s`, and a value that is not usable (a Kp over 1.0, or over 0.6 with a lead
under 0.5 s; a lead over 1 s) stops the executor at start-up. If the velocity stops coming the
lead is left out, the gain is held to 0.6 and the log says so. The 2026 keys
(`centering_gain_mps_per_m`, `max_centering_speed_mps`) are not read with the flag on;
`centering_tolerance_m` and `centering_dwell_s` still decide "centred".
`python3 tools/cen_loop_sim.py` runs the same class in a simulated loop, to try a gain
before it is flown.

`use_ring_fit` (declared, off, in the same eight sequences; needs
`use_target_estimator`) makes the estimator listen to the near-field ring fit
(`drone_target_cv`'s `ring_fit` node, CEN-13) on `/perception/ring_fit/detections` as
well as to the detectors of `estimator_detectors`. Its detections are `bullseye`s like
the box detector's and go to the same tracks; they are weighed by the ring fit's own
figure for its centre's noise (2 px, `estimator_feed.DETECTOR_PIXEL_SIGMA_PX`) instead of
by the size of their box. Near the ground, where the box detector's box is cut by the
image's edge and is not used, the ring fit is what keeps the track CONFIRMED. The node has
its own flag, `features.ring_fit`, and is started only by a launch given `ring_fit:=true`.

`drop_release_gates` (declared, off, in the four sequences that fly a drop; needs
`use_target_estimator`, `centre_on_estimator` and `use_height_estimator`) replaces the drop's fixed GPS column and
timed hover with FINAL_ALIGN (`release_gates.py`, and `drop_final_align.py` in
`drone_mission_demo`, CEN-04). From the handoff height the mission descends on the
estimator to the release height for the sensor that gives `height_agl()`: 2.4 m ± 0.2 on
the rangefinder, 3.0 m ± 0.2 on the barometer. It releases, at once and where it is, when
all of these have held together for 0.5 s: aim error ≤ 0.10 m, the track's sigma ≤ 0.08 m,
ground speed ≤ 0.15 m/s, tilt ≤ 6°, `may_release` (a CONFIRMED track hit within 0.3 s),
height inside the band. Under 2.0 m it climbs straight back before anything else. After
25 s without the gates holding it looks for the target again, once; after a second 25 s (or
the first, if the approach has used every recovery attempt) it releases anyway if it is
within 0.5 m on a track `may_release` accepts and not under the lowest release height
(2.0 m on the rangefinder, 2.8 m on the barometer), and otherwise fails, which the
sequence's failure policy handles. The numbers are constants, not config. The release is
still the `DROP_PAYLOAD` state and `fake_drop` still decides whether the actuator is
commanded. While the release command cannot be sent the mission stays on the gates,
steering; a command that fails is sent again only after the gates have held again.

`survey_search` (declared, off, in `search_pattern_demo.yaml`; needs `use_height_estimator`)
turns on the `search_pattern` mission (`drone_mission_demo/missions/search_pattern_mission.py`,
LOC-3); with it off the mission is skipped. The mission plans its tracks with the coverage
planner from `search_area` (corners as latitude and longitude), climbs to the survey height
above the ground, sets `survey_speed_mps` with DO_CHANGE_SPEED, starts the localisation
pipeline through `/loc/set_enabled` (`context.set_localisation_enabled`; flown without it
when the service is not there, unless `localisation_required`), and flies the tracks at one
yaw with the height guard asked every tick. It ends with SUCCESS when the tracks are flown,
when `time_budget_s` (600 s) runs out, or when the guard ends the survey low (after the
climb back); a survey that ends with no height above the ground fails, as does one whose
speed command is refused or unanswered three times. Heights are asked for with
`ground_below_home_m`; a corner of the area more than 1 km away, a plan with a leg outside
`flight_boundary`, or a way to the first track that leaves it, fails the mission before it
moves; `speed_after_mps`, when given, is asked of the flight controller when the survey is
over. Its configuration is checked when the sequence is built, and its modules are loaded
only then, with the flag on.

`survey_height.py` (LOC-1) holds the localisation survey's height and its guard.
`survey_height(config)` is 35 ft (10.668 m), 30 ft (9.144 m) when the sequence sets
`survey_height_judged_from_log: true`, or the sequence's own `survey_height_m` (9.02 to
11.5 m). `SurveyHeightGuard.update(context.height_agl(), now)`, asked from the start of
the survey's first track, answers CONTINUE, RECOVER (under 8.22 m: the
survey waits and the vehicle climbs back, until within 0.3 m of the survey height) or ABORT
(under 7.77 m, or 5 s with no height: the survey is over for good and the climb comes
first). A height that is not the rangefinder's counts 0.5 m lower, and the climb asked for
on it is 0.5 m higher. The rules' floor, 7.62 m, is `safety_limits.SURVEY_MIN_HEIGHT_M`. Nothing calls it yet (LOC-3
does); there is no flag.

`coverage_planner.py` (LOC-2) plans the localisation survey: `plan(polygon, height, camera)`
gives a lawnmower of east/north waypoints over the search area, tracks at most
0.8 W - 0.85 m apart (W the footprint across the track; 15.0 m from 30 ft at 4K, so
11.15 m), along the direction that needs the fewest, the outer tracks W/2 - 0.85 m in from
the edge, each track as long as the polygon reaches anywhere in its swath. `yaw_deg` is the
direction the image's top must point so its long side stays across the tracks (the
vehicle's yaw for a camera mounted top-forward; a turned mount is the caller's to allow
for). No plan is made under 7.62 m or over 4 m/s; given the flight boundary, waypoints are
kept 5 m inside it and a leg that still leaves it is counted in the plan. `coverage(plan, polygon)`
gives the share of the area in at least one and at least two frames. `plan_wide_pass` and
`plan_revisit` are the two-phase option: everything once on wider tracks, then short passes
over what the first pass saw. Nothing flies it yet (LOC-3 does); there is no flag.

`smart_reacquire` (declared, off, in the eight sequences that fly a drop or a delivery;
needs `use_target_estimator` and `centre_on_estimator`) replaces the 2026 recovery climb
(`reacquire.py`, and `smart_reacquire.py` in `drone_mission_demo`, CEN-06). When the track
the mission was flying to is LOST, the mission does not wait its own
`target_loss_grace_s`: it goes to the height (3 sigma + 2.5 m) / k over the track's last
estimate, where k is half the camera footprint's short side per metre of height (0.463
for the drop camera mode; taken from the mission's calibrated camera), never lower than it
is and never over `max_recovery_altitude_m`. From arriving there (within
`recovery_alt_tolerance_m` and 1 m of the estimate) it holds up to 10 s for the class's
primary track to be CONFIRMED and freshly hit, then goes to ACQUIRE_TARGET as in 2026;
otherwise the mission fails and its failure policy acts. Each look is one of
`max_recovery_attempts`. A mission that never had a track flies the 2026 recovery.

`delivery_soft_set_down` (declared, off, in the four sequences that fly a delivery; needs
`use_target_estimator`, `centre_on_estimator` and `use_height_estimator`) replaces the
delivery's fixed GPS column with FINAL_SET_DOWN (`set_down.py`, and `delivery_set_down.py`
in `drone_mission_demo`, CEN-05). From the handoff height the mission goes on down on the
estimator's track, easing to 0.12 m/s. On the tick it is both low and centred on a track
that may be descended on, it takes the hold: the GPS point it is over, no more steering,
and its height setpoint lowered at 0.12 m/s. Low is `height_agl()` at 0.6 m or less (a
height that is not the rangefinder's counts as 0.5 m lower than it says), or the flight
controller's height within 0.05 m of `min_projection_altitude_m`, under which the mission
takes no target. Contact is the rangefinder at or below the mission's `package_height_m`,
a vertical speed within 0.05 m/s of zero, and the throttle 15% under what it was while
descending, together for 0.3 s, each no older than 0.2 s; it is asked only if the hold was
taken on the rangefinder's height. The flight controller's "landed" for
`touchdown_dwell_s` is contact too, before or in the hold. Then the release (still the
`GROUND_DWELL` state, `fake_drop` as before; refused above 1.82 m unless the flight
controller says landed), 1 s, and the 2026 relaunch. Twenty seconds in the hold without
contact, or ten with no height, fail the mission. `package_height_m` is what the
rangefinder reads with the package resting on the ground; with the flag on and no usable
value the sequence does not build, so the executor stops on the ground. The throttle is
`context.throttle()`, from `/mavros/vfr_hud` (the `vfr_hud` plugin in
`config/mavros/argus_mavros.yaml`, 20 Hz by the stream keeper); the raw rangefinder
reading is `context.rangefinder_range()`. Both are None with their flags off.

`use_height_estimator` (declared, off, in the eight sequences that fly a drop or
delivery; needs `use_frame_time_pose`) makes `height_agl()` the answer of the height
estimator (`height_estimator.py`, CEN-03), fed by its own subscription to
`/mavros/rangefinder/rangefinder`. The height is the rangefinder's only when its readings
are inside the sensor's range (0.1 to 12 m, the TFmini's: MAVROS's topic gives limits of 0
and 1000 m, so they are held in code), no older than 0.5 s, taken with the vehicle tilted
15 degrees or less (each corrected by the cosine of the tilt at its own moment), within
0.5 m of each other and within 5 m of where the ground is expected; and the vehicle is
tilted 15 degrees or less now, the sensor's last three readings were not all out of range,
and its reading has not stayed the same through 0.2 m of the vehicle's own climb or descent
(a stuck sensor). A reading within 0.05 m of either end of the range counts as out of
range, so the readings used are from 0.15 to 11.95 m. Otherwise it is the
height above home plus `ground_below_home_m`, `source` is `baro` and `reason` says why. A
weak return is ArduPilot's to judge: its driver reports one as out of range. With the flag
off, the rangefinder path is projection v2's (25 degrees, the message's own limits).

The descend and release rules are answered there so that no mission has to carry them. A CONFIRMED track starts COASTING 0.3 s after its last hit, so in practice both rules say no from 0.3 s on; the 0.5 s limit for descending is the spec's and is never the one that bites.

Node flags are not set here. They are ROS parameters named `features.<flag>` on the node
that uses them, declared with default `false`. Never put `features:` at the top level of a
params file: rcl will not parse it and no node in that launch will start.

## Running

This package is usually launched through a mission package that supplies:
- `mission_modules`
- `sequence_file`
- any ROS params for the executor and shared takeoff service

Direct executable:

```bash
ros2 run drone_mission_core mission_executor --ros-args \
  -p mission_modules:="[drone_mission_demo.missions]" \
  -p sequence_file:=/absolute/path/to/sequence.yaml
```

## Design Notes

- Missions are modular Python objects, not standalone ROS nodes.
- The executor owns shared control surfaces so missions do not compete over publishers or service clients.
- The framework is intentionally light. It avoids ROS2 actions for now because the current repo already uses timer-driven state machines successfully and this keeps the code easier for the team to follow.
