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
