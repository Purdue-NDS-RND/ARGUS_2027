# drone_utils

Shared ROS 2 utility nodes and Python helpers used across the CUASC 2026 workspace. Provides reusable services for takeoff, gimbal control, and Gazebo object spawning so mission packages such as `drone_mission_demo`, `drone_control`, and `drone_demo` don't duplicate common functionality.

## Nodes

### `simple_takeoff_service`

A timer-driven takeoff service that handles MAVROS stream setup, mode switching, arming, and takeoff in sequence. Every mission node in the workspace calls this service — stream setup happens here so nothing else needs to worry about it.

**Service provided:**
| Service | Type | Description |
|---|---|---|
| `drone_utils/takeoff` | `mavros_msgs/CommandTOL` | Accepts a target altitude (0 → uses default param). Triggers the full takeoff sequence. |

**Sequence (timer-driven, non-blocking):**
1. Wait for MAVROS connection (`/mavros/state.connected`)
2. Request all MAVLink streams from the FCU via `/mavros/set_stream_rate`
3. Explicitly request MAVLink `EXTENDED_SYS_STATE` (message `245`) via `/mavros/set_message_interval`
4. Wait for required telemetry topics to publish at least once
5. Switch to GUIDED mode
6. Arm the vehicle (with configurable retry limit)
7. Send takeoff command

**Why stream setup lives here:**
On real hardware (Pixhawk over USB) the FCU does **not** automatically stream telemetry the way SITL does. Without an explicit stream request, topics like `/mavros/local_position/pose`, `/mavros/imu/data`, and `/mavros/extended_state` may never publish. `simple_takeoff_service` now sends both the legacy `StreamRate` request and an explicit `MessageInterval` request for `EXTENDED_SYS_STATE` because the old stream-group API alone was not enough to make `/mavros/extended_state` appear on hardware. Since every mission must take off, gating streams here means zero changes to downstream mission nodes.

In SITL the topics are already flowing, so the health checks pass instantly and there is no extra delay.

**Parameters:**
| Parameter | Type | Default | Description |
|---|---|---|---|
| `default_takeoff_altitude_m` | double | 20.0 | Altitude when caller passes ≤ 0 |
| `guided_mode_name` | string | `"GUIDED"` | ArduPilot mode string |
| `mode_retry_s` | double | 2.0 | Seconds between mode-switch retries |
| `arm_retry_s` | double | 5.0 | Seconds between arm retries |
| `max_arm_attempts` | int | 5 | Abort after this many failed arm attempts |
| `takeoff_retry_s` | double | 5.0 | Seconds between takeoff retries |
| `loop_rate_hz` | double | 2.0 | Timer frequency |
| `stream_rate_hz` | int | 20 | MAVLink stream rate (Hz) requested from FCU |
| `extended_state_rate_hz` | double | 2.0 | Explicit MAVLink rate request for `EXTENDED_SYS_STATE` (message `245`) |
| `required_topics` | string[] | `["/mavros/local_position/pose", "/mavros/imu/data", "/mavros/extended_state"]` | Topics that must publish before arming proceeds |

**Adding more required topics:**

1. Add the topic name and its message type to `TOPIC_TYPE_MAP` at the top of `simple_takeoff_service.py`:
   ```python
   TOPIC_TYPE_MAP: dict[str, type] = {
       "/mavros/local_position/pose": PoseStamped,
       "/mavros/imu/data": Imu,
       "/mavros/global_position/global": NavSatFix,   # already in the map
       "/mavros/extended_state": ExtendedState,       # required for touchdown-capable missions
   }
   ```
2. Add the topic name to `required_topics` in your mission YAML.

`/mavros/extended_state` is now part of the default readiness contract because
missions such as package delivery and live landing tests depend on valid
`landed_state` data before they will begin touchdown logic.

## Manual Stream Preflight

`set_stream_rate.sh` is the manual hardware preflight helper:

```bash
./set_stream_rate.sh [stream_rate] [extended_state_rate]
```

It now sends the same two requests used by `simple_takeoff_service`:
- service: `/mavros/set_stream_rate`
- `stream_id: 0`
- `message_rate: <stream_rate>`
- `on_off: true`
- service: `/mavros/set_message_interval`
- `message_id: 245`
- `message_rate: <extended_state_rate>`

It only requests streams. It does **not** verify that any specific topic
actually became live afterward.

`preflight_checks.sh` is the manual verification helper:

```bash
./preflight_checks.sh [stream_rate] [extended_state_rate] [topic_timeout_s]
```

It first verifies that installed YAML configs exactly match the source configs
for:
- `src/drone_mission_demo/config`
- `src/drone_target_cv/config`

Then it calls `check_mavros_streams.sh`, which requests MAVROS streams and
waits for:
- `/mavros/imu/data`
- `/mavros/local_position/pose`
- `/mavros/extended_state`

It fails if `/mavros/extended_state` still reports `landed_state=0`
(`UNDEFINED`).

For manual real-hardware workflows such as `drone_live_tests`, verify the
landing-state stream explicitly before launching missions:

```bash
./preflight_checks.sh 20
```

You want `/mavros/extended_state` to publish a non-`UNDEFINED` `landed_state`
value before takeoff. In SITL this often works automatically; on hardware it
must be treated as part of the preflight telemetry contract.

**Example call:**
```bash
ros2 service call drone_utils/takeoff mavros_msgs/srv/CommandTOL "{altitude: 25.0}"
```

---

### `gimbal_point_service`

Thin proxy that exposes a friendlier service name for pointing the gimbal.

**Service provided:**
| Service | Type | Description |
|---|---|---|
| `drone_utils/set_gimbal_point` | `mavros_msgs/GimbalManagerPitchyaw` | Set gimbal pitch/yaw in degrees |

Forwards the request to the MAVROS gimbal manager at `/mavros/gimbal_control/manager/pitchyaw`.

**Example — point camera straight down:**
```bash
ros2 service call drone_utils/set_gimbal_point mavros_msgs/srv/GimbalManagerPitchyaw \
    "{pitch: -90.0, yaw: 0.0, pitch_rate: .nan, yaw_rate: .nan, flags: 0}"
```

---

### `session_logger`

Mission-session logger that creates one timestamped folder per run and records
controller commands plus image snapshots.

**Default outputs:**
| Path | Description |
|---|---|
| `~/cuasc_logs/mission_YYYYmmdd_HHMMSS/metadata.json` | Session metadata and subscribed topics |
| `~/cuasc_logs/mission_YYYYmmdd_HHMMSS/command_velocity.csv` | `/mavros/setpoint_raw/local` velocity commands |
| `~/cuasc_logs/mission_YYYYmmdd_HHMMSS/images/camera/` | Camera snapshots as JPG from the configured camera topic |
| `~/cuasc_logs/mission_YYYYmmdd_HHMMSS/images/annotated/` | `/target_cv/annotated` snapshots as JPG |
| `~/cuasc_logs/mission_YYYYmmdd_HHMMSS/images/mask/` | `/target_cv/mask` snapshots as PNG |

The package-drop and package-delivery mission launches start this node by default.
Disable it with `log_session:=false`. Logger settings come from the
`session_logger` block in the selected mission params YAML.

---

### `stream_keeper`

Keeps the MAVLink message rates the 2027 stack needs: `LOCAL_POSITION_NED` 30 Hz, `ATTITUDE` 50 Hz, `RANGEFINDER` 20 Hz, `GPS_RAW_INT` 5 Hz and `BATTERY_STATUS` 2 Hz, plus three at the rates the 2026 start-up gives them: `GLOBAL_POSITION_INT` 20 Hz, `EXTENDED_SYS_STATE` 2 Hz and `SYS_STATUS` 2 Hz. It is not part of the 2026 start-up: `config/mavros/argus_mavros.launch.py` starts it with `stream_keeper:=true`.

- Each rate is requested as `MAV_CMD_SET_MESSAGE_INTERVAL` through `/mavros/cmd/command` once when MAVROS connects, and after that only when there is a reason: a reboot (MAVROS reconnecting, or the flight controller's clock on `/mavros/timesync_status` going back by more than a second), or the rate it measures on that message's topic falling under 80% of what was asked over 5 s. That second case is what undoes `simple_takeoff_service` asking for all streams at 20 Hz. There is no periodic repeat.
- It sends nothing while a mission is in a release or actuation state: `package_drop`'s `FINAL_FIXED_DROP_COLUMN` and `DROP_PAYLOAD`, `package_delivery`'s `FINAL_FIXED_COLUMN_DESCENT` and `GROUND_DWELL` (the final column counts because the delivery sends its release one tick into the ground dwell), `takeoff`'s `TAKING_OFF` and `WAITING_FOR_ALTITUDE`, and any state whose name contains `RELEASE`, `GRIPPER`, `ENGAGE` or `ACTUAT`. A hold lapses if nothing at all is heard from the executor for 60 s. It learns the state from the `[mission] State: OLD -> NEW` lines the missions already log on `/rosout`; a test drives the real missions to pin that. MAVROS serves every command in turn, the gripper's included, and waits up to 5 s for each acknowledgement, so a request of ours in that queue could delay a release.
- One request is out at a time, after a refused or unanswered one it asks for nothing for 5 s, and a message that stays low after three accepted requests (a sensor that is not fitted) is left alone until it does arrive at its rate, or the next reboot. It does not use MAVROS's `set_message_interval` service: that one blocks the group that also serves `set_mode` for a second per call.
- The table and the timing are in `drone_utils/stream_keeper.py`, with no ROS in it; `stream_keeper_node.py` is the wrapper.

## Library Module

### `sdf_objects.py`

Pure-Python helpers for generating Gazebo SDF models at runtime. Not a ROS node — imported by other nodes (e.g. `target_spawner` in `drone_control`) or runnable as a standalone script.

**Public functions:**

| Function | Description |
|---|---|
| `build_red_white_target_sdf(model_name, size, num_rings)` | Red/white bullseye target (~5 m default) for package drop missions |
| `build_bw_target_sdf(model_name, size, digit)` | Black/white X-pattern GCP with a 7-segment digit (0–9) |
| `spawn_sdf(sdf_str, model_name, x, y, z, world_name)` | Spawn any SDF string into a running Gazebo sim via the `gz` CLI |

**Standalone usage:**
```bash
python3 -m drone_utils.sdf_objects   # spawns a test red/white target at (5, 5, 1)
```

---

## Entry Points

Registered in `setup.py`:

```
simple_takeoff_service = drone_utils.simple_takeoff_service:main
gimbal_point_service   = drone_utils.gimble_point_service:main
session_logger         = drone_utils.session_logger:main
```

Run individually:
```bash
ros2 run drone_utils simple_takeoff_service
ros2 run drone_utils gimbal_point_service
ros2 run drone_utils session_logger
```

Or launch via a mission launch file that loads the matching YAML config (e.g. `mission_params.yaml`).

## Dependencies

- `rclpy`
- `mavros_msgs`
- `geometry_msgs`
- `sensor_msgs`
