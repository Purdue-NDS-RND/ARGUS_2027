# Mission simulator baseline (2026-11-04)

The first end-to-end runs of the mission simulator (`sim/run_sim.sh`, see `sim/README.md`) with our unchanged code and YAML. Every stage passed.

| Stage | Result |
| --- | --- |
| 1. `/mavros/state` connected | Yes, MAVROS on UDP 14551 |
| 2. `/camera/image` rate | 640x360, not 30 Hz: 12.5 Hz on the ground (Gazebo at about 0.45x real time), about 22 Hz reaching `target_cv` in flight (Gazebo at 0.98x). See below. |
| 3. Target detected | Yes: 1931 of 2010 `target_detection` messages in run 3b contained the target |
| Camera orientation | East and north checks both PASS, with offsets within 0.01 of the pinhole prediction |
| 4. Full mission | Completed 4 of 4 flights (runs 1, 2, 3a on fresh sims; 3b relaunched in the same sim); touchdown 0.11, 0.08, 0.03 and 0.09 m from the target centre; landed and disarmed at home every time |

Run 3 was flown twice. The first run 3 was a relaunch while run 2's launch was still running, so two copies of `target_cv`, `mission_executor` and `simple_takeoff_service` ran side by side under the same names. Its results are discarded. Runs 3a and 3b below replace it.

## Setup

- **Machine:** club drive (Lenovo 82BG laptop), Intel i7-1195G7 (4 cores / 8 threads), Intel Iris Xe graphics, 15 GiB RAM, Ubuntu 24.04 host.
- **Image:** `argus-sim:humble`, built from `sim/Dockerfile` on top of `ardupilot-gz:4.6.3`. It adds ROS 2 Humble, MAVROS 2, and `ros-humble-ros-gzharmonic` 0.244.12. 7.17 GB.
- **Code:** branch `sim/humble` from `main` at `0367274`. No changes under `src/`.
- **Config:** `package_delivery_demo.launch.py` defaults: `sim_target_mission.yaml` and `package_delivery_demo.yaml`.
- **Sim:** world `argus_runway` with the Gazebo GUI open (the normal way to run it). SITL home is the world origin, −35.363262, 149.165237.
- **Camera:** 640x360, `update_rate` 30, horizontal FOV 1.37635 rad, pointing straight down.

## Before: the club sim

```
$ cd ~/Drone_Sim_Docker && make run      # then, from another terminal:
$ docker compose exec -T simulator python3 -c "...wait_heartbeat on udpin:127.0.0.1:14551..."
heartbeat on 14551: HEARTBEAT sys 1
pos -35.3632621 149.1652374
```

Container healthy; Ogre reports `GL_RENDERER = Mesa Intel(R) Xe Graphics (TGL GT2)` (hardware rendering). SITL's position is the Gazebo world origin, not CMAC.

Real-time factor of the unmodified club sim on this laptop:

```
$ docker exec ardupilot-gz-sim bash -c 'gz topic -e -t /world/iris_runway/stats -n 1 | grep real_time_factor'   # 3 samples
real_time_factor: 0.42257818330258373
real_time_factor: 0.33566159403677037
real_time_factor: 0.4507936221718335
```

## Build (rosdep, colcon)

```
$ sim/run_sim.sh --build
...
#9 15.44 #All required rosdeps installed successfully

$ sim/run_sim.sh --exec bash -c 'rosdep update --rosdistro humble >/dev/null; rosdep check --rosdistro humble --from-paths src --ignore-src --skip-keys ament_python'
All system dependencies have been satisfied

$ sim/run_sim.sh        # builds at start-up; sim/.state/logs/latest/colcon.log
Summary: 6 packages finished [4.60s]
```

`--skip-keys ament_python`: four `package.xml` files list `ament_python` as a `buildtool_depend`. That is a build type, not a rosdep key, so rosdep fails on it, but colcon doesn't use it. No source changes were needed to build.

## Start-up

```
$ sim/run_sim.sh
== Spawning the delivery target
Spawned 'delivery_target' in 'argus_runway' at x=10.07 m east, y=15.02 m north of home, from package_delivery_demo.yaml (-35.3631271, 149.1653479)
...
   MAVROS connected.
   OK: SITL home -35.3632620, 149.1652370 is 0.00 m from the world origin
   home set after 28s.
   pre-arm checks pass after 66s.
== READY.
```

`run_sim.sh` waits for ArduPilot's pre-arm checks before it prints READY. On the first attempt the mission launched as soon as MAVROS connected. The takeoff service's five arm attempts (25 s) all failed with `Arm: Gyros inconsistent` / `Arm: Accels inconsistent`, and the IMU was steady on the ground (|gyro| 0.0052 rad/s, |accel| 9.807 m/s² constant). ArduPilot's SYS_STATUS pre-arm health bit tracks this. In 5 starts it was set 63 to 69 s after start, and the mission armed first time on every launch after READY.

## Stage 1: MAVROS connected

```
$ sim/run_sim.sh --exec ros2 topic echo --once /mavros/state
header:
  stamp:
    sec: 1793825671
    nanosec: 26521034
  frame_id: ''
connected: true
armed: false
guided: false
manual_input: true
mode: STABILIZE
system_status: 3
---
```

## Stage 2: camera rate

```
$ sim/run_sim.sh --exec timeout 25 ros2 topic hz /camera/image
$ sim/run_sim.sh --exec bash -c 'gz topic -e -t /world/argus_runway/stats -n 1 | grep real_time_factor'   # 3 samples
```

| Resolution | Gazebo GUI | `/camera/image` | Real-time factor |
| --- | --- | --- | --- |
| 1280x720 | open | 8.6 Hz (`average rate: 8.622`) | 0.497, 0.481, 0.467 |
| 640x360 | open | **12.2 Hz** (`average rate: 12.289`) | 0.425, 0.433, 0.582 |
| 640x360 | `HEADLESS=1` | 17.8 Hz (`average rate: 17.783`) | 0.683, 0.480, 0.642 |

At 1280x720, `top` showed the Gazebo server at 125% CPU, the GUI at 100% and MAVROS at 133%, on 4 cores. The camera publishes 30 Hz of *sim* time, so the wall-clock rate is at most 30 × RTF, about 15 Hz here. The club sim without ROS runs at the same RTF (0.34 to 0.45 above), so the limit is this laptop, not what we added.

The table above was measured with the drone sitting on the ground. Re-measured on the run 3 sim before take-off (640x360, GUI open). `cam_rate.py` is a throwaway raw subscriber, not committed, that counts images without deserialising them and compares their header stamps with wall time:

```
$ sim/run_sim.sh --exec timeout 25 ros2 topic hz /camera/image
average rate: 12.572
$ sim/run_sim.sh --exec python3 cam_rate.py 20
250 images (640x360) in 20.0 s wall: 12.5 Hz wall-clock
header stamps span 9.21 s sim over 19.98 s wall: 27.0 Hz sim time, sim/wall ratio 0.46
```

In flight, Gazebo runs much faster. SITL's `time_boot_ms` against MAVProxy's wall-clock stamps in `mavlink.tlog`, in 20 s windows:

| Run | On the ground (before take-off, after landing) | In the air |
| --- | --- | --- |
| 3a | 0.44, 0.55 | 0.90 to 0.98 |
| 3b | 0.60 | 0.98 to 0.99 (0.68 to 0.80 in windows that include a landing) |

`target_cv` sees that faster rate. Run 3b published 21.8 detection messages per second while detection was on. That's one per image, and most gaps between messages were 30 to 40 ms. There was exactly one `target_cv` node (`ros2 node list` during the run). So in flight the mission gets about 22 Hz at 640x360, against the 12 Hz measured on the ground. 1280x720 was measured only on the ground (8.6 Hz).

The sim ships at 640x360 with the GUI open. `HEADLESS=1` is documented as the faster option. The mission scales `camera_fx_px` to the image width, so no YAML changes.

`/camera/camera_info` matches: at 1280x720, `k[0]` = 778.33 = 2334.99647 × 1280/3840.

## Camera orientation

`sim/orientation_check.py` flies the delivery mission with its target set to home, so the drone climbs straight up over the spawn point facing north (yaw 90° ENU). A red target sits 4 m east, or 4 m north, of the spawn point. The tool records the first detection and the first centring velocity setpoint on `/mavros/setpoint_raw/local` (ENU). One direction per fresh sim start:

```
$ SPAWN_TARGET=0 sim/run_sim.sh
$ sim/run_sim.sh --exec python3 sim/orientation_check.py --direction east

Target 4.0 m east of home
  first detection:        x_norm +0.347, y_norm +0.000  (alt 13.98 m, local E 0.00 N 0.00 m, yaw 90.0 deg ENU)
  detection at setpoint:  x_norm +0.331, y_norm +0.000
  first centring setpoint: v_east +0.450 m/s, v_north +0.000 m/s, v_up +0.000 m/s  (alt 14.84 m, local E 0.00 N 0.00 m, yaw 90.0 deg ENU)
  expected: x_norm > 0 (right of centre), v_east > 0
  detection OK, setpoint OK -> PASS

$ sim/run_sim.sh --down; SPAWN_TARGET=0 sim/run_sim.sh
$ sim/run_sim.sh --exec python3 sim/orientation_check.py --direction north

Target 4.0 m north of home
  first detection:        x_norm +0.006, y_norm -0.628  (alt 13.95 m, local E 0.00 N 0.00 m, yaw 90.0 deg ENU)
  detection at setpoint:  x_norm +0.006, y_norm -0.594
  first centring setpoint: v_east +0.000 m/s, v_north +0.450 m/s, v_up +0.000 m/s  (alt 14.83 m, local E 0.00 N 0.00 m, yaw 90.0 deg ENU)
  expected: y_norm < 0 (above centre), v_north > 0
  detection OK, setpoint OK -> PASS
```

The detection sits right of centre for the east target and above centre for the north target. Each first setpoint points at its target, capped at `max_centering_speed_mps` 0.45. The offset magnitudes also match a pinhole camera with the mission's intrinsics:

| Target | Predicted | Measured |
| --- | --- | --- |
| 4 m east at 13.98 m | x_norm = (4 / 13.98) × 389.17 / 320 = 0.348 | 0.347 |
| 4 m north at 13.95 m | y_norm = −(4 / 13.95) × 389.68 / 180 = −0.621 | −0.628 |

389.17 and 389.68 are `camera_fx_px` and `camera_fy_px` scaled to 640x360.

## Stage 3: target detected

Run 3b: the relaunch (see stage 4), with detections recorded alongside. Each command runs in its own process group so it can be stopped like Ctrl+C:

```
$ sim/run_sim.sh --exec bash -c '
    setsid ros2 topic echo --csv /drone_package_drop/target_detection geometry_msgs/msg/PointStamped > run3b_detections.csv &
    setsid ros2 launch drone_mission_demo package_delivery_demo.launch.py > run3b_mission.log 2>&1 &
    ...'
[target_cv]: Image size: 640x360
[package_delivery] Freezing touchdown column at (-35.3631263, 149.1653478)
[package_delivery] Fake delivery complete
Mission sequence complete
```

Summary of `run3b_detections.csv` (x/y are normalised, −1 to 1; (−1, −1) means no target):

```
2010 detection messages over 92 s (21.8 msg/s): 1931 with the target, 79 'no target' (-1,-1)
first hit x_norm=-0.016 y_norm=-0.972; target reported over 89 s
last hit  x_norm=+0.159 y_norm=+0.294
```

The first hit is at the top edge of the image (y −0.97) as the drone flies north towards the target. The "no target" messages are before arrival and after the drone climbs away.

## Stage 4: full mission

```
$ sim/run_sim.sh            # terminal 1, wait for READY
$ sim/run_sim.sh --shell    # terminal 2
(argus-sim) $ ros2 launch drone_mission_demo package_delivery_demo.launch.py
```

Mission log, run 1. Runs 2, 3a and 3b went through the same states:

```
[takeoff] State: WAITING_FOR_ALTITUDE -> COMPLETE
Mission 'takeoff' completed
[package_delivery] State: WAITING_FOR_GPS -> TRANSIT_TO_TARGET
[package_delivery] State: TRANSIT_TO_TARGET -> ACQUIRE_TARGET
[package_delivery] State: ACQUIRE_TARGET -> TRACK_AND_DESCEND
[package_delivery] Entering final touchdown column at 6.20 m (threshold 6.00 m, handoff tolerance 0.20 m)
[package_delivery] Freezing touchdown column at (-35.3631261, 149.1653479)
[package_delivery] State: FINAL_FIXED_COLUMN_DESCENT -> GROUND_DWELL
[package_delivery] Holding on target for 5.0 s before relaunch
[package_delivery] Fake delivery complete
[package_delivery] State: GROUND_DWELL -> GUIDED_RELAUNCH
[package_delivery] State: GUIDED_RELAUNCH -> COMPLETE
[rtl] Requesting mode RTL
Mission 'rtl' completed
Mission sequence complete
```

The flight path comes from the MAVProxy telemetry log, `sim/.state/logs/<run>/mavlink.tlog`. Positions are converted to north/east from home with `sim/target_placement.offset_ne_m`, ArduPilot's formula. The landing comes from `/mavros/state` and `/mavros/local_position/pose` after RTL.

| Run | Start | Above 1 m (first to last) | Touchdown on target | After RTL |
| --- | --- | --- | --- | --- |
| 1 | fresh sim | 150 s, max 15.3 m | N 15.13 E 10.07 m: **0.11 m** from the target centre (N 15.02 E 10.07) | disarmed, 0.00 m from home (Gazebo pose −0.003, −0.003 m) |
| 2 | fresh sim | 151 s, max 15.3 m | N 15.09 E 10.04 m: **0.08 m** from the centre | disarmed, 0.00 m from home, 45 s after `Mission sequence complete` |
| 3a | fresh sim | 148 s, max 15.3 m | N 15.03 E 10.04 m: **0.03 m** from the centre | disarmed, 0.00 m from home, 54 s after `Mission sequence complete` |
| 3b | relaunch after 3a, no restart | 153 s, max 15.3 m | N 15.11 E 10.06 m: **0.09 m** from the centre | disarmed, 0.00 m from home, 61 s after `Mission sequence complete` |

Runs 3a and 3b used one sim. Run 3a's tlog numbers come from the part of `mavlink.tlog` up to 3b's start, and 3b's from the rest.

Run 3b shows the mission can be relaunched without restarting the sim, once the drone has landed and disarmed and the previous launch is stopped. The launch stays up after `Mission sequence complete` until Ctrl+C. Between the two runs, 3a's launch got SIGINT on its process group (what Ctrl+C sends). `ros2 node list --no-daemon` then showed only MAVROS, `ros_gz_bridge` and a transform listener. During 3b it showed exactly one each of `target_cv`, `mission_executor` and `simple_takeoff_service`.

On that SIGINT, every mission node logs `process has died ... exit code 1`. That's a shutdown error, not a crash in flight. rclpy's own signal handler has already shut the context down, and then the node's `main()` calls `rclpy.shutdown()` again and gets `RCLError: rcl_shutdown already called`. That's existing node code and is left as is.

## Not measured

- 30 Hz camera: not reachable on a club laptop (stage 2). Not tried on a faster machine.
- 1280x720 in flight: measured only on the ground. With the in-flight speed-up it may be usable; not tried.
- Repeatability beyond 3 runs, wind, or targets at other positions.
