# Mission simulator

One command on a club drive starts ArduCopter 4.6.3 SITL, Gazebo Harmonic, ROS 2 Humble, MAVROS and our code, all in one Docker container. Then our real mission launch flies the package delivery demo: it takes off, finds the red target, descends onto it, fakes the release, climbs, and returns to launch.

```bash
sim/run_sim.sh                     # terminal 1: the whole sim; Ctrl+C stops it
sim/run_sim.sh --shell             # terminal 2: a shell with ROS 2 Humble and our code
ros2 launch drone_mission_demo package_delivery_demo.launch.py
```

The measured baseline (camera rate, detection, a full mission) is in [docs/results/2026-11-04-sim-baseline.md](../docs/results/2026-11-04-sim-baseline.md).

## What's in it

| Piece | Where |
| --- | --- |
| Image `argus-sim:humble`: the club image `ardupilot-gz:4.6.3` plus ROS 2 Humble, MAVROS (with GeographicLib datasets), `ros_gz` for Harmonic, and our rosdep dependencies | `sim/Dockerfile` |
| Start-up order: build, Gazebo, target, camera bridge, SITL, MAVROS | `sim/container/start.sh` |
| Iris with a camera hard-mounted looking straight down | `sim/models/iris_downcam/` |
| World: the club `iris_runway` world with that Iris | `sim/worlds/argus_runway.sdf` |
| `/camera/image` and `/camera/camera_info` bridge | `sim/config/bridge.yaml` |
| Red target, placed at `target_latitude`/`target_longitude` from `package_delivery_demo.yaml` | `sim/spawn_target.py`, `sim/target_placement.py` |
| Camera orientation check | `sim/orientation_check.py` |

**Camera:** 640x360 at a 30 Hz update rate, with a 1.376 rad (78.9°) horizontal field of view.
- That FOV matches the mission's calibrated camera: `camera_fx_px` 2335 at `camera_calibration_width_px` 3840.
- The mission scales fx to whatever width it receives, so the resolution needs no YAML change.
- On a club laptop's Intel Iris Xe, with the drone on the ground, 1280x720 gave only 8.6 Hz and 640x360 gives about 12 Hz. In flight Gazebo runs close to real time, and `target_cv` gets about 22 Hz at 640x360.
- On a faster machine, set `<width>1280</width>` and `<height>720</height>` in `sim/models/iris_downcam/model.sdf`.

**Ports:**
- QGroundControl on the host keeps UDP 14550.
- MAVROS uses the SITL API port, UDP 14551 (`./mavros_boot.sh sim`).
- ROS and Gazebo traffic stays on this machine (`ROS_LOCALHOST_ONLY=1`, `GZ_IP=127.0.0.1`), so two drives on the same Wi-Fi don't see each other.
- The catch: ROS 2 Jazzy on the host can't see the container's topics. Use `--shell` instead.

**Speed:** a club laptop runs Gazebo at about half real time (real-time factor 0.4 to 0.6) while the drone is on the ground, and close to real time (0.9 to 0.99) in the air. The unmodified club sim runs at the same speed. The sim is slow, not broken: ArduPilot runs in lockstep with Gazebo, so it flies correctly, just slower than real time.

## Running it from a fresh club drive

Each step says what to do when it fails. If you're still stuck, post in the team chat: what you ran, what you expected, and the full error.

### 1. Check Docker and the club image

```bash
docker info > /dev/null && echo docker OK
docker image ls ardupilot-gz:4.6.3
```

You should see `docker OK` and one line for `ardupilot-gz:4.6.3`. Every club drive ships with both.

- **`permission denied` / `cannot connect to the Docker daemon`:** run `sudo usermod -aG docker "$USER"`, then log out and back in. (This needs the drive's sudo password: ask the lead if you don't have it.)
- **No `ardupilot-gz:4.6.3` line:** build the club image once (about 30 minutes): `cd ~/Drone_Sim_Docker && make build`.

### 2. Get the code

```bash
git clone https://github.com/Purdue-NDS-RND/ARGUS_2027.git ~/ARGUS_2027   # or: cd ~/ARGUS_2027 && git pull
cd ~/ARGUS_2027
```

### 3. Build the sim image (once, 10 to 20 minutes)

```bash
sim/run_sim.sh --build
```

It ends with `argus-sim:humble  Built`. The image is about 7.2 GB, 1.2 GB more than the club image. You only need to rebuild when `sim/Dockerfile` or a `package.xml` changes. Code changes don't need a rebuild.

- **A download fails (`Could not resolve host`, `404`, `Hash Sum mismatch`):** usually the network or a mirror. Run it again.
- **`no space left on device`:** check `df -h /`; the build needs about 3 GB free. `docker builder prune` frees build cache.
- **`ERROR: the following packages/stacks could not have their rosdep keys resolved`:** someone added a dependency that has no rosdep key. Fix the `package.xml`, or add the key to `--skip-keys` in `sim/Dockerfile` if it isn't a real system package.

### 4. Start the sim

Open a terminal on the desktop (not over SSH), then:

```bash
cd ~/ARGUS_2027 && sim/run_sim.sh
```

It builds our packages, opens the Gazebo window, spawns the target, and starts the bridge, SITL and MAVROS. Then it waits until ArduPilot passes its pre-arm checks, which takes about 70 seconds after start. Wait for:

```
   pre-arm checks pass after 66s.
== READY. In another terminal:
```

Leave this terminal running. Ctrl+C stops everything. Logs go to `sim/.state/logs/latest/`, one file per process.

When it fails:

| You see | Do this |
| --- | --- |
| `ERROR: the club base image ... is missing` | Step 1. |
| `ERROR: the club simulator (ardupilot-gz-sim) is running` | `cd ~/Drone_Sim_Docker && make down`, then retry. They use the same ports. |
| `ERROR: the sim is already running` | Use `sim/run_sim.sh --shell`, or stop it: `sim/run_sim.sh --down`. |
| `DISPLAY is empty` / `no Xauthority cookie` | Run from a terminal in the desktop session. Log out and back in if it persists. To run without a window: `HEADLESS=1 sim/run_sim.sh`. |
| `colcon build failed` | Our code doesn't build. The last 40 lines are printed; the full log is `sim/.state/logs/latest/colcon.log`. |
| `Gazebo did not publish a clock` or a black Gazebo window | `SOFTWARE_RENDERING=1 sim/run_sim.sh` (slow, but rules out the GPU). Look in `sim/.state/logs/latest/gazebo.log`. |
| `Gazebo is not running world 'argus_runway'` / `Gazebo refused to spawn` | The target spawn failed. Check `gazebo.log`. `SPAWN_TARGET=0` starts without it, and you can spawn it by hand later (see "Useful commands"). |
| `MAVROS did not connect to SITL on UDP 14551` | Look at `sitl.log` and `arducopter.log` (SITL started?) and `mavros.log`. Something else on the host may hold 14551: `ss -lunp \| grep 1455`. |
| `WARNING: SITL home ... is N m from the world origin` | The target is no longer where the mission thinks it is. Something changed SITL's home; check `sitl.log` for `--home`. |
| `WARNING: pre-arm checks still failing after 600s` | The last ArduPilot `PreArm` messages are printed. `WIPE_PARAMS=1 sim/run_sim.sh` resets SITL parameters once. |

### 5. Optional: QGroundControl

Start `~/Applications/QGroundControl.AppImage`. It connects on UDP 14550 and shows the vehicle, so you can watch the mission on the map. Don't fly it from QGC while our mission runs.

### 6. Fly the mission

In a second terminal:

```bash
cd ~/ARGUS_2027 && sim/run_sim.sh --shell
ros2 launch drone_mission_demo package_delivery_demo.launch.py
```

The mission log goes through these states:
1. `takeoff` climbs to 15 m.
2. `package_delivery` goes through `TRANSIT_TO_TARGET` → `ACQUIRE_TARGET` → `TRACK_AND_DESCEND` → `FINAL_FIXED_COLUMN_DESCENT` → `GROUND_DWELL` (`Fake delivery complete`) → `GUIDED_RELAUNCH`.
3. `rtl` hands over to ArduPilot RTL.
4. Then `Mission sequence complete`.

On a club laptop the flight takes about 2.5 minutes. ArduPilot then flies home, lands and disarms by itself, which takes about 45 seconds more. The launch keeps running after `Mission sequence complete`; stop it with Ctrl+C.

Watch it in Gazebo, or watch the camera and detections:

```bash
ros2 topic hz /camera/image                              # club laptop: about 12 Hz on the ground, about 22 in flight
ros2 topic echo /drone_package_drop/target_detection     # x, y in -1..1; -1,-1 means no target
```

When it fails:
- **`Max arm attempts reached`:** you launched before `READY`. Wait for it and relaunch.
- **The mission sits in `ACQUIRE_TARGET`, then climbs (`TARGET_NOT_FOUND`):** the camera doesn't see the target.
  - Check `ros2 topic hz /camera/image`.
  - Check that the target is in Gazebo, about 10 m east and 15 m north of the drone's start.
  - Run the orientation check below.
- **The drone drifts away from the target while descending:** the camera mount or FOV is wrong. Run the orientation check below and `scripts/check.sh` (its `sim/test/test_sim_camera.py` checks the camera against the mission config).
- **`process has died`:** the full error is a few lines above it in the launch output. Session logs are in `sim/.state/home/cuasc_logs/`.

To fly again, either relaunch after the drone has landed and disarmed, or restart the sim for a clean start. Before relaunching, stop the previous launch with Ctrl+C and check that `ros2 node list` no longer shows `target_cv` or `mission_executor`. Otherwise the old nodes keep running next to the new ones under the same names, and two `target_cv` nodes publish on the same detection topic.

### 7. Stop

Press Ctrl+C in terminal 1, or run `sim/run_sim.sh --down` from anywhere.

## Changing code

The repo is mounted into the container, so edits show up immediately. Every `sim/run_sim.sh` start rebuilds the packages (about 5 seconds). To rebuild while the sim is running, from the `--shell`:

```bash
sim/container/build_ws.sh                                   # all packages
sim/container/build_ws.sh --packages-select drone_target_cv # one package
```

The container build lives in `sim/.ws/`, separate from any `build/` and `install/` in the repo root, because the host is Jazzy and the container is Humble.

## Useful commands

```bash
sim/run_sim.sh --exec ros2 topic echo --once /mavros/state   # one command, no shell
sim/run_sim.sh --exec python3 sim/spawn_target.py --remove   # delete the target
sim/run_sim.sh --exec python3 sim/spawn_target.py            # put it back
sim/run_sim.sh --exec python3 sim/spawn_target.py --east 4 --north 0 --name extra_target
```

Start-up options (environment variables):

| Variable | Effect |
| --- | --- |
| `HEADLESS=1` | No Gazebo window. Faster: the camera reaches about 18 Hz on a club laptop. |
| `SOFTWARE_RENDERING=1` | No GPU. Slow; for diagnosing rendering problems. |
| `SPAWN_TARGET=0` | Don't spawn the delivery target. |
| `SKIP_BUILD=1` | Don't run colcon at start-up. |
| `WIPE_PARAMS=1` | Reset SITL's saved parameters once. |

## Camera orientation check

A mirrored or rotated camera can look fine in the Gazebo window and still steer the drone away from the target. `sim/orientation_check.py` checks it with numbers:
1. It flies the real delivery mission with its target moved to home, so the drone climbs straight up over the spawn point.
2. It puts a red target 4 m east (or north) of the spawn point.
3. It records the first detection and the first centring velocity setpoint.

A target to the east must appear right of the image centre and get an eastward setpoint. A target to the north must appear above the centre and get a northward setpoint. Run one direction per sim start:

```bash
SPAWN_TARGET=0 sim/run_sim.sh                                    # terminal 1
sim/run_sim.sh --exec python3 sim/orientation_check.py --direction east
sim/run_sim.sh --down; SPAWN_TARGET=0 sim/run_sim.sh             # fresh start
sim/run_sim.sh --exec python3 sim/orientation_check.py --direction north
```

Each run ends with `PASS` or `FAIL` and the numbers. Run both after any change to the camera model.
