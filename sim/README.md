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
| `SITE=apollo_xi` | Start at a competition site with its geofence on (below). |
| `RUN_DIR=<folder>` | Start a scenario's world at its site (`sim/scenario/README.md`). |

## Flying scenarios with nobody watching

`sim/run_scenarios.sh` builds a scenario, starts a fresh sim for it in the background, flies the mission, scores it from Gazebo's true positions and stops the sim. It keeps the laptop awake while it runs.

```bash
sim/run_scenarios.sh smoke                    # one 2026 delivery flight, about 5 minutes
sim/run_scenarios.sh delivery/clutter --seeds 1,2,3 --variants off,on
sim/run_scenarios.sh nightly                  # every scenario, 3 seeds, flags off and on
sim/run_scenarios.sh --list gates             # show the flights without flying
```

Suites are in `sim/scenarios/suites.yaml`; scenarios and their file format are in `sim/scenario/README.md`. Variant `off` is every flag false, which is the 2026 behaviour; `on` sets the scenario's flags. Run `smoke` before a pull request that touches missions, CV or the sim, and paste its table under "How I tested it".

It ends with a table of flags off beside flags on, and exit code 0 only if every flight was OK:

```
| Scenario | Seed | 2026 (flags off) | Points | Result | 2027 (flags on) | Points | Result |
| --- | --- | --- | --- | --- | --- | --- | --- |
| delivery/baseline | 1 | OK | 16.10 | 0.12 m from the target | not run |  |  |
```

| Status | Meaning |
| --- | --- |
| `OK` | Flown to a disarm on the ground, and scored |
| `TIMEOUT` | Not over within the scenario's `timeout_s` of sim time; scored as far as it got |
| `NEVER_ARMED` | The vehicle did not arm within 300 s of sim time |
| `INVALID` | The result does not count: Gazebo averaged below 0.8 of real time while armed and the run was on the wall clock; or a run on sim time failed its time check; or the scenario asks for wind or GPS faults, which the sim cannot do yet |
| `ERROR` | The sim did not start or the flight script failed; look at `sim.log` and `fly.log` |

Results go to `sim/.state/suites/<suite>-<time>/`: `summary.md`, and under `runs/` one folder per flight with `report.md`, `score.json`, `trace.jsonl` (true poses and events on sim time), `launch.log` (the mission), `sim.log` (the container) and `fly.log`.

How a flight is recorded and ended:
- `sim/runner/fly.py` runs inside the container. It reads true poses and sim time straight from Gazebo, and arm state, the mission's release lines and the autopilot's warnings from ROS.
- The flight ends when the vehicle has been disarmed on the ground for 5 s, or at the timeout. Both are sim time, so a slow laptop does not cut a flight short.
- `sim/runner/gcs_audit.py` then reads MAVProxy's MAVLink log: a command from a ground station (system id 255) or an RC override during the flight is human input, and the judge takes the autonomy bonus away.

## Sim time

By default ROS in the sim runs on the wall clock while Gazebo, SITL and the camera's image stamps run on sim time, as in 2026. That hides timing bugs, and it is why a slow run is `INVALID`. To put everything on Gazebo's clock:

```bash
SIM_TIME=1 sim/run_sim.sh                                  # terminal 1
sim/run_sim.sh --shell                                     # terminal 2
ros2 launch drone_mission_demo package_delivery_demo.launch.py use_sim_time:=true
sim/run_sim.sh --exec python3 sim/time_check.py sim/.state --duration 30   # optional: prove it
```

- `SIM_TIME=1` bridges Gazebo's clock to `/clock` (`sim/clock_bridge.py`: one message per 4 ms of sim time, because passing on all 1000 per second cost more CPU than the rest of the sim), and starts the camera bridge and MAVROS on it. MAVROS is started from `config/mavros/argus_mavros.launch.py`, which loads `config/mavros/argus_mavros.yaml` (time sync at 10 Hz, only the plugins we use) on top of MAVROS's ArduPilot defaults.
- On sim time MAVROS is connected straight to SITL's second serial port (TCP 5762), as it is wired to the flight controller on the aircraft, and the stream keeper (`drone_utils stream_keeper`) holds the message rates: pose 30 Hz, attitude 50 Hz, rangefinder 20 Hz, GPS 5 Hz, battery 2 Hz, and the global position, landed state and system status at their 2026 rates. Through MAVProxy, which the default sim still uses, poses come at 4 Hz. Check the rates with `sim/run_sim.sh --exec python3 sim/rate_check.py`. The keeper asks once and then only after a reboot or a measured rate drop, and is silent while a mission is in a release or actuation state, so `sim/rate_check.py` judges each topic on its median 5 s window. One limit: MAVROS does not reconnect a TCP link, so rebooting SITL in this mode means restarting the sim.
- Every mission launch file takes `use_sim_time:=true` and passes it to all its nodes. Without the argument the nodes stay on the wall clock.
- READY takes up to two minutes longer: the start script waits until MAVROS stamps poses with the flight controller's time (`sim/time_check.py --wait-synced`). MAVROS only applies its time-sync offset once its filter has converged.
- A scenario asks for it with `sim_time: true`, which applies in variant `on` only. The runner then runs `sim/time_check.py` beside the flight, and the run is `INVALID` unless pose stamps agree with `/clock` within 20 ms (95th percentile), at least 100 of them are synchronised flight-controller time, and every node has `use_sim_time` true. A run on sim time is not held to the 0.8 real-time rule. `delivery/time_base` is the example; measured numbers are in [docs/results/2026-11-17-time-base.md](../docs/results/2026-11-17-time-base.md).

## Acceptance gates

The gates G0 to G8 from the build plan are data in `sim/scenarios/gates.yaml`: what each flies, and the rules its runs must meet. A gate is also a suite, so flying and checking one is two commands:

```bash
python3 sim/gate.py list                       # every gate, its pass line, and whether it can fly yet
sim/run_scenarios.sh G0                        # fly it: the 2026 delivery, flags off, five times
python3 sim/gate.py check G0 sim/.state/suites/G0-<time>     # PASS, FAIL or INVALID, rule by rule
```

- **PASS:** every flight of the gate was flown and every rule holds. **FAIL:** a flight is missing or did not finish, or a rule does not hold. **INVALID:** a flight has only runs the sim was too slow to count; fly the gate again and give `check` both folders.
- The first run of a flight that counts is the one judged. Flying again helps only a run that was INVALID, never one the vehicle lost.
- `--write docs/results/<date>-gate-G0.md` writes the result file: commit, command, every run, each rule and what was found.
- A gate's seeds and variants are fixed in `gates.yaml`; `--seeds` and `--variants` are refused for a gate. To compare flags off with on, fly its scenario by name.
- G0-W2, G0-W3 and G0-W4 are G0 again after a later wave. G1 to G6 have their rules but no scenarios yet; the lane that builds a gate's scenarios adds them to `gates.yaml`. G7 and G8 need field data and a real flight.

## A/B: flags off against flags on

`sim/ab/run_ab.py` flies a plan's scenarios with every seed twice, flags off (the 2026 flight) and flags on, and says whether the plan's flag may be turned on (CEN-12). The plans are in `sim/ab/plans.yaml`: `drop` decides `drop_release_gates`, `delivery` decides `delivery_soft_set_down`; each has scenarios in calm air and in the strong wind (7 m/s), all on the 2027 sequences' sim copies, so the tuning compared is the aircraft's.

```bash
python3 sim/ab/run_ab.py drop                              # 12 flights with the plan's three seeds, about an hour
python3 sim/ab/run_ab.py drop --seeds 1                    # one seed: 4 flights
python3 sim/ab/run_ab.py drop --from sim/.state/suites/<folder> ...   # report from suites already flown
python3 sim/ab/run_ab.py drop --report docs/results/YYYY-MM-DD-ab-drop.md
```

The report gives each side's flights, failures, median and largest error and median flight time, for calm and for wind. The flag passes when, with the flags on, the drop's median error is at most 0.15 m in calm air and 0.30 m in wind, nothing was released or flown under the floor (a delivery: released above 1.83 m), and there were no more failures than with the flags off. A missing or INVALID flight makes the comparison not a pass. The command's exit code is 0 for a pass.

## Competition sites

`sim/sites/<name>.yaml` holds a field's coordinates: origin, ground elevation, runway, flight boundary, maximum height, takeoff point, compass rose and survey area. `sim/sites/apollo_xi.yaml` is the 2027 site; its coordinates are placeholders until someone checks them against the organisers' field map.

```bash
python3 sim/site_file.py check sim/sites/apollo_xi.yaml      # check the file and print a summary
python3 sim/fence_upload.py --site sim/sites/apollo_xi.yaml --dry-run   # the fence as ArduPilot gets it
SITE=apollo_xi HEADLESS=1 sim/run_sim.sh                     # start the sim there
```

With `SITE` set, start-up differs from the plain sim in five ways:
- The world is the 2026 runway world with its origin and runway strip moved to the site, written to `sim/.state/worlds/<name>.sdf`. SITL's home is that origin.
- SITL's stored parameters are wiped, and wiped again on the first plain start afterwards, so a site's fence never follows the vehicle to another world.
- SITL's terrain database is off (`SIM_TERRAIN 0`), so the ground is flat at the site's elevation.
- The 2026 delivery target is not spawned; its coordinates belong to the plain world.
- MAVProxy gets a tools port, UDP 14552, and `sim/fence_upload.py` uploads the flight boundary as an inclusion polygon and sets `FENCE_TYPE`, `FENCE_ALT_MAX`, `FENCE_MARGIN`, `FENCE_ACTION` and `FENCE_ENABLE` before `READY`.

`fence_upload.py --url <pymavlink connection>` sends the same file to a real flight controller. It refuses a site that says `verified: false`; only the sim's start-up passes `--allow-unverified`.

To check the fence does its job, with the sim at `READY` at a site:

```bash
sim/run_sim.sh --exec env -u PYTHONNOUSERSITE python3 sim/fence_check.py --site sim/sites/apollo_xi.yaml
```

It takes off, flies to a point inside the fence, sends a waypoint 20 m outside it (ArduPilot refuses that in GUIDED), then flies east through the fence at 5 m/s (`--speed`). It ends with `RESULT: PASS` if RTL began within 2 s of the fence line and the vehicle went no more than 10 m outside, and waits for the vehicle to come home and disarm. Measured results: [docs/results/2026-11-15-site-fence.md](../docs/results/2026-11-15-site-fence.md).

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

## Recording and replay

`sim/replay/` records a flight's topics, in the sim and on the aircraft, and runs the flight code on the recording again, on the bench (SIM-09).

**Record.** In the sim, add `--record` to any scenario run; the run folder gets a `bag/`:

```
sim/run_scenarios.sh delivery/baseline --record
```

Anywhere else (the aircraft, a hand-flown sim), with the nodes already running:

```
ros2 launch sim/replay/record.launch.py out:=/path/to/bag            # MCAP, wall time
ros2 launch sim/replay/record.launch.py out:=bag storage:=sqlite3 use_sim_time:=true
ros2 launch sim/replay/record.launch.py out:=bag tap_images:=/camera/image,/target_cv/annotated     # with frame taps: the bench
```

What is recorded is one list, `sim/replay/topics.py`: MAVROS pose, velocity, IMU, GPS, state, rangefinder and setpoints (servo outputs join the list with SIM-06, which decides how they reach ROS); camera info, under the 2026 name and the 2027 driver's; the 2026 detection topics and every detector on the contract; the estimator's tracks. Images are not recorded, and nothing writes frames to disk during a recording yet (`argus_camera`'s frame log is the writer for that; it is not wired to the driver).

A **frame tap** (`taps.py`) puts one small message in the bag for every frame seen on an image topic, so the bag says when each frame arrived. The sim's `--record` taps `/camera/image` and `/target_cv/annotated`, and on a wall-time run Gazebo's clock (`/replay/sim_clock`). The launch file taps nothing unless `tap_images:=` names topics: a tap is one more subscriber, and on the aircraft nothing else listens to the raw `/camera/image`, so a tap makes the 2026 grabber deliver every raw frame, before the compressed frame the detector reads. Record a flight that counts without taps until the cost has been measured on the Jetson.

MCAP needs rosbag2's MCAP plugin (`ros-humble-rosbag2-storage-mcap`); where it is not installed the sim's recorder falls back to sqlite3. The replay reads both.

**Replay without ROS (`core`).** Reads the bag in plain Python and gives its poses, rangefinder readings and detections to the same objects the executor runs (pose history, projection v2, the target estimator), on the bag's own times. The same bag gives the same answer on any machine; CI runs it on bags the tests generate.

```
python3 sim/replay/core.py <bag> --truth-file <run folder>/truth.json --out metrics.json
```

It prints the estimator's error against the truth, the mean NIS and the fraction above 9.21, the time to lock, the time without a lock, and the capture-to-heard latency.

**Where a frame's age comes from.** From the same bag, stage by stage (render, bridge, detector, publish):

```
python3 sim/replay/latency.py <bag> --out stages.json
```

**Replay through the real nodes (`ros`).** For the bench only. Plays the bag with its original stamps and `/clock`; start the nodes under test with `use_sim_time:=true` beside it. What the flight's own nodes published comes out under `/recorded/...` (recorded setpoints always do). The bag's poses and state come out under MAVROS's own names, so the launch does not start if a MAVROS is live in the same ROS domain, or if it cannot tell: never play a bag on the aircraft's network.

```
ros2 launch sim/replay/play.launch.py bag:=/path/to/bag
```

**The layouts.** The replay reads messages with its own table of layouts (`msgdefs.py`). Check the table against the installed messages whenever a message package changes: `sim/run_sim.sh --exec python3 sim/replay/check_defs.py`.

## Wind

A scenario's `wind:` is blown by `sim/wind.py`, which the runner starts beside the flight (SIM-03). On the vehicle the force fades out in the last half metre above where it stood (nothing below 0.1 m): standing in a strong wind the sim's vehicle is pushed about on its skids, the flight controller never sees it landed, and the flight ends in a timeout instead of a disarm. Payloads and buckets get the whole wind at any height.

```yaml
wind: {level: moderate, direction_deg: 225.0}       # 4 m/s with 1 m/s gusts, from the south-west
wind: {mean_mps: 5.0, direction_deg: 270.0}          # a steady 5 m/s from the west
```

Levels: `calm`; `moderate`, 4 m/s with 1 m/s gusts; `strong`, 7 m/s with 2 m/s gusts and one 3 m/s discrete gust. `mean_mps`, `gust_mps` and `discrete_gust_mps` override a level. `direction_deg` is the bearing the wind comes from (0 is a north wind).

Twenty times a second of sim time the tool takes the true velocity of the vehicle (and of every payload and bucket in the scenario) from Gazebo, and applies our own drag force, F = ½ ρ CdA |v_rel| v_rel, through Gazebo's ApplyLinkWrench system. CdA is 0.08 m² for the Iris. Gusts are Ornstein-Uhlenbeck processes along the wind, across it and vertically, with a correlation time of 2 to 5 s drawn per flight from the seed, so the same seed gives the same wind. Only a windy run's world has the wrench system; a calm run's world is the base world.

The run folder gets `wind.jsonl` (every update) and `wind.json`, and the report says what the wind was. `hover/wind_5` is the pass flight: a 40 s hover in a steady 5 m/s wind, whose tilt must be within 20% of atan(F / mg).

```
sim/run_scenarios.sh hover/wind_5
sim/run_scenarios.sh delivery/wind_moderate
```

## Payloads and the gripper

A scenario's payload can hang from the vehicle and really be let go (SIM-06):

```yaml
payload: parcel
config:
  package_drop:
    fake_drop: false          # the mission sends its release command
objects:
  - name: parcel
    kind: payload
    mass_kg: 0.5
    size_m: [0.10, 0.10, 0.10]
    carried: true             # hangs under the vehicle on a Gazebo detachable joint
```

The builder puts the package beside the camera, between the legs, with a detachable joint to the vehicle, and gives the flight controller a gripper (servo output 7) and a sprayer (servo output 8, what the 2026 drop commands). `sim/payload_bridge.py`, started by the runner beside the flight, reads the flight controller's servo outputs and is the only thing that acts on them: when a release output arrives at its release value it detaches the package, once. The judge then scores where the package came to rest. A carried payload can be at most 0.20 x 0.10 x 0.10 m, so that it stays out of the camera's view and clear of the legs and the ground.

With `fake_drop: true` (the shipped sequences) the mission sends no command, no output moves, and the judge scores the vehicle's position at the mission's fake-release log line, as before.

For a bucket (`bucket: <name>` in the scenario) the bridge takes hold when the gripper output arrives at its grab value, but only if the hook is within 0.10 m of the handle horizontally and 0.15 m vertically; otherwise nothing is attached and `grab_miss` is logged. No vehicle has a hook yet (REC-8), so this half has only been tried in unit tests.

The run folder gets `payload_bridge.jsonl` and `payload_bridge.json`, and the report a "Payload bridge" row.

```
sim/run_scenarios.sh drop/release
sim/run_scenarios.sh drop/release_wind
```

## Sensors

The vehicle comes in four camera profiles, all generated from one template (`sim/models/iris_camera.sdf.template`) by `sim/sensor_profiles.py` (SIM-05). A scenario picks one with `sensor_profile:`.

| Profile | Model | Downward camera | Also |
| --- | --- | --- | --- |
| `delivery` (the default) | `iris_downcam` | 640 x 360 at 30 Hz | today's model, byte for byte |
| `survey` | `iris_survey` | 1280 x 720 at 10 Hz | a flight's sensor check holds it to 8 Hz of sim time and a real-time factor of 0.8 |
| `still4k` | `iris_still4k` | 3840 x 2160 at 1 Hz | headless only: the sim refuses to open a window on it |
| `flow` | `iris_flow` | 640 x 360 at 30 Hz | a flow camera, 640 x 400 mono at 60 Hz, on `/argus/camera/flow/image_raw` |

Every downward camera has the IMX519's calibrated field of view (1.37635 rad). The models, their bridge files (`sim/config/bridge_<profile>.yaml`) and the camera driver's calibrations (`src/argus_camera/config/calibration/sim_<profile>.yaml`, modes `sim_survey`, `sim_still4k`, `sim_flow`) are committed. After changing the template or a profile:

```bash
python3 sim/sensor_profiles.py           # write the generated files
python3 sim/sensor_profiles.py --check   # exit 1 if one is out of date (a test does this too)
```

Two scenario keys go with them:

- `sensor_check: true` runs `sim/sensor_check.py` beside the flight. It counts the camera's frames as they arrive in ROS (by their sim-time stamps) and compares MAVROS's rangefinder with the true height (how far the vehicle has risen from where it stood) over the readings taken in a hover: above 2 m, slower than 0.3 m/s, tilted less than 5 degrees. The run fails if the 95th percentile is over 0.1 m, if the profile's frame-rate floor is missed, or if a flow camera gave no frames. The report gets "Sensors" rows.
- `truth_poses: true` runs `sim/truth_poses.py`, which publishes the true pose of the vehicle and of every object of the scenario on `/sim/truth/poses` (`tf2_msgs/TFMessage`, 30 Hz of sim time; `child_frame_id` is the model's name). Nothing on the aircraft has such a topic: it is for the sim's checks and the sim-only oracle detector.

```bash
sim/run_scenarios.sh hover/survey_camera --seeds 1 --variants on   # the pass flight
```

The rangefinder is SITL's, from the default parameters (`RNGFND1_TYPE 1`): it reads the height above the flat ground and sees no bucket or target. On sim-time runs MAVROS starts with the sim's own plugin list, `sim/config/mavros_sim.yaml`: the flight's twelve plugins plus `optical_flow`, `vision_pose`, `vision_speed` and `distance_sensor`. Wall-time runs start MAVROS as in 2026.

### Large frames (`dds_profile`)

A 1280 x 720 frame is 2.8 MB and a 4K one 25 MB; on Fast DDS's built-in transports a best-effort subscriber loses about half of them. A scenario that carries such frames says `dds_profile: large_frames`, and its container then runs Fast DDS with `sim/config/fastdds_large_frames.xml` (a 256 MB shared-memory segment). `hover/survey_camera` and `hover/still4k_camera` do; scenarios on the 640 x 360 delivery camera do not need it and do not get it.

## Surveys

`survey: {mission, area, boundary}` gives a `search_pattern` mission its search area: the corners in metres east and north of the take-off point, written into the mission's `search_area` as latitude and longitude, with the site's flight boundary as its `flight_boundary` when `boundary: true`. The mission plans its own tracks; `waypoints:` naming the same mission lists where their ends should be, for the judge only (`waypoint_navigation/survey`).

`expect_log:` lists pieces of text the mission's log (`launch.log`) must contain when the flight is flown with the scenario's flags on. A run that would otherwise be OK is FAIL without them, and the report names the missing ones. The survey scenarios use it because a survey passes its track ends whether or not its height guard acted: `survey_dip` and `survey_abort` expect the guard's lines, `survey_budget` the time budget's.

```bash
sim/run_scenarios.sh waypoint_navigation/survey --seeds 1 --variants on
```

## GPS faults

Two different things can be wrong with GPS in a scenario (SIM-04):

- **The coordinates the mission is given are off.** `coordinate_error:` in the scenario (`{east_m, north_m}`, `{radius_m}` or `{max_radius_m}`) moves the target coordinates written into the mission's sequence when the run is built. The vehicle's own GPS is untouched.
- **The vehicle's GPS misbehaves in flight.** `gps_events:` lists faults, each at `at_s` seconds after arming, in sim time. `sim/inject.py`, the one fault tool, runs beside the flight and sets SITL's simulated-GPS parameters at those times:

```yaml
gps_events:
  - {at_s: 25.0, kind: glitch, north_m: 5.0, for_s: 10.0}   # the fix steps 5 m north, and back after 10 s
  - {at_s: 40.0, kind: satellites, count: 5, for_s: 10.0}
  - {at_s: 55.0, kind: dropout, for_s: 3.0}                 # also: noise (amplitude_m), jam, clear
```

Without `for_s` a fault stays until a `clear`. The injector asks the running SITL which parameter names it has (`SIM_GPS_GLITCH_X` in ArduPilot 4.6, `SIM_GPS1_GLTCH_X` later) and stops if it finds neither. It listens on a MAVLink port of its own (UDP 14553), which the sim opens only for runs with `gps_events`, so it can run beside a driver or the payload bridge. When the flight ends it puts every parameter back.

One kind is not a GPS fault: `{at_s, kind: rangefinder, scale, for_s}` makes the rangefinder read `scale` times the true range (SITL's `SIM_SONAR_SCALE`, asked for only by a scenario with such an event). The sim's rangefinder is SITL's own, over flat ground, and sees nothing in the Gazebo world, so this is how a height above the ground is made to dip: `waypoint_navigation/survey_dip` and `survey_abort` fly the survey's height guard with it. The height check beside the flight counts the heights of that time and does not judge them.

The run folder gets `gps_events.jsonl` and `gps_events.json`, and the report a "GPS faults" row. For each glitch the injector watches the raw GPS fix: it must show the step, to within 0.5 m, inside a second. A run whose events were not all made, or whose glitch did not show, is INVALID: it is not counted as a flight with faults.

```bash
sim/run_scenarios.sh hover/gps_glitch --seeds 1 --variants off   # the pass flight
```
