# Mission sim quickstart

Fly the package delivery mission in the simulator in three steps. This page
assumes the sim image is already built; for first-time setup, rebuilding,
logs and everything else, see [sim/README.md](../sim/README.md).

## 1. Terminal 1: start the sim

```bash
cd ~/ARGUS_2027 && sim/run_sim.sh
```

Wait for `== READY. In another terminal:` (about 70 seconds; the line above
it says `pre-arm checks pass after ...`). Leave this terminal running;
Ctrl+C stops everything.

## 2. Terminal 2: open a shell in the sim

```bash
cd ~/ARGUS_2027 && sim/run_sim.sh --shell
```

The prompt changes to `(argus-sim)`. You are now inside the container.

## 3. At the `(argus-sim)` prompt: fly

```bash
ros2 launch drone_mission_demo package_delivery_demo.launch.py
```

## What you should see

In Gazebo: the Iris quadcopter on the runway, and a red and white bullseye
about 5 m across, about 10 m east and 15 m north of it. The drone takes off
to 15 m, flies to the bullseye, centres over it and descends slowly, touches
down on it, waits 5 seconds, climbs back up, then flies home and lands by
itself.

In terminal 2, the mission log goes through these states in order:

1. `takeoff` climbs to 15 m.
2. `package_delivery`: `INIT` → `WAITING_FOR_CONNECTION` → `WAITING_FOR_GPS`
   → `TRANSIT_TO_TARGET` → `ACQUIRE_TARGET` → `TRACK_AND_DESCEND` →
   `FINAL_FIXED_COLUMN_DESCENT` → `GROUND_DWELL` (`Fake delivery complete`)
   → `GUIDED_RELAUNCH` → `COMPLETE`.
3. `rtl` hands over to ArduPilot's RTL.
4. `Mission sequence complete`.

The flight takes about 2.5 minutes, plus about 45 seconds for RTL to land.
The launch keeps running after `Mission sequence complete`; stop it with
Ctrl+C.

To measure how close it landed to the target, run this in a third terminal
before the drone reaches the target:

```bash
cd ~/ARGUS_2027 && sim/run_sim.sh --exec python3 sim/touchdown_report.py
```

## The four commonest errors

| You see | Why, and the fix |
| --- | --- |
| `ERROR: Docker is not installed` after `sim/run_sim.sh --shell` | You ran `--shell` again from inside the container (the prompt says `(argus-sim)`). You're already in: just run the `ros2 launch` command. |
| `ros2: command not found`, or `Package 'drone_mission_demo' not found` | The opposite: you're on the host, not in the sim. Run `sim/run_sim.sh --shell` first and launch at the `(argus-sim)` prompt. |
| `ERROR: the sim is not running` | Terminal 1 isn't running or has stopped. Start `sim/run_sim.sh` there and wait for `READY`. If it says `the club simulator (ardupilot-gz-sim) is running`, stop that first: `cd ~/Drone_Sim_Docker && make down`. |
| `Max arm attempts reached` | You launched before `READY`. Wait for it, stop the launch with Ctrl+C, and launch again. |

Anything else, including the mission sitting in `ACQUIRE_TARGET` or
climbing in `TARGET_NOT_FOUND`: see the "When it fails" lists in
[sim/README.md](../sim/README.md).
