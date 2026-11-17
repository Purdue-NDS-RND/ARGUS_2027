# MAVROS message rates, the direct SITL link and the stream keeper (PLT-06)

Measured on 17 Nov 2026 on a club laptop, on `platform/build-2027` at `25bd900` plus the PLT-06 change. Sim image `argus-sim:humble`: ArduCopter 4.6.3 SITL, Gazebo Harmonic, MAVROS 2.16.0.

**Result: PASS.** With the stream keeper on MAVROS's own link to SITL, every requested rate was held through a whole delivery flight, the flight landed 0.02 m from the target, and pose stamps agreed with the sim clock to 4.6 ms at the 95th percentile. That also closes the FND-03 line "pose stamps agree with the clock within 20 ms", which the MAVProxy link met in one flight of three.

## The scored flight (final code)

```
$ sim/run_scenarios.sh delivery/time_base --variants on
   READY after 119 s; flying...
   OK          delivery/time_base seed 1 on: 16.22 points, 0.02 m from the target
```

From the run's `report.md`, `rate_check.json` and `time_check.json`. Both checks run beside the flight for all of it, 160 s of sim time, which includes `simple_takeoff_service` asking for every stream at 20 Hz when the mission starts:

| Topic | MAVLink message | Asked | Measured in flight |
| --- | --- | --- | --- |
| `/mavros/local_position/pose` | `LOCAL_POSITION_NED` | 30 Hz | 29.9 Hz |
| `/mavros/imu/data` | `ATTITUDE` | 50 Hz | 48.7 Hz |
| `/mavros/rangefinder/rangefinder` | `RANGEFINDER` | 20 Hz | 20.0 Hz |
| `/mavros/gpsstatus/gps1/raw` | `GPS_RAW_INT` | 5 Hz | 5.8 Hz |
| `/mavros/battery` | `BATTERY_STATUS` | 2 Hz | 2.9 Hz |
| `/mavros/global_position/global` | `GLOBAL_POSITION_INT` | 20 Hz | 20.0 Hz |
| `/mavros/extended_state` | `EXTENDED_SYS_STATE` | 2 Hz | 2.0 Hz |
| `/mavros/sys_status` | `SYS_STATUS` | 2 Hz | 3.0 Hz |

The slow ones read a little high and the fast ones a little low because the takeoff service's request sets everything to 20 Hz until the keeper's next pass, up to 10 s later.

| | |
| --- | --- |
| Pose stamps against the sim clock | 4833 poses, median -2.8 ms, 95th percentile 4.6 ms, maximum 37.3 ms (limit 20 ms) |
| Stamps that are synchronised flight-controller time | 4812 of 4833 |
| Nodes with `use_sim_time` true | 23 of 23 checked |
| Real-time factor while armed | 0.91 |
| `service timeout`, `command plugin service call failed` or `already in progress` lines in MAVROS's log | 0 |

The keeper's whole log for the run:

```
[stream_keeper]: Stream keeper: holding LOCAL_POSITION_NED 30 Hz, ATTITUDE 50 Hz, RANGEFINDER 20 Hz, GPS_RAW_INT 5 Hz, BATTERY_STATUS 2 Hz, GLOBAL_POSITION_INT 20 Hz, EXTENDED_SYS_STATE 2 Hz, SYS_STATUS 2 Hz
[stream_keeper]: Stream keeper: the flight controller accepted every rate
```

Plugins MAVROS loaded, from its log (`Plugin ... created`), 12 of the 62 installed: `command gimbal_control global_position gps_status home_position imu local_position rangefinder setpoint_position setpoint_raw sys_status sys_time`. The ROS graph in a flight is 23 nodes instead of 65, and MAVROS took a quarter of a core instead of 87% (measured on the ground when fifteen plugins were still loaded).

## Before and after

| | Through MAVProxy (FND-03's three flights) | Direct link with the keeper (four flights) |
| --- | --- | --- |
| Pose rate in sim time | 4.2 to 4.7 Hz | 29.9 to 30.2 Hz |
| Clock minus pose stamp, median | +10.1, +11.5, +12.9 ms | -2.1, -1.4, -2.1, -2.8 ms |
| 95th percentile (limit 20 ms) | 16.0, 23.0, 23.9 ms | 4.6, 5.4, 4.4, 4.6 ms |
| Real-time factor while armed | 0.86 to 0.88 | 0.91 to 0.94 |

On the ground with no mission running (`sim/run_sim.sh --exec python3 sim/rate_check.py --duration 20` in a sim started with `SIM_TIME=1`): pose 30.3 Hz, attitude 50.0 Hz, global position 5.0 Hz (then asked at 5), GPS 5.0 Hz, battery 2.0 Hz, rangefinder 20.0 Hz. After `ros2 service call /mavros/set_stream_rate ... "{stream_id: 0, message_rate: 4, on_off: true}"`, the next 20 s averaged pose 29.3 Hz and attitude 48.4 Hz: the keeper had put them back.

## What went wrong on the way

**1. The first keeper made MAVROS time out for a minute at a stretch.** It sent every request at once, every 5 s, through MAVROS's `set_message_interval` service, and repeated a refused one after 1 s. The flights passed, but MAVROS's log had runs like this, 55 lines a second apart through an RTL and landing (found by the flight-guardian review):

```
[mavros.sys]: SYS: set_message_interval service timeout
[mavros.sys]: SYS: command plugin service call failed!
```

From MAVROS 2.16's source: that service waits 1 s on the command plugin inside the callback group that also serves `set_mode`, and the command plugin serves every command in turn with a 5 s wait for the acknowledgement. So a burst queues up, each queued call times out, the keeper retries, and a mode change or a gripper command would wait behind it. No delayed mode change was seen, but the path was there. The keeper now sends `MAV_CMD_SET_MESSAGE_INTERVAL` through `/mavros/cmd/command`, one request at a time, every 10 s, and stays quiet for 5 s after a failure. The scored flight above is on that code: no timeout lines.

**2. A reboot cannot be told apart in the sim.** On the 2026 UDP link (the direct TCP link does not survive a SITL reboot: MAVROS does not reconnect), with the first keeper started by hand:

```
$ ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 246, param1: 1.0}"
arducopter.log:  bind port 5760 for SERIAL0 / Waiting for connection .... / Connection on serial port 5760      (a second time: it rebooted)
rate_check, 45 s later:  pose 23.7 Hz, attitude 38.5 Hz, rangefinder 15.9 Hz
```

The rates came back (MAVProxy alone leaves pose at 4 Hz; the figures are below the targets because MAVProxy keeps resetting them on that link), but the keeper never logged a restart. Neither of its reboot signals shows in the sim: MAVROS did not report a disconnect, and SITL takes its clock from Gazebo, so the flight controller's clock does not go back. What restored the rates was the periodic repeat.

The same run with the final keeper (one request at a time, repeat every 10 s; `sim/.state/fnd03/reboot_test2.log`):

```
before the reboot:  pose 16.4 Hz, attitude 25.1 Hz, global position 10.7 Hz, rangefinder 11.0 Hz
after the reboot:   pose 18.9 Hz, attitude 29.2 Hz, global position 11.3 Hz, rangefinder 12.3 Hz
[stream_keeper]: Stream keeper: the flight controller accepted every rate
[stream_keeper]: Stream keeper: a rate request was refused or not answered; trying again after a pause
[stream_keeper]: Stream keeper: the flight controller accepted every rate
```

SITL rebooted (`bind port 5760` twice in its log). One request failed while it was down, the keeper paused, and afterwards every rate was accepted again and the rates were back where they had been. They sit at about half the targets on this link because MAVProxy resets them between the keeper's passes, which is why the sim-time start-up does not use it. There is still no "restarted" line: detection of a real reboot is covered by unit tests and by a bench step under "Needs a person".

**3. With the stale-data guard on, the delivery never got down.** One flight had `time_health_guard` on:

```
   OK          delivery/time_base seed 1 on: 0.00 points, nothing released
[package_delivery] time base UNHEALTHY (frame is 258 ms old (limit 250 ms)): treating it as no target, holding, no descent
```

17 such lines, with frame ages from 250 to 282 ms, each followed by a hold of 1 to 5 s; the recovery climbs were used up and the mission failed to RTL. Pose age no longer trips it (pose is at 30 Hz). The guard is doing its job: on this laptop a frame is a quarter of a second old by the time the mission can act on it, which is the 2026 problem FND-01 and PLT-03 exist to fix. The scenario leaves the guard off until then.

**4. One flight was INVALID for a reason of the tooling's own.** The time check counted the new rate-check node as "on wall time". Both checkers are on the wall clock on purpose; the time check now leaves that node out.

## Commands

```
SIM_TIME=1 HEADLESS=1 sim/run_sim.sh --detach
sim/run_sim.sh --exec python3 sim/rate_check.py --duration 20
sim/run_scenarios.sh delivery/time_base --variants on
```

Run folders (not committed), in order: `sim/.state/suites/delivery-time_base-20261117T221646Z/` (guard on), `-222217Z/`, `-222914Z/` (the INVALID one), `-223654Z/` (final code).
