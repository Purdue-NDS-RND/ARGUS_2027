# GPS missions in the club sim (2026-11-11)

Last year's three non-vision missions flown against the club simulator with
the `sim/gps/` harness: `square_only`, `waypoint_navigation_live` and
`circuit_time_trial_live`. All four flights passed: the time trial on its own,
then all three in a row with `fly_all.sh`.

**SITL GPS is ideal, so these distances are unrealistically tight.** The club
sim's SITL ran with `SIM_GPS_NOISE 0`, no glitch offsets, no altitude drift,
no jamming and 10 satellites, and reported fix_type 6 (RTK fixed). Real GPS
is metres off, not centimetres. Read the closest distances as "the mission
logic steers to the right point", not as expected accuracy on the field.

| Run | Sequence | Result | Flight time, arm to disarm | Gazebo real-time factor while armed (mean, min to max, samples) |
| --- | --- | --- | --- | --- |
| 1 | `circuit_time_trial_live` | PASS | 236.1 s | 0.99, 0.75 to 1.00, 46 |
| 2 | `square_only` | PASS | 87.9 s | 0.98, 0.82 to 1.00, 17 |
| 3 | `waypoint_navigation_live` | PASS | 251.9 s | 0.96, 0.28 to 1.57, 48 |
| 4 | `circuit_time_trial_live` | PASS | 243.5 s | 0.98, 0.34 to 1.76, 45 |

Run 1 was `run_mission.sh` alone; runs 2 to 4 were one `fly_all.sh --yes`.
PASS means: every mission in the sequence logged "completed", the executor
logged "Mission sequence complete", the vehicle left the ground, landed, and
disarmed on the ground (`sim/gps/flight_log.py`). Flight times are on the
monotonic clock (the new `mono=` stamps in `state.log`).

## Closest distance to each waypoint (metres)

From the mission's own GPS waypoint advisory summary in `launch.log`.
`square_only` flies a local pattern and logs no GPS distances.

| Waypoint | 3 `waypoint_navigation_live` | 1 `circuit_time_trial_live` | 4 `circuit_time_trial_live` |
| --- | --- | --- | --- |
| wp1 | 0.01 | 0.14 | 0.09 |
| wp2 | 0.00 | 0.87 | 0.95 |
| wp3 | 0.01 | 0.91 | 0.91 |
| wp4 | 0.00 | 0.76 | 0.72 |
| wp5 | 0.00 | 1.00 | 0.96 |
| wp6 | 0.01 | 0.89 | 0.90 |
| wp7 | 0.13 | 3.77 | 3.78 |
| circuit elapsed (wall-clock, logged by the mission) | | 152.35 s | 153.50 s |

- Waypoint navigation uses `arrival_radius_m: 1.0` and `hold_time_s: 0.5`, so
  the vehicle settles on each point.
- The time trial uses `arrival_radius_m: 4.0` and `hold_time_s: 0.0`: a
  waypoint counts as soon as the vehicle is inside 4 m, so it turns early
  (0.7 to 1.0 m). wp7 is the last waypoint; the circuit ends as soon as the
  vehicle is inside its 4 m radius and RTL takes over, which fits 3.77 and
  3.78 m in both runs.
- The two time-trial runs agree to within 0.1 m per waypoint and 1.2 s on
  the circuit.

## Setup

- **Machine:** club drive (Lenovo 82BG laptop), the same as the
  2026-11-04 sim baseline.
- **Sim:** the club simulator (`~/Drone_Sim_Docker`, `make run`),
  `ardupilot-gz:4.6.3`, with QGroundControl on UDP 14550. Started by hand;
  QGC showed Ready to Fly, a GPS fix and no pre-arm messages before run 1.
- **MAVROS:** `sim/gps/start_mavros.sh`, container `argus-gps` from
  `argus-sim:humble`, on SITL's SERIAL1 (TCP 5762), `ROS_DOMAIN_ID=42`.
  colcon built `drone_utils`, `drone_mission_core`, `drone_mission_demo`.
- **Code:** branch `sim/gps-missions-v2` from `main` at `e066041`. No changes
  under `src/`; sequences, waypoints and params as on `main`.
- **SITL home:** moved to the competition site for the flights, put back to
  CMAC afterwards (below).
- **Suspend:** each flight ran under
  `systemd-inhibit --what=sleep:idle:handle-lid-switch --mode=block`
  (`systemd-inhibit --list` showed one `ARGUS sim/gps` lock during the runs,
  none after).

## Commands and output

SITL home, before and after moving it (`home-saved.txt` keeps the CMAC values
for `--restore`):

```
$ sim/gps/set_home.sh --show
  SIM_OPOS_LAT = -35.36326217651367
  SIM_OPOS_LNG = 149.1652374267578
  SIM_OPOS_ALT = 584.0
  SIM_OPOS_HDG = 353.0

$ sim/gps/set_home.sh
...
Reconnected. SITL position now:
  latitude: 35.0497016
  longitude: -118.1509017
  altitude: 800.2378343297373
Stored values after the reboot:
  SIM_OPOS_LAT = 35.04970169067383
  SIM_OPOS_LNG = -118.1509017944336
  SIM_OPOS_ALT = 831.7000122070312
  SIM_OPOS_HDG = 353.0
  rangefinder: 0.0 m (on the ground, OK)
```

The 800.2 m altitude is MAVROS's height above the WGS84 ellipsoid; the geoid
is about 31.5 m below it there, so it matches 831.7 m AMSL.

Pre-flight check:

```
$ sim/gps/run_mission.sh --check
== /mavros/state: connected, armed: false mode: STABILIZE
== /mavros/local_position/pose            17.2 Hz
== /mavros/imu/data                       17.3 Hz
== /mavros/extended_state                  1.7 Hz
== /mavros/global_position/global         17.3 Hz
== GPS: fix_type 6 (3 = 3D fix), 10 satellites
== position: latitude: 35.0497016 longitude: -118.1509017
Pre-flight checks passed.
```

The streams arrive at about 17 Hz against the 20 Hz requested.

Run 1:

```
$ sim/gps/run_mission.sh circuit_time_trial_live
RESULT: PASS
missions completed: takeoff, circuit_time_trial, rtl
sequence complete: yes
armed -> disarmed: yes; final state on ground
flight time, arm to disarm (monotonic): 236.1 s
Gazebo real-time factor while armed: mean 0.99, min 0.75, max 1.00 (46 samples)
closest to wp1: 0.14 m
...
closest to wp7: 3.77 m
circuit elapsed (wall-clock): 152.35 s
```

Runs 2 to 4:

```
$ sim/gps/fly_all.sh --yes
== SITL home is already at the competition site (SIM_OPOS_LAT 35.04970169067383)
...
================ Summary ================
PASS  square_only
PASS  waypoint_navigation_live
PASS  circuit_time_trial_live
```

Each run's summary can be printed again from its logs with
`python3 sim/gps/flight_log.py sim/.state/gps/runs/<run>` (logs are not
committed).

SITL home put back:

```
$ sim/gps/set_home.sh --show
  SIM_OPOS_LAT = 35.04970169067383
  SIM_OPOS_LNG = -118.1509017944336
  SIM_OPOS_ALT = 831.7000122070312
  SIM_OPOS_HDG = 353.0

$ sim/gps/set_home.sh --restore
...
Stored values after the reboot:
  SIM_OPOS_LAT = -35.36326217651367
  SIM_OPOS_LNG = 149.1652374267578
  SIM_OPOS_ALT = 584.0
  SIM_OPOS_HDG = 353.0

$ sim/gps/run_mission.sh --check
== position: latitude: -35.3632621 longitude: 149.1652374
```

## Notes

- Every launch log ends with `RCLError: failed to shutdown: rcl_shutdown
  already called` from `mission_executor` and `simple_takeoff_service`, then
  launch reporting "process has died". It comes after the disarm, when
  `run_mission.sh` stops the launch with SIGINT, and the 2026-11-04 GPS runs
  show the same two lines. It is a shutdown-order issue in code under `src/`
  and is not fixed here. Apart from those and launch's "user interrupted
  with ctrl-c" warning for the same SIGINT, no error or warning lines were
  logged in any of the four runs.
- The real-time factor dipped to 0.28 and 0.34 and peaked at 1.57 and 1.76 in
  runs 3 and 4 (5 s samples); the means stayed at 0.96 to 0.99.
- "circuit elapsed" is the mission's own wall-clock timer, from code under
  `src/`; only the harness's flight time moved to the monotonic clock.
