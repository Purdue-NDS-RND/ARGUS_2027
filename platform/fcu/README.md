# The flight controller's settings (PLT-08)

ArduCopter 4.6 on the Pixhawk 6C.

| File | What it is |
| --- | --- |
| `argus_2027.param` | The 2027 baseline: the settings 2027 needs changed or added, each with its reason. 29 parameters, loaded over what the aircraft has. |
| `comp_2026.param` | How the aircraft was set at the June 2026 competition, all 1055 parameters, read out of that flight's own log (`src/vision_pipeline/vision_pipeline/LOGS/00000031.BIN`) by `tools/dataflash_params.py`. Reference only: do not load it (it carries that day's calibration). |
| `docs/fc/recovery.parm` | What a recovery flight needs on top of the baseline (REC-9). |

In 2026 the fence was off, no failsafe but the EKF's had an action, the gripper was disabled and no rangefinder was fitted. The baseline turns the fence on, gives the battery, radio and ground-station failsafes an action, keeps the EKF's, enables the gripper on a spare output with `FS_OPTIONS` bit 5 clear (a failsafe never drops the payload), fits the rangefinder, and sets `EK3_SRC_OPTIONS` and `GPS2_TYPE` to 0.

```
python3 tools/fc_param_check.py platform/fcu/argus_2027.param --rules baseline            # the file against its rules
python3 tools/fc_param_check.py platform/fcu/argus_2027.param --diff platform/fcu/comp_2026.param   # what it changes from 2026
python3 tools/dataflash_params.py <a newer log.BIN> --out now.param                       # how the aircraft is set today, from a log
```

Nothing loads the baseline by itself. The sim does not use it except in the three failsafe scenarios, which start SITL on its values (`sim/scenarios/hover/failsafe_*.yaml`).

## Before it is loaded: values to confirm

The file marks these "to confirm"; they are the session's working values.

- **Battery.** `BATT_LOW_VOLT 21` and `BATT_CRT_VOLT 19.8` are 3.5 and 3.3 V a cell for the 6S pack the 2026 logs show (25.6 V full, 22.4 V the lowest in flight). Set them for the pack that will fly, and `BATT_CAPACITY` with it (still ArduPilot's default 3300).
- **Gripper.** Output 7 and PWMs 1900, 1100, 1500 are the sim's. The mechanical team's output and PWMs replace them, here, in `docs/fc/recovery.parm` and in `sim/payload_bridge.py`.
- **Rangefinder.** `RNGFND1_TYPE 20` is a TFmini on a serial port; the port's `SERIALn_PROTOCOL 9` and `SERIALn_BAUD 115` are not in the file because the port is not chosen.
- **Ground-station failsafe.** `FS_GCS_ENABLE 1` brings the aircraft home 5 s after the telemetry link to the laptop is lost, even in the middle of a mission (the companion computer is not the ground station). Check that against the competition's rule on lost links and the range of the telemetry radio over the field.
- **The fence's ceiling.** `FENCE_ALT_MAX 120` is the site file's `max_altitude_agl_m`, which is itself a placeholder until the organisers' map is checked.

## On the bench and in the field, with the safety pilot

1. Save the aircraft's parameters as it is (Mission Planner: Full Parameter List, Save to file). `python3 tools/fc_param_check.py <that file> --diff platform/fcu/comp_2026.param` shows what has changed on the aircraft since June; keep the file.
2. Load `argus_2027.param` (Load from file, Write Params), reboot the flight controller. For a recovery flight load `docs/fc/recovery.parm` after it.
3. Upload the fence from the verified site file: `python3 sim/fence_upload.py --site sim/sites/<site>.yaml --url <the aircraft's link>`. It refuses a site that says `verified: false`.
4. Save the parameters again and check them: `python3 tools/fc_param_check.py <dump> --rules baseline --profile platform/fcu/argus_2027.param` must end in `PASS`.
5. Props off, armed on the bench, each failsafe in turn, watching the mode and the gripper (which must not open): transmitter off (RTL or LAND on the ground), telemetry radio unplugged for 6 s, and the battery line by raising `BATT_LOW_VOLT` above the pack's voltage for the test and putting it back.
   Do the telemetry test with the Jetson and MAVROS connected and running: the baseline counts on MAVROS not being taken for the ground station (it is not system 255), and the sim does not show that. Set the receiver to stop sending, or to drop the throttle channel under 975, when it loses the transmitter. The EKF failsafe is not tried: there is no safe way to trip it on the bench.
6. In a hover at 5 m, 15 m from home: transmitter off, then the telemetry radio unplugged; each must bring the aircraft home. Fly at the fence at walking speed: RTL at the line. Write what happened in `docs/results/`.

The readiness video of the fence and the failsafes is OPS-02.

## The companion watchdog (PLT-09)

`scripts/argus_watchdog.lua` runs on the flight controller. With the vehicle flying in GUIDED, if the companion's setpoints stop for more than a second it switches to LOITER and says so ("ARGUS watchdog: setpoints stopped, now LOITER"). It is armed only after five setpoints have come in that stretch of flying, and starts counting again whenever the vehicle is not in GUIDED, is disarmed, is not flying, or is taking off: a take-off the flight controller flies itself, and the delivery's time on the ground, are left alone.

It is not part of the baseline file. To load it: `SCR_ENABLE 1`, reboot, copy the script to `APM/scripts` on the SD card, reboot, and look for "ARGUS watchdog loaded" among the ground station's messages.

Before it is loaded on the aircraft:

- Fly every mission that will be flown with it in the sim first (`fc_scripts` in a scenario, as `sim/scenarios/delivery/watchdog.yaml` has it). A mission that goes quiet in the air in GUIDED for more than a second, after it has been sending setpoints, is taken for a dead companion: the vehicle goes to LOITER, and with the takeover latch on the executor then stays out. Only the 2026 delivery has been flown with it.
- LOITER needs the safety pilot, with the throttle stick at its middle: in LOITER the stick is the climb rate (`PILOT_SPEED_UP 250`, `PILOT_SPEED_DN 0` in 2026's set), so a stick resting low brings the vehicle down at up to 2.5 m/s. The pilot's procedure during autonomous flight is mid-stick. Left alone at mid-stick the copter holds its place until its battery failsafe. That is the point (the pilot takes it from a known state), and it is why the baseline's battery and radio failsafes should be loaded with it.
- On the bench, props off, armed in GUIDED: the script cannot be tripped on the ground (it acts only when flying). Its first real test is a hover with the safety pilot: start a mission, kill the executor (`Ctrl-C` on the launch), and see LOITER within two seconds.

## In the sim

```
sim/run_scenarios.sh hover/failsafe_battery          # low voltage -> RTL, critical -> LAND
sim/run_scenarios.sh hover/failsafe_radio            # receiver silent -> RTL
sim/run_scenarios.sh hover/failsafe_ground_station   # no ground-station heartbeat -> RTL
sim/run_scenarios.sh geofence/breach                 # the fence (SIM-02)
sim/run_scenarios.sh hover/watchdog                  # the watchdog: setpoints stop -> LOITER in 1 to 2 s
sim/run_scenarios.sh delivery/watchdog --variants off   # the 2026 delivery with the watchdog loaded: it must not act
```

Each failsafe flight starts SITL on the baseline's values, is flown by `sim/failsafe_check.py`, and counts only if the mode the baseline asks for began in time with the gripper held. The EKF failsafe is not tripped in the sim: its action is 2026's and unchanged.
