# Flight-controller baseline (PLT-08): the failsafes in the sim

2026-11-23, build machine. ArduCopter SITL started on the values of `platform/fcu/argus_2027.param` (each scenario's `sitl_params`, held to the file by a test, and read back from the autopilot by the check before it flies). Each flight: take off to 10 m, close the gripper, fly 15 m east, take away what the failsafe watches. These are SITL's answers: the aircraft's own are a person's to try (`docs/build/PROGRESS.md`, "Needs a person", PLT-08).

```
$ sim/run_scenarios.sh hover/failsafe_radio
   OK          hover/failsafe_radio seed 1 off: 1.00 points, held within 15.10 m for 58 s
$ sim/run_scenarios.sh hover/failsafe_ground_station
   OK          hover/failsafe_ground_station seed 1 off: 1.00 points, held within 15.10 m for 62 s
$ sim/run_scenarios.sh hover/failsafe_battery
   OK          hover/failsafe_battery seed 1 off: 1.00 points, held within 15.10 m for 68 s
```

| Failsafe | What was taken away | Mode before → after | After the trigger | Limit | Gripper output | Ended |
| --- | --- | --- | --- | --- | --- | --- |
| Radio | the receiver stops sending (`SIM_RC_FAIL 1`) | GUIDED → RTL | 1.0 s | 5 s | 1900 (grab) throughout | home, down and disarmed |
| Ground station | its heartbeat no longer counted (`SYSID_MYGCS 77`) | GUIDED → RTL | 4.7 s | 10 s (`FS_GCS_TIMEOUT` 5 s and 5 s) | 1900 | home, down and disarmed |
| Battery, low | 20.5 V, under `BATT_LOW_VOLT 21` | GUIDED → RTL | 10.3 s | 15 s (`BATT_LOW_TIMER` 10 s and 5 s) | 1900 | |
| Battery, critical | 19.3 V, under `BATT_CRT_VOLT 19.8` | RTL → LAND | 10.2 s | 15 s | 1900 | down where it was, disarmed |

The autopilot's own words in the logs: `Radio Failsafe`; `GCS Failsafe` (and `GCS Failsafe Cleared` when the check gave the ground station back); `Battery 1 is low 20.50V`, `Battery Failsafe`, `Battery 1 is critical 19.30V`, `Battery Failsafe`.

The gripper's output stayed at its grab value through every failsafe: `FS_OPTIONS 16` has bit 5 clear, so no failsafe lets the payload go.

"OK" is the runner's: the hover judge (the vehicle stayed within 25 m and came down) and the check's own verdict in `failsafe_check.json`, without which the run does not count. "off" is the variant's name; a flight flown by a tool has no mission flags.

The delays are from the tool's clock at the moment it changed the SITL parameter to the first heartbeat in the new mode (heartbeats at 5 Hz), on a sim running near real time.

One flight of each, seed 1. After these flights the check was tightened to need the gripper's output at its grab value after the grab and at every step (it used to fail only at the release value); the reports above have it at 1900 throughout. `hover/failsafe_radio` was flown again with the tightened check: OK, RTL 1.0 s after the receiver went silent, the output at 1900 after the grab and 1900 after the trigger.

## Not shown here

- The fence: `geofence/breach` (SIM-02, `docs/results/2026-11-15-site-fence.md`): RTL 0.04 to 0.06 s after the line. The baseline's four fence values are the ones that upload sets (a test holds them to the site file).
- The EKF failsafe: not tripped. Its action (Land) is 2026's and unchanged.
- Anything on the aircraft.
