# PLT-06: MAVROS config and stream keeper

Flag: launch argument `stream_keeper` (default `false`) on `config/mavros/argus_mavros.launch.py`; the launch file itself is the opt-in for the plugin list and time sync (the 2026 start-up does not use it) · Status: PARTIAL · Commits: `git log --oneline --grep '^PLT-06:' platform/build-2027`

## What it solves

The 2027 stack needs poses at 30 Hz and attitude at 50 Hz; 2026 asked for everything at 20 Hz once, and in the sim MAVProxy cut that to 4 Hz. Nothing asked again after a flight-controller reboot. This requests the rates we need and keeps them, and loads only the MAVROS plugins we use. It fixes the cause, on the ARGUS MAVROS launch; the 2026 start-up is unchanged.

## Architecture

- `config/mavros/argus_mavros.yaml`: adds a plugin allow-list of twelve (deny `*`, allow the list) to the time-sync block from FND-03.
- `config/mavros/argus_mavros.launch.py`: new argument `stream_keeper`; when true it also starts the node below.
- `drone_utils/stream_keeper.py` (new, no ROS): the rate table and `StreamKeeper`, which says which request is due.
- `drone_utils/stream_keeper_node.py` (new) and the `stream_keeper` entry point: subscribes to `/mavros/state` and `/mavros/timesync_status`, calls `/mavros/cmd/command`.
- Sim, sim-time branch only: MAVROS connects to SITL's second serial port (TCP 5762) instead of MAVProxy's UDP output, with the keeper. `sim/rate_check.py` (new) measures rates in sim time and runs beside every sim-time flight; a failed rate check makes the run INVALID.

## Flow

Before (2026 link, sim): MAVROS → MAVProxy → SITL. `simple_takeoff_service` asks for all streams at 20 Hz once; MAVProxy's own requests put them back to 4 Hz. Pose 4.2 to 4.7 Hz.

After (`SIM_TIME=1` or a `sim_time` scenario): MAVROS → SITL directly. The keeper sees `/mavros/state` connected, sends `MAV_CMD_SET_MESSAGE_INTERVAL` for one message, waits for the answer, sends the next, and repeats the pass every 10 s. Pose 30 Hz within two seconds of connecting and for the whole flight.

## Code

- `stream_keeper.py:28`, `MESSAGE_RATES_HZ`: the five rates in the spec, and three more held at their 2026 rates (global position 20 Hz, landed state 2 Hz, system status 2 Hz) so the keeper takes nothing away from the missions.
- `stream_keeper.py:100`, `due`: at most one request out; an unanswered one is given up after 6 s; nothing is asked for 5 s after a failure.
- `stream_keeper.py:74` and `:86`, `on_state` and `on_fcu_time`: a reconnect, or the flight controller's clock going back by more than 1 s, is a reboot and makes every rate due.
- `stream_keeper_node.py:42`, `_tick`: sends through `/mavros/cmd/command`, not MAVROS's `set_message_interval` service (which blocks the group that serves `set_mode`).
- `sim/container/start.sh`, the MAVROS step of the sim-time branch: waits for SITL to open TCP 5762, then starts the ARGUS launch with `stream_keeper:=true`.

## Evidence

```
$ PYTHONPATH=src/drone_utils python3 -m pytest -q -p no:cacheprovider src/drone_utils/test/test_stream_keeper.py
20 passed in 0.24s
$ python3 -m pytest -q -p no:cacheprovider sim/test/test_rate_check.py sim/test/test_time_base.py
59 passed in 0.31s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh delivery/time_base --variants on
   OK          delivery/time_base seed 1 on: 16.22 points, 0.02 m from the target
| Pose stamps against the sim clock | PASS: p95 4.6 ms, max 37.3 ms over 4833 poses (limit 20 ms) |
| MAVROS message rates (sim time) | PASS: local_position 29.9 Hz, imu 48.7 Hz, global_position 20.0 Hz, gpsstatus 5.8 Hz, battery 2.9 Hz, sys_status 3.0 Hz, extended_state 2.0 Hz, rangefinder 20.0 Hz |
```

The keeper's tests were written before it and failed on import. The rate check was written during the measurements and its tests after it. Everything measured, including the first keeper's timeouts in MAVROS and the reboot run, is in [docs/results/2026-11-17-mavros-rates.md](../../results/2026-11-17-mavros-rates.md). The flags-off sim was not flown again for this feature: nothing on its path changed (`mavros_boot.sh`, the default branch of `start.sh`), and a test pins that.

## Spec check

spec-auditor, two rounds, INCOMPLETE both times. Round 1: no results or notes file yet, the deviations not recorded, two plugins listed that nothing used, nothing automated that would notice pose falling back to 4 Hz, and no evidence of a re-request after a reboot. Round 2 found all of that closed except the reboot: no running node has detected one. That is why the row is PARTIAL; the stale interval it found in a Decisions row is corrected.

flight-guardian, two rounds, CONCERNS both times, no blocker. Round 1 found, in the sim's MAVROS logs, that the first keeper's bursts through `set_message_interval` produced runs of service timeouts in the group that also serves `set_mode`; that holding the global position at 5 Hz made the missions' hold-point fix staler than in 2026; and that the landed state would not come back after a reboot. All three led to the rewrite and round 2 found them resolved. From round 2:

- **Fixed after the round, not reviewed or flown again:** one message the flight controller keeps refusing no longer starves the ones after it (it waits a full repeat and the others go first; `test_one_message_the_flight_controller_never_gives_does_not_starve_the_others`). No flight exercised that path: SITL accepts all eight.
- **Left standing:** the keeper's requests go through the same MAVROS command service as the gripper, the sprayer and arming, and MAVROS serves commands in turn with a 5 s wait for each acknowledgement. If one keeper request is not acknowledged, a release sent in that window is late by up to 5 s (late, not misplaced: the drop holds its column meanwhile). The scored flights used `fake_drop`, so no release has gone through MAVROS beside the keeper. Bench step under "Needs a person".

What differs from the spec, each recorded under Decisions: the switch is a launch argument, not a `features.` parameter; three messages beyond the spec's five are held at their 2026 rates; the rates are re-requested every 10 s as well as on a reboot; the command goes through `/mavros/cmd/command`.

## Not done

- **A reboot is not detected in the sim**, which is why this row is PARTIAL: SITL's clock does not restart and MAVROS reports no disconnect, so the detection path is proven by unit tests only. The rates did come back after a SITL reboot, through the 10 s repeat. The bench step (reboot the flight controller, expect the keeper's "restarted" line and 30 Hz again) is under "Needs a person".
- **Nothing starts the keeper on the aircraft yet.** PLT-07's bringup launches MAVROS from `argus_mavros.launch.py` with `stream_keeper:=true`; until then `mavros_boot.sh device` is the 2026 start-up.
- **On the direct link a SITL reboot ends MAVROS's connection** (MAVROS does not reconnect TCP). The default sim still uses MAVProxy's UDP output.
- **The stale-data guard is still off in `delivery/time_base`**: frames are 250 to 280 ms old in the sim. FND-01 and PLT-03.
- `waypoint`, `param` and `distance_sensor` are not loaded; PLT-13 and CEN-03 add theirs when they use them.
