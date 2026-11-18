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

After (`SIM_TIME=1` or a `sim_time` scenario): MAVROS → SITL directly. The keeper sees `/mavros/state` connected and sends `MAV_CMD_SET_MESSAGE_INTERVAL` for one message at a time until all eight are accepted. From then on it counts each message's topic. During the takeoff it is silent; when the takeoff is over it finds pose at 20 Hz and attitude at 20 Hz (the takeoff service's doing), asks for those two again, and is silent again through the delivery's ground dwell. Ten requests in the flight.

## Code

- `stream_keeper.py`, `MESSAGE_RATES_HZ`: the five rates in the spec, and three more held at their 2026 rates (global position 20 Hz, landed state 2 Hz, system status 2 Hz) so the keeper takes nothing away from the missions.
- `stream_keeper.py`, `StreamKeeper._measure`: every 5 s per message, the measured rate; under 80% of what was asked makes that message due again, and after three accepted requests that change nothing the message is left alone.
- `stream_keeper.py`, `StreamKeeper.due`: nothing while `hold` is set, at most one request out, an unanswered one given up after 6 s, 5 s of silence after a failure.
- `stream_keeper.py`, `ActuationWatch`: reads the missions' `State: OLD -> NEW` lines and the executor's start and end lines; `is_actuation_state` is the list of release and actuation states.
- `stream_keeper_node.py`: counts the eight topics (raw, not decoded), listens to `/rosout` for the executor, and sends through `/mavros/cmd/command`, not MAVROS's `set_message_interval` service (which blocks the group that serves `set_mode`).
- `sim/container/start.sh`, the MAVROS step of the sim-time branch: waits for SITL to open TCP 5762, then starts the ARGUS launch with `stream_keeper:=true`.

## Evidence

```
$ PYTHONPATH=src/drone_utils python3 -m pytest -q -p no:cacheprovider src/drone_utils/test/test_stream_keeper.py
51 passed in 0.33s
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

**The 18 Nov change** (the lead's instruction: no periodic repeat, nothing during a release or actuation state):

```
$ python3 -m pytest -q -p no:cacheprovider src/drone_mission_demo/test/test_actuation_states.py
5 passed in 0.02s
$ sim/run_scenarios.sh delivery/time_base --variants on
   OK          delivery/time_base seed 1 on: 16.20 points, 0.04 m from the target
| MAVROS message rates (sim time) | PASS: local_position 30.2 Hz, imu 50.0 Hz, global_position 20.0 Hz, ... rangefinder 20.0 Hz |
[stream_keeper]: Stream keeper: the flight controller accepted every rate
[stream_keeper]: Stream keeper: quiet while the mission is in takeoff: TAKING_OFF
[stream_keeper]: Stream keeper: a rate dropped (LOCAL_POSITION_NED 21.0 Hz, ATTITUDE 21.0 Hz); requesting it again
[stream_keeper]: Stream keeper: mission out of its actuation state
[stream_keeper]: Stream keeper: the flight controller accepted every rate
[stream_keeper]: Stream keeper: quiet while the mission is in package_delivery: FINAL_FIXED_COLUMN_DESCENT
[stream_keeper]: Stream keeper: mission out of its actuation state
```

That is the keeper's log for the last flight on the final code (the result line above is from the first of the two flights on the change; the second landed 0.06 m from the target with the same rates). With the hold check, the rate-drop check, the absence of a repeat, or `GROUND_DWELL` removed in turn, 2, 5, 7 and 4 of the tests fail.

## Spec check

spec-auditor, two rounds, INCOMPLETE both times. Round 1: no results or notes file yet, the deviations not recorded, two plugins listed that nothing used, nothing automated that would notice pose falling back to 4 Hz, and no evidence of a re-request after a reboot. Round 2 found all of that closed except the reboot: no running node has detected one. That is why the row is PARTIAL; the stale interval it found in a Decisions row is corrected.

flight-guardian, two rounds, CONCERNS both times, no blocker. Round 1 found, in the sim's MAVROS logs, that the first keeper's bursts through `set_message_interval` produced runs of service timeouts in the group that also serves `set_mode`; that holding the global position at 5 Hz made the missions' hold-point fix staler than in 2026; and that the landed state would not come back after a reboot. All three led to the rewrite and round 2 found them resolved. From round 2:

- **Fixed after the round, not reviewed or flown again:** one message the flight controller keeps refusing no longer starves the ones after it (it waits a full repeat and the others go first; `test_one_message_the_flight_controller_never_gives_does_not_starve_the_others`). No flight exercised that path: SITL accepts all eight.
- **Answered on 18 Nov:** the keeper's requests share MAVROS's command queue with the gripper, the sprayer and arming, and an unacknowledged one could delay a release by up to 5 s. The keeper now sends nothing while a mission is in a release or actuation state and has no periodic repeat. That change had one flight-guardian round of its own (below). The scored flights used `fake_drop`, so no real release has gone through MAVROS beside the keeper; bench step under "Needs a person".

flight-guardian on the 18 Nov change, one round: CONCERNS, no blocker. Done after it and not reviewed again, but flown: the hold now starts with the final column (the delivery sends its release one tick after entering `GROUND_DWELL`), a hold lapses after 60 s of silence from the executor, a message given up is watched again once it arrives, a step of the clock is not read as a rate drop, the sim's rate check also fails a topic that is low in more than a quarter of its windows, and `package.xml` names the two packages the node now imports. Left standing:

- **The hold depends on the executor's log**: its logger name (`mission_executor`), the missions' `State: OLD -> NEW` wording at INFO level, and the state names. A test drives the real missions and fails if those change, but a mission added later with an actuation state named outside the list would not be held for.
- **A request already sent cannot be withdrawn.** One that went out just before the hold began can still be in MAVROS's queue; the lead is now the length of the final column, seconds, not one tick.

What differs from the spec, each recorded under Decisions: the switch is a launch argument, not a `features.` parameter; three messages beyond the spec's five are held at their 2026 rates; the rates are re-requested on a measured rate drop as well as on a reboot; the command goes through `/mavros/cmd/command`.

## Not done

- **A reboot is not detected in the sim**, which is why this row is PARTIAL: SITL's clock does not restart and MAVROS reports no disconnect, so the detection path is proven by unit tests only. A reboot nobody announces now shows as every rate dropping, which the keeper does act on (unit-tested; the SITL reboot run was on the earlier keeper). The bench step (reboot the flight controller, expect the keeper's "restarted" line and 30 Hz again) is under "Needs a person".
- **Nothing starts the keeper on the aircraft yet.** PLT-07's bringup launches MAVROS from `argus_mavros.launch.py` with `stream_keeper:=true`; until then `mavros_boot.sh device` is the 2026 start-up.
- **On the direct link a SITL reboot ends MAVROS's connection** (MAVROS does not reconnect TCP). The default sim still uses MAVProxy's UDP output.
- **The stale-data guard is still off in `delivery/time_base`**: frames are 250 to 280 ms old in the sim. FND-01 and PLT-03.
- `waypoint`, `param` and `distance_sensor` are not loaded; PLT-13 and CEN-03 add theirs when they use them.
