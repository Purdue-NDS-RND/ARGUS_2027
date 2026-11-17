# FND-03: Time base (MAVROS time sync, sim clock, stale-data guard)

Flags: `time_health_guard` (mission flag, default false, declared in the six drop and delivery sequences); `features.stamp_from_image` (node flag on `target_cv`, default false); in the sim, `SIM_TIME=1` / scenario `sim_time: true` / launch argument `use_sim_time:=true` (all default off) · Status: DONE · Commits: `git log --oneline --grep '^FND-03:' platform/build-2027`

## What it solves

In 2026 nothing checked that a frame and a pose belonged to the same moment: the sim mixed wall time and sim time, MAVROS's pose stamps were never verified, and a mission steered on whatever arrived last. This gives the stack one time base it can check (the cause, for the sim), and a guard that stops a vision mission steering or descending when the time base cannot be trusted (a safety net, behind a flag).

## Architecture

- `config/mavros/argus_mavros.yaml` (new): the `sys_time` block, time sync at 10 Hz. `config/mavros/argus_mavros.launch.py` (new) loads it after MAVROS's own ArduPilot config and takes `use_sim_time`. `mavros_boot.sh` is untouched. PLT-06 fills the rest of the file.
- `drone_mission_core/time_health.py` (new, no ROS): `TimeHealth` and `TimeHealthReport`, and `is_synced`, which tells a synchronised MAVROS stamp from an arrival-time one.
- `drone_mission_core/time_guard.py` (new): `attach_time_health` (the executor's three extra subscriptions, only with the flag on) and `TimeGuard` (a mission's view, with 1 s of hysteresis and a hold timer).
- `MissionContext.time_health()` (new call; `None` with the flag off). New subscription with the flag on: `/mavros/timesync_status`.
- `drone_target_cv/detection_stamp.py` (new, no ROS) and the `features.stamp_from_image` parameter on `target_cv`.
- Sim: `sim/clock_bridge.py` (Gazebo clock to `/clock`, one message per 4 ms of sim time), `sim/time_check.py` (the proof, and `--wait-synced`), a `sim_time` scenario key, the scenario `delivery/time_base`, and a `use_sim_time` argument on the twelve executor launch files.

## Flow

Before: in the sim the executor, the detector and MAVROS ran on the wall clock while the camera's stamps were sim time. A pose was stamped by MAVROS and used by the mission with no check of its age.

After, everything off: the same, command for command (tests drive the real missions with a context that raises if the time base is consulted).

After, in a `sim_time: true` flight: Gazebo clock → `sim/clock_bridge.py` → `/clock` → every node. MAVROS exchanges TIMESYNC with SITL and stamps a pose `time_boot_ms + offset`. `sim/time_check.py` listens for the whole flight and the run is INVALID unless the stamps agree with `/clock`.

After, with `time_health_guard` on: each tick the mission asks `context.time_health()`. UNHEALTHY while tracking → `_has_recent_target_detection` is False → the 2026 "target lost" path (zero velocity for the grace time, then the recovery climb). UNHEALTHY in the blind column → hold the column's point at the vehicle's height; FAILURE after 10 s.

## Code

- `time_health.py:150`, `report`: pose age over 100 ms, frame age over 250 ms, an offset step over 10 ms (held for 1 s), time sync silent for 2 s, a stamp ahead of the clock, anything never heard, or no synchronised pose stamp in the last 0.5 s.
- `time_health.py:48`, `is_synced`: a synchronised stamp minus MAVROS's offset is a whole number of milliseconds. This is how the guard and the sim check know a stamp is flight-controller time and not the arrival time MAVROS uses until its filter converges.
- `time_guard.py:90`, `TimeGuard.unhealthy`: UNHEALTHY from the first bad report until 1 s of good ones, so a flickering time base neither resets the tracker every tick nor dodges the hold timeout.
- `red_bullseye_mission_base.py:141`, `:150` and `:249` (`_time_is_unhealthy`, `_hold_for_time_guard`, and the check in `_has_recent_target_detection`), and the two column handlers (`package_drop_mission.py:228`, `package_delivery_mission.py:282`): the only edits to 2026 mission code, each behind `get_feature_flag`.
- `sim/container/start.sh`: three `if [[ $SIM_TIME == 1 ]]` branches (clock bridge, MAVROS from the ARGUS launch, wait for time sync). The other branch is the 2026 start-up.

## Evidence

```
$ PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_time_health.py
42 passed in 0.03s
$ python3 -m pytest -q -p no:cacheprovider src/drone_mission_demo/test/test_time_guard_missions.py
32 passed in 0.05s
$ python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_executor_time_health.py
4 passed in 0.08s
$ PYTHONPATH=src/drone_target_cv python3 -m pytest -q -p no:cacheprovider src/drone_target_cv/test/test_detection_stamp.py
6 passed in 0.02s
$ python3 -m pytest -q -p no:cacheprovider sim/test/test_time_base.py
50 passed in 0.18s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

The tests were written first and failed before the code existed (import errors; 19 of 24 mission cases). With the guard's answer forced to "healthy" once, 15 mission cases failed; it was put back.

Sim, flags on (`sim_time` and the detector's `stamp_from_image`), three flights of `sim/run_scenarios.sh delivery/time_base --variants on`:

```
   OK          delivery/time_base seed 1 on: 16.18 points, 0.06 m from the target      p95 16.0 ms
   INVALID     delivery/time_base seed 1 on: 16.14 points, 0.09 m from the target      p95 23.0 ms
   INVALID     delivery/time_base seed 1 on: 16.22 points, 0.02 m from the target      p95 23.9 ms
```

**The sim pass line (20 ms) was met in one flight of three.** In all three, 65 of 65 nodes were on sim time and all but one to three stamps were synchronised flight-controller time; the median difference is 10 to 13 ms. Numbers, the MAVROS convergence story and the CPU cost of a full-rate clock are in [docs/results/2026-11-17-time-base.md](../../results/2026-11-17-time-base.md).

Flags off, after the launch-file, executor and mission edits (`sim/.state/fnd03/smoke_off.log`):

```
$ sim/run_scenarios.sh smoke
| delivery/baseline | 1 | OK | 16.21 | 0.03 m from the target | not run |  |  |
```

Real-time factor 0.97, and the launch command carried no `use_sim_time`. That flight was before the last detector edit (the clock read moved back to where 2026 has it, at the publish); flags off, that edit is covered by `test_detection_stamp.py` and by G0-W2 at the end of wave 2.

**After PLT-06** (MAVROS on SITL's own port with the stream keeper), the same scenario, four flights: pose stamps 4.6, 5.4, 4.4 and 4.6 ms from the sim clock at the 95th percentile, pose at 30 Hz, the last flight OK at 0.02 m. The 20 ms pass line is met with margin and the row is DONE. [docs/results/2026-11-17-mavros-rates.md](../../results/2026-11-17-mavros-rates.md).

## Spec check

spec-auditor, two rounds, INCOMPLETE both times. Round 1: the sim result was not written, the frame age was measured from a `now()` stamp, and four choices were not recorded (sim time opt-in, the 95th-percentile reading of "within 20 ms", the extra UNHEALTHY causes, the hold timeout). Round 2 found all of those closed and two new things: my detector edit had moved the flags-off stamp earlier than in 2026, and a "Needs a person" sentence said the guard had flown. Both were fixed after round 2 and not reviewed again; the first now has its own test.

flight-guardian, two rounds: BLOCKER, then CONCERNS. The blocker was the `now()` frame stamp, answered by the detector's `features.stamp_from_image`. Fixed from its concerns: the guard read HEALTHY on arrival-time pose stamps (now the synchronised-stamp proof, two-sided), a flickering time base could stall the column for ever (1 s of hysteresis), the hold was at the column's setpoint instead of the vehicle's height, and a dead pose stream left the recovery climb waiting for ever (it now fails after the hold timeout). The last two fixes were made after round 2 and not reviewed again. Concerns left standing:

- **Two flags, no coupling.** `time_health_guard` on with `features.stamp_from_image` off still measures frame age from the detector's clock, and nothing warns. PLT-07's profiles should turn them on together.
- **On the aircraft the image stamp is the grabber's publish time**, not capture time, until FND-01.
- **`stamp_from_image` on with the nodes on the wall clock in the sim** makes every detection look decades old (the image stamps are sim time). The mission fails safe, but nothing prevents the mix.
- **A fault that recurs every 1.1 to 1.5 s never times out**; it slows the column to under a third of its rate.
- **Not verified against MAVROS's source on the aircraft:** that a real clock step shows as a step of more than 10 ms in the filtered estimate (the filter's gain is low; the check may only fire on a filter reset).

What differs from the spec, each recorded under Decisions: sim time is opt-in; `/clock` comes from our own thinned bridge, not `ros_gz_bridge`; "within 20 ms" is judged at the 95th percentile, against Gazebo's own clock.

## Not done

- **The guard has not completed a flight with its flag on.** Pose age no longer trips it (PLT-06 holds pose at 30 Hz), but frame age does: in the sim a frame is 250 to 282 ms old when the mission can use its detection, against the 250 ms limit, and the one flight with the guard on held, climbed and failed to RTL. That is the guard working on a camera chain that is too slow. FND-01 and PLT-03 own that chain; the flag goes into `delivery/time_base` when frames are fresh enough.
- **Not flyable on the aircraft yet.** `mavros_boot.sh device` does not load `argus_mavros.yaml`, and the frame age is only the frame's with `features.stamp_from_image` on (grabber publish time until FND-01 makes it capture time). The bench step is under "Needs a person".
- **Sim time is opt-in**, so flags-off flights and G0 are unchanged. Making it the sim's default is the lead's call.
- **Only `local_position/pose` is checked** against `/clock`; IMU and GPS stamps go through the same MAVROS code and were not measured.
