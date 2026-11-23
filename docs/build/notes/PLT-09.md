# PLT-09: Takeover latch, actuation lock, Lua watchdog

Flags: `takeover_latch` and `actuation_lock` (mission flags, default false). The watchdog is a script a person loads on the flight controller; it has no flag · Status: DONE · Commits: `git log --oneline --grep '^PLT-09:' platform/build-2027`

## What it solves

Three ways the 2026 stack could work against the people around it. The executor kept commanding after the safety pilot changed mode. Any code path that reached `command_gripper` or the drop servo's command could move an actuator, whatever state the mission was in. And a companion computer that died in GUIDED left the vehicle flying its last order. Each is fixed at its cause, behind a flag or on the flight controller.

## Architecture

- `drone_mission_core/takeover.py` (new): `TakeoverLatch` (ROS-free rule), `attach_takeover` (the re-arm service `~/rearm_autonomy`, a client of the take-off service's new cancel), `takeover_tick`.
- `drone_mission_core/actuation_lock.py` (new): `ActuationLock` (ROS-free rule), `lock_violations`, `attach_actuation_lock`.
- `mission_executor.py`: attaches both when their flag is on; with the latch on, a latched tick returns before anything is sent; a locked sequence whose release would not ask for a permit does not start.
- `mission_context.py`: `permit_actuation`; `command_gripper` and `command_sprayer(enable=True)` go through the lock when there is one; `request_mode_change` and `request_takeoff` tell the latch what autonomy asked for.
- The gated release paths ask for their permit: the drop's (`package_drop_mission.py`, DROP_PAYLOAD), the delivery's soft release (`package_delivery_mission.py`, GROUND_DWELL), the recovery's close and open (`package_recovery_mission.py`, ENGAGE and SET_DOWN).
- `drone_utils/simple_takeoff_service.py`: behind a new node flag, `features.takeoff_cancel` (false in every params file), a service `drone_utils/cancel_takeoff` (std_srvs/Trigger) that drops a take-off accepted and not yet sent.
- `platform/fcu/scripts/argus_watchdog.lua` (new) and `sim/watchdog_check.py` (new).
- Sim: scenarios may name `fc_scripts` (SITL runs them) and `forbid_log` (text the mission's log must not have); the injector has a `mode` event; six new scenarios. Both flags are declared `false` in the sequence files (`takeover_latch` in all 19, `actuation_lock` in the ten with a mission that actuates).

## Flow

- Takeover, before: the pilot switches to LOITER; the executor goes on publishing setpoints, and a mission that fails asks for RTL over the pilot. After (`takeover_latch`): the mode is one autonomy did not ask for → the executor clears its setpoints, tells the take-off service to drop anything pending, says `TAKEOVER` once, and from then on sends nothing, ticks nothing and asks for nothing, until a person calls `/mission_executor/rearm_autonomy` with the vehicle in GUIDED; then the interrupted mission ends by its failure policy (home, or a landing where one is named) and is never resumed. With the latch on the executor starts no mission until the take-off node's cancel service is there.
- Actuation, after (`actuation_lock`): a release state calls `permit_actuation(actuator, state, gates_hold)` and then the command on the same tick → the lock lets that one command out. A command with no permit behind it, from a state that is not a release state, or with gates that do not hold, is not sent; the mission hears "not accepted".
- Watchdog: the script counts setpoint messages; armed after five in a stretch of flying in GUIDED; more than 1 s without one → LOITER.

## Code

- The latch's rule: `src/drone_mission_core/drone_mission_core/takeover.py:59`; the executor's side: `:119`; the tick that returns: `src/drone_mission_core/drone_mission_core/mission_executor.py:272`.
- The lock's rule: `src/drone_mission_core/drone_mission_core/actuation_lock.py:58` and `:71`; where a command is stopped: `src/drone_mission_core/drone_mission_core/mission_context.py:404`.
- The watchdog: `platform/fcu/scripts/argus_watchdog.lua:44` (the trip at `:61`).
- The take-off service's cancel: `src/drone_utils/drone_utils/simple_takeoff_service.py:283`.

## Evidence

```
$ pytest test_takeover.py test_actuation_lock.py test_executor_takeover.py test_takeoff_cancel.py test_watchdog_check.py   (as scripts/check.sh runs them)
13 passed, 20 passed, 13 passed, 4 passed, 10 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh hover/watchdog                          ->  OK    LOITER 1.04 s after the last setpoint (must be over 1 s, within 2 s)
$ sim/run_scenarios.sh delivery/watchdog --variants off        ->  OK    16.21 points, 0.03 m: the baseline's score, the script loaded and silent
$ sim/run_scenarios.sh delivery/takeover --variants on         ->  OK    TAKEOVER at the injected LOITER; no abort, no mode request, no delivery after it
$ sim/run_scenarios.sh delivery/actuation_lock --variants on   ->  OK    16.53 points, 0.03 m; one gripper command, no refusal, no TAKEOVER
$ sim/run_scenarios.sh recovery/pickup_locked --variants on    ->  OK    19.99 points, 0.01 m; close and open through the lock, no TAKEOVER
$ sim/run_scenarios.sh drop/actuation_lock --variants on       ->  OK    17.47 points, 0.03 m; one drop-servo command, no refusal, no TAKEOVER (a first flight was marked FAIL by CEN-03's height check, 0.102 m against 0.10)
```

SITL is ArduCopter 4.6.3, the aircraft's version. `docs/results/2026-11-23-takeover-lock-watchdog.md` has the tables and the two flights that went wrong on the way (an injector bug that left the first takeover flight INVALID, and a status text that was too long). The rule tests were written before their modules; the executor, wiring and watchdog tests after.

## Spec check

- spec-auditor, round 1: INCOMPLETE on five rows. Fixed: the take-off node would have asked for GUIDED again over a pilot who took over between arming and the take-off (it now has a cancel, behind its own node flag, which the latch calls); the drop servo and the recovery's gripper had not been run against the real lock (two more flights); no healthy flight had flown with the latch on (the three locked flights now do, and must not say TAKEOVER); the recovery passed the lock a constant yes (it now says whether the command is its state's own). Recorded under "Decisions": the watchdog's arming condition and what it leaves unwatched, ENGAGE as a release state, the unlocked servo-off. No second round.
- flight-guardian, round 1: CONCERNS, no blocker, six rows. Fixed: the cancel is asked for every second until it answers; the cancel service is behind a node flag; a re-arm no longer resumes a mission with its old timers; an abort interrupted by a takeover asks again.
- flight-guardian, round 2: CONCERNS, no blocker; three rows resolved. Acted on after the round, unit-tested, not reviewed again: with the latch on no mission starts until the cancel service is there (two flags in two files); a re-arm ends the interrupted mission by its failure policy instead of entering it again (a recovery with the bucket on its hook, a take-off in mid-air). Left standing, for the lead ("Needs a person"): the watchdog's LOITER latches the executor out, a failed mission's abort included; the script is behind no flag and only the delivery has flown with it; in LOITER the throttle stick is the climb rate.

## Not done

- The aircraft: the watchdog loaded and tripped in a hover, a real transmitter switch against the latch, the lock with the real actuators ("Needs a person").
- Only the 2026 delivery has flown with the watchdog loaded. The waypoint, drop, survey and recovery missions have not, and the competition's combined sequences (`final_waypoint_drop.yaml`, `final_curcuit_delivery.yaml`) clear their setpoints at the hand-over from waypoints to the drop or delivery: if the detector's switch-on is slow there, the gap could pass a second. The script is not to be loaded for a mission that has not flown with it in the sim.
- With the watchdog loaded and the latch on, a watchdog LOITER is a mode nobody asked for: the executor latches out and stays in LOITER until the pilot or the battery failsafe. That includes a failed mission whose abort RTL is not taken within a second.
- The watchdog does not catch a companion that dies during a take-off or before its fifth setpoint after one.
- The latch does not act before arming: a mode switched on the pad while disarmed is asked back to GUIDED by the take-off node, as in 2026.
- A command refused by the lock in the air, the re-arm service, and the take-off cancel dropping a real pending take-off are unit-tested, not flown.
- `MissionContext.set_abort_mode` (REC-2) and the latch: a latched executor does not run its abort at all, whatever mode is named.
