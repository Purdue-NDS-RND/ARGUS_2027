# Takeover latch, actuation lock, Lua watchdog (PLT-09): sim flights

2026-11-23, build machine. ArduCopter SITL V4.6.3 (the aircraft's own version). One flight of each, seed 1. The aircraft is a person's to try (`docs/build/PROGRESS.md`, "Needs a person", PLT-09).

## The watchdog (`platform/fcu/scripts/argus_watchdog.lua`)

```
$ sim/run_scenarios.sh hover/watchdog
   OK          hover/watchdog seed 1 off: 1.00 points, held within 5.08 m for 56 s
$ sim/run_scenarios.sh delivery/watchdog --variants off
   OK          delivery/watchdog seed 1 off: 16.21 points, 0.03 m from the target
```

| Step of `sim/watchdog_check.py` | Must | Measured |
| --- | --- | --- |
| Take-off, no setpoints sent | still GUIDED | GUIDED |
| 3 s hover, no setpoints sent | still GUIDED (nothing had started) | GUIDED |
| 6 s of position setpoints at 10 Hz | GUIDED, and the vehicle follows | GUIDED; moved 5.01 m to a setpoint 5 m away |
| Setpoints stop | LOITER more than 1 s and no more than 2 s after the last one | LOITER after 1.04 s; `ARGUS watchdog: setpoints stopped, now LOITER` |

The script as first written was flown too (LOITER after 1.11 s); its message was longer than a status text carries and came through cut off, so it was shortened and the check flown again: the numbers above.

The 2026 delivery with every flag off and the script loaded scored what `delivery/baseline` scored at gate G0-W3 (16.21 points, 0.03 m from the target, seed 1). SITL's log for that flight has `ARGUS watchdog loaded` and no other line from the script, and the mission's log has no LOITER: the watchdog did not act in the take-off, at the delivery on the ground, or in the relaunch (18 s in GUIDED with no setpoints). No other mission has been flown with the script.

## The takeover latch (`takeover_latch`)

```
$ sim/run_scenarios.sh delivery/takeover --variants on
   OK          delivery/takeover seed 1 on: 0.00 points, nothing released
```

The 2026 delivery with the latch on (and the take-off node's `features.takeoff_cancel`). `sim/inject.py` changed the mode to LOITER 45.0 s after arming and to LAND at 60.0 s (`flight mode LOITER taken`, `flight mode LAND taken`). The executor's log:

```
[mission_executor]: Takeover latch on: a mode autonomy did not ask for stops the executor until it is re-armed by hand
[mission_executor]: TAKEOVER: the mode went from GUIDED to LOITER, which autonomy did not ask for. The executor sends nothing and asks for nothing until autonomy is re-armed by hand (in GUIDED: ros2 service call /mission_executor/rearm_autonomy std_srvs/srv/Trigger)
```

and nothing from the executor after that line: `expect_log.missing` is empty, and none of the lines it must not have is there (`Mission failure policy triggered abort`, `Abort RTL mode request sent`, `Mission 'package_delivery' completed`). The vehicle landed where the injector's LAND put it and disarmed. Nothing was delivered, which is the point: 0 points. That no setpoint was published after the takeover is the unit test's proof (`test_executor_takeover.py`), not this flight's.

The first flight of this scenario was INVALID, rightly: the injector's new mode event refused its own checked form and never ran, so the mission flew on to a normal delivery (16.20 points). The runner would not count a flight whose events were not injected. Fixed, with a test.

A healthy flight with the latch on is not latched out: the three locked flights below had `takeover_latch` on as well, with `TAKEOVER` among the lines their logs must not have.

## The actuation lock (`actuation_lock`)

| Flight (flags on) | Without the lock | With the lock and the latch on |
| --- | --- | --- |
| `delivery/actuation_lock` (the soft set-down of a 0.5 kg package) | `delivery/set_down`: 16.54 points, 0.02 m (20 November) | OK, 16.53 points, 0.03 m; `Releasing gripper payload` once, no refusal |
| `drop/actuation_lock` (the gated drop of a 0.5 kg package) | `drop/gated`: 17.45 points, 0.04 m (gate G2, seed 1) | OK, 17.47 points, 0.03 m; `Closing sprayer for payload drop` once, no refusal. (A first flight dropped to 0.04 m for 17.45 points and was marked FAIL by the height estimator's own check, CEN-03's: rangefinder heights 0.102 m from the truth at the 95th percentile against a limit of 0.10. The drop and the lock did what they do in both.) |
| `recovery/pickup_locked` (the recovery on the oracle stand-in) | `recovery/pickup_oracle`: 19.99 points, 0.01 m | OK, 19.99 points, 0.01 m; the close from ENGAGE and the open from SET_DOWN both went out, no refusal; the mission's own LAND at the end was not taken for a takeover |

```
$ sim/run_scenarios.sh delivery/actuation_lock --variants on
   OK          delivery/actuation_lock seed 1 on: 16.53 points, 0.03 m from the target
```

Each has `Actuation lock on` in its log and no `Actuation lock: the ... command was not sent`. A command refused by the lock in the air has not been flown: that is `test_executor_takeover.py`, on the real node and context.

## After the reviews

Two changes came after the flights above and are unit-tested: with the latch on the executor starts no mission until the take-off node's cancel service is there, and a re-arm ends the interrupted mission by its failure policy. The locked delivery was flown once more on that code, with the take-off node's `features.takeoff_cancel` on: OK, 16.53 points, 0.03 m; `Takeoff cancel service ready on drone_utils/cancel_takeoff`, then the sequence began as usual; one gripper command, no refusal, no TAKEOVER.
