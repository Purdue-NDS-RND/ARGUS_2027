# The way back and the set-down by REC-7's rules, in the sim

2026-11-21 (UTC). `recovery/return_set_down` and `recovery/return_local`: REC-8's scripted check flight (`sim/recovery_check.py`, beside the payload bridge) with the way back and the set-down flown by `drone_mission_core/recovery_return.py`. Vehicle `iris_recovery` (1.755 kg), a 1 kg bucket 14.4 m from the takeoff point, calm air. Nine flights; the last two, one of each scenario on the code as committed, are the ones tabled.

## Commands

```
sim/run_scenarios.sh recovery/return_set_down --seeds 1 --variants on
sim/run_scenarios.sh recovery/return_local --seeds 1 --variants on
python3 sim/carry_swing.py sim/.state/suites/recovery-return_set_down-20261121T032018Z/runs/recovery-return_set_down-s1-on
python3 sim/carry_swing.py sim/.state/suites/recovery-return_local-20261121T031740Z/runs/recovery-return_local-s1-on
```

Both scenarios: `carry_speed_mps: 3.0`, `shaped: true`, `attach_check: true`, `set_down_rule: true`. `return_set_down` adds `return_frame: global` (the takeoff as the target); `return_local` adds `return_frame: local` and `return_target: latlon` (the takeoff point's own latitude and longitude, from the site file, as a published point). The numbers below are from each run's `result.json`, `score.json` and `recovery_check.json`.

## Result

| | `return_set_down`: GPS trusted, to the takeoff | `return_local`: GPS untrusted, to a published point |
| --- | --- | --- |
| Status | OK, real-time factor 0.91; the check's own verdict PASS | OK, 0.91; PASS |
| Judge (recovery, from true positions) | 19.988 points: lifted 2.17 m, the bucket at rest 0.0099 m from the takeoff point, let go at -0.02 m (standing on the ground) | 19.969 points: lifted 2.17 m, at rest 0.0250 m from the takeoff point, let go at -0.02 m |
| The way back | frame `global`: 223 global setpoints and no local one, walked along a shaped leg of 14.42 m to the takeoff's own latitude and longitude (taken before takeoff) | frame `local`: 220 local setpoints and no global one; the published point put at (0.0, 0.0) of the local frame through the takeoff's pair |
| The leg, from true positions (`sim/carry_swing.py`) | 14.45 m in 7.33 s, top speed 2.84 m/s; the bucket at most 0.155 m out during the leg and 0.013 m once stopped (watched 2.95 s) | 14.44 m in 7.43 s, 2.85 m/s; 0.152 m and 0.012 m (2.84 s) |
| Home's height in the local frame | 0.19 m: each global setpoint's height is given that much lower | not used |
| Attach check | attached: throttle 0.32 empty, 0.45 loaded (ratio 1.4062; the rule's model gives 1.2529) | the same: 0.32, 0.45 |
| Touch height (the rangefinder at the grab) | 0.67 m | 0.68 m |
| LOWER | from 2.81 m of rangefinder height to 1.16 m, 5.90 s | 2.81 m to 1.17 m, 5.91 s |
| SET_DOWN | at 0.12 m/s for 4.30 s, until the throttle (0.34) was under the line (0.385) for 0.3 s; the rangefinder read 0.65 m | 4.30 s; throttle 0.35; 0.66 m |
| RELEASE | allowed by the release guard at 0.65 m (ceiling 1.5 m); the gripper's open acknowledged with result 0 within one step; one attempt | at 0.66 m; one attempt |
| CONFIRM | up 0.5 m and the throttle still the empty hover's: confirmed | confirmed |
| Readings | 314 ticks of the rule, no rangefinder height older than 0.2 s, 5 steps longer than 0.15 s | 326 ticks, none, 1 step |

## The seven flights before them, and what they found

| Flight | Status | What happened | What was changed |
| --- | --- | --- | --- |
| 1 | NEVER_ARMED | The tool wanted the takeoff's latitude and longitude from its first step, before a global position had come. | It waits up to 200 positions for the first one. |
| 2 | OK, the check FAILED | The attach check's hold had 1.2 s of fresh throttle in its 1.8 s, and the tool ended the flight. | The hold runs on (up to 5 s more) until the check has its 1.5 s. |
| 3 | OK, the check FAILED | The global leg was flown and the bucket landed 0.013 m from the takeoff point, but the leg never "arrived": a global setpoint's height is above home, which is 0.19 m over the local frame's zero, so the vehicle flew 0.18 m higher than the local height the arrival was judged at. | The difference is read off on the ground before takeoff and every global setpoint's height is given in home's terms. |
| 4 | OK, the check FAILED | The attach check's first hover had under five fresh throttle readings in its two seconds. | The first hover runs on until there are; every throttle sample is kept in the report. |
| 5 | OK, 19.985 points | The way back by global setpoint worked. The set-down rule answered LAND 4.7 s into the lowering, on one rangefinder reading 0.25 s old, landed with the bucket and let it go on the ground. | The rule holds on a height that is not fresh and lands only after 1.0 s without one. The touch height is taken from the rangefinder at the grab (it reads 0.19 m under the local height). |
| 6 | OK, 19.989 points | The same landing, 1.65 s into the lowering: a whole second with no rangefinder reading under 0.2 s old. | See below. |
| 7 | OK, 19.988 points, PASS | The whole of `return_set_down` by the rules: 221 global setpoints, the set-down felt at 0.65 m, released, confirmed; the bucket 0.0096 m from the takeoff point. | After it, from the reviews: readings aged on the autopilot's clock (not the wall's), the confirming climb given the lowering's second of grace, global heights through `global_height_m`. The tabled flight is the same scenario flown again after these. |

Flights 2, 4, 5 and 6 are one fault of the sim's plumbing, not of the rules. The driver's port is an output of MAVProxy, which asks the autopilot for its own stream rates (4 Hz) again and again (`sim/container/start.sh` says as much where it wires MAVROS past it); the tool asked for its own rates only every 2 s, so for up to 2 s at a time positions, throttle and rangefinder all came at 4 Hz, and the tool, which aged a reading on a clock that only moved with each position, saw every one as a step (0.25 s) old. A flight by the rules now asks for its rates every half second and ages a reading by when it came, on the autopilot's clock carried between positions at the sim's own rate. In flight 7 and in both tabled flights no height was stale. The mechanism is inferred from the code and from the flights' logs (steps 0.25 s apart, ages of 0.22 to 0.41 s); MAVProxy's own traffic was not captured.

## What it shows

- A return that is a global setpoint when GPS is trusted and a local one when it is not, walked so that the bucket does not swing, ending 0.01 m and 0.03 m from the takeoff point.
- A published latitude and longitude placed in the local frame through the takeoff's pair, to the centimetre.
- A set-down that is felt, not timed: the release came 0.02 m under the height at which the bucket touches, on the throttle falling from 0.45 to 0.35.
- The release went through REC-4's guard with a fresh rangefinder height of 0.65 m, and the confirming climb saw the empty hover's throttle.

## Not shown

- GPS really untrusted: `return_local` takes it as untrusted by a setting, with the sim's GPS healthy; the EKF's frame did not drift. REC-8's runs with GPS faults fly with the recovery mission.
- A published point with GPS trusted, a marker target, and homing, in a flight: no flight; homing fixes come with REC-7b.
- A refused gripper, a bucket still on the hook after the release, a set-down never felt, in a flight: unit-tested (the last happened unasked in flights 5 and 6, and the rule's landing and release on the ground worked).
- The sim's hook is a rigid rod: when the bucket stands, the vehicle can rest on it, and the throttle falls further than it would on a rope (to 0.0 in flights 5 and 6 once landed on it). On a rope the throttle falls only to the empty hover's.
- `abort_rtl_mode: LAND` in a flight: the params file exists and is tested; nothing launches a recovery mission yet.
- Wind.
