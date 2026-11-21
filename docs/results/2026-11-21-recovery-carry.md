# The shaped carry and the attach check, in the sim (REC-4)

2026-11-21 (UTC). `recovery/hook_carry_shaped`, flown twice (the second flight, on the code as committed, is the one tabled): the scripted check flight of REC-8 (`sim/recovery_check.py`, beside the payload bridge) with the carry to the takeoff point at 3 m/s under REC-4's swing shaping, and REC-4's attach check run on the flight's own hover throttle. The vehicle is `iris_recovery` (1.755 kg) with a 1 kg bucket on a hook; the pendulum from the hinge to the bucket's middle is 0.72 m, a period of 1.70 s.

## Commands

```
sim/run_scenarios.sh recovery/hook_carry_shaped --seeds 1 --variants on
python3 sim/carry_swing.py sim/.state/suites/recovery-hook_carry_shaped-20261121T022453Z/runs/recovery-hook_carry_shaped-s1-on
python3 sim/carry_swing.py sim/.state/suites/recovery-hook_check-20261121T002756Z/runs/recovery-hook_check-s1-on
```

## Result

| | |
| --- | --- |
| Status | OK, real-time factor 0.91; the check's own verdict PASS, landed |
| Scenario settings | `carry_speed_mps: 3.0`, `shaped: true`, `attach_check: true` |
| Judge (recovery, from true positions) | 19.995 points: lifted 2.17 m, released 0.06 m up, the bucket at rest 0.004 m from the takeoff point |
| Grab | the hook 0.010 m sideways and 0.013 m in height from the handle |
| Attach check | attached. Hover throttle 0.31 before the grab (the hook at the handle) and 0.45 in the hold after the lift: a ratio of 1.4516. The rule's expected ratio for these masses is 1.2529 (the square root of 2.755 / 1.755), and it asks for half of that rise: 1.126 |

## The carry, from true positions (`sim/carry_swing.py`)

| | Shaped, 3 m/s (this flight) | Point walked at 1 m/s (`recovery/hook_check`, REC-8) |
| --- | --- | --- |
| Leg | 14.44 m in 7.42 s | 14.42 m in 15.23 s |
| Top speed | 2.94 m/s | 1.17 m/s |
| The bucket out from where it hangs, at most, during the leg | 0.153 m | 0.081 m |
| The same once the vehicle had stopped | 0.012 m (watched 3.1 s) | 0.021 m (watched 1.5 s) |

The first flight (`recovery-hook_carry_shaped-20261121T021240Z`, before the review's changes to the tool: the throttle's age, and the attach check behind its own scenario setting) gave the same: OK, 19.998 points, the same throttle ratio, 14.43 m in 7.43 s, 0.144 m in the leg and 0.012 m after the stop.

## What it shows

- A carry at three times the speed, in half the time, that ends with the bucket hanging still to about a centimetre: the leg's start and stop each last one pendulum period.
- The attach check works on VFR_HUD's throttle, which comes in whole percent: 0.31 and 0.45.
- The throttle rose more than the rule's model says. The rule (and the spec's "about 1.125 for 1 kg on a 4 kg aircraft") takes throttle as the square root of thrust; the sim's autopilot gave 1.45 where that model gives 1.25 and the weights' own ratio is 1.57. The check is a floor (half the modelled rise), so it passed; what the ratio is on the aircraft is a measurement for a person.

## Not shown

- The same 3 m/s carry without shaping: not flown. REC-8's second flight, sent at the autopilot's own speed with no shaping, came down under the swinging bucket; the unit test integrates the pendulum and gets 0.25 m and more of swing from an unshaped change of speed against under 0.02 m from a shaped one.
- A hook that came up empty, in a flight: the attach check's "not attached" is unit-tested on the check flight with a stand-in vehicle.
- The engage (plunge or sweep), the release guard and the loss watch in a flight: they have no caller until the recovery mission (REC-2). This flight's grab and release are the scripted tool's.
- Wind. The flight is in calm air.
