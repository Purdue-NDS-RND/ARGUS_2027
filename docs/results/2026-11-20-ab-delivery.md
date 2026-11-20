# A/B: delivery_soft_set_down, flags off against flags on

Plan `delivery` of `sim/ab/plans.yaml`: seeds 1; calm: delivery/live_tuning_2027; wind: delivery/live_tuning_2027_wind.

Error is where the package came to rest (or the vehicle was at a fake release) from the target's centre. A failure is a flight that did not score; a flight that released but missed a check beside it is listed apart. Time is the median flight time of the flights that scored.

| Air | Off: flights | failures | median error | largest | time | On: flights | failures | median error | largest | time |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| calm | 1 | 0 | 0.44 m | 0.44 m | 186 s | 1 | 0 | 0.04 m | 0.04 m | 138 s |
| wind | 1 | 0 | 0.59 m | 0.59 m | 196 s | 1 | 1 | n/a | n/a | n/a |

## Verdict

FAIL: delivery_soft_set_down stays off.

- wind: 1 failure with the flags on, 0 with them off
- wind: no flight with the flags on scored, so there is no error to judge

Pass lines: no release above 1.83 m; no more failures than with the flags off. The spec gives the delivery's error no line: it is reported.

## Flown

- `sim/run_scenarios.sh delivery/live_tuning_2027 --seeds 1 --variants off,on`
- `sim/run_scenarios.sh delivery/live_tuning_2027_wind --seeds 1 --variants off,on`

## About these flights

Measured on 20 Nov 2026 (UTC) on a club laptop, on `platform/build-2027` at `2a53273` plus the CEN-12 change. Sim image `argus-sim:humble`. One seed a side: every number above is one flight.

```
python3 sim/ab/run_ab.py delivery --seeds 1 --report docs/results/2026-11-20-ab-delivery.md
```

Both sides fly `sim_package_delivery_2027.yaml` (the live file's tuning): flags off is the 2026 live delivery, on is the 2027 flight flags with `delivery_soft_set_down`. Releases are fake.

| Scenario | Flags | Run | Error | Flight time |
| --- | --- | --- | --- | --- |
| delivery/live_tuning_2027 | off | OK | 0.44 m | 186 s |
| delivery/live_tuning_2027 | on | OK | 0.04 m | 138 s |
| delivery/live_tuning_2027_wind | off | OK | 0.592 m | 196 s |
| delivery/live_tuning_2027_wind | on | OK by the runner, nothing delivered | n/a | 155 s |

**The FAIL is real: in strong wind, with the flags on, this delivery was not made.** The mission lost its track at about 6.1 m above the ground four times and used all four recovery attempts:

```
[package_delivery] Entering final set-down at 11.00 m (threshold 8.00 m)
[package_delivery] final set-down: 6.27 m on the rangefinder, error 0.24 m
[package_delivery] final set-down: 6.10 m on the rangefinder, error 0.20 m
[package_delivery] no target to centre on: bullseye track 1 is LOST
[package_delivery] Target lost, looking again from 6.1 m above the ground over its last estimate (3 x 0.08 m + 2.5 m in view; already above the 5.9 m that shows it all; attempt 1/4)
...
[package_delivery] Target recovery exceeded 4 attempts
Mission 'package_delivery' ended with status FAILURE
```

What the log shows: leaning into the wind, the vehicle sees the 5 m bullseye leave the picture from about 6.3 m (level, it fits down to 5.4 m), so the box detector's box is cut and the estimator refuses it; the ring fit, which takes over where the box is cut, was set to look only below 6.0 m on the height above home, and the vehicle was at 6.3 m by that height. Off the centre by 0.17 to 0.28 m in the gusts (0.20 m counts as centred), it did not descend, and hovered in the band where neither source gave a centre until the track was LOST. Each look found the target again (it was never far) and the same thing happened. The descend rule (CONFIRMED only) was not changed. The drop's wind flights went through the same band and got through, one of them on its last recovery attempt (`docs/results/2026-11-20-ab-drop.md`: seed 3 looked four times).

## After letting the ring fit look from 8 m

The sim's ring fit limit was raised from 6.0 to 8.0 m (`sim_target_mission.yaml`, the node's `max_height_m`; the node's own default is still 4 m), so that the two sources overlap when the vehicle leans, and the two wind scenarios were flown again with the flags on:

```
sim/run_scenarios.sh delivery/live_tuning_2027_wind --seeds 1 --variants on
sim/run_scenarios.sh drop/live_tuning_2027_wind --seeds 1 --variants on
python3 sim/ab/run_ab.py delivery --seeds 1 --from sim/.state/suites/delivery-live_tuning_2027-20261120T141732Z sim/.state/suites/delivery-live_tuning_2027_wind-20261120T142626Z sim/.state/suites/delivery-live_tuning_2027_wind-20261120T143645Z
```

| Air | Off: flights | failures | median error | largest | time | On: flights | failures | median error | largest | time |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| calm | 1 | 0 | 0.44 m | 0.44 m | 186 s | 1 | 0 | 0.04 m | 0.04 m | 138 s |
| wind | 1 | 0 | 0.59 m | 0.59 m | 196 s | 1 | 0 | 0.40 m | 0.40 m | 165 s |

`PASS: delivery_soft_set_down may be turned on, by CEN-12's pass lines.` (The harness takes the later of two flights of the same scenario, seed and side.)

- The delivery in wind was made, with no lost track and no recovery: rings found in 574 of 1849 frames looked at, from 8 m down. The drop in wind (seed 1) released on its gates 0.09 m from the target, and this time its projection check was inside its line.
- **0.40 m in wind is the set-down's own cost.** The hold was taken 0.17 m from the target (0.20 m counts as centred), and over the last 0.6 m, where the spec has the vehicle hold its position and make no corrections, the gusts moved it to 0.40 m by the time it touched. The 2026 delivery in the same wind and seed was 0.59 m off. The spec gives the delivery's error no A/B line (gate G3's is a 0.5 m median, flown calm).
- One seed. The verdict above is one flight a side and is not the decision: the plan's three seeds are twelve flights (`python3 sim/ab/run_ab.py delivery`).

