# Wind compensation (CEN-07): the simulated loop, then holds in the sim's strong wind, 2026-11-24 and 25

## Commands

```
python3 tools/cen_loop_sim.py --steady-wind 0.3 --seconds 60
python3 tools/cen_loop_sim.py --steady-wind 0.3 --wind strong --seconds 120
sim/run_scenarios.sh drop/hold_6m_strong --variants on
sim/run_scenarios.sh drop/hold_6m_strong_wind_comp --variants on
sim/run_scenarios.sh drop/hold_2m_strong --variants on
sim/run_scenarios.sh drop/hold_2m_strong_wind_comp --variants on
```

The holds are the 2026 drop mission made to hold over a bullseye on the retuned controller (CEN-02) in the sim's strong wind, seed 1; the `_wind_comp` ones have the mission flag `wind_compensation` on as well. The hold error is the judge's: the vehicle's true distance from the target's centre over the whole time at the hold height. One flight each. The two without the flag were flown on 24 November; the two with it were flown on 25 November on the code as committed (the command held to the controller's slew limit, nothing learnt from a track that is only remembered). The same two flown the day before, without those two rules, gave 0.344 m and 0.311 m p95.

## The simulated loop

A steady push of 0.3 m/s that the flight controller is taken to leave (the vehicle moves over the ground at what it is asked plus the push):

```
2027 retune, kp 0.60     6.0 m  calm, 0.3 m/s steady  settles never  overshoot 0.57 m  hold p95 0.56 m  max command 0.48 m/s  rings
2027 retune, kp 0.60, wind compensation   6.0 m  calm, 0.3 m/s steady  settles   7.5 s  overshoot 0.13 m  hold p95 0.05 m  max command 0.43 m/s  rings
2027 retune, kp 0.60     6.0 m  strong, 0.3 m/s steady  settles never  overshoot 0.97 m  hold p95 0.86 m  max command 0.48 m/s
2027 retune, kp 0.60, wind compensation   6.0 m  strong, 0.3 m/s steady  settles never  overshoot 0.40 m  hold p95 0.39 m  max command 0.48 m/s
```

With such a push the proportional controller sits downwind by the push over its gain (0.56 m of the 0.5 m that is 0.3 / 0.6), and the compensation takes it out (0.05 m). ("Overshoot" and "rings" here are the push carrying the vehicle past the target from the upwind side it started on, not the loop ringing.) In the loop a plain step with no wind overshoots 0.06 m with the compensation on, against none without.

## The flights

| Scenario | Wind compensation | Run | Hold error: p95; median; max | Time at the hold height |
| --- | --- | --- | --- | --- |
| `drop/hold_6m_strong` | off | OK | 0.280 m; 0.097 m; 0.691 m | 283 s |
| `drop/hold_6m_strong_wind_comp` | on | OK | 0.341 m; 0.112 m; 0.869 m | 285 s |
| `drop/hold_2m_strong` | off | OK | 0.272 m; 0.095 m; 0.763 m | 393 s |
| `drop/hold_2m_strong_wind_comp` | on | OK | 0.292 m; 0.108 m; 1.046 m | 387 s |

## What it shows

**In the sim's strong wind the compensation does not help: it is a little worse.** At both heights the 95th percentile, the median and the largest error are all a little larger with the flag on (p95 0.341 against 0.280 m at 6 m, 0.292 against 0.272 m at 2 m). One seed each: a few centimetres is within what one seed's gusts do, but nothing here says it helps. The sim's wind is a mean with gusts, and ArduPilot's own velocity loop already holds the commanded velocity against the mean: there is no steady push left over for the observer to find (the median error is a tenth of a metre with or without), and what the two terms add is a slow reaction to gusts that have already passed. The judge's figure is a distance; the mean offset along the wind, which is what "downwind" means, was not taken from these runs. The retune's own result of 20 November for the 6 m hold was 0.270 m; today's 0.280 m without the flag is the same controller on today's code.

The case the spec describes, a vehicle "sitting downwind of the target", is a flight controller that leaves a steady velocity error in wind. The simulated loop has it when told to; the sim's Iris does not show it. Whether the aircraft does is a field question: the hold's median error in a steady wind, with the retune alone.

## Not shown

More than one seed; any wind but the sim's "strong"; the aircraft.
