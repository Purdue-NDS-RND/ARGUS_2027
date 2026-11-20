# The retuned centring controller: the loop simulator, then holds in wind in Gazebo

Measured on 20 Nov 2026 (UTC) on a club laptop, on `platform/build-2027` at `2c31c76` plus the CEN-02 change. Sim image `argus-sim:humble`. Holds are the 2026 drop mission made to hold over a bullseye for 20 s on its centring (`drop/hold_*`, see `2026-11-20-centring.md`), one flight each, seed 1, with `use_frame_time_pose`, `use_target_estimator`, `centre_on_estimator` and `centring_retune` on. The wind is from 225°: moderate 4 m/s with 1 m/s gusts, strong 7 m/s with 2 m/s gusts and one 3 m/s gust.

**Result.** With the controller as committed (the spec's soft deadband 0.03 m, Kp 0.6, cap clamp(0.08 h, 0.25, 1.0) m/s and slew 0.5 m/s², plus a 0.5 s lead the spec does not have), the vehicle's true distance from the target's centre at the 95th percentile was 0.10 m in moderate wind and 0.27 m in strong wind, at 6 m and at 2 m. Gate G1 allows 0.20 m and 0.30 m. Without the lead the spec's control held 0.12 m in moderate wind but 0.335 m in strong wind.

| Scenario | Run | Hold error: p95; median; max | Time at the hold height | G1 allows |
| --- | --- | --- | --- | --- |
| `drop/hold_6m_moderate` | OK | 0.103 m; 0.040 m; 0.111 m | 19 s | 0.20 m |
| `drop/hold_2m_moderate` | OK | 0.100 m; 0.043 m; 0.107 m | 19 s | 0.20 m |
| `drop/hold_6m_strong` | OK | 0.270 m; 0.097 m; 0.691 m | 286 s | 0.30 m |
| `drop/hold_2m_strong` | OK | 0.266 m; 0.095 m; 0.762 m | 269 s | 0.30 m |

In strong wind the hold lasts minutes, not 20 s: the mission must be centred within 0.22 m for 20 s without a break before it releases, and a gust keeps breaking that (38 and 49 starts of the 20 s). The whole time at the hold height is what is measured, so the strong-wind numbers include every gust in five minutes, the 3 m/s one among them.

The margin in strong wind is 0.03 m on one seed. It is a pass, not a comfortable one.

## How the controller got its lead

Experiment flights of the two strong-wind holds, in the order flown. Each is one flight, so differences of a few centimetres between rows are within what one seed's gusts can do; the drop from the first two rows to the last two is not.

| Controller | 6 m, strong: p95 (time at the height; starts of the 20 s) | 2 m, strong: p95 |
| --- | --- | --- |
| The spec's: Kp 0.6 | 0.335 m (400 s; 68) | 0.310 m (1.5 m target) |
| Kp 1.0 | 0.305 m (287 s; 46) | 0.301 m (1.5 m target) |
| Kp 1.0, lead 0.5 s | 0.219 m (33 s; 4) | 0.281 m (1.5 m target; the run timed out) |
| Kp 0.6, lead 0.5 s | 0.232 m (63 s; 5; the run timed out after landing) | 0.264 m (1.0 m target) |
| As committed (Kp 0.6, lead 0.5 s), the table above | 0.270 m (286 s; 38) | 0.266 m (1.0 m target) |

- **The commands were nowhere near the speed cap** in the first flight (median 0.06 m/s, 95th percentile 0.18 m/s against a cap of 0.48 m/s at 6 m), so the cap was not what limited the hold.
- **The mean offset was 0.03 m.** The vehicle does not sit downwind of the target; an integral term would have had nothing to remove.
- **The error swings.** Its autocorrelation is 0.48 at 1 s and negative from 2 s to 8 s (-0.28 at 4 s): a period of about 7 s, shorter with Kp 1.0. That is the loop itself, ringing against the vehicle's slow answer to a velocity command, set going by gusts. More gain alone makes it ring faster, which is why Kp 1.0 bought so little.
- **The lead damps it**: the error is taken for where the vehicle will be half a second on at its present velocity. With it the excursions past 0.22 m became rare enough at 6 m for the 20 s to complete in a handful of tries in two flights, though not in the committed code's flight (38 tries).

## At 2 m the target has to stay in the picture

With the 1.5 m bullseye the 2 m hold in strong wind lost its target again and again (30 to 40 "track is LOST" lines a flight, each good for 2 s; two recovery climbs). In a 7 m/s wind the vehicle holds a tilt of about 8° (`wind.log`: 7.7°), which at 2.2 m moves the hard-mounted camera's view by 0.3 m; the image is 2.0 m tall there, so a 1.5 m target is cut by its edge whenever the tilt and the error add up, and a box cut by the edge is not a measurement of the centre. The 2 m wind holds now use a 1.0 m target, and the track still went LOST about eight times in the strong-wind flight. This is the near field CEN-13's ring fit is for.

## Two strong-wind runs timed out after doing everything

The Kp 0.6 with lead flight at 6 m released after 156 s, returned, landed, and never disarmed: the wind tool went on pushing the standing vehicle with 2 to 4 N, the flight controller never saw it landed, and the run ended as a TIMEOUT at 600 s. The wind on the vehicle now fades out in the last half metre above where it stood (`sim/wind_model.py`, `near_ground_factor`). All four flights of the table ended in a disarm.

## The loop simulator

`tools/cen_loop_sim.py` runs the same controller class in a simulated 20 Hz loop with one tick of delay, a noisy estimate, a vehicle that follows a velocity command with a lag, and gusts.

```
$ python3 tools/cen_loop_sim.py --sweep-kp 0.3,0.6,1.0,1.5,2.5,4.0 --height 15
kp 0.30  15.0 m  settles   8.0 s  overshoot 0.00 m
kp 0.60  15.0 m  settles   4.5 s  overshoot 0.00 m
kp 1.00  15.0 m  settles   3.4 s  overshoot 0.02 m
kp 1.50  15.0 m  settles   2.8 s  overshoot 0.06 m
kp 2.50  15.0 m  settles   5.2 s  overshoot 0.33 m  rings
kp 4.00  15.0 m  settles  13.9 s  overshoot 0.57 m  rings
$ python3 tools/cen_loop_sim.py --seconds 60        # the 6 m rows
2026 demo                6.0 m  calm      settles   8.5 s  overshoot 0.00 m  hold p95 0.06 m  max command 0.51 m/s
2026 live                6.0 m  calm      settles never  overshoot 0.00 m  hold p95 0.29 m  max command 0.31 m/s
2027 retune, kp 0.60     6.0 m  calm      settles   6.6 s  overshoot 0.02 m  hold p95 0.02 m  max command 0.48 m/s
2026 demo                6.0 m  moderate  settles never  overshoot 0.11 m  hold p95 0.26 m  max command 0.51 m/s
2026 live                6.0 m  moderate  settles never  overshoot 0.00 m  hold p95 0.53 m  max command 0.30 m/s
2027 retune, kp 0.60     6.0 m  moderate  settles  58.2 s  overshoot 0.08 m  hold p95 0.14 m  max command 0.48 m/s
2026 demo                6.0 m  strong    settles never  overshoot 0.36 m  hold p95 0.62 m  max command 0.51 m/s
2026 live                6.0 m  strong    settles never  overshoot 0.34 m  hold p95 0.94 m  max command 0.30 m/s
2027 retune, kp 0.60     6.0 m  strong    settles never  overshoot 0.21 m  hold p95 0.35 m  max command 0.48 m/s
```

The gain limit a sequence is held to, 1.0 (0.6 with a lead under 0.5 s), comes from the sweep and from what was flown: with the lead a step of 0.8 m overshoots 0.02 m at 1.0 and rings by 2.5; without it, 0.04 m at 0.6 and 0.12 m at 1.0.

**How far to trust it.** Its first version (a vehicle lag of 0.5 s, gusts guessed) said Kp 1.0 would take a third off the gust error with no ringing; Gazebo said a tenth. The lag (0.9 s) and the gusts' size are now fitted to the Gazebo holds, and it still under-states the lead: its strong-wind hold at 6 m is 0.35 m with the lead where Gazebo flew 0.22 to 0.27 m. Use it to see whether a gain rings and how fast it settles. It does not predict a hold error.

## How it was measured

```
$ sim/run_scenarios.sh G1
| drop/hold_2m_moderate | 1 | not run |  |  | OK | 17.45 | 0.04 m from the target |
| drop/hold_2m_strong | 1 | not run |  |  | OK | 17.45 | 0.04 m from the target |
| drop/hold_6m_moderate | 1 | not run |  |  | OK | 17.41 | 0.07 m from the target |
| drop/hold_6m_strong | 1 | not run |  |  | OK | 17.30 | 0.16 m from the target |
4 runs: 4 OK
```

The experiment rows were `sim/run_scenarios.sh drop/hold_6m_strong --seeds 1 --variants on` and the same for `drop/hold_2m_strong`, with the controller's defaults edited between flights. The hold numbers are `details.hold_error_p95_m` and its neighbours in each run's `score.json`.

## What to keep in mind

- One seed per scenario. A different seed's gusts could put a strong-wind hold over 0.30 m.
- The flights are of the working tree that was committed as CEN-02, except for one rule added afterwards that refuses a sequence asking for a gain over 0.6 with a lead under 0.5 s; it does not run in these flights, which use the defaults.
- The speed cap's 2 m margin on a barometer height was not exercised: the sim's rangefinder gave the height throughout.
- The sim's wind is a drag force on the vehicle's body; rotor effects, and what a real 7 m/s wind does to a real airframe's attitude loop, are not in it.

Run folders (not committed), under `sim/.state/suites/`: `G1-20261120T084609Z/` (the table); the experiments are `drop-hold_6m_strong-*` and `drop-hold_2m_strong-*` from 07:03Z to 08:11Z and `drop-hold_6m_moderate-20261120T065900Z/`.
