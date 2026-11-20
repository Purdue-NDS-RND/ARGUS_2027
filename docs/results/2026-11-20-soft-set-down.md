# The delivery's soft set-down: on the estimator to the ground, release on contact

Measured on 20 Nov 2026 (UTC) on a club laptop, on `platform/build-2027` at `2941e9e` plus the CEN-05 change. Sim image `argus-sim:humble`. Calm air, one seed.

**Result.** With `delivery_soft_set_down` on, the delivery came down on the estimator from 6 m, held its position below 0.6 m, touched at 0.12 m/s, took contact 0.8 s after the legs touched, released and was off the ground again 2.6 s after touching. The package came to rest 0.010 m from the target's centre. The 2026 delivery with the same package touched at 0.25 m/s, released 2.6 s after touching, stood 7.5 s on the ground and left the package 0.165 m from the centre.

## Commands

```
sim/run_scenarios.sh delivery/set_down --seeds 1 --variants off,on     # first version of the change
sim/run_scenarios.sh delivery/set_down --seeds 1 --variants on         # the code as committed
sim/run_scenarios.sh delivery/set_down_floor --seeds 1 --variants on   # the code as committed
```

```
| Scenario | Seed | 2026 (flags off) | Points | Result | 2027 (flags on) | Points | Result |
| delivery/set_down | 1 | OK | 16.36 | 0.17 m from the target | OK | 16.54 | 0.02 m from the target |
| delivery/set_down | 1 | not run |  |  | OK | 16.55 | 0.01 m from the target |
| delivery/set_down_floor | 1 | not run |  |  | OK | 16.52 | 0.03 m from the target |
```

**Which code each flight flew.** The first pair was flown on the first version of the change; its flags-off flight is the 2026 column below (the change does not touch that path). The two reviews then changed when the hold is taken and added limits (see "What the reviews changed"), and the two flags-on flights that follow are of the code as committed. The first version's flags-on flight gave 0.018 m, 0.75 s from touch to release and 2.5 s on the ground.

The scenario: a real 0.5 kg package on the vehicle's release joint (`fake_drop: false`), a 5 m bullseye, sequence `package_delivery_demo.yaml`. Flags on: `use_frame_time_pose`, `use_target_estimator`, `use_height_estimator`, `centre_on_estimator`, `centring_retune`, `use_ring_fit`, `delivery_soft_set_down`; node flags `target_cv.stamp_from_image`, `target_cv.detection_contract`, `ring_fit.ring_fit`; airframe `sim_iris_parcel`; `package_height_m: 0.05`. Variant off is on wall time (real-time factor 0.98), variant on is on sim time.

## The two flights

Times are sim time, from the true poses in each run's `trace.jsonl`.

| | 2026 (flags off) | 2027 (flags on) |
| --- | --- | --- |
| Package at rest, from the target's centre | 0.165 m | 0.010 m |
| From 6 m to the legs touching | 23.0 s | 22.7 s |
| Descent speed over the last 0.5 to 0.1 m | 0.25 m/s | 0.12 m/s |
| Legs touching to the release command | 2.6 s | 0.79 s |
| On the ground in all | 7.5 s | 2.56 s |
| Release height (true, the judge's) | 0.025 m | 0.025 m |
| Hardest contact of the package | 0.316 N s | 0.292 N s |

- The 0.165 m of the 2026 flight is mostly where the sim hangs the package (0.13 m to the left of the vehicle's centre): 2026 centres the vehicle. The 2027 flight is told where the package hangs (`sim_iris_parcel`) and centres the package.
- **The package's contact in the sim is not the set-down.** The sim's package hangs clear of the ground while the vehicle stands, so its only contact is its fall from the hook after the release (0.26 to 0.32 N s in all four flights, whichever the variant). The number that the set-down changes is the speed at which the vehicle, and on the aircraft the package under it, meets the ground: 0.12 m/s against 0.25. The impulse itself is OPS-07's rig.

### The 2027 flight's own lines

```
[package_delivery] Entering final set-down at 6.19 m (threshold 6.00 m)
[package_delivery] State: TRACK_AND_DESCEND -> FINAL_SET_DOWN
[package_delivery] final set-down: 5.99 m on the rangefinder, error 0.02 m
[package_delivery] final set-down: 1.13 m on the rangefinder, error 0.03 m
[package_delivery] final set-down: 0.68 m on the rangefinder, error 0.03 m
[package_delivery] final set-down: at 0.60 m on the rangefinder, error 0.02 m: holding the position and descending at 0.12 m/s
[package_delivery] set-down: waiting on range, speed, throttle
[package_delivery] contact: rangefinder 0.01 m (package 0.05 m), vertical speed +0.00 m/s, throttle 0.28 against 0.37 while descending, together for 0.3 s
[package_delivery] State: FINAL_SET_DOWN -> GROUND_DWELL
[package_delivery] Releasing gripper payload
[package_delivery] State: GROUND_DWELL -> GUIDED_RELAUNCH
[package_delivery] State: GUIDED_RELAUNCH -> COMPLETE
```

- The track was held all the way to 0.6 m: no pause on a COASTING track, no loss. Ring fit: looked at 1376 frames below 6 m, rings found in 338.
- The throttle was 0.37 while descending and 0.28 when contact was declared, a quarter down; the rule asks for 15%.
- Checks beside the flight (run OK): message rates held, `/mavros/vfr_hud` among them at the 20 Hz asked; the estimator and the height estimator inside their lines.

### The 2026 flight's own lines

```
[package_delivery] Entering final touchdown column at 6.20 m (threshold 6.00 m, handoff tolerance 0.20 m)
[package_delivery] Freezing touchdown column at (34.1751354, -118.4823915)
[package_delivery] Ground contact detected, debouncing for 0.5 s
[package_delivery] Touchdown confirmed by FCU landed state
[package_delivery] Holding on target for 5.0 s before relaunch
[package_delivery] Releasing gripper payload
[package_delivery] State: GROUND_DWELL -> GUIDED_RELAUNCH
```

## `delivery/set_down_floor`: the hold where the mission stops taking a target

The mission takes no target under `min_projection_altitude_m` on the flight controller's height (0.75 m in the sequences); this scenario sets it to 1.2 m, which leaves a band down to 0.6 m with nothing to steer on.

```
[package_delivery] Entering final set-down at 6.20 m (threshold 6.00 m)
[package_delivery] final set-down: at 1.05 m on the rangefinder (under the height the mission steers down to), error 0.04 m: holding the position and descending at 0.12 m/s
[package_delivery] contact: rangefinder 0.01 m (package 0.05 m), vertical speed +0.00 m/s, throttle 0.28 against 0.37 while descending, together for 0.3 s
[package_delivery] Releasing gripper payload
[package_delivery] State: GROUND_DWELL -> GUIDED_RELAUNCH
```

The hold was taken at 1.05 m above the ground (1.25 m on the flight controller's height), centred, and the set-down went on from there: package 0.033 m from the centre, 0.70 s from touch to release, 2.39 s on the ground, 0.12 m/s over the last half metre. No lost target, no recovery climb. One pause on a COASTING track a second into TRACK_AND_DESCEND, at 15 m.

## What the reviews changed

- The first version took the hold on height alone (0.6 m above the ground), before the track was looked at. In the first flight the hold came with the flight controller's height at about 0.80 m, 0.05 m above the 0.75 m under which the mission takes no target: on ground where that height does not read 0.19 m high, as the sim's does, the vehicle would have lost its target a little under a metre up and climbed away. Now the hold is taken only while centred on a track that may be descended on, and also within 0.05 m of that floor.
- Added: the flight controller's "landed" as contact before the hold; the contact rule only when the hold was taken on the rangefinder's height; signals no older than 0.2 s; the 1.82 m ceiling at the release; 20 s and 10 s limits; the handoff from under the 2026 window; `package_height_m` checked when the sequence is built. Each has a mission test; none changes what the calm flights above do, and none was flown. One was made after the last flight: at the release a height that is not the rangefinder's counts 0.5 m higher against the 1.82 m ceiling (in these flights the height at the release was the barometer's 0.2 m, under the ceiling either way).

## Where the contact rule's numbers came from

A recorded 2026 sim delivery (`sim/run_scenarios.sh delivery/baseline --record`, flown 18 Nov, read with `sim/replay/bag.py`): around the touchdown the rangefinder went from 0.07 m to 0.01 m and stayed, the vertical speed from -0.25 m/s to 0.000, and the mean of the four motor outputs from 1555 µs to 1466 µs within 0.25 s and to idle (1150 µs) within 1.5 s. The flight controller's landed state changed 2.75 s after that.

## Not measured here

- Wind, and more than one seed.
- The aircraft: its rangefinder reading with the package on the ground (`package_height_m`), its throttle's behaviour on contact, and the contact impulse (OPS-07). All three are under "Needs a person".
