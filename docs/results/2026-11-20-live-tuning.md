# The 2026 live tuning flown in the sim

Measured on 20 Nov 2026 (UTC) on a club laptop, on `platform/build-2027` at `60056ef` plus the CEN-11 change. Sim image `argus-sim:humble`. One flight of each scenario, seed 1, calm, every flag off, `fake_drop: true`, 5 m bullseye 18 m from the takeoff point.

**Result.** Both sim copies of the live files fly to the end and score. With the aircraft's tuning the delivery touches down 0.43 m from the centre, where the sim demo's tuning (gate G0) puts it at 0.02 to 0.08 m.

| Scenario | Sequence | Result | Distance from the centre | Release height | Arm to disarm |
| --- | --- | --- | --- | --- | --- |
| `delivery/live_tuning` | `sim_package_delivery_live.yaml` | OK, 15.71 points | 0.43 m | on the ground | 187 s |
| `drop/live_tuning` | `sim_package_drop_live.yaml` | OK, 17.16 points | 0.27 m | 6.00 m (lowest height flown 5.97 m) | 97 s |

For comparison, the demo tuning on the same target: delivery 0.02 to 0.08 m over five seeds (`2026-11-18-gate-G0-W2.md`).

## What the live tuning does differently

- **Delivery.** The live file centres to 0.8 m with a 0.5 m deadband and a gain of 0.15 (the demo: 0.20 m, 0.15 m, 0.25), and hands over to the fixed GPS column at 8 m with a 3 m tolerance, which in this flight was at 10.95 m (the demo: 6 m, 0.2 m). An error inside the deadband is treated as zero, so the column is frozen wherever inside about half a metre the vehicle happens to be.
- **Drop.** The live file's drop height is 6 m with a 3 m tolerance, so the mission counts itself "at drop altitude" from 9 m: it went from `TRACK_AND_DESCEND` straight to `DROP_PAYLOAD` without the fixed column, then came down to 6 m during the 8 s hover and released there.

In that hover nothing is checked before the release: the 2026 `DROP_PAYLOAD` state releases when its 8 s are up whatever the height or the detection's age, and since the column was never frozen it holds wherever the GPS says the vehicle is on each tick, not a fixed point. That is the live path as it flew, unchanged here; it is what CEN-04's release gates replace, and worth remembering when the A/B runs (CEN-12) use this flight as the reference.

## How it was measured

```
$ sim/run_scenarios.sh delivery/live_tuning --seeds 1 --variants off
| delivery/live_tuning | 1 | OK | 15.71 | 0.43 m from the target | not run |  |  |
$ sim/run_scenarios.sh drop/live_tuning --seeds 1 --variants off
| drop/live_tuning | 1 | OK | 17.16 | 0.27 m from the target | not run |  |  |
```

## What to keep in mind

- One flight each, one seed, calm air, no payload hanging (a fake release): the distance is the vehicle's at the release, not where a package landed.
- The sim's vehicle, camera and target are not the aircraft's; this says what the live numbers do in the sim, as a reference for the 2027 A/B runs (CEN-12), not what they did at the competition.
- The files flown on the competition day, `final_curcuit_delivery.yaml` and `final_waypoint_drop.yaml`, have a waypoint leg first and are not copied. Their drop settings equal `package_drop_live.yaml`'s; their delivery differs from `package_delivery_live.yaml` in two values (`final_descent_rate_mps` 0.25 against 0.14, `touchdown_min_altitude_m` -2.0 against -5.0).

Run folders (not committed): `sim/.state/suites/delivery-live_tuning-20261120T044837Z/`, `sim/.state/suites/drop-live_tuning-20261120T045239Z/`.
