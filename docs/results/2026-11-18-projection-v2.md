# Projection v2 against the truth in the sim (FND-05)

Measured on 18 Nov 2026 on a club laptop, on `platform/build-2027` at `e7da77e` plus the FND-05 change. Sim image `argus-sim:humble`.

**Result.** With `use_frame_time_pose` on, the 2026 delivery flew on projection v2 three times and landed 0.06, 0.04 and 0.05 m from the target. `sim/projection_check.py`, running beside each flight with the same projection code, put the bullseye within 0.22, 0.22 and 0.25 m of where the scenario placed it at the 95th percentile of the detections taken near 15 m (pass line 0.3 m; medians 0.07, 0.06 and 0.09 m). The third flight is on the code as committed.

## The flights

```
$ sim/run_scenarios.sh delivery/frame_time --variants on        (three times)
| delivery/frame_time | 1 | not run |  |  | OK | 16.18 | 0.06 m from the target |
| delivery/frame_time | 1 | not run |  |  | OK | 16.20 | 0.04 m from the target |
| delivery/frame_time | 1 | not run |  |  | OK | 16.18 | 0.05 m from the target |
```

Variant `on`: `sim_time: true`, the detector's `features.stamp_from_image`, and the mission flag `use_frame_time_pose` (camera mode `sim_delivery`, airframe `sim_iris_downcam`, from `package_delivery_demo.yaml`). From each run's `report.md` and `projection_check.json`:

| | Flight 1 | Flight 2 | Flight 3 |
| --- | --- | --- | --- |
| Distance from the target at touchdown | 0.06 m | 0.04 m | 0.05 m |
| Detections of the target heard; projected; refused | 1844; 1844; 0 | 1849; 1849; 0 | 1847; 1847; 0 |
| Projected bullseye against the truth, 13.5 to 16.5 m: detections | 300 | 291 | 287 |
| median; 95th percentile; maximum | 0.07 m; 0.22 m; 2.25 m | 0.06 m; 0.22 m; 2.37 m | 0.09 m; 0.25 m; 2.37 m |
| The same pixels with the newest pose at arrival (what 2026 did): median; 95th percentile | 0.07 m; 0.21 m | 0.06 m; 0.26 m | 0.11 m; 0.24 m |
| All heights: median; 95th percentile | 0.05 m; 0.13 m | 0.03 m; 0.12 m | 0.05 m; 0.14 m |
| Below 6 m: median; 95th percentile; maximum | 0.06 m; 0.12 m; 0.15 m | 0.03 m; 0.11 m; 0.15 m | 0.06 m; 0.12 m; 0.17 m |
| Ground height from the rangefinder; from home's height (all heights) | 1618; 226 | 1639; 210 | 1622; 225 |
| "no ground projection" lines in the mission's log | 0 | 0 | 0 |
| Pose stamps against the sim clock (95th percentile) | 4.6 ms | 4.5 ms | 4.4 ms |
| Real-time factor while armed | 0.95 | 0.95 | 0.96 |

Flight 2 was flown after the checker learned to list its five worst detections at 15 m; the projection and mission code were the same as in flight 1. Flight 3 is on the committed code, after the review fixes: a frame over 0.5 s old gives no tracking solution, and the rangefinder's ground is the median of several readings with plausibility limits. Neither changed the picture, and neither refused anything in this flight.

How to read it:
- The truth is the scenario's target relative to the take-off point; the projection is in MAVROS's local frame, whose origin is the take-off point. So the figure contains the autopilot's own position error as well as the projection's.
- **The maximum (2.3 m) is the detector, not the projection.** The five worst detections at 15 m in flight 2 are five frames within 0.3 s of each other, at the very top edge of the image (`y_norm` -0.99, -0.96, -0.88, -0.86, -0.83; `x_norm` about 0), with the error falling from 2.37 m to 0.97 m as they go (flight 3 the same: `y_norm` -0.97 to -0.87, 2.37 m falling to 1.40 m). That is the 5 m bullseye coming into view from the top of the frame as the vehicle arrives: the detector reports the centre of the part it can see. Nothing in the projection can know that, and it is what pulls the 95th percentile up to 0.22 m; FND-06's gate and CEN-13's centre finder are where it is dealt with.
- **The frame-time pose made little difference in these flights** (0.22, 0.22 and 0.25 m against 0.21, 0.26 and 0.24 m with the latest pose). The 2026 delivery arrives over the target and hovers; it is not moving or tilting much while it looks. The unit tests show the difference on a moving vehicle (over 0.2 m at 2 m/s with a frame 150 ms old); a sim flight that shows it needs a mission that looks while it moves (the survey, LOC).
- **What the sim cannot show:** the sim camera has no distortion and sits at the flight controller, so undoing the lens and the lever arm rest on the unit tests (nadir pixel, round trip under a micrometre with the real IMX519 numbers, Jacobian against a numerical derivative).
- At 15 m every detection used the rangefinder's ground (291 of 291 and 287 of 287 in flights 2 and 3). The 210 to 226 that used home's height were at other heights; which heights, and why the rangefinder was not usable there, was not looked into.
- MAVROS publishes the rangefinder with `min_range: 0.0` and `max_range: 1000.0` whatever the sensor (one message recorded beside flight 3, `ros2 topic echo --once /mavros/rangefinder/rangefinder`, on the ground: `range: 0.0`). So "at the sensor's limits" only catches a reading of zero; a blocked beam or no return is caught by the plausibility limits instead (the ground more than 5 m from home's height, or readings that disagree).

## Commands

```
sim/run_scenarios.sh delivery/frame_time --variants on
python3 -m pytest -q src/drone_mission_core/test/test_projection_v2.py        (nadir, round trip, Jacobian)
```

Run folders (not committed): `sim/.state/suites/delivery-frame_time-20261118T021842Z/`, `-20261118T022551Z/` and `-20261118T023441Z/`.
