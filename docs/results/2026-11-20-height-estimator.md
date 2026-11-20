# The height estimator against the true height in the sim

Measured on 20 Nov 2026 (UTC) on a club laptop, on `platform/build-2027` at `2d36a75` plus the CEN-03 change. Sim image `argus-sim:humble`. Scenario `delivery/height_estimator`, seed 1, calm, flags `use_frame_time_pose` and `use_height_estimator` on, sim time. Four flights, because the estimator changed between them; the fourth is on the code as committed.

**Result.** PASS. From 11 m down every height came from the rangefinder and was within 0.04 m of the true height at the 95th percentile (0.10 m allowed). Above the sensor's 12 m range, and in the 22 samples where the vehicle was tilted over 17°, every height came from the barometer; those were 0.19 m from the truth. That is the flight controller's own height error in this sim, the kind of error the rangefinder is there to remove.

| Line | Limit | Flight 4 (the committed code) |
| --- | --- | --- |
| Heights from the rangefinder against the true height, p95 | 0.10 m | 0.04 m (median 0.01 m, 2822 heights) |
| Between 0.5 and 11 m, tilted 10° or less: share from the rangefinder | at least 90% | 100% of 2583 heights |
| Heights from the rangefinder above 12.5 m | none | 0 of 1066 |
| Heights from the rangefinder while the true tilt is over 17° | none | 0 of 22 |
| Heights from the barometer against the true height (not judged) | | median 0.19 m, p95 0.19 m (1905 heights: 1856 out of range, 37 tilted, 11 with too few readings yet, 1 before the first reading) |

The delivery itself is unchanged by the flags (no mission reads the height yet): OK, 0.03 m from the target.

## The four flights

| Flight | Estimator | Height check | Run |
| --- | --- | --- | --- |
| 1 | As first written | FAIL: 1 of 1068 heights above 12.5 m came from the rangefinder | FAIL |
| 2 | Barometer at once after three readings in a row out of range | PASS: p95 0.02 m, 100% in the band, 0 of 1066 above 12.5 m | OK, 0.05 m |
| 3 | Also: barometer when tilted over 15° now, and on a stuck reading | PASS: p95 0.01 m, 100%, 0 of 1062, 0 of 22 tilted | FAIL on another check (below), 0.07 m |
| 4 | The stuck-reading test made when a height is asked for | PASS (the table above) | OK, 0.03 m |

Flight 1: climbing through 12 m after take-off, the estimator went on answering from readings up to half a second old, which were still right about where the ground is but no longer "in range". Flights 3 and 4 follow the two reviews (tilt at the moment of the answer; a sensor repeating its last value).

**Flight 3's failure was projection v2's line, not the height's.** Every flight with `use_frame_time_pose` on is also held to FND-05's line: the projected bullseye within 0.30 m of the truth at the 95th percentile between 13.5 and 16.5 m. In these four flights that number was 0.20, 0.28, 0.33 and 0.28 m; on 18 Nov it was 0.19 to 0.25 m over eight flights. The tail is the same in every flight: about ten detections in the first third of a second after arrival, with the 5 m bullseye cut by the top edge of the image (errors of 1.4 to 2.5 m, `y_norm` under -0.88), which the detector reports as if they were whole. With 250 to 280 detections in the band instead of 290 to 300, those ten reach the 95th percentile. Fewer detections were heard with one more tool, and in flights 2 and 3 the recorder, running beside the flight. The projection code's results are unchanged by this feature (the ground it uses is computed by the same lines as before). It is recorded under Decisions for the lead.

## The same flight through the field tool

Flights 2 and 3 were recorded, and `tools/height_log.py`, which is what a person runs on a recording from the aircraft, gives the same picture from the bag alone (flight 3):

```
$ python3 tools/height_log.py sim/.state/suites/delivery-height_estimator-20261120T052612Z/runs/delivery-height_estimator-s1-on/bag
4720 heights from 3215 rangefinder readings
  0 to 3 m: 48% from the rangefinder (1438 heights)
  3 to 6 m: 100% from the rangefinder (749 heights)
  6 to 9 m: 100% from the rangefinder (859 heights)
  9 to 12 m: 100% from the rangefinder (527 heights)
  12 m and over: 0% from the rangefinder (1147 heights)
  barometer used because: no reading 2, out of range 1848, tilt 37, too few readings 10
rangefinder height minus height above home: median -0.19 m, 95% within 0.01 m of that (2823 heights)
```

The 48% under 3 m is the time on the ground before take-off and during the delivery's dwell: the sim's rangefinder reads 0 there, under the sensor's 0.1 m minimum.

## How it was measured

```
$ sim/run_scenarios.sh delivery/height_estimator --seeds 1 --variants on          # flight 4; flights 2 and 3 with --record
| delivery/height_estimator | 1 | not run |  |  | OK | 16.21 | 0.03 m from the target |
height estimator against the true height: rangefinder source: PASS, median 0.01 m, p95 0.04 m over 2822 heights (limit 0.1 m)
height estimator against the true height: between 0.5 and 11 m: 100% of 2583 heights from the rangefinder
height estimator against the true height: baro source: median 0.19 m, p95 0.19 m from the truth over 1905 heights (no reading 1, out of range 1856, tilt 37, too few readings 11)
height estimator against the true height: above 12.5 m: 1066 heights, 0 from the rangefinder
height estimator against the true height: tilted over 17 degrees: 22 heights, 0 from the rangefinder
height check: PASS
projected target against the truth at 15 m: median 0.10 m, p95 0.28 m over 271 detections
```

`sim/height_check.py` runs beside the flight. It feeds the same estimator module the executor uses from the same two topics (MAVROS's pose and rangefinder) and asks for the height at every pose, with that pose's attitude; the true height and tilt are the vehicle's in Gazebo.

## What to keep in mind

- One seed, calm, flat ground. The sim's rangefinder is SITL's analog one: it has no noise to speak of, no dropouts, never sticks, and reads far beyond 12 m, so the 12 m limit here is the estimator's own, and the stuck-reading rule is proven by unit tests only.
- The check runs a second copy of the estimator beside the executor's, on the same topics. The executor's own copy is covered by unit tests through `context.height_agl()`; no mission reads it in a flight until CEN-04.
- The 0.19 m is this sim's (the vehicle's origin is 0.195 m above the ground where it stands). On the aircraft the difference between the two sources is unknown until the taped hover is flown (Needs a person, CEN-03).

Run folders (not committed): `sim/.state/suites/delivery-height_estimator-20261120T051012Z/`, `...T051613Z/`, `...T052612Z/`, `...T053103Z/` (flights 1 to 4).
