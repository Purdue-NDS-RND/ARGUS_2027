# The target estimator against the truth in the sim (FND-06)

Measured on 18 Nov 2026 (UTC) on a club laptop, on `platform/build-2027` at `c353a4b` plus the FND-06 change. Sim image `argus-sim:humble`.

**Result: PASS on all three lines, three flights out of three.** With `use_target_estimator` on, the estimator's bullseye track was within 0.08 to 0.10 m of where the scenario put the target at the 95th percentile at 15 m (line 0.30 m), 0.02 to 0.03 m at 6 m (line 0.15 m) and 0.01 to 0.02 m at 2.4 m (line 0.08 m). Calm, one seed, with the 2026 colour detector as the only source. No mission reads the estimator yet, so each flight is the 2026 delivery on projection v2: they landed 0.05, 0.03 and 0.06 m from the target.

## The flights that count

```
$ sim/run_scenarios.sh delivery/estimator --seeds 1 --variants on        (three times)
| delivery/estimator | 1 | not run |  |  | OK | 16.19 | 0.05 m from the target |
| delivery/estimator | 1 | not run |  |  | OK | 16.21 | 0.03 m from the target |
| delivery/estimator | 1 | not run |  |  | OK | 16.18 | 0.06 m from the target |
```

Variant `on`: `sim_time: true`, the detector's `features.stamp_from_image` and `features.detection_contract`, the mission flags `use_frame_time_pose` and `use_target_estimator`. The bullseye is 1.5 m across (see "The first flight"). The three flights are the code at three moments: flight 1 before the reviewers' fixes, flight 2 after the first round of them, flight 3 the code as committed. From each run's `report.md` and `estimator_check.json` (`sim/estimator_check.py` listens to `/estimator/targets` for the whole flight and takes the oldest CONFIRMED bullseye track each time the tracks are published, at 10 Hz):

| | Flight 1 | Flight 2 | Flight 3 (as committed) |
| --- | --- | --- | --- |
| 13.5 to 16.5 m: samples with a CONFIRMED track | 121 | 121 | 122 |
| error: median; 95th percentile; maximum (line 0.30 m) | 0.033 m; 0.083 m; 0.101 m | 0.036 m; 0.096 m; 0.119 m | 0.037 m; 0.102 m; 0.182 m |
| 5 to 7 m: samples | 125 | 125 | 125 |
| error: median; 95th percentile; maximum (line 0.15 m) | 0.010 m; 0.020 m; 0.028 m | 0.017 m; 0.026 m; 0.029 m | 0.011 m; 0.021 m; 0.023 m |
| 2.0 to 2.8 m: samples | 45 | 45 | 46 |
| error: median; 95th percentile; maximum (line 0.08 m) | 0.008 m; 0.017 m; 0.018 m | 0.012 m; 0.015 m; 0.015 m | 0.005 m; 0.013 m; 0.013 m |
| The filter's own sigma, median, at 15; 6; 2.4 m | 0.056; 0.038; 0.030 m | 0.056; 0.037; 0.028 m | 0.057; 0.038; 0.031 m |
| Errors within 3 sigma, all three bands | 100% | 100% | 100% |
| Raw detections projected at 15 m (FND-05's check): median; 95th percentile | 0.09 m; 0.20 m | 0.10 m; 0.24 m | 0.10 m; 0.24 m |
| Pose stamps against the sim clock (95th percentile) | 4.4 ms | 4.4 ms | 4.5 ms |

- One track for the whole flight each time (id 1), and no sample in any band was COASTING: the report counts the samples of each band by state, and the only states are CONFIRMED, one TENTATIVE at 15 m (the first sighting), LOST at 15 m and "no track".
- The samples without a CONFIRMED track are the climb after take-off (no track yet) and the return and landing (the track LOST at 15 m once the mission has switched the detector off, then dropped): in flight 3, at 15 m 34 with no track, 1 TENTATIVE and 157 LOST; at 6 m 48 with no track; at 2.4 m 20 with no track.
- The filter takes the raw projections' 0.20 to 0.24 m at the 95th percentile down to 0.08 to 0.10 m at 15 m.
- Time check, rate check and projection check all PASS in all three.

## What a step costs

The estimator shares the executor's one thread with the 20 Hz tick. On this laptop:

```
$ python3 tools/estimator_step_cost.py
one bullseye in one frame: median 0.22 ms, p95 0.23 ms, max 0.34 ms over 200 steps, 1 tracks (the tick is 50 ms)
the worst the bounds allow (4 frames of 16 detections): median 12.18 ms, p95 12.48 ms, max 13.70 ms over 200 steps, 16 tracks (the tick is 50 ms)
```

Not measured on the Jetson; the same command runs there (under "Needs a person").

## The first flight, and what changed after it

```
$ sim/run_scenarios.sh delivery/estimator --seeds 1 --variants on      (5 m bullseye, before the cut-box rule)
| delivery/estimator | 1 | not run |  |  | FAIL | 16.19 | 0.05 m from the target |
```

| Height band | Samples with a CONFIRMED track | Error: median; 95th percentile; maximum | Errors within 3 sigma |
| --- | --- | --- | --- |
| 13.5 to 16.5 m | 123 | 0.033 m; 0.627 m; 1.691 m | 79% |
| 5 to 7 m | 125 | 0.014 m; 0.119 m; 0.134 m | 93% |
| 2.0 to 2.8 m | 4 (of 64) | 0.008 m; 0.011 m; 0.011 m | 100% |

The track was confirmed on the bullseye while it was still coming into view at the edge of the image. The 2026 detector reports the centre of the red it can see, which is up to 1.7 m from the centre of a 5 m target that is half out of view, the same way in every frame, so the filter averaged a biased measurement and was sure of it. Two changes followed (both under Decisions in `docs/build/PROGRESS.md`):

1. A detection whose box reaches the edge of the image is not given to the filter. It is counted.
2. `delivery/estimator` uses a 1.5 m bullseye. A 5 m one stops fitting in the 640 x 360 image below about 6.5 m (the camera sees 2.2 m of ground at 2.4 m), and with rule 1 a box detector then gives the estimator nothing: the 2.4 m line could not be measured at all. Below the height where a target stops fitting, the centre is the ring fit's job (CEN-13).

## What this does not show

- Calm only (SIM-03 brings wind), one seed, and a hover or slow descent over the target. The survey missions will show the estimator on a moving vehicle.
- The detector is the 2026 colour detector on a clean sim image. A trained detector's boxes will be noisier.
- The position-reset handling ran only in unit tests; SIM-04's GPS glitches will exercise it in a flight.
- The sim camera has no lens distortion and the airframe's mount is the sim's, not the aircraft's.

## Commands

```
sim/run_scenarios.sh delivery/estimator --seeds 1 --variants on
python3 tools/estimator_step_cost.py
python3 -c "import json; r=json.load(open('<run folder>/estimator_check.json')); print(r['at_15_m'], r['at_6_m'], r['at_2_4_m'])"
```

Run folders (not committed): `sim/.state/suites/delivery-estimator-20261118T041024Z/`, `-20261118T042240Z/`, `-20261118T042928Z/` (flights 1 to 3), `-20261118T040423Z/` (the first flight, on the 5 m bullseye).
