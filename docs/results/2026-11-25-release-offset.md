# Release offset model (CEN-08): drops in the sim's strong wind, 2026-11-25

## Commands

```
sim/run_scenarios.sh drop/release_offset_wind --seeds 1,2,3 --variants on
python3 tools/release_offset_fit.py --runs sim/.state/suites/drop-release_offset_wind-20261125T142032Z/runs/*
sim/run_scenarios.sh drop/release_offset_wind_fitted --seeds 1,2,3 --variants on
python3 tools/release_offset_fit.py --runs sim/.state/suites/drop-release_offset_wind_fitted-20261125T144639Z/runs/*
```

Both scenarios are `drop/gated_wind` (the gated drop of a real 0.2 kg parcel from 2.4 m in the sim's strong wind, 7 m/s from the south-west with gusts) with `release_offset_model` on: the first with K at 0, where nothing is shifted and the lean is logged; the second with the K the first gave.

## The fit, from three drops

```
| Drop | Height (m) | Lean (east, north) | Drift (east, north) m | Drift (m) |
| drop-release_offset_wind-s1-on | 2.42 | (-0.049, -0.055) | (+0.05, +0.12) | 0.13 |
| drop-release_offset_wind-s2-on | 2.42 | (-0.088, -0.079) | (+0.32, +0.43) | 0.54 |
| drop-release_offset_wind-s3-on | 2.39 | (-0.075, -0.111) | (+0.34, +0.43) | 0.55 |
K = 1.61 from 3 drop(s) (the largest lean 0.134). The drifts' RMS is 0.45 m; with this K the model leaves 0.12 m.
```

## The same three seeds with that K

| Seed | K 0: from the target | K 1.61: from the target | Aim shifted (east, north) m | Drift with K 1.61 (m) |
| --- | --- | --- | --- | --- |
| 1 | 0.08 m | 1.09 m | (-0.200, -0.191) | 1.30 |
| 2 | 0.45 m | 0.44 m | (-0.343, -0.321) | 0.16 |
| 3 | 0.40 m | 1.46 m | (-0.275, -0.417) | 2.11 |

**With the aim point shifted the drops were worse: 1.00 m from the target on average against 0.31 m.** This was the kindest test there is (the same seeds the K was fitted on), and it failed it. On seed 3 the release was the gates' last resort: with the aim point moving with the lean, the gates did not hold within their 25 s.

The fit's drift is to where the parcel came to rest, so it includes what the parcel did on the ground. The six drops' drifts, from where the parcel was let go to where it came to rest, were 0.13, 0.54, 0.55, 1.30, 0.16 and 2.11 m at leans that hardly differ (0.07 to 0.14), all from 2.4 m. Fitted on the second three the K would be 4.17 and leave 0.83 m of 1.43 m. Split at the parcel's first touch of the ground (its lowest point in the trace), the fall itself moved it 0.13, 0.16, 0.54, 0.61, 0.16 and 1.07 m, and in three of the six it then went on along the ground for 0.4 to 1.1 m more. So in the sim the parcel's drift is not a steady multiple of the vehicle's lean: it is the gust of the moment on a 0.2 kg, 10 cm box, and what the box does after it lands. The first three drops fitting to 0.12 m was three points agreeing by chance.

A seventh drop, flown on the code as committed (`sim/run_scenarios.sh drop/release_offset_wind --seeds 1 --variants on`, K 0): 0.82 m from the target, where the same seed and settings gave 0.08 m the first time. The release line was logged on the tick the release was commanded (height 2.54 m, lean (-0.0235, -0.0536), nothing shifted). The sim's drops in this wind do not repeat from one flight to the next, which is the finding above said another way: three drops are far too few to fit or to judge anything. (The runner marked that flight FAIL on the height estimator's own check, 0.10 m against its 0.1 m limit; the release gates had needed a second try.)

## What it means

The model is not supported by the sim, and no K is carried into any sequence: the flag stays off and `release_offset_k` is 0 where it appears. Whether the aircraft's package behaves like the sim's parcel is a field question: the sim's parcel is light for its size (the gate's note: with 0.5 kg the sim's Iris cannot hold its height in this wind), which makes the wind's push on it large and the gusts' share of it larger. Field drops with the flag on and K 0 (the lean and height are logged at each release) are what would say, with at least ten drops and a check on drops the fit did not see.

The flights were flown before the reviews' changes to the mission's wiring (the model now needs the gated release, reads only a height above the ground of the moment, and logs on the tick the release is commanded). Those change nothing these flights did: all six were gated releases with the rangefinder's height.

## Not shown

Any release height but 2.4 m (the height's part in the model is untested); any wind but the sim's strong; a held-out drop; the aircraft.
