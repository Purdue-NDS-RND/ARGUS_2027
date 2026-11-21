# The green X and bucket detector (REC-1): synthetic renders and one sim flight

2026-11-20, on `c169330` plus the REC-1 change (uncommitted when run; it is the commit this file comes with).

## Synthetic renders: the spec's pass

```bash
python3 tools/green_x_try.py --n 300 --negatives 200 --seed 1
```

Positives: the X mat as a 1280 x 720 camera with a 778 px focal length sees it from 3 to 30 m, any rotation, up
to a quarter of the picture's height off centre, tilted up to 8 degrees each way, blurred, noisy, darker and
brighter, on a noisy green field; every fifth mat round, a bucket on every second. Negatives: bullseyes,
numbered markers of both kinds and bare green-noise fields.

| Line (the spec's) | Found | Met |
| --- | --- | --- |
| Recall at least 98%, counting an X only with its centre within max(2 px, 1% of its span) and its angle within 3 degrees | 99.7%: 299 of 300 (all 300 found; one outside a line) | yes |
| No hit on 200 bullseye, marker and green-noise renders | 0 | yes |

```
PASS: recall 99.7% (299 of 300 within the centre and angle lines; 300 found at all), 0 false alarm(s) on 200 bullseye, marker and green-noise renders; lines 98% and none
worst hit: centre at 61% of its limit, angle 2.17 deg (limit 3); seed 1
bucket: found on 103 of the 150 mats that had one, and on 0 of the 150 that had none
misses: #29 at 26.9 m: centre 0.7 px (limit 2.0), angle 3.0 deg
time a frame: median 12.48 ms, 95th percentile 21.74 ms (this machine, 1280 x 720)
```

- The one miss is found and just outside a line: its angle is 3.0 degrees out on a tilted mat from 27 m.
- The bucket is found on two mats in three (above about 15 m it is a few pixels across) and on no mat without one.
- Below about 3 m the X is not found (the mat fills the picture); that range is not in these renders or in the
  spec's line, and is the flow camera's in the spec.

## One sim flight

```bash
sim/run_scenarios.sh waypoint_navigation/survey_x_mat --seeds 1 --variants on
```

The survey (three tracks at 10.9 m, the survey vehicle's 1280 x 720 camera at 10 Hz, sim time, real-time factor
0.95) over a field with the recovery mat and a 1 kg bucket on it, the detector's node running beside the
executor. `sim/recovery_cv_check.py` puts the true centre of the X and of the bucket's rim through the camera
model at each frame's stamp and compares.

| | |
| --- | --- |
| Status | OK |
| Frames the node looked at | 1465; the slowest took 30 ms |
| Frames with the whole mat in the picture (by the truth) | 24 |
| The X reported in them | 24 (the line is 95%) |
| The X's centre against the truth, in those frames | median 4.3 px, 95th percentile 9.9 px, flying at a median 3.5 m/s and up to 7.0 m/s |
| Allowed: the spec's line (4.8 px at this height), 2 px, and one 33 ms truth period of flight | met: the 95th percentile is 5.1 px inside it |
| The bucket reported in them | 24, median 4.1 px from its rim's true centre |
| The X reported with the mat partly out of the picture | 17 frames (not judged) |
| Reports of an X when the mat was nowhere in view | none (the node's count of X frames, 41, is 24 and 17) |

- This flight proves the node on the sim's own pictures at speed: it finds the X and the bucket in every frame
  that has the mat in view, and nothing when it is not there.
- It does not prove the centre to the spec's 1%. Every frame with the mat in view was taken at 2 m/s or more.
  In all 24 the truth lay between two true poses, so the truth was not short of a pose; even so the centre is a
  median 4.3 px (6 cm on the ground, 17 ms of flight at 3.5 m/s) from it. Whether that is the truth's stamps
  against the camera's or a lag in the detections cannot be told from a pass at speed. A hover over the mat is
  what tells; no mission hovers there yet (REC-2's will).
- Five flights. The first found the X only with the mat half out of view: a runway's white in the picture hid
  the mat from the first way of looking for it. The second and third found it in every whole-mat frame and were
  failed by the check, which held a moving vehicle's frames to a still vehicle's line. The fourth passed (24 of
  25, median 3.6 px); this is the fifth, flown after the reviews' changes.
