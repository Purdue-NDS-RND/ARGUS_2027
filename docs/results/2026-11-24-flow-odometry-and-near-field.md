# Flow odometry and the detector's near field in the sim (REC-5), 2026-11-24

Two things were flown: the optical-flow odometry against the sim's true velocity, and the recovery's pickup on the real detector with its new near field (the lead's note 4).

## The pickup on the detector (the lead's note 4)

```
sim/run_scenarios.sh recovery/pickup_detector --variants on
```

The recovery mission on `recovery_cv` (the green X and bucket detector, not the oracle), seed 1, calm, sim time.

| | Before (REC-1 as built) | Now (with the near field) |
| --- | --- | --- |
| The run | the detections stopped at about 3 m, the cone held, three descents, the mission failed to RTL | `OK`, 19.99 points |
| States | ... `CONE_DESCENT`, `LOST_TARGET`, `ABORT_CLIMB` three times | `CENTRE -> CONE_DESCENT -> ENGAGE -> LIFT -> VERIFY_ATTACH -> CLIMB -> RETURN -> SET_DOWN -> CLEAR -> LANDING -> COMPLETE`, one descent, no `LOST_TARGET` |
| The bucket | not taken | lifted (4.4 m), set down 0.008 m from the return point, released on the ground |

The detector's own check on that flight (`sim/recovery_cv_check.py`, against Gazebo's truth):

- With the whole mat in view (5.4 to 15 m): the X reported in 266 of 266 frames, centre error median 1.2 px and 95th percentile 3.1 px; as before the near field.
- The near field (under 3 m, the bucket's rim in the picture): the bucket reported in 416 of 1051 frames, down to 0.84 m; centre error median 0.9 px and 95th percentile 2.6 px, about 4 mm and 11 mm on the ground. The X was still fitted in 47 of them.
- The 635 near frames with no bucket reported are not a gap in the descent: the count runs over the whole flight under 3 m, and after the pickup the bucket hangs under the camera and is later set down on bare ground at the takeoff point. There it is not on the mat, and the near search, which wants mat all round the blob, rightly says nothing. During the descent the bucket's track stayed CONFIRMED, which is the mission's own rule for going down: the cone never held.

These are the third flight's numbers. The second flew the near search as first written, held to nothing but colour and shape (501 of 1031 frames, 0.8 px median, the same pickup); the flight-safety review asked for it to be tied to the X, and the third flew it tied: on the arms' crossing and a bucket's size against the arm's width. The first had a diagnostic node beside it, gave the same mission result and was marked INVALID by the time check (the diagnostic node was on wall time); it is not counted.

The detector's full synthetic run after the change (`python3 tools/green_x_try.py --n 300 --negatives 200 --seed 1`, a bucket with no X now counted as a false alarm too): no false alarm on the 200 bullseye, marker and green-noise renders, and no bucket on the 150 mats that had none.

The descend and release rules were not touched. No rule had to be added for the hand-off: the estimator's bucket track goes on being fed from the same detector on the same topic.

## The flow odometry against the true velocity

```
sim/run_scenarios.sh recovery/flow_odometry --variants on
```

The recovery of `recovery/pickup_oracle` with the node flag `flow_odometry` on: the recovery vehicle's flow camera in the sim (320 x 200, mono, 20 Hz, the hook hanging in the middle of the picture), the rangefinder, MAVROS's pose. `sim/flow_check.py` compares every sample with the true mean velocity over the same two frames. The spec's line: RMS under 0.15 m/s at 5 to 15 m.

Each flight changed one thing, because the flight before showed what was wrong:

| Flight | What was new | Samples with quality at 5 to 15 m | RMS at 5 to 15 m | RMS elsewhere | What it showed |
| --- | --- | --- | --- | --- | --- |
| 1 | as first written: the bare rangefinder, frames 50 ms apart, four corner patches | 6 of 658 | 0.172 m/s (6 samples) | 0.188 | 1520 samples had no height (the transit is above the rangefinder's 12 m) and 900 no attitude (the frame arrives before the pose that covers it) |
| 2 | the height estimator's height; frames wait for the pose | 356 of 658 | 0.470 | 0.153 | speed read 27 to 33% low at 2 to 10 m: the carried bucket fills the corner patches |
| 3 | eight border patches | 446 of 658 | 0.496 | 0.109 | hover noise in proportion to height (0.04 m/s at 1 m, 0.14 at 3 m): a pixel error, not a scale error |
| 4 | the longest baseline the picture allows, up to four frames | 291 of 374 | 0.418 | 0.097 | hover noise down to 0.012 to 0.037 m/s; the error left is all at height. A frame saved in flight showed why: to the mono camera the sim's grass is black, and the picture holds the mat and the hook and nothing else |
| 5 | fourteen 3 m bullseyes laid out as ground to look at | 570 of 661 | 0.355 | 0.040 | scale within 1 to 7%; the error left is in the transit, where the vehicle is also climbing or descending and the picture grows or shrinks |
| 6 | shift, growing and turning of the picture fitted together across the patches, by majority | 562 of 662 | **0.333** | 0.038 | the 5 to 10 m band came down to 0.179; the first transit, at 10 to 15 m, did not move (0.500) |

By height on the last flight:

| True height | Samples | With quality | RMS | Speed read along the flight |
| --- | --- | --- | --- | --- |
| under 2 m | 1125 | 358 | 0.016 m/s | (hovering) |
| 2 to 5 m | 842 | 285 | 0.054 m/s | (slow) |
| 5 to 10 m | 453 | 359 | 0.179 m/s | 12.6% low |
| 10 to 15 m | 209 | 203 | 0.500 m/s | 5.3% low |

By time on the last flight (10 s windows): the climb-out and first transit (10 m, 1.2 m/s on average, accelerating to 4 m/s and climbing) 0.578 m/s; its end 0.161; the hovers at 3 m, 1.1 m and 1.9 m 0.045, 0.013 and 0.017; the carry's start at 4.9 m 0.206 and 0.156; then no quality for the rest of the carry and the set-down (2 or 3 of the eight patches with anything to follow: the ground away from the laid-out bullseyes is bare to this camera).

No quality, on the last flight: 989 samples with fewer than five patches that had texture, 341 on the ground (height under 0.3 m). Every such sample went out with quality 0 and no velocity.

## What it shows

- **The pass line is not met in the sim: 0.333 m/s RMS at 5 to 15 m against 0.15.** The scenario fails its check and is committed failing; its header says so.
- Under 5 m, where the recovery does its work, the odometry is good: 0.016 m/s hovering, 0.054 m/s at 2 to 5 m, with the hook and then the bucket in the picture.
- A bare surface gives no quality and no velocity, as the spec asks: to this camera the sim's grass is one.
- What is left is at height and in the first transit, where the vehicle accelerates, pitches and climbs at once. Its cause is not established. What the numbers allow: at 15 m one pixel between two frames 0.2 s apart is 0.32 m/s, so 0.5 m/s is an error of a pixel and a half; the turning taken out there is several pixels a frame, and it is taken from MAVROS's pose, whose timing against the frames nothing here measures (the spec asks for 2 ms on the aircraft). A pixel and a half of attitude is 6 milliradians, or 12 ms at the pitch rates of that transit. That is a candidate, not a finding.
- Each of the five changes between flights fixed something a flight had shown, and each is a unit test now: the height above the rangefinder's range, the frame that arrives before its pose, the carried bucket in the patches, the baseline, the descent that grows the picture.

## Not shown

- The aircraft's camera: 120 Hz, colour, another lens. Nothing here is a number for it.
- The spec's own pass: a replay against GPS velocity on a real flight.
- The camera and the attitude timed within 2 ms: the sim's pose comes through MAVROS with whatever delay that has, and nothing here measures it.
- What the flight controller does with OPTICAL_FLOW: in these six flights the messages went out to MAVROS and nothing looked at the flight controller's side; its estimator was not set to use them. Sending is now off unless a params file turns it on (`send_optical_flow`), because the message's pixel fields have not been checked against MAVROS's plugin or ArduPilot (REC-6 needs that done).
- The near field on a real mat: one colour of bucket (orange), no shadows but the vehicle's own, no grass beside the mat (green grass against the mat's edge cannot be told from an arm by colour).
