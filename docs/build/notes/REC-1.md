# REC-1: Green X and bucket detector

Flag: `features.recovery_cv` (node flag of the new `recovery_cv` node, default false) · Status: PARTIAL · Commits: `git log --oneline --grep '^REC-1:' platform/build-2027`

## What it solves

Recovery starts by finding the mat's green X and the bucket on it. Nothing looked for either. With the flag on, a node finds both in each frame of the downward camera, with no trained model, and reports them the way every other detector does.

## Architecture

- `drone_target_cv/green_x.py` (new, no ROS): `detect(image)` → the X (centre, angle, span, confidence and its four parts) and the bucket (centre, size, handle).
- `drone_target_cv/recovery_cv_node.py` (new): the node. Images with a queue of one; `green_x` and `bucket` on `/perception/recovery_cv/detections` (FND-02); the handle's two ends on `/perception/recovery_cv/handle` for every frame, empty when none shows.
- `tools/green_x_try.py` (new): the spec's pass on synthetic renders. `sim/recovery_cv_check.py` (new): the detector against the truth in a flight; scenario `waypoint_navigation/survey_x_mat`; the survey launch starts the node on `recovery_cv:=true`.

## Flow

A frame with the mat in it:

- Before: nothing.
- After: CLAHE → green mask and white mask → the green that white shuts in (or, with the mat running out of the picture, the green inside a white patch) → two arms by RANSAC then Huber → centre where they cross, confidence from white surround, arm angle, inlier share and arm widths → on the crossing, a round solid blob of a bucket's size → `green_x` and `bucket` boxes with the image's stamp.

## Code

- Is it an X, and its confidence: `src/drone_target_cv/drone_target_cv/green_x.py:274`; where the X's green is looked for: `:324` (shut in by white), `:345` (inside a white patch).
- The bucket and its handle: `:202`, `:249`; what goes on the contract: `:419`.
- The node's frame: `src/drone_target_cv/drone_target_cv/recovery_cv_node.py:122`; the sim check's verdict: `sim/recovery_cv_check.py:69`.

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider src/drone_target_cv/test/test_green_x.py
15 passed in 9.60s
$ python3 tools/green_x_try.py --n 300 --negatives 200 --seed 1
PASS: recall 99.7% (299 of 300 within the centre and angle lines; 300 found at all), 0 false alarm(s) on 200 bullseye, marker and green-noise renders; lines 98% and none
bucket: found on 103 of the 150 mats that had one, and on 0 of the 150 that had none
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh waypoint_navigation/survey_x_mat --seeds 1 --variants on
| waypoint_navigation/survey_x_mat | 1 | not run |  |  | OK | 6.00 | 6 of 6 waypoints |
```

`docs/results/2026-11-20-green-x-detector.md`: in the sim the X and the bucket are reported in all 24 frames that have the whole mat in view, the X a median 4.3 px from the truth at 3.5 m/s, and nothing is reported when the mat is not there.

## Spec check

- spec-auditor, one round: INCOMPLETE. Not built and recorded: the flow camera taking over below 2 m and the hook-to-handle offset (the row is PARTIAL for it). Acted on by recording: the handle's narrow band and its proof, how the pass lines are read, what "scale from the picture" changes. The scenario was flown again under its committed name.
- flight-guardian, one round: CONCERNS, no blocker; nothing reads the topics yet. Acted on: a shape test so that a hook or a shadow is not a bucket (and false buckets counted: none in 150); a handle message for every frame; a bounded closing step and the slowest frame in the log; the truth's interpolation counted in the sim check. Left for REC-2 and REC-3, under Decisions: the estimator needs a pixel sigma for this detector, and a hover to prove the centre.

## Not done

- Below about 3 m the X is not found, and the spec's hand-off to the flow camera there is not built: from 3 m down to the handle there is no detection.
- The centre to the spec's 1% in a flight: every sim frame was at speed.
- Real pictures of a real mat and bucket; time a frame on the Jetson.
- The X's angle is found and not published.
