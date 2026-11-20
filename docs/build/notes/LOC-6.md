# LOC-6: Digit reader

Flags: `features.digit_reader`, and `features.template_digit_classifier` for the stand-in classifier (both node flags of `loc_node`, default false) · Status: PARTIAL · Commits: `git log --oneline --grep '^LOC-6:' platform/build-2027`

## What it solves

The detector says where a marker is, not which one. With the flag on, each `marker` crop the pipeline cuts goes to a digit, or to no answer with the reason. A wrong ID costs points and an unread marker is seen again, so the reader would sooner not answer.

## Architecture

- `argus_localisation/digit_reader.py` (new, no ROS): corners, warp, which way up, the classifier interface, the abstention rules, the frame's limits. Two classifiers: `OnnxDigitClassifier` (ONNX Runtime on the CPU; no model is shipped) and `TemplateDigitClassifier`, a stand-in that reads the seven-segment digit of the sim's marker model and starts only in the sim.
- `loc_node` builds the reader when its flag is on; the pipeline calls it after the tracker and keeps the readings in the frame's result. No new topic or message: `/loc/status` gains `digits` (read, abstained, skipped, faults, last_ms, or `error`).
- `tools/synth_targets.py` gains the 2027 marker (`SurveyMarker`, `marker27`), drawn from `build_marker_sdf`'s shapes. `tools/digit_reader_eval.py` (new) scores the reader.

## Flow

A frame with one marker box, flag on:

- Before: the crop was cut and nothing looked at it.
- After: crop → the plate's four corners (pixels outside the greys of the crop's rim, sides fitted) → 64 x 64 patch through the lens model → stood by the X's arms → classifier on four quarter turns → `DigitReading(digit or None, confidence, ten log-probabilities, reason, side_px, sharpness)` in `FrameResult.readings`. This year's marker has no cue for the half turn: a digit that reads one way only, or the same both ways, is answered; a 6 or a 9 comes out as "6 or 9, equally" for LOC-8's vote.

## Code

- Which way up and when to answer: `src/argus_localisation/argus_localisation/digit_reader.py:445` (`read`), with the arms' quarter turn at `:388`.
- Corners and the warp in undistorted coordinates (the lens asked about a 9 x 9 grid): `:307`, `:355`.
- A frame's limits (only `marker` crops, 6 a frame, 30 ms, no crop over 512 px, one crop's fault its own): `:493`. A reader that cannot start leaves the node running: `src/argus_localisation/argus_localisation/loc_node.py:204`.

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider src/argus_localisation/test/test_digit_reader.py tools/test/test_digit_reader_eval.py
44 passed in 10.79s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ python3 tools/digit_reader_eval.py --n 1000 --seed 1
FAIL: precision 100.0% (782 of 782 answers right), coverage 78.2% (782 of 1000 crops answered); lines 99% and 90%
6 or 9: 209 of 211 such crops left as 'a 6 or a 9' (this layout does not say which way up); of the other 789 crops, 99.1% answered
$ python3 tools/digit_reader_eval.py --n 1000 --seed 1 --layout plate_2026
PASS: precision 100.0% (965 of 965 answers right), coverage 96.5% (965 of 1000 crops answered); lines 99% and 90%
```

`docs/results/2026-11-20-digit-reader-synthetic.md` has the sizes from 44 to 143 px and the time a crop takes (2.6 ms on the build machine).

## Spec check

- spec-auditor, round 1: INCOMPLETE. The first build read only last year's plate (digit to one side); the 2027 marker has its digit on the X. Also: the unclear-cue branch untested, a plate with no digit answered "1", no lens test that could fail, the tool unable to take a model or a crop set, no timing, the stand-in free to run on the aircraft. All acted on. Round 2: every flagged row closed or owed to a person, with two left for the builder, both done after it and not re-reviewed: a test that one crop's fault costs only its own reading, and DAT-08's `unreadable` label in the tool.
- flight-guardian, round 1: CONCERNS, no blocker: unbounded work beside the detector, a setting of the wrong kind stopping the node with the flag off, a reader that cannot start taking the detections with it. Round 2: the last two resolved; the first resolved but for one crop's size, since capped at 512 px (after the round, not re-reviewed).
- Recorded deviation: with no cue the spec breaks 6 against 9 "by the rotation that best matches the X arms"; the arms look the same turned half round, so the reader answers neither. Under Decisions and "Needs a person".

## Not done

- The spec's pass (99% precision at 90% coverage on a held-out field day) and the trained classifier: the data hold. On this year's synthetic marker the coverage line is missed for the 6s and 9s alone, until organiser question 8 says whether the marker has a 6/9 mark.
- The Jetson: ONNX Runtime on its CPU and the reader's time beside the detector.
- Not flown in the sim: a marker there is 44 px across.
- Nothing uses the readings yet: the vote over a track is LOC-8, the re-visit LOC-4.
