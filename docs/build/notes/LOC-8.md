# LOC-8: Weighted vote

Flag: `features.digit_vote` (node flag of `loc_node`, default false; needs `features.marker_tracker` and `features.digit_reader`). Setting: `digit_ids_unique` (default true) · Status: DONE · Commits: `git log --oneline --grep '^LOC-8:' platform/build-2027`

## What it solves

One reading of a marker is one opinion, and some are wrong or no answer at all; a track is seen about 25 times in a survey. With the flag on a track's digit is the weighted vote of all its readings, and two tracks claiming one digit are settled over the whole field instead of both reporting it.

## Architecture

- `argus_localisation/digit_vote.py` (new, no ROS): the weight of a sighting, the summed log-probabilities of a track, when a track has a digit, the assignment over tracks and digits (drone_mission_core's Hungarian method).
- `marker_tracker.py` says which track took each of a frame's sightings (`last_assignment`) and fills a track's `digit` and `digit_log_prob` from the vote; `pipeline.py` adds each frame's readings to their tracks' votes; `loc_node.py` builds the vote behind the flag.
- No new topic or message: `MarkerTrack`'s `digit` and `digit_log_prob`, at -1 and zeros since LOC-7, are now filled. `/loc/status` gains `digits.vote`.

## Flow

A frame with one marker box, tracker, reader and vote on:

- Before: the box went to its track; its reading stayed in the frame's result; the track's `digit` was -1.
- After: box → track (`last_assignment`) → the reading's ten log-probabilities, floored at ln 0.001, times its weight (size x sharpness x cos² of the view angle x detector confidence), added to the track's sum → for listed tracks, one assignment over tracks and digits → `digit` on `/loc/tracks`, or -1 while the evidence does not say.

## Code

- The weight and the sum: `src/argus_localisation/argus_localisation/digit_vote.py:73`, `:115`; a frame's readings to their tracks: `:135`.
- When a track has a digit, and the assignment with its margin (a digit is kept only if taking it away would cost ln 9): `:197`, `:219`.
- The tracker's side (listed tracks only, a fault in the vote costing the digits and not the tracks): `src/argus_localisation/argus_localisation/marker_tracker.py:372`; the node's flag and setting: `src/argus_localisation/argus_localisation/loc_node.py:246`.

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider src/argus_localisation/test/test_digit_vote.py
12 passed in 1.25s
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider src/argus_localisation/test
149 passed in 12.09s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

- Two tracks claiming a 1, one with twenty sure readings and one whose readings lean to 1 over 7: the first is the 1 and the second the 7; with `digit_ids_unique` false the second is left unknown.
- Reader and vote together on drawn markers 60 px across and blurred, eight sightings each: all ten digits right on last year's plate; on this year's marker the eight answerable digits right and the 6 and the 9 unknown.
- Through the node, four frames of a drawn 7: the track's digit on `/loc/tracks` is -1, -1, 7, 7 (a digit once the track is confirmed).

## Spec check

- spec-auditor: COMPLETE in one round. Two observations acted on: votes are forgotten when a clock step drops a track, and the 20-track cap has a test.
- flight-guardian, round 1: CONCERNS, no blocker: the vote read unguarded inside the tracker's `tracks()`, the assignment solved on every voting frame beside the detector, the vote's setting declared with the reader's, votes kept past a clock step. Round 2: all four resolved, NO FINDINGS.
- Beyond the spec's words, each under Decisions: the form of the four weights, the floor on a reading, the score against "nothing can be said", the ln 9 margin (equal claims are left unknown, not split), listed tracks only, the cap and the twice-a-second solve.

## Not done

- Not flown in the sim: the reader is not (a marker there is 44 px), and the row is proven in Unit.
- The weights' constants are judgement on synthetic patches; the field day's crops are where they are checked.
- With this year's marker model a 6 and a 9 stay without a digit (LOC-6, organiser question 8).
- The solve's time on the Jetson: `digits.vote.last_solve_ms` in `/loc/status` on the bench.
