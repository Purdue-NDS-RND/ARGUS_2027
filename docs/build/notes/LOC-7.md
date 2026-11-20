# LOC-7: Marker tracker

Flag: `features.marker_tracker` (node flag of `loc_node`, default false) · Status: DONE · Commits: `git log --oneline --grep '^LOC-7:' platform/build-2027`

## What it solves

The localisation pipeline produced a marker box for every frame a marker was in, about 25 of them a pass, and nothing said they were one marker. Last year's clustering did that after the flight, from logged points. The tracker does it in flight: one track a marker, with a position that narrows as sightings come and a state the judges' list can be made from. It is the missing stage, not a workaround.

## Architecture

- `src/argus_localisation/argus_localisation/marker_tracker.py` (new, no ROS): `MarkerTracker`. `track_messages.py` (new): the tracks as `argus_interfaces` messages.
- `loc_node` with the flag on: the tracker is handed to the pipeline, and `/loc/tracks` (`argus_interfaces/MarkerTrackArray`) is published after every frame; `/loc/status` counts its tracks. The node also watches MAVROS's pose for a position reset.
- Reused from FND-06, not copied: a sighting's noise (`target_estimator.measurement_from_point`), the covariance floor and process noise, `assignment.assign_gated`, `box_is_cut`, `estimator_feed.PositionResetDetector`.
- Sim: `sim/loc_check.py` holds the last `/loc/tracks` message to the true markers; `survey_pipeline` has a marker outside the corners.

## Flow

A marker under the survey's second track, flag on:

- Before: 25 boxes on `/loc/detections`, 25 ground points, no memory of any.
- After: first sighting → a TENTATIVE track with the sighting's covariance; each later sighting within both gates is fused (third: CONFIRMED); out of view for a second → COASTING, kept for good; once the four corner markers are confirmed, a track outside their hull is marked outside and left off the list.

## Code

- One frame, class by class, the two gates and the Hungarian assignment: `marker_tracker.py:214`; a sighting no track took (the 1 m rule, the move after five) `:232`.
- A track: fusing a sighting `:128`; ageing `:248`.
- The boundary: four best-seen corner tracks, each a hull corner `:287`; outside the hull `:161`.
- A position reset and a clock step: `marker_tracker.py:258`, `:267`; the node's detector `loc_node.py:261`.
- The message: `track_messages.py:21`. Cut boxes marked in the pipeline: `pipeline.py:165`.

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider src/argus_localisation/test
69 passed in 2.31s
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider sim/test/test_loc_check.py
23 passed in 0.22s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh waypoint_navigation/survey_pipeline --seeds 1 --variants off,on
| waypoint_navigation/survey_pipeline | 1 | OK | 0.00 | 0 of 6 waypoints | OK | 6.00 | 6 of 6 waypoints |
localisation pipeline: tracks: 3 of 3 markers inside the boundary have their one listed track, the furthest 0.04 m from its marker; 3 listed, 1 marked outside
```

`docs/results/2026-11-20-marker-tracker.md`.

## Spec check

- spec-auditor, one round: INCOMPLETE on one row, no test that only the 1 m gate could pass (added, and shown to fail with the gate taken out), and the results file (written). The second round was skipped: the fixes were a test and the record.
- flight-guardian, round 1: five concerns, no blocker (the published height always 0, no handling of a position reset or a clock step, cut boxes fused, a tracker fault costing the frame its boxes). Round 2: NO FINDINGS, all five resolved; its notes on what the sim cannot show are recorded.
- Beyond the spec, each under Decisions: the 1 m "same marker" rule and the move after five; coasting kept for good; the 0.5 m on the hull and the four separate corners; marker tracks only on `/loc/tracks`; the 200-track limit.

## Not done

- `digit` is -1 on every track until the digit reader and the vote exist (LOC-6, LOC-8); the final position by weighted median is LOC-9, from the sightings the tracker keeps.
- The corner markers sit 0.25 m inside the swath's edge in the planner's layout: whether the survey should fly wider for them is the lead's ("Needs a person").
- One seed, calm, an oracle for a detector.
