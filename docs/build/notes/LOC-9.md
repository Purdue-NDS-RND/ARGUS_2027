# LOC-9: Final position by weighted median

Flag: `features.loc_median_position` (node flag of `loc_node`, default false; needs `features.marker_tracker`) · Status: DONE · Commits: `git log --oneline --grep '^LOC-9:' platform/build-2027`

## What it solves

The tracker's position for a marker is a weighted mean: every sighting moves it, so a projection made with a wrong attitude or height stays in the answer. With the flag on, the position a track reports is the weighted median of its sightings, which the odd bad projection does not move. It fixes that cause; it does nothing for the error common to all sightings, the vehicle's own position.

## Architecture

- `src/argus_localisation/argus_localisation/final_position.py` (new, standard library only): `weighted_median`, `final_position`.
- `MarkerTracker(median_positions=...)`: a track's state now has the filter's position (`east_m`, `north_m`, what sightings are gated against) and the reported one (`reported_east_m`, `reported_north_m`). `/loc/tracks` carries the reported one. `/loc/status` says which (`track_positions`).
- No new node, topic or message.

## Flow

A marker with 25 sightings, three of them projected 0.6 m off:

- Before (flag off): `/loc/tracks` position = the filter's, pulled a few centimetres toward the three.
- After (flag on): position = the median east and the median north of the 25, by weight: on the marker. Whether the track is outside the boundary is decided on that position too.

## Code

- The median: `final_position.py:26`; a track's position from its sightings `:48`.
- The reported position, worked out when a track's sightings change: `marker_tracker.py:320`; the state's two positions `:91`.
- A moved track's sightings step with it: `marker_tracker.py:151`.
- The flag and its need of the tracker: `loc_node.py:196`. The status: `runtime.py:165`.

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider src/argus_localisation/test sim/test/test_loc_check.py
101 passed in 2.46s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh waypoint_navigation/survey_pipeline --seeds 1 --variants on
| waypoint_navigation/survey_pipeline | 1 | not run |  |  | OK | 6.00 | 6 of 6 waypoints |
localisation pipeline: tracks: 3 of 3 markers inside the boundary have their one listed track, the furthest 0.02 m from its marker; 3 listed, 1 marked outside; positions reported: the median
```

`docs/results/2026-11-20-final-position.md`, with the error budget.

## Spec check

- spec-auditor, one round: INCOMPLETE on the results file and the error budget's record, both written; no code change asked for. The second round was skipped (the fixes were the record and a test).
- flight-guardian, round 1: four concerns, no blocker (a moved track reporting its old place under a fresh stamp; every median worked out every frame; a sim pass that could not tell the median from the filter; zero defaults for the reported position). Round 2: the first, second and fourth resolved; the third resolved by the node's status and the check, with the nightly suite's choice recorded; one new low point recorded (a moved track's history steps by a slip measured from one sighting).
- Beyond the spec, each under Decisions: the median per axis; the weight; the midway tie; the newest 400 sightings.

## Not done

- The published covariance is still the filter's: LOC-10 should not read it as the median's.
- The budget's GNSS term is the aircraft's; nothing here measures it.
