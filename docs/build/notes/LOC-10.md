# LOC-10: Judges' list

Flag: `features.judges_list` (node flag of `loc_node`, default false; needs `features.marker_tracker`). Setting: `judges_list_dir` · Status: DONE · Commits: `git log --oneline --grep '^LOC-10:' platform/build-2027`

## What it solves

In 2026 the list was typed by hand after the flight. With the flag on the localisation node writes `ID,Latitude,Longitude` during the survey, so the list exists before the vehicle lands and is never more than a second behind the tracks.

## Architecture

- `argus_localisation/judges_list.py` (new, no ROS): rows from tracks, local metres to latitude and longitude, the CSV, the KML, the map image, the atomic write, and a writer with its own thread.
- `loc_node.py`: with the flag on, a subscription to `/mavros/global_position/global` (to place the local frame on the earth), a publisher on `/mavros/statustext/send` (the count, to the ground station), and `list` in `/loc/status`.
- Sim: `sim/scenarios/localisation/survey_list.yaml`, the first scenario scored by the localisation judge; the build points the node's folder at the run folder, and `sim/loc_check.py` keeps the reader's, the vote's and the list's own status.

## Flow

A marker's track gets its digit in the middle of a survey:

- Before: the track went out on `/loc/tracks`, and nothing was written.
- After: listed track with a digit → its reported position, through the local frame's origin (the median of GPS fixes paired with the local position of their moment), to latitude and longitude → a row → the writer's thread, within a second: temporary file beside `judges_list.csv`, synced, renamed onto it; then the KML and the map → "LOC list: 3 markers" as a STATUSTEXT. Stopping the pipeline writes the list at once.

## Code

- Rows, and why an unread marker is not one: `src/argus_localisation/argus_localisation/judges_list.py:140`; the origin: `:89`; metres to degrees: `:73`.
- The write that is whole or not at all: `:234`; the list's three files, an earlier list put aside: `:260`, `:300`; the writer's thread: `:320`.
- The node: `src/argus_localisation/argus_localisation/loc_node.py:294` (start), `:556` (each frame's list), `:574` (the count, at most every 2 s), `:644` (the close).

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider src/argus_localisation/test sim/test/test_loc_check.py
189 passed in 12.80s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh localisation/survey_list --seeds 1 --variants on
| localisation/survey_list | 1 | not run |  |  | OK | 3.75 | 3 of 4 markers correct |
```

`docs/results/2026-11-20-judges-list.md`: three rows within 0.04 m of their markers, the fourth marker outside the search area and off the list by design, the list on the disk before landing and rewritten 13 times in flight, nine status texts sent, the pipeline at the camera's 10 frames a second with everything on.

## Spec check

- spec-auditor, round 1: INCOMPLETE on two proof gaps, no missing behaviour: the writer's in-flight write had no test that could fail (now one that neither stops nor wakes it), and the sim did not show a STATUSTEXT being sent (now counted in `/loc/status` and required by `sim/loc_check.py`; the third flight sent nine). Its notes acted on: the list's close is guarded, `mavros_msgs` is a declared dependency, the vote's `settle` has a test. The fixes were tests and a counter; no second round.
- flight-guardian: NO FINDINGS in one round. Its three observations acted on: the count is sent at most every 2 s, the text says when the list is not reaching the disk, and a list already in the folder is put aside instead of written over.
- Recorded under Decisions: unread markers are not rows; "every change" is at most once a second on the list's own thread; the origin from the vehicle's own fixes.

## Not done

- QGroundControl showing the status text, and `judges_list_dir` on the aircraft: bench and profile ("Needs a person").
- The flags-off variant of the scenario writes no list and scores 0 by design; it is in the nightly suite with the others.
- The merged list after landing is LOC-12.
