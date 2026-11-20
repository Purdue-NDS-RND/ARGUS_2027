# LOC-2: Coverage planner

Flag: none (a new module; nothing flies it until LOC-3) · Status: DONE · Commits: `git log --oneline --grep '^LOC-2:' platform/build-2027`

## What it solves

The 2026 localisation flew a hand-written pattern. The survey needs tracks that are provably close enough that every marker is in at least two frames, for any search area the organisers give, at whatever height is flown. This is new, not a fix.

## Architecture

- `src/drone_mission_core/drone_mission_core/coverage_planner.py` (new; numpy and the standard library, no ROS). No node, topic or config key. LOC-3's `search_pattern` mission is its caller.

## Flow

A search area's corners in the local frame, a height and a camera model go in; a `CoveragePlan` comes out: the waypoints in flying order, the yaw to hold, the track spacing, the path's length and time, and how many frames a marker is in. `coverage(plan, polygon)` gives the share of the area in at least one and at least two frames.

## Code

- The footprint and the spacing rule (0.8 W - 0.85 m): `coverage_planner.py:footprint`, `track_spacing`.
- The lawnmower (`_lawnmower`): the direction that needs the fewest tracks, the outer tracks W/2 - 0.85 m in from the edge, each track as long as the polygon reaches anywhere within its swath, begun at the end nearest `start` when one is given.
- The limits (`_checked`, `_confine`): no plan under 7.62 m or over 4 m/s; with the flight boundary, every waypoint 5 m inside it.
- The two-phase option: `plan_wide_pass` (everything once, tracks W - 0.85 m apart) and `plan_revisit` (short passes over what was seen, each place inside its pass's swath by the marker margin).
- The check: `coverage` counts, for random points inside the polygon, the frames whose footprint holds them, turns included.

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_coverage_planner.py
47 passed in 6.00s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

Thirty random polygons (convex and concave, 40 to 120 m across, at 30 ft and at 35 ft): each at least 99% of its area in two or more frames. At 30 ft with the 4K calibration the footprint is 15.04 x 8.45 m, the spacing 11.15 m or less, and a marker is in 31.7 frames at 4 m/s and 15 Hz.

## Spec check

- spec-auditor, round 1: INCOMPLETE on the revisit pass (places drifting across the tracks could drag one pass off the first of them) and on three things unrecorded (local coordinates in, spacing as an upper limit, the refinement from boundary markers with no owner). Round 2: COMPLETE; the refinement is carried forward to LOC-3 and is a row for the lead under "Needs a person".
- flight-guardian, round 1: five concerns, no blocker: track ends could lie outside the flight boundary; heights under the survey floor and speeds over its cap were planned for; revisit places that are not numbers; `coverage()` too heavy for a tick; the yaw assumes the image's top is forward. Round 2: three resolved; two reduced and still open for LOC-3, which is where they can be closed: the boundary is only checked when it is given, and a re-plan's cost on the Jetson is not measured. After the round: the plan now says whether a boundary was checked, counts legs that cross outside it, refuses a margin under 5 m and a boundary no track fits in, and the pull-back is one vectorised pass a track.
- Beyond the spec, each under Decisions: the `start` point, the boundary, the floor and the cap.

## Not done

- Refining the polygon from the four boundary markers once seen is carried forward to LOC-3 (Decisions): the planner takes whatever polygon it is given and can begin at the end nearest the vehicle.
- Nothing is flown here: the row is proven in Unit. LOC-3's sim flight is where the plan meets the vehicle (turn radius at 4 m/s, the time budget).
