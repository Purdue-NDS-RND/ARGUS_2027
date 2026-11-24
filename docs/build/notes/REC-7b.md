# REC-7b: Visual homing: marker (P1), keyframes (P2)

Flags: mission flags `marker_homing`, `keyframe_homing` (default false; need `package_recovery`), and the recovery detector node's `features.marker_homing`, `features.keyframe_homing` (default false; need `features.recovery_cv`) · Status: PARTIAL · Commits: `git log --oneline --grep '^REC-7b:' platform/build-2027`

## What it solves

The bucket is set down where the return point's coordinates say, and coordinates are off by what GPS is off, or by 3 to 5% of the distance flown without it. REC-7 left a place for something that sees the return point in the last metres; nothing gave it. Now two things do: a marker lying at the return point, and the pictures of the takeoff climb.

## Architecture

- `drone_target_cv/home_marker.py` (new, no ROS): an AprilTag 36h11 of one id found in a frame (OpenCV's aruco).
- `drone_target_cv/keyframe_homing.py` (new, no ROS): `KeyframeHoming` (pictures kept at four heights of the climb; the takeoff point found in a later picture by ORB matches and a homography), `Climb` (where the takeoff point is, which pictures are the climb's).
- `recovery_cv_node.py`: hosts both behind its node flags; either gives a `home_point` detection on the contract.
- `detection_contract.py`, `target_estimator.py`: the class `home_point`, on the ground at height 0.
- `recovery_flight.py`: `Readings.homing`, the rule for believing and keeping a fix, handed to REC-7's `return_goal`. `package_recovery_mission.py` reads the estimator's confirmed `home_point` track.
- Sim: the object kind `home_tag` (`sdf_objects.build_home_tag_sdf`), scenarios `recovery/marker_homing` and `recovery/keyframe_homing`.

## Flow

- Before: RETURN flies to the return point's coordinates and sets the bucket down there.
- After (flag on): the detector node sees the marker (or finds the takeoff point from the climb's pictures) → `home_point` on the contract → the estimator's track → once CONFIRMED and within 4 m of where the return point was thought to be, the mission says `homing: the return point is taken as seen` and, inside the last 10 m, the way back goes there instead.

## Code

- The marker in a frame: `src/drone_target_cv/drone_target_cv/home_marker.py:76`. The takeoff point from the keyframes, and what is believed: `src/drone_target_cv/drone_target_cv/keyframe_homing.py:141`; the climb: `:172`.
- The node: the marker at once, a frame held for the pose of its moment: `src/drone_target_cv/drone_target_cv/recovery_cv_node.py:174`, `:154`.
- The fix believed, kept, and handed on: `src/drone_mission_core/drone_mission_core/recovery_flight.py:807`; which return targets each flag is allowed for: `:1020`.
- The mission's reading of the track: `src/drone_mission_demo/drone_mission_demo/missions/package_recovery_mission.py:345`.

## Evidence

```
$ pytest test_home_finders.py test_recovery_cv_node.py test_recovery_flight.py test_package_recovery_mission.py   (as scripts/check.sh runs them)
12 passed, 7 passed, 38 passed, 26 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh recovery/keyframe_homing --variants on
   OK          recovery/keyframe_homing seed 1 on: 19.95 points, 0.04 m from the return point
Keyframe homing: a picture kept at 3.3 m, the takeoff point at (320, 180) px in it (1 kept)      [and at 6.3, 9.5, 13.3 m]
Keyframe homing: the takeoff point first found, at (264, 217) px by 26 matches with the picture kept at 9.5 m
[package_recovery] homing: the return point is taken as seen, at (0.02, 0.03) in the local frame
$ sim/run_scenarios.sh recovery/marker_homing --variants on
   FAIL        recovery/marker_homing seed 1 on: 18.14 points, 1.49 m from the return point
the mission's log has no line with: homing: the return point is taken as seen      [the tag read 65 times in the climb, 0 times on the way back]
```

Marker homing set the bucket down on the marker once (0.063 m, against 1.488 m without the flags), under a first rule that the flight-safety review blocked. The rule's tests were written with the rule; the hold of a frame for its pose, the second of agreement, the wait at the coordinates and the two classes each came from a flight or a review.

`docs/results/2026-11-24-visual-homing.md` has every flight.

## Spec check

spec-auditor, two rounds: INCOMPLETE, as a PARTIAL row is. Round one: the marker target's id is never read (now said in the log and recorded), the mission could not tell the two finders apart (now two classes), and the sim proof was unrecorded. Round two: those closed; what stays open is the sim proof of the marker on the committed code. flight-guardian, two rounds: BLOCKER (a fix believed on one tick was kept for the rest of the mission), then CONCERNS after the rework; the ones left are under "Not done".

## Not done

- Marker homing in the sim on the committed rule (why PARTIAL): three flights, no place taken; the tag is read in almost no frame on the way back with the bucket on the hook, and why was not established. The scenario fails its own log line.
- No flight corrects an error through keyframes: the sim's local frame does not drift. Their correction is a unit test's.
- Nothing on real ground, a real camera or the aircraft's computer; one seed.
- Left by the flight-safety review for the lead: the place taken is flown to and the bucket lowered on it up to 8 s after it was last seen (and through the whole lowering), which is a decision about how old a sighting may be; the landed state that decides the takeoff point has no age; a keyframe track's sureness is the detector's 3 px, not the match's; the marker's id in the return target is not checked against the detector's.
- A keyframe match took up to 0.3 s in the sim's container. It runs after the frame's X and bucket have gone out, three times a second at most, within 10 m of the takeoff point.
