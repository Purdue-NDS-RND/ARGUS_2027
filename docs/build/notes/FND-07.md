# FND-07: MissionContext additions

Flag: none of its own. The estimator calls answer "nothing" unless `use_target_estimator` is on (default false); `height_agl()` and `latlon_to_local_enu()` need no flag · Status: DONE · Commits: `git log --oneline --grep '^FND-07:' platform/build-2027`

## What it solves

FND-06 put an estimator inside the executor, but a mission had no way to read it, and the rules for acting on a track (how fresh, in what state) would otherwise be rewritten in every mission. This gives missions one set of calls on their context and answers the two rules there. It is plumbing: no mission calls any of it until CEN-01.

## Architecture

- `drone_mission_core/target_access.py` (new, standard library only): `PrimaryTarget`, `Permission`, `HeightAgl`, the two rules (`descent_permission`, `release_permission`), `height_agl`, `latlon_to_local_enu`.
- `MissionContext` gains `estimator_ready()`, `primary_target(cls, near=None)`, `may_descend(cls, track_id=None)`, `may_release(cls, track_id=None)`, `height_agl()`, `actuation_time_ns()`, `latlon_to_local_enu(lat, lon)`. `pose_at(t)` was already there (FND-04).
- `EstimatorFeed.ready(now_ns)` (new): the pose history can place a frame taken now.
- No new topic, node, parameter or dependency. The flags-off executor imports one more module, `target_access.py`, which needs neither ROS nor numpy.

## Flow

Before: a mission could read the latest raw detection and the latest pose, nothing else.

After, with the estimator on: `ctx.primary_target("bullseye")` → the feed's primary track for now + 0.08 s → position, covariance, sigma, age and state. `ctx.may_descend("bullseye")` → that track must be CONFIRMED and its last hit no more than 0.5 s old, or the answer is false with the reason ("the bullseye track is COASTING, not CONFIRMED"). With the estimator off: no target, not ready, and both rules say no ("the target estimator is off").

## Code

- `target_access.py:90`, `_permission`: no track, a track that is not CONFIRMED, an age that is not a number or is negative, or an age over the limit all refuse, each with its reason and the track it is about.
- `mission_context.py:147`, `_target_to_judge`, with the two rules at `:159` and `:171`: asked with a `track_id` they answer about that track only ("gone" if it no longer exists), so a second target of the same class cannot answer for a lost one.
- `mission_context.py:183`, `height_agl`: the rangefinder's ground from projection v2 when it is healthy, otherwise the height above home, always with the source; nothing when the newest pose is over 0.5 s from now.
- `mission_context.py:200`, `latlon_to_local_enu`, with `target_access.py:133`: the GPS fix is paired with the local position of its own moment, and the offset to the asked coordinate is added in ArduPilot's scale. No fix, an old fix or no pose of that moment gives nothing.

## Evidence

```
$ PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_target_access.py
15 passed in 0.03s
$ python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_context_target_access.py
8 passed in 0.43s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

`test_context_target_access.py` is the "fake context" test: the real `MissionContext` on the real executor class with ROS replaced by stand-ins. With the flags on it feeds poses and three contract detections and reads a CONFIRMED target, both permissions, and then, 0.4 s after the last frame, a COASTING target with both refused. With the flags off it checks every call says nothing and none of the estimator's numpy modules is imported. The CONFIRMED rule was broken once on purpose (two tests in the first file and one in the second failed) and put back.

Unit level only, as the ledger row says. No sim run: nothing calls these yet.

## Spec check

spec-auditor, one round: COMPLETE, every item DONE. Its notes for the lead are under Decisions: the rules are advisory until CEN-01 and CEN-04 call them, and the 0.5 s descend limit never binds because a track coasts at 0.3 s.

flight-guardian, two rounds, CONCERNS both times, no blocker. Round two closed the staleness and pairing findings; after it `near` and `track_id` became keyword-only and the height gained the age of its pose, not reviewed again. Round one: Fixed after it: the rules can be asked about one track by its id; a pose over 0.5 s old gives no height; the GPS fix is paired with the pose of its own moment and a stale or missing fix gives no position. Left standing:

- **`height_agl()` on `baro` is the height above home, not above the ground.** The spec asks for exactly this pair (value and source). A release-height gate must read `source`; `safety_limits.py`'s gates take a bare number. CEN-03's height estimator (tilt limit, ground offset) replaces the source, and CEN-04 is where the gate is wired.
- **The rules are advisory.** Nothing stops a mission that never asks. CEN-01 and CEN-04 must show their missions do.
- **A steady payload in the rangefinder's beam** would be believed as ground up to 5 m above home (FND-05's limit).
- **Asked without a `track_id`, a rule answers about whichever track of the class is primary.** That is the convenient form for a mission with one target in sight; a mission that has chosen a track must pass its id (CEN-01).
- **A track's id does not pin its position.** FND-06's five-rejection rule can move a CONFIRMED track up to 2 m; a mission about to release should compare the position with the one it chose.
- **The 0.5 s pose limit is wider than the 0.17 m between the 2.0 m release gate and the 1.83 m rule** at a 0.5 m/s descent. `pose_age_s` is there for the release gate to apply its own, tighter bound (CEN-04).

## Not done

- No mission calls any of this yet (CEN-01, CEN-04, CEN-06, LOC lane).
- The mission tests' own `FakeContext` (`test_package_drop_mission.py`) does not have the new calls; CEN-01 adds them when a mission first uses one.
- `height_agl()` passes on projection v2's ground height until CEN-03 builds `height_estimator.py`.
- With FND-06's numbers the 0.5 s limit for descending cannot be reached (a track is COASTING from 0.3 s). Recorded under Decisions for the lead.
