# CEN-01: Centring on the estimator, aimed at the payload

Flag: `centre_on_estimator` (mission flag, default false; needs `use_target_estimator`) · Status: DONE · Commits: `git log --oneline --grep '^CEN-01:' platform/build-2027`

## What it solves

The 2026 drop and delivery steer on the newest raw detection, smoothed by a slow filter and ignored inside a 0.4 to 0.5 m deadband, and they put the camera over the target, not the payload. With the flag on they steer on the estimator's track of the bullseye, which is already filtered and is given for the time a command takes effect, and the point they put over the target is the payload's release point.

## Architecture

- `drone_mission_core/centring.py` (new, no ROS): `aim_error` (target − R(yaw) · payload offset − vehicle) and `EstimatorCentring`, which takes the class's primary track once, keeps to it by its id, and says whether the descent is permitted.
- `MissionContext.tracked_target(cls, track_id)` (new call beside FND-07's): the track with that id, whichever is primary now.
- `frame_projection.payload_offset_from_config`: the payload's offset for the mission's `airframe`, from argus_camera's `robot.yaml`; an unverified airframe is refused at start-up.
- `red_bullseye_mission_base.py`: with the flag on, "is there a target" and the tracking solution come from `EstimatorCentring`; the 2026 lines are untouched and run when the flag is off.
- `features.FLAG_REQUIRES`: `centre_on_estimator` needs `use_target_estimator`. Declared false in the eight drop and delivery sequences.
- Sim: the drop judge reports a hold's true error (`hold_height_m`), at the vehicle or at a point of it (`hold_point_body_m`, placed with the vehicle's true heading, which the trace now records). Scenarios `drop/hold_6m`, `drop/hold_2m` (the holds gate G1 needs), `drop/hold_6m_payload` (a `robot.yaml` row, `sim_iris_payload_ahead`, whose release point is 0.30 m ahead and 0.10 m left) and `delivery/centring`, all in the nightly suite. `sim/estimator_check.py` no longer fails a height band a flight was never in.

## Flow

Before (flag off, and still so): raw detection → projection → deadband → low-pass filter (α 0.12 to 0.25) → gain → velocity; descends while the filtered error is inside the tolerance.

After (flag on): `context.primary_target('bullseye', near=...)` once, then `tracked_target` for that track → CONFIRMED or COASTING → error for the payload (`aim_error`) → gain and speed cap → velocity, with no deadband and no filter. While `context.may_descend` says no for that track (COASTING, or not hit for 0.5 s) the steering goes on but the error is reported as infinite: no descent, not "centred", so no handover to the fixed column and no dwell toward the release. Its track TENTATIVE, LOST or gone → no target, even if another bullseye track is there → the 2026 grace time, then the recovery climb, where the track is given up and the next one found is taken.

## Code

- `centring.py:43`, `aim_error`: the payload's forward and left offset turned by the vehicle's yaw and taken off the target.
- `centring.py`, `EstimatorCentring._track`: one approach, one track; another track is refused with the reason.
- `centring.py`, `EstimatorCentring.solve`: the payload's error and the descend rule's answer for that track.
- `red_bullseye_mission_base.py`, `_estimator_tracking_solution`: the flag-on tracking solution; an infinite error while the descent is not permitted. `_estimator_solution` keeps the 2026 limits (`min_projection_altitude_m`, `max_projection_distance_m`). The 2026 lines of the base are not edited: two early `if self._estimator_centring is not None` switches, and `forget()` when a recovery climb starts.
- `sim/judge/missions.py`, `_hold_details`: the hold's true error, over the last unbroken stretch at the hold height before the release.

## Evidence

```
$ python3 -m pytest -q src/drone_mission_demo/test/test_centring_missions.py
34 passed in 0.53s
$ python3 -m pytest -q sim/test/test_judge.py sim/test/test_estimator_check.py sim/test/test_runner.py
175 passed in 6.18s
$ sim/run_scenarios.sh drop/hold_6m --seeds 1 --variants on
| drop/hold_6m | 1 | not run |  |  | OK | 17.49 | 0.00 m from the target |
$ sim/run_scenarios.sh drop/hold_2m --seeds 1 --variants on
| drop/hold_2m | 1 | not run |  |  | OK | 17.49 | 0.01 m from the target |
$ sim/run_scenarios.sh drop/hold_6m_payload --seeds 1 --variants on
| drop/hold_6m_payload | 1 | not run |  |  | OK | 17.12 | 0.31 m from the target |
$ sim/run_scenarios.sh delivery/centring --seeds 1 --variants on
| delivery/centring | 1 | not run |  |  | OK | 16.23 | 0.01 m from the target |
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh
OK
```

Hold error at p95, calm: 0.024 m at 6 m and 0.018 m at 2 m with the flag on, 0.067 m and 0.069 m with the 2026 centring. The payload's release point, 0.30 m ahead and 0.10 m left: 0.019 m, with the vehicle 0.321 m away. Numbers, the failed first attempts and why: `docs/results/2026-11-20-centring.md`.

## Spec check

spec-auditor, two rounds. Round one: every row DONE except payload aiming, which was proven by unit tests only though the ledger says Sim. Closed with `drop/hold_6m_payload` (a sim airframe row with an offset, the vehicle's true heading in the trace, the judge scoring the release point). Round two: that row proven in flight, INCOMPLETE only until the results file had it, which it now has. Its deviations are all under Decisions: acquisition also asks the estimator; a COASTING track is steered on; `estimator_check.py` skips bands not flown; the holds are the 2026 drop mission on a four-ring target.

flight-guardian, two rounds. Round one: BLOCKER, a COASTING track still counted as "centred", so the mission could start its fixed column or its release dwell on a remembered point; and three concerns (another track taken over when its own coasted; the 2026 distance and height limits skipped; the 2 m hold's config allowing 1.7 m). All four fixed. Round two: NO FINDINGS. It notes the payload flight trusts that the sim model's +x axis is the nose: the flight itself bears that out, since the mission's yaw (from MAVROS) and the trace's (from Gazebo) had to agree for the judged point to land on the target.

## Not done

- Wind and the retuned controller (CEN-02); gate G1 is judged on those flights.
- The payload offset has been flown at one heading (north) on a sim airframe row; the aircraft's is not measured (Needs a person: PLT-03, FND-05, CEN-01).
- From 6 m (delivery) or the handoff height (drop) the missions still go down a fixed GPS column: CEN-04 and CEN-05 replace it.
- Descents, and now the handover to the column and the release dwell, pause while the track is COASTING (Decisions).
- `target_timeout_s` is not read on the flag-on path: the track's state does that job.
