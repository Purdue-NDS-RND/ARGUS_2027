# FND-04: Pose history buffer

Flag: `use_frame_time_pose` (mission flag, default false, declared in all ten sequences; turn it on with `feature_overrides:=use_frame_time_pose`) · Status: DONE · Commits: `git log --oneline --grep '^FND-04:' platform/build-2027`

## What it solves

A frame is tens of milliseconds old when anything can act on it, and in 2026 a detection was put on the ground with whatever pose had arrived last. This keeps the recent poses so that later code can ask where the vehicle was at the moment a frame was taken. It is the building block for the fix; nothing uses it yet.

## Architecture

- `drone_mission_core/pose_buffer.py` (new, no ROS, standard library only): `PoseBuffer` with `add_pose`, `add_velocity`, `pose_at`, `velocity_at`; `PoseAt`.
- `drone_mission_core/frame_time.py` (new): `attach_pose_buffer`, the executor's two extra subscriptions (`/mavros/local_position/pose`, `/mavros/local_position/velocity_local`), made only with the flag on.
- `mission_executor.py`: one attribute, `self._pose_buffer` (None with the flag off).

## Flow

Before: the executor keeps only the latest pose.

After, flag off: the same; no buffer, no extra subscription (tested through the real executor class).

After, flag on: each pose and velocity from MAVROS goes into the buffer by its stamp. `node._pose_buffer.pose_at(stamp_ns)` then gives the pose between the two samples around that instant, or None.

## Code

- `pose_buffer.py`, `pose_at`: linear position and SLERP attitude between the bracketing samples; None if they are more than 0.12 s apart or the stamp is before the oldest.
- `pose_buffer.py`, `_extrapolate`: up to 30 ms past the newest sample, position by the reported velocity, attitude by the turn between the newest pose and one at least 10 ms older; None without those.
- `pose_buffer.py`, `_insert`: keeps 5 s in stamp order; ignores a stray old stamp; restarts the history after three in a row (the clock stepped back).
- `frame_time.py`, `attach_pose_buffer`: returns None before any ROS import when the flag is off.

## Evidence

```
$ PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_pose_buffer.py
21 passed in 0.06s
$ python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_executor_pose_buffer.py
4 passed in 0.08s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

The spec's acceptance, from `test_pose_buffer.py`: a flight at 1 m/s yawing 30 °/s, sampled at 30, 20 and 10 Hz and asked every 7 ms, is rebuilt to under 1 cm and 0.2° (in fact exactly, the motion being linear); a 130 ms gap, 31 ms ahead, and every case with no usable velocity or turn give None. The buffer's tests were written before the module. Proven at Unit level, as the row asks; no sim run.

## Spec check

spec-auditor, one round: every spec line DONE; INCOMPLETE only until the choices the spec leaves open were recorded, which they now are under Decisions (the flag, the stamp type, the extra refusals when extrapolating, the velocity topic, out-of-order samples, where the buffer lives). It said a second round was not needed for that.

flight-guardian, one round: CONCERNS, no blocker, all about what happens once something reads the buffer. Fixed after the round and not reviewed again: a backward step of the clock no longer mixes two timelines (the history restarts), and the turn is no longer taken from two poses a few milliseconds apart. Left for the consumers:

- **At 4 to 5 Hz of pose the buffer answers None for every frame** (the gap rule). It needs the stream keeper's 30 Hz. FND-05 and FND-07 must treat None as "no target".
- **The buffer does not check that a pose stamp is synchronised flight-controller time.** FND-03's guard does; the two flags are not tied together.

## Not done

- Nothing reads the buffer yet: FND-05 (projection at capture time) and FND-07 (`pose_at` on the mission context).
- No angular rate is kept; if a consumer needs the turn rate itself, `velocity_body` would have to be added.
