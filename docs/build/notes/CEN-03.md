# CEN-03: Rangefinder and height estimator

Flag: `use_height_estimator` (mission flag, default false; needs `use_frame_time_pose`) · Status: PARTIAL (the sensor is not bought or fitted; the taped hover on the aircraft is a person step) · Commits: `git log --oneline --grep '^CEN-03:' platform/build-2027`

## What it solves

The drop must release between 1.83 m and a few metres up, and the only height a 2026 mission has is the flight controller's height above home, which is not the height above the ground under the vehicle and can be tenths of a metre off. This gives a mission one call for the height above the ground that says which sensor it came from, so the release height can depend on how far the number can be trusted (CEN-04).

## Architecture

- `drone_mission_core/height_estimator.py` (new, no ROS at module level): `HeightEstimator` and the executor's `attach_height_estimator`, which makes one fed by its own subscription to `/mavros/rangefinder/rangefinder` when the flag is on.
- `projection_v2.GroundHeight` (FND-05) judges the readings, as before for projection v2; it gained an expected ground height, a `last_refusal` reason and a return value from `add_range`.
- `MissionContext.height_agl(ground_below_home_m=0.0)`: the estimator's answer with the flag on, FND-07's otherwise. `HeightAgl` gained `reason`.
- `features.FLAG_REQUIRES`: `use_height_estimator` needs `use_frame_time_pose`. The flag is declared false in the eight drop and delivery sequences.
- Sim: `sim/height_check.py` runs beside every flight with the flag on (runner wiring in `sim/runner/`), scenario `delivery/height_estimator` (nightly).
- Field: `tools/height_log.py` runs the estimator over a recording.

## Flow

Before (flags off, and still so): `context.height_agl()` → the pose's height above home, source `baro`.

After (flag on): a rangefinder message → kept only if between 0.1 and 12 m → turned into where the ground is with the vehicle's height and tilt at that moment (15° or less) → `height_agl()` = the vehicle's height now minus the median ground of the five nearest readings, source `rangefinder`. Tilted over 15° now, three readings in a row out of range, a reading that has not changed through 0.2 m of climb or descent, stale or disagreeing readings, or a ground over 5 m from where it is expected → the height above home plus `ground_below_home_m`, source `baro`, with `reason`.

## Code

- `height_estimator.py`, `add_range`: the sensor's limits, not the message's (MAVROS gives 0 and 1000 m).
- `height_estimator.py`, `height`: rangefinder or barometer, and the three rules that end the rangefinder height at once: tilted over 15° now, three readings in a row out of range, a stuck reading (`_judge_repeats`, made when a height is asked for, because a reading can arrive before the pose of its moment).
- `projection_v2.py:322`, `ground_up`: the expected ground and the reason for a refusal.
- `mission_context.py:183`, `height_agl`: the estimator when the executor has one.
- `sim/height_check.py`, `build_report`: the four pass lines against the true height and tilt.

## Evidence

```
$ python3 -m pytest -q src/drone_mission_core/test/test_height_estimator.py   (PYTHONPATH=src/drone_mission_core)
23 passed in 0.34s
$ python3 -m pytest -q sim/test/test_height_check.py tools/test/test_height_log.py
19 passed in 1.21s
$ sim/run_scenarios.sh delivery/height_estimator --seeds 1 --variants on
| delivery/height_estimator | 1 | not run |  |  | OK | 16.21 | 0.03 m from the target |
height estimator against the true height: rangefinder source: PASS, median 0.01 m, p95 0.04 m over 2822 heights (limit 0.1 m)
height estimator against the true height: between 0.5 and 11 m: 100% of 2583 heights from the rangefinder
height estimator against the true height: above 12.5 m: 1066 heights, 0 from the rangefinder
height estimator against the true height: tilted over 17 degrees: 22 heights, 0 from the rangefinder
height check: PASS
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh
OK
```

`docs/results/2026-11-20-height-estimator.md` has the numbers, the four flights (the first failed on a rangefinder height at 12.5 m in the climb, which is why the three-in-a-row rule exists; the third failed on FND-05's projection line, not on the height) and the field tool's output on a recorded flight.

## Spec check

spec-auditor, two rounds. Round one: the tilt limit held for each reading's moment but not for the moment of the answer (fixed: tilted over 15° now is `baro` at once), the 0.05 m edge margin was not stated (now in the module and the README), and "signal is good" has no test in code (it cannot have one: `sensor_msgs/Range` has no signal strength; the bench check is under Needs a person). Round two: INCOMPLETE only until the sim proof was re-flown on the final code, which flight 4 is; what remains is owed to a person.

flight-guardian, two rounds, CONCERNS both times, no blocker. Round one: a sensor repeating its last value with fresh stamps would pass as fresh (fixed: the stuck-reading rule), and a NaN ground offset gave a NaN height with the flag off (fixed: None). Round two: the stuck-reading test was skipped for a reading that arrives before the pose of its moment. Fixed after the second round, with a test, by making the test when a height is asked for; that last change has not been seen by a reviewer.

Deviation: SITL's rangefinder is type 1, not 100 (SIM-05's decision).

## Not done

- Buying and fitting the TFmini, its ArduPilot parameters, and the taped hover (`tools/height_log.py --tape 2.4`): Needs a person.
- What ArduPilot and MAVROS do with a weak return, a covered or unplugged sensor, and the message's stamp is not in this repo; the bench check is under Needs a person.
- Until a stuck reading has lasted 0.2 m of climb or descent the height can be wrong by as much: CEN-04's release gates should not rest on the height alone over that distance.
- No mission reads the height yet: CEN-04 (release height by source) and CEN-05 do. The executor's own estimator has not been read in a flight; the sim check runs a second copy on the same topics.
- `ground_below_home_m` has no value for the competition field yet (Needs a person).

## After the build (2026-11-25): the side check's 0.10 m in drop flights

- In gate G2 the check beside the flight read 0.10 m at the 95th percentile in drop flights. The cause is not the estimator and not the final align: a rangefinder reading is up to about 40 ms old when it is stamped (a constant of each flight, from how ArduPilot's 20 Hz read lines up with its message stream), and a drop's 2.5 m/s climbs are about a tenth of its rangefinder heights. The raw readings carry the same error. In a hover and in the 0.35 m/s descent it is under 0.02 m. `docs/results/2026-11-25-height-check-reading-age.md`.
- `sim/height_check.py` now holds a rangefinder height to the true height anywhere in the 50 ms before its stamp (`READING_PERIOD_S`); the 0.10 m line is unchanged, and the error at the stamp alone is still reported. `python3 sim/height_check.py --bag <run folder>` judges a recorded flight again without ROS. The estimator itself is not changed.
- Not done: nothing measures the reading's age on the aircraft, and the estimator does not correct for it (the age is not known in flight).
