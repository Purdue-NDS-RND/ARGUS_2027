# The gated drop: a real package from 2.4 m, and the three paths round it

Measured on 20 Nov 2026 (UTC) on a club laptop, on `platform/build-2027` at `c850dc0` plus the CEN-04 change. Sim image `argus-sim:humble`. One flight per scenario, calm air, flags on (`--variants on`), all on sim time.

**Result.** With `drop_release_gates` on, the drop mission steered on the estimator from the 8 m handoff down to the release height, let a real 0.5 kg package go from 2.55 m when every gate had held for 0.5 s, and the package came to rest 0.03 m from the target's centre (0.14 m in the five flights before the mission was told where the sim hangs the package). The three other paths the spec names were each flown once: a track lost at 3 m (nothing released, four recovery attempts, then the mission failed and the sequence moved on), the timeout path (one re-acquisition after 25 s, a last-resort release after the second 25 s), and the floor (handed over under 2.0 m, climbed back, released in the band).

## Commands

```
sim/run_scenarios.sh drop/gated --seeds 1 --variants on
sim/run_scenarios.sh drop/gated_track_loss --seeds 1 --variants on
sim/run_scenarios.sh drop/gated_timeout --seeds 1 --variants on
sim/run_scenarios.sh drop/gated_floor --seeds 1 --variants on
```

Flags in every scenario: `use_frame_time_pose`, `use_target_estimator`, `use_height_estimator`, `centre_on_estimator`, `centring_retune`, `drop_release_gates`; node flags `target_cv.stamp_from_image`, `target_cv.detection_contract`. `use_ring_fit` and the `ring_fit` node are on in all but the track-loss scenario. Sequence `package_drop_demo.yaml`, target at 10 m east, 15 m north.

## The four flights

| Scenario | What it sets up | Run | Released | Release height (true) | Distance from the target | Lowest true height |
| --- | --- | --- | --- | --- | --- | --- |
| `drop/gated` | 5 m bullseye, real 0.5 kg package (`fake_drop: false`), airframe `sim_iris_parcel` | OK | yes, on the gates | 2.545 m | 0.027 m (package at rest) | 2.51 m |
| `drop/gated_track_loss` | 2.8 m bullseye, no ring fit: the box is cut by the picture's edge at about 3 m | OK | no | n/a | n/a | 2.894 m |
| `drop/gated_timeout` | descent slowed to 0.1 m/s by the scenario's config, so the band is not reached in 25 s | OK | yes, last resort | 4.129 m | 0.035 m (vehicle at release, fake drop) | 4.061 m |
| `drop/gated_floor` | handoff at 2.25 m on the flight controller's height, which reads 0.19 m high | OK | yes, on the gates | 2.288 m | 0.018 m (vehicle at release, fake drop) | 1.898 m |

**Which code each flight flew.** `drop/gated` and `drop/gated_floor` are flights of the code as committed. `drop/gated_track_loss` and `drop/gated_timeout` were flown just before two later changes, neither of which is on their paths' logic: the fix to FINAL_ALIGN's log helper (found by the first floor flight, below: which line of code logs, and one timer per kind of line instead of one for all), and the `sim_iris_parcel` airframe row, which only `drop/gated` names. Everything else in the change, including the last resort's lowest height by sensor and the release command not being sent on gates that held earlier, was in the code they flew. They were not flown again: one flight proves a scenario, and the mission tests cover the final code.

"Run OK" is the runner's verdict: the mission sequence ended, and the checks beside the flight (message rates, projection against the truth, estimator against the truth, height estimator against the true height) passed.

### `drop/gated`: the mission's own lines

```
[package_drop] Entering final align at 8.00 m (handoff 8.00 m)
[package_drop] State: TRACK_AND_DESCEND -> FINAL_ALIGN
[package_drop] release gates held for 0.5 s: error 0.03 m, sigma 0.03 m, speed 0.00 m/s, tilt 0.1 deg, 2.55 m on the rangefinder
[package_drop] State: FINAL_ALIGN -> DROP_PAYLOAD
[package_drop] Closing sprayer for payload drop
[package_drop] State: DROP_PAYLOAD -> COMPLETE
```

- From entering FINAL_ALIGN to the gates holding: 23.7 s of wall time at a real-time factor of 0.81, about 19 s of the 25 s the spec allows, with no pause in the descent (18 s in the flight before it).
- Checks beside the flight: estimator against the truth 0.15 m at p95 at 15 m (limit 0.3) and 0.02 m at 6 m (limit 0.15); near 2.4 m the vehicle was there for 7 samples before the release, every one on a confirmed track 0.01 m from the truth ("passed through, not judged"). Height from the rangefinder within 0.07 m of the truth at p95.
- Ring fit: looked at 974 frames below 6 m, rings found in 261.
- Between 15 and 8 m (before FINAL_ALIGN) the descent paused four times on a COASTING track, each for less than the 2 s at which the line is repeated; that descent took 25 s of sim time, the same as in the flight before, which had no pause. Reported under Decisions; the rule is as built.

**The 0.14 m of the first five flights was not aim error.** The sim hangs a carried payload 0.13 m to the left of the vehicle's centre (`CARRY_OFFSET_BODY_M` in `sim/scenario/build.py`), and those flights centred the vehicle: the package came to rest 0.144, 0.130, 0.135, 0.141 and 0.143 m from the target. With the airframe row `sim_iris_parcel` (the same vehicle and camera, `payload_offset_m: [0.0, 0.13, -0.12]`) named in the scenario, CEN-01's payload aiming puts the package over the target: 0.027 m. This is the sim's stand-in for measuring the real release point (`argus_2027`'s row in `robot.yaml` is still unmeasured).

### `drop/gated_track_loss`

```
[package_drop] Entering final align at 8.00 m (handoff 8.00 m)
[package_drop] not descending: the bullseye track is COASTING, not CONFIRMED
[package_drop] no target to centre on: bullseye track 1 is LOST
[package_drop] Target tracking lost; holding for 4.0 s before recovery
[package_drop] State: FINAL_ALIGN -> TARGET_NOT_FOUND
[package_drop] Target lost, climbing to 4.1 m (attempt 1/4)
[package_drop] Entering final align at 4.23 m (handoff 8.00 m)
[package_drop] release gates: waiting on track, height (2.90 m on the rangefinder, error 0.02 m)
...
[package_drop] Target lost, climbing to 4.1 m (attempt 4/4)
[package_drop] Target recovery exceeded 4 attempts
Mission 'package_drop' ended with status FAILURE
Mission 'package_drop' failed; continuing to next mission
```

The descent stopped at 2.9 m each time, on the coasting track, 0.3 m above the band; nothing was released and the vehicle never went lower. Five spells in FINAL_ALIGN, four recovery climbs.

### `drop/gated_timeout`

```
[package_drop] Entering final align at 8.00 m (handoff 8.00 m)
[package_drop] the release gates did not hold within 25 s: looking for the target again
[package_drop] State: FINAL_ALIGN -> TARGET_NOT_FOUND
[package_drop] Target lost, climbing to 6.5 m (attempt 1/4)
[package_drop] Entering final align at 6.62 m (handoff 8.00 m)
[package_drop] not descending: the bullseye track is COASTING, not CONFIRMED
[package_drop] the release gates did not hold within the last 25 s: releasing at the current estimate, 0.03 m off
[package_drop] State: FINAL_ALIGN -> DROP_PAYLOAD
[package_drop] Fake drop complete
```

The two spells lasted 28.5 s and 28.4 s of wall time at a real-time factor of 0.90: 25.7 and 25.6 s of sim time. The last-resort release was from 4.13 m, 0.035 m from the target.

In the second spell the descent paused once on a COASTING track (between 5.5 and 6 m, where the 5 m bullseye no longer fits the picture and the ring fit finds rings in about a third of the frames). The rule was left as it is; it is reported under Decisions.

### `drop/gated_floor`

```
[package_drop] Entering final align at 2.25 m (handoff 2.25 m)
[package_drop] release gates: waiting on height (2.03 m on the rangefinder, error 0.02 m)
[package_drop] final align: 1.99 m is below 2 m, climbing back
[package_drop] release gates: all holding for 0.15 s
[package_drop] release gates held for 0.5 s: error 0.03 m, sigma 0.03 m, speed 0.01 m/s, tilt 0.4 deg, 2.28 m on the rangefinder
[package_drop] Fake drop complete
```

The descent's own overshoot carried the vehicle to 1.898 m (true), under our 2.0 m floor and above the rules' 1.83 m. FINAL_ALIGN climbed straight back, went on up into the band (2.2 m and over) and released there, at 2.288 m. Nothing was released under 2.0 m. Estimator against the truth near 2.4 m: 0.02 m at p95 over 41 samples (limit 0.08). Height from the rangefinder within 0.01 m at p95.

## What the flights found

- **A log helper that rclpy refuses.** The first floor flight ended with `Mission 'package_drop' raised during update: Logger severity cannot be changed between calls.`: FINAL_ALIGN's throttled log helper chose `warn` or `info` on one line, and rclpy allows a line of code one severity. The mission failed, the sequence moved on and nothing was released. Fixed (two lines), with a test that holds the mission's logger to rclpy's rule on every FINAL_ALIGN path (`test_every_path_through_final_align_logs_as_rclpy_allows`).
- **Two checks of earlier features failed a flight that flew as designed**, and were corrected (both under Decisions):
  - Projection (FND-05): 0.44 m at p95 at 15 m in the first flight, from a dozen arrival frames in which the 5 m bullseye was half out of the picture. The check now judges frames in which, by the truth, the whole target was in view (0.14 to 0.15 m since), and counts the others apart (10 to 15 a flight, up to 2.5 m off).
  - Estimator (FND-06): "only 15 samples of a confirmed track between 2 and 2.8 m" in a flight that released half a second after reaching the height. The check no longer fails a band the vehicle only passed through, and stops counting at the mission's release.
- **A run made invalid by this session, not by the code.** The second floor flight was marked FAIL by the height check (rangefinder heights 0.10 m from the truth at p95, limit 0.1) while the full test suite was running on the same machine; the mission itself climbed back and released at 2.29 m. Flown again with the machine idle: 0.01 m at p95, run OK (the row above). The checks beside a flight compare each sample with the truth at the moment it arrives, so a loaded machine shows up as error: flights are not run beside the test suite.

## Not measured here

Wind. Gate G2 flies the drop in calm and strong wind over several seeds and has the pass lines for the release error.
