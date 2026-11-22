# The recovery mission in the sim (REC-2)

2026-11-22 (UTC). The `package_recovery` mission flown by the executor on the recovery vehicle (`iris_recovery`, 1.755 kg, a hook 0.55 m under it), a 1 kg bucket on the X mat 14.4 m from the takeoff point, calm air, sim time. One flight of each scenario, flown after the reviews' fixes; the earlier flights are described below. Two changes were made after these flights and do not touch the path they took: a landing is not asked for again once LAND has been seen, and the mission warns when no flight boundary is given.

## Commands

```
sim/run_scenarios.sh recovery/pickup_oracle --seeds 1 --variants on
sim/run_scenarios.sh recovery/pickup_detector --seeds 1 --variants on
```

Flags on in both: `use_frame_time_pose`, `use_target_estimator`, `use_height_estimator`, `package_recovery`. Numbers from each run's `result.json`, `score.json`, `payload_bridge.json` and `launch.log` (`sim/.state/suites/recovery-pickup_oracle-20261122T223453Z`, `recovery-pickup_detector-20261122T223919Z`).

## Results

| | `pickup_oracle`: the sim's oracle detector (a stand-in, sim only) | `pickup_detector`: the green X detector as built (REC-1) |
| --- | --- | --- |
| Status | OK, real-time factor 0.73 (sim time) | OK, 0.66 (sim time): the log lines the scenario expects were there |
| States | CHECKS, TRANSIT, ACQUIRE, APPROACH, CENTRE, CONE_DESCENT, ENGAGE, LIFT, VERIFY_ATTACH, CLIMB, RETURN, SET_DOWN, CLEAR, LANDING, COMPLETE | TRANSIT, ACQUIRE, APPROACH, CENTRE, then CONE_DESCENT and LOST_TARGET three times, then RETREAT (up and 2 m aside) and FAILED |
| Descents onto the handle | 1 | 3, each ended by the cone: "at 0.8 m and above with no detection to descend on" |
| Grab | taken at once: the hook 0.001 m sideways and 0.052 m in height from the handle (the bridge's measure) | none commanded |
| Judge (recovery, from true positions) | 19.993 points: lifted 4.41 m, let go standing on the ground (-0.02 m), the bucket at rest 0.0052 m from the takeoff point | 0 points: the bucket not lifted |
| Detector | the oracle reports the true bucket and X through the camera model | the green X detector, from the transit height down to about 3 m |
| End | landed 2 m beside the bucket, mission SUCCESS | mission FAILURE, "3 descents onto the handle did not take the bucket"; the executor's abort mode (LAND) |

## What it shows

- The mission flies the whole recovery when something sees the bucket all the way down: the cone, the engage with the gripper's acknowledgement, the attach check on the hover throttle, the return, the set-down by the throttle, the release through the guard, the clear and the landing.
- On the detector as built the chain from the camera to the cone works, and the mission does not descend blind: with no detection below about 3 m the cone holds, says the handle is lost, and after three tries the mission fails without having closed the gripper.

## The flights before these, and what they found

- `pickup_oracle`'s first flight (`recovery-pickup_oracle-20261121T034956Z`) flew the same walk and scored 19.992 points, and the runner marked it FAIL: three side checks that belong to other missions were run beside it (the bullseye's projection check and estimator check, and the survey pipeline's check). They are no longer asked for in a recovery flight.
- One flight of each scenario before the reviews' fixes (`recovery-pickup_oracle-20261122T220517Z`: OK, 19.991 points, 0.0072 m; `recovery-pickup_detector-20261122T220921Z`: OK, three lost descents, the X and the bucket in 898 of 1632 frames) ended the failed mission over the bucket, where the abort's landing would have come down on it. The mission now goes up and 2 m aside before it fails.

## Not shown

- A pickup on a real detector: REC-1 sees nothing below about 3 m, and the hand-off to the flow camera is not built.
- REC-8's runs R1 to R6 (wind, a refused grab, GPS faults, a search from wrong coordinates): not flown. The refused grab, the search, a refused release and the bucket lost on the way are unit-tested against a simulated world.
- `swing_shaping` and `recovery_gps_untrusted` on in a mission flight: unit-tested; flown only by the scripted check flight (REC-4, REC-7).
- The estimator's bucket track against the truth, as a check of its own: the grab 1 mm from the handle is what there is.
