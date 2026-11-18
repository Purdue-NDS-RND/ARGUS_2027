# FND-02: Detection contract

Flag: `features.detection_contract` (node flag on `target_cv`, default false, declared in the five params files) · Status: DONE · Commits: `git log --oneline --grep '^FND-02:' platform/build-2027`

## What it solves

In 2026 each detector had its own output: the bullseye detector a point from -1 to 1, the YOLO node boxes with numeric classes. Everything built in 2027 (the estimator, the ring fit, the green X, the localiser) needs one form. This defines it once, in code, and puts the 2026 detector on it behind a flag. It is the cause-level fix: one module both sides import.

## Architecture

- `drone_mission_core/detection_contract.py` (new, no ROS): the class list, `topic_for(detector)`, `Detection` and `DetectionSet`, `encode` and `decode` to and from `vision_msgs/Detection2DArray`, `scale_to_full_resolution`, `best_of_class`, `to_legacy_point`.
- `drone_target_cv/contract_output.py` (new, no ROS): the 2026 detector's result as a `bullseye` detection, and the two 2026 messages from a contract set.
- `target_cv`: with the flag on, one more publisher, `/perception/target_cv/detections`. The 2026 topics are still published by the node itself.
- `drone_target_cv/legacy_adapter.py` (new node, `detection_legacy_adapter`): contract topic of a named detector to `/drone_package_drop/target_detection` and `/drone_package_drop/image_size`. Nothing shipped starts it, and it stays silent beside another publisher of the 2026 topic.
- New dependencies of `drone_target_cv`: `vision_msgs` and `drone_mission_core`, imported only with the flag on (or by the adapter).

## Flow

Before, and after with the flag off: frame → `target_cv` → a point on `/drone_package_drop/target_detection` → mission.

After, flag on: the same, and also frame → `target_cv` → `Detection2DArray` on `/perception/target_cv/detections` with the frame's stamp and frame id: one `bullseye` with the detector's confidence, centre and box in pixels, or an empty array when it found nothing.

For a detector that only speaks the contract: its topic → `detection_legacy_adapter` → the two 2026 topics → the 2026 mission. That detector also has to serve the enable service the missions wait for.

## Code

- `detection_contract.py:83` and `:110`, `encode` and `decode`: both check the set (class in the list, score from 0 to 1, a box with a size, a frame id, a whole-nanosecond stamp), so a message from outside the contract is refused where it is read, with the reason.
- `detection_contract.py:136`, `scale_to_full_resolution`: from the size a detector ran on to the full image, plain or letterboxed (1280 x 736 for 16:9); two sizes of different shape without `letterbox` are refused.
- `contract_output.py:12`, `bullseye_detection`: centre from the detector, box from the red pixels of the target it chose, score clamped to 0 to 1.
- `target_cv.py`, `_start_contract_output` and `_publish_on_contract`: both catch their own failures, and the contract message goes out after the 2026 topics, so the flag cannot stop or delay what the missions fly on.
- `legacy_adapter.py`, `_on_detections`, with `contract_output.legacy_messages` and `stamp_is_usable`: decode, take the most confident `bullseye` that is at least 0.25 and inside the image, publish the 2026 point with the contract's stamp. Nothing until `camera_info` has given the image size, nothing for a stamp more than 0.1 s ahead of the clock.

## Evidence

```
$ PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_detection_contract.py
19 passed in 0.01s
$ PYTHONPATH=src/drone_target_cv python3 -m pytest -q -p no:cacheprovider src/drone_target_cv/test/test_contract_output.py src/drone_target_cv/test/test_contract_node.py
19 passed in 0.54s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

The spec's two unit tests are `test_encode_then_decode_gives_the_same_detections_back` and `test_a_detector_that_ran_on_a_1280_frame_scales_back_up_to_4k`. The contract tests were written first and failed on the missing module. `test_contract_node.py` builds the real `TargetCV` and `LegacyAdapter` classes on stand-ins for rclpy, with the flag off and on, and runs the real detector on a drawn bullseye; its `vision_msgs` stand-ins have only the real field names.

Sim, flag on, two flights, the second on the committed code ([docs/results/2026-11-18-detection-contract.md](../../results/2026-11-18-detection-contract.md)):

```
$ sim/run_scenarios.sh sim/.state/fnd02/contract.yaml --variants on
| delivery/contract | 1 | not run |  |  | OK | 16.19 | 0.04 m from the target |
```

1972 contract messages, none refused by `decode`, all with a camera frame's stamp; the adapter's point equal to the detector's own 2026 point for all 1972 (largest difference 0.0). The scenario and the probe are scratch files, not committed.

## Spec check

spec-auditor, one round: INCOMPLETE on three rows. Fixed after it, not reviewed again: a committed test that turns the flag on and drives the real node (`test_contract_node.py`); the adapter's `detector` has no default and it stays silent beside another publisher; a Decisions row that the 2026 YOLO node stays off the contract until LOC-5.

flight-guardian, one round: CONCERNS, no blocker. Fixed after it, not reviewed again: the flag-on set-up can no longer stop the node (it falls back to the 2026 detector and says so); the contract message goes out after the 2026 topics; the adapter has a score floor (0.25, the 2026 detector's own) and an inside-the-image check, and does not pass on a stamp from the future. Left standing:

- **The adapter does not serve the enable service** the 2026 missions wait for. A contract-only detector has to, or the mission stays in INIT.
- **The frame id is the image's, not checked to be an optical frame** (`downcam_link` in the sim, `camera_link` from the 2026 grabbers). `decode` accepts any non-empty frame id.
- **Whether `vision_msgs` is on the aircraft's image was not checked.** If it is not, the flag-on node says so and runs as the 2026 detector.

What differs from the spec, each recorded under Decisions: the 2026 topic stays alive because the 2026 detector still publishes it, and the adapter is for contract-only detectors; pixels and frame id are the incoming image's; the YOLO node is left as it is.

## Not done

- **No detector publishes only on the contract yet**, so the 2026 missions have not flown on the adapter alone. CEN-13 and LOC-5 bring the first ones.
- **The YOLO node is not on the contract.** It publishes `Detection2DArray` with its model's numeric classes on its own topic; LOC-5 replaces it and DAT-01 names the classes.
- **"Full-resolution calibrated image" and "optical frame" depend on the driver.** The 2026 detector reports in the pixels and frame id of the image it is given: right with `argus_camera`, not with the 2026 grabbers (`camera_link`, a reduced image). PLT-07 makes `argus_camera` the driver.
- **Nothing reads the contract in flight yet.** FND-06's estimator is the first reader.
