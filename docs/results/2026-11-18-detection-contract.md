# The 2026 detector on the detection contract, in sim flights (FND-02)

Measured on 18 Nov 2026 on a club laptop, on `platform/build-2027` at `dbdf93a` plus the FND-02 change. Sim image `argus-sim:humble`.

**Result.** Two 2026 delivery flights on sim time with the detector's `features.detection_contract` on, both OK (0.06 m and 0.04 m from the target). Every message on `/perception/target_cv/detections` decoded under the contract and carried the stamp of a camera frame. The legacy adapter, fed only by that topic, gave the same target point as the 2026 detector's own topic for every one of them.

Flight 1 was on the code as first written, flight 2 on the code as committed (after the review fixes: the contract message is published after the 2026 topics, the adapter has its guards).

## The flights

The scenario is `delivery/time_base` with one more node flag. It is a scratch file and so is the probe: neither is committed, so this is a one-off measurement, and the committed proof is the unit tests (`test_contract_node.py` drives the real node classes with the flag on).

```
$ sim/run_scenarios.sh sim/.state/fnd02/contract.yaml --variants on        (twice)
| delivery/contract | 1 | not run |  |  | OK | 16.17 | 0.06 m from the target |
| delivery/contract | 1 | not run |  |  | OK | 16.19 | 0.04 m from the target |
```

`node_flags: target_cv: {stamp_from_image: true, detection_contract: true}`, `sim_time: true`. The time check and the rate check ran beside each flight as in every sim-time flight, and both counted (real-time factor 0.95 while armed).

Beside each flight, in the container, the adapter ran with its two outputs renamed, so that it did not publish beside the detector, and a probe listened to all three topics and to `/camera/camera_info`:

```
ros2 run drone_target_cv detection_legacy_adapter --ros-args -p detector:=target_cv -p use_sim_time:=true \
    -r /drone_package_drop/target_detection:=/probe/target_detection -r /drone_package_drop/image_size:=/probe/image_size
python3 sim/.state/fnd02/probe.py sim/.state/fnd02/probe2.json        (scratch; decodes with detection_contract.decode)
```

| | Flight 1 | Flight 2 |
| --- | --- | --- |
| Camera frames heard (640 x 360, frame id `downcam_link`) | 4813 | 4888 |
| Messages on `/perception/target_cv/detections` | 1944 | 1972 |
| Refused by `decode` | 0 | 0 |
| With the stamp of a camera frame | 1944 | 1972 |
| With a `bullseye` (the rest are empty: looked, nothing found) | 1865 | 1888 |
| Class ids seen | `bullseye` only | `bullseye` only |
| Score, lowest to highest | 0.42 to 0.94 | 0.42 to 0.94 |
| Box width, smallest to largest | 63 to 640 px | 57 to 640 px |
| Messages on the 2026 topic from the detector | 1945 | 1973 |
| Messages from the adapter | 1944 | 1972 |
| Same stamp on both | 1944 | 1972 |
| Found or not found agrees | 1944 | 1972 |
| Largest difference in x and in y (the -1 to 1 point) | 0.0 and 0.0 | 0.0 and 0.0 |
| Adapter's `z` over the detector's, median | 2.37 | 2.37 |

The detector's one extra message is the "not found" it sends when the mission switches it off, which has no image. The adapter's `z` is the box's area against the detector's red area; no mission reads `z`. In flight 2 the detector's log has no contract error line.

The detector published for 1944 and 1972 of the frames the probe heard. The mission switches it on only for the part of the flight over the target; whether it also skipped frames while it was on was not measured.

## What this does not show

- A detector on a shrunk frame: `scale_to_full_resolution` (1280 to 4K, and letterboxed 1280 x 736) is covered by unit tests only. The 2026 detector does not shrink its input.
- The 2026 missions flying on the adapter alone, with no 2026 detector: there is no contract-only detector yet (CEN-13, LOC-5), and the adapter does not serve the enable service the missions wait for.
- What the contract output costs per frame on the Jetson. It now runs after the 2026 topics are published.
- Anything on the aircraft.
