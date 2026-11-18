# Capture time in the sim: frame and detection stamps (FND-01)

Measured on 18 Nov 2026 on a club laptop, on `platform/build-2027` at `396c133` plus the FND-01 change. Sim image `argus-sim:humble`. No real camera: the PTS and kernel-timestamp paths are for the Jetson bench.

**Result: PASS for the sim line.** Frames are stamped within 2 to 3 ms of the sim clock at the 95th percentile, against a limit of one frame period (33.3 ms), and with the detector's `features.stamp_from_image` on, every detection but one carries the stamp of a frame.

```
$ sim/run_scenarios.sh delivery/time_base --variants on          (twice)
   OK          delivery/time_base seed 1 on: 16.21 points, 0.03 m from the target
   OK          delivery/time_base seed 1 on: 16.17 points, 0.06 m from the target
```

From each run's `time_check.json` (`sim/time_check.py` listens for the whole flight; the sim clock is Gazebo's own):

| | Flight 1 | Flight 2 |
| --- | --- | --- |
| Frames heard (`/camera/camera_info`, one per frame) | 4867 | 4868 |
| Sim clock minus frame stamp: median; 95th percentile; maximum | +1.0 ms; 2.0 ms; 27.0 ms | +1.0 ms; 3.0 ms; 25.0 ms |
| Detections heard | 2003 | 1954 |
| Detections whose stamp is a frame's stamp | 2002 | 1953 |
| Age of a detection's frame when the detection is heard: median; 95th percentile; maximum | 40 ms; 275 ms; 430 ms | 40 ms; 259 ms; 432 ms |
| Time check | PASS | PASS |

- The stamp on a sim frame is Gazebo's render time, passed through the bridge; the 1 to 3 ms is the delivery to the checker.
- The one detection in each flight without a frame stamp is the single "not found" the detector sends when it is switched off at the end of the mission, which has no image and is stamped with the clock by design.
- With the flag off the same check is not applied: 2026 stamps a detection with the detector's own clock, about 2 ms before it is heard, whatever the age of the frame behind it.

## What the frame age says about the stale-data guard

The last row is the number the guard (FND-03) compares with 250 ms. Half the time the frame is 40 ms old when the mission can use its detection. But one detection in twenty is over a quarter of a second old, and the worst was 430 ms. That is why the guard, flown on the 2026 delivery, tripped 17 times and the delivery never got down (docs/results/2026-11-17-mavros-rates.md).

Where that tail comes from has not been established. It is not the camera (frames arrive within 3 ms of their stamp) and not a steady cost per frame (the median is 40 ms). It is in or after the 2026 detector. The guard stays off in `delivery/time_base` until the tail is found and removed.

## Commands

```
sim/run_scenarios.sh delivery/time_base --variants on
python3 -c "import json; print(json.load(open('<run folder>/time_check.json'))['frames'])"
```

Run folders (not committed): `sim/.state/suites/delivery-time_base-20261118T012528Z/`, `-20261118T013221Z/`.
