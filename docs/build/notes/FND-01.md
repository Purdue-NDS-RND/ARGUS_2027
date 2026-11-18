# FND-01: Capture timestamps on every frame

Flag: `features.stamp_from_image` on the detector `target_cv` (node flag, default false) for what is made from a frame. The capture clock itself is part of `argus_camera`, which runs only when its own launch is started · Status: PARTIAL · Commits: `git log --oneline --grep '^FND-01:' platform/build-2027`

## What it solves

In 2026 a frame was stamped when its publish timer fired and a detection when the detector finished, so the 40 to 120 ms between the shutter and the result was invisible and the vehicle steered on a stale picture as if it were current. Now a frame carries the time it was taken, and what is made from it carries the same stamp. It fixes the cause in the new driver and, behind the flag, in the detector.

## Architecture

- `argus_camera/capture_clock.py` (new, no ROS): `stamp_from_pts`, `stamp_from_kernel`, `CaptureClock` (sanity check, fallback, count).
- `argus_camera/frame_source.py`: `GstCsiSource` (new: the CSI camera through GStreamer's own bindings, stamped from the buffer's PTS), chosen with `csi_backend:=gstreamer`; `CaptureSource` stamps a USB frame from the kernel's buffer time; `Frame.stamp_source`.
- `modes.yaml`: `capture_offset_s` per mode (0 until PLT-05 measures it).
- `target_cv.py`: the image-size message now follows the same stamp rule as the detections (the detections did since FND-03).
- `sim/time_check.py`: a frame section, run beside every sim-time flight.

No new node or topic. New optional dependency on the Jetson: GStreamer's Python bindings.

## Flow

Before: IMX519 → `mipi_grabber` reads the newest frame on a 30 Hz timer and stamps it then → `target_cv` detects and stamps the detection when it publishes.

After (the new driver, `stamp_from_image` on): the frame leaves the sensor; its buffer has a PTS; `GstCsiSource` reads it, takes the pipeline clock and the ROS clock at that moment, and stamps the frame `ROS now − (pipeline now − (base time + PTS)) − capture_offset`. The node publishes it with that stamp. `target_cv` copies the stamp onto the detection. The executor can then ask the pose buffer (FND-04) where the vehicle was at that instant.

In the sim the stamp is Gazebo's render time, already on the frame; the relay and the detector pass it on.

## Code

- `capture_clock.py`, `CaptureClock._checked`: a timestamp more than 2 ms in the future is not used (the frame is stamped at the read, marked `read`, counted); a frame more than 1 s old by its own timestamp is dropped and counted.
- `frame_source.py`, `GstCsiSource._grab`: pulls a sample from the appsink, copies the pixels, and stamps from `buffer.pts`, `pipeline.get_base_time()` and the pipeline clock.
- `frame_source.py`, `CaptureSource._grab`: for V4L2, the kernel's buffer time through `CAP_PROP_POS_MSEC`; for CSI through OpenCV there is no capture time, and it says so.
- `frame_source.py`, `load_gst` and `open_source`: the bindings are imported inside a guard; `csi_backend` picks the path and `gstreamer` without the bindings is an error. `GstCsiSource.open` waits for the pipeline to reach PLAYING and `_grab` reads the bus, so a camera that is not there, or fails later, raises.
- `sim/time_check.py`, `FrameStamps` and `frame_problems`: frame stamps against the sim clock, and whether detections carry a frame's stamp.

## Evidence

```
$ PYTHONPATH=src/argus_camera python3 -m pytest -q -p no:cacheprovider src/argus_camera/test/test_capture_clock.py
30 passed in 0.24s
$ PYTHONPATH=src/drone_target_cv python3 -m pytest -q -p no:cacheprovider src/drone_target_cv/test/test_detection_stamp.py
7 passed in 0.02s
$ python3 -m pytest -q -p no:cacheprovider sim/test/test_time_base.py
59 passed in 0.26s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh delivery/time_base --variants on        (twice)
   OK          delivery/time_base seed 1 on: 16.21 points, 0.03 m from the target
frame stamps against the sim clock: 4867 frames, median +1.0 ms, p95 2.0 ms
detections: 2003, 2002 with a frame's stamp; age when heard median 40 ms, p95 275 ms
time check: PASS
```

Unit line of the spec: `test_synthetic_60_fps_pts_come_out_evenly_spaced_whatever_the_read_jitter` and the conversion cases. Sim line: frames within 2 to 3 ms of the sim clock at the 95th percentile against one frame period (33.3 ms). Numbers in [docs/results/2026-11-18-capture-time.md](../../results/2026-11-18-capture-time.md). The capture clock's tests were written first and failed on import; the frame section of the time check and its tests were written together.

## Spec check

spec-auditor, one round: INCOMPLETE on three rows. Fixed after it, not reviewed again: tests that a `capture_offset_s` written in `modes.yaml` reaches the stamp of a CSI and of a USB frame, and that a bad value is refused. Recorded under Decisions: on the aircraft a detection carries a capture time only once `argus_camera` is the driver, which no flight launch does yet (PLT-07). The third row is the bench line, owed to PLT-05's probe and the Jetson, which is why this is PARTIAL.

flight-guardian, one round: CONCERNS, no blocker. Fixed after it, not reviewed again: a frame over a second old by its own timestamp is dropped instead of being restamped as new; the offset cannot be negative; the GStreamer path is an explicit choice (`csi_backend`), not a consequence of what is installed; a pipeline that fails to start or errors later raises. Left standing:

- **Whether `base time + PTS` is the capture moment for `nvarguscamerasrc` has not been seen on hardware.** A constant error under a second would pass every check. The bench tool now prints each frame's age at the read so a person can see it; the LED probe settles it.
- **A read-stamped frame is not marked on the message.** `stamp_source` is in the `Frame` and in the node's log line, not in the image header.
- **The sim result says nothing about `capture_clock.py`.** The sim's frames come from Gazebo's bridge with Gazebo's stamps; the sim line proves only that nothing in between restamps them and that the detector copies them.

What differs from the spec, each recorded under Decisions: the module is in `argus_camera`, not `cv/`; the offset is per sensor mode; the CSI capture-time path is opt-in until the bench has seen it.

## Not done

- **Bench line, which is why this row is PARTIAL:** the LED probe residual of 5 ms or less needs PLT-05's probe and the camera. `capture_offset_s` is 0 until then.
- **The PTS path has never run on a real pipeline.** It is tested with stand-ins for GStreamer; the Jetson needs `python3-gi` and `gir1.2-gstreamer-1.0` installed, and the node started with `csi_backend:=gstreamer`. By default the CSI camera is stamped at the read.
- **The 2026 grabbers still stamp at publish.** They are the fallback and are not changed; capture time comes with the new driver.
- **The frame is often old by the time the mission hears of it:** median 40 ms, but 259 to 275 ms at the 95th percentile in the sim. FND-01 makes that visible; it does not make it shorter. The stale-data guard stays off in `delivery/time_base` because of it.
