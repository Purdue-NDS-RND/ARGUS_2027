# PLT-03: Unified camera driver `argus_camera`

Flag: none. It is a new package whose node runs only when `argus_camera.launch.py` is started; nothing in the 2026 start-up uses it, and a test fails if anything outside the package refers to it. The 2026 topic names are the parameter `legacy_topics`, default false · Status: PARTIAL · Commits: `git log --oneline --grep '^PLT-03:' platform/build-2027`

## What it solves

2026 had two separate grabbers, a camera matrix scaled from another sensor mode, a projection matrix that did not match it, and three descriptions of where the camera is. This gives one driver with one interface to every camera, a calibration per sensor mode that is never scaled, frames published as the sensor gives them, and one robot description. It fixes the cause in new code; the 2026 grabbers stay as they are for the fallback.

## Architecture

New package `src/argus_camera` (ament_python). Everything but the node is free of ROS:

- `modes.py` and `config/modes.yaml`: `imx519_4k` (17 fps), `imx519_720p60`, `ov9782_flow`, `sim_delivery`; the IMX519 modes carry the exposure and white balance that flew in 2026.
- `calibration.py` and `config/calibration/*.yaml`: K and D per mode, and the `camera_info` fields for raw frames.
- `frame_source.py`: `FrameSource`, `CaptureSource` (CSI through a GStreamer pipeline, USB through V4L2), `SyntheticSource`, `open_source`. The caller passes the clock.
- `robot_description.py` and `config/robot.yaml`: camera mounts and the payload offset, per airframe; `get_airframe` refuses an unverified one unless asked.
- `frame_log.py`: the frame log core (JPEG quality 90 or PNG, an index, optional worker threads with a bounded queue).
- `publishing.py`: topic names, the 2026 aliases, their rate limit and the two guards.
- `bench.py` (`camera_bench`): the bench check for a camera in one mode.
- `camera_node.py` (`camera_node`) and `launch/argus_camera.launch.py`. Topics: `/argus/camera/<camera>/image_raw` and `camera_info`; with `legacy_topics:=true` also `/camera/image`, `/camera/image/compressed` and `/camera/camera_info` at 30 a second.

## Flow

Before: `mipi_grabber` reads 720p, scales the 4K camera matrix by the size ratio, stamps the frame when its publish timer fires, and publishes it with a projection matrix from a different calibration.

After, on the vehicle (`mode:=imx519_720p60`): `CaptureSource` opens the sensor in that mode, a thread reads each frame, and the node publishes it untouched with the mode's own K and D on its own topic, and on the 2026 names if asked. For 4K nothing is published: the localisation node embeds `FrameSource` and reads the same frames in its own process.

After, in the sim (`mode:=sim_delivery`): the node takes the bridged Gazebo image and republishes it with the same stamp and the sim camera's calibration.

## Code

- `frame_source.py`, `FrameSource.read`: a frame of the wrong size raises; nothing is resized. A camera that gives nothing returns None, never the previous frame.
- `calibration.py`, `for_mode`: a calibration file for another resolution is refused, and a mode with no file says so. `camera_info_fields`: identity R, P equal to K.
- `robot_description.py`, `CameraMount.rotation_body_from_optical`: the one place the camera's orientation becomes a matrix.
- `publishing.py`, `check_input_topic` and `other_publishers`: a relay may not read a topic the node publishes, and a second publisher on a 2026 name is noticed.
- `camera_node.py`, `_watch`: stops the node on a capture-thread error or a start-up conflict on the 2026 names. `_capture_loop` hands image data to rclpy as an array (plain bytes cost a third of a second per 720p frame).

## Evidence

```
$ PYTHONPATH=src/argus_camera python3 -m pytest -q -p no:cacheprovider src/argus_camera/test/test_argus_camera.py
70 passed in 0.95s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

In the sim container (ROS 2 Humble), from [docs/results/2026-11-17-argus-camera.md](../../results/2026-11-17-argus-camera.md):

```
mode:=sim_delivery (relay of the bridged Gazebo camera) with the 2026 detector on the node's topic:
    node 22.3 Hz, detections 23.2 Hz; 1 ms added delay; K matches Gazebo's to the sixth digit
mode:=imx519_720p60 synthetic:=true:  60.0 frames a second published
    with the 2026 names: /camera/image/compressed at 30.0 Hz, the 2026 detector reading it at 20.5 detections a second
legacy_topics:=true beside the sim's bridge:  FATAL ... another node also publishes on /camera/camera_info, /camera/image; exit 1
mode:=sim_delivery legacy_topics:=true:  TopicError: input_topic /camera/image is also a topic this node publishes
```

The library's tests were written first and failed on import. The bench tool and its tests were written together. The node was changed after each of its runs in ROS; its behaviour there is covered by the recorded runs and by checks on its source, not by unit tests.

## Spec check

spec-auditor, two rounds, INCOMPLETE both times. Round 1 found the 2026 aliases without `/camera/image/compressed`, a relay that could read its own output, a default input topic nothing publishes, a rate figure that measured a sleep, no test that nothing else starts the node, and a frame log that did not fit LOC-11. Round 2 found those closed and three small things, fixed after it and not reviewed again: the bench command now finds its config in an installed package, a stale figure in a person step, and a test that could pass on an empty search. What keeps the row from DONE is the bench: no camera opened on hardware, two modes not calibrated, the spare not listed, the airframe not measured.

flight-guardian, two rounds, CONCERNS both times, no blocker. Round 1 led to: the 2026 names off by default and exclusive, the 2026 exposure and white balance in the modes, the compressed alias at the 2026 rate, the relay guard, the watchdog on the capture thread, no default clock, and `get_airframe`. From round 2, done after it and not reviewed again: a second publisher on a 2026 name stops the node only in its first ten seconds and is reported after that; the alias rate limit can no longer pass more than about 33 a second; the node says when it is publishing a test pattern and when no frame has come for 5 s. Left standing:

- **A camera that stops giving frames leaves the node up**, as the 2026 grabber does; it only warns.
- **Differences on the 2026 names that no current 2026 code reads:** the frame id is `down_camera_optical` (2026: `camera_link`), the stamp is the driver read time (2026: publish time), the publisher keeps 5 messages (2026: 1), and the alias frames are published from the capture thread.
- **`robot_description.load` still returns unverified airframes**; the refusal is in `get_airframe`. FND-05 and CEN-01 must go through that.
- **The frame log writes in the caller unless given workers.** Whoever logs from a callback must pass `workers`.
- **After a hard kill and restart** the dead node's publishers may still be in the graph for a moment, and the start-up guard could then stop the new one. Not tried.
- **Nothing has run on real hardware.**

What differs from the spec, each recorded under Decisions: no flag; the 12 MP spare is not listed; two calibrations are not measured; in the sim the missions are not routed through the node.

## Not done

- **No real camera has been opened.** The CSI and V4L2 paths are tested with stand-in capture objects. Bench steps for the Jetson (both IMX519 modes, the OV9782) are under "Needs a person", which is why this row is PARTIAL.
- **Two modes are not calibrated** (720p60 is the 4K numbers divided by three and says so; the OV9782 has no file), and **the 12 MP spare is not in `modes.yaml`**: nothing here says which sensor it is.
- **The aircraft's entry in `robot.yaml` is not verified.** Its numbers are copied from the 2026 files, which disagree about which way up the image is.
- **The sim's missions do not run through the node.** `target_cv` still reads the bridge's `/camera/image`; SIM-05's camera profiles are where that changes.
- **Capture time is the clock when the frame leaves the driver.** FND-01 is next and replaces it.
- **Nothing reads `robot.yaml` or the frame log yet**; FND-05, CEN-01 and LOC-11 do.
