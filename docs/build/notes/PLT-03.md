# PLT-03: Unified camera driver `argus_camera`

Flag: none. It is a new package whose node runs only when `argus_camera.launch.py` is started; nothing in the 2026 start-up uses it · Status: in progress (DOING) · Commits: `git log --oneline --grep '^PLT-03:' platform/build-2027`

## What it solves

2026 had two separate grabbers, a camera matrix scaled from another sensor mode, a projection matrix that did not match it, and three descriptions of where the camera is. This gives one driver with one interface to every camera, a calibration per sensor mode that is never scaled, frames published as the sensor gives them, and one robot description. It fixes the cause in new code; the 2026 grabbers stay as they are for the fallback.

## Architecture

New package `src/argus_camera` (ament_python). Everything but the node is free of ROS:

- `modes.py` and `config/modes.yaml`: `imx519_4k` (17 fps), `imx519_720p60`, `ov9782_flow`, `sim_delivery`.
- `calibration.py` and `config/calibration/*.yaml`: K and D per mode, and the `camera_info` fields for raw frames.
- `frame_source.py`: `FrameSource`, `CaptureSource` (CSI through a GStreamer pipeline, USB through V4L2), `SyntheticSource`, `open_source`.
- `robot_description.py` and `config/robot.yaml`: camera mounts and the payload offset, per airframe.
- `frame_log.py`: the frame log core (lossless frames and an index).
- `bench.py` (`camera_bench`): the bench check for a camera in one mode.
- `camera_node.py` (`camera_node`) and `launch/argus_camera.launch.py`. Topics: `/argus/camera/<camera>/image_raw` and `camera_info`, plus the 2026 names `/camera/image` and `/camera/camera_info` as aliases.

## Flow

Before: `mipi_grabber` reads 720p, scales the 4K camera matrix by the size ratio, stamps the frame when its publish timer fires, and publishes it with a projection matrix from a different calibration.

After, on the vehicle (`mode:=imx519_720p60`): `CaptureSource` opens the sensor in that mode, a thread reads each frame, and the node publishes it untouched with the mode's own K and D, on the new topic and on `/camera/image`. For 4K nothing is published: the localisation node embeds `FrameSource` and reads the same frames in its own process.

After, in the sim (`mode:=sim_delivery`): the node takes the bridged Gazebo image and republishes it with the same stamp and the sim camera's calibration.

## Code

- `frame_source.py:58`, `FrameSource.read`: a frame of the wrong size raises; nothing is resized. A camera that gives nothing returns None, never the previous frame.
- `calibration.py:63`, `for_mode`: a calibration file for another resolution is refused, and a mode with no file says so.
- `calibration.py:76`, `camera_info_fields`: identity R, P equal to K: raw frames.
- `robot_description.py:41`, `CameraMount.rotation_body_from_optical`: the one place the camera's orientation becomes a matrix.
- `camera_node.py`: refuses `in_process_only` modes (4K, the flow camera); hands image data to rclpy as an array (plain bytes cost a third of a second per 720p frame).

## Evidence

```
$ PYTHONPATH=src/argus_camera python3 -m pytest -q -p no:cacheprovider src/argus_camera/test/test_argus_camera.py
48 passed in 0.87s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

In the sim container (ROS 2 Humble), from [docs/results/2026-11-17-argus-camera.md](../../results/2026-11-17-argus-camera.md):

```
mode:=sim_delivery on the bridged Gazebo camera:  in 25.5 Hz, out 23.1 Hz; delay from the image's stamp 9 ms in, 10 ms out
    k: [389.16536, 0, 320, 0, 389.16536, 180, 0, 0, 1]        Gazebo's own: [389.16534, 0, 320, 0, 389.16540, 180, 0, 0, 1]
mode:=imx519_720p60 synthetic:=true:  1280 x 720 bgr8, frame_id down_camera_optical, 48 to 52 frames a second published
    /camera/image publishers: argus_camera, ros_gz_bridge
camera_bench --mode imx519_4k --seconds 3 --synthetic:  3840 x 2160, 15.8 fps (mode 17), camera bench: PASS
```

The library's tests were written first and failed on import. The bench tool and its tests were written together, and the node was changed three times after its first run in ROS (unclean shutdown, a rate log, the array fix); those changes have no unit test beyond the checks on the node's source.

## Spec check

**Round one is open: these findings have not been acted on yet.** The session was paused on 17 Nov 2026 straight after both reviews came back.

spec-auditor, round 1: INCOMPLETE. To fix in code:

1. The 2026 aliases leave out `/camera/image/compressed`, which is what the shipped live params make `target_cv` and the session logger read. Publish it, or record that the alias covers raw-image configs only and correct the README. Record a run of `target_cv` on the node's output.
2. In a `ros_topic` mode the node can subscribe to its own alias: with `input_topic:=/camera/image` and `legacy_topics` left true it relays its own output. The default `input_topic` (`/sim/camera/image`) is published by nothing. Refuse an input topic the node also publishes on.
3. The 48 to 52 frames a second figure measures the test pattern's pacing sleep, not the node. Pace it to a deadline and measure again.
4. Add an OV9782 capture step (and a check of its device path) to "Needs a person".
5. Add a test that nothing outside `src/argus_camera` starts the node.
6. Note: LOC-11's spec asks for JPEG quality 90 on worker threads; the frame log core writes PNG in the caller. Say so in the Decisions row, or change the core.

flight-guardian, round 1: CONCERNS, no blocker:

- `legacy_topics` defaults to true, so starting the node beside a 2026 grabber or the sim bridge puts a second publisher on the detector's input. Default it to false, or refuse when another publisher is there.
- The CSI pipeline drops the fixed exposure and white balance that flew in 2026 (`shutter_speed`, `wb_mode` in the live params) and publishes at 60 Hz where 2026 published at 30. Give the modes exposure and white-balance fields before claiming the 2026 detector runs on it unchanged.
- No `/camera/image/compressed` (same as the audit's first item).
- The relay loop (same as the audit's second item).
- An error in the capture thread ends the thread while the node stays up printing "0.0 frames a second". Log it and stop the node.
- The library's default clock is wall time; only the node passes the ROS clock. An in-process user on sim time would stamp frames on the wrong clock. Make the clock a required argument or fix the README example.
- `verified: false` in `robot.yaml` is only data; the loader should make a consumer ask for an unverified airframe on purpose.

## Not done

- **No real camera has been opened.** The CSI and V4L2 paths are tested with a stand-in capture object. Bench steps for the Jetson are under "Needs a person", which is why this row is PARTIAL.
- **Two modes are not calibrated** (720p60 is the 4K numbers divided by three and says so; the OV9782 has no file), and **the 12 MP spare is not in `modes.yaml`**: nothing here says which sensor it is.
- **The aircraft's entry in `robot.yaml` is not verified.** Its numbers are copied from the 2026 files, which disagree about which way up the image is.
- **The sim's missions do not run through the node.** `target_cv` still reads the bridge's `/camera/image`; SIM-05's camera profiles are where that changes.
- **Capture time is the clock when the frame leaves the driver.** FND-01 is next and replaces it.
- **Nothing reads `robot.yaml` or the frame log yet**; FND-05, CEN-01 and LOC-11 do.
