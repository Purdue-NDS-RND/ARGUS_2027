# argus_camera

One camera driver for the 2027 stack (PLT-03): a library, `FrameSource`, and a node. It is not part of the 2026 start-up; the 2026 grabbers in `drone_target_cv` are untouched and still what flies with everything off.

## What is in it

| Piece | Where |
| --- | --- |
| Sensor modes: which sensor, driver, size, rate and calibration file | `config/modes.yaml`, `argus_camera/modes.py` |
| Calibration per mode, and the `camera_info` for raw frames | `config/calibration/`, `argus_camera/calibration.py` |
| `FrameSource`: frames from a CSI or USB camera, or a test pattern | `argus_camera/frame_source.py` |
| The one robot description: camera mounts and the payload offset | `config/robot.yaml`, `argus_camera/robot_description.py` |
| Frame log core: JPEG or PNG frames and an index on disk, on worker threads if asked | `argus_camera/frame_log.py` |
| Topic names, the 2026 aliases and their guards | `argus_camera/publishing.py` |
| The node | `argus_camera/camera_node.py`, `launch/argus_camera.launch.py` |
| Bench check for a camera in one mode | `argus_camera/bench.py` (`camera_bench`) |

Everything except the node is free of ROS and is tested by `scripts/check.sh`.

## Sensor modes

| Mode | Sensor | Size and rate | For | Calibration |
| --- | --- | --- | --- | --- |
| `imx519_4k` | IMX519 (CSI) | 3840 x 2160, 17 fps | Localisation; stays in one process | Measured (the 2026 one) |
| `imx519_720p60` | IMX519 (CSI) | 1280 x 720, 60 fps | Drop and delivery, as topics | Not measured: the 4K numbers divided by three |
| `ov9782_flow` | OV9782 (USB) | 1280 x 800, 120 fps | Optical flow, in-process | None yet |
| `sim_delivery` | The sim's Gazebo camera | 640 x 360, 30 fps | The sim | Computed from the sim model |

- A calibration is measured in its own mode. A file for another size is refused, never scaled.
- `imx519_4k` and `ov9782_flow` are `in_process_only`: the node refuses them, because 4K frames and 120 fps do not go over ROS topics. The process that needs them embeds `FrameSource`.
- A mode with no calibration cannot be published by the node. A mode whose calibration is not measured runs with a warning.
- The 12 MP spare is not listed: its sensor and modes are not known here. It is one more entry in `modes.yaml` plus its calibration file.

## Raw frames

The driver publishes frames as the sensor gives them. `camera_info` carries the mode's own K and D, an identity R, and P equal to K. Nothing is resized, undistorted or remapped on the vehicle; projection undistorts the few points it needs. In 2026 the published projection matrix was a different camera matrix from K, and the K was scaled from another mode.

## Using it

In another process, with no ROS hop (the 4K path):

```python
from argus_camera.modes import load_modes, get_mode
from argus_camera.frame_source import open_source

mode = get_mode(load_modes("config/modes.yaml"), "imx519_4k")
with open_source(mode, clock=lambda: node.get_clock().now().nanoseconds) as source:
    frame = source.read()      # Frame(image, seq, capture_ns, mode), or None
```

`clock` has no default. It must be the clock every other stamp in the process uses, the ROS clock in a node, or frames end up on a different clock from the poses when the sim runs on sim time.

As topics:

```bash
ros2 launch argus_camera argus_camera.launch.py mode:=imx519_720p60
ros2 launch argus_camera argus_camera.launch.py mode:=imx519_720p60 legacy_topics:=true  # and the 2026 topic names
ros2 launch argus_camera argus_camera.launch.py mode:=imx519_720p60 synthetic:=true      # test pattern, no camera
ros2 launch argus_camera argus_camera.launch.py mode:=sim_delivery use_sim_time:=true    # in the sim: relays the bridge's /camera/image
```

The node publishes `/argus/camera/<camera>/image_raw` and `/argus/camera/<camera>/camera_info`. The frame id is the camera's optical frame from `robot.yaml`.

With `legacy_topics:=true` it also serves the three names the 2026 grabber publishes, so the 2026 detector and session logger can run on it: `/camera/image`, `/camera/image/compressed` (JPEG, quality 20) and `/camera/camera_info`, thinned to the 2026 rate of 30 a second. This is off by default, and the node stops with a FATAL line if anything else publishes on those names: it replaces the 2026 grabber, it does not run beside it. What is the same as 2026 on those names: the topics, the rate, the JPEG quality, and the IMX519's exposure (1/1000 s) and white balance, which are in `modes.yaml`. What differs: the frames carry the mode's own calibration and frame id.

An error in the capture thread stops the node, and so does a second publisher on a 2026 name in its first ten seconds (a set-up mistake). A second publisher that appears later is reported every 5 s but does not stop the only camera. A camera that simply stops giving frames leaves the node up, as with the 2026 grabber; it says `no frame from the camera in the last 5 s`. In a relay mode the input topic may not be one the node publishes. With `synthetic:=true` the node says it is publishing a test pattern; the frames themselves carry no mark.

## Capture time (FND-01)

A frame's `capture_ns`, and the stamp of the image the node publishes, is when the frame was taken, on the clock the source was given (`capture_clock.py`):

| Camera | Where the time comes from | `stamp_source` |
| --- | --- | --- |
| IMX519 on CSI with `csi_backend:=gstreamer` (`GstCsiSource`, GStreamer's own bindings) | The buffer's PTS plus the pipeline's base time, moved from the pipeline's clock to ours, minus the mode's `capture_offset_s` | `pts` |
| OV9782 on USB | The kernel's time for the buffer (monotonic clock), moved to ours | `kernel` |
| IMX519 with `csi_backend:=opencv` (the default), the test pattern | The clock when the frame came out of the driver | `read` |
| The sim's camera (relay) | Gazebo's render stamp, passed through | (the input's own) |

- `nvarguscamerasrc` does not pass the sensor's own timestamp on, so the PTS is when the frame entered the pipeline. The fixed delay before that is `capture_offset_s` in `modes.yaml`, 0 until the LED probe (PLT-05) measures it.
- A timestamp that is missing or from the future is not used: the frame is stamped at the read and the fallback is counted. A frame that is over a second old by its own timestamp (the pipeline stalled) is dropped, not restamped as new. The node's rate line says where its stamps come from and counts both.
- How the CSI camera is read is a choice, `csi_backend`, never a guess from what is installed. The default, `opencv`, is the capture PLT-03 was built on and stamps a frame when it is read. `gstreamer` stamps from the PTS and needs GStreamer's Python bindings (`python3-gi`, `gir1.2-gstreamer-1.0`); without them it is an error. It has only run against stand-ins: the bench step under "Needs a person" is what makes it the one to fly.
- A pipeline that does not start, or reports an error later, raises, and the node stops.
- Downstream: with its `features.stamp_from_image` flag on, the 2026 detector copies the image's stamp onto everything it makes from a frame; the YOLO node already does.

## The robot description

`config/robot.yaml` is the only place 2027 code reads a camera mount or the payload offset from, through `robot_description.get_airframe`, which refuses an airframe that is not verified unless the caller passes `allow_unverified=True`. Body frame: x forward, y left, z up, origin at the flight controller. Each airframe says whether anyone has measured it (`verified`). The aircraft's entry is not verified: its numbers are copied from the 2026 files, which disagree with each other about which way up the image is.

## Bench check (on the Jetson, camera fitted)

```bash
ros2 run argus_camera camera_bench --mode imx519_720p60 --seconds 10
ros2 run argus_camera camera_bench --mode imx519_4k --seconds 10
```

It passes when the camera gives frames of the mode's size at 90% of the mode's rate or more, with no gap over three frame periods, and the mode's calibration is measured. `--synthetic` tries the tool without a camera.
