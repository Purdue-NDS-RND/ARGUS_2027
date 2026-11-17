# argus_camera

One camera driver for the 2027 stack (PLT-03): a library, `FrameSource`, and a node. It is not part of the 2026 start-up; the 2026 grabbers in `drone_target_cv` are untouched and still what flies with everything off.

## What is in it

| Piece | Where |
| --- | --- |
| Sensor modes: which sensor, driver, size, rate and calibration file | `config/modes.yaml`, `argus_camera/modes.py` |
| Calibration per mode, and the `camera_info` for raw frames | `config/calibration/`, `argus_camera/calibration.py` |
| `FrameSource`: frames from a CSI or USB camera, or a test pattern | `argus_camera/frame_source.py` |
| The one robot description: camera mounts and the payload offset | `config/robot.yaml`, `argus_camera/robot_description.py` |
| Frame log core: lossless frames and an index on disk | `argus_camera/frame_log.py` |
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
with open_source(mode) as source:
    frame = source.read()      # Frame(image, seq, capture_ns, mode), or None
```

As topics:

```bash
ros2 launch argus_camera argus_camera.launch.py mode:=imx519_720p60
ros2 launch argus_camera argus_camera.launch.py mode:=imx519_720p60 synthetic:=true      # test pattern, no camera
ros2 launch argus_camera argus_camera.launch.py mode:=sim_delivery input_topic:=/camera/image legacy_topics:=false use_sim_time:=true   # in the sim
```

The node publishes `/argus/camera/<camera>/image_raw` and `/argus/camera/<camera>/camera_info`, and also the 2026 names `/camera/image` and `/camera/camera_info` unless `legacy_topics:=false`, so `target_cv` and the 2026 missions run on it unchanged. The frame id is the camera's optical frame from `robot.yaml`.

`capture_ns` and the image stamp are, for now, the clock when the frame came out of the driver (in the sim: Gazebo's own render stamp). FND-01 replaces that with the sensor's capture time.

## The robot description

`config/robot.yaml` is the only place 2027 code reads a camera mount or the payload offset from. Body frame: x forward, y left, z up, origin at the flight controller. Each airframe says whether anyone has measured it (`verified`). The aircraft's entry is not verified: its numbers are copied from the 2026 files, which disagree with each other about which way up the image is.

## Bench check (on the Jetson, camera fitted)

```bash
ros2 run argus_camera camera_bench --mode imx519_720p60 --seconds 10
ros2 run argus_camera camera_bench --mode imx519_4k --seconds 10
```

It passes when the camera gives frames of the mode's size at 90% of the mode's rate or more, with no gap over three frame periods, and the mode's calibration is measured. `--synthetic` tries the tool without a camera.
