# argus_camera in the sim container (PLT-03)

Measured on 17 Nov 2026 on a club laptop, on `platform/build-2027` at `9f7d171` plus the PLT-03 change, inside the sim container (`argus-sim:humble`, ROS 2 Humble). No real camera was involved: the IMX519 and OV9782 paths are for the Jetson bench.

## The sim's camera through the node

Sim started with `SIM_TIME=1 HEADLESS=1 sim/run_sim.sh --detach`, vehicle on the ground, then:

```
ros2 launch argus_camera argus_camera.launch.py mode:=sim_delivery input_topic:=/camera/image legacy_topics:=false use_sim_time:=true
ros2 topic hz    /camera/image                    --use-sim-time     ->  average rate: 25.473
ros2 topic hz    /argus/camera/down/image_raw     --use-sim-time     ->  average rate: 23.136
ros2 topic delay /camera/image                    --use-sim-time     ->  average delay: 0.009   (min 0.006, max 0.014)
ros2 topic delay /argus/camera/down/image_raw     --use-sim-time     ->  average delay: 0.010   (min 0.007, max 0.015)
ros2 topic echo --once /argus/camera/down/camera_info
    frame_id: down_camera_optical   640 x 360   d: all 0
    k: [389.16536, 0, 320, 0, 389.16536, 180, 0, 0, 1]      p: [389.16536, 0, 320, 0,  0, 389.16536, 180, 0,  0, 0, 1, 0]
ros2 topic echo --once --field k /camera/camera_info        (Gazebo's own)
    [389.16534424, 0, 320, 0, 389.16539669, 180, 0, 0, 1]
```

- The node relays the bridged Gazebo camera with the image's own stamp (the delay is measured from that stamp), adding about 1 ms. The two rates were measured one after the other, 12 s each, so the 2 Hz between them is not a loss figure.
- The mode's calibration, computed from the sim model's field of view, matches the matrix Gazebo publishes to the sixth digit.
- When a 1280 x 720 frame appeared on its input (another publisher on `/camera/image` during the test), the node dropped it and said so: `1280 x 720 frame on the input but mode sim_delivery is 640 x 360; dropped`.

The sim's missions are not routed through the node: `target_cv` still reads the bridge's `/camera/image`. Doing that belongs with the sim's camera profiles (SIM-05).

## The FrameSource path, with the test pattern

```
ros2 launch argus_camera argus_camera.launch.py mode:=imx519_720p60 synthetic:=true
[argus_camera]: Mode imx519_720p60: calibration is not measured (scaled from the 4K calibration by 1/3; to be replaced by a calibration in this mode)
[argus_camera]: argus_camera: mode imx519_720p60, 1280 x 720 at 60 fps
[argus_camera]: argus_camera: 50.2 frames a second published (mode 60)       (then 48.0, 48.2, 51.8, 47.8)
ros2 topic info /camera/image -v     ->  publishers: argus_camera, ros_gz_bridge       (the 2026 name is an alias)
ros2 topic echo --once /argus/camera/down/image_raw --no-arr
    frame_id: down_camera_optical   height: 720   width: 1280   encoding: bgr8   step: 3840
ros2 topic echo --once --field k /argus/camera/down/camera_info
    [778.33216, 0, 624.68618, 0, 779.35542, 344.33087, 0, 0, 1]
ros2 run argus_camera camera_node --ros-args -p mode:=ov9782_flow
    RuntimeError / CalibrationError: the node refuses a mode that is in-process only or has no calibration
ros2 run argus_camera camera_bench --mode imx519_4k --seconds 3 --synthetic
    imx519_4k: 3840 x 2160, 48 frames, 15.8 fps (mode 17), longest gap 73 ms
    camera bench: PASS
```

- About 50 frames a second on two topic pairs (the new names and the 2026 aliases) from a Python thread on the laptop, with the test pattern pacing itself by sleeping. Whether the Jetson reaches 60 with the real sensor is a bench question; `legacy_topics:=false` halves the copying.
- `ros2 topic hz` on the 720p topic read 39.6 Hz: the command-line subscriber is the limit there, not the node.

## One thing found on the way: rclpy and image bytes

The first run published 2.8 frames a second. Assigning Python `bytes` to an image message's `data` makes rclpy check every byte in Python (an `assert` over the whole sequence), about a third of a second for one 720p frame. Given an `array.array('B', ...)` it copies the buffer and that is all. The node now does that: 2.8 to about 50 frames a second.

The 2026 detector builds its debug images the same slow way (`Image(..., data=annotated.tobytes())` twice per frame in `target_cv.py`, with `debug_view: true` in every shipped params file). At 640 x 360 that is roughly 75 ms per image by the same arithmetic. It is the likely reason a frame is 250 to 280 ms old by the time the sim's mission can use it (docs/results/2026-11-17-mavros-rates.md), and FND-01 should measure it first.
