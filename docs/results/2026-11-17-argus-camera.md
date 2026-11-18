# argus_camera in the sim container (PLT-03)

Measured on 17 Nov 2026 on a club laptop, on `platform/build-2027` at `9f7d171` plus the PLT-03 change, inside the sim container (`argus-sim:humble`, ROS 2 Humble). No real camera was involved: the IMX519 and OV9782 paths are for the Jetson bench.

## The sim's camera through the node

Sim started with `SIM_TIME=1 HEADLESS=1 sim/run_sim.sh --detach`, vehicle on the ground. The node relays the bridged Gazebo camera, and the 2026 detector reads the node's topic:

```
ros2 launch argus_camera argus_camera.launch.py mode:=sim_delivery use_sim_time:=true
ros2 run drone_target_cv target_cv --ros-args -p image_topic:=/argus/camera/down/image_raw -p start_enabled:=true -p sim_hsv:=true -p use_sim_time:=true
ros2 topic hz /argus/camera/down/image_raw          --use-sim-time   ->  average rate: 22.325
ros2 topic hz /drone_package_drop/target_detection  --use-sim-time   ->  average rate: 23.226      (one detection per frame)
```

From the first run of the relay (before the review fixes; the relay path itself did not change):

```
ros2 topic hz    /camera/image                 --use-sim-time   ->  average rate: 25.473
ros2 topic delay /camera/image                 --use-sim-time   ->  average delay: 0.009   (min 0.006, max 0.014)
ros2 topic delay /argus/camera/down/image_raw  --use-sim-time   ->  average delay: 0.010   (min 0.007, max 0.015)
ros2 topic echo --once /argus/camera/down/camera_info
    frame_id: down_camera_optical   640 x 360   d: all 0
    k: [389.16536, 0, 320, 0, 389.16536, 180, 0, 0, 1]      p: [389.16536, 0, 320, 0,  0, 389.16536, 180, 0,  0, 0, 1, 0]
ros2 topic echo --once --field k /camera/camera_info        (Gazebo's own)
    [389.16534424, 0, 320, 0, 389.16539669, 180, 0, 0, 1]
```

- The relay keeps the image's own stamp (the delay is measured from it) and adds about 1 ms. Rates were measured one after the other, 12 s each, so the few Hz between them are not a loss figure.
- The mode's calibration, computed from the sim model's field of view, matches the matrix Gazebo publishes to the sixth digit.

The sim's missions are not routed through the node: in a flight `target_cv` still reads the bridge's `/camera/image`. Doing that belongs with the sim's camera profiles (SIM-05).

## The two guards, live

```
ros2 launch argus_camera argus_camera.launch.py mode:=imx519_720p60 synthetic:=true legacy_topics:=true      (beside the sim's bridge)
[FATAL] [argus_camera]: argus_camera stopping: another node also publishes on /camera/camera_info, /camera/image; two cameras on one topic would interleave their frames
[ERROR] [camera_node-1]: process has died [pid 834, exit code 1, ...]

ros2 launch argus_camera argus_camera.launch.py mode:=sim_delivery legacy_topics:=true
argus_camera.publishing.TopicError: input_topic /camera/image is also a topic this node publishes: it would relay its own output for ever. Turn legacy_topics off or take the input from another topic
```

## The FrameSource path, with the test pattern

```
ros2 launch argus_camera argus_camera.launch.py mode:=imx519_720p60 synthetic:=true
[argus_camera]: argus_camera: 60.0 frames a second published (mode 60)       (three 5 s windows, all 60.0)
```

With the 2026 names on, moved aside with remaps because the sim's bridge owns the real ones, and the 2026 detector reading the compressed one as the live params make it:

```
ros2 run argus_camera camera_node --ros-args -p mode:=imx519_720p60 -p synthetic:=true -p legacy_topics:=true \
    -r /camera/image:=/t/camera/image -r /camera/image/compressed:=/t/camera/image/compressed -r /camera/camera_info:=/t/camera/camera_info
ros2 run drone_target_cv target_cv --ros-args -p image_topic:=/t/camera/image/compressed -p compressed_input:=true -p start_enabled:=true
[argus_camera]: argus_camera: 59.8 frames a second published (mode 60)       (then 60.0, 60.0)
ros2 topic hz /t/camera/image/compressed            ->  average rate: 30.009
ros2 topic hz /drone_package_drop/target_detection  ->  average rate: 20.471
```

- The node's own topic runs at the mode's 60 frames a second while the 2026 names get the 2026 rate, 30, including the JPEG.
- The detector kept up with 20 of those 30 frames a second at 1280 x 720 on the laptop; it publishes one detection per frame it processes.
- Earlier runs, from the first version: 1280 x 720 `bgr8` with frame id `down_camera_optical`; `k: [778.33216, 0, 624.68618, 0, 779.35542, 344.33087, 0, 0, 1]`; `ros2 run argus_camera camera_node --ros-args -p mode:=ov9782_flow` refused; `camera_bench --mode imx519_4k --seconds 3 --synthetic` gave 15.8 fps against 17 and PASS.

On the final node code (after the second review round) the relay and the start-up guard were run once more: the relay published 21.0 then 25.8 frames a second, the node said `publishing a TEST PATTERN, not a camera` and stopped on the 2026 names beside the bridge as before, and `ros2 run argus_camera camera_bench --mode imx519_720p60 --seconds 2 --synthetic` ran as an installed command (FAIL, correctly: that mode's calibration is not measured).

## One thing found on the way: rclpy and image bytes

The first run published 2.8 frames a second, and the second about 50 because the test pattern slept a whole frame period after its work; it now paces to a deadline (60.0 above). Assigning Python `bytes` to an image message's `data` makes rclpy check every byte in Python (an `assert` over the whole sequence), about a third of a second for one 720p frame. Given an `array.array('B', ...)` it copies the buffer and that is all. The node now does that.

The 2026 detector builds its debug images the same slow way (`Image(..., data=annotated.tobytes())` twice per frame in `target_cv.py`, with `debug_view: true` in every shipped params file). At 640 x 360 that is roughly 75 ms per image by the same arithmetic. It is the likely reason a frame is 250 to 280 ms old by the time the sim's mission can use it (docs/results/2026-11-17-mavros-rates.md), and FND-01 should measure it first.
