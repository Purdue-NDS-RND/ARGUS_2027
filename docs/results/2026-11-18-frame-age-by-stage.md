# Where a frame's age comes from in the sim, stage by stage

Measured on 18 Nov 2026 (UTC) on a club laptop, on `platform/build-2027` at `89f87b5` plus the SIM-09 change. Sim image `argus-sim:humble`. This follows up the wave 1 finding that a frame is about 40 ms old at the median when the mission hears its detection but 260 to 275 ms at the 95th percentile (`docs/results/2026-11-18-capture-time.md`).

**Result.** On one `delivery/baseline` flight, every flag off, the frame age was 35 ms at the median and 228 ms at the 95th percentile, and the tail is in one stage: the 2026 detector. Rendering, the bridge and the publish are flat. Inside the detector the time is frames waiting in its queue behind earlier frames, and it happens below about 6 m, where the bullseye is large in the image.

| Stage | What it is | Median | 95th percentile | Maximum |
| --- | --- | --- | --- | --- |
| Render | Gazebo's render stamp → `camera_info` heard in ROS | 0.7 ms | 1.9 ms | 6.0 ms |
| Bridge | `camera_info` heard → the image itself heard by a subscriber | 14.2 ms | 17.7 ms | 29.4 ms |
| Detector | image heard → `target_cv` publishes its detection | 19.3 ms | 213.0 ms | 327.8 ms |
| Publish | detection published → detection heard | 0.2 ms | 0.8 ms | 3.6 ms |
| **Frame age** | stamp → detection heard (the four together) | **35 ms** | **228 ms** | **343 ms** |

1906 frames followed from stamp to detection. In the slowest tenth of them (87 ms and over) the mean by stage is render 1 ms, bridge 14 ms, detector 194 ms, publish 0.3 ms.

## Inside the detector stage

The detector handles one frame at a time. From the moment each frame reached a subscriber and the moment the detector's annotated copy of it came out, its stage splits into waiting (for the frame before to be finished) and working:

| | Median | 95th percentile | Maximum |
| --- | --- | --- | --- |
| Working on a frame | 20.1 ms | 53.7 ms | 186.9 ms |
| Waiting behind earlier frames | 0.0 ms | 152.2 ms | 267.5 ms |

And by the vehicle's height (1986 frames):

| Height | Frames | Working: median; 95th percentile | Waiting: 95th percentile |
| --- | --- | --- | --- |
| 15 to 18 m | 210 | 19.0 ms; 36.1 ms | 2.7 ms |
| 12 to 15 m | 223 | 18.5 ms; 31.5 ms | 0.0 ms |
| 9 to 12 m | 279 | 19.4 ms; 33.6 ms | 0.0 ms |
| 6 to 9 m | 360 | 20.1 ms; 36.4 ms | 2.7 ms |
| 3 to 6 m | 381 | 23.1 ms; 57.4 ms | 174.4 ms |
| 0 to 3 m | 533 | 23.3 ms; 59.1 ms | 185.0 ms |

A frame arrives every 33 ms. Above 6 m the detector's work is under that at the 95th percentile and nothing waits. Below 6 m its work is over 33 ms often enough (57 to 59 ms at the 95th percentile, bursts up to 187 ms) that frames pile up, and the detector then works through them oldest first: every detection it publishes is of a frame that has already waited 150 to 270 ms.

## The likely cause

Two things in the 2026 detector node together, neither changed here:

1. **Its work per frame grows with the size of the target in the image.** Below 6 m the 5 m sim bullseye fills the 640 x 360 frame. The work is not the two debug images it publishes every frame in the sim (`debug_view: true`): those are the same size at every height, and the work is flat from 18 m down to 6 m.
2. **It keeps a queue of five frames and takes the oldest first** (the image subscription uses the sensor-data QoS, keep last 5). When it falls behind it does not skip to the newest frame; it publishes detections of frames up to five periods old until it catches up, and at 20 ms a frame against 33 ms between frames that takes many frames.

A second recorded flight supports the first point. `delivery/estimator` has a 1.5 m bullseye, which is small in the image until the last 3 m:

| | `delivery/baseline`, 5 m bullseye, flags off, wall time | `delivery/estimator`, 1.5 m bullseye, flags on, sim time |
| --- | --- | --- |
| Frame age: median; 95th percentile; maximum | 35 ms; 228 ms; 343 ms | 42 ms; 98 ms; 447 ms |
| Detector stage: median; 95th percentile | 19 ms; 213 ms | 28 ms; 80 ms |
| Waiting at 3 to 6 m, 95th percentile | 174 ms | 20 ms |
| Waiting at 0 to 3 m, 95th percentile | 185 ms | 184 ms |

With the small target the queue only builds in the last 3 m, where it too fills the frame, and the 95th percentile of the frame age over the flight falls from 228 ms to 98 ms. (That flight is on sim time, so its times are in steps of the 4 ms sim clock, and it has the estimator and the contract output running; the comparison is of shape, not of milliseconds.)

## What this means for the build

- The stale-data guard's 250 ms limit (FND-03) and the estimator's 0.3 s coast (FND-06, FND-07) are both close to this tail. Below 6 m on a 5 m target, one frame in twenty is over 200 ms old.
- The fix belongs in how frames are taken, not in the limits: a detector that takes the newest frame and drops the rest (a queue of one) would publish frames 20 to 60 ms old at these heights instead of 150 to 270 ms. The 2027 pipeline's camera driver and detector node (PLT-03, LOC-5) can be built that way. The 2026 detector is left as it is.
- On the aircraft the numbers will differ (a real camera, the Jetson, no Gazebo sharing the CPU); the same recording and the same command give them there.

## How it was measured

```
$ sim/run_scenarios.sh delivery/baseline --seeds 1 --variants off --record
| delivery/baseline | 1 | OK | 16.20 | 0.04 m from the target | not run |  |  |
$ python3 sim/replay/latency.py sim/.state/suites/delivery-baseline-20261118T050015Z/runs/delivery-baseline-s1-off/bag
1906 frames followed from stamp to detection (frame stamps converted through the bag's sim-clock tap)
frame age when the detection is heard: median 35 ms, p95 228 ms, max 343 ms
  render   median    0.7 ms, p95    1.9 ms, max    6.0 ms
  bridge   median   14.2 ms, p95   17.7 ms, max   29.4 ms
  detector median   19.3 ms, p95  213.0 ms, max  327.8 ms
  publish  median    0.2 ms, p95    0.8 ms, max    3.6 ms
  detector waiting: median 0.0 ms, p95 152.2 ms, max 267.5 ms over 1986 frames
  detector working: median 20.1 ms, p95 53.7 ms, max 186.9 ms over 1986 frames
  at 0 to 3 m: working median 23.3 ms, p95 59.1 ms; waiting p95 185.0 ms (533 frames)
  at 3 to 6 m: working median 23.1 ms, p95 57.4 ms; waiting p95 174.4 ms (381 frames)
  at 6 to 9 m: working median 20.1 ms, p95 36.4 ms; waiting p95 2.7 ms (360 frames)
  at 9 to 12 m: working median 19.4 ms, p95 33.6 ms; waiting p95 0.0 ms (279 frames)
  at 12 to 15 m: working median 18.5 ms, p95 31.5 ms; waiting p95 0.0 ms (223 frames)
  at 15 to 18 m: working median 19.0 ms, p95 36.1 ms; waiting p95 2.7 ms (210 frames)
the slowest 10% of frames (from 87 ms), mean ms by stage: render 1, bridge 14, detector 194, publish 0; largest: detector
$ sim/run_scenarios.sh delivery/estimator --seeds 1 --variants on --record
$ python3 sim/replay/latency.py sim/.state/suites/delivery-estimator-20261118T050446Z/runs/delivery-estimator-s1-on/bag
1696 frames followed from stamp to detection (the recorder is on the frames' clock)
frame age when the detection is heard: median 42 ms, p95 98 ms, max 447 ms
```

The recorder (`--record`, SIM-09) stores no images. It records `camera_info`, the detections, and one small message for every frame heard on `/camera/image` and on the detector's `/target_cv/annotated`, which carries the frame's own header; a 2026 detection, stamped with the detector's clock, is matched to the annotated copy published just before it. The run was on wall time, so frame stamps (Gazebo's clock) are set against the recorder's clock through a tap of Gazebo's clock at 250 Hz.

## What to keep in mind

- One flight of each scenario, one seed, one laptop. The real-time factor was 0.99.
- The taps add two subscribers to two image topics, which costs the bridge and the detector a little; the stages they measure include that.
- Image topics are best effort: of 4880 frames rendered the tap heard 4400 on `/camera/image`, and of the detector's annotated copies it heard 1994. 226 detections had no annotated copy within 15 ms and were left out rather than guessed at. If the frames that went missing were the slow ones, the tail is understated.
- "Working" is everything the node does for a frame (detection, the overlay, publishing two debug images, the detection itself) plus any time it spent on other callbacks. Which part grows with the target's size was not separated: that would mean instrumenting the detector.
- The bridge's 14 ms is the image's 691 kB through `ros_gz_bridge` and to a Python subscriber. It is steady, so it adds to every frame's age but not to the tail.

Run folders (not committed): `sim/.state/suites/delivery-baseline-20261118T050015Z/`, `sim/.state/suites/delivery-estimator-20261118T050446Z/`; each has `bag/` and `stages.json`.
