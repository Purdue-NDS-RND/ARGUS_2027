# The sim's large camera frames: where they went, and all of them arriving

Measured on 20 Nov 2026 (UTC) on a club laptop, on `platform/build-2027` at `f915acd` plus the change described here. Sim image `argus-sim:humble`, headless. One flight of each scenario; the first try met the line.

**Result.** With a Fast DDS profile whose shared-memory segment holds whole frames, every frame the sim's survey camera makes arrives at a best-effort ROS subscriber: 10.0 Hz (8 needed; 5.5 Hz before). The 4K still camera's frames all arrive too: 1.0 Hz (0.6 Hz before). Only scenarios that ask for the profile get it.

## Commands

```
sim/run_scenarios.sh hover/survey_camera --seeds 1 --variants off
sim/run_scenarios.sh hover/still4k_camera --seeds 1 --variants off
```

| Scenario | Run | Frames made (`camera_info`) | Frames arriving (image topic, best effort) | Real-time factor | Before (18 Nov, `docs/results/2026-11-18-sim-sensors.md`) |
| --- | --- | --- | --- | --- | --- |
| `hover/survey_camera` (1280 x 720, 10 Hz) | OK | 10.0 Hz, 900 frames | 10.0 Hz, 901 frames, slowest 5 s window 10.0 Hz | 1.00 | 5.5 Hz, 485 of 893: FAIL |
| `hover/still4k_camera` (3840 x 2160, 1 Hz) | OK | 1.0 Hz, 89 frames | 1.0 Hz, 89 frames | 1.00 | 0.6 Hz, 54 of 90 |

Rangefinder in the survey hover: 0.007 m from the true height at the 95th percentile (limit 0.1 m), as before. True poses at 29.1 Hz.

## Where the frames went

A 1280 x 720 frame is 2.8 MB and a 4K one 25 MB. Fast DDS's built-in shared-memory transport has a 0.5 MB segment, so a frame is cut into fragments that overrun it before a subscriber has read them, and a best-effort subscriber loses the whole frame. The loss was in the transport between `ros_gz_bridge` and the subscriber, not in Gazebo (which made every frame) or in the subscriber.

## What was changed

- `sim/config/fastdds_large_frames.xml`: a participant profile with a 256 MB shared-memory segment (the container's `/dev/shm` is 1 GB, `sim/compose.yaml`) and UDP as the second transport.
- A scenario asks for it with `dds_profile: large_frames`; `sim/container/start.sh` then points `FASTRTPS_DEFAULT_PROFILES_FILE` at the file for the container's processes, and `sim/container/env.sh` does the same for the tools the runner starts in it. No `sudo`, nothing outside the repo.
- The two scenarios above ask for it. No other scenario does: the flags-off baseline, gate G0 and every flight on the 640 x 360 delivery camera run on Fast DDS's built-in transports as before (`test_a_scenario_can_ask_for_the_large_frame_dds_profile_and_only_then_is_it_used`).

## Not measured here

- A flight with the survey camera and a mission's own nodes subscribed (LOC-3 and LOC-5 are the first): more subscribers of the same frames share the one segment.
- The aircraft: there the 4K frames never cross a ROS topic (LOC-5 reads them in-process), so this profile is for the sim only.
