# The survey's frame logging on the build machine (LOC-11)

2026-11-20, on `a1d0a52` plus the LOC-11 change (uncommitted when run; it is the commit this file comes with).
Run on the build machine (an 8-thread laptop CPU, an SSD), not on the Orin Nano: these numbers show the tool and
the policy working, and say nothing of the Jetson's CPU or its NVMe. The Jetson's run is a person's step.

## Commands

```bash
python3 tools/frame_log_bench.py --out <folder> --seconds 5 --workers 1 --min-free-gb 5
python3 tools/frame_log_bench.py --out <folder> --seconds 5 --workers 3 --min-free-gb 5
python3 tools/frame_log_bench.py --out <folder> --seconds 10 --min-free-gb 5                                        # the defaults: 4K, 17 Hz, 3 workers
python3 tools/frame_log_bench.py --out <folder> --width 1280 --height 720 --rate 10 --seconds 5 --min-free-gb 5   # the sim's survey camera
```

The tool feeds textured test frames to the policy the localisation node uses
(`argus_localisation/frame_logging.py` over argus_camera's `FrameLog`), at the camera's rate, with a detector that
reports nothing wrong. 17 frames a second is the 4K sensor mode's rate (`argus_camera/config/modes.yaml`), which
is what "every frame" is; the spec's 42 MB a second is 15 a second at 2.8 MB a frame. `--min-free-gb 5` is for
this machine's small scratch disk; the flight's floor, and the tool's default, is 20 GB.

## Results

| Frames | Rate | Workers | Logged | MB a second | MB a frame | CPU cores used |
| --- | --- | --- | --- | --- | --- | --- |
| 3840 x 2160 | 17 Hz, 5 s | 1 | 85 of 85 | 71.4 | 4.18 | 0.67 |
| 3840 x 2160 | 17 Hz, 5 s | 3 | 85 of 85 | 71.3 | 4.18 | 0.70 |
| 3840 x 2160 | 17 Hz, 10 s | 3 | 170 of 170 | 71.2 | 4.18 | 0.70 |
| 1280 x 720 | 10 Hz, 5 s | 3 | 50 of 50 | 4.7 | 0.47 | 0.12 |

```
PASS: every frame logged
170 of 170 frames of 3840 x 2160 at 17 a second, 3 worker(s), JPEG quality 90
71.2 MB a second written (4.18 MB a frame); 0.70 CPU cores used over 10.0 s
JPEG library of this OpenCV: /usr/lib/x86_64-linux-gnu/libjpeg.so (ver 80)
```

## Reading them

- On this machine a 4K frame costs about 40 ms of one core to encode at quality 90 (0.7 cores at 17 a second), so
  one worker keeps up and more workers change nothing. The Orin Nano's cores are several times slower: there the
  encoding may need two or three of its six cores, at the lowered priority the policy gives its threads. That is
  what the Jetson's run is for.
- 4.18 MB a frame is the test frame's, which is noise over a gradient and compresses worse than a field. At the
  spec's 2.8 MB a frame, 17 a second is 48 MB a second. A 600 s survey is then about 29 GB (43 GB at this run's
  71 MB a second): the disk holds some twelve to seventeen surveys before the 20 GB floor stops the logging.
- The JPEG line is this machine's OpenCV (the system libjpeg, version 80, which on this distribution is
  libjpeg-turbo's library). The Jetson's OpenCV must be looked at with the same line: the spec counts on
  libjpeg-turbo.
- The logger never slowed (`keep_every` stayed 1): nothing was short of CPU here.

## Not shown

- The Orin Nano, its NVMe, and the logger beside a real detector and the mission executor: the bench step under
  "Needs a person".
- The logger giving way, the full disk, the disk that is not mounted, the stop that does not wait: tests
  (`src/argus_localisation/test/test_frame_logging.py`, and through the node in `test_loc_node.py`).
