# The post-landing second pass (LOC-12), 2026-11-24

Bench numbers from this laptop (8 threads, i7-1195G7), on a made-up survey. Nothing here is from the aircraft's computer, a real camera or a trained detector.

## Commands

```
# a made-up logged survey: 30 frames of 3840 x 2160 as JPEG, with its index and its record of poses and ranges
python3 -c "import sys; sys.path[:0]=['src/argus_localisation/test','src/argus_localisation','src/drone_mission_core','src/argus_camera']; from pathlib import Path; import test_second_pass as made; from drone_mission_core.frame_projection import load_camera; made.survey(Path('sim/.state/loc12/survey_4k'), *load_camera('sim_still4k', 'sim_iris_downcam'), frames=30, height=40.0, suffix='.jpg', mode='sim_still4k')"
python3 tools/second_pass.py sim/.state/loc12/survey_4k --camera-mode sim_still4k --airframe sim_iris_downcam --time-only --tile-ms 25
pytest src/argus_localisation/test/test_second_pass.py tools/test/test_second_pass_tool.py        (as scripts/check.sh runs them)
```

## What a frame costs without the detector

```
second pass: 30 of 30 logged frame(s) looked at in 0 s (every frame was looked at), 0 box(es)
reading and tiling a frame: 16 ms on this computer, 16 tile(s) a frame (3840 x 2160)
with a detector at 25 ms a tile: 0.42 s a frame, so 2054 frame(s) fit in the 900 s: all 30 of this survey
```

- A 4K frame is 16 tiles of the detector's size.
- Reading and tiling took 16 ms a frame here. The made-up frames are dark with three white plates and are 128 KiB each as JPEG; a real frame at quality 90 is megabytes and will take longer to decode, on a slower computer.
- The 25 ms a tile is a guess to plan with, not a measurement: there is no engine (the training-data hold). At that guess 2054 frames fit in the 855 s the pass has (900 s less 45 s for the merge and the list). The camera's 4K mode gives 17 frames a second, so a four-minute survey logged whole is about 4080 frames: every second one.

## What the tests show

| What | Result |
| --- | --- |
| A logged survey of 40 frames over three plates, replayed through the real pipeline (tiling on, a stand-in detector that finds bright blobs, the real projection, the real tracker) from the recorded poses and ranges | three confirmed tracks, each within 0.15 m of its plate |
| 3000 frames at 2 s each against the 900 s | 380 to 428 frames looked at, in order, spread to the survey's end (no gap over 9 frames after the first few), the budget kept with its reserve |
| A frame costing more than what is left | not begun |
| Merge: a live track 0.6 m from a second-pass track | one marker, placed by sightings, votes summed: a digit neither pass was sure of alone |
| Merge: live tracks with no partner, a tentative or outside second-pass track | every live track kept as it was; only confirmed second-pass tracks inside the area added |
| Merge: two tracks with the same claim to one digit | both left unread (the vote's own rule) |
| A survey in whose record the flight controller's position steps 3 m east and 2 m south halfway | one step found and followed: three tracks, each within 0.35 m of where the frame after the step puts its plate (not five) |
| A survey of a number plate (the synthetic marker, digit 7) that the flight listed and did not read, with the stand-in template classifier | the second pass reads it: `track 1: digit not read -> 7`, one row, ID 7 |
| The node with the flag on, through a survey with the vote on | the record and the tracks written beside the frames by the record's own thread; the track's digit, vote weight and count as the vote holds them; the pipeline's settings with them |
| The tool on a logged survey with the flight's tracks | the merged list written (CSV, KML, map), what changed printed, the earlier list kept under another name |

## Not measured

The detector's cost on a tile on the aircraft's computer; a real survey's frames; how many more markers, or better digits, a second pass gives. Those need an engine and a flight.
