# Smart re-acquisition: a lost track looked for from the height that shows it

Measured on 20 Nov 2026 (UTC) on a club laptop, on `platform/build-2027` at `2c3cf99` plus the CEN-06 change. Sim image `argus-sim:humble`. Calm air, flags on, sim time.

**Result.** Each of the four times the gated drop lost its track at 2.9 m, the mission went at once to 5.9 m above the ground over the track's last estimate ((3 x 0.08 m + 2.5 m) / 0.4625), had a CONFIRMED track again on arriving, and was descending on it again 5 s after the estimator called the track LOST. With the 2026 recovery on the same target (`drop/gated_track_loss`, flown for CEN-04) that took 7 s, from one metre up: a 4 s grace time, then the climb.

## Command

```
sim/run_scenarios.sh drop/reacquire --seeds 1 --variants on
```

```
| drop/reacquire | 1 | not run |  |  | OK | 0.00 | nothing released |
```

The scenario is `drop/gated_track_loss` with `smart_reacquire` on: a 2.8 m bullseye, no ring fit, so the box detector's box is cut by the picture's edge at about 3 m and the track cannot be kept below that. Flags: `use_frame_time_pose`, `use_target_estimator`, `use_height_estimator`, `centre_on_estimator`, `centring_retune`, `drop_release_gates`, `smart_reacquire`. Nothing is released in either scenario, by design: the release band is at 2.4 m.

## The flight's own lines (the first of four identical rounds, and the end)

Times are the log's wall-clock seconds; the real-time factor was 0.94.

```
78823.2 [package_drop] not descending: the bullseye track is COASTING, not CONFIRMED
78825.0 [package_drop] no target to centre on: bullseye track 1 is LOST
78825.0 [package_drop] State: FINAL_ALIGN -> TARGET_NOT_FOUND
78825.0 [package_drop] Target lost, looking again from 5.9 m above the ground over its last estimate (3 x 0.08 m + 2.5 m in view; attempt 1/4)
78828.9 [package_drop] at the height to look from (6.0 m above home): holding up to 10 s for the track to be CONFIRMED again
78828.9 [package_drop] bullseye track 1 CONFIRMED again after 0.0 s at the height
78828.9 [package_drop] State: TARGET_NOT_FOUND -> ACQUIRE_TARGET
78828.9 [package_drop] Target acquired, holding lock for 1.0 s
78829.9 [package_drop] State: ACQUIRE_TARGET -> TRACK_AND_DESCEND
78830.0 [package_drop] Entering final align at 6.34 m (handoff 8.00 m)
...
78879.1 [package_drop] Target lost, looking again from 5.9 m above the ground over its last estimate (3 x 0.08 m + 2.5 m in view; attempt 4/4)
78883.0 [package_drop] bullseye track 1 CONFIRMED again after 0.0 s at the height
78896.0 [package_drop] not descending: the bullseye track is COASTING, not CONFIRMED
78897.8 [package_drop] Target recovery exceeded 4 attempts
Mission 'package_drop' ended with status FAILURE
```

## Against the 2026 recovery on the same target

| | 2026 recovery (`drop/gated_track_loss`, 10:54Z) | `smart_reacquire` (`drop/reacquire`, 12:44Z) |
| --- | --- | --- |
| From the track being LOST to the climb starting | 4.3 s (the mission's grace time) | 0.0 s |
| Height looked from | 4.1 m above home (one metre up) | 5.9 m above the ground asked; taken as arrived at 6.0 m above home, which in the sim is 5.8 m above the ground |
| From LOST to descending in FINAL_ALIGN again | 7.3 s | 5.0 s |
| Rounds before the mission failed | 4 | 4 |
| Lowest true height | 2.894 m | 2.886 m |

(Wall-clock seconds at real-time factors of 0.92 and 0.94.)

- The track was the same track each time (id 1): the estimator keeps a LOST track for 30 s, and the first whole boxes from 5 m up confirmed it again before the vehicle had finished arriving.
- At 2.9 m the track's sigma was 0.08 m, so the margin (2.5 m) is nearly all of the height asked. A wider track looks from higher: 1.0 m of sigma asks 11.9 m.
- An earlier flight of the first version of the change (12:30Z) took the look as arrived half a metre under the height asked (5.4 m), the mission's climb tolerance; after the reviews the look is from no more than 0.1 m under it, the height is above the ground and not above home, and the vehicle moves at most 2.5 m sideways. The flight above is of the code as committed.

## Not flown

- The 10 s running out, the climb that does not arrive, the ceiling, an estimate far from the vehicle, the gated drop's timeout as the way in, and a track found again as a new track: mission tests (`src/drone_mission_demo/test/test_smart_reacquire.py`).
- A target that is found again and then kept: this scenario's target cannot be kept below 3 m, so every round ends the same way. Gate G2's flights in wind are where a real loss and recovery would show.
