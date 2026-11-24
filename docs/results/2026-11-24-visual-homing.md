# Visual homing in the sim (REC-7b), 2026-11-24

## Commands

```
sim/run_scenarios.sh recovery/marker_homing --variants on
sim/run_scenarios.sh recovery/keyframe_homing --variants on
sim/run_scenarios.sh <recovery/marker_homing with the two marker flags taken out> --variants on      (the control; the file was a copy under sim/.state/)
```

The recovery on the real detector (`recovery_cv`), sim time, seed 1, calm. "Distance" is the judge's: the bucket's true place against the true takeoff point, from the sim's own poses. The scenarios pass on their log lines; the distance is read off beside them. One seed throughout.

## The marker (P1)

The return target is `{marker: 0, near: <a point 1.5 m east of the takeoff point>}`; the tag (36h11, id 0, 0.8 m, built of flat plates) lies at the takeoff point.

| Flight | Code | Result | Distance |
| --- | --- | --- | --- |
| control: the mission flag and the node flag off | as committed, less the flags | OK, 18.14 points | 1.488 m: set down at the coordinates |
| marker homing, first rule (a fix taken on one sighting) | before the flight-safety review | OK, 19.92 points; `homing: the return point is taken as seen, at (0.06, 0.03)` 3.5 s before SET_DOWN | 0.063 m |
| marker homing, second rule (a second of agreement) | before the wait at the coordinates | FAIL: no place was taken (the tag came into view about two seconds before the coordinates were reached, and the set-down began while its fixes were still agreeing); 18.14 points | 1.485 m |
| marker homing, as committed (the set-down waits up to 3 s at the coordinates while fixes agree; the tag looked for in every frame), three flights | as committed, and two steps before it | FAIL, FAIL, FAIL: no place was taken. The tag was read 32, 32 and 65 times in the takeoff climb and 2, 0 and 0 times after it | 1.484 m, 1.483 m, 1.492 m |

**Marker homing is not shown in the sim on the committed rule.** The one flight that set the bucket down on the marker flew the first rule, under which a single sighting decided; in that flight the tag was read 108 times in the takeoff climb and 11 times after it. The committed rule needs a second of agreeing fixes, and on the way back with the bucket on the hook the tag was read in almost no frame (the node's own counts above), so there was nothing to agree. Why it is read so rarely there was not established: the picture's middle is where the bucket hangs, and the tag passes close to it; two attempts to save the camera's frames in a flight for a look saved none. Looking for the tag costs 5 to 6 ms a frame in the sim's container (the node's report). The scenario `recovery/marker_homing` is committed as it is and fails its own log line.

Two earlier marker flights tried to make the error with a GPS step of 2 m instead of with the coordinates (0.056 m and 0.068 m). They show nothing: the control for them came back to 0.01 m as well, because the simulated flight controller's position estimate did not follow the step (docs/results/2026-11-24-gps-health.md).

## The keyframes (P2)

The return target is the takeoff point; number plates of three sizes lie about it, because the sim's ground is bare. The sim's local frame does not drift, so these flights show that the place the keyframes give is the true takeoff point. None corrects an error: that is shown only in the unit tests (a return point 1.8 m from where its coordinates say).

| Flight | Code | Pictures kept | Takeoff point found | Place taken | Distance |
| --- | --- | --- | --- | --- | --- |
| 1 | first rule | 4 (3.5, 6.7, 9.3, 13.3 m) | 152 times, first by 78 matches | (0.04, 0.04) | 0.057 m |
| 2 | first rule | 4 | 9 times: FAIL, no place taken | none | 0.01 m (the coordinates, which are right in the sim) |
| 3 | frames held for the pose of their moment | 4 | 638 times | (0.09, 0.07) | 0.111 m |
| 4 | the same | 4 | 628 times | (0.07, 0.06) | 0.104 m |
| 5 | second rule, the landed state, matched at most 5 times a second | 4 (3.3, 6.3, 9.3, 13.3 m) | 107 times, first by 39 matches | (0.05, 0.06) | 0.085 m |
| 6 | as committed (matched at most 3 times a second, only within 10 m, the set-down's wait) | 4 (3.3, 6.3, 9.5, 13.3 m) | 66 times, first by 26 matches | (0.02, 0.03) | 0.038 m |

Flight 2 is why a frame is now held until the pose of its moment has come: frames reached the node before their poses, and a frame without a pose was passed over. One keyframe match took up to 313 ms in the sim's container (flight 5; 195 ms in flight 6; the node's own report); it runs after the frame's X and bucket have gone out, at most three times a second, and only within 10 m of the takeoff point.

## What it shows and does not

- Under the first rule (one sighting decides), a marker 1.5 m from where the coordinates say was found and the bucket set down on it (0.063 m), where without the flags it went to the coordinates (1.488 m). Under the committed rule the marker was not read often enough on the way back to be taken, in three flights: the bucket went to the coordinates.
- The keyframes of the takeoff climb give the takeoff point to about 0.1 m in the sim.
- Not shown: any of it on real ground, with a real camera, or with a local frame that has drifted; the keyframes correcting an error in a flight; more than one seed; what the matching costs on the aircraft's computer.
