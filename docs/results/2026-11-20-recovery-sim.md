# The recovery sim: hook, grab, lift, carry, set down (REC-8)

2026-11-20, on `84a610c` plus the REC-8 change (uncommitted when flown; it is the commit this file comes with).

## Command

```bash
sim/run_scenarios.sh recovery/hook_check --seeds 1 --variants on
```

ArduPilot SITL and Gazebo. The recovery vehicle (`iris_recovery`: downward camera, flow camera, a hook on a
damped joint) is flown by a scripted tool, `sim/recovery_check.py`, not by a mission: there is none yet (REC-2).
The tool is told where the bucket truly is. A 1 kg bucket stands on the X mat 14.4 m from the takeoff point.

## Result (the fourth flight, as committed)

| | |
| --- | --- |
| Status | OK, real-time factor 0.91 |
| The check's own verdict | PASS: every step reached, landed 2 m beside the bucket |
| Grab | taken at once, in the bridge's own envelope (0.10 m sideways, 0.15 m up and down): the hook 0.006 m sideways and 0.009 m in height from the handle, by the bridge's measure from true positions and the hook's place at rest |
| Lift | the bucket rose 2.17 m (the judge's line is 0.5 m) |
| Carry | 14.4 m in 17.7 s of sim time, the commanded point walked at 1 m/s; the bucket swung at most 0.10 m out from under the vehicle |
| Release | the judge's measure: the bucket 0.05 m above where it stood (the rules' ceiling is 1.83 m) |
| Where the bucket ended | 0.004 m from the takeoff point |
| Judge (recovery, from true positions) | 19.99 points: 16 less the distance, and the autonomy bonus |

## The three flights before it, and what they found

| Flight | Status | What happened | What was changed |
| --- | --- | --- | --- |
| 1 | INVALID, real-time factor 0.57 | The vehicle flew to the bucket and held within 1 cm, and the check never saw it arrive: it judged arrival on the height above home, which is not the height the local frame commands in. | The check reads the autopilot's local position. The recovery vehicle's flow camera went from 640 x 400 at 60 Hz to 320 x 200 at 20 Hz: with the full one the sim cannot keep 0.8. |
| 2 | OK, 0.91, the check FAILED | Grab at 0.007 m, lift to 2.69 m. Sent to the takeoff point at the autopilot's own speed, the vehicle dashed, braked, and the swinging bucket brought it down 0.9 m short. | With the bucket on the hook the commanded point is walked at 1 m/s. |
| 3 | OK, 0.91, the check PASSED | The whole flight, with the envelope widened to 0.3 m and the judge not seeing the release (it read releases from a mission's log only): 19.98 points, the bucket 0.013 m from the takeoff point. | After the reviews: the bridge's own envelope, the check on the sim's clock, and a bucket's release written into the trace for the judge. |

## Reading it

- The sim's recovery parts work together: the vehicle flies with its hook, the payload bridge joins the bucket
  to the hook when the gripper output says grab and the hook is in the envelope, the bucket hangs and swings on
  the damped joint, the bridge lets it go, and the judge scores what happened.
- A 1 kg bucket under a 1.76 kg vehicle is a pendulum that matters: the second flight is the reason the mission
  needs its swing shaping and speed cap (REC-4) before it carries anything.
- The hook was within a centimetre of the handle. That is the sim's GPS at a place the tool is told, with the
  hook hanging still: not a test of aim, and the bridge judges the envelope on where the hook hangs at rest.

## Not shown

- Runs R1 to R6 of the spec: they need the recovery mission (REC-2 and REC-3 to REC-7).
- A release above 1.83 m being scored zero in a flight: the judge now sees a bucket's release whoever flew it
  (unit-tested), and this flight's was at 0.05 m.
- Anything about the real mechanism: the hook's length, damping and envelope are the sim's own numbers.
