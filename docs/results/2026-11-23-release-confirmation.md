# Release confirmation on sim flights (CEN-14), 2026-11-23

What `tools/release_confirm.py` says of flights where the sim knows the answer: the sim's payload bridge either let a 0.5 kg package go from the vehicle or had nothing to let go. Each flight is flown through the GO command as an attempt (PLT-12); `sim/attempt_check.py` writes the post-flight report into the bundle and fails the run when the report's release confirmation is not what the bridge did.

## Commands

```
sim/run_scenarios.sh drop/attempt_release --variants on
sim/run_scenarios.sh drop/attempt_release_empty --variants on
sim/run_scenarios.sh delivery/attempt_release --variants on
sim/run_scenarios.sh delivery/attempt_release_empty --variants on
```

Sim time, no flag on (the profile is `comp_2026`: the 2026 missions with `fake_drop: false`), seed 1, calm.

## Results

| Flight | The sim | Command | Output | Step (in the air) | Flight (steady throttle after against before) | Report | Run |
| --- | --- | --- | --- | --- | --- | --- | --- |
| drop, 0.5 kg carried | let it go | DO_SPRAYER, accepted | 8: 0 to 1900 us, 0.28 s after | throttle fell 11.0%, rose at 0.16 m/s | fell 4.3% | CONFIRMED (step, flight) | OK |
| drop, nothing carried | let nothing go | DO_SPRAYER, accepted | 8: 0 to 1900 us, 0.31 s after | fell 0.1%, 0.02 m/s | rose 6.1% | NOT CONFIRMED | OK |
| delivery, 0.5 kg carried (second flight) | let it go | DO_GRIPPER, accepted | 7: 1500 to 1100 us, 0.05 s after | none: released on the ground | fell 7.7% | CONFIRMED (flight) | OK |
| delivery, 0.5 kg carried (first flight, the rule as first written) | let it go | DO_GRIPPER, accepted | read as "stayed at 1100 us" | none | fell 7.8% | NOT CONFIRMED: wrong | FAIL |
| delivery, nothing carried | let nothing go | DO_GRIPPER, accepted | 7: 1500 to 1100 us, 0.05 s after | none: released on the ground | rose 3.7% | NOT CONFIRMED | OK |

The rangefinder saw nothing pass in any of them, as expected: the sim's package hangs beside the camera and outside the rangefinder's beam.

Two earlier real releases (SIM-06's flights of 18 November, read from SITL's own log files): the drop's step was 14.7% and 13.7%, and the delivery's steady throttle fell 6.7%. A 0.5 kg package on the sim's 1.5 kg vehicle should take the hover throttle down by about 13% (thrust with weight, throttle with its square root).

## What it shows

- The step separates cleanly: 11 to 15% with a package against 0.1% without. The threshold is 4%.
- The steady-flight comparison is the weaker evidence: +7 to 8% with a package against -4% without on the delivery (a climb away compared with a descent), and on the drop it is muddied by the climb that follows the release (4.3% with, -6.1% without). It is what a release on the ground has.
- The first delivery flight failed its run, rightly: the flight controller logged the gripper's output at its new value with the very time stamp of the command, and the rule took that sample for "before" and said the output had not moved. The rule now counts a sample within 0.05 s before the command's stamp as after it (`Thresholds.same_loop_s`), a unit test holds the case, and the scenario was flown again: CONFIRMED. On the first flight's own log the corrected rule says CONFIRMED too.
- The empty delivery's real-time factor was 0.78 (the strict check was running beside it); the run is on sim time and its numbers are in sim seconds.

## Not shown

- The aircraft: a lighter payload on a heavier vehicle steps the throttle less (0.25 kg on 4 kg: about 3%), and a real hover's throttle is noisier than the sim's. The thresholds are to be measured with a dummy payload ("Needs a person").
- The rangefinder evidence on a real fall through the beam: unit tests only.
- A latch that does not open with the package on it: the control here is a vehicle carrying nothing, which is the same to the throttle at the release and not the same before it.
