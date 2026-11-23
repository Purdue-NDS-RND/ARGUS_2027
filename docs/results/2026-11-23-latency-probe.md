# Latency probe (PLT-05): the second test's numbers in the sim

2026-11-23, build machine (the sim on a laptop, sim time). These are the sim's numbers, not the aircraft's: the LED probe and both numbers below are still owed on the bench and in the field ("Needs a person", PLT-05).

## Actuation delay: a command step to the start of the tilt

```
$ sim/run_scenarios.sh hover/step_probe --variants on --record
   OK          hover/step_probe seed 1 on: 1.00 points, held within 1.03 m for 60 s
$ python3 sim/replay/command_delay.py sim/.state/suites/hover-step_probe-20261123T133104Z/runs/hover-step_probe-s1-on/bag
capture to command: not measured (0 frame-stamped detections, none followed by a setpoint)
actuation delay: median 79 ms, max 96 ms over 12 of 12 command steps (FND-06 forecasts over 80 ms)
```

(The flight was made twice, before and after the flight-safety review's changes to the mission: the first gave a median of 79 ms and a largest of 102 ms.)

Six velocity steps of 1 m/s for 1 s (flag `command_step_probe`), so twelve edges. The median, 79 ms, is what FND-06 forecasts over (80 ms). It is ArduPilot SITL's response with MAVROS on its serial port, heard by the recorder: the aircraft's link and airframe will give their own number.

The sim's hover is noisy (the attitude wanders 0.25 degrees RMS at rest), so the tilt is first seen above its noise about 100 ms after it starts: read at that first sighting the first flight's twelve edges gave a median of 180 ms. The tool reports the start, carried back along a line through the next 150 ms of tilt.

## Capture to command: a frame's stamp to the first setpoint that could use it

The line is 150 ms at the 95th percentile. From two delivery flights recorded on sim time on 18 and 20 November (bags under `sim/.state/`, not in git):

| Flight | Detections followed | Median | p95 | Max | Line |
| --- | --- | --- | --- | --- | --- |
| `delivery/estimator` on, seed 1 | 1524 | 66 ms | 102 ms | 225 ms | PASS |
| `delivery/height_estimator` on, seed 1 | 714 | 65 ms | 92 ms | 220 ms | PASS |

```
$ python3 sim/replay/command_delay.py sim/.state/suites/delivery-estimator-20261118T050446Z/runs/delivery-estimator-s1-on/bag
capture to command: median 66 ms, p95 102 ms, max 225 ms over 1524 detections (line: p95 within 150 ms): PASS
```

A flags-off flight (`delivery/baseline`) cannot be followed: its 2139 detections carry the detector's clock, not their frame's stamp (`features.stamp_from_image` is off, as in 2026), and the tool says so instead of guessing.

In those flights the commands themselves have no steps (the largest change between two setpoints is a few centimetres a second), which is why the actuation delay needs the step probe.
