# The GPS health monitor and the source-set switch in the sim (REC-6), 2026-11-24

## Commands

```
sim/run_scenarios.sh recovery/gps_step --variants on
pytest src/drone_mission_core/test/test_gps_health.py        (as scripts/check.sh runs it)
```

The recovery on the oracle detector with the mission flag `ekf_source_switching` on, sim time, seed 1, calm, and a 10 m step north of the simulated GPS 41 s after arming, in the descent over the mat, held for 60 s (the spec's run R3). Times are the sim's own clock: the launch log's lines placed on it through the run's clock marks. Four flights, one seed each; the rule changed between them, and each says which rule flew.

## The spec's three pass lines

| Line | Where it is shown | Result |
| --- | --- | --- |
| A 5 m step detected within 1.5 s | unit tests on made-up flights (hovering and at 3 m/s, with vision and without), and four sim flights with a 10 m step | unit: within 1.5 s in every case. Sim: 0.11 s, 0.19 s, 0.22 s and 0.11 s after the step was set |
| A 0.5 m/s drift within 5 s | unit tests only: with vision, and with a target that stands still and no vision | within 5 s in both. With neither, a slow drift cannot be told from flight and nothing is said. The sim's injector has no drift event; none was flown |
| No false alarm in 30 minutes of clean data | unit test: 30 minutes of hovering and flight, four seeds, 0.25 m of noise on every fix and 0.1 m/s on the flow | no UNTRUSTED; SUSPECT about 2% of the time. Made-up noise, not a receiver's |

## The flight on the committed rule (flight 4)

`OK`, 19.99 points, the bucket set down 0.006 m from the return point. No flow odometry started, so nothing feeds source set 2, and the mission does not ask for it.

| Sim time | What |
| --- | --- |
| 66.528 s | `making sure the flight controller is on source set 1`; `holding until the flight controller answers for source set 1` |
| 66.532 s | `source set 1 acknowledged by the flight controller`; the mission starts on the next tick (`State: HOLD -> TRANSIT`, 66.585 s) |
| 94.195 s | the step is set (10 m north) |
| 94.301 s | `GPS UNTRUSTED (the GPS position moved 3.3 m beyond what was flown in 1 s (by its own velocity))`: 0.11 s after the step. `source set 2 is not asked for, nothing is feeding it (no optical flow is being sent to the flight controller); it stays on set 1` |
| 98.9 to 105 s | ENGAGE, LIFT, attach verified: flown on local setpoints by the bucket's track, the flight controller still on GPS |
| 105.05 s | `GPS TRUSTED`: ten seconds with nothing wrong (the step is still on; a step that stays looks like a steady GPS) |
| 154.214 s | the step is taken off (10 m back) |
| 154.306 s | `GPS UNTRUSTED` again, 0.09 s after; set 2 again not asked for |
| 176.5 s | `done: the bucket is set down (1 descent(s))` |

Flight 3 flew the same rule before the start's hold was added: `OK`, 19.99 points, the step found 0.19 s after it was set, set 2 not asked for, and the flight controller's own words were `EKF3 lane switch 1` once and no failsafe.

## The two flights before the flight-safety review (set 2 asked for although nothing fed it)

These flew the rule as first written: UNTRUSTED asked for source set 2 whether or not anything fed it. The review called that a blocker, and the committed rule does not do it; they are kept here because they are the only flights in which the switch itself was commanded.

- **Flight 2 (no flow odometry):** `OK`, 19.99 points. Step set at 94.19 s; `GPS UNTRUSTED` at 94.40 s (0.22 s); `asking the flight controller for source set 2`; `source set 2 acknowledged by the flight controller` at 94.42 s. ENGAGE, LIFT and CLIMB were flown on set 2. `GPS TRUSTED` at 105.0 s; in RETURN at 112.4 s set 1 asked and acknowledged 0.02 s later, with the step still on. Step off at 154.20 s, found 0.10 s later, set 2 asked and acknowledged, and the mission ended on set 2 (the review's second blocker: the committed rule asks for set 1 as the mission ends, and makes sure of it at the next start).
- **Flight 1 (flow odometry also started):** the step found 0.11 s after it was set, `by vision` and by the drift against vision together; set 2 acknowledged 0.01 s later; 20.00 points. On the way home at 15 m the monitor called the GPS drifting (`drifts 0.30 m/s against vision`) when it was the flow that was off: the flow is now a judge only from under 5 m, where REC-5 showed it good. The run was marked FAIL by the flow odometry's own pass line (REC-5's open item).

In both, the flight controller said `EKF3 IMU0 stopped aiding` and `EKF3 IMU1 stopped aiding` at each switch to set 2, `is using GPS` at each switch back, and raised no EKF failsafe (SITL's default `FS_EKF_ACTION` and `FS_EKF_THRESH`) in the 18 s it was on set 2 with no flow coming in. That it held in SITL says nothing for the aircraft.

## What it shows and does not

- A step is found fast, with or without vision.
- The source-set command reaches the flight controller and is acknowledged in about two hundredths of a second (set 1 in every flight; set 2 in flights 1 and 2).
- On the committed rule a step with nothing feeding set 2 leaves the flight controller on GPS: its position estimate follows the step, and the mission flew on by the bucket's track and completed. One seed.
- Going back: the monitor says TRUSTED ten seconds after a step that stays. Nothing on board can see a steady offset. That is recorded for the lead ("Needs a person").
- Not shown at all: a flight on source set 2 with flow reaching the flight controller (the switch as the spec means it); a drift in a flight; the spec's eight of ten for R3; that the switch gets in before the estimator's failsafe (PLT-08's owed flight); the estimator's own flag (not fed); a real receiver's thirty minutes.
