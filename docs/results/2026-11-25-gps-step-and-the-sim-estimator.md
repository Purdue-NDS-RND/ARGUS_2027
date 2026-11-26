# What the flight controller's estimator did with the 10 m GPS step (REC-6), and what the sim flies on

25 November 2026. No new flight: the flight controller's own log of `recovery/gps_step` flight 4 of 24 November (`sim/.state/sitl/logs/00000424.BIN`; the step is set 41.6 s into the log and held for 60 s), read with `tools/dataflash_params.py`'s `messages` (`XKF1`, `XKF3`, `XKF4`, `SIM2`, `GPS`, `POS`, `MSG`) and `read_parameters`.

## EKF3 did follow the step, after about 10 s

North, each EKF3 core's position (`XKF1.PN`) against SITL's true position (`SIM2.PN`):

| From the step | Core 0 | Core 1 | Position innovation (`XKF3.IPN`), test ratio (`XKF4.SP`) |
| --- | --- | --- | --- |
| 2 s before | -0.02 m | -0.02 m | -0.01 m, 0 |
| +2 s | -0.03 m | -0.03 m | -10.01 m, 2.53: the fix is rejected |
| +6 s | -0.01 m | -0.01 m | -10.00 m, 2.28 |
| +10 s | -0.01 m | **+9.99 m** | core 1 has reset onto the GPS |
| +14 s to +59 s | **+9.97 to +10.00 m** | +9.97 to +10.00 m | about 0: both cores are on the stepped GPS |
| step removed, +63 s to +67 s | +9.97 to +9.99 m | +9.97 to +9.99 m | +9.97 m, 2.48: rejected again |
| +71 s | -0.01 m | -0.01 m | reset back |

The flight controller said `EKF3 lane switch 1` 0.2 s after the step and `EKF3 lane switch 0` 10 s after its removal, and nothing else. So ArduPilot 4.6.3's EKF3, on source set 1, rejects a 10 m step for about 10 s and then resets onto it: a step that stays is followed.

## The vehicle did not, because this sim does not fly on EKF3

`AHRS_EKF_TYPE` is 10 in every sim flight looked at (logs 300, 380, 424 and 465, ArduCopter 4.6.3): SITL's own simulated estimate, which is the simulator's true state. EKF3 runs beside it and is logged, and nothing is flown on it. The aircraft has `AHRS_EKF_TYPE 3` (`platform/fcu/comp_2026.param`). The value comes with the simulator's default parameters; nothing in this repo sets it.

That is why the vehicle's position (`POS`) stayed within 0.01 m of the truth through the whole step, and why the bucket was set down 0.006 m from the true return point on a global setpoint with the raw GPS 10 m off (released 52 s after the step, 8 s before it was removed).

## What this changes

- **The note of 24 November is wrong in its reason.** "The flight controller's position estimate did not follow the step, the simulated estimator's own doing" (`docs/results/2026-11-24-gps-health.md`, and the REC-6 row under "Needs a person"): the estimator that would fly the aircraft followed it after 10 s; the sim's vehicle is flown on the truth. On the aircraft, on source set 1, a 10 m step that stays moves the position estimate 10 m about 10 s later, and a mission on a global setpoint goes with it.
- **No sim flight so far says what the flight controller's navigation does with a bad GPS, with source set 2, or with optical flow.** That covers the 5 m glitch of `docs/results/2026-11-18-gps-faults.md` ("the autopilot's estimator did not follow"), the marker flights' control in `docs/results/2026-11-24-visual-homing.md`, gate G5's glitch (the vehicle was never moved by it), and "no EKF failsafe in 18 s on set 2 with no flow" in the 24 November file: the vehicle was on the truth each time. Those files are records of their flights and are left as they are.
- **Not changed here:** the sim's `AHRS_EKF_TYPE`. Flying the sim on EKF3 is a change to every scenario and every gate's baseline, and is the lead's to decide ("Needs a person").
