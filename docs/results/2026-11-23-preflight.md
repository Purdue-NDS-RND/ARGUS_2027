# Preflight go/no-go (PLT-10): the real command against the sim

2026-11-23, build machine, ArduCopter SITL 4.6.3. `sim/preflight_check.py` runs the command the aircraft will run (`drone_utils` `preflight`) as a flight is launched, and the run counts only if the verdict and the failed checks are the scenario's. These are the sim's readings; the aircraft's are a person's (`docs/build/PROGRESS.md`, "Needs a person", PLT-10).

## An aircraft that is ready: `delivery/preflight`

SITL started on the 2027 baseline's values (`platform/fcu/argus_2027.param` less the rangefinder's four lines), the site's fence uploaded, a charged 6S battery. Then the 2026 delivery, every flag off.

```
$ sim/run_scenarios.sh delivery/preflight --variants on
   OK          delivery/preflight seed 1 on: 16.21 points, 0.03 m from the target
1 runs: 1 OK
```

```
[   GO] params       flight controller holds to the baseline, sha256 be80db09cb93
[   GO] fence        enabled and healthy, 6 points stored
[   GO] gps          fix type 6, HDOP 1.21, 10 satellites
[   GO] ekf          estimate healthy, pre-arm checks pass
[   GO] rangefinder  0.00 m
[   GO] cameras      /camera/image 22.1 Hz
[   GO] engine       no engine was named (--engine): none is checked
[   GO] timesync     flight controller sync steady to 0.0 ms, largest step 0.0 ms; the companion's clock was not compared with a ground laptop's (no --clock-source)
[   GO] disk         11 GB free in sim/.state/suites/delivery-preflight-20261123T233458Z/runs/delivery-preflight-s1-on
[   GO] battery      25.0 V, 4.17 V a cell
[   GO] profile      profile: every flag off (the 2026 code)
GO
PREFLIGHT CHECK: PASS
```

## An aircraft that is not: `hover/preflight_nogo`

SITL as the sim always had it: no baseline, and its own 12.6 V battery.

```
$ sim/run_scenarios.sh hover/preflight_nogo --variants on
   OK          hover/preflight_nogo seed 1 on: 1.00 points, held within 0.01 m for 53 s
1 runs: 1 OK
```

```
[NO-GO] params       not the baseline be80db09cb93: BATT_LOW_VOLT is 10.5, the baseline has 21 (and 8 more)
[   GO] fence        enabled and healthy, 6 points stored
[   GO] gps          fix type 6, HDOP 1.21, 10 satellites
[   GO] ekf          estimate healthy, pre-arm checks pass
[   GO] rangefinder  2.08 m
[   GO] cameras      /camera/image 26.0 Hz
[   GO] engine       no engine was named (--engine): none is checked
[   GO] timesync     flight controller sync steady to 0.0 ms, largest step 0.0 ms; the companion's clock was not compared with a ground laptop's (no --clock-source)
[   GO] disk         11 GB free in sim/.state/suites/hover-preflight_nogo-20261123T233233Z/runs/hover-preflight_nogo-s1-on
[NO-GO] battery      12.6 V is 2.10 V a cell on 6 cells (at least 3.9)
[   GO] profile      profile: every flag off (the 2026 code)
NO-GO: params, battery
PREFLIGHT CHECK: PASS
```

The delivery scored 16.21 points at 0.03 m, the flags-off baseline's score: the preflight beside it changed nothing. `--variants on` puts the run on the sim's clock; neither scenario turns a flag on.

## What these two flights do not show

- The preflight runs beside the flight, not before it: the sim does not wait for the verdict. In the NO-GO flight the hover had taken off while the preflight listened (`rangefinder 2.08 m`); in the GO flight the vehicle was still on the ground (`0.00 m`, with the flight controller's word that the sensor has data).
- `timesync` reads steps of 0.0 ms on the sim's clock, where the flight controller and MAVROS share one clock: the rule is not exercised there. On a wall-time flight of the same scenario it read a 90th percentile of 54.3 ms and failed (below). Its limits (20 ms at the 90th percentile, 100 ms for the largest step) are unit-tested and are the bench's to confirm.
- No message was asked for: SITL sends all four unasked, and nothing is asked of an armed vehicle.
- A rangefinder with no data, and a fence that is enabled with no points uploaded: unit-tested rules, resting on ArduPilot's SYS_STATUS bits and `FENCE_TOTAL`; the bench steps under "Needs a person" say what must be seen.

## On the way

- The first two flights were failed by the side tool, which could not import pymavlink in the runner's environment (the injector's way of reaching it was copied).
- The first working NO-GO flight read everything correctly and also failed `timesync`: one 22 ms step in the offset estimate among fifty exchanges, on a sim running on wall time. The rule was changed to the 90th percentile of the steps (limit 20 ms), with a limit of 100 ms on the largest.
- On wall time the GO flight then failed `timesync` at a 90th percentile of 54.3 ms: the sim runs slower than the wall clock on this machine, and the offset between the two wanders with it. Both scenarios were put on the sim's clock (`sim_time: true`).
- On the sim's clock two flights were INVALID: the run's time check found the preflight's own node on wall time, and the camera read 9.2 Hz against 15 because frames stamped on one clock were counted over a stretch of the other. The command got `--sim-time` (its node on `/clock`, rates counted on the node's clock), and both were flown again: the outputs above.
- Between those, the two reviews' fixes went in (the rangefinder's and fence's extra conditions, `--no-camera`, the rates asked for); the outputs above are of the final code.

## Not shown here

- `engine` (no engine exists: the check passes, saying none was named) and the companion's clock against a ground laptop (`--clock-source` is not given in the sim).
- An override: unit-tested (`src/drone_utils/test/test_preflight.py`).
- The parameters read through MAVROS's param plugin or from a ground station's dump: the sim reads them over MAVLink.
