# SIM-04: GPS faults

Flag: none (sim only; a scenario's `gps_events:` turns it on, and a scenario without them is built, started and flown exactly as before) · Status: DONE · Commits: `git log --oneline --grep '^SIM-04:' platform/build-2027`

## What it solves

A scenario could list GPS events and the runner refused to fly it: nothing could make the vehicle's GPS misbehave. Recovery's "GPS may not be trustworthy" needs both a wrong coordinate (already in the builder) and a GPS that glitches, loses satellites or drops out in flight. This adds the one tool that does the second.

## Architecture

- `sim/inject.py` (new): the fault tool the runner starts beside a flight with `gps_events`. The schedule, the parameter values, the name lookup and the glitch's pass check are plain functions; `run` talks to SITL with pymavlink and reads Gazebo's clock.
- `sim/container/start.sh`: a second MAVLink output (UDP 14553) for the injector, opened only for runs whose scenario has `gps_events`. The tools port stays with drivers and the payload bridge.
- `sim/scenario/spec.py`: events are checked when the run is built (a time, a known kind, only that kind's settings).
- `sim/runner/result.py`: a run with events counts only if the injector's report says every change was made and every glitch showed; one "GPS faults" report row.
- Scenarios `hover/gps_glitch` (in the nightly suite) and `hover/gps_faults` (the other four kinds, once each).

## Flow

Before: a scenario with `gps_events` → "not flown: the sim cannot inject GPS events yet".

After: the runner starts `inject.py` → it asks SITL which of `SIM_GPS_GLITCH_X` or `SIM_GPS1_GLTCH_X` (and the rest) exist, and stops if neither → first armed heartbeat marks time zero → at each event's time the parameters are set and confirmed → for a glitch, the raw fix before and after is compared → at the end everything is put back and `gps_events.json` written → the runner reads it.

## Code

- `inject.py:96`, `resolve_names`: one name per fault from whichever family the running SITL has; an error if a fault has none.
- `inject.py:110`, `changes_for`, and `:129`, `Schedule`: a glitch in metres becomes degrees of latitude and longitude at the vehicle's latitude; each event is an "on" and, with `for_s`, an "off".
- `inject.py:166`, `glitch_result`: the pass line, the first raw fix within 1 s that has moved to within 0.5 m of the step.
- `inject.py:262`, `run`: heartbeats are heard throughout, so an arming during the name lookup is not seen late (it was, by 6 s, in the first flight).
- `runner/result.py:207`: INVALID when the events were not all injected.

## Evidence

```
$ python3 -m pytest -q -p no:cacheprovider sim/test/test_inject.py
18 passed in 0.11s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

Sim ([docs/results/2026-11-18-gps-faults.md](../../results/2026-11-18-gps-faults.md)):

```
$ sim/run_scenarios.sh hover/gps_glitch --seeds 1 --variants off
| hover/gps_glitch | 1 | OK | 1.00 | held within 0.01 m for 73 s | not run |  |  |
| GPS faults | 2 of 2 changes made; 1 of 1 glitch(es) seen in the raw fix; the first moved the raw fix +5.00 m north, +0.00 m east 0.23 s after the command |
```

The pass line: 4.998 m north, 0.000 m east, 0.23 s after the command, against 5 m to within 0.5 m inside 1 s.

```
$ sim/run_scenarios.sh hover/gps_faults --seeds 1 --variants off
| hover/gps_faults | 1 | OK | 1.00 | held within 0.01 m for 73 s | not run |  |  |
| GPS faults | 8 of 8 changes made |
```

Noise, satellite count, jamming and dropout: each confirmed by the flight controller and seen in the raw fix a second later (altitude 1.2 m off; 5 satellites; no fix; no fix).

## Spec check

spec-auditor, one round: INCOMPLETE on one row, every other row DONE: four of the five fault kinds had never been sent to a SITL. Flown after it (`hover/gps_faults`), with the raw fix logged around each change; not reviewed again (a scenario and logging, no rule changed). A wrong run-folder name in the results file was corrected.

flight-guardian did not run: nothing under `src/` and no flight config changed.

## Not done

- **A mission reacting to a fault**: REC-8 and the recovery lane. The hover is flown by the autopilot alone.
- **A later ArduPilot's parameter names** are unchecked beyond 4.6.3; a missing name stops the injector and the run is INVALID.
- No flight was added for the coordinate error, which the builder has applied since SIM-01.
- One flight of each scenario, one seed.
