# GPS faults in the sim (SIM-04)

Measured on 18 Nov 2026 (UTC) on a club laptop, on `platform/build-2027` at `159fcf9` plus the SIM-04 change. Sim image `argus-sim:humble` (ArduCopter 4.6.3 SITL), headless.

**Result: PASS.** A 5 m north glitch, commanded 25 s after arming in a 10 m hover, moved the raw GPS fix 5.00 m north and 0.00 m east 0.23 s after the command (5 m to within 0.5 m, inside 1 s).

## The pass flight

```
$ sim/run_scenarios.sh hover/gps_glitch --seeds 1 --variants off
| hover/gps_glitch | 1 | OK | 1.00 | held within 0.01 m for 73 s | not run |  |  |
| GPS faults | 2 of 2 changes made; 1 of 1 glitch(es) seen in the raw fix; the first moved the raw fix +5.00 m north, +0.00 m east 0.23 s after the command |
```

`sim/inject.py` ran beside the flight on its own MAVLink port. Its log (`gps_events.jsonl`; `t` is sim time):

```
{"t": 53.425, "since_arm_s": 25.031, "event": "glitch", "what": "on", "at_s": 25.0, "north_m": 5.0, "set": {"SIM_GPS_GLITCH_X": 4.4915760491198755e-05, "SIM_GPS_GLITCH_Y": 0.0, "SIM_GPS_GLITCH_Z": 0.0}, "confirmed": true}
{"t": 53.652, "event": "glitch_seen", "passed": true, "after_s": 0.23, "moved_north_m": 4.998, "moved_east_m": 0.0}
{"t": 63.413, "since_arm_s": 35.016, "event": "glitch", "what": "off", "at_s": 35.0, "set": {"SIM_GPS_GLITCH_X": 0.0, "SIM_GPS_GLITCH_Y": 0.0, "SIM_GPS_GLITCH_Z": 0.0}, "confirmed": true}
```

| | |
| --- | --- |
| Parameter names this SITL answered to | `SIM_GPS_GLITCH_X/Y/Z`, `SIM_GPS_NOISE`, `SIM_GPS_NUMSATS`, `SIM_GPS_JAM`, `SIM_GPS_DISABLE` (the 4.6 family; none of the `SIM_GPS1_` names) |
| Armed, by the injector (first armed heartbeat) and by the runner's trace | sim time 28.39 s and 28.59 s |
| Glitch commanded | 25.03 s after the injector's arming time; the flight controller confirmed the three parameters |
| Raw fix (`GPS_RAW_INT`) before and after | moved 4.998 m north, 0.000 m east, first seen 0.23 s after the command |
| Glitch removed | at 35.02 s, confirmed |
| The vehicle's true position from 24 s to 45 s after arming | moved 0.002 m: the autopilot's estimator did not follow the 5 m step in its 10 s |

The raw fix is read at 10 Hz from the flight controller, so "0.23 s" is the first message that showed the step, not the instant it happened.

## The other four kinds

```
$ sim/run_scenarios.sh hover/gps_faults --seeds 1 --variants off
| hover/gps_faults | 1 | OK | 1.00 | held within 0.01 m for 73 s | not run |  |  |
| GPS faults | 8 of 8 changes made |
```

Each fault was switched on for a few seconds in the same hover. The flight controller confirmed all eight parameter changes, and the injector noted the raw fix one second after each:

| Fault | Parameter set | Raw fix before | One second after "on" | One second after "off" |
| --- | --- | --- | --- | --- |
| `noise`, 2 m, at 16 s for 5 s | `SIM_GPS_NOISE 2` | altitude 225.19 m | altitude 223.97 m | 225.19 m |
| `satellites`, 5, at 23 s for 5 s | `SIM_GPS_NUMSATS 5` | 10 satellites | 5 satellites | 10 |
| `jam`, at 30 s for 3 s | `SIM_GPS_JAM 1` | 10 satellites, 3D fix (type 6) | 3 satellites, no fix (type 1) | 10, type 6 |
| `dropout`, at 36 s for 3 s | `SIM_GPS_DISABLE 1` | 10 satellites, type 6 | 3 satellites, no fix (type 1) | 10, type 6 |

The vehicle held within 0.01 m throughout and no failsafe was counted; the autopilot logged "PrecLand: Failsafe Measures" once, during the jamming. The scenario is not in the nightly suite: what the autopilot does about jamming or a dropout is not this check's to judge.

## A first flight that was 6 s late

The first flight of this scenario also showed the step (4.998 m, 0.14 s after the command), but the injector put the glitch 31 s after arming, not 25: it was still waiting for the parameter names of the other ArduPilot family, which never answer, when the vehicle armed, and took the first heartbeat it read afterwards as the arming. Seen by comparing its arming time (36.51 s) with the trace's (30.42 s). The name lookup now ends as soon as every fault has a name and heartbeats are heard throughout; the flight above is after that fix.

## Not shown

- **Longer or combined faults.** Each of the other four kinds was on once, for 3 to 5 s. `noise` is SITL's own: an error on the GPS altitude of the given amplitude; 4.6.3 has no parameter for horizontal noise.
- **A mission reacting to a fault.** The hover holds position on the autopilot alone. What the 2027 missions do when GPS misbehaves is REC-8's and the recovery lane's to fly.
- **The coordinate error** (the other effect in the spec) is the scenario's `coordinate_error`, applied by the builder since SIM-01 and covered by its tests; no flight was added for it here.
- **A later ArduPilot's names.** Only 4.6.3 was flown. The later family's names in the injector's table come from the spec for the glitch and by analogy for the rest; they have not been checked against a later SITL (its dropout parameter may be an enable with the opposite sense). A name that is missing stops the injector, and the run is INVALID.
- One flight, one seed.

## Commands

```bash
python3 -m pytest -q sim/test/test_inject.py
sim/run_scenarios.sh hover/gps_glitch --seeds 1 --variants off
sim/run_scenarios.sh hover/gps_faults --seeds 1 --variants off
cat <run folder>/gps_events.jsonl
```

Run folders (not committed): `sim/.state/suites/hover-gps_glitch-20261118T071229Z/`, `sim/.state/suites/hover-gps_faults-20261118T072007Z/` and, for the late one, `sim/.state/suites/hover-gps_glitch-20261118T070921Z/`.
