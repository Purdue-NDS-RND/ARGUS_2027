# Readiness kit (OPS-02): the geofence demonstration in the sim

2026-11-23, build machine, ArduCopter SITL 4.6.3, the placeholder site (`sim/sites/apollo_xi.yaml`: the east fence 150 m from the takeoff point, `FENCE_MARGIN` 2 m, `AVOID_ENABLE` 3 as on the aircraft in June 2026). The scenario `geofence/readiness_demo` is the demonstration as it will be flown: `tools/go.py` with the profile `fence_demo` and the sequence `fence_demo.yaml`, recorded and bundled as an attempt, and `tools/readiness_kit.py` run on the bundle.

```
$ sim/run_scenarios.sh geofence/readiness_demo --variants on
   OK          geofence/readiness_demo seed 1 on: 1.00 points, held 1.87 m inside the fence for 37 s
1 runs: 1 OK
```

The run counts only if the geofence judge passes it on the true position (here: within 4 m of the line, inside, for 4 s or more, never across), the attempt's bundle is whole, and the kit says READY on that bundle (`sim/attempt_check.py` runs it).

The mission's own lines, and the `rtl` mission after it finding the vehicle in RTL already:

```
[fence_demo] in 3 s: 5.0 m/s on bearing 90 until the fence acts; the boundary is 150 m away and the run fails at 160 m
[fence_demo] run: velocity (+5.00, +0.00) m/s east, north
[fence_demo] held at the fence: stopped 148.0 m into the run, 2.0 m short of the boundary, for 30 s with 5.0 m/s still commanded. Asking for RTL
[fence_demo] the mode is RTL: on the way home after the hold at the fence. The mission commands nothing
[fence_demo] over: the vehicle is disarmed, in RTL
[rtl] State: REQUESTING_RTL -> COMPLETE
```

`readiness.md`, written into the bundle by `tools/readiness_kit.py` from the flight controller's log:

```
# Readiness: the geofence demonstration, 2026-11-24_readiness_geofence

Written by `tools/readiness_kit.py` from the attempt's bundle. Verdict: **READY**.

Flight controller: ArduCopter V4.6.3 (92b0cd78). Log: `dataflash/00000357.BIN`.

## What the bundle and the flight controller's log show

- the bundle is complete: the bag, the flight controller's log, the config and the preflight's log
- every file is as the manifest recorded it
- the preflight said GO, with no override
- the flag profile flown was fence_demo, and nothing else was on
- the code was at comp-2026-106-g0bdf615-dirty with uncommitted changes
- FENCE_ENABLE is 1: the fence was on
- FENCE_ACTION is 1: a breach ends the flight
- FENCE_TYPE 5 has the polygon, and 6 fence points were stored
- from 114.1 s, for 30.2 s, the flight controller's avoidance held the vehicle still with 5.0 m/s commanded, in GUIDED: no breach
- it was held 1.9 m inside the fence line, by the polygon of 4 corners the log itself holds
- 0.0 s after the hold the flight controller was in RTL (mode change reason 2), on the way home
```

## What the flights showed about ArduCopter's fence

| Flight | Commanded | What the flight controller did |
| --- | --- | --- |
| The mission, first version (gave up when the run stalled) | 2 m/s east | Slowed the vehicle and stopped it at 148.0 m, 2 m inside the line; no breach, no mode change. The mission failed its run and the failure policy's RTL brought it home. |
| The mission, at 5 m/s (gave up 5 s into the stall) | 5 m/s east | The same: stopped at 148.0 m. |
| `sim/fence_check.py` (`geofence/breach`, SIM-02's check: the same velocity sent over MAVLink directly) | 5 m/s east | Stopped at 148.1 m for 14 s, then went over the line with the velocity still commanded: `Polygon fence breached`, RTL 0.09 s later, 1.05 m outside at most. |
| The mission, final (commands through the hold for 30 s) | 5 m/s east | Stopped at 148.0 m and stayed there the whole 30 s (1.87 m inside the line by the true position); the mission asked for RTL. Flown twice: before and after the flight-safety review's changes (the run above is the second). |

- The stop is the flight controller's avoidance: its log has `SA` messages for the whole hold with the allowed velocity at zero, while the guided target (`GUIP`) stays at 5 m/s. The kit reads the hold from those two and the vehicle's distance inside the fence polygon the log itself holds (`FNCE`).
- Why the check's flight crossed after 14 s and the mission's did not in 30 s is not known. Both command a velocity-only target in GUIDED; the mission's goes through MAVROS at 20 Hz with a heading. The mission and the kit take either ending.
- The breach flight's log confirmed what the kit looks for in that case: an `ERR` of subsystem 9 with code 4 (polygon), and `MODE` 6 (RTL) with reason 10 at the same instant.
- The first flights also showed that a geofence run flown by the mission stack was counted OK when the judge had said the fence was never reached; the result now fails such a run, and the judge passes a vehicle held inside the line only for a scenario that asks for it (`held_within_m`).

## The kit on a real breach

The flight of `sim/fence_check.py` that crossed (`sim/run_scenarios.sh geofence/breach`, SITL's own log file of it) was put through the kit's reading of a log (`readiness_kit.read_log` and `judge_log`):

```
- FENCE_ENABLE is 1: the fence was on
- FENCE_ACTION is 1: a breach ends the flight
- FENCE_TYPE 5 has the polygon, and 6 fence points were stored
- before the breach the flight controller's avoidance held the vehicle still for 15.0 s with 5.0 m/s commanded
- at 107.4 s the flight controller reported a breach of the polygon
- the vehicle was in GUIDED, under the companion computer's command, when it was breached
- 0.00 s later the flight controller was in RTL by its own fence action (mode change reason 10)
- 3.4 s after the breach the vehicle was back inside the fence
```

The first reading of that log was wrong, and the spec audit had predicted how: ArduCopter writes the mode change before the error message, with the same time stamp, so the kit took the fence's own RTL for the mode the breach happened in and said "in RTL at the breach, not in GUIDED". The kit now looks for the fence's answer from just before the error, takes the mode before that answer as the one breached in, and counts an answer only if its mode change reason is the fence's (10): a pilot's or a companion's RTL after a breach is not the flight controller's own. The test logs are built in the real order.

## Not shown here

- The aircraft. Its avoidance parameters in June 2026 were SITL's (`AVOID_ENABLE` 3, `AVOID_MARGIN` 2, `AVOID_BEHAVE` 0), so the hold is what to expect, but nothing here was flown on it.
- A breach through the mission stack: the mission's flights were all held. That ending is unit-tested in the mission; the kit's reading of it was tried on the check's real breach log (above), which is not a bundle and is not kept in the repo.
- The video itself, and what the organisers want in it (open question 18).
