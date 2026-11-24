# OPS-02: Readiness kit (May 5 video and logs)

Flag: `fence_demo` (mission flag, default false) for the demonstration flight; the kit and the checklist are tools and a document · Status: PARTIAL · Commits: `git log --oneline --grep '^OPS-02:' platform/build-2027`

## What it solves

By 5 May the team must show, on video and in logs, that the aircraft's geofence works. Nothing flew at a fence under the autonomous stack, nothing said what the flight controller's log has to contain to prove it, and nobody had written down what to film.

## Architecture

- `drone_mission_core/fence_demo.py` (new, ROS-free rule) and the mission `fence_demo` (`missions/fence_demo_mission.py`): hold, then one velocity along a compass bearing towards the flight boundary, commanded until the flight controller ends the run.
- `config/sequences/fence_demo.yaml` (takeoff, the demonstration, an `rtl` for the flag-off flight; every flag false: as shipped the mission is skipped) and `profiles/fence_demo.yaml` (the one flag).
- `tools/field_setup.py`: the day file's `fence_demo: {bearing_deg}` becomes the sequence's `bearing_deg`, `boundary_m` and `max_run_m` (the boundary's distance on that bearing, and 10 m past it) and the takeoff point's coordinates.
- `tools/readiness_kit.py` (new): reads the demonstration's attempt bundle (PLT-12, mission `geofence`) and writes `readiness.md` into it: READY or NOT READY, from the flight controller's own log. `tools/dataflash_params.py` gained a reader for any message of a log.
- `docs/ops/readiness_video.md` (new): the procedure and the video checklist.
- Sim: scenario `geofence/readiness_demo`; the geofence judge passes a vehicle held inside the line for a scenario that asks for it; a fence run flown by the mission stack fails when the judge does not pass it.

## Flow

- Before: no way to show the fence under autonomy.
- After: `go --profile fence_demo --sequence .../fence_demo.yaml --record <folder>` → take-off, 3 s hold, 5 m/s on the bearing → the flight controller stops the vehicle at the fence (held 2 m inside in the sim; the mission pushes for 30 s and then asks for RTL), or the vehicle goes over and the fence's own RTL takes it (the mission goes quiet) → landing → `dataflash_pull.py`, `bundle_attempt.py --mission geofence`, `readiness_kit.py <bundle>` → `readiness.md` and `READY`.

## Code

- The rule, with the push at the fence, the two endings and the cross-track limit: `src/drone_mission_core/drone_mission_core/fence_demo.py:129`.
- The mission: a failure is a stop first `src/drone_mission_demo/drone_mission_demo/missions/fence_demo_mission.py:130`; quiet when the mode is not its own `:158`; flown only from the takeoff point `:209`; the hold and the RTL it asks for `:221`.
- The run's bearing, how square it is to the edge, and its length, from the day file: `tools/field_setup.py:216`.
- The kit: the hold read from the log `tools/readiness_kit.py:150`, how far inside the polygon `:171`, the verdict on the log `:224`.

## Evidence

```
$ pytest test_fence_demo.py test_fence_demo_mission.py test_readiness_kit.py test_field_setup.py test_attempt_check.py   (as scripts/check.sh runs them)
17 passed, 17 passed, 26 passed, 40 passed, 10 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh geofence/readiness_demo --variants on
   OK          geofence/readiness_demo seed 1 on: 1.00 points, held 1.87 m inside the fence for 37 s
[fence_demo] held at the fence: stopped 148.0 m into the run, 2.0 m short of the boundary, for 30 s with 5.0 m/s still commanded. Asking for RTL
[fence_demo] over: the vehicle is disarmed, in RTL
ATTEMPT CHECK: PASS        (the bundle whole, and tools/readiness_kit.py on it: READY)
```

Five fence flights were needed to find out what ArduCopter does at its fence, and the mission, the judge and the kit were changed on what they showed: `docs/results/2026-11-23-readiness-demo.md`. The rule's and the mission's tests were written with their modules; the tests of the hold, the push and the kit's hold path were written after the flights that showed them.

## Spec check

- spec-auditor, round 1: INCOMPLETE on three rows. Fixed: the kit's breach reading was wrong on a real breach log, exactly as the audit predicted (ArduCopter writes the mode change before the error, with one time stamp), and it now also requires the mode change to be the fence's own; the kit's READY is part of the sim scenario's pass; the results file and this note. No second round: the fix was confirmed on the real log.
- flight-guardian, round 1: CONCERNS, no blocker, five rows, all acted on. A failure is now a position hold commanded for 2 s before it is reported (the executor clears a failed mission's setpoints, and the last velocity target would have been flown on); the run is flown only from within 10 m of the surveyed takeoff point; a cross-track limit of 10 m and a bearing within 15 degrees of square to the edge (the avoidance slides along a fence met at an angle); the stop past the boundary is 10 m, not 15; a hand-back in the air fails the mission so that the policy asks for the way home; a vehicle the pilot landed is left alone; `rtl_mode` is checked; the sequence ends with an `rtl` mission for the flag-off flight. Left, and in the checklist: the mission cannot see whether the flight controller has a fence (the preflight can; the flight is started through `go`, never with an override), and a fence action that does not fly home leaves the vehicle to the pilot. No second round: tested rules, and the scenario was flown again on them.

## Not done

- The aircraft (why PARTIAL): the demonstration flown and filmed, with the day's real coordinates and a bearing chosen on the field ("Needs a person").
- What the video must show is open question 18: the checklist is the session's reading.
- A breach through the mission stack was not flown: every mission flight was held. The breach ending is unit-tested, and its log signature is from `sim/fence_check.py`'s flight. Why that flight crossed and the mission's did not is not known.
- The kit reads the first hold or the first breach of a log. A log with several flights in it (the flight controller not restarted between them) has not been tried.
