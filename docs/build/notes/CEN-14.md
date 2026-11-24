# CEN-14: Release confirmation

Flag: none (read from the flight controller's log after the flight; nothing in flight changes) · Status: PARTIAL · Commits: `git log --oneline --grep '^CEN-14:' platform/build-2027`

## What it solves

A release command that was sent is not a payload that left: a servo output that is not assigned, a latch that does not open, or a command the flight controller refused all look the same to the mission. After the flight nobody could say from a record which it was.

## Architecture

- `tools/release_confirm.py` (new, no ROS, standard library): the rules. Acknowledged (the command's result in the log), the output moved (the servo output with the gripper's or the drop servo's function), and the payload left, by the step in the air, by the lighter flight afterwards, or by the rangefinder seeing something fall away.
- `tools/post_flight.py`: reads the servo outputs and the throttle from the log, puts the confirmation into `report.md` and `report.json`, and writes `release_confirmation.json` into the attempt's bundle.
- `sim/attempt_check.py`: for a drop or delivery flown as an attempt, writes the report and fails the run when its confirmation is not what the sim's payload bridge did.
- Scenarios `drop/attempt_release`, `drop/attempt_release_empty`, `delivery/attempt_release`, `delivery/attempt_release_empty`.

## Flow

- Before: `[package_drop] Closing sprayer for payload drop`, and a hope.
- After: the flight → the bundle → `python3 tools/post_flight.py <bundle>` → `**RELEASE CONFIRMED**` with the command's result, the output's move and the measured throttle step, or `**RELEASE NOT CONFIRMED**: nothing shows the payload leaving`.

## Code

- The output that a command moves, and whether it moved (a sample with the command's own time stamp counts as after): `tools/release_confirm.py:86`.
- The step in the air: `:120`; steady flight before against after: `:144`; the rangefinder: `:161`; the verdict: `:181`.
- Into the report and the bundle: `tools/post_flight.py:220`. Against the sim's truth: `sim/attempt_check.py:123`.

## Evidence

```
$ pytest tools/test/test_release_confirm.py sim/test/test_attempt_check.py tools/test/test_post_flight.py   (as scripts/check.sh runs them)
10 passed, 22 passed, 23 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh drop/attempt_release --variants on          -> OK; the report: RELEASE CONFIRMED (throttle fell 11.0%, rose at 0.16 m/s)
$ sim/run_scenarios.sh drop/attempt_release_empty --variants on    -> OK; the report: RELEASE NOT CONFIRMED (throttle fell 0.1%)
$ sim/run_scenarios.sh delivery/attempt_release_empty --variants on -> OK; NOT CONFIRMED (steady throttle rose 3.7%)
$ sim/run_scenarios.sh delivery/attempt_release --variants on       -> OK; RELEASE CONFIRMED (steady throttle fell 7.7%)   [the second flight; the first found a fault, below]
```

Numbers and what they do not show: `docs/results/2026-11-23-release-confirmation.md`. The rules were first tried on SITL's own logs of two real releases, then the tests were written, then the four flights were flown; the first delivery flight found a fault in the output rule (below).

## Spec check

- spec-auditor, one round: INCOMPLETE on the one row that was still in the air (the carried delivery's flight), now flown and recorded. Its other findings: the rangefinder evidence is unit-proven only (the row closes PARTIAL and the evidence says so); a command row with no result column was read as accepted (now read as not acknowledged, with a test). Recorded as deviations under "Decisions": the camera's frame difference is not built, the evidence that is flown (the throttle) is not one the spec names, and the check runs after the flight. No second round: the fixes were one default and a results row.
- The first flight of `delivery/attempt_release` failed, and the failure was the check doing its work: the sim had let the package go and the report said NOT CONFIRMED. The output rule took a sample logged with the command's own time stamp for "before". Fixed, tested, flown again.
- No flight-guardian review: nothing under `src/` and no config changed.

## Not done

- The camera's frame difference, which the spec names beside the rangefinder: not built ("Decisions"). The sim's package cannot be seen by either sensor, and the drop and delivery log no frames.
- The rangefinder evidence has only unit tests.
- The thresholds on the aircraft (why PARTIAL, with Field): a dummy payload on the bench and one flight each way ("Needs a person").
- Nothing acts on the result in flight; the report says it after the landing.
