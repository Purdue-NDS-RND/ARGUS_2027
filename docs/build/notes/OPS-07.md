# OPS-07: Impulse rig (delivery)

Flag: none (an analysis tool and a procedure for a rig that is not built yet) · Status: PARTIAL · Commits: `git log --oneline --grep '^OPS-07:' platform/build-2027`

## What it solves

The delivery will be scored against a contact impulse limit that the organisers have not given. When they do, the team must know within a day how fast the package may be set down. This is the measurement's analysis and procedure, ready before the limit and before the rig.

## Architecture

- `tools/impulse_rig.py` (new, no ROS, standard library only): reads set-down recordings from a plate on a load cell (`t_s,force_n`), one per trial with its descent speed.
- `docs/ops/impulse_rig.md` (new): how to build the rig, the procedure, how to read the result.

## Flow

- Before: the set-down speed (0.12 m/s) is a number nobody has held against a force.
- After: three or more set-downs on the plate → `impulse_rig.py --trial <file>:<speed> ... --package-kg 0.5 --limit-ns <limit>` → a table (impulse, peak, contact time, the mass the momentum implies) → the impulse and the peak at 0.12 m/s → the fastest descent under the limit, by the worst trial with a fifth in hand → exit code 1 when 0.12 m/s is over it.

## Code

- One set-down: the touch, rest, and the integral of force over the weight between them: `tools/impulse_rig.py:106`.
- The fits and the fastest descent under a limit, and set-downs that bound none: `:145`.
- A recording too slow to show the peak is refused: `:130`.

## Evidence

```
$ pytest tools/test/test_impulse_rig.py   (as scripts/check.sh runs it)
11 passed
```

On generated set-downs whose momentum is known: the impulse comes back as mass times speed within 2%, the peak as the pulse's; twice the speed gives twice both; a limit as an impulse or as a peak force gives the fastest descent, and the stricter of the two sets it. Refused with the reason: 80 samples a second, a recording that never settles, one that starts on the touch. The tests were written after the tool.

A set-down in which nothing knocks (the weight comes on in a straight line, as when the vehicle holds the package until it stands) bounds no speed and is said so, not called over the limit; a file of an accelerometer's readings is refused.

## Spec check

spec-auditor, two rounds: INCOMPLETE, as a PARTIAL row is (there is no rig). Round one found the accelerometer path wrong for any real recording (taken out: only a load cell's recording is read, recorded as a decision) and a set-down with no knock answered with a negative speed and "over the limit" (now said to bound no speed). Round two: both DONE; these notes brought up to date.

## Not done

- The rig (why PARTIAL): no plate, no load cell, no recording. The tool has seen only generated signals; a real plate rings, and a real set-down is on a line ("Needs a person").
- The spec's other option, an accelerometer on a free plate, is not read and has no analysis: only a load cell's recording is (Decisions).
- The fit is a straight line through zero: a "fastest descent" far beyond the fastest speed tried is that line carried on, not a measurement.
- The limit and how the organisers measure it are their open answer (question 9): the tool takes a limit as an impulse or as a peak force.
- Gate G3 stays FAIL: it needs the limit.
- The delivery's set-down speed is not changed by anything here.
