# SIM-11: Acceptance gates G0 to G8

Flag: none (sim tooling). G0 flies with every flag off; G1 to G6 will fly with flags on · Status: DONE · Commits: `git log --oneline --grep '^SIM-11:' platform/build-2027`

## What it solves

The gate table in the build plan was prose: someone had to decide which flights a gate means and whether the numbers pass. Now each gate is data (its flights and its rules) and one command gives PASS, FAIL or INVALID with every number it used. This fixes the cause: a gate's verdict no longer depends on who reads the logs. Only G0 can fly today; G1 to G6 have their rules but wait for their lanes' scenarios.

## Architecture

- `sim/scenarios/gates.yaml`: G0 to G8, each with the spec's "proves" and "pass" text, its flights (or what it waits for) and its rules.
- `sim/gate.py` (ROS-free): `list`, `flights <gate>`, `check <gate> <suite folder>... [--write <result file>]`.
- `sim/runner/suites.py`: a gate name is a suite (`sim/run_scenarios.sh G0`), `gates` is all of them; a gate's seeds and variants cannot be overridden.

No new dependency.

## Flow

Before: G0 was "delivery touchdown ≤ 0.3 m in 5/5", checked by hand from a mission log.

After: `sim/run_scenarios.sh G0` flies `delivery/baseline` with seeds 1 to 5, flags off. `python3 sim/gate.py check G0 <suite folder>` reads each run's `result.json`, `score.json` and `scenario.json`, takes the first run of each flight that is not INVALID, applies the rule `each details.distance_m ≤ 0.3`, and prints `G0: PASS` with the five distances. `--write` produces the result file the `/gate` step commits.

## Code

- `sim/gate.py` `check`: a missing or unfinished flight, or an every-run rule broken by a judged run, is FAIL; a flight with only INVALID runs makes the gate INVALID when nothing is already lost; the first run that counts is the one judged, so a lost flight cannot be flown again into a pass.
- `sim/gate.py` `evaluate_rule`: the rule kinds `each`, `count`, `median`, `maximum`, `minimum`, `p95`, `ratio` and `flown`, with `where` and `pooled`; a metric may be a judge setting (`settings.<name>`), which pins a limit a scenario could loosen. A run that did not finish OK or has no value never counts in a rule's favour; an unknown kind or limit is an error.
- `sim/gate.py` `render_result`: the result file, with commit, command, the flags that were on (mission and node flags), every run and what each rule found.
- `sim/scenarios/gates.yaml`: the rules for G1 to G6, written against what the judges report, so each lane knows what its gate will read.

## Evidence

```
$ python3 -m pytest -q -p no:cacheprovider sim/test/test_gates.py sim/test/test_runner.py
122 passed in 6.81s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh G0
| delivery/baseline | 1 | OK | 16.23 | 0.02 m from the target | not run |  |  |
| delivery/baseline | 2 | OK | 16.23 | 0.01 m from the target | not run |  |  |
| delivery/baseline | 3 | OK | 16.22 | 0.02 m from the target | not run |  |  |
| delivery/baseline | 4 | OK | 16.19 | 0.04 m from the target | not run |  |  |
| delivery/baseline | 5 | OK | 16.22 | 0.03 m from the target | not run |  |  |

5 runs: 5 OK
$ python3 sim/gate.py check G0 sim/.state/suites/G0-20261115T193742Z
G0: PASS
  ok   every touchdown within 0.3 m of the target: 5 of 5 ≤ 0.3 (needs 5): 0.01987, 0.01492, 0.02357, 0.04486, 0.02635
```

That was a trial of the gate machinery with this feature uncommitted, and the first live suite of more than one flight. The gate's recorded run is the `/gate G0` step that follows, at the committed code. `test_gates.py` was run before `sim/gate.py` existed and failed (`ModuleNotFoundError`). The tests also run on Python 3.10 (the sim image), the version CI uses.

## Spec check

spec-auditor: INCOMPLETE in round 1. G0 and the machinery were done, but it found a regression (a scenario named by a relative file path no longer resolved), no test pinning the G1 to G6 limits, pass-line parts that no rule checked (G5's ten runs, G6's five flights and moderate wind), limits held only by judge defaults a scenario could loosen, and a result file that would leave out node flags. COMPLETE in round 2: the path fix, a both-sides case for every shipped rule, the `flown` kind and `fraction` limit, rules pinning the judge's settings, and node flags in the result file.

flight-guardian did not run: no mission, executor, target CV or `config/` YAML changed.

What the rules cannot check, and the three metrics no judge reports yet, are under Decisions and in "Not done".

## Not done

- G1 to G6 have rules but no flights: their scenarios need wind (SIM-03), GPS faults (SIM-04), payload physics (SIM-06) and the mission lanes. Each lane adds `scenarios`, `seeds` and `variants` to its gate in `gates.yaml`.
- Two pass-line conditions are not checkable by a rule and are owed by the lanes' scenarios: G1's hold heights, G5's coordinate error and glitch (Decisions).
- Three rules read a metric no judge reports yet: G1 `details.hold_error_p95_m` (a hold judge, CEN lane), G5 `details.ended_safely` (REC lane), G3 `details.impulse_ok` (needs `contact` events, SIM-06).
- G7 and G8 need field data and a real flight; the tool says so and does not check them.
- Flying an INVALID flight again is by hand: fly the gate again and give `check` both folders.
