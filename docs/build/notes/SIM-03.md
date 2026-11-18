# SIM-03: Wind through applied drag force

Flag: none (sim only; a scenario's `wind:` turns it on, and a calm scenario is built and flown exactly as before) · Status: DONE · Commits: `git log --oneline --grep '^SIM-03:' platform/build-2027`

## What it solves

The sim had no wind: a scenario could say `wind: {level: moderate}` and the runner refused to fly it, so nothing about centring or drop drift could be tried in moving air. This applies our own drag force to the vehicle and to payloads, with gusts, so the same scenario can be flown calm, moderate or strong.

## Architecture

- `sim/wind_model.py` (new, standard library only): `Wind` (mean, Ornstein-Uhlenbeck gusts along, across and vertical, one discrete gust; repeatable from the seed), `drag_force`, `hover_tilt_deg`, and `ForcePlan`, which turns a changing force into messages for a system that only adds.
- `sim/wind.py` (new): the side tool the runner starts beside a windy flight. Reads true poses from Gazebo, applies the force through ApplyLinkWrench at 20 Hz of sim time, writes `wind.jsonl` and `wind.json`.
- `sim/hover.py` (new): a driver that takes off, holds and returns, for the pass flight. A new `hover` judge.
- `sim/scenario/build.py`: a windy run's world gets the ApplyLinkWrench system; the manifest carries the vehicle's mass (1.755 kg) and drag area (0.08 m²).
- `sim/runner/result.py`: a windy run with no record that the wind blew is INVALID; the hover flight must also meet the tilt line; two report rows.
- Scenarios: `hover/wind_5`, `delivery/wind_moderate`, `delivery/wind_strong`, all in the nightly suite.

## Flow

Before: a scenario with wind → "not flown: the sim does not apply wind yet".

After: the runner starts `sim/wind.py` → every 50 ms of sim time, each body's velocity from its last 0.1 s of true poses → the wind then (mean + gusts) → F = ½ ρ CdA |v_rel| v_rel → the change in force since the last update goes to Gazebo as a persistent wrench → the autopilot leans into it. At the end `wind.json` has what the wind was and, where the vehicle hovered, its tilt against atan(F / mg).

## Code

- `wind_model.py:48`, `drag_force`, and `:100`, `Wind`: the force law and the wind; `OrnsteinUhlenbeck.step` uses the exact update, so the gusts' spread does not depend on the step.
- `wind_model.py:144`, `ForcePlan`: Gazebo's persistent wrenches accumulate, so the force is sent as its change, and every 5 s the pile is cleared and the whole force sent again after 10 ms.
- `wind.py:165`, `blow`: the only part that touches Gazebo; `:95`, `summarise`, picks out the hover and compares its tilt.
- `scenario/build.py:190`, `WRENCH_SYSTEM`: added to a world only when the scenario has wind.
- `judge/missions.py:457`, `judge_hover`, and `hover.py:34`, `run_hover`.

## Evidence

```
$ python3 -m pytest -q -p no:cacheprovider sim/test/test_wind_model.py
12 passed in 0.67s
$ python3 -m pytest -q -p no:cacheprovider sim/test/test_wind.py
15 passed in 0.15s
$ python3 -m pytest -q -p no:cacheprovider sim/test/test_judge.py
98 passed in 0.55s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

Sim ([docs/results/2026-11-18-wind.md](../../results/2026-11-18-wind.md)):

```
$ sim/run_scenarios.sh hover/wind_5 --seeds 1 --variants off
| hover/wind_5 | 1 | OK | 1.00 | held within 0.18 m for 74 s | not run |  |  |
| Hover tilt in the wind | PASS: 4.10 degrees against atan(F / mg) = 4.10 degrees (force 1.23 N, mass 1.755 kg, limit 20%) |
$ sim/run_scenarios.sh delivery/wind_moderate --seeds 1 --variants off
| delivery/wind_moderate | 1 | OK | 16.20 | 0.04 m from the target | not run |  |  |
| Wind | mean 3.8 m/s, 0.7 to 6.4 m/s, from 225 degrees, over 3125 updates |
```

The pass line: 4.101° measured against 4.096° expected, 0.1% apart where 20% is allowed.

The drag law was broken once on purpose (half the force): the force test failed, and it was put back.

## Spec check

spec-auditor, one round: INCOMPLETE on two rows, every other row DONE. Fixed after it, not reviewed again (tests and one default): a bucket that names no size now gets the drag area of the bucket the builder builds (0.19 m by 0.20 m; the fallback had been 0.3 by 0.3, a force 2.4 times too large), with a test that ties the defaults to the builder's; the hover judge and its headline have unit tests (holds, drifts, too short, never rises). Stated, not fixed: no flight has had a payload or bucket in a windy world, so the force on a payload is unit-tested only until SIM-06 gives a payload a release.

flight-guardian did not run: nothing under `src/` and no flight config changed.

## Not done

- **A payload dropping in wind** was flown under SIM-06 (`drop/release_wind`): the package came to rest 0.57 m downwind of where it was let go. `docs/results/2026-11-18-payload-release.md`.
- **The strong level** (`delivery/wind_strong`) is in the nightly suite and was not flown in the session.
- **Our airframe's drag area.** 0.08 m² is the Iris's until DAT-10's hover test.
- One flight of each scenario, one seed.
