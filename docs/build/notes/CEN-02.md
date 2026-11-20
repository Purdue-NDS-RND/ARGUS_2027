# CEN-02: Controller retune and offline loop simulator

Flag: `centring_retune` (mission flag, default false; needs `centre_on_estimator`) · Status: DONE · Commits: `git log --oneline --grep '^CEN-02:' platform/build-2027`

## What it solves

Centring on the estimator (CEN-01) still used the 2026 gain and speed cap. The spec's controller (soft deadband, Kp 0.6, a cap that follows the height, a slew limit) replaces them, and a tool lets a gain be tried in a simulated loop before it is flown. Flown in Gazebo, the spec's controller held well in moderate wind and missed gate G1's line in strong wind; a lead term, which the spec does not have, brought it under.

## Architecture

- `drone_mission_core/centring_controller.py` (new, standard library only): `CentringGains` (defaults in code, a mission's `centring_` keys override, bad values refused) and `CentringController.step`.
- `red_bullseye_mission_base.py`: with the flag on, `_estimator_tracking_solution` takes its velocity from the controller (`_retuned_velocity`) instead of the 2026 gain.
- `MissionContext.vehicle_velocity()` and `PoseBuffer.newest_velocity()` (new calls): the vehicle's newest velocity, for the lead.
- `tools/cen_loop_sim.py` (new): the same controller class in a simulated 20 Hz loop.
- Sim: scenarios `drop/hold_{6m,2m}_{moderate,strong}`, which are gate G1's flights (`sim/scenarios/gates.yaml`); the wind fades out on the vehicle in the last half metre (`sim/wind_model.py`).

## Flow

Before (centre on the estimator, retune off): error → 2026 gain (0.15 to 0.25) → 2026 speed cap → velocity.

After (retune on): error → less the vehicle's velocity × 0.5 s (the lead) → less 0.03 m of its length (soft deadband) → × Kp 0.6 → capped at clamp(0.08 × height, 0.25, 1.0) m/s → changed by at most 0.5 m/s² → velocity. Target lost → zero at once, and the next command starts from rest.

## Code

- `centring_controller.py`, `CentringController.step`: the five steps above; without a velocity the lead is left out and the gain held to 0.6.
- `centring_controller.py`, `CentringGains.from_config`: the limits a sequence's gains are held to (Kp 1.0 at most, 0.6 with a lead under 0.5 s).
- `centring_controller.py`, `speed_cap`: the cap by height; never over `safety_limits`' 1.0 m/s.
- `red_bullseye_mission_base.py`, `_retuned_velocity`: the height for the cap (a barometer's taken 2 m lower) and the log line when the lead has no velocity.
- `tools/cen_loop_sim.py`, `simulate`: the loop, with its stand-ins for the estimator, the vehicle and the wind.

## Evidence

```
$ python3 -m pytest -q src/drone_mission_core/test/test_centring_controller.py
11 passed in 0.02s
$ python3 -m pytest -q tools/test/test_cen_loop_sim.py src/drone_mission_demo/test/test_centring_missions.py
71 passed in 2.14s
$ sim/run_scenarios.sh G1
| drop/hold_2m_moderate | 1 | not run |  |  | OK | 17.45 | 0.04 m from the target |
| drop/hold_2m_strong | 1 | not run |  |  | OK | 17.45 | 0.04 m from the target |
| drop/hold_6m_moderate | 1 | not run |  |  | OK | 17.41 | 0.07 m from the target |
| drop/hold_6m_strong | 1 | not run |  |  | OK | 17.30 | 0.16 m from the target |
4 runs: 4 OK
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh
OK
```

Hold error at p95: 0.10 m in moderate wind at 6 m and 2 m, 0.27 m in strong wind at both (G1: 0.20 and 0.30). The experiments behind the lead, the loop simulator's sweep and how far to trust it: `docs/results/2026-11-20-centring-retune.md`.

## Spec check

spec-auditor, one round: no code defect; every number of the spec's first bullet proven by a unit test and a mission test. Its deviations are under Decisions: the lead term; the gains as code defaults until CEN-10 writes the 2027 sequences; zero at once on a lost target; the wind fading near the ground. Owed then and done since: the results file, these notes, and a test that fails if the estimator noise is taken out of the loop simulator.

flight-guardian, two rounds, CONCERNS, no blocker. Round one: the gain range was only shown steady with the lead, which could drop out unnoticed (fixed: gain held to 0.6 without it, a log line, and a sequence refused); a barometer height was trusted like a rangefinder's for the speed cap (fixed: taken 2 m lower). Round two: a tiny lead with a high gain still passed validation. Fixed after the second round by requiring the flown 0.5 s lead for any gain over 0.6; that last rule has not been seen by a reviewer.

## Not done

- The strong-wind margin is 0.03 m on one seed.
- The gains are not in a 2027 sequence file yet (CEN-10).
- At 2 m in strong wind a full-size target leaves the camera's view when the vehicle tilts: CEN-13.
- Wind compensation proper (an integral term, a disturbance observer) is CEN-07, P2; the mean offset measured here, 0.03 m, says there is little for it to do in this sim.
