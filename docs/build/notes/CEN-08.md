# CEN-08: Release offset model

Flag: `release_offset_model` (mission flag, default false; needs `centre_on_estimator` and `drop_release_gates`) · Status: PARTIAL · Commits: `git log --oneline --grep '^CEN-08:' platform/build-2027`

## What it solves

A package let go in wind lands downwind of the release point. The model predicts the drift from the release height and the vehicle's own lean into the wind (no wind speed is measured) and moves the aim point upwind by it. **In the sim it made the drops worse, so K stays 0 and the flag off**: see Evidence.

## Architecture

- `drone_mission_core/release_offset.py` (new, no ROS): the lean from the attitude, low-passed over 2 s; drift = K x height x lean; the shift, 0.5 m at most; `fit()` for K from drops.
- `tools/release_offset_fit.py` (new): K from sim runs (`--runs`) or a CSV of field drops (`--drops`).
- The drop mission makes the model with the flag on; the estimator's centring solution is moved by the shift, so the centring, the descent and the release gates all work to the aim point. One log line at the release carries the height, the lean and the shift.
- Scenarios `drop/release_offset_wind` (K 0, for the fit) and `drop/release_offset_wind_fitted`. New key `release_offset_k: 0.0` in `package_drop_demo.yaml` (a scenario can only set a key the sequence has).

## Flow

- Flag off, or K 0: the aim point is the target.
- Flag on with a K: track → centring error → plus K x height x lean (upwind) → the controller and the release gates see the error to the aim point → release → `release offset: height 2.42 m, lean (-0.0792, -0.1201), aim shifted (-0.275, -0.417) m (K 1.61)`.

## Code

- The model and its limits: `src/drone_mission_core/drone_mission_core/release_offset.py:87`, `:118`; the fit: `:157`.
- Where the model is made (the drop only, both flags on, at construction): `src/drone_mission_demo/drone_mission_demo/missions/red_bullseye_mission_base.py:166`.
- The aim point moved: `src/drone_mission_demo/drone_mission_demo/missions/red_bullseye_mission_base.py:492`; the line at the release: `src/drone_mission_demo/drone_mission_demo/missions/package_drop_mission.py:373`.

## Evidence

```
$ pytest src/drone_mission_core/test/test_release_offset.py src/drone_mission_demo/test/test_release_offset_mission.py tools/test/test_release_offset_fit.py     (as scripts/check.sh runs them)
4 passed, 4 passed, 2 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh drop/release_offset_wind --seeds 1,2,3 --variants on        (K 0)
0.08 m, 0.45 m, 0.40 m from the target
$ python3 tools/release_offset_fit.py --runs sim/.state/suites/drop-release_offset_wind-20261125T142032Z/runs/*
K = 1.61 from 3 drop(s) (the largest lean 0.134). The drifts' RMS is 0.45 m; with this K the model leaves 0.12 m.
$ sim/run_scenarios.sh drop/release_offset_wind_fitted --seeds 1,2,3 --variants on  (K 1.61)
1.09 m, 0.44 m, 1.46 m from the target
$ sim/run_scenarios.sh drop/release_offset_wind --seeds 1 --variants on              (the code as committed, K 0)
0.82 m from the target; `release offset: height 2.54 m, lean (-0.0235, -0.0536), aim shifted (-0.000, -0.000) m (K 0)`
```

`docs/results/2026-11-25-release-offset.md`. The tests were written with the code.

## Spec check

spec-auditor: INCOMPLETE on round one: a DEVIATION (with the gated release off, the shift was for the wrong height and the logged lean was stale), the sim proof unfinished, the field drops owed. Round two (those rows only): the deviation DONE (the model now needs `drop_release_gates`); the sim proof PARTIAL because it failed, not because it is owed. Left as it is: the release's log line is said once, so if a commanded release fails and is tried again the line is the first try's (it matters only to the fit's data).

flight-guardian: CONCERNS, no blocker. Taken: the model is made at construction, so a bad K stops the executor on the ground; no height above the ground of the moment means no lean followed and no shift; the line is logged on the tick of the command; K at most 5 and the shift at most 0.5 m. Recorded, not changed: the release gates measure to the aim point, so a gated release can be up to 0.5 m upwind of the track by design, and an aim point that moves with the vehicle's own lean can keep the gates from holding (seed 3's last-resort release).

## Not done

- The model does not hold in the sim (why PARTIAL, with the field drops): the same seeds flown with the fitted K landed further off, and the parcel's drift varied tenfold at one lean and one height. Nothing ships a K.
- Field drops (a person): at least ten, flag on with K 0, then the fit, then drops the fit did not see.
- The height's part of the model is untested: every drop was from 2.4 m.
