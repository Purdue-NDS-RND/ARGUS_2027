# SIM-01: Scenario files, world builder, object models

Flag: none (sim tooling). A scenario's own flags are applied only with `--variant on`, and only to generated copies · Status: DONE · Commits: `git log --oneline --grep '^SIM-01:' platform/build-2027`

## What it solves

A sim run used to be whatever was in the one world and the shipped YAML, with the target spawned after start-up. This makes a test case a file: one scenario YAML names the mission, the site, the objects on the field, the error in the coordinates the mission is given, and the flags; the builder turns it into a run folder the sim starts from. It fixes the cause (no way to describe or repeat a test); the things that act on wind, GPS events and payloads come in later features.

## Architecture

- `sim/scenario/spec.py`: reads and checks a scenario file. `sim/scenario/build.py`: places the objects, writes the run folder. `sim/scenario/README.md`: the file format.
- `sim/scenarios/<mission>/*.yaml`: `delivery/baseline`, `drop/baseline`, `delivery/clutter`, `waypoint_navigation/site_circuit`.
- `src/drone_utils/drone_utils/sdf_objects.py`: new `build_marker_sdf`, `build_x_mat_sdf`, `build_bucket_sdf`, `build_payload_sdf`. The 2026 builders are untouched (a test pins their output).
- `sim/target_placement.py`: `latlon_for`, the inverse of `offset_ne_m`.
- `sim/run_sim.sh`, `sim/container/start.sh`, `sim/compose.yaml`: optional `RUN_DIR=<run folder>`.

No new dependency, node or topic.

## Flow

Before: `sim/run_sim.sh` started the runway world, then `spawn_target.py` asked Gazebo to create the bullseye at the coordinates in the shipped sequence.

After: `python3 sim/scenario/build.py sim/scenarios/delivery/clutter.yaml` places 11 objects (three markers at seeded random spots in the survey area), embeds their models in the site world (under the 2026 light unless the scenario sets a sun), copies the shipped delivery sequence with `target_latitude` and `target_longitude` set to the bullseye's true position, and writes `truth.json` and `judge.yaml`. `RUN_DIR=... sim/run_sim.sh` starts Gazebo on that world: every object is there from the first step.

## Code

- `sim/scenario/build.py:89` `place_objects`: fixed, seeded-random (inside an area, apart from the others, clear of its edge) or on top of another object; anything outside the flight boundary fails the build.
- `sim/scenario/build.py:212` `build_sequence`: the shipped sequence plus config overrides, target coordinates with the coordinate error, the waypoint file and, in variant `on`, the flags. An undeclared flag, an unknown mission name or an unknown setting fails the build.
- `sim/scenario/build.py:185` `build_world`: the site world with each object's model inserted with its pose, and the sun's direction.
- `sim/scenario/spec.py:140` `parse_scenario`: every key and object setting is checked against a list, so a typo is an error.
- `src/drone_utils/drone_utils/sdf_objects.py:418` `build_marker_sdf` and `:526` `build_bucket_sdf`: the marker with its digit centred; the bucket as one dynamic link with a solid cylinder's inertia and an upright handle.

## Evidence

```
$ python3 -m pytest -q -p no:cacheprovider sim/test/test_scenario.py sim/test/test_target_placement.py sim/test/test_site.py
101 passed in 2.02s
$ PYTHONPATH=src/drone_utils python3 -m pytest -q -p no:cacheprovider src/drone_utils/test/test_sdf_objects.py
23 passed in 0.02s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

`test_scenario.py` and `test_sdf_objects.py` were run before their modules existed and failed (`ModuleNotFoundError`, import error).

In the sim (hand-started; the runner is SIM-07):

```
$ python3 sim/scenario/build.py sim/scenarios/delivery/clutter.yaml
built sim/.state/runs/delivery-clutter-s1-off
$ RUN_DIR=sim/.state/runs/delivery-clutter-s1-off HEADLESS=1 sim/run_sim.sh
== Scenario run folder: <repo>/sim/.state/runs/delivery-clutter-s1-off
== Starting Gazebo: world.sdf
fence uploaded and read back: 4 vertices; fence enabled
== READY. In another terminal:
$ sim/run_sim.sh --exec gz model --list            # copied from the terminal; not kept in a log file
    - axes
    - runway
    - iris_downcam
    - delivery_target
    - boundary_sw
    - boundary_se
    - boundary_ne
    - boundary_nw
    - marker_1
    - marker_4
    - marker_7
    - x_mat
    - bucket
    - package
$ sim/run_sim.sh --exec gz model -m bucket -p        # truth.json: bucket at -30.0 E, 40.0 N on the mat
    [-30.000000 40.000000 0.020000]
$ sim/run_sim.sh --exec gz model -m marker_4 -p      # truth.json: 103.2 E, 109.2 N, heading 135
    [103.195000 109.196000 0.000000]
    [0.000000 0.000000 -2.356190]
```

Then the 2026 delivery mission, launched by hand with the run folder's `sequence.yaml` and `params.yaml`, flew the whole sequence at the new site with the clutter on the field: `TRANSIT_TO_TARGET` → `ACQUIRE_TARGET` ("Target acquired") → `TRACK_AND_DESCEND` → `FINAL_FIXED_COLUMN_DESCENT` → `GROUND_DWELL` ("Fake delivery complete") → `GUIDED_RELAUNCH` → `Mission 'rtl' completed` → `Mission sequence complete`. Its touchdown distance was not measured: nothing records true poses until SIM-07.

## Spec check

spec-auditor: INCOMPLETE in round 1 on one row, `truth.json`: any package or bucket on the field became the judge's payload or bucket, so the loose package in `delivery/clutter` earned the weight bonus. COMPLETE in round 2, after scenarios were made to name their payload and bucket (`payload:`, `bucket:`), with tests that score clutter and baseline alike. The same round added a test for the `radius_m` error form and kept the 2026 light when a scenario sets no sun.

flight-guardian did not run: no mission, executor, target CV or `config/` YAML changed (`sdf_objects.py` in `drone_utils` only gained builders). Choices the spec left open are in PROGRESS.md under Decisions.

## Not done

- `wind`, `gps_events` and `sensor_profile` are carried to the runner but nothing acts on them yet (SIM-03, SIM-04, SIM-05). The payload and bucket are loose bodies; attaching them to the vehicle is SIM-06.
- One vehicle variant (`iris_downcam`, the 2026 camera) with its bridge file.
- The marker's look is a guess at the rules' drawing (organiser question 8).
- A scored, unattended run of a scenario needs the runner and recorder (SIM-07).
