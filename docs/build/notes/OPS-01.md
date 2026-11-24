# OPS-01: Field setup tool

Flag: none (a tool run before the flights; it writes beside the committed files and changes none of them) · Status: DONE · Commits: `git log --oneline --grep '^OPS-01:' platform/build-2027`

## What it solves

In 2026 the day's coordinates were typed into several sequence files and a waypoint file by hand, on the field, and the fence was drawn in the ground station separately. A digit wrong in one of them was found in the air. Now they are typed once, into one day file, checked against the fence and the rules, and every file that needs them is generated.

## Architecture

- `tools/field_setup.py` (new, no ROS): reads a day file, checks it, writes `field/<name>/`. `tools/go.py` refuses an unverified day's sequence outside the sim.
- `field/example_day.yaml` (new): the placeholder Apollo XI site of `sim/sites/apollo_xi.yaml` with placeholder targets and waypoints; a test holds the two sites together.
- It reuses `sim/site_file.py` (the site's own checks, the fence plan), `drone_mission_core/safety_limits.py` (the 5 m a commanded point keeps from the fence) and the line editing of `tools/sim_sequence_copies.py`.
- `field/*/` is in `.gitignore`: generated folders are rebuilt from their day file.

## Flow

- Before: edit `target_latitude` in each sequence, edit the waypoint file, draw the fence, hope they agree.
- After: `field/<day>.yaml` → `python3 tools/field_setup.py field/<day>.yaml --for-flight` → the checks (each failure printed, nothing written) → `field/<name>/site.yaml`, `fence.plan`, `fence.param`, `waypoints/competition_waypoints.yaml`, `sequences/*.yaml`, `summary.txt` → `tools/go.py --sequence field/<name>/sequences/package_delivery_live.yaml`; the fence from `sim/fence_upload.py --site field/<name>/site.yaml`.

## Code

- The day file's shape: `tools/field_setup.py:122`; the checks against the fence and the rules, the route's legs included: `:197`.
- Which sequence takes what: `:80`; a sequence with the day's coordinates, line for line: `:257`; the refusal of any other difference: `:324`.
- Everything written: `:392`.
- `tools/go.py` refuses, outside the sim, a sequence generated from a day file that is not verified.

## Evidence

```
$ pytest tools/test/test_field_setup.py   (as scripts/check.sh runs it)
37 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ python3 tools/field_setup.py field/example_day.yaml --check | tail -1
OK: nothing was written (--check)
$ python3 tools/field_setup.py field/example_day.yaml --out <folder> | grep -c '^wrote'
15
$ python3 sim/site_file.py check <folder>/site.yaml | head -1
site example_day: NOT VERIFIED (placeholder coordinates)
```

The tests were written after the tool, and each was run against it once; the self-check inside the tool (a generated sequence may differ from its source only in coordinates) failed on the recovery's `pickup` the first time the tool ran, which is how its path rule was corrected.

## Spec check

- spec-auditor, round 1: INCOMPLETE on three rows. Fixed: the day's flight boundary now reaches every mission that reads one (a `flight_boundary` line is added beside the route's `waypoint_file` and the recovery's `pickup`; with every flag off nothing reads it); the recovery's return point comes from the day file (`recovery_return: takeoff | compass_rose`); the guard test covers every way a sequence holds a coordinate. Added from its observations: each leg of the route is checked against the fence (a boundary with a notch), and `go` will not fly an unverified day's sequence on the aircraft. Recorded under "Decisions": the survey and the recovery have no live sequence, so the day's survey area and pickup reach only their sim sequences until one exists. No second round: tested rules.
- No flight-guardian review: no mission, executor or detector code and no config under `src/` changed. `go` gained one refusal.

## Not done

- No real coordinates: the example is placeholders, and every generated file says NOT VERIFIED until a day file is marked verified ("Needs a person").
- `fence.plan` has not been opened in QGroundControl. The upload the sim has proven is `sim/fence_upload.py` with the generated site file.
- The survey and the recovery have only sim sequences: their generated copies carry the day's coordinates with the sim's camera and vehicle, and say so. Until live sequences for them exist, the day's survey area and pickup do not reach the aircraft through this tool.
- A generated sequence has not been flown: this row is proven by unit tests. The generated files differ from flown ones only in coordinates and in the added boundary line, which the tool checks of its own output.
- Heights in the sequences are not re-checked here: they are the committed files' own, and `enforce_safety_limits` (FND-09) is what holds them to the rules at start.
