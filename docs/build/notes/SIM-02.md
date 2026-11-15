# SIM-02: Competition-site world, home and geofence

Flag: none (sim tooling; chosen per run with `SITE=<name>`) · Status: DONE · Commits: `git log --oneline --grep '^SIM-02:' platform/build-2027`

## What it solves

The sim only knew the 2026 runway world, with no fence. This adds one file per competition site that the world, SITL's home, the geofence and the judges' truth are all derived from, and a tool that uploads the fence to the flight controller. It fixes the cause of coordinates being typed in several places; the Apollo XI coordinates themselves are placeholders until someone checks the field map.

## Architecture

- `sim/sites/apollo_xi.yaml`: origin, ground elevation, runway, flight boundary, maximum height, takeoff point, compass rose, survey area, fence action and margin. `verified: false`.
- `sim/site_file.py` (ROS-free): loads and checks a site, converts to metres with ArduPilot's flat-earth formula, and derives the site world, the fence plan and the site part of `truth.json`. Commands `check`, `world`, `truth`.
- `sim/fence_upload.py`: the upload conversation (ROS-free, no pymavlink) plus a thin pymavlink link. `--dry-run` prints the plan; `--url` points it at any flight controller.
- `sim/fence_check.py`: flies the pass line in the sim (inside target, the waypoint 20 m outside, then a velocity breach) and exits 0 only if RTL comes within 2 s and 10 m. The measuring and the flight steps are ROS-free and pymavlink-free.
- `sim/run_sim.sh`, `sim/container/start.sh`, `sim/compose.yaml`: optional `SITE=<name>`. New MAVProxy output UDP 14552 (tools port), only in a site run.
- `sim/container/sitl_storage.sh`: the rule for when SITL's stored parameters are wiped.

## Flow

Before: `sim/run_sim.sh` always started `argus_runway.sdf` at the 2026 origin, with no fence.

After, with `SITE=apollo_xi`: `start.sh` checks the site file, writes `sim/.state/worlds/apollo_xi.sdf` (the base world with origin and runway strip moved), wipes SITL's stored parameters, starts SITL at the site origin with `SIM_TERRAIN 0`, waits for home, runs `fence_upload.py` (4 polygon vertices, read back, then `FENCE_*` with `FENCE_ENABLE` last), and reports `READY` once pre-arm checks pass with the fence on. Without `SITE`, the start-up takes none of those branches.

## Code

- `sim/site_file.py:135` `parse_site`: refuses a site whose boundary crosses itself, whose takeoff point is outside or within 10 m of the fence, whose survey area leaves the boundary, whose maximum height is above 400 ft, or whose fence only reports.
- `sim/site_file.py:253` `fence_plan`: one inclusion polygon (`MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION`) and `FENCE_TYPE 5`, with `FENCE_ENABLE` last.
- `sim/site_file.py:283` `site_world`: swaps the world name, `<spherical_coordinates>` and the runway pose in the base world; the vehicle and plugins are untouched.
- `sim/fence_upload.py:40` `upload`: polygon, read-back of the item count, then parameters, each read back; a lost message is retried three times and any failure leaves the fence parameters untouched.
- `sim/container/sitl_storage.sh:14` `sitl_wipe_needed`: wipe SITL storage for a site run and for the first plain run after one, so a site's fence cannot follow the vehicle to the 2026 world; a plain run after a plain run keeps its storage, as in 2026.
- `sim/fence_check.py:63` `analyse` and `:109` `run_check`: interpolate when the track left the polygon, find the first RTL sample and the largest excursion; refuse to fly unless the autopilot is SITL, and to conclude anything if the vehicle ignores even an inside target.
- `sim/fence_upload.py:185` `main`: a site that is not verified is not uploaded unless `--allow-unverified` is given (only `start.sh` gives it).

## Evidence

```
$ python3 -m pytest -q -p no:cacheprovider sim/test/test_site.py sim/test/test_fence_upload.py sim/test/test_fence_check.py
63 passed in 1.09s

$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK

$ SITE=apollo_xi HEADLESS=1 sim/run_sim.sh
   OK: SITL home 34.1750000, -118.4825000 is 0.00 m from the world origin
fence uploaded and read back: 4 vertices; fence enabled
   pre-arm checks pass after 34s.
== READY. In another terminal:

$ sim/run_sim.sh --exec env -u PYTHONNOUSERSITE python3 sim/fence_check.py --site sim/sites/apollo_xi.yaml --speed 5
inside_target_followed: True
outside_target_refused: True
rtl_delay_s: 0.041
max_excursion_m: 0.985
RESULT: PASS
```

At 10 m/s: `rtl_delay_s: 0.058`, `max_excursion_m: 4.457`, PASS. A third run at 5 m/s, after the sim-only guard was added: 0.050 s, 1.068 m, PASS. A plain start straight afterwards wiped the storage once and reached READY in the 2026 world with the fence off. Full output: [docs/results/2026-11-15-site-fence.md](../../results/2026-11-15-site-fence.md).

Each test file was run before its module existed and failed (`ModuleNotFoundError`). The first live start failed too: `start.sh` found `target_placement` from the world's path, which a generated world breaks. Fixed; every start since reached READY.

## Spec check

spec-auditor: INCOMPLETE in round 1 (the breach probe was not in the repo, the plain start was not re-run, the wipe rule had no test, an unverified site could be uploaded anywhere), COMPLETE in round 2 with one recorded deviation: the pass line's trigger. In GUIDED the waypoint 20 m outside the fence is refused, so the limits were met on a velocity breach instead; AUTO is untested. Only the lead can close that, by rewording the line.

flight-guardian did not run: no mission, executor, target CV or `config/` YAML changed. Choices the spec left open are in PROGRESS.md under Decisions.

## Not done

- **The pass line as written does not happen in GUIDED.** ArduCopter does not fly to a position target outside the fence, so "a waypoint 20 m outside" causes no breach (measured, with an inside target as the control). `fence_check.py` breaches on a velocity command instead. The lead should reword the line; AUTO is untested.
- The scored breach run through the headless runner and the judge's fence report on true poses is owed until SIM-07 exists; `fence_check.py` reads the autopilot's position.
- The site coordinates are placeholders: a person checks them against the field map (PROGRESS.md, "Needs a person").
- "Scenario runs always wipe SITL's stored parameters" holds for every `SITE` run; the runner (SIM-07) inherits it by starting with a site.
- The maximum-height fence was not flown. The runway strip drawn is the base model's, not the site's length and width.
