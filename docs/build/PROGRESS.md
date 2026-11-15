# ARGUS 2027 build progress

The build session's log. Feature and gate status live in `docs/build/features.json`; read them with `python3 tools/build_ledger.py summary`. The live copies of both files are on the integration branch `platform/build-2027`; `main` keeps the starting copies until the build ends. How the session works is in `docs/build/SESSION.md`.

## Needs a person

Steps the session cannot do. The session adds a row whenever a feature leaves one.

| For | What | Who |
| --- | --- | --- |
| DAT lane, PLT-04 | Training data: Roboflow key rotated, 2026 full-frame export, `.pt` weights and ONNX, field data days. Lifting the hold is the lead's call | Lead |
| CEN-03 | Buy and fit the downward rangefinder (TFmini Plus or TFmini-S) | Lead |
| REC-4, REC-9 | Recovery mechanism and its numbers: capture half-widths, hook position, suspension length, hover throttle with 1 and 2 kg | Mechanical team |
| PLT-05 | Wire an LED to a Jetson GPIO in the camera's view | Lead |
| SIM-10 | A club machine that never sleeps for nightly runs, and a GitHub token limited to issues | Lead |
| Wave 4 | Jetson on the bench, camera calibration per sensor mode, flight-controller parameters loaded, failsafes tested with the safety pilot | Lead |
| PLT-13, G6 | Any flight in the club simulator (`sim/gps/`), which a person starts | Lead |
| All | The organiser questions in FEATURES.md, "Open questions" | Lead |
| SIM-02, PLT-08, OPS-01 | Check every coordinate in `sim/sites/apollo_xi.yaml` against the organisers' field map (origin, ground elevation, runway, flight boundary, maximum height, takeoff point, compass rose, survey area), then set `verified: true`. They are placeholders | Lead |
| SIM-02 | Reword the SIM-02 pass line in FEATURES.md: a GUIDED waypoint outside the fence is refused by ArduPilot and causes no breach (see Decisions) | Lead |

## Decisions made during the build

Choices the specs did not settle, with the reason. Newest first.

| Date | Feature | Decision | Why |
| --- | --- | --- | --- |
| 2026-11-15 | SIM-01 | A scenario names the object its mission carries (`payload:`) and the one recovery must bring back (`bucket:`); only those become the judge's payload and bucket. Other packages and buckets are just on the field | Otherwise the loose package in `delivery/clutter` earned the delivery weight bonus, and would have been scored as the delivered package |
| 2026-11-15 | SIM-01 | A scenario with no `sun:` keeps the base world's light, the one the 2026 sim flew under; `sun: {elevation_deg, azimuth_deg}` replaces it | The flags-off baseline (G0) should see what the 2026 detector was measured with |
| 2026-11-15 | SIM-01 | The 2027 marker model (`build_marker_sdf`) is a black-and-white X plate with a seven-segment digit centred on a white field over the crossing, no underline; a marker with no digit is a 1.2 m boundary marker. The 2026 `build_bw_target_sdf` (digit off-centre, underlined) is left as it was | The spec says "digit centred in the X as the rules show"; the font and the 6/9 mark are organiser question 8. One builder to change when the answer comes |
| 2026-11-15 | SIM-01 | A scenario is built as variant `off` (every flag false, the 2026 behaviour) or `on` (the scenario's flags); flags are written only to the copies in the run folder. A mission flag must be declared under `features:` in the shipped sequence, a node flag under that node's `ros__parameters.features` in the shipped params file | The spec's rule that shipped YAML stays false, and the runner's off-against-on table (SIM-07) |
| 2026-11-15 | SIM-01 | `wind`, `gps_events` and `sensor_profile` are checked and passed to the runner in `scenario.json`, but nothing acts on them yet; the only sensor profile is `delivery` (the 2026 camera); the payload and bucket are loose bodies, not attached to the vehicle | Wind is SIM-03, GPS faults SIM-04, camera profiles SIM-05, payload joints SIM-06 |
| 2026-11-15 | SIM-01 | A scenario's `config` override must name a setting that exists in the shipped sequence; an object is placed with `at`, or `on_top_of` another object | A typo would otherwise be ignored by the mission. `on:` cannot be used as a YAML key: YAML reads it as `true` |
| 2026-11-15 | SIM-01 | Shipped scenarios: `delivery/baseline` and `drop/baseline` (the 2026 missions, bullseye 10 m east and 15 m north of the takeoff point as in the 2026 sim baseline), `delivery/clutter` (the same with every 2027 object on the field) and `waypoint_navigation/site_circuit` (an invented seven-point route inside the placeholder boundary) | The 2026 waypoint route is at last year's field; the lanes add their own scenarios |
| 2026-11-15 | SIM-01 | `sim/run_sim.sh` takes `RUN_DIR=<run folder>`: the world, bridge file and SITL parameters come from the folder and the site from its scenario | A built scenario has to be startable before the runner (SIM-07) exists; the runner uses the same switch |
| 2026-11-15 | SIM-02 | `fence_upload.py` refuses a site that says `verified: false` unless `--allow-unverified` is given; only the sim's `start.sh` gives it | The Apollo XI file is a placeholder, and the same tool will upload to the aircraft (PLT-08) |
| 2026-11-15 | SIM-02 | The site's runway `length_m` and `width_m` are recorded but not drawn: the site world keeps the base runway model (1500 × 100 m strip, which also carries the ground plane), turned to the site's heading | Visual only; nothing reads the strip's size. SIM-01 can draw the real strip when it builds scenario worlds |
| 2026-11-15 | SIM-02 | Added `sim/fence_check.py`, which the spec does not name: it flies the pass line (inside target, the outside waypoint, then a velocity breach) and exits 0 only inside the limits | The pass line needs a command that reruns; no 2026 mission flies through a fence |
| 2026-11-15 | SIM-02 | `sim/sites/apollo_xi.yaml` holds placeholder coordinates: origin 34.1750, −118.4825 at 215 m, a 300 × 240 m flight boundary, 120 m maximum height, the takeoff point at the origin, the compass rose 20 m east, a 70 m survey square. The file says `verified: false` and every tool that reads it prints that | The spec says the exact coordinates are still to verify from the field map; the sim needs a site to fly meanwhile |
| 2026-11-15 | SIM-02 | The SIM-02 pass line cannot be met as written for the way our missions fly: in GUIDED, ArduCopter 4.6.3 did not fly to a position target 20 m outside the fence (a target inside it was followed just before), so that waypoint causes no breach. `sim/fence_check.py` sends that waypoint, and when it is refused flies a velocity command through the fence and applies the pass line's own limits: RTL 0.04 to 0.06 s after the line, 0.99 to 4.46 m outside (`docs/results/2026-11-15-site-fence.md`). An AUTO mission with a waypoint outside the fence is untested | Measured in the sim. The limits (RTL within 2 s, at most 10 m) are kept; only the trigger changes |
| 2026-11-15 | SIM-02 | A site run is chosen with `SITE=<name>` on `sim/run_sim.sh`. Only then: the world is moved to the site, SITL storage is wiped (and once more on the next plain start), `SIM_TERRAIN 0`, a MAVProxy tools port on UDP 14552, the fence upload, and no 2026 delivery target. With no `SITE` the start-up is unchanged | The plain sim is the 2026 baseline that G0 measures; a fence left in SITL's storage would stop it arming at the old origin |
| 2026-11-15 | SIM-02 | The site module is `sim/site_file.py`, not `site.py` | `site` is a Python standard-library module; a file of that name on `sys.path` breaks interpreter start-up |
| 2026-11-15 | SIM-02 | `fence_upload.py` reads the polygon back and only then sets the `FENCE_*` parameters, `FENCE_ENABLE` last; `FENCE_TYPE` is 5 (maximum height and polygon, no circle); a site whose `fence.action` is 0 (report only) is refused | A fence switched on without its polygon, or one that only reports, is worse than none |
| 2026-11-15 | SIM-08 | The autonomy bonus (+25%) is applied to all six missions, as the SIM-08 text says. The rules table in FEATURES.md lists only localisation, drop, delivery and recovery; `autonomy_bonus_fraction: 0` in a scenario's judge settings turns it off for the GPS missions | SIM-08 says "every mission"; the lead can flip the default in one place (`sim/judge/score.py`) |
| 2026-11-15 | SIM-08 | Localisation points: +1 per correct marker; a wrong ID, a wrong location or both score −0.5, −0.5, −1 outright (not deducted from a +1); an extra row is −1, a missed marker 0; the total never goes below 0 | The spec gives only the penalties. Outright penalties match decision 18 (a wrong ID must cost more than abstaining). All are settings |
| 2026-11-15 | SIM-08 | Time trial: with several reference times, middle ranks are interpolated between 12 and 8; with none, the run scores 8. Tolerance is 3D | "12 or 8 by rank" is exact for one reference time; no reference means fastest is unproven, so the lower score |
| 2026-11-15 | SIM-08 | Points the spec does not give: waypoint navigation 1 per waypoint; recovery 16 − 1 per metre from the return point | Same shape as drop (14 − d) and delivery (13 − d); 16 is the recovery maximum in the REC section. Settings |
| 2026-11-15 | SIM-08 | "Vehicle below 1.83 m" (drop) and the localisation height bands ignore the vehicle within 5 m of the takeoff point; localisation judges height only inside `survey_area` when the truth file has one | Takeoff and landing are always low; the rule is about the mission area. `takeoff_zone_radius_m` is a setting |
| 2026-11-15 | SIM-08 | Drop: "released below 1.83 m" and "vehicle below 1.83 m" are two separate −6 penalties, so a low release costs 12 | The SIM-08 row lists both; the rules table has one. Either fails gate G2 anyway |
| 2026-11-15 | SIM-08 | Release height is the vehicle's for drop; for delivery and recovery it is the package's or bucket's own height above where it rests (the vehicle's at a fake release) | A bucket hangs below the vehicle, so the vehicle's height would zero a correct set-down |
| 2026-11-15 | SIM-08 | Delivery weight bonus counts whole 500 g steps and is added only when the delivery itself scores | "+0.25 per 500 g" with no rounding rule; a miss should not score on weight alone |
| 2026-11-15 | SIM-08 | Only the first flight (arm to disarm, having left the ground) is scored; later flights are counted and reported | "Single flight" with no stated consequence; delivery's touchdown and relaunch stays one flight because it stays armed |

## Build log

One line per finished feature, gate run or stop, newest first: `- YYYY-MM-DD <ID> DONE: <one line>`, `- YYYY-MM-DD G0: PASS ...`, `- STOP YYYY-MM-DD: <what is needed>`.

- 2026-11-15 SIM-01 DONE: scenario files (`sim/scenarios/<mission>/`), `sim/scenario/build.py` writes a run folder (world with every object embedded, sequence, SITL parameters, truth, judge settings), new marker, X mat, bucket and payload models; `RUN_DIR=<folder> sim/run_sim.sh` starts it. Hand-flown once in the sim; the scored, unattended run is owed to SIM-07.
- 2026-11-15 SIM-02 DONE: site file `sim/sites/apollo_xi.yaml` (placeholder coordinates), `sim/site_file.py`, fence upload and `SITE=<name>` start-up; live fence check PASS (RTL 0.04 to 0.06 s after the line, at most 4.46 m out). Pass-line trigger deviates (GUIDED refuses the outside waypoint); the true-pose scored run is owed to SIM-07.
- 2026-11-15 SIM-08 DONE: six ROS-free judges in `sim/judge/` score a run folder from true poses (`python3 sim/judge/cli.py <run>` writes `score.json`); 84 tests. SIM-07 owes the recorder that writes `trace.jsonl` and the copy of the list at landing.

