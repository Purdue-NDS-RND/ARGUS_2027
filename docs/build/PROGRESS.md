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

## Decisions made during the build

Choices the specs did not settle, with the reason. Newest first.

| Date | Feature | Decision | Why |
| --- | --- | --- | --- |
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

- 2026-11-15 SIM-08 DONE: six ROS-free judges in `sim/judge/` score a run folder from true poses (`python3 sim/judge/cli.py <run>` writes `score.json`); 84 tests. SIM-07 owes the recorder that writes `trace.jsonl` and the copy of the list at landing.

