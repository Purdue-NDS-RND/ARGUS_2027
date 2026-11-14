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

## Build log

One line per finished feature, gate run or stop, newest first: `- YYYY-MM-DD <ID> DONE: <one line>`, `- YYYY-MM-DD G0: PASS ...`, `- STOP YYYY-MM-DD: <what is needed>`.

