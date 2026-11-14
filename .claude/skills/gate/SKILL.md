---
name: gate
description: Runs one ARGUS 2027 sim acceptance gate (G0 to G6, G0-W2/W3/W4) from docs/build/FEATURES.md and records the result. Use only when the user types /gate, or when a /build session's ledger says "GATE <name>"; never otherwise.
argument-hint: "<G0..G6 | G0-W2 | G0-W3 | G0-W4>"
---

Run acceptance gate: $ARGUMENTS

## 1. What the gate demands

Read the gate's row in the SIM-11 table in `docs/build/FEATURES.md` (the G0 re-checks G0-W2, G0-W3 and G0-W4 use G0's row, flags off). Note the suite, the number of runs and the pass rule. G7 and G8 need field data and the aircraft: if asked for them, say so and return.

## 2. Run it

- The gate needs the scenario runner, `sim/run_scenarios.sh` (SIM-07). If it doesn't exist yet, say the gate can't run yet and return without recording anything.
- Run the gate's suite with the runner, sending the full output to a log under `sim/.state/gates/`. Use `--variants off,on` when the gate compares flags off with flags on; G0 and its re-checks run flags off only. The runner keeps the laptop awake itself.
- One Gazebo instance at a time. If a sim is already running, stop it first with `sim/run_sim.sh --down`.
- Until every node runs on sim time (FND-03), a run whose mean real-time factor over the airborne part is below 0.8 is INVALID; re-run it. After three INVALID runs in a row, record the gate INVALID with the numbers and return.

## 3. Record it

- Write `docs/results/YYYY-MM-DD-gate-<name>.md`: the gate, the commit (`git rev-parse --short HEAD`), the flags on, the exact command, a table of every run, the pass rule, and PASS, FAIL or INVALID. Copy numbers from the output; don't round them into a pass. Say when G4 used the oracle detector.
- `python3 tools/build_ledger.py gate <name> <PASS|FAIL|INVALID> --result docs/results/<file>`, and add a build-log line to `docs/build/PROGRESS.md`.
- Commit the result file, the ledger and PROGRESS as `<name>: <PASS|FAIL|INVALID> <short summary>` on the integration branch, and push it.

## 4. If it fails

Report which runs failed and why, from the logs. Inside a `/build` session the failure is the next piece of work (SESSION.md, "Gates": at most three fix-and-rerun cycles). Run by hand, end with the report.
