# ARGUS 2027: instructions for Claude Code

This repo is Purdue's autonomy and computer-vision stack for the 2027 C-UASC competition: ROS 2 Humble, MAVROS and ArduPilot, with Python missions run by a 20 Hz executor. The tag `comp-2026` is the code that flew at the June 2026 competition. It is the fallback, so with every new flag off, the drone must behave exactly as it did then.

## Rules

- Never commit to `main` or push to it. Work on a branch named `<area>/<short-name>`, where the area is one of centring, localisation, recovery, sim or platform, and open a pull request.
- Pull requests go through `/ship`, which runs the checks and the flight-guardian, commits, pushes and opens the PR. Don't open one any other way: if the user asks for a PR, ask them to run `/ship`. Never merge or approve a pull request. The lead reviews with `/review-pr <number>`.
- CI runs four PR gates from `scripts/pr_checks.py`: hygiene (no AI attribution or local machine details in commits, the PR title, the description or added lines), tests, tuning and size. Each has one escape marker that goes in the PR description with a reason: `[hygiene: waived <reason>]`, `[tests: n/a <reason>]`, `[tuning: <reason>]`, `[large-file: <reason>]`. Never add a marker yourself; ask the user for the reason.
- Work only on the issue you were given. If you notice anything else, write it under "Not done" in the pull request instead of fixing it.
- Put new behaviour behind a flag that defaults to `false`. There are two kinds:
  - Mission flags go in the `features:` block under `mission_sequence:` in the sequence YAML (`src/drone_mission_demo/config/sequences/`). Missions read them with `get_feature_flag` from `src/drone_mission_core/drone_mission_core/features.py`, never with a bare dictionary lookup.
  - Node flags, for the grabbers, `target_cv` and other nodes, are ROS parameters named `features.<flag>`. They go under that node's `ros__parameters` in its params file, and the node declares them with a default of `false`.
  - Never put `features:` at the top level of a ROS params file. The file stops parsing and no node in that launch starts.
- Do not change these interfaces: the mission API (`on_enter`, `update`, `on_exit`), the 20 Hz executor tick, `fake_drop`, and the failure policies.
- Do not change existing tuned values (gains, altitudes, deadbands, thresholds, calibration) unless the issue asks for it.
- Keep logic in plain Python modules with no ROS imports, so it can be tested without ROS. ROS nodes stay thin wrappers around that logic.
- Every change comes with a test. Add a `run_test` line for each new test file in `scripts/check.sh`. Tests that import `cv2`, `numpy` or `yaml` go inside the block in `scripts/check.sh` that checks for those modules, so they print `SKIP` instead of failing on a machine without them. Run `scripts/check.sh` before you open a pull request; it must pass. Break the code once and confirm the new test fails, then restore it.
- The repo is public. Never commit passwords, tokens, keys, datasets, ROS bags, flight logs, model training runs or TensorRT engines.
- Measured numbers (accuracy, latency, A/B runs) go in `docs/results/YYYY-MM-DD-<topic>.md`, with the command that produced them.
- Ask the user before using `sudo`, deleting files, installing system packages, or changing anything outside this repo.
- Propose a plan and wait for the user's go before writing code.
- In the pull request, fill in every section of the template: what changed, how it was tested (commands and output), which flag turns it on, and what is left undone.

## Where things are

| Path | What it is |
| --- | --- |
| `src/drone_mission_core` | Executor, mission API, registry, `tracking_projection.py` (pixel to ground) |
| `src/drone_mission_demo` | Missions (`red_bullseye_mission_base.py`, package drop, package delivery, waypoints), sequence and params YAML under `config/` |
| `src/drone_target_cv` | Camera grabbers (`mipi_grabber.py`, `usb_grabber.py`), the `target_cv` node, `RedTargetDetector` in `target_detector.py` |
| `src/vision_pipeline` | YOLO marker pipeline (`yolo_node.py`), `mission_logger.py` (pose interpolation), clustering |
| `src/drone_utils` | Logging, SDF models for the sim (`sdf_objects.py`) |
| `tools/` | ROS-free tools: `bullseye_try.py`, `synth_targets.py` |
| `scripts/check.sh` | The one check: CI runs it, and so should you |
| `scripts/pr_checks.py` | The PR gates; `scripts/check.sh` runs them |
| `.claude/` | Shared Claude Code settings, the `/ship` and `/review-pr` skills, the `flight-guardian` reviewer |
| `.githooks/` | Git hooks (no commits on `main`, hygiene checks); a session-start hook turns them on |
| `docs/review-checklist.md` | The rules `/review-pr` checks, by id |
| `AGENTS.md` | Last year's notes on the mission framework |

## Running tests

`scripts/check.sh` runs the ROS-free tests with plain `pytest`. The mission tests stub `rclpy`, so they run without ROS installed. Tests that need ROS are listed as not run. If a tool the script needs is missing, it prints `SKIP: <what>`. CI runs it with `CHECK_STRICT=1`, which turns any SKIP into a failure.

## End-of-task report (always)
1. Problem solved, in one sentence; root cause or symptom.
2. Architecture: new or moved modules, nodes, topics, dependencies. Or "none".
3. Flow: one concrete path (frame, message or mission state) before vs after.
4. Code: the 3 most important logic changes, in plain words, with file:line.
5. How to see it: the exact commands to run it, and what output to look for.
6. Not done or not verified.
