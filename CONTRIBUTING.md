# Contributing to ARGUS 2027

## Before you start

- Read your assigned issue from top to bottom. It says what to build, what to hand in, and when it is due.
- You need a club drive, a GitHub account with access to this repo, and Claude Code.
- What runs on a club drive today: the club Docker simulator (flown by hand in QGroundControl), `tools/bullseye_try.py`, `tools/synth_targets.py`, and the tests in `scripts/check.sh`. The mission simulator that flies our own code is in progress.

## Workflow

1. Clone the repo into your home folder; several scripts expect this path.
   ```bash
   git clone https://github.com/Purdue-NDS-RND/ARGUS_2027.git ~/ARGUS_2027
   ```
2. Start a branch from an up-to-date `main`. Areas: centring, localisation, recovery, sim, platform.
   ```bash
   cd ~/ARGUS_2027 && git switch main && git pull
   git switch -c recovery/green-x-detector
   ```
   Turn on the repo's git hooks once per clone. They refuse commits on `main` and check commits for AI attribution and local machine paths:
   ```bash
   git config core.hooksPath .githooks
   ```
3. Start Claude Code in the repo folder. It reads `CLAUDE.md` automatically. When it asks whether to trust this folder, say yes: the session-start hook (which also turns on the git hooks) and the flight-guardian's read block only run after that, and the guardian's block needs this folder itself trusted, not a parent folder. The deny and ask rules in `.claude/settings.json` apply either way. Paste in your issue and ask for a plan first. Read the plan before you let it write code.
4. Run `scripts/check.sh` until it passes.
5. Open the pull request with `/ship` in Claude Code. It runs the checks, has the flight-guardian review changes to flight code and config, commits, pushes, and fills in the template from what it actually ran. Read the description before you leave it.
6. The lead reviews it with `/review-pr`. Answer every comment. The pull request is squash-merged once it is approved and the check is green; the PR title and description become the commit on `main`.
7. Show your result in two minutes at the next team meeting. If you cannot explain your change, it is not ready to merge.

## Rules

- `main` is protected. Nobody pushes to it directly; every change goes through a pull request.
- One issue per branch and per pull request. If you find something else, write it under "Not done" in the pull request, or open a new issue.
- New behaviour goes behind a flag that defaults to `false`. Mission flags live in the `features:` block of the sequence YAML. Node flags are ROS parameters named `features.<flag>` on the node that uses them. `CLAUDE.md` has the details. With every flag off, the drone must fly exactly as the `comp-2026` code did.
- Do not change existing tuned values (gains, altitudes, thresholds, calibration) unless your issue says to.
- Every change has a test, and every test file has a `run_test` line in `scripts/check.sh`. Tests that import `cv2`, `numpy` or `yaml` go inside the block in `scripts/check.sh` that checks for those modules, so they print `SKIP` instead of failing on a machine without them. Break your code once and watch the test fail.
- The repo is public. Never commit passwords, tokens, keys, datasets, ROS bags, flight logs or TensorRT engines. Datasets and photos go on the team Drive.
- Numbers you measure go in `docs/results/YYYY-MM-DD-<topic>.md`, with the command that produced them.

## PR gates

CI runs `scripts/pr_checks.py` on every pull request, and again when you edit the title or description. Each gate has one escape marker. Put it in the PR description with a real reason, so every waiver ends up in the history of `main`.

| Gate | Fails when | Marker |
| --- | --- | --- |
| Hygiene | A commit message, the PR title or the description has AI attribution or local machine details (home paths, temp paths, machine names), or an added line has a home or temp path | `[hygiene: waived <reason>]` |
| Tests | Python under `src/` or `tools/` changes and no test file does | `[tests: n/a <reason>]` |
| Tuning | An existing value in a YAML file under `src/*/config/` changes or is removed (adding a key is fine) | `[tuning: <reason>]` |
| Size | An added file is over 5 MB | `[large-file: <reason>]` |

Locally, `scripts/check.sh` runs the gates against `origin/main`. It prints `SKIP: no PR context` for the title and description; set `PR_TITLE` and `PR_BODY` to check them too.

## Getting help

Post in the team chat with three things: what you ran, what you expected, and what happened (paste the full error).
