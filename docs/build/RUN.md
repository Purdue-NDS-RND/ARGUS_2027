# Running the ARGUS 2027 build

How the lead starts, watches, resumes and lands the build sessions. The session's own procedure is `SESSION.md`; feature status is in `features.json` (`python3 tools/build_ledger.py summary`).

## Before the first session

1. Update Claude Code: `claude update`. The run needs auto mode and `/goal`.
2. Check the test modules: `python3 -c "import pytest, numpy, cv2, yaml"`. If it fails, install the missing ones before starting.
3. Copy `docs/build/settings.local.example.json` to `.claude/settings.local.json`. It is ignored by git; `git status` must not list it.
4. Check the simulator: `HEADLESS=1 sim/run_sim.sh` must reach `== READY`; then `sim/run_sim.sh --down`.
5. Plug the laptop in. The launch command below stops it sleeping, including with the lid shut.
6. Optional: start a `tmux` session first, so closing the terminal doesn't end the run.

## Session order

One fresh session per scope, in this order. Each starts from the ledger and git, so a new session loses nothing.

| Session | `/build` argument | Name |
| --- | --- | --- |
| 1 | `wave 0` | `argus-w0` |
| 2 | `wave 1` | `argus-w1` |
| 3 | `wave 2` | `argus-w2` |
| 4 | `wave 3 CEN` | `argus-w3-cen` |
| 5 | `wave 3 LOC` | `argus-w3-loc` |
| 6 | `wave 3 REC` | `argus-w3-rec` |
| 7 | `wave 3 PLT` | `argus-w3-plt` |
| 8 | `wave 4` | `argus-w4` |
| 9 | `wave 5` | `argus-w5` |

## Start a session

From the repository root, with a clean working tree:

```bash
systemd-inhibit --what=sleep:idle:handle-lid-switch --who=argus --why="ARGUS build" \
  claude --model fable --effort high --permission-mode auto \
  --settings docs/build/run-settings.json -n argus-w0 --remote-control argus-w0
```

Change both names (`argus-w0`) to the session's name from the table. `--remote-control` lets you watch and answer from the Claude app; leave it out if you don't want that.

Then type two lines in the session. The first starts the work; the second queues and keeps it going until the scope is finished:

```text
/build wave 0
```

```text
/goal The /build session for scope wave:0 is finished: "python3 tools/build_ledger.py next --scope wave:0" printed NONE, or the build log in docs/build/PROGRESS.md starts with a STOP line. Claude's last message shows the output of "python3 tools/build_ledger.py summary --scope wave:0", the last three lines of "CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh" ending in OK, and "git status -sb" with a clean tree that is not ahead of origin/platform/build-2027. Or stop after 150 turns.
```

Change `wave 0` and `wave:0` for each session: `wave 3 CEN` becomes `wave:3:CEN`.

## The first session: watch before you leave

Stay for the first two or three features of wave 0. After each one, read its notes file in `docs/build/notes/`, its commit, and the spec-auditor's verdict. If the session misreads the procedure, press Esc, fix `SESSION.md` or the skill in a normal session, and start again. Once a few features look right, let it run.

## While it runs

- The Claude app (Remote Control) shows the session; the desktop shows a notification when it waits for input.
- Progress is on GitHub: the commits on `platform/build-2027`, and its `docs/build/PROGRESS.md` build log.
- When a usage limit is reached, the session waits and continues on its own while it stays open.

## When it stops

- **Scope finished:** read the end-of-turn report, then start the next session from the table.
- **A STOP line:** do what it asks, then start a new session with the same scope.
- **The process ended (crash, reboot, closed terminal):** run the same launch command with `--resume argus-w0` in place of `-n argus-w0`, then type `/build wave 0` again. The goal comes back by itself.
- Avoid `/clear` in a running session: it removes the goal.
- Afterwards, look at `/permissions` → "Recently denied", and "Needs a person" in PROGRESS.md.

## Landing on main

Between sessions, with no `/build` running in this checkout, start a normal session (plain `claude`, without the run settings) and type `/land`. It prepares the next landing group on its own branch. Then type `/ship`, review the pull request, and merge it. Repeat for the next group. The next `/build` session merges `main` in by itself.

## Later

- **Lifting the training-data hold:** change the DAT rows and PLT-04 in `features.json` from HOLD-DATA to TODO, and update "The training-data hold" in `SESSION.md`, in a normal pull request.
- **After the build:** one docs pull request brings the integration branch's `features.json` and `PROGRESS.md` to `main`.
