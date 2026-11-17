# ARGUS 2027 build session: how it runs

This is the procedure for the long build session that implements `docs/build/FEATURES.md`. It is started with `/build`, normally one session per wave (one per lane in wave 3), using the launch steps in `docs/build/RUN.md`. The lead approved the feature list and the build order, so this file and FEATURES.md together are the plan: the session does not stop to propose a plan for each feature.

Nobody watches the session turn by turn. Context is compacted automatically when it fills; that is expected, so don't stop early or rush because of it. Everything needed to resume is on disk: the ledger, the progress file and git.

## How this fits with CLAUDE.md

Everything in `CLAUDE.md` applies, with four exceptions that hold only inside a `/build` session:

1. "Work only on the issue you were given": the work is the scope `/build` was started with.
2. "Propose a plan and wait for the user's go": the plan is approved. Stop only for the reasons under "When to stop".
3. "Pull requests go through `/ship`": the session commits to the integration branch. Features reach `main` later through `/land` and `/ship`, which the lead runs.
4. "End-of-task report": `/build`'s end-of-turn report replaces it.

**Why the flags matter.** This is flight code, and the June 2026 code (`comp-2026`) is the fallback. Every new behaviour sits behind a flag that defaults to false, so the drone flies exactly as in 2026 until a flag is turned on deliberately. With all flags off, nothing may change.

## Files

| File | What it is | Who changes it |
| --- | --- | --- |
| `docs/build/FEATURES.md` | Every feature's spec, the build order, the gates, the open questions | The lead only |
| `docs/build/features.json` | The ledger: status of every feature and gate | The session, only through `tools/build_ledger.py` |
| `docs/build/PROGRESS.md` | Needs a person, decisions made during the build, the build log | The session |
| `docs/build/notes/<ID>.md` | A one-page explanation of each finished feature, for the lead | The session |
| `docs/results/` | Measured numbers and gate results, with the command that produced them | The session |
| `docs/build/run-settings.json`, `.claude/hooks/build-context.sh` | Run-only deny rules, the hook that restores context after compaction, desktop notifications | The lead only |

`tools/build_ledger.py` commands: `next --scope S`, `summary --scope S`, `set <ID> <DOING|DONE|PARTIAL|BLOCKED|TODO> [--evidence "..."] [--reason "..."] [--attempt]`, `gate <name> <PASS|FAIL|INVALID> --result <file>`, `check`. Scopes: `all`, `wave:N`, `wave:N:LANE`, `id:ID`.

Statuses: TODO, DOING, DONE (everything the session can do is done), PARTIAL (built as far as the session can; the evidence says what is left and who does it), BLOCKED (stuck; the reason says why and what would unblock it), and three that belong to the lead: HOLD-DATA, HUMAN, LEAD. Only feature IDs in `deps` block a feature; DONE and PARTIAL satisfy a dependency.

## Branch

- All work goes on `platform/build-2027`, cut from `origin/main`. Push it after every feature with `git push origin platform/build-2027` (use `-u` the first time). That push is approved; pushing anything else is not part of the session.
- One feature is one commit, or a few that all start with the same ID. Message: `<ID>: <imperative summary>`, then a body saying what changed, which flag turns it on and how it was tested. The commit-msg hook rejects AI attribution, the words it lists and local machine paths: fix the message and commit again.
- Stage files by path.

## Start of every working stretch

Also after a compaction or a resume (the hook prints a reminder):

1. `git fetch origin`. Switch to `platform/build-2027`, creating it from `origin/main` the first time. Then `git merge --ff-only origin/platform/build-2027`.
2. If `origin/main` has moved (teammates' pull requests, landed groups), merge it in, don't rebase. On a conflict in `docs/build/PROGRESS.md` or `docs/build/features.json`, keep ours (`git checkout --ours <file> && git add <file>`). Run the check. A fix needed after the merge is committed with the ID of the feature it fixes.
3. `python3 tools/build_ledger.py summary --scope <scope>`, and read the newest build-log lines in PROGRESS.md. If a row is DOING, look at what is already committed for it (`git log --oneline --grep "^<ID>:"`) and carry on from there.
4. Read only the spec sections you need from FEATURES.md, not the whole file.

## One feature

`python3 tools/build_ledger.py next --scope <scope>` gives a feature ID, `GATE <name>` (see "Gates") or `NONE` (finish the turn with the report). For a feature:

1. **Claim it.** `build_ledger.py set <ID> DOING`.
2. **Look first.** Search `origin/main` and the integration branch for code that already does part of it, including teammates' work (see "Teammates' assignments"). Extend what exists instead of writing a second version.
3. **Tests first.** Each acceptance line in the spec becomes a test where a unit test can show it. Include at least one test that turns the feature's flag on and drives the real entry point (the node, mission or tool), and keep the flags-off behaviour covered. Register every new test file with a `run_test` line in `scripts/check.sh` (in the cv2/numpy/yaml block when it imports those). Run the new tests: they must fail before the implementation exists. This replaces "break the code once": never use `git checkout`, `restore` or `stash` on uncommitted work.
4. **Implement**, following the rules under "Writing the code".
5. **Check.** `CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh` must end in exactly `OK`.
6. **Prove it at its level.** The ledger's `proven_in` says Unit, Sim, Bench or Field.
   - Sim: once `sim/run_scenarios.sh` exists (SIM-07), add or extend a scenario and run it. Before then, unit tests are the proof and the evidence says the sim run is still owed. Measured numbers go in `docs/results/`.
   - Bench or Field: write the script, the procedure and the analysis, test them on generated or recorded data, finish the row as PARTIAL, and add the person step to "Needs a person".
7. **Review.** Save the feature's diff to `sim/.state/review/<ID>.diff` (`git diff origin/main...HEAD -- <changed files>` plus `git diff` for anything not yet committed). Then:
   - If the change touches `src/drone_mission_core/`, `src/drone_mission_demo/`, `src/drone_target_cv/`, a new flight node, or any YAML under a `config/` directory, run the `flight-guardian` subagent with the diff path and the list of changed files, without telling it what you think the change does.
   - Always run the `spec-auditor` subagent with the feature ID, its spec section copied from FEATURES.md, and the diff path.
   - Wait for both verdicts before going on. Fix every BLOCKER and every MISSING item, then run the reviewer again. If it still fails after two rounds, block the feature (below).
8. **Explain it.** Write `docs/build/notes/<ID>.md` from `docs/build/notes/TEMPLATE.md`, under a page, with the real commands and the tail of their output as evidence.
9. **Record and commit.** `build_ledger.py set <ID> DONE --evidence "<command> -> <result>; <sim or results file>"` (or PARTIAL). Add a build-log line to PROGRESS.md: `- YYYY-MM-DD <ID> DONE: <one line>`. Add any decision and any person step. Commit the feature, its tests, notes, ledger and PROGRESS change together, then push.

**When it won't come together.** After three failed attempts at the same problem, or two review rounds that still fail, run `build_ledger.py set <ID> BLOCKED --reason "<what failed, what was tried, what would unblock it>" --attempt`, add a build-log line, commit what is useful (behind its flag, tests passing) or leave it uncommitted, and take the next row. Don't keep retrying the same fix.

## Writing the code

- Logic goes in a ROS-free module; the node is a thin wrapper.
- New behaviour goes behind a flag that defaults to false. Where a 2026 module would have to change, put the new code in a new module beside it, and switch between them on the flag, so the flags-off path runs the 2026 code untouched.
- New tunable values go in new config files or new keys. Profiles that turn flags on live in `profiles/` (FND-08), never in committed sequence or params YAML.
- ROS-free code and its tests import only the standard library, numpy, cv2 and yaml, and must run on Python 3.10, which is what CI uses. Write small algorithms (the Hungarian method, polygon maths) in numpy. If another package is truly needed, don't install it: guard the import so the test reports a skip, and record it under "Decisions" and "Needs a person".
- Finish what you start: no placeholder bodies, no `NotImplementedError`, no TODO comments standing in for spec items. If part of a feature can't be done, say so in the evidence and finish the row as PARTIAL.
- Don't add process of your own (trackers, monitors, extra orchestration) beyond what the spec asks for.
- Keep command output short. Send long output (sim logs, full test runs) to a file under `sim/.state/` and read its tail or grep it.

## Token and time budget

- Usage limits are shared with the lead's other work, so spend tokens and wall-clock time only where they buy proof.
- Run the sim only when the row's `proven_in` includes Sim, the change touches flight code, launch or sequence files, or a gate is due. Unit-level features get unit tests only; the flags-off baseline is re-checked by the G0-W gates at wave ends.
- One sim run proves a scenario. Repeat runs only where a pass rule asks for several.
- While iterating, run only the new or changed test files (`python3 -m pytest -q <file>`). Run the full strict check once, before the commit.
- Run long commands in the background and wait for the completion notice; don't poll with sleep or wait loops. Afterwards read only the summary lines or the tail of the log.
- Read the spec section, the changed files and log tails, not whole large files.
- Reviewers get the spec section and the diff path only. A second round re-checks only the rows the first round flagged; skip it when the fixes were tests or wording only.
- Keep notes files and reports short; don't restate the spec.
- Stop background tasks as soon as they're no longer needed.

## Gates

`next` returns `GATE <name>` when a gate's features are finished. Run it with the `/gate` skill.

- A failing gate is work: find the cause, fix it in the feature it belongs to (commit with that feature's ID), and run the gate again. After three fix-and-rerun cycles on one gate, record it FAIL with the evidence, mark the suspected feature BLOCKED, and carry on.
- G0 and its re-checks (G0-W2, G0-W3, G0-W4) prove that flags off still equals `comp-2026`. If one of them fails twice, write a STOP line: the fallback itself is broken.
- G7 and G8 need field data and the aircraft; they stay WAITING.

## The training-data hold

There is no 2027 training data yet. Until the lead lifts the hold:

- The DAT lane and the engine builder (PLT-04) are HOLD-DATA. Don't train models, convert or download datasets, or build TensorRT engines.
- Anything that uses a model is built against stand-ins, each behind its own flag and named as a stand-in:
  - **Oracle detector, sim only** (built under LOC-5). It projects the true object poses from `/sim/truth/poses` through the camera model, adds pixel noise, misses and false alarms, and publishes on the FND-02 detection contract. It refuses to start unless the node uses sim time and has a sim-only parameter set, so it can never run on the aircraft. Flag: `features.sim_oracle_detector`.
  - **Classical detectors** that need no training: the 2026 bullseye colour detector, the green X detector (REC-1), the ring-fit centre finder (CEN-13).
  - **Template digit classifier.** LOC-6 defines a `DigitClassifier` interface; the stand-in matches the synthetic marker styles from `tools/synth_targets.py`. The trained network replaces it later without changing the pipeline.
- Gate G4 runs on the oracle detector, and its result file says so.

## Teammates' assignments

Issues #4 to #7 (milestone "Nov 21 baseline") are teammates' work: #4 capture time and frame-time tilt (overlaps FND-01, FND-04, FND-05), #5 the digit reader (LOC-6), #6 the green X detector (REC-1), #7 synthetic data v0 (DAT-06, on hold anyway).

- Before starting one of those features, check `origin/main` and `gh pr list` for the teammate's module. If it has merged, build on it. If a pull request is open, build the feature to its spec without touching their branch, and note the overlap under "Decisions" so the lead can choose.
- Leave teammates' branches and pull requests alone: no edits, rebases or comments.

## When to stop

Write a `- STOP YYYY-MM-DD: <exactly what is needed>` line at the top of the build log, commit, push, and end the turn with the report when:

- Going on would break a `CLAUDE.md` rule: changing a tuned value or a fixed interface (mission API, 20 Hz tick, `fake_drop`, failure policies), committing something that must not be public, or needing `sudo`, a system package or a change outside the repo.
- A decision is needed that the spec does not settle, could reasonably go either way, and would be expensive to undo: a message or interface other features build on (beyond adding fields), a safety limit, or anything that changes flight behaviour with flags off.
- The argus sim won't start after one attempt at each fix in `sim/README.md`. The club simulator (`sim/gps/`, `~/Drone_Sim_Docker`) is started by a person; the session doesn't start it.
- A G0 gate failed twice (see "Gates").
- `next` says `NONE: nothing runnable`.

Everything else is not a stop:

- A small choice the spec leaves open: pick the option closest to the spec, record it under "Decisions", and carry on.
- A feature that won't come together: block it and take the next row.
- An action that auto mode blocks: don't retry it another way. Record it under "Needs a person", block the feature if it depends on it, and carry on.

## Outside this session's job

These belong to the lead or to other tools. The run settings deny most of them outright:

- Pushing to `main` or any branch other than the integration branch, force-pushing, merging, approving or commenting on pull requests: `main` only changes through reviewed pull requests.
- `--no-verify`, waiver markers, or editing FEATURES.md, SESSION.md, RUN.md, the ledger's lead fields, `.claude/`, `.githooks/`, `.github/`, `CLAUDE.md` or `scripts/pr_checks.py`: those are the guardrails.
- Setting a flag to true in a committed sequence or params YAML. Scenarios turn flags on only in their generated run files under `sim/.state/`.
- Changing what a 2026 code path does with flags off, or retiring any of it (LOC-14 needs the lead).
- Committing datasets, frames, bags, flight logs, model weights or engines: the repo is public and has a 5 MiB file limit.
- Deleting files (`rm -r`, `git clean`, `reset --hard`): leave scratch under `sim/.state/`.
- Running two Gazebo sims at once, or touching real hardware.

## Landing on main

The integration branch is never merged as a whole. Between waves or after the session, with no `/build` running in that checkout, the lead runs `/land`, which puts the next landing group's commits on a fresh `<area>/land-<group>` branch from `origin/main`. The lead then runs `/ship` and reviews the pull request. The session itself doesn't wait for landing.
