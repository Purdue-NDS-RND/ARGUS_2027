---
name: build
description: Run the ARGUS 2027 build session over a scope of docs/build/features.json, following docs/build/SESSION.md, on the platform/build-2027 integration branch. Person-invoked only; launch it as docs/build/RUN.md describes.
disable-model-invocation: true
argument-hint: "[all | wave <n> [FND|CEN|LOC|REC|PLT|SIM|OPS] | <feature ID>]"
---

# ARGUS 2027 build session

Arguments: $ARGUMENTS

Turn them into a ledger scope and use it everywhere below: nothing or `all` → `all`; `wave 3` → `wave:3`; `wave 3 CEN` → `wave:3:CEN`; a feature ID such as `FND-04` → `id:FND-04`.

You are running unattended. Nobody answers questions turn by turn, and the context will be compacted automatically when it fills: that is expected, so keep working at the same pace and don't wrap up early because of it. Everything needed to resume is on disk: `docs/build/features.json` (read and change it only with `python3 tools/build_ledger.py`), `docs/build/PROGRESS.md` and git. The full procedure is `docs/build/SESSION.md`: read it now in full, and again after any compaction.

## The loop

1. Start of a working stretch: fetch, switch to `platform/build-2027`, `git merge --ff-only origin/platform/build-2027`, merge `origin/main` if it moved, then `python3 tools/build_ledger.py summary --scope <scope>`.
2. `python3 tools/build_ledger.py next --scope <scope>`. It prints a feature ID, `GATE <name>` or `NONE ...`.
3. `GATE <name>`: run `/gate <name>`, then go back to 2. `NONE`: give the end-of-turn report and stop.
4. Feature: `set <ID> DOING`, read its spec section in FEATURES.md, look for existing code (and teammates' work), write the tests first and see them fail.
5. Implement behind the feature's flag (default false), as "Writing the code" in SESSION.md says. No placeholders.
6. `CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh` ends in exactly `OK`. Run the sim scenario if the row's `proven_in` says Sim and the runner exists.
7. Save the diff to `sim/.state/review/<ID>.diff`. Run `flight-guardian` if flight code or config changed, and `spec-auditor` always. Wait for both verdicts, fix, re-run; two rounds at most.
8. Write `docs/build/notes/<ID>.md`, add the build-log line, `set <ID> DONE --evidence "..."` (or PARTIAL), commit `<ID>: <summary>` with everything together, push the integration branch. Back to 2.

If a feature fails three attempts, or two review rounds, run `set <ID> BLOCKED --reason "..." --attempt`, log it, and go back to 2. Stop only for the reasons under "When to stop" in SESSION.md, writing a STOP line first.

## End-of-turn report

This is also the evidence the `/goal` check reads, so print the real output, not a description of it:

1. Features finished this turn: ID, one line, notes file.
2. Gates run and their results.
3. BLOCKED rows and what would unblock each.
4. New "Needs a person" and "Decisions" entries.
5. The output of `python3 tools/build_ledger.py summary --scope <scope>`.
6. The last three lines of `CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh`.
7. The output of `git status -sb`.
