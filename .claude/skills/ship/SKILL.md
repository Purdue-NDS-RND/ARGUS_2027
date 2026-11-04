---
name: ship
description: Open a pull request for the current branch, the only way PRs are opened in this repo. Runs scripts/check.sh, runs the flight-guardian on flight code and config changes, commits, pushes and opens the PR with the template filled in from real output. Never merges or approves.
disable-model-invocation: true
argument-hint: "[issue-number]"
---

Ship the current branch as a pull request. The issue number, if given: $ARGUMENTS

Work through these steps in order. Stop and report to the user at any "stop". Never merge, never approve, and never push to `main`.

## 1. Preconditions

```bash
git fetch origin
git branch --show-current
git status --short
```

- If the branch is `main`, or HEAD is detached: stop. Tell the user to create a branch named `<area>/<short-name>` (centring, localisation, recovery, sim, platform).
- If there is nothing to ship (no commits ahead of `origin/main` and a clean working tree): stop and say so.
- If the branch is behind `origin/main`, tell the user and ask whether to rebase before going on.

## 2. Run the checks

```bash
scripts/check.sh
```

Keep the full output; it goes in the PR. It must end in `OK` (a `SKIP: no PR context` line is expected here, because the PR doesn't exist yet). If it prints `FAILED`, stop and show the user the failing part. Don't fix code inside this skill.

If a PR gate fails and the user says a waiver is right, ask them for the reason in their own words. Never write a waiver marker (`[hygiene: waived ...]`, `[tests: n/a ...]`, `[tuning: ...]`, `[large-file: ...]`) on your own.

## 3. Flight-guardian

```bash
git diff --name-only origin/main...HEAD; git status --short
```

If any changed or new file is under `src/drone_mission_core/`, `src/drone_mission_demo/` or `src/drone_target_cv/`, or is a YAML file under any `config/` directory, run the `flight-guardian` subagent and wait for its report before going on:

- Give it the full diff against `origin/main` (committed and uncommitted: `git diff origin/main` plus the content of new untracked files), the list of changed files, and the issue text if there is one.
- Don't tell it what you think the change does or whether it is safe.

If its verdict is BLOCKER: stop. Show the user the report. Don't commit, push or open the PR.
If it reports CONCERNS: show them to the user and include them in the PR (step 6), then go on.

## 4. Commit

- Review `git status --short`. Stage the files that belong to this change by path (`git add <paths>`), not `git add -A`. Leave out anything that must not be in a public repo (secrets, datasets, bags, logs, engines) and anything unrelated to the issue; mention unrelated files to the user.
- Write a commit message: an imperative subject under 72 characters, then a short body on what changed and why. No AI attribution and no local machine details. The commit-msg hook checks this.
- If a hook rejects the commit, fix the message or the content. Don't use `--no-verify`.

## 5. Write the PR description

Read `.github/pull_request_template.md` and fill in every section from what actually happened:

- **What changed**: from the diff, in plain words.
- **Issue**: `Closes #<n>` if an issue number was given or is in the branch's commits; otherwise ask the user.
- **Flag**: the flag name and where it is set to `false`, or "none".
- **How I tested it**: the commands you ran and their output, copied from steps 2 and 3. Trim long output to the relevant lines, but don't paraphrase it. Replace absolute local paths in pasted output (home folders, temp folders) with repo-relative paths or `<tmp>`; the hygiene gate rejects them. Include the flight-guardian verdict and any CONCERNS. Tick a checkbox only when the output shows it; for "I broke the code once and saw my new test fail", ask the user unless it happened in this session.
- **Not done / follow-ups**: anything noticed but out of scope.
- **Waivers**: only markers the user gave a reason for.

Write the title and description to a temp file outside the repo (`mktemp`), never to a file in the repo.

Re-run the PR gates against the real title and description before anything leaves the machine:

```bash
PR_TITLE="<title>" PR_BODY="$(cat "$body_file")" python3 scripts/pr_checks.py
```

It must exit 0. If it fails on the title or description, reword them; if it fails on a commit message, amend the commit (step 4) and run it again.

## 6. Push and open the PR

```bash
git push -u origin "$(git branch --show-current)"
gh pr create --base main --title "<title>" --body-file "$body_file"
```

Then report to the user: the PR URL, the check.sh result, the flight-guardian verdict, and what CI is running (`gh pr checks <number>`). Stop there. Merging is for the lead after `/review-pr` and approval.
