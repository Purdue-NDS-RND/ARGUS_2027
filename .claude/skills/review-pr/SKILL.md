---
name: review-pr
description: Lead's review of a pull request by number. Reads the diff cold before the description, runs scripts/check.sh on the branch, runs the flight-guardian, and goes through docs/review-checklist.md. Reports BLOCKED, CHANGES SUGGESTED or CLEAN with findings by rule id and file:line. Only reports; never edits, comments, approves or merges.
disable-model-invocation: true
argument-hint: "<pr-number>"
---

Review pull request #$ARGUMENTS.

This skill only reports. Don't edit files, don't commit, don't post anything to GitHub (no `gh pr comment`, `gh pr review`, `gh pr merge`, or `gh api` writes), and don't approve. If there is no PR number, ask for one and stop.

## 1. Preconditions

```bash
git status --short
git branch --show-current
```

If the working tree has uncommitted changes, stop: checking out the PR would mix them in. Remember the current branch so you can return to it in step 6.

## 2. Read the diff cold

```bash
gh pr diff $ARGUMENTS --name-only
gh pr diff $ARGUMENTS
```

Before you look at the title or description, write down in a few lines what the diff actually does: which behaviour changes, behind which flag, and what is touched that a reader might not expect. This is your independent reading.

## 3. Then read the description

```bash
gh pr view $ARGUMENTS --json number,title,body,headRefName,baseRefName,author,url
```

Compare it with your reading from step 2. Anything the diff does that the description doesn't say, or anything the description claims that the diff doesn't do, is a finding under rule `scope`.

## 4. Run the checks on the branch

```bash
git fetch origin
gh pr checkout $ARGUMENTS
PR_TITLE="$(gh pr view $ARGUMENTS --json title -q .title)" \
PR_BODY="$(gh pr view $ARGUMENTS --json body -q .body)" \
scripts/check.sh
```

Keep the output. A `FAILED` result makes the verdict BLOCKED. Also note the CI status: `gh pr checks $ARGUMENTS`.

## 5. Flight-guardian and checklist

Run the `flight-guardian` subagent on the diff and wait for its report. Give it the full diff from step 2 and the list of changed files. Don't give it your reading or the PR description.

Then read `docs/review-checklist.md` and go through every rule in order: scope, tests, flags, interfaces, tuning, timing, safety, results. For each, look at the code, not the description. Use the guardian's report for `safety` and `timing`, but check its findings against the code yourself; a guardian finding you can show is wrong is not a finding.

## 6. Return to where you were

```bash
git switch <the branch from step 1>
```

## 7. Report

Reply in this shape:

```
PR #<n>: <title>
Verdict: BLOCKED | CHANGES SUGGESTED | CLEAN

What the diff does (read before the description):
<2-4 lines>

Checks: scripts/check.sh <OK | FAILED: ...>; CI <state>; flight-guardian <verdict>

Findings:
- [blocking][scope] path/to/file.py:42: <what, and why it matters>
- [suggestion][tests] path/to/test_file.py:10: <...>

Rules with no findings: <ids>
```

- BLOCKED: check.sh failed, an unresolved guardian BLOCKER, or any blocking finding.
- CHANGES SUGGESTED: only suggestions or guardian CONCERNs.
- CLEAN: no findings.

Every finding cites one rule id from the checklist and a `file:line` (for a description mismatch, cite the diff line it concerns). Then stop. Approving, commenting and merging are up to the lead.
