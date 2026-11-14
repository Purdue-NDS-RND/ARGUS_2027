---
name: land
description: Prepare the next landing group from the platform/build-2027 integration branch as its own branch off origin/main, ready for the lead to run /ship. Never opens, merges or approves a pull request. Person-invoked only.
disable-model-invocation: true
argument-hint: "[<group, e.g. L06 or CEN-14>]"
---

Prepare a landing group for review. Group, if given: $ARGUMENTS

The integration branch is never merged whole. Each landing group (the `land` field in `docs/build/features.json`; a feature whose group is its own ID lands alone) becomes one branch off `origin/main` holding only that group's commits, so the lead reviews a few related features at a time. What has already landed is recorded on `main` in `docs/build/landed.txt`, one line per integration-branch commit: `<sha> <ID or gate> <group>`. Stop and report at any "stop".

## 1. Preconditions

- Run this only when no `/build` session is working in this checkout; it switches branches.
- `git fetch origin`. Stop if `git status --short` shows changes.
- Read the ledger and the landed list from the remote branches, not the working tree:
  `git show origin/platform/build-2027:docs/build/features.json` and `git show origin/main:docs/build/landed.txt` (the file may not exist yet: nothing has landed).

## 2. Choose the group

- If a group was given, use it. Otherwise take the first group, in ledger order, that has features not yet in landed.txt.
- Every feature in the group must be DONE or PARTIAL. A BLOCKED feature that nothing else in the group depends on is left out and named in the report; otherwise stop and say which rows are missing.
- Every dependency outside the group must already be in landed.txt (`python3 tools/build_ledger.py --ledger <copy of the remote ledger> group <name>` lists them). If one is only on an open landing branch, stop: that pull request has to merge first.
- Branch: `<area>/land-<group in lower case>`, where the area is `sim` for SIM features, `centring` for CEN, `localisation` for LOC, `recovery` for REC and `platform` for everything else (the area of most features in the group). Stop if that branch already exists on origin and its pull request hasn't merged.

## 3. Collect the commits

In commit order (one `git log --reverse` over all patterns, not feature by feature), from `origin/platform/build-2027`, every commit whose subject starts with:

- `<ID>:` for each feature in the group,
- `<ID>:` for features of already landed groups, if that commit's sha is not in landed.txt (late fixes),
- `G0:` … `G6:` and `G0-W…:` (gate results) not in landed.txt.

```bash
git log --reverse --format='%H %s' --grep '^FND-04:' --grep '^FND-05:' --grep '^G[0-9]' origin/platform/build-2027
```

Filter the list against landed.txt, then show it to the lead before applying it.

## 4. Build the branch

```bash
git switch -c <branch> origin/main
for sha in <the collected shas, oldest first>; do
  git cherry-pick --no-commit "$sha" || true
  git checkout origin/main -- docs/build/PROGRESS.md docs/build/features.json
  if git diff --cached --quiet && [ -z "$(git diff --name-only --diff-filter=U)" ]; then git cherry-pick --quit; continue; fi
  git commit -C "$sha"
done
```

- `PROGRESS.md` and `features.json` stay as they are on main; the checkout above resolves their expected conflicts.
- A conflict in `scripts/check.sh`: keep main's lines plus only this commit's own new `run_test` lines.
- Any other conflict: keep both behaviours if that is clearly right and the tests agree; otherwise `git cherry-pick --abort`, delete the branch, and stop with the conflict shown.
- Last, append one line per applied sha to `docs/build/landed.txt` (`<sha> <ID or gate> <group>`, the integration-branch sha, not the new one) and commit it as `Land <group>: record landed commits`.

## 5. Check and hand over

```bash
CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh
```

It must end in exactly `OK`. If it fails, show the failing part and stop; don't fix code here.

Report to the lead:

- the branch, the features in it (with their notes files `docs/build/notes/<ID>.md`), any late fixes and gate results it carries, and any BLOCKED feature left out;
- the check result;
- "Run `/ship` on this branch. Start 'What changed' with `Landing group <group>: <IDs>`."

Don't push the branch, open the pull request, merge or approve anything here: `/ship` and the lead do that. This skill never commits to `platform/build-2027`.
