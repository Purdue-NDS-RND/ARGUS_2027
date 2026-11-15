# SIM-10: CI tiers and nightly runs

Flag: none (test and CI tooling) · Status: PARTIAL · Commits: `git log --oneline --grep '^SIM-10:' platform/build-2027`

## What it solves

CI ran the PR gates and the ROS-free tests, but nothing looked for secrets (a live API key once sat in a notebook), nothing noticed a test file left out of `scripts/check.sh`, the tests gate ignored `sim/`, and no sim flight ran unless a person started it. This adds the secret scan and a guard on test coverage to tier A, and the nightly script for tier B. It is PARTIAL because two pieces belong to the lead: one line in the PR gate file the session may not edit, and the nightly machine with its token.

## Architecture

- `scripts/secret_scan.py` (new, ROS-free): reads the same changes as the PR gates through `pr_checks.load_context` and fails on secret-shaped text in added lines, commit messages, the title or the description. `scripts/check.sh` runs it right after `scripts/pr_checks.py`.
- `scripts/test/test_check_coverage.py` (new): every test file is run by `scripts/check.sh` or listed as needing ROS; tests that import `cv2`, `numpy` or `yaml` are in the guarded block; every Python module under `sim/` is reached by a test.
- `sim/nightly.sh` (new): tier B. A fresh worktree of `origin/main`, that worktree's `sim/run_scenarios.sh nightly`, the table posted with `gh issue comment`.
- `CONTRIBUTING.md`: the four tiers, and what the secret scan does.

No change to `scripts/pr_checks.py`, `.github/` or `.githooks/`.

## Flow

Before: a pull request adding `api_key="..."` under `sim/` or `tools/` passed CI.

After: `scripts/check.sh` runs `scripts/secret_scan.py`; it prints `FAIL secrets` with `train.py:2: a literal value assigned to 'api_key'` (never the value) and the check ends `FAILED`. A reviewer-approved false positive is waived with `[hygiene: waived <reason>]`, the same marker as the hygiene gate.

## Code

- `scripts/secret_scan.py` `secret_findings`: token formats that identify themselves, a password in a URL, and a quoted literal with letters and digits assigned to a name that says key, secret, token or password, also in a notebook's JSON where the quotes are escaped; placeholders and environment-variable names pass.
- `scripts/secret_scan.py` `run`: the same three settings as the PR gates (pull request, push to main, local), the same exit codes, the hygiene waiver.
- `scripts/test/test_check_coverage.py` `untested_sim_modules`: follows imports from the tests through `sim/`, so a module counts if a test reaches it or runs it by file name.
- `sim/nightly.sh`: the worktree is remade every night; the report is posted even when flights fail; the script never touches the token.

## Evidence

```
$ PYTHONPATH=scripts python3 -m pytest -q -p no:cacheprovider scripts/test/test_secret_scan.py scripts/test/test_check_coverage.py sim/test/test_nightly.py
56 passed in 1.10s
$ python3 scripts/secret_scan.py
PASS secrets (local, working tree against origin/main (1b6e7e6))
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

The scan passes over everything the integration branch adds (about 12,000 lines), so it has no false positive there. `test_secret_scan.py` and `test_nightly.py` were run before their scripts existed and failed. The coverage guard failed for real twice while it was written: it caught its own test file and `test_nightly.py` missing from `scripts/check.sh`. The new tests also pass on Python 3.10, the version CI uses.

`sim/nightly.sh` has only run against a stand-in repository and a stand-in `gh` in its tests. It has not flown a suite or posted to GitHub.

## Spec check

spec-auditor, round 1: PARTIAL is the right status, with one gap the session could fix (the secret rule missed a key in a notebook's JSON, where the quotes are escaped, and let an all-capitals key through) and hand-off notes that named the wrong test, called a required edit optional, overstated the coverage guard and left tier D without an owner. Round 2: all five fixed; nothing left for the session. The rows still open are owed by the lead (the `pr_checks.py` edit, folding the scan into the hygiene gate, the nightly machine and token, the PR template line, an owner for tier D) or by later features (the tier A tests for things not built yet). By its rubric that is INCOMPLETE; the row is finished as PARTIAL.

flight-guardian did not run: no mission, executor, target CV or `config/` YAML changed.

## Not done

- **`pr_checks.py` counts `sim/` as code:** not done, so the wave-0 line "CI requires tests for `sim/`" is not met as written. The session may not edit `scripts/pr_checks.py`. The one-line change and the test to add are written out under "Needs a person" in PROGRESS.md. Until then the coverage guard catches a `sim/` module with no test at all, which is weaker than a per-pull-request rule.
- **The secret rule inside the hygiene gate:** it runs beside the gate, from `scripts/check.sh`, not inside `pr_checks.py` or the pre-commit hook. So `/ship`'s re-check of the real title and description does not scan them; CI does. Same person step.
- **Tier B for real:** the club machine, the token limited to issues, the pinned issue and the timer are person steps. The script flies `origin/main`, so it has nothing to fly until the runner has landed on `main`.
- **Tier C in the PR template:** `.github/pull_request_template.md` is the lead's; CONTRIBUTING.md and `sim/README.md` say to run `smoke`.
- **Tier D:** no script and no owner for the weekly run on the GPU desktop (a person step); the 4K stills and replay it names come with SIM-05, SIM-09 and SIM-12.
- **Tier A's other tests:** wind, GPS conversions, payload bridge, replay `core`, dataset and registry tools and model parity are registered by the features that build them (SIM-03, SIM-04, SIM-06, SIM-09, DAT lane).
