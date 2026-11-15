"""sim/nightly.sh: tier B, the nightly sim run on a club machine.

Each night it takes a fresh checkout of main, flies the nightly suite there
and posts the table to a GitHub issue. The tests run it against a small
stand-in repository and a stand-in `gh`, so no sim, network or token is
involved. No ROS needed.
"""

import os
import subprocess
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
NIGHTLY = SIM / "nightly.sh"

FAKE_RUNNER = """#!/usr/bin/env bash
# Stand-in for sim/run_scenarios.sh on main: writes a summary and exits as told.
out="$(dirname "$0")/.state/suites/nightly-test"
mkdir -p "$out"
printf '# Suite nightly\\n\\n| Scenario | Seed |\\n| --- | --- |\\n| delivery/baseline | 1 |\\n\\n%s\\n' "$(cat "$(dirname "$0")/outcome.txt")" > "$out/summary.md"
echo "ran: $*"
echo "Summary: sim/.state/suites/nightly-test/summary.md"
grep -q FAIL "$(dirname "$0")/outcome.txt" && exit 1
exit 0
"""
FAKE_GH = """#!/usr/bin/env bash
echo "$*" >> "$GH_LOG"
for arg in "$@"; do [[ -f $arg ]] && cp "$arg" "$GH_LOG.body"; done
exit "${GH_EXIT:-0}"
"""


def git(repo, *args):
    return subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True, text=True).stdout.strip()


@pytest.fixture
def origin(tmp_path):
    """A repository whose main has a stand-in scenario runner."""
    repo = tmp_path / "origin"
    (repo / "sim").mkdir(parents=True)
    git(repo, "init", "-q", "-b", "main")
    git(repo, "config", "user.email", "dev@example.org")
    git(repo, "config", "user.name", "Dev")
    (repo / "sim" / "run_scenarios.sh").write_text(FAKE_RUNNER)
    (repo / "sim" / "run_scenarios.sh").chmod(0o755)
    (repo / "sim" / "outcome.txt").write_text("3 runs: 3 OK\n")
    (repo / ".gitignore").write_text("sim/.state/\n")
    git(repo, "add", "-A")
    git(repo, "commit", "-q", "-m", "main")
    return repo


def nightly(origin, tmp_path, *args, issue="42", gh_exit="0", ref="main"):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir(exist_ok=True)
    (bin_dir / "gh").write_text(FAKE_GH)
    (bin_dir / "gh").chmod(0o755)
    env = {
        **os.environ,
        "PATH": f"{bin_dir}:{os.environ['PATH']}",
        "NIGHTLY_REPO": str(origin),
        "NIGHTLY_REF": ref,
        "NIGHTLY_DIR": str(tmp_path / "nightly"),
        "GH_LOG": str(tmp_path / "gh.log"),
        "GH_EXIT": gh_exit,
    }
    env.pop("ARGUS_NIGHTLY_ISSUE", None)
    if issue:
        env["ARGUS_NIGHTLY_ISSUE"] = issue
    done = subprocess.run(["bash", str(NIGHTLY), *args], capture_output=True, text=True, env=env, timeout=120)
    log = tmp_path / "gh.log"
    calls = log.read_text().splitlines() if log.exists() else []
    body = Path(str(log) + ".body").read_text() if Path(str(log) + ".body").exists() else ""
    return done, calls, body


def test_nightly_flies_main_in_a_fresh_worktree_and_posts_the_table(origin, tmp_path):
    done, calls, body = nightly(origin, tmp_path)
    assert done.returncode == 0, done.stdout + done.stderr
    assert "ran: nightly" in (tmp_path / "nightly" / "run.log").read_text()  # the suite, on main's own runner
    assert len(calls) == 1 and calls[0].startswith("issue comment 42 --body-file ")
    assert git(origin, "rev-parse", "--short", "main") in body
    assert "| delivery/baseline | 1 |" in body and "3 runs: 3 OK" in body
    assert "every flight OK" in body
    # The checkout is main's commit, apart from the caller's working tree.
    assert git(tmp_path / "nightly" / "worktree", "rev-parse", "HEAD") == git(origin, "rev-parse", "main")


def test_nightly_takes_the_new_main_each_night(origin, tmp_path):
    nightly(origin, tmp_path)
    (origin / "sim" / "outcome.txt").write_text("3 runs: 2 OK, 1 FAIL\n")
    git(origin, "commit", "-q", "-am", "a regression lands on main")
    done, calls, body = nightly(origin, tmp_path)
    assert done.returncode == 1  # a flight that is not OK fails the night
    assert len(calls) == 2  # ...and is still reported
    assert "2 OK, 1 FAIL" in body and "NOT every flight was OK" in body
    assert git(origin, "rev-parse", "--short", "main") in body


def test_nightly_fetches_origin_so_a_stale_checkout_still_flies_todays_main(origin, tmp_path):
    """The real setting: the club machine's checkout was cloned days ago, and main has moved since."""
    clone = tmp_path / "clone"
    subprocess.run(["git", "clone", "-q", str(origin), str(clone)], check=True)
    (origin / "sim" / "outcome.txt").write_text("3 runs: 3 OK (after today's merge)\n")
    git(origin, "commit", "-q", "-am", "merged today")
    assert git(clone, "rev-parse", "origin/main") != git(origin, "rev-parse", "main")  # the clone has not heard yet
    done, _, body = nightly(clone, tmp_path, ref="origin/main")
    assert done.returncode == 0, done.stdout + done.stderr
    assert git(tmp_path / "nightly" / "worktree", "rev-parse", "HEAD") == git(origin, "rev-parse", "main")
    assert "after today's merge" in body


def test_nightly_without_an_issue_prints_the_report_and_posts_nothing(origin, tmp_path):
    done, calls, _ = nightly(origin, tmp_path, issue="")
    assert done.returncode == 0
    assert calls == []
    assert "3 runs: 3 OK" in done.stdout and "ARGUS_NIGHTLY_ISSUE" in done.stdout


def test_nightly_says_so_when_the_post_fails(origin, tmp_path):
    done, calls, _ = nightly(origin, tmp_path, gh_exit="1")
    assert done.returncode == 1
    assert "could not post" in done.stdout + done.stderr


def test_nightly_dry_run_changes_nothing(origin, tmp_path):
    done, calls, _ = nightly(origin, tmp_path, "--dry-run")
    assert done.returncode == 0
    assert calls == [] and not (tmp_path / "nightly").exists()
    assert "would" in done.stdout


def test_nightly_takes_another_suite_and_refuses_unknown_options(origin, tmp_path):
    done, _, _ = nightly(origin, tmp_path, "--suite", "smoke")
    assert "ran: smoke" in (tmp_path / "nightly" / "run.log").read_text()
    assert nightly(origin, tmp_path, "--bogus")[0].returncode == 2


def test_nightly_never_echoes_a_token(origin, tmp_path):
    code = [line for line in NIGHTLY.read_text().splitlines() if not line.lstrip().startswith("#")]
    assert not any("set -x" in line or "TOKEN" in line for line in code)  # gh reads its own token; the script never touches it
    assert subprocess.run(["bash", "-n", str(NIGHTLY)]).returncode == 0
