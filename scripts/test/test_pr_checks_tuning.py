"""Tests for the tuning gate in scripts/pr_checks.py. Needs PyYAML."""

import io
import subprocess
from pathlib import Path

import pytest

import pr_checks

PARAMS = """\
package_drop:
  ros__parameters:
    drop_altitude_m: 6.0
    descent_rate_mps: 0.5
    mission_modules: [a, b]
"""

SEQUENCE = """\
mission_sequence:
  on_failure: abort_and_rtl
  missions:
    - type: takeoff
      name: takeoff
      config:
        target_altitude_m: 15.0
    - type: package_drop
      name: package_drop
      config:
        drop_altitude_m: 6.0
"""


def problems(old, new, path="src/pkg/config/params/p.yaml"):
    return pr_checks.tuning_problems(path, old.encode(), new.encode())


def test_changed_value_fails():
    found = problems(PARAMS, PARAMS.replace("drop_altitude_m: 6.0", "drop_altitude_m: 5.0"))
    assert len(found) == 1
    assert "package_drop.ros__parameters.drop_altitude_m changed 6.0 -> 5.0" in found[0]


def test_new_key_passes():
    assert problems(PARAMS, PARAMS + "    features.use_frame_time_pose: false\n") == []


def test_comment_and_format_changes_pass():
    edited = PARAMS.replace("descent_rate_mps: 0.5", "descent_rate_mps: 0.50  # m/s")
    assert problems(PARAMS, edited) == []


def test_removed_key_fails():
    assert problems(PARAMS, PARAMS.replace("    descent_rate_mps: 0.5\n", ""))


def test_type_change_fails():
    assert problems(PARAMS, PARAMS.replace("6.0", "6"))


def test_list_append_passes_and_list_edit_fails():
    assert problems(PARAMS, PARAMS.replace("[a, b]", "[a, b, c]")) == []
    assert problems(PARAMS, PARAMS.replace("[a, b]", "[a, c]"))


def test_missions_are_keyed_by_name():
    reordered = """\
mission_sequence:
  on_failure: abort_and_rtl
  missions:
    - type: package_drop
      name: package_drop
      config:
        drop_altitude_m: 6.0
    - type: takeoff
      name: takeoff
      config:
        target_altitude_m: 15.0
"""
    assert problems(SEQUENCE, reordered) == []
    found = problems(SEQUENCE, SEQUENCE.replace("target_altitude_m: 15.0", "target_altitude_m: 12.0"))
    assert found and "missions[name=takeoff].config.target_altitude_m" in found[0]


def test_unparseable_head_fails():
    found = problems(PARAMS, PARAMS + "  bad: [unclosed\n")
    assert found and "does not parse" in found[0]


# --- Gate, end to end -----------------------------------------------------------


def git(repo, *args):
    return subprocess.run(
        ["git", *args], cwd=repo, check=True, capture_output=True, text=True
    ).stdout.strip()


@pytest.fixture
def repo(tmp_path):
    git(tmp_path, "init", "-q", "-b", "main")
    git(tmp_path, "config", "user.name", "Test Student")
    git(tmp_path, "config", "user.email", "student@example.edu")
    git(tmp_path, "config", "commit.gpgsign", "false")
    (tmp_path / "src/pkg/config/params").mkdir(parents=True)
    (tmp_path / "src/pkg/config/params/p.yaml").write_text(PARAMS)
    (tmp_path / ".github").mkdir()
    (tmp_path / ".github/other.yaml").write_text("a: 1\n")
    git(tmp_path, "add", "-A")
    git(tmp_path, "commit", "-q", "--no-verify", "-m", "Initial commit")
    git(tmp_path, "update-ref", "refs/remotes/origin/main", "main")
    git(tmp_path, "switch", "-q", "-c", "platform/tune")
    return tmp_path


def run(repo, body="Tune it."):
    env = {
        "GITHUB_EVENT_NAME": "pull_request",
        "PR_BASE_SHA": git(repo, "rev-parse", "origin/main"),
        "PR_HEAD_SHA": git(repo, "rev-parse", "HEAD"),
        "PR_TITLE": "Tune",
        "PR_BODY": body,
    }
    out = io.StringIO()
    return pr_checks.run_gates(Path(repo), env, out=out), out.getvalue()


def edit_and_commit(repo, path, text):
    (Path(repo) / path).write_text(text)
    git(repo, "commit", "-q", "--no-verify", "-am", "Edit")


def test_gate_fails_on_changed_value_and_marker_waives(repo):
    edit_and_commit(repo, "src/pkg/config/params/p.yaml", PARAMS.replace("0.5", "0.4"))
    code, out = run(repo)
    assert code == 1
    assert "FAIL tuning" in out
    code, out = run(repo, body="[tuning: issue #12 asks for a slower descent]")
    assert code == 0
    assert "WAIVED tuning" in out


def test_gate_ignores_yaml_outside_config(repo):
    edit_and_commit(repo, ".github/other.yaml", "a: 2\n")
    assert run(repo)[0] == 0


def test_gate_fails_on_deleted_config(repo):
    git(repo, "rm", "-q", "src/pkg/config/params/p.yaml")
    git(repo, "commit", "-q", "--no-verify", "-m", "Remove")
    assert run(repo)[0] == 1
