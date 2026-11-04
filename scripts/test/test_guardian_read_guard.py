"""Tests for .claude/hooks/guardian-read-guard.py, the flight-guardian's calibration-file block."""

import json
import subprocess
import sys
from pathlib import Path

import pytest

HOOK = Path(__file__).resolve().parents[2] / ".claude/hooks/guardian-read-guard.py"


def run(tmp_path, tool, **tool_input):
    event = {"tool_name": tool, "tool_input": tool_input, "cwd": str(tmp_path)}
    result = subprocess.run(
        [sys.executable, str(HOOK)],
        input=json.dumps(event),
        capture_output=True,
        text=True,
        env={"CLAUDE_PROJECT_DIR": str(tmp_path)},
    )
    return result.returncode


@pytest.fixture
def repo(tmp_path):
    (tmp_path / "docs").mkdir()
    (tmp_path / "docs/guardian-calibration.md").write_text("cases\n")
    (tmp_path / "src").mkdir()
    return tmp_path


def test_read_of_calibration_file_is_blocked(repo):
    assert run(repo, "Read", file_path=str(repo / "docs/guardian-calibration.md")) == 2
    assert run(repo, "Read", file_path="docs/guardian-calibration.md") == 2
    assert run(repo, "Read", file_path=str(repo / "docs/review-checklist.md")) == 0


def test_grep_content_over_docs_is_blocked_unless_filtered(repo):
    assert run(repo, "Grep", pattern="sleep", output_mode="content") == 2
    assert run(repo, "Grep", pattern="sleep", output_mode="content", path="docs") == 2
    assert run(repo, "Grep", pattern="sleep", output_mode="content", glob="*.md") == 2
    assert run(repo, "Grep", pattern="sleep", output_mode="content", glob="*.py") == 0
    assert run(repo, "Grep", pattern="sleep", output_mode="content", type="py") == 0
    assert run(repo, "Grep", pattern="sleep", output_mode="content", path="src") == 0
    assert run(repo, "Grep", pattern="sleep") == 0  # file names only


def test_other_tools_pass_and_errors_fail_open(repo):
    assert run(repo, "Glob", pattern="docs/*.md") == 0
    result = subprocess.run([sys.executable, str(HOOK)], input="not json", capture_output=True, text=True)
    assert result.returncode == 0
