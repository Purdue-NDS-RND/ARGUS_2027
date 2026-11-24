"""OPS-06: the code freeze check (tools/freeze_check.py), on small git repositories made here. No ROS."""

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1]
REPO = TOOLS.parent
sys.path.insert(0, str(TOOLS))

import freeze_check as fc  # noqa: E402

GIT = shutil.which("git")
SEQUENCE = "mission_sequence:\n  features:\n    use_frame_time_pose: false\n  missions:\n    - type: package_drop\n      config:\n        drop_altitude_m: 5.0   # tuned\n        kp: 0.4\n"


def git(repo, *args):
    env = {**os.environ, "GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@t", "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@t"}
    return subprocess.run([GIT, "-C", str(repo), *args], capture_output=True, text=True, env=env, check=True).stdout.strip()


@pytest.fixture()
def repo(tmp_path):
    """A repository at its release candidate: a mission, a tuned sequence, a profile, a day file, a document."""
    root = tmp_path / "repo"
    files = {
        "src/drone_mission_demo/drone_mission_demo/mission.py": "GAIN = 1\n",
        "src/drone_mission_demo/config/sequences/package_drop_live.yaml": SEQUENCE,
        "profiles/comp_2027.yaml": "description: x\nenable: []\n",
        "field/day1.yaml": "name: day1\n",
        "docs/ops/notes.md": "notes\n",
        "tools/go.py": "print('go')\n",
    }
    for name, text in files.items():
        (root / name).parent.mkdir(parents=True, exist_ok=True)
        (root / name).write_text(text)
    git(tmp_path, "init", "-q", "-b", "main", str(root))
    git(root, "add", "."), git(root, "commit", "-q", "-m", "release candidate")
    git(root, "tag", "comp-2027-rc1")
    return root


def commit(repo, name, text, message="field change"):
    (repo / name).parent.mkdir(parents=True, exist_ok=True)
    (repo / name).write_text(text)
    git(repo, "add", name), git(repo, "commit", "-q", "-m", message)


def test_nothing_changed_holds_the_freeze(repo, capsys):
    assert fc.main(["--repo", str(repo)]) == 0
    assert capsys.readouterr().out.strip().splitlines()[-1] == "FREEZE HOLDS: nothing has changed"


def test_a_tuned_value_a_profile_a_day_file_and_a_document_may_change_and_each_tuned_value_is_listed(repo, capsys):
    commit(repo, "src/drone_mission_demo/config/sequences/package_drop_live.yaml", SEQUENCE.replace("drop_altitude_m: 5.0", "drop_altitude_m: 4.5").replace("kp: 0.4", "kp: 0.45"),
           "Lower the drop and stiffen centring\n\n[tuning: 6 m/s wind on the day; the payload drifted 0.8 m]")  # fmt: skip
    commit(repo, "profiles/comp_2027.yaml", "description: x\nenable: [use_frame_time_pose]\n")
    commit(repo, "field/day2.yaml", "name: day2\n")
    commit(repo, "docs/ops/notes.md", "more notes\n")
    commit(repo, "README.md", "readme\n")
    assert fc.main(["--repo", str(repo)]) == 0
    out = capsys.readouterr().out
    assert "since comp-2027-rc1, up to HEAD: 0 code, 1 tuning, 2 field, 2 docs" in out
    assert "tuning  src/drone_mission_demo/config/sequences/package_drop_live.yaml" in out
    assert "mission_sequence.missions[0].config.drop_altitude_m: 5.0 -> 4.5" in out and "mission_sequence.missions[0].config.kp: 0.4 -> 0.45" in out
    assert "reason: 6 m/s wind on the day; the payload drifted 0.8 m" in out
    assert "field   profiles/comp_2027.yaml" in out and "field   field/day2.yaml" in out
    assert out.strip().splitlines()[-1] == "FREEZE HOLDS: nothing but tuning, field files and docs has changed"


def test_a_tuned_value_needs_its_reason_in_the_commit_before_it_is_flown(repo, capsys):
    """The spec audit: "fly, and open the pull request that evening" would fly a tuned value with no waiver on record anywhere."""
    path = "src/drone_mission_demo/config/sequences/package_drop_live.yaml"
    commit(repo, path, SEQUENCE.replace("kp: 0.4", "kp: 0.5"), "try a higher gain")
    assert fc.main(["--repo", str(repo)]) == 1
    out = capsys.readouterr().out
    assert "with no [tuning: <reason>] in its message" in out and "config.kp: 0.4 -> 0.5" in out and out.strip().splitlines()[-1].startswith("FREEZE BROKEN: 1")
    commit(repo, path, SEQUENCE.replace("kp: 0.4", "kp: 0.45"), "[tuning: 0.5 oscillated in the hover test]")
    assert fc.main(["--repo", str(repo)]) == 1  # the first commit still has none: every tuning commit says why
    capsys.readouterr()
    # An edit that is not committed has no reason on record.
    other = tmp = repo
    (tmp / path).write_text(SEQUENCE.replace("kp: 0.4", "kp: 0.6"))
    assert fc.main(["--repo", str(other), "--worktree"]) == 1
    assert "edited and not committed: commit it with [tuning: <reason>] in the message before it is flown" in capsys.readouterr().out


@pytest.mark.parametrize(
    "name, text, word",
    [
        ("src/drone_mission_demo/drone_mission_demo/mission.py", "GAIN = 2\n", "src/drone_mission_demo/drone_mission_demo/mission.py (changed)"),
        ("tools/go.py", "print('went')\n", "tools/go.py (changed)"),
        ("src/drone_mission_demo/drone_mission_demo/new_mission.py", "x = 1\n", "new_mission.py (added)"),
        ("scripts/check.sh", "exit 0\n", "scripts/check.sh (added)"),
        ("platform/fcu/argus_2027.param", "FENCE_ENABLE,0\n", "platform/fcu/argus_2027.param (added)"),  # the flight controller's baseline is not a field tuning file
        ("src/drone_mission_demo/config/sequences/package_drop_live.yaml", "mission_sequence: [unclosed\n", "does not parse"),
    ],
)
def test_code_and_anything_else_breaks_the_freeze_and_is_named(repo, name, text, word, capsys):
    commit(repo, name, text, "[tuning: a reason does not make code a tuning file]")
    assert fc.main(["--repo", str(repo)]) == 1
    out = capsys.readouterr().out
    assert word in out and out.strip().splitlines()[-1] == "FREEZE BROKEN: 1 change(s) the freeze does not allow"


def test_a_removed_file_and_keys_added_or_removed_in_a_tuning_file_are_said(repo, capsys):
    git(repo, "rm", "-q", "tools/go.py"), git(repo, "commit", "-q", "-m", "gone")
    commit(repo, "src/drone_mission_demo/config/sequences/package_drop_live.yaml", SEQUENCE.replace("        kp: 0.4\n", "        ki: 0.1\n"), "[tuning: integral term for the wind]")
    assert fc.main(["--repo", str(repo)]) == 1
    out = capsys.readouterr().out
    assert "tools/go.py (removed)" in out and "config.kp: removed (was 0.4)" in out and "config.ki: added (0.1)" in out


def test_a_comment_only_edit_of_a_tuning_file_differs_in_no_value(repo, capsys):
    commit(repo, "src/drone_mission_demo/config/sequences/package_drop_live.yaml", SEQUENCE.replace("# tuned", "# tuned on the field"))
    assert fc.main(["--repo", str(repo)]) == 0
    assert "(comments or layout only: no value differs)" in capsys.readouterr().out


def test_the_working_tree_is_looked_at_only_when_asked(repo, capsys):
    (repo / "src/drone_mission_demo/drone_mission_demo/mission.py").write_text("GAIN = 3\n")  # edited, not committed
    (repo / "tools/new_tool.py").write_text("x = 1\n")  # new, not added
    (repo / "src/drone_mission_demo/config/sequences/package_drop_live.yaml").write_text(SEQUENCE.replace("kp: 0.4", "kp: 0.5"))
    assert fc.main(["--repo", str(repo)]) == 0  # what is committed is the release candidate
    capsys.readouterr()
    assert fc.main(["--repo", str(repo), "--worktree"]) == 1
    out = capsys.readouterr().out
    assert "up to the working tree" in out and "mission.py (changed)" in out and "tools/new_tool.py (added)" in out and "config.kp: 0.4 -> 0.5" in out
    assert "FREEZE BROKEN: 3" in out  # the code edit, the new tool, and the tuned value that is not committed


def test_no_freeze_tag_yet_is_exit_2_and_says_how_the_tag_is_made(repo, capsys):
    assert fc.main(["--repo", str(repo), "--since", "comp-2028-rc1"]) == 2
    err = capsys.readouterr().err
    assert "there is no tag comp-2028-rc1 yet" in err and "git tag -a comp-2028-rc1" in err
    assert fc.main(["--repo", str(repo.parent / "nowhere")]) == 2


def test_what_counts_as_a_tuning_file_is_the_pull_request_gates_own_rule():
    """scripts/pr_checks.py asks a [tuning: reason] waiver for exactly these files; the freeze lets exactly these be tuned."""
    text = (REPO / "scripts" / "pr_checks.py").read_text()
    assert f'TUNING_FILE = re.compile(r"{fc.TUNING_FILE.pattern}")' in text
    assert fc.kind_of("src/drone_mission_demo/config/params/live_target_mission.yaml") == "tuning" and fc.kind_of("src/drone_mission_demo/config/sequences/package_drop_live.yaml") == "tuning"
    assert fc.kind_of("profiles/delivery_estimator.yaml") == "field" and fc.kind_of("field/example_day.yaml") == "field" and fc.kind_of("tools/score_model.yaml") == "field"
    assert fc.kind_of("docs/ops/readiness_video.md") == "docs" and fc.kind_of("src/drone_utils/README.md") == "docs"
    for code in ("src/drone_mission_core/drone_mission_core/mission_executor.py", "tools/go.py", "sim/scenarios/delivery/baseline.yaml", "platform/fcu/argus_2027.param", "scripts/check.sh", "field/example_day/site.yaml",
                 "src/drone_mission_demo/launch/argus_bringup.launch.py", "config/mavros/argus_mavros.yaml", ".github/workflows/check.yml"):  # fmt: skip
        assert fc.kind_of(code) == "code", code


def test_on_this_repository_the_tool_runs_and_the_freeze_tag_is_not_set_by_the_build(capsys):
    assert fc.main([]) in (0, 1, 2)  # 2 until the lead sets the tag at the freeze
    assert fc.FREEZE_TAG == "comp-2027-rc1"
