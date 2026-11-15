"""tools/flag_profile.py: a flag profile as the value of the feature_overrides launch argument."""

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools" / "flag_profile.py"


def run(*args):
    return subprocess.run([sys.executable, str(TOOL), *map(str, args)], capture_output=True, text=True)


def profile(tmp_path, text):
    path = tmp_path / "profile.yaml"
    path.write_text(text)
    return path


def test_the_2026_profile_prints_nothing_to_turn_on():
    done = run(ROOT / "profiles" / "comp_2026.yaml")
    assert done.returncode == 0, done.stderr
    assert done.stdout == "\n"  # an empty feature_overrides: the code as flown in 2026


def test_profile_prints_its_flags_comma_separated_in_order(tmp_path):
    done = run(profile(tmp_path, "enable: [use_frame_time_pose, another_flag]\n"))
    assert done.returncode == 0, done.stderr
    assert done.stdout == "use_frame_time_pose,another_flag\n"


def test_sequence_check_accepts_declared_flags_and_refuses_the_rest(tmp_path):
    good = profile(tmp_path, "enable: [use_frame_time_pose]\n")
    assert run(good, "--sequence", "package_delivery_demo.yaml").stdout == "use_frame_time_pose\n"
    bad = run(profile(tmp_path, "enable: [use_frame_time_pose, not_in_that_sequence]\n"), "--sequence", "package_delivery_demo.yaml")
    assert bad.returncode == 2 and "not_in_that_sequence" in bad.stderr and bad.stdout == "PROFILE_ERROR\n"
    assert run(good, "--sequence", "no_such_sequence.yaml").returncode == 2


def test_profile_with_a_missing_dependency_is_refused(tmp_path):
    done = run(profile(tmp_path, "enable: [drop_release_gates]\n"))
    assert done.returncode == 2
    assert "drop_release_gates requires use_target_estimator" in done.stderr
    assert run(profile(tmp_path, "enable: [use_target_estimator, drop_release_gates]\n")).returncode == 0


def test_a_bad_profile_makes_the_launch_line_fail_instead_of_flying_with_flags_off(tmp_path):
    """feature_overrides:="$(flag_profile.py ...)" must not become an empty string when the profile is wrong."""
    sys.path.insert(0, str(ROOT / "src" / "drone_mission_core"))
    from drone_mission_core.features import FeatureFlagError, parse_feature_overrides

    for bad in (profile(tmp_path, "enable: [drop_release_gates]\n"), tmp_path / "missing.yaml"):
        done = run(bad)
        assert done.returncode == 2
        assert done.stdout.strip() == "PROFILE_ERROR"
        try:
            parse_feature_overrides(done.stdout.strip())
        except FeatureFlagError:
            pass
        else:
            raise AssertionError("the executor would have started with every flag off")


def test_bad_profile_file_is_refused(tmp_path):
    assert run(tmp_path / "missing.yaml").returncode == 2
    assert run(profile(tmp_path, "enable: use_frame_time_pose\n")).returncode == 2
    assert run().returncode == 2


def test_every_way_of_failing_prints_the_word_the_executor_refuses(tmp_path):
    good = profile(tmp_path, "enable: [use_frame_time_pose]\n")
    broken_sequence = tmp_path / "sequence.yaml"
    broken_sequence.write_text("mission_sequence: [unclosed\n")
    not_a_mapping = tmp_path / "list.yaml"
    not_a_mapping.write_text("- just\n- a list\n")
    for args in ((), (good, "--sequnce", "package_delivery_demo.yaml"), (good, "--sequence", broken_sequence), (good, "--sequence", not_a_mapping), (good, "--sequence")):
        done = run(*args)
        assert done.returncode == 2, args
        assert done.stdout.strip() == "PROFILE_ERROR", (args, done.stdout, done.stderr)


def test_list_shows_the_shipped_profiles():
    done = run("--list")
    assert done.returncode == 0, done.stderr
    assert "profiles/comp_2026.yaml: no flags" in done.stdout
