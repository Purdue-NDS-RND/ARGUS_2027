"""Feature-flag plumbing: overrides from the launch line, flag dependencies, profiles.

Flags are the fallback switch: with every flag off the drone flies the June
2026 code. So the plumbing must never turn one on by accident, must refuse a
flag nobody declared (a typo would otherwise silently fly 2026 code while the
operator believes otherwise), and must refuse to start with a flag whose
dependency is off. No ROS needed.
"""

import dataclasses
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "drone_mission_core"))
for module_name in list(sys.modules):
    if module_name == "drone_mission_core" or module_name.startswith("drone_mission_core."):
        del sys.modules[module_name]

from drone_mission_core import features  # noqa: E402
from drone_mission_core.features import (  # noqa: E402
    FLAG_REQUIRES,
    FeatureFlagError,
    apply_feature_overrides,
    get_feature_flag,
    load_profile,
    missing_requirements,
    parse_feature_overrides,
    resolve_features,
)
from drone_mission_core.registry import load_sequence_file  # noqa: E402

SEQUENCES = sorted((ROOT / "src" / "drone_mission_demo" / "config" / "sequences").glob("*.yaml"))
PROFILES = ROOT / "profiles"


# --- the override string from the launch line -----------------------------------


@pytest.mark.parametrize(
    "text, expected",
    [
        ("", {}),
        ("   ", {}),
        ("use_frame_time_pose", {"use_frame_time_pose": True}),
        ("a_flag,b_flag", {"a_flag": True, "b_flag": True}),
        (" a_flag , b_flag ,", {"a_flag": True, "b_flag": True}),
        ("a_flag=true,b_flag=false", {"a_flag": True, "b_flag": False}),
        ("a_flag=on,b_flag=off,c_flag=1,d_flag=0", {"a_flag": True, "b_flag": False, "c_flag": True, "d_flag": False}),
        ("A_Flag=TRUE", None),  # flag names are lower case, as in the YAML
    ],
)
def test_parse_feature_overrides(text, expected):
    if expected is None:
        with pytest.raises(FeatureFlagError):
            parse_feature_overrides(text)
    else:
        assert parse_feature_overrides(text) == expected


@pytest.mark.parametrize("text", ["a_flag=maybe", "a flag", "9lives", "a_flag==true", "a_flag=true=false", "features.a_flag"])
def test_parse_refuses_what_it_cannot_read(text):
    with pytest.raises(FeatureFlagError):
        parse_feature_overrides(text)


def test_parse_refuses_the_same_flag_twice_with_different_values():
    with pytest.raises(FeatureFlagError, match="a_flag"):
        parse_feature_overrides("a_flag,a_flag=false")


# --- overrides against what the sequence declares ----------------------------------


def test_override_turns_on_a_declared_flag_and_leaves_the_input_alone():
    declared = {"use_frame_time_pose": False, "other_flag": False}
    resolved = resolve_features(declared, {"use_frame_time_pose": True})
    assert resolved == {"use_frame_time_pose": True, "other_flag": False}
    assert declared == {"use_frame_time_pose": False, "other_flag": False}
    assert get_feature_flag(resolved, "use_frame_time_pose") is True


def test_no_overrides_leaves_every_flag_as_the_sequence_has_it():
    declared = {"use_frame_time_pose": False}
    assert resolve_features(declared, {}) == declared


def test_override_of_a_flag_the_sequence_does_not_declare_is_refused():
    with pytest.raises(FeatureFlagError) as error:
        resolve_features({"use_frame_time_pose": False}, {"use_frame_time_poes": True})
    assert "use_frame_time_poes" in str(error.value) and "use_frame_time_pose" in str(error.value)


def test_override_can_turn_a_flag_off_again():
    assert resolve_features({"a_flag": True}, {"a_flag": False}) == {"a_flag": False}


# --- flag dependencies ---------------------------------------------------------------


def test_flag_requires_lists_the_release_gates_needing_the_estimator():
    assert "use_target_estimator" in FLAG_REQUIRES["drop_release_gates"]


def test_flag_requires_is_well_formed():
    import re

    for flag, needs in FLAG_REQUIRES.items():
        assert re.fullmatch(r"[a-z][a-z0-9_]*", flag), flag
        assert isinstance(needs, tuple) and needs, flag
        assert flag not in needs
        for need in needs:
            assert re.fullmatch(r"[a-z][a-z0-9_]*", need), need

    def reaches(start, seen=()):
        for need in FLAG_REQUIRES.get(start, ()):
            assert need not in seen, f"dependency loop through {need}"
            reaches(need, seen + (start,))

    for flag in FLAG_REQUIRES:
        reaches(flag)


def test_flag_on_without_its_requirement_is_reported(monkeypatch):
    monkeypatch.setattr(features, "FLAG_REQUIRES", {"gates": ("estimator",), "estimator": ("time_base",)})
    assert missing_requirements({"gates": False, "estimator": False, "time_base": False}) == []
    assert missing_requirements({"gates": True, "estimator": True, "time_base": True}) == []
    problems = missing_requirements({"gates": True, "estimator": False, "time_base": False})
    assert problems == ["gates requires estimator"]
    both = missing_requirements({"gates": True, "estimator": True, "time_base": False})
    assert both == ["estimator requires time_base"]
    assert missing_requirements({"gates": True}) == ["gates requires estimator"]  # an absent flag is off


def test_resolve_refuses_to_leave_a_dependency_missing(monkeypatch):
    monkeypatch.setattr(features, "FLAG_REQUIRES", {"gates": ("estimator",)})
    declared = {"gates": False, "estimator": False}
    with pytest.raises(FeatureFlagError, match="gates requires estimator"):
        resolve_features(declared, {"gates": True})
    assert resolve_features(declared, {"gates": True, "estimator": True}) == {"gates": True, "estimator": True}
    # The same check applies to a sequence that turns a flag on itself.
    with pytest.raises(FeatureFlagError, match="gates requires estimator"):
        resolve_features({"gates": True, "estimator": False}, {})


def test_a_flag_that_is_not_true_or_false_is_refused():
    """A mission reads `if get_feature_flag(...)`: 1, "true" and even "false" would all read as on."""
    for value in (1, "true", "false", 0, None, [True]):
        with pytest.raises(FeatureFlagError, match="true or false"):
            resolve_features({"use_frame_time_pose": value}, {})
    assert resolve_features({"use_frame_time_pose": True}, {}) == {"use_frame_time_pose": True}


def test_dependency_check_cannot_be_slipped_with_a_truthy_value(monkeypatch):
    monkeypatch.setattr(features, "FLAG_REQUIRES", {"gates": ("estimator",)})
    assert missing_requirements({"gates": 1, "estimator": False}) == ["gates requires estimator"]
    assert missing_requirements({"gates": "true"}) == ["gates requires estimator"]
    assert missing_requirements({"gates": True, "estimator": 1}) == ["gates requires estimator"]  # only True satisfies
    assert missing_requirements({"gates": False, "estimator": 1}) == []


# --- on mission specs: what the executor does at start-up --------------------------------


@pytest.mark.parametrize("path", SEQUENCES, ids=lambda p: p.name)
def test_no_overrides_leaves_every_shipped_sequence_exactly_as_loaded(path):
    """Flags off is comp-2026: with nothing on the launch line the specs are unchanged."""
    loaded = load_sequence_file(str(path))
    assert apply_feature_overrides(loaded, "") == loaded
    assert all(value is False for spec in loaded for value in spec.features.values())


def test_overrides_reach_every_mission_of_the_sequence():
    path = ROOT / "src" / "drone_mission_demo" / "config" / "sequences" / "package_delivery_demo.yaml"
    loaded = load_sequence_file(str(path))
    changed = apply_feature_overrides(loaded, "use_frame_time_pose")
    assert len(changed) == len(loaded) == 3
    for before, after in zip(loaded, changed):
        assert after.features == {"use_frame_time_pose": True}
        assert before.features == {"use_frame_time_pose": False}  # the loaded specs are not modified
        assert dataclasses.replace(after, features=before.features) == before  # nothing else changed


def test_bad_overrides_stop_the_sequence_from_loading():
    loaded = load_sequence_file(str(SEQUENCES[0]))
    for text in ("no_such_flag", "use_frame_time_pose=perhaps"):
        with pytest.raises(FeatureFlagError):
            apply_feature_overrides(loaded, text)
    assert issubclass(FeatureFlagError, ValueError)  # the executor's start-up treats it like a bad sequence file


# --- profiles ---------------------------------------------------------------------------


def test_profiles_live_outside_src_and_the_2026_profile_enables_nothing():
    assert PROFILES.is_dir() and "src" not in PROFILES.relative_to(ROOT).parts
    assert load_profile(PROFILES / "comp_2026.yaml") == {}
    assert (PROFILES / "README.md").is_file()


@pytest.mark.parametrize("path", sorted((ROOT / "profiles").glob("*.yaml")), ids=lambda p: p.name)
def test_every_shipped_profile_loads_and_satisfies_the_dependencies(path):
    enabled = load_profile(path)
    assert all(value is True for value in enabled.values())  # a profile is an enable-list
    assert missing_requirements(enabled) == []


def test_profile_is_an_enable_list(tmp_path, monkeypatch):
    monkeypatch.setattr(features, "FLAG_REQUIRES", {"gates": ("estimator",)})
    good = tmp_path / "good.yaml"
    good.write_text("description: drop with gates\nenable:\n  - estimator\n  - gates\n")
    assert load_profile(good) == {"estimator": True, "gates": True}
    for text, message in (
        ("enable: [gates]\n", "gates requires estimator"),
        ("enable: gates\n", "list"),
        ("enable: [gates, gates, estimator]\n", "twice"),
        ("enable: ['Bad Name']\n", "Bad Name"),
        ("enabled: [estimator]\n", "enabled"),
        ("- estimator\n", "mapping"),
    ):
        bad = tmp_path / "bad.yaml"
        bad.write_text(text)
        with pytest.raises(FeatureFlagError, match=message):
            load_profile(bad)
    with pytest.raises(FeatureFlagError, match="nowhere.yaml"):
        load_profile(tmp_path / "nowhere.yaml")
