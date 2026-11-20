"""The 2027 drop and delivery sequences (CEN-10).

`package_drop_2027.yaml` and `package_delivery_2027.yaml` are the 2026 live
files with the 2027 settings added: every 2026 value is the live file's, so
with every flag off they fly as the live files do; every 2027 flag is
declared, and false; and the settings the flags read are there, so that a
profile that turns flags on has nothing else to add.

rclpy and the message packages are the stand-ins from
test_package_drop_mission.py.
"""

import dataclasses
import sys
from pathlib import Path

import pytest
import yaml

SEQUENCES = Path(__file__).resolve().parents[1] / "config" / "sequences"
PAIRS = {"package_drop_2027.yaml": "package_drop_live.yaml", "package_delivery_2027.yaml": "package_delivery_live.yaml"}
LOADED = {name: yaml.safe_load((SEQUENCES / name).read_text()) for name in list(PAIRS) + list(PAIRS.values()) + [f"sim_{name}" for name in PAIRS] if (SEQUENCES / name).is_file()}

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_centring_missions as centring_tests  # noqa: E402  (installs the ROS stand-ins)

from drone_mission_core.centring_controller import CentringGains  # noqa: E402
from drone_mission_core.features import FLAG_REQUIRES  # noqa: E402
from drone_mission_core.mission_api import MissionSpec  # noqa: E402
from drone_mission_demo.missions.package_delivery_mission import PackageDeliveryMission  # noqa: E402
from drone_mission_demo.missions.package_drop_mission import PackageDropMission  # noqa: E402

# The settings only 2027 flags read. Everything else in a 2027 file is its live file's.
SETTINGS_2027 = {
    "camera_mode": "imx519_720p60",
    "airframe": "argus_2027",
    "frame_time_max_age_s": 0.5,
    "time_guard_hold_timeout_s": 10.0,
    "estimator_centering_tolerance_m": 0.20,
    "ground_below_home_m": 0.0,
    **{f"centring_{field.name}": field.default for field in dataclasses.fields(CentringGains)},
}
MISSION = {"package_drop_2027.yaml": ("package_drop", PackageDropMission), "package_delivery_2027.yaml": ("package_delivery", PackageDeliveryMission)}


def root(name):
    return LOADED[name]["mission_sequence"]


def vision(name):
    kind = "package_drop" if "drop" in name else "package_delivery"
    return next(entry for entry in root(name)["missions"] if entry["type"] == kind)


@pytest.mark.parametrize("name", sorted(PAIRS))
def test_with_every_flag_off_a_2027_file_is_its_2026_live_file(name):
    new, live = root(name), root(PAIRS[name])
    assert {key: value for key, value in new.items() if key != "missions"} == {key: value for key, value in live.items() if key != "missions"}
    assert all(value is False for value in new["features"].values()) and len(new["features"]) >= 10
    assert [(m["type"], m["name"], m.get("failure_policy")) for m in new["missions"]] == [(m["type"], m["name"], m.get("failure_policy")) for m in live["missions"]]
    for mine, theirs in zip(new["missions"], live["missions"]):
        added = set(mine["config"]) - set(theirs["config"])
        assert {key: mine["config"][key] for key in theirs["config"]} == theirs["config"], mine["name"]  # every 2026 value, unchanged
        if mine["type"] == MISSION[name][0]:
            expected = set(SETTINGS_2027) | ({"package_height_m"} if mine["type"] == "package_delivery" else set())
            assert added == expected
        else:
            assert added == set()


@pytest.mark.parametrize("name", sorted(PAIRS))
def test_the_2027_settings_are_the_specs_values_and_every_flag_the_missions_know_is_declared(name):
    config = vision(name)["config"]
    assert {key: config[key] for key in SETTINGS_2027} == SETTINGS_2027
    declared = set(root(name)["features"])
    other = "delivery_soft_set_down" if "drop" in name else "drop_release_gates"
    assert set(FLAG_REQUIRES) - {other, "survey_search"} <= declared  # (the survey's flag belongs to its own sequence)
    if "delivery" in name:
        # What the rangefinder reads with the package on the ground: nobody has measured it, and nothing is guessed.
        assert config["package_height_m"] is None


@pytest.mark.parametrize("name", sorted(PAIRS))
def test_with_every_flag_on_the_live_2027_file_does_not_build_until_the_aircraft_is_measured(name):
    kind, mission_class = MISSION[name]
    features = {flag: True for flag in root(name)["features"]}
    spec = MissionSpec(type_name=kind, name=kind, config=dict(vision(name)["config"]), base_dir=Path("."), features=features)
    # The camera mode's calibration is a stand-in and the airframe's row in robot.yaml is not verified: whichever is
    # met first, the executor stops on the ground.
    with pytest.raises(ValueError, match="not calibrated yet|argus_2027"):
        mission_class(spec)


@pytest.mark.parametrize("name", sorted(PAIRS))
def test_with_every_flag_on_the_sim_copy_builds_and_uses_the_2027_settings(name):
    kind, mission_class = MISSION[name]
    sim = f"sim_{name}"
    config = dict(vision(sim)["config"])
    assert config["airframe"] == "sim_iris_downcam" and config["fake_drop"] is True
    features = {flag: True for flag in root(sim)["features"]}
    mission = mission_class(MissionSpec(type_name=kind, name=kind, config=config, base_dir=Path("."), features=features))
    mission.on_enter(centring_tests.Context())
    assert mission._centering_tolerance_m == 0.20  # the estimator's tolerance, not the live file's for the raw detection
    assert mission._centring_controller is not None and mission._smart_reacquire is not None
    if kind == "package_delivery":
        assert config["package_height_m"] == 0.05 and mission._set_down is not None  # the sim's own, put in by the copy
    else:
        assert mission._final_align is not None
    # With the flags off the same file uses the live tolerance.
    plain = mission_class(MissionSpec(type_name=kind, name=kind, config=config, base_dir=Path("."), features={}))
    plain.on_enter(centring_tests.Context())
    assert plain._centering_tolerance_m == vision(PAIRS[name])["config"]["centering_tolerance_m"]


def test_the_estimators_tolerance_is_read_only_with_centring_on_the_estimator():
    base = {**centring_tests.BASE, "centering_tolerance_m": 0.8, "estimator_centering_tolerance_m": 0.2, "drop_altitude_m": 4.0}
    for features, expected in ((centring_tests.FLAGS_ON, 0.2), ({"use_frame_time_pose": True, "use_target_estimator": True}, 0.8), ({}, 0.8)):
        mission = PackageDropMission(MissionSpec(type_name="package_drop", name="package_drop", config=base, base_dir=Path("."), features=dict(features)))
        mission.on_enter(centring_tests.Context())
        assert mission._centering_tolerance_m == expected
    # Left out, the mission's own tolerance stands with the flag on too: the 2026 files are not changed by this.
    mission = PackageDropMission(
        MissionSpec(type_name="package_drop", name="package_drop", config={k: v for k, v in base.items() if k != "estimator_centering_tolerance_m"}, base_dir=Path("."), features=dict(centring_tests.FLAGS_ON))
    )
    mission.on_enter(centring_tests.Context())
    assert mission._centering_tolerance_m == 0.8
    # A value that cannot be a tolerance stops the sequence being built (the executor stops on the ground), and an
    # infinite one is refused too: "not centred" is an infinite error, which an infinite tolerance would let through.
    for bad in (0.0, -0.1, "wide", None, True, float("inf"), float("nan"), 1.5):
        with pytest.raises(ValueError, match="estimator_centering_tolerance_m"):
            PackageDropMission(MissionSpec(type_name="package_drop", name="package_drop", config={**base, "estimator_centering_tolerance_m": bad}, base_dir=Path("."), features=dict(centring_tests.FLAGS_ON)))
    # With the flag off the key is not read, whatever it holds.
    PackageDropMission(MissionSpec(type_name="package_drop", name="package_drop", config={**base, "estimator_centering_tolerance_m": "wide"}, base_dir=Path("."), features={}))
