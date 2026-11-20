"""tools/sim_sequence_copies.py: sim copies of the live sequence files (CEN-11).

A sim copy is the live file with sim coordinates, the sim camera named and
fake_drop on, and nothing else changed, so the tuning a live file flies with
can be flown in Gazebo. The copies are generated and committed; these tests
fail when one is out of date or differs from its live file anywhere else, and
check that the scenarios which fly them build. No ROS needed.
"""

import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools" / "sim_sequence_copies.py"
SEQUENCES = ROOT / "src" / "drone_mission_demo" / "config" / "sequences"
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "sim"))

import sim_sequence_copies as copies  # noqa: E402
from scenario import build_run, load_scenario  # noqa: E402

PAIRS = sorted(copies.COPIES.items())
SCENARIOS = {
    "sim_package_delivery_live.yaml": ROOT / "sim" / "scenarios" / "delivery" / "live_tuning.yaml",
    "sim_package_drop_live.yaml": ROOT / "sim" / "scenarios" / "drop" / "live_tuning.yaml",
}


def run(*args):
    return subprocess.run([sys.executable, str(TOOL), *map(str, args)], capture_output=True, text=True)


def load(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def missions(sequence: dict) -> list:
    return sequence["mission_sequence"]["missions"]


def test_the_2026_drop_and_delivery_live_files_each_have_a_sim_copy():
    assert copies.COPIES["package_drop_live.yaml"] == "sim_package_drop_live.yaml"
    assert copies.COPIES["package_delivery_live.yaml"] == "sim_package_delivery_live.yaml"
    for live, sim in PAIRS:
        assert (SEQUENCES / live).is_file(), live
        assert (SEQUENCES / sim).is_file(), f"{sim} is not committed: run python3 tools/sim_sequence_copies.py --write"


def test_every_live_and_2027_sequence_that_tracks_a_target_has_a_sim_copy():
    """A new live file (CEN-10's package_drop_2027.yaml and package_delivery_2027.yaml) cannot land without its copy."""
    owed = []
    for path in sorted(SEQUENCES.glob("*.yaml")):
        if path.name.startswith("sim_") or not path.stem.endswith(("_live", "_2027")):
            continue
        if any("target_latitude" in (m.get("config") or {}) for m in missions(load(path))) and path.name not in copies.COPIES:
            owed.append(path.name)
    assert owed == [], "add to COPIES in tools/sim_sequence_copies.py, run it with --write, and give each a live_tuning scenario"
    assert all(sim == f"sim_{live}" for live, sim in PAIRS)


@pytest.mark.parametrize("live, sim", PAIRS)
def test_committed_copy_is_what_the_generator_makes_from_the_live_file(live, sim):
    expected = copies.sim_copy_text((SEQUENCES / live).read_text(encoding="utf-8"), live)
    assert (SEQUENCES / sim).read_text(encoding="utf-8") == expected, "out of date: run python3 tools/sim_sequence_copies.py --write"


@pytest.mark.parametrize("live, sim", PAIRS)
def test_copy_differs_from_the_live_file_only_where_the_sim_needs_it(live, sim):
    live_data, sim_data = load(SEQUENCES / live), load(SEQUENCES / sim)
    live_root, sim_root = live_data["mission_sequence"], sim_data["mission_sequence"]
    assert {k: v for k, v in sim_root.items() if k != "missions"} == {k: v for k, v in live_root.items() if k != "missions"}
    assert [(m["type"], m["name"], m.get("failure_policy")) for m in missions(sim_data)] == [
        (m["type"], m["name"], m.get("failure_policy")) for m in missions(live_data)
    ]
    retargeted = 0
    for live_mission, sim_mission in zip(missions(live_data), missions(sim_data)):
        live_config, sim_config = live_mission.get("config", {}), sim_mission.get("config", {})
        expected = dict(live_config)
        if "target_latitude" in live_config:
            retargeted += 1
            expected.update(target_latitude=copies.SIM_LATITUDE, target_longitude=copies.SIM_LONGITUDE)
            expected.update(camera_mode=copies.SIM_CAMERA_MODE, airframe=copies.SIM_AIRFRAME)
        if "fake_drop" in live_config:
            expected["fake_drop"] = True
        assert sim_config == expected, sim_mission["name"]
    assert retargeted == 1


@pytest.mark.parametrize("live, sim", PAIRS)
def test_copy_never_releases_a_payload_and_keeps_every_flag_off(live, sim):
    data = load(SEQUENCES / sim)
    drops = [m["config"]["fake_drop"] for m in missions(data) if "fake_drop" in m.get("config", {})]
    assert drops == [True]
    features = data["mission_sequence"]["features"]
    assert features and all(value is False for value in features.values())


@pytest.mark.parametrize("live, sim", PAIRS)
def test_copy_says_it_is_generated_and_keeps_the_live_files_comments(live, sim):
    text = (SEQUENCES / sim).read_text(encoding="utf-8")
    head = text.split("mission_sequence:")[0]
    assert "tools/sim_sequence_copies.py" in head and live in head
    assert "# Maximum commanded lateral tracking speed." in text
    assert "Live-flight template" not in text and "real flight" not in text


def test_a_tuned_value_changed_in_the_live_file_reaches_its_copy():
    live = (SEQUENCES / "package_drop_live.yaml").read_text(encoding="utf-8")
    changed = live.replace("centering_gain_mps_per_m: 0.15", "centering_gain_mps_per_m: 0.31")
    assert changed != live
    config = yaml.safe_load(copies.sim_copy_text(changed, "package_drop_live.yaml"))["mission_sequence"]["missions"][1]["config"]
    assert config["centering_gain_mps_per_m"] == 0.31


def test_live_file_without_a_release_or_a_target_is_refused():
    live = (SEQUENCES / "package_drop_live.yaml").read_text(encoding="utf-8")
    no_drop = "\n".join(line for line in live.splitlines() if "fake_drop" not in line) + "\n"
    with pytest.raises(copies.CopyError, match="fake_drop"):
        copies.sim_copy_text(no_drop, "package_drop_live.yaml")
    no_target = "\n".join(line for line in live.splitlines() if "target_latitude" not in line) + "\n"
    with pytest.raises(copies.CopyError, match="target_latitude"):
        copies.sim_copy_text(no_target, "package_drop_live.yaml")


def test_every_mission_with_a_target_must_have_its_own_fake_drop():
    """One mission's fake_drop does not cover another's: a second release mission without the setting would release for real."""
    live = (SEQUENCES / "package_drop_live.yaml").read_text(encoding="utf-8")
    second = """
        - type: package_delivery
          name: second_release
          config:
              target_latitude: 35.0498030
              target_longitude: -118.1509860
"""
    with pytest.raises(copies.CopyError, match="second_release.*fake_drop"):
        copies.sim_copy_text(live.replace("\n        - type: rtl", second + "\n        - type: rtl"), "package_drop_live.yaml")


def test_live_file_that_already_names_a_camera_is_not_given_a_second_one():
    live = (SEQUENCES / "package_drop_live.yaml").read_text(encoding="utf-8")
    named = live.replace("              camera_yaw_offset_deg: 0.0", "              camera_mode: imx519_720p60\n              airframe: argus_2027\n              camera_yaw_offset_deg: 0.0")
    assert named != live
    text = copies.sim_copy_text(named, "package_drop_live.yaml")
    config = yaml.safe_load(text)["mission_sequence"]["missions"][1]["config"]
    assert (config["camera_mode"], config["airframe"]) == (copies.SIM_CAMERA_MODE, copies.SIM_AIRFRAME)
    assert text.count("camera_mode:") == 1 and text.count("airframe:") == 1


def test_check_passes_on_the_committed_copies_and_fails_on_a_stale_one(tmp_path):
    assert run("--check").returncode == 0
    for live, sim in PAIRS:
        (tmp_path / live).write_text((SEQUENCES / live).read_text(encoding="utf-8"))
    assert run("--write", "--dir", tmp_path).returncode == 0
    assert run("--check", "--dir", tmp_path).returncode == 0
    for live, sim in PAIRS:
        assert (tmp_path / sim).read_text() == (SEQUENCES / sim).read_text(encoding="utf-8")
        assert (tmp_path / live).read_text() == (SEQUENCES / live).read_text(encoding="utf-8")  # the live file is only read
    stale = tmp_path / PAIRS[0][1]
    stale.write_text(stale.read_text().replace("fake_drop: true", "fake_drop: false"))
    done = run("--check", "--dir", tmp_path)
    assert done.returncode == 1
    assert PAIRS[0][1] in done.stdout + done.stderr


@pytest.mark.parametrize("sim", sorted(SCENARIOS))
@pytest.mark.parametrize("variant", ["off", "on"])
def test_scenario_flies_the_live_tuning_at_the_sim_target_without_a_release(sim, variant, tmp_path):
    live = next(name for name, copy in PAIRS if copy == sim)
    assert load_scenario(SCENARIOS[sim]).sequence == sim
    run_dir = build_run(SCENARIOS[sim], tmp_path / "run", variant=variant)
    flown = load(run_dir / "sequence.yaml")
    shipped = load(SEQUENCES / live)
    tracking = [m for m in missions(flown) if "target_latitude" in m.get("config", {})]
    assert len(tracking) == 1
    live_config = next(m for m in missions(shipped) if m["name"] == tracking[0]["name"])["config"]
    different = {key for key in set(live_config) | set(tracking[0]["config"]) if live_config.get(key) != tracking[0]["config"].get(key)}
    assert different == {"target_latitude", "target_longitude", "fake_drop", "camera_mode", "airframe"}
    assert tracking[0]["config"]["fake_drop"] is True
    assert all(value is False for value in flown["mission_sequence"]["features"].values())
    manifest = json.loads((run_dir / "scenario.json").read_text())
    # The fallback launch has no detector; the live tuning needs the one the 2026 demo launch starts.
    assert manifest["launch"]["file"] in ("package_delivery_demo.launch.py", "package_drop_demo.launch.py")
