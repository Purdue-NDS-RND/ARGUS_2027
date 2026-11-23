"""PLT-02: boot services and the shared clock (platform/jetson/services.sh, boot/argus-boot.sh).

services.sh is run against the fake Jetson of test_setup.py. argus-boot.sh,
which the units run, is asked to print the command it would start.
"""

import os
import subprocess
import threading
import time
from pathlib import Path

import pytest
from test_setup import Jetson, status

ROOT = Path(__file__).resolve().parents[3]
SERVICES = ROOT / "platform" / "jetson" / "services.sh"
BOOT = ROOT / "platform" / "jetson" / "boot" / "argus-boot.sh"
UNITS = ["argus-mavros.service", "argus-camera.service"]
STEPS = ["boot_env", "mavros_service", "camera_service", "chrony"]


class Services(Jetson):
    def run(self, *args, env=None):
        environment = dict(os.environ, ARGUS_SYSROOT=str(self.sysroot), ARGUS_SETUP_BIN=str(self.bin), FAKE=str(self.fake), CALLS=str(self.calls), ARGUS_USER="argus")
        environment.update(env or {})
        return subprocess.run(["bash", str(SERVICES), *args], capture_output=True, text=True, env=environment, timeout=60)


@pytest.fixture
def jetson(tmp_path):
    machine = Services(tmp_path)
    (machine.fake / "installed").write_text("chrony\n")  # setup.sh's ros step installs it
    return machine


def boot(*args, **env):
    environment = {k: v for k, v in os.environ.items() if not k.startswith("ARGUS_")}
    environment.update(env)
    return subprocess.run(["bash", str(BOOT), *args], capture_output=True, text=True, env=environment, timeout=30)


# ------------------------------------------------------------ services.sh


def test_the_scripts_parse():
    for script in (SERVICES, BOOT):
        assert subprocess.run(["bash", "-n", str(script)], capture_output=True, text=True).returncode == 0


def test_check_on_a_jetson_without_the_services_changes_nothing(jetson):
    before = jetson.tree()
    result = jetson.run("--check")
    assert result.returncode == 1
    found = status(result.stdout)
    assert list(found) == STEPS and set(found.values()) == {"MISSING"}
    assert jetson.changing_calls() == [] and jetson.tree() == before


def test_apply_installs_the_units_the_boot_settings_and_the_clock(jetson):
    result = jetson.run("--apply", "--ground-ip", "192.168.10.2")
    assert result.returncode == 0, result.stdout + result.stderr
    assert set(status(result.stdout).values()) == {"ok"}
    log = jetson.log()

    for unit in UNITS:
        text = jetson.read(f"etc/systemd/system/{unit}")
        assert "User=argus" in text
        assert "EnvironmentFile=/etc/argus/boot.env" in text
        assert f"ExecStart={ROOT}/platform/jetson/boot/argus-boot.sh {unit.split('-')[1].split('.')[0]}" in text
        assert "Restart=on-failure" in text and "RestartPreventExitStatus=2" in text
        assert "WantedBy=multi-user.target" in text
        assert any(line.startswith("systemctl enable") and unit in line for line in log)

    settings = jetson.read("etc/argus/boot.env")
    # What ships starts MAVROS the 2026 way and leaves the camera to the 2026 grabber.
    assert "ARGUS_BOOT_MAVROS=apm\n" in settings
    assert "ARGUS_BOOT_CAMERA=off\n" in settings
    assert f"ARGUS_WS={ROOT}\n" in settings

    chrony = jetson.read("etc/chrony/conf.d/argus.conf")
    assert "server 192.168.10.2 iburst" in chrony and "prefer" in chrony
    assert "systemctl enable chrony" in log and "systemctl restart chrony" in log


def test_the_clock_is_never_stepped_in_flight(jetson):
    jetson.run("--apply", "--ground-ip", "192.168.10.2")
    makestep = [line.split() for line in jetson.read("etc/chrony/conf.d/argus.conf").splitlines() if line.startswith("makestep")]
    assert len(makestep) == 1
    # makestep <threshold> <limit>: only the first <limit> updates after chrony starts may step.
    assert int(makestep[0][2]) > 0


def test_a_second_apply_changes_nothing_and_keeps_edited_boot_settings(jetson):
    jetson.run("--apply", "--ground-ip", "192.168.10.2")
    settings = jetson.sysroot / "etc" / "argus" / "boot.env"
    settings.write_text(settings.read_text().replace("ARGUS_BOOT_CAMERA=off", "ARGUS_BOOT_CAMERA=imx519_720p60"))
    before = jetson.tree()
    jetson.calls.write_text("")
    result = jetson.run("--apply")  # the ground laptop's address is kept from the file
    assert result.returncode == 0, result.stdout
    assert jetson.changing_calls() == [] and jetson.tree() == before


def test_boot_settings_with_a_line_missing_are_reported_not_rewritten(jetson):
    settings = jetson.sysroot / "etc" / "argus" / "boot.env"
    settings.parent.mkdir(parents=True)
    settings.write_text("ARGUS_BOOT_MAVROS=argus\n")
    result = jetson.run("--apply", "--only", "boot_env")
    assert result.returncode == 1
    assert "ARGUS_BOOT_CAMERA" in result.stdout
    assert settings.read_text() == "ARGUS_BOOT_MAVROS=argus\n"


def test_the_clock_step_needs_the_ground_laptops_address(jetson):
    result = jetson.run("--apply", "--only", "chrony")
    assert result.returncode == 1
    assert "--ground-ip" in result.stdout
    assert not (jetson.sysroot / "etc/chrony/conf.d/argus.conf").exists()


@pytest.mark.parametrize("address", ["laptop.local; reboot", "192.168.10", "300.1.1.1", ""])
def test_an_address_that_is_not_an_ipv4_address_is_refused(jetson, address):
    result = jetson.run("--apply", "--ground-ip", address)
    assert result.returncode == 2
    assert jetson.changing_calls() == []


def test_a_new_ground_address_replaces_the_old_one(jetson):
    jetson.run("--apply", "--ground-ip", "192.168.10.2")
    assert jetson.run("--check", "--ground-ip", "192.168.10.7").returncode == 1
    assert jetson.run("--apply", "--ground-ip", "192.168.10.7").returncode == 0
    chrony = jetson.read("etc/chrony/conf.d/argus.conf")
    assert "192.168.10.7" in chrony and "192.168.10.2" not in chrony


def test_the_ground_laptop_serves_its_clock_with_or_without_internet(jetson):
    result = jetson.run("--apply", "--role", "ground", "--allow", "192.168.10.0/24")
    assert result.returncode == 0, result.stdout + result.stderr
    assert list(status(result.stdout)) == ["chrony"]
    conf = jetson.read("etc/chrony/conf.d/argus-ground.conf")
    assert "allow 192.168.10.0/24" in conf
    assert any(line.startswith("local stratum") for line in conf.splitlines())
    # Nothing of the Jetson's is installed on the laptop.
    assert not (jetson.sysroot / "etc/argus").exists()
    assert not list((jetson.sysroot / "etc/systemd/system").iterdir())


def test_a_machine_without_chrony_is_told_to_install_it(jetson):
    (jetson.fake / "installed").write_text("")
    result = jetson.run("--apply", "--role", "ground", "--allow", "192.168.10.0/24")
    assert result.returncode == 1
    assert "chrony is not installed" in result.stdout and "apt-get install chrony" in result.stdout
    assert jetson.changing_calls() == []
    assert not (jetson.sysroot / "etc/chrony/conf.d/argus-ground.conf").exists()


def test_the_ground_role_needs_the_network_to_serve(jetson):
    assert jetson.run("--apply", "--role", "ground").returncode == 1
    assert jetson.run("--apply", "--role", "ground", "--allow", "everyone").returncode == 2


def test_only_missions_are_left_to_the_go_launcher():
    # Nothing a unit runs names a mission, the executor or a mission launch file.
    folder = ROOT / "platform" / "jetson"
    for path in [*folder.glob("systemd/*"), folder / "boot" / "argus-boot.sh"]:
        text = path.read_text().lower()
        for word in ("drone_mission", "mission_executor", "sequence", "feature_overrides"):
            assert word not in text, f"{path.name} mentions {word}"
    assert sorted(p.name for p in (folder / "systemd").iterdir()) == sorted(UNITS)


# ----------------------------------------------------------- argus-boot.sh


def test_mavros_starts_as_in_2026_on_the_fixed_device_name():
    result = boot("--print", "mavros", ARGUS_BOOT_MAVROS="apm")
    assert result.returncode == 0
    assert result.stdout.strip() == "ros2 launch mavros apm.launch fcu_url:=/dev/argus/pixhawk:115200"


def test_mavros_can_start_with_the_argus_settings_and_the_stream_keeper():
    result = boot("--print", "mavros", ARGUS_BOOT_MAVROS="argus")
    assert result.stdout.strip() == f"ros2 launch {ROOT}/config/mavros/argus_mavros.launch.py fcu_url:=/dev/argus/pixhawk:115200 stream_keeper:=true"


def test_the_camera_is_off_unless_a_mode_is_named():
    off = boot("--print", "camera", ARGUS_BOOT_CAMERA="off")
    assert off.returncode == 0 and off.stdout.strip() == "off"
    on = boot("--print", "camera", ARGUS_BOOT_CAMERA="imx519_720p60", ARGUS_BOOT_CAMERA_LEGACY_TOPICS="true")
    assert on.stdout.strip() == "ros2 launch argus_camera argus_camera.launch.py mode:=imx519_720p60 legacy_topics:=true"
    assert boot("--print", "camera", ARGUS_BOOT_CAMERA="imx519_720p60").stdout.strip().endswith("legacy_topics:=false")


def test_with_nothing_set_mavros_is_2026s_and_the_camera_is_off():
    assert "apm.launch" in boot("--print", "mavros").stdout
    assert boot("--print", "camera").stdout.strip() == "off"


@pytest.mark.parametrize(
    "service, env",
    [
        ("mavros", {"ARGUS_BOOT_MAVROS": "px4"}),
        ("camera", {"ARGUS_BOOT_CAMERA": "imx519; rm -rf /"}),
        ("camera", {"ARGUS_BOOT_CAMERA": "imx519_720p60", "ARGUS_BOOT_CAMERA_LEGACY_TOPICS": "yes"}),
        ("mission", {}),
    ],
)
def test_a_setting_it_does_not_know_is_exit_2_which_systemd_does_not_restart(service, env):
    result = boot("--print", service, **env)
    assert result.returncode == 2
    assert result.stdout == ""


def test_mavros_waits_for_the_flight_controller_and_then_starts(tmp_path):
    workspace = tmp_path / "ws"
    (workspace / "install").mkdir(parents=True)
    started = tmp_path / "started"
    stand_in = tmp_path / "bin" / "ros2"
    stand_in.parent.mkdir()
    stand_in.write_text(f'#!/bin/bash\necho "$*" > "{started}"\n')
    stand_in.chmod(0o755)
    (workspace / "install" / "setup.bash").write_text(f'export PATH="{stand_in.parent}:$PATH"\n')
    ros_setup = tmp_path / "ros_setup.bash"
    ros_setup.write_text("")
    device = tmp_path / "pixhawk"

    def plug_in():
        time.sleep(0.6)
        device.write_text("")

    threading.Thread(target=plug_in).start()
    began = time.monotonic()
    result = boot("mavros", ARGUS_BOOT_MAVROS="apm", ARGUS_WS=str(workspace), ARGUS_ROS_SETUP=str(ros_setup), ARGUS_PIXHAWK_DEVICE=str(device), ARGUS_BOOT_POLL_S="0.1", PATH=os.environ["PATH"])
    assert result.returncode == 0, result.stderr
    assert time.monotonic() - began >= 0.5
    assert started.read_text().strip() == f"launch mavros apm.launch fcu_url:={device}:115200"


def test_a_service_that_is_off_starts_nothing(tmp_path):
    result = boot("mavros", ARGUS_BOOT_MAVROS="off", ARGUS_WS=str(tmp_path), ARGUS_ROS_SETUP=str(tmp_path / "none"))
    assert result.returncode == 0
