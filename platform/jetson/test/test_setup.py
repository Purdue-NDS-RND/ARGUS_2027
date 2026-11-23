"""PLT-01: platform/jetson/setup.sh against a fake Jetson.

The script is run for real, with every file it reads or writes under a
temporary root (ARGUS_SYSROOT) and every system command it calls replaced by a
recording stand-in (ARGUS_SETUP_BIN). The stand-ins keep just enough state for
the script's own checks to see what its changes did.
"""

import os
import stat
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SETUP = ROOT / "platform" / "jetson" / "setup.sh"
STEPS = ["platform", "power_mode", "jetson_clocks", "fan", "log_disk", "ros", "onnxruntime", "udev"]

# Commands that change the machine. A --check run must call none of them.
CHANGING = ("nvpmodel -m", "systemctl enable", "systemctl daemon-reload", "systemctl stop", "systemctl start", "apt-get", "pip", "mkfs", "mount ", "udevadm", "usermod", "curl", "geographiclib")

STUBS = {
    # nvpmodel: -q prints the mode held in a state file; -m N sets it from the conf's table.
    "nvpmodel": r"""
state="$FAKE/power_mode"
if [[ $1 == -q ]]; then
    echo "NV Power Mode: $(cat "$state")"
    echo "0"
elif [[ $1 == -m ]]; then
    [[ -e "$FAKE/nvpmodel_needs_reboot" ]] && { echo "NVPM WARN: Reboot required for changing to this power mode: $2"; exit 0; }
    name=$(sed -n "s/^< POWER_MODEL ID=$2 NAME=\([A-Z0-9_]*\) >.*/\1/p" "$ARGUS_SYSROOT/etc/nvpmodel.conf")
    [[ -n $name ]] || exit 1
    echo "$name" > "$state"
fi
""",
    "systemctl": r"""
case $1 in
    is-enabled) [[ -e "$FAKE/enabled/$2" ]] && echo enabled || { echo disabled; exit 1; } ;;
    enable) for unit in "$@"; do [[ $unit == enable || $unit == --now ]] || { mkdir -p "$FAKE/enabled"; touch "$FAKE/enabled/$unit"; }; done ;;
esac
""",
    "findmnt": r"""
# findmnt -n -o SOURCE /            -> the root device
# findmnt -n -o SOURCE <mountpoint> -> what is mounted there, or nothing and exit 1
target=${@: -1}
if [[ $target == / ]]; then cat "$FAKE/root_device"; exit 0; fi
[[ -e "$FAKE/mounted" ]] && { cat "$FAKE/mounted"; exit 0; }
exit 1
""",
    "blkid": r"""
if [[ $1 == -L ]]; then
    [[ -e "$FAKE/label_$2" ]] && { cat "$FAKE/label_$2"; exit 0; }
    exit 2
fi
# blkid -o value -s TYPE <device>
device=${@: -1}
[[ -e "$FAKE/fstype_$(basename "$device")" ]] && { cat "$FAKE/fstype_$(basename "$device")"; exit 0; }
exit 2
""",
    "mkfs.ext4": r"""
# mkfs.ext4 -L <label> <device>
echo "$3" > "$FAKE/label_$2"
echo ext4 > "$FAKE/fstype_$(basename "$3")"
""",
    "mount": r"""
grep -q "LABEL=argus-logs" "$ARGUS_SYSROOT/etc/fstab" || exit 32
cat "$FAKE/label_argus-logs" > "$FAKE/mounted"
""",
    "dpkg-query": r"""
# dpkg-query -W -f '${Status}' <package>
package=${@: -1}
grep -qx "$package" "$FAKE/installed" 2>/dev/null && { echo "install ok installed"; exit 0; }
exit 1
""",
    "apt-get": r"""
[[ $1 == install ]] || exit 0
shift
for package in "$@"; do [[ $package == -* ]] || echo "$package" >> "$FAKE/installed"; done
if grep -qx ros-humble-ros-base "$FAKE/installed"; then
    mkdir -p "$ARGUS_SYSROOT/opt/ros/humble"; touch "$ARGUS_SYSROOT/opt/ros/humble/setup.bash"
fi
""",
    "curl": r"""
# curl -fsSL <url> -o <file>
while (($#)); do [[ $1 == -o ]] && { echo key > "$2"; }; shift; done
""",
    "geographiclib-get-geoids": r"""
mkdir -p "$ARGUS_SYSROOT/usr/share/GeographicLib/geoids"; touch "$ARGUS_SYSROOT/usr/share/GeographicLib/geoids/$1.pgm"
""",
    "python3": r"""
# Only two uses: asking for onnxruntime's providers, and pip.
if [[ $1 == -m && $2 == pip ]]; then
    [[ -e "$FAKE/pip_fails" ]] && exit 1
    echo "1.23.0 CUDAExecutionProvider,CPUExecutionProvider" > "$FAKE/onnxruntime"; exit 0
fi
[[ -e "$FAKE/onnxruntime" ]] && { cat "$FAKE/onnxruntime"; exit 0; }
echo "ModuleNotFoundError: No module named 'onnxruntime'" >&2; exit 1
""",
    "udevadm": "",
    "usermod": r"""
# usermod -aG dialout <user>
echo dialout >> "$FAKE/groups_$3"
""",
    "id": r"""
# id -nG <user>
echo "$2 $(cat "$FAKE/groups_$2" 2>/dev/null | tr '\n' ' ')"
""",
    "chown": "",
    "git": "echo abc1234",
    "date": "echo 2027-01-15T10:00:00Z",
}


class Jetson:
    def __init__(self, tmp: Path):
        self.sysroot = tmp / "root"
        self.bin = tmp / "bin"
        self.fake = tmp / "fake"
        self.calls = tmp / "calls.log"
        for folder in (self.sysroot / "etc" / "udev" / "rules.d", self.sysroot / "etc" / "systemd" / "system", self.sysroot / "proc" / "device-tree", self.bin, self.fake):
            folder.mkdir(parents=True)
        self.calls.write_text("")
        (self.sysroot / "etc" / "nv_tegra_release").write_text("# R36 (release), REVISION: 4.3, GCID: 38968081, BOARD: generic, EABI: aarch64, DATE: Wed Jan  8 01:49:37 UTC 2025\n")
        (self.sysroot / "proc" / "device-tree" / "model").write_bytes(b"NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super\x00")
        (self.sysroot / "etc" / "nvpmodel.conf").write_text("< POWER_MODEL ID=0 NAME=15W >\nCPU_ONLINE CORE_0 1\n< POWER_MODEL ID=1 NAME=25W >\n< POWER_MODEL ID=2 NAME=MAXN_SUPER >\n< PM_CONFIG DEFAULT=1 >\n")
        (self.sysroot / "etc" / "nvfancontrol.conf").write_text("POLLING_INTERVAL 2\n<FAN 1>\n\tFAN_DEFAULT_CONTROL close_loop\n\tFAN_DEFAULT_PROFILE quiet\n\tFAN_DEFAULT_GOVERNOR cont\n")
        (self.sysroot / "etc" / "fstab").write_text("/dev/root / ext4 defaults 0 1\n")
        (self.fake / "power_mode").write_text("25W\n")
        (self.fake / "root_device").write_text("/dev/mmcblk0p1\n")
        for name, body in STUBS.items():
            path = self.bin / name
            path.write_text(f'#!/bin/bash\necho "{name} $*" >> "$CALLS"\n{body}\n')
            path.chmod(path.stat().st_mode | stat.S_IXUSR)

    def label_disk(self, device="/dev/nvme0n1p1"):
        (self.fake / "label_argus-logs").write_text(device + "\n")
        (self.fake / f"fstype_{Path(device).name}").write_text("ext4\n")

    def run(self, *args, env=None):
        environment = dict(os.environ, ARGUS_SYSROOT=str(self.sysroot), ARGUS_SETUP_BIN=str(self.bin), FAKE=str(self.fake), CALLS=str(self.calls), ARGUS_USER="argus")
        environment.update(env or {})
        return subprocess.run(["bash", str(SETUP), *args], capture_output=True, text=True, env=environment, timeout=60)

    def log(self) -> list[str]:
        return self.calls.read_text().splitlines()

    def changing_calls(self) -> list[str]:
        return [line for line in self.log() if line.startswith(CHANGING)]

    def read(self, relative: str) -> str:
        return (self.sysroot / relative).read_text()

    def tree(self) -> dict[str, bytes]:
        return {str(p.relative_to(self.sysroot)): p.read_bytes() for p in sorted(self.sysroot.rglob("*")) if p.is_file()}


@pytest.fixture
def jetson(tmp_path):
    return Jetson(tmp_path)


def status(output: str) -> dict[str, str]:
    """{'fan': 'ok', ...} from the checklist lines '[ok] fan: ...'."""
    found = {}
    for line in output.splitlines():
        if line.startswith("[") and "]" in line and ":" in line:
            mark, rest = line[1:].split("]", 1)
            found[rest.split(":", 1)[0].strip()] = mark.strip()
    return found


def test_the_script_parses():
    assert subprocess.run(["bash", "-n", str(SETUP)], capture_output=True, text=True).returncode == 0


def test_list_names_the_steps_in_order(jetson):
    result = jetson.run("--list")
    assert result.returncode == 0
    assert result.stdout.split() == STEPS


def test_no_mode_given_changes_nothing_and_says_how_to_use_it(jetson):
    result = jetson.run()
    assert result.returncode == 2
    assert "--check" in result.stderr and "--apply" in result.stderr
    assert jetson.changing_calls() == []


def test_check_on_a_fresh_jetpack_reports_what_is_missing_and_changes_nothing(jetson):
    before = jetson.tree()
    result = jetson.run("--check")
    assert result.returncode == 1
    found = status(result.stdout)
    assert list(found) == STEPS
    assert found["platform"] == "ok"
    assert all(found[step] == "MISSING" for step in STEPS[1:]), found
    assert "CHECKLIST: 1 of 8 ok" in result.stdout
    assert jetson.changing_calls() == []
    assert jetson.tree() == before


def test_apply_brings_a_fresh_jetpack_to_flight_state(jetson):
    jetson.label_disk()
    result = jetson.run("--apply")
    assert result.returncode == 0, result.stdout + result.stderr
    assert all(mark == "ok" for mark in status(result.stdout).values()), result.stdout
    assert "CHECKLIST: 8 of 8 ok" in result.stdout
    log = jetson.log()

    # Power mode by the ID the conf file gives MAXN_SUPER, not a number written in the script.
    assert "nvpmodel -m 2" in log
    assert (jetson.fake / "power_mode").read_text().strip() == "MAXN_SUPER"

    unit = jetson.read("etc/systemd/system/argus-jetson-clocks.service")
    assert "ExecStart=/usr/bin/jetson_clocks" in unit and "After=nvpmodel.service" in unit and "WantedBy=multi-user.target" in unit
    assert any(line.startswith("systemctl enable") and "argus-jetson-clocks.service" in line for line in log)

    fan = jetson.read("etc/nvfancontrol.conf")
    assert "FAN_DEFAULT_PROFILE cool" in fan and "quiet" not in fan
    assert "FAN_DEFAULT_GOVERNOR cont" in fan  # the rest of the file is left alone
    assert log.index("systemctl stop nvfancontrol") < log.index("systemctl start nvfancontrol")

    fstab = jetson.read("etc/fstab")
    assert fstab.count("LABEL=argus-logs /mnt/argus-logs ext4 defaults,noatime,nofail 0 2") == 1
    assert fstab.startswith("/dev/root / ext4 defaults 0 1\n")
    assert (jetson.sysroot / "mnt" / "argus-logs").is_dir()
    assert not any(line.startswith("mkfs") for line in log)

    installed = (jetson.fake / "installed").read_text().split()
    for package in ("ros-humble-ros-base", "ros-humble-mavros", "ros-humble-mavros-extras", "ros-humble-cv-bridge", "ros-humble-vision-msgs", "ros-humble-rosbag2-storage-mcap", "python3-colcon-common-extensions"):
        assert package in installed
    source = jetson.read("etc/apt/sources.list.d/ros2.list")
    assert "packages.ros.org/ros2/ubuntu" in source and "jammy main" in source and "signed-by=" in source
    assert (jetson.sysroot / "usr/share/GeographicLib/geoids/egm96-5.pgm").exists()

    assert any(line.startswith("python3 -m pip install") and "onnxruntime" in line for line in log)

    rule = jetson.read("etc/udev/rules.d/99-argus-pixhawk.rules")
    assert 'SYMLINK+="argus/pixhawk"' in rule and 'SUBSYSTEM=="tty"' in rule
    assert 'ENV{ID_USB_INTERFACE_NUM}=="00"' in rule  # ArduPilot's second USB serial port is not MAVLink
    assert 'GROUP="dialout"' in rule
    assert "udevadm control --reload-rules" in log
    assert "usermod -aG dialout argus" in log

    record = jetson.read("var/lib/argus/setup-checklist.txt")
    assert "CHECKLIST: 8 of 8 ok" in record and "2027-01-15T10:00:00Z" in record and "abc1234" in record
    assert all(f"[ok] {step}:" in record for step in STEPS)


def test_a_second_apply_changes_nothing(jetson):
    jetson.label_disk()
    assert jetson.run("--apply").returncode == 0
    before = jetson.tree()
    jetson.calls.write_text("")
    result = jetson.run("--apply")
    assert result.returncode == 0
    assert jetson.changing_calls() == []
    after = jetson.tree()
    after.pop("var/lib/argus/setup-checklist.txt")
    before.pop("var/lib/argus/setup-checklist.txt")
    assert after == before
    assert jetson.run("--check").returncode == 0


@pytest.mark.parametrize(
    "release, model",
    [
        ("# R35 (release), REVISION: 4.1, GCID: 1, BOARD: t186ref", b"NVIDIA Jetson Orin Nano Developer Kit\x00"),
        ("# R36 (release), REVISION: 3.0, GCID: 1, BOARD: generic", b"NVIDIA Jetson Orin Nano Developer Kit\x00"),
        ("# R36 (release), REVISION: 4.0, GCID: 1, BOARD: generic", b"NVIDIA Jetson Orin Nano Developer Kit\x00"),  # JetPack 6.1
        ("# R36 (release), REVISION: 4.3, GCID: 1, BOARD: generic", b"NVIDIA Jetson AGX Xavier\x00"),
    ],
)
def test_apply_refuses_a_machine_that_is_not_a_jetpack_6_2_orin_nano(jetson, release, model):
    (jetson.sysroot / "etc" / "nv_tegra_release").write_text(release + "\n")
    (jetson.sysroot / "proc" / "device-tree" / "model").write_bytes(model)
    before = jetson.tree()
    result = jetson.run("--apply")
    assert result.returncode == 1
    assert status(result.stdout)["platform"] == "MISSING"
    assert jetson.changing_calls() == []
    assert jetson.tree() == before


def test_a_later_jetpack_6_2_point_release_is_accepted(jetson):
    (jetson.sysroot / "etc" / "nv_tegra_release").write_text("# R36 (release), REVISION: 4.4, GCID: 1, BOARD: generic\n")
    assert status(jetson.run("--check", "--only", "platform").stdout)["platform"] == "ok"


def test_power_mode_id_is_read_from_the_conf_file(jetson):
    conf = jetson.sysroot / "etc" / "nvpmodel.conf"
    conf.write_text(conf.read_text().replace("ID=2 NAME=MAXN_SUPER", "ID=7 NAME=MAXN_SUPER"))
    assert jetson.run("--apply", "--only", "power_mode").returncode == 0
    assert "nvpmodel -m 7" in jetson.log()


def test_a_board_without_maxn_super_is_reported_and_the_other_steps_still_run(jetson):
    jetson.label_disk()
    conf = jetson.sysroot / "etc" / "nvpmodel.conf"
    conf.write_text(conf.read_text().replace("< POWER_MODEL ID=2 NAME=MAXN_SUPER >\n", ""))
    result = jetson.run("--apply")
    assert result.returncode == 1
    found = status(result.stdout)
    assert found["power_mode"] == "MISSING"
    assert "MAXN_SUPER" in result.stdout
    assert all(found[step] == "ok" for step in STEPS if step != "power_mode"), found
    assert not any(line.startswith("nvpmodel -m") for line in jetson.log())


def test_a_power_mode_that_needs_a_reboot_says_so(jetson):
    (jetson.fake / "nvpmodel_needs_reboot").write_text("")
    result = jetson.run("--apply", "--only", "power_mode")
    assert result.returncode == 1
    assert status(result.stdout)["power_mode"] == "MISSING"
    assert "reboot" in result.stdout.lower()


def test_only_runs_the_named_steps(jetson):
    result = jetson.run("--apply", "--only", "fan,udev")
    assert result.returncode == 0, result.stdout
    assert list(status(result.stdout)) == ["fan", "udev"]
    assert not any(line.startswith(("apt-get", "nvpmodel -m", "python3 -m pip")) for line in jetson.log())
    assert not (jetson.sysroot / "var/lib/argus/setup-checklist.txt").exists()  # a partial run is not the record


def test_only_rejects_a_step_that_does_not_exist(jetson):
    result = jetson.run("--apply", "--only", "fan,turbo")
    assert result.returncode == 2
    assert "turbo" in result.stderr
    assert jetson.changing_calls() == []


def test_log_disk_without_a_labelled_filesystem_is_missing_and_nothing_is_formatted(jetson):
    result = jetson.run("--apply", "--only", "log_disk")
    assert result.returncode == 1
    assert status(result.stdout)["log_disk"] == "MISSING"
    assert "--log-device" in result.stdout
    assert not any(line.startswith(("mkfs", "mount ")) for line in jetson.log())
    assert "argus-logs" not in jetson.read("etc/fstab")


def test_log_device_formats_a_blank_partition_and_mounts_it(jetson):
    result = jetson.run("--apply", "--only", "log_disk", "--log-device", "/dev/nvme0n1p1")
    assert result.returncode == 0, result.stdout
    log = jetson.log()
    assert "mkfs.ext4 -L argus-logs /dev/nvme0n1p1" in log
    assert any(line.startswith("mount ") for line in log)
    assert "chown argus /mnt/argus-logs" in " ".join(log).replace(str(jetson.sysroot), "")


def test_log_device_never_formats_a_partition_that_holds_a_filesystem(jetson):
    (jetson.fake / "fstype_nvme0n1p1").write_text("ext4\n")
    result = jetson.run("--apply", "--only", "log_disk", "--log-device", "/dev/nvme0n1p1")
    assert result.returncode == 1
    assert "already holds" in result.stdout
    assert not any(line.startswith("mkfs") for line in jetson.log())


def test_log_device_never_formats_the_disk_the_system_boots_from(jetson):
    (jetson.fake / "root_device").write_text("/dev/sda1\n")
    result = jetson.run("--apply", "--only", "log_disk", "--log-device", "/dev/sda1")
    assert result.returncode == 1
    assert not any(line.startswith("mkfs") for line in jetson.log())


def test_a_jetson_that_boots_from_the_nvme_gets_a_plain_folder(jetson):
    (jetson.fake / "root_device").write_text("/dev/nvme0n1p1\n")
    result = jetson.run("--apply", "--only", "log_disk")
    assert result.returncode == 0, result.stdout
    assert (jetson.sysroot / "mnt" / "argus-logs").is_dir()
    assert "argus-logs" not in jetson.read("etc/fstab")
    assert not any(line.startswith(("mkfs", "mount ")) for line in jetson.log())
    assert "boots from the NVMe" in result.stdout


def test_onnxruntime_without_a_gpu_provider_does_not_count(jetson):
    (jetson.fake / "onnxruntime").write_text("1.23.0 CPUExecutionProvider\n")
    (jetson.fake / "pip_fails").write_text("")
    result = jetson.run("--apply", "--only", "onnxruntime")
    assert result.returncode == 1
    assert status(result.stdout)["onnxruntime"] == "MISSING"
    assert "CPUExecutionProvider" in result.stdout


def test_the_pixhawk_rule_can_be_pinned_to_one_serial_number(jetson):
    result = jetson.run("--apply", "--only", "udev", env={"ARGUS_PIXHAWK_SERIAL": "1F0034000F51333130383733"})
    assert result.returncode == 0
    rule = jetson.read("etc/udev/rules.d/99-argus-pixhawk.rules")
    assert 'ATTRS{serial}=="1F0034000F51333130383733"' in rule
    # A different rule on disk is not "ok": the check compares the whole file.
    assert jetson.run("--check", "--only", "udev").returncode == 1


def test_the_checklist_says_whether_the_flight_controller_is_plugged_in(jetson):
    jetson.run("--apply", "--only", "udev")
    assert "not plugged in" in jetson.run("--check", "--only", "udev").stdout
    (jetson.sysroot / "dev" / "argus").mkdir(parents=True)
    (jetson.sysroot / "dev" / "argus" / "pixhawk").write_text("")
    assert "/dev/argus/pixhawk is present" in jetson.run("--check", "--only", "udev").stdout


def test_a_sandboxed_run_never_reaches_the_real_system_commands(jetson):
    environment = {k: v for k, v in os.environ.items() if k != "ARGUS_SETUP_BIN"}
    environment["ARGUS_SYSROOT"] = str(jetson.sysroot)
    result = subprocess.run(["bash", str(SETUP), "--apply"], capture_output=True, text=True, env=environment, timeout=60)
    assert result.returncode == 2
    assert "ARGUS_SETUP_BIN" in result.stderr
    assert jetson.log() == []


def test_apply_on_the_real_system_needs_root():
    if os.geteuid() == 0:
        pytest.skip("running as root: this test would change the machine")
    environment = {k: v for k, v in os.environ.items() if k not in ("ARGUS_SYSROOT", "ARGUS_SETUP_BIN")}
    result = subprocess.run(["bash", str(SETUP), "--apply", "--only", "udev"], capture_output=True, text=True, env=environment, timeout=60)
    assert result.returncode == 2
    assert "root" in result.stderr
