"""OPS-06: platform/jetson/fallback.sh against a fake machine and a small real git repository.

The script is run for real. git is the real git (through a stand-in that only passes it on), on a temporary
repository with a comp-2026 tag; colcon and ros2 are recording stand-ins; every path is under a temporary root.
"""

import os
import shutil
import stat
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "platform" / "jetson" / "fallback.sh"
GIT = shutil.which("git")

STUBS = {
    "git": f'exec {GIT} "$@"\n',
    # colcon: "builds" by writing the install's setup file, unless told to fail.
    "colcon": r"""
echo "colcon $*" >> "$FAKE/calls"
[[ -e "$FAKE/colcon_fails" ]] && { echo "CMake Error: something" ; exit 1; }
install=""
while (($#)); do [[ $1 == --install-base ]] && install=$2; shift; done
mkdir -p "$install"
echo "export FALLBACK_INSTALL_SOURCED=$install" > "$install/setup.bash"
""",
    "ros2": r"""
echo "ros2 $* | ros=$FAKE_ROS_SOURCED | install=$FALLBACK_INSTALL_SOURCED" >> "$FAKE/calls"
exit "${FAKE_LAUNCH_EXIT:-0}"
""",
}


def git(repo, *args):
    env = {**os.environ, "GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@t", "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@t"}
    return subprocess.run([GIT, "-C", str(repo), *args], capture_output=True, text=True, env=env, check=True).stdout.strip()


@pytest.fixture()
def machine(tmp_path):
    """A flight computer with ROS "installed", and a 2027 checkout whose history has the comp-2026 tag two commits back."""
    sysroot, bin_dir, fake = tmp_path / "root", tmp_path / "bin", tmp_path / "fake"
    for folder in (sysroot / "opt/ros/humble", sysroot / "flight/pilot", bin_dir, fake):
        folder.mkdir(parents=True)
    (sysroot / "opt/ros/humble/setup.bash").write_text("export FAKE_ROS_SOURCED=1\n")
    for name, body in STUBS.items():
        path = bin_dir / name
        path.write_text("#!/usr/bin/env bash\n" + body)
        path.chmod(path.stat().st_mode | stat.S_IEXEC)
    repo = tmp_path / "ARGUS_2027"
    (repo / "src" / "drone_mission_demo").mkdir(parents=True)
    git(tmp_path, "init", "-q", "-b", "main", str(repo))
    (repo / "src" / "drone_mission_demo" / "mission.py").write_text("YEAR = 2026\n")
    git(repo, "add", "."), git(repo, "commit", "-q", "-m", "the code that flew")
    git(repo, "tag", "comp-2026")
    (repo / "src" / "drone_mission_demo" / "mission.py").write_text("YEAR = 2027\n")
    git(repo, "commit", "-q", "-am", "2027 work")
    env = {**os.environ, "ARGUS_SYSROOT": str(sysroot), "ARGUS_SETUP_BIN": str(bin_dir), "FAKE": str(fake), "HOME": "/flight/pilot"}
    env.pop("ARGUS_ROS_SETUP", None)

    def run(*args, **extra_env):
        return subprocess.run(["bash", str(SCRIPT), "--repo", str(repo), *args], capture_output=True, text=True, env={**env, **extra_env})

    return SimpleMachine(run, repo, sysroot / "flight/pilot/ARGUS_2026_fallback", fake)


class SimpleMachine:
    def __init__(self, run, repo, fallback, fake):
        self.run, self.repo, self.fallback, self.fake = run, repo, fallback, fake

    def calls(self):
        path = self.fake / "calls"
        return path.read_text().splitlines() if path.exists() else []


def test_check_on_a_machine_with_no_fallback_says_what_is_missing_and_changes_nothing(machine):
    done = machine.run("--check")
    assert done.returncode == 1
    assert "[MISSING] clone:" in done.stdout and "is not there" in done.stdout and "CHECKLIST: 0 of 3 ok" in done.stdout
    assert not machine.fallback.exists() and machine.calls() == []


def test_apply_clones_the_tag_builds_it_and_a_second_apply_does_nothing(machine):
    done = machine.run("--apply")
    assert done.returncode == 0, done.stdout + done.stderr
    assert "CHECKLIST: 3 of 3 ok" in done.stdout and "[ok] at_tag: at comp-2026" in done.stdout and "[ok] build: built from comp-2026" in done.stdout
    # The fallback holds the code that flew, not the 2027 work, and its build is of that commit.
    assert (machine.fallback / "src/drone_mission_demo/mission.py").read_text() == "YEAR = 2026\n"
    tag = git(machine.repo, "rev-parse", "comp-2026^{commit}")
    assert git(machine.fallback, "rev-parse", "HEAD") == tag and (machine.fallback / "install/.argus_fallback_built").read_text().strip() == tag
    builds = [line for line in machine.calls() if line.startswith("colcon")]
    assert len(builds) == 1 and f"--base-paths {machine.fallback}/src" in builds[0] and f"--install-base {machine.fallback}/install" in builds[0]
    assert machine.run("--apply").returncode == 0 and len([line for line in machine.calls() if line.startswith("colcon")]) == 1  # already done: not built again
    assert machine.run("--check").returncode == 0
    # The 2027 checkout was only read.
    assert git(machine.repo, "status", "--porcelain") == "" and (machine.repo / "src/drone_mission_demo/mission.py").read_text() == "YEAR = 2027\n"


def test_launch_runs_the_2026_launch_from_the_fallbacks_own_install(machine):
    machine.run("--apply")
    done = machine.run("launch", "package_delivery_live.launch.py", "log_session:=false", FAKE_LAUNCH_EXIT="7")
    assert done.returncode == 7  # the launch's own exit code
    assert "launching package_delivery_live.launch.py" in done.stdout and "comp-2026" in done.stdout
    launched = [line for line in machine.calls() if line.startswith("ros2")]
    assert launched == [f"ros2 launch drone_mission_demo package_delivery_live.launch.py log_session:=false | ros=1 | install={machine.fallback}/install"]


def test_launch_starts_nothing_from_a_fallback_that_is_not_ready(machine):
    assert machine.run("launch", "package_delivery_live.launch.py").returncode == 1
    machine.run("--apply")
    (machine.fallback / "src/drone_mission_demo/mission.py").write_text("YEAR = 2026  # a field edit\n")
    done = machine.run("launch", "package_delivery_live.launch.py")
    assert done.returncode == 1 and "1 file(s) changed: not the code that flew" in done.stderr and "nothing was started" in done.stderr
    assert not [line for line in machine.calls() if line.startswith("ros2")]
    assert machine.run("launch").returncode == 2  # no launch file named


def test_a_fallback_with_local_changes_is_reported_and_left_alone(machine):
    machine.run("--apply")
    edited = machine.fallback / "src/drone_mission_demo/mission.py"
    edited.write_text("YEAR = 2026  # a field edit\n")
    done = machine.run("--apply")
    assert done.returncode == 1 and "left alone" in done.stdout and "[MISSING] at_tag" in done.stdout
    assert edited.read_text() == "YEAR = 2026  # a field edit\n"  # nothing was discarded


def test_a_clone_with_a_file_changed_is_never_built_so_its_build_cannot_be_marked_as_the_tags(machine):
    """The spec audit: a dirty clone was built in the same run that said "left alone", and the mark then said comp-2026."""
    (machine.fake / "colcon_fails").write_text("")
    machine.run("--apply")  # the clone is made; the build fails: no install yet
    (machine.fake / "colcon_fails").unlink()
    builds_before = len([line for line in machine.calls() if line.startswith("colcon")])
    edited = machine.fallback / "src/drone_mission_demo/mission.py"
    edited.write_text("YEAR = 2026  # edited to get it to build\n")
    done = machine.run("--apply")
    assert done.returncode == 1 and "[MISSING] at_tag" in done.stdout and "[MISSING] build: not built: the clone must be at comp-2026 with nothing changed first" in done.stdout
    assert len([line for line in machine.calls() if line.startswith("colcon")]) == builds_before  # colcon was not run on the edited tree
    assert not (machine.fallback / "install/.argus_fallback_built").exists()
    git(machine.fallback, "checkout", "--", "src/drone_mission_demo/mission.py")  # the edit is taken back
    assert machine.run("--check").returncode == 1  # still not built: nothing was built from the edited tree
    assert machine.run("--apply").returncode == 0 and machine.run("launch", "package_delivery_live.launch.py").returncode == 0


def test_launch_starts_nothing_while_the_boot_settings_are_2027s_and_says_how_to_set_them(machine):
    """The spec audit: the 2027 stack runs with its own MAVROS start-up and camera driver; the 2026 launch opens the camera itself."""
    machine.run("--apply")
    boot = machine.fallback.parents[2] / "etc/argus/boot.env"
    boot.parent.mkdir(parents=True)
    boot.write_text("ARGUS_BOOT_MAVROS=argus\nARGUS_BOOT_CAMERA=imx519_720p60\n")
    done = machine.run("launch", "package_delivery_live.launch.py")
    assert done.returncode == 1 and "ARGUS_BOOT_CAMERA=imx519_720p60" in done.stderr and "Nothing was started" in done.stderr
    assert "set ARGUS_BOOT_MAVROS=apm and ARGUS_BOOT_CAMERA=off, then: sudo systemctl restart argus-mavros argus-camera" in done.stderr
    boot.write_text("ARGUS_BOOT_MAVROS=argus\nARGUS_BOOT_CAMERA=off\n")
    done = machine.run("launch", "package_delivery_live.launch.py")
    assert done.returncode == 1 and "ARGUS_BOOT_MAVROS=argus" in done.stderr
    assert not [line for line in machine.calls() if line.startswith("ros2")]
    boot.write_text("# shipped\nARGUS_BOOT_MAVROS=apm\nARGUS_BOOT_CAMERA=off\n")
    assert machine.run("launch", "package_delivery_live.launch.py").returncode == 0  # the 2026 start-up: launched
    assert boot.read_text() == "# shipped\nARGUS_BOOT_MAVROS=apm\nARGUS_BOOT_CAMERA=off\n"  # read, never written


def test_a_fallback_at_another_commit_is_brought_to_the_tag_and_built_again(machine):
    machine.run("--apply")
    git(machine.fallback, "fetch", "-q", str(machine.repo), "main")
    git(machine.fallback, "checkout", "-q", "--detach", "FETCH_HEAD")
    check = machine.run("--check")
    assert check.returncode == 1 and "and comp-2026 is" in check.stdout
    done = machine.run("--apply")
    assert done.returncode == 0, done.stdout
    assert (machine.fallback / "src/drone_mission_demo/mission.py").read_text() == "YEAR = 2026\n"


def test_an_install_built_from_another_commit_is_not_ready(machine):
    machine.run("--apply")
    (machine.fallback / "install/.argus_fallback_built").write_text("0123456789abcdef\n")
    check = machine.run("--check")
    assert check.returncode == 1 and "the install was built from 0123456789abcdef, not from comp-2026" in check.stdout
    assert machine.run("--apply").returncode == 0 and len([line for line in machine.calls() if line.startswith("colcon")]) == 2


def test_a_build_that_fails_is_said_with_where_to_look(machine):
    (machine.fake / "colcon_fails").write_text("")
    done = machine.run("--apply")
    assert done.returncode == 1 and "colcon build failed: see" in done.stdout and "[MISSING] build" in done.stdout
    assert "CMake Error" in (machine.fallback / "colcon.log").read_text()


def test_a_folder_that_is_not_a_clone_is_not_touched_and_wrong_calls_are_refused(machine):
    machine.fallback.mkdir(parents=True)
    (machine.fallback / "notes.txt").write_text("mine\n")
    done = machine.run("--apply")
    assert done.returncode == 1 and "exists and is not a clone: move it away first" in done.stdout and (machine.fallback / "notes.txt").read_text() == "mine\n"
    assert machine.calls() == [] and sorted(path.name for path in machine.fallback.iterdir()) == ["notes.txt"]  # colcon was not run in it, and nothing was written there
    assert machine.run().returncode == 2 and machine.run("--frobnicate").returncode == 2 and machine.run("--check", "--only", "nothing").returncode == 2
    assert machine.run("--check", "--only", "clone").returncode == 1


def test_it_will_not_run_as_root_and_a_sandboxed_run_needs_its_stand_ins():
    text = SCRIPT.read_text()
    assert "$EUID -eq 0" in text and "not as root" in text
    done = subprocess.run(["bash", str(SCRIPT), "--check"], capture_output=True, text=True, env={**os.environ, "ARGUS_SYSROOT": "/tmp/nowhere", "ARGUS_SETUP_BIN": ""})
    assert done.returncode == 2 and "ARGUS_SETUP_BIN must name" in done.stderr
    # Nothing in the script deletes or discards: no rm, no reset, no clean, no forced checkout.
    code = [line for line in text.splitlines() if not line.lstrip().startswith("#")]
    assert not any(word in line for line in code for word in ("rm -", "reset --hard", "git clean", "checkout -f", "--force"))
