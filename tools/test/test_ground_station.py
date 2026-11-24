"""PLT-14: the wrapper that runs a ground tool in the Humble container (tools/ground_station.sh), and the ground station's document. No ROS, no docker."""

import os
import re
import shlex
import subprocess
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1]
REPO = TOOLS.parent
SCRIPT = TOOLS / "ground_station.sh"
DOC = REPO / "docs" / "ops" / "ground_station.md"


def run(*arguments, env=None):
    return subprocess.run([str(SCRIPT), *arguments], capture_output=True, text=True, env={**os.environ, **(env or {})}, stdin=subprocess.DEVNULL)


def test_a_tool_of_the_repo_is_run_with_python_in_the_repos_own_humble_image_as_the_caller():
    done = run("--print", "judges_list_view.py", "a list.csv", "--site", "x; rm -rf y")
    words = shlex.split(done.stdout)
    assert done.returncode == 0 and words[:3] == ["docker", "run", "--rm"] and "argus-sim:humble" in words
    assert words[words.index("--user") + 1] == f"{os.getuid()}:{os.getgid()}" and words[words.index("-w") + 1] == str(REPO) and f"{REPO}:{REPO}" in words
    # The command's words arrive as arguments of the inner shell, each whole: nothing of them is read as shell.
    assert words[-5:] == ["python3", "tools/judges_list_view.py", "a list.csv", "--site", "x; rm -rf y"]
    assert words[words.index("argus-sim:humble") + 1 : words.index("argus-sim:humble") + 5] == ["bash", "-c", 'source /opt/ros/humble/setup.bash && exec "$@"', "ground_station"]
    assert "--privileged" not in words and not [word for word in words if ".ssh" in word]  # the user's keys are not handed to the container
    # Anything that is not a tool here is run as it is, with ROS sourced.
    assert shlex.split(run("--print", "ros2", "bag", "info", "attempts/x/bag").stdout)[-4:] == ["ros2", "bag", "info", "attempts/x/bag"]
    # A path is not taken for a tool: only a bare name of a file in tools/.
    assert shlex.split(run("--print", "../tools/go.py").stdout)[-1] == "../tools/go.py"


def test_the_download_is_run_on_the_laptop_itself_for_the_image_has_no_ssh_and_no_rsync(tmp_path):
    """The spec audit, then a run in the image: `rsync: not found`, `ssh: not found`. The download needs the laptop's own ssh, keys and known hosts."""
    done = run("--print", "attempt_download.py", "pilot@aircraft", "--list")
    assert done.returncode == 0 and shlex.split(done.stdout) == ["python3", str(TOOLS / "attempt_download.py"), "pilot@aircraft", "--list"]
    # Run for real, with no docker at all: the tool itself answers (here, that the host's name is not one).
    ran = run("attempt_download.py", "not a host", "--list", env={"ARGUS_DOCKER": str(tmp_path / "no_docker")})
    assert ran.returncode == 2 and "docker" not in ran.stderr and ran.stderr.strip()
    text = DOC.read_text()
    assert "runs on the laptop itself" in text and "ssh pilot@aircraft" in text  # the document says so, and says to meet the aircraft's host key once by hand


def test_with_a_stand_in_docker_the_command_is_run_and_its_exit_code_is_the_tools(tmp_path):
    fake = tmp_path / "docker"
    fake.write_text('#!/usr/bin/env bash\nif [[ $1 == image ]]; then [[ -z ${NO_IMAGE:-} ]]; exit; fi\nprintf "%s\\n" "$@" > "$CALLS"\nexit 7\n')
    fake.chmod(0o755)
    env = {"ARGUS_DOCKER": str(fake), "CALLS": str(tmp_path / "calls")}
    done = run("judges_list_view.py", "list.csv", env=env)
    assert done.returncode == 7 and (tmp_path / "calls").read_text().splitlines()[-3:] == ["python3", "tools/judges_list_view.py", "list.csv"]
    missing = run("judges_list_view.py", "list.csv", env={**env, "NO_IMAGE": "1"})
    assert missing.returncode == 2 and "is not on this laptop: sim/run_sim.sh --build makes it" in missing.stderr
    none = run("judges_list_view.py", "list.csv", env={"ARGUS_DOCKER": str(tmp_path / "no_docker")})
    assert none.returncode == 2 and "docker is not installed. The Python tools also run without it: python3 tools/judges_list_view.py" in none.stderr
    assert run().returncode == 2 and "tools/ground_station.sh judges_list_view.py" in run().stderr


def test_the_document_names_tools_options_and_parameters_that_exist():
    text = DOC.read_text()
    for tool in set(re.findall(r"(?<![A-Za-z0-9_/>-])((?:tools|platform|sim|docs)/[A-Za-z0-9_./-]+\.(?:py|sh|md|param))", text)):
        assert (REPO / tool).is_file(), tool
    for name in set(re.findall(r"ground_station\.sh ([a-z_]+\.py)", text)):
        assert (TOOLS / name).is_file(), name
    sys.path.insert(0, str(REPO / "src" / "drone_utils"))
    from drone_utils import preflight

    baseline = preflight.read_param_file(REPO / "platform" / "fcu" / "argus_2027.param")
    # What the document says the aircraft does when the ground station goes quiet is the baseline's.
    assert int(baseline["FS_GCS_ENABLE"]) == 1 and f"{baseline['FS_GCS_TIMEOUT']:g} s" in text and "`FS_GCS_ENABLE`" in text
    for option in ("--list-first", "--list", "--site", "--map"):
        assert option in text
    listing = (REPO / "src/argus_localisation/argus_localisation/judges_list.py").read_text()
    assert 'text = f"LOC list: {len(rows)} marker' in listing and "`LOC list: <n> markers`" in text
