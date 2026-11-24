"""PLT-07: the GO command (tools/go.py): the profile checked, the preflight run, the flight launched. No ROS."""

import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1]
REPO = TOOLS.parent
sys.path.insert(0, str(TOOLS))

import go  # noqa: E402


@pytest.fixture(autouse=True)
def no_installed_workspace(monkeypatch):
    """These tests plan against the source tree, whatever workspace the machine running them has sourced."""
    monkeypatch.setattr(go, "installed_share", lambda: None)

BASE = ["--profile", "comp_2026", "--sequence", "package_delivery_demo.yaml", "--params", "sim_target_mission.yaml", "--detector", "target_cv"]


def made(*extra):
    return go.plan(go.build_parser().parse_args([*BASE, *extra]))


def value(command, name):
    return next(item.split(":=", 1)[1] for item in command if item.startswith(name + ":="))


def profile(tmp_path, text):
    path = tmp_path / "profile.yaml"
    path.write_text(text)
    return str(path)


ESTIMATOR = """
description: the estimator with the ring fit
enable: [use_frame_time_pose, use_target_estimator, use_ring_fit]
nodes:
  target_cv: [stamp_from_image, detection_contract]
  ring_fit: [ring_fit]
start:
  detector: target_cv
"""


def test_the_2026_profile_launches_with_every_flag_off():
    plan = made()
    assert plan.launch[:4] == ["ros2", "launch", "drone_mission_demo", "argus_bringup.launch.py"]
    # No flags: the two arguments are not given at all (`ros2 launch` refuses `name:=` with no value), and the launch file's defaults are no flags.
    assert not any(item.startswith(("feature_overrides:=", "node_flags:=")) for item in plan.launch)
    assert all(item.split(":=", 1)[1] for item in plan.launch[4:])
    launch_file = (REPO / "src/drone_mission_demo/launch/argus_bringup.launch.py").read_text()
    for name in ("feature_overrides", "node_flags"):
        assert f'"{name}",\n                default_value="",' in launch_file
    assert value(plan.launch, "sequence") == str(REPO / "src/drone_mission_demo/config/sequences/package_delivery_demo.yaml")
    assert value(plan.launch, "params") == str(REPO / "src/drone_mission_demo/config/params/sim_target_mission.yaml")
    assert "use_sim_time:=true" not in plan.launch and plan.flags == ""
    assert plan.preflight[:4] == ["ros2", "run", "drone_utils", "preflight"]
    assert plan.preflight[plan.preflight.index("--baseline") + 1] == str(REPO / "platform/fcu/argus_2027.param")
    assert plan.preflight[plan.preflight.index("--profile") + 1] == str(REPO / "profiles/comp_2026.yaml")
    assert "--override" not in plan.preflight and "--clock-tool" not in plan.preflight


def test_a_profile_becomes_the_mission_flags_the_node_flags_and_the_nodes(tmp_path):
    plan = go.plan(go.build_parser().parse_args(["--profile", profile(tmp_path, ESTIMATOR), "--sequence", "package_delivery_demo.yaml", "--params", "sim_target_mission.yaml", "--sim"]))
    assert value(plan.launch, "feature_overrides") == "use_frame_time_pose,use_target_estimator,use_ring_fit"
    assert value(plan.launch, "node_flags") == "ring_fit.ring_fit,target_cv.stamp_from_image,target_cv.detection_contract"
    assert value(plan.launch, "detector") == "target_cv" and value(plan.launch, "camera") == "none"
    assert plan.preflight[-1] == "--sim-time"
    assert "use_sim_time:=true" in plan.launch and all(item.split(":=", 1)[1] for item in plan.launch[4:])
    assert plan.nodes == ("simple_takeoff_service", "target_cv", "mission_executor", "ring_fit", "session_logger")


def test_what_the_preflight_needs_is_passed_through():
    plan = made("--param-dump", "/tmp/today.param", "--log-dir", "/mnt/argus-logs", "--camera", "/camera/image=30", "--camera", "/argus/camera/flow/image_raw=60",
                "--clock-source", "192.168.10.2", "--cells", "6", "--min-cell-v", "3.9", "--min-free-gb", "50", "--override", "HDOP under the roof")  # fmt: skip
    command = plan.preflight
    for option, expected in (("--param-dump", "/tmp/today.param"), ("--log-dir", "/mnt/argus-logs"), ("--clock-source", "192.168.10.2"), ("--cells", "6"),
                             ("--min-cell-v", "3.9"), ("--min-free-gb", "50.0"), ("--override", "HDOP under the roof"), ("--clock-tool", str(REPO / "platform/jetson/clock_sync.py"))):  # fmt: skip
        assert command[command.index(option) + 1] == expected, option
    assert [command[i + 1] for i, item in enumerate(command) if item == "--camera"] == ["/camera/image=30", "/argus/camera/flow/image_raw=60"]


@pytest.mark.parametrize(
    "arguments, word",
    [
        (["--profile", "no_such_profile", "--sequence", "package_delivery_demo.yaml", "--params", "sim_target_mission.yaml"], "no profile"),
        (["--profile", "comp_2026", "--sequence", "no_such.yaml", "--params", "sim_target_mission.yaml"], "no sequence file"),
        (["--profile", "comp_2026", "--sequence", "package_delivery_demo.yaml", "--params", "no_such.yaml"], "no params file"),
    ],
)
def test_a_flight_whose_files_are_not_there_cannot_be_planned(arguments, word):
    with pytest.raises(go.GoError, match=word):
        go.plan(go.build_parser().parse_args(arguments))


def test_a_profile_that_does_not_fit_the_sequence_or_the_params_cannot_be_planned(tmp_path):
    survey = profile(tmp_path, "description: x\nenable: [use_frame_time_pose, use_height_estimator, survey_search]\n")
    with pytest.raises(go.GoError, match="survey_search"):  # not a flag of the delivery's sequence
        go.plan(go.build_parser().parse_args(["--profile", survey, "--sequence", "package_delivery_demo.yaml", "--params", "sim_target_mission.yaml"]))
    undeclared = profile(tmp_path, "description: x\nenable: []\nnodes: {target_cv: [no_such_flag]}\nstart: {detector: target_cv}\n")
    with pytest.raises(go.GoError, match="not declared"):
        go.plan(go.build_parser().parse_args(["--profile", undeclared, "--sequence", "package_delivery_demo.yaml", "--params", "sim_target_mission.yaml"]))
    top_level = tmp_path / "params.yaml"
    top_level.write_text("features: {x: false}\nmission_executor: {ros__parameters: {loop_rate_hz: 20.0}}\n")
    with pytest.raises(go.GoError, match="top level"):
        go.plan(go.build_parser().parse_args(["--profile", "comp_2026", "--sequence", "package_delivery_demo.yaml", "--params", str(top_level)]))


class Runner:
    """Stands in for the three ways `go` starts something: the preflight and the launch (run), the grabber (start), the wait (sleep)."""

    def __init__(self, preflight_code=0, launch_code=0):
        self.codes, self.calls, self.commands, self.grabber = {"preflight": preflight_code, "launch": launch_code}, [], {}, None

    def __call__(self, command):
        kind = "preflight" if command[1] == "run" else "launch"
        self.calls.append(kind)
        self.commands[kind] = command
        return self.codes[kind]

    launch = __call__

    def start(self, command):
        kind = "recorder" if command[:3] == ["ros2", "bag", "record"] else "camera"
        self.calls.append(kind)
        self.commands[kind] = command
        runner = self

        class Grabber:
            def poll(self):
                return None

            def terminate(self):
                runner.calls.append(f"{kind} stopped")

            def wait(self, timeout=None):
                return 0

        self.grabber = Grabber()
        return self.grabber

    def sleep(self, seconds):
        self.calls.append(f"wait {seconds:g} s")


def fly(runner, *extra):
    return go.main([*BASE, *extra], run=runner, start=runner.start, sleep=runner.sleep, launch=runner.launch, stop=lambda grabber: grabber.terminate())


def test_a_2026_grabber_is_started_before_the_preflight_that_checks_its_rate():
    """The spec audit: the launch used to start the grabber, after a preflight that had just counted its frames."""
    runner = Runner()
    assert fly(runner, "--start-camera", "mipi", "--detector", "target_cv", "--camera", "/camera/image=30") == 0
    assert runner.calls == ["camera", "wait 5 s", "preflight", "launch", "camera stopped"]
    assert runner.commands["camera"] == ["ros2", "run", "drone_target_cv", "mipi_grabber", "--ros-args", "-r", "__node:=mipi_grabber", "--params-file",
                                         str(REPO / "src/drone_mission_demo/config/params/sim_target_mission.yaml")]  # fmt: skip
    # The launch is told to start none: one grabber, not two. The rest of the 2026 nodes are the launch's.
    assert "camera:=none" in runner.commands["launch"] and "detector:=target_cv" in runner.commands["launch"]


def test_a_grabber_started_for_a_no_go_is_stopped_again_and_no_flight_node_is_started():
    runner = Runner(preflight_code=1)
    assert fly(runner, "--start-camera", "usb", "--detector", "target_cv", "--camera", "/camera/image=30", "--camera-wait-s", "2") == 1
    assert runner.calls == ["camera", "wait 2 s", "preflight", "camera stopped"]


def test_a_flight_with_no_grabber_starts_none():
    runner = Runner()
    assert fly(runner) == 0 and runner.calls == ["preflight", "launch"]
    assert made().camera == [] and made("--start-camera", "mipi", "--detector", "target_cv", "--sim").camera[-2:] == ["-p", "use_sim_time:=true"]


def test_go_runs_the_preflight_and_then_the_launch(capsys):
    runner = Runner()
    assert go.main(BASE, run=runner, launch=runner) == 0
    assert runner.calls == ["preflight", "launch"]
    out = capsys.readouterr().out.strip().splitlines()
    assert out[-1] == "GO" and "mission flags: (none: the 2026 code)" in out[0]


@pytest.mark.parametrize("code", [1, 2])
def test_a_no_go_starts_nothing(capsys, code):
    runner = Runner(preflight_code=code)
    assert go.main(BASE, run=runner, launch=runner) == 1
    assert runner.calls == ["preflight"]  # the launch was never run: 1 is NO-GO, 2 is a preflight that could not look
    assert capsys.readouterr().out.strip().splitlines()[-1] == "NO-GO"


def test_the_launchs_exit_code_is_gos(capsys):
    assert fly(Runner(launch_code=3)) == 3


def test_a_flight_that_cannot_be_planned_is_no_go_and_runs_nothing(capsys):
    runner = Runner()
    assert go.main(["--profile", "no_such_profile", "--sequence", "package_delivery_demo.yaml", "--params", "sim_target_mission.yaml"], run=runner, launch=runner) == 2
    assert runner.calls == []
    out = capsys.readouterr()
    assert out.out.strip().splitlines()[-1].startswith("NO-GO") and "no profile" in out.err


def test_a_dry_run_prints_both_commands_and_starts_nothing(capsys):
    runner = Runner()
    assert go.main([*BASE, "--dry-run"], run=runner, launch=runner) == 0
    assert runner.calls == []
    out = capsys.readouterr().out
    assert "preflight: ros2 run drone_utils preflight" in out and "launch:    ros2 launch drone_mission_demo argus_bringup.launch.py" in out
    assert go.main([*BASE, "--dry-run", "--start-camera", "mipi", "--detector", "target_cv"], run=runner, start=runner.start) == 0 and runner.calls == []
    assert "camera:    ros2 run drone_target_cv mipi_grabber" in capsys.readouterr().out


def test_what_is_started_can_be_said_on_the_command_line():
    plan = made("--start-camera", "mipi", "--detector", "target_cv", "--camera", "/camera/image=30")
    assert value(plan.launch, "camera") == "none" and value(plan.launch, "detector") == "target_cv" and plan.camera[3] == "mipi_grabber"
    assert plan.nodes[0] == "mipi_grabber" and "--no-camera" not in plan.preflight
    assert "--no-camera" not in made().preflight and "--camera" not in made().preflight  # nothing said: the preflight's camera check is NO-GO
    waypoints = go.plan(go.build_parser().parse_args(["--profile", "comp_2026", "--sequence", "waypoint_navigation_live.yaml", "--params", "live_target_mission.yaml", "--no-camera"]))
    assert "--no-camera" in waypoints.preflight  # a flight with no detector may say it needs no camera


def test_the_shipped_example_profile_flies_the_delivery():
    plan = go.plan(go.build_parser().parse_args(["--profile", "delivery_estimator", "--sequence", "package_delivery_demo.yaml", "--params", "sim_target_mission.yaml"]))
    assert value(plan.launch, "feature_overrides") == "use_frame_time_pose,use_target_estimator"
    assert value(plan.launch, "node_flags") == "target_cv.stamp_from_image,target_cv.detection_contract"


def test_every_option_go_passes_on_is_one_the_preflight_takes():
    sys.path.insert(0, str(REPO / "src" / "drone_utils"))
    from drone_utils import preflight_node

    known = {option for action in preflight_node.build_parser()._actions for option in action.option_strings}
    command = made("--param-dump", "a", "--fc-url", "b", "--log-dir", "c", "--camera", "/x=1", "--engine", "d", "--engine-sha256", "e", "--clock-source", "f",
                   "--cells", "6", "--min-cell-v", "3.9", "--min-free-gb", "5", "--override", "why", "--sim").preflight  # fmt: skip
    assert {item for item in command if item.startswith("--")} <= known
    no_camera = go.plan(go.build_parser().parse_args(["--profile", "comp_2026", "--sequence", "waypoint_navigation_live.yaml", "--params", "live_target_mission.yaml", "--no-camera"]))
    assert {item for item in no_camera.preflight if item.startswith("--")} <= known


def test_the_launch_file_declares_every_argument_go_gives_it():
    text = (REPO / "src/drone_mission_demo/launch/argus_bringup.launch.py").read_text()
    estimator = go.plan(go.build_parser().parse_args(["--profile", "delivery_estimator", "--sequence", "package_delivery_demo.yaml", "--params", "sim_target_mission.yaml", "--sim"]))
    for item in [*made("--sim").launch[4:], *estimator.launch[4:]]:
        assert f'DeclareLaunchArgument(\n                "{item.split(":=", 1)[0]}",' in text, item


def test_a_drop_or_a_delivery_with_no_detector_started_cannot_be_planned():
    """The flight-safety review: it would fly to the target and never see it."""
    blind = ["--profile", "comp_2026", "--sequence", "package_delivery_demo.yaml", "--params", "sim_target_mission.yaml"]
    with pytest.raises(go.GoError, match="package_delivery, which needs the detector"):
        go.plan(go.build_parser().parse_args(blind))
    # The type is read as the registry reads it, and a detector that is started needs its camera checked.
    sys.path.insert(0, str(REPO / "src" / "drone_mission_core"))
    from drone_mission_core import bringup

    assert bringup.missions_without_their_detector({"mission_sequence": {"missions": [{"type": " Package_Delivery "}, {"type": "takeoff"}]}}, "none") == ["package_delivery"]
    assert bringup.missions_without_their_detector({"mission_sequence": {"missions": [{"type": "package_drop"}]}}, "target_cv") == []
    with pytest.raises(go.GoError, match="needs a camera"):
        made("--no-camera")
    waypoints = go.plan(go.build_parser().parse_args(["--profile", "comp_2026", "--sequence", "waypoint_navigation_live.yaml", "--params", "live_target_mission.yaml"]))
    assert "detector:=none" in waypoints.launch and "target_cv" not in waypoints.nodes


def test_the_missions_that_need_the_detector_are_the_ones_built_on_the_bullseye_base():
    import re

    sys.path.insert(0, str(REPO / "src" / "drone_mission_core"))
    from drone_mission_core import bringup

    found = set()
    for path in (REPO / "src/drone_mission_demo/drone_mission_demo/missions").glob("*.py"):
        found |= set(re.findall(r'@register_mission\("([a-z_]+)"\)\nclass \w+\(RedBullseyeMissionBase\)', path.read_text()))
    assert found == set(bringup.DETECTOR_MISSIONS)


def test_a_mission_flag_that_reads_frame_times_needs_the_detector_to_stamp_them(tmp_path):
    """The flight-safety review: use_frame_time_pose with target_cv stamping at publish time is neither 2026 nor 2027."""
    half = profile(tmp_path, "description: x\nenable: [use_frame_time_pose]\nstart: {detector: target_cv}\n")
    with pytest.raises(go.GoError, match="needs the node flag target_cv.stamp_from_image"):
        go.plan(go.build_parser().parse_args(["--profile", half, "--sequence", "package_delivery_demo.yaml", "--params", "sim_target_mission.yaml"]))
    whole = profile(tmp_path, "description: x\nenable: [use_frame_time_pose]\nnodes: {target_cv: [stamp_from_image]}\nstart: {detector: target_cv}\n")
    go.plan(go.build_parser().parse_args(["--profile", whole, "--sequence", "package_delivery_demo.yaml", "--params", "sim_target_mission.yaml"]))
    # A sequence file that has the flag true by itself is held to the same rule: the flags checked are the ones that will fly.
    edited = tmp_path / "edited.yaml"
    edited.write_text((REPO / "src/drone_mission_demo/config/sequences/package_delivery_demo.yaml").read_text().replace("use_frame_time_pose: false", "use_frame_time_pose: true", 1))
    with pytest.raises(go.GoError, match="needs the node flag target_cv.stamp_from_image"):
        go.plan(go.build_parser().parse_args(["--profile", "comp_2026", "--sequence", str(edited), "--params", "sim_target_mission.yaml", "--detector", "target_cv"]))


def test_the_launch_run_by_hand_is_held_to_the_rules_go_checks():
    """The launch calls the same function `go` does, with the executor's own reading of feature_overrides."""
    import yaml

    sys.path.insert(0, str(REPO / "src" / "drone_mission_core"))
    from drone_mission_core import bringup
    from drone_mission_core.features import FeatureFlagError

    sequence = yaml.safe_load((REPO / "src/drone_mission_demo/config/sequences/package_delivery_demo.yaml").read_text())
    started = ("simple_takeoff_service", "target_cv", "mission_executor")
    assert bringup.flight_problems(sequence, "", {}, started, "target_cv") == []
    for text in ("use_frame_time_pose", "use_frame_time_pose = true", " use_frame_time_pose=on "):  # as the executor reads them
        assert "target_cv.stamp_from_image" in bringup.flight_problems(sequence, text, {}, started, "target_cv")[0], text
    assert bringup.flight_problems(sequence, "use_frame_time_pose=false", {}, started, "target_cv") == []
    assert bringup.flight_problems(sequence, "use_frame_time_pose", {"target_cv": ("stamp_from_image",)}, started, "target_cv") == []
    assert "needs the detector" in bringup.flight_problems(sequence, "", {}, ("simple_takeoff_service", "mission_executor"), "none")[0]
    with pytest.raises(FeatureFlagError):
        bringup.flight_problems(sequence, "no_such_flag", {}, started, "target_cv")
    launch_file = (REPO / "src/drone_mission_demo/launch/argus_bringup.launch.py").read_text()
    assert "bringup.flight_problems(" in launch_file and "bringup.undeclared_node_flags(" in launch_file


def test_every_sim_scenario_that_flies_the_detector_on_frame_times_has_it_stamp_them():
    """The rule above is what the sim's flights already do: a scenario that broke it would have been flown on the wrong clock."""
    import yaml

    sys.path.insert(0, str(REPO / "src" / "drone_mission_core"))
    from drone_mission_core import bringup

    checked = 0
    for path in sorted((REPO / "sim" / "scenarios").glob("*/*.yaml")):
        data = yaml.safe_load(path.read_text()) or {}
        flags = [flag for flag, value in (data.get("flags") or {}).items() if value is True]
        node_flags = {node: [flag for flag, value in (item or {}).items() if value is True] for node, item in (data.get("node_flags") or {}).items()}
        sequence = REPO / "src/drone_mission_demo/config/sequences" / str(data.get("sequence"))
        if not sequence.is_file():
            continue
        types = {item.get("type") for item in yaml.safe_load(sequence.read_text())["mission_sequence"]["missions"]}
        if types & set(bringup.DETECTOR_MISSIONS):
            checked += 1
            assert bringup.missing_node_flags(flags, node_flags, ("target_cv",)) == [], path.name
    assert checked >= 20


def test_a_file_named_is_the_installed_one_and_must_be_the_trees(tmp_path):
    """The flight-safety review: the 2026 launch files fly the installed files; `go` must not fly different ones without saying."""
    share = tmp_path / "share"
    for folder, name in (("sequences", "package_delivery_demo.yaml"), ("params", "sim_target_mission.yaml")):
        (share / "config" / folder).mkdir(parents=True)
        (share / "config" / folder / name).write_bytes((REPO / "src/drone_mission_demo/config" / folder / name).read_bytes())
    plan = go.plan(go.build_parser().parse_args(BASE), share)
    assert value(plan.launch, "sequence") == str(share / "config/sequences/package_delivery_demo.yaml")
    assert value(plan.launch, "params") == str(share / "config/params/sim_target_mission.yaml")
    stale = share / "config/sequences/package_delivery_demo.yaml"
    stale.write_text(stale.read_text().replace("target_altitude_m: 15.0", "target_altitude_m: 12.0", 1))
    with pytest.raises(go.GoError, match="is not the checked-out tree's.*rebuild the workspace"):
        go.plan(go.build_parser().parse_args(BASE), share)
    # A file that is in the tree and not in the install (added, not rebuilt) is refused too.
    (share / "config/params/sim_target_mission.yaml").rename(share / "config/params/moved_away.yaml")
    stale.write_bytes((REPO / "src/drone_mission_demo/config/sequences/package_delivery_demo.yaml").read_bytes())
    with pytest.raises(go.GoError, match="not in the installed workspace"):
        go.plan(go.build_parser().parse_args(BASE), share)
    (share / "config/params/moved_away.yaml").rename(share / "config/params/sim_target_mission.yaml")
    stale.write_text(stale.read_text().replace("target_altitude_m: 15.0", "target_altitude_m: 12.0", 1))
    # A file given by its path is flown as given.
    by_path = [*BASE]
    by_path[by_path.index("--sequence") + 1] = str(stale)
    assert value(go.plan(go.build_parser().parse_args(by_path), share).launch, "sequence") == str(stale)


def test_a_sigterm_to_go_is_passed_on_to_the_launch_which_ends_in_its_own_way(tmp_path):
    """The flight-safety review: a `kill` of go used to leave the launch and every node running."""
    import signal
    import subprocess
    import time

    child = tmp_path / "launch_stand_in.py"
    child.write_text(
        "import signal, sys, time, pathlib\n"
        "def stop(*_):\n"
        f"    pathlib.Path({str(tmp_path / 'interrupted')!r}).write_text('SIGINT')\n"
        "    sys.exit(7)\n"
        "signal.signal(signal.SIGINT, stop)\n"
        f"pathlib.Path({str(tmp_path / 'up')!r}).write_text('up')\n"
        "time.sleep(30)\n"
    )
    parent = subprocess.Popen([sys.executable, "-c", f"import sys; sys.path.insert(0, {str(TOOLS)!r}); import go; sys.exit(go._launch([sys.executable, {str(child)!r}]))"])
    deadline = time.monotonic() + 10.0
    while not (tmp_path / "up").exists() and time.monotonic() < deadline:
        time.sleep(0.05)
    assert (tmp_path / "up").exists()
    parent.send_signal(signal.SIGTERM)
    assert parent.wait(timeout=10.0) == 7  # the launch's own exit code, after its own shutdown
    assert (tmp_path / "interrupted").read_text() == "SIGINT"


def test_stopping_a_grabber_ends_the_node_under_its_wrapper_too(tmp_path):
    """The spec audit: `ros2 run` does not pass a SIGTERM on to its node, and a grabber left behind keeps the camera.

    A stand-in for `ros2 run`: a wrapper that starts a node and waits. The wrapper does nothing about signals;
    the node ends on an interrupt. Both must be gone after _stop, which signals the group.
    """
    import os
    import time

    node = tmp_path / "node.py"
    node.write_text(f"import pathlib, os, time\npathlib.Path({str(tmp_path / 'node.pid')!r}).write_text(str(os.getpid()))\ntime.sleep(60)\n")
    wrapper = tmp_path / "wrapper.py"
    wrapper.write_text(f"import signal, subprocess, sys\nsignal.signal(signal.SIGTERM, signal.SIG_DFL)\nsys.exit(subprocess.call([sys.executable, {str(node)!r}]))\n")
    grabber = go._start_grabber([sys.executable, str(wrapper)])
    deadline = time.monotonic() + 10.0
    while not (tmp_path / "node.pid").exists() and time.monotonic() < deadline:
        time.sleep(0.05)
    node_pid = int((tmp_path / "node.pid").read_text())
    assert os.getpgid(node_pid) == grabber.pid != os.getpgid(0)  # a group of its own, with its node in it
    go._stop(grabber)
    assert grabber.poll() is not None and not go._group_alive(grabber.pid)
    with pytest.raises(ProcessLookupError):
        os.kill(node_pid, 0)


def test_a_node_that_ignores_the_interrupt_is_ended_harder(tmp_path):
    import os
    import time

    stubborn = tmp_path / "stubborn.py"
    stubborn.write_text(f"import signal, pathlib, time\nsignal.signal(signal.SIGINT, signal.SIG_IGN)\nsignal.signal(signal.SIGTERM, signal.SIG_IGN)\npathlib.Path({str(tmp_path / 'up')!r}).write_text('up')\ntime.sleep(60)\n")
    grabber = go._start_grabber([sys.executable, str(stubborn)])
    deadline = time.monotonic() + 10.0
    while not (tmp_path / "up").exists() and time.monotonic() < deadline:
        time.sleep(0.05)
    go._stop(grabber, waits=((go.signal.SIGINT, 0.3), (go.signal.SIGTERM, 0.3), (go.signal.SIGKILL, 2.0)))
    assert grabber.poll() == -9 and not go._group_alive(grabber.pid)


def test_a_sigterm_while_the_preflight_runs_still_stops_the_grabber():
    runner = Runner()

    def killed(command):
        runner.calls.append("preflight")
        go.signal.raise_signal(go.signal.SIGTERM)
        return 0

    with pytest.raises(SystemExit) as stopped:
        go.main([*BASE, "--start-camera", "mipi", "--camera", "/camera/image=30"], run=killed, start=runner.start, sleep=runner.sleep, launch=runner.launch, stop=lambda grabber: grabber.terminate())
    assert stopped.value.code == 143 and runner.calls == ["camera", "wait 5 s", "preflight", "camera stopped"]
    assert go.signal.getsignal(go.signal.SIGTERM) is go.signal.SIG_DFL  # and the handler is put back


def test_record_starts_the_bag_at_go_and_closes_it_after_the_launch_with_the_preflights_log_beside_it(tmp_path):
    """PLT-12: the attempt's bag, of key topics and no images, and one folder for the flight."""
    runner = Runner()
    folder = tmp_path / "2027-05-21_2_delivery"
    code = go.main([*BASE, "--camera", "/camera/image=30", "--start-camera", "mipi", "--record", str(folder), "--sim"], run=runner, start=runner.start, sleep=runner.sleep, launch=runner.launch,
                   stop=lambda process: process.terminate(), bag_storage="mcap")  # fmt: skip
    assert code == 0 and runner.calls == ["camera", "wait 5 s", "preflight", "recorder", "launch", "recorder stopped", "camera stopped"]
    command = runner.commands["recorder"]
    assert command[:6] == ["ros2", "bag", "record", "--storage", "mcap", "--output"] and command[6] == str(folder / "bag") and "--use-sim-time" in command
    topics = [item for item in command if item.startswith("/")][1:]  # after the output path
    assert "/mavros/local_position/pose" in topics and "/mavros/state" in topics
    assert not any("image" in topic and "camera_info" not in topic and "image_size" not in topic for topic in topics), topics  # no images, and no taps on them
    preflight = runner.commands["preflight"]
    assert preflight[preflight.index("--log-dir") + 1] == str(folder)
    # What was flown is written in the flight's own folder, for the attempt's bundle.
    import json

    flown = json.loads((folder / "go.json").read_text())
    assert flown["profile"] == str(REPO / "profiles" / "comp_2026.yaml") and flown["sequence"].endswith("config/sequences/package_delivery_demo.yaml")
    assert flown["params"].endswith("config/params/sim_target_mission.yaml") and flown["mission_flags"] == "" and flown["launch"] == runner.commands["launch"]


def test_a_no_go_records_nothing_and_a_folder_that_has_a_bag_is_refused(tmp_path):
    runner = Runner(preflight_code=1)
    folder = tmp_path / "attempt"
    assert go.main([*BASE, "--record", str(folder)], run=runner, start=runner.start, sleep=runner.sleep, launch=runner.launch, stop=lambda process: process.terminate(), bag_storage="mcap") == 1
    assert runner.calls == ["preflight"]
    (folder / "bag").mkdir(parents=True)
    with pytest.raises(go.GoError, match="exists already"):
        made("--record", str(folder))
    explicit = made("--record", str(tmp_path / "other"), "--log-dir", "/mnt/argus-logs")
    assert explicit.preflight[explicit.preflight.index("--log-dir") + 1] == "/mnt/argus-logs" and explicit.record_to == tmp_path / "other" / "bag"
    assert made().record_to is None


def test_a_sequence_generated_from_an_unverified_day_file_is_for_the_sim_only(tmp_path):
    """OPS-01: tools/field_setup.py marks such a file in its first lines; go will not fly it on the aircraft."""
    source = (REPO / "src/drone_mission_demo/config/sequences/package_delivery_demo.yaml").read_text()
    unverified = tmp_path / "package_delivery_demo.yaml"
    unverified.write_text("# GENERATED by tools/field_setup.py: do not edit.\n# NOT VERIFIED: the day file's coordinates have not been checked. Not for flight.\n" + source)
    arguments = ["--profile", "comp_2026", "--sequence", str(unverified), "--params", "sim_target_mission.yaml", "--detector", "target_cv"]
    with pytest.raises(go.GoError, match="not verified.*for the sim only"):
        go.plan(go.build_parser().parse_args(arguments))
    assert go.plan(go.build_parser().parse_args([*arguments, "--sim"])).launch  # the sim may rehearse it
    verified = tmp_path / "verified.yaml"
    verified.write_text("# GENERATED by tools/field_setup.py: do not edit.\n" + source)
    assert go.plan(go.build_parser().parse_args(["--profile", "comp_2026", "--sequence", str(verified), "--params", "sim_target_mission.yaml", "--detector", "target_cv"])).launch
