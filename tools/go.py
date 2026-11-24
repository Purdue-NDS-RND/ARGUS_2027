#!/usr/bin/env python3
"""GO: the one command that starts a flight (PLT-07). In the sim and on the aircraft.

  python3 tools/go.py --profile profiles/comp_2026.yaml --sequence package_delivery_live.yaml --params live_target_mission.yaml
  python3 tools/go.py --profile ... --sequence ... --params ... --override "<reason>"     # go ahead on a NO-GO
  python3 tools/go.py --profile ... --sequence ... --params ... --dry-run                  # print the two commands, start nothing
  python3 tools/go.py --profile ... --sequence ... --params ... --record <folder>          # and record the flight's bag there (PLT-12)

It does these things, in order, and stops at the first that fails:

  1. Reads the profile against the sequence and the params file it will be
     flown with: every mission flag declared in the sequence, every node flag
     declared (false) in the params file, every dependency met.
  2. If the flight uses a 2026 grabber (`--start-camera mipi|usb`, or the
     profile's `start: camera:`), starts it now and gives it a few seconds:
     the preflight checks the camera's rate, and a camera that the launch
     would start only after the preflight could never pass. A grabber reads
     a camera and commands nothing. It is stopped again on NO-GO.
  3. Runs the preflight (`ros2 run drone_utils preflight ...`, PLT-10) and
     prints its lines. NO-GO stops here: no flight node is started.
     `--override` with a reason goes ahead, and the preflight logs who and why.
  4. Starts the flight: `ros2 launch drone_mission_demo argus_bringup.launch.py`
     with the sequence, the params, the profile's mission flags and node
     flags. This command then waits on the launch: it returns when the
     flight's nodes stop, a Ctrl-C stops them as it would under `ros2 launch`,
     and a SIGTERM to this command is passed on to the launch.

With `--record FOLDER` the flight's key topics (sim/replay/topics.py: no
images) are recorded to FOLDER/bag from GO until the launch ends, and the
preflight's log goes to FOLDER too unless `--log-dir` says otherwise: FOLDER
is then what tools/bundle_attempt.py makes the attempt's bundle from.

MAVROS is not started here: on the aircraft it is a boot service
(platform/jetson, PLT-02), in the sim the sim starts it. With no MAVROS the
preflight reads nothing and says NO-GO. Nor is the 2027 camera driver, a boot
service too.

The last line it prints before the launch is `GO` or `NO-GO`.

Exit code: the launch's own after a GO; 1 for NO-GO; 2 when the profile, the
sequence or the params file cannot be flown as written.

`plan` is the part with no ROS in it: it turns the arguments into the two
commands. Everything the preflight needs to know about this aircraft or this
sim (the baseline file, where the parameters are read from, the cameras and
their rates, the log folder, the battery's cells) is passed through.
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "drone_mission_core"))

CONFIG = ROOT / "src" / "drone_mission_demo" / "config"
BASELINE = ROOT / "platform" / "fcu" / "argus_2027.param"
CLOCK_TOOL = ROOT / "platform" / "jetson" / "clock_sync.py"
LAUNCH = ("drone_mission_demo", "argus_bringup.launch.py")
GO, NO_GO = "GO", "NO-GO"


class GoError(ValueError):
    """The flight cannot be started as asked. The message says why."""


@dataclass(frozen=True)
class Plan:
    camera: List[str]  # the grabber's command, started before the preflight; empty when the flight starts none
    record_to: Optional[Path]  # where the flight's bag is recorded (--record FOLDER gives FOLDER/bag), or None
    preflight: List[str]
    launch: List[str]
    flags: str  # the mission flags, for the operator to read
    node_flags: str
    nodes: tuple
    files: tuple = ()  # (profile, sequence, params) as flown: written beside a recording for the attempt's bundle


def installed_share() -> Optional[Path]:
    """The installed drone_mission_demo package's share folder, or None with no ROS workspace sourced."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("drone_mission_demo"))
    except Exception:  # no ament, or the package is not installed in this environment
        return None


def _find(name: str, folder: str, what: str, share: Optional[Path]) -> Path:
    """A sequence or params file by path, or by name.

    By name it is the installed workspace's file, the one the 2026 launch files fly, when a workspace is sourced;
    and it must be the checked-out tree's file too, byte for byte. An install that is behind the tree would fly
    yesterday's tuned values under today's flags, so that is refused until the workspace is rebuilt.
    """
    path = Path(name)
    if path.is_file():
        return path.resolve()
    source = CONFIG / folder / name
    installed = None if share is None else share / "config" / folder / name
    if not source.is_file() and (installed is None or not installed.is_file()):
        raise GoError(f"no {what} file '{name}' (looked in {CONFIG / folder})")
    if installed is None:
        return source.resolve()  # no workspace sourced: a dry run, or a test
    if not installed.is_file():
        raise GoError(f"the {what} file {source} is in the checked-out tree and not in the installed workspace: rebuild the workspace (colcon build), or give the file's path")
    if source.is_file() and installed.read_bytes() != source.read_bytes():
        raise GoError(f"the installed {what} file {installed} is not the checked-out tree's {source}: rebuild the workspace (colcon build), or give the file's path")
    return installed.resolve()


def plan(args, share: Optional[Path] = None) -> Plan:
    """The preflight command and the launch command for these arguments. Raises GoError for a flight that cannot be started.

    `share` is the installed drone_mission_demo share folder (installed_share()), when there is one.
    """
    import yaml
    from drone_mission_core import bringup
    from drone_mission_core.features import FeatureFlagError

    profile = Path(args.profile)
    if not profile.is_file():
        profile = ROOT / "profiles" / (args.profile if args.profile.endswith(".yaml") else f"{args.profile}.yaml")
    if not profile.is_file():
        raise GoError(f"no profile '{args.profile}' (looked in {ROOT / 'profiles'})")
    sequence, params = _find(args.sequence, "sequences", "sequence", share), _find(args.params, "params", "params", share)
    try:
        params_data = yaml.safe_load(params.read_text(encoding="utf-8")) or {}
        if "features" in params_data:
            raise GoError(f"{params} has `features:` at its top level: no node of the launch would start")
        made = bringup.load_bringup(profile, params_data, log_session=not args.no_session_log, camera=args.start_camera, detector=args.detector)
        sequence_text = sequence.read_text(encoding="utf-8")
        if not args.sim and any(line.startswith("# NOT VERIFIED") for line in sequence_text.splitlines()[:8]):
            raise GoError(f"{sequence} was generated from a day file that is not verified (tools/field_setup.py): for the sim only (--sim), not for the aircraft")
        sequence_data = yaml.safe_load(sequence_text) or {}
        problems = bringup.flight_problems(sequence_data, made.feature_overrides, made.node_flags, made.nodes, made.detector)
        if problems:
            raise GoError(f"{sequence.name} with {profile.name}: " + "; ".join(problems) + " (--detector target_cv, or a profile whose start: names it, starts the detector)")
        if made.detector != "none" and args.no_camera:
            raise GoError("this flight starts the detector, which needs a camera: name it for the preflight with --camera TOPIC=HZ instead of --no-camera")
    except (bringup.BringupError, FeatureFlagError, yaml.YAMLError) as problem:
        raise GoError(str(problem)) from problem

    record_to = None
    if args.record:
        record_to = Path(args.record).resolve() / "bag"
        if record_to.exists():
            raise GoError(f"{record_to} exists already: every flight's recording gets a folder of its own")
        if args.log_dir is None:
            args.log_dir = str(record_to.parent)  # one folder for the flight: the preflight's log beside the bag
    preflight = ["ros2", "run", "drone_utils", "preflight", "--profile", str(profile), "--sequence", str(sequence), "--baseline", str(args.baseline)]
    for option, value in (("--param-dump", args.param_dump), ("--fc-url", args.fc_url), ("--log-dir", args.log_dir), ("--engine", args.engine), ("--engine-sha256", args.engine_sha256),
                          ("--clock-source", args.clock_source), ("--cells", args.cells), ("--min-cell-v", args.min_cell_v), ("--min-free-gb", args.min_free_gb), ("--override", args.override)):  # fmt: skip
        if value is not None:
            preflight += [option, str(value)]
    if args.clock_source:
        preflight += ["--clock-tool", str(CLOCK_TOOL)]
    for camera in args.camera or []:
        preflight += ["--camera", camera]

    # The grabber is started by this command, before the preflight, exactly as the launch would start it (same
    # executable, node name and params file); the launch is then told to start none.
    camera: List[str] = []
    if made.camera != "none":
        grabber = f"{made.camera}_grabber"
        if made.node_flags.get(grabber):
            # No grabber has a flag today. One that did would be started here without it, and the launch, told to
            # start no camera, would then refuse the flag after GO.
            raise GoError(f"the profile sets flags for {grabber}, which this command starts before the preflight without them: not supported")
        camera = ["ros2", "run", bringup.NODES[grabber][0], bringup.NODES[grabber][1], "--ros-args", "-r", f"__node:={grabber}", "--params-file", str(params)]
        if args.sim:
            camera += ["-p", "use_sim_time:=true"]
    launch = ["ros2", "launch", *LAUNCH, f"sequence:={sequence}", f"params:={params}", "camera:=none", f"detector:={made.detector}",
              f"log_session:={'false' if args.no_session_log else 'true'}"]  # fmt: skip
    # `ros2 launch` refuses an argument with nothing after `:=`. With no flags the two arguments are left out, and
    # the launch file's own defaults (no flags) stand.
    if made.feature_overrides:
        launch.append(f"feature_overrides:={made.feature_overrides}")
    if made.node_flags:
        launch.append(f"node_flags:={made.node_flags_argument()}")
    if args.sim:
        launch.append("use_sim_time:=true")
        preflight.append("--sim-time")
    if args.no_camera:
        preflight.append("--no-camera")
    return Plan(camera, record_to, preflight, launch, made.feature_overrides, made.node_flags_argument(), made.nodes, (str(profile.resolve()), str(sequence), str(params)))


def recorder_command(out: Path, sim: bool, storage: Optional[str] = None) -> List[str]:
    """The `ros2 bag record` command for the flight's bag: the recorder's own topic list (no images, no taps).

    MCAP where rosbag2's MCAP plugin is installed, sqlite3 otherwise, as the sim's recorder chooses.
    """
    sys.path.insert(0, str(ROOT / "sim"))
    from replay import record_run

    if storage is None:
        storage = record_run.choose_storage(record_run.mcap_installed())
    return record_run.record_command(out, storage, sim, tapped_images=())


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ARGUS GO: preflight, then the flight.")
    parser.add_argument("--profile", required=True, help="a profile's name or file (profiles/<name>.yaml)")
    parser.add_argument("--sequence", required=True, help="the mission sequence: a file, or a name under config/sequences")
    parser.add_argument("--params", required=True, help="the params file: a file, or a name under config/params")
    parser.add_argument("--sim", action="store_true", help="the flight is in the sim: every node on sim time")
    parser.add_argument("--override", metavar="REASON", help="go ahead on a NO-GO, for this reason (logged by the preflight)")
    parser.add_argument("--dry-run", action="store_true", help="print the preflight and launch commands and start nothing")
    parser.add_argument("--no-session-log", action="store_true", help="do not start the session logger")
    parser.add_argument("--start-camera", choices=("none", "mipi", "usb"), help="the 2026 grabber to start, over the profile's start.camera")
    parser.add_argument("--detector", choices=("none", "target_cv"), help="the detector to start, over the profile's start.detector")
    parser.add_argument("--camera-wait-s", type=float, default=5.0, help="how long a grabber started here is given before the preflight looks at it")
    parser.add_argument("--record", metavar="FOLDER", help="record the flight's key topics to FOLDER/bag (no images); the preflight's log goes to FOLDER too")
    group = parser.add_argument_group("passed to the preflight")
    group.add_argument("--baseline", default=str(BASELINE))
    group.add_argument("--param-dump")
    group.add_argument("--fc-url")
    group.add_argument("--log-dir")
    group.add_argument("--camera", action="append", metavar="TOPIC=HZ", help="a camera topic the flight needs and its rate; repeat for more")
    group.add_argument("--no-camera", action="store_true", help="the flight needs no camera")
    group.add_argument("--engine")
    group.add_argument("--engine-sha256")
    group.add_argument("--clock-source")
    group.add_argument("--cells", type=int)
    group.add_argument("--min-cell-v", type=float)
    group.add_argument("--min-free-gb", type=float)
    return parser


def _launch(command: List[str]) -> int:
    """Run the launch and stay out of its way: its exit code.

    `ros2 launch` stops its nodes in its own order when it is interrupted (SIGINT, then harder). A Ctrl-C at the
    terminal reaches it directly, and this command waits for it to finish instead of killing it. A SIGTERM sent
    to this command alone (a `kill`, a service stop) is passed on as that same interrupt, so the flight's nodes
    never outlive the command that started them.
    """
    process = subprocess.Popen(command)

    def pass_on(_signum, _frame) -> None:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)

    before = signal.signal(signal.SIGTERM, pass_on)
    try:
        while True:
            try:
                return process.wait()
            except KeyboardInterrupt:
                continue  # the launch got the same Ctrl-C: wait for it to stop its nodes
    finally:
        signal.signal(signal.SIGTERM, before)


def _start_grabber(command: List[str]):
    """Start a grabber in a process group of its own, so that it and its node can be ended together (see _stop)."""
    return subprocess.Popen(command, start_new_session=True)


def _group_alive(pid: int) -> bool:
    try:
        os.killpg(pid, 0)
    except (ProcessLookupError, PermissionError):
        return False
    return True


def _stop(process, waits=((signal.SIGINT, 5.0), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0))) -> None:
    """End a grabber this command started: the `ros2 run` wrapper and the node under it.

    The whole process group is signalled, not the wrapper alone: `ros2 run` does not pass a SIGTERM on to its
    node, and a grabber left behind would keep the camera from the next flight's. An interrupt first, as a
    Ctrl-C would give both; then harder, until nothing of the group is left.
    """
    for signum, wait_s in waits:
        try:
            os.killpg(process.pid, signum)
        except (ProcessLookupError, PermissionError):
            break
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            process.poll()  # reap the wrapper, or the group never reads as gone
            if not _group_alive(process.pid):
                return
            time.sleep(0.05)
    process.poll()


def main(argv: Optional[List[str]] = None, run=subprocess.call, start=_start_grabber, sleep=time.sleep, launch=_launch, stop=_stop, bag_storage: Optional[str] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        made = plan(args, installed_share())
    except GoError as problem:
        print(f"go: {problem}", file=sys.stderr)
        print(f"{NO_GO}: the flight cannot be started as asked")
        return 2
    # Flushed line by line: the preflight and the launch write to the same output, and the order must be the order of events.
    print(f"profile: {args.profile}; mission flags: {made.flags or '(none: the 2026 code)'}; node flags: {made.node_flags or '(none)'}")
    print(f"nodes: {', '.join(made.nodes)}")
    print(f"sequence: {made.launch[4].split(':=', 1)[1]}\nparams: {made.launch[5].split(':=', 1)[1]}", flush=True)
    if args.dry_run:
        if made.camera:
            print("camera:    " + shlex.join(made.camera))
        print("preflight: " + shlex.join(made.preflight))
        if made.record_to is not None:
            print("record:    " + shlex.join(recorder_command(made.record_to, args.sim, bag_storage or "mcap")))
        print("launch:    " + shlex.join(made.launch))
        return 0
    grabber, recorder = None, None

    def ended(_signum, _frame) -> None:
        raise SystemExit(143)  # a SIGTERM while the grabber or the preflight runs: leave through the `finally` below

    before = signal.signal(signal.SIGTERM, ended)
    try:
        if made.camera:
            print(f"camera: starting {made.nodes[0]} for the preflight to look at", flush=True)
            grabber = start(made.camera)
            sleep(args.camera_wait_s)
        if run(made.preflight) != 0:
            print(NO_GO, flush=True)
            return 1
        print(GO, flush=True)
        if made.record_to is not None:
            # From GO to the end of the launch, in a process group of its own: the recorder closes its file on the
            # interrupt it is stopped with, after the flight's nodes have gone.
            made.record_to.parent.mkdir(parents=True, exist_ok=True)
            # What was flown, in the flight's own folder: tools/bundle_attempt.py takes the config from here.
            flown = {"profile": made.files[0], "sequence": made.files[1], "params": made.files[2], "mission_flags": made.flags, "node_flags": made.node_flags,
                     "nodes": list(made.nodes), "launch": made.launch}  # fmt: skip
            (made.record_to.parent / "go.json").write_text(json.dumps(flown, indent=2) + "\n", encoding="utf-8")
            print(f"record: the flight's bag goes to {made.record_to}", flush=True)
            recorder = start(recorder_command(made.record_to, args.sim, bag_storage))
        return launch(made.launch)
    finally:
        signal.signal(signal.SIGTERM, before)
        for process in (recorder, grabber):
            if process is not None:
                stop(process)


if __name__ == "__main__":
    sys.exit(main())
