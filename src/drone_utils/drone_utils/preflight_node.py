#!/usr/bin/env python3
"""The preflight go/no-go as a command (PLT-10): listen to the aircraft for a few seconds, print GO or NO-GO.

  ros2 run drone_utils preflight --profile profiles/comp_2026.yaml --sequence <sequence.yaml> \\
      --baseline platform/fcu/argus_2027.param --param-dump <today's dump> --log-dir /mnt/argus-logs \\
      --camera /camera/image=30

Exit code 0 for GO (or NO-GO overridden with --override "<reason>"), 1 for
NO-GO, 2 when it could not run (a file that is not there, a link that did
not answer): that is NO-GO too, printed and logged as one, and cannot be
overridden. Every run is appended to
<log dir>/preflight.jsonl; an override is logged with who and why before the
command says GO, and if it cannot be logged it is not an override.

This file is the thin side: it fills a `Snapshot` and hands it to the rules
(preflight.py). What it reads:

  /mavros/sys_status            fence, estimate and pre-arm health bits
  /mavros/gpsstatus/gps1/raw    fix type, HDOP, satellites
  /mavros/rangefinder/rangefinder
  /mavros/battery
  /mavros/timesync_status
  each --camera TOPIC=HZ        counted without decoding the images

and, for the parameters, the first of: `--fc-url` (MAVLink, with pymavlink:
the sim's tools port), MAVROS's param plugin if this MAVROS has it loaded,
or `--param-dump` (the ground station's file, saved today). With none of the
three the parameter check is NO-GO.
"""

from __future__ import annotations

import argparse
import getpass
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional

from drone_utils import preflight as pf


# What the preflight reads over MAVROS, and the MAVLink message behind each. A message that is not arriving is asked
# for at the stream keeper's own rate (stream_keeper.MESSAGE_RATES_HZ, which is what flies); one that is arriving
# is left alone, so the preflight never lowers a rate. TIMESYNC is MAVROS's own exchange and is not asked for.
MESSAGE_OF = {"sys_status": 1, "gps": 24, "battery": 147, "range": 173}
FENCE_POINTS = "FENCE_TOTAL"  # how many fence points the flight controller has stored
ASK_AFTER_S = 1.5  # of listening without hearing a message, before it is asked for


def to_ask_for(heard) -> Dict[int, float]:
    """{MAVLink message id: rate in Hz} for what the preflight reads and has not heard, at the stream keeper's rates."""
    from drone_utils.stream_keeper import MESSAGE_RATES_HZ

    return {MESSAGE_OF[name]: MESSAGE_RATES_HZ[MESSAGE_OF[name]][1] for name in MESSAGE_OF if name not in heard}


def may_ask(state) -> bool:
    """Whether the flight controller may be asked for messages now: only when it is known to be disarmed.

    The request goes through MAVROS's command queue, which a mode change also waits in: nothing is put in it
    under a vehicle that is armed, or whose state is not known.
    """
    return state is not None and bool(getattr(state, "connected", False)) and not bool(getattr(state, "armed", True))


def parse_cameras(items: List[str]) -> Dict[str, float]:
    """["/camera/image=30"] -> {"/camera/image": 30.0}. Raises ValueError for anything else."""
    cameras = {}
    for item in items or []:
        topic, sep, rate = item.partition("=")
        try:
            hz = float(rate)
        except ValueError:
            hz = 0.0
        if not sep or not topic.startswith("/") or not hz > 0.0:
            raise ValueError(f"--camera takes TOPIC=HZ (for example /camera/image=30), got {item!r}")
        cameras[topic] = hz
    return cameras


def clock_verdict(tool: Optional[str], source: Optional[str]):
    """chrony's word through platform/jetson/clock_sync.py: (ok, its line), or None when no clock source is named."""
    if not source:
        return None
    if not tool:
        return False, "no clock tool given (platform/jetson/clock_sync.py)"
    try:
        asked = subprocess.run([sys.executable, tool, "--source", source], capture_output=True, text=True, timeout=15)
    except (OSError, subprocess.TimeoutExpired) as problem:
        return False, f"the clock tool could not be run ({problem})"
    line = (asked.stdout.strip().splitlines() or ["the clock tool said nothing"])[0]
    return asked.returncode == 0, line.replace("CLOCK OK: ", "").replace("CLOCK NOT OK: ", "")


def _mavutil():
    """pymavlink's mavutil. ImportError when it is not installed."""
    try:
        from pymavlink import mavutil
    except ImportError:
        # pymavlink installed for the user, in an environment that hides the user's packages (the sim's image sets
        # PYTHONNOUSERSITE for ROS): look there too, after everything else, and for nothing but this.
        import site

        user_site = site.getusersitepackages()
        if user_site not in sys.path:
            sys.path.append(user_site)
        from pymavlink import mavutil
    return mavutil


def params_from_fc(url: str, names: List[str], timeout_s: float = 10.0) -> Dict[str, Optional[float]]:
    """The named parameters, read from the flight controller over MAVLink. A name it did not answer for is None."""
    mavutil = _mavutil()

    master = mavutil.mavlink_connection(url, source_system=247, source_component=195)
    if master.wait_heartbeat(timeout=timeout_s) is None:
        raise RuntimeError(f"no heartbeat from the flight controller on {url}")
    values: Dict[str, Optional[float]] = {name: None for name in names}
    deadline = time.monotonic() + timeout_s
    asked_at = 0.0
    while time.monotonic() < deadline and any(value is None for value in values.values()):
        if time.monotonic() - asked_at > 1.0:
            for name, value in values.items():
                if value is None:
                    master.mav.param_request_read_send(1, 1, name.encode(), -1)
            asked_at = time.monotonic()
        message = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
        if message is not None:
            name = (message.param_id if isinstance(message.param_id, str) else message.param_id.decode()).rstrip("\x00")
            if name in values:
                values[name] = float(message.param_value)
    return values


def listen(args, cameras: Dict[str, float]) -> pf.Snapshot:
    """Spin a node for `args.listen_s` and return what was heard."""
    import rclpy
    from mavros_msgs.msg import GPSRAW, State, SysStatus, TimesyncStatus
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import BatteryState, Image, Range

    rclpy.init(args=None)
    overrides = []
    if args.sim_time:
        # In the sim every node is on Gazebo's clock (FND-03), this one included: ages and rates are then counted
        # in the sim's seconds, which pass more slowly than the wall's on a loaded laptop.
        from rclpy.parameter import Parameter

        overrides.append(Parameter("use_sim_time", value=True))
    node = Node("argus_preflight", parameter_overrides=overrides)
    heard: dict = {"offsets": [], "frames": {topic: [] for topic in cameras}}

    def now_s() -> float:
        return node.get_clock().now().nanoseconds * 1.0e-9

    if args.sim_time:
        waited = time.monotonic()
        while now_s() == 0.0:  # no /clock heard yet
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() - waited > 10.0:
                node.destroy_node()
                rclpy.shutdown()
                raise RuntimeError("--sim-time was given but no /clock was heard in 10 s")

    def keep(name):
        def callback(message):
            heard[name] = (now_s(), message)

        return callback

    def on_timesync(message):
        heard["timesync"] = now_s()
        heard["offsets"].append(int(message.estimated_offset_ns))

    node.create_subscription(State, "/mavros/state", keep("state"), 10)
    node.create_subscription(SysStatus, "/mavros/sys_status", keep("sys_status"), qos_profile_sensor_data)
    node.create_subscription(GPSRAW, "/mavros/gpsstatus/gps1/raw", keep("gps"), qos_profile_sensor_data)
    node.create_subscription(Range, "/mavros/rangefinder/rangefinder", keep("range"), qos_profile_sensor_data)
    node.create_subscription(BatteryState, "/mavros/battery", keep("battery"), qos_profile_sensor_data)
    node.create_subscription(TimesyncStatus, "/mavros/timesync_status", on_timesync, qos_profile_sensor_data)
    for topic in cameras:
        node.create_subscription(Image, topic, lambda _raw, topic=topic: heard["frames"][topic].append(now_s()), qos_profile_sensor_data, raw=True)

    # MAVROS's param plugin, if this MAVROS has it: the parameters as ROS parameters of /mavros/param.
    param_client = None
    if args.baseline and not args.fc_url:
        from rcl_interfaces.srv import GetParameters

        param_client = node.create_client(GetParameters, "/mavros/param/get_parameters")

    started = time.monotonic()
    started_clock_s = now_s()
    asked = False
    while time.monotonic() - started < args.listen_s:
        rclpy.spin_once(node, timeout_sec=0.05)
        if not asked and time.monotonic() - started >= ASK_AFTER_S:
            # A flight controller on a serial link sends little until it is asked (SITL sends everything, and so does
            # an aircraft whose stream keeper is running). What has not been heard by now is asked for, once.
            asked = True
            silent = to_ask_for(heard)
            if silent and may_ask(heard.get("state", (None, None))[1]):
                from mavros_msgs.srv import MessageInterval

                client = node.create_client(MessageInterval, "/mavros/set_message_interval")
                if client.wait_for_service(timeout_sec=0.5):
                    for message_id, rate_hz in silent.items():
                        client.call_async(MessageInterval.Request(message_id=message_id, message_rate=rate_hz))
    listened_s = max(1.0e-3, now_s() - started_clock_s)  # on the node's clock, as the frames are counted

    snapshot = pf.Snapshot(now_s=now_s())
    age = lambda name: None if name not in heard else snapshot.now_s - heard[name][0]  # noqa: E731
    if "sys_status" in heard:
        message = heard["sys_status"][1]
        snapshot.sensors_present, snapshot.sensors_enabled, snapshot.sensors_health = int(message.sensors_present), int(message.sensors_enabled), int(message.sensors_health)
        snapshot.sys_status_age_s = age("sys_status")
    if "gps" in heard:
        message = heard["gps"][1]
        snapshot.gps_fix_type, snapshot.gps_satellites, snapshot.gps_age_s = int(message.fix_type), int(message.satellites_visible), age("gps")
        snapshot.gps_hdop = None if int(message.eph) == 65535 else int(message.eph) / 100.0  # GPS_RAW_INT.eph: HDOP x 100, 65535 unknown
    if "range" in heard:
        message = heard["range"][1]
        snapshot.range_m, snapshot.range_min_m, snapshot.range_max_m, snapshot.range_age_s = float(message.range), float(message.min_range), float(message.max_range), age("range")
    if "battery" in heard:
        snapshot.battery_v, snapshot.battery_age_s = float(heard["battery"][1].voltage), age("battery")
    if "timesync" in heard:
        snapshot.timesync_age_s = snapshot.now_s - heard["timesync"]
        snapshot.timesync_jitter_ms, snapshot.timesync_largest_step_ms = pf.timesync_steps_ms(heard["offsets"])
    if cameras or args.no_camera:
        snapshot.camera_rates = {topic: (len(heard["frames"][topic]) / listened_s, hz) for topic, hz in cameras.items()}

    if param_client is not None and param_client.wait_for_service(timeout_sec=1.0):
        from rcl_interfaces.srv import GetParameters

        baseline = pf.read_param_file(args.baseline)
        names = sorted(baseline)
        future = param_client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        result = future.result()
        if result is not None and len(result.values) == len(names):
            have = {}
            for name, value in zip(names, result.values):
                have[name] = float(value.double_value) if value.type == 3 else float(value.integer_value) if value.type == 2 else None
            snapshot.params_source, snapshot.params_problems = "flight controller (MAVROS)", pf.compare_params(have, baseline)
        count = param_client.call_async(GetParameters.Request(names=[FENCE_POINTS]))
        rclpy.spin_until_future_complete(node, count, timeout_sec=2.0)
        if count.result() is not None and count.result().values and count.result().values[0].type == 2:
            snapshot.fence_points = int(count.result().values[0].integer_value)
    node.destroy_node()
    rclpy.shutdown()
    return snapshot


def fill_offline(snapshot: pf.Snapshot, args) -> pf.Snapshot:
    """The parts of the snapshot that need no ROS: parameters from MAVLink or a dump, the disk, the clock, the engine, the profile."""
    if args.baseline:
        snapshot.baseline_sha256 = pf.sha256_of(args.baseline)
        baseline = pf.read_param_file(args.baseline)
        if args.fc_url:
            have = params_from_fc(args.fc_url, sorted({*baseline, FENCE_POINTS}))
            snapshot.params_source, snapshot.params_problems = "flight controller", pf.compare_params(have, baseline)
            snapshot.fence_points = None if have.get(FENCE_POINTS) is None else int(have[FENCE_POINTS])
        elif snapshot.params_problems is None and args.param_dump:
            dump = Path(args.param_dump)
            dumped = pf.read_param_file(dump)
            snapshot.params_source = str(dump)
            snapshot.params_age_h = (time.time() - dump.stat().st_mtime) / 3600.0
            snapshot.params_problems = pf.compare_params(dumped, baseline)
            snapshot.fence_points = None if FENCE_POINTS not in dumped else int(dumped[FENCE_POINTS])
    if args.log_dir:
        snapshot.log_dir = str(args.log_dir)
        try:
            snapshot.disk_free_gb = shutil.disk_usage(args.log_dir).free / 1.0e9
        except OSError:
            snapshot.disk_free_gb = None
    snapshot.clock_wanted = bool(args.clock_source)
    snapshot.clock = clock_verdict(args.clock_tool, args.clock_source)
    if args.engine:
        snapshot.engine = {"path": str(args.engine), "sha256": pf.sha256_of(args.engine), "manifest_sha256": args.engine_sha256}
    if args.profile and args.sequence:
        snapshot.profile, snapshot.profile_name = pf.check_profile(args.profile, args.sequence), Path(args.profile).stem
    return snapshot


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ARGUS preflight: GO or NO-GO.")
    parser.add_argument("--profile", help="the flag profile to be flown (profiles/<name>.yaml)")
    parser.add_argument("--sequence", help="the mission sequence to be flown")
    parser.add_argument("--baseline", help="the flight controller's baseline (platform/fcu/argus_2027.param)")
    parser.add_argument("--param-dump", help="the aircraft's parameters as the ground station saved them today")
    parser.add_argument("--fc-url", help="read the parameters over MAVLink instead (pymavlink URL; the sim's tools port)")
    parser.add_argument("--log-dir", help="where this flight's logs go; the preflight's own log is written there")
    parser.add_argument("--camera", action="append", default=[], metavar="TOPIC=HZ", help="a camera the flight needs and its mode's rate; repeat for more")
    parser.add_argument("--no-camera", action="store_true", help="the flight needs no camera (with no --camera and without this, the camera check is NO-GO)")
    parser.add_argument("--engine", help="the detector's engine file, when the profile uses one")
    parser.add_argument("--engine-sha256", help="its sha256 in the model registry's manifest")
    parser.add_argument("--clock-source", help="the ground laptop's address: the companion's clock must follow it")
    parser.add_argument("--clock-tool", help="platform/jetson/clock_sync.py")
    parser.add_argument("--cells", type=int, default=pf.Limits().cells)
    parser.add_argument("--min-cell-v", type=float, default=pf.Limits().min_cell_v)
    parser.add_argument("--min-free-gb", type=float, default=pf.Limits().min_free_gb)
    parser.add_argument("--listen-s", type=float, default=5.0, help="how long to listen to the aircraft")
    parser.add_argument("--sim-time", action="store_true", help="the sim on one clock: this node takes its time from /clock too")
    parser.add_argument("--override", metavar="REASON", help="go ahead on a NO-GO, for this reason; it is logged with your user name")
    parser.add_argument("--out", help="also write this run's record here as JSON")
    return parser


def judge_and_log(args, checks: List[pf.Check]) -> int:
    """Print the checks and the verdict, log the run; the exit code for the verdict."""
    import json

    who = getpass.getuser()
    result = pf.verdict(checks, args.override, who)
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    record = pf.log_record(checks, result, stamp, args.override, who, {"profile": args.profile, "sequence": args.sequence, "baseline_sha256": pf.sha256_of(args.baseline) if args.baseline else None})
    logged = True
    if args.log_dir:
        try:
            pf.append_log(Path(args.log_dir) / "preflight.jsonl", record)
        except OSError as problem:
            logged = False
            print(f"preflight: could not write {Path(args.log_dir) / 'preflight.jsonl'}: {problem}", file=sys.stderr)
    else:
        logged = False
    if args.out:
        try:
            Path(args.out).write_text(json.dumps(record, indent=2) + "\n")
        except OSError as problem:
            print(f"preflight: could not write {args.out}: {problem}", file=sys.stderr)
    print("\n".join(pf.report_lines(checks, result)))
    if result.go and not result.clean and not logged:
        print(f"{pf.NO_GO}: the override could not be logged (give --log-dir a folder that can be written), so it does not stand")
        return 1
    return 0 if result.go else 1


def run(args, snapshot: pf.Snapshot) -> int:
    """Judge a filled snapshot, print, log; the exit code."""
    limits = pf.Limits(cells=args.cells, min_cell_v=args.min_cell_v, min_free_gb=args.min_free_gb)
    return judge_and_log(args, pf.run_checks(snapshot, limits))


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        cameras = parse_cameras(args.camera)
        snapshot = listen(args, cameras)
        snapshot = fill_offline(snapshot, args)
    except (ValueError, OSError, RuntimeError, ImportError) as problem:
        # A preflight that could not look is NO-GO like any other, and is logged; an override cannot stand on it.
        print(f"preflight: {problem}", file=sys.stderr)
        args.override = None
        judge_and_log(args, pf.could_not_run(str(problem)))
        return 2
    return run(args, snapshot)


if __name__ == "__main__":
    sys.exit(main())
