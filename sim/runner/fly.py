#!/usr/bin/env python3
"""Fly one built scenario in the running sim and record it. Runs inside the sim container.

  sim/run_sim.sh --exec python3 sim/runner/fly.py <run folder>

Started by sim/run_scenarios.sh once the sim is READY. It

  1. refuses a scenario the sim cannot fly faithfully yet (wind, GPS events),
  2. records true poses and sim time from Gazebo, and arm state, mission
     releases and autopilot warnings from ROS, into <run folder>/trace.jsonl,
  3. launches the mission the scenario names (output in launch.log),
  4. stops when the vehicle has disarmed on the ground, or at the timeout,
     both in sim time, and writes flight.json.

This file is the thin ROS and Gazebo wrapper; the decisions are in events.py,
trace.py and watch.py.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))

from runner import events  # noqa: E402
from runner.helpers import ListCapture, SideTools, stop_group  # noqa: E402
from runner.trace import TraceWriter, yaw_of_quaternion  # noqa: E402
from runner.watch import DONE, EndWatch  # noqa: E402

# Tools a scenario may need, started with the run folder as their argument
# and stopped when the flight ends. Wind and GPS events arrive with SIM-03
# and SIM-04; the time check and the rate check run beside every flight on
# sim time (FND-03, PLT-06).
TOOLS = {
    "wind": SIM_DIR / "wind.py",
    "gps_events": SIM_DIR / "inject.py",
    "time_check": SIM_DIR / "time_check.py",
    "rate_check": SIM_DIR / "rate_check.py",
    "projection_check": SIM_DIR / "projection_check.py",  # beside flights with use_frame_time_pose on (FND-05)
    "estimator_check": SIM_DIR / "estimator_check.py",  # beside flights with use_target_estimator on (FND-06)
    "height_check": SIM_DIR / "height_check.py",  # beside flights with use_height_estimator on (CEN-03)
    "record": SIM_DIR / "replay" / "record_run.py",  # beside flights started with --record (SIM-09)
    "payload_bridge": SIM_DIR / "payload_bridge.py",  # beside flights with a carried payload or a bucket (SIM-06)
    "sensor_check": SIM_DIR / "sensor_check.py",  # beside flights whose scenario asks for it (SIM-05)
    "truth_poses": SIM_DIR / "truth_poses.py",  # beside flights whose scenario asks for true poses on a topic (SIM-05)
    "loc_check": SIM_DIR / "loc_check.py",  # beside flights with the localisation node on (LOC-5)
    "recovery_cv_check": SIM_DIR / "recovery_cv_check.py",  # beside flights with the green X and bucket detector on (REC-1)
}
CLOCK_MARK_S = 5.0
STATIC_POSE_S = 1.0  # how often objects that are not moving are recorded


class Recorder:
    """Gazebo's true poses and clock, written to the trace on sim time."""

    def __init__(self, world: str, vehicle_model: str, tracked: list[str], writer: TraceWriter, masses_kg: dict | None = None) -> None:
        from gz.msgs10.clock_pb2 import Clock
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node

        self.writer = writer
        self.lock = threading.Lock()
        self.names = {vehicle_model: "vehicle", **{name: name for name in tracked}}
        self.landing = events.LandingWatcher(masses_kg or {})  # the impulse a released payload or bucket lands with
        self.sim_now: float | None = None
        self.vehicle_up: float | None = None  # latest true height of the vehicle model
        self.vehicle_start_up: float | None = None  # and its first, standing on the ground
        self._last_mark = -1.0e9
        self._last_static = -1.0e9
        self._node = Node()
        ok = self._node.subscribe(Clock, f"/world/{world}/clock", self._on_clock)
        ok = self._node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", self._on_dynamic) and ok
        ok = self._node.subscribe(Pose_V, f"/world/{world}/pose/info", self._on_all) and ok
        if not ok:
            raise RuntimeError(f"could not subscribe to Gazebo world '{world}'")

    def _on_clock(self, message) -> None:
        now = message.sim.sec + message.sim.nsec * 1.0e-9
        with self.lock:
            self.sim_now = now
            if now - self._last_mark >= CLOCK_MARK_S:
                self._last_mark = now
                self.writer.clock(now, time.time())

    def _write_poses(self, message) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nsec * 1.0e-9
        with self.lock:
            when = stamp if stamp > 0.0 else self.sim_now
            if when is None:
                return
            for pose in message.pose:
                name = self.names.get(pose.name)
                if name is not None:
                    # The vehicle's true heading goes with its pose: a judge that scores a point off the vehicle's centre needs it.
                    yaw = yaw_of_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) if name == "vehicle" else None
                    self.writer.pose(when, name, pose.position.x, pose.position.y, pose.position.z, yaw)
                    contact = self.landing.update(name, when, (pose.position.x, pose.position.y, pose.position.z))
                    if contact is not None:
                        self.writer.event(when, contact.pop("event"), **contact)
                    if name == "vehicle":
                        self.vehicle_up = pose.position.z
                        if self.vehicle_start_up is None:
                            self.vehicle_start_up = pose.position.z

    def _on_dynamic(self, message) -> None:
        self._write_poses(message)

    def _on_all(self, message) -> None:
        # Everything, moving or not, but only now and then: a bucket standing
        # on its mat is not in the dynamic poses until something moves it.
        now = self.sim_now
        if now is None or now - self._last_static < STATIC_POSE_S:
            return
        self._last_static = now
        self._write_poses(message)

    def event(self, kind: str, **fields) -> None:
        with self.lock:
            if self.sim_now is not None:
                self.writer.event(self.sim_now, kind, **fields)

    def now(self) -> float:
        return self.sim_now if self.sim_now is not None else 0.0

    def height(self) -> float | None:
        """The vehicle's true height above where it stood when the recording began."""
        if self.vehicle_up is None or self.vehicle_start_up is None:
            return None
        return self.vehicle_up - self.vehicle_start_up


def fly(run_dir: Path) -> dict:
    import rclpy
    from mavros_msgs.msg import ExtendedState, State, StatusText
    from rcl_interfaces.msg import Log
    from rclpy.qos import qos_profile_sensor_data

    manifest = json.loads((run_dir / "scenario.json").read_text())
    truth = json.loads((run_dir / "truth.json").read_text())
    reason = events.unsupported(manifest, {name for name, path in TOOLS.items() if path.is_file()})
    if reason:
        return {"outcome": "INVALID", "reason": f"the scenario was not flown: {reason}"}

    tracked = [item["name"] for item in truth.get("objects", []) if item["kind"] in ("bucket", "payload")]
    writer = TraceWriter(run_dir / "trace.jsonl")
    masses = {item["name"]: float(item.get("mass_kg", 1.5 if item["kind"] == "bucket" else 0.5)) for item in truth.get("objects", []) if item["name"] in tracked}
    released_body = (truth.get("payload") or truth.get("bucket") or {}).get("name")  # None: every tracked body
    recorder = Recorder(manifest["site"], manifest["vehicle"]["model"], tracked, writer, masses)
    deadline = time.monotonic() + 30.0
    while recorder.sim_now is None:
        if time.monotonic() > deadline:
            return {"outcome": "ERROR", "reason": "no clock from Gazebo within 30 s"}
        time.sleep(0.1)

    rclpy.init()
    overrides = []
    if manifest.get("sim_time") is True:
        # The recorder takes its time from Gazebo directly; this only keeps
        # "every node is on sim time" true of the whole graph.
        from rclpy.parameter import Parameter

        overrides = [Parameter("use_sim_time", value=True)]
    node = rclpy.create_node("argus_scenario_recorder", parameter_overrides=overrides)
    watch = EndWatch(recorder.now, timeout_s=float(manifest["timeout_s"]), height=recorder.height)
    arm = events.ArmWatcher()
    modes = events.ModeWatcher()

    def on_state(message) -> None:
        mode = modes.update(message.connected, message.mode)
        if mode:
            recorder.event("mode", mode=mode)
        change = arm.update(message.connected, message.armed)
        if change:
            recorder.event(change, mode=message.mode)
        if message.connected:
            watch.update(armed=message.armed)

    release_times: list[float] = []  # when the mission's log said it let its payload go

    def on_log(message) -> None:
        if message.name == "mission_executor":
            for item in events.from_mission_log(message.msg):
                if item["event"] == "release" and not item.get("fake"):
                    # A real release: the landing of the mission's payload (or, with none, its bucket) is watched from here.
                    recorder.landing.released(recorder.now(), name=released_body)
                    release_times.append(recorder.now())
                recorder.event(item.pop("event"), **item)

    def on_statustext(message) -> None:
        for item in events.from_statustext(message.text):
            recorder.event(item.pop("event"), **item)

    node.create_subscription(State, "/mavros/state", on_state, 10)
    node.create_subscription(ExtendedState, "/mavros/extended_state", lambda m: watch.update(landed_state=m.landed_state), 10)
    node.create_subscription(Log, "/rosout", on_log, 200)
    # MAVROS publishes the autopilot's text with sensor-data QoS: a default
    # (reliable) subscription is refused and silently gets nothing.
    node.create_subscription(StatusText, "/mavros/statustext/recv", on_statustext, qos_profile_sensor_data)

    judges_list = ListCapture(Path(manifest["list_file"]) if manifest.get("list_file") else None, run_dir)
    tools = SideTools(TOOLS, run_dir)
    tools.start(events.tools_needed(manifest))

    if manifest.get("driver"):
        command = list(manifest["driver"]["command"])  # a tool flies the vehicle itself
    else:
        launch = manifest["launch"]
        command = ["ros2", "launch", launch["package"], launch["file"], *[f"{key}:={value}" for key, value in launch["arguments"].items()]]
    (run_dir / "command.txt").write_text(" ".join(command) + "\n")
    t_launch = recorder.now()
    wall_start = time.time()
    recorder.event("launch", what=" ".join(command[:4]))
    with open(run_dir / "launch.log", "w") as launch_log:
        mission = subprocess.Popen(command, stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
        outcome = None
        try:
            while outcome is None:
                rclpy.spin_once(node, timeout_sec=0.05)
                outcome = watch.check()
                if judges_list.poll(flying=watch.tracker.armed):
                    recorder.event("list_written")
        except KeyboardInterrupt:
            outcome = "INTERRUPTED"
        finally:
            launch_exit = stop_group(mission)
            tools.stop()
    # A bucket the payload bridge let go that no mission log line told of is a release all the same: the judge
    # holds it to the rules' height (a scripted flight has no mission log; a mission's line may not be a known one).
    bridge_log = run_dir / "payload_bridge.jsonl"
    if bridge_log.is_file():
        for t, item in events.from_bridge_log(bridge_log.read_text().splitlines(), release_times):
            writer.event(t, item.pop("event"), **item)
    t_end = recorder.now()
    recorder.event("end", outcome=outcome)
    writer.close()
    node.destroy_node()
    rclpy.try_shutdown()
    result = {
        "outcome": outcome,
        "reason": "" if outcome == DONE else f"the flight ended as {outcome}",
        "t_launch": t_launch,
        "t_end": t_end,
        "wall_s": time.time() - wall_start,
        "launch_exit_code": launch_exit,
    }
    return result


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__.strip().splitlines()[2].strip(), file=sys.stderr)
        return 2
    run_dir = Path(argv[1]).resolve()
    try:
        result = fly(run_dir)
    except Exception as error:  # noqa: BLE001  (whatever went wrong, the run must end with a flight.json saying so)
        result = {"outcome": "ERROR", "reason": f"{type(error).__name__}: {error}"}
    (run_dir / "flight.json").write_text(json.dumps(result, indent=2) + "\n")
    print(f"flight: {result['outcome']}" + (f" ({result['reason']})" if result.get("reason") else ""), flush=True)
    return 0 if result["outcome"] == DONE else 1


if __name__ == "__main__":
    code = main(sys.argv)
    # Leave without tearing the interpreter down: Gazebo's subscriber threads
    # are still delivering messages, and one arriving during teardown aborts
    # the process ("thread state must be current"). Everything is on disk.
    sys.stderr.flush()
    os._exit(code)
