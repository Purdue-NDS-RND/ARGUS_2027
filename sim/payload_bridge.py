#!/usr/bin/env python3
"""The sim's payload bridge (SIM-06): servo outputs in, payloads let go and buckets taken hold of. Runs inside the sim container.

  env -u PYTHONNOUSERSITE python3 sim/payload_bridge.py <run folder>

Started by sim/runner/fly.py beside every flight whose scenario has a
carried payload or a bucket to pick up, and interrupted when the flight
ends. It is the one place where what the flight controller does to its
servo outputs becomes something happening to a body in Gazebo:

  - a carried payload hangs from the vehicle by a Gazebo detachable joint
    (the scenario builder puts the joint in the payload's model). When a
    release output goes to its release value the bridge detaches it, once;
  - a bucket is taken hold of when the gripper output goes to its grab
    value, but only if the hook is then within the capture half-width
    (0.10 m by default) of the handle horizontally and within 0.15 m of it
    vertically. Otherwise nothing is attached and `grab_miss` is logged.
    The hook is taken to hang straight down from a level vehicle: where it
    hangs at rest (the recovery vehicle's hook swings; its swing is not in
    this check). A scenario's `grab_refusals: N` makes the first N grabs
    inside the envelope attach nothing (`grab_refused`): a latch that did
    not close, for the mission's attach check to find.

The flight controller's outputs are read from its SERVO_OUTPUT_RAW
messages on the sim's tools port; nothing is read from the mission. With
`fake_drop: true` a mission sends no command, no output moves and the
bridge does nothing: the judge scores the vehicle's position at the
mission's fake-release log line, as before.

The run folder gets payload_bridge.jsonl (every event, with sim time and
the PWM that caused it) and payload_bridge.json (what happened, at the
end). The rules are the plain class `Bridge`, with no MAVLink or Gazebo in
it; `run` feeds it.
"""

from __future__ import annotations

import json
import math
import os
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# The sim's actuator outputs. The 2026 aircraft's logs have the gripper and the sprayer disabled and no
# servo output assigned to either, so there is nothing to copy yet: these are the sim's own until the
# aircraft's are set (PLT-08, REC-9), and then they are made the same. See PROGRESS.md, Decisions.
GRIPPER_SERVO = 7  # SERVO7_FUNCTION 28 (gripper)
SPRAYER_SERVO = 8  # SERVO8_FUNCTION 22 (sprayer pump): what the 2026 drop mission commands
GRIPPER_RELEASE_PWM, GRIPPER_GRAB_PWM, GRIPPER_NEUTRAL_PWM = 1100, 1900, 1500
SITL_PARAMS = {
    "GRIP_ENABLE": 1,
    "GRIP_TYPE": 1,  # a servo gripper
    "GRIP_RELEASE": GRIPPER_RELEASE_PWM,
    "GRIP_GRAB": GRIPPER_GRAB_PWM,
    "GRIP_NEUTRAL": GRIPPER_NEUTRAL_PWM,
    f"SERVO{GRIPPER_SERVO}_FUNCTION": 28,
    "SPRAY_ENABLE": 1,
    "SPRAY_SPEED_MIN": 0,  # cm/s: the drop is made from a hover, where a crop sprayer would not spray
    "SPRAY_PUMP_MIN": 100,  # percent: the pump is fully on whenever the sprayer is
    f"SERVO{SPRAYER_SERVO}_FUNCTION": 22,
}
PWM_MARGIN = 150  # an output this close to a set value has reached it
SPRAYER_ON_PWM = 1500  # the pump's output above this: the sprayer is on
CAPTURE_HALF_WIDTH_M = 0.10
CAPTURE_HALF_HEIGHT_M = 0.15
SERVO_RATE_HZ = 20.0

Vector = Tuple[float, float, float]


class Bridge:
    """The rules. Give it servo outputs and true positions; it says what to detach and attach."""

    def __init__(
        self,
        carried: Optional[str] = None,
        bucket: Optional[str] = None,
        hook_offset_m: Vector = (0.0, 0.0, -0.30),
        handle_height_m: float = 0.32,
        capture_half_width_m: float = CAPTURE_HALF_WIDTH_M,
        capture_half_height_m: float = CAPTURE_HALF_HEIGHT_M,
        refuse_grabs: int = 0,
    ) -> None:
        self.carried, self.bucket = carried, bucket
        self.refusals_left = max(0, int(refuse_grabs))  # grabs still to be refused though the hook is in the envelope (REC-8, run R6)
        self.hook_offset_m, self.handle_height_m = hook_offset_m, float(handle_height_m)
        self.capture_half_width_m, self.capture_half_height_m = float(capture_half_width_m), float(capture_half_height_m)
        self.released = False  # the carried payload has been let go
        self.holding = False  # the bucket is attached
        self.positions: Dict[str, Vector] = {}
        self.events: List[dict] = []
        self._last: Dict[int, int] = {}  # the last output seen on each servo
        self._start_actions = [("detach", bucket)] if bucket else []  # a bucket's joint starts made: it is unmade at once

    # --- in ----------------------------------------------------------------------------

    def on_position(self, name: str, position: Vector) -> None:
        self.positions[name] = (float(position[0]), float(position[1]), float(position[2]))

    def start(self) -> List[Tuple[str, str]]:
        """What to do before anything else: a bucket's joint is made by Gazebo when the world loads, so it is unmade."""
        actions, self._start_actions = self._start_actions, []
        return actions

    def on_servos(self, t_s: float, pwm: Dict[int, int]) -> List[Tuple[str, str]]:
        """One SERVO_OUTPUT_RAW: {servo number: PWM}. Returns [("detach" or "attach", model name)] to carry out."""
        actions: List[Tuple[str, str]] = []
        gripper, sprayer = pwm.get(GRIPPER_SERVO), pwm.get(SPRAYER_SERVO)
        was_gripper, was_sprayer = self._last.get(GRIPPER_SERVO), self._last.get(SPRAYER_SERVO)
        if gripper:
            release = abs(gripper - GRIPPER_RELEASE_PWM) <= PWM_MARGIN
            grab = abs(gripper - GRIPPER_GRAB_PWM) <= PWM_MARGIN
            was_release = was_gripper is not None and abs(was_gripper - GRIPPER_RELEASE_PWM) <= PWM_MARGIN
            was_grab = was_gripper is not None and abs(was_gripper - GRIPPER_GRAB_PWM) <= PWM_MARGIN
            # An output is acted on when it arrives at a value, not for as long as it stays there. The first
            # output seen is where the servo already was: nothing has been commanded yet.
            if was_gripper is not None and release and not was_release:
                actions += self._release(t_s, "gripper", gripper)
            if was_gripper is not None and grab and not was_grab:
                actions += self._grab(t_s, gripper)
            self._last[GRIPPER_SERVO] = gripper
        if sprayer:
            on = sprayer > SPRAYER_ON_PWM
            was_on = was_sprayer is not None and was_sprayer > SPRAYER_ON_PWM
            if was_sprayer is not None and on and not was_on:
                actions += self._release(t_s, "sprayer", sprayer)
            self._last[SPRAYER_SERVO] = sprayer
        return actions

    # --- the rules ---------------------------------------------------------------------

    def _log(self, t_s: float, event: str, **fields) -> None:
        self.events.append({"t": round(t_s, 3), "event": event, **fields})

    def _release(self, t_s: float, output: str, pwm: int) -> List[Tuple[str, str]]:
        actions: List[Tuple[str, str]] = []
        if self.carried and not self.released:
            self.released = True
            self._log(t_s, "payload_released", name=self.carried, output=output, pwm=pwm, at=self.positions.get(self.carried))
            actions.append(("detach", self.carried))
        if self.bucket and self.holding:
            self.holding = False
            self._log(t_s, "bucket_released", name=self.bucket, output=output, pwm=pwm, at=self.positions.get(self.bucket))
            actions.append(("detach", self.bucket))
        if not actions:
            self._log(t_s, "release_with_nothing_held", output=output, pwm=pwm)
        return actions

    def hook_to_handle(self, vehicle: str) -> Optional[Tuple[float, float]]:
        """(horizontal, vertical) distance from the hook to the bucket's handle, or None if either position is not known."""
        if not self.bucket or vehicle not in self.positions or self.bucket not in self.positions:
            return None
        body, bucket = self.positions[vehicle], self.positions[self.bucket]
        hook = tuple(b + o for b, o in zip(body, self.hook_offset_m))
        handle = (bucket[0], bucket[1], bucket[2] + self.handle_height_m)
        return math.hypot(hook[0] - handle[0], hook[1] - handle[1]), abs(hook[2] - handle[2])

    def _grab(self, t_s: float, pwm: int) -> List[Tuple[str, str]]:
        if not self.bucket or self.holding:
            self._log(t_s, "grab_with_nothing_to_take", pwm=pwm)
            return []
        apart = self.hook_to_handle(self.vehicle)
        if apart is None:
            self._log(t_s, "grab_miss", name=self.bucket, pwm=pwm, why="the hook's or the bucket's position is not known")
            return []
        horizontal, vertical = apart
        if horizontal > self.capture_half_width_m or vertical > self.capture_half_height_m:
            self._log(t_s, "grab_miss", name=self.bucket, pwm=pwm, horizontal_m=round(horizontal, 3), vertical_m=round(vertical, 3))
            return []
        if self.refusals_left > 0:
            # A latch that did not close: the hook was where it should be, and nothing is attached.
            self.refusals_left -= 1
            self._log(t_s, "grab_refused", name=self.bucket, pwm=pwm, horizontal_m=round(horizontal, 3), vertical_m=round(vertical, 3), refusals_left=self.refusals_left)
            return []
        self.holding = True
        self._log(t_s, "bucket_grabbed", name=self.bucket, pwm=pwm, horizontal_m=round(horizontal, 3), vertical_m=round(vertical, 3))
        return [("attach", self.bucket)]

    vehicle = "vehicle"  # the name the vehicle's position is given under; `run` sets it to the model's

    def summary(self) -> dict:
        kinds = [item["event"] for item in self.events]
        return {
            "ran": True,
            "carried": self.carried,
            "bucket": self.bucket,
            "released": self.released,
            "holding": self.holding,
            "releases": kinds.count("payload_released") + kinds.count("bucket_released"),
            "grabs": kinds.count("bucket_grabbed"),
            "grab_misses": kinds.count("grab_miss"),
            "grabs_refused": kinds.count("grab_refused"),
            "events": self.events,
        }


def settings_from(manifest: dict) -> Optional[dict]:
    """The bridge's part of a built scenario, or None if the scenario has nothing for it to do."""
    settings = manifest.get("payload_bridge")
    return settings if isinstance(settings, dict) and (settings.get("carried") or settings.get("bucket")) else None


def wanted(manifest: dict) -> bool:
    return settings_from(manifest) is not None


def joint_topic(name: str, what: str) -> str:
    """The Gazebo topic that makes ("attach") or unmakes ("detach") a body's joint to the vehicle."""
    return f"/payload/{name}/{what}"


def read_bridge(folder) -> Optional[dict]:
    """The summary a run folder holds, or None if there is none that can be read."""
    try:
        report = json.loads((Path(folder) / "payload_bridge.json").read_text())
    except (OSError, json.JSONDecodeError):
        return None
    return report if isinstance(report, dict) and report.get("ran") is True else None


def servo_outputs(message) -> Dict[int, int]:
    """{servo number: PWM} from a SERVO_OUTPUT_RAW message (servo1_raw ... servo16_raw; 0 is "no output")."""
    outputs = {}
    for number in range(1, 17):
        value = getattr(message, f"servo{number}_raw", None)
        if value:
            outputs[number] = int(value)
    return outputs


# --- MAVLink and Gazebo --------------------------------------------------------------


def run(run_dir: Path, url: str = "udpin:127.0.0.1:14552") -> dict:
    """Bridge until interrupted; write payload_bridge.jsonl as it goes and payload_bridge.json at the end."""
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node
    from pymavlink import mavutil

    manifest = json.loads((run_dir / "scenario.json").read_text())
    settings = settings_from(manifest) or {}
    world, vehicle = manifest["site"], manifest["vehicle"]["model"]
    bridge = Bridge(
        carried=settings.get("carried"),
        bucket=settings.get("bucket"),
        hook_offset_m=tuple(settings.get("hook_offset_m", (0.0, 0.0, -0.30))),
        handle_height_m=float(settings.get("handle_height_m", 0.32)),
        capture_half_width_m=float(settings.get("capture_half_width_m", CAPTURE_HALF_WIDTH_M)),
        refuse_grabs=int(settings.get("refuse_grabs", 0)),
    )
    bridge.vehicle = vehicle
    lock = threading.Lock()
    clock = {"t": 0.0}
    node = Node()
    publishers = {}
    for name in (bridge.carried, bridge.bucket):
        if name:
            for what in ("attach", "detach"):
                publishers[(what, name)] = node.advertise(joint_topic(name, what), Empty)
    log = open(run_dir / "payload_bridge.jsonl", "w")
    written = 0

    def carry_out(actions) -> None:
        nonlocal written
        for what, name in actions:
            publishers[(what, name)].publish(Empty())
        for item in bridge.events[written:]:
            log.write(json.dumps(item) + "\n")
            print(f"payload bridge: {item}", flush=True)
        log.flush()
        written = len(bridge.events)

    def on_poses(message) -> None:
        with lock:
            clock["t"] = message.header.stamp.sec + message.header.stamp.nsec * 1.0e-9
            for pose in message.pose:
                if pose.name in (vehicle, bridge.carried, bridge.bucket):
                    bridge.on_position(pose.name, (pose.position.x, pose.position.y, pose.position.z))

    if not node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", on_poses):
        raise RuntimeError(f"could not subscribe to the poses of Gazebo world '{world}'")
    node.subscribe(Pose_V, f"/world/{world}/pose/info", on_poses)  # a bucket standing still is only in here

    master = mavutil.mavlink_connection(url, source_system=246, source_component=193)
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    time.sleep(1.0)  # let the joint topics be discovered before the first message on them
    with lock:
        carry_out(bridge.start())
    asked_at = 0.0
    heard = 0
    print(f"payload bridge: carried {bridge.carried}, bucket {bridge.bucket}; gripper on servo {GRIPPER_SERVO}, sprayer on servo {SPRAYER_SERVO}", flush=True)
    while not stopping.is_set():
        if time.monotonic() - asked_at > 2.0:  # ask for the outputs, and keep asking: the stream is not on by default
            master.mav.command_long_send(
                master.target_system or 1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 1.0e6 / SERVO_RATE_HZ, 0, 0, 0, 0, 0,
            )  # fmt: skip
            asked_at = time.monotonic()
        message = master.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=0.5)
        if message is None:
            continue
        heard += 1
        with lock:
            carry_out(bridge.on_servos(clock["t"], servo_outputs(message)))
    with lock:
        report = {**bridge.summary(), "servo_messages": heard}
    log.close()
    (run_dir / "payload_bridge.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main(argv: Optional[List[str]] = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__)
        return 2
    if os.environ.get("PYTHONNOUSERSITE"):
        # pymavlink lives in the sim image's user site, which the runner's environment switches off: start again with it on.
        environment = {key: value for key, value in os.environ.items() if key != "PYTHONNOUSERSITE"}
        os.execvpe(sys.executable, [sys.executable, str(Path(__file__).resolve()), *argv], environment)
    try:
        report = run(Path(argv[0]))
    except ImportError as error:
        print(f"payload_bridge: {error} (in the sim container, run with PYTHONNOUSERSITE unset)", file=sys.stderr)
        return 2
    print(f"payload bridge: {report['servo_messages']} servo messages, {report['releases']} release(s), {report['grabs']} grab(s), {report['grab_misses']} miss(es)", flush=True)
    return 0 if report["servo_messages"] else 1


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
