"""Replay a bag through the real nodes (SIM-09, the `ros` mode). For the bench, never beside a live vehicle.

  ros2 launch sim/replay/play.launch.py bag:=/path/to/bag
  ros2 launch sim/replay/play.launch.py bag:=bag rate:=0.5 keep_outputs:=true

Plays the bag with its original stamps and publishes /clock from the bag's
own times, so nodes started with use_sim_time:=true live the flight again:
start the nodes under test (a detector, the estimator's executor) beside
this launch.

What the flight's own nodes put out (detections, the estimator's tracks,
setpoints) is replayed under /recorded/..., so that it does not collide
with what the nodes under test publish now and the two can be compared.
`keep_outputs:=true` replays the detections and tracks under their own
names instead, for looking at a flight as it was. Setpoints are replayed
under /recorded/... whatever is asked: a recorded setpoint is never put on
a topic MAVROS listens to.

The bag's poses, state and GPS do come out under MAVROS's own names: that
is what a replay is. So the launch first looks for a live MAVROS in this
ROS domain (a publisher on /mavros/state) and does not start if there is
one, or if it cannot tell, unless `beside_live_mavros:=true` says the
operator knows. It looks once, before it starts: the rule is still that a
bag is never played on the aircraft's network.
"""

import subprocess
import sys
from pathlib import Path
from typing import Optional

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

REPLAY_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(REPLAY_DIR.parent))

from replay import topics  # noqa: E402

# What the recorded flight's own nodes published: outputs of the code a replay puts under test.
# Every recorded topic MAVROS would take as a command: found by name, so a setpoint topic added to the recorder's
# list later is moved aside without anyone having to remember this file.
SETPOINTS = [topic for topic in topics.record_topics() if topic.startswith("/mavros/") and topic.split("/")[2].startswith("setpoint")]
OUTPUTS = [topics.LEGACY_DETECTION, topics.LEGACY_IMAGE_SIZE, topics.ESTIMATOR_TARGETS] + [topics.contract_topic(name) for name in topics.DETECTORS]
LIVE_MAVROS_TOPIC = topics.STATE


def play_command(bag: str, rate: str = "1.0", keep_outputs: bool = False) -> list:
    """The `ros2 bag play` command: original stamps, /clock from the bag, the flight's outputs under /recorded."""
    # A bag of a sim-time run holds the /clock of the flight. The player publishes its own from the bag's times,
    # so the recorded one is left out: every recorded topic is named but that one.
    played = [topic for topic in topics.record_topics() if topic != topics.CLOCK]
    moved = SETPOINTS + ([] if keep_outputs else OUTPUTS)  # recorded setpoints never go out under MAVROS's names
    return ["ros2", "bag", "play", str(bag), "--clock", "--rate", str(rate), "--topics"] + played + ["--remap"] + [f"{topic}:=/recorded{topic}" for topic in moved]


def publisher_count(topic_info: str) -> Optional[int]:
    """How many publishers `ros2 topic info` reports: the number after "Publisher count:", 0 for "Unknown topic", None if it says neither."""
    for line in topic_info.splitlines():
        name, _, value = line.partition(":")
        if name.strip() == "Publisher count" and value.strip().isdigit():
            return int(value.strip())
    return 0 if "Unknown topic" in topic_info else None


def live_mavros() -> Optional[bool]:
    """Whether something in this ROS domain is publishing MAVROS's state now. None when that could not be found out."""
    try:
        done = subprocess.run(["ros2", "topic", "info", LIVE_MAVROS_TOPIC], capture_output=True, text=True, timeout=20)
    except (OSError, subprocess.SubprocessError):
        return None
    count = publisher_count((done.stdout or "") + (done.stderr or ""))
    return None if count is None else count > 0


def _true(text: str) -> bool:
    return str(text).strip().lower() in ("true", "1", "on", "yes")


def processes(context):
    if not _true(LaunchConfiguration("beside_live_mavros").perform(context)):
        live = live_mavros()
        if live is None:  # not knowing is not "no": the replay does not start on a guess
            raise RuntimeError(
                f"could not find out whether a MAVROS is live in this ROS domain (`ros2 topic info {LIVE_MAVROS_TOPIC}` gave no answer). "
                "A replay beside a live vehicle would mix a recorded flight's poses and state into it. (beside_live_mavros:=true overrides this.)"
            )
        if live:
            raise RuntimeError(
                f"something is publishing {LIVE_MAVROS_TOPIC} in this ROS domain: a replay would mix a recorded flight's poses and state "
                "into a live one. Stop it, or use another ROS_DOMAIN_ID. (beside_live_mavros:=true overrides this.)"
            )
    command = play_command(LaunchConfiguration("bag").perform(context), LaunchConfiguration("rate").perform(context), _true(LaunchConfiguration("keep_outputs").perform(context)))
    return [ExecuteProcess(cmd=command, output="screen")]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag", description="the bag folder to play"),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument("keep_outputs", default_value="false"),
            DeclareLaunchArgument("beside_live_mavros", default_value="false", description="true: play even though a MAVROS is live in this ROS domain"),
            OpaqueFunction(function=processes),
        ]
    )
