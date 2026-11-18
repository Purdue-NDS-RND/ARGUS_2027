"""Record a flight for replay (SIM-09): in the sim and on the aircraft.

  ros2 launch sim/replay/record.launch.py out:=/path/to/bag
  ros2 launch sim/replay/record.launch.py out:=bag storage:=sqlite3 use_sim_time:=true

Records the topics listed in topics.py (MAVROS pose, velocity, IMU, GPS,
state, rangefinder and setpoints; camera info; detections; the estimator's
tracks) as a rosbag2 bag, MCAP by default. Images are not recorded.

Frame taps are off unless asked for. A tap puts the time each frame was
seen on an image topic into the bag, but it is one more subscriber to that
topic: on the aircraft nothing else listens to the raw image, so the 2026
grabber then has every raw frame to deliver before the frame the detector
reads, and the 2026 detector publishes its annotated copy before its
detection. Until the cost is
measured on the Jetson, record a flight that counts without taps, and use
them on the bench (tap_images:=/camera/image,/target_cv/annotated).

Arguments:
  out            the bag folder to write (it must not exist)
  storage        mcap (default) or sqlite3
  use_sim_time   true to record on /clock (a sim run on sim time)
  sim_clock      true to tap Gazebo's clock (a sim run on wall time)
  tap_images     image topics to tap, comma-separated (default: none)
"""

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

REPLAY_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(REPLAY_DIR.parent))

from replay import record_run, topics  # noqa: E402


def _true(text: str) -> bool:
    return str(text).strip().lower() in ("true", "1", "on", "yes")


def processes(context):
    out = Path(LaunchConfiguration("out").perform(context))
    storage = LaunchConfiguration("storage").perform(context)
    sim_time = _true(LaunchConfiguration("use_sim_time").perform(context))
    sim_clock = _true(LaunchConfiguration("sim_clock").perform(context))
    tapped = [topic.strip() for topic in LaunchConfiguration("tap_images").perform(context).split(",") if topic.strip()]
    recorder = ExecuteProcess(cmd=record_run.record_command(out, storage, sim_time, tapped), output="screen")
    if not tapped and not sim_clock:
        return [recorder]  # nothing to tap: the recorder alone, which only listens
    taps = [sys.executable, str(REPLAY_DIR / "taps.py")]
    for topic in tapped:
        taps += ["--image", topic]
    taps += (["--no-images"] if not tapped else []) + (["--use-sim-time"] if sim_time else []) + (["--sim-clock"] if sim_clock else [])
    return [recorder, ExecuteProcess(cmd=taps, output="screen")]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("out", description="the bag folder to write"),
            DeclareLaunchArgument("storage", default_value="mcap", description="mcap or sqlite3"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("sim_clock", default_value="false"),
            DeclareLaunchArgument("tap_images", default_value="", description="image topics to tap, comma-separated; none by default"),
            OpaqueFunction(function=processes),
        ]
    )
