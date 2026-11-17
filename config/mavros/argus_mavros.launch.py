"""MAVROS for ARGUS: MAVROS's ArduPilot defaults with config/mavros/argus_mavros.yaml on top.

  ros2 launch config/mavros/argus_mavros.launch.py fcu_url:=udp://:14551@ use_sim_time:=true

The 2026 start-up (mavros_boot.sh, MAVROS's own apm.launch) is unchanged and
does not use this file. The sim uses it when it runs on sim time (SIM_TIME=1).
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ARGUS_CONFIG = Path(__file__).resolve().parent / "argus_mavros.yaml"


def generate_launch_description() -> LaunchDescription:
    mavros_launch_dir = PathJoinSubstitution([FindPackageShare("mavros"), "launch"])
    # No node name: the parameters go to every node in the MAVROS process
    # (router, UAS and each plugin), as they do in MAVROS's own node.launch.
    mavros = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([mavros_launch_dir, "apm_pluginlists.yaml"]),
            PathJoinSubstitution([mavros_launch_dir, "apm_config.yaml"]),
            LaunchConfiguration("config"),
            {
                "fcu_url": LaunchConfiguration("fcu_url"),
                "gcs_url": "",
                "tgt_system": 1,
                "tgt_component": 1,
                "fcu_protocol": "v2.0",
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("fcu_url", description="MAVLink link to the flight controller, e.g. udp://:14551@ or /dev/ttyACM0:115200"),
            DeclareLaunchArgument("config", default_value=str(ARGUS_CONFIG), description="ARGUS MAVROS settings, loaded after apm_config.yaml"),
            DeclareLaunchArgument("use_sim_time", default_value="false", description="Take time from /clock (the sim started with SIM_TIME=1)"),
            mavros,
        ]
    )
