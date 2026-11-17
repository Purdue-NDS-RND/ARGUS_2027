"""Start the argus_camera node in one sensor mode.

  ros2 launch argus_camera argus_camera.launch.py mode:=imx519_720p60
  ros2 launch argus_camera argus_camera.launch.py mode:=sim_delivery input_topic:=/camera/image legacy_topics:=false use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    arguments = {
        "mode": ("imx519_720p60", "Sensor mode from config/modes.yaml"),
        "synthetic": ("false", "Publish a test pattern in the mode's size instead of opening a camera"),
        "legacy_topics": ("true", "Also publish on /camera/image and /camera/camera_info, the 2026 names"),
        "input_topic": ("/sim/camera/image", "Where a ros_topic mode (the sim) takes its frames from"),
        "use_sim_time": ("false", "Take time from /clock (the sim started with SIM_TIME=1)"),
    }
    node = Node(
        package="argus_camera",
        executable="camera_node",
        name="argus_camera",
        output="screen",
        parameters=[
            {
                "mode": LaunchConfiguration("mode"),
                "synthetic": ParameterValue(LaunchConfiguration("synthetic"), value_type=bool),
                "legacy_topics": ParameterValue(LaunchConfiguration("legacy_topics"), value_type=bool),
                "input_topic": LaunchConfiguration("input_topic"),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            }
        ],
    )
    declared = [DeclareLaunchArgument(name, default_value=default, description=text) for name, (default, text) in arguments.items()]
    return LaunchDescription([*declared, node])
