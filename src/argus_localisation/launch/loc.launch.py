from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description() -> LaunchDescription:
    loc_node = Node(
        package="argus_localisation",
        executable="loc_node",
        name="loc_node",
        output="screen",
        parameters=[LaunchConfiguration("params")],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false", description="Run the node on /clock (the sim started with SIM_TIME=1)"),
            SetParameter(name="use_sim_time", value=True, condition=IfCondition(LaunchConfiguration("use_sim_time"))),
            DeclareLaunchArgument("params", description="ROS params YAML with a loc_node section (config/loc_params.yaml is the shipped one)"),
            loc_node,
        ]
    )
