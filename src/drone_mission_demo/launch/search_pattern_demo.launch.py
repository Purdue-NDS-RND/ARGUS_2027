from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params = PathJoinSubstitution(
        [
            FindPackageShare("drone_mission_demo"),
            LaunchConfiguration("params"),
        ]
    )
    sequence = PathJoinSubstitution(
        [
            FindPackageShare("drone_mission_demo"),
            LaunchConfiguration("sequence"),
        ]
    )

    takeoff_service_node = Node(
        package="drone_utils",
        executable="simple_takeoff_service",
        name="simple_takeoff_service",
        output="screen",
        parameters=[params],
    )

    executor_node = Node(
        package="drone_mission_core",
        executable="mission_executor",
        name="mission_executor",
        output="screen",
        parameters=[
            params,
            {
                "sequence_file": sequence,
                # Flags to turn on for this launch; empty changes nothing.
                "feature_overrides": ParameterValue(
                    LaunchConfiguration("feature_overrides"), value_type=str
                ),
                "mission_modules": ["drone_mission_demo.missions"],
            },
        ],
    )

    # The localisation node (LOC-5). Started only when asked (loc_node:=true); its features.loc_node flag must be
    # on in the params file as well, or it does nothing.
    loc_node = Node(
        package="argus_localisation",
        executable="loc_node",
        name="loc_node",
        output="screen",
        parameters=[params],
        condition=IfCondition(LaunchConfiguration("loc_node")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Run every node of this launch on /clock (the sim started with SIM_TIME=1)",
            ),
            # Adds nothing to any node unless use_sim_time is true.
            SetParameter(
                name="use_sim_time",
                value=True,
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
            DeclareLaunchArgument(
                "feature_overrides",
                default_value="",
                description="Feature flags to turn on for this launch, comma-separated (profiles/README.md)",
            ),
            DeclareLaunchArgument(
                "params",
                default_value="config/params/sim_survey_mission.yaml",
                description="ROS params YAML relative to the drone_mission_demo package",
            ),
            DeclareLaunchArgument(
                "sequence",
                default_value="config/sequences/search_pattern_demo.yaml",
                description="Mission sequence YAML relative to the drone_mission_demo package",
            ),
            DeclareLaunchArgument(
                "loc_node",
                default_value="false",
                description="Start the localisation node (its features.loc_node flag must be on as well)",
            ),
            takeoff_service_node,
            executor_node,
            loc_node,
        ]
    )
