"""One launch for every 2027 flight (PLT-07): a sequence, a params file, and the flags to turn on.

  ros2 launch drone_mission_demo argus_bringup.launch.py \\
      sequence:=<file> params:=<file> feature_overrides:=<mission flags> node_flags:=<node.flag,...> \\
      camera:=none|mipi|usb detector:=none|target_cv

The `go` command (tools/go.py) is what normally writes this line, from a
profile, after the preflight has said GO. With no feature_overrides and no
node_flags every flag is false and the nodes run the 2026 code.

What is started is decided by drone_mission_core/bringup.py (no ROS in it,
and tested without ROS): the take-off service and the executor always; the
2026 grabber and the detector named; a node that exists only for its flag
(ring_fit, recovery_cv, loc_node) when that flag is among node_flags. A
node's flags are passed to it in a small params file of their own, keyed by
the node's name and given after the flight's params file, which keeps them
false. The 2027 camera driver is a boot service
(PLT-02) and is not started here.

`sequence` and `params` are files: absolute paths, or paths relative to the
drone_mission_demo package as the 2026 launch files take them.

The launch takes flags, not a profile. Start flights with `go`, which reads
the profile, checks it against the sequence and runs the preflight first. Run
by hand, this launch still refuses a node flag its node does not declare, a
frame-time mission flag without the detector's stamp flag, and a drop or a
delivery with no detector.
"""

import os
import tempfile
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from drone_mission_core import bringup

ARGUMENTS = ("sequence", "params", "feature_overrides", "node_flags", "camera", "detector", "log_session", "use_sim_time")


def _file(context, name: str) -> str:
    given = LaunchConfiguration(name).perform(context)
    if Path(given).is_absolute():
        return given
    return str(Path(FindPackageShare("drone_mission_demo").perform(context)) / given)


def _nodes(context, *_args, **_kwargs):
    params, sequence = _file(context, "params"), _file(context, "sequence")
    text = {name: LaunchConfiguration(name).perform(context) for name in ARGUMENTS}
    node_flags = bringup.parse_node_flags(text["node_flags"])
    started = bringup.nodes_to_start(text["camera"], text["detector"], node_flags, log_session=text["log_session"].lower() == "true")
    homeless = sorted(node for node in node_flags if node not in started)
    if homeless:
        raise RuntimeError(f"node_flags sets flags for {', '.join(homeless)}, which this launch does not start")
    # The same checks `go` makes, for a launch run by hand: a node ignores an override for a flag it never
    # declared, and the launch line would say "on" for a flag that is off.
    problems = bringup.undeclared_node_flags(node_flags, yaml.safe_load(Path(params).read_text(encoding="utf-8")) or {})
    problems += bringup.flight_problems(yaml.safe_load(Path(sequence).read_text(encoding="utf-8")) or {}, text["feature_overrides"], node_flags, started, text["detector"])
    if problems:
        raise RuntimeError("; ".join(problems))
    actions = []
    for name in started:
        package, executable = bringup.NODES[name]
        parameters = [params]
        flags = bringup.feature_parameters(name, node_flags)
        if flags:
            # A file keyed by the node's name, after the params file: see bringup.feature_parameters for why not a dictionary.
            handle, flags_file = tempfile.mkstemp(prefix=f"argus_{name}_flags_", suffix=".yaml")
            with os.fdopen(handle, "w") as out:
                yaml.safe_dump(flags, out)
            parameters.append(flags_file)
        if name == "mission_executor":
            parameters.append(
                {
                    "sequence_file": sequence,
                    "feature_overrides": ParameterValue(text["feature_overrides"], value_type=str),
                    "mission_modules": ["drone_mission_demo.missions"],
                }
            )
        actions.append(Node(package=package, executable=executable, name=name, output="screen", parameters=parameters))
    return actions


def generate_launch_description() -> LaunchDescription:
    # The same shape as the 2026 launch files (sim/test/test_time_base.py holds every executor launch to it):
    # use_sim_time declared false, one SetParameter on it, and that before anything that starts a node.
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Run every node of this launch on /clock (the sim started with SIM_TIME=1)",
            ),
            DeclareLaunchArgument(
                "sequence",
                default_value="config/sequences/package_delivery_live.yaml",
                description="Mission sequence YAML: absolute, or relative to the drone_mission_demo package",
            ),
            DeclareLaunchArgument(
                "params",
                default_value="config/params/live_target_mission.yaml",
                description="ROS params YAML: absolute, or relative to the drone_mission_demo package",
            ),
            DeclareLaunchArgument(
                "feature_overrides",
                default_value="",
                description="Mission flags to turn on, comma-separated (profiles/README.md)",
            ),
            DeclareLaunchArgument(
                "node_flags",
                default_value="",
                description="Node flags to turn on, as node.flag items, comma-separated",
            ),
            DeclareLaunchArgument(
                "camera",
                default_value="none",
                description="The 2026 grabber this launch starts: none, mipi or usb",
            ),
            DeclareLaunchArgument(
                "detector",
                default_value="none",
                description="The detector this launch starts: none or target_cv",
            ),
            DeclareLaunchArgument(
                "log_session",
                default_value="true",
                description="Start the mission session logger",
            ),
            SetParameter(
                name="use_sim_time",
                value=True,
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
