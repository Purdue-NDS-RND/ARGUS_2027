"""What the bringup launch starts, and with which flags (PLT-07). No ROS imports.

One launch file, `argus_bringup.launch.py`, starts a flight from three
things: a mission sequence, a params file, and a profile. A profile
(`profiles/<name>.yaml`) is the list of flags to turn on:

    description: the delivery with the target estimator
    enable:                      # mission flags: the executor's feature_overrides
      - use_frame_time_pose
      - use_target_estimator
    nodes:                       # node flags: features.<flag> parameters, node by node
      target_cv: [stamp_from_image, detection_contract]
    start:                       # what the launch starts besides the take-off service and the executor
      camera: none               # none, mipi or usb: the 2026 grabber to start
      detector: target_cv        # none or target_cv

`profiles/comp_2026.yaml` enables nothing: the launch then starts what it
is told to with every flag false, which is the 2026 code.

The 2027 camera driver is not started here: it is a boot
service (PLT-02), and nothing but that service starts it (PLT-03).

This module turns a profile into what the launch needs, and checks it
against the files it will be flown with: every node flag must be declared
(as `false`) in that node's block of the params file, as the rule for node
flags asks, and a node that exists only for its flag (`ring_fit`,
`recovery_cv`, `loc_node`, `flow_odometry`) is started only when that flag is on.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Tuple

CAMERAS = ("none", "mipi", "usb")
DETECTORS = ("none", "target_cv")
# Nodes the launch starts only when the node's flag of the same name is on (the scenario builder's FLAG_STARTED_NODES).
FLAG_STARTED_NODES = ("ring_fit", "recovery_cv", "loc_node", "flow_odometry")
# package, executable of every node the launch can start
NODES = {
    "simple_takeoff_service": ("drone_utils", "simple_takeoff_service"),
    "mission_executor": ("drone_mission_core", "mission_executor"),
    "target_cv": ("drone_target_cv", "target_cv"),
    "mipi_grabber": ("drone_target_cv", "mipi_grabber"),
    "usb_grabber": ("drone_target_cv", "usb_grabber"),
    "ring_fit": ("drone_target_cv", "ring_fit"),
    "recovery_cv": ("drone_target_cv", "recovery_cv"),
    "flow_odometry": ("drone_target_cv", "flow_odometry"),
    "loc_node": ("argus_localisation", "loc_node"),
    "session_logger": ("drone_utils", "session_logger"),
}
# Mission flags that read a detection's stamp as its frame's capture time (the frame-time pose, the time guard's
# frame-age limit), and the node flag without which that node stamps with its own clock at publish time. A flight
# that starts the node with the mission flag on and the node flag off would fly neither 2026 nor 2027 timing.
NODE_FLAG_NEEDED = {"target_cv": {"stamp_from_image": ("use_frame_time_pose", "time_health_guard")}}
# Mission types that fly on the bullseye detector (missions built on RedBullseyeMissionBase): a sequence with one
# needs the detector started.
DETECTOR_MISSIONS = ("package_drop", "package_delivery")
_NAME = re.compile(r"[a-z][a-z0-9_]*")


class BringupError(ValueError):
    """A profile that cannot be flown as written. The message says why."""


@dataclass(frozen=True)
class Bringup:
    feature_overrides: str  # the executor's launch argument: the mission flags, comma-separated
    node_flags: Dict[str, Tuple[str, ...]]  # node -> the features.<flag> parameters to set true
    camera: str = "none"
    detector: str = "none"
    nodes: Tuple[str, ...] = field(default_factory=tuple)  # every node the launch starts, in order

    def node_flags_argument(self) -> str:
        """The launch argument form: "target_cv.stamp_from_image,ring_fit.ring_fit"."""
        return ",".join(f"{node}.{flag}" for node in sorted(self.node_flags) for flag in self.node_flags[node])


def parse_node_flags(text: Optional[str]) -> Dict[str, Tuple[str, ...]]:
    """ "target_cv.stamp_from_image,ring_fit.ring_fit" -> {"target_cv": ("stamp_from_image",), "ring_fit": ("ring_fit",)}."""
    flags: Dict[str, List[str]] = {}
    for item in (text or "").split(","):
        item = item.strip()
        if not item:
            continue
        node, sep, flag = item.partition(".")
        if not sep or not _NAME.fullmatch(node) or not _NAME.fullmatch(flag):
            raise BringupError(f"node_flags takes node.flag items, comma-separated; got '{item}'")
        if flag in flags.setdefault(node, []):
            raise BringupError(f"node flag {node}.{flag} is given twice")
        flags[node].append(flag)
    return {node: tuple(names) for node, names in flags.items()}


def nodes_to_start(camera: str, detector: str, node_flags: Mapping[str, Any], log_session: bool = True) -> Tuple[str, ...]:
    """The nodes of a flight, in the order the launch lists them."""
    if camera not in CAMERAS:
        raise BringupError(f"camera is one of {', '.join(CAMERAS)}; got '{camera}'")
    if detector not in DETECTORS:
        raise BringupError(f"detector is one of {', '.join(DETECTORS)}; got '{detector}'")
    nodes = []
    if camera in ("mipi", "usb"):
        nodes.append(f"{camera}_grabber")
    nodes.append("simple_takeoff_service")
    if detector == "target_cv":
        nodes.append("target_cv")
    nodes.append("mission_executor")
    for node in FLAG_STARTED_NODES:
        if node in (node_flags.get(node) or ()):
            nodes.append(node)
    if log_session:
        nodes.append("session_logger")
    return tuple(nodes)


def feature_parameters(node: str, node_flags: Mapping[str, Any]) -> Dict[str, Any]:
    """A params file's content that turns a node's flags on, keyed by the node's name. Empty for a node with none.

    Keyed by the node's name, in a file of its own given after the flight's params file, and not as a plain
    parameter dictionary on the launch's Node: launch_ros writes such a dictionary under the wildcard `/**`, and
    a wildcard entry loses to the params file's entry for the node by name whenever any global parameter rule
    comes first on the node's command line, as `use_sim_time` does in the sim. The flag then stays false while
    the launch line says it is on (seen in the sim on ROS 2 Humble, 23 November 2026: the first flags-on flight
    through `go` flew with the detector's flags off). A later file with the node's own name wins either way.
    """
    flags = node_flags.get(node) or ()
    return {node: {"ros__parameters": {"features": {flag: True for flag in flags}}}} if flags else {}


def undeclared_node_flags(node_flags: Mapping[str, Any], params: Mapping[str, Any]) -> List[str]:
    """Node flags a params file does not declare: a flag must be there, false, under its node's ros__parameters.features."""
    problems = []
    for node in sorted(node_flags):
        declared = (((params.get(node) or {}).get("ros__parameters") or {}).get("features")) or {}
        for flag in node_flags[node]:
            if flag not in declared:
                problems.append(f"{node}.{flag} is not declared under {node}: ros__parameters: features: in the params file")
            elif declared[flag] is not False:
                problems.append(f"{node}.{flag} is {declared[flag]!r} in the params file: a shipped params file keeps every flag false")
    return problems


def missing_node_flags(mission_flags, node_flags: Mapping[str, Any], started) -> List[str]:
    """Mission flags that are on while a started node's flag they depend on is off."""
    problems = []
    for node, needs in NODE_FLAG_NEEDED.items():
        if node not in started:
            continue
        for node_flag, readers in needs.items():
            on = [flag for flag in readers if flag in mission_flags]
            if on and node_flag not in (node_flags.get(node) or ()):
                problems.append(f"{', '.join(on)} needs the node flag {node}.{node_flag} (nodes: {{{node}: [{node_flag}]}}): without it {node} stamps with its own clock, not the frame's")
    return problems


def missions_without_their_detector(sequence: Mapping[str, Any], detector: str) -> List[str]:
    """The sequence's missions that fly on the bullseye detector when the flight starts none."""
    if detector != "none":
        return []
    root = sequence.get("mission_sequence", sequence) or {}
    # A type is read as the registry reads it: stripped and in lower case.
    types = {str(item.get("type")).strip().lower() for item in root.get("missions") or [] if isinstance(item, dict)}
    return sorted(types & set(DETECTOR_MISSIONS))


def flight_problems(sequence: Mapping[str, Any], feature_overrides: Optional[str], node_flags: Mapping[str, Any], started, detector: str) -> List[str]:
    """What stops a flight from being started as asked, from the flags that will be on and the nodes that will run.

    The flags are the ones the executor will fly: the sequence's own with `feature_overrides` applied, read by
    the executor's own parser (a flag the sequence file itself has true counts). Used by `go` and by the
    launch, so a launch run by hand is held to the same rules. Raises FeatureFlagError for overrides the
    executor would refuse.
    """
    from drone_mission_core.features import parse_feature_overrides, resolve_features

    root = sequence.get("mission_sequence", sequence) or {}
    flown = resolve_features(dict(root.get("features") or {}), parse_feature_overrides(feature_overrides))
    problems = missing_node_flags([flag for flag, value in flown.items() if value is True], node_flags, started)
    blind = missions_without_their_detector(sequence, detector)
    if blind:
        problems.append(f"the sequence flies {', '.join(blind)}, which needs the detector, and this flight starts none (detector: target_cv)")
    return problems


def load_bringup(profile_path, params: Optional[Mapping[str, Any]] = None, log_session: bool = True, camera: Optional[str] = None, detector: Optional[str] = None) -> Bringup:
    """A profile as the launch needs it. Raises BringupError (or the flag plumbing's FeatureFlagError) for one that cannot be flown.

    `params` is the params file's content, when the node flags are to be checked against it. `camera` and
    `detector`, when given, take the place of the profile's `start:` values: what a flight needs started depends
    on its mission too (a waypoint flight needs no detector), and the 2026 profile names neither.
    """
    import yaml
    from drone_mission_core.features import load_profile

    enabled = load_profile(profile_path)  # checks the keys, the enable-list and the flags' dependencies
    data = yaml.safe_load(Path(profile_path).read_text(encoding="utf-8")) or {}
    nodes = data.get("nodes") or {}
    if not isinstance(nodes, dict):
        raise BringupError(f"{profile_path}: 'nodes' maps a node's name to the list of its flags to turn on")
    node_flags: Dict[str, Tuple[str, ...]] = {}
    for node, flags in nodes.items():
        if not isinstance(node, str) or not _NAME.fullmatch(node) or not isinstance(flags, list) or not all(isinstance(f, str) and _NAME.fullmatch(f) for f in flags):
            raise BringupError(f"{profile_path}: nodes.{node} must be a list of flag names")
        if len(set(flags)) != len(flags):
            raise BringupError(f"{profile_path}: nodes.{node} lists a flag twice")
        if flags:
            node_flags[node] = tuple(flags)
    start = data.get("start") or {}
    if not isinstance(start, dict) or set(start) - {"camera", "detector"}:
        raise BringupError(f"{profile_path}: 'start' has camera and detector")
    camera = str(start.get("camera", "none")) if camera is None else camera
    detector = str(start.get("detector", "none")) if detector is None else detector
    started = nodes_to_start(camera, detector, node_flags, log_session)
    homeless = sorted(node for node in node_flags if node not in started)
    if homeless:
        raise BringupError(f"{profile_path}: flags are set for {', '.join(homeless)}, which this flight does not start")
    problems = missing_node_flags(enabled, node_flags, started)
    if params is not None:
        problems += undeclared_node_flags(node_flags, params)
    if problems:
        raise BringupError(f"{profile_path}: " + "; ".join(problems))
    return Bringup(",".join(enabled), node_flags, camera, detector, started)
