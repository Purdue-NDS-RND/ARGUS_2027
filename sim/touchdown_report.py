#!/usr/bin/env python3
"""Report where the package delivery touched down, relative to the target.

Run it in the running sim, from a third terminal, before the drone reaches
the target (any time before TRACK_AND_DESCEND):

    sim/run_sim.sh --exec python3 sim/touchdown_report.py [--sequence <yaml>]

It watches /mavros/local_position/pose, /mavros/extended_state and the
mission log on /rosout, and prints once the drone touches down:

- the altitude where the final fixed-column descent began
- the touchdown position (local ENU: x east, y north of home)
- the horizontal distance from touchdown to the target centre
- the time spent in TRACK_AND_DESCEND, if the mission log was seen

The target is target_latitude/target_longitude of the first mission in the
sequence YAML that has them (default package_delivery_demo.yaml), converted
to metres from home with sim/target_placement.py, as spawn_target.py does.
That assumes the local position origin is SITL home and home is the world
origin; sim/container/start.sh starts SITL there and warns if it isn't.

Everything above main() is plain Python with no ROS, and is unit-tested in
sim/test/test_touchdown_report.py.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping, Optional, Sequence

SIM_DIR = Path(__file__).resolve().parent
REPO = SIM_DIR.parent
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))

from target_placement import GeoOrigin, read_world, world_xy_for  # noqa: E402

DEFAULT_WORLD = SIM_DIR / "worlds" / "argus_runway.sdf"
DEFAULT_SEQUENCE = (
    REPO / "src" / "drone_mission_demo" / "config" / "sequences" / "package_delivery_demo.yaml"
)

# mavros_msgs/ExtendedState landed_state values.
LANDED_STATE_UNDEFINED = 0
LANDED_STATE_ON_GROUND = 1
LANDED_STATE_IN_AIR = 2
LANDED_STATE_TAKEOFF = 3
LANDED_STATE_LANDING = 4
AIRBORNE_STATES = (LANDED_STATE_IN_AIR, LANDED_STATE_TAKEOFF, LANDED_STATE_LANDING)

TRACK_STATE = "TRACK_AND_DESCEND"
FINAL_STATE = "FINAL_FIXED_COLUMN_DESCENT"

# Lines logged by package_delivery_mission.py.
_TRANSITION = re.compile(r"\[(?P<mission>[^\]]+)\] State: (?P<old>\w+) -> (?P<new>\w+)")
_FINAL_COLUMN = re.compile(
    r"\[(?P<mission>[^\]]+)\] Entering final touchdown column at (?P<alt>-?\d+(?:\.\d+)?) m"
)


# --- Inputs -------------------------------------------------------------------


@dataclass(frozen=True)
class Target:
    mission_name: str
    latitude_deg: float
    longitude_deg: float


def mission_target(sequence: Mapping) -> Target:
    """The first mission in a parsed sequence YAML with a GPS target."""

    missions = (sequence.get("mission_sequence") or {}).get("missions") or []
    for mission in missions:
        config = mission.get("config") or {}
        if "target_latitude" in config and "target_longitude" in config:
            return Target(
                mission_name=str(mission.get("name") or mission.get("type") or ""),
                latitude_deg=float(config["target_latitude"]),
                longitude_deg=float(config["target_longitude"]),
            )
    raise ValueError("no mission with target_latitude/target_longitude")


def target_local_xy(origin: GeoOrigin, target: Target) -> tuple[float, float]:
    """Target position in local ENU metres from home: (east, north)."""

    return world_xy_for(origin, target.latitude_deg, target.longitude_deg)


def parse_transition(text: str, mission_name: str) -> Optional[tuple[str, str]]:
    """(old, new) state names from a mission 'State: A -> B' log line, else None."""

    match = _TRANSITION.search(text)
    if match is None or match.group("mission") != mission_name:
        return None
    return match.group("old"), match.group("new")


def parse_final_column_altitude(text: str, mission_name: str) -> Optional[float]:
    """Altitude from an 'Entering final touchdown column at X m' line, else None."""

    match = _FINAL_COLUMN.search(text)
    if match is None or match.group("mission") != mission_name:
        return None
    return float(match.group("alt"))


def horizontal_distance_m(x: float, y: float, target_x: float, target_y: float) -> float:
    return math.hypot(x - target_x, y - target_y)


# --- Tracking -----------------------------------------------------------------


@dataclass
class Report:
    target_xy: tuple[float, float]
    final_descent_alt_m: Optional[float] = None
    final_descent_alt_source: str = ""
    touchdown_xyz: Optional[tuple[float, float, float]] = None
    distance_m: Optional[float] = None
    track_time_s: Optional[float] = None
    track_visits: int = 0
    track_partial: bool = False
    log_seen: bool = False
    notes: list[str] = field(default_factory=list)


class TouchdownTracker:
    """Turns timestamped pose, landed-state and log events into a Report.

    Touchdown is the first change from an airborne landed state to ON_GROUND
    after the final fixed-column descent began. If no mission log line was
    seen at all, it is the first airborne-to-ground change instead.
    """

    def __init__(self, mission_name: str, target_xy: tuple[float, float]):
        self.mission_name = mission_name
        self.target_xy = target_xy
        self._pose: Optional[tuple[float, float, float]] = None
        self._landed: Optional[int] = None
        self._log_seen = False
        self._final_started = False
        self._final_alt: Optional[float] = None
        self._final_alt_source = ""
        self._track_entered_at: Optional[float] = None
        self._track_total_s = 0.0
        self._track_visits = 0
        self._track_partial = False
        self._last_t: Optional[float] = None
        self._touchdown: Optional[tuple[float, float, float]] = None

    @property
    def done(self) -> bool:
        return self._touchdown is not None

    def on_pose(self, t: float, x: float, y: float, z: float) -> None:
        self._last_t = t
        self._pose = (x, y, z)

    def on_landed_state(self, t: float, state: int) -> None:
        self._last_t = t
        previous, self._landed = self._landed, state
        if self.done or state != LANDED_STATE_ON_GROUND or previous not in AIRBORNE_STATES:
            return
        if self._final_started or not self._log_seen:
            if self._pose is not None:
                self._touchdown = self._pose
                self._close_track(t)

    def on_log(self, t: float, text: str) -> None:
        self._last_t = t
        altitude = parse_final_column_altitude(text, self.mission_name)
        if altitude is not None:
            self._log_seen = True
            if not self._final_started:
                self._final_alt = altitude
                self._final_alt_source = "mission log"
            return

        transition = parse_transition(text, self.mission_name)
        if transition is None:
            return
        self._log_seen = True
        old, new = transition
        if old == TRACK_STATE:
            if self._track_entered_at is None and self._track_visits == 0:
                # It was already in TRACK_AND_DESCEND when we started.
                self._track_partial = True
            self._close_track(t)
        if new == TRACK_STATE and not self.done:
            self._track_entered_at = t
            self._track_visits += 1
        if new == FINAL_STATE and not self._final_started:
            self._final_started = True
            if self._final_alt is None and self._pose is not None:
                self._final_alt = self._pose[2]
                self._final_alt_source = "local pose at the state change"

    def _close_track(self, t: float) -> None:
        if self._track_entered_at is not None:
            self._track_total_s += t - self._track_entered_at
            self._track_entered_at = None

    def report(self) -> Report:
        report = Report(target_xy=self.target_xy, log_seen=self._log_seen)
        report.final_descent_alt_m = self._final_alt
        report.final_descent_alt_source = self._final_alt_source
        report.touchdown_xyz = self._touchdown
        if self._touchdown is not None:
            report.distance_m = horizontal_distance_m(
                self._touchdown[0], self._touchdown[1], *self.target_xy
            )
        if self._track_visits or self._track_partial:
            total = self._track_total_s
            if self._track_entered_at is not None and self._last_t is not None:
                total += self._last_t - self._track_entered_at
                report.notes.append("still in TRACK_AND_DESCEND when the report was made")
            report.track_time_s = total
            report.track_visits = self._track_visits
            report.track_partial = self._track_partial
        if not self._log_seen:
            report.notes.append(
                f"no '[{self.mission_name}]' lines seen on /rosout: "
                "no final-descent log or TRACK_AND_DESCEND time"
            )
        if self._touchdown is None:
            report.notes.append("no touchdown seen")
        return report


def format_report(report: Report, target: Target, sequence_name: str) -> str:
    def m(value: Optional[float]) -> str:
        return "not seen" if value is None else f"{value:.2f} m"

    east, north = report.target_xy
    lines = [
        f"Touchdown report: {sequence_name}, mission '{target.mission_name}'",
        f"  target:                      x {east:.2f} m east, y {north:.2f} m north of home"
        f" ({target.latitude_deg}, {target.longitude_deg})",
    ]
    alt = m(report.final_descent_alt_m)
    if report.final_descent_alt_m is not None:
        alt += f" ({report.final_descent_alt_source})"
    lines.append(f"  final fixed-column descent:  began at {alt}")
    if report.touchdown_xyz is None:
        lines.append("  touchdown:                   not seen")
    else:
        x, y, z = report.touchdown_xyz
        lines.append(f"  touchdown:                   x {x:.2f} m, y {y:.2f} m, z {z:.2f} m")
    lines.append(f"  distance to target:          {m(report.distance_m)} horizontal")
    if report.track_time_s is None:
        lines.append("  time in TRACK_AND_DESCEND:   not available")
    else:
        detail = f"{report.track_visits} entr{'y' if report.track_visits == 1 else 'ies'}"
        if report.track_partial:
            detail += ", already in it when this started: a lower bound"
        lines.append(f"  time in TRACK_AND_DESCEND:   {report.track_time_s:.1f} s ({detail})")
    lines.extend(f"  note: {note}" for note in report.notes)
    return "\n".join(lines)


# --- ROS (untested: needs the running sim) ---------------------------------------


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--sequence", type=Path, default=DEFAULT_SEQUENCE)
    parser.add_argument("--world-file", type=Path, default=DEFAULT_WORLD)
    parser.add_argument("--timeout", type=float, default=900.0, help="give up after this many s")
    args = parser.parse_args(argv)

    import yaml

    target = mission_target(yaml.safe_load(args.sequence.read_text()))
    _, origin = read_world(args.world_file.read_text())
    tracker = TouchdownTracker(target.mission_name, target_local_xy(origin, target))

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import ExtendedState
    from rcl_interfaces.msg import Log
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node("touchdown_report")
    # Best effort subscribers match both best-effort and reliable publishers.
    node.create_subscription(
        PoseStamped,
        "/mavros/local_position/pose",
        lambda msg: tracker.on_pose(
            time.monotonic(), msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ),
        qos_profile_sensor_data,
    )
    node.create_subscription(
        ExtendedState,
        "/mavros/extended_state",
        lambda msg: tracker.on_landed_state(time.monotonic(), int(msg.landed_state)),
        qos_profile_sensor_data,
    )
    node.create_subscription(
        Log, "/rosout", lambda msg: tracker.on_log(time.monotonic(), msg.msg), 100
    )

    print(
        f"Watching for the '{target.mission_name}' touchdown "
        f"(target x {tracker.target_xy[0]:.2f} m, y {tracker.target_xy[1]:.2f} m). Ctrl+C stops.",
        flush=True,
    )
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not tracker.done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        print(format_report(tracker.report(), target, args.sequence.name), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if tracker.done else 1


if __name__ == "__main__":
    sys.exit(main())
