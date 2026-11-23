"""Rules for the GPS missions (PLT-13): a waypoint's timeout and hold, the route's order, a flying start, and
the items of an AUTO mission. No ROS; the standard library only.

Each is behind a mission flag of the GPS waypoint mission, all false by default:

  waypoint_timeout_hold   `WaypointClock`: a waypoint that is not reached within its timeout is given up and the
                          next one flown to; one that asks for a hold is held for that long, inside its radius,
                          before the next.
  time_trial_auto         `auto_items`: the route as an AUTO mission for the flight controller to fly itself, with
                          a pass radius of AUTO_PASS_RADIUS_M (2 m). GUIDED, the 2026 way, is the fallback.
  route_optimiser         `optimise_order`: the shortest way through every waypoint from where the vehicle starts.
  flying_start            `run_in_point`: a point before the first waypoint, on the line of the first leg, so that
                          the first waypoint is passed at speed.

The optimiser and the flying start change what is flown round the course: they are for when the organisers allow
them (FEATURES.md, open question 16).
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

EARTH_RADIUS_M = 6_371_000.0  # as the 2026 mission's haversine
AUTO_PASS_RADIUS_M = 2.0
EXACT_UP_TO = 9  # waypoints: every order is tried up to this many; beyond it, nearest neighbour and 2-opt
MAV_CMD_NAV_WAYPOINT = 16
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

GO, HOLD, REACHED, TIMED_OUT = "go", "hold", "reached", "timed_out"

LatLon = Tuple[float, float]


def _number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def distance_m(a: LatLon, b: LatLon) -> float:
    """The ground distance between two latitude and longitude pairs (haversine, as the 2026 mission's)."""
    lat1, lon1, lat2, lon2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    h = math.sin((lat2 - lat1) / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin((lon2 - lon1) / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(h))


def route_length_m(points: Sequence[LatLon], start: Optional[LatLon] = None) -> float:
    """The length of the way through the points in the order given, from `start` when there is one."""
    way = ([start] if start is not None else []) + list(points)
    return sum(distance_m(a, b) for a, b in zip(way, way[1:]))


def optimise_order(points: Sequence[LatLon], start: Optional[LatLon] = None) -> List[int]:
    """The order (indices into `points`) of the shortest way through every point, from `start` when there is one.

    Every order is tried for up to EXACT_UP_TO points (a course has seven). More than that: nearest neighbour, then
    2-opt until no swap shortens it. Of two orders of one length the earlier in index order is kept, so the same
    course always gives the same route.
    """
    count = len(points)
    for point in list(points) + ([start] if start is not None else []):
        if not isinstance(point, (list, tuple)) or len(point) != 2 or not all(_number(v) for v in point):
            raise ValueError(f"a waypoint is a latitude and a longitude, got {point!r}")
    if count <= 1:
        return list(range(count))

    def length(order: Sequence[int]) -> float:
        return route_length_m([points[i] for i in order], start)

    if count <= EXACT_UP_TO:
        best, best_length = None, math.inf
        for order in itertools.permutations(range(count)):
            total = length(order)
            if total < best_length - 1.0e-9:
                best, best_length = order, total
        return list(best)
    left = set(range(count))
    here = start
    order: List[int] = []
    while left:
        nearest = min(sorted(left), key=lambda i: 0.0 if here is None else distance_m(here, points[i]))
        order.append(nearest)
        left.remove(nearest)
        here = points[nearest]
    improved = True
    while improved:
        improved = False
        for i in range(count - 1):
            for j in range(i + 1, count):
                swapped = order[:i] + order[i : j + 1][::-1] + order[j + 1 :]
                if length(swapped) < length(order) - 1.0e-9:
                    order, improved = swapped, True
    return order


def run_in_point(first: LatLon, second: LatLon, run_in_m: float) -> LatLon:
    """The point `run_in_m` before the first waypoint on the line of the first leg: flown to first, so that the
    vehicle is already at speed, and on the leg's heading, when it passes the first waypoint."""
    if not _number(run_in_m) or run_in_m <= 0.0:
        raise ValueError(f"the run-in must be a distance above 0, got {run_in_m!r}")
    leg = distance_m(first, second)
    if leg <= 0.0:
        raise ValueError("the first two waypoints are the same place: the first leg has no direction to run in on")
    north = (second[0] - first[0]) / leg  # degrees of latitude per metre of leg
    east = (second[1] - first[1]) / leg
    return first[0] - north * run_in_m, first[1] - east * run_in_m


class WaypointClock:
    """One waypoint's timeout and hold. `update(now_s, inside)` each tick, `inside` being within its radius."""

    def __init__(self, timeout_s: Optional[float], hold_s: float = 0.0) -> None:
        if timeout_s is not None and (not _number(timeout_s) or timeout_s <= 0.0):
            raise ValueError(f"a waypoint's timeout must be a time above 0 or none, got {timeout_s!r}")
        if not _number(hold_s) or hold_s < 0.0:
            raise ValueError(f"a waypoint's hold must be a time of 0 or more, got {hold_s!r}")
        self.timeout_s, self.hold_s = timeout_s, float(hold_s)
        self._since_s: Optional[float] = None
        self._inside_since_s: Optional[float] = None
        self.outcome: Optional[str] = None  # REACHED or TIMED_OUT, once it is over

    def update(self, now_s: float, inside: bool) -> str:
        if self.outcome is not None:
            return self.outcome
        now = float(now_s)
        if self._since_s is None:
            self._since_s = now
        if inside:
            if self._inside_since_s is None:
                self._inside_since_s = now
            if now - self._inside_since_s >= self.hold_s:
                self.outcome = REACHED
                return REACHED
        else:
            self._inside_since_s = None  # out of the radius again: the hold starts over
        # The timeout runs through a hold too: a waypoint the vehicle cannot stay at is given up like one it cannot reach.
        if self.timeout_s is not None and now - self._since_s >= self.timeout_s:
            self.outcome = TIMED_OUT
            return TIMED_OUT
        return HOLD if inside else GO


@dataclass(frozen=True)
class MissionItem:
    """One item of an AUTO mission, as MAVLink's MISSION_ITEM fields."""

    frame: int
    command: int
    is_current: bool
    autocontinue: bool
    param1: float  # hold time, s
    param2: float  # acceptance radius, m
    param3: float  # pass radius: 0 passes through the waypoint
    param4: float  # yaw: not a number leaves the heading to the flight controller
    latitude: float
    longitude: float
    altitude_m: float  # above home


def auto_items(home: LatLon, waypoints: Sequence[Tuple[float, float, float]], pass_radius_m: float = AUTO_PASS_RADIUS_M) -> List[MissionItem]:
    """The route as AUTO mission items: `waypoints` are (latitude, longitude, height above home).

    The first item is home: ArduPilot keeps item 0 for it and starts flying at item 1.
    """
    if not _number(pass_radius_m) or pass_radius_m <= 0.0:
        raise ValueError(f"the pass radius must be a distance above 0, got {pass_radius_m!r}")
    if not waypoints:
        raise ValueError("an AUTO mission needs a waypoint")
    items = [MissionItem(MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT, False, True, 0.0, 0.0, 0.0, math.nan, float(home[0]), float(home[1]), 0.0)]
    for index, waypoint in enumerate(waypoints):
        if len(waypoint) != 3 or not all(_number(v) for v in waypoint) or waypoint[2] <= 0.0:
            raise ValueError(f"waypoint {index + 1} must be a latitude, a longitude and a height above 0, got {waypoint!r}")
        items.append(
            MissionItem(MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT, index == 0, True, 0.0, float(pass_radius_m), 0.0, math.nan, float(waypoint[0]), float(waypoint[1]), float(waypoint[2]))
        )
    return items


# --- the executor's thin ROS side: the AUTO mission's upload ------------------------------------

AUTO_FLAG = "time_trial_auto"
MISSION_PUSH_SERVICE = "/mavros/mission/push"  # mavros_msgs/srv/WaypointPush (MAVROS's `waypoint` plugin)
MISSION_REACHED_TOPIC = "/mavros/mission/reached"  # mavros_msgs/msg/WaypointReached


class MissionUpload:
    """The AUTO mission's upload and the items the flight controller says it has reached."""

    def __init__(self, client, make_request) -> None:
        self._client, self._make_request = client, make_request
        self.reached_seq: Optional[int] = None

    def ready(self) -> bool:
        return bool(self._client.service_is_ready())

    def push(self, items: Sequence[MissionItem], done_callback=None):
        self.reached_seq = None  # a new mission: nothing of it is reached yet
        future = self._client.call_async(self._make_request(items))
        if done_callback is not None:
            future.add_done_callback(done_callback)
        return future

    def on_reached(self, message) -> None:
        self.reached_seq = int(message.wp_seq)


def attach_mission_upload(node, specs) -> Optional[MissionUpload]:
    """A MissionUpload on `node`'s own client and subscription, or None with the flag off. With the flag on and
    MAVROS's waypoint plugin not running, the service never becomes ready and the mission flies in GUIDED."""
    from .features import get_feature_flag

    if not any(get_feature_flag(spec.features, AUTO_FLAG) for spec in specs):
        return None
    from mavros_msgs.msg import Waypoint, WaypointReached
    from mavros_msgs.srv import WaypointPush

    def make_request(items: Sequence[MissionItem]):
        request = WaypointPush.Request()
        request.start_index = 0
        for item in items:
            waypoint = Waypoint()
            waypoint.frame, waypoint.command = item.frame, item.command
            waypoint.is_current, waypoint.autocontinue = item.is_current, item.autocontinue
            waypoint.param1, waypoint.param2, waypoint.param3, waypoint.param4 = item.param1, item.param2, item.param3, item.param4
            waypoint.x_lat, waypoint.y_long, waypoint.z_alt = item.latitude, item.longitude, item.altitude_m
            request.waypoints.append(waypoint)
        return request

    upload = MissionUpload(node.create_client(WaypointPush, MISSION_PUSH_SERVICE), make_request)
    node.create_subscription(WaypointReached, MISSION_REACHED_TOPIC, upload.on_reached, 10)
    node.get_logger().info(f"Mission upload on ({AUTO_FLAG}): {MISSION_PUSH_SERVICE}, {MISSION_REACHED_TOPIC}")
    return upload
