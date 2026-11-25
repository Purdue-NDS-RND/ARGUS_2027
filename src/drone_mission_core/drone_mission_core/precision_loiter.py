"""The estimator's target as ArduPilot's precision landing wants it (CEN-09): a benchmark, not the competition's path.

Behind the mission flag `precision_loiter_bridge`, which defaults to false
and needs `use_target_estimator`. With the flag off nothing here is used.

ArduPilot can hold a multicopter over a target by itself (precision loiter,
its PLND library) when something tells it where the target is, as a
LANDING_TARGET message: the direction to the target in the body's frame and
the distance to it. This module makes that message's numbers from the
target estimator's track and the vehicle's pose, so that ArduPilot's own
loop can be flown against ours over the same target. It is sent only in
GUIDED and LOITER, only from a pose under 0.2 s old, and stamped with that
pose's time. The track is the estimator's primary bullseye, which with two
bullseyes in view need not be the one a mission is centring on: the
benchmark is for one target. It commands nothing
and asks for no mode: whoever flies the benchmark puts the vehicle in LOITER
with precision loiter switched on (docs/build/notes/CEN-09.md), and the
flight controller does the holding. The message also carries the target's
position in the body's frame, which MAVROS marks valid and a flight
controller that reads positions uses in place of the angles.

The message's two angles are the target's offset as a camera looking down
would see it: `angle_x` to the right, `angle_y` downwards in the picture,
which is backwards over the ground. ArduPilot's companion backend reads the
line of sight in the body's frame (forward, right, down) as
(-tan(angle_y), tan(angle_x), 1), so

    angle_x = atan2(right, down)        angle_y = -atan2(forward, down)

with (forward, right, down) the vector from the vehicle to the target in
the body's own axes, tilt included: the flight controller turns it back
into the world with its own attitude. Which way ArduPilot reads the two
angles was taken from its source as remembered, not from a document that
was to hand: the benchmark flight is the check (a wrong sign flies away
from the target, and is seen at once).

No ROS imports, standard library only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable, Optional, Sequence

from .features import get_feature_flag

FLAG = "precision_loiter_bridge"
MIN_DOWN_M = 0.3  # a target not at least this far under the vehicle is not one to hold over
MAX_ANGLE_RAD = math.radians(60.0)  # further off the body's own "down" than this, the track is not in any downward camera's view
MAX_AGE_S = 0.5  # a track older than this is not sent: the flight controller must lose the target when we have
MAX_POSE_AGE_S = 0.2  # the vehicle's newest pose must be this fresh: the target's place in the body's frame is only as new as the pose it was made with
SENT_IN_MODES = ("GUIDED", "LOITER")  # in any other mode (RTL, LAND, a pilot's) nothing is sent: a landing is never drawn onto the benchmark's target
MAV_FRAME_BODY_FRD = 12
PUBLISH_HZ = 10.0
TOPIC = "/mavros/landing_target/raw"  # MAVROS's landing_target plugin, which listens only with its `listen_lt` parameter true
TARGET_CLASS = "bullseye"
LANDING_TARGET_TYPE_VISION_OTHER = 3


@dataclass(frozen=True)
class LandingTarget:
    angle_x_rad: float  # the target to the right of the body's down axis
    angle_y_rad: float  # the target behind it (positive: backwards)
    distance_m: float
    forward_m: float
    right_m: float
    down_m: float


def _rotate_into_body(vector_enu: Sequence[float], orientation_xyzw: Sequence[float]) -> Optional[Sequence[float]]:
    """A world vector (east, north, up) in the body's axes (forward, left, up), by the inverse of the body's attitude."""
    x, y, z, w = (float(value) for value in orientation_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1.0e-6:
        return None
    x, y, z, w = -x / norm, -y / norm, -z / norm, w / norm  # the conjugate: world to body
    vx, vy, vz = (float(value) for value in vector_enu)
    # v' = v + 2 q_v x (q_v x v + w v)
    cx, cy, cz = y * vz - z * vy + w * vx, z * vx - x * vz + w * vy, x * vy - y * vx + w * vz
    return vx + 2.0 * (y * cz - z * cy), vy + 2.0 * (z * cx - x * cz), vz + 2.0 * (x * cy - y * cx)


def landing_target(target_enu: Sequence[float], vehicle_enu: Sequence[float], orientation_xyzw: Sequence[float], age_s: float = 0.0) -> Optional[LandingTarget]:
    """The message's numbers for a target at `target_enu` seen from a vehicle at `vehicle_enu` with the attitude `orientation_xyzw`
    (the body's forward-left-up axes in the east-north-up frame). None when the track is old, the target is not under the vehicle,
    or anything is not a number."""
    try:
        offset = [float(target_enu[index]) - float(vehicle_enu[index]) for index in range(3)]
    except (TypeError, ValueError, IndexError):
        return None
    if not all(math.isfinite(value) for value in offset) or not math.isfinite(age_s) or not 0.0 <= age_s <= MAX_AGE_S:
        return None
    body = _rotate_into_body(offset, orientation_xyzw)
    if body is None or not all(math.isfinite(value) for value in body):
        return None
    forward, right, down = body[0], -body[1], -body[2]
    if down < MIN_DOWN_M:
        return None
    angle_x, angle_y = math.atan2(right, down), -math.atan2(forward, down)
    if max(abs(angle_x), abs(angle_y)) > MAX_ANGLE_RAD:
        return None
    return LandingTarget(angle_x, angle_y, math.sqrt(forward * forward + right * right + down * down), forward, right, down)


def attach_precision_loiter(node: Any, specs: Iterable[Any], estimator_feed: Any, pose_buffer: Any) -> Optional[Any]:
    """The bridge on the executor: the estimator's confirmed bullseye track to MAVROS's landing_target plugin, PUBLISH_HZ times a second.
    None (and nothing published or made) unless a mission has the flag on. Returns the bridge's counters, for the log and for tests."""
    if not any(get_feature_flag(spec.features, FLAG) for spec in specs):
        return None
    if estimator_feed is None or pose_buffer is None:
        node.get_logger().error(f"{FLAG} is on but the target estimator or the pose history is not running: nothing is sent to the flight controller's precision landing")
        return None
    from mavros_msgs.msg import LandingTarget as Message

    from .target_access import from_estimate

    publisher = node.create_publisher(Message, TOPIC, 10)
    counts = {"sent": 0, "no confirmed track": 0, "no pose": 0, "not under the vehicle": 0, "another mode": 0}

    def tick() -> None:
        try:
            now_ns = int(node.get_clock().now().nanoseconds)
            if getattr(getattr(node, "_mavros_state", None), "mode", None) not in SENT_IN_MODES:
                counts["another mode"] += 1
                return
            estimate = estimator_feed.primary(TARGET_CLASS, now_ns)
            track = None if estimate is None else from_estimate(estimate)
            if track is None or track.state != "CONFIRMED":
                counts["no confirmed track"] += 1
                return
            # The newest pose as it was reported, and only if it is fresh: nothing is extrapolated, and the message is stamped
            # with the pose's own time, so what the flight controller is told is never newer than what it was made from.
            newest_ns = pose_buffer.newest_ns()
            pose = None if newest_ns is None or not 0 <= now_ns - newest_ns <= int(MAX_POSE_AGE_S * 1.0e9) else pose_buffer.pose_at(newest_ns)
            if pose is None:
                counts["no pose"] += 1
                return
            fields = landing_target(track.position, pose.position, pose.orientation, float(track.age_s))
            if fields is None:
                counts["not under the vehicle"] += 1
                return
            message = Message()
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(int(newest_ns), 1_000_000_000)
            message.header.frame_id = "base_link"
            message.target_num, message.frame, message.type = 0, MAV_FRAME_BODY_FRD, LANDING_TARGET_TYPE_VISION_OTHER
            message.angle = [float(fields.angle_x_rad), float(fields.angle_y_rad)]
            message.distance = float(fields.distance_m)
            message.size = [0.0, 0.0]
            # MAVROS turns this from the body's forward-left-up into the flight controller's forward-right-down and marks the position valid.
            message.pose.position.x, message.pose.position.y, message.pose.position.z = float(fields.forward_m), -float(fields.right_m), -float(fields.down_m)
            message.pose.orientation.w = 1.0
            publisher.publish(message)
            counts["sent"] += 1
        except Exception as error:  # noqa: BLE001  (the bridge is a benchmark's: a fault in it must not reach the executor's tick)
            node.get_logger().error(f"precision loiter bridge: {error}", throttle_duration_sec=5.0)

    node.create_timer(1.0 / PUBLISH_HZ, tick)
    node.get_logger().warn(f"Precision loiter bridge on ({FLAG}): the estimator's confirmed {TARGET_CLASS} track goes to {TOPIC} at {PUBLISH_HZ:g} Hz. A benchmark, not the competition's path")
    counts["tick"] = tick  # (for tests: the timer's own callback)
    return counts
