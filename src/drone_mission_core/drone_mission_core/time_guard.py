"""The stale-data guard as missions and the executor use it (FND-03).

Behind the mission flag `time_health_guard`, which defaults to false. With
the flag off the executor creates no monitor, `MissionContext.time_health()`
returns None, and no mission asks.

  - `attach_time_health` is the executor's side: with the flag on in the
    sequence it makes a `TimeHealth` (time_health.py) and feeds it from the
    pose, the target detection and MAVROS's time-sync status.
  - `TimeGuard` is a mission's side: it turns the report into "may I steer and
    descend?", says in the log when that changes, and times how long a hold
    has lasted so a mission can give up instead of hovering for ever. Once
    UNHEALTHY it stays so until the time base has been healthy for a full
    second, so a time base that flickers counts as one long fault.
"""

from __future__ import annotations

from typing import Any, Iterable, Optional

from .features import get_feature_flag
from .time_health import TimeHealth

TIME_HEALTH_FLAG = "time_health_guard"
# How long a mission holds a blind descent for an UNHEALTHY time base before
# it fails and its failure policy takes over. A mission may set its own with
# the config key `time_guard_hold_timeout_s`.
DEFAULT_HOLD_TIMEOUT_S = 10.0
# After an UNHEALTHY report the guard keeps holding until the reports have
# been healthy, without a break, for this long.
RECOVER_S = 1.0

POSE_TOPIC = "/mavros/local_position/pose"
DETECTION_TOPIC = "/drone_package_drop/target_detection"
TIMESYNC_TOPIC = "/mavros/timesync_status"


def guard_requested(specs: Iterable[Any]) -> bool:
    """Whether any mission of the sequence has the time_health_guard flag on."""
    return any(get_feature_flag(spec.features, TIME_HEALTH_FLAG) for spec in specs)


def _stamp_s(stamp: Any) -> float:
    return stamp.sec + stamp.nanosec * 1.0e-9


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def attach_time_health(node: Any, specs: Iterable[Any]) -> Optional[TimeHealth]:
    """A TimeHealth fed by `node`'s own subscriptions, or None when the flag is off.

    The subscriptions are the monitor's own, beside the executor's, so the
    executor's 2026 callbacks are not touched. The frame age is taken from
    the stamp of the target detection, which the detector publishes once per
    frame. That stamp is the frame's own only when the detector runs with
    its `features.stamp_from_image` flag on; with it off (2026) the detector
    stamps its output with its own clock, and the guard then sees a camera
    or detector that has stopped but not a frame that arrived late.
    """
    if not guard_requested(specs):
        return None
    from geometry_msgs.msg import PointStamped, PoseStamped
    from mavros_msgs.msg import TimesyncStatus
    from rclpy.qos import qos_profile_sensor_data

    health = TimeHealth()

    def on_timesync(message: Any) -> None:
        now_s = node.get_clock().now().nanoseconds * 1.0e-9
        health.on_timesync(now_s, message.estimated_offset_ns, message.round_trip_time_ms)

    node.create_subscription(PoseStamped, POSE_TOPIC, lambda message: health.on_pose(_stamp_ns(message.header.stamp)), qos_profile_sensor_data)
    node.create_subscription(PointStamped, DETECTION_TOPIC, lambda message: health.on_frame(_stamp_s(message.header.stamp)), 10)
    node.create_subscription(TimesyncStatus, TIMESYNC_TOPIC, on_timesync, qos_profile_sensor_data)
    node.get_logger().info("Stale-data guard on (time_health_guard): watching pose age, frame age and the time-sync offset")
    return health


class TimeGuard:
    """One mission's view of the time base. Only used with the flag on."""

    def __init__(self, mission_name: str, hold_timeout_s: float = DEFAULT_HOLD_TIMEOUT_S) -> None:
        self._name = mission_name
        self._hold_timeout_s = float(hold_timeout_s)
        self._unhealthy_since: Any = None
        self._healthy_since: Any = None

    def unhealthy(self, context: Any) -> bool:
        """True from an UNHEALTHY report (or no monitor) until the reports have been healthy for RECOVER_S."""
        read = getattr(context, "time_health", None)
        report = read() if callable(read) else None
        if report is None:
            bad, why = True, "the executor has no time monitor"
        else:
            bad, why = not report.healthy, "; ".join(report.reasons)
        if bad:
            self._healthy_since = None
            if self._unhealthy_since is None:
                self._unhealthy_since = context.now()
                context.logger.warn(f"[{self._name}] time base UNHEALTHY ({why}): treating it as no target, holding, no descent")
            return True
        if self._unhealthy_since is None:
            return False
        if self._healthy_since is None:
            self._healthy_since = context.now()
        if context.seconds_since(self._healthy_since) < RECOVER_S:
            return True
        held_s = context.seconds_since(self._unhealthy_since)
        self._unhealthy_since = None
        self._healthy_since = None
        context.logger.info(f"[{self._name}] time base healthy again after {held_s:.1f} s")
        return False

    def hold_expired(self, context: Any) -> bool:
        """Whether the guard has been holding, without a full recovery, for the hold timeout."""
        if self._unhealthy_since is None:
            return False
        return context.seconds_since(self._unhealthy_since) >= self._hold_timeout_s
