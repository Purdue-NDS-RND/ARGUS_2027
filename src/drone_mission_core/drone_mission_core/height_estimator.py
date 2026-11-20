"""Height above the ground, and which sensor said so (CEN-03). No ROS imports at module level.

Behind the mission flag `use_height_estimator`, which defaults to false and
needs `use_frame_time_pose` (the pose history). With the flag off the
executor makes no estimator and `MissionContext.height_agl()` answers as
FND-07 built it.

With it on, `height_agl()` is this estimator's answer:

  - the rangefinder's height, when its readings are inside the sensor's range
    (0.1 to 12 m, the TFmini's), fresh, taken with the vehicle tilted no more
    than 15 degrees, agree with each other and put the ground near where it
    is expected, while the vehicle is tilted no more than 15 degrees now,
    the sensor's last three readings were not all out of range, and its
    reading has not stayed the same through 0.2 m of the vehicle's own
    climb or descent (a stuck sensor). A reading within 0.05 m of either end
    of the range is the sensor saying "out of range", so the readings used
    are from 0.15 to 11.95 m. Each reading is corrected by the cosine of the tilt the
    vehicle had at the reading's own moment, and turned into where the ground
    is; the height is the vehicle's height now above that ground, so it does
    not lag in a descent;
  - otherwise the barometer's: the height above home plus the known ground
    offset (how far the ground here is below home), with the reason the
    rangefinder was not used.

The answer always carries its source, "rangefinder" or "baro": the drop's
release height depends on it (CEN-04), so the source matters as much as the
number.

"Signal is good" is ArduPilot's judgement. A sensor_msgs/Range carries no
signal strength; ArduPilot's driver reports a weak return as out of range,
which fails the range test here. MAVROS's rangefinder topic gives limits of
0 and 1000 m whatever the sensor, so the sensor's limits are held here, and
a message with tighter limits of its own is held to those.

The readings are judged by projection v2's GroundHeight (projection_v2.py):
at least three within half a second, the median of the five nearest, within
half a metre of each other. This module adds the tighter tilt limit, the
sensor's range, the expected ground, the barometer path and the reason.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Any, Callable, Deque, Iterable, Optional, Tuple

from .features import get_feature_flag
from .target_access import BARO, RANGEFINDER, HeightAgl

HEIGHT_ESTIMATOR_FLAG = "use_height_estimator"

# The Benewake TFmini Plus and TFmini-S: 0.1 to 12 m.
SENSOR_MIN_M = 0.1
SENSOR_MAX_M = 12.0
# Tilted further than this the beam is not measuring the height under the vehicle.
MAX_TILT_DEG = 15.0
# A rejected reading is the reason for a baro height only while it is this recent.
REJECTED_MAX_AGE_S = 0.5
# This many readings in a row outside the sensor's range, and the sensor is
# out of range now: the height is the barometer's at once, though readings
# from before are still fresh. Fewer is a dropout, and the fresh readings stand.
OUT_OF_RANGE_RUN = 3
# A reading that repeats (to within STUCK_SAME_M) while the vehicle's own
# height changes by more than this is a sensor that has stopped measuring.
# A hover repeats too, so only travel tells the two apart, and until this
# much of it the height can be wrong by as much.
STUCK_TRAVEL_M = 0.2
STUCK_SAME_M = 0.001
# Kept readings wait this long, at most, for the pose of their own moment
# before the stuck-sensor test gives up on them.
STUCK_POSE_WAIT_S = 0.5


class HeightEstimator:
    """height(): the vehicle's height above the ground and its source, from rangefinder readings and the pose history."""

    def __init__(
        self,
        pose_at: Callable[[int], Any],
        sensor_min_m: float = SENSOR_MIN_M,
        sensor_max_m: float = SENSOR_MAX_M,
        max_tilt_deg: float = MAX_TILT_DEG,
    ) -> None:
        from .projection_v2 import GroundHeight  # needs numpy: imported only with the flag on

        self._ground = GroundHeight(pose_at, max_tilt_deg=max_tilt_deg)
        self._pose_at = pose_at
        self._max_tilt_deg = float(max_tilt_deg)
        self._same: Optional[Tuple[float, float, float]] = None  # (range, vehicle height when it was first read, travel since) of the repeating reading
        # Kept readings not yet put to the stuck-sensor test. A reading can arrive
        # stamped ahead of the newest pose, so the pose of its moment is looked up
        # when a height is asked for, as the votes' poses are.
        self._unjudged: Deque[Tuple[int, float]] = deque(maxlen=64)
        self._sensor_min_m = float(sensor_min_m)
        self._sensor_max_m = float(sensor_max_m)
        self._last_kept_ns: Optional[int] = None
        self._last_rejected: Optional[Tuple[int, str]] = None  # (stamp, why) of the newest reading not kept
        self._rejected_run = 0  # readings not kept since the last one that was
        self._newest_raw: Optional[Tuple[int, float]] = None  # (stamp, range) of the newest reading that was a number, kept or not

    def add_range(self, stamp_ns: int, range_m: float, min_m: float, max_m: float) -> None:
        """One rangefinder reading with the limits its message gave. Only a reading inside the sensor's range is kept."""
        low = max(self._sensor_min_m, _finite_or(min_m, self._sensor_min_m))
        high = min(self._sensor_max_m, _finite_or(max_m, self._sensor_max_m))
        try:
            range_m = float(range_m)
        except (TypeError, ValueError):
            range_m = math.nan
        if not math.isfinite(range_m):
            self._last_rejected = (int(stamp_ns), "the rangefinder's reading is not a number")
            self._rejected_run += 1
            return
        self._newest_raw = (int(stamp_ns), range_m)
        if self._ground.add_range(int(stamp_ns), range_m, low, high):
            self._last_kept_ns = int(stamp_ns)
            self._rejected_run = 0
            self._unjudged.append((int(stamp_ns), range_m))
        else:
            self._last_rejected = (int(stamp_ns), f"the rangefinder reads {range_m:.2f} m, outside its range ({low:g} to {high:g} m)")
            self._rejected_run += 1

    def newest_range(self, now_ns: int) -> Optional[Tuple[float, float]]:
        """(range, its age in seconds) of the newest reading that was a number, inside the sensor's range or not. None before any.

        Not a height: a reading under the sensor's minimum is given as it
        came. For the delivery's contact rule (CEN-05), which asks only
        whether the rangefinder reads the package's height or less.
        """
        if self._newest_raw is None:
            return None
        return self._newest_raw[1], (int(now_ns) - self._newest_raw[0]) * 1.0e-9

    def _judge_repeats(self, now_ns: int) -> None:
        """Keep how far the vehicle's own height has moved while the reading stayed the same, over the readings whose pose is known by now."""
        while self._unjudged:
            stamp_ns, range_m = self._unjudged[0]
            pose = self._pose_at(stamp_ns)
            if pose is None:
                if abs(now_ns - stamp_ns) <= int(STUCK_POSE_WAIT_S * 1.0e9):
                    return  # its pose may still arrive: this reading and the ones after it wait
                self._unjudged.popleft()  # no pose will come for it
                continue
            self._unjudged.popleft()
            up_m = float(pose.position[2])
            if self._same is not None and abs(range_m - self._same[0]) <= STUCK_SAME_M:
                self._same = (self._same[0], self._same[1], max(self._same[2], abs(up_m - self._same[1])))
            else:
                self._same = (range_m, up_m, 0.0)

    def _tilt_now_deg(self, orientation: Any, now_ns: int) -> Optional[float]:
        """The vehicle's tilt now: from the orientation the caller gave, else from the pose history. None when neither says."""
        if orientation is None:
            pose = self._pose_at(now_ns)
            if pose is None:
                return None
            orientation = pose.orientation
        try:
            x, y = float(orientation[0]), float(orientation[1])
        except (TypeError, ValueError, IndexError):
            return None
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))

    def height(
        self, vehicle_up_m: float, now_ns: int, ground_below_home_m: float = 0.0, pose_age_s: float = 0.0, orientation: Any = None
    ) -> Optional[HeightAgl]:
        """The height above the ground now, as a `HeightAgl` with its source, or None when the vehicle's own height is not a number.

        `orientation` is the vehicle's now as (x, y, z, w); without it the
        pose history is asked. Tilted over the limit now, the answer is the
        barometer's whatever the readings of a moment ago said.

        `vehicle_up_m` is the vehicle's height in the local frame (above
        home) now. `ground_below_home_m` is how far the ground here is known
        to be below home (negative: above it; 0 when nothing is known): it is
        added to the barometer's height, and it is where the rangefinder's
        ground is expected to be.
        """
        try:
            vehicle_up_m, offset_m = float(vehicle_up_m), float(ground_below_home_m)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(vehicle_up_m) or not math.isfinite(offset_m):
            return None
        ground_up, source = self._ground.ground_up(int(now_ns), expected_ground_up_m=-offset_m)
        out_of_range_now = (
            self._rejected_run >= OUT_OF_RANGE_RUN
            and self._last_rejected is not None
            and abs(int(now_ns) - self._last_rejected[0]) <= int(REJECTED_MAX_AGE_S * 1.0e9)
        )
        self._judge_repeats(int(now_ns))
        tilt_now = self._tilt_now_deg(orientation, int(now_ns))
        if tilt_now is not None and tilt_now > self._max_tilt_deg:
            why = f"the vehicle is tilted {tilt_now:.0f} degrees now (limit {self._max_tilt_deg:g} degrees)"
        elif self._same is not None and self._same[2] > STUCK_TRAVEL_M:
            why = f"the rangefinder's reading has not changed from {self._same[0]:.2f} m while the vehicle's height changed by {self._same[2]:.2f} m"
        elif source == RANGEFINDER and math.isfinite(ground_up) and not out_of_range_now:
            return HeightAgl(vehicle_up_m - ground_up, RANGEFINDER, float(pose_age_s), "")
        else:
            why = self._why_baro(int(now_ns))
        return HeightAgl(vehicle_up_m + offset_m, BARO, float(pose_age_s), why)

    def _why_baro(self, now_ns: int) -> str:
        rejected = self._last_rejected
        if (
            rejected is not None
            and abs(now_ns - rejected[0]) <= int(REJECTED_MAX_AGE_S * 1.0e9)
            and (self._last_kept_ns is None or rejected[0] >= self._last_kept_ns)
        ):
            return rejected[1]  # the newest thing the sensor said was not usable: that is the reason, not the older readings' state
        return self._ground.last_refusal or "the rangefinder was not believed"


def _finite_or(value: Any, fallback: float) -> float:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return fallback
    return value if math.isfinite(value) else fallback


def attach_height_estimator(node: Any, specs: Iterable[Any], pose_buffer: Any) -> Optional[HeightEstimator]:
    """A HeightEstimator fed by the rangefinder through `node`'s own subscription, or None when the flag is off.

    The executor's thin ROS side. The flag's dependency on the pose history is
    checked when the flags are resolved, before this is called; a sequence
    whose flags were not resolved that way gets no estimator without one.
    """
    if not any(get_feature_flag(spec.features, HEIGHT_ESTIMATOR_FLAG) for spec in specs):
        return None
    if pose_buffer is None:
        node.get_logger().error("Height estimator off: use_height_estimator needs the pose history (use_frame_time_pose)")
        return None
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Range

    from .frame_time import RANGEFINDER_TOPIC

    estimator = HeightEstimator(pose_buffer.pose_at)

    def on_range(message: Any) -> None:
        stamp = message.header.stamp
        estimator.add_range(int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec), message.range, message.min_range, message.max_range)

    node.create_subscription(Range, RANGEFINDER_TOPIC, on_range, qos_profile_sensor_data)
    node.get_logger().info(
        f"Height estimator on (use_height_estimator): the rangefinder between {SENSOR_MIN_M:g} and {SENSOR_MAX_M:g} m "
        f"and under {MAX_TILT_DEG:g} degrees of tilt, the height above home otherwise"
    )
    return estimator
