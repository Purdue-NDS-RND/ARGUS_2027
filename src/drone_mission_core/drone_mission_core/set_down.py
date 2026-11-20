"""The delivery's soft set-down (CEN-05): when the package is on the ground, and how fast to come down to it.

Behind the mission flag `delivery_soft_set_down`, which defaults to false.
In 2026 the delivery froze a GPS point at the handoff height, came down that
column, and waited for the flight controller to say "landed" (which it says
about 2.5 s after the legs touch) plus half a second. With the flag on the
delivery mission (drone_mission_demo/missions/delivery_set_down.py) comes
down on the estimator's track; below HOLD_BELOW_M it holds its horizontal
position, so that no correction drags the package, and descends at
SET_DOWN_RATE_MPS until contact.

Contact is three things at once, for CONTACT_HOLD_S without a break:

  range     the rangefinder reads the package's height or less (the mission's
            `package_height_m`: what the rangefinder reads with the package
            resting on the ground). A reading under the sensor's own minimum
            counts: it is "or less".
  speed     no vertical speed: within STILL_MPS of zero.
  throttle  the throttle has dropped by THROTTLE_DROP of what it was while
            the vehicle was still coming down. The flight controller is
            asked to keep descending; on the ground it cannot, and takes
            the throttle off.

The throttle it compares with is the median of the throttle heard while the
vehicle was descending, over the last BASELINE_S of descending. Samples
taken while the vehicle is still are not part of it: on the ground the
throttle runs down to idle, and a baseline that followed it would hide the
drop. With no throttle heard during the descent there is no baseline and no
contact by this rule; the mission then has the flight controller's own
"landed" to go on.

The throttle is MAVROS's /mavros/vfr_hud (VFR_HUD's throttle, 0 to 1): the
`vfr_hud` plugin, which config/mavros/argus_mavros.yaml allows, at the rate
the stream keeper asks for. attach_throttle is the executor's thin ROS side:
with the flag off it subscribes to nothing.

No ROS at import time; the standard library only.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from statistics import median
from typing import Any, Deque, Iterable, Optional, Tuple

from .features import get_feature_flag

SET_DOWN_FLAG = "delivery_soft_set_down"

HOLD_BELOW_M = 0.6  # below this the horizontal position is held and the descent is at the set-down rate
SET_DOWN_RATE_MPS = 0.12
APPROACH_PER_S = 0.3  # descent rate per metre above HOLD_BELOW_M, on top of the set-down rate
CONTACT_HOLD_S = 0.3
AFTER_RELEASE_WAIT_S = 1.0  # between the release and the climb

STILL_MPS = 0.05  # "vertical speed near zero"; the descent itself is 0.12 m/s
THROTTLE_DROP = 0.15  # the share of the descent's throttle that has to be gone
BASELINE_S = 3.0  # the descent's throttle is its median over this much descending
BASELINE_MIN_SAMPLES = 5
# A rangefinder reading or a throttle older than this is not now. Both come at 20 Hz; it is under the contact
# hold, so that the 0.3 s cannot rest on one sample of a stream that has stopped.
SIGNAL_MAX_AGE_S = 0.2
# Two ticks further apart than this are not one unbroken stretch: the executor's tick is 0.05 s.
MAX_TICK_GAP_S = 0.2

# A height that is not the rangefinder's (the barometer's: above home, not above this ground) is taken to be this
# much lower than it says, so the hold and the 0.12 m/s begin that much earlier.
BARO_MARGIN_M = 0.5
# The hold also begins this close above the height below which the mission takes no target to steer on.
FLOOR_MARGIN_M = 0.05
HOLD_TIMEOUT_S = 20.0  # in the hold without contact: the last 0.6 m at 0.12 m/s is 5 s
NO_HEIGHT_TIMEOUT_S = 10.0  # with no height above the ground at all

THROTTLE_TOPIC = "/mavros/vfr_hud"


def _number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def working_height(height_m: float, from_rangefinder: bool) -> float:
    """The height the set-down acts on: the rangefinder's as it is, any other source's less BARO_MARGIN_M."""
    return float(height_m) if from_rangefinder else float(height_m) - BARO_MARGIN_M


def descent_rate(height_m: float, rate_mps: float) -> float:
    """How fast to come down at this (working) height: the mission's rate while high, easing to the set-down rate at HOLD_BELOW_M."""
    easing = SET_DOWN_RATE_MPS + APPROACH_PER_S * max(0.0, float(height_m) - HOLD_BELOW_M)
    return min(abs(float(rate_mps)), easing)


@dataclass(frozen=True)
class ContactReading:
    """What one tick knows of the three signals. None: not known."""

    range_m: Optional[float]  # the rangefinder's newest reading, whatever its range limits say
    range_age_s: Optional[float]
    vertical_speed_mps: Optional[float]  # up is positive
    throttle: Optional[float]  # 0 to 1
    throttle_age_s: Optional[float]


class ContactDetector:
    """update() each tick; True once the three signals have been there together for CONTACT_HOLD_S."""

    def __init__(self, package_height_m: Any) -> None:
        if not _number(package_height_m) or not 0.0 < package_height_m < HOLD_BELOW_M:
            raise ValueError(
                f"package_height_m must be what the rangefinder reads with the package on the ground: above 0 and under {HOLD_BELOW_M:g} m, "
                f"got {package_height_m!r}"
            )
        self.package_height_m = float(package_height_m)
        self._descending: Deque[Tuple[float, float]] = deque()  # (time, throttle) while the vehicle was coming down
        self._since: Optional[float] = None
        self._last: Optional[float] = None
        self.baseline: Optional[float] = None  # the throttle while descending
        self.missing: Tuple[str, ...] = ("range", "speed", "throttle")

    def reset(self) -> None:
        self._since = None
        self._last = None

    def _fresh(self, value: Optional[float], age_s: Optional[float]) -> bool:
        return _number(value) and _number(age_s) and 0.0 <= age_s <= SIGNAL_MAX_AGE_S

    def observe(self, reading: ContactReading, now_s: float) -> None:
        """Take one tick's throttle into the picture of the descent, if the vehicle is coming down. update() does this itself."""
        if not self._fresh(reading.throttle, reading.throttle_age_s) or not _number(reading.vertical_speed_mps):
            return
        if reading.vertical_speed_mps >= -STILL_MPS:
            return  # not coming down: on the ground the throttle runs to idle, and that is not the descent's throttle
        self._descending.append((now_s, float(reading.throttle)))
        while self._descending and now_s - self._descending[0][0] > BASELINE_S:
            self._descending.popleft()
        if len(self._descending) >= BASELINE_MIN_SAMPLES:
            self.baseline = median(value for _when, value in self._descending)

    def update(self, reading: ContactReading, now_s: float) -> bool:
        self.observe(reading, now_s)
        missing = []
        if not (self._fresh(reading.range_m, reading.range_age_s) and reading.range_m <= self.package_height_m):
            missing.append("range")
        if not (_number(reading.vertical_speed_mps) and abs(reading.vertical_speed_mps) <= STILL_MPS):
            missing.append("speed")
        if not (
            self.baseline is not None
            and self._fresh(reading.throttle, reading.throttle_age_s)
            and reading.throttle <= (1.0 - THROTTLE_DROP) * self.baseline
        ):
            missing.append("throttle")
        self.missing = tuple(missing)
        broken = self._last is not None and now_s - self._last > MAX_TICK_GAP_S
        self._last = now_s
        if missing:
            self._since = None
            return False
        if self._since is None or broken:
            self._since = now_s
        return now_s - self._since >= CONTACT_HOLD_S - 1.0e-9

    @property
    def held_s(self) -> float:
        return 0.0 if self._since is None or self._last is None else self._last - self._since


class ThrottleFeed:
    """The newest throttle heard from the flight controller."""

    def __init__(self) -> None:
        self._newest: Optional[Tuple[int, float]] = None

    def add(self, stamp_ns: int, throttle: Any) -> None:
        try:
            value = float(throttle)
        except (TypeError, ValueError):
            return
        if math.isfinite(value) and 0.0 <= value <= 1.0:
            self._newest = (int(stamp_ns), value)

    def newest(self, now_ns: int) -> Optional[Tuple[float, float]]:
        """(throttle, its age in seconds), or None when none was heard."""
        if self._newest is None:
            return None
        return self._newest[1], (int(now_ns) - self._newest[0]) * 1.0e-9


def attach_throttle(node: Any, specs: Iterable[Any]) -> Optional[ThrottleFeed]:
    """A ThrottleFeed fed by MAVROS's VFR_HUD through `node`'s own subscription, or None when the flag is off."""
    if not any(get_feature_flag(spec.features, SET_DOWN_FLAG) for spec in specs):
        return None
    from mavros_msgs.msg import VfrHud
    from rclpy.qos import qos_profile_sensor_data

    feed = ThrottleFeed()

    def on_hud(message: Any) -> None:
        stamp = message.header.stamp
        feed.add(int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec), message.throttle)

    node.create_subscription(VfrHud, THROTTLE_TOPIC, on_hud, qos_profile_sensor_data)
    node.get_logger().info(f"Throttle feed on ({SET_DOWN_FLAG}): {THROTTLE_TOPIC}, for the delivery's contact rule")
    return feed
