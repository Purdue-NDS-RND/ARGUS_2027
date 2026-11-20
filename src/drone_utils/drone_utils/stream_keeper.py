"""Which MAVLink message rates ARGUS needs from the flight controller, and when to ask for them. No ROS imports.

The flight controller forgets a requested rate when it reboots, and anything
else that sets rates (the takeoff service's request for all streams, a ground
station) replaces ours. `StreamKeeper` decides when a rate has to be
requested: once on connection, after a reboot, and when the rate it measures
for a message has dropped below what was asked. There is no periodic repeat.
The node around it (stream_keeper_node.py) sends each request as
MAV_CMD_SET_MESSAGE_INTERVAL through MAVROS's command service.

MAVROS serves every command in turn, the gripper's and the arming request
included, and waits up to 5 s for the flight controller to acknowledge each.
So a request of ours sitting in that queue can delay a release. Three rules
keep it out of the way:

  - nothing is sent while a mission is in a release or actuation state
    (`ActuationWatch`, fed from the mission executor's log);
  - one request is out at a time, and after a refused or unanswered one the
    keeper asks for nothing for a while;
  - a message whose rate does not come up after three accepted requests is
    left alone (a sensor that is not fitted) until the next reboot.
"""

from __future__ import annotations

import re
from typing import Dict, List, Mapping, Optional, Tuple

# MAVLink message id: (name, rate in Hz). PLT-06: pose 30, attitude 50,
# rangefinder 20, GPS 5, battery 2. The last three are not in that list:
# they are held at the rates the 2026 start-up gives them, so that the
# keeper replacing the takeoff service's request takes nothing away, and so
# that they too come back after a reboot.
MESSAGE_RATES_HZ: Dict[int, Tuple[str, float]] = {
    32: ("LOCAL_POSITION_NED", 30.0),  # local_position/pose
    30: ("ATTITUDE", 50.0),  # imu/data
    173: ("RANGEFINDER", 20.0),  # rangefinder/rangefinder
    24: ("GPS_RAW_INT", 5.0),  # gpsstatus/gps1/raw: the receiver
    147: ("BATTERY_STATUS", 2.0),  # battery
    33: ("GLOBAL_POSITION_INT", 20.0),  # global_position/global: the fix the missions hold position on
    245: ("EXTENDED_SYS_STATE", 2.0),  # extended_state: landed or in the air
    1: ("SYS_STATUS", 2.0),  # sys_status: the pre-arm health bit
}
SET_MESSAGE_INTERVAL = 511  # MAV_CMD_SET_MESSAGE_INTERVAL: param1 message id, param2 interval in microseconds
RATE_WINDOW_S = 5.0  # a message's rate is measured over this long
DROP_FRACTION = 0.8  # a measured rate under this share of the asked rate is a drop
MAX_FRUITLESS = 3  # accepted requests that leave the rate low before the keeper stops asking for that message
ANSWER_TIMEOUT_S = 6.0  # an unanswered request is given up after this long (MAVROS waits 5 s for the acknowledgement)
FAILURE_PAUSE_S = 5.0  # after a refused or unanswered request nothing is asked for this long
REBOOT_BACKSTEP_NS = 1_000_000_000  # the flight controller's clock going back by more than this is a reboot

# A mission state whose name contains one of these is a release or actuation
# state: the mission is about to send, or is sending, a command through
# MAVROS's command queue. Today: package_drop's DROP_PAYLOAD (the sprayer
# release), package_delivery's GROUND_DWELL (the gripper release), and
# takeoff's TAKING_OFF and WAITING_FOR_ALTITUDE (the takeoff service arms in
# the first and sends the takeoff command a few seconds into the second).
# The final column that leads into each release counts too (FINAL_FIXED...):
# the delivery sends its release one tick after entering GROUND_DWELL, so the
# keeper has to be quiet well before that. The 2027 gated drop's FINAL_ALIGN
# (CEN-04) is the same: its release is commanded on the tick after the gates
# have held.
ACTUATION_MARKS = ("FINAL_FIXED", "FINAL_ALIGN", "DROP_PAYLOAD", "GROUND_DWELL", "TAKING_OFF", "WAITING_FOR_ALTITUDE", "RELEASE", "GRIPPER", "ENGAGE", "ACTUAT")
HOLD_STALE_S = 60.0  # with nothing at all heard from the executor for this long, a hold no longer stands (the executor is gone)
_STATE_LINE = re.compile(r"\[(?P<mission>[^\]]+)\] State: \w+ -> (?P<state>\w+)")
_MISSION_OVER = re.compile(r"Mission '[^']*' (completed|ended with status)|Starting mission '")


def interval_us(rate_hz: float) -> float:
    """The interval MAV_CMD_SET_MESSAGE_INTERVAL takes for a rate."""
    return 1_000_000.0 / float(rate_hz)


def is_actuation_state(state_name: str) -> bool:
    return any(mark in state_name for mark in ACTUATION_MARKS)


class ActuationWatch:
    """Follows the mission executor's log to know when a mission is in a release or actuation state.

    Every mission logs "[name] State: OLD -> NEW" when it changes state, and
    the executor logs when a mission starts or ends. `active` is True from a
    change into an actuation state until the next change of state or the end
    of that mission.
    """

    def __init__(self) -> None:
        self.active = False
        self.state: Optional[str] = None  # "mission: STATE" while active
        self._heard_s: Optional[float] = None

    def holding(self, now_s: float) -> bool:
        """Whether the hold stands now: an actuation state, and the executor heard from within HOLD_STALE_S."""
        if not self.active:
            return False
        if self._heard_s is not None and now_s - self._heard_s > HOLD_STALE_S:
            self.active, self.state = False, None  # the executor went silent in that state: it is not running
        return self.active

    def on_log(self, text: str, now_s: Optional[float] = None) -> None:
        """One line of the mission executor's log, and the clock when it was heard."""
        if now_s is not None:
            self._heard_s = now_s
        match = _STATE_LINE.search(text)
        if match:
            self.active = is_actuation_state(match.group("state"))
            self.state = f"{match.group('mission')}: {match.group('state')}" if self.active else None
        elif _MISSION_OVER.search(text):
            self.active, self.state = False, None


class StreamKeeper:
    """Tracks which rate request is due. Times are seconds on the caller's clock."""

    def __init__(self, rates: Optional[Mapping[int, Tuple[str, float]]] = None) -> None:
        self.rates = dict(MESSAGE_RATES_HZ if rates is None else rates)
        for message_id, (name, rate) in self.rates.items():
            if not rate > 0.0:
                raise ValueError(f"rate for {name} ({message_id}) must be above 0 Hz, got {rate}")
        self._connected = False
        self._was_connected = False
        self._last_fcu_ns: Optional[int] = None
        self._due: Dict[int, float] = {}  # message id: when its request may go out
        self._confirmed: set = set()
        self._outstanding: Optional[int] = None
        self._sent_at = 0.0
        self._quiet_until = 0.0
        self._count: Dict[int, int] = {}
        self._window_start: Dict[int, float] = {}
        self._fruitless: Dict[int, int] = {}
        self.given_up: set = set()  # messages that stayed low after MAX_FRUITLESS accepted requests
        self.measured_hz: Dict[int, float] = {}
        self.hold = False  # True while a mission is in a release or actuation state
        self.reboots = 0
        self.rate_drops = 0
        self.failures = 0  # refused or unanswered requests since the last accepted one
        self.last_failed: Optional[int] = None

    def _request_all(self, now_s: float) -> None:
        self._confirmed.clear()
        self.given_up.clear()
        self._fruitless = {}
        self._due = {message_id: now_s for message_id in self.rates}
        self._count = {message_id: 0 for message_id in self.rates}
        self._window_start = {message_id: now_s for message_id in self.rates}

    def on_state(self, connected: bool, now_s: float) -> None:
        """MAVROS's link state to the flight controller."""
        connected = bool(connected)
        if connected and not self._connected:
            if self._was_connected:
                self.reboots += 1  # it went away and came back
            self._was_connected = True
            self._request_all(now_s)
        elif not connected and self._connected:
            self._confirmed.clear()
        self._connected = connected

    def on_fcu_time(self, boot_ns: int, now_s: float) -> None:
        """The flight controller's own clock (time since it booted), as MAVROS's time sync reports it."""
        boot_ns = int(boot_ns)
        if self._last_fcu_ns is not None and boot_ns < self._last_fcu_ns - REBOOT_BACKSTEP_NS:
            self.reboots += 1
            self._request_all(now_s)
        self._last_fcu_ns = boot_ns

    def on_message(self, message_id: int) -> None:
        """One message of this kind arrived from the flight controller (as its MAVROS topic)."""
        if message_id in self._count:
            self._count[message_id] += 1

    def _measure(self, now_s: float) -> None:
        """Close each rate window that is over; a confirmed message found below its rate becomes due again."""
        for message_id, (_name, rate) in self.rates.items():
            started = self._window_start.get(message_id, now_s)
            elapsed = now_s - started
            if elapsed < 0.0 or elapsed > 3.0 * RATE_WINDOW_S:
                # The clock stepped (or the keeper was not ticked): this window measures nothing. Start it again.
                self._count[message_id], self._window_start[message_id] = 0, now_s
                continue
            if elapsed < RATE_WINDOW_S:
                continue
            measured = self._count.get(message_id, 0) / elapsed
            self.measured_hz[message_id] = measured
            self._count[message_id], self._window_start[message_id] = 0, now_s
            if message_id in self.given_up:
                if measured >= DROP_FRACTION * rate:  # it does come after all: watch it again
                    self.given_up.discard(message_id)
                    self._confirmed.add(message_id)
                    self._fruitless[message_id] = 0
                continue
            if message_id not in self._confirmed or message_id == self._outstanding:
                continue
            if measured >= DROP_FRACTION * rate:
                self._fruitless[message_id] = 0
                continue
            self._fruitless[message_id] = self._fruitless.get(message_id, 0) + 1
            if self._fruitless[message_id] > MAX_FRUITLESS:
                self.given_up.add(message_id)  # asked and accepted, yet it does not come: the sensor is not there
                continue
            self.rate_drops += 1
            self._confirmed.discard(message_id)
            self._due[message_id] = now_s

    def _failed(self, message_id: int, now_s: float) -> None:
        self.failures += 1
        self.last_failed = message_id
        self._confirmed.discard(message_id)
        self._due[message_id] = now_s + 2.0 * FAILURE_PAUSE_S  # after the pause the other messages go first
        self._quiet_until = now_s + FAILURE_PAUSE_S

    def due(self, now_s: float) -> List[Tuple[int, float]]:
        """The request to send now: one (message id, rate), or nothing. It is then considered sent."""
        if not self._connected:
            return []
        self._measure(now_s)
        if self._outstanding is not None:
            if now_s - self._sent_at < ANSWER_TIMEOUT_S:
                return []  # one at a time
            self._failed(self._outstanding, now_s)  # no answer came
            self._outstanding = None
        if self.hold or now_s < self._quiet_until:
            return []
        waiting = [message_id for message_id, when in self._due.items() if when <= now_s]
        if not waiting:
            return []
        message_id = min(waiting, key=lambda candidate: self._due[candidate])  # the longest overdue
        del self._due[message_id]
        self._outstanding = message_id
        self._sent_at = now_s
        return [(message_id, self.rates[message_id][1])]

    def on_result(self, message_id: int, success: bool, now_s: float) -> None:
        """The flight controller's answer to the request that is out (False also for a call that failed)."""
        if message_id != self._outstanding:
            return  # not ours, or given up already
        self._outstanding = None
        if success:
            self.failures = 0
            self._confirmed.add(message_id)
            self._count[message_id], self._window_start[message_id] = 0, now_s  # measure from the acceptance on
        else:
            self._failed(message_id, now_s)

    def holding(self) -> bool:
        """True while connected with every rate accepted by the flight controller (or given up as not available)."""
        return self._connected and self._confirmed | self.given_up == set(self.rates)
