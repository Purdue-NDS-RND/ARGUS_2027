"""Which MAVLink message rates ARGUS needs from the flight controller, and when to ask for them. No ROS imports.

The flight controller forgets a requested rate when it reboots, and anything
else that sets rates (the takeoff service's request for all streams, a ground
station) replaces ours. `StreamKeeper` decides when each rate has to be
requested again: on connection, after a reboot, after a refused or
unanswered request, and every ten seconds regardless. The node around it
(stream_keeper_node.py) sends each request as MAV_CMD_SET_MESSAGE_INTERVAL
through MAVROS's command service.

It asks for one message at a time and waits for the answer. MAVROS serves
every command, the gripper's and the arming request included, one after the
other, and waits up to 5 s for the flight controller to acknowledge each. A
burst of requests would queue ahead of a command that matters, so there is
never more than one of ours in that queue, and after a failure the keeper
stays quiet for a while.
"""

from __future__ import annotations

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
REASSERT_S = 10.0  # every rate is requested again this often, whatever happened in between
ANSWER_TIMEOUT_S = 6.0  # an unanswered request is given up after this long (MAVROS waits 5 s for the acknowledgement)
FAILURE_PAUSE_S = 5.0  # after a refused or unanswered request nothing is asked for this long
REBOOT_BACKSTEP_NS = 1_000_000_000  # the flight controller's clock going back by more than this is a reboot


def interval_us(rate_hz: float) -> float:
    """The interval MAV_CMD_SET_MESSAGE_INTERVAL takes for a rate."""
    return 1_000_000.0 / float(rate_hz)


class StreamKeeper:
    """Tracks which rate request is due. Times are seconds on the caller's clock."""

    def __init__(self, rates: Optional[Mapping[int, Tuple[str, float]]] = None, reassert_s: float = REASSERT_S) -> None:
        self.rates = dict(MESSAGE_RATES_HZ if rates is None else rates)
        for message_id, (name, rate) in self.rates.items():
            if not rate > 0.0:
                raise ValueError(f"rate for {name} ({message_id}) must be above 0 Hz, got {rate}")
        self._reassert_s = float(reassert_s)
        self._connected = False
        self._was_connected = False
        self._last_fcu_ns: Optional[int] = None
        self._next_due: Dict[int, float] = {}
        self._confirmed: set = set()
        self._outstanding: Optional[int] = None
        self._sent_at = 0.0
        self._quiet_until = 0.0
        self.reboots = 0
        self.failures = 0  # refused or unanswered requests since the last accepted one
        self.last_failed: Optional[int] = None  # the message id of the newest failure

    def _request_all(self, now_s: float) -> None:
        self._confirmed.clear()
        self._next_due = {message_id: now_s for message_id in self.rates}

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

    def _failed(self, message_id: int, now_s: float) -> None:
        self.failures += 1
        self.last_failed = message_id
        self._confirmed.discard(message_id)
        # The failed message waits a full repeat, so that after the pause the
        # others go first: one message the flight controller will not give
        # must not keep the rest from being asked for.
        self._next_due[message_id] = now_s + self._reassert_s
        self._quiet_until = now_s + FAILURE_PAUSE_S

    def due(self, now_s: float) -> List[Tuple[int, float]]:
        """The request to send now: one (message id, rate), or nothing. It is then considered sent."""
        if not self._connected:
            return []
        if self._outstanding is not None:
            if now_s - self._sent_at < ANSWER_TIMEOUT_S:
                return []  # one at a time
            self._failed(self._outstanding, now_s)  # no answer came
            self._outstanding = None
        if now_s < self._quiet_until:
            return []
        waiting = [message_id for message_id in self.rates if self._next_due.get(message_id, now_s) <= now_s]
        if not waiting:
            return []
        message_id = min(waiting, key=lambda candidate: self._next_due.get(candidate, now_s))  # the longest overdue
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
            self._next_due[message_id] = now_s + self._reassert_s
        else:
            self._failed(message_id, now_s)

    def holding(self) -> bool:
        """True while connected with every rate accepted by the flight controller."""
        return self._connected and self._confirmed == set(self.rates)
