#!/usr/bin/env python3
"""Log MAVROS arm/mode/landed changes and exit when the flight is over.

Run inside the argus-gps container by sim/gps/run_mission.sh. Writes
"<epoch> armed=<0|1> mode=<mode> landed=<n> mono=<s>" lines to stdout. Exit
codes: 0 flight over (armed, then disarmed on the ground), 2 timed out,
3 never armed. The decisions are FlightTracker and FlightWatch in
flight_log.py, timed on the monotonic clock so a laptop suspend does not fire
the timeouts. The epoch stamp is only there to line up with rtf.log.

Usage: flight_monitor.py [--timeout S] [--arm-timeout S]
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rclpy
from mavros_msgs.msg import ExtendedState, State
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from flight_log import (  # noqa: E402
    ARM_TIMEOUT_S,
    FLIGHT_TIMEOUT_S,
    FlightTracker,
    FlightWatch,
)


class FlightMonitor(Node):
    def __init__(self, watch: FlightWatch) -> None:
        super().__init__("argus_gps_flight_monitor")
        self.watch = watch
        self.tracker = watch.tracker
        self._mode = ""
        self._last_line = ""
        self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.create_subscription(ExtendedState, "/mavros/extended_state", self._on_extended, 10)

    def _on_state(self, msg: State) -> None:
        if not msg.connected:
            # MAVROS publishes armed=false and an empty mode while it thinks
            # the link is down; that says nothing about the vehicle.
            print(f"{time.time():.3f} link-down (ignored) mono={self.watch.now():.3f}", flush=True)
            return
        self._mode = msg.mode
        self.tracker.update(self.watch.now(), armed=msg.armed)
        self._log()

    def _on_extended(self, msg: ExtendedState) -> None:
        self.tracker.update(self.watch.now(), landed_state=msg.landed_state)
        self._log()

    def _log(self) -> None:
        line = (
            f"armed={int(self.tracker.armed)} mode={self._mode} "
            f"landed={self.tracker.landed_state}"
        )
        if line != self._last_line:
            print(f"{time.time():.3f} {line} mono={self.watch.now():.3f}", flush=True)
            self._last_line = line


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=FLIGHT_TIMEOUT_S)
    parser.add_argument("--arm-timeout", type=float, default=ARM_TIMEOUT_S)
    args = parser.parse_args()

    rclpy.init()
    watch = FlightWatch(FlightTracker(), timeout_s=args.timeout, arm_timeout_s=args.arm_timeout)
    node = FlightMonitor(watch)
    code = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            result = watch.check()
            if result is not None:
                code = result
                break
    except KeyboardInterrupt:
        code = 130
    node.destroy_node()
    rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
