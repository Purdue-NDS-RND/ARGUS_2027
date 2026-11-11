#!/usr/bin/env python3
"""Get or set SITL parameters, or reboot SITL, over a spare MAVLink port.

Used by sim/gps/set_home.sh, inside the argus-gps container. It talks to
SITL's SERIAL2 (TCP 5763), which nothing else uses, so it doesn't depend on
MAVROS having downloaded the parameter list. On MAVROS's link the 20 Hz
streams leave so little room that the download never finishes.

  sitl_params.py get NAME...
  sitl_params.py set NAME=VALUE...      prints the value read back
  sitl_params.py reboot

Needs pymavlink (in the base image's /opt/ardupilot-python).
"""

from __future__ import annotations

import os
import sys
import time

from pymavlink import mavutil

URL = os.environ.get("SITL_PARAM_URL", "tcp:127.0.0.1:5763")
TIMEOUT_S = 10.0


def connect():
    master = mavutil.mavlink_connection(URL, source_system=245, source_component=190)
    if master.wait_heartbeat(timeout=TIMEOUT_S) is None:
        raise SystemExit(f"ERROR: no heartbeat from SITL on {URL}")
    return master


def get(master, name: str) -> float:
    for _ in range(3):
        master.mav.param_request_read_send(
            master.target_system, master.target_component, name.encode(), -1
        )
        deadline = time.time() + TIMEOUT_S / 3
        while time.time() < deadline:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg is not None and msg.param_id == name:
                return float(msg.param_value)
    raise SystemExit(f"ERROR: SITL did not return {name}")


def set_(master, name: str, value: float) -> float:
    for _ in range(3):
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            name.encode(),
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        deadline = time.time() + TIMEOUT_S / 3
        while time.time() < deadline:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg is not None and msg.param_id == name:
                # float32 storage: compare with a tolerance
                if abs(msg.param_value - value) <= max(1e-4, abs(value) * 1e-6):
                    return float(msg.param_value)
    raise SystemExit(f"ERROR: SITL did not confirm {name}={value}")


def reboot(master) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0, 1, 0, 0, 0, 0, 0, 0,
    )
    msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=TIMEOUT_S)
    result = getattr(msg, "result", None)
    print(f"reboot ack: {result} (0 = accepted)")
    if result not in (None, 0):
        raise SystemExit("ERROR: SITL refused the reboot (is it armed?)")


def main(argv: list[str]) -> int:
    if len(argv) < 2 or argv[1] not in {"get", "set", "reboot"}:
        print(__doc__, file=sys.stderr)
        return 2
    master = connect()
    if argv[1] == "get":
        for name in argv[2:]:
            print(f"{name} {get(master, name)!r}")
    elif argv[1] == "set":
        for pair in argv[2:]:
            name, _, value = pair.partition("=")
            print(f"{name} {set_(master, name, float(value))!r}")
    else:
        reboot(master)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
