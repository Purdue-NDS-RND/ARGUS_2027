#!/usr/bin/env python3
"""The STATUSTEXT lines a flight's telemetry log holds: what a ground station was sent.

  python3 sim/statustext_from_tlog.py sim/.state/logs/latest/mavlink.tlog --prefix "ARGUS H"

Run where pymavlink is (the sim's image has it):

  docker run --rm -v "$PWD:$PWD" -w "$PWD" argus-sim:humble python3 sim/statustext_from_tlog.py sim/.state/logs/latest/mavlink.tlog --prefix "ARGUS H"

Prints the count and each line as (system, component, severity, text). Exit code 0 when at least one line
with the prefix is in the log, 1 when none is, 2 when the log cannot be read.
"""

from __future__ import annotations

import argparse
import sys
from typing import List, Optional, Tuple


def status_texts(path: str, prefix: str = "") -> List[Tuple[int, int, int, str]]:
    from pymavlink import mavutil

    log = mavutil.mavlink_connection(path)
    found = []
    while True:
        message = log.recv_match(type="STATUSTEXT", blocking=False)
        if message is None:
            break
        text = str(message.text)
        if text.startswith(prefix):
            found.append((int(message.get_srcSystem()), int(message.get_srcComponent()), int(message.severity), text))
    return found


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="The STATUSTEXT lines in a telemetry log.")
    parser.add_argument("tlog")
    parser.add_argument("--prefix", default="", help="only lines that start with this")
    args = parser.parse_args(argv)
    try:
        found = status_texts(args.tlog, args.prefix)
    except Exception as problem:  # noqa: BLE001  (a log that cannot be read, or no pymavlink here)
        print(f"statustext_from_tlog: {type(problem).__name__}: {problem}", file=sys.stderr)
        return 2
    print(f"{len(found)} status text(s)" + (f" starting with {args.prefix!r}" if args.prefix else "") + " in the telemetry log")
    for line in found:
        print(line)
    return 0 if found else 1


if __name__ == "__main__":
    sys.exit(main())
