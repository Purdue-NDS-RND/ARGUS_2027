#!/usr/bin/env python3
"""Pull the flight controller's dataflash log of the last flight over MAVLink (PLT-12).

  python3 tools/dataflash_pull.py --fc-url udpin:127.0.0.1:14555 --out attempts/<name>/dataflash
  python3 tools/dataflash_pull.py --fc-url /dev/ttyACM0 --list            # what logs the flight controller holds

After landing and disarming: ArduPilot closes a flight's log at disarm, and
serves logs only while disarmed. The newest log is pulled unless `--log N`
names another. The file is written as <number padded to 8>.BIN, the name the
SD card gives it. The link is the ground station's or a spare telemetry port;
over a 57600 baud radio a flight's log takes many minutes, over USB or the
companion's serial link far less. Taking the SD card out gives the same file.

`pull` is the protocol (LOG_REQUEST_LIST, LOG_REQUEST_DATA in 90-byte
packets, the gaps asked for again, LOG_REQUEST_END) and takes any link with
pymavlink's `recv_match` and `mav` senders, so it is tested with a stand-in.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional

PACKET = 90  # bytes of log in one LOG_DATA message
WINDOW = PACKET * 512  # asked for at a time: a lost packet costs a re-request of at most this much
QUIET_S = 1.0  # with nothing heard for this long, what is still missing is asked for again
GIVE_UP_AFTER = 20  # re-requests in a row with nothing new before the pull fails


class PullError(RuntimeError):
    """The log could not be pulled. The message says why."""


def list_logs(link, target_system: int, target_component: int, timeout_s: float = 10.0, clock: Callable[[], float] = time.monotonic) -> Dict[int, int]:
    """{log number: size in bytes} of every log the flight controller holds."""
    link.mav.log_request_list_send(target_system, target_component, 0, 0xFFFF)
    logs: Dict[int, int] = {}
    expected = None
    deadline = clock() + timeout_s
    while clock() < deadline:
        entry = link.recv_match(type="LOG_ENTRY", blocking=True, timeout=0.5)
        if entry is None:
            continue
        expected = entry.num_logs
        if entry.num_logs == 0:
            return {}
        logs[entry.id] = entry.size
        if len(logs) >= expected:
            return logs
    if expected is None:
        raise PullError("the flight controller did not answer the request for its list of logs (is it disarmed, and is this link's stream to it open?)")
    return logs


def pull(link, target_system: int, target_component: int, log_id: int, size: int, clock: Callable[[], float] = time.monotonic,
         progress: Optional[Callable[[int, int], None]] = None) -> bytes:  # fmt: skip
    """The bytes of log `log_id`, all `size` of them. Raises PullError when the flight controller stops answering."""
    data = bytearray(size)
    have = bytearray(size)  # 1 where a byte has arrived
    received = 0
    stalls = 0

    def first_gap() -> Optional[int]:
        at = have.find(0)
        return None if at < 0 else at - at % PACKET

    def ask(offset: int) -> None:
        link.mav.log_request_data_send(target_system, target_component, log_id, offset, min(WINDOW, size - offset))

    ask(0)
    heard = clock()
    while received < size:
        message = link.recv_match(type="LOG_DATA", blocking=True, timeout=0.2)
        if message is not None and message.id == log_id and message.count > 0 and message.ofs + message.count <= size:
            new = have[message.ofs : message.ofs + message.count].count(0)
            data[message.ofs : message.ofs + message.count] = bytes(message.data[: message.count])
            have[message.ofs : message.ofs + message.count] = b"\x01" * message.count
            if new:
                received += new
                stalls = 0
                heard = clock()
                if progress is not None:
                    progress(received, size)
                # The end of a window that arrived whole: ask for the next from the first byte still missing.
                if (message.ofs + message.count) % WINDOW == 0 and received < size:
                    ask(first_gap())
            continue
        if clock() - heard >= QUIET_S:
            stalls += 1
            if stalls > GIVE_UP_AFTER:
                raise PullError(f"log {log_id}: {received} of {size} bytes arrived and the flight controller stopped answering")
            ask(first_gap())
            heard = clock()
    link.mav.log_request_end_send(target_system, target_component)
    return bytes(data)


def pull_newest(link, out_dir: Path, log_id: Optional[int] = None, clock: Callable[[], float] = time.monotonic, progress=None) -> Path:
    """Pull one log (the newest unless `log_id` is given) into `out_dir`; the file written."""
    target_system, target_component = link.target_system, link.target_component
    logs = list_logs(link, target_system, target_component, clock=clock)
    if not logs:
        raise PullError("the flight controller holds no log (LOG_BACKEND_TYPE, the SD card)")
    if log_id is None:
        log_id = max(logs)
    if log_id not in logs:
        raise PullError(f"the flight controller has no log {log_id} (it has {', '.join(map(str, sorted(logs)))})")
    if logs[log_id] <= 0:
        raise PullError(f"log {log_id} is empty")
    data = pull(link, target_system, target_component, log_id, logs[log_id], clock=clock, progress=progress)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{log_id:08d}.BIN"
    if path.exists():
        raise PullError(f"{path} exists already")
    path.write_bytes(data)
    return path


def connect(url: str, timeout_s: float = 10.0):
    """A pymavlink link to the flight controller, with its heartbeat heard."""
    try:
        from pymavlink import mavutil
    except ImportError:
        import site

        sys.path.append(site.getusersitepackages())  # installed for the user, in an environment that hides the user's packages (the sim's image)
        from pymavlink import mavutil
    link = mavutil.mavlink_connection(url, source_system=247, source_component=196)
    if link.wait_heartbeat(timeout=timeout_s) is None:
        raise PullError(f"no heartbeat from a flight controller on {url} in {timeout_s:g} s")
    return link


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Pull a dataflash log from the flight controller over MAVLink.")
    parser.add_argument("--fc-url", required=True, help="pymavlink URL of the flight controller")
    parser.add_argument("--out", help="the folder the log is written to")
    parser.add_argument("--log", type=int, help="the log's number (default: the newest)")
    parser.add_argument("--list", action="store_true", help="list the logs and pull nothing")
    args = parser.parse_args(argv)
    if not args.list and not args.out:
        parser.error("--out FOLDER, or --list")
    try:
        link = connect(args.fc_url)
        if args.list:
            for number, size in sorted(list_logs(link, link.target_system, link.target_component).items()):
                print(f"{number:8d}  {size / 1e6:8.2f} MB")
            return 0
        started = time.monotonic()
        path = pull_newest(link, Path(args.out), args.log)
        print(f"{path}: {path.stat().st_size / 1e6:.2f} MB in {time.monotonic() - started:.0f} s")
        return 0
    except (PullError, OSError, ImportError) as problem:
        print(f"dataflash_pull: {problem}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
