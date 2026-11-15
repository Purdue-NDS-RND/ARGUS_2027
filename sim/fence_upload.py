#!/usr/bin/env python3
"""Upload a site's geofence to the flight controller and switch it on.

The polygon and limits come from the site file (sim/site_file.py), the same
file the sim world and the judges use. The upload is the MAVLink mission
protocol with mission type FENCE; then the polygon is read back, and only
then are the FENCE_* parameters set, FENCE_ENABLE last, each one read back.
Any step that fails stops the upload with the fence parameters untouched.

  python3 sim/fence_upload.py --site sim/sites/apollo_xi.yaml --dry-run
  python3 sim/fence_upload.py --site sim/sites/apollo_xi.yaml [--url udpin:127.0.0.1:14552]

In the sim, start.sh runs it on the tools port (UDP 14552) when a site is set.
The logic needs no pymavlink; only the real connection does.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import site_file  # noqa: E402

DEFAULT_URL = "udpin:127.0.0.1:14552"
RETRIES = 3
TIMEOUT_S = 3.0
MAV_MISSION_TYPE_FENCE = 1
MAV_MISSION_ACCEPTED = 0
PARAM_TOLERANCE = 1.0e-3


class UploadError(RuntimeError):
    pass


def upload(link, plan: site_file.FencePlan, timeout_s: float = TIMEOUT_S) -> dict:
    """Send the polygon, read its size back, then set and read back the parameters.

    `link` is the conversation with the autopilot: send_count(n),
    send_item(item), request_count(), set_param(name, value) and
    recv(types, timeout_s), which returns a dict with "type" or None.
    """

    count = len(plan.items)
    _send_polygon(link, plan.items, timeout_s)

    stored = None
    for _ in range(RETRIES):
        link.request_count()
        message = link.recv(("MISSION_COUNT",), timeout_s)
        if message is not None:
            stored = int(message["count"])
            break
    if stored is None:
        raise UploadError("no answer when reading the fence back")
    if stored != count:
        raise UploadError(f"the autopilot holds {stored} fence items after an upload of {count}")

    params = {}
    for name, value in plan.params.items():
        params[name] = _set_param(link, name, float(value), timeout_s)
    return {"vertices": count, "params": params}


def _send_polygon(link, items: list[dict], timeout_s: float) -> None:
    count = len(items)
    link.send_count(count)
    silent = 1  # sends in a row with no answer
    last_sent = None
    while True:
        message = link.recv(("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"), timeout_s)
        if message is None:
            if silent >= RETRIES:
                raise UploadError(f"no answer from the autopilot after {RETRIES} tries (fence upload)")
            silent += 1
            if last_sent is None:
                link.send_count(count)
            else:
                link.send_item(items[last_sent])
            continue
        silent = 1
        if message["type"] == "MISSION_ACK":
            result = int(message["result"])
            if result != MAV_MISSION_ACCEPTED:
                raise UploadError(f"the autopilot rejected the fence (MAV_MISSION_RESULT {result})")
            if last_sent != count - 1:
                raise UploadError(f"the autopilot acknowledged the fence after {0 if last_sent is None else last_sent + 1} of {count} items")
            return
        seq = int(message["seq"])
        if not 0 <= seq < count:
            raise UploadError(f"the autopilot asked for fence item {seq} of {count}")
        link.send_item(items[seq])
        last_sent = seq


def _set_param(link, name: str, value: float, timeout_s: float) -> float:
    seen = None
    for _ in range(RETRIES):
        link.set_param(name, value)
        deadline = time.monotonic() + timeout_s
        while True:
            message = link.recv(("PARAM_VALUE",), max(0.0, deadline - time.monotonic()))
            if message is None:
                break
            if message["name"] != name:
                continue
            seen = float(message["value"])
            if abs(seen - value) <= PARAM_TOLERANCE:
                return seen
            break
    if seen is None:
        raise UploadError(f"parameter {name}: no answer (does this firmware have it?)")
    raise UploadError(f"parameter {name}: set {value}, read back {seen}")


class MavlinkLink:
    """The upload conversation over a real MAVLink connection (pymavlink)."""

    def __init__(self, url: str, heartbeat_timeout_s: float = 30.0) -> None:
        from pymavlink import mavutil

        self._mavutil = mavutil
        self.master = mavutil.mavlink_connection(url, source_system=245, source_component=191)
        # Wait for the autopilot itself, not a ground station or a router on the same link.
        deadline = time.monotonic() + heartbeat_timeout_s
        while True:
            heartbeat = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if heartbeat is not None and heartbeat.get_srcComponent() == 1 and heartbeat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                self.system, self.component = heartbeat.get_srcSystem(), 1
                break
            if time.monotonic() > deadline:
                raise UploadError(f"no autopilot heartbeat on {url} within {heartbeat_timeout_s:.0f} s")

    def send_count(self, count: int) -> None:
        self.master.mav.mission_count_send(self.system, self.component, count, MAV_MISSION_TYPE_FENCE)

    def send_item(self, item: dict) -> None:
        self.master.mav.mission_item_int_send(
            self.system, self.component, item["seq"], item["frame"], item["command"],
            0, 0, float(item["param1"]), 0.0, 0.0, 0.0, item["lat_e7"], item["lon_e7"], 0.0, MAV_MISSION_TYPE_FENCE,
        )  # fmt: skip

    def request_count(self) -> None:
        self.master.mav.mission_request_list_send(self.system, self.component, MAV_MISSION_TYPE_FENCE)

    def set_param(self, name: str, value: float) -> None:
        self.master.mav.param_set_send(
            self.system, self.component, name.encode(), value, self._mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

    def recv(self, types, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return None
            message = self.master.recv_match(type=list(types), blocking=True, timeout=remaining)
            if message is None:
                return None
            kind = message.get_type()
            if kind == "PARAM_VALUE":
                return {"type": kind, "name": message.param_id.rstrip("\x00"), "value": message.param_value}
            if getattr(message, "mission_type", MAV_MISSION_TYPE_FENCE) != MAV_MISSION_TYPE_FENCE:
                continue  # another tool's mission traffic on the same link
            if kind == "MISSION_COUNT":
                # Close the download we opened just to learn the count.
                self.master.mav.mission_ack_send(self.system, self.component, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_FENCE)
                return {"type": kind, "count": message.count}
            if kind == "MISSION_ACK":
                return {"type": kind, "result": message.type}
            return {"type": kind, "seq": message.seq}


def describe(site: site_file.Site, plan: site_file.FencePlan) -> str:
    lines = [f"fence for site {site.name}: {len(plan.items)} vertices"]
    lines += [f"  {item['seq']}: {item['lat_e7'] / 1e7:.7f}, {item['lon_e7'] / 1e7:.7f}" for item in plan.items]
    lines += [f"  {name} = {value:g}" for name, value in plan.params.items()]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Upload a site's geofence and switch it on.")
    parser.add_argument("--site", type=Path, required=True)
    parser.add_argument("--url", default=DEFAULT_URL, help=f"pymavlink connection (default {DEFAULT_URL})")
    parser.add_argument("--dry-run", action="store_true", help="print the plan and stop")
    parser.add_argument(
        "--allow-unverified",
        action="store_true",
        help="upload a site that says verified: false (the sim does; never use it on the aircraft)",
    )
    args = parser.parse_args(argv)
    try:
        site = site_file.load_site(args.site)
    except site_file.SiteError as error:
        print(f"fence_upload: {error}", file=sys.stderr)
        return 2
    plan = site_file.fence_plan(site)
    print(describe(site, plan))
    if not site.verified:
        print("  NOTE: this site's coordinates are not verified against the field map")
    if args.dry_run:
        print("dry run: nothing sent")
        return 0
    if not site.verified and not args.allow_unverified:
        print(
            f"fence_upload: {args.site} is not verified against the field map, so its fence is not uploaded. "
            "Check the coordinates and set verified: true. (The sim passes --allow-unverified.)",
            file=sys.stderr,
        )
        return 2
    try:
        report = upload(MavlinkLink(args.url), plan)
    except ImportError:
        print("fence_upload: pymavlink is not installed here (in the sim container, run with PYTHONNOUSERSITE unset)", file=sys.stderr)
        return 2
    except UploadError as error:
        print(f"fence_upload: FAILED: {error}", file=sys.stderr)
        return 1
    print(f"fence uploaded and read back: {report['vertices']} vertices; fence enabled")
    return 0


if __name__ == "__main__":
    sys.exit(main())
