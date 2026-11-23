#!/usr/bin/env python3
"""Stop the companion's setpoints in the sim and measure what the flight controller's watchdog does (PLT-09).

platform/fcu/scripts/argus_watchdog.lua runs on the flight controller: with
the vehicle flying in GUIDED, setpoints that stop for more than a second
must end in LOITER. This plays the companion against SITL with the script
loaded (the scenario's `fc_scripts`):

  1. Take off. The take-off is the flight controller's own and no setpoint
     is sent during it: the watchdog must leave it alone.
  2. Hover for 3 s, still sending nothing. No setpoint has come since the
     take-off, so there is nothing that has stopped: still GUIDED.
  3. Send position setpoints at 10 Hz for 6 s. Still GUIDED, and the
     vehicle goes where they say (the script's counting must not take the
     messages away from the autopilot).
  4. Stop. LOITER must begin more than 1 s and no more than 2 s after the
     last setpoint.

  sim/run_scenarios.sh hover/watchdog

It arms and takes off, so it is for the sim only: it refuses to fly unless
the autopilot answers for a SITL-only parameter. Times are message arrival
times at this tool (heartbeats at 5 Hz).

Exit code 0 passed, 1 failed, 2 could not run. The verdict needs no pymavlink.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import site_file  # noqa: E402
from fence_check import MavlinkVehicle, Sample  # noqa: E402

GUIDED, LOITER = "GUIDED", "LOITER"
TIMEOUT_S = 1.0  # the script's
LATEST_S = 2.0  # LOITER must have begun by this long after the last setpoint
QUIET_HOVER_S = 3.0
STREAM_S = 6.0
STREAM_HZ = 10.0
HOP_M = 5.0


def verdict(report: dict) -> str:
    """Why the flight does not show a working watchdog, or "" if it does."""
    if not report.get("guided_after_takeoff"):
        return f"the mode after the take-off was {report.get('mode_after_takeoff')}, not GUIDED: the watchdog (or something else) disturbed a take-off that sends no setpoints"
    if not report.get("guided_after_quiet_hover"):
        return "the mode left GUIDED during the quiet hover before any setpoint was sent: nothing had stopped, so the watchdog should not have acted"
    if not report.get("guided_while_streaming"):
        return "the mode left GUIDED while setpoints were being sent"
    if not report.get("followed_setpoints"):
        return f"the vehicle moved {report.get('moved_m', 0.0):.1f} m toward a setpoint {HOP_M:g} m away: the setpoints did not reach the autopilot"
    delay = report.get("loiter_delay_s")
    if delay is None:
        return f"no LOITER within {report.get('watched_s', 0.0):.0f} s of the last setpoint (the mode was {report.get('mode_after_stop')}): the watchdog did not act"
    if delay <= TIMEOUT_S:
        return f"LOITER began {delay:.2f} s after the last setpoint, inside the {TIMEOUT_S:g} s the companion is allowed to be quiet"
    if delay > LATEST_S:
        return f"LOITER began {delay:.2f} s after the last setpoint (limit {LATEST_S:g} s)"
    return ""


def run_check(vehicle, height_m: float = 10.0, land_timeout_s: float = 240.0) -> dict:
    """The flight. `vehicle` is fence_check's: is_simulator, takeoff, goto, return_home, step, now, is_armed."""
    report = {"passed": False, "problem": "", "loiter_delay_s": None}
    if not vehicle.is_simulator():
        report["problem"] = "this autopilot is not SITL: watchdog_check arms and flies, so it runs in the sim only"
        return report

    def watch(seconds: float, each=None, until=None) -> list[Sample]:
        seen = []
        deadline = vehicle.now() + seconds
        while vehicle.now() < deadline:
            if each is not None:
                each()
            seen.append(vehicle.step())
            if until is not None and until(seen):
                break
        return seen

    vehicle.takeoff(height_m)
    start = vehicle.step()
    report["mode_after_takeoff"] = start.mode
    report["guided_after_takeoff"] = start.mode == GUIDED
    quiet = watch(QUIET_HOVER_S)
    report["guided_after_quiet_hover"] = bool(quiet) and all(s.mode == GUIDED for s in quiet)

    last_sent = [vehicle.now(), 0.0]  # when the last setpoint went, and when the one before the next is due

    def send() -> None:
        if vehicle.now() >= last_sent[1]:
            vehicle.goto(start.e + HOP_M, start.n, height_m)
            last_sent[0], last_sent[1] = vehicle.now(), vehicle.now() + 1.0 / STREAM_HZ

    streamed = watch(STREAM_S, each=send)
    report["guided_while_streaming"] = bool(streamed) and all(s.mode == GUIDED for s in streamed)
    report["moved_m"] = math.hypot(streamed[-1].e - start.e, streamed[-1].n - start.n) if streamed else 0.0
    report["followed_setpoints"] = report["moved_m"] >= HOP_M / 2.0

    stopped_at = last_sent[0]
    after = watch(LATEST_S + 3.0, until=lambda seen: seen[-1].mode != GUIDED)
    report["watched_s"] = vehicle.now() - stopped_at
    report["mode_after_stop"] = after[-1].mode if after else None
    if after and after[-1].mode == LOITER:
        report["loiter_delay_s"] = after[-1].t - stopped_at

    vehicle.return_home()  # LOITER with no pilot is not a way down
    watch(land_timeout_s, until=lambda seen: not vehicle.is_armed())
    report["landed_and_disarmed"] = not vehicle.is_armed()
    report["problem"] = verdict(report)
    if not report["problem"] and not report["landed_and_disarmed"]:
        report["problem"] = f"the vehicle was not down and disarmed {land_timeout_s:g} s after the check"
    report["passed"] = not report["problem"]
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Stop the setpoints in the sim and time the watchdog's LOITER.")
    parser.add_argument("--site", type=Path, required=True)
    parser.add_argument("--url", default="udpin:127.0.0.1:14552", help="pymavlink connection (the sim's tools port)")
    parser.add_argument("--height", type=float, default=10.0)
    parser.add_argument("--out", type=Path, help="write the report as JSON")
    args = parser.parse_args(argv)
    try:
        site = site_file.load_site(args.site)
    except site_file.SiteError as error:
        print(f"watchdog_check: {error}", file=sys.stderr)
        return 2
    try:
        report = run_check(MavlinkVehicle(args.url, site), height_m=args.height)
    except ImportError:
        print("watchdog_check: pymavlink is not installed here (in the sim container, run with PYTHONNOUSERSITE unset)", file=sys.stderr)
        return 2
    except RuntimeError as error:
        print(f"watchdog_check: {error}", file=sys.stderr)
        return 2
    if args.out:
        args.out.write_text(json.dumps(report, indent=2) + "\n")
    for key, value in report.items():
        print(f"{key}: {value:.3f}" if isinstance(value, float) else f"{key}: {value}")
    print("RESULT: PASS" if report["passed"] else f"RESULT: FAIL ({report['problem']})")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
