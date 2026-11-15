#!/usr/bin/env python3
"""Fly at the geofence in the sim and measure what the autopilot does.

This is the SIM-02 pass line as a command: leaving the fence must end in RTL
within 2 s and no more than 10 m outside. Start the sim at a site, then:

  SITE=apollo_xi HEADLESS=1 sim/run_sim.sh                      # terminal 1
  sim/run_sim.sh --exec env -u PYTHONNOUSERSITE \
      python3 sim/fence_check.py --site sim/sites/apollo_xi.yaml   # terminal 2

It arms and takes off, so it is for the sim only: it refuses to fly unless the
autopilot answers for a SITL-only parameter. Steps:

  1. Fly to a target inside the fence. If the vehicle does not follow, stop:
     nothing after that would mean anything.
  2. Send the waypoint from the pass line, 20 m outside the east edge.
     ArduCopter in GUIDED refuses a position target outside its fence, and
     the vehicle stays put. If it flies there instead, that flight is measured.
  3. If the waypoint was refused, fly east at a fixed speed until the fence
     is breached.
  4. Measure, wait for the vehicle to come home and disarm, and report.

Positions are the autopilot's (GLOBAL_POSITION_INT), in metres from the site
origin. SITL's GPS is ideal, so they are close to the truth, but the judge's
fence report (sim/judge) on Gazebo's true poses is the scored measurement.
Times are message arrival times: as good as the position rate, which the
report gives. The measuring and the flight steps need no pymavlink.

Exit code 0 passed, 1 failed, 2 could not run.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import site_file  # noqa: E402
from judge.geometry import distance_outside_polygon  # noqa: E402

MAX_RTL_DELAY_S = 2.0
MAX_EXCURSION_M = 10.0
OUTSIDE_BY_M = 20.0  # the pass line's waypoint
INSIDE_HOP_M = 30.0
RTL = "RTL"


@dataclass(frozen=True)
class Sample:
    t: float
    e: float
    n: float
    u: float
    mode: str


def analyse(samples, polygon, max_rtl_delay_s: float = MAX_RTL_DELAY_S, max_excursion_m: float = MAX_EXCURSION_M) -> dict:
    """When the track left the polygon, when RTL began, and how far out it went."""

    t_cross = None
    max_excursion = 0.0
    for a, b in zip(samples, samples[1:]):
        outside_a = distance_outside_polygon(a.e, a.n, polygon)
        outside_b = distance_outside_polygon(b.e, b.n, polygon)
        max_excursion = max(max_excursion, outside_a, outside_b)
        if t_cross is None and outside_a == 0.0 and outside_b > 0.0:
            lo, hi = 0.0, 1.0  # the fraction of this leg at which it leaves the polygon
            for _ in range(30):
                mid = (lo + hi) / 2.0
                if distance_outside_polygon(a.e + (b.e - a.e) * mid, a.n + (b.n - a.n) * mid, polygon) > 0.0:
                    hi = mid
                else:
                    lo = mid
            t_cross = a.t + (b.t - a.t) * hi
    if samples and t_cross is None and distance_outside_polygon(samples[0].e, samples[0].n, polygon) > 0.0:
        t_cross = samples[0].t
        max_excursion = max(max_excursion, distance_outside_polygon(samples[0].e, samples[0].n, polygon))

    t_rtl = next((s.t for s in samples if s.mode == RTL), None)
    rtl_delay = None if t_rtl is None or t_cross is None else t_rtl - t_cross
    rate = None
    if len(samples) > 1 and samples[-1].t > samples[0].t:
        rate = (len(samples) - 1) / (samples[-1].t - samples[0].t)

    problem = ""
    if t_rtl is None:
        problem = "no RTL after flying at the fence"
    elif rtl_delay is not None and rtl_delay > max_rtl_delay_s:
        problem = f"RTL came {rtl_delay:.2f} s after the fence line (limit {max_rtl_delay_s} s)"
    elif max_excursion > max_excursion_m:
        problem = f"the vehicle went {max_excursion:.2f} m outside the fence (limit {max_excursion_m} m)"
    return {
        "t_cross": t_cross,
        "t_rtl": t_rtl,
        "rtl_delay_s": rtl_delay,  # negative: RTL began before the line was crossed
        "max_excursion_m": max_excursion,
        "position_rate_hz": rate,
        "passed": not problem,
        "problem": problem,
    }


def run_check(
    vehicle,
    site: site_file.Site,
    speed_mps: float = 5.0,
    height_m: float = 15.0,
    breach_timeout_s: float = 120.0,
    land_timeout_s: float = 300.0,
) -> dict:
    """The flight. `vehicle` has is_simulator, takeoff, goto, velocity, return_home, step, now and is_armed."""

    polygon = site.boundary
    east_edge = max(e for e, _ in polygon)
    report = {
        "site": site.name,
        "speed_mps": speed_mps,
        "inside_target_followed": False,
        "outside_target_refused": None,
        "breach_by": None,
        "passed": False,
        "problem": "",
    }

    if not vehicle.is_simulator():
        report["problem"] = "this autopilot is not SITL: fence_check arms and flies at the fence, so it runs in the sim only"
        return report

    vehicle.takeoff(height_m)
    start = vehicle.step()

    def watch(seconds: float, until=None) -> list[Sample]:
        seen = []
        deadline = vehicle.now() + seconds
        while vehicle.now() < deadline:
            seen.append(vehicle.step())
            if until is not None and until(seen):
                break
        return seen

    def settled(seen: list[Sample]) -> bool:
        recent = [s for s in seen if s.t >= seen[-1].t - 3.0]
        return seen[-1].t - seen[0].t >= 3.0 and all(math.hypot(s.e - seen[-1].e, s.n - seen[-1].n) < 0.5 for s in recent)

    # 1. A target inside the fence: the vehicle must follow it.
    hop = min(INSIDE_HOP_M, (east_edge - start.e) / 2.0)
    vehicle.goto(start.e + hop, start.n, height_m)
    leg = watch(60.0, until=lambda seen: math.hypot(seen[-1].e - start.e, seen[-1].n - start.n) > 3.0 and settled(seen))
    here = leg[-1] if leg else start
    report["inside_target_followed"] = math.hypot(here.e - start.e, here.n - start.n) >= hop / 2.0
    if not report["inside_target_followed"]:
        report["problem"] = "the vehicle did not fly to a target inside the fence, so the fence was not tested"
        vehicle.return_home()
        return report

    # 2. The pass line's waypoint, 20 m outside the east edge.
    vehicle.goto(east_edge + OUTSIDE_BY_M, here.n, height_m)
    samples = watch(15.0, until=lambda seen: math.hypot(seen[-1].e - here.e, seen[-1].n - here.n) > 3.0)
    moved = bool(samples) and math.hypot(samples[-1].e - here.e, samples[-1].n - here.n) > 3.0
    report["outside_target_refused"] = not moved

    # 3. Reach the fence: on the way to that waypoint, or on a velocity command.
    if moved:
        report["breach_by"] = "waypoint outside the fence"
        samples += watch(breach_timeout_s, until=lambda seen: seen[-1].mode == RTL)
    else:
        report["breach_by"] = "velocity command"
        deadline = vehicle.now() + breach_timeout_s
        while vehicle.now() < deadline:
            vehicle.velocity(speed_mps, 0.0)
            samples.append(vehicle.step())
            if samples[-1].mode == RTL:
                break

    # 4. The overshoot peaks after RTL begins: keep measuring until it is back inside.
    if samples and samples[-1].mode == RTL:
        t_rtl = samples[-1].t
        samples += watch(60.0, until=lambda seen: seen[-1].t - t_rtl > 3.0 and distance_outside_polygon(seen[-1].e, seen[-1].n, polygon) == 0.0)
    else:
        vehicle.return_home()  # never leave it flying away

    report.update(analyse(samples, polygon))
    watch(land_timeout_s, until=lambda seen: not vehicle.is_armed())
    report["landed_and_disarmed"] = not vehicle.is_armed()
    return report


class MavlinkVehicle:
    """The vehicle over MAVLink (pymavlink). Sim only: it arms and takes off."""

    def __init__(self, url: str, site: site_file.Site) -> None:
        from pymavlink import mavutil

        self._mavutil = mavutil
        self.site = site
        self.master = mavutil.mavlink_connection(url, source_system=245, source_component=192)
        deadline = time.monotonic() + 30.0
        while True:
            heartbeat = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if heartbeat is not None and heartbeat.get_srcComponent() == 1 and heartbeat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                self.system = heartbeat.get_srcSystem()
                break
            if time.monotonic() > deadline:
                raise RuntimeError(f"no autopilot heartbeat on {url}")
        self.t0 = time.monotonic()
        self.mode = mavutil.mode_string_v10(heartbeat)
        self.armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.last = None
        self._rates_at = 0.0

    def now(self) -> float:
        return time.monotonic() - self.t0

    def is_armed(self) -> bool:
        return self.armed

    def is_simulator(self) -> bool:
        """Only SITL has SIM_* parameters; a real flight controller does not answer for one."""
        for _ in range(3):
            self.master.mav.param_request_read_send(self.system, 1, b"SIM_SPEEDUP", -1)
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                message = self.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
                if message is not None and message.param_id.rstrip("\x00") == "SIM_SPEEDUP":
                    return True
        return False

    def _command(self, command: int, *params: float) -> None:
        padded = list(params) + [0.0] * (7 - len(params))
        self.master.mav.command_long_send(self.system, 1, command, 0, *padded)

    def _request_rates(self) -> None:
        # MAVProxy keeps asking for its own 4 Hz streams, so ask again now and then.
        if time.monotonic() - self._rates_at > 2.0:
            mav = self._mavutil.mavlink
            self._command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 50000)
            self._command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAVLINK_MSG_ID_HEARTBEAT, 200000)
            self._rates_at = time.monotonic()

    def step(self) -> Sample:
        """Wait for the next position message; keep mode and armed state up to date."""
        self._request_rates()
        mav = self._mavutil.mavlink
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            message = self.master.recv_match(type=["GLOBAL_POSITION_INT", "HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.5)
            if message is None:
                continue
            kind = message.get_type()
            if kind == "HEARTBEAT":
                if message.get_srcComponent() == 1 and message.autopilot != mav.MAV_AUTOPILOT_INVALID:
                    self.mode = self._mavutil.mode_string_v10(message)
                    self.armed = bool(message.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
            elif kind == "STATUSTEXT":
                print(f"  [{self.now():7.2f}] autopilot: {message.text}", flush=True)
            else:
                east, north = self.site.to_enu(message.lat / 1.0e7, message.lon / 1.0e7)
                self.last = Sample(self.now(), east, north, message.relative_alt / 1000.0, self.mode)
                return self.last
        if self.last is None:
            raise RuntimeError("no position from the autopilot")
        return Sample(self.now(), self.last.e, self.last.n, self.last.u, self.mode)

    def takeoff(self, height_m: float) -> None:
        mav = self._mavutil.mavlink
        self.master.set_mode("GUIDED")
        deadline = time.monotonic() + 60.0
        while not self.armed:
            if time.monotonic() > deadline:
                raise RuntimeError("the vehicle did not arm")
            self._command(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            for _ in range(8):
                self.step()
        self._command(mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, height_m)
        deadline = time.monotonic() + 90.0
        while self.step().u < height_m - 1.0:
            if time.monotonic() > deadline:
                raise RuntimeError(f"the vehicle did not climb to {height_m} m")

    def _target(self, mask: int, north: float, east: float, down: float, v_north: float, v_east: float) -> None:
        self.master.mav.set_position_target_local_ned_send(
            0, self.system, 1, self._mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
            north, east, down, v_north, v_east, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        )  # fmt: skip

    def goto(self, e: float, n: float, u: float) -> None:
        # Local NED from the EKF origin, which is home: the site origin in the sim.
        self._target(0b0000111111111000, n, e, -u, 0.0, 0.0)

    def velocity(self, east_mps: float, north_mps: float) -> None:
        self._target(0b0000111111000111, 0.0, 0.0, 0.0, north_mps, east_mps)

    def return_home(self) -> None:
        self.master.set_mode("RTL")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Fly at the geofence in the sim and measure the response.")
    parser.add_argument("--site", type=Path, required=True)
    parser.add_argument("--url", default="udpin:127.0.0.1:14552", help="pymavlink connection (the sim's tools port)")
    parser.add_argument("--speed", type=float, default=5.0, help="speed of the velocity command, m/s")
    parser.add_argument("--height", type=float, default=15.0)
    parser.add_argument("--out", type=Path, help="write the report as JSON")
    args = parser.parse_args(argv)
    try:
        site = site_file.load_site(args.site)
    except site_file.SiteError as error:
        print(f"fence_check: {error}", file=sys.stderr)
        return 2
    try:
        report = run_check(MavlinkVehicle(args.url, site), site, speed_mps=args.speed, height_m=args.height)
    except ImportError:
        print("fence_check: pymavlink is not installed here (in the sim container, run with PYTHONNOUSERSITE unset)", file=sys.stderr)
        return 2
    except RuntimeError as error:
        print(f"fence_check: {error}", file=sys.stderr)
        return 2
    if args.out:
        args.out.write_text(json.dumps(report, indent=2) + "\n")
    for key, value in report.items():
        print(f"{key}: {value:.3f}" if isinstance(value, float) else f"{key}: {value}")
    print("RESULT: PASS" if report["passed"] else f"RESULT: FAIL ({report['problem']})")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
