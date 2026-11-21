#!/usr/bin/env python3
"""Take hold of the bucket, lift it, bring it back and set it down (REC-8's check of the sim itself). Runs inside the sim container.

  env -u PYTHONNOUSERSITE python3 sim/recovery_check.py --site sim/sites/apollo_xi.yaml --run-dir <run folder>

A scenario's driver: it flies the recovery vehicle itself over MAVLink, in
place of a mission, to show that the sim's recovery parts work together
before there is a mission to fly them (REC-2): the hook under the vehicle,
the payload bridge that joins the bucket to it when the gripper output
says grab, the joint the bucket swings on, and the recovery judge.

It is told where the bucket truly is (the run's truth.json): it has no
camera and finds nothing. It flies over the bucket, comes down until the
hook is at the handle, commands the gripper to grab, climbs, flies to the
return point, comes down until the bucket is just off the ground, commands
the release, climbs, moves aside and lands.

Sim only: it refuses an autopilot that is not SITL, as sim/fence_check.py
does. `run_check` takes any vehicle with is_simulator, takeoff, step, now,
goto, gripper, land and is_armed, so the flight's logic is tested without
MAVLink.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

SIM_DIR = Path(__file__).resolve().parent
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))

import site_file  # noqa: E402

ARRIVE_M = 0.08  # over a point: this close to it sideways
LEVEL_M = 0.06  # at a height: this close to it
SETTLE_S = 2.0  # and for this long
STEP_TIMEOUT_S = 90.0
GRIP_WAIT_S = 2.0  # after a gripper command, before moving
SET_DOWN_CLEARANCE_M = 0.05  # the bucket's base above the ground when it is let go
CLEAR_ASIDE_M = 2.0  # sideways from the bucket before landing
LAND_TIMEOUT_S = 120.0
CARRY_SPEED_MPS = 1.0  # with the bucket on the hook the point the vehicle is sent to moves no faster than this


def run_check(
    vehicle,
    bucket_en: Tuple[float, float],
    return_en: Tuple[float, float],
    grab_height_m: float,
    height_m: float = 5.0,
    lift_m: float = 3.0,
    step_timeout_s: float = STEP_TIMEOUT_S,
) -> dict:
    """The flight. `grab_height_m` is the vehicle's height when its hook is at the bucket's handle."""
    report = {"height_m": height_m, "lift_m": lift_m, "grab_height_m": grab_height_m, "steps": [], "landed": False, "passed": False, "problem": ""}
    if not vehicle.is_simulator():
        report["problem"] = "this autopilot is not SITL: the recovery check arms and takes off, so it runs in the sim only"
        return report

    here = {"sample": None}

    def go(name: str, e: float, n: float, u: float, speed_mps: Optional[float] = None) -> bool:
        """Fly to a point and stay there SETTLE_S. False (and the problem said) if it is not reached in time.

        With `speed_mps` the point the vehicle is sent to is walked from where the vehicle is to the goal at
        that speed: a bucket on the hook is a pendulum, and the autopilot's own dash and stop to a far point
        swung it hard enough, in the first flight that carried one, to bring the vehicle down.
        """
        started, since = vehicle.now(), None
        origin = here["sample"]
        distance = 0.0 if origin is None else math.dist((origin.e, origin.n, origin.u), (e, n, u))
        allowed = step_timeout_s + (distance / speed_mps if speed_mps and origin is not None else 0.0)
        while vehicle.now() - started < allowed:
            if speed_mps and origin is not None and distance > 0.0:
                share = min(1.0, (vehicle.now() - started) * speed_mps / distance)
                vehicle.goto(origin.e + share * (e - origin.e), origin.n + share * (n - origin.n), origin.u + share * (u - origin.u))
            else:
                vehicle.goto(e, n, u)
            sample = vehicle.step()
            if not getattr(vehicle, "fresh", True):
                since = None  # no position has come since the last one: where the vehicle is, is not known
                continue
            here["sample"] = sample
            if not vehicle.is_armed():
                report["problem"] = f"the vehicle disarmed on the way to '{name}'"
                return False
            there = math.hypot(sample.e - e, sample.n - n) <= ARRIVE_M and abs(sample.u - u) <= LEVEL_M
            since = (since if since is not None else vehicle.now()) if there else None
            if since is not None and vehicle.now() - since >= SETTLE_S:
                report["steps"].append({"step": name, "t": round(vehicle.now(), 2), "e": round(sample.e, 3), "n": round(sample.n, 3), "u": round(sample.u, 3)})
                return True
        report["problem"] = f"'{name}' was not reached within {allowed:g} s"
        return False

    def wait(seconds: float) -> None:
        until = vehicle.now() + seconds
        while vehicle.now() < until:
            vehicle.step()

    vehicle.takeoff(height_m)
    (be, bn), (re_, rn) = bucket_en, return_en
    # Each step is a point to reach (and, with the bucket on the hook, how fast to go there), or a gripper
    # command (True: grab, False: release).
    steps = [
        ("over the bucket", (be, bn, height_m)),
        ("hook at the handle", (be, bn, grab_height_m)),
        ("grab commanded", True),
        ("lifted", (be, bn, lift_m, CARRY_SPEED_MPS)),
        ("over the return point", (re_, rn, lift_m, CARRY_SPEED_MPS)),
        ("bucket just off the ground", (re_, rn, grab_height_m + SET_DOWN_CLEARANCE_M, CARRY_SPEED_MPS)),
        ("release commanded", False),
        ("clear above", (re_, rn, lift_m)),
        ("clear aside", (re_ + CLEAR_ASIDE_M, rn, lift_m)),
    ]
    flown = True
    for name, what in steps:
        if isinstance(what, bool):
            vehicle.gripper(what)
            wait(GRIP_WAIT_S)
            report["steps"].append({"step": name, "t": round(vehicle.now(), 2)})
        elif not go(name, *what):
            flown = False
            break
    vehicle.land()  # whatever happened, come down
    deadline = vehicle.now() + LAND_TIMEOUT_S
    while vehicle.is_armed() and vehicle.now() < deadline:
        vehicle.step()
    report["landed"] = not vehicle.is_armed()
    if flown and not report["landed"]:
        report["problem"] = f"the vehicle had not landed and disarmed {LAND_TIMEOUT_S:g} s after the flight"
    report["passed"] = bool(flown) and report["landed"]
    return report


def plan_from(run_dir: Path, return_point: str = "takeoff") -> dict:
    """Where the bucket and the return point are, and how low the hook must come, from a built run folder.

    `return_point` is the judge's (the scenario's `judge: {return_point: ...}`), which the build hands the tool."""
    truth = json.loads((Path(run_dir) / "truth.json").read_text())
    manifest = json.loads((Path(run_dir) / "scenario.json").read_text())
    bucket = truth.get("bucket")
    if not bucket:
        raise ValueError("the run's truth has no bucket: the recovery check needs a scenario with one (`bucket:`)")
    settings = manifest.get("payload_bridge") or {}
    hook = settings.get("hook_offset_m")
    if not hook:
        raise ValueError("the run's vehicle has no hook: the recovery check needs sensor_profile: recovery")
    if return_point not in truth["return_points"]:
        raise ValueError(f"the run's truth has no return point '{return_point}' (it has {sorted(truth['return_points'])})")
    point = truth["return_points"][return_point]
    return {
        "bucket_en": (float(bucket["e"]), float(bucket["n"])),
        "return_en": (float(point["e"]), float(point["n"])),
        # The handle's height above the ground, and the hook's tip under the vehicle's origin. The sim's local
        # frame has its zero at the ground the vehicle stands on, not at the vehicle's origin (a first flight,
        # commanded to 5 m, held a true 5.00 m), so this is the height to command.
        "grab_height_m": float(settings.get("handle_height_m", 0.32)) - float(hook[2]),
    }


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Grab the bucket, lift it, bring it back and set it down, in the sim.")
    parser.add_argument("--site", type=Path, required=True)
    parser.add_argument("--run-dir", type=Path, required=True, help="the built run folder (truth.json, scenario.json)")
    parser.add_argument("--url", default="udpin:127.0.0.1:14554", help="pymavlink connection (the sim's driver port: the payload bridge has the tools port)")
    parser.add_argument("--height", type=float, default=5.0)
    parser.add_argument("--lift", type=float, default=3.0)
    parser.add_argument("--return-point", default="takeoff", help="where the bucket is brought: the judge's return point")
    parser.add_argument("--out", type=Path, help="write the report as JSON")
    args = parser.parse_args(argv)
    try:
        site = site_file.load_site(args.site)
        plan = plan_from(args.run_dir, args.return_point)
    except (site_file.SiteError, OSError, KeyError, ValueError) as error:
        print(f"recovery_check: {error}", file=sys.stderr)
        return 2
    try:
        from fence_check import MavlinkVehicle

        from fence_check import Sample

        class HookVehicle(MavlinkVehicle):
            """The vehicle seen in the frame it is commanded in: the autopilot's local position (LOCAL_POSITION_NED).
            The check's points are local targets, and arriving is judged where they are given. (The global
            position's height is above home, which is not the local frame's zero.)"""

            def gripper(self, grab: bool) -> None:
                self._command(self._mavutil.mavlink.MAV_CMD_DO_GRIPPER, 1, 1 if grab else 0)

            def land(self) -> None:
                self.master.set_mode("LAND")

            fresh = True  # whether the last step's sample was a new position
            _boot_s = None  # the autopilot's own clock at the last position: the sim's time

            def now(self) -> float:
                """Seconds on the autopilot's clock, which in the sim is the sim's: the carry speed and the waits
                are then the same flight however fast the sim runs. Before the first position, the wall's."""
                return self._boot_s if self._boot_s is not None else time.monotonic() - self.t0

            def step(self) -> Sample:
                mav = self._mavutil.mavlink
                if time.monotonic() - self._rates_at > 2.0:
                    self._command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50000)
                    self._command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAVLINK_MSG_ID_HEARTBEAT, 200000)
                    self._rates_at = time.monotonic()
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    message = self.master.recv_match(type=["LOCAL_POSITION_NED", "HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.5)
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
                        self._boot_s = float(message.time_boot_ms) / 1000.0
                        self.fresh = True
                        self.last = Sample(self.now(), float(message.y), float(message.x), -float(message.z), self.mode)
                        return self.last
                if self.last is None:
                    raise RuntimeError("no local position from the autopilot")
                # Nothing new in two seconds: the old position is handed back marked as such, and the flight's
                # steps do not count it as being anywhere.
                self.fresh = False
                self._boot_s = (self._boot_s or 0.0) + 2.0  # the clock still moves, so a step's time limit still ends it
                return Sample(self.now(), self.last.e, self.last.n, self.last.u, self.mode)

        report = run_check(HookVehicle(args.url, site), height_m=args.height, lift_m=args.lift, **plan)
    except ImportError:
        print("recovery_check: pymavlink is not installed here (in the sim container, run with PYTHONNOUSERSITE unset)", file=sys.stderr)
        return 2
    except RuntimeError as error:
        print(f"recovery_check: {error}", file=sys.stderr)
        return 2
    if args.out:
        args.out.write_text(json.dumps(report, indent=2) + "\n")
    for step in report["steps"]:
        print(f"step: {step}")
    print(f"landed: {report['landed']}")
    print("RESULT: PASS" if report["passed"] else f"RESULT: FAIL ({report['problem']})")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
