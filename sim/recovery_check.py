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
CORE_DIR = SIM_DIR.parent / "src" / "drone_mission_core"
if str(CORE_DIR) not in sys.path:
    sys.path.append(str(CORE_DIR))

import site_file  # noqa: E402
from drone_mission_core import recovery_grip  # noqa: E402  (REC-4's rules: the attach check and the shaped leg)

ARRIVE_M = 0.08  # over a point: this close to it sideways
LEVEL_M = 0.06  # at a height: this close to it
SETTLE_S = 2.0  # and for this long
STEP_TIMEOUT_S = 90.0
GRIP_WAIT_S = 2.0  # after a gripper command, before moving
SET_DOWN_CLEARANCE_M = 0.05  # the bucket's base above the ground when it is let go
CLEAR_ASIDE_M = 2.0  # sideways from the bucket before landing
LAND_TIMEOUT_S = 120.0
CARRY_SPEED_MPS = 1.0  # with the bucket on the hook the point the vehicle is sent to moves no faster than this
BASELINE_HOVER_S = 2.0  # the hover before the grab whose throttle the attach check compares with
PENDULUM_BELOW_HOOK_M = 0.17  # the middle of the bucket hangs this far under the hook's tip, less the hinge's own 0.05 m under the body


def run_check(
    vehicle,
    bucket_en: Tuple[float, float],
    return_en: Tuple[float, float],
    grab_height_m: float,
    height_m: float = 5.0,
    lift_m: float = 3.0,
    step_timeout_s: float = STEP_TIMEOUT_S,
    carry_speed_mps: float = CARRY_SPEED_MPS,
    pendulum_m: Optional[float] = None,
    attach: Optional[recovery_grip.AttachCheck] = None,
) -> dict:
    """The flight. `grab_height_m` is the vehicle's height when its hook is at the bucket's handle.

    `pendulum_m` (the hinge to the middle of what hangs) carries with REC-4's swing shaping: each carried leg
    starts and stops over one pendulum period, at up to `carry_speed_mps`. Without it the commanded point is
    walked at `carry_speed_mps`. `attach` is REC-4's attach check, fed the hover throttle before the grab and
    after the lift: its verdict goes in the report, and "not attached", or a check that could not be made for want
    of a fresh throttle, ends the flight there.
    """
    report = {"height_m": height_m, "lift_m": lift_m, "grab_height_m": grab_height_m, "carry_speed_mps": carry_speed_mps, "shaped": pendulum_m is not None, "steps": [], "landed": False, "passed": False, "problem": ""}
    if not vehicle.is_simulator():
        report["problem"] = "this autopilot is not SITL: the recovery check arms and takes off, so it runs in the sim only"
        return report

    if isinstance(carry_speed_mps, bool) or not isinstance(carry_speed_mps, (int, float)) or not math.isfinite(carry_speed_mps) or carry_speed_mps <= 0.0:
        report["problem"] = f"the carry's speed must be a number of m/s above 0, got {carry_speed_mps!r}"  # said before takeoff, not found in the air
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
        leg = recovery_grip.ShapedLeg(distance, speed_mps, pendulum_m) if speed_mps and pendulum_m and origin is not None and distance > 0.0 else None
        allowed = step_timeout_s + (leg.duration_s if leg is not None else distance / speed_mps if speed_mps and origin is not None else 0.0)
        while vehicle.now() - started < allowed:
            if leg is not None:
                share = leg.travelled_m(vehicle.now() - started) / distance  # a shaped start and a shaped stop
                vehicle.goto(origin.e + share * (e - origin.e), origin.n + share * (n - origin.n), origin.u + share * (u - origin.u))
            elif speed_mps and origin is not None and distance > 0.0:
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

    def wait(seconds: float, sample=None) -> None:
        """Stay put for a while. `sample` is given the throttle heard on each step (the attach check's hovers)."""
        until = vehicle.now() + seconds
        while vehicle.now() < until:
            vehicle.step()
            if sample is not None and hasattr(vehicle, "throttle"):
                age = vehicle.throttle_age_s() if hasattr(vehicle, "throttle_age_s") else 0.0
                sample(vehicle.throttle(), vehicle.now(), age)  # a throttle that has stopped coming is not a sample

    vehicle.takeoff(height_m)
    (be, bn), (re_, rn) = bucket_en, return_en
    # Each step is a point to reach (and, with the bucket on the hook, how fast to go there), or a gripper
    # command (True: grab, False: release).
    steps = [
        ("over the bucket", (be, bn, height_m)),
        ("hook at the handle", (be, bn, grab_height_m)),
        ("grab commanded", True),
        ("lifted", (be, bn, lift_m, min(carry_speed_mps, CARRY_SPEED_MPS))),  # up and down, slowly whatever the carry's speed
        ("over the return point", (re_, rn, lift_m, carry_speed_mps)),
        ("bucket just off the ground", (re_, rn, grab_height_m + SET_DOWN_CLEARANCE_M, min(carry_speed_mps, CARRY_SPEED_MPS))),
        ("release commanded", False),
        ("clear above", (re_, rn, lift_m)),
        ("clear aside", (re_ + CLEAR_ASIDE_M, rn, lift_m)),
    ]
    flown = True
    for name, what in steps:
        if isinstance(what, bool):
            if what and attach is not None:
                wait(BASELINE_HOVER_S, attach.baseline_sample)  # the empty hover, the hook at the handle
            vehicle.gripper(what)
            wait(GRIP_WAIT_S)
            report["steps"].append({"step": name, "t": round(vehicle.now(), 2)})
        elif not go(name, *what):
            flown = False
            break
        elif name == "lifted" and attach is not None:
            wait(recovery_grip.VERIFY_HOLD_S + 0.3, attach.loaded_sample)  # the hold the attach check is made in
            verdict = attach.verdict()
            report["attach"] = {
                "attached": verdict.attached,
                "throttle_ratio": None if verdict.throttle_ratio is None else round(verdict.throttle_ratio, 4),
                "expected_ratio": round(verdict.expected_ratio, 4),
                "baseline_throttle": attach.baseline_throttle,
                "loaded_throttle": attach.loaded_throttle,
                "reason": verdict.reason,
            }
            if not verdict.attached:
                # Asked for and not passed. That holds too for a check that could not be made (no throttle heard, or
                # none fresh): a flight that was to be judged on it does not pass without it.
                made = attach.baseline_throttle is not None and attach.loaded_throttle is not None
                report["problem"] = f"the attach check says the bucket is not on the hook: {verdict.reason}" if made else f"the attach check was asked for and could not be made: {verdict.reason}"
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
        "vehicle_kg": float((manifest.get("vehicle") or {}).get("mass_kg", 0.0)),
        "load_kg": float(bucket.get("mass_kg", 0.0)),
        "pendulum_m": abs(float(hook[2])) + PENDULUM_BELOW_HOOK_M,
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
    parser.add_argument("--carry-speed", type=float, default=CARRY_SPEED_MPS, help="the speed of the carry to the return point, m/s")
    parser.add_argument("--shaped", action="store_true", help="carry with swing shaping: each leg starts and stops over one pendulum period (REC-4)")
    parser.add_argument("--attach-check", action="store_true", help="hover before the grab and after the lift, and judge by the throttle whether the bucket is on the hook (REC-4)")
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
            _throttle = None  # the newest VFR_HUD throttle, 0 to 1
            _throttle_at = None  # and when it came, on the autopilot's clock

            def throttle(self):
                return self._throttle

            def throttle_age_s(self):
                return None if self._throttle_at is None else self.now() - self._throttle_at

            def now(self) -> float:
                """Seconds on the autopilot's clock, which in the sim is the sim's: the carry speed and the waits
                are then the same flight however fast the sim runs. Before the first position, the wall's."""
                return self._boot_s if self._boot_s is not None else time.monotonic() - self.t0

            def step(self) -> Sample:
                mav = self._mavutil.mavlink
                if time.monotonic() - self._rates_at > 2.0:
                    self._command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50000)
                    self._command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAVLINK_MSG_ID_VFR_HUD, 100000)
                    self._command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAVLINK_MSG_ID_HEARTBEAT, 200000)
                    self._rates_at = time.monotonic()
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    message = self.master.recv_match(type=["LOCAL_POSITION_NED", "HEARTBEAT", "STATUSTEXT", "VFR_HUD"], blocking=True, timeout=0.5)
                    if message is None:
                        continue
                    kind = message.get_type()
                    if kind == "HEARTBEAT":
                        if message.get_srcComponent() == 1 and message.autopilot != mav.MAV_AUTOPILOT_INVALID:
                            self.mode = self._mavutil.mode_string_v10(message)
                            self.armed = bool(message.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
                    elif kind == "STATUSTEXT":
                        print(f"  [{self.now():7.2f}] autopilot: {message.text}", flush=True)
                    elif kind == "VFR_HUD":
                        self._throttle = float(message.throttle) / 100.0  # percent
                        self._throttle_at = self.now()
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

        attach = recovery_grip.AttachCheck(plan["vehicle_kg"], plan["load_kg"]) if args.attach_check and plan["vehicle_kg"] > 0.0 else None
        pendulum_m = plan["pendulum_m"]
        report = run_check(
            HookVehicle(args.url, site), height_m=args.height, lift_m=args.lift, carry_speed_mps=args.carry_speed,
            pendulum_m=pendulum_m if args.shaped else None, attach=attach, **{key: plan[key] for key in ("bucket_en", "return_en", "grab_height_m")},
        )  # fmt: skip
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
    if report.get("attach"):
        print(f"attach check: {report['attach']}")
    print(f"landed: {report['landed']}")
    print("RESULT: PASS" if report["passed"] else f"RESULT: FAIL ({report['problem']})")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
