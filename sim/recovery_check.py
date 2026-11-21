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
from drone_mission_core import recovery_return  # noqa: E402  (REC-7's rules: the way back's frame, and the set-down)

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
SET_DOWN_TIMEOUT_S = 90.0  # the rule's whole set-down, from over the return point to done
VERIFY_EXTRA_S = 5.0  # the attach check's hold may run this much over, waiting for fresh throttle
GLOBAL_WAIT_STEPS = 200  # positions waited through, before takeoff, for the first global one (10 s at 20 Hz)
LANDED_RELEASE_S = 5.0  # after a landing with the bucket, how long the rule has to let go on the ground
RANGE_MAX_AGE_S = 1.0  # the tool's own "the rangefinder is alive"; the rule asks more of the reading's age
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
    return_frame: Optional[str] = None,
    return_target: Optional[recovery_return.ReturnTarget] = None,
    set_down_rule: bool = False,
) -> dict:
    """The flight. `grab_height_m` is the vehicle's height when its hook is at the bucket's handle.

    `pendulum_m` (the hinge to the middle of what hangs) carries with REC-4's swing shaping: each carried leg
    starts and stops over one pendulum period, at up to `carry_speed_mps`. Without it the commanded point is
    walked at `carry_speed_mps`. `attach` is REC-4's attach check, fed the hover throttle before the grab and
    after the lift: its verdict goes in the report, and "not attached", or a check that could not be made for want
    of a fresh throttle, ends the flight there.

    `return_frame` ("global" or "local") flies the way back by REC-7's rule: the goal is `return_target` (the
    takeoff when not given), "global" stands for GPS trusted (a global setpoint, walked along the leg) and "local"
    for untrusted (a local one). `set_down_rule` sets the bucket down by REC-7's rule in place of the scripted
    lowering and release: down slowly until the throttle says the bucket stands, the release guard, the gripper's
    acknowledgement, and a climb to see that it stayed behind. It needs `attach` (the two throttles are its).
    """
    report = {"height_m": height_m, "lift_m": lift_m, "grab_height_m": grab_height_m, "carry_speed_mps": carry_speed_mps, "shaped": pendulum_m is not None, "steps": [], "landed": False, "passed": False, "problem": ""}
    if not vehicle.is_simulator():
        report["problem"] = "this autopilot is not SITL: the recovery check arms and takes off, so it runs in the sim only"
        return report

    if isinstance(carry_speed_mps, bool) or not isinstance(carry_speed_mps, (int, float)) or not math.isfinite(carry_speed_mps) or carry_speed_mps <= 0.0:
        report["problem"] = f"the carry's speed must be a number of m/s above 0, got {carry_speed_mps!r}"  # said before takeoff, not found in the air
        return report

    if return_frame not in (None, recovery_return.GLOBAL, recovery_return.LOCAL):
        report["problem"] = f"return_frame is 'global' or 'local', got {return_frame!r}"
        return report
    if set_down_rule and attach is None:
        report["problem"] = "the set-down by rule needs the attach check: the empty and the loaded hover's throttle are what it feels by"
        return report
    report["return_frame"] = return_frame

    here = {"sample": None}
    takeoff_pair = None
    if return_frame is not None:
        # Where the vehicle stands before it takes off, in both frames: what a return by the rule rests on.
        ground = vehicle.step()
        lat = lon = None
        for _ in range(GLOBAL_WAIT_STEPS if hasattr(vehicle, "global_position") else 0):  # (the pair is taken on the ground, where GPS is trusted, whichever frame the way back is flown in)
            lat, lon = vehicle.global_position()  # (asked for with the first step: it takes a moment to come)
            if lat is not None:
                break
            ground = vehicle.step()
        if return_frame == recovery_return.GLOBAL and lat is None:
            report["problem"] = "no global position from the autopilot before takeoff: a global return has nothing to go by"
            return report
        takeoff_pair = recovery_return.Anchor(ground.e, ground.n, lat, lon)
        # A global setpoint's height is above home; a local one's is in the local frame, whose zero is not home's
        # (a flight sent to "3 m above home" flew 0.18 m higher than one sent to a local 3 m). Standing on the
        # ground, the difference is read off, and every global setpoint's height is given in home's terms.
        above_home = vehicle.height_above_home_m() if return_frame == recovery_return.GLOBAL and hasattr(vehicle, "height_above_home_m") else None
        report["home_up_m"] = home_up = 0.0 if above_home is None else round(ground.u - above_home, 3)  # home's height in the local frame
        if return_target is not None and return_target.kind == recovery_return.LATLON and lat is None:
            report["problem"] = "no global position from the autopilot before takeoff: a published return point cannot be placed"
            return report

    def go(name: str, e: float, n: float, u: float, speed_mps: Optional[float] = None, send=None) -> bool:
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
            elif speed_mps and origin is not None and distance > 0.0:
                share = min(1.0, (vehicle.now() - started) * speed_mps / distance)
            else:
                share = 1.0
            if send is not None:
                send(share)  # the rule's own setpoint that far along (REC-7)
            elif share < 1.0:
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

    ages = report["throttle_ages"] = {"samples": 0, "stale": 0, "oldest_s": 0.0}  # of the throttle fed to the attach check
    heard = report["throttle_log"] = []  # [time, throttle, age] of each: what the two hovers were judged on

    def wait(seconds: float, sample=None, until_true=None, at_most_s: float = 0.0) -> None:
        """Stay put for a while. `sample` is given the throttle heard on each step (the attach check's hovers);
        with `until_true` the wait goes on past `seconds`, up to `at_most_s` in all, until it holds."""
        started = vehicle.now()
        while vehicle.now() - started < seconds or (until_true is not None and not until_true() and vehicle.now() - started < at_most_s):
            vehicle.step()
            if sample is not None and hasattr(vehicle, "throttle"):
                age = vehicle.throttle_age_s() if hasattr(vehicle, "throttle_age_s") else 0.0
                ages["samples"] += 1
                heard.append([round(vehicle.now(), 2), vehicle.throttle(), None if not isinstance(age, (int, float)) else round(age, 3)])
                if isinstance(age, (int, float)):
                    ages["oldest_s"] = round(max(ages["oldest_s"], age), 3)
                    ages["stale"] += age > recovery_grip.SIGNAL_MAX_AGE_S
                sample(vehicle.throttle(), vehicle.now(), age)  # a throttle that has stopped coming is not a sample

    def carry_back(name: str, u: float) -> bool:
        """The way back by REC-7's rule: the goal and its frame from `return_goal`, the setpoint walked along the leg."""
        origin = here["sample"]
        goal = recovery_return.return_goal(return_target or recovery_return.ReturnTarget(recovery_return.TAKEOFF), gps_trusted=return_frame == recovery_return.GLOBAL, takeoff=takeoff_pair)
        lat, lon = vehicle.global_position() if goal.frame == recovery_return.GLOBAL else (None, None)
        if goal.frame == recovery_return.GLOBAL and lat is None:
            report["problem"] = f"no global position from the autopilot at the start of '{name}'"
            return False
        start = recovery_return.Anchor(origin.e, origin.n, lat, lon)
        report["return"] = {"target": (return_target.kind if return_target is not None else recovery_return.TAKEOFF), "frame": goal.frame, "e": round(goal.e, 3), "n": round(goal.n, 3), "lat": goal.lat, "lon": goal.lon, "length_m": round(recovery_return.leg_length_m(start, goal), 2), "reason": goal.reason}
        sent = {"global": 0, "local": 0}

        def send(share: float) -> None:
            point = recovery_return.setpoint_along(start, goal, share)
            sent[point.frame] += 1
            if point.frame == recovery_return.GLOBAL:
                vehicle.goto_global(point.lat, point.lon, recovery_return.global_height_m(u, home_up))
            else:
                vehicle.goto(point.e, point.n, u)

        reached = go(name, goal.e, goal.n, u, carry_speed_mps, send)
        report["return"]["setpoints"] = sent
        return reached

    def set_down_by_rule() -> bool:
        """REC-7's set-down, over where the vehicle is. True when the rule is done; the report has what it did."""
        over = here["sample"]
        # The touch height is the rangefinder's own reading from when the hook was at the handle of the bucket
        # standing on the ground (it reads 0.19 m under the local height in the sim); without one, the local height.
        touch = report["touch_height_m"] = round(grabbed.get("range_m") or grab_height_m, 3)
        rule = recovery_return.SetDown(recovery_return.SetDownPlan(touch_height_m=touch), attach.baseline_throttle, attach.loaded_throttle)
        told = {"u": over.u, "t": vehicle.now(), "ack": None}
        log = report["set_down"] = {"phases": [], "released": False, "confirmed": False, "release_attempts": 0, "set_downs": 0, "reason": "", "land_first": False, "readings": 0, "stale_heights": 0, "slow_steps": 0}
        deadline = vehicle.now() + SET_DOWN_TIMEOUT_S
        landing, disarmed_at = False, None
        while vehicle.now() < deadline:
            sample = vehicle.step()
            now = vehicle.now()
            height, height_age = vehicle.range_m()
            landed, landed_age = vehicle.landed()
            code = vehicle.gripper_ack()
            ack = recovery_grip.Ack(rule.attempt, code) if code is not None and told["ack"] == rule.attempt else None
            log["readings"] += 1
            log["stale_heights"] += not (isinstance(height_age, (int, float)) and height_age <= recovery_grip.SIGNAL_MAX_AGE_S)
            log["slow_steps"] += now - told["t"] > 0.15
            answer = rule.update(recovery_return.SetDownReading(now, height, height_age, height is not None and height_age is not None and height_age <= RANGE_MAX_AGE_S, vehicle.throttle(), vehicle.throttle_age_s(), landed, landed_age, ack))
            if not log["phases"] or log["phases"][-1]["phase"] != answer.phase:
                log["phases"].append({"phase": answer.phase, "t": round(now, 2), "u": round(sample.u, 3), "range_m": None if height is None else round(height, 3), "throttle": vehicle.throttle()})
            if answer.open_gripper:
                vehicle.gripper(False)  # (this clears the acknowledgement the vehicle holds: the next one is this command's)
                told["ack"] = answer.attempt
                log["released_at"] = {"t": round(now, 2), "u": round(sample.u, 3), "range_m": None if height is None else round(height, 3)}
            if answer.action == recovery_return.LAND and not landing:
                vehicle.land()
                landing = log["land_first"] = True
            elif not landing:
                told["u"] += answer.rate_up_mps * (now - told["t"])
                vehicle.goto(over.e, over.n, told["u"])  # the place over the ground is held; only the height is walked
            told["t"] = now
            log.update(released=rule.released, confirmed=rule.confirmed, release_attempts=rule.release_attempts, set_downs=rule.set_downs, reason=rule.last_reason)
            if answer.action == recovery_return.DONE:
                here["sample"] = sample
                report["steps"].append({"step": "set down by the rule", "t": round(now, 2), "e": round(sample.e, 3), "n": round(sample.n, 3), "u": round(sample.u, 3)})
                return True
            if answer.action == recovery_return.FAULT:
                report["problem"] = f"the set-down ended in a fault: {answer.reason or rule.last_reason}"
                return False
            if landing and not vehicle.is_armed():
                # Down and disarmed: the rule has a few seconds more to hear "landed" and let go on the ground.
                disarmed_at = disarmed_at if disarmed_at is not None else now
                if now - disarmed_at >= LANDED_RELEASE_S:
                    report["problem"] = f"landed with the bucket still on the hook: {rule.last_reason}"
                    return False
        report["problem"] = f"the set-down by the rule was not done in {SET_DOWN_TIMEOUT_S:g} s ({rule.phase}: {rule.last_reason})"
        return False

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
    grabbed = {}  # what the rangefinder read at the grab
    for name, what in steps:
        if set_down_rule and name in ("bucket just off the ground", "release commanded"):
            if name == "release commanded" and not set_down_by_rule():
                flown = False
                break
            continue  # the rule lowers and lets go, in place of these two scripted steps
        if report.get("set_down", {}).get("land_first") and name.startswith("clear"):
            continue  # it landed to let go: there is nothing to clear away from in the air
        if return_frame is not None and name == "over the return point":
            if not carry_back(name, lift_m):
                flown = False
                break
            continue
        if isinstance(what, bool):
            if what and attach is not None:
                # The empty hover, the hook at the handle: its two seconds, and on until there is fresh throttle to go by.
                wait(BASELINE_HOVER_S, attach.baseline_sample, lambda: attach.baseline_throttle is not None, BASELINE_HOVER_S + VERIFY_EXTRA_S)
            if what and hasattr(vehicle, "range_m"):
                height, age = vehicle.range_m()
                if height is not None and age is not None and age <= RANGE_MAX_AGE_S:
                    grabbed["range_m"] = height
            vehicle.gripper(what)
            wait(GRIP_WAIT_S)
            report["steps"].append({"step": name, "t": round(vehicle.now(), 2)})
        elif not go(name, *what):
            flown = False
            break
        elif name == "lifted" and attach is not None:
            # The hold the attach check is made in: its 1.5 s, and on until the check has that much fresh throttle.
            wait(recovery_grip.VERIFY_HOLD_S + 0.3, attach.loaded_sample, lambda: attach.ready, recovery_grip.VERIFY_HOLD_S + VERIFY_EXTRA_S)
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
    parser.add_argument("--return-frame", choices=["global", "local"], help="fly the way back by REC-7's rule: a global setpoint (GPS trusted) or a local one (untrusted)")
    parser.add_argument("--return-target", choices=["takeoff", "latlon"], default="takeoff", help="with --return-frame: the return point as the takeoff itself, or as a published latitude and longitude (the site's, for the judge's return point)")
    parser.add_argument("--set-down-rule", action="store_true", help="set the bucket down by REC-7's rule: felt by the throttle, let go through the release guard, confirmed by a climb (needs --attach-check)")
    parser.add_argument("--out", type=Path, help="write the report as JSON")
    args = parser.parse_args(argv)
    rule_signals = bool(args.return_frame or args.set_down_rule)
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
                self._gripper_ack = None  # the next acknowledgement is this command's
                self._command(self._mavutil.mavlink.MAV_CMD_DO_GRIPPER, 1, 1 if grab else 0)

            _gripper_ack = None  # the result of the newest COMMAND_ACK for a gripper command since the last one sent
            _range = None  # (metres, when it came): the newest RANGEFINDER
            _landed = None  # (on the ground or not, when it came): the newest EXTENDED_SYS_STATE
            _global = None  # (latitude, longitude, height above home): the newest GLOBAL_POSITION_INT

            def gripper_ack(self):
                return self._gripper_ack

            def range_m(self):
                """The rangefinder's height above the ground and its age, or (None, None)."""
                return (None, None) if self._range is None else (self._range[0], self._age(self._range[1]))

            def landed(self):
                """The flight controller's landed state and its age, or (None, None)."""
                return (None, None) if self._landed is None else (self._landed[0], self._age(self._landed[1]))

            def global_position(self):
                return (None, None) if self._global is None else self._global[:2]

            def height_above_home_m(self):
                return None if self._global is None else self._global[2]

            def goto_global(self, lat: float, lon: float, up_m: float) -> None:
                """A global setpoint: latitude and longitude, and the height above home."""
                mav = self._mavutil.mavlink
                self.master.mav.set_position_target_global_int_send(
                    0, self.system, 1, mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b0000111111111000,
                    int(round(lat * 1.0e7)), int(round(lon * 1.0e7)), up_m, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                )  # fmt: skip

            def land(self) -> None:
                self.master.set_mode("LAND")

            fresh = True  # whether the last step's sample was a new position
            _boot_s = None  # the autopilot's own clock at the last position: the sim's time
            _throttle = None  # the newest VFR_HUD throttle, 0 to 1
            _throttle_at = None  # and when it came, on the autopilot's clock (see _fine_now)

            def throttle(self):
                return self._throttle

            _position_wall = None  # the wall's clock at the last position
            _rate = 1.0  # the autopilot's seconds per wall second (the sim's real-time factor), from the positions

            def _fine_now(self) -> float:
                """The autopilot's clock between positions: its time at the last one, and the wall's time since then
                at the sim's own rate. (`now()` only moves with each position, so a reading that came just after
                one would look a whole step old at the next; the wall's clock alone runs at another rate.)"""
                if self._boot_s is None or self._position_wall is None:
                    return time.monotonic() - self.t0
                return self._boot_s + (time.monotonic() - self._position_wall) * self._rate

            def _age(self, came_at):
                """How long ago a message came, in the autopilot's seconds."""
                return None if came_at is None else max(0.0, self._fine_now() - came_at)

            def throttle_age_s(self):
                return self._age(self._throttle_at)

            def now(self) -> float:
                """Seconds on the autopilot's clock, which in the sim is the sim's: the carry speed and the waits
                are then the same flight however fast the sim runs. Before the first position, the wall's."""
                return self._boot_s if self._boot_s is not None else time.monotonic() - self.t0

            def step(self) -> Sample:
                mav = self._mavutil.mavlink
                # The driver's port is one of MAVProxy's outputs, and MAVProxy asks the autopilot for its own stream
                # rates (4 Hz) again and again: until this tool next asks, positions, throttle and rangefinder all
                # come at 4 Hz. A flight by the rules, which want readings under 0.2 s old, asks every half second.
                if time.monotonic() - self._rates_at > (0.5 if rule_signals else 2.0):
                    self._command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50000)
                    self._command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAVLINK_MSG_ID_VFR_HUD, 100000)
                    if rule_signals:  # what REC-7's rules read, asked for only by a flight that uses them
                        self._command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAVLINK_MSG_ID_RANGEFINDER, 50000)
                        self._command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 200000)
                        self._command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000)
                    self._command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAVLINK_MSG_ID_HEARTBEAT, 200000)
                    self._rates_at = time.monotonic()
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    message = self.master.recv_match(type=["LOCAL_POSITION_NED", "HEARTBEAT", "STATUSTEXT", "VFR_HUD", "RANGEFINDER", "EXTENDED_SYS_STATE", "GLOBAL_POSITION_INT", "COMMAND_ACK"], blocking=True, timeout=0.5)
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
                        self._throttle_at = self._fine_now()
                    elif kind == "RANGEFINDER":
                        self._range = (float(message.distance), self._fine_now())
                    elif kind == "EXTENDED_SYS_STATE":
                        self._landed = (message.landed_state == mav.MAV_LANDED_STATE_ON_GROUND, self._fine_now())
                    elif kind == "GLOBAL_POSITION_INT":
                        self._global = (message.lat / 1.0e7, message.lon / 1.0e7, message.relative_alt / 1000.0)
                    elif kind == "COMMAND_ACK":
                        if message.command == mav.MAV_CMD_DO_GRIPPER:
                            self._gripper_ack = int(message.result)
                    else:
                        boot, wall = float(message.time_boot_ms) / 1000.0, time.monotonic()
                        if self._boot_s is not None and self._position_wall is not None and 0.02 < wall - self._position_wall < 1.0 and boot > self._boot_s:
                            self._rate += 0.1 * (min(4.0, (boot - self._boot_s) / (wall - self._position_wall)) - self._rate)
                        self._boot_s, self._position_wall = boot, wall
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
        target = None
        if args.return_frame and args.return_target == "latlon":
            from target_placement import latlon_for

            east, north = plan["return_en"]  # the published point: the return point's own latitude and longitude, from the site's origin
            target = recovery_return.parse_return_target({"latlon": list(latlon_for(site.origin.latitude_deg, site.origin.longitude_deg, north, east))})
        pendulum_m = plan["pendulum_m"]
        report = run_check(
            HookVehicle(args.url, site), height_m=args.height, lift_m=args.lift, carry_speed_mps=args.carry_speed,
            pendulum_m=pendulum_m if args.shaped else None, attach=attach, return_frame=args.return_frame, return_target=target, set_down_rule=args.set_down_rule,
            **{key: plan[key] for key in ("bucket_en", "return_en", "grab_height_m")},
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
    for key in ("return", "set_down"):
        if report.get(key):
            print(f"{key}: {report[key]}")
    print(f"landed: {report['landed']}")
    print("RESULT: PASS" if report["passed"] else f"RESULT: FAIL ({report['problem']})")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
