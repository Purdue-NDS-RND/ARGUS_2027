#!/usr/bin/env python3
"""Trip a flight-controller failsafe in the sim and measure what the autopilot does (PLT-08).

The 2027 baseline (platform/fcu/argus_2027.param) turns the failsafes on. This
flies SITL with the baseline's failsafe settings loaded, takes away the thing
a failsafe watches, and checks that the mode the baseline asks for begins in
time and that the gripper is not let go (FS_OPTIONS bit 5 clear):

  battery          the voltage falls under BATT_LOW_VOLT, then under BATT_CRT_VOLT
  radio            the receiver stops sending (SIM_RC_FAIL)
  ground_station   the ground station's heartbeat is no longer heard

  sim/run_scenarios.sh hover/failsafe_battery    # and hover/failsafe_radio, hover/failsafe_ground_station

The scenario starts SITL with the baseline's values (a test holds the scenario
files to the baseline file); this tool reads them back from the autopilot
before it flies, so a pass is a pass of the baseline and not of SITL's
defaults. It arms and takes off, so it is for the sim only: it refuses to fly
unless the autopilot answers for a SITL-only parameter. The fence's own check
is sim/fence_check.py; the EKF failsafe is not tripped here (its action is
2026's and unchanged).

The ground station is "lost" by telling the autopilot its ground station is a
system that does not exist (SYSID_MYGCS): the sim's MAVProxy goes on sending,
and the autopilot stops counting it. That is the same timer as a dead radio.

Exit code 0 passed, 1 failed, 2 could not run. The verdict needs no pymavlink.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import site_file  # noqa: E402
from fc_param_check import read_params  # noqa: E402
from fence_check import MavlinkVehicle, Sample  # noqa: E402

RTL, LAND = "RTL", "LAND"
MARGIN_S = 5.0  # over the failsafe's own timer: the mode must have begun by then
AWAY_M = 15.0  # east of the takeoff point, so that an RTL is a flight and not a landing on the spot
UNDER_V = 0.5  # how far under a threshold the simulated battery is put
HEALTHY_OVER_V = 4.0  # over BATT_LOW_VOLT: the battery the flight starts on
PWM_NEAR = 50.0  # how close to GRIP_GRAB the gripper's output must read to count as closed
NOBODY = 77  # a MAVLink system id nothing in the sim has
# What each action parameter's value makes the autopilot do (ArduCopter 4.6). A value not here is not judged.
ACTIONS = {
    "BATT_FS_LOW_ACT": {1: LAND, 2: RTL},
    "BATT_FS_CRT_ACT": {1: LAND, 2: RTL},
    "FS_THR_ENABLE": {1: RTL, 3: LAND},
    "FS_GCS_ENABLE": {1: RTL, 5: LAND},
}
# The baseline's parameters each failsafe is flown with: read back from the autopilot before the flight.
WATCHED = {
    "battery": ("BATT_LOW_VOLT", "BATT_CRT_VOLT", "BATT_LOW_TIMER", "BATT_FS_LOW_ACT", "BATT_FS_CRT_ACT", "FS_OPTIONS"),
    "radio": ("FS_THR_ENABLE", "FS_THR_VALUE", "FS_OPTIONS"),
    "ground_station": ("FS_GCS_ENABLE", "FS_GCS_TIMEOUT", "FS_OPTIONS"),
}
GRIPPER = ("GRIP_ENABLE", "GRIP_GRAB", "GRIP_RELEASE")


class CheckError(ValueError):
    """The baseline cannot be flown as a failsafe check. The message says why."""


def plan(failsafe: str, baseline: dict) -> list[dict]:
    """The steps of one check: what is changed on SITL, the mode that must follow and how long it may take."""
    if failsafe not in WATCHED:
        raise CheckError(f"no failsafe called '{failsafe}' (there are: {', '.join(WATCHED)})")
    missing = [name for name in WATCHED[failsafe] if name not in baseline]
    if missing:
        raise CheckError(f"the baseline has no {', '.join(missing)}")

    def mode_for(name: str) -> str:
        value = baseline[name]
        if value != int(value) or int(value) not in ACTIONS[name]:
            raise CheckError(f"{name} is {value:g}: this check does not know what mode that action is (it knows {sorted(ACTIONS[name])})")
        return ACTIONS[name][int(value)]

    if failsafe == "battery":
        low, critical, timer = baseline["BATT_LOW_VOLT"], baseline["BATT_CRT_VOLT"], baseline["BATT_LOW_TIMER"]
        return [
            {"name": "battery low", "set": {"SIM_BATT_VOLTAGE": low - UNDER_V}, "expect": mode_for("BATT_FS_LOW_ACT"), "within_s": timer + MARGIN_S},
            {"name": "battery critical", "set": {"SIM_BATT_VOLTAGE": critical - UNDER_V}, "expect": mode_for("BATT_FS_CRT_ACT"), "within_s": timer + MARGIN_S},
        ]
    if failsafe == "radio":
        return [{"name": "radio lost", "set": {"SIM_RC_FAIL": 1.0}, "expect": mode_for("FS_THR_ENABLE"), "within_s": MARGIN_S}]
    return [{"name": "ground station lost", "set": {"SYSID_MYGCS": float(NOBODY)}, "expect": mode_for("FS_GCS_ENABLE"), "within_s": baseline["FS_GCS_TIMEOUT"] + MARGIN_S}]


def judge_step(step: dict, mode_before: str, samples: list[Sample], t_trigger: float) -> dict:
    """One step's verdict from the modes seen after its trigger."""
    result = {"name": step["name"], "expect": step["expect"], "within_s": step["within_s"], "mode_before": mode_before, "delay_s": None, "passed": False, "problem": ""}
    if mode_before == step["expect"]:
        result["problem"] = f"the vehicle was already in {mode_before} before the failsafe was tripped: nothing was shown"
        return result
    changed = next((s for s in samples if s.t >= t_trigger and s.mode != mode_before), None)
    if changed is None:
        result["problem"] = f"the mode stayed {mode_before} for {step['within_s']:g} s: the failsafe did not act"
        return result
    result["mode_seen"], result["delay_s"] = changed.mode, changed.t - t_trigger
    if changed.mode != step["expect"]:
        result["problem"] = f"the mode became {changed.mode}, not {step['expect']}"
    elif result["delay_s"] > step["within_s"]:
        result["problem"] = f"{step['expect']} began {result['delay_s']:.1f} s after the trigger (limit {step['within_s']:g} s)"
    else:
        result["passed"] = True
    return result


def run_check(vehicle, baseline: dict, failsafe: str, height_m: float = 10.0, land_timeout_s: float = 240.0) -> dict:
    """The flight. `vehicle` is fence_check's vehicle with read_param, set_param, grab and servo_pwm added."""
    report = {"failsafe": failsafe, "steps": [], "loaded": {}, "gripper_held": None, "landed_and_disarmed": False, "passed": False, "problem": ""}
    steps = plan(failsafe, baseline)  # raises CheckError before anything flies
    if not vehicle.is_simulator():
        report["problem"] = "this autopilot is not SITL: failsafe_check arms, flies and trips failsafes, so it runs in the sim only"
        return report

    # The autopilot must be flying the baseline's values, or the flight shows nothing about the baseline.
    for name in WATCHED[failsafe] + tuple(n for n in GRIPPER if n in baseline):
        value = vehicle.read_param(name)
        report["loaded"][name] = value
        if value is None or abs(value - baseline[name]) > 1.0e-3:
            report["problem"] = f"the autopilot has {name} {'unread' if value is None else format(value, 'g')}, the baseline has {baseline[name]:g}: the scenario did not start SITL with the baseline"
            return report
    restore = {"SIM_BATT_VOLTAGE": baseline.get("BATT_LOW_VOLT", 0.0) + HEALTHY_OVER_V, "SIM_RC_FAIL": 0.0, "SYSID_MYGCS": vehicle.read_param("SYSID_MYGCS") or 255.0}

    def watch(seconds: float, until=None) -> list[Sample]:
        seen = []
        deadline = vehicle.now() + seconds
        while vehicle.now() < deadline:
            seen.append(vehicle.step())
            if until is not None and until(seen):
                break
        return seen

    vehicle.takeoff(height_m)
    start = vehicle.step()
    gripper = all(name in baseline for name in GRIPPER) and baseline["GRIP_ENABLE"] == 1
    if gripper:
        # The gripper must be seen closed first: an output that never moved is not a payload held through a failsafe.
        vehicle.grab()
        watch(2.0)
        pwm = vehicle.servo_pwm()
        report["gripper_pwm_after_grab"] = pwm
        if pwm is None or abs(pwm - baseline["GRIP_GRAB"]) > PWM_NEAR:
            report["problem"] = f"the gripper's output reads {pwm} after the grab, not GRIP_GRAB {baseline['GRIP_GRAB']:g}: nothing would show whether a failsafe lets go"
            vehicle.return_home()
            watch(land_timeout_s, until=lambda seen: not vehicle.is_armed())
            report["landed_and_disarmed"] = not vehicle.is_armed()
            return report
    vehicle.goto(start.e + AWAY_M, start.n, height_m)
    leg = watch(60.0, until=lambda seen: math.hypot(seen[-1].e - start.e, seen[-1].n - start.n) > AWAY_M - 2.0)
    if not leg or math.hypot(leg[-1].e - start.e, leg[-1].n - start.n) <= AWAY_M - 2.0:
        report["problem"] = "the vehicle did not fly to its place away from home, so no failsafe was tripped"
        vehicle.return_home()
        return report
    watch(3.0)

    released = False
    for step in steps:
        mode_before = vehicle.step().mode
        for name, value in step["set"].items():
            if not vehicle.set_param(name, value):
                report["problem"] = f"SITL did not take {name} {value:g}: the failsafe could not be tripped"
                break
        if report["problem"]:
            break
        t_trigger = vehicle.now()
        seen = watch(step["within_s"] + 2.0, until=lambda seen: seen[-1].mode != mode_before)
        result = judge_step(step, mode_before, seen, t_trigger)
        if gripper:
            pwm = vehicle.servo_pwm()
            result["gripper_pwm"] = pwm
            released = released or pwm is None or abs(pwm - baseline["GRIP_GRAB"]) > PWM_NEAR  # anything but still grabbing
        report["steps"].append(result)
        print(f"  {result['name']}: {'ok' if result['passed'] else result['problem']}" + (f" ({result['delay_s']:.1f} s)" if result["delay_s"] is not None else ""), flush=True)
        if not result["passed"]:
            break
        watch(3.0)

    # Give back what was taken, so that the landing is the autopilot's own and not a second failsafe's.
    for name, value in restore.items():
        vehicle.set_param(name, value)
    if not report["steps"] or not report["steps"][-1]["passed"]:
        vehicle.return_home()  # never leave it hovering with a failsafe that did not act
    if gripper:
        report["gripper_held"] = not released
    watch(land_timeout_s, until=lambda seen: not vehicle.is_armed())
    report["landed_and_disarmed"] = not vehicle.is_armed()

    failed = next((step for step in report["steps"] if not step["passed"]), None)
    if report["problem"]:
        pass
    elif failed is not None:
        report["problem"] = f"{failed['name']}: {failed['problem']}"
    elif len(report["steps"]) < len(steps):
        report["problem"] = "not every step was flown"
    elif gripper and released:
        report["problem"] = "the gripper's output left its grab value (or could not be read) during a failsafe: FS_OPTIONS must not release it"
    elif not report["landed_and_disarmed"]:
        report["problem"] = f"the vehicle was not down and disarmed {land_timeout_s:g} s after the failsafe"
    else:
        report["passed"] = True
    return report


class FailsafeVehicle(MavlinkVehicle):
    """fence_check's vehicle, with parameters, the gripper and its servo output. Sim only."""

    def __init__(self, url: str, site: site_file.Site, gripper_output: int = 7) -> None:
        super().__init__(url, site)
        self._output = gripper_output

    def _wait_param(self, name: str, timeout_s: float = 3.0):
        import time

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            message = self.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if message is not None and message.param_id.rstrip("\x00") == name:
                return float(message.param_value)
        return None

    def read_param(self, name: str):
        for _ in range(3):
            self.master.mav.param_request_read_send(self.system, 1, name.encode(), -1)
            value = self._wait_param(name)
            if value is not None:
                return value
        return None

    def set_param(self, name: str, value: float) -> bool:
        for _ in range(3):
            self.master.mav.param_set_send(self.system, 1, name.encode(), float(value), self._mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            answer = self._wait_param(name)
            if answer is not None and abs(answer - value) < 1.0e-3:
                return True
        return False

    def grab(self) -> None:
        self._command(self._mavutil.mavlink.MAV_CMD_DO_GRIPPER, 1, self._mavutil.mavlink.GRIPPER_ACTION_GRAB)

    def servo_pwm(self):
        """The gripper output's PWM, from the next SERVO_OUTPUT_RAW; None if none comes."""
        import time

        mav = self._mavutil.mavlink
        self._command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 200000)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            message = self.master.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=0.5)
            if message is not None:
                return float(getattr(message, f"servo{self._output}_raw"))
        return None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Trip a failsafe in the sim and measure the response.")
    parser.add_argument("--site", type=Path, required=True)
    parser.add_argument("--failsafe", required=True, choices=sorted(WATCHED))
    parser.add_argument("--params", type=Path, required=True, help="the baseline parameter file the autopilot must be flying")
    parser.add_argument("--url", default="udpin:127.0.0.1:14552", help="pymavlink connection (the sim's tools port)")
    parser.add_argument("--height", type=float, default=10.0)
    parser.add_argument("--out", type=Path, help="write the report as JSON")
    args = parser.parse_args(argv)
    try:
        site = site_file.load_site(args.site)
        baseline = read_params(args.params)
        plan(args.failsafe, baseline)
    except (site_file.SiteError, OSError, ValueError) as error:
        print(f"failsafe_check: {error}", file=sys.stderr)
        return 2
    output = next((int(name[5:-9]) for name, value in baseline.items() if name.startswith("SERVO") and name.endswith("_FUNCTION") and value == 28), 7)
    try:
        report = run_check(FailsafeVehicle(args.url, site, gripper_output=output), baseline, args.failsafe, height_m=args.height)
    except ImportError:
        print("failsafe_check: pymavlink is not installed here (in the sim container, run with PYTHONNOUSERSITE unset)", file=sys.stderr)
        return 2
    except RuntimeError as error:
        print(f"failsafe_check: {error}", file=sys.stderr)
        return 2
    if args.out:
        args.out.write_text(json.dumps(report, indent=2) + "\n")
    print("RESULT: PASS" if report["passed"] else f"RESULT: FAIL ({report['problem']})")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
