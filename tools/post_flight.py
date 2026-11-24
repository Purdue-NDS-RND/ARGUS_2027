#!/usr/bin/env python3
"""The post-flight report (OPS-03): from an attempt's bundle, what happened, a score estimate, and the autonomy audit.

  python3 tools/post_flight.py attempts/<date>_<slot>_<mission>
  python3 tools/post_flight.py attempts/<...> --companion 1:191        # the companion's MAVLink ids, when not these

Reads the bundle tools/bundle_attempt.py made (PLT-12) and writes report.md
and report.json into it. Everything comes from the flight controller's own
dataflash log in the bundle, read with tools/dataflash_params.py:

  What happened   arming and disarming, every change of mode with its reason,
                  every error the flight controller logged, its own text
                  lines, the flight time, the greatest height, the lowest
                  battery voltage.
  Autonomy audit  The evidence for the autonomy bonus: inside the autonomous
                  window (armed to disarmed) the log is searched for a
                  person's input. Found, when there is any:
                    - a mode changed from the transmitter (mode change reason 1);
                    - a command (MAVC) whose sender is not the companion
                      computer: a ground station's;
                    - an RC override in force (RCI2's override mask);
                    - a stick moved (RCIN channels 1 to 4 away from where they
                      were at arming), or the mode switch;
                    - an arm or disarm by the transmitter.
                    - a mode changed over MAVLink to one the stack never asks
                      for (LOITER, say): not the companion's.
                  A mode changed over MAVLink carries no sender in this log:
                  the companion's own (GUIDED, RTL) look like a ground
                  station's. A change to a mode the sequence does ask for is
                  listed as not attributed, for the launch log and the ground
                  station's log to settle, and the verdict says how many
                  there are. Nor does this log give a sender for a position
                  target sent as a plain message; a ground station's "fly to
                  here" as a command (DO_REPOSITION) is found.
                  The window is the armed period in which the vehicle flew
                  highest; other armed periods of the same log are counted.
  Score estimate  With the sim's judges (sim/judge, SIM-08) on the log's own
                  track where the log has what a judge needs: the waypoint
                  mission and the time trial (the route is in the bundle),
                  the geofence demonstration (the fence polygon is in the
                  log), the drop and the delivery. For those two the log does
                  not say where the payload came down: the judge scores from
                  where the vehicle was at its first release command, with
                  its own height rules, and the report says so and gives the
                  rangefinder's reading at that moment beside the 6 ft line.
                  The localisation and the recovery are not estimated.

The verdict is the audit's: `NO HUMAN INPUT FOUND` (exit code 0) or `HUMAN
INPUT: <n> finding(s)` (exit code 1). Exit code 2: not a bundle, or no log
in it. No ROS imports.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / "tools", ROOT / "sim"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import dataflash_params  # noqa: E402
import release_confirm  # noqa: E402

COMPANION = ((1, 191),)  # MAVROS as this stack runs it: system 1, component 191 (seen in the sim's logs)
EARTH_RADIUS_M = 6378137.0
FEET_TO_METERS = 0.3048
STICK_MOVE_US = 50  # a stick this far from where it was at arming, in microseconds of PWM, has been moved
STICK_HOLD_S = 0.3  # for at least this long: one odd sample is not a hand
MODE_NAMES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT", 16: "POSHOLD",
              17: "BRAKE", 18: "THROW", 20: "GUIDED_NOGPS", 21: "SMART_RTL"}  # fmt: skip
# ArduPilot ModeReason. 1, 2 and 10 were read off real logs; the others are named as ArduPilot's source names them.
MODE_REASONS = {0: "unknown", 1: "the transmitter", 2: "a MAVLink command", 3: "radio failsafe", 4: "battery failsafe", 5: "ground-station failsafe", 6: "EKF failsafe",
                7: "GPS glitch", 8: "mission end", 10: "fence breach", 11: "terrain failsafe"}  # fmt: skip
REASON_TRANSMITTER, REASON_MAVLINK = 1, 2
ERR_SUBSYSTEMS = {2: "radio", 3: "compass", 5: "radio failsafe", 6: "battery failsafe", 8: "ground-station failsafe", 9: "fence", 10: "flight mode", 11: "GPS", 12: "crash check",
                  16: "EKF check", 17: "EKF failsafe", 18: "barometer", 22: "terrain", 24: "EKF primary", 25: "thrust loss", 29: "vibration failsafe"}  # fmt: skip
ERR_SUBSYSTEM_FENCE = 9
ARM_BY_TRANSMITTER = {0: "the rudder stick", 2: "a switch"}  # AP_Arming::Method
COMMAND_NAMES = {20: "RETURN_TO_LAUNCH", 21: "LAND", 22: "TAKEOFF", 176: "DO_SET_MODE", 183: "DO_SET_SERVO", 192: "DO_REPOSITION", 211: "DO_GRIPPER", 216: "DO_SPRAYER",
                 300: "MISSION_START", 400: "ARM_DISARM", 511: "SET_MESSAGE_INTERVAL", 512: "REQUEST_MESSAGE", 519: "REQUEST_PROTOCOL_VERSION", 520: "REQUEST_AUTOPILOT_CAPABILITIES"}  # fmt: skip
HOUSEKEEPING = {511, 512, 519, 520}  # a ground station asking what to be sent: not an input to the flight
# The modes the stack's own code asks for, whatever the sequence: GUIDED (the take-off, and the route's way back from
# AUTO), RTL (the rtl mission, the failure policy), LAND (the recovery's abort with the bucket on the hook), AUTO (the
# time trial's uploaded route). A test holds this to the code. A config adds its own (`rtl_mode: SMART_RTL`).
STACK_MODES = ("GUIDED", "RTL", "LAND", "AUTO")
SELF_SET_PARAMETERS = ("STAT_", "MOT_THST_HOVER", "COMPASS_", "INS_", "AHRS_TRIM", "BARO", "SR")  # written by the flight controller itself in flight, or stream rates
SIX_FEET_M = 1.83
RANGEFINDER_GOOD = 4  # RFND Stat of a reading that can be believed


class ReportError(ValueError):
    """Not an attempt's bundle, or no flight log in it. The message says why."""


@dataclass
class FlightLog:
    params: Dict[str, float] = field(default_factory=dict)
    firmware: Optional[str] = None
    first_s: Optional[float] = None
    last_s: Optional[float] = None
    positions: List[Tuple[float, float, float, float]] = field(default_factory=list)  # (s, latitude, longitude, height above home)
    modes: List[Tuple[float, int, int]] = field(default_factory=list)  # (s, mode, reason)
    errors: List[Tuple[float, int, int]] = field(default_factory=list)  # (s, subsystem, code)
    texts: List[Tuple[float, str]] = field(default_factory=list)
    commands: List[Dict[str, Any]] = field(default_factory=list)  # {"t", "system", "component", "command", "p1", "p2", "result"}
    rangefinder: List[Tuple[float, float, int]] = field(default_factory=list)  # (s, metres, status)
    param_changes: List[Tuple[float, str, float]] = field(default_factory=list)  # (s, name, new value): a parameter written after its first value
    sticks: List[Tuple[float, Tuple[int, ...]]] = field(default_factory=list)  # (s, channels 1 to 8)
    overrides: List[Tuple[float, int]] = field(default_factory=list)  # (s, override mask) where it is not zero
    arming: List[Tuple[float, int, int]] = field(default_factory=list)  # (s, armed 1 or 0, method)
    battery: List[Tuple[float, float]] = field(default_factory=list)  # (s, volts)
    polygon: List[Tuple[float, float]] = field(default_factory=list)  # the inclusion fence, (latitude, longitude)
    outputs: List[Tuple[float, Tuple[int, ...]]] = field(default_factory=list)  # (s, servo outputs 1 to 14, PWM)
    tuning: List[Tuple[float, float, float, float]] = field(default_factory=list)  # (s, throttle 0 to 1, climb asked for m/s, climb m/s)


def read_flight(path) -> FlightLog:
    log = FlightLog()
    try:
        parameters = dataflash_params.read_parameters(path)
    except (dataflash_params.DataflashError, OSError) as problem:
        raise ReportError(str(problem)) from problem
    log.params, log.firmware = parameters.values, parameters.firmware
    seen: Dict[str, float] = {}
    for name, row in dataflash_params.messages(path, ("POS", "MODE", "ERR", "MSG", "MAVC", "RCIN", "RCI2", "ARM", "BAT", "FNCE", "RFND", "PARM", "RCOU", "CTUN")):
        t = float(row.get("TimeUS", 0)) * 1.0e-6
        if name == "PARM":
            parameter, value = dataflash_params._text(row["Name"]), float(row["Value"])
            if parameter in seen and seen[parameter] != value:
                log.param_changes.append((t, parameter, value))
            seen[parameter] = value
            continue
        log.first_s = t if log.first_s is None else log.first_s
        log.last_s = t
        if name == "RFND" and "Dist" in row:
            if int(row.get("Instance", 0)) == 0:
                log.rangefinder.append((t, float(row["Dist"]) * 0.01, int(row.get("Stat", 0))))  # the log holds centimetres
            continue
        if name == "RCOU" and "C1" in row:
            log.outputs.append((t, tuple(int(row.get(f"C{n}", 0)) for n in range(1, 15))))
            continue
        if name == "CTUN" and "ThO" in row:
            log.tuning.append((t, float(row["ThO"]), float(row.get("DCRt", 0)) * 0.01, float(row.get("CRt", 0)) * 0.01))  # the log holds cm/s
            continue
        if name == "POS" and "Lat" in row:
            log.positions.append((t, row["Lat"] * 1.0e-7, row["Lng"] * 1.0e-7, float(row.get("RelHomeAlt", 0.0))))
        elif name == "MODE" and "ModeNum" in row:
            log.modes.append((t, int(row["ModeNum"]), int(row.get("Rsn", 0))))
        elif name == "ERR":
            log.errors.append((t, int(row.get("Subsys", -1)), int(row.get("ECode", 0))))
        elif name == "MSG" and "Message" in row:
            log.texts.append((t, dataflash_params._text(row["Message"])))
        elif name == "MAVC" and "Cmd" in row:
            log.commands.append({"t": t, "system": int(row.get("SS", 0)), "component": int(row.get("SC", 0)), "command": int(row["Cmd"]), "p1": float(row.get("P1", 0.0)), "p2": float(row.get("P2", 0.0)),
                                 "result": int(row.get("Res", -1))})  # fmt: skip  (-1: a log whose MAVC has no result column acknowledges nothing)
        elif name == "RCIN" and "C1" in row:
            log.sticks.append((t, tuple(int(row.get(f"C{n}", 0)) for n in range(1, 9))))
        elif name == "RCI2" and int(row.get("OMask", 0)):
            log.overrides.append((t, int(row["OMask"])))
        elif name == "ARM" and "ArmState" in row:
            log.arming.append((t, int(row["ArmState"]), int(row.get("Method", -1))))
        elif name == "BAT" and int(row.get("Inst", 0)) == 0 and "Volt" in row:
            log.battery.append((t, float(row["Volt"])))
        elif name == "FNCE" and int(row.get("Type", -1)) == 98 and "Lat" in row:
            if int(row.get("Seq", 0)) == 0:
                log.polygon = []
            log.polygon.append((row["Lat"] * 1.0e-7, row["Lng"] * 1.0e-7))
    if log.first_s is None:
        raise ReportError(f"{path}: no flight data in the log")
    return log


def armed_periods(log: FlightLog) -> List[Tuple[float, float]]:
    """Every armed period of the log. ArduCopter starts its log at arming, so with no record of the first arming it is the log's start."""
    changes = [(t, state) for t, state, _method in log.arming]
    changes += [(t, 1) for t, text in log.texts if text == "Arming motors"] + [(t, 0) for t, text in log.texts if text == "Disarming motors"]
    periods, t_arm = [], None
    for t, state in sorted(changes):
        if state == 1 and t_arm is None:
            t_arm = t
        elif state == 0:
            periods.append((log.first_s if t_arm is None and not periods else t_arm, t)) if (t_arm is not None or not periods) else None
            t_arm = None
    if t_arm is not None:
        periods.append((t_arm, log.last_s))
    return periods or [(log.first_s, log.last_s)]


def window(log: FlightLog) -> Tuple[float, float]:
    """The autonomous window: the armed period in which the vehicle flew highest (the flight), armed to disarmed."""

    def top(period: Tuple[float, float]) -> float:
        return max((height for t, _lat, _lon, height in log.positions if period[0] <= t <= period[1]), default=0.0)

    return max(armed_periods(log), key=top)


def first_release(log: FlightLog) -> Optional[Dict[str, Any]]:
    """The first command inside the window that lets a payload go, as the stack sends them.

    DO_GRIPPER with its action 0 (1 is a grab), DO_SPRAYER switched on (the drop servo; off is the servo's return),
    or a DO_SET_SERVO.
    """
    t0, t1 = window(log)
    for command in log.commands:
        if not t0 <= command["t"] <= t1:
            continue
        if (command["command"] == 211 and command["p2"] == 0.0) or (command["command"] == 216 and command["p1"] == 1.0) or command["command"] == 183:
            return command
    return None


def release_confirmation(log: FlightLog) -> Dict[str, Any]:
    """CEN-14: was the first release acknowledged, did its output move, did the payload leave (tools/release_confirm.py)."""
    return release_confirm.confirm(first_release(log), log.outputs, log.tuning, log.rangefinder, log.params)


def rangefinder_at(log: FlightLog, when: float) -> Optional[float]:
    """The rangefinder's reading nearest a time, if it is within half a second and good; else None."""
    near = [(abs(t - when), metres) for t, metres, status in log.rangefinder if abs(t - when) <= 0.5 and status == RANGEFINDER_GOOD]
    return min(near)[1] if near else None


def mode_name(mode: int) -> str:
    return MODE_NAMES.get(mode, f"mode {mode}")


def timeline(log: FlightLog) -> List[Tuple[float, str]]:
    """What the flight controller recorded, in order."""
    t0, t1 = window(log)
    lines = [(t0, "armed")]
    previous = None
    for t, mode, reason in log.modes:
        if mode != previous:
            lines.append((t, f"mode {mode_name(mode)} (by {MODE_REASONS.get(reason, f'reason {reason}')})"))
        previous = mode
    for t, subsystem, code in log.errors:
        what = ERR_SUBSYSTEMS.get(subsystem, f"subsystem {subsystem}")
        lines.append((t, f"error: {what}, code {code}" if code else f"cleared: {what}"))
    boring = ("ArduCopter", "Param space", "RC Protocol", "New mission", "New rally", "New fence", "Frame:", "GPS ", "Arming motors", "Disarming motors")
    lines += [(t, f'"{text}"') for t, text in log.texts if not text.startswith(boring) and len(text) != 32]
    airborne = [t for t, _lat, _lon, height in log.positions if height > 1.0]
    if airborne:
        lines.append((airborne[0], "left the ground"))
    methods = [method for t, state, method in log.arming if state == 0 and abs(t - t1) < 0.5]
    lines.append((t1, "disarmed" + (f" (method {methods[0]})" if methods else "")))
    return sorted(lines, key=lambda line: line[0])


def numbers(log: FlightLog) -> Dict[str, Any]:
    t0, t1 = window(log)
    inside = [p for p in log.positions if t0 <= p[0] <= t1]
    distance = sum(_metres(a[1], a[2], b[1], b[2]) for a, b in zip(inside, inside[1:]))
    volts = [v for t, v in log.battery if t0 <= t <= t1]
    return {"armed_s": t1 - t0, "max_height_m": max((p[3] for p in inside), default=None), "distance_m": distance, "min_battery_v": min(volts, default=None)}


def _metres(lat_a: float, lon_a: float, lat_b: float, lon_b: float) -> float:
    north = math.radians(lat_b - lat_a) * EARTH_RADIUS_M
    east = math.radians(lon_b - lon_a) * EARTH_RADIUS_M * math.cos(math.radians(lat_a))
    return math.hypot(north, east)


def audit(log: FlightLog, companion: Set[Tuple[int, int]] = frozenset(COMPANION), expected_modes: Optional[Set[str]] = None) -> Dict[str, Any]:
    """The autonomy audit: a person's inputs inside the autonomous window, and the mode changes the log cannot attribute.

    `expected_modes` are the modes the flight's own sequence asks for (expected_modes_of): a mode changed over
    MAVLink to any other is not the companion's. With None, no MAVLink mode change is held against the flight.
    """
    t0, t1 = window(log)
    findings: List[Dict[str, Any]] = []
    unattributed: List[Dict[str, Any]] = []
    previous = None
    for t, mode, reason in log.modes:
        changed = mode != previous
        previous = mode
        if not (t0 < t <= t1) or not changed:
            continue  # the mode at arming is the start of the window, not an input inside it
        if reason == REASON_TRANSMITTER:
            findings.append({"t": t, "kind": "mode from the transmitter", "what": f"{mode_name(mode)} was switched to from the transmitter"})
        elif reason == REASON_MAVLINK and expected_modes is not None and mode_name(mode) not in expected_modes:
            findings.append({"t": t, "kind": "mode the stack never asks for", "what": f"{mode_name(mode)} by a MAVLink command; the stack asks only for {', '.join(sorted(expected_modes))}"})
        elif reason == REASON_MAVLINK:
            unattributed.append({"t": t, "what": f"{mode_name(mode)} by a MAVLink command"})
    for command in log.commands:
        if not t0 <= command["t"] <= t1 or (command["system"], command["component"]) in companion or command["command"] in HOUSEKEEPING:
            continue
        name = COMMAND_NAMES.get(command["command"], f"command {command['command']}")
        findings.append({"t": command["t"], "kind": "command not from the companion", "what": f"{name} from system {command['system']}, component {command['component']}"})
    inside = [(t, mask) for t, mask in log.overrides if t0 <= t <= t1]
    if inside:
        findings.append({"t": inside[0][0], "kind": "RC override", "what": f"an RC override was in force (mask {inside[0][1]:#x}) for {inside[-1][0] - inside[0][0]:.1f} s"})
    findings += _stick_findings(log, t0, t1)
    for t, state, method in log.arming:
        if t0 <= t <= t1 and method in ARM_BY_TRANSMITTER:
            findings.append({"t": t, "kind": "arming from the transmitter", "what": f"{'armed' if state else 'disarmed'} by {ARM_BY_TRANSMITTER[method]}"})
    findings.sort(key=lambda item: item["t"])
    changed = [{"t": t, "what": f"{name} set to {value:g}"} for t, name, value in log.param_changes if t0 <= t <= t1 and not name.startswith(SELF_SET_PARAMETERS)]
    return {"window": [t0, t1], "armed_periods": len(armed_periods(log)), "findings": findings, "unattributed": unattributed, "parameters_changed": changed, "clean": not findings, "commands_from_companion": sum(1 for c in log.commands if t0 <= c["t"] <= t1 and (c["system"], c["component"]) in companion),
            "sticks_logged": len([1 for t, _c in log.sticks if t0 <= t <= t1])}  # fmt: skip


def _stick_findings(log: FlightLog, t0: float, t1: float) -> List[Dict[str, Any]]:
    """A stick or the mode switch away from where it was at arming, for longer than one sample."""
    inside = [(t, channels) for t, channels in log.sticks if t0 <= t <= t1]
    if not inside:
        return []
    start = inside[0][1]
    mode_channel = int(log.params.get("FLTMODE_CH", 5))
    watched = {1: "roll stick", 2: "pitch stick", 3: "throttle stick", 4: "yaw stick", mode_channel: "mode switch"}
    found = []
    for channel, name in watched.items():
        if not 1 <= channel <= len(start) or start[channel - 1] == 0:
            continue  # no such channel on this receiver
        since = None
        for t, channels in inside:
            moved = abs(channels[channel - 1] - start[channel - 1]) > STICK_MOVE_US
            since = (t if since is None else since) if moved else None
            if since is not None and t - since >= STICK_HOLD_S:
                found.append({"t": since, "kind": "stick moved", "what": f"the {name} (channel {channel}) moved from {start[channel - 1]} to {channels[channel - 1]} us"})
                break
    return found


def _enu(log: FlightLog, latitude: float, longitude: float) -> Tuple[float, float]:
    home = log.positions[0]
    north = math.radians(latitude - home[1]) * EARTH_RADIUS_M
    east = math.radians(longitude - home[2]) * EARTH_RADIUS_M * math.cos(math.radians(home[1]))
    return east, north


def trace_lines(log: FlightLog, checked: Dict[str, Any]) -> List[Dict[str, Any]]:
    """The log as the sim's judges take a flight: the vehicle's track from home, and the events they read."""
    t0, t1 = window(log)
    lines: List[Dict[str, Any]] = [{"t": t0, "event": "armed"}, {"t": t1, "event": "disarmed"}]
    for t, latitude, longitude, height in log.positions:
        east, north = _enu(log, latitude, longitude)
        lines.append({"t": t, "pose": "vehicle", "e": east, "n": north, "u": height, "yaw": None})
    lines += [{"t": t, "event": "mode", "mode": mode_name(mode)} for t, mode, _reason in log.modes]
    lines += [{"t": t, "event": "fence_breach", "what": "fence breached"} for t, subsystem, code in log.errors if subsystem == ERR_SUBSYSTEM_FENCE and code]
    lines += [{"t": item["t"], "event": "gcs_command", "what": item["what"]} for item in checked["findings"]]  # the judge takes its autonomy bonus away for any
    return lines


def _sequence_values(folder: Path, *keys: str) -> Dict[str, Any]:
    """The first value of each key anywhere in the bundle's sequence file."""
    import yaml

    found: Dict[str, Any] = {}

    def walk(node: Any) -> None:
        if isinstance(node, dict):
            for key, value in node.items():
                if key in keys and key not in found:
                    found[key] = value
                walk(value)
        elif isinstance(node, list):
            for item in node:
                walk(item)

    path = folder / "config" / "sequence.yaml"
    if path.is_file():
        walk(yaml.safe_load(path.read_text(encoding="utf-8")))
    return found


def expected_modes_of(folder: Path) -> Set[str]:
    """The modes this flight's stack can ask the flight controller for: the ones its code asks for, and any the bundle's config names."""
    import yaml

    modes = set(STACK_MODES)

    def walk(node: Any) -> None:
        if isinstance(node, dict):
            for key, value in node.items():
                if isinstance(value, str) and (key.endswith("_mode") or key == "mode") and value.isupper():
                    modes.add(value)
                walk(value)
        elif isinstance(node, list):
            for item in node:
                walk(item)

    for name in ("sequence.yaml", "params.yaml"):
        path = folder / "config" / name
        if path.is_file():
            try:
                walk(yaml.safe_load(path.read_text(encoding="utf-8")))
            except yaml.YAMLError:
                pass
    return modes


def estimate(folder: Path, mission: str, log: FlightLog, checked: Dict[str, Any]) -> Dict[str, Any]:
    """A score estimate for the mission, by the sim's judge where the log has what the judge needs; else why not."""
    from judge.record import Record
    from judge.score import default_settings, judge_run

    if not log.positions:
        return {"estimated": False, "why": "the log has no positions"}
    record = Record.from_lines(trace_lines(log, checked))
    if mission in ("waypoint_navigation", "time_trial"):
        import yaml

        route_file = folder / "config" / "waypoints.yaml"
        if not route_file.is_file():
            return {"estimated": False, "why": "the bundle has no route (config/waypoints.yaml): bundle it with the sequence that names its waypoint file"}
        waypoints = []
        for item in (yaml.safe_load(route_file.read_text(encoding="utf-8")) or {}).get("waypoints") or []:
            east, north = _enu(log, float(item["latitude"]), float(item["longitude"]))
            waypoints.append({"name": str(item.get("name", "")), "e": east, "n": north, "u": float(item.get("altitude_agl_ft", 0.0)) * FEET_TO_METERS})
        score = judge_run(mission, record, {"waypoints": waypoints})
        return {"estimated": True, "how": f"the sim's {mission} judge on the log's own track (GPS and EKF positions, not a surveyed truth)", "score": score}
    if mission == "geofence":
        if len(log.polygon) < 3:
            return {"estimated": False, "why": "the log holds no fence polygon (FNCE messages)"}
        truth = {"fence": {"polygon": [_enu(log, latitude, longitude) for latitude, longitude in log.polygon], "max_height_m": log.params.get("FENCE_ALT_MAX")}}
        score = judge_run("geofence", record, truth, {"held_within_m": 4.0, "held_min_s": 4.0})
        return {"estimated": True, "how": "the sim's geofence judge on the log's own track and the fence polygon the log holds", "score": score}
    if mission in ("drop", "delivery"):
        target = _sequence_values(folder, "target_latitude", "target_longitude")
        release = first_release(log)
        if len(target) < 2:
            return {"estimated": False, "why": "the bundle's sequence names no target"}
        if release is None:
            return {"estimated": False, "why": "the log has no release command (the gripper's release, the drop servo switched on, or a servo command): nothing was let go, or it was a fake drop"}
        east, north = _enu(log, float(target["target_latitude"]), float(target["target_longitude"]))
        lines = trace_lines(log, checked) + [{"t": release["t"], "event": "release"}]
        score = judge_run(mission, Record.from_lines(lines), {"target": {"e": east, "n": north}})
        return {"estimated": True, "how": f"the sim's {mission} judge from where the vehicle was at its first release command: the log does not say where the payload came down",
                "score": score, "release": {"t": release["t"], "command": COMMAND_NAMES.get(release["command"], str(release["command"])), "rangefinder_m": rangefinder_at(log, release["t"])}}  # fmt: skip
    return {"estimated": False, "why": f"the {mission} needs what the log does not hold (the markers' or the bucket's true places): score it from the judges' own sheet"}


def report_text(manifest: Dict[str, Any], log: FlightLog, checked: Dict[str, Any], scored: Dict[str, Any], log_name: str) -> str:
    t0, t1 = checked["window"]
    facts = numbers(log)
    out = [f"# Post-flight report: {manifest.get('attempt')}", "",
           f"Written by `tools/post_flight.py` from the attempt's bundle. Flight controller: {log.firmware or 'firmware line not in the log'}; log `dataflash/{log_name}`.", ""]  # fmt: skip
    profile, git, preflight = manifest.get("profile") or {}, manifest.get("git") or {}, (manifest.get("parts") or {}).get("preflight") or {}
    out += ["## The attempt", "",
            f"- Mission: {manifest.get('mission')}, slot {manifest.get('slot')}, {manifest.get('date')}.",
            f"- Code: {git.get('describe') or git.get('hash') or 'not recorded'}" + (" with uncommitted changes" if git.get("dirty") else "") + ".",
            f"- Flag profile: {profile.get('name', 'not in the bundle')} ({', '.join(profile.get('flags') or []) or 'no flags: the 2026 code'}).",
            f"- Preflight: {preflight.get('verdict', 'no record')}" + (f", overridden: {preflight['override']}" if preflight.get("override") else "") + ".",
            "- Bundle: " + ("complete." if manifest.get("complete") else "INCOMPLETE, owing " + ", ".join(manifest.get("owed") or []) + "."), ""]  # fmt: skip
    out += ["## What happened", "",
            f"Armed for {facts['armed_s']:.0f} s; greatest height {_fmt(facts['max_height_m'], 'm')}; {facts['distance_m']:.0f} m flown; lowest battery voltage {_fmt(facts['min_battery_v'], 'V')}.", ""]  # fmt: skip
    out += [f"- {t - t0:7.1f} s  {text}" for t, text in timeline(log)] + [""]
    more = "" if checked["armed_periods"] == 1 else f" The log has {checked['armed_periods']} armed periods; this is the one the vehicle flew highest in."
    out += ["## Autonomy audit", "", f"The autonomous window is armed to disarmed: {t1 - t0:.0f} s.{more} Times below are from arming.", ""]
    if checked["clean"]:
        open_points = f", with {len(checked['unattributed'])} mode change(s) over MAVLink to confirm (below)" if checked["unattributed"] else ""
        out.append(f"**NO HUMAN INPUT FOUND**{open_points}. In the flight controller's log: no mode from the transmitter and none the stack never asks for, no command (COMMAND_LONG or COMMAND_INT) "
                   f"from anything but the companion ({checked['commands_from_companion']} commands, all its own), no RC override, no stick or mode switch moved ({checked['sticks_logged']} RC samples), "
                   "no arming from the transmitter.")  # fmt: skip
    else:
        out.append(f"**HUMAN INPUT: {len(checked['findings'])} finding(s).** The autonomy bonus cannot be claimed on this log as it stands.")
        out += [""] + [f"- {item['t'] - t0:7.1f} s  {item['kind']}: {item['what']}" for item in checked["findings"]]
    if checked["unattributed"]:
        out += ["", "Not attributed by this log (a mode changed over MAVLink carries no sender here; the companion's own GUIDED and RTL look the same as a ground station's). "
                "Hold these against the launch log and the ground station's log:", ""]  # fmt: skip
        out += [f"- {item['t'] - t0:7.1f} s  {item['what']}" for item in checked["unattributed"]]
    if checked["parameters_changed"]:
        out += ["", "Parameters written during the window (the log does not say by whom):", ""] + [f"- {item['t'] - t0:7.1f} s  {item['what']}" for item in checked["parameters_changed"]]
    out += ["", "Not visible in this log: who sent a mode change or a position target that came as a plain MAVLink message. The ground station's own log has that.",
            "", "## Score estimate", ""]  # fmt: skip
    if not scored.get("estimated"):
        out.append(f"Not estimated: {scored.get('why')}.")
    else:
        score = scored["score"]
        out += [f"By {scored['how']}:", "", f"- {score['points']:.2f} points ({score['base_points']:.2f} and {score['autonomy_bonus']:.2f} autonomy bonus).",
                f"- details: {json.dumps(_brief(score['details']))}"] + [f"- note: {note}" for note in score.get("notes") or []]  # fmt: skip
        if "release" in scored:
            release, details = scored["release"], score["details"]
            height = f"{details['release_height_m']:.2f} m above home by the flight controller's position" if details.get("release_height_m") is not None else "height not known"
            seen = "no good rangefinder reading at that moment" if release["rangefinder_m"] is None else f"{release['rangefinder_m']:.2f} m by the rangefinder"
            out.append(f"- released by {release['command']} at {release['t'] - t0:.1f} s, {height}; {seen} (6 ft is {SIX_FEET_M} m).")
    confirmation = checked.get("release_confirmation") or {"verdict": "NO RELEASE"}
    out += ["", "## Release confirmation", ""]
    if confirmation["verdict"] == "NO RELEASE":
        out.append("No release command in the flight: nothing to confirm.")
    else:
        why = "" if confirmation["verdict"] == "CONFIRMED" else ": " + "; ".join(confirmation["missing"])
        out += [f"**RELEASE {confirmation['verdict']}**{why}. At {confirmation['t'] - t0:.1f} s, from the flight controller's log:", ""] + [f"- {line}" for line in confirmation["lines"]]
    return "\n".join(out) + "\n"


def _fmt(value: Optional[float], unit: str) -> str:
    return "not in the log" if value is None else f"{value:.1f} {unit}"


def _brief(details: Dict[str, Any]) -> Dict[str, Any]:
    """A judge's details without its long lists, and with numbers rounded."""
    return {key: (round(value, 2) if isinstance(value, float) else value) for key, value in details.items() if not isinstance(value, (list, dict)) or key == "held"}


def make_report(folder: Path, companion: Set[Tuple[int, int]]) -> Tuple[str, Dict[str, Any]]:
    try:
        manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as problem:
        raise ReportError(f"{folder} is not an attempt's bundle: {problem}") from problem
    logs = sorted((folder / "dataflash").glob("*.BIN")) if (folder / "dataflash").is_dir() else []
    if not logs:
        raise ReportError(f"{folder} has no dataflash log: the report and the audit are read from it")
    log = read_flight(logs[-1])
    checked = audit(log, companion, expected_modes_of(folder))
    checked["release_confirmation"] = release_confirmation(log)
    scored = estimate(folder, str(manifest.get("mission")), log, checked)
    data = {"attempt": manifest.get("attempt"), "mission": manifest.get("mission"), "log": logs[-1].name, "numbers": numbers(log), "audit": checked,
            "estimate": {**{key: value for key, value in scored.items() if key != "score"},
                         **({"points": scored["score"]["points"], "base_points": scored["score"]["base_points"], "autonomy_bonus": scored["score"]["autonomy_bonus"],
                             "details": _brief(scored["score"]["details"]), "notes": scored["score"]["notes"]} if "score" in scored else {})}}  # fmt: skip
    return report_text(manifest, log, checked, scored, logs[-1].name), data


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="The post-flight report and autonomy audit of an attempt's bundle.")
    parser.add_argument("bundle")
    parser.add_argument("--companion", action="append", metavar="SYSTEM:COMPONENT", help="the companion computer's MAVLink ids (default 1:191); repeat for more")
    args = parser.parse_args(argv)
    try:
        companion = {tuple(int(part) for part in item.split(":")) for item in args.companion} if args.companion else set(COMPANION)
        if any(len(ids) != 2 for ids in companion):
            raise ValueError("--companion takes SYSTEM:COMPONENT")
        text, data = make_report(Path(args.bundle), companion)
    except (ReportError, ValueError) as problem:
        print(f"post_flight: {problem}", file=sys.stderr)
        return 2
    folder = Path(args.bundle)
    (folder / "report.md").write_text(text, encoding="utf-8")
    (folder / "report.json").write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    (folder / "release_confirmation.json").write_text(json.dumps(data["audit"]["release_confirmation"], indent=2) + "\n", encoding="utf-8")
    print(text, end="")
    print(f"\nwrote {folder / 'report.md'}, report.json and release_confirmation.json")
    print(f"RELEASE {data['audit']['release_confirmation']['verdict']}" if data["audit"]["release_confirmation"]["verdict"] != "NO RELEASE" else "no release in the flight")
    print("NO HUMAN INPUT FOUND" if data["audit"]["clean"] else f"HUMAN INPUT: {len(data['audit']['findings'])} finding(s)")
    return 0 if data["audit"]["clean"] else 1


if __name__ == "__main__":
    sys.exit(main())
