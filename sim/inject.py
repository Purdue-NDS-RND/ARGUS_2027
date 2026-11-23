#!/usr/bin/env python3
"""Inject GPS faults into the sim's vehicle at scheduled times (SIM-04). Runs inside the sim container.

  env -u PYTHONNOUSERSITE python3 sim/inject.py <run folder>

The one fault tool: started by sim/runner/fly.py beside every flight whose
scenario has `gps_events`, and interrupted when the flight ends. Each event
is a change to the simulated GPS at a time counted from the moment the
vehicle arms, in sim time:

  - {at_s, kind: glitch, north_m, east_m, up_m, for_s}   the fix steps by this much (for_s: and back again after)
  - {at_s, kind: noise, amplitude_m, for_s}              the simulated GPS's altitude noise
  - {at_s, kind: satellites, count, for_s}               the satellite count it reports
  - {at_s, kind: jam, for_s}                             jamming: position, velocity and satellites wander
  - {at_s, kind: dropout, for_s}                         no GPS at all
  - {at_s, kind: rangefinder, scale, for_s}              the rangefinder reads `scale` times the true range
  - {at_s, kind: mode, mode}                             the flight mode is changed, as the safety pilot's switch would
  - {at_s, kind: clear}                                  everything back as it was

(Nor is a mode change a GPS fault: it is how a flight with the takeover latch on is taken over at a set moment
(PLT-09). It is sent from this tool's own MAVLink system, not the ground station's, so the ground-station
audit does not count it as a person's input.)

(The rangefinder's is not a GPS fault. It is here because this is the one
tool that changes SITL's sensors at a scheduled time: a survey's height
guard is flown with it, the reading dipping as it would over rising ground,
which SITL's rangefinder, on flat ground whatever the Gazebo world holds,
cannot see. sim/height_check.py leaves the heights of that time unjudged.)

The faults are SITL's own, set through its SIM_GPS parameters over MAVLink.
The names differ between ArduPilot versions (`SIM_GPS_GLITCH_X` in 4.6,
`SIM_GPS1_GLTCH_X` in later ones), so the tool asks the running SITL which
it has before the flight and stops if it finds neither: a scenario with GPS
faults is then not flown calm and called faulted.

(The other half of SIM-04, an error in the coordinates a mission is given,
is the scenario's `coordinate_error`, applied when the run is built.)

The run folder gets gps_events.jsonl (each change, with the sim time it was
made and, for a glitch, the raw GPS fix before and after) and
gps_events.json. The schedule and the parameter values are plain functions
and a class with no MAVLink in them.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

METRES_PER_DEGREE = 111319.5  # of latitude; of longitude at the equator
KINDS = {
    "glitch": {"north_m", "east_m", "up_m", "for_s"},
    "noise": {"amplitude_m", "for_s"},
    "satellites": {"count", "for_s"},
    "jam": {"for_s"},
    "dropout": {"for_s"},
    "rangefinder": {"scale", "for_s"},
    "mode": {"mode", "for_s"},  # for_s is carried (as None) by a checked event; a mode is not undone after a time
    "clear": set(),
}
TAKEOVER_MODES = ("LOITER", "ALT_HOLD", "STABILIZE", "BRAKE", "LAND", "RTL", "GUIDED")  # what a mode event may ask for
RANGEFINDER_SCALE = (0.1, 2.0)  # what a rangefinder fault's `scale` may be
# What each fault is called in the two families of ArduPilot's SITL. {role: (4.6 name, later name)}.
PARAMETER_NAMES = {
    "glitch_lat": ("SIM_GPS_GLITCH_X", "SIM_GPS1_GLTCH_X"),
    "glitch_lon": ("SIM_GPS_GLITCH_Y", "SIM_GPS1_GLTCH_Y"),
    "glitch_alt": ("SIM_GPS_GLITCH_Z", "SIM_GPS1_GLTCH_Z"),
    "noise": ("SIM_GPS_NOISE", "SIM_GPS1_NOISE"),
    "satellites": ("SIM_GPS_NUMSATS", "SIM_GPS1_NUMSATS"),
    "jam": ("SIM_GPS_JAM", "SIM_GPS1_JAM"),
    "disable": ("SIM_GPS_DISABLE", "SIM_GPS1_DISABLE"),
    # Metres a volt of SITL's analog rangefinder: the flight controller reads true range x (its own scaling / this).
    "sonar_scale": ("SIM_SONAR_SCALE", "SIM_SONAR_SCALE"),
}
# Roles only some scenarios use: a SITL without their parameter still flies every scenario that does not ask for them.
OPTIONAL_ROLES = {"sonar_scale": "rangefinder"}  # {role: the kind of event that needs it}
FCU_SYSTEM, FCU_COMPONENT = 1, 1  # the flight controller, whoever else is heard on the link
RAW_AFTER_S = 1.0  # how long after a change the raw fix's satellite count and fix type are noted
GLITCH_PASS_M = 0.5  # a commanded step must show in the raw fix to within this,
GLITCH_PASS_S = 1.0  # within this long


class InjectError(ValueError):
    """Events that cannot be injected. The message says why."""


def check_events(events: list) -> List[dict]:
    """The scenario's gps_events, checked: each has a time, a known kind and only that kind's settings."""
    checked = []
    for index, event in enumerate(events or []):
        if not isinstance(event, dict) or "at_s" not in event or event.get("kind") not in KINDS:
            raise InjectError(f"gps_events[{index}] needs at_s and a kind, one of: {', '.join(KINDS)}")
        extra = sorted(set(event) - {"at_s", "kind"} - KINDS[event["kind"]])
        if extra:
            raise InjectError(f"gps_events[{index}] ({event['kind']}) has unknown setting(s) {', '.join(extra)}")
        try:
            at_s = float(event["at_s"])
            for_s = None if event.get("for_s") is None else float(event["for_s"])
        except (TypeError, ValueError):
            raise InjectError(f"gps_events[{index}]: at_s and for_s are numbers of seconds") from None
        if at_s < 0.0 or (for_s is not None and for_s <= 0.0):
            raise InjectError(f"gps_events[{index}]: at_s cannot be negative and for_s must be above zero")
        if event["kind"] == "rangefinder":
            scale = event.get("scale")
            if isinstance(scale, bool) or not isinstance(scale, (int, float)) or not RANGEFINDER_SCALE[0] <= scale <= RANGEFINDER_SCALE[1]:
                raise InjectError(f"gps_events[{index}] (rangefinder) needs a scale between {RANGEFINDER_SCALE[0]:g} and {RANGEFINDER_SCALE[1]:g}, got {scale!r}")
        if event["kind"] == "mode" and event.get("mode") not in TAKEOVER_MODES:
            raise InjectError(f"gps_events[{index}] (mode) needs a mode, one of: {', '.join(TAKEOVER_MODES)}; got {event.get('mode')!r}")
        if event["kind"] == "mode" and for_s is not None:
            raise InjectError(f"gps_events[{index}] (mode) takes no for_s: a mode is not changed back after a time; give a second mode event")
        checked.append({**event, "at_s": at_s, "for_s": for_s})
    return checked


def glitch_degrees(north_m: float, east_m: float, latitude_deg: float) -> Tuple[float, float]:
    """A step in metres as the degrees of latitude and longitude SITL's glitch parameters take."""
    return float(north_m) / METRES_PER_DEGREE, float(east_m) / (METRES_PER_DEGREE * math.cos(math.radians(latitude_deg)))


def roles_needed(events: list) -> set:
    """The optional roles a scenario's events use."""
    kinds = {event.get("kind") for event in events or [] if isinstance(event, dict)}
    return {role for role, kind in OPTIONAL_ROLES.items() if kind in kinds}


def resolve_names(have: set, needed: frozenset = frozenset()) -> Dict[str, str]:
    """{role: the parameter name this SITL has}. Raises if a role has no name here; an optional one only if it is `needed`."""
    names, missing = {}, []
    for role, candidates in PARAMETER_NAMES.items():
        found = next((name for name in candidates if name in have), None)
        if found is None and role in OPTIONAL_ROLES and role not in needed:
            continue
        if found is None:
            missing.append(" or ".join(dict.fromkeys(candidates)))
        else:
            names[role] = found
    if missing:
        raise InjectError("this SITL has none of the parameters for: " + "; ".join(missing))
    return names


def changes_for(event: dict, latitude_deg: float, defaults: Dict[str, float]) -> Tuple[Dict[str, float], Dict[str, float]]:
    """({role: value} to set for an event, {role: value} that undoes it)."""
    kind = event["kind"]
    if kind == "glitch":
        d_lat, d_lon = glitch_degrees(event.get("north_m", 0.0), event.get("east_m", 0.0), latitude_deg)
        on = {"glitch_lat": d_lat, "glitch_lon": d_lon, "glitch_alt": float(event.get("up_m", 0.0))}
    elif kind == "noise":
        on = {"noise": float(event.get("amplitude_m", 1.0))}
    elif kind == "satellites":
        on = {"satellites": float(event.get("count", 4))}
    elif kind == "jam":
        on = {"jam": 1.0}
    elif kind == "dropout":
        on = {"disable": 1.0}
    elif kind == "rangefinder":
        on = {"sonar_scale": defaults["sonar_scale"] / float(event["scale"])}
    elif kind == "mode":
        return {}, {}  # no parameter: the injector asks the flight controller for the mode itself
    else:  # clear
        return dict(defaults), {}
    return on, {role: defaults[role] for role in on}


class Schedule:
    """What to set and when, in seconds since the vehicle armed."""

    def __init__(self, events: list, latitude_deg: float, defaults: Dict[str, float]) -> None:
        self._steps: List[Tuple[float, Dict[str, float], dict]] = []
        for event in check_events(events):
            on, off = changes_for(event, latitude_deg, defaults)
            self._steps.append((event["at_s"], on, {"event": event["kind"], "what": "on", **{k: v for k, v in event.items() if k not in ("kind", "for_s")}}))
            if event["for_s"] is not None and off:
                self._steps.append((event["at_s"] + event["for_s"], off, {"event": event["kind"], "what": "off", "at_s": event["at_s"] + event["for_s"]}))
        self._steps.sort(key=lambda step: step[0])
        self._next = 0

    def __len__(self) -> int:
        return len(self._steps)

    def due(self, since_arm_s: float) -> List[Tuple[Dict[str, float], dict]]:
        """The steps whose time has come, in order, each once."""
        found = []
        while self._next < len(self._steps) and self._steps[self._next][0] <= since_arm_s:
            _at, values, about = self._steps[self._next]
            found.append((values, about))
            self._next += 1
        return found

    @property
    def done(self) -> bool:
        return self._next >= len(self._steps)


def fix_step_m(before: Tuple[float, float], after: Tuple[float, float]) -> Tuple[float, float]:
    """(north, east) in metres from one raw fix (latitude, longitude in degrees) to another."""
    north = (after[0] - before[0]) * METRES_PER_DEGREE
    east = (after[1] - before[1]) * METRES_PER_DEGREE * math.cos(math.radians(before[0]))
    return north, east


def glitch_result(wanted_ne: Tuple[float, float], fixes: List[Tuple[float, float, float]], t_set: float, before: Tuple[float, float]) -> dict:
    """Whether a glitch showed in the raw fix: the first fix within 1 s of the command that has moved to within 0.5 m of the step asked for.

    `fixes` are (time, latitude, longitude) of the raw GPS after the command;
    `before` the last raw fix before it. The vehicle's own movement in that
    second is part of what is seen, so a glitch is judged in a hover.
    """
    for t, latitude, longitude in fixes:
        if t - t_set > GLITCH_PASS_S:
            break
        north, east = fix_step_m(before, (latitude, longitude))
        if math.hypot(north - wanted_ne[0], east - wanted_ne[1]) <= GLITCH_PASS_M:
            return {"passed": True, "after_s": round(t - t_set, 3), "moved_north_m": round(north, 3), "moved_east_m": round(east, 3)}
    last = fixes[-1] if fixes else None
    moved = (None, None) if last is None else fix_step_m(before, (last[1], last[2]))
    return {"passed": False, "after_s": None, "moved_north_m": None if moved[0] is None else round(moved[0], 3), "moved_east_m": None if moved[1] is None else round(moved[1], 3)}


def wanted(manifest: dict) -> bool:
    return bool(manifest.get("gps_events"))


def read_gps_events(folder) -> Optional[dict]:
    try:
        report = json.loads((Path(folder) / "gps_events.json").read_text())
    except (OSError, json.JSONDecodeError):
        return None
    return report if isinstance(report, dict) and isinstance(report.get("passed"), bool) else None


def summary_line(report: dict) -> str:
    glitches = [item for item in report.get("glitches", [])]
    shown = sum(1 for item in glitches if item.get("passed"))
    text = f"{report.get('applied', 0)} of {report.get('scheduled', 0)} changes made"
    if glitches:
        first = glitches[0]
        moved = "" if first.get("moved_north_m") is None else f"; the first moved the raw fix {first['moved_north_m']:+.2f} m north, {first['moved_east_m']:+.2f} m east"
        after = "" if first.get("after_s") is None else f" {first['after_s']:.2f} s after the command"
        text += f"; {shown} of {len(glitches)} glitch(es) seen in the raw fix{moved}{after}"
    return text


def build_report(scheduled: int, applied: List[dict], names: Dict[str, str], armed: bool) -> dict:
    """What the flight's faults came to. Passed: the vehicle armed, every scheduled change was made, and every glitch showed in the raw fix."""
    glitches = [item["seen"] for item in applied if item.get("seen") is not None]
    problems = []
    if not armed:
        problems.append("the vehicle never armed, so no event's time came")
    elif len(applied) < scheduled:
        problems.append(f"only {len(applied)} of {scheduled} scheduled changes were made before the flight ended")
    for item in applied:
        if item.get("confirmed") is False:
            problems.append(f"the flight controller did not confirm {item['event']} ({item['what']}) at {item['since_arm_s']:.1f} s")
    for index, seen in enumerate(glitches):
        if not seen["passed"]:
            problems.append(f"glitch {index + 1} did not show in the raw GPS fix within {GLITCH_PASS_S:g} s to {GLITCH_PASS_M} m")
    return {"passed": not problems, "problems": problems, "scheduled": scheduled, "applied": len(applied), "glitches": glitches, "parameters": names, "changes": applied}


# --- the part that talks to SITL and Gazebo ----------------------------------------


def names_complete(have: set, needed: frozenset = frozenset()) -> bool:
    """True once every fault has a parameter name among `have`: the injector need not wait for the other family's names, which never answer.

    An optional role is waited for only when it is `needed`.
    """
    return all(any(name in have for name in candidates) for role, candidates in PARAMETER_NAMES.items() if role not in OPTIONAL_ROLES or role in needed)


def _read_parameters(master, wanted_names: List[str], on_heartbeat, needed: frozenset = frozenset(), timeout_s: float = 6.0) -> Dict[str, float]:
    """{name: value} of those of `wanted_names` this SITL has. One that does not exist gets no answer.

    Returns as soon as every fault has a name. Heartbeats heard meanwhile go
    to `on_heartbeat`, so that an arming during the read is not seen late.
    """
    found: Dict[str, float] = {}
    deadline = time.monotonic() + timeout_s
    asked_at = 0.0
    while time.monotonic() < deadline and not names_complete(set(found), needed):
        if time.monotonic() - asked_at > 2.0:
            for name in wanted_names:
                if name not in found:
                    master.mav.param_request_read_send(FCU_SYSTEM, FCU_COMPONENT, name.encode(), -1)
            asked_at = time.monotonic()
        message = master.recv_match(type=["PARAM_VALUE", "HEARTBEAT"], blocking=True, timeout=0.2)
        if message is None:
            continue
        if message.get_type() == "HEARTBEAT":
            on_heartbeat(message)
            continue
        name = message.param_id if isinstance(message.param_id, str) else message.param_id.decode()
        name = name.rstrip("\x00")
        if name in wanted_names:
            found[name] = float(message.param_value)
    return found


def run(run_dir: Path, url: str = "udpin:127.0.0.1:14553") -> dict:
    """Inject the run's gps_events until interrupted; write gps_events.jsonl as it goes and gps_events.json at the end."""
    import signal
    import threading

    from gz.msgs10.clock_pb2 import Clock
    from gz.transport13 import Node
    from pymavlink import mavutil

    manifest = json.loads((run_dir / "scenario.json").read_text())
    events = check_events(manifest.get("gps_events") or [])
    clock = {"t": 0.0}
    node = Node()
    if not node.subscribe(Clock, "/clock", lambda message: clock.__setitem__("t", message.sim.sec + message.sim.nsec * 1.0e-9)):
        raise RuntimeError("could not subscribe to Gazebo's clock")

    master = mavutil.mavlink_connection(url, source_system=246, source_component=194)
    armed = {"at": None}  # the sim time the flight controller was first heard armed

    def on_heartbeat(message) -> None:
        if (message.get_srcSystem(), message.get_srcComponent()) != (FCU_SYSTEM, FCU_COMPONENT):
            return
        if armed["at"] is None and message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            armed["at"] = clock["t"]
            print(f"gps events: armed at sim time {clock['t']:.2f} s", flush=True)

    if master.wait_heartbeat(timeout=30) is None:
        raise RuntimeError(f"no heartbeat from the flight controller on {url}")
    every_name = [name for candidates in PARAMETER_NAMES.values() for name in candidates]
    needed = frozenset(roles_needed(events))
    every_name = [name for name in dict.fromkeys(every_name) if not any(name in PARAMETER_NAMES[role] for role in OPTIONAL_ROLES if role not in needed)]
    have = _read_parameters(master, every_name, on_heartbeat, needed)
    names = resolve_names(set(have), needed)  # raises InjectError, and the flight is not flown as if it had faults
    defaults = {role: have[name] for role, name in names.items()}
    print(f"gps events: {len(events)} event(s); this SITL's parameters: {', '.join(sorted(names.values()))}", flush=True)

    log = open(run_dir / "gps_events.jsonl", "w")
    applied: List[dict] = []
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    schedule: Optional[Schedule] = None
    last_fix: Optional[Tuple[float, float, float]] = None  # sim time, latitude, longitude of the newest raw fix
    watching: List[dict] = []  # glitches whose effect on the raw fix is still being watched
    raw: dict = {}  # the newest raw fix's satellite count, fix type and altitude, and when it was heard
    settling: List[dict] = []  # changes whose raw fix a second later is still to be noted
    asked_at = 0.0

    def set_parameters(values: Dict[str, float]) -> bool:
        confirmed = True
        for role, value in values.items():
            name = names[role]
            master.mav.param_set_send(FCU_SYSTEM, FCU_COMPONENT, name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        deadline = time.monotonic() + 1.0
        waiting = {names[role]: float(value) for role, value in values.items()}
        while waiting and time.monotonic() < deadline:
            message = master.recv_match(type=["PARAM_VALUE", "HEARTBEAT"], blocking=True, timeout=0.1)
            if message is not None and message.get_type() == "HEARTBEAT":
                on_heartbeat(message)
            elif message is not None:
                name = (message.param_id if isinstance(message.param_id, str) else message.param_id.decode()).rstrip("\x00")
                if name in waiting and math.isclose(message.param_value, waiting[name], rel_tol=1e-5, abs_tol=1e-9):
                    del waiting[name]
        if waiting:
            confirmed = False
        return confirmed

    def set_flight_mode(name: str) -> bool:
        """Ask for a flight mode and wait up to 2 s to hear the flight controller in it."""
        master.set_mode(name)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            message = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.2)
            if message is not None and (message.get_srcSystem(), message.get_srcComponent()) == (FCU_SYSTEM, FCU_COMPONENT):
                on_heartbeat(message)
                if mavutil.mode_string_v10(message) == name:
                    return True
        return False

    while not stopping.is_set():
        if time.monotonic() - asked_at > 2.0:  # the raw fix at 10 Hz on this link, and keep asking
            master.mav.command_long_send(
                FCU_SYSTEM, FCU_COMPONENT, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1.0e5, 0, 0, 0, 0, 0,
            )  # fmt: skip
            asked_at = time.monotonic()
        message = master.recv_match(type=["HEARTBEAT", "GPS_RAW_INT"], blocking=True, timeout=0.05)
        now = clock["t"]
        armed_at = armed["at"]
        if message is not None and message.get_type() == "HEARTBEAT":
            on_heartbeat(message)
        elif message is not None and message.get_type() == "GPS_RAW_INT":
            last_fix = (now, message.lat * 1.0e-7, message.lon * 1.0e-7)
            raw.update(satellites=int(message.satellites_visible), fix_type=int(message.fix_type), altitude_m=round(message.alt * 1.0e-3, 2), heard_at=round(now, 3))
            for item in watching:
                item["fixes"].append(last_fix)
        for item in list(watching):  # a glitch is judged once its second is up, or as soon as it shows
            seen = glitch_result(item["wanted"], item["fixes"], item["t_set"], item["before"])
            if seen["passed"] or now - item["t_set"] > GLITCH_PASS_S + 0.5:
                item["record"]["seen"] = seen
                watching.remove(item)
                log.write(json.dumps({"t": round(now, 3), "event": "glitch_seen", **seen}) + "\n")
                log.flush()
                print(f"gps events: glitch {'seen' if seen['passed'] else 'NOT seen'} in the raw fix: {seen}", flush=True)
        for record in list(settling):  # what the raw fix says a second after each change (a dropout: nothing newer is heard)
            if now - record["t"] >= RAW_AFTER_S:
                record["raw_after"] = dict(raw)
                settling.remove(record)
                log.write(json.dumps({"t": round(now, 3), "event": "raw_fix", "after": f"{record['event']} {record['what']}", **raw}) + "\n")
                log.flush()
        if armed_at is None or last_fix is None:
            continue
        if schedule is None:  # a glitch in metres needs the latitude, which the first raw fix gives
            schedule = Schedule(events, last_fix[1], defaults)
        for values, about in schedule.due(now - armed_at):
            before = last_fix
            confirmed = set_parameters(values)
            if about["event"] == "mode":
                confirmed = set_flight_mode(str(about["mode"]))
                print(f"gps events: flight mode {about['mode']} {'taken' if confirmed else 'NOT taken'} at {now - armed_at:.1f} s after arming", flush=True)
            record = {"t": round(clock["t"], 3), "since_arm_s": round(now - armed_at, 3), **about, "set": {names[role]: value for role, value in values.items()}, "confirmed": confirmed, "seen": None}
            if about["event"] == "glitch" and about["what"] == "on" and before is not None:
                wanted_ne = (float(about.get("north_m", 0.0)), float(about.get("east_m", 0.0)))
                watching.append({"wanted": wanted_ne, "fixes": [], "t_set": now, "before": (before[1], before[2]), "record": record})
            record["raw_before"] = dict(raw)
            settling.append(record)
            applied.append(record)
            log.write(json.dumps({key: value for key, value in record.items() if key not in ("seen", "raw_after")}) + "\n")
            log.flush()
            print(f"gps events: {about['event']} {about['what']} at {now - armed_at:.1f} s after arming ({'confirmed' if confirmed else 'NOT confirmed'})", flush=True)

    # Leave the simulated GPS as it was found: the next flight in this container must start clean.
    set_parameters(defaults)
    for item in watching:
        item["record"]["seen"] = glitch_result(item["wanted"], item["fixes"], item["t_set"], item["before"])
    for record in settling:
        record["raw_after"] = dict(raw)
    report = build_report(len(schedule) if schedule is not None else len(events), applied, names, armed["at"] is not None)
    report["armed_at_s"] = armed["at"]  # sim time; the events' times count from here
    log.close()
    (run_dir / "gps_events.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main(argv: Optional[List[str]] = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__)
        return 2
    if os.environ.get("PYTHONNOUSERSITE"):  # pymavlink is installed for the user in the sim image
        environment = {key: value for key, value in os.environ.items() if key != "PYTHONNOUSERSITE"}
        os.execve(sys.executable, [sys.executable, os.path.abspath(__file__), *argv], environment)
    run_dir = Path(argv[0])
    try:
        report = run(run_dir)
    except (InjectError, RuntimeError) as error:
        # Nothing was injected. The report says so, and the runner calls the flight INVALID, not faulted.
        report = {"passed": False, "problems": [str(error)], "scheduled": 0, "applied": 0, "glitches": [], "parameters": {}, "changes": []}
        (run_dir / "gps_events.json").write_text(json.dumps(report, indent=2) + "\n")
        print(f"gps events: {error}", file=sys.stderr, flush=True)
        return 1
    print("gps events: " + summary_line(report), flush=True)
    for problem in report["problems"]:
        print(f"  {problem}", flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
