"""Turn what the mission, the autopilot and the ground link say into trace events. No ROS."""

from __future__ import annotations

import re

# Mission log lines that mean the payload was let go. "Fake" lines are the
# 2026 fake_drop path: nothing is commanded and the judge scores the vehicle's
# position at that moment.
_RELEASE_LINES = (
    (re.compile(r"Fake (delivery|drop) complete"), True),
    (re.compile(r"Releasing gripper payload"), False),
    (re.compile(r"Closing sprayer for payload drop"), False),
)


def from_mission_log(text: str) -> list[dict]:
    """Events in one line of the mission executor's log."""
    for pattern, fake in _RELEASE_LINES:
        if pattern.search(text):
            return [{"event": "release", "fake": fake, "what": text}]
    return []


def from_statustext(text: str) -> list[dict]:
    """Events in one STATUSTEXT from the autopilot."""
    lower = text.lower()
    if "fence breached" in lower:
        return [{"event": "fence_breach", "what": text}]
    # "PrecLand: Failsafe Measures" is printed at every landing when precision landing is off.
    if re.search(r"\bfailsafe\b", lower) and not lower.startswith("precland") and "cleared" not in lower:
        return [{"event": "failsafe", "what": text}]
    return []


class ArmWatcher:
    """Report arm and disarm once each, from MAVROS state messages."""

    def __init__(self) -> None:
        self.armed = False

    def update(self, connected: bool, armed: bool) -> str | None:
        # While MAVROS thinks the link is down it publishes armed=false; that
        # says nothing about the vehicle.
        if not connected or armed == self.armed:
            return None
        self.armed = armed
        return "armed" if armed else "disarmed"


class ModeWatcher:
    """Report each change of flight mode once, from MAVROS state messages."""

    def __init__(self) -> None:
        self.mode: str | None = None

    def update(self, connected: bool, mode: str) -> str | None:
        if not connected or not mode or mode == self.mode:
            return None
        self.mode = mode
        return mode


# MAVLink system ids that are a person's ground station. MAVProxy and
# QGroundControl both use 255. Our own senders are 1 (MAVROS shares the
# vehicle's system id) and 245 (sim tools).
GROUND_STATION_SYSTEMS = frozenset({255})
RC_OVERRIDE = "RC_CHANNELS_OVERRIDE"
COMMAND_MESSAGES = frozenset(
    {
        "SET_MODE", "COMMAND_LONG", "COMMAND_INT", "MANUAL_CONTROL", "PARAM_SET",
        "MISSION_ITEM", "MISSION_ITEM_INT", "MISSION_COUNT", "MISSION_CLEAR_ALL", "MISSION_SET_CURRENT",
        "SET_POSITION_TARGET_LOCAL_NED", "SET_POSITION_TARGET_GLOBAL_INT", "SET_ATTITUDE_TARGET",
    }
)  # fmt: skip
# COMMAND_LONG ids a ground station sends by itself to ask for data, with nobody touching it.
# Seen from MAVProxy in the sim's own log: 410 (get home position), once at start-up.
HOUSEKEEPING_COMMANDS = frozenset({410, 511, 512, 519, 520})  # get home, set interval, request message, protocol version, capabilities


def human_inputs(messages, ground_stations=GROUND_STATION_SYSTEMS) -> list[dict]:
    """Ground-station commands and RC overrides among (wall time, system id, type[, command id]) messages."""
    found = []
    for message in messages:
        wall, system, kind = message[0], message[1], message[2]
        detail = message[3] if len(message) > 3 else None
        if system not in ground_stations:
            continue
        if kind == RC_OVERRIDE:
            found.append({"wall": wall, "event": "rc_override", "what": kind})
        elif kind in COMMAND_MESSAGES:
            if kind in ("COMMAND_LONG", "COMMAND_INT") and detail in HOUSEKEEPING_COMMANDS:
                continue
            what = kind if detail is None else f"{kind} {detail}"
            found.append({"wall": wall, "event": "gcs_command", "what": what})
    return found


def tools_needed(manifest: dict) -> set[str]:
    """Which sim tools a scenario needs running beside the flight: "wind", "gps_events", "time_check", "rate_check", "projection_check", "estimator_check"."""
    needed = set()
    wind = manifest.get("wind") or {}
    if wind.get("mean_mps") or wind.get("gust_mps") or wind.get("discrete_gust_mps"):
        needed.add("wind")
    if manifest.get("gps_events"):
        needed.add("gps_events")
    if manifest.get("sim_time") is True:
        needed.add("time_check")  # a run on sim time is checked to really be on one clock
        needed.add("rate_check")  # and to have the message rates the stream keeper holds
    if (manifest.get("flags") or {}).get("use_frame_time_pose") is True:
        needed.add("projection_check")  # projection v2's ground point is compared with the truth
    if (manifest.get("flags") or {}).get("use_target_estimator") is True:
        needed.add("estimator_check")  # the estimator's bullseye track is compared with the truth
    return needed


def unsupported(manifest: dict, tools_present) -> str:
    """Why this scenario cannot be flown faithfully yet, or "" if it can.

    A scenario that asks for wind or GPS faults must not be flown calm and
    reported as if it had them.
    """
    reasons = {
        "wind": "it asks for wind, which the sim does not apply yet (SIM-03)",
        "gps_events": "it has GPS events, which the sim cannot inject yet (SIM-04)",
        "time_check": "it runs on sim time, and the time check that proves it (sim/time_check.py) is missing",
        "rate_check": "it runs on sim time, and the rate check for MAVROS's messages (sim/rate_check.py) is missing",
        "projection_check": "it turns use_frame_time_pose on, and the check of projection v2 against the truth (sim/projection_check.py) is missing",
        "estimator_check": "it turns use_target_estimator on, and the check of the estimator against the truth (sim/estimator_check.py) is missing",
    }
    missing = tools_needed(manifest) - set(tools_present)
    return "; ".join(reasons[name] for name in sorted(missing, reverse=True))
