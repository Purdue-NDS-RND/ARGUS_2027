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


class LandingWatcher:
    """The impulse a released body takes when it lands, from its true poses: a `contact` event (SIM-06).

    After `released(t)`, the body's velocity is followed by differencing
    its poses. An impact is a change of velocity too large for gravity and
    drag: the largest change seen across three consecutive poses, times the
    body's mass, is the impulse of its hardest contact. It is reported once,
    when the body has been still for a second: a package set down gently
    reports a contact too, with an impulse near zero, so that "the impulse
    was under the limit" can always be answered for a real package.
    """

    STILL_MPS = 0.05
    STILL_FOR_S = 1.0
    MIN_STEP_S = 0.005  # poses closer together than this are not differenced: the velocity would be noise

    def __init__(self, masses_kg: dict) -> None:
        self._masses = {name: float(mass) for name, mass in masses_kg.items()}
        self._poses: dict = {name: [] for name in self._masses}  # the last three (t, position) of each body
        self._velocities: dict = {name: [] for name in self._masses}  # and its last two velocities
        self._released_at: dict = {}
        self._largest: dict = {}
        self._still_since: dict = {}
        self._reported: set = set()

    def released(self, t: float, name: str | None = None) -> None:
        """A release was commanded at sim time `t`: watch the named body, or every body, from now."""
        for body in self._masses if name is None else [name]:
            if body in self._masses and body not in self._released_at:
                self._released_at[body] = t
                self._largest[body] = 0.0

    def update(self, name: str, t: float, position) -> dict | None:
        """One true pose of a body. Returns a `contact` event when its landing is over, once."""
        if name not in self._masses:
            return None
        poses = self._poses[name]
        if poses and t - poses[-1][0] < self.MIN_STEP_S:
            return None
        poses.append((t, tuple(float(value) for value in position)))
        if len(poses) < 2:
            return None
        (t0, p0), (t1, p1) = poses[-2], poses[-1]
        velocity = tuple((b - a) / (t1 - t0) for a, b in zip(p0, p1))
        velocities = self._velocities[name]
        velocities.append(velocity)
        del poses[:-3], velocities[:-3]
        if name not in self._released_at or name in self._reported or t < self._released_at[name]:
            return None
        if len(velocities) >= 3:  # across two steps: an impact inside one pose step is split over the two differences around it
            change = sum((after - before) ** 2 for before, after in zip(velocities[-3], velocities[-1])) ** 0.5
            self._largest[name] = max(self._largest[name], change)
        speed = sum(component * component for component in velocity) ** 0.5
        if speed > self.STILL_MPS:
            self._still_since.pop(name, None)
            return None
        since = self._still_since.setdefault(name, t)
        if t - since < self.STILL_FOR_S:
            return None
        self._reported.add(name)
        change = self._largest[name]
        return {"event": "contact", "name": name, "impulse_ns": round(self._masses[name] * change, 4), "velocity_change_mps": round(change, 3)}


def tools_needed(manifest: dict) -> set[str]:
    """Which sim tools a scenario needs running beside the flight: "wind", "gps_events", "time_check", "rate_check", "projection_check", "estimator_check", "record", "payload_bridge", "sensor_check", "truth_poses"."""
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
    if manifest.get("record") is True:
        needed.add("record")  # the flight's topics are recorded for replay (sim/run_scenarios.sh --record)
    bridge = manifest.get("payload_bridge")
    if isinstance(bridge, dict) and (bridge.get("carried") or bridge.get("bucket")):
        needed.add("payload_bridge")  # servo outputs let the carried payload go and take hold of the bucket
    if manifest.get("sensor_check") is True:
        needed.add("sensor_check")  # the camera's rate and the rangefinder are checked against the truth
    if manifest.get("truth_poses") is True:
        needed.add("truth_poses")  # true poses go out on /sim/truth/poses
    return needed


def unsupported(manifest: dict, tools_present) -> str:
    """Why this scenario cannot be flown faithfully yet, or "" if it can.

    A scenario that asks for wind or GPS faults must not be flown calm and
    reported as if it had them.
    """
    reasons = {
        "wind": "it asks for wind, and the tool that blows it (sim/wind.py) is missing",
        "gps_events": "it has GPS events, and the tool that injects them (sim/inject.py) is missing",
        "time_check": "it runs on sim time, and the time check that proves it (sim/time_check.py) is missing",
        "rate_check": "it runs on sim time, and the rate check for MAVROS's messages (sim/rate_check.py) is missing",
        "projection_check": "it turns use_frame_time_pose on, and the check of projection v2 against the truth (sim/projection_check.py) is missing",
        "estimator_check": "it turns use_target_estimator on, and the check of the estimator against the truth (sim/estimator_check.py) is missing",
        "record": "a recording was asked for, and the recorder (sim/replay/record_run.py) is missing",
        "payload_bridge": "it has a carried payload or a bucket to pick up, and the payload bridge (sim/payload_bridge.py) is missing",
        "sensor_check": "it asks for the sensor check, and the tool (sim/sensor_check.py) is missing",
        "truth_poses": "it asks for true poses on a topic, and the tool that publishes them (sim/truth_poses.py) is missing",
    }
    missing = tools_needed(manifest) - set(tools_present)
    return "; ".join(reasons[name] for name in sorted(missing, reverse=True))
