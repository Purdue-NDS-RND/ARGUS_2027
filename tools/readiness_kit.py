#!/usr/bin/env python3
"""The readiness kit (OPS-02): from the geofence demonstration's attempt bundle, the proof that the fence acted.

  python3 tools/readiness_kit.py attempts/<date>_<slot>_geofence
  python3 tools/readiness_kit.py attempts/<date>_<slot>_geofence --checklist     # only print the video checklist

The demonstration flight is the sequence fence_demo.yaml flown with the
profile fence_demo through tools/go.py --record, and bundled with
tools/bundle_attempt.py --mission geofence (docs/ops/readiness_video.md has
the whole procedure and what to film). This tool reads the bundle and writes
`readiness.md` into it: what the flight controller's own log says happened,
line by line, and the checklist for the video with the lines the log can
answer already filled in.

What the log must show for the kit to say READY:

  - the fence was on in the flight controller's parameters: FENCE_ENABLE 1, an
    action that stops the flight, a polygon (FENCE_TYPE) with points stored;
  - the fence stopped the vehicle while it was under autonomous command
    (GUIDED), in one of the two ways ArduCopter does it:
      breached  an ERR message of the fence subsystem with a breach code, and
                within RESPONSE_LIMIT_S of it a mode that brings the vehicle
                back or stops it, changed to by the fence's own action (the
                mode change's reason says whose it was);
      held      no breach: for HOLD_MIN_S or more the flight controller's
                avoidance allowed no velocity (its SA messages) while a
                velocity was being commanded (the guided target, GUIP), with
                the vehicle inside the fence polygon the log itself holds
                (FNCE) and no further than HOLD_NEAR_M from it, and a mode
                that brings the vehicle home after it;
  - the bundle is complete and is as its manifest says, the preflight said GO
    without an override, and the flag profile flown was fence_demo.

Anything missing is listed and the verdict is NOT READY (exit code 1).
readiness.md is written either way. Exit code 2: not a bundle.

No ROS imports. The log is read with tools/dataflash_params.py.
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import bundle_attempt  # noqa: E402
import dataflash_params  # noqa: E402

CHECKLIST = ROOT / "docs" / "ops" / "readiness_video.md"
RESPONSE_LIMIT_S = 2.0  # the flight controller's own mode change, after its report of the breach
HOLD_MIN_S = 5.0  # a hold at the fence: the avoidance keeps the vehicle still against a commanded velocity this long
HOLD_COMMANDED_MPS = 1.0  # a commanded speed of at least this, cut to
HOLD_STILL_MPS = 0.3  # no more than this
HOLD_NEAR_M = 10.0  # and the vehicle no further than this inside the fence line
SA_GAP_S = 1.0  # avoidance messages further apart than this are two holds, not one
FENCE_POINT_INCLUSION = 98  # FNCE Type of an inclusion polygon's vertex (AC_PolyFenceType::POLYGON_INCLUSION)
EARTH_RADIUS_M = 6378137.0
ERR_SUBSYSTEM_FENCE = 9  # ArduPilot LogErrorSubsystem::FAILSAFE_FENCE
MODE_REASON_FENCE = 10  # ArduPilot ModeReason::FENCE_BREACHED: the flight controller's own change of mode on a breach
# Both read off a real breach's log (ArduCopter 4.6.3 in SITL, docs/results/2026-11-23-readiness-demo.md), which
# also showed the order: the mode change is written first, then the error, with the same time stamp.
RESPONSE_BEFORE_S = 0.5  # so the fence's mode change is looked for from this long before its error message
# ArduCopter flight mode numbers.
GUIDED = 4
STOPPING_MODES = {6: "RTL", 9: "LAND", 17: "BRAKE", 21: "SMART_RTL"}
MODE_NAMES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND", 16: "POSHOLD", 17: "BRAKE", 21: "SMART_RTL"}
FENCE_BREACHES = {1: "the ceiling", 2: "the circle", 4: "the polygon", 8: "the floor"}  # the breach code is a sum of these
STOPPING_ACTIONS = (1, 2, 3, 4, 5)  # FENCE_ACTION values that end the flight (sim/site_file.py FENCE_ACTIONS)
FENCE_TYPE_POLYGON = 4


class KitError(ValueError):
    """Not an attempt's bundle. The message says why."""


@dataclass
class LogStory:
    """What a dataflash log says about a fence demonstration."""

    params: Dict[str, float] = field(default_factory=dict)
    firmware: Optional[str] = None
    modes: List[Tuple[float, int, int]] = field(default_factory=list)  # (seconds, mode number, reason)
    breaches: List[Tuple[float, int]] = field(default_factory=list)  # (seconds, breach code)
    cleared: List[float] = field(default_factory=list)  # seconds at which the fence error cleared (back inside)
    avoidance: List[Tuple[float, float]] = field(default_factory=list)  # (seconds, the speed the avoidance allowed, m/s), while it was limiting
    commanded: List[Tuple[float, float]] = field(default_factory=list)  # (seconds, the speed of the velocity target the companion gave, m/s)
    polygon: List[Tuple[float, float]] = field(default_factory=list)  # the inclusion fence's corners, (latitude, longitude) in degrees
    positions: List[Tuple[float, float, float]] = field(default_factory=list)  # (seconds, latitude, longitude)


def read_log(path) -> LogStory:
    """The fence's parameters, every mode change and every fence message of a dataflash log."""
    story = LogStory()
    try:
        parameters = dataflash_params.read_parameters(path)
    except dataflash_params.DataflashError as problem:
        raise KitError(str(problem)) from problem
    story.params, story.firmware = parameters.values, parameters.firmware
    for name, row in dataflash_params.messages(path, ("MODE", "ERR", "SA", "GUIP", "FNCE", "POS")):
        seconds = float(row.get("TimeUS", 0)) * 1.0e-6
        if name == "SA" and "MVelX" in row:
            # Simple avoidance, logged while it limits the velocity. Both of its velocities are the limited one
            # (the "desired" it logs has already been cut on the tick before); what was asked for is in GUIP.
            if int(row.get("State", 0)):
                story.avoidance.append((seconds, math.hypot(row["MVelX"], row["MVelY"])))
            continue
        if name == "GUIP" and "vX" in row:
            story.commanded.append((seconds, math.hypot(row["vX"], row["vY"]) * 0.01))  # the guided target, in cm/s
            continue
        if name == "FNCE" and "Lat" in row:
            if int(row.get("Type", -1)) == FENCE_POINT_INCLUSION:
                if int(row.get("Seq", 0)) == 0:
                    story.polygon = []  # the fence written again: the last one stands
                story.polygon.append((row["Lat"] * 1.0e-7, row["Lng"] * 1.0e-7))
            continue
        if name == "POS" and "Lat" in row:
            story.positions.append((seconds, row["Lat"] * 1.0e-7, row["Lng"] * 1.0e-7))
            continue
        if name == "MODE" and "ModeNum" in row:
            story.modes.append((seconds, int(row["ModeNum"]), int(row.get("Rsn", -1))))
        elif name == "ERR" and int(row.get("Subsys", -1)) == ERR_SUBSYSTEM_FENCE:
            code = int(row.get("ECode", 0))
            if code:
                story.breaches.append((seconds, code))
            else:
                story.cleared.append(seconds)
    return story


def breach_names(code: int) -> str:
    named = [name for bit, name in FENCE_BREACHES.items() if code & bit]
    return " and ".join(named) if named else f"code {code}"


def mode_at(story: LogStory, seconds: float) -> Optional[int]:
    """The mode the vehicle was in at a time: the last mode message at or before it."""
    current = None
    for when, mode, _reason in story.modes:
        if when <= seconds:
            current = mode
    return current


def holds(story: LogStory) -> List[Tuple[float, float, float]]:
    """Stretches in which the avoidance kept the vehicle still against a commanded velocity: (start, end, speed commanded)."""
    stretches, start, last = [], None, None
    for seconds, allowed in story.avoidance:
        still = allowed <= HOLD_STILL_MPS
        if start is not None and (not still or seconds - last > SA_GAP_S):
            stretches.append((start, last))
            start = None
        if still:
            start = seconds if start is None else start
            last = seconds
    if start is not None:
        stretches.append((start, last))
    found = []
    for start, end in stretches:
        asked = sorted(speed for seconds, speed in story.commanded if start <= seconds <= end)
        if asked and asked[len(asked) // 2] >= HOLD_COMMANDED_MPS:  # the median of what was commanded through the stretch
            found.append((start, end, asked[len(asked) // 2]))
    return found


def inside_by_m(polygon: List[Tuple[float, float]], latitude: float, longitude: float) -> Optional[float]:
    """How far inside a fence polygon a point is, in metres; negative when it is outside. None with no polygon."""
    if len(polygon) < 3:
        return None
    lat0 = math.radians(polygon[0][0])

    def metres(point):
        return (math.radians(point[1] - polygon[0][1]) * math.cos(lat0) * EARTH_RADIUS_M, math.radians(point[0] - polygon[0][0]) * EARTH_RADIUS_M)

    corners = [metres(point) for point in polygon]
    x, y = metres((latitude, longitude))
    inside, nearest = False, None
    for index, a in enumerate(corners):
        b = corners[(index + 1) % len(corners)]
        if (a[1] > y) != (b[1] > y) and x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]:
            inside = not inside
        length2 = (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
        along = 0.0 if length2 == 0.0 else max(0.0, min(1.0, ((x - a[0]) * (b[0] - a[0]) + (y - a[1]) * (b[1] - a[1])) / length2))
        distance = math.hypot(x - (a[0] + along * (b[0] - a[0])), y - (a[1] + along * (b[1] - a[1])))
        nearest = distance if nearest is None else min(nearest, distance)
    return nearest if inside else -nearest


def judge_hold(story: LogStory) -> Tuple[List[str], List[str]]:
    """The log's lines for a flight with no breach: was the vehicle held at the fence by the flight controller?"""
    shown, missing = [], []
    long_holds = [hold for hold in holds(story) if hold[1] - hold[0] >= HOLD_MIN_S and mode_at(story, hold[0]) == GUIDED]
    if not long_holds:
        missing.append(
            "the log has no fence breach (no ERR message of the fence subsystem) and no hold at the fence "
            f"(the avoidance keeping the vehicle still against a commanded velocity for {HOLD_MIN_S:g} s in GUIDED): the vehicle did not reach the fence, or the fence did not act"
        )
        return shown, missing
    start, end, commanded = long_holds[0]
    shown.append(f"from {start:.1f} s, for {end - start:.1f} s, the flight controller's avoidance held the vehicle still with {commanded:.1f} m/s commanded, in GUIDED: no breach")
    during = [inside_by_m(story.polygon, latitude, longitude) for seconds, latitude, longitude in story.positions if start <= seconds <= end]
    during = [value for value in during if value is not None]
    if not during:
        missing.append("the log does not say where the vehicle was held (no fence polygon in FNCE messages, or no position during the hold)")
    elif min(during) < 0.0:
        missing.append(f"during the hold the vehicle was {-min(during):.1f} m outside the fence polygon in the log")
    elif min(during) > HOLD_NEAR_M:
        missing.append(f"the vehicle was held {min(during):.1f} m inside the fence line: not at the fence (within {HOLD_NEAR_M:g} m)")
    else:
        shown.append(f"it was held {min(during):.1f} m inside the fence line, by the polygon of {len(story.polygon)} corners the log itself holds")
    after = [(t, mode, reason) for t, mode, reason in story.modes if t >= end and mode in STOPPING_MODES]
    if after:
        shown.append(f"{after[0][0] - end:.1f} s after the hold the flight controller was in {STOPPING_MODES[after[0][1]]} (mode change reason {after[0][2]}), on the way home")
    else:
        missing.append("after the hold the log shows no mode that brings the vehicle home")
    return shown, missing


def judge_log(story: LogStory) -> Tuple[List[str], List[str]]:
    """(what the log shows, what it does not): the lines of the kit's verdict on the flight controller's log."""
    shown, missing = [], []
    p = story.params
    if p.get("FENCE_ENABLE") == 1:
        shown.append("FENCE_ENABLE is 1: the fence was on")
    else:
        missing.append(f"FENCE_ENABLE is {p.get('FENCE_ENABLE', 'not in the log')}: the fence was not on")
    action = p.get("FENCE_ACTION")
    if action in STOPPING_ACTIONS:
        shown.append(f"FENCE_ACTION is {action:g}: a breach ends the flight")
    else:
        missing.append(f"FENCE_ACTION is {action if action is not None else 'not in the log'}: not an action that ends the flight (one of {', '.join(map(str, STOPPING_ACTIONS))})")
    kind, points = p.get("FENCE_TYPE"), p.get("FENCE_TOTAL")
    if kind is not None and int(kind) & FENCE_TYPE_POLYGON and points is not None and points >= 3:
        shown.append(f"FENCE_TYPE {kind:g} has the polygon, and {points:g} fence points were stored")
    else:
        missing.append(f"no polygon fence in the parameters (FENCE_TYPE {kind if kind is not None else 'not in the log'}, FENCE_TOTAL {points if points is not None else 'not in the log'})")
    if not story.breaches:
        hold_shown, hold_missing = judge_hold(story)
        return shown + hold_shown, missing + hold_missing
    held_first = [hold for hold in holds(story) if hold[1] <= story.breaches[0][0] and hold[1] - hold[0] >= 1.0 and mode_at(story, hold[0]) == GUIDED]
    if held_first:
        start, end, commanded = held_first[-1]
        shown.append(f"before the breach the flight controller's avoidance held the vehicle still for {end - start:.1f} s with {commanded:.1f} m/s commanded")
    when, code = story.breaches[0]
    # The fence's own answer: a stopping mode whose reason is the fence, from just before the error message on.
    answers = [(t, mode, reason) for t, mode, reason in story.modes if t >= when - RESPONSE_BEFORE_S and mode in STOPPING_MODES]
    fences = [answer for answer in answers if answer[2] == MODE_REASON_FENCE]
    before = mode_at(story, (fences[0][0] if fences else when) - 1.0e-6)  # the mode the breach happened in, not the answer to it
    shown.append(f"at {when:.1f} s the flight controller reported a breach of {breach_names(code)}")
    if before == GUIDED:
        shown.append("the vehicle was in GUIDED, under the companion computer's command, when it was breached")
    else:
        missing.append(f"the vehicle was in {MODE_NAMES.get(before, before)} at the breach, not in GUIDED: this was not the autonomous stack flying at the fence")
    if fences:
        t_mode, mode, reason = fences[0]
        delay = max(0.0, t_mode - when)
        line = f"{delay:.2f} s later the flight controller was in {STOPPING_MODES[mode]} by its own fence action (mode change reason {reason})"
        if delay <= RESPONSE_LIMIT_S:
            shown.append(line)
        else:
            missing.append(f"{line}: later than {RESPONSE_LIMIT_S:g} s")
    elif answers:
        t_mode, mode, reason = answers[0]
        missing.append(f"{max(0.0, t_mode - when):.2f} s after the breach the vehicle was in {STOPPING_MODES[mode]}, but by mode change reason {reason}, not the fence's own ({MODE_REASON_FENCE}): somebody else brought it back")
    else:
        missing.append("after the breach the flight controller changed to no mode that brings the vehicle back or stops it")
    if story.cleared and story.cleared[0] > when:
        shown.append(f"{story.cleared[0] - when:.1f} s after the breach the vehicle was back inside the fence")
    return shown, missing


def judge_bundle(folder: Path, manifest: dict) -> Tuple[List[str], List[str]]:
    """(what the bundle shows, what it does not), apart from the log."""
    shown, missing = [], []
    if manifest.get("mission") != "geofence":
        missing.append(f"the bundle is of a {manifest.get('mission')} attempt, not of the geofence demonstration (bundle_attempt.py --mission geofence)")
    if manifest.get("complete"):
        shown.append("the bundle is complete: the bag, the flight controller's log, the config and the preflight's log")
    else:
        missing.append("the bundle is incomplete: " + ", ".join(manifest.get("owed") or []))
    problems = bundle_attempt.verify(folder)
    if problems:
        missing += [f"the bundle is not as its manifest says: {problem}" for problem in problems]
    else:
        shown.append("every file is as the manifest recorded it")
    preflight = (manifest.get("parts") or {}).get("preflight") or {}
    if preflight.get("verdict") == "GO" and not preflight.get("override"):
        shown.append("the preflight said GO, with no override")
    else:
        missing.append(f"the preflight's last verdict was {preflight.get('verdict')}" + (" (overridden)" if preflight.get("override") else "") + ", not a clean GO")
    profile = manifest.get("profile") or {}
    if profile.get("flags") == ["fence_demo"]:
        shown.append("the flag profile flown was fence_demo, and nothing else was on")
    else:
        missing.append(f"the flag profile in the bundle turns on {profile.get('flags')}, not fence_demo alone")
    git = manifest.get("git") or {}
    if git.get("hash"):
        shown.append(f"the code was at {git.get('describe') or git['hash']}" + (" with uncommitted changes" if git.get("dirty") else ""))
    else:
        missing.append("the manifest has no git commit")
    return shown, missing


def checklist_text() -> str:
    """The video checklist's own lines (docs/ops/readiness_video.md, from its "## The video" heading on)."""
    text = CHECKLIST.read_text(encoding="utf-8")
    start = text.index("## The video")
    return text[start:].rstrip() + "\n"


def report(folder: Path) -> Tuple[str, bool]:
    """readiness.md's text for a bundle, and whether the kit says READY."""
    try:
        manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as problem:
        raise KitError(f"{folder} is not an attempt's bundle: {problem}") from problem
    shown, missing = judge_bundle(folder, manifest)
    logs = sorted((folder / "dataflash").glob("*.BIN")) if (folder / "dataflash").is_dir() else []
    story = None
    if not logs:
        missing.append("the bundle has no dataflash log: the fence's own record of the flight is not there")
    else:
        story = read_log(logs[-1])
        log_shown, log_missing = judge_log(story)
        shown, missing = shown + log_shown, missing + log_missing
    ready = not missing
    out = [f"# Readiness: the geofence demonstration, {manifest.get('attempt')}", "",
           f"Written by `tools/readiness_kit.py` from the attempt's bundle. Verdict: **{'READY' if ready else 'NOT READY'}**.", ""]  # fmt: skip
    if story is not None:
        out += [f"Flight controller: {story.firmware or 'firmware line not in the log'}. Log: `dataflash/{logs[-1].name}`.", ""]
    out += ["## What the bundle and the flight controller's log show", ""] + [f"- {line}" for line in shown]
    if missing:
        out += ["", "## What is missing", ""] + [f"- {line}" for line in missing]
    out += ["", checklist_text()]
    return "\n".join(out), ready


def main(argv: Optional[List[str]] = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if "--checklist" in argv:
        print(checklist_text(), end="")
        return 0
    if len(argv) != 1:
        print(__doc__)
        return 2
    folder = Path(argv[0])
    try:
        text, ready = report(folder)
    except KitError as problem:
        print(f"readiness_kit: {problem}", file=sys.stderr)
        return 2
    (folder / "readiness.md").write_text(text, encoding="utf-8")
    body = text.split("## The video", 1)[0].rstrip()
    print(body)
    print(f"\nwrote {folder / 'readiness.md'} (with the video checklist)")
    print("READY" if ready else "NOT READY")
    return 0 if ready else 1


if __name__ == "__main__":
    sys.exit(main())
