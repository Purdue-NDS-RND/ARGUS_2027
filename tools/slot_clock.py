#!/usr/bin/env python3
"""The slot clock (OPS-05): where the team is in its slot, and what is left.

  python3 tools/slot_clock.py start --slot 2              # the slot starts now (or --at 14:05)
  python3 tools/slot_clock.py show                        # one line, and the timeline
  python3 tools/slot_clock.py show --watch                # the line again every second: the ground laptop's clock
  python3 tools/slot_clock.py stop

A slot is a fixed length of time from the moment the team is called, cut
into phases in order (setup, preflight, flight, pack-up), and one deadline
after it: the localisation list, due 30 minutes after the slot. The lengths
are in tools/slot_clock.yaml (the slot's own length is an assumption until
the organisers give it).

`start` writes the slot's number and start time to a small file
(attempts/slot_clock.json, which git ignores); `show` and the GO launcher
(tools/go.py) read it and print the same line. The clock is the computer's
own: the companion follows the ground laptop's clock (platform/jetson/
clock_sync.py), so the laptop and the aircraft show the same times. On the
aircraft, `start --slot 2 --at <the time the laptop shows>` gives it the same
slot; or the launcher is given `--slot-start` itself.

The clock tells; it stops nothing. The launcher prints a warning when less of
the slot is left than the flight is budgeted, and goes on: whether to fly is
the lead's call.

No ROS imports.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

REPO = Path(__file__).resolve().parents[1]
BUDGET = Path(__file__).with_suffix(".yaml")
STATE = REPO / "attempts" / "slot_clock.json"
PHASES = ("setup", "preflight", "flight", "pack_up")


class ClockError(ValueError):
    """The budget or the slot's start cannot be used. The message says why."""


def load_budget(path: Path = BUDGET) -> Dict[str, Any]:
    import yaml

    try:
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as problem:
        raise ClockError(f"{path}: {problem}") from problem
    return check_budget(raw, str(path))


def check_budget(raw: Any, where: str = "the budget") -> Dict[str, Any]:
    if not isinstance(raw, dict) or set(raw) != {"slot_min", "phases", "list_due_after_slot_min"}:
        raise ClockError(f"{where}: expected slot_min, phases and list_due_after_slot_min, and nothing else")
    phases = raw["phases"]
    if not isinstance(phases, dict) or tuple(phases) != PHASES:
        raise ClockError(f"{where}: phases must be {', '.join(PHASES)}, in that order")
    numbers = {"slot_min": raw["slot_min"], "list_due_after_slot_min": raw["list_due_after_slot_min"], **{f"phases.{name}": value for name, value in phases.items()}}
    for name, value in numbers.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
            raise ClockError(f"{where}: {name} must be a number of minutes above zero, not {value!r}")
    if sum(phases.values()) > raw["slot_min"] + 1e-9:
        raise ClockError(f"{where}: the phases add up to {sum(phases.values()):g} min, and the slot is {raw['slot_min']:g} min")
    return {"slot_s": raw["slot_min"] * 60.0, "phases": [(name, phases[name] * 60.0) for name in PHASES], "list_due_s": raw["list_due_after_slot_min"] * 60.0}


def timeline(budget: Dict[str, Any], start: datetime) -> List[Tuple[str, datetime, datetime]]:
    """Each phase as planned (name, begins, ends), then the spare time to the slot's end if there is any, then the list's deadline."""
    rows, at = [], start
    for name, length in budget["phases"]:
        rows.append((name, at, at + timedelta(seconds=length)))
        at = rows[-1][2]
    end = start + timedelta(seconds=budget["slot_s"])
    if end > at:
        rows.append(("spare", at, end))
    rows.append(("list due", end, end + timedelta(seconds=budget["list_due_s"])))
    return rows


def status(budget: Dict[str, Any], start: datetime, now: datetime) -> Dict[str, Any]:
    """Where `now` is in the slot: the planned phase, what is left of it, of the slot and of the list's time."""
    elapsed = (now - start).total_seconds()
    slot_left = budget["slot_s"] - elapsed
    list_left = slot_left + budget["list_due_s"]
    if elapsed < 0:
        return {"phase": "before the slot", "elapsed_s": elapsed, "phase_left_s": -elapsed, "slot_left_s": slot_left, "list_left_s": list_left}
    phase, phase_left = "after the list's deadline", 0.0
    for name, begins, ends in timeline(budget, start):
        if begins <= now < ends:
            phase, phase_left = name, (ends - now).total_seconds()
            break
    return {"phase": phase, "elapsed_s": elapsed, "phase_left_s": phase_left, "slot_left_s": slot_left, "list_left_s": list_left}


def clock(seconds: float) -> str:
    """mm:ss, or h:mm:ss from an hour up; a time that has passed is written 'over by'."""
    whole = int(abs(seconds))
    text = f"{whole // 3600}:{whole % 3600 // 60:02d}:{whole % 60:02d}" if whole >= 3600 else f"{whole // 60:02d}:{whole % 60:02d}"
    return text if seconds >= 0 else f"over by {text}"


def line(slot: str, budget: Dict[str, Any], start: datetime, now: datetime) -> str:
    """The one line the launcher and the ground laptop show."""
    found = status(budget, start, now)
    if found["phase"] == "before the slot":
        return f"SLOT {slot}: starts in {clock(found['phase_left_s'])}"
    names = {"pack_up": "pack-up", "list due": "slot over: the list"}
    parts = [f"SLOT {slot}: {clock(found['elapsed_s'])} in"]
    if found["phase"] in ("list due", "after the list's deadline"):
        parts.append(f"slot over by {clock(-found['slot_left_s'])}")
        parts.append(f"list due in {clock(found['list_left_s'])}" if found["list_left_s"] >= 0 else f"LIST DEADLINE PASSED {clock(-found['list_left_s'])} ago")
        return " | ".join(parts)
    parts.append(f"planned: {names.get(found['phase'], found['phase'])}, {clock(found['phase_left_s'])} left")
    parts.append(f"slot ends in {clock(found['slot_left_s'])}")
    parts.append(f"list due in {clock(found['list_left_s'])}")
    return " | ".join(parts)


def flight_warning(budget: Dict[str, Any], start: datetime, now: datetime) -> Optional[str]:
    """What the launcher says when less of the slot is left than a flight and its pack-up are budgeted; None when there is room."""
    lengths = dict(budget["phases"])
    need, left = lengths["flight"] + lengths["pack_up"], budget["slot_s"] - (now - start).total_seconds()
    if left >= need:
        return None
    if left <= 0:
        return f"SLOT CLOCK: the slot ended {clock(-left)} ago"
    return f"SLOT CLOCK: {clock(left)} of the slot is left, and a flight with its pack-up is budgeted {clock(need)}"


def parse_start(text: str, now: datetime) -> datetime:
    """HH:MM or HH:MM:SS today on this computer's clock (the nearest such time: a start just before midnight read just after it is yesterday's), or an ISO time."""
    try:
        if "T" in text or "-" in text:
            found = datetime.fromisoformat(text)
            return found if found.tzinfo else found.astimezone()
        parts = [int(part) for part in text.split(":")]
        if len(parts) not in (2, 3) or not (0 <= parts[0] < 24 and all(0 <= part < 60 for part in parts[1:])):
            raise ValueError(text)
    except ValueError as problem:
        raise ClockError(f"'{text}' is not a time: HH:MM, HH:MM:SS or an ISO date and time") from problem
    local = now.astimezone()
    found = local.replace(hour=parts[0], minute=parts[1], second=parts[2] if len(parts) == 3 else 0, microsecond=0)
    if found - local > timedelta(hours=12):
        found -= timedelta(days=1)
    elif local - found > timedelta(hours=12):
        found += timedelta(days=1)
    return found


def read_state(path: Path = STATE) -> Optional[Dict[str, Any]]:
    """The running slot ({"slot", "start"}), or None when none was started."""
    try:
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        start = datetime.fromisoformat(raw["start"])
        if start.tzinfo is None:
            return None  # a start with no zone is not an instant (a file written by hand): not a running slot
        return {"slot": str(raw["slot"]), "start": start}
    except (OSError, ValueError, KeyError, TypeError):
        return None


def for_go(slot_start: Optional[str], slot: Optional[str], now: datetime, state: Path = STATE, budget_file: Path = BUDGET) -> Optional[Tuple[str, Optional[str], Dict[str, Any]]]:
    """For tools/go.py: (the line, the warning or None, what to record), from --slot-start or the running slot's file; None with neither."""
    if slot_start is not None:
        running = {"slot": slot or "?", "start": parse_start(slot_start, now)}
    else:
        running = read_state(state)
        if running is None:
            return None
    budget = load_budget(budget_file)
    found = status(budget, running["start"], now)
    record = {"slot": running["slot"], "start": running["start"].isoformat(), "at_go_s": round(found["elapsed_s"], 1), "slot_left_s": round(found["slot_left_s"], 1), "planned_phase": found["phase"]}
    return line(running["slot"], budget, running["start"], now), flight_warning(budget, running["start"], now), record


def table(slot: str, budget: Dict[str, Any], start: datetime, now: datetime) -> str:
    rows = [line(slot, budget, start, now), ""]
    for name, begins, ends in timeline(budget, start):
        mark = "  <- now" if begins <= now < ends else ""
        rows.append(f"  {begins.astimezone():%H:%M:%S}  {name.replace('_', '-'):<10} {clock((ends - begins).total_seconds()):>8}{mark}")
    return "\n".join(rows)


def main(argv: Optional[List[str]] = None, now=lambda: datetime.now(timezone.utc), sleep=time.sleep, ticks: Optional[int] = None) -> int:
    parser = argparse.ArgumentParser(description="The slot clock: where the team is in its slot.")
    parser.add_argument("action", choices=("start", "show", "stop"))
    parser.add_argument("--slot", help="start: the slot's number or name")
    parser.add_argument("--at", help="start: when the slot began (HH:MM, HH:MM:SS or ISO); now when not given")
    parser.add_argument("--watch", action="store_true", help="show: the line again every second, until interrupted")
    parser.add_argument("--state", default=str(STATE), help="the running slot's file")
    parser.add_argument("--budget", default=str(BUDGET))
    args = parser.parse_args(argv)
    state = Path(args.state)
    try:
        budget = load_budget(Path(args.budget))
        if args.action == "start":
            if not args.slot:
                raise ClockError("start needs --slot")
            start = parse_start(args.at, now()) if args.at else now()
            state.parent.mkdir(parents=True, exist_ok=True)
            state.write_text(json.dumps({"slot": args.slot, "start": start.isoformat()}) + "\n", encoding="utf-8")
            print(table(args.slot, budget, start, now()))
            # The start goes to the aircraft as an instant, with its zone: the two computers' clocks agree (PLT-13), their zones need not.
            print(f"\non the aircraft: python3 tools/slot_clock.py start --slot {args.slot} --at {start.astimezone().isoformat(timespec='seconds')}")
            return 0
        if args.action == "stop":
            if state.is_file():
                state.rename(state.with_name(state.name + ".last"))  # kept, not deleted: the last slot's start is a record
            print("no slot is running")
            return 0
        running = read_state(state)
        if running is None:
            print(f"no slot is running: python3 tools/slot_clock.py start --slot <n>  ({state} not found)")
            return 1
        if not args.watch:
            print(table(running["slot"], budget, running["start"], now()))
            return 0
        count = 0
        while ticks is None or count < ticks:
            print(line(running["slot"], budget, running["start"], now()), flush=True)
            count += 1
            sleep(1.0)
        return 0
    except ClockError as problem:
        print(f"slot_clock: {problem}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
