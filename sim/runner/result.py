#!/usr/bin/env python3
"""What a run came to: its status, its report, and the suite table. No ROS.

  python3 sim/runner/result.py <run folder>                   writes result.json and report.md
  python3 sim/runner/result.py --suite <out folder> <run folder>...   writes summary.md

Statuses:
  OK           flown to a disarm on the ground and scored
  TIMEOUT      the flight did not end within the scenario's timeout (sim time); scored as far as it got
  NEVER_ARMED  the vehicle did not arm
  INVALID      flown, but the result does not count: the sim ran below 0.8 of
               real time on wall-clock nodes, a run on sim time failed its time
               check, or the scenario needs something the sim cannot do yet
  ERROR        the sim or the flight script failed
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))

from judge.record import Record  # noqa: E402
from time_check import read_time_check  # noqa: E402

if __package__ in (None, ""):
    from runner.trace import real_time_factor  # noqa: E402
else:
    from .trace import real_time_factor  # noqa: E402

# On the wall clock a slow sim changes what our nodes do, so such a run is not
# a fair test. A run on sim time (FND-03) has no such limit; it has to pass
# its time check instead, which shows every node really was on the sim clock.
MIN_REAL_TIME_FACTOR = 0.8


def _read_json(path: Path):
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return None


def headline(score: dict) -> str:
    """The one number a person looks for first."""
    mission, details = score.get("mission"), score.get("details") or {}
    if mission in ("drop", "delivery"):
        if details.get("distance_m") is None:
            return "nothing released"
        return f"{details['distance_m']:.2f} m from the target"
    if mission == "recovery":
        if not details.get("lifted"):
            return "bucket not lifted"
        return f"{details['distance_m']:.2f} m from the return point"
    if mission == "waypoint_navigation":
        return f"{details.get('hit', 0)} of {details.get('of', 0)} waypoints"
    if mission == "time_trial":
        elapsed = details.get("elapsed_s")
        passed = f"{details.get('hit', 0)} of {details.get('of', 0)} waypoints"
        return passed if elapsed is None else f"{elapsed:.1f} s, {passed}"
    if mission == "geofence":
        if not details.get("passed"):
            return "fence not held"
        delay = details.get("response_delay_s")
        when = "stopped before the line" if delay is None else f"response {delay:+.2f} s from the line"
        return f"{when}, {details.get('max_excursion_m', 0.0):.2f} m outside"
    if mission == "localisation":
        return f"{details.get('correct', 0)} of {details.get('markers', 0)} markers correct"
    return ""


def run_result(run_dir: Path) -> dict:
    run_dir = Path(run_dir)
    manifest = _read_json(run_dir / "scenario.json") or {}
    flight = _read_json(run_dir / "flight.json")
    score = _read_json(run_dir / "score.json")
    result = {
        "scenario": f"{manifest.get('mission', '?')}/{manifest.get('name', run_dir.name)}",
        "seed": manifest.get("seed"),
        "variant": manifest.get("variant", "off"),
        "flags": manifest.get("flags") or {},
        "run_dir": str(run_dir),
        "status": "ERROR",
        "reason": "",
        "points": None,
        "base_points": None,
        "headline": "",
        "real_time_factor": None,
        "sim_time": manifest.get("sim_time") is True,
        "time_check": None,
        "flight_s": None,
        "notes": [],
        "fence_breaches": [],
        "failsafes": [],
        "human_inputs": [],
    }

    if score is not None:
        common = score.get("common") or {}
        result.update(
            points=score.get("points"),
            base_points=score.get("base_points"),
            headline=headline(score),
            notes=list(score.get("notes") or []),
            fence_breaches=common.get("fence_breaches") or [],
            failsafes=common.get("failsafes") or [],
            human_inputs=common.get("human_inputs") or [],
        )

    trace = run_dir / "trace.jsonl"
    if trace.is_file():
        try:
            record = Record.load(trace)
        except ValueError as error:
            result["reason"] = f"the trace could not be read: {error}"
            return result
        armed = record.events_of("armed")
        if armed:
            t_arm = armed[0]["t"]
            disarmed = record.events_of("disarmed", t0=t_arm)
            t_end = disarmed[0]["t"] if disarmed else max((item["t"] for item in record.events), default=t_arm)
            if disarmed:
                result["flight_s"] = t_end - t_arm
            result["real_time_factor"] = real_time_factor(record, t_arm, t_end)

    if result["sim_time"]:
        result["time_check"] = read_time_check(run_dir)
    time_check = result["time_check"]

    outcome = (flight or {}).get("outcome")
    rtf = result["real_time_factor"]
    if flight is None:
        result["reason"] = "the flight did not run (the sim did not start, or the flight script failed)"
    elif outcome == "INVALID":
        result.update(status="INVALID", reason=flight.get("reason", "the scenario could not be flown as written"))
    elif outcome == "NEVER_ARMED":
        result.update(status="NEVER_ARMED", reason="the vehicle did not arm")
    elif score is None:
        result["reason"] = "the flight ran but was not scored"
    elif outcome == "TIMEOUT":
        result.update(status="TIMEOUT", reason=f"the flight did not end within {manifest.get('timeout_s')} s of sim time")
    elif outcome != "DONE":
        result["reason"] = f"the flight script reported '{outcome}'"
    elif result["sim_time"] and time_check is None:
        result.update(status="INVALID", reason="the run was on sim time but left no time check (time_check.json), so nothing shows its nodes shared one clock")
    elif result["sim_time"] and not time_check["passed"]:
        problems = time_check.get("problems") or ["no reason given"]
        result.update(status="INVALID", reason="the run was on sim time but failed its time check: " + "; ".join(problems))
    elif result["sim_time"]:
        result["status"] = "OK"
    elif rtf is None:
        result.update(status="INVALID", reason="no clock marks in the trace, so the real-time factor is unknown")
    elif rtf < MIN_REAL_TIME_FACTOR:
        result.update(status="INVALID", reason=f"real-time factor {rtf:.2f} is below {MIN_REAL_TIME_FACTOR}: the sim ran too slowly for the result to count")
    else:
        result["status"] = "OK"
    return result


def _number(value, pattern: str = "{:.2f}") -> str:
    return "n/a" if value is None else pattern.format(value)


def _time_check_rows(result: dict) -> list[str]:
    """Report rows for a run on sim time: how well pose stamps agreed with /clock."""
    if not result.get("sim_time"):
        return []
    pose = (result.get("time_check") or {}).get("pose") or {}
    if not pose.get("count"):
        return ["| Pose stamps against the sim clock | no time check |"]
    verdict = "PASS" if result["time_check"]["passed"] else "FAIL"
    return [f"| Pose stamps against the sim clock | {verdict}: p95 {pose['p95_abs_ms']:.1f} ms, max {pose['max_abs_ms']:.1f} ms over {pose['count']} poses (limit 20 ms) |"]


def render_report(result: dict) -> str:
    flags = "flags on" if result["variant"] == "on" else "flags off"
    if result["flags"]:
        flags += " (" + ", ".join(f"{name}={value}" for name, value in result["flags"].items()) + ")"
    lines = [
        f"# {result['scenario']}, seed {result['seed']}, {flags}",
        "",
        f"**{result['status']}**" + (f": {result['reason']}" if result["reason"] else ""),
        "",
        "| | |",
        "| --- | --- |",
        f"| Points | {_number(result['points'])} ({_number(result['base_points'])} before the autonomy bonus) |",
        f"| Result | {result['headline'] or 'n/a'} |",
        f"| Flight time, arm to disarm (sim time) | {_number(result['flight_s'], '{:.0f} s')} |",
        f"| Real-time factor while armed | {_number(result['real_time_factor'])} |",
        *_time_check_rows(result),
        f"| Fence breaches (true position) | {len(result['fence_breaches'])} |",
        f"| Failsafes | {len(result['failsafes'])} |",
        f"| Human input during the flight | {len(result['human_inputs'])} |",
    ]
    details = []
    for breach in result["fence_breaches"]:
        details.append(f"- Fence breach ({breach['kind']}): {breach['max_excursion_m']:.1f} m outside, sim time {breach['t_start']:.1f} to {breach['t_end']:.1f} s")
    for item in result["failsafes"]:
        details.append(f"- Failsafe at {item['t']:.1f} s: {item['what']}")
    for item in result["human_inputs"]:
        details.append(f"- Human input at {item['t']:.1f} s: {item['what']}")
    details += [f"- Note: {note}" for note in result["notes"]]
    if details:
        lines += ["", *details]
    lines += ["", f"Run folder: `{result['run_dir']}` (`score.json`, `trace.jsonl`, `launch.log`)."]
    return "\n".join(lines) + "\n"


def suite_table(results: list[dict]) -> str:
    """One row per scenario and seed: flags off (2026) beside flags on (2027)."""
    rows: dict[tuple, dict] = {}
    for result in results:
        rows.setdefault((result["scenario"], result["seed"]), {})[result["variant"]] = result
    lines = [
        "| Scenario | Seed | 2026 (flags off) | Points | Result | 2027 (flags on) | Points | Result |",
        "| --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for (scenario, seed), variants in sorted(rows.items(), key=lambda item: (item[0][0], item[0][1] or 0)):
        off, on = variants.get("off"), variants.get("on")
        cells = []
        for result in (off, on):
            if result is None:
                cells += ["not run", "", ""]
            else:
                cells += [result["status"], _number(result["points"]), result["headline"]]
        lines.append(f"| {scenario} | {seed} | " + " | ".join(cells) + " |")
    counts: dict[str, int] = {}
    for result in results:
        counts[result["status"]] = counts.get(result["status"], 0) + 1
    order = ["OK", "INVALID", "TIMEOUT", "NEVER_ARMED", "ERROR"]
    summary = ", ".join(f"{counts[status]} {status}" for status in order if status in counts)
    lines += ["", f"{len(results)} runs: {summary}"]
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Summarise a run folder, or a suite of them.")
    parser.add_argument("--suite", type=Path, help="write summary.md for these runs into this folder")
    parser.add_argument("--title", default="Suite", help="heading of summary.md")
    parser.add_argument("runs", nargs="+", type=Path)
    args = parser.parse_args(argv)
    results = []
    for run_dir in args.runs:
        result = run_result(run_dir)
        results.append(result)
        if run_dir.is_dir():
            (run_dir / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            (run_dir / "report.md").write_text(render_report(result))
        print(f"{result['status']:<11} {result['scenario']} seed {result['seed']} {result['variant']}: "
              f"{_number(result['points'])} points, {result['headline'] or result['reason']}")
    if args.suite:
        args.suite.mkdir(parents=True, exist_ok=True)
        text = f"# {args.title}\n\n{suite_table(results)}"
        (args.suite / "summary.md").write_text(text)
        (args.suite / "results.json").write_text(json.dumps(results, indent=2) + "\n")
        print(f"\n{text}")
    return 0 if all(result["status"] == "OK" for result in results) else 1


if __name__ == "__main__":
    sys.exit(main())
