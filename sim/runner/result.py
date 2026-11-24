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
               check or its rate check, or the scenario needs something the
               sim cannot do yet
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
from estimator_check import read_estimator_check, summary_lines as estimator_summary_lines  # noqa: E402
from estimator_check import wanted as estimator_check_wanted  # noqa: E402
from height_check import read_height_check, summary_lines as height_summary_lines  # noqa: E402
from height_check import wanted as height_check_wanted  # noqa: E402
from payload_bridge import read_bridge  # noqa: E402
from inject import read_gps_events, summary_line as gps_line  # noqa: E402
from inject import wanted as gps_events_wanted  # noqa: E402
from sensor_check import read_sensor_check, summary_lines as sensor_lines  # noqa: E402
from sensor_check import wanted as sensor_check_wanted  # noqa: E402
from loc_check import read_loc_check, summary_lines as loc_lines  # noqa: E402
from loc_check import wanted as loc_check_wanted  # noqa: E402
from recovery_cv_check import read_check as read_recovery_cv_check, summary_lines as recovery_cv_lines  # noqa: E402
from recovery_cv_check import wanted as recovery_cv_check_wanted  # noqa: E402
from payload_bridge import wanted as bridge_wanted  # noqa: E402
from projection_check import read_projection_check  # noqa: E402
from wind import read_wind, tilt_problem  # noqa: E402
from wind import wanted as wind_wanted  # noqa: E402
from projection_check import wanted as projection_check_wanted  # noqa: E402
from rate_check import read_rate_check  # noqa: E402
from time_check import read_time_check  # noqa: E402

if __package__ in (None, ""):
    from runner.trace import real_time_factor  # noqa: E402
else:
    from .trace import real_time_factor  # noqa: E402

# On the wall clock a slow sim changes what our nodes do, so such a run is not
# a fair test. A run on sim time (FND-03) has no such limit; it has to pass
# its time check instead, which shows every node really was on the sim clock.
MIN_REAL_TIME_FACTOR = 0.8


# Driver tools whose own report decides the run with the judge: {tool: what it is called in a reason}.
DRIVER_REPORTS = {"failsafe_check": "failsafe check", "watchdog_check": "watchdog check"}


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
    if mission == "hover":
        if details.get("max_drift_m") is None:
            return "never airborne"
        return f"held within {details['max_drift_m']:.2f} m for {details.get('held_s', 0.0):.0f} s"
    if mission == "geofence":
        if not details.get("passed"):
            return "fence not held"
        if details.get("held"):
            return f"held {details['held']['closest_m']:.2f} m inside the fence for {details['held']['held_s']:.0f} s"
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
        "rate_check": None,
        "projection_check": None,
        "estimator_check": None,
        "height_check": None,
        "wind": None,
        "payload_bridge": None,
        "sensor_check": None,
        "gps_events": None,
        "expect_log": None,
        "loc_check": None,
        "recovery_cv_check": None,
        "failsafe_check": None,
        "watchdog_check": None,
        "preflight_check": None,
        "attempt_check": None,
        "forbid_log": None,
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
    commanded_real_release = False  # the mission said it was letting the payload go for real (not a fake drop)
    if trace.is_file():
        try:
            record = Record.load(trace)
        except ValueError as error:
            result["reason"] = f"the trace could not be read: {error}"
            return result
        commanded_real_release = any(item.get("fake") is False for item in record.events_of("release"))
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
        result["rate_check"] = read_rate_check(run_dir)
    time_check, rate_check = result["time_check"], result["rate_check"]
    projection_wanted = projection_check_wanted(manifest)
    if projection_wanted:
        result["projection_check"] = read_projection_check(run_dir)
    projection_check = result["projection_check"]
    estimator_wanted = estimator_check_wanted(manifest)
    if estimator_wanted:
        result["estimator_check"] = read_estimator_check(run_dir)
    estimator_check = result["estimator_check"]
    height_wanted = height_check_wanted(manifest)
    if height_wanted:
        result["height_check"] = read_height_check(run_dir)
    height_check = result["height_check"]

    bridged = bridge_wanted(manifest)
    if bridged:
        result["payload_bridge"] = read_bridge(run_dir)
    bridge = result["payload_bridge"]
    faulted = gps_events_wanted(manifest)
    if faulted:
        result["gps_events"] = read_gps_events(run_dir)
    faults = result["gps_events"]
    sensors_wanted = sensor_check_wanted(manifest)
    if sensors_wanted:
        result["sensor_check"] = read_sensor_check(run_dir)
    sensors = result["sensor_check"]
    min_rtf = (manifest.get("vehicle") or {}).get("min_real_time_factor") if sensors_wanted else None
    windy = wind_wanted(manifest)
    if windy:
        result["wind"] = read_wind(run_dir)
    wind = result["wind"]
    hover_test = ((manifest.get("driver") or {}).get("tool") == "hover") and windy

    outcome = (flight or {}).get("outcome")
    rtf = result["real_time_factor"]
    if flight is None:
        result["reason"] = "the flight did not run (the sim did not start, or the flight script failed)"
    elif outcome == "INVALID":
        result.update(status="INVALID", reason=flight.get("reason", "the scenario could not be flown as written"))
    elif outcome == "NEVER_ARMED":
        result.update(status="NEVER_ARMED", reason="the vehicle did not arm")
    elif outcome == "NOT_STARTED":
        # The GO command (PLT-07) ended without starting a flight. That is the right end of a scenario whose
        # preflight must say NO-GO, and of no other: the vehicle was watched and did not arm.
        expected = (manifest.get("preflight") or {}).get("expect")
        if expected == "NO-GO" and flight.get("launch_exit_code") == 1:
            result.update(status="OK", reason="")
        else:
            result.update(status="FAIL", reason=f"the GO command started no flight (exit code {flight.get('launch_exit_code')}) where the scenario expects {expected}")
    elif score is None:
        result["reason"] = "the flight ran but was not scored"
    elif outcome == "TIMEOUT":
        result.update(status="TIMEOUT", reason=f"the flight did not end within {manifest.get('timeout_s')} s of sim time")
    elif outcome != "DONE":
        result["reason"] = f"the flight script reported '{outcome}'"
    elif bridged and (bridge is None or not bridge.get("servo_messages")):
        result.update(status="INVALID", reason="the scenario has a payload or a bucket on a joint but the run left no record that the payload bridge heard the servo outputs (payload_bridge.json)")
    elif bridged and bridge.get("carried") and commanded_real_release and not bridge.get("released"):
        result.update(status="FAIL", reason=f"the mission commanded a release but the payload bridge never let {bridge['carried']} go: no servo output arrived at a release value")
    elif faulted and faults is None:
        result.update(status="INVALID", reason="the scenario has GPS events but the run left no record that they were injected (gps_events.json): it may have been flown without them")
    elif faulted and not faults["passed"]:
        result.update(status="INVALID", reason="the scenario's GPS events were not all injected: " + "; ".join(faults.get("problems") or ["no reason given"]))
    elif sensors_wanted and sensors is None:
        result.update(status="INVALID", reason="the scenario asks for the sensor check but the run left none (sensor_check.json)")
    elif sensors_wanted and not sensors["passed"]:
        result.update(status="FAIL", reason="the sensor check failed: " + "; ".join(sensors.get("problems") or ["no reason given"]))
    elif min_rtf is not None and (rtf is None or rtf < min_rtf):
        result.update(status="FAIL", reason=f"the real-time factor was {'unknown' if rtf is None else format(rtf, '.2f')} (at least {min_rtf} needed with this camera profile)")
    elif windy and (wind is None or not wind["passed"]):
        result.update(status="INVALID", reason="the scenario has wind but the run left no record that it blew (wind.json): it may have been flown calm")
    elif hover_test and tilt_problem(wind):
        result.update(status="FAIL", reason="the wind missed its pass line: " + tilt_problem(wind))
    elif result["sim_time"] and time_check is None:
        result.update(status="INVALID", reason="the run was on sim time but left no time check (time_check.json), so nothing shows its nodes shared one clock")
    elif result["sim_time"] and not time_check["passed"]:
        problems = time_check.get("problems") or ["no reason given"]
        result.update(status="INVALID", reason="the run was on sim time but failed its time check: " + "; ".join(problems))
    elif result["sim_time"] and rate_check is None:
        result.update(status="INVALID", reason="the run was on sim time but left no rate check (rate_check.json), so nothing shows MAVROS's message rates were held")
    elif result["sim_time"] and not rate_check["passed"]:
        problems = rate_check.get("problems") or ["no reason given"]
        result.update(status="INVALID", reason="the run was on sim time but MAVROS's message rates were not held: " + "; ".join(problems))
    elif projection_wanted and projection_check is None:
        result.update(status="INVALID", reason="the run had use_frame_time_pose on but left no projection check (projection_check.json), so nothing shows where projection v2 put the target")
    elif projection_wanted and not projection_check["passed"]:
        problems = projection_check.get("problems") or ["no reason given"]
        result.update(status="FAIL", reason="projection v2 missed its pass line: " + "; ".join(problems))
    elif estimator_wanted and estimator_check is None:
        result.update(status="INVALID", reason="the run had use_target_estimator on but left no estimator check (estimator_check.json), so nothing shows where the estimator put the target")
    elif estimator_wanted and not estimator_check["passed"]:
        problems = estimator_check.get("problems") or ["no reason given"]
        result.update(status="FAIL", reason="the target estimator missed a pass line: " + "; ".join(problems))
    elif height_wanted and height_check is None:
        result.update(status="INVALID", reason="the run had use_height_estimator on but left no height check (height_check.json), so nothing shows what the height estimator said")
    elif height_wanted and not height_check["passed"]:
        problems = height_check.get("problems") or ["no reason given"]
        result.update(status="FAIL", reason="the height estimator missed its pass line: " + "; ".join(problems))
    elif result["sim_time"]:
        result["status"] = "OK"
    elif rtf is None:
        result.update(status="INVALID", reason="no clock marks in the trace, so the real-time factor is unknown")
    elif rtf < MIN_REAL_TIME_FACTOR:
        result.update(status="INVALID", reason=f"real-time factor {rtf:.2f} is below {MIN_REAL_TIME_FACTOR}: the sim ran too slowly for the result to count")
    else:
        result["status"] = "OK"
    if loc_check_wanted(manifest):
        # Beside a flight with the localisation node on: a run that is otherwise OK counts only with its check passed.
        result["loc_check"] = read_loc_check(run_dir)
        if result["status"] == "OK" and result["loc_check"] is None:
            result.update(status="INVALID", reason="the run had the localisation node on but left no check of its pipeline (loc_check.json)")
        elif result["status"] == "OK" and not result["loc_check"]["passed"]:
            result.update(status="FAIL", reason="the localisation pipeline missed a pass line: " + "; ".join(result["loc_check"].get("problems") or ["no reason given"]))
    if recovery_cv_check_wanted(manifest):
        # Beside a flight with the recovery detector on: a run that is otherwise OK counts only with its check passed.
        result["recovery_cv_check"] = read_recovery_cv_check(run_dir)
        if result["status"] == "OK" and result["recovery_cv_check"] is None:
            result.update(status="INVALID", reason="the run had the recovery detector on but left no check of it (recovery_cv_check.json)")
        elif result["status"] == "OK" and not result["recovery_cv_check"]["passed"]:
            result.update(status="FAIL", reason="the recovery detector missed a pass line: " + "; ".join(result["recovery_cv_check"].get("problems") or ["no reason given"]))
    if manifest.get("preflight"):
        # The preflight was run before the flight (PLT-10): the run counts only if it said what the scenario expects.
        report = _read_json(run_dir / "preflight_check.json")
        result["preflight_check"] = report
        if result["status"] == "OK" and report is None:
            result.update(status="INVALID", reason="the scenario says what the preflight must say but the run left no check of it (preflight_check.json)")
        elif result["status"] == "OK" and not report.get("passed"):
            result.update(status="FAIL", reason="the preflight check failed: " + (report.get("problem") or "no reason given"))
    if manifest.get("mission") == "geofence" and manifest.get("go") and result["status"] == "OK" and not ((score or {}).get("details") or {}).get("passed"):
        # The fence demonstration flown by the mission stack (OPS-02): no tool's exit code says whether the fence
        # answered, so the judge's verdict on the true position is what the run counts by.
        notes = [note for note in (score or {}).get("notes") or [] if note.startswith("geofence:")]
        result.update(status="FAIL", reason="the fence did not answer the demonstration: " + ("; ".join(notes) or "the geofence judge did not pass the run"))
    if manifest.get("attempt"):
        # The attempt's bundle was made after the landing (PLT-12): the run counts only if it is whole.
        report = _read_json(run_dir / "attempt_check.json")
        result["attempt_check"] = report
        if result["status"] == "OK" and report is None:
            result.update(status="INVALID", reason="the scenario makes an attempt's bundle but the run left no check of it (attempt_check.json)")
        elif result["status"] == "OK" and not report.get("passed"):
            result.update(status="FAIL", reason="the attempt's bundle is not whole: " + "; ".join(report.get("problems") or ["no reason given"]))
    tool = (manifest.get("driver") or {}).get("tool")
    if tool in DRIVER_REPORTS:
        # Flown by a check of the flight controller (PLT-08's failsafes, PLT-09's watchdog): the hover judge only says
        # the vehicle stayed near and came down; the check's own verdict says whether what it is for was shown.
        report = _read_json(run_dir / f"{tool}.json")
        result[tool] = report
        if result["status"] == "OK" and report is None:
            result.update(status="INVALID", reason=f"the flight was flown by the {DRIVER_REPORTS[tool]} but left no report ({tool}.json)")
        elif result["status"] == "OK" and not report.get("passed"):
            result.update(status="FAIL", reason=f"the {DRIVER_REPORTS[tool]} failed: " + (report.get("problem") or "no reason given"))
    wanted = [str(line) for line in manifest.get("expect_log") or []]
    if wanted:
        # A flight can pass its judge without doing what the scenario is there to show (a height guard that never
        # acted still passes its waypoints): the mission's own lines are part of the verdict.
        try:
            log = (run_dir / "launch.log").read_text(errors="replace")
        except OSError:
            log = ""
        missing = [line for line in wanted if line not in log]
        result["expect_log"] = {"wanted": wanted, "missing": missing}
        if missing and result["status"] == "OK":
            result.update(status="FAIL", reason="the mission's log has no line with: " + "; ".join(missing))
    unwanted = [str(line) for line in manifest.get("forbid_log") or []]
    if unwanted:
        # The other half: something the flight is there to show did not happen (a latched-out executor asked for no mode).
        try:
            log = (run_dir / "launch.log").read_text(errors="replace")
        except OSError:
            log = ""
        found = [line for line in unwanted if line in log]
        result["forbid_log"] = {"unwanted": unwanted, "found": found}
        if found and result["status"] == "OK":
            result.update(status="FAIL", reason="the mission's log has a line it must not have: " + "; ".join(found))
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
    rows = [f"| Pose stamps against the sim clock | {verdict}: p95 {pose['p95_abs_ms']:.1f} ms, max {pose['max_abs_ms']:.1f} ms over {pose['count']} poses (limit 20 ms) |"]
    rates = result.get("rate_check")
    if rates is None:
        rows.append("| MAVROS message rates (sim time) | no rate check |")
    else:
        measured = ", ".join(f"{topic.split('/')[2]} {row['measured_hz']:.1f} Hz" for topic, row in rates.get("topics", {}).items())
        rows.append(f"| MAVROS message rates (sim time) | {'PASS' if rates['passed'] else 'FAIL'}: {measured} |")
    return rows


def _projection_check_rows(result: dict) -> list[str]:
    """Report rows for a run with use_frame_time_pose on: projection v2's ground point against the truth."""
    check = result.get("projection_check")
    if check is None:
        return []
    at_height = check.get("at_15_m")
    if not at_height:
        return [f"| Projected target against the truth at 15 m | FAIL: {'; '.join(check.get('problems') or ['nothing projected'])} |"]
    row = f"| Projected target against the truth at 15 m | {'PASS' if check['passed'] else 'FAIL'}: median {at_height['median_m']:.2f} m, p95 {at_height['p95_m']:.2f} m over {at_height['count']} detections (limit {check.get('limit_m', 0.3)} m)"
    latest = check.get("at_15_m_with_the_latest_pose")
    if latest:
        row += f"; with the latest pose instead, p95 {latest['p95_m']:.2f} m"
    partly = check.get("at_15_m_target_partly_out_of_view")
    if partly:
        row += f"; not judged: {partly['count']} detections with the target partly out of the picture (up to {partly['max_m']:.2f} m off)"
    return [row + " |"]


def _estimator_check_rows(result: dict) -> list[str]:
    """Report rows for a run with use_target_estimator on: the estimator's bullseye track against the truth, one row per pass line."""
    check = result.get("estimator_check")
    if check is None:
        return []
    return [f"| Estimator's bullseye track against the truth | {line} |" for line in estimator_summary_lines(check)]


def _height_check_rows(result: dict) -> list[str]:
    """Report rows for a run with use_height_estimator on: the estimator's height and its source against the true height."""
    check = result.get("height_check")
    if check is None:
        return []
    return [f"| Height estimator against the true height | {line} |" for line in height_summary_lines(check)]


def _wind_rows(result: dict) -> list[str]:
    """Report rows for a windy run: what the wind was, and the hover's tilt against atan(F / mg) when there was a hover."""
    wind = result.get("wind")
    if wind is None:
        return []
    speed = wind.get("wind_speed_mps")
    if not speed:
        return ["| Wind | none recorded |"]
    settings = wind.get("settings") or {}
    rows = [f"| Wind | mean {speed['mean']:.1f} m/s, {speed['min']:.1f} to {speed['max']:.1f} m/s, from {settings.get('direction_deg', 0.0):g} degrees, over {wind['updates']} updates |"]
    hover = wind.get("hover")
    if hover:
        verdict = "PASS" if hover["within_tolerance"] else "FAIL"
        rows.append(
            f"| Hover tilt in the wind | {verdict}: {hover['mean_tilt_deg']:.2f} degrees against atan(F / mg) = {hover['expected_tilt_deg']:.2f} degrees "
            f"(force {hover['mean_force_n']:.2f} N, mass {hover['mass_kg']:.3f} kg, limit 20%) |"
        )
    return rows


def _loc_rows(result: dict) -> list[str]:
    report = result.get("loc_check")
    return [] if not report else [f"| Localisation pipeline | {'; '.join(loc_lines(report))} |"]


def _recovery_cv_rows(result: dict) -> list[str]:
    report = result.get("recovery_cv_check")
    return [] if not report else [f"| Recovery detector | {'; '.join(recovery_cv_lines(report))} |"]


def _expect_log_rows(result: dict) -> list[str]:
    expected = result.get("expect_log")
    if not expected:
        return []
    wanted, missing = expected["wanted"], expected["missing"]
    text = f"{len(wanted) - len(missing)} of {len(wanted)} found"
    if missing:
        text += "; missing: " + "; ".join(missing)
    return [f"| Lines expected in the mission's log | {text} |"]


def _gps_rows(result: dict) -> list[str]:
    """Report row for a run with GPS events: what was injected and whether the raw fix showed it."""
    report = result.get("gps_events")
    return [] if report is None else [f"| GPS faults | {gps_line(report)} |"]


def _sensor_rows(result: dict) -> list[str]:
    """Report rows for a run with the sensor check: the camera's rate and the rangefinder against the truth."""
    report = result.get("sensor_check")
    if report is None:
        return []
    return [f"| Sensors | {line} |" for line in sensor_lines(report)]


def _payload_rows(result: dict) -> list[str]:
    """Report rows for a run with a carried payload or a bucket: what the payload bridge did."""
    bridge = result.get("payload_bridge")
    if bridge is None:
        return []
    parts = []
    if bridge.get("carried"):
        parts.append(f"{bridge['carried']} {'released' if bridge.get('released') else 'not released'}")
    if bridge.get("bucket"):
        parts.append(f"{bridge['bucket']}: {bridge.get('grabs', 0)} grab(s), {bridge.get('grab_misses', 0)} miss(es){', still held' if bridge.get('holding') else ''}")
    return [f"| Payload bridge | {'; '.join(parts) or 'nothing to do'} ({bridge.get('servo_messages', 0)} servo messages heard) |"]


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
        *_projection_check_rows(result),
        *_estimator_check_rows(result),
        *_height_check_rows(result),
        *_wind_rows(result),
        *_payload_rows(result),
        *_sensor_rows(result),
        *_gps_rows(result),
        *_loc_rows(result),
        *_recovery_cv_rows(result),
        *_expect_log_rows(result),
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
    order = ["OK", "FAIL", "INVALID", "TIMEOUT", "NEVER_ARMED", "ERROR"]
    order += sorted(set(counts) - set(order))  # a status this list has not heard of is still counted
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
