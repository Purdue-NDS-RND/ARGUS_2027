"""The scoring rules, one function per mission (FEATURES.md, SIM-08).

Every judge reads true positions from the run record, never the EKF, and
looks only at the scored flight (the first one). Each returns the points
before the autonomy bonus and a dictionary of the numbers behind them.

Heights are metres above the ground; 1.83 m is the rules' 6 ft, 6.10 m is
20 ft and 7.62 m is 25 ft.
"""

from __future__ import annotations

import csv
import io
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .fence import fence_breaches
from .geometry import point_in_polygon
from .hungarian import assign
from .record import AIRBORNE_HEIGHT_M, VEHICLE, Flight, Pose, Record

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import target_placement  # noqa: E402  (sim/target_placement.py: ArduPilot's flat-earth offsets)

SIX_FEET_M = 1.83
TWENTY_FEET_M = 6.10
TWENTY_FIVE_FEET_M = 7.62

LIST_HEADER = "ID,Latitude,Longitude"
STATE_MESSAGE_PERIOD_S = 1.0  # the autopilot's heartbeat, which carries the flight mode


@dataclass
class Run:
    """What a judge is given: the record, the scored flight, truth and settings."""

    record: Record
    flight: Flight
    truth: dict
    settings: dict
    lists: dict
    notes: list[str] = field(default_factory=list)

    def need(self, key: str) -> object:
        if self.truth.get(key) is None:
            raise ValueError(f"truth has no '{key}', which this judge needs")
        return self.truth[key]

    def vehicle(self) -> list[Pose]:
        return self.record.track(VEHICLE, self.flight.t_arm, self.flight.t_disarm)

    def takeoff_point(self) -> tuple[float, float]:
        takeoff = self.truth.get("takeoff")
        if takeoff is not None:
            return float(takeoff["e"]), float(takeoff["n"])
        start = self.record.pose_at(VEHICLE, self.flight.t_arm)
        return start.e, start.n

    def away_from_takeoff(self) -> list[Pose]:
        """Vehicle poses outside the takeoff zone, where being low is not a takeoff or landing."""
        e0, n0 = self.takeoff_point()
        radius = self.settings["takeoff_zone_radius_m"]
        return [pose for pose in self.vehicle() if math.hypot(pose.e - e0, pose.n - n0) > radius]

    def first_event(self, kind: str) -> dict | None:
        events = self.record.events_of(kind, t0=self.flight.t_arm, t1=self.flight.t_disarm)
        return events[0] if events else None


def _horizontal(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


# --- waypoints ------------------------------------------------------------


def _closest_on_leg(a: Pose, b: Pose, point: tuple[float, ...]) -> tuple[float, float]:
    """Closest distance to `point` on the straight leg a to b, and when."""
    pa = (a.e, a.n, a.u)[: len(point)]
    pb = (b.e, b.n, b.u)[: len(point)]
    leg = [q - p for p, q in zip(pa, pb)]
    length_sq = sum(d * d for d in leg)
    f = 0.0
    if length_sq > 0.0:
        f = sum((w - p) * d for p, d, w in zip(pa, leg, point)) / length_sq
        f = max(0.0, min(1.0, f))
    closest = [p + f * d for p, d in zip(pa, leg)]
    return math.dist(closest, point), a.t + f * (b.t - a.t)


def pass_in_order(poses: list[Pose], waypoints: list[dict], tolerance_m: float, dimensions: int) -> list[dict]:
    """Which waypoints the vehicle passed within the tolerance, in the given order.

    The search for each waypoint starts where the previous one was reached, so
    a waypoint passed early does not count. A missed waypoint does not stop
    the ones after it from counting.
    """

    if dimensions not in (2, 3):
        raise ValueError("dimensions must be 2 or 3")
    results = []
    start = 0  # index of the leg the next search begins on
    start_time = poses[0].t if poses else 0.0
    for wp in waypoints:
        point = (float(wp["e"]), float(wp["n"]), float(wp.get("u", 0.0)))[:dimensions]
        hit_time = None
        closest = math.inf
        for i in range(start, max(len(poses) - 1, 0)):
            a, b = poses[i], poses[i + 1]
            if b.t < start_time:
                continue
            distance, when = _closest_on_leg(a, b, point)
            closest = min(closest, distance)
            if distance <= tolerance_m and hit_time is None:
                hit_time = max(when, start_time)
                start, start_time = i, hit_time
                break
        results.append({"name": wp.get("name", f"wp{len(results) + 1}"), "hit": hit_time is not None, "t": hit_time, "closest_m": closest})
    # Closest approach: from where the search began until the next waypoint was reached.
    begin = poses[0].t if poses else 0.0
    for k, result in enumerate(results):
        later = [r["t"] for r in results[k + 1 :] if r["hit"]]
        end = later[0] if later else (poses[-1].t if poses else 0.0)
        point = (float(waypoints[k]["e"]), float(waypoints[k]["n"]), float(waypoints[k].get("u", 0.0)))[:dimensions]
        closest = math.inf
        for a, b in zip(poses, poses[1:]):
            if b.t < begin or a.t > end:
                continue
            closest = min(closest, _closest_on_leg(a, b, point)[0])
        result["closest_m"] = None if math.isinf(closest) else closest
        if result["hit"]:
            begin = result["t"]
    return results


def judge_waypoint_navigation(run: Run) -> tuple[float, dict]:
    s = run.settings
    waypoints = run.need("waypoints")
    results = pass_in_order(run.vehicle(), waypoints, s["tolerance_m"], s["dimensions"])
    hit = sum(1 for result in results if result["hit"])
    return hit * s["points_per_waypoint"], {"hit": hit, "of": len(results), "waypoints": results}


def judge_time_trial(run: Run) -> tuple[float, dict]:
    s = run.settings
    waypoints = run.need("waypoints")
    results = pass_in_order(run.vehicle(), waypoints, s["tolerance_m"], s["dimensions"])
    hit = sum(1 for result in results if result["hit"])
    details = {"hit": hit, "of": len(results), "waypoints": results, "elapsed_s": None, "rank": None, "of_teams": None}
    if not results or hit < len(results):
        run.notes.append("time trial: not every waypoint was passed in order, so there is no time")
        return 0.0, details

    elapsed = results[-1]["t"] - results[0]["t"]
    references = sorted(float(value) for value in s["reference_times_s"])
    details["elapsed_s"] = elapsed
    if not references:
        run.notes.append("time trial: no reference times given, scored as the slowest team")
        return float(s["points_slowest"]), details

    teams = len(references) + 1
    rank = 1 + sum(1 for value in references if value < elapsed)
    details["rank"], details["of_teams"] = rank, teams
    span = s["points_fastest"] - s["points_slowest"]
    return s["points_fastest"] - span * (rank - 1) / (teams - 1), details


# --- localisation ---------------------------------------------------------


def _marker_id(value) -> str:
    text = str(value).strip()
    try:
        return str(int(text))
    except ValueError:
        return text


def parse_list(text: str, origin: dict) -> list[tuple[str, float, float]]:
    """The judges' list (ID,Latitude,Longitude) as (id, east, north) in the world frame."""

    rows = [row for row in csv.reader(io.StringIO(text)) if any(cell.strip() for cell in row)]
    header = ",".join(cell.strip() for cell in rows[0]) if rows else ""
    if header.lower() != LIST_HEADER.lower():
        raise ValueError(f"the judges' list must start with the header {LIST_HEADER}, got {header!r}")
    entries = []
    for number, row in enumerate(rows[1:], start=2):
        try:
            north, east = target_placement.offset_ne_m(
                float(origin["latitude_deg"]), float(origin["longitude_deg"]), float(row[1]), float(row[2])
            )
        except (IndexError, ValueError) as error:
            raise ValueError(f"the judges' list, line {number}: {error}") from error
        entries.append((_marker_id(row[0]), east, north))
    return entries


def score_list(entries: list[tuple[str, float, float]], markers: list[dict], s: dict) -> dict:
    """Match the list to the true markers and add up the points.

    The Hungarian method picks the pairing that loses the fewest points, with
    distance as the tie-break. A row that pairs with no marker (more rows than
    markers) is wrong on both counts.
    """

    result = {"listed": len(entries), "markers": len(markers), "correct": 0, "wrong_id": 0, "wrong_location": 0, "both_wrong": 0, "missed": 0, "errors_m": [], "points": 0.0}
    paired_rows: set[int] = set()
    paired_markers: set[int] = set()
    if entries and markers:
        distance = np.array([[_horizontal(entry[1:], (m["e"], m["n"])) for m in markers] for entry in entries])
        id_wrong = np.array([[entry[0] != _marker_id(m["id"]) for m in markers] for entry in entries])
        far = distance > s["tolerance_m"]
        loss = np.where(
            id_wrong & far,
            s["both_wrong_penalty"],
            np.where(id_wrong, s["wrong_id_penalty"], np.where(far, s["wrong_location_penalty"], -s["points_per_marker"])),
        )
        for row, col in assign(loss + 1.0e-6 * np.minimum(distance, 1000.0)):
            if id_wrong[row, col] and far[row, col]:
                continue  # not a match at all: the row is spurious and the marker missed
            paired_rows.add(row)
            paired_markers.add(col)
            result["errors_m"].append(float(distance[row, col]))
            if id_wrong[row, col]:
                result["wrong_id"] += 1
            elif far[row, col]:
                result["wrong_location"] += 1
            else:
                result["correct"] += 1
    result["both_wrong"] = len(entries) - len(paired_rows)
    result["missed"] = len(markers) - len(paired_markers)
    result["points"] = (
        result["correct"] * s["points_per_marker"]
        - result["wrong_id"] * s["wrong_id_penalty"]
        - result["wrong_location"] * s["wrong_location_penalty"]
        - result["both_wrong"] * s["both_wrong_penalty"]
    )
    return result


def judge_localisation(run: Run) -> tuple[float, dict]:
    s = run.settings
    markers = run.need("markers")
    origin = run.need("origin")

    survey_area = run.truth.get("survey_area")
    if survey_area:
        surveying = [pose for pose in run.vehicle() if point_in_polygon(pose.e, pose.n, survey_area)]
    else:
        surveying = run.away_from_takeoff()
    min_height = min((pose.u for pose in surveying), default=None)
    if min_height is None:
        run.notes.append("localisation: the vehicle never entered the survey area, so no height was judged")

    airborne = [pose.t for pose in run.vehicle() if pose.u > AIRBORNE_HEIGHT_M]
    t_landing = airborne[-1] if airborne else run.flight.t_disarm
    written = run.record.events_of("list_written", t1=t_landing)

    text = run.lists.get("list")
    details = score_list(parse_list(text, origin), markers, s) if text is not None else score_list([], markers, s)
    merged_text = run.lists.get("merged_list")
    details["merged"] = score_list(parse_list(merged_text, origin), markers, s) if merged_text is not None else None
    details.update({"min_height_m": min_height, "height_penalty": 0.0, "list_before_landing": bool(written), "t_landing": t_landing})

    points = details["points"]
    if text is None:
        run.notes.append("localisation: no judges' list, scored 0")
        points = 0.0
    elif not written:
        run.notes.append("localisation: the list was not written before landing, scored 0")
        points = 0.0
    if min_height is not None and min_height < s["height_zero_below_m"]:
        run.notes.append(f"localisation: flew at {min_height:.2f} m, below {s['height_zero_below_m']} m, scored 0")
        points = 0.0
    elif min_height is not None and min_height < s["height_penalty_below_m"]:
        details["height_penalty"] = float(s["height_penalty_points"])
        points -= s["height_penalty_points"]
    return max(0.0, points), details


# --- drop and delivery ----------------------------------------------------


def _resting_place(run: Run, release: dict, key: str) -> tuple[tuple[float, float], str]:
    """Where the payload ended up: its own true pose, or the vehicle's at a fake release."""
    name = (run.truth.get(key) or {}).get("name")
    rest = run.record.final(name) if name else None
    if rest is not None and not release.get("fake"):
        return (rest.e, rest.n), f"{key} at rest"
    if not release.get("fake") and name:
        run.notes.append(f"no true poses recorded for '{name}': scored from the vehicle position at release")
    vehicle = run.record.pose_at(VEHICLE, release["t"])
    return (vehicle.e, vehicle.n), "vehicle at release"


def _hold_details(run: Run, target: dict) -> dict:
    """How well the vehicle held over the target before the release, when the scenario sets `hold_height_m` (gate G1). {} otherwise.

    The hold is the last unbroken stretch, ending at the release, in which
    the vehicle was within `hold_band_m` of the hold height; its first
    `hold_settle_s` are left out. The error is the vehicle's true horizontal
    distance from the target. A hold shorter than `hold_min_s`, or a flight
    with no release to end it, has no number: nothing is reported as held
    that was not.

    With `hold_point_body_m` ([forward, left] of the vehicle's centre, the
    payload's release point) the error is that point's, placed with the
    vehicle's true heading, and the vehicle's own is reported beside it.
    """
    s = run.settings
    height = s["hold_height_m"]
    if height is None:
        return {}
    details = {"hold_error_p95_m": None, "hold_error_median_m": None, "hold_error_max_m": None, "hold_s": 0.0, "hold_height_mean_m": None, "hold_samples": 0}
    release = run.first_event("release")
    if release is None:
        run.notes.append(f"hold: nothing was released, so the hold at {height:g} m has no end and is not measured")
        return details
    held: list[Pose] = []
    for pose in reversed([pose for pose in run.vehicle() if pose.t <= release["t"]]):
        if abs(pose.u - height) > s["hold_band_m"]:
            break
        held.append(pose)
    held.reverse()
    if not held:
        run.notes.append(f"hold: the vehicle was not within {s['hold_band_m']:g} m of {height:g} m at the release")
        return details
    held = [pose for pose in held if pose.t >= held[0].t + s["hold_settle_s"]]
    hold_s = held[-1].t - held[0].t if len(held) >= 2 else 0.0
    details.update(hold_s=hold_s, hold_samples=len(held))
    if hold_s < s["hold_min_s"]:
        run.notes.append(f"hold: held at {height:g} m for {hold_s:.1f} s after settling (at least {s['hold_min_s']:g} s needed), not measured")
        return details
    vehicle_errors = sorted(math.hypot(pose.e - target["e"], pose.n - target["n"]) for pose in held)
    errors = vehicle_errors
    point = s["hold_point_body_m"]
    if point is not None:
        forward, left = float(point[0]), float(point[1])
        if any(pose.yaw is None for pose in held):
            run.notes.append("hold: the trace has no yaw for the vehicle, so where the payload's point was cannot be said; not measured")
            return details
        errors = sorted(
            math.hypot(
                pose.e + forward * math.cos(pose.yaw) - left * math.sin(pose.yaw) - target["e"],
                pose.n + forward * math.sin(pose.yaw) + left * math.cos(pose.yaw) - target["n"],
            )
            for pose in held
        )
        details["hold_vehicle_error_p95_m"] = vehicle_errors[min(len(vehicle_errors) - 1, math.ceil(0.95 * len(vehicle_errors)) - 1)]
    details.update(
        hold_error_p95_m=errors[min(len(errors) - 1, math.ceil(0.95 * len(errors)) - 1)],
        hold_error_median_m=errors[len(errors) // 2],
        hold_error_max_m=errors[-1],
        hold_height_mean_m=sum(pose.u for pose in held) / len(held),
    )
    return details


def judge_drop(run: Run) -> tuple[float, dict]:
    s = run.settings
    target = run.need("target")
    low = run.away_from_takeoff()
    min_height = min((pose.u for pose in low), default=None)
    flew_low = min_height is not None and min_height < s["min_height_m"]
    details = {"released": False, "distance_m": None, "release_height_m": None, "released_low": False, "flew_low": flew_low, "min_height_m": min_height, "scored_from": None}
    details.update(_hold_details(run, target))

    release = run.first_event("release")
    if release is None:
        run.notes.append("drop: nothing was released, scored 0")
        return 0.0, details

    vehicle = run.record.pose_at(VEHICLE, release["t"])
    place, scored_from = _resting_place(run, release, "payload")
    distance = _horizontal(place, (target["e"], target["n"]))
    released_low = vehicle.u < s["min_height_m"]
    details.update({"released": True, "distance_m": distance, "release_height_m": vehicle.u, "released_low": released_low, "scored_from": scored_from})

    points = s["max_points"] - distance * s["points_per_metre"]
    if released_low:
        points -= s["low_release_penalty"]
    if flew_low:
        points -= s["low_flight_penalty"]
    return max(0.0, points), details


def judge_delivery(run: Run) -> tuple[float, dict]:
    s = run.settings
    target = run.need("target")
    payload = run.truth.get("payload") or {}
    contacts = [float(item["impulse_ns"]) for item in run.record.events_of("contact", t0=run.flight.t_arm) if "impulse_ns" in item]
    max_impulse = max(contacts, default=None)
    limit = s["impulse_limit_ns"]
    impulse_ok = None if limit is None or max_impulse is None else max_impulse <= limit
    details = {"released": False, "distance_m": None, "release_height_m": None, "released_high": False, "weight_bonus": 0.0, "max_impulse_ns": max_impulse, "impulse_limit_ns": limit, "impulse_ok": impulse_ok, "scored_from": None}

    release = run.first_event("release")
    if release is None:
        run.notes.append("delivery: nothing was released, scored 0")
        return 0.0, details

    place, scored_from = _resting_place(run, release, "payload")
    name = payload.get("name")
    if scored_from == "payload at rest":
        # How far the package fell: its height at release above where it came to rest.
        height = run.record.pose_at(name, release["t"]).u - run.record.final(name).u
    else:
        height = run.record.pose_at(VEHICLE, release["t"]).u
    distance = _horizontal(place, (target["e"], target["n"]))
    released_high = height > s["max_release_height_m"]
    details.update({"released": True, "distance_m": distance, "release_height_m": height, "released_high": released_high, "scored_from": scored_from})
    if released_high:
        run.notes.append(f"delivery: released at {height:.2f} m, above {s['max_release_height_m']} m, scored 0")
        return 0.0, details

    points = max(0.0, s["max_points"] - distance * s["points_per_metre"])
    if points > 0.0 and payload.get("mass_kg") is not None:
        details["weight_bonus"] = s["weight_bonus_per_500g"] * math.floor(float(payload["mass_kg"]) * 1000.0 / 500.0 + 1.0e-9)
    return points + details["weight_bonus"], details


# --- recovery -------------------------------------------------------------


def judge_recovery(run: Run) -> tuple[float, dict]:
    s = run.settings
    bucket = run.need("bucket")
    return_points = run.need("return_points")
    if s["return_point"] not in return_points:
        raise ValueError(f"truth has no return point '{s['return_point']}' (it has {sorted(return_points)})")
    goal = return_points[s["return_point"]]
    name = bucket["name"]
    start, rest = run.record.first(name), run.record.final(name)
    if start is None:
        raise ValueError(f"the trace has no true poses for the bucket '{name}'")

    lifts = [pose.u - start.u for pose in run.record.track(name, run.flight.t_arm, run.flight.t_disarm)]
    max_lift = max(lifts, default=0.0)
    lifted = max_lift > s["lift_height_m"]
    distance = _horizontal((rest.e, rest.n), (goal["e"], goal["n"]))
    release = run.first_event("release")
    release_height = None if release is None else run.record.pose_at(name, release["t"]).u - start.u
    released_high = release_height is not None and release_height > s["max_release_height_m"]
    details = {"lifted": lifted, "max_lift_m": max_lift, "released": release is not None, "release_height_m": release_height, "released_high": released_high, "return_point": s["return_point"], "distance_m": distance}

    # How the flight ended, bucket or no bucket: down and disarmed, having come down no faster than a landing does.
    # (A failed recovery lands where it is, beside the bucket, by REC-7's abort mode; gate G5 asks this of every flight.)
    disarmed = run.flight.t_disarm
    last = [] if disarmed is None else run.record.track(VEHICLE, disarmed - 5.0, disarmed)
    falls = [(a.u - b.u) / (b.t - a.t) for a, b in zip(last, last[1:]) if b.t > a.t]
    details["touchdown_mps"] = max(falls, default=None) if falls else None
    details["ended_safely"] = bool(falls) and max(falls) <= s["max_touchdown_mps"] and last[-1].u - run.record.pose_at(VEHICLE, run.flight.t_arm).u < s["landed_below_m"]

    if not lifted:
        run.notes.append(f"recovery: the bucket never rose above {s['lift_height_m']} m, scored 0")
        return 0.0, details
    if released_high:
        run.notes.append(f"recovery: the bucket was released at {release_height:.2f} m, above {s['max_release_height_m']} m, scored 0")
        return 0.0, details
    return max(0.0, s["max_points"] - distance * s["points_per_metre"]), details


# --- geofence ---------------------------------------------------------------


def judge_geofence(run: Run) -> tuple[float, dict]:
    """The fence must stop a vehicle that flies at it: 1 point if it does.

    The autopilot's response is its report of the breach (a `fence_breach`
    event) followed by a mode it flies home or stops in. The response must
    come within `max_response_s` of the true position crossing the fence, and
    the true position must stay within `max_excursion_m` of it. A vehicle
    stopped before the line passes with no excursion.
    """

    s = run.settings
    fence = run.need("fence")
    t0, t1 = run.flight.t_arm, run.flight.t_disarm
    spans = fence_breaches(run.record, fence, t0, t1)
    t_cross = spans[0]["t_start"] if spans else None
    max_excursion = max((span["max_excursion_m"] for span in spans), default=0.0)
    reports = run.record.events_of("fence_breach", t0=t0, t1=t1)
    t_reported = reports[0]["t"] if reports else None
    # The mode comes from a 1 Hz state message, so it can read up to a second
    # late; the report is prompt. A response is a report with such a mode after it.
    after = t_reported if t_reported is not None else (t_cross if t_cross is not None else t0)
    # The mode itself must still come in time: within the limit, plus the one
    # second its message can lag, of the crossing (or of the report, if the
    # vehicle was stopped before the line).
    latest = (t_cross if t_cross is not None else after) + s["max_response_s"] + STATE_MESSAGE_PERIOD_S
    modes = [item for item in run.record.events_of("mode", t0=after - 1.5, t1=min(t1, latest)) if item.get("mode") in s["response_modes"]]
    t_mode = modes[0]["t"] if modes else None
    t_response = None
    if t_mode is not None:
        t_response = t_mode if t_reported is None else min(t_mode, t_reported)
    delay = None if t_response is None or t_cross is None else t_response - t_cross

    problem = ""
    if t_cross is None and t_reported is None:
        problem = "the vehicle never reached the fence"
    elif t_response is None:
        problem = "the autopilot did not leave the commanded flight after the breach"
    elif delay is not None and delay > s["max_response_s"]:
        problem = f"the response came {delay:.2f} s after the fence line (limit {s['max_response_s']} s)"
    elif max_excursion > s["max_excursion_m"]:
        problem = f"the vehicle went {max_excursion:.2f} m outside the fence (limit {s['max_excursion_m']} m)"
    if problem:
        run.notes.append(f"geofence: {problem}")
    details = {
        "passed": not problem,
        "t_cross": t_cross,
        "t_reported": t_reported,
        "t_mode": t_mode,
        "response_mode": modes[0]["mode"] if modes else None,
        "response_delay_s": delay,  # negative: the autopilot responded before the true position crossed the line
        "max_excursion_m": max_excursion,
    }
    return (0.0 if problem else 1.0), details


# --- hover ------------------------------------------------------------------


def judge_hover(run: Run) -> tuple[float, dict]:
    """A hover must hold its place: 1 point if the vehicle stayed within `max_drift_m` of its takeoff point while it was up.

    Not a competition mission: the flight SIM-03's wind is checked on. The
    tilt against atan(F / mg) is judged from the wind tool's own record
    (wind.json), which has the force; this judge only says the hover was one.
    """
    s = run.settings
    e0, n0 = run.takeoff_point()
    start = run.record.pose_at(VEHICLE, run.flight.t_arm)
    up = [pose for pose in run.vehicle() if pose.u - start.u >= s["min_height_m"]]
    drift = max((math.hypot(pose.e - e0, pose.n - n0) for pose in up), default=None)
    held_s = (up[-1].t - up[0].t) if len(up) >= 2 else 0.0
    problem = ""
    if drift is None:
        problem = f"the vehicle never rose {s['min_height_m']} m"
    elif held_s < s["min_hold_s"]:
        problem = f"the vehicle was up for {held_s:.1f} s (at least {s['min_hold_s']} s needed)"
    elif drift > s["max_drift_m"]:
        problem = f"the vehicle drifted {drift:.2f} m from its takeoff point (limit {s['max_drift_m']} m)"
    if problem:
        run.notes.append(f"hover: {problem}")
    return (0.0 if problem else 1.0), {"passed": not problem, "max_drift_m": drift, "held_s": held_s}
