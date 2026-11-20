"""sim/judge: the judges that score a sim run the way the organisers will.

Each test builds a small run record (true poses and events on sim time) and
checks one line of the scoring table in docs/build/FEATURES.md, SIM-08. A
wrong judge would make every later sim gate lie, so the numbers here are
worked out by hand. No ROS needed.
"""

import itertools
import math
import random
import sys
from pathlib import Path

import numpy as np
import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import target_placement as tp  # noqa: E402
from judge import judge_run  # noqa: E402
from judge.geometry import distance_outside_polygon, point_in_polygon  # noqa: E402
from judge.hungarian import assign  # noqa: E402
from judge.record import Record  # noqa: E402

ORIGIN = {"latitude_deg": 34.1760, "longitude_deg": -118.4810, "elevation_m": 216.0}


# --- building run records -------------------------------------------------


def path(name, knots, dt=0.1):
    """Pose lines for `name` along straight legs between (t, e, n, u) knots."""
    lines = []
    for (t0, *p0), (t1, *p1) in zip(knots, knots[1:]):
        steps = max(1, int(round((t1 - t0) / dt)))
        for i in range(steps):
            f = i / steps
            e, n, u = (a + (b - a) * f for a, b in zip(p0, p1))
            lines.append({"t": t0 + (t1 - t0) * f, "pose": name, "e": e, "n": n, "u": u})
    t, e, n, u = knots[-1]
    lines.append({"t": t, "pose": name, "e": e, "n": n, "u": u})
    return lines


def event(t, kind, **fields):
    return {"t": t, "event": kind, **fields}


def record(*parts, **kwargs):
    lines = []
    for part in parts:
        lines.extend(part if isinstance(part, list) else [part])
    return Record.from_lines(lines, **kwargs)


def flight(knots, t_arm=None, t_disarm=None):
    """A vehicle path with the arm and disarm events around it."""
    t_arm = knots[0][0] if t_arm is None else t_arm
    t_disarm = knots[-1][0] if t_disarm is None else t_disarm
    return path("vehicle", knots) + [event(t_arm, "armed"), event(t_disarm, "disarmed")]


def truth(**extra):
    base = {"origin": ORIGIN, "takeoff": {"e": 0.0, "n": 0.0}}
    base.update(extra)
    return base


def latlon(e, n):
    """A latitude and longitude that target_placement maps back to (e, n)."""
    lat = ORIGIN["latitude_deg"] + n / (tp.LOCATION_SCALING_FACTOR * 1e7)
    lon = ORIGIN["longitude_deg"]
    for _ in range(5):
        _, east = tp.offset_ne_m(ORIGIN["latitude_deg"], ORIGIN["longitude_deg"], lat, lon)
        scale = tp.LOCATION_SCALING_FACTOR * 1e7 * math.cos(math.radians(lat))
        lon += (e - east) / scale
    return lat, lon


def csv_text(rows):
    lines = ["ID,Latitude,Longitude"]
    for marker_id, e, n in rows:
        lat, lon = latlon(e, n)
        lines.append(f"{marker_id},{lat:.7f},{lon:.7f}")
    return "\n".join(lines) + "\n"


# --- Hungarian method -----------------------------------------------------


def brute_force(cost):
    rows, cols = cost.shape
    small, large = min(rows, cols), max(rows, cols)
    best = math.inf
    for perm in itertools.permutations(range(large), small):
        if rows <= cols:
            total = sum(cost[i, j] for i, j in enumerate(perm))
        else:
            total = sum(cost[i, j] for j, i in enumerate(perm))
        best = min(best, total)
    return best


def test_hungarian_matches_brute_force_on_random_matrices():
    rng = random.Random(7)
    for _ in range(200):
        rows, cols = rng.randint(1, 6), rng.randint(1, 6)
        cost = np.array([[rng.uniform(0, 10) for _ in range(cols)] for _ in range(rows)])
        pairs = assign(cost)
        assert len(pairs) == min(rows, cols)
        assert len({i for i, _ in pairs}) == len(pairs)
        assert len({j for _, j in pairs}) == len(pairs)
        assert sum(cost[i, j] for i, j in pairs) == pytest.approx(brute_force(cost))


def test_hungarian_handles_empty_and_tied_costs():
    assert assign(np.zeros((0, 3))) == []
    assert assign(np.zeros((3, 0))) == []
    assert sorted(assign(np.ones((3, 3)))) == [(0, 0), (1, 1), (2, 2)] or len(assign(np.ones((3, 3)))) == 3


# --- geometry -------------------------------------------------------------

SQUARE = [(-50.0, -50.0), (50.0, -50.0), (50.0, 50.0), (-50.0, 50.0)]


def test_polygon_inside_and_distance_outside():
    assert point_in_polygon(0.0, 0.0, SQUARE)
    assert not point_in_polygon(60.0, 0.0, SQUARE)
    assert distance_outside_polygon(0.0, 0.0, SQUARE) == 0.0
    assert distance_outside_polygon(60.0, 0.0, SQUARE) == pytest.approx(10.0)
    assert distance_outside_polygon(53.0, 54.0, SQUARE) == pytest.approx(5.0)


# --- the run record -------------------------------------------------------


def test_record_interpolates_poses_and_applies_the_ground_offset():
    rec = record(path("vehicle", [(0, 0, 0, 0.2), (10, 10, 0, 5.2)], dt=5), vehicle_ground_offset_m=0.2)
    pose = rec.pose_at("vehicle", 2.5)
    assert (pose.e, pose.n, pose.u) == pytest.approx((2.5, 0.0, 1.25))
    assert rec.pose_at("vehicle", 99).u == pytest.approx(5.0)  # clamped to the last sample
    assert rec.pose_at("payload", 1.0) is None


def test_record_counts_only_flights_that_left_the_ground():
    rec = record(
        flight([(0, 0, 0, 0), (5, 0, 0, 0)]),  # armed and disarmed on the ground
        flight([(10, 0, 0, 0), (20, 0, 0, 10), (30, 0, 0, 0)]),
        flight([(40, 0, 0, 0), (50, 0, 0, 10), (60, 0, 0, 0)]),
    )
    assert [(f.t_arm, f.t_disarm) for f in rec.flights()] == [(10, 30), (40, 60)]


def test_record_without_arm_events_is_one_flight_over_the_whole_trace():
    rec = record(path("vehicle", [(3, 0, 0, 0), (10, 0, 0, 10), (20, 0, 0, 0)]))
    assert [(f.t_arm, f.t_disarm) for f in rec.flights()] == [(3, 20)]


def test_record_rejects_a_line_that_is_neither_pose_nor_event():
    with pytest.raises(ValueError, match="line 1"):
        Record.from_lines([{"t": 1.0, "what": "?"}])


# --- waypoint navigation --------------------------------------------------

WAYPOINTS = [
    {"name": f"wp{i + 1}", "e": 20.0 * (i + 1), "n": 10.0 * (i % 2), "u": 15.0} for i in range(7)
]


def visit(waypoints, speed=5.0, start=(0.0, 0.0, 0.0), offsets=None, t0=0.0):
    """Knots that climb, pass each waypoint (plus an optional offset), and fly home to land."""
    knots = [(t0, *start), (t0 + 5, start[0], start[1], 15.0)]
    for i, wp in enumerate(waypoints):
        de, dn, du = (offsets or {}).get(i, (0.0, 0.0, 0.0))
        target = (wp["e"] + de, wp["n"] + dn, wp["u"] + du)
        t = knots[-1][0] + math.dist(knots[-1][1:], target) / speed
        knots.append((t, *target))
    # Fly home south of the course, so the way back passes no waypoint again.
    for corner in ((knots[-1][1], -30.0, 15.0), (0.0, -30.0, 15.0), (0.0, 0.0, 15.0)):
        t = knots[-1][0] + math.dist(knots[-1][1:], corner) / speed
        knots.append((t, *corner))
    knots.append((knots[-1][0] + 10, 0.0, 0.0, 0.0))
    return knots


def test_waypoint_navigation_scores_every_waypoint_reached_in_order():
    score = judge_run("waypoint_navigation", record(flight(visit(WAYPOINTS))), truth(waypoints=WAYPOINTS))
    assert score["details"]["hit"] == 7
    assert score["base_points"] == pytest.approx(7.0)
    assert all(wp["closest_m"] < 0.2 for wp in score["details"]["waypoints"])


def test_waypoint_navigation_misses_a_waypoint_passed_just_outside_one_metre():
    rec = record(flight(visit(WAYPOINTS, offsets={2: (0.0, 0.0, 1.2)})))
    score = judge_run("waypoint_navigation", rec, truth(waypoints=WAYPOINTS))
    hits = [wp["hit"] for wp in score["details"]["waypoints"]]
    assert hits == [True, True, False, True, True, True, True]
    assert score["details"]["waypoints"][2]["closest_m"] == pytest.approx(1.2, abs=0.05)
    assert score["base_points"] == pytest.approx(6.0)


def test_waypoint_navigation_tolerance_is_3d_by_default_and_2d_on_request():
    rec = record(flight(visit(WAYPOINTS, offsets={i: (0.0, 0.0, -1.5) for i in range(7)})))
    assert judge_run("waypoint_navigation", rec, truth(waypoints=WAYPOINTS))["details"]["hit"] == 0
    flat = judge_run("waypoint_navigation", rec, truth(waypoints=WAYPOINTS), {"dimensions": 2})
    assert flat["details"]["hit"] == 7


def test_waypoint_navigation_does_not_count_a_waypoint_visited_out_of_order():
    swapped = [WAYPOINTS[1], WAYPOINTS[0]] + WAYPOINTS[2:]
    score = judge_run("waypoint_navigation", record(flight(visit(swapped))), truth(waypoints=WAYPOINTS))
    hits = [wp["hit"] for wp in score["details"]["waypoints"]]
    # wp2 was passed before wp1 and never again.
    assert hits == [True, False, True, True, True, True, True]


# --- time trial -----------------------------------------------------------


def time_trial(reference_times, **kwargs):
    knots = visit(WAYPOINTS, **kwargs)
    rec = record(flight(knots))
    return judge_run("time_trial", rec, truth(waypoints=WAYPOINTS), {"reference_times_s": reference_times}), knots


def test_time_trial_times_first_to_last_waypoint_and_ranks_against_references():
    score, knots = time_trial([1000.0])
    # The clock starts when the vehicle first comes within 4 m of wp1 and
    # stops when it first comes within 4 m of wp7, at 5 m/s.
    expected = (knots[8][0] - 4.0 / 5.0) - (knots[2][0] - 4.0 / 5.0)
    assert score["details"]["elapsed_s"] == pytest.approx(expected, abs=0.25)
    assert score["details"]["rank"] == 1
    assert score["base_points"] == pytest.approx(12.0)


def test_time_trial_slowest_gets_eight_points():
    score, _ = time_trial([1.0])
    assert score["details"]["rank"] == 2
    assert score["base_points"] == pytest.approx(8.0)


def test_time_trial_middle_rank_is_between_twelve_and_eight():
    score, _ = time_trial([1.0, 1000.0, 2000.0])
    assert score["details"]["rank"] == 2
    assert score["base_points"] == pytest.approx(12.0 - 4.0 / 3.0)


def test_time_trial_without_reference_times_scores_as_slowest():
    score, _ = time_trial([])
    assert score["base_points"] == pytest.approx(8.0)
    assert any("reference" in note for note in score["notes"])


def test_time_trial_scores_zero_when_a_waypoint_is_missed():
    score, _ = time_trial([1000.0], offsets={4: (0.0, 8.0, 0.0)})
    assert score["details"]["hit"] == 6
    assert score["base_points"] == 0.0
    assert score["details"]["elapsed_s"] is None


# --- localisation ---------------------------------------------------------

MARKERS = [
    {"name": "marker_a", "id": 3, "e": 40.0, "n": 20.0},
    {"name": "marker_b", "id": 7, "e": 60.0, "n": 20.0},
    {"name": "marker_c", "id": 1, "e": 40.0, "n": 50.0},
    {"name": "marker_d", "id": 9, "e": 60.0, "n": 50.0},
]
SURVEY_AREA = [(30.0, 10.0), (70.0, 10.0), (70.0, 60.0), (30.0, 60.0)]
PERFECT = [(3, 40.0, 20.0), (7, 60.0, 20.0), (1, 40.0, 50.0), (9, 60.0, 50.0)]


def survey(height=10.67, written_at=60.0, dip=None):
    knots = [
        (0, 0, 0, 0),
        (10, 0, 0, height),
        (30, 35, 15, height),
        (50, 65, 15, dip if dip is not None else height),
        (70, 65, 55, height),
        (90, 35, 55, height),
        (110, 0, 0, height),
        (120, 0, 0, 0),
    ]
    parts = [flight(knots)]
    if written_at is not None:
        parts.append(event(written_at, "list_written"))
    return record(*parts)


def localise(rows, rec=None, merged=None, settings=None):
    lists = {"list": csv_text(rows)}
    if merged is not None:
        lists["merged_list"] = csv_text(merged)
    return judge_run(
        "localisation",
        rec if rec is not None else survey(),
        truth(markers=MARKERS, survey_area=SURVEY_AREA),
        settings,
        lists=lists,
    )


def test_localisation_perfect_list_scores_every_marker():
    score = localise(PERFECT)
    assert score["base_points"] == pytest.approx(4.0)
    assert score["details"]["correct"] == 4
    assert score["details"]["min_height_m"] == pytest.approx(10.67, abs=0.01)


def test_localisation_position_within_tolerance_counts_and_error_is_reported():
    rows = [(3, 41.5, 20.0)] + PERFECT[1:]
    score = localise(rows)
    assert score["base_points"] == pytest.approx(4.0)
    assert max(score["details"]["errors_m"]) == pytest.approx(1.5, abs=0.02)


def test_localisation_wrong_id_costs_half_a_point():
    rows = [(4, 40.0, 20.0)] + PERFECT[1:]
    score = localise(rows)
    assert score["details"]["wrong_id"] == 1
    assert score["base_points"] == pytest.approx(3.0 - 0.5)


def test_localisation_wrong_location_costs_half_a_point():
    rows = [(3, 43.0, 20.0)] + PERFECT[1:]
    score = localise(rows)
    assert score["details"]["wrong_location"] == 1
    assert score["base_points"] == pytest.approx(3.0 - 0.5)


def test_localisation_wrong_id_and_location_costs_a_point():
    rows = [(4, 43.0, 20.0)] + PERFECT[1:]
    score = localise(rows)
    assert score["details"]["both_wrong"] == 1
    assert score["base_points"] == pytest.approx(3.0 - 1.0)


def test_localisation_extra_row_costs_a_point_and_a_missing_marker_scores_nothing():
    extra = localise(PERFECT + [(5, 50.0, 35.0)])
    assert extra["details"]["both_wrong"] == 1
    assert extra["base_points"] == pytest.approx(4.0 - 1.0)
    missing = localise(PERFECT[:3])
    assert missing["details"]["missed"] == 1
    assert missing["base_points"] == pytest.approx(3.0)


def test_localisation_matching_prefers_the_pairing_with_the_fewest_errors():
    # Two markers 1.5 m apart, both within tolerance of both rows: pairing by
    # distance alone would swap them and report two wrong IDs.
    markers = [{"name": "a", "id": 2, "e": 40.0, "n": 20.0}, {"name": "b", "id": 5, "e": 41.5, "n": 20.0}]
    rows = [(2, 40.9, 20.0), (5, 40.6, 20.0)]
    score = judge_run(
        "localisation", survey(), truth(markers=markers), None, lists={"list": csv_text(rows)}
    )
    assert score["details"]["correct"] == 2


def test_localisation_tolerance_is_a_parameter():
    rows = [(3, 41.5, 20.0)] + PERFECT[1:]
    score = localise(rows, settings={"tolerance_m": 1.0})
    assert score["details"]["wrong_location"] == 1


def test_localisation_between_20_and_25_ft_costs_three_points():
    score = localise(PERFECT, rec=survey(dip=7.0))
    assert score["details"]["min_height_m"] == pytest.approx(7.0, abs=0.01)
    assert score["base_points"] == pytest.approx(4.0 - 3.0)


def test_localisation_below_20_ft_scores_zero():
    score = localise(PERFECT, rec=survey(dip=6.0))
    assert score["base_points"] == 0.0


def test_localisation_height_ignores_takeoff_and_landing_outside_the_survey_area():
    # The vehicle is on the ground at home at both ends of the flight.
    assert localise(PERFECT)["details"]["height_penalty"] == 0.0


def test_localisation_list_must_exist_before_landing():
    late = localise(PERFECT, rec=survey(written_at=125.0))
    assert late["details"]["list_before_landing"] is False
    assert late["base_points"] == 0.0
    never = localise(PERFECT, rec=survey(written_at=None))
    assert never["base_points"] == 0.0


def test_localisation_merged_list_is_scored_separately():
    score = localise(PERFECT[:2], merged=PERFECT)
    assert score["base_points"] == pytest.approx(2.0)
    assert score["details"]["merged"]["points"] == pytest.approx(4.0)


def test_localisation_score_never_goes_below_zero():
    rows = [(0, 100.0, 100.0), (0, 110.0, 100.0), (0, 120.0, 100.0)]
    assert localise(rows)["base_points"] == 0.0


def test_localisation_rejects_a_list_without_the_judges_header():
    with pytest.raises(ValueError, match="ID,Latitude,Longitude"):
        judge_run(
            "localisation", survey(), truth(markers=MARKERS), None, lists={"list": "3,34.1,-118.4\n"}
        )


# --- drop -----------------------------------------------------------------

TARGET = {"name": "drop_target", "e": 30.0, "n": 40.0}


def drop_flight(release_height=2.4, payload_rest=(31.2, 40.0), dip=None, fake=False):
    knots = [
        (0, 0, 0, 0),
        (10, 0, 0, 10),
        (30, 30, 40, 10),
        (40, 30, 40, release_height),
        (45, 30, 40, release_height),
    ]
    if dip is not None:
        knots += [(50, 30, 40, dip), (55, 30, 40, 10)]
    knots += [(70, 30, 40, 10), (90, 0, 0, 10), (100, 0, 0, 0)]
    parts = [flight(knots), event(42.0, "release", fake=fake)]
    if payload_rest is not None:
        parts.append(
            path(
                "payload",
                [(0, 0, 0, 0), (42, 30, 40, release_height), (43, payload_rest[0], payload_rest[1], 0.05), (100, payload_rest[0], payload_rest[1], 0.05)],
            )
        )
    return record(*parts)


def drop(rec, settings=None):
    return judge_run("drop", rec, truth(target=TARGET, payload={"name": "payload"}), settings)


def test_drop_scores_fourteen_minus_the_distance_of_the_payload_from_the_target():
    score = drop(drop_flight())
    assert score["details"]["distance_m"] == pytest.approx(1.2)
    assert score["details"]["release_height_m"] == pytest.approx(2.4)
    assert score["base_points"] == pytest.approx(14.0 - 1.2)


def test_drop_release_below_six_feet_costs_six_and_being_below_six_feet_costs_six():
    score = drop(drop_flight(release_height=1.5))
    assert score["details"]["released_low"] is True
    assert score["details"]["flew_low"] is True
    assert score["base_points"] == pytest.approx(14.0 - 1.2 - 6.0 - 6.0)


def test_drop_vehicle_dipping_below_six_feet_after_a_good_release_costs_six():
    score = drop(drop_flight(dip=1.7))
    assert score["details"]["released_low"] is False
    assert score["details"]["flew_low"] is True
    assert score["details"]["min_height_m"] == pytest.approx(1.7, abs=0.01)
    assert score["base_points"] == pytest.approx(14.0 - 1.2 - 6.0)


def test_drop_takeoff_and_landing_at_home_are_not_flying_low():
    assert drop(drop_flight())["details"]["flew_low"] is False


def test_fake_drop_scores_the_vehicle_position_at_the_release_line():
    rec = drop_flight(payload_rest=None, fake=True)
    score = judge_run("drop", rec, truth(target={"name": "t", "e": 30.5, "n": 40.0}))
    assert score["details"]["scored_from"] == "vehicle at release"
    assert score["base_points"] == pytest.approx(14.0 - 0.5)


def hold_flight(offsets, hold_height=6.0, hold_s=20.0, release=True):
    """Come down over the target to the hold height, stay `hold_s` at the offsets given (east of the target, cycled), release, leave."""
    knots = [(0, 0, 0, 0), (10, 0, 0, 12), (30, 30, 40, 12), (42, 30, 40, hold_height)]
    steps = int(hold_s * 2)
    for index in range(steps + 1):
        knots.append((42 + 0.5 * index, 30 + offsets[index % len(offsets)], 40, hold_height))
    end = 42 + hold_s
    knots += [(end + 5, 30, 40, 12), (end + 25, 0, 0, 12), (end + 35, 0, 0, 0)]
    parts = [flight(knots)]
    if release:
        parts.append(event(end, "release", fake=True))
    return record(*parts)


def test_drop_without_a_hold_height_reports_no_hold():
    details = drop(drop_flight())["details"]
    assert "hold_error_p95_m" not in details


def test_drop_hold_error_is_the_vehicles_true_distance_from_the_target_while_it_holds():
    # Mostly 0.05 m off, a tenth of the time 0.30 m off: the 95th percentile is in the tenth.
    score = drop(hold_flight([0.05] * 9 + [0.30]), {"hold_height_m": 6.0})
    details = score["details"]
    assert details["hold_error_median_m"] == pytest.approx(0.05, abs=0.03)
    assert 0.2 < details["hold_error_p95_m"] <= 0.30 and details["hold_error_max_m"] == pytest.approx(0.30, abs=0.01)
    assert details["hold_s"] == pytest.approx(19.0, abs=0.2)  # the last second of the descent and 20 s at the height, less the 2 s to settle
    assert details["hold_height_mean_m"] == pytest.approx(6.0, abs=0.01) and score["notes"] == []
    steady = drop(hold_flight([0.04]), {"hold_height_m": 6.0})["details"]
    assert steady["hold_error_p95_m"] == pytest.approx(0.04, abs=0.005)


def test_drop_hold_counts_only_the_time_at_the_hold_height_before_the_release():
    # The approach from 12 m and the climb away are far from the target's column in neither case, but they are not the hold:
    # a hold 1 m off that is flown at the wrong height is not measured at all.
    score = drop(hold_flight([0.05], hold_height=6.0), {"hold_height_m": 2.0})
    assert score["details"]["hold_error_p95_m"] is None
    assert any("hold" in note and "2" in note for note in score["notes"])


def test_drop_hold_that_is_too_short_or_never_released_is_not_a_number():
    short = drop(hold_flight([0.05], hold_s=6.0), {"hold_height_m": 6.0})
    assert short["details"]["hold_error_p95_m"] is None and short["details"]["hold_s"] == pytest.approx(5.0, abs=0.2)
    assert any("at least 10" in note for note in short["notes"])
    never = drop(hold_flight([0.05], release=False), {"hold_height_m": 6.0})
    assert never["details"]["hold_error_p95_m"] is None


def test_drop_hold_can_be_judged_at_the_payloads_point_not_the_vehicles():
    # The release point is 0.30 m ahead of the vehicle and 0.10 m to its left. Nose north (yaw 90 degrees):
    # with the payload over the target, the vehicle is 0.30 m south and 0.10 m east of it.
    rec = hold_flight([0.10])
    lines = []
    for name, track in rec.poses.items():
        for pose in track:
            lines.append({"t": pose.t, "pose": name, "e": pose.e, "n": pose.n - 0.30 if pose.u < 7 and 40 < pose.t < 63 else pose.n, "u": pose.u, "yaw": math.pi / 2.0})
    with_yaw = Record.from_lines(lines + rec.events)
    assert with_yaw.poses["vehicle"][0].yaw == pytest.approx(math.pi / 2.0)
    settings = {"hold_height_m": 6.0, "hold_point_body_m": [0.30, 0.10]}
    details = judge_run("drop", with_yaw, truth(target=TARGET, payload={"name": "payload"}), settings)["details"]
    assert details["hold_error_p95_m"] == pytest.approx(0.0, abs=0.005)  # the payload's point is on the target
    assert details["hold_vehicle_error_p95_m"] == pytest.approx(math.hypot(0.30, 0.10), abs=0.005)  # the vehicle is not
    # The same flight judged at the vehicle is 0.32 m off.
    plain = judge_run("drop", with_yaw, truth(target=TARGET, payload={"name": "payload"}), {"hold_height_m": 6.0})["details"]
    assert plain["hold_error_p95_m"] == pytest.approx(math.hypot(0.30, 0.10), abs=0.005) and "hold_vehicle_error_p95_m" not in plain
    # A trace without the vehicle's yaw cannot say where the payload's point was.
    no_yaw = drop(hold_flight([0.05]), settings)
    assert no_yaw["details"]["hold_error_p95_m"] is None and any("yaw" in note for note in no_yaw["notes"])


def test_drop_hold_settings_can_be_set_and_a_typo_is_refused():
    wide = drop(hold_flight([0.05], hold_s=6.0), {"hold_height_m": 6.0, "hold_min_s": 3.0, "hold_settle_s": 1.0, "hold_band_m": 0.3})
    assert wide["details"]["hold_error_p95_m"] == pytest.approx(0.05, abs=0.03)
    with pytest.raises(ValueError, match="hold_heigth_m"):
        drop(hold_flight([0.05]), {"hold_heigth_m": 6.0})


def test_drop_without_a_release_scores_nothing():
    rec = record(flight([(0, 0, 0, 0), (10, 0, 0, 10), (30, 30, 40, 10), (60, 0, 0, 10), (70, 0, 0, 0)]))
    score = drop(rec)
    assert score["base_points"] == 0.0
    assert score["details"]["released"] is False


def test_drop_score_is_never_negative():
    assert drop(drop_flight(payload_rest=(60.0, 40.0)))["base_points"] == 0.0


# --- delivery -------------------------------------------------------------


def delivery_flight(release_height=0.0, rest=(30.2, 40.0), impulses=(), relaunch=True):
    knots = [(0, 0, 0, 0), (10, 0, 0, 10), (30, 30, 40, 10), (60, 30, 40, release_height), (65, 30, 40, release_height)]
    if relaunch:
        knots += [(75, 30, 40, 10), (95, 0, 0, 10), (105, 0, 0, 0)]
    parts = [
        flight(knots),
        event(62.0, "release"),
        path("package", [(0, 0, 0, 0), (62, 30, 40, release_height), (63, rest[0], rest[1], 0.0), (105, rest[0], rest[1], 0.0)]),
    ]
    parts += [event(61.0 + i, "contact", object="package", impulse_ns=value) for i, value in enumerate(impulses)]
    return record(*parts)


def deliver(rec, settings=None, mass_kg=None):
    payload = {"name": "package"}
    if mass_kg is not None:
        payload["mass_kg"] = mass_kg
    return judge_run("delivery", rec, truth(target=TARGET, payload=payload), settings)


def test_delivery_scores_thirteen_minus_distance():
    score = deliver(delivery_flight())
    assert score["details"]["distance_m"] == pytest.approx(0.2)
    assert score["base_points"] == pytest.approx(13.0 - 0.2)


def test_delivery_touchdown_and_relaunch_is_still_one_flight():
    score = deliver(delivery_flight())
    assert score["common"]["flights"] == 1
    assert score["common"]["single_flight"] is True


def test_delivery_released_above_six_feet_scores_zero():
    score = deliver(delivery_flight(release_height=2.5))
    assert score["details"]["released_high"] is True
    assert score["base_points"] == 0.0


def test_delivery_weight_bonus_is_a_quarter_point_per_500_g():
    score = deliver(delivery_flight(), mass_kg=1.2)
    assert score["details"]["weight_bonus"] == pytest.approx(0.5)
    assert score["base_points"] == pytest.approx(13.0 - 0.2 + 0.5)


def test_delivery_logs_contact_impulse_against_the_limit():
    rec = delivery_flight(impulses=(0.4, 1.8, 0.2))
    unknown = deliver(rec)
    assert unknown["details"]["max_impulse_ns"] == pytest.approx(1.8)
    assert unknown["details"]["impulse_ok"] is None  # no limit announced yet
    over = deliver(rec, {"impulse_limit_ns": 1.0})
    assert over["details"]["impulse_ok"] is False
    under = deliver(rec, {"impulse_limit_ns": 2.0})
    assert under["details"]["impulse_ok"] is True
    assert under["base_points"] == over["base_points"]  # logged, not scored


def test_delivery_without_a_release_scores_nothing():
    rec = record(flight([(0, 0, 0, 0), (10, 0, 0, 10), (30, 30, 40, 10), (60, 0, 0, 10), (70, 0, 0, 0)]))
    assert deliver(rec)["base_points"] == 0.0


# --- recovery -------------------------------------------------------------

X_MAT = (50.0, 30.0)
ROSE = {"e": 5.0, "n": -8.0}


def recovery_flight(lift=3.0, set_down=(0.8, 0.0), release_at_height=0.0, release=True):
    vehicle = [
        (0, 0, 0, 0),
        (10, 0, 0, 10),
        (30, *X_MAT, 10),
        (45, *X_MAT, 1.5),
        (55, *X_MAT, 1.5 + lift),
        (80, *set_down, 1.5 + lift),
        (90, *set_down, 1.5 + release_at_height),
        (95, *set_down, 5.0),
        (100, 0, 0, 5.0),
        (105, 0, 0, 0),
    ]
    bucket = [
        (0, *X_MAT, 0.12),
        (45, *X_MAT, 0.12),
        (55, *X_MAT, 0.12 + lift),
        (80, *set_down, 0.12 + lift),
        (90, *set_down, 0.12 + release_at_height),
        (91, *set_down, 0.12),
        (105, *set_down, 0.12),
    ]
    parts = [flight(vehicle), path("bucket", bucket)]
    if release:
        parts.append(event(90.0, "release"))
    return record(*parts)


def recover(rec, settings=None):
    world = truth(bucket={"name": "bucket", "mass_kg": 1.5}, return_points={"takeoff": {"e": 0.0, "n": 0.0}, "compass_rose": ROSE})
    return judge_run("recovery", rec, world, settings)


def test_recovery_scores_sixteen_minus_distance_to_the_takeoff_point():
    score = recover(recovery_flight())
    assert score["details"]["lifted"] is True
    assert score["details"]["max_lift_m"] == pytest.approx(3.0)
    assert score["details"]["distance_m"] == pytest.approx(0.8)
    assert score["base_points"] == pytest.approx(16.0 - 0.8)


def test_recovery_bucket_that_never_rises_half_a_metre_scores_zero():
    score = recover(recovery_flight(lift=0.4))
    assert score["details"]["lifted"] is False
    assert score["base_points"] == 0.0


def test_recovery_bucket_released_above_six_feet_scores_zero():
    score = recover(recovery_flight(release_at_height=2.0))
    assert score["details"]["released_high"] is True
    assert score["base_points"] == 0.0


def test_recovery_return_point_is_a_parameter():
    rec = recovery_flight(set_down=(5.0, -7.0))
    assert recover(rec)["details"]["distance_m"] == pytest.approx(math.hypot(5.0, 7.0))
    rose = recover(rec, {"return_point": "compass_rose"})
    assert rose["details"]["distance_m"] == pytest.approx(1.0)
    assert rose["base_points"] == pytest.approx(15.0)


def test_recovery_landing_with_the_bucket_still_attached_is_scored_where_it_rests():
    score = recover(recovery_flight(release=False))
    assert score["details"]["released"] is False
    assert score["base_points"] == pytest.approx(16.0 - 0.8)


# --- every mission --------------------------------------------------------


def test_autonomy_bonus_is_a_quarter_of_the_points():
    score = drop(drop_flight())
    assert score["common"]["autonomous"] is True
    assert score["autonomy_bonus"] == pytest.approx(0.25 * (14.0 - 1.2))
    assert score["points"] == pytest.approx(1.25 * (14.0 - 1.2))


@pytest.mark.parametrize("mission, settings", [("waypoint_navigation", {}), ("time_trial", {"reference_times_s": [1000.0]})])
def test_waypoint_navigation_and_time_trial_get_no_autonomy_bonus(mission, settings):
    score = judge_run(mission, record(flight(visit(WAYPOINTS))), truth(waypoints=WAYPOINTS), settings)
    assert score["common"]["autonomous"] is True
    assert score["base_points"] > 0.0
    assert score["autonomy_bonus"] == 0.0
    assert score["points"] == score["base_points"]


def test_only_localisation_drop_delivery_and_recovery_get_the_autonomy_bonus():
    from judge import MISSIONS, default_settings

    fractions = {mission: default_settings(mission)["autonomy_bonus_fraction"] for mission in MISSIONS}
    assert {mission for mission, fraction in fractions.items() if fraction} == {"localisation", "drop", "delivery", "recovery"}
    assert all(fraction in (0.0, 0.25) for fraction in fractions.values())


@pytest.mark.parametrize("kind", ["gcs_command", "rc_override"])
def test_ground_station_input_during_the_flight_loses_the_bonus(kind):
    rec = drop_flight()
    rec = Record(rec.poses, rec.events + [event(35.0, kind, what="SET_MODE")])
    score = drop(rec)
    assert score["common"]["autonomous"] is False
    assert score["common"]["human_inputs"][0]["what"] == "SET_MODE"
    assert score["autonomy_bonus"] == 0.0
    assert score["points"] == pytest.approx(14.0 - 1.2)


def test_ground_station_input_before_arming_keeps_the_bonus():
    lines = flight([(10, 0, 0, 0), (20, 0, 0, 10), (40, 30, 40, 10), (50, 30, 40, 2.4), (70, 0, 0, 10), (80, 0, 0, 0)])
    rec = record(lines, event(52.0, "release"), event(2.0, "gcs_command", what="PARAM_SET"))
    assert judge_run("drop", rec, truth(target=TARGET))["common"]["autonomous"] is True


def test_second_flight_is_reported_and_does_not_score():
    first = flight([(0, 0, 0, 0), (10, 0, 0, 10), (30, 30, 40, 10), (50, 0, 0, 10), (60, 0, 0, 0)])
    second = flight([(70, 0, 0, 0), (80, 0, 0, 10), (100, 30, 40, 10), (110, 30, 40, 2.4), (130, 0, 0, 10), (140, 0, 0, 0)])
    rec = record(first, second, event(112.0, "release"))
    score = judge_run("drop", rec, truth(target=TARGET))
    assert score["common"]["flights"] == 2
    assert score["common"]["single_flight"] is False
    assert score["details"]["released"] is False  # the release was in the second flight
    assert score["points"] == 0.0


def test_geofence_breaches_are_measured_from_true_positions():
    fence = {"polygon": SQUARE, "max_height_m": 30.0}
    knots = [(0, 0, 0, 0), (10, 0, 0, 10), (20, 45, 0, 10), (24, 57, 0, 10), (28, 45, 0, 10), (40, 0, 0, 40), (50, 0, 0, 10), (60, 0, 0, 0)]
    rec = record(flight(knots), event(22.5, "fence_breach"), event(23.0, "failsafe", what="RTL"))
    score = judge_run("waypoint_navigation", rec, truth(waypoints=WAYPOINTS, fence=fence))
    breaches = score["common"]["fence_breaches"]
    assert [b["kind"] for b in breaches] == ["lateral", "height"]
    assert breaches[0]["max_excursion_m"] == pytest.approx(7.0, abs=0.05)
    assert breaches[0]["t_start"] == pytest.approx(21.67, abs=0.15)
    assert breaches[1]["max_excursion_m"] == pytest.approx(10.0, abs=0.05)
    assert score["common"]["reported_fence_breaches"] == 1
    assert score["common"]["failsafes"] == [{"t": 23.0, "what": "RTL"}]


def test_no_fence_and_no_events_reports_clean():
    common = drop(drop_flight())["common"]
    assert common["fence_breaches"] == []
    assert common["failsafes"] == []
    assert common["reported_fence_breaches"] == 0


def test_run_with_no_flight_scores_zero():
    rec = record(path("vehicle", [(0, 0, 0, 0), (30, 0, 0, 0)]), event(1, "armed"), event(20, "disarmed"))
    score = judge_run("drop", rec, truth(target=TARGET))
    assert score["common"]["flights"] == 0
    assert score["points"] == 0.0


def test_unknown_mission_or_setting_is_an_error():
    rec = drop_flight()
    with pytest.raises(ValueError, match="unknown mission"):
        judge_run("juggling", rec, truth())
    with pytest.raises(ValueError, match="tolerence_m"):
        judge_run("waypoint_navigation", rec, truth(waypoints=WAYPOINTS), {"tolerence_m": 2.0})


def test_missing_truth_is_an_error_that_names_what_is_missing():
    with pytest.raises(ValueError, match="target"):
        judge_run("drop", drop_flight(), truth())


# --- the rule thresholds, probed just inside and just outside ---------------


def test_default_settings_are_the_values_in_the_rules():
    from judge import default_settings

    assert default_settings("waypoint_navigation")["tolerance_m"] == 1.0
    assert default_settings("waypoint_navigation")["dimensions"] == 3
    assert default_settings("time_trial")["tolerance_m"] == 4.0
    assert (default_settings("time_trial")["points_fastest"], default_settings("time_trial")["points_slowest"]) == (12.0, 8.0)
    loc = default_settings("localisation")
    assert (loc["tolerance_m"], loc["height_penalty_below_m"], loc["height_zero_below_m"]) == (2.0, 7.62, 6.10)
    assert (loc["wrong_id_penalty"], loc["wrong_location_penalty"], loc["both_wrong_penalty"], loc["height_penalty_points"]) == (0.5, 0.5, 1.0, 3.0)
    drop_defaults = default_settings("drop")
    assert (drop_defaults["max_points"], drop_defaults["min_height_m"], drop_defaults["low_release_penalty"], drop_defaults["low_flight_penalty"]) == (14.0, 1.83, 6.0, 6.0)
    delivery = default_settings("delivery")
    assert (delivery["max_points"], delivery["max_release_height_m"], delivery["weight_bonus_per_500g"]) == (13.0, 1.83, 0.25)
    recovery = default_settings("recovery")
    assert (recovery["max_points"], recovery["lift_height_m"], recovery["max_release_height_m"], recovery["return_point"]) == (16.0, 0.5, 1.83, "takeoff")
    assert default_settings("drop")["autonomy_bonus_fraction"] == 0.25


@pytest.mark.parametrize("offset, hit", [(0.9, True), (1.1, False)])
def test_waypoint_tolerance_is_one_metre(offset, hit):
    rec = record(flight(visit(WAYPOINTS, offsets={3: (0.0, 0.0, offset)})))
    score = judge_run("waypoint_navigation", rec, truth(waypoints=WAYPOINTS))
    assert score["details"]["waypoints"][3]["hit"] is hit


@pytest.mark.parametrize("offset, hit", [(3.7, True), (4.3, False)])
def test_time_trial_tolerance_is_four_metres(offset, hit):
    # Passing above the waypoint: the legs to and from it are level, so the
    # closest approach is within 2% of the height offset.
    score, _ = time_trial([1000.0], offsets={3: (0.0, 0.0, offset)})
    assert score["details"]["waypoints"][3]["hit"] is hit


@pytest.mark.parametrize("height, penalty, points", [(7.70, 0.0, 4.0), (7.55, 3.0, 1.0), (6.15, 3.0, 1.0), (6.05, 0.0, 0.0)])
def test_localisation_height_bands_are_25_ft_and_20_ft(height, penalty, points):
    score = localise(PERFECT, rec=survey(dip=height))
    assert score["details"]["height_penalty"] == penalty
    assert score["base_points"] == pytest.approx(points)


@pytest.mark.parametrize("distance, wrong", [(1.9, 0), (2.1, 1)])
def test_localisation_default_tolerance_is_two_metres(distance, wrong):
    score = localise([(3, 40.0 + distance, 20.0)] + PERFECT[1:])
    assert score["details"]["wrong_location"] == wrong


@pytest.mark.parametrize("height, low", [(1.88, False), (1.78, True)])
def test_drop_height_limit_is_six_feet(height, low):
    score = drop(drop_flight(release_height=height))
    assert score["details"]["released_low"] is low
    assert score["details"]["flew_low"] is low


@pytest.mark.parametrize("height, high", [(1.78, False), (1.88, True)])
def test_delivery_release_limit_is_six_feet(height, high):
    assert deliver(delivery_flight(release_height=height))["details"]["released_high"] is high


@pytest.mark.parametrize("height, high", [(1.78, False), (1.88, True)])
def test_recovery_release_limit_is_six_feet(height, high):
    assert recover(recovery_flight(release_at_height=height))["details"]["released_high"] is high


@pytest.mark.parametrize("lift, lifted", [(0.45, False), (0.55, True)])
def test_recovery_lift_threshold_is_half_a_metre(lift, lifted):
    assert recover(recovery_flight(lift=lift))["details"]["lifted"] is lifted


# --- geofence (the SIM-02 pass line, from true positions) ----------------------------------

FENCE = {"polygon": SQUARE, "max_height_m": 100.0}


def fence_flight(furthest=52.0, report_at=None, mode_at=None, mode="RTL", speed=2.0):
    """Fly east from the origin at `speed` to `furthest`, then home. The fence is at 50 m east."""
    t_out = 10.0 + furthest / speed
    knots = [(0, 0, 0, 0), (10, 0, 0, 15), (t_out, furthest, 0, 15), (t_out + furthest / speed, 0, 0, 15), (t_out + furthest / speed + 10, 0, 0, 0)]
    parts = [flight(knots), event(1.0, "mode", mode="GUIDED")]
    if report_at is not None:
        parts.append(event(report_at, "fence_breach", what="Polygon fence breached"))
    if mode_at is not None:
        parts.append(event(mode_at, "mode", mode=mode))
    return record(*parts), 10.0 + 50.0 / speed  # the record, and when the true position crosses the line


def geofence(rec, settings=None):
    return judge_run("geofence", rec, truth(fence=FENCE), settings)


def test_geofence_passes_when_the_autopilot_reports_and_turns_back_in_time():
    rec, t_cross = fence_flight(furthest=52.0, report_at=35.1, mode_at=35.8)
    score = geofence(rec)
    details = score["details"]
    assert details["passed"] is True and score["base_points"] == 1.0
    assert details["t_cross"] == pytest.approx(t_cross, abs=0.11)
    assert details["response_delay_s"] == pytest.approx(0.1, abs=0.11)  # the report, not the 1 Hz mode message
    assert details["max_excursion_m"] == pytest.approx(2.0, abs=0.05)
    assert details["response_mode"] == "RTL"


def test_geofence_response_before_the_line_is_a_negative_delay():
    rec, _ = fence_flight(furthest=51.0, report_at=34.0, mode_at=34.5)
    details = geofence(rec)["details"]
    assert details["passed"] is True and details["response_delay_s"] == pytest.approx(-1.0, abs=0.11)


def test_geofence_vehicle_stopped_before_the_line_passes_with_no_excursion():
    rec, _ = fence_flight(furthest=49.0, report_at=33.0, mode_at=33.5)
    details = geofence(rec)["details"]
    assert details["passed"] is True
    assert details["t_cross"] is None and details["response_delay_s"] is None and details["max_excursion_m"] == 0.0


@pytest.mark.parametrize("mode_at, passed", [(36.8, True), (37.3, False)])
def test_geofence_response_must_come_within_two_seconds(mode_at, passed):
    rec, t_cross = fence_flight(furthest=58.0, mode_at=mode_at)  # no report: only the mode change shows the response
    score = geofence(rec)
    assert score["details"]["passed"] is passed
    assert score["details"]["response_delay_s"] == pytest.approx(mode_at - t_cross, abs=0.11)
    assert score["base_points"] == (1.0 if passed else 0.0)


@pytest.mark.parametrize("furthest, passed", [(59.5, True), (60.5, False)])
def test_geofence_excursion_must_stay_within_ten_metres(furthest, passed):
    rec, _ = fence_flight(furthest=furthest, report_at=35.0, mode_at=35.5)
    assert geofence(rec)["details"]["passed"] is passed


def test_geofence_prompt_report_does_not_excuse_a_late_turn_back():
    """The report alone is not a response: the mode must follow within the limit plus one state message."""
    rec, t_cross = fence_flight(furthest=58.0, report_at=35.1, mode_at=37.9)  # 2.9 s after the line: just in time
    assert geofence(rec)["details"]["passed"] is True
    rec, _ = fence_flight(furthest=58.0, report_at=35.1, mode_at=38.5)  # 3.5 s after it
    score = geofence(rec)
    assert score["details"]["passed"] is False
    assert any("did not leave the commanded flight" in note for note in score["notes"])


def test_geofence_fails_when_the_autopilot_reports_but_keeps_flying():
    rec, _ = fence_flight(furthest=55.0, report_at=35.0)
    score = geofence(rec)
    assert score["details"]["passed"] is False
    assert any("did not leave the commanded flight" in note for note in score["notes"])


def test_geofence_fails_when_the_vehicle_never_reaches_the_fence():
    rec, _ = fence_flight(furthest=30.0)
    score = geofence(rec)
    assert score["details"]["passed"] is False and score["base_points"] == 0.0
    assert any("never reached the fence" in note for note in score["notes"])


def test_geofence_ignores_the_rtl_that_ends_any_flight():
    """Going home at the end of a flight that never neared the fence is not a fence response."""
    rec, _ = fence_flight(furthest=30.0, mode_at=60.0)
    assert geofence(rec)["details"]["passed"] is False


def test_geofence_limits_are_the_pass_lines():
    from judge import default_settings

    defaults = default_settings("geofence")
    assert (defaults["max_response_s"], defaults["max_excursion_m"]) == (2.0, 10.0)
    assert "RTL" in defaults["response_modes"] and "GUIDED" not in defaults["response_modes"]


# --- hover (the flight SIM-03's wind is checked on) ---------------------------------------------------


def hover_flight(drift_e=0.2, hold_s=40.0, height=10.0):
    """Take off, hold `hold_s` seconds at `height`, drifting `drift_e` metres east and back, and land."""
    up = 10.0 + hold_s
    return flight([(0, 0, 0, 0), (10, 0, 0, height), (10 + hold_s / 2.0, drift_e, 0, height), (up, 0, 0, height), (up + 10, 0, 0, 0)])


def test_hover_that_holds_its_place_scores_its_point():
    score = judge_run("hover", record(hover_flight()), truth())
    assert score["base_points"] == 1.0 and score["points"] == 1.0  # no autonomy bonus: it is not a competition mission
    assert score["details"]["passed"] is True and score["details"]["max_drift_m"] == pytest.approx(0.2, abs=0.01)
    assert score["details"]["held_s"] == pytest.approx(56.0, abs=0.3)  # above 2 m: the hold and most of the climb and descent


@pytest.mark.parametrize(
    "kwargs, problem",
    [
        ({"drift_e": 3.5}, "drifted 3.50 m from its takeoff point (limit 3.0 m)"),
        ({"hold_s": 2.0}, "was up for"),
        ({"height": 1.5}, "never rose 2.0 m"),
    ],
)
def test_hover_that_drifts_is_too_short_or_never_rises_scores_nothing(kwargs, problem):
    score = judge_run("hover", record(hover_flight(**kwargs)), truth())
    assert score["base_points"] == 0.0 and score["details"]["passed"] is False
    assert any(problem in note for note in score["notes"]), score["notes"]


def test_hover_headline():
    sys.path.insert(0, str(SIM))
    from runner.result import headline

    assert headline({"mission": "hover", "details": {"max_drift_m": 0.18, "held_s": 74.2}}) == "held within 0.18 m for 74 s"
    assert headline({"mission": "hover", "details": {"max_drift_m": None}}) == "never airborne"
