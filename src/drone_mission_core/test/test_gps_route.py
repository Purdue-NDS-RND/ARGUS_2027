"""The GPS missions' rules (PLT-13): a waypoint's timeout and hold, the route's order, a flying start, AUTO items."""

import math
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_mission_core import gps_route as gr  # noqa: E402

LAT, LON = 40.0, -86.0
M_LAT = 1.0 / 111_194.93
M_LON = M_LAT / math.cos(math.radians(LAT))


def at(east_m, north_m):
    return (LAT + north_m * M_LAT, LON + east_m * M_LON)


def test_a_waypoint_not_reached_in_its_time_is_given_up_and_one_with_a_hold_is_held():
    clock = gr.WaypointClock(timeout_s=30.0, hold_s=2.0)
    assert clock.update(0.0, False) == gr.GO and clock.update(10.0, True) == gr.HOLD and clock.update(11.9, True) == gr.HOLD
    assert clock.update(12.0, True) == gr.REACHED and clock.update(99.0, False) == gr.REACHED and clock.outcome == gr.REACHED
    # Blown out of the radius during the hold: the hold starts over.
    gusty = gr.WaypointClock(30.0, 2.0)
    assert [gusty.update(t, inside) for t, inside in ((0.0, True), (1.5, True), (1.6, False), (2.0, True), (3.9, True), (4.0, True))] == [gr.HOLD, gr.HOLD, gr.GO, gr.HOLD, gr.HOLD, gr.REACHED]
    # Never reached: given up at the timeout, and it stays given up.
    late = gr.WaypointClock(30.0)
    assert late.update(5.0, False) == gr.GO and late.update(34.9, False) == gr.GO and late.update(35.0, False) == gr.TIMED_OUT and late.update(36.0, True) == gr.TIMED_OUT
    # A hold that cannot be kept ends at the timeout too; no timeout means it is waited for.
    assert [gr.WaypointClock(3.0, 5.0).update(t, True) for t in (0.0,)] == [gr.HOLD]
    kept = gr.WaypointClock(3.0, 5.0)
    assert kept.update(0.0, True) == gr.HOLD and kept.update(3.0, True) == gr.TIMED_OUT
    patient = gr.WaypointClock(None)
    assert patient.update(0.0, False) == gr.GO and patient.update(1.0e6, False) == gr.GO and patient.update(1.0e6 + 1, True) == gr.REACHED
    for bad in ((0.0, 0.0), (-1.0, 0.0), (10.0, -1.0), (float("nan"), 0.0)):
        with pytest.raises(ValueError):
            gr.WaypointClock(*bad)


def test_the_shortest_order_is_found_and_is_the_same_every_time():
    course = [at(0, 100), at(0, 50), at(0, 0), at(60, 0), at(60, 150), at(60, 100), at(60, 50)]  # given order doubles back
    given = gr.route_length_m(course)
    order = gr.optimise_order(course)
    best = gr.route_length_m([course[i] for i in order])
    assert sorted(order) == list(range(7)) and best < given - 50.0 and best == pytest.approx(310.0, abs=1.0)  # 410 m as given; 310 m down one side, across, and along the other
    assert gr.optimise_order(course) == order
    # From where the vehicle starts: the first leg counts.
    from_start = gr.optimise_order(course, start=at(60, 160))
    assert from_start[0] == 4 and gr.route_length_m([course[i] for i in from_start], at(60, 160)) <= gr.route_length_m([course[i] for i in order], at(60, 160))
    # Against every order, on random courses.
    rng = random.Random(3)
    for _ in range(20):
        points = [at(rng.uniform(0, 300), rng.uniform(0, 300)) for _ in range(6)]
        found = gr.route_length_m([points[i] for i in gr.optimise_order(points, start=at(0, 0))], at(0, 0))
        import itertools

        assert found == pytest.approx(min(gr.route_length_m([points[i] for i in order], at(0, 0)) for order in itertools.permutations(range(6))))
    # More waypoints than are tried exhaustively: still every waypoint once, and no longer than the given order.
    many = [at(rng.uniform(0, 500), rng.uniform(0, 500)) for _ in range(14)]
    long_order = gr.optimise_order(many, start=at(0, 0))
    assert sorted(long_order) == list(range(14)) and gr.route_length_m([many[i] for i in long_order], at(0, 0)) <= gr.route_length_m(many, at(0, 0))
    assert gr.optimise_order([]) == [] and gr.optimise_order([at(1, 1)]) == [0]
    with pytest.raises(ValueError):
        gr.optimise_order([at(0, 0), (1.0,)])


def test_the_run_in_point_is_before_the_first_waypoint_on_the_first_legs_line():
    first, second = at(100.0, 100.0), at(100.0, 200.0)  # the first leg goes north
    lat, lon = gr.run_in_point(first, second, 30.0)
    assert gr.distance_m((lat, lon), first) == pytest.approx(30.0, abs=0.05) and gr.distance_m((lat, lon), second) == pytest.approx(130.0, abs=0.05)
    assert lon == pytest.approx(first[1]) and lat < first[0]
    with pytest.raises(ValueError):
        gr.run_in_point(first, first, 30.0)
    with pytest.raises(ValueError):
        gr.run_in_point(first, second, 0.0)


def test_the_auto_mission_is_home_then_every_waypoint_with_a_two_metre_pass_radius():
    items = gr.auto_items(at(0, 0), [(*at(0, 100), 15.24), (*at(50, 100), 15.24)])
    assert len(items) == 3 and all(item.command == 16 and item.frame == 3 and item.autocontinue for item in items)
    home, first, second = items
    assert (home.latitude, home.longitude) == at(0, 0) and not home.is_current and first.is_current and not second.is_current
    assert first.param2 == second.param2 == gr.AUTO_PASS_RADIUS_M == 2.0 and first.param1 == 0.0 and first.param3 == 0.0 and math.isnan(first.param4)
    assert (first.latitude, first.longitude, first.altitude_m) == (*at(0, 100), 15.24)
    for bad in ([], [(*at(0, 100), 0.0)], [(40.0, -86.0)]):
        with pytest.raises(ValueError):
            gr.auto_items(at(0, 0), bad)
    with pytest.raises(ValueError):
        gr.auto_items(at(0, 0), [(*at(0, 100), 15.0)], pass_radius_m=0.0)


def test_the_upload_sends_the_items_and_keeps_the_newest_item_reached():
    sent = []

    class Client:
        def service_is_ready(self):
            return True

        def call_async(self, request):
            sent.append(request)
            return type("Future", (), {"add_done_callback": lambda self, callback: callback(self)})()

    upload = gr.MissionUpload(Client(), lambda items: [item.altitude_m for item in items])
    assert upload.ready() and upload.reached_seq is None
    upload.on_reached(type("Reached", (), {"wp_seq": 2})())
    assert upload.reached_seq == 2
    answered = []
    upload.push(gr.auto_items(at(0, 0), [(*at(0, 100), 15.0)]), answered.append)
    assert sent == [[0.0, 15.0]] and len(answered) == 1 and upload.reached_seq is None  # a new mission starts from nothing reached
    assert (gr.AUTO_FLAG, gr.MISSION_PUSH_SERVICE, gr.MISSION_REACHED_TOPIC) == ("time_trial_auto", "/mavros/mission/push", "/mavros/mission/reached")


def _least_cpu_s(run, repeats=5):
    """The least processor time of several runs. A busy machine makes the wall clock say anything it likes about
    this code; the processor time the code itself used, at its least over a few runs, it barely moves."""
    import time

    least = float("inf")
    for _ in range(repeats):
        started = time.process_time()
        run()
        least = min(least, time.process_time() - started)
    return least


def test_the_exact_search_fits_in_a_tick_and_the_boundary_holds_a_way():
    """The flight-safety review: trying every order of nine waypoints took seconds inside one 50 ms tick."""
    rng = random.Random(9)
    most = [at(rng.uniform(0, 500), rng.uniform(0, 500)) for _ in range(gr.EXACT_UP_TO)]
    took = []
    for points in (most, most[:7]):
        assert sorted(gr.optimise_order(points, start=at(0, 0))) == list(range(len(points)))
        took.append(_least_cpu_s(lambda: gr.optimise_order(points, start=at(0, 0))))  # noqa: B023
    assert gr.EXACT_UP_TO == 8 and took[0] < 0.05 and took[1] < 0.05, took  # inside one 50 ms tick (about 1 ms and 0.5 ms of processor time here)
    field = [at(0, 0), at(100, 0), at(100, 100), at(0, 100)]
    assert gr.inside_boundary(at(50, 50), field) and not gr.inside_boundary(at(2, 50), field) and not gr.inside_boundary(at(150, 50), field)
    assert gr.inside_boundary(at(150, 50), None) and gr.way_inside_boundary([at(10, 10), at(90, 90)], field)
    assert not gr.way_inside_boundary([at(10, 10), at(50, 120), at(90, 10)], field)
