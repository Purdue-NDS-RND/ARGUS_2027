"""The GPS waypoint mission's 2027 behaviours (PLT-13): timeout and hold, the optimiser, a flying start, AUTO.

rclpy and the message packages are the stand-ins from test_package_drop_mission.py.
"""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_centring_missions as centring_tests  # noqa: E402  (installs the ROS stand-ins)

from drone_mission_core import gps_route  # noqa: E402
from drone_mission_core.mission_api import MissionSpec, MissionStatus  # noqa: E402
from drone_mission_demo.missions import gps_route_2027 as route_module  # noqa: E402
from drone_mission_demo.missions.gps_waypoint_mission import GpsWaypointMission  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
LAT, LON = 40.0, -86.0
M_LAT = 1.0 / 111_194.93
M_LON = M_LAT / math.cos(math.radians(LAT))
S = 1_000_000_000


def at(east_m, north_m):
    return (LAT + north_m * M_LAT, LON + east_m * M_LON)


SQUARE = [("wp1", 0.0, 60.0), ("wp2", 60.0, 60.0), ("wp3", 60.0, 0.0), ("wp4", 0.0, 0.0)]


def waypoints(points=SQUARE):
    return [{"name": name, "latitude": at(east, north)[0], "longitude": at(east, north)[1]} for name, east, north in points]


def mission(flags=(), mode="waypoint_navigation", points=SQUARE, **config):
    spec = MissionSpec(
        type_name="gps_waypoint", name="route", base_dir=REPO,
        config={"waypoints": waypoints(points), "waypoint_altitude_m": 15.0, "arrival_radius_m": 3.0, "arrival_alt_tolerance_m": 2.0, "hold_time_s": 0.0, "mode": mode, **config},
        features={flag: True for flag in flags},
    )  # fmt: skip
    return GpsWaypointMission(spec)


class Context:
    """A vehicle at 15 m that flies at 10 m/s to the global setpoint it is given; in AUTO it flies its uploaded items."""

    def __init__(self, east=-30.0, north=30.0, blocked=(), upload="ok", auto="ok", service=True, auto_stalls_after=None):
        self.t = 100.0
        self.pos = [east, north]
        self.blocked = set(blocked)  # setpoints within 1 m of these (east, north) are never reached: the vehicle stops 20 m short
        self.infos, self.warnings = [], []
        self.logger = SimpleNamespace(info=self.infos.append, warn=self.warnings.append, error=self.warnings.append)
        self.mavros_state = SimpleNamespace(connected=True, mode="GUIDED")
        self.setpoint = None
        self.setpoints = []
        self.upload, self.auto, self.service, self.auto_stalls_after = upload, auto, service, auto_stalls_after
        self.items, self.reached_seq, self.modes, self.pending = None, None, [], []

    def now(self):
        return centring_tests.stand_ins.FakeTime(int(round(self.t * S)))

    def seconds_since(self, start):
        return self.t - start.nanoseconds / S

    @property
    def global_gps(self):
        lat, lon = at(*self.pos)
        return SimpleNamespace(latitude=lat, longitude=lon, status=SimpleNamespace(status=0))

    @property
    def local_pose(self):
        return SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=self.pos[0], y=self.pos[1], z=15.0)))

    def set_global_position_setpoint(self, latitude, longitude, altitude_m, yaw_deg=90.0, lock_yaw=True):
        self.setpoint = ((longitude - LON) / M_LON, (latitude - LAT) / M_LAT)
        self.setpoints.append((round(self.t, 2), round(self.setpoint[0], 1), round(self.setpoint[1], 1)))

    def clear_global_position_setpoint(self):
        self.setpoint = None

    def request_mode_change(self, mode_name, done_callback=None):
        self.modes.append(mode_name)
        taken = mode_name != "AUTO" or self.auto == "ok"
        if taken:
            self.mavros_state.mode = mode_name
        if done_callback is not None:
            self.pending.append((done_callback, SimpleNamespace(mode_sent=taken)))

    def mission_upload_ready(self):
        return self.service

    def upload_mission(self, items, done_callback=None):
        self.items = list(items)
        if self.upload != "silent" and done_callback is not None:
            self.pending.append((done_callback, SimpleNamespace(success=self.upload == "ok")))

    def mission_reached_seq(self):
        return self.reached_seq

    def _toward(self, goal, dt, speed=10.0):
        stop_short = 20.0 if any(math.hypot(goal[0] - e, goal[1] - n) < 1.0 for e, n in self.blocked) else 0.0
        delta = [goal[0] - self.pos[0], goal[1] - self.pos[1]]
        distance = math.hypot(*delta)
        if distance > stop_short + 1.0e-9:
            step = min(speed * dt, distance - stop_short)
            self.pos = [self.pos[0] + delta[0] * step / distance, self.pos[1] + delta[1] * step / distance]
        return distance

    def tick(self, update, dt=0.05):
        self.t += dt
        status = update(self)
        while self.pending:
            callback, response = self.pending.pop(0)
            callback(SimpleNamespace(result=lambda response=response: response))
        if self.mavros_state.mode == "AUTO" and self.items:
            seq = 1 if self.reached_seq is None else self.reached_seq + 1
            stalled = self.auto_stalls_after is not None and seq > self.auto_stalls_after
            if seq < len(self.items) and not stalled:
                item = self.items[seq]
                goal = ((item.longitude - LON) / M_LON, (item.latitude - LAT) / M_LAT)
                if self._toward(goal, dt) <= 2.0:
                    self.reached_seq = seq
        elif self.setpoint is not None:
            self._toward(self.setpoint, dt)
        return status


def fly(route, context, seconds=300.0):
    status = MissionStatus.RUNNING
    for _ in range(int(seconds * 20)):
        status = context.tick(route.update)
        if status not in (MissionStatus.RUNNING, MissionStatus.WAITING):
            break
    return status


def started(flags=(), context=None, **config):
    flown = mission(flags, **config)
    context = context or Context()
    flown.on_enter(context)
    return flown, route_module.Route2027(flown), context


def test_a_waypoint_that_cannot_be_reached_is_given_up_at_its_timeout_and_the_rest_are_flown():
    flown, route, context = started(("waypoint_timeout_hold",), Context(blocked=[(60.0, 60.0)]), waypoint_timeout_s=15.0)
    assert fly(route, context) == MissionStatus.SUCCESS
    assert route.reached == ["wp1", "wp3", "wp4"] and route.given_up == ["wp2"]
    assert any("wp2" in line and "given up" in line for line in context.warnings) and any("3 of 4 waypoints reached; given up: wp2" in line for line in context.infos)
    # Without the timeout the same route waits at wp2 for ever (the 2026 behaviour, kept when no timeout is given).
    flown, route, context = started(("waypoint_timeout_hold",), Context(blocked=[(60.0, 60.0)]))
    assert fly(route, context, seconds=120.0) == MissionStatus.RUNNING and route.reached == ["wp1"]
    # One waypoint's own timeout, by name.
    flown, route, context = started(("waypoint_timeout_hold",), Context(blocked=[(60.0, 60.0)]), waypoint_timeouts_s={"wp2": 8.0})
    assert fly(route, context) == MissionStatus.SUCCESS and route.given_up == ["wp2"]


def test_a_waypoint_with_a_hold_is_held_inside_its_radius_before_the_next():
    flown, route, context = started(("waypoint_timeout_hold",), waypoint_holds_s={"wp2": 4.0})
    assert fly(route, context) == MissionStatus.SUCCESS and route.reached == ["wp1", "wp2", "wp3", "wp4"]
    to_wp2 = [t for t, east, north in context.setpoints if (east, north) == (60.0, 60.0)]
    to_wp1 = [t for t, east, north in context.setpoints if (east, north) == (0.0, 60.0)]
    assert (to_wp2[-1] - to_wp2[0]) - (to_wp1[-1] - to_wp1[0]) > 3.5  # wp2 was the setpoint four seconds longer than its leg needs
    for bad in ({"waypoint_holds_s": {"wp9": 1.0}}, {"waypoint_timeout_s": 0.0}, {"waypoint_timeouts_s": {"wp1": -1.0}}, {"flying_start_run_in_m": 500.0}):
        with pytest.raises(ValueError):
            started(("waypoint_timeout_hold",), **bad)


def test_the_optimiser_flies_the_shortest_order_from_where_the_vehicle_is():
    doubling_back = [("a", 0.0, 100.0), ("b", 0.0, 0.0), ("c", 0.0, 50.0), ("d", 0.0, 150.0)]
    flown, route, context = started(("route_optimiser",), Context(east=0.0, north=-20.0), points=doubling_back)
    assert fly(route, context) == MissionStatus.SUCCESS and route.reached == ["b", "c", "a", "d"]
    assert any("Route optimiser: order 2-3-1-4, 170 m against 370 m as given" in line for line in context.infos)
    # The 2026 course: 934 m against 1278 m, in the order 3-2-1-4-7-6-5 (FEATURES.md).
    course = yaml.safe_load((REPO / "src" / "drone_mission_demo" / "config" / "waypoints" / "competition_waypoints.yaml").read_text())["waypoints"]
    points = [(float(item["latitude"]), float(item["longitude"])) for item in course]
    order = gps_route.optimise_order(points)
    assert [i + 1 for i in order] == [3, 2, 1, 4, 7, 6, 5]
    assert round(gps_route.route_length_m(points)) == 1278 and round(gps_route.route_length_m([points[i] for i in order])) == 934


def test_a_flying_start_runs_in_on_the_first_legs_line_and_is_not_a_waypoint_of_the_course():
    flown, route, context = started(("flying_start",), mode="circuit_time_trial", flying_start_run_in_m=30.0)
    assert fly(route, context) == MissionStatus.SUCCESS and route.reached == ["wp1", "wp2", "wp3", "wp4"]
    assert context.setpoints[0][1:] == (-30.0, 60.0)  # 30 m before wp1 on the line wp1 -> wp2 (east)
    assert any("Flying start" in line for line in context.infos) and any("Circuit timer started" in line for line in context.infos)
    # The vehicle passes wp1 moving along the first leg: it came from the run-in point, due west of it.
    first_wp1 = next(i for i, (_t, east, north) in enumerate(context.setpoints) if (east, north) == (0.0, 60.0))
    assert all(point[1:] == (-30.0, 60.0) for point in context.setpoints[:first_wp1])
    assert flown._route_start_time is not None and flown._route_end_time is not None


def test_the_time_trial_is_uploaded_and_flown_in_auto_and_handed_back_to_guided():
    flown, route, context = started(("time_trial_auto",), mode="circuit_time_trial")
    assert fly(route, context) == MissionStatus.SUCCESS and route.flown_by == "AUTO" and route.reached == ["wp1", "wp2", "wp3", "wp4"]
    assert context.modes == ["AUTO", "GUIDED"] and context.mavros_state.mode == "GUIDED"
    assert len(context.items) == 5 and all(item.param2 == 2.0 for item in context.items[1:]) and context.items[1].is_current
    in_auto = [point for point in context.setpoints if point[1:] != (0.0, 0.0)]
    assert in_auto == []  # no setpoint of ours while the flight controller flew the route; the last waypoint is held once GUIDED has it
    assert any("Route flown by AUTO: 4 of 4" in line for line in context.infos)
    # With a flying start the run-in point is the first item, and is not counted.
    flown, route, context = started(("time_trial_auto", "flying_start"), mode="circuit_time_trial")
    assert fly(route, context) == MissionStatus.SUCCESS and len(context.items) == 6 and route.reached == ["wp1", "wp2", "wp3", "wp4"]


@pytest.mark.parametrize(
    "fault, reason",
    [({"service": False}, "upload service is not there"), ({"upload": "refused"}, "upload was refused"), ({"upload": "silent"}, "upload was not answered"), ({"auto": "refused"}, "did not take AUTO")],
)
def test_when_auto_cannot_be_flown_the_route_is_flown_in_guided(fault, reason):
    flown, route, context = started(("time_trial_auto",), Context(**fault), mode="circuit_time_trial")
    assert fly(route, context) == MissionStatus.SUCCESS and route.flown_by == "GUIDED" and reason in route.fallback_reason
    assert route.reached == ["wp1", "wp2", "wp3", "wp4"] and any("the rest of the route in GUIDED" in line for line in context.warnings)


def test_auto_that_stops_making_progress_is_taken_back_and_the_rest_flown_in_guided():
    flown, route, context = started(("time_trial_auto",), Context(auto_stalls_after=2), mode="circuit_time_trial", auto_stall_s=5.0)
    assert fly(route, context) == MissionStatus.SUCCESS and route.reached == ["wp1", "wp2", "wp3", "wp4"]
    assert "no progress toward wp3 in 5 s of AUTO" in route.fallback_reason and context.modes[:2] == ["AUTO", "GUIDED"] and context.mavros_state.mode == "GUIDED"
    assert [point[1:] for point in context.setpoints if point[1:] in ((60.0, 0.0), (0.0, 0.0))]  # wp3 and wp4 by our own setpoints


# --- wired into the 2026 mission ---------------------------------------------------------------


def run_mission(flown, context, seconds=300.0):
    flown.on_enter(context)
    status = MissionStatus.RUNNING
    for _ in range(int(seconds * 20)):
        status = context.tick(flown.update)
        if status not in (MissionStatus.RUNNING, MissionStatus.WAITING):
            break
    return status


def test_with_every_flag_off_the_2026_mission_flies_and_the_2027_module_is_not_used():
    from drone_mission_demo.missions import gps_waypoint_mission as module

    assert module.ROUTE_2027_FLAGS == route_module.FLAGS and gps_route.AUTO_FLAG == route_module.AUTO_FLAG
    plain, context = mission(), Context()
    assert run_mission(plain, context) == MissionStatus.SUCCESS and plain._route_2027 is None
    assert not any("Route" in line for line in context.infos) and any("Reached GPS waypoint #4" in line for line in context.infos)
    # The same setpoints, in the same order, as a flag-on flight that changes nothing about this route.
    flagged, flagged_context = mission(("waypoint_timeout_hold",)), Context()
    assert run_mission(flagged, flagged_context) == MissionStatus.SUCCESS and flagged._route_2027 is not None
    assert [point[1:] for point in context.setpoints] == [point[1:] for point in flagged_context.setpoints]


def test_with_a_flag_on_the_missions_own_update_flies_the_2027_route():
    flown, context = mission(("waypoint_timeout_hold", "route_optimiser"), waypoint_timeout_s=15.0), Context(east=70.0, north=-10.0, blocked=[(0.0, 60.0)])
    assert run_mission(flown, context) == MissionStatus.SUCCESS
    assert flown._route_2027.reached == ["wp3", "wp2", "wp4"]  # the shortest order from where it starts is 3-2-1-4, and wp1 could not be reached
    assert flown._route_2027.given_up == ["wp1"] and any("GPS waypoint advisory summary" in line for line in context.infos)


def test_the_live_sequences_declare_the_four_flags_false_and_carry_their_settings():
    for name in ("waypoint_navigation_live.yaml", "circuit_time_trial_live.yaml"):
        sequence = yaml.safe_load((REPO / "src" / "drone_mission_demo" / "config" / "sequences" / name).read_text())["mission_sequence"]
        assert all(sequence["features"][flag] is False for flag in route_module.FLAGS), name
        config = next(item["config"] for item in sequence["missions"] if item["type"] == "gps_waypoint")
        assert (config["waypoint_timeout_s"], config["waypoint_timeouts_s"], config["waypoint_holds_s"], config["flying_start_run_in_m"], config["auto_stall_s"]) == (None, {}, {}, 30.0, 20.0)


# --- the flight-safety review's cases ------------------------------------------------------------


def test_a_mode_taken_by_the_pilot_or_a_failsafe_during_auto_is_not_taken_back():
    """The review's blocker: any mode but AUTO was answered with a GUIDED request, and the route resumed."""
    flown, route, context = started(("time_trial_auto",), mode="circuit_time_trial")
    for _ in range(int(60 * 20)):
        context.tick(route.update)
        if route.state == route_module.AUTO_FLY and len(route.reached) == 1:
            break
    asked = list(context.modes)
    context.mavros_state.mode = "RTL"  # a failsafe, or the safety pilot
    for _ in range(int(30 * 20)):
        assert context.tick(route.update) == MissionStatus.RUNNING
    assert context.modes == asked and route.state == route_module.AUTO_PAUSED and context.setpoint is None  # nothing asked for, nothing commanded
    assert any("taken by the pilot or a failsafe" in line for line in context.warnings)
    # Handed back in GUIDED: the rest of the route is flown, with no mode asked for.
    context.mavros_state.mode = "GUIDED"
    assert fly(route, context) == MissionStatus.SUCCESS and context.modes == asked and route.reached == ["wp1", "wp2", "wp3", "wp4"]
    # At the route's end, a mode that is not ours is left alone too.
    flown, route, context = started(("time_trial_auto",), mode="circuit_time_trial")
    for _ in range(int(120 * 20)):
        context.tick(route.update)
        if len(route.reached) == 4:
            break
    context.mavros_state.mode = "LOITER"
    asked = list(context.modes)
    assert fly(route, context, seconds=10.0) == MissionStatus.SUCCESS and context.modes == asked


def test_guided_is_asked_for_only_out_of_our_own_auto_and_three_times_at_most():
    class Sticky(Context):
        def request_mode_change(self, mode_name, done_callback=None):
            self.modes.append(mode_name)
            if mode_name == "AUTO":
                self.mavros_state.mode = "AUTO"  # and it never leaves AUTO again, whatever is asked
            if done_callback is not None:
                self.pending.append((done_callback, SimpleNamespace(mode_sent=True)))

    flown, route, context = started(("time_trial_auto",), Sticky(), mode="circuit_time_trial")
    assert fly(route, context) == MissionStatus.FAILURE and context.modes == ["AUTO", "GUIDED", "GUIDED", "GUIDED"]
    assert "stayed in AUTO after GUIDED was asked for 3 times" in context.warnings[-1]
    # Left in mid-AUTO (a failure elsewhere, a cancel): the AUTO mission is not left flying by itself.
    flown, context = mission(("time_trial_auto",), mode="circuit_time_trial"), Context()
    flown.on_enter(context)
    for _ in range(int(20 * 20)):
        context.tick(flown.update)
        if flown._route_2027.state == route_module.AUTO_FLY:
            break
    flown.on_exit(context)
    assert context.modes == ["AUTO", "GUIDED"]
    # With the flags off the mission's exit asks for nothing.
    plain, context = mission(), Context()
    plain.on_enter(context)
    plain.on_exit(context)
    assert context.modes == []


def test_auto_on_a_long_leg_is_not_called_stalled_while_the_waypoint_comes_nearer():
    far = [("wp1", 0.0, 60.0), ("wp2", 600.0, 60.0)]  # a 600 m leg: a minute at the test vehicle's 10 m/s
    flown, route, context = started(("time_trial_auto",), mode="circuit_time_trial", points=far, auto_stall_s=5.0)
    assert fly(route, context) == MissionStatus.SUCCESS and route.flown_by == "AUTO" and route.fallback_reason == ""


def test_a_route_on_which_nothing_was_reached_fails():
    everywhere = [(east, north) for _name, east, north in SQUARE]
    flown, route, context = started(("waypoint_timeout_hold",), Context(blocked=everywhere), waypoint_timeout_s=5.0)
    assert fly(route, context) == MissionStatus.FAILURE and route.reached == [] and len(route.given_up) == 4
    assert "no waypoint of the route was reached" in context.warnings[-1] and route.update(context) == MissionStatus.FAILURE


def test_the_run_in_point_and_a_reordered_route_stay_inside_the_flight_boundary():
    tight = [list(at(east, north)) for east, north in ((-40.0, -20.0), (80.0, -20.0), (80.0, 80.0), (-10.0, 80.0), (-10.0, 20.0), (-40.0, 20.0))]  # the run-in point (-30, 60) is outside
    flown, route, context = started(("flying_start",), mode="circuit_time_trial", flight_boundary=tight)
    assert fly(route, context) == MissionStatus.SUCCESS and route.run_in is None and context.setpoints[0][1:] == (0.0, 60.0)
    assert any("leaves the flight boundary: no run-in" in line for line in context.warnings)
    roomy = [list(at(east, north)) for east, north in ((-60.0, -40.0), (100.0, -40.0), (100.0, 100.0), (-60.0, 100.0))]
    flown, route, context = started(("flying_start",), mode="circuit_time_trial", flight_boundary=roomy)
    assert fly(route, context) == MissionStatus.SUCCESS and context.setpoints[0][1:] == (-30.0, 60.0) and not context.warnings
    # No boundary given: the mission says what then holds the run-in point.
    flown, route, context = started(("flying_start",), mode="circuit_time_trial")
    fly(route, context)
    assert any("no flight_boundary is given" in line for line in context.warnings)
    with pytest.raises(ValueError, match="flight_boundary"):
        started(("flying_start",), flight_boundary=[[1.0, 2.0]])


def test_every_path_logs_as_rclpy_allows():
    """Found in the sim: a helper that chose info or warn at one call site ended the mission at its first warning."""
    from test_drop_release_gates import OneSeverityPerLine

    def strict(context):
        context.logger = OneSeverityPerLine()
        return context

    cases = [
        (("waypoint_timeout_hold",), strict(Context(blocked=[(60.0, 60.0)])), {"waypoint_timeout_s": 8.0, "waypoint_holds_s": {"wp1": 1.0}}),
        (("time_trial_auto", "flying_start", "route_optimiser"), strict(Context()), {"mode": "circuit_time_trial"}),
        (("time_trial_auto",), strict(Context(service=False)), {"mode": "circuit_time_trial"}),
        (("time_trial_auto",), strict(Context(auto_stalls_after=1)), {"mode": "circuit_time_trial", "auto_stall_s": 3.0}),
        (("waypoint_timeout_hold",), strict(Context(blocked=[(east, north) for _n, east, north in SQUARE])), {"waypoint_timeout_s": 3.0}),
    ]
    for flags, context, config in cases:
        flown = mission(flags, **config)
        status = run_mission(flown, context)
        flown.on_exit(context)
        assert status in (MissionStatus.SUCCESS, MissionStatus.FAILURE) and context.logger.lines, flags
    # A pilot's mode during AUTO: the warning and the waiting log on their own lines too.
    flown, context = mission(("time_trial_auto",), mode="circuit_time_trial"), strict(Context())
    flown.on_enter(context)
    for _ in range(int(20 * 20)):
        context.tick(flown.update)
        if flown._route_2027.state == route_module.AUTO_FLY:
            break
    context.mavros_state.mode = "LOITER"
    for _ in range(40):
        context.tick(flown.update)
    assert flown._route_2027.state == route_module.AUTO_PAUSED


def test_a_waypoint_is_not_given_up_for_time_somebody_else_had_the_vehicle():
    """The review, round two: in the pilot's LOITER the waypoints timed out one by one and the sequence went on to RTL."""
    flown, route, context = started(("waypoint_timeout_hold",), waypoint_timeout_s=10.0)
    for _ in range(int(3 * 20)):
        context.tick(route.update)
    context.mavros_state.mode = "LOITER"
    held = list(context.pos)
    context._toward = lambda goal, dt, speed=10.0: 99.0  # the vehicle does not follow our setpoint: it is the pilot's
    for _ in range(int(60 * 20)):
        assert context.tick(route.update) == MissionStatus.RUNNING
    assert route.given_up == [] and context.pos == held
    # Handed back: the waypoint's time starts again, and the route is flown.
    del context._toward
    context.mavros_state.mode = "GUIDED"
    assert fly(route, context) == MissionStatus.SUCCESS and route.given_up == [] and len(route.reached) == 4


def test_more_waypoints_than_are_ordered_in_a_tick_are_flown_as_given():
    many = [(f"p{i}", 10.0 * i, 0.0 if i % 2 else 40.0) for i in range(gps_route.EXACT_UP_TO + 1)]
    flown, route, context = started(("route_optimiser",), points=many)
    assert fly(route, context) == MissionStatus.SUCCESS and route.order == list(range(len(many)))
    assert any("the given order is flown" in line for line in context.warnings)
