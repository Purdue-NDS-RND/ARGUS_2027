"""The survey mission (LOC-3), through its real update().

`search_pattern` plans a lawnmower over the search area (LOC-2), climbs to the
survey height (LOC-1), sets the survey speed, starts the localisation
pipeline, flies the tracks at a constant yaw with the height guard watching,
and ends cleanly when the tracks are done or its time is up. Behind the
mission flag `survey_search`: with the flag off the mission is skipped.

rclpy and the message packages are the stand-ins from
test_package_drop_mission.py.
"""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml  # noqa: F401  (the real one, before the stand-ins put an empty module in its place)

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_centring_missions as centring_tests  # noqa: E402  (installs the ROS stand-ins)
from test_drop_release_gates import OneSeverityPerLine  # noqa: E402

from drone_mission_core import coverage_planner, survey_height, target_access  # noqa: E402
from drone_mission_core.features import FLAG_REQUIRES  # noqa: E402
from drone_mission_core.mission_api import MissionSpec, MissionStatus  # noqa: E402
from drone_mission_demo.missions.search_pattern_mission import SearchPatternMission, SearchState  # noqa: E402

FLAG = "survey_search"
ON = {"use_frame_time_pose": True, "use_height_estimator": True, FLAG: True}
LAT0, LON0 = 47.3977, 8.5456
M_PER_DEG_LAT = 111_320.0
M_PER_DEG_LON = 111_320.0 * math.cos(math.radians(LAT0))


def corner(east, north):
    return [LAT0 + north / M_PER_DEG_LAT, LON0 + east / M_PER_DEG_LON]


AREA = [corner(20.0, 10.0), corner(80.0, 10.0), corner(80.0, 50.0), corner(20.0, 50.0)]  # 60 x 40 m, east of home
CONFIG = {"search_area": AREA, "camera_mode": "sim_survey", "airframe": "sim_iris_downcam", "time_budget_s": 600.0}
HEIGHT = survey_height.DEFAULT_HEIGHT_M


class Context:
    """What a mission sees, with a vehicle that goes where it is told."""

    def __init__(self):
        self._now_ns = 100 * 1_000_000_000
        self.infos, self.warnings = [], []
        self.logger = SimpleNamespace(info=self.infos.append, warn=self.warnings.append, error=self.warnings.append)
        self.position = [0.0, 0.0, 12.0]
        self.local_pose = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0, z=12.0), orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
        self.global_gps = SimpleNamespace(latitude=LAT0, longitude=LON0)
        self.ground_up = 0.0  # the ground under the vehicle, in the local frame
        self.height_source = "rangefinder"
        self.no_height = False
        self.setpoints = []
        self.speed_commands, self.speed_answers = [], True
        self.pending = []
        self.loc_ready, self.loc_calls = True, []
        self.fly = True  # the vehicle moves toward its setpoint
        self.speed = 4.0
        self.cleared = 0
        self.height_asked_with = set()
        self.answer = True  # service calls are answered; False: the answer never comes

    def now(self):
        return centring_tests.stand_ins.FakeTime(self._now_ns)

    def seconds_since(self, start):
        return (self._now_ns - start.nanoseconds) * 1.0e-9

    def latlon_to_local_enu(self, latitude, longitude):
        return ((longitude - LON0) * M_PER_DEG_LON, (latitude - LAT0) * M_PER_DEG_LAT)

    def height_agl(self, ground_below_home_m=0.0):
        self.height_asked_with.add(ground_below_home_m)
        if self.no_height:
            return None
        return target_access.HeightAgl(self.position[2] - self.ground_up, self.height_source)

    def set_local_position_setpoint(self, east_m, north_m, up_m, yaw_deg=90.0):
        self.setpoints.append((east_m, north_m, up_m, yaw_deg))

    def clear_all_setpoints(self):
        self.cleared += 1

    def command_service_ready(self):
        return True

    def command_change_speed(self, speed_mps, done_callback=None):
        self.speed_commands.append(speed_mps)
        self.pending.append((done_callback, self.speed_answers))

    def localisation_control_ready(self):
        return self.loc_ready

    def set_localisation_enabled(self, enabled, done_callback=None):
        self.loc_calls.append(enabled)
        self.pending.append((done_callback, True))

    def tick(self, mission, dt=0.05):
        self._now_ns += int(dt * 1.0e9)
        status = mission.update(self)
        while self.pending:
            callback, success = self.pending.pop(0)
            if callback is not None and self.answer:
                callback(centring_tests.stand_ins.FakeFuture(success=success))
        if self.fly and self.setpoints:
            east, north, up, _yaw = self.setpoints[-1]
            step = self.speed * dt
            delta = [east - self.position[0], north - self.position[1], up - self.position[2]]
            distance = math.sqrt(sum(d * d for d in delta))
            if distance > 1.0e-9:
                share = min(1.0, step / distance)
                self.position = [p + d * share for p, d in zip(self.position, delta)]
            pose = self.local_pose.pose.position
            pose.x, pose.y, pose.z = self.position
        return status

    def run(self, mission, seconds, until=None, dt=0.05):
        statuses = []
        for _ in range(int(round(seconds / dt))):
            statuses.append(self.tick(mission, dt))
            if until is not None and until(mission, statuses[-1]):
                break
        return statuses


def mission_for(context, features=ON, **config):
    spec = MissionSpec(type_name="search_pattern", name="search_pattern", config={**CONFIG, **config}, base_dir=Path("."), features=dict(features))
    mission = SearchPatternMission(spec)
    mission.on_enter(context)
    return mission


def surveying(**config):
    context = Context()
    mission = mission_for(context, **config)
    context.run(mission, 60.0, until=lambda m, _s: m.state == SearchState.SURVEY)
    assert mission.state == SearchState.SURVEY
    return context, mission


def test_the_flag_needs_the_height_estimator_and_with_it_off_the_mission_is_skipped():
    assert FLAG_REQUIRES[FLAG] == ("use_height_estimator",)
    context = Context()
    mission = mission_for(context, features={})
    assert context.tick(mission) == MissionStatus.SUCCESS
    assert context.setpoints == [] and context.speed_commands == [] and context.loc_calls == []
    assert any("skipped" in text and FLAG in text for text in context.infos)
    # With the flag off nothing in the config is looked at: a sequence can list the mission before it is configured.
    SearchPatternMission(MissionSpec(type_name="search_pattern", name="search_pattern", config={}, base_dir=Path("."), features={}))


def test_the_sequence_that_ships_declares_the_flag_off_and_builds():
    import yaml as real_yaml

    path = Path(__file__).resolve().parents[1] / "config" / "sequences" / "search_pattern_demo.yaml"
    root = real_yaml.safe_load(path.read_text())["mission_sequence"]
    assert root["features"][FLAG] is False and root["features"]["use_height_estimator"] is False
    entry = next(item for item in root["missions"] if item["type"] == "search_pattern")
    assert [item["type"] for item in root["missions"]] == ["takeoff", "search_pattern", "rtl"]
    SearchPatternMission(MissionSpec(type_name="search_pattern", name="search_pattern", config=entry["config"], base_dir=Path("."), features=dict(ON)))  # builds with the flag on


@pytest.mark.parametrize(
    "change, message",
    [
        ({"search_area": [corner(0, 0), corner(10, 0)]}, "search_area"),
        ({"search_area": "the field"}, "search_area"),
        ({"survey_height_m": 8.0}, "survey_height_m"),
        ({"survey_speed_mps": 6.0}, "4 m/s"),
        ({"survey_speed_mps": 0.0}, "survey_speed_mps"),
        ({"time_budget_s": 0.0}, "time_budget_s"),
        ({"camera_mode": "no_such_mode"}, "no_such_mode"),
        ({"flight_boundary": [corner(0, 0)]}, "flight_boundary"),
        ({"ground_below_home_m": "low"}, "ground_below_home_m"),
        ({"ground_below_home_m": float("nan")}, "ground_below_home_m"),
        ({"speed_after_mps": 0.0}, "speed_after_mps"),
        ({"speed_after_mps": 40.0}, "speed_after_mps"),  # 40 for 4.0
        ({"ground_below_home_m": 110.0}, "ground_below_home_m"),  # 110 for 1.10
        ({"ground_below_home_m": -25.0}, "ground_below_home_m"),
    ],
)
def test_a_configuration_that_cannot_be_flown_stops_the_sequence_being_built(change, message):
    with pytest.raises(ValueError, match=message):
        SearchPatternMission(MissionSpec(type_name="search_pattern", name="search_pattern", config={**CONFIG, **change}, base_dir=Path("."), features=dict(ON)))


def test_it_plans_the_tracks_climbs_to_the_survey_height_sets_the_speed_and_starts_the_pipeline_before_the_first_track():
    context = Context()
    mission = mission_for(context)
    context.run(mission, 60.0, until=lambda m, _s: m.state == SearchState.SURVEY)
    plan = mission.plan
    assert plan.tracks >= 2 and plan.height_m == pytest.approx(HEIGHT) and plan.speed_mps == 4.0
    # The same plan the planner gives for the area in the local frame, begun at the end nearest the vehicle.
    area = [(20.0, 10.0), (80.0, 10.0), (80.0, 50.0), (20.0, 50.0)]
    expected = coverage_planner.plan(area, HEIGHT, mission.camera, start=(0.0, 0.0))
    assert [tuple(round(v, 2) for v in point) for point in plan.waypoints] == [tuple(round(v, 2) for v in point) for point in expected.waypoints]
    first = plan.waypoints[0]
    assert math.dist(context.position[:2], first) < 1.5 and context.position[2] == pytest.approx(HEIGHT, abs=0.5)  # over the first track's start, at height
    assert context.speed_commands == [4.0] and context.loc_calls == [True]
    assert {round(yaw, 3) for _e, _n, _u, yaw in context.setpoints} == {round(mission.yaw_deg, 3)}  # one yaw throughout
    assert any("tracks" in text and "frames" in text for text in context.infos)


def test_the_tracks_are_flown_in_order_at_the_survey_height_and_the_mission_ends_cleanly_with_the_pipeline_stopped():
    context, mission = surveying()
    visited = []
    statuses = context.run(mission, 400.0, until=lambda m, s: s != MissionStatus.RUNNING)
    for east, north, up, _yaw in context.setpoints:
        if (round(east, 2), round(north, 2)) not in visited:
            visited.append((round(east, 2), round(north, 2)))
    wanted = [(round(e, 2), round(n, 2)) for e, n in mission.plan.waypoints]
    assert [point for point in visited if point in wanted] == wanted
    assert statuses[-1] == MissionStatus.SUCCESS and mission.state == SearchState.COMPLETE
    assert context.loc_calls == [True, False]
    ups = [up for _e, _n, up, _yaw in context.setpoints[-200:]]
    assert max(ups) - min(ups) < 0.2 and ups[-1] == pytest.approx(HEIGHT, abs=0.3)
    assert any("survey finished" in text and f"{mission.plan.tracks} of {mission.plan.tracks} tracks" in text for text in context.infos)


def test_the_height_is_above_the_ground_under_the_survey_not_above_home():
    context = Context()
    context.ground_up = 2.0  # the field is 2 m above home
    mission = mission_for(context)
    context.run(mission, 60.0, until=lambda m, _s: m.state == SearchState.SURVEY)
    assert context.position[2] == pytest.approx(2.0 + HEIGHT, abs=0.5)
    # On the barometer the survey is flown half a metre higher, so that the guard, which counts it lower, is content.
    context = Context()
    context.height_source = "baro"
    mission = mission_for(context)
    context.run(mission, 60.0, until=lambda m, _s: m.state == SearchState.SURVEY)
    assert context.position[2] == pytest.approx(HEIGHT + survey_height.BARO_MARGIN_M, abs=0.5)


def test_ground_that_comes_close_is_climbed_clear_of_and_the_track_goes_on_from_that_height():
    """The flight-safety review's finding: the first setpoint after a recovery came from the height before it."""
    context, mission = surveying()
    context.run(mission, 3.0)
    context.ground_up = 2.6  # the ground under the vehicle is 2.6 m higher: 8.07 m above it
    context.run(mission, 0.1)
    assert mission.recovering
    context.run(mission, 6.0, until=lambda m, _s: not m.recovering)
    assert not mission.recovering and context.position[2] == pytest.approx(2.6 + HEIGHT, abs=0.4)
    context.run(mission, 1.0)
    ups = [up for _e, _n, up, _yaw in context.setpoints[-20:]]
    assert min(ups) > 2.6 + HEIGHT - 0.4  # no setpoint back down toward the height that needed the recovery
    assert context.run(mission, 400.0, until=lambda m, s: s != MissionStatus.RUNNING)[-1] == MissionStatus.SUCCESS


def test_a_moment_on_the_barometer_does_not_pull_the_survey_down_to_home_level():
    """Seen in the sim: after a recovery over ground the rangefinder had found 2.7 m up, a second of barometer height
    (the vehicle tilted as it set off again) took the height setpoint back toward the survey height above home."""
    context, mission = surveying()
    context.ground_up = 2.6
    context.run(mission, 8.0)  # the recovery, and the track going on above the higher ground
    assert not mission.recovering and context.position[2] == pytest.approx(2.6 + HEIGHT, abs=0.4)
    context.ground_up, context.height_source = 0.0, "baro"  # the barometer's height: above home
    context.run(mission, 1.0)
    assert min(up for _e, _n, up, _yaw in context.setpoints[-20:]) > 2.6 + HEIGHT - 0.4  # the rangefinder's ground is kept
    # A rangefinder that stays away leaves the height to the barometer: its survey height, half a metre higher, above home.
    context.run(mission, 8.0)
    assert context.setpoints[-1][2] == pytest.approx(HEIGHT + survey_height.BARO_MARGIN_M, abs=0.2)


def test_the_height_is_asked_for_with_the_sequences_ground_offset():
    context, mission = surveying(ground_below_home_m=-1.5)
    context.run(mission, 2.0)
    assert context.height_asked_with == {-1.5}
    context, mission = surveying()
    assert context.height_asked_with == {0.0}


def test_the_speed_the_sequence_wants_after_the_survey_is_asked_for_once_when_it_is_over():
    context, mission = surveying(speed_after_mps=8.0)
    context.run(mission, 400.0, until=lambda m, s: s != MissionStatus.RUNNING)
    mission.on_exit(context)
    assert context.speed_commands == [4.0, 8.0]
    context, mission = surveying(speed_after_mps=8.0)  # left in mid-survey
    mission.on_exit(context)
    assert context.speed_commands == [4.0, 8.0]
    # A restore the flight controller refuses is said, not passed over.
    context, mission = surveying(speed_after_mps=8.0)
    context.speed_answers = False
    context.run(mission, 400.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert context.speed_commands == [4.0, 8.0] and any("refused" in text and "8 m/s" in text for text in context.warnings)
    context, mission = surveying()  # the sequence names none: the survey's speed is left
    context.run(mission, 400.0, until=lambda m, s: s != MissionStatus.RUNNING)
    mission.on_exit(context)
    assert context.speed_commands == [4.0]


def test_coordinates_the_wrong_way_round_and_a_way_out_of_the_boundary_fail_before_the_vehicle_moves():
    swapped = [[longitude, latitude] for latitude, longitude in AREA]
    context = Context()
    mission = mission_for(context, search_area=swapped)
    statuses = context.run(mission, 5.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert statuses[-1] == MissionStatus.FAILURE and context.setpoints == []
    assert any("from the vehicle" in text and "right way round" in text for text in context.warnings)
    # The area is inside the boundary, but a notch of it lies between the vehicle and the first track.
    notched = [corner(-10, -10), corner(120, -10), corner(120, 80), corner(12, 80), corner(12, -5), corner(8, -5), corner(8, 80), corner(-10, 80)]
    context = Context()
    mission = mission_for(context, flight_boundary=notched)
    statuses = context.run(mission, 5.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert statuses[-1] == MissionStatus.FAILURE and context.setpoints == []
    assert any("way to the first track leaves the flight boundary" in text for text in context.warnings)
    # With no boundary the survey is flown, and says that nothing was checked.
    context, mission = surveying()
    assert any("no flight_boundary" in text for text in context.warnings)


def test_a_dip_under_8_22_m_stops_the_track_climbs_back_and_goes_on():
    context, mission = surveying()
    context.run(mission, 3.0)
    track_setpoint = context.setpoints[-1]
    context.fly = False
    context.position[2] = 8.1
    context.local_pose.pose.position.z = 8.1
    context.tick(mission)
    east, north, up, _yaw = context.setpoints[-1]
    assert mission.state == SearchState.SURVEY and mission.recovering
    assert math.dist((east, north), context.position[:2]) < 0.01  # the survey waits where it is
    assert up == pytest.approx(HEIGHT, abs=0.05)  # and climbs back
    assert any("8.10 m" in text and "climbing back" in text for text in context.warnings)
    context.fly = True
    context.run(mission, 5.0)
    assert not mission.recovering and context.position[2] == pytest.approx(HEIGHT, abs=0.4)
    assert context.setpoints[-1][:2] == track_setpoint[:2] or mission.track_index >= 0  # on the track again
    assert context.run(mission, 400.0, until=lambda m, s: s != MissionStatus.RUNNING)[-1] == MissionStatus.SUCCESS


def test_a_recovery_keeps_its_climb_while_the_height_is_the_barometers_for_a_moment():
    """The sim's finding: the stop for a recovery tilts the vehicle, and the height is the barometer's meanwhile."""
    context, mission = surveying()
    context.run(mission, 3.0)
    context.fly = False
    context.position[2] = 8.1
    context.local_pose.pose.position.z = 8.1
    context.tick(mission)
    climb = context.setpoints[-1]
    assert mission.recovering and climb[2] == pytest.approx(HEIGHT, abs=0.05)
    context.ground_up = -2.6  # the barometer's height: above home, where the ground has not come close
    context.height_source = "baro"
    context.run(mission, 1.0)
    assert mission.recovering and context.setpoints[-1] == pytest.approx(climb)  # the same point and the same climb
    context.ground_up, context.height_source = 0.0, "rangefinder"
    context.fly = True
    context.run(mission, 5.0)
    assert not mission.recovering and context.position[2] == pytest.approx(HEIGHT, abs=0.4)


def test_a_dip_under_7_77_m_ends_the_survey_with_the_climb_first_and_the_pipeline_stopped():
    context, mission = surveying()
    context.run(mission, 3.0)
    context.fly = False
    context.position[2] = 7.5
    context.local_pose.pose.position.z = 7.5
    context.tick(mission)
    assert mission.state == SearchState.ENDING
    east, north, up, _yaw = context.setpoints[-1]
    assert math.dist((east, north), context.position[:2]) < 0.01 and up == pytest.approx(HEIGHT, abs=0.05)
    assert context.loc_calls == [True]  # not yet: the climb comes before anything else
    context.fly = True
    statuses = context.run(mission, 30.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert statuses[-1] == MissionStatus.SUCCESS and context.loc_calls == [True, False]
    assert context.position[2] == pytest.approx(HEIGHT, abs=0.5)
    assert any("survey ended early" in text and "7.50 m" in text for text in context.warnings)


def test_five_seconds_without_a_height_fail_the_mission_to_its_failure_policy():
    context, mission = surveying()
    context.run(mission, 2.0)
    context.no_height = True
    statuses = context.run(mission, 4.5)
    assert set(statuses) == {MissionStatus.RUNNING}
    hold = context.setpoints[-1]
    assert math.dist(hold[:2], context.position[:2]) < 0.5  # waiting where it is
    statuses = context.run(mission, 1.0)
    assert statuses[-1] == MissionStatus.FAILURE and context.loc_calls == [True, False]
    assert any("no height above the ground" in text for text in context.warnings)


def test_a_height_that_comes_and_goes_stops_the_survey_once_and_fails_it_after_five_seconds():
    context, mission = surveying()
    context.run(mission, 2.0)
    statuses, ticks = [], 0
    while len(statuses) < 140 and (not statuses or statuses[-1] == MissionStatus.RUNNING):  # up to 7 s
        context.no_height = ticks % 20 < 12  # no height for 0.6 s, then one for 0.4 s
        statuses.append(context.tick(mission))
        ticks += 1
    assert statuses[-1] == MissionStatus.FAILURE and 4.9 <= len(statuses) * 0.05 <= 5.3
    # (Which of the guard's two lines ends it depends on whether the fifth second falls on a tick with a height.)
    assert any("came and went" in text or "no height above the ground for 5 s" in text for text in context.warnings)
    holds = {(round(e, 3), round(n, 3)) for e, n, _u, _y in context.setpoints[-80:]}
    assert len(holds) == 1  # one place held throughout, not a stop and a start with every height
    assert sum("survey waiting" in text for text in context.warnings) <= 4  # and the log is not filled with it
    # A gap of 0.3 s is flown through: the track's waypoint stays the setpoint.
    context, mission = surveying()
    context.run(mission, 2.0)
    waypoint = context.setpoints[-1][:2]
    context.no_height = True
    context.run(mission, 0.3)
    context.no_height = False
    context.run(mission, 0.5)
    assert not mission.recovering and {point[:2] for point in context.setpoints[-16:]} == {waypoint}


def test_the_time_budget_also_ends_a_recovery_with_the_climb_it_had_asked_for():
    context, mission = surveying(time_budget_s=20.0)
    context.run(mission, 2.0)
    context.fly = False  # the vehicle cannot climb back
    context.position[2] = 8.1
    context.local_pose.pose.position.z = 8.1
    statuses = context.run(mission, 40.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert mission.state == SearchState.ENDING or statuses[-1] != MissionStatus.RUNNING
    assert any("time budget" in text and "climbing back" in text for text in context.warnings)
    climb = context.setpoints[-1]
    assert climb[2] == pytest.approx(HEIGHT, abs=0.05) and math.dist(climb[:2], context.position[:2]) < 0.01  # still the climb, where it is
    context.fly = True
    statuses = context.run(mission, 40.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert statuses[-1] == MissionStatus.SUCCESS and context.loc_calls == [True, False]


def test_the_guard_is_asked_only_from_the_first_track_so_a_climb_from_below_is_not_a_dip():
    context = Context()
    context.position[2] = 5.0  # the survey mission starts under both of the guard's lines
    context.local_pose.pose.position.z = 5.0
    mission = mission_for(context)
    context.run(mission, 60.0, until=lambda m, _s: m.state == SearchState.SURVEY)
    assert mission.state == SearchState.SURVEY and context.position[2] == pytest.approx(HEIGHT, abs=0.5)
    assert not any("survey ended early" in text or "survey waiting" in text for text in context.warnings)
    assert context.run(mission, 400.0, until=lambda m, s: s != MissionStatus.RUNNING)[-1] == MissionStatus.SUCCESS


def test_the_budget_is_600_s_unless_the_sequence_says_otherwise():
    import yaml as real_yaml

    from drone_mission_demo.missions import search_pattern_mission

    assert search_pattern_mission.DEFAULT_TIME_BUDGET_S == 600.0
    config = {key: value for key, value in CONFIG.items() if key != "time_budget_s"}
    context = Context()
    mission = SearchPatternMission(MissionSpec(type_name="search_pattern", name="search_pattern", config=config, base_dir=Path("."), features=dict(ON)))
    mission.on_enter(context)
    context.speed = 0.01  # a vehicle that goes nowhere: only the budget ends this
    context.run(mission, 599.0, dt=0.5)
    assert mission.state == SearchState.CLIMB
    statuses = context.run(mission, 3.0, dt=0.5)
    assert statuses[-1] == MissionStatus.SUCCESS and any("time budget of 600 s" in text for text in context.warnings)
    path = Path(__file__).resolve().parents[1] / "config" / "sequences" / "search_pattern_demo.yaml"
    entry = next(item for item in real_yaml.safe_load(path.read_text())["mission_sequence"]["missions"] if item["type"] == "search_pattern")
    assert entry["config"]["time_budget_s"] == 600.0


def test_the_yaw_held_is_the_plans_less_the_way_the_camera_is_mounted(monkeypatch):
    from drone_mission_core import frame_projection
    from drone_mission_demo.missions import search_pattern_mission

    # Where the image's top points in the body: forward on the sim's airframe, to the left on a camera turned a quarter turn.
    forward = SimpleNamespace(rotation_body_from_optical=((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0)))
    left = SimpleNamespace(rotation_body_from_optical=((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0)))
    assert search_pattern_mission.image_top_yaw_deg(forward) == pytest.approx(0.0)
    assert search_pattern_mission.image_top_yaw_deg(left) == pytest.approx(90.0)

    tall = [corner(20.0, 10.0), corner(60.0, 10.0), corner(60.0, 90.0), corner(20.0, 90.0)]  # 40 m east to west, 80 m north to south
    context = Context()
    mission = mission_for(context, search_area=tall)
    context.run(mission, 60.0, until=lambda m, _s: m.state == SearchState.SURVEY)
    assert abs(math.cos(math.radians(mission.plan.yaw_deg))) < 1.0e-6  # the tracks run north and south: the image's top along them
    assert {round(yaw, 3) for _e, _n, _u, yaw in context.setpoints} == {round(mission.plan.yaw_deg, 3)}  # forward-mounted: the plan's yaw as it is

    camera, _mount = frame_projection.camera_from_config(CONFIG, "search_pattern")
    monkeypatch.setattr(frame_projection, "camera_from_config", lambda config, name: (camera, left))
    context = Context()
    mission = mission_for(context, search_area=tall)
    context.run(mission, 60.0, until=lambda m, _s: m.state == SearchState.SURVEY)
    assert mission.yaw_deg == pytest.approx(mission.plan.yaw_deg - 90.0)  # the body is turned so that the image's top is still along the tracks
    assert {round(yaw, 3) for _e, _n, _u, yaw in context.setpoints} == {round(mission.plan.yaw_deg - 90.0, 3)}


def test_when_the_time_budget_runs_out_the_mission_ends_cleanly_with_what_it_has():
    context, mission = surveying(time_budget_s=30.0)
    statuses = context.run(mission, 60.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert statuses[-1] == MissionStatus.SUCCESS and context.loc_calls == [True, False]
    assert any("time budget" in text for text in context.warnings)
    assert mission.tracks_flown < mission.plan.tracks


def test_a_speed_command_that_is_refused_three_times_fails_the_mission_before_any_track():
    context = Context()
    context.speed_answers = False
    mission = mission_for(context)
    statuses = context.run(mission, 60.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert statuses[-1] == MissionStatus.FAILURE and len(context.speed_commands) == 3 and context.loc_calls == []
    assert any("survey speed" in text for text in context.warnings)


def test_a_speed_command_that_is_never_answered_is_asked_again_and_fails_the_mission_after_three():
    context = Context()
    context.answer = False
    mission = mission_for(context)
    statuses = context.run(mission, 60.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert statuses[-1] == MissionStatus.FAILURE and len(context.speed_commands) == 3
    assert any("did not answer" in text for text in context.warnings)


def test_a_pipeline_that_never_answers_its_start_is_not_waited_for_past_its_time():
    context = Context()
    mission = mission_for(context)
    context.run(mission, 60.0, until=lambda m, _s: m.state == SearchState.START_PIPELINE)
    context.answer = False
    context.run(mission, 10.0, until=lambda m, _s: m.state == SearchState.SURVEY)
    assert mission.state == SearchState.SURVEY and context.loc_calls == [True]
    assert any("did not answer" in text and "without the localisation pipeline" in text for text in context.warnings)
    # It was asked to start, so it is asked to stop at the end, whether or not it ever said it had started.
    context.answer = True
    context.run(mission, 400.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert context.loc_calls == [True, False]


def test_no_height_before_the_first_track_holds_and_then_fails_the_mission():
    context = Context()
    context.no_height = True
    mission = mission_for(context)
    statuses = context.run(mission, 4.5)
    assert set(statuses) == {MissionStatus.RUNNING} and math.dist(context.position[:2], (0.0, 0.0)) < 0.01  # held where it was
    statuses = context.run(mission, 1.5)
    assert statuses[-1] == MissionStatus.FAILURE and context.speed_commands == [] and context.loc_calls == []
    assert any("no height above the ground" in text for text in context.warnings)


def test_the_time_budget_also_ends_a_survey_that_never_reached_its_first_track():
    context = Context()
    context.speed = 0.2  # a vehicle that barely moves
    mission = mission_for(context, time_budget_s=20.0)
    statuses = context.run(mission, 40.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert statuses[-1] == MissionStatus.SUCCESS and mission.tracks_flown == 0 and context.loc_calls == []
    assert any("time budget" in text for text in context.warnings)


def test_leaving_the_mission_in_mid_survey_stops_the_pipeline_and_with_the_flag_off_touches_nothing():
    context, mission = surveying()
    context.run(mission, 3.0)
    mission.on_exit(context)
    assert context.loc_calls == [True, False] and context.cleared == 1
    mission.on_exit(context)  # (and it is not stopped twice)
    assert context.loc_calls == [True, False]
    context = Context()
    mission = mission_for(context, features={})
    context.tick(mission)
    mission.on_exit(context)
    assert context.cleared == 0 and context.loc_calls == []


def test_without_the_localisation_service_the_survey_is_flown_unless_the_sequence_requires_it():
    context = Context()
    context.loc_ready = False
    mission = mission_for(context)
    context.run(mission, 60.0, until=lambda m, _s: m.state == SearchState.SURVEY)
    assert mission.state == SearchState.SURVEY and context.loc_calls == []
    assert any("without the localisation pipeline" in text for text in context.warnings)
    context = Context()
    context.loc_ready = False
    mission = mission_for(context, localisation_required=True)
    statuses = context.run(mission, 60.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert statuses[-1] == MissionStatus.FAILURE


def test_a_search_area_that_cannot_be_flown_inside_the_flight_boundary_fails_before_take_off_is_wasted():
    notched = [corner(-10, -10), corner(120, -10), corner(120, 80), corner(52, 80), corner(52, 20), corner(48, 20), corner(48, 80), corner(-10, 80)]
    context = Context()
    mission = mission_for(context, flight_boundary=notched)
    statuses = context.run(mission, 5.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert statuses[-1] == MissionStatus.FAILURE and context.speed_commands == []
    assert any("flight boundary" in text for text in context.warnings)
    # A boundary the area is well inside: flown, and the plan says it was checked.
    context = Context()
    mission = mission_for(context, flight_boundary=[corner(-20, -20), corner(120, -20), corner(120, 90), corner(-20, 90)])
    context.run(mission, 60.0, until=lambda m, _s: m.state == SearchState.SURVEY)
    assert mission.plan.boundary_checked and mission.plan.legs_outside_boundary == 0


def test_every_path_logs_as_rclpy_allows():
    context, mission = surveying()
    context.logger = OneSeverityPerLine()
    context.run(mission, 3.0)
    context.fly = False
    context.position[2] = 8.1
    context.local_pose.pose.position.z = 8.1
    context.run(mission, 2.5)
    context.fly = True
    context.run(mission, 6.0)
    context.no_height = True
    context.run(mission, 2.5)
    context.no_height = False
    context.run(mission, 400.0, until=lambda m, s: s != MissionStatus.RUNNING)
    text = "\n".join(context.logger.lines)
    for expected in ("climbing back", "no height above the ground", "survey finished"):
        assert expected in text, expected


# --- LOC-4: the re-visit of uncertain markers after the tracks ---------------------------------------------

REVISIT_ON = {**ON, "survey_revisit": True}


def marker(track_id, east, north, digit=7, sightings=20, alive=True, outside=False):
    from drone_mission_core import revisit

    return revisit.TrackView(track_id, east, north, digit, sightings, alive, outside)


def revisiting(tracks, age_s=0.2, features=REVISIT_ON, **config):
    """A survey with the re-visit flag on and a tracker that has these tracks, flown to the end of its tracks' part."""
    context = Context()
    context.tracks, context.tracks_age_s = tracks, age_s
    context.marker_tracks = lambda: None if context.tracks is None else (list(context.tracks), context.tracks_age_s)
    mission = mission_for(context, features=features, **config)
    context.run(mission, 60.0, until=lambda m, _s: m.state == SearchState.SURVEY)
    return context, mission


def test_the_re_visit_flag_needs_the_survey_and_ships_off():
    from drone_mission_core import revisit

    assert FLAG_REQUIRES[revisit.FLAG] == (FLAG,)
    sequence = (Path(__file__).resolve().parents[1] / "config" / "sequences" / "search_pattern_demo.yaml").read_text()
    assert "survey_revisit: false" in sequence and "survey_revisit: true" not in sequence


def test_after_the_tracks_the_survey_hovers_three_seconds_over_each_uncertain_marker_with_the_pipeline_still_on():
    tracks = [marker(1, 30.0, 20.0), marker(2, 70.0, 40.0, digit=-1), marker(3, 40.0, 30.0, sightings=3), marker(4, 60.0, 15.0, digit=-1, outside=True)]
    context, mission = revisiting(tracks)
    stopped_at, hovers = [], {2: [], 3: []}

    def watch(m, status):
        if context.loc_calls[-1:] == [False] and not stopped_at:
            stopped_at.append(context._now_ns)
        for track_id, (east, north) in ((2, (70.0, 40.0)), (3, (40.0, 30.0))):
            if math.dist(context.position[:2], (east, north)) <= 1.0 and not stopped_at:
                hovers[track_id].append((context._now_ns * 1.0e-9, context.position[2]))
        return status != MissionStatus.RUNNING

    statuses = context.run(mission, 600.0, until=watch)
    assert statuses[-1] == MissionStatus.SUCCESS and mission.state == SearchState.COMPLETE
    said = " | ".join(context.infos)
    assert "survey finished: 3 of 3 tracks flown" in said and "re-visit: 2 track(s) to look at again" in said
    assert "hovered 3 s over track 2" in said and "hovered 3 s over track 3" in said and "re-visit round over: hovered over 2 of 2 track(s)" in said
    assert "track 1" not in said.split("re-visit:")[1].split("|")[0] and "track 4" not in said  # read and seen often; outside the field
    for track_id, seen in hovers.items():
        times = [t for t, _up in seen]
        assert times[-1] - times[0] >= 3.0 - 0.1, track_id  # three seconds within a metre of it
        assert all(abs(up - HEIGHT) < 0.5 for _t, up in seen), track_id  # at the survey height
    assert context.loc_calls == [True, False]  # the pipeline was on through the round and stopped once, after it
    assert said.index("hovered 3 s over track") < said.index("State: SURVEY -> ENDING")


def test_with_the_flag_off_or_nothing_uncertain_the_survey_ends_as_it_did():
    tracks = [marker(2, 70.0, 40.0, digit=-1)]
    context, mission = revisiting(tracks, features=ON)  # the tracker has an unread marker, and the flag is off
    statuses = context.run(mission, 400.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert statuses[-1] == MissionStatus.SUCCESS and not any("re-visit" in text for text in context.infos + context.warnings)
    off_points = [(round(e, 2), round(n, 2)) for e, n, _u, _y in context.setpoints]
    plain = Context()
    plain_mission = mission_for(plain)
    plain.run(plain_mission, 460.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert off_points == [(round(e, 2), round(n, 2)) for e, n, _u, _y in plain.setpoints]  # setpoint for setpoint the survey of LOC-3
    # The flag on and every marker read: no round, said once.
    context, mission = revisiting([marker(1, 30.0, 20.0)])
    statuses = context.run(mission, 400.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert statuses[-1] == MissionStatus.SUCCESS and any("no re-visit: no track to fly back to (1 track(s) heard)" in text for text in context.infos)


def test_tracks_that_are_not_there_or_are_old_mean_no_round_and_the_survey_still_ends_cleanly():
    for tracks, age_s, word in ((None, 0.0, "no re-visit: the tracker has published no tracks"), ([marker(2, 70.0, 40.0, digit=-1)], 30.0, "no re-visit: the tracker's tracks are 30 s old")):
        context, mission = revisiting(tracks, age_s=age_s)
        statuses = context.run(mission, 400.0, until=lambda m, s: s != MissionStatus.RUNNING)
        assert statuses[-1] == MissionStatus.SUCCESS and context.loc_calls == [True, False]
        assert any(word in text for text in context.warnings)


def test_the_round_keeps_to_the_time_budget_and_the_flight_boundary_and_the_height_guard_still_rules():
    # Little time left when the tracks are flown: the far marker is left out, and said.
    probe_context, probe = revisiting([])
    probe_context.run(probe, 400.0, until=lambda m, s: s != MissionStatus.RUNNING)
    tracks_s = probe_context.seconds_since(probe._started)
    near_end = probe.plan.waypoints[-1]
    close = marker(5, near_end[0] - 4.0, near_end[1], sightings=2)
    far = marker(6, near_end[0] - 55.0 if near_end[0] > 50 else near_end[0] + 55.0, 30.0, digit=-1)
    context, mission = revisiting([close, far], time_budget_s=tracks_s + 10.0 + 9.0)
    statuses = context.run(mission, 600.0, until=lambda m, s: s != MissionStatus.RUNNING)
    said = " | ".join(context.infos)
    assert statuses[-1] == MissionStatus.SUCCESS and "hovered 3 s over track 5" in said and "re-visit leaves out track 6 (digit not read (20 sightings)): no time for it" in said
    # A marker by the fence is not flown to.
    boundary = [corner(-10.0, -10.0), corner(82.0, -10.0), corner(82.0, 60.0), corner(-10.0, 60.0)]  # home is inside it; its east edge is 2 m past the area's
    context, mission = revisiting([marker(7, 79.0, 30.0, digit=-1)], flight_boundary=boundary)  # in the search area, and 3 m from the boundary
    context.run(mission, 400.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert any("re-visit leaves out track 7" in text and "not 5 m inside the flight boundary" in text for text in context.infos)
    assert max(east for east, _n, _u, _y in context.setpoints) < 78.0
    # The flight guardian's blocker: the shipped sequence has no flight boundary, and a track far outside the search area is still not flown to.
    context, mission = revisiting([marker(9, 900.0, -300.0, digit=-1), marker(10, 50.0, 30.0, sightings=1)])
    statuses = context.run(mission, 600.0, until=lambda m, s: s != MissionStatus.RUNNING)
    said = " | ".join(context.infos)
    assert statuses[-1] == MissionStatus.SUCCESS and "re-visit leaves out track 9 (digit not read (20 sightings)): 877 m outside the search area" in said and "hovered 3 s over track 10" in said
    assert max(east for east, _n, _u, _y in context.setpoints) < 82.0 and min(north for _e, north, _u, _y in context.setpoints) > -1.0
    # The height guard ends a round as it ends a track: low ground under the hover.
    context, mission = revisiting([marker(8, 50.0, 30.0, digit=-1)])
    context.run(mission, 400.0, until=lambda m, _s: any("re-visit 1 of 1" in text for text in context.infos))
    context.ground_up = HEIGHT - 7.0  # the ground comes up to 7 m under the vehicle: under the abort height
    statuses = context.run(mission, 120.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert any("survey ended early" in text for text in context.warnings) and context.loc_calls[-1] is False
    assert not any("hovered 3 s over track 8" in text for text in context.infos)


def test_a_tracker_that_goes_quiet_in_mid_round_ends_the_round_and_a_climb_back_does_not_cost_the_leg():
    context, mission = revisiting([marker(2, 70.0, 40.0, digit=-1), marker(3, 30.0, 20.0, digit=-1)])
    context.run(mission, 400.0, until=lambda m, _s: any("re-visit 1 of 2" in text for text in context.infos))
    context.tracks_age_s = 30.0  # the localisation node has stopped publishing
    statuses = context.run(mission, 120.0, until=lambda m, s: s != MissionStatus.RUNNING)
    assert statuses[-1] == MissionStatus.SUCCESS and context.loc_calls == [True, False]
    assert any("the tracker's tracks stopped during the re-visit (30 s old)" in text for text in context.warnings)
    assert not any("hovered 3 s over track 3" in text for text in context.infos) or any("hovered 3 s over track 2" in text for text in context.infos)
    # The height guard takes the vehicle off a leg (a dip under the survey's lower line): after the climb back the leg goes on and is not passed over.
    context, mission = revisiting([marker(8, 22.0, 12.0, digit=-1)])
    context.run(mission, 400.0, until=lambda m, _s: any("re-visit 1 of 1" in text for text in context.infos))
    context.ground_up = HEIGHT - 8.0  # the ground comes up to 8 m under the vehicle: the guard stops the track and climbs
    context.run(mission, 3.0)
    assert any("survey waiting" in text for text in context.warnings)  # the guard took the vehicle and climbed it back
    statuses = context.run(mission, 200.0, until=lambda m, s: s != MissionStatus.RUNNING)
    said = " | ".join(context.infos)
    assert statuses[-1] == MissionStatus.SUCCESS and "hovered 3 s over track 8" in said and "passed over" not in said
