"""OPS-02: the geofence demonstration mission, through its real update(), and its sequence file and profile.

rclpy and the message packages are the stand-ins from test_package_drop_mission.py; the vehicle is the step probe test's.
"""

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_step_probe_mission as probe_test  # noqa: E402  (installs the ROS stand-ins; its Context is a vehicle that follows velocity setpoints)

from drone_mission_core import fence_demo  # noqa: E402
from drone_mission_core.features import get_feature_flag, load_profile, resolve_features  # noqa: E402
from drone_mission_core.mission_api import MissionSpec, MissionStatus  # noqa: E402
from drone_mission_core.registry import create_mission  # noqa: E402
from drone_mission_demo.missions import fence_demo_mission as mission_module  # noqa: E402
from drone_mission_demo.missions.fence_demo_mission import FenceDemoMission  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
SEQUENCE = REPO / "src" / "drone_mission_demo" / "config" / "sequences" / "fence_demo.yaml"
LOADED = yaml.safe_load(SEQUENCE.read_text())["mission_sequence"]
CONFIG = next(item["config"] for item in LOADED["missions"] if item["type"] == "fence_demo")
HZ = probe_test.HZ


class Context(probe_test.Context):
    """The step probe test's vehicle, with a fence: at `fence_east_m` the flight controller takes it and flies it home."""

    def __init__(self, fence_east_m=None):
        super().__init__()
        self.fence_east_m = fence_east_m
        self.armed = True
        self.global_gps = SimpleNamespace(latitude=CONFIG["start_latitude"], longitude=CONFIG["start_longitude"])  # at the surveyed takeoff point
        self.infos = []
        self.logger = SimpleNamespace(info=self.infos.append, warn=self.warnings.append, error=self.errors.append)

    @property
    def mavros_state(self):
        return SimpleNamespace(connected=self.connected, armed=self.armed, mode=self.mode)

    @mavros_state.setter
    def mavros_state(self, _value):
        pass

    def tick(self, mission, drift_mps=0.0):
        status = super().tick(mission, drift_mps)
        if self.fence_east_m is not None and self.mode == "GUIDED" and self.east >= self.fence_east_m:
            self.mode = "RTL"  # the fence's action
        return status


def mission(on=True, **config):
    spec = MissionSpec(type_name="fence_demo", name="fence_demo", config={**CONFIG, **config}, base_dir=SEQUENCE.parent, features={"fence_demo": on})
    return FenceDemoMission(spec)


def started(context, **config):
    made = mission(**config)
    made.on_enter(context)
    return made


def run(context, made, seconds):
    status = None
    for _ in range(int(seconds * HZ)):
        status = context.tick(made)
        if status in (MissionStatus.SUCCESS, MissionStatus.FAILURE):
            break
    return status


def test_with_the_flag_off_the_mission_is_skipped_and_commands_nothing():
    context = Context()
    made = mission(on=False)
    made.on_enter(context)
    assert context.tick(made) == MissionStatus.SUCCESS
    assert context.local_velocity_commands == [] and context.position_holds == [] and context.cleared == 0 and context.modes_asked == []
    assert made.demo is None and not hasattr(made, "plan")
    # Off, even a config that could not be flown is not read: the 2026 path imports and checks nothing new.
    assert mission(on=False, bearing_deg=999.0).update(Context()) == MissionStatus.SUCCESS


def test_it_holds_then_flies_east_at_the_commanded_speed_until_the_fence_takes_the_vehicle():
    context = Context(fence_east_m=150.0)
    made = started(context)
    assert run(context, made, 2.9) == MissionStatus.RUNNING and context.east == 0.0
    assert all(command[:3] == (0.0, 0.0, 0.0) for command in context.local_velocity_commands)
    assert run(context, made, 10.0) == MissionStatus.RUNNING
    east, north, up, yaw = context.local_velocity_commands[-1]
    assert (east, up) == (pytest.approx(5.0), 0.0) and north == pytest.approx(0.0, abs=1e-9) and yaw == pytest.approx(30.0)  # bearing 90 is east; the heading is kept
    commands_before = None
    while context.mode == "GUIDED":
        commands_before = len(context.local_velocity_commands)
        assert context.tick(made) == MissionStatus.RUNNING
    assert 150.0 <= context.east < 150.3
    # From the tick the mode is not GUIDED: setpoints cleared once, nothing commanded, no mode asked for, no failure.
    assert run(context, made, 30.0) == MissionStatus.RUNNING
    assert len(context.local_velocity_commands) == commands_before + 1 and context.cleared == 1 and context.modes_asked == [] and context.errors == []
    assert len(context.warnings) == 1 and "the mode is RTL after 1" in context.warnings[0] and " m of the run" in context.warnings[0] and "the fence acted" in context.warnings[0]


def test_it_ends_when_the_vehicle_is_disarmed_after_the_fence_brought_it_home():
    context = Context(fence_east_m=20.0)
    made = started(context)
    assert run(context, made, 40.0) == MissionStatus.RUNNING and context.mode == "RTL"
    context.armed = False
    assert context.tick(made) == MissionStatus.SUCCESS
    assert any("over: the vehicle is disarmed, in RTL" in line for line in context.infos) and context.modes_asked == []


def test_a_fence_that_does_not_act_fails_the_run_at_its_length_holding():
    context = Context()  # no fence
    made = started(context, boundary_m=15.0, max_run_m=30.0)
    assert run(context, made, 60.0) == MissionStatus.FAILURE
    assert "the fence did not act" in context.errors[0] and "stopping:" in context.errors[0] and "failed:" in context.errors[-1]
    assert context.position_holds[-1] == pytest.approx((context.position_holds[-1][0], 0.0, 10.0, 30.0), abs=1e-6) and 30.0 <= context.position_holds[-1][0] < 30.3
    assert context.modes_asked == []  # the way home is the failure policy's (abort_and_rtl), not the mission's


def test_a_failure_is_a_commanded_stop_for_two_seconds_before_it_is_reported():
    """The flight-safety review: the executor clears a failed mission's setpoints, and the last velocity target would be flown on."""
    context = Context()
    made = started(context, boundary_m=15.0, max_run_m=30.0)
    status, ticks_stopping = None, 0
    for _ in range(60 * HZ):
        status = context.tick(made)
        if context.errors:
            ticks_stopping += 1
        if status == MissionStatus.FAILURE:
            break
    assert status == MissionStatus.FAILURE and ticks_stopping == pytest.approx(mission_module.STOP_S * HZ, abs=2)
    # While it stopped, the mission's setpoint was the position hold, tick after tick, and no velocity was commanded again.
    stop_index = next(i for i, command in enumerate(context.local_velocity_commands) if command[0] == pytest.approx(5.0))
    assert len(context.position_holds) == 1 and all(command[0] == pytest.approx(5.0) or command[0] == 0.0 for command in context.local_velocity_commands[stop_index:])


class Held(Context):
    """A vehicle the flight controller's avoidance stops 2 m inside the line, in GUIDED, as the sim's does; it takes a mode that is asked for."""

    stop_east_m = 148.0
    takes_modes = True

    def tick(self, mission, drift_mps=0.0):
        status = super().tick(mission, drift_mps)
        self.east = min(self.east, self.stop_east_m)
        if self.takes_modes and self.modes_asked and self.mode == "GUIDED":
            self.mode = self.modes_asked[-1]
        return status


def test_a_vehicle_the_flight_controller_holds_at_the_fence_is_the_demonstration_and_the_mission_asks_for_the_way_home():
    """A vehicle the flight controller keeps at the fence's margin through the whole push. Still ours, so RTL is ours to ask for."""
    context = Held()
    made = started(context)
    assert run(context, made, 3.0 + 30.0 + 20.0) == MissionStatus.RUNNING
    assert context.modes_asked == [] and context.local_velocity_commands[-1][0] == pytest.approx(5.0)  # 20 s at the fence: still pushing
    assert run(context, made, 120.0) == MissionStatus.RUNNING
    assert context.modes_asked == ["RTL"] and context.mode == "RTL" and context.errors == [] and context.warnings == []
    held = next(line for line in context.infos if "held at the fence" in line)
    assert "stopped 148.0 m into the run, 2.0 m short of the boundary, for 30 s" in held and "Asking for RTL" in held
    assert context.position_holds[-1] == pytest.approx((148.0, 0.0, 10.0, 30.0), abs=1e-6)  # held where it is while RTL is awaited
    assert any("on the way home after the hold at the fence" in line for line in context.infos) and context.cleared == 1
    commands = len(context.local_velocity_commands)
    assert run(context, made, 30.0) == MissionStatus.RUNNING and len(context.local_velocity_commands) == commands and context.modes_asked == ["RTL"]
    context.armed = False
    assert context.tick(made) == MissionStatus.SUCCESS


def test_a_vehicle_held_and_then_let_over_the_line_is_taken_by_the_fence_and_the_mission_goes_quiet():
    """What the sim shows: held at the margin for about 14 s with the velocity commanded, then over the line, and the fence's RTL."""

    class HeldThenOver(Context):
        def __init__(self):
            super().__init__(fence_east_m=150.0)
            self.held_since = None

        def tick(self, mission, drift_mps=0.0):
            before = self.east
            status = super().tick(mission, drift_mps)
            if self.mode == "GUIDED" and self.east >= 148.0 and (self.held_since is None or self._now_ns * 1e-9 - self.held_since < 14.0):
                self.held_since = self._now_ns * 1e-9 if self.held_since is None else self.held_since
                self.east = 148.0
            return status

    context = HeldThenOver()
    made = started(context)
    assert run(context, made, 120.0) == MissionStatus.RUNNING
    assert context.mode == "RTL" and context.modes_asked == [] and context.errors == []  # the fence's own action; nothing asked for
    assert "the fence acted" in context.warnings[0] and context.cleared == 1
    context.armed = False
    assert context.tick(made) == MissionStatus.SUCCESS


def test_a_hold_whose_rtl_is_not_taken_fails_so_that_the_failure_policy_asks_again():
    context = Held()
    context.takes_modes = False
    made = started(context)
    assert run(context, made, 120.0) == MissionStatus.FAILURE
    assert context.modes_asked == ["RTL"] and "did not take RTL within 5 s" in context.errors[0] and "failed:" in context.errors[-1]


def test_a_vehicle_stopped_anywhere_but_at_the_fence_fails_the_mission_holding():
    context = Held()
    context.stop_east_m = 60.0  # 90 m short of the boundary: not the fence
    made = started(context)
    assert run(context, made, 120.0) == MissionStatus.FAILURE
    assert "stopped 60.0 m into the run, 90.0 m from the boundary" in context.errors[0] and "it is not the fence that holds it" in context.errors[0]
    assert context.position_holds[-1][0] == pytest.approx(60.0) and context.modes_asked == [] and context.mode == "GUIDED"


def test_a_takeover_before_the_run_and_a_hand_back_do_not_start_it():
    context = Context(fence_east_m=150.0)
    made = started(context)
    run(context, made, 1.0)
    context.mode = "LOITER"  # the safety pilot, during the settle
    assert run(context, made, 5.0) == MissionStatus.RUNNING
    assert "before the run began" in context.warnings[0] and context.cleared == 1 and context.east == 0.0
    commands = len(context.local_velocity_commands)
    context.mode = "GUIDED"
    # Handed back in the air: stopped where it is, the run not started, and a failure so that the policy asks for the way home.
    assert run(context, made, 10.0) == MissionStatus.FAILURE
    assert len(context.local_velocity_commands) == commands and context.position_holds[-1][:2] == (0.0, 0.0) and "the run is not repeated" in context.errors[-1]
    assert context.modes_asked == []


def test_a_vehicle_the_pilot_landed_in_a_mode_of_their_own_is_left_alone():
    context = Context(fence_east_m=150.0)
    made = started(context)
    run(context, made, 5.0)
    context.mode = "LOITER"
    context.armed = False  # landed and disarmed by the pilot
    assert run(context, made, 20.0) == MissionStatus.RUNNING  # quiet: no success that would let a later mission ask for a mode
    assert context.modes_asked == [] and context.errors == []


def test_the_run_is_flown_only_from_the_surveyed_takeoff_point():
    """The flight-safety review: the run's lengths are from the surveyed point; launched 40 m nearer, the stop would be 40 m further out."""
    context = Context()
    context.global_gps = SimpleNamespace(latitude=CONFIG["start_latitude"], longitude=CONFIG["start_longitude"] + 40.0 / (111319.5 * 0.8273))  # 40 m east of it
    made = started(context)
    assert run(context, made, 20.0) == MissionStatus.FAILURE
    assert "the vehicle is 40 m from the takeoff point" in context.errors[0] and context.east == 0.0
    assert all(command[:3] == (0.0, 0.0, 0.0) for command in context.local_velocity_commands)
    near = Context()
    near.global_gps = SimpleNamespace(latitude=CONFIG["start_latitude"] + 5.0 / 111319.5, longitude=CONFIG["start_longitude"])  # 5 m north: within the 10
    made = started(near)
    assert run(near, made, 6.0) == MissionStatus.RUNNING and near.east > 5.0
    blind = Context()
    blind.global_gps = None
    made = started(blind)
    assert run(blind, made, 20.0) == MissionStatus.FAILURE and "no position from the flight controller" in blind.errors[0] and blind.east == 0.0


def test_no_pose_of_this_moment_flies_nothing_and_fails_in_time():
    context = Context()
    made = started(context)
    run(context, made, 4.0)  # running
    context.has_pose = False
    status = run(context, made, 10.0)
    assert status == MissionStatus.FAILURE and "no local pose of this moment for 5 s" in context.errors[0]
    assert context.local_velocity_commands[-1][:3] == (0.0, 0.0, 0.0)  # zero velocity while it waited
    stale = Context()
    made = started(stale)
    stale.pose_age_s = 2.0  # a pose, but an old one
    assert stale.tick(made) == MissionStatus.WAITING and stale.local_velocity_commands == []


def test_a_config_that_cannot_be_flown_is_refused_when_the_mission_is_made():
    with pytest.raises(ValueError, match="mission 'fence_demo': speed_mps must be a number from 0.5 to 6"):
        mission(speed_mps=8.0)
    spec = MissionSpec(type_name="fence_demo", name="fence_demo", config={"speed_mps": 2.0, "start_latitude": 34.175, "start_longitude": -118.4825}, base_dir=SEQUENCE.parent, features={"fence_demo": True})
    with pytest.raises(ValueError, match="bearing_deg and boundary_m and max_run_m must be given"):
        FenceDemoMission(spec)
    with pytest.raises(ValueError, match="start_latitude and start_longitude must be given"):
        FenceDemoMission(MissionSpec(type_name="fence_demo", name="fence_demo", config={key: value for key, value in CONFIG.items() if key != "start_latitude"}, base_dir=SEQUENCE.parent, features={"fence_demo": True}))
    with pytest.raises(ValueError, match="rtl_mode must be one of RTL, SMART_RTL"):
        mission(rtl_mode="LOITER")


def test_the_sequence_ships_with_the_flag_off_and_no_rtl_mission_after_the_demonstration():
    assert LOADED["features"]["fence_demo"] is False and all(value is False for value in LOADED["features"].values())
    assert [item["type"] for item in LOADED["missions"]] == ["takeoff", "fence_demo", "rtl"]  # the rtl is for the flag-off flight: after a demonstration the vehicle is down, in RTL
    assert LOADED["on_failure"] == "abort_and_rtl"
    plan_config = {key: value for key, value in CONFIG.items() if key not in ("rtl_mode", "start_latitude", "start_longitude")}
    assert fence_demo.plan_from_config(plan_config) == fence_demo.DemoPlan(bearing_deg=90.0, boundary_m=150.0, max_run_m=160.0, speed_mps=5.0, settle_s=3.0)
    assert CONFIG["rtl_mode"] == "RTL" and (CONFIG["start_latitude"], CONFIG["start_longitude"]) == (34.175, -118.4825)
    assert mission_module.FLAG == fence_demo.FLAG == "fence_demo"
    made = create_mission(MissionSpec(type_name="fence_demo", name="fence_demo", config=CONFIG, base_dir=SEQUENCE.parent, features=dict(LOADED["features"])))
    assert isinstance(made, FenceDemoMission) and made.update(Context()) == MissionStatus.SUCCESS  # as shipped: skipped


def test_the_placeholder_run_is_the_placeholder_sites_east_edge_plus_a_margin():
    sys.path.insert(0, str(REPO / "sim"))
    import site_file

    site = site_file.load_site(REPO / "sim" / "sites" / "apollo_xi.yaml")
    east_edge = max(corner[0] for corner in site.boundary)
    assert CONFIG["bearing_deg"] == 90.0 and CONFIG["boundary_m"] == pytest.approx(east_edge - site.takeoff[0], abs=0.01) and CONFIG["max_run_m"] == CONFIG["boundary_m"] + 10.0
    assert (CONFIG["start_latitude"], CONFIG["start_longitude"]) == (site.origin.latitude_deg, site.origin.longitude_deg) and site.takeoff == (0.0, 0.0)


def test_the_profile_turns_on_the_one_flag_and_fits_the_sequence():
    enabled = load_profile(REPO / "profiles" / "fence_demo.yaml")
    assert enabled == {"fence_demo": True}
    flown = resolve_features(dict(LOADED["features"]), enabled)
    assert get_feature_flag(flown, "fence_demo") is True and sum(value is True for value in flown.values()) == 1
