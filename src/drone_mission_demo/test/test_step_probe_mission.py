"""PLT-05: the command step probe mission, through its real update(), and its sequence file.

rclpy and the message packages are the stand-ins from test_package_drop_mission.py.
"""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_package_drop_mission as stand_ins  # noqa: E402  (installs the ROS stand-ins)

from drone_mission_core import step_probe  # noqa: E402
from drone_mission_core.features import apply_feature_overrides, get_feature_flag  # noqa: E402
from drone_mission_core.mission_api import MissionSpec, MissionStatus  # noqa: E402
from drone_mission_core.registry import create_mission  # noqa: E402
from drone_mission_demo.missions import step_probe_mission as mission_module  # noqa: E402
from drone_mission_demo.missions.step_probe_mission import CommandStepProbeMission  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
SEQUENCE = REPO / "src" / "drone_mission_demo" / "config" / "sequences" / "command_step_probe.yaml"
LOADED = yaml.safe_load(SEQUENCE.read_text())["mission_sequence"]
CONFIG = next(item["config"] for item in LOADED["missions"] if item["type"] == "command_step_probe")
HZ = 20


class Context(stand_ins.FakeContext):
    """A vehicle in GUIDED at 10 m that moves as its velocity setpoint says. It faces 30 degrees from east, so a
    heading the mission kept can be told from the context's default of 90."""

    yaw_deg = 30.0

    def __init__(self):
        super().__init__()
        self._now_ns = 500 * 1_000_000_000
        self.east = self.north = 0.0
        self.mode = "GUIDED"
        self.position_holds = []
        self.cleared = 0
        self.has_pose = True
        self.errors, self.warnings = [], []
        self.logger = SimpleNamespace(info=lambda _line: None, warn=self.warnings.append, error=self.errors.append)
        self.connected = True
        self.pose_age_s = 0.02
        self.modes_asked = []

    @property
    def mavros_state(self):
        return SimpleNamespace(connected=self.connected, armed=True, mode=self.mode)

    @mavros_state.setter
    def mavros_state(self, _value):
        pass

    @property
    def local_pose(self):
        if not self.has_pose:
            return None
        half = math.radians(self.yaw_deg) / 2.0
        stamp_ns = self._now_ns - int(self.pose_age_s * 1e9)
        header = SimpleNamespace(stamp=SimpleNamespace(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000))
        return SimpleNamespace(header=header, pose=SimpleNamespace(position=SimpleNamespace(x=self.east, y=self.north, z=10.0), orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))))

    @local_pose.setter
    def local_pose(self, _value):
        pass

    def set_local_position_setpoint(self, east, north, up, yaw_deg=90.0):
        self.position_holds.append((east, north, up, yaw_deg))

    def clear_all_setpoints(self):
        self.cleared += 1

    def request_mode_change(self, mode, done_callback=None):
        self.modes_asked.append(mode)

    def tick(self, mission, drift_mps=0.0):
        status = mission.update(self)
        if self.local_velocity_commands and status == MissionStatus.RUNNING:
            east, north, _up, _yaw = self.local_velocity_commands[-1]
            self.east += (east + drift_mps) / HZ
            self.north += north / HZ
        self.advance(1.0 / HZ)
        return status


def mission(on=True, **config):
    spec = MissionSpec(type_name="command_step_probe", name="command_step_probe", config={**CONFIG, **config}, base_dir=SEQUENCE.parent, features={"command_step_probe": on})
    return CommandStepProbeMission(spec)


def fly(probe, context, limit_s=120.0, **tick):
    probe.on_enter(context)
    for _ in range(int(limit_s * HZ)):
        status = context.tick(probe, **tick)
        if status != MissionStatus.RUNNING:
            return status
    return status


def test_the_flag_names_agree_and_the_mission_is_registered():
    assert mission_module.FLAG == step_probe.FLAG
    spec = MissionSpec(type_name="command_step_probe", name="probe", config=dict(CONFIG), base_dir=SEQUENCE.parent, features={})
    assert isinstance(create_mission(spec), CommandStepProbeMission)


def test_with_the_flag_off_the_mission_is_skipped_and_commands_nothing():
    context = Context()
    probe = mission(on=False)
    probe.on_enter(context)
    assert probe.update(context) == MissionStatus.SUCCESS
    probe.on_exit(context)
    assert context.local_velocity_commands == [] and context.position_holds == [] and context.cleared == 0
    assert probe.probe is None
    assert "drone_mission_core.step_probe" not in vars(mission_module)  # the flag-off path brings nothing new in


def test_the_shipped_sequence_has_the_flag_false_and_nothing_else_new():
    assert LOADED["features"]["command_step_probe"] is False
    assert set(LOADED["features"].values()) == {False}  # and so is every other flag it declares
    assert get_feature_flag(LOADED["features"], "command_step_probe") is False
    assert [item["type"] for item in LOADED["missions"]] == ["takeoff", "command_step_probe", "rtl"]
    assert step_probe.plan_from_config(CONFIG) == step_probe.StepPlan()


def test_with_the_flag_on_it_commands_the_steps_and_ends_holding_where_it_is():
    context = Context()
    status = fly(mission(), context)
    assert status == MissionStatus.SUCCESS
    speeds = [east for east, _north, _up, _yaw in context.local_velocity_commands]
    rises = [i for i in range(1, len(speeds)) if speeds[i - 1] == 0.0 and speeds[i] != 0.0]
    assert [speeds[i] for i in rises] == [1.0, -1.0, 1.0, -1.0, 1.0, -1.0]
    assert rises[0] == pytest.approx(3.0 * HZ, abs=1)  # three seconds of zero velocity first
    assert all(up == 0.0 and yaw == pytest.approx(30.0) for _e, _n, up, yaw in context.local_velocity_commands)  # level, heading kept
    assert len(context.position_holds) == 1
    east, north, up, yaw = context.position_holds[0]
    assert math.hypot(east, north) < 0.2 and up == 10.0 and yaw == pytest.approx(30.0)


def test_the_executors_override_turns_it_on():
    specs = [MissionSpec(type_name=item["type"], name=item["name"], config=item.get("config", {}), base_dir=SEQUENCE.parent, features=dict(LOADED["features"])) for item in LOADED["missions"]]
    on = apply_feature_overrides(specs, "command_step_probe")
    probe = CommandStepProbeMission(next(spec for spec in on if spec.type_name == "command_step_probe"))
    assert probe.probe is None and probe.plan.steps == 6
    assert fly(probe, Context()) == MissionStatus.SUCCESS


def test_a_mode_taken_by_the_pilot_or_a_failsafe_is_not_taken_back():
    """The flight-safety review's blocker: FAILURE here would have had abort_and_rtl ask for RTL over the pilot."""
    assert LOADED["on_failure"] == "abort_and_rtl"  # which is why the mission must not fail on a takeover
    context = Context()
    probe = mission()
    probe.on_enter(context)
    for _ in range(5 * HZ):
        assert context.tick(probe) == MissionStatus.RUNNING
    commanded = len(context.local_velocity_commands)
    context.mode = "LOITER"  # the safety pilot's switch
    for _ in range(30 * HZ):
        assert context.tick(probe) == MissionStatus.RUNNING  # not FAILURE: the executor's failure policy never runs
    assert len(context.local_velocity_commands) == commanded and context.position_holds == []
    assert context.cleared == 1 and context.modes_asked == []
    assert any("taken by the pilot or a failsafe" in line for line in context.warnings)


def test_handed_back_in_guided_the_probe_is_over_and_the_vehicle_is_held():
    context = Context()
    probe = mission()
    probe.on_enter(context)
    for _ in range(5 * HZ):
        context.tick(probe)
    context.mode = "LAND"
    for _ in range(2 * HZ):
        context.tick(probe)
    context.east, context.north = 4.0, -2.0  # where the pilot left it
    commanded = len(context.local_velocity_commands)
    context.mode = "GUIDED"
    context.yaw_deg = 140.0  # and facing another way
    assert context.tick(probe) == MissionStatus.SUCCESS
    assert len(context.local_velocity_commands) == commanded  # no step under whoever just let go
    assert context.position_holds == [(4.0, -2.0, 10.0, pytest.approx(140.0))]  # held as it stands now: no turn
    assert context.modes_asked == []
    assert any("1 of 6 steps" in line for line in context.warnings)


@pytest.mark.parametrize("when", ["before the first pose", "with a stale pose"])
def test_handed_back_without_a_pose_of_this_moment_nothing_is_commanded(when):
    """The second review's concern: a zero-velocity hold here went out at the context's default heading, a turn to north."""
    context = Context()
    probe = mission()
    probe.on_enter(context)
    if when == "before the first pose":
        context.mode = "STABILIZE"
        context.tick(probe)
    else:
        for _ in range(5 * HZ):
            context.tick(probe)
        context.mode = "STABILIZE"
        context.tick(probe)
        context.pose_age_s = 2.0
    commanded = len(context.local_velocity_commands)
    context.mode = "GUIDED"
    if when == "before the first pose":
        assert context.tick(probe) == MissionStatus.SUCCESS
        assert context.position_holds == [(0.0, 0.0, 10.0, pytest.approx(30.0))]  # the heading is the pose's own, not 90
    else:
        assert context.tick(probe) == MissionStatus.SUCCESS
        assert context.position_holds == []
    assert len(context.local_velocity_commands) == commanded and context.modes_asked == []


def test_a_pose_that_comes_back_after_a_gap_ends_the_probe_and_says_why():
    context = Context()
    probe = mission()
    probe.on_enter(context)
    for _ in range(int(3.5 * HZ)):
        context.tick(probe)
    context.has_pose = False
    for _ in range(2 * HZ):
        assert context.tick(probe) == MissionStatus.WAITING
    context.has_pose = True
    assert context.tick(probe) == MissionStatus.FAILURE
    assert any("no local pose of this moment for 2." in line for line in context.errors)
    assert len(context.position_holds) == 1 and context.position_holds[0][3] == pytest.approx(30.0)


@pytest.mark.parametrize("fault", ["stale", "disconnected", "unstamped"])
def test_a_pose_that_is_not_of_this_moment_stops_the_steps(fault):
    context = Context()
    probe = mission()
    probe.on_enter(context)
    for _ in range(int(3.5 * HZ)):  # into the first step
        assert context.tick(probe) == MissionStatus.RUNNING
    assert context.local_velocity_commands[-1][0] == 1.0
    if fault == "stale":
        context.pose_age_s = 0.8
    elif fault == "disconnected":
        context.connected = False
    else:
        context.has_pose = False
    assert context.tick(probe) == MissionStatus.WAITING
    assert context.local_velocity_commands[-1][:3] == (0.0, 0.0, 0.0)  # the step is dropped at once
    for _ in range(5 * HZ):
        context.pose_age_s += 1.0 / HZ if fault == "stale" else 0.0
        status = context.tick(probe)
    assert status == MissionStatus.FAILURE
    assert context.position_holds == []  # no hold at a place it cannot see
    assert all(command[:3] == (0.0, 0.0, 0.0) for command in context.local_velocity_commands[int(3.5 * HZ) :])


def test_a_clock_that_steps_ends_it_holding():
    context = Context()
    probe = mission()
    probe.on_enter(context)
    for _ in range(4 * HZ):
        context.tick(probe)
    context._now_ns -= 5 * 1_000_000_000
    assert context.tick(probe) == MissionStatus.FAILURE
    assert len(context.position_holds) == 1 and any("clock" in line for line in context.errors)


def test_a_vehicle_carried_away_ends_it_holding():
    context = Context()
    status = fly(mission(max_drift_m=3.0), context, drift_mps=1.5)
    assert status == MissionStatus.FAILURE
    assert len(context.position_holds) == 1
    assert any("3.0 m" in line for line in context.errors)


def test_no_pose_from_the_start_waits_and_then_fails_having_commanded_nothing():
    context = Context()
    context.has_pose = False
    probe = mission()
    probe.on_enter(context)
    assert context.tick(probe) == MissionStatus.WAITING
    context.advance(6.0)
    assert context.tick(probe) == MissionStatus.FAILURE
    assert context.local_velocity_commands == [] and context.position_holds == []


def test_a_plan_that_should_not_be_flown_stops_the_mission_being_built():
    with pytest.raises(ValueError, match="command_step_probe.*step_speed_mps"):
        mission(step_speed_mps=5.0)
    mission(on=False, step_speed_mps=5.0)  # with the flag off the config is not read
