"""The stale-data guard in the drop and delivery missions (FND-03), through their real update().

With the `time_health_guard` flag on, an UNHEALTHY time base is "no target":
the mission holds, may climb to look again, and never descends. With the flag
off the missions do not look at the time base at all.

rclpy and the message packages are the stand-ins from
test_package_drop_mission.py, so this runs without ROS.
"""

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_package_drop_mission as stand_ins  # noqa: E402  (installs the ROS stand-ins, then imports the missions)

from drone_mission_core.mission_api import MissionSpec, MissionStatus  # noqa: E402
from drone_mission_core.time_health import TimeHealthReport  # noqa: E402
from drone_mission_demo.missions.package_delivery_mission import PackageDeliveryMission, PackageDeliveryState  # noqa: E402
from drone_mission_demo.missions.package_drop_mission import PackageDropMission, PackageDropState  # noqa: E402

FLAG = "time_health_guard"
HEALTHY = TimeHealthReport(True, (), 0.02, 0.05, 0.1, 5.0, 0.0, 2.0)
STALE_POSE = TimeHealthReport(False, ("pose is 400 ms old (limit 100 ms)",), 0.4, 0.05, 0.1, 5.0, 0.0, 2.0)


class Logger:
    def __init__(self) -> None:
        self.lines = []

    def info(self, text: str) -> None:
        self.lines.append(("info", text))

    def warn(self, text: str) -> None:
        self.lines.append(("warn", text))

    def error(self, text: str) -> None:
        self.lines.append(("error", text))


class Context(stand_ins.FakeContext):
    """The drop test's context, plus a detection in view, a landed state and a time-health report."""

    def __init__(self, report=HEALTHY, altitude_m: float = 10.0) -> None:
        super().__init__()
        self.logger = Logger()
        self.report = report
        self.time_health_calls = 0
        self.landed = False
        self.local_pose.pose.position.z = altitude_m
        self.see_target()

    def see_target(self) -> None:
        """A detection at the image centre, stamped now."""
        self.image_size = (640, 360)
        self.target_detection = SimpleNamespace(header=SimpleNamespace(stamp=self.now()), point=SimpleNamespace(x=0.0, y=0.0, z=900.0))

    def time_health(self):
        self.time_health_calls += 1
        return self.report

    def landing_state_available(self) -> bool:
        return True

    def vehicle_is_landed(self) -> bool:
        return self.landed

    def tick(self, mission, seconds: float = 0.05):
        self.advance(seconds)
        self.see_target()
        return mission.update(self)


class NoTimeHealthContext(Context):
    """A context from before FND-03: asking it for the time health is an error."""

    def time_health(self):
        raise AssertionError("the time base was consulted with the flag off")


DROP_CONFIG = {
    "target_latitude": 47.397742,
    "target_longitude": 8.545594,
    "target_loss_grace_s": 4.0,
    "not_found_ascent_m": 1.0,
    "max_recovery_attempts": 4,
    "drop_altitude_m": 4.0,
    "drop_altitude_tolerance_m": 0.5,
    "drop_column_handoff_altitude_m": 8.0,
    "drop_hover_dwell_s": 0.0,
    "descent_rate_mps": 0.5,
    "fake_drop": True,
}
DELIVERY_CONFIG = {
    "target_latitude": 47.397742,
    "target_longitude": 8.545594,
    "target_loss_grace_s": 4.0,
    "landing_check_threshold_m": 6.0,
    "final_descent_rate_mps": 0.25,
    "descent_rate_mps": 0.35,
    "fake_drop": True,
}


def make(kind, flag: bool, context, **config):
    mission_class, base = {"drop": (PackageDropMission, DROP_CONFIG), "delivery": (PackageDeliveryMission, DELIVERY_CONFIG)}[kind]
    spec = MissionSpec(type_name=f"package_{kind}", name=f"package_{kind}", config={**base, **config}, base_dir=Path("."), features={FLAG: flag})
    mission = mission_class(spec)
    mission.on_enter(context)
    context.see_target()  # on_enter clears the tracking state
    return mission


def track_state(kind):
    return PackageDropState.TRACK_AND_DESCEND if kind == "drop" else PackageDeliveryState.TRACK_AND_DESCEND


def column_state(kind):
    return PackageDropState.FINAL_FIXED_DROP_COLUMN if kind == "drop" else PackageDeliveryState.FINAL_FIXED_COLUMN_DESCENT


KINDS = ["drop", "delivery"]


# --- flag off: 2026 ------------------------------------------------------------------------


@pytest.mark.parametrize("kind", KINDS)
def test_flag_off_never_looks_at_the_time_base_and_descends_as_in_2026(kind):
    context = NoTimeHealthContext(report=STALE_POSE, altitude_m=12.0)
    mission = make(kind, False, context)
    mission._state = track_state(kind)
    for _ in range(5):
        assert context.tick(mission) == MissionStatus.RUNNING
    east, north, up, _yaw = context.local_velocity_commands[-1]
    assert up < 0.0  # centred on the target: it descends


@pytest.mark.parametrize("kind", KINDS)
def test_flag_off_column_descends_whatever_the_time_base_says(kind):
    context = NoTimeHealthContext(report=STALE_POSE, altitude_m=7.5)
    mission = make(kind, False, context)
    mission._state = column_state(kind)
    context.tick(mission)
    first = context.global_position_commands[-1][2]
    context.tick(mission, 1.0)
    assert context.global_position_commands[-1][2] < first


@pytest.mark.parametrize("kind", KINDS)
def test_flag_missing_from_the_sequence_is_off(kind):
    mission_class = PackageDropMission if kind == "drop" else PackageDeliveryMission
    base = DROP_CONFIG if kind == "drop" else DELIVERY_CONFIG
    context = NoTimeHealthContext(report=STALE_POSE, altitude_m=12.0)
    mission = mission_class(MissionSpec(type_name=f"package_{kind}", name=kind, config=dict(base), base_dir=Path(".")))
    mission.on_enter(context)
    mission._state = track_state(kind)
    context.tick(mission)
    assert context.local_velocity_commands[-1][2] < 0.0


# --- flag on, healthy: flies as with the flag off ------------------------------------------------------


@pytest.mark.parametrize("kind", KINDS)
def test_flag_on_and_healthy_commands_exactly_what_flag_off_commands(kind):
    commands = {}
    for flag in (False, True):
        context = Context(report=HEALTHY, altitude_m=12.0)
        mission = make(kind, flag, context)
        mission._state = track_state(kind)
        for _ in range(10):
            context.tick(mission)
        commands[flag] = (list(context.local_velocity_commands), list(context.global_position_commands))
    assert commands[True] == commands[False]
    assert commands[True][0][-1][2] < 0.0


# --- flag on, unhealthy: no target, hold, never descend -------------------------------------------


@pytest.mark.parametrize("kind", KINDS)
def test_unhealthy_while_tracking_holds_then_climbs_and_never_descends(kind):
    context = Context(report=HEALTHY, altitude_m=12.0)
    mission = make(kind, True, context)
    mission._state = track_state(kind)
    context.tick(mission)
    assert context.local_velocity_commands[-1][2] < 0.0  # healthy and centred: descending

    context.report = STALE_POSE
    start = len(context.local_velocity_commands)
    for _ in range(40):  # 2 s, inside the 4 s grace
        assert context.tick(mission) == MissionStatus.RUNNING
    held = context.local_velocity_commands[start:]
    assert held and all(command[:3] == (0.0, 0.0, 0.0) for command in held)
    assert mission._state == track_state(kind)
    assert any(level == "warn" and "time base" in text and "pose is 400 ms old" in text for level, text in context.logger.lines)

    for _ in range(60):  # past the grace: the 2026 recovery takes over, which climbs
        context.tick(mission)
    assert mission._state.name in ("TARGET_NOT_FOUND", "ACQUIRE_TARGET")
    assert all(command[2] >= 0.0 for command in context.local_velocity_commands[start:])
    climbs = [command[2] for command in context.global_position_commands]
    assert climbs and min(climbs) >= 12.0  # every position setpoint is at or above where it was


@pytest.mark.parametrize("kind", KINDS)
def test_unhealthy_never_acquires_a_target_it_can_see(kind):
    context = Context(report=STALE_POSE, altitude_m=12.0)
    mission = make(kind, True, context)
    mission._state = PackageDropState.ACQUIRE_TARGET if kind == "drop" else PackageDeliveryState.ACQUIRE_TARGET
    for _ in range(40):
        context.tick(mission)
    assert mission._state.name == "ACQUIRE_TARGET"
    context.report = HEALTHY
    for _ in range(60):  # 1 s for the guard to trust the time base again, 1 s holding the lock
        context.tick(mission)
    assert mission._state == track_state(kind)


@pytest.mark.parametrize("kind", KINDS)
def test_tracking_resumes_when_the_time_base_recovers(kind):
    context = Context(report=STALE_POSE, altitude_m=12.0)
    mission = make(kind, True, context)
    mission._state = track_state(kind)
    for _ in range(10):
        context.tick(mission)
    assert context.local_velocity_commands[-1][:3] == (0.0, 0.0, 0.0)
    context.report = HEALTHY
    for _ in range(19):  # healthy for 0.95 s: not yet trusted
        context.tick(mission)
        assert context.local_velocity_commands[-1][:3] == (0.0, 0.0, 0.0)
    context.tick(mission)
    context.tick(mission)
    assert context.local_velocity_commands[-1][2] < 0.0
    assert any(level == "info" and "time base healthy again" in text for level, text in context.logger.lines)


@pytest.mark.parametrize("kind", KINDS)
def test_a_time_base_that_flickers_is_one_long_fault(kind):
    context = Context(report=HEALTHY, altitude_m=12.0)
    mission = make(kind, True, context)
    mission._state = track_state(kind)
    context.tick(mission)
    start = len(context.local_velocity_commands)
    for index in range(60):  # 3 s: unhealthy on one tick in four
        context.report = STALE_POSE if index % 4 == 0 else HEALTHY
        context.tick(mission)
    held = context.local_velocity_commands[start:]
    assert held and all(command[:3] == (0.0, 0.0, 0.0) for command in held)
    warnings = [text for level, text in context.logger.lines if level == "warn" and "time base UNHEALTHY" in text]
    assert len(warnings) == 1  # said once, not once per flicker


@pytest.mark.parametrize("kind", KINDS)
def test_unhealthy_in_the_fixed_column_stops_the_descent_and_healthy_resumes_it(kind):
    context = Context(report=HEALTHY, altitude_m=7.5)
    mission = make(kind, True, context)
    mission._state = column_state(kind)
    context.tick(mission)
    context.tick(mission, 1.0)
    before = context.global_position_commands[-1]
    assert before[2] < 7.5  # descending in the column

    context.report = STALE_POSE
    start = len(context.global_position_commands)
    for _ in range(40):
        assert context.tick(mission, 0.1) == MissionStatus.RUNNING
    held = context.global_position_commands[start:]
    assert len(held) == 40
    # The column's own point, at the vehicle's height: not the lower setpoint
    # the vehicle was following, which would let the descent go on.
    assert all(command[:2] == before[:2] and command[2] == 7.5 for command in held)
    assert mission._state == column_state(kind)

    context.report = HEALTHY
    for _ in range(9):  # 0.9 s healthy: still holding
        context.tick(mission, 0.1)
    assert context.global_position_commands[-1][2] == 7.5
    context.tick(mission, 0.1)
    context.tick(mission, 0.1)
    context.tick(mission, 1.0)
    after = context.global_position_commands[-1]
    assert after[2] < before[2]
    assert before[2] - after[2] <= 0.5 * 1.1 + 1e-9  # it resumes at the descent rate, with no catch-up step


@pytest.mark.parametrize("kind", KINDS)
def test_a_flickering_time_base_in_the_column_never_descends_and_fails_after_the_timeout(kind):
    context = Context(report=HEALTHY, altitude_m=7.5)
    mission = make(kind, True, context, time_guard_hold_timeout_s=3.0)
    mission._state = column_state(kind)
    context.tick(mission)
    start = len(context.global_position_commands)
    statuses = []
    for index in range(40):  # 4 s; healthy on every other tick
        context.report = STALE_POSE if index % 2 == 0 else HEALTHY
        statuses.append(context.tick(mission, 0.1))
        if statuses[-1] == MissionStatus.FAILURE:
            break
    assert statuses[-1] == MissionStatus.FAILURE and 29 <= len(statuses) <= 32
    assert all(command[2] == 7.5 for command in context.global_position_commands[start:])


@pytest.mark.parametrize("kind", KINDS)
def test_unhealthy_on_entering_the_column_holds_where_it_is(kind):
    context = Context(report=STALE_POSE, altitude_m=7.5)
    mission = make(kind, True, context)
    mission._state = column_state(kind)
    for _ in range(10):
        context.tick(mission, 0.5)
    heights = [command[2] for command in context.global_position_commands]
    assert heights and all(height == 7.5 for height in heights)


@pytest.mark.parametrize("kind", KINDS)
def test_a_column_hold_that_does_not_clear_fails_the_mission_after_the_timeout(kind):
    context = Context(report=STALE_POSE, altitude_m=7.5)
    mission = make(kind, True, context, time_guard_hold_timeout_s=3.0)
    mission._state = column_state(kind)
    statuses = [context.tick(mission, 0.5) for _ in range(8)]
    assert statuses[:5] == [MissionStatus.RUNNING] * 5  # up to 2.5 s
    assert statuses[-1] == MissionStatus.FAILURE
    assert any(level == "error" and "time base" in text for level, text in context.logger.lines)
    assert all(command[2] == 7.5 for command in context.global_position_commands)


@pytest.mark.parametrize("kind", KINDS)
def test_a_time_base_that_stays_unhealthy_ends_the_recovery_climb_with_a_failure(kind):
    # A dead pose stream: the height never changes, so the 2026 recovery would wait on its climb for ever.
    context = Context(report=STALE_POSE, altitude_m=12.0)
    mission = make(kind, True, context, time_guard_hold_timeout_s=6.0, recovery_alt_tolerance_m=0.5)
    mission._state = track_state(kind)
    statuses = [context.tick(mission, 0.1) for _ in range(70)]
    assert MissionStatus.FAILURE in statuses
    failed_at = statuses.index(MissionStatus.FAILURE)
    assert 59 <= failed_at <= 62  # 4 s of grace, then the climb, given up 6 s after the fault began
    assert mission._state.name == "TARGET_NOT_FOUND"
    assert all(command[2] >= 0.0 for command in context.local_velocity_commands)
    assert all(command[2] >= 12.0 for command in context.global_position_commands)


@pytest.mark.parametrize("kind", KINDS)
def test_flag_off_recovery_waits_on_its_climb_as_in_2026(kind):
    context = NoTimeHealthContext(report=STALE_POSE, altitude_m=12.0)
    mission = make(kind, False, context, time_guard_hold_timeout_s=1.0)
    mission._state = PackageDropState.TARGET_NOT_FOUND if kind == "drop" else PackageDeliveryState.TARGET_NOT_FOUND
    statuses = [context.tick(mission, 0.1) for _ in range(100)]
    assert set(statuses) == {MissionStatus.RUNNING}


def test_the_hold_timeout_defaults_to_ten_seconds():
    context = Context(report=STALE_POSE, altitude_m=7.5)
    mission = make("drop", True, context)
    mission._state = column_state("drop")
    statuses = [context.tick(mission, 0.5) for _ in range(22)]
    assert statuses[18] == MissionStatus.RUNNING  # 9.5 s
    assert statuses[-1] == MissionStatus.FAILURE


def test_delivery_already_on_the_ground_finishes_its_touchdown_while_unhealthy():
    context = Context(report=STALE_POSE, altitude_m=0.05)
    mission = make("delivery", True, context)
    mission._state = PackageDeliveryState.FINAL_FIXED_COLUMN_DESCENT
    context.landed = True
    for _ in range(30):
        context.tick(mission)
    assert mission._state == PackageDeliveryState.GROUND_DWELL


@pytest.mark.parametrize("kind", KINDS)
def test_flag_on_without_a_monitor_is_unhealthy(kind):
    context = Context(report=None, altitude_m=12.0)  # the executor has no monitor to ask
    mission = make(kind, True, context)
    mission._state = track_state(kind)
    for _ in range(5):
        context.tick(mission)
    assert all(command[:3] == (0.0, 0.0, 0.0) for command in context.local_velocity_commands)
