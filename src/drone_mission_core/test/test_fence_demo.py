"""OPS-02: the geofence demonstration's rule (drone_mission_core/fence_demo.py). No ROS."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_mission_core import fence_demo  # noqa: E402
from drone_mission_core.fence_demo import DemoPlan, FenceDemo  # noqa: E402

PLAN = DemoPlan(bearing_deg=90.0, boundary_m=30.0, max_run_m=40.0, speed_mps=2.0, settle_s=3.0)


def fly(demo, seconds, start_s=0.0, east=0.0, north=0.0, follow=True, dt=0.05):
    """Tick the rule at 20 Hz with a vehicle that does what it is told (or stays put); the last command and where it ended."""
    t, command = start_s, None
    for _ in range(int(round(seconds / dt))):
        command = demo.update(t, east, north)
        if command.failed:
            break
        if follow:
            east, north = east + command.east_mps * dt, north + command.north_mps * dt
        t += dt
    return command, east, north, t


def test_it_holds_still_and_then_runs_along_the_compass_bearing():
    demo = FenceDemo(PLAN)
    command, east, north, t = fly(demo, 2.9)
    assert (command.east_mps, command.north_mps, command.running) == (0.0, 0.0, False) and east == 0.0
    command, east, north, t = fly(demo, 5.0, start_s=t)
    assert command.running and command.east_mps == pytest.approx(2.0) and command.north_mps == pytest.approx(0.0, abs=1e-9)  # 90 degrees is east
    assert east == pytest.approx(2.0 * 4.9, abs=0.2) and demo.run_m == pytest.approx(east, abs=0.2)
    for bearing, (east_mps, north_mps) in ((0.0, (0.0, 2.0)), (180.0, (0.0, -2.0)), (270.0, (-2.0, 0.0)), (45.0, (math.sqrt(2.0), math.sqrt(2.0)))):
        command, *_ = fly(FenceDemo(DemoPlan(bearing_deg=bearing, boundary_m=30.0, max_run_m=40.0, speed_mps=2.0)), 4.0)
        assert (command.east_mps, command.north_mps) == pytest.approx((east_mps, north_mps), abs=1e-9), bearing


def test_a_run_that_reaches_its_length_with_no_fence_fails_and_commands_nothing_more():
    demo = FenceDemo(PLAN)
    command, east, north, t = fly(demo, 60.0)
    assert command.failed and "the fence did not act" in command.reason and (command.east_mps, command.north_mps) == (0.0, 0.0)
    assert 40.0 <= east < 40.2 and t == pytest.approx(3.0 + 20.0, abs=0.2)
    assert demo.update(t + 0.05, east, north) is command  # and it stays ended


def test_the_run_is_measured_along_the_bearing_from_where_it_began():
    demo = FenceDemo(DemoPlan(bearing_deg=0.0, boundary_m=5.0, max_run_m=10.0, settle_s=1.0))
    fly(demo, 1.0, east=50.0, north=-20.0, follow=False)
    assert not demo.update(1.05, 50.0, -20.0).failed
    assert not demo.update(1.10, 58.0, -11.0).failed and demo.run_m == pytest.approx(9.0)  # 8 m sideways is not ground gained
    assert demo.update(1.15, 50.0, -10.0).failed


def test_a_vehicle_that_leaves_the_runs_line_sideways_ends_the_run():
    """The flight-safety review: the aircraft's avoidance slides along a fence it meets at an angle, and the run would go on gaining ground."""
    demo = FenceDemo(DemoPlan(bearing_deg=90.0, boundary_m=30.0, max_run_m=40.0, speed_mps=2.0, settle_s=1.0))
    fly(demo, 1.0, follow=False)
    assert not demo.update(1.05, 0.0, 0.0).failed
    assert not demo.update(1.10, 5.0, 9.9).failed
    command = demo.update(1.15, 6.0, -10.5)
    assert command.failed and "10.5 m to the side of the run's line" in command.reason and "along something" in command.reason


def held_at(demo, stop_m, seconds=120.0, cross_after_s=None):
    """A vehicle that follows until `stop_m` and is stopped there; with `cross_after_s`, it goes on again that long after stopping."""
    t, east, command, stopped_at = 0.0, 0.0, None, None
    commands = []
    while t < seconds:
        command = demo.update(t, east, 0.0)
        commands.append(command)
        if command.failed or command.held:
            break
        if stopped_at is None and east >= stop_m:
            stopped_at = t
        free = stopped_at is None or (cross_after_s is not None and t - stopped_at >= cross_after_s)
        east = east + command.east_mps * 0.05 if free else stop_m
        t += 0.05
    return command, east, t, commands, stopped_at


def test_at_the_fence_the_velocity_stays_commanded_and_a_vehicle_still_there_after_the_push_is_held():
    """Found in the sim: the flight controller's avoidance stops the vehicle 2 m inside the line, under command."""
    demo = FenceDemo(PLAN)  # the boundary 30 m away
    command, east, t, commands, stopped_at = held_at(demo, 28.0)
    assert command.held and not command.failed and (command.east_mps, command.north_mps) == (0.0, 0.0)
    assert "stopped 28.0 m into the run, 2.0 m short of the boundary, for 30 s with 2.0 m/s still commanded" in command.reason
    assert t - stopped_at == pytest.approx(fence_demo.HOLD_PUSH_S, abs=0.5)
    assert all(c.east_mps == pytest.approx(2.0) for c in commands[int((stopped_at + 0.1) / 0.05) : -1])  # pushed all the while
    assert demo.update(t + 0.05, 28.0, 0.0) is command  # the run is over


def test_a_vehicle_that_goes_over_during_the_push_is_still_being_commanded_when_it_does():
    """Found in the sim: after about 14 s at the margin the vehicle crosses, and the fence's own action takes it (the mission sees the mode)."""
    demo = FenceDemo(PLAN)
    command, east, t, commands, stopped_at = held_at(demo, 28.0, seconds=3.0 + 14.0 + 14.0 + 1.5, cross_after_s=14.0)
    assert not command.held and not command.failed and command.east_mps == pytest.approx(2.0)
    assert east > 30.0 and demo.at_fence_since is None  # over the line, gaining ground, the push forgotten


@pytest.mark.parametrize("stop_m, held", [(21.9, False), (22.1, True), (29.9, True), (30.9, True), (31.2, False), (10.0, False), (0.0, False)])
def test_only_a_stop_at_the_boundary_is_the_fences(stop_m, held):
    """Within 8 m short of where the boundary is said to be, or a metre past it. Anywhere else something other than the fence stopped it."""
    demo = FenceDemo(PLAN)
    command, east, t, commands, stopped_at = held_at(demo, stop_m)
    assert command.held is held and command.failed is (not held)
    if not held:
        assert "it is not the fence that holds it" in command.reason and t - max(stopped_at, PLAN.settle_s) == pytest.approx(fence_demo.STALL_S, abs=0.5)  # said in 5 s, not 30


def test_a_vehicle_that_crawls_ends_the_run_on_time():
    demo = FenceDemo(PLAN)
    t, east, command = 0.0, 0.0, None
    while t < 200.0:
        command = demo.update(t, east, 0.0)
        if command.failed:
            break
        east += 0.3 * 0.05 if command.running else 0.0  # 0.3 m/s where 2 was asked: gaining ground, far too slowly
        t += 0.05
    assert command.failed and "the vehicle is not following" in command.reason
    assert t == pytest.approx(3.0 + fence_demo.run_timeout_s(PLAN), abs=0.2) and fence_demo.run_timeout_s(PLAN) == 40.0 + fence_demo.HOLD_PUSH_S


@pytest.mark.parametrize("jump_s", [1.5, -0.5])
def test_a_clock_that_steps_ends_the_run(jump_s):
    demo = FenceDemo(PLAN)
    _, east, north, t = fly(demo, 5.0)
    command = demo.update(t + jump_s, east, north)
    assert command.failed and "the clock moved" in command.reason and command.running


def test_the_direction_and_the_length_must_be_given_and_every_number_is_bounded():
    plan = fence_demo.plan_from_config({"bearing_deg": 90, "boundary_m": 150.0, "max_run_m": 165.0})
    assert plan == DemoPlan(bearing_deg=90.0, boundary_m=150.0, max_run_m=165.0, speed_mps=5.0, settle_s=3.0)
    good = {"bearing_deg": 90.0, "boundary_m": 85.0, "max_run_m": 100.0}
    for config, word in (
        ({"boundary_m": 85.0, "max_run_m": 100.0}, "bearing_deg must be given"),
        ({"bearing_deg": 90.0, "boundary_m": 85.0}, "max_run_m must be given"),
        ({}, "bearing_deg and boundary_m and max_run_m must be given"),
        ({**good, "speed_mps": 8.0}, "speed_mps must be a number from 0.5 to 6"),
        ({**good, "bearing_deg": 400.0}, "bearing_deg must be a number from 0 to 360"),
        ({**good, "max_run_m": 900.0}, "max_run_m must be a number from 15 to 500"),
        ({**good, "max_run_m": True}, "max_run_m must be a number"),
        ({**good, "max_run_m": 80.0}, "max_run_m .80. must be past boundary_m .85."),
        ({**good, "height_m": 10.0}, "unknown setting 'height_m'"),
    ):
        with pytest.raises(ValueError, match=word):
            fence_demo.plan_from_config(config)
