"""PLT-05: the command step probe's rule (drone_mission_core/step_probe.py). No ROS."""

import math

import pytest

from drone_mission_core import step_probe
from drone_mission_core.step_probe import StepPlan, StepProbe


def fly(plan, hz=20, drift=None, seconds=120.0):
    """Tick the probe as the executor would; the vehicle follows each velocity command at once. Returns the commands."""
    probe = StepProbe(plan)
    east = north = 0.0
    commands = []
    for tick in range(int(seconds * hz)):
        t = 100.0 + tick / hz
        command = probe.update(t, east, north)
        commands.append((t, command))
        if command.done or command.failed:
            break
        east += command.east_mps / hz + (drift or 0.0) / hz
        north += command.north_mps / hz
    return commands


def test_the_flag_has_one_name():
    assert step_probe.FLAG == "command_step_probe"


def test_it_rests_steps_and_rests_again_for_each_step():
    plan = StepPlan(steps=4, step_speed_mps=1.0, step_s=1.0, rest_s=3.0)
    commands = fly(plan)
    assert commands[-1][1].done and not commands[-1][1].failed
    speeds = [command.east_mps for _, command in commands]
    # Edges of the command: from rest to a step and back, four times.
    rises = [i for i in range(1, len(speeds)) if speeds[i - 1] == 0.0 and speeds[i] != 0.0]
    falls = [i for i in range(1, len(speeds)) if speeds[i - 1] != 0.0 and speeds[i] == 0.0]
    assert len(rises) == 4 and len(falls) == 4
    assert [speeds[i] for i in rises] == [1.0, -1.0, 1.0, -1.0]  # there and back, so the vehicle ends where it began
    t = [time for time, _ in commands]
    assert t[rises[0]] - t[0] == pytest.approx(3.0, abs=0.06)  # steady for the whole rest before the first step
    for rise, fall in zip(rises, falls):
        assert t[fall] - t[rise] == pytest.approx(1.0, abs=0.06)
    for fall, rise in zip(falls, rises[1:]):
        assert t[rise] - t[fall] == pytest.approx(3.0, abs=0.06)
    assert t[-1] - t[falls[-1]] == pytest.approx(3.0, abs=0.06)  # and a rest after the last one
    assert all(command.north_mps == 0.0 for _, command in commands)


def test_the_direction_is_the_plans():
    commands = fly(StepPlan(steps=1, heading_deg=90.0))
    stepping = [command for _, command in commands if command.east_mps or command.north_mps]
    assert stepping and all(abs(c.east_mps) < 1e-9 and c.north_mps == pytest.approx(1.0) for c in stepping)


def test_a_vehicle_that_wanders_from_where_it_started_ends_the_probe():
    commands = fly(StepPlan(steps=6, max_drift_m=5.0), drift=1.0)
    last = commands[-1][1]
    assert last.failed and not last.done
    assert "5.0 m" in last.reason
    assert last.east_mps == 0.0 and last.north_mps == 0.0
    assert commands[-1][0] - commands[0][0] < 8.0


def test_done_stays_done_and_commands_nothing():
    probe = StepProbe(StepPlan(steps=1, rest_s=2.0, step_s=0.5))
    for tick in range(200):
        command = probe.update(tick * 0.05, 0.0, 0.0)
    assert command.done and command.east_mps == 0.0 and command.north_mps == 0.0


@pytest.mark.parametrize(
    "config, word",
    [
        ({"steps": 0}, "steps"),
        ({"steps": 21}, "steps"),
        ({"step_speed_mps": 0.1}, "step_speed_mps"),
        ({"step_speed_mps": 3.0}, "step_speed_mps"),
        ({"step_s": 0.1}, "step_s"),
        ({"rest_s": 1.0}, "rest_s"),
        ({"step_speed_mps": 2.0, "step_s": 2.0}, "3 m"),
        ({"max_drift_m": 50.0}, "max_drift_m"),
        ({"steps": True}, "steps"),
        ({"step_s": "long"}, "step_s"),
        ({"speed": 1.0}, "speed"),
    ],
)
def test_a_plan_that_should_not_be_flown_is_refused(config, word):
    with pytest.raises(ValueError, match=word):
        step_probe.plan_from_config(config)


def test_the_defaults_are_a_plan():
    plan = step_probe.plan_from_config({})
    assert plan == StepPlan()
    assert plan.steps == 6 and plan.step_speed_mps == 1.0 and plan.step_s == 1.0 and plan.rest_s == 3.0 and plan.max_drift_m == 10.0
    assert math.isclose(step_probe.duration_s(plan), 3.0 + 6 * 4.0)


@pytest.mark.parametrize("jump_s", [-5.0, -0.2, 1.5, 30.0])
def test_a_clock_that_steps_ends_the_probe_instead_of_repeating_or_running_on(jump_s):
    probe = StepProbe(StepPlan(steps=6))
    t = 100.0
    for _ in range(90):  # 4.5 s in: the first step is over
        assert not probe.update(t, 0.0, 0.0).failed
        t += 0.05
    command = probe.update(t + jump_s, 0.0, 0.0)
    assert command.failed and not command.done and "clock" in command.reason
    assert command.east_mps == 0.0 and command.north_mps == 0.0
    assert probe.update(t + jump_s + 0.05, 0.0, 0.0) is command  # and stays ended


def test_it_counts_the_steps_it_began():
    probe = StepProbe(StepPlan(steps=3))
    t = 0.0
    while not probe.update(t, 0.0, 0.0).done:
        t += 0.05
    assert probe.steps_begun == 3
