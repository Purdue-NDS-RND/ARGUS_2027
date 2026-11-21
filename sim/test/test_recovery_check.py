"""The recovery check's flight (REC-8), with a vehicle that is not MAVLink."""

import json
import sys
from pathlib import Path
from types import SimpleNamespace

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import recovery_check as tool  # noqa: E402


class Vehicle:
    """A point that flies at 2 m/s toward where it is told, 20 steps a second."""

    def __init__(self, simulator=True, stuck_below=None, disarm_at=None):
        self.simulator, self.stuck_below, self.disarm_at = simulator, stuck_below, disarm_at
        self.t, self.e, self.n, self.u = 0.0, 0.0, 0.0, 0.0
        self.target, self.armed, self.grips, self.landing = None, False, [], False

    def is_simulator(self):
        return self.simulator

    def is_armed(self):
        return self.armed

    def now(self):
        return self.t

    def takeoff(self, height_m):
        self.armed, self.u = True, height_m

    def goto(self, e, n, u):
        self.target = (e, n, u)

    def gripper(self, grab):
        self.grips.append((round(self.t, 2), grab, round(self.e, 2), round(self.n, 2), round(self.u, 2)))

    def land(self):
        self.landing = True

    def step(self):
        self.t += 0.05
        if self.disarm_at is not None and self.t >= self.disarm_at:
            self.armed = False
        if self.landing:
            self.u = max(0.0, self.u - 0.05)
            self.armed = self.armed and self.u > 0.0
        elif self.target is not None:
            for axis, goal in zip(("e", "n", "u"), self.target):
                here = getattr(self, axis)
                if axis == "u" and self.stuck_below is not None:
                    goal = max(goal, self.stuck_below)
                setattr(self, axis, here + max(-0.1, min(0.1, goal - here)))
        return SimpleNamespace(t=self.t, e=self.e, n=self.n, u=self.u, mode="GUIDED")


PLAN = dict(bucket_en=(12.0, 8.0), return_en=(0.0, 0.0), grab_height_m=0.675)


def test_the_flight_grabs_at_the_handle_lifts_carries_sets_down_releases_and_lands():
    vehicle = Vehicle()
    report = tool.run_check(vehicle, **PLAN)
    assert report["passed"] and report["landed"] and report["problem"] == ""
    assert [step["step"] for step in report["steps"]] == [
        "over the bucket", "hook at the handle", "grab commanded", "lifted", "over the return point", "bucket just off the ground", "release commanded", "clear above", "clear aside",
    ]  # fmt: skip
    (_t_grab, grab, e, n, u), (_t_release, release, e2, n2, u2) = vehicle.grips
    # The grab is commanded over the bucket with the hook at the handle; the release over the return point, the bucket 5 cm up.
    assert grab is True and (e, n) == (12.0, 8.0) and abs(u - 0.675) <= tool.LEVEL_M
    assert release is False and (e2, n2) == (0.0, 0.0) and abs(u2 - (0.675 + tool.SET_DOWN_CLEARANCE_M)) <= tool.LEVEL_M
    lifted = next(step for step in report["steps"] if step["step"] == "lifted")
    assert lifted["u"] > 2.9  # well over the judge's 0.5 m, and over the rules' release ceiling only on the way
    assert u2 < 1.83  # the release itself is far under 1.83 m
    assert report["steps"][-1]["e"] == tool.CLEAR_ASIDE_M  # it lands beside the bucket, not on it
    # With the bucket on the hook the vehicle is walked, not sent: 14.4 m to the return point at 1 m/s takes 14 s
    # and more, where the way out to the bucket (this vehicle flies 2 m/s) took about 7.
    times = {step["step"]: step["t"] for step in report["steps"]}
    assert times["over the return point"] - times["lifted"] >= 14.4 / tool.CARRY_SPEED_MPS and times["over the bucket"] < 12.0
    assert tool.CARRY_SPEED_MPS == 1.0
    json.dumps(report)


def test_it_flies_in_the_sim_only_and_says_what_stopped_it():
    real = tool.run_check(Vehicle(simulator=False), **PLAN)
    assert not real["passed"] and "sim only" in real["problem"] and real["steps"] == []
    # A vehicle that cannot come down to the handle: the step times out, no grab is commanded, and it still lands.
    high = Vehicle(stuck_below=2.0)
    report = tool.run_check(high, step_timeout_s=20.0, **PLAN)
    assert not report["passed"] and "'hook at the handle' was not reached" in report["problem"] and high.grips == [] and report["landed"]
    # A vehicle that disarms in the air (a failsafe, a crash): said.
    fallen = tool.run_check(Vehicle(disarm_at=3.0), **PLAN)
    assert not fallen["passed"] and "disarmed on the way to" in fallen["problem"]


def test_the_plan_comes_from_the_built_run(tmp_path):
    from scenario.build import build_run

    run = build_run(SIM / "scenarios" / "recovery" / "hook_check.yaml", tmp_path / "run", variant="on")
    plan = tool.plan_from(run)
    assert plan["bucket_en"] == (12.0, 8.0) and plan["return_en"] == (0.0, 0.0)
    # The handle is 0.32 m up and the hook's tip 0.55 m under the vehicle's origin.
    assert abs(plan["grab_height_m"] - (0.32 + 0.55)) < 1.0e-9
    # A run with no bucket, or a vehicle with no hook, is said.
    truth = json.loads((run / "truth.json").read_text())
    del truth["bucket"]
    (run / "truth.json").write_text(json.dumps(truth))
    try:
        tool.plan_from(run)
        raise AssertionError("a run with no bucket was planned")
    except ValueError as error:
        assert "no bucket" in str(error)
    assert tool.main(["--site", str(SIM / "sites" / "apollo_xi.yaml"), "--run-dir", str(run)]) == 2


# --- the flight-safety review's points ---


def test_a_position_that_has_stopped_coming_is_not_counted_as_being_anywhere():
    """A vehicle whose position freezes while it is at the handle must not be taken to be still there: no grab is
    commanded on a position two seconds old, the step runs out of time, and the vehicle lands."""

    class Frozen(Vehicle):
        fresh = True

        def step(self):
            sample = super().step()
            at_handle = self.target is not None and abs(self.u - PLAN["grab_height_m"]) < 0.01 and abs(self.e - 12.0) < 0.01
            self.fresh = self.landing or not at_handle  # nothing new once it is at the handle
            return sample

    frozen = Frozen()
    report = tool.run_check(frozen, step_timeout_s=15.0, **PLAN)
    assert not report["passed"] and "'hook at the handle' was not reached" in report["problem"] and frozen.grips == [] and report["landed"]


def test_the_bucket_goes_to_the_judges_return_point(tmp_path):
    from scenario.build import build_run

    run = build_run(SIM / "scenarios" / "recovery" / "hook_check.yaml", tmp_path / "run", variant="on")
    command = json.loads((run / "scenario.json").read_text())["driver"]["command"]
    assert command[command.index("--return-point") + 1] == "takeoff"  # the recovery judge's default
    truth = json.loads((run / "truth.json").read_text())
    rose = truth["return_points"]["compass_rose"]
    assert tool.plan_from(run, "compass_rose")["return_en"] == (rose["e"], rose["n"]) != tool.plan_from(run)["return_en"]
    try:
        tool.plan_from(run, "the moon")
        raise AssertionError("an unknown return point was planned")
    except ValueError as error:
        assert "no return point 'the moon'" in str(error)
    # A scenario that has the judge score against the compass rose has the tool carry the bucket there.
    other = tmp_path / "rose.yaml"
    other.write_text((SIM / "scenarios" / "recovery" / "hook_check.yaml").read_text() + "judge:\n  return_point: compass_rose\n")
    command = json.loads((build_run(other, tmp_path / "rose", variant="on") / "scenario.json").read_text())["driver"]["command"]
    assert command[command.index("--return-point") + 1] == "compass_rose"
