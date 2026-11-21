"""The recovery check's flight (REC-8), with a vehicle that is not MAVLink."""

import json
import math
import sys

import pytest
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


# --- REC-4 in the check flight: the attach check and the shaped carry ---


class Loaded(Vehicle):
    """The stand-in vehicle with a throttle: 0.40 in the empty hover, more once the gripper has closed (the bucket on)."""

    def __init__(self, loaded_throttle=0.50, **settings):
        super().__init__(**settings)
        self.loaded_throttle = loaded_throttle

    def throttle(self):
        holding = bool(self.grips) and self.grips[-1][1] is True
        return self.loaded_throttle if holding else 0.40


def test_the_check_flight_runs_the_attach_check_on_its_own_hover_throttle():
    from drone_mission_core import recovery_grip

    attach = recovery_grip.AttachCheck(vehicle_kg=1.755, load_kg=1.0)
    report = tool.run_check(Loaded(), attach=attach, **PLAN)
    assert report["passed"] and report["attach"]["attached"] is True
    assert report["attach"]["throttle_ratio"] == 1.25 and report["attach"]["expected_ratio"] == round((2.755 / 1.755) ** 0.5, 4)
    assert (report["attach"]["baseline_throttle"], report["attach"]["loaded_throttle"]) == (0.40, 0.50)
    # A hook that came up empty: the throttle did not rise, the check says so, and the flight ends there (no carry, a landing).
    empty = Loaded(loaded_throttle=0.405)
    missed = tool.run_check(empty, attach=recovery_grip.AttachCheck(1.755, 1.0), **PLAN)
    assert not missed["passed"] and "not on the hook" in missed["problem"] and missed["attach"]["attached"] is False and missed["landed"]
    assert [step["step"] for step in missed["steps"]][-1] == "lifted" and len(empty.grips) == 1  # no release was commanded: nothing to release
    # A check that was asked for and could not be made is not a pass: a vehicle with no throttle to give,
    unheard = tool.run_check(Vehicle(), attach=recovery_grip.AttachCheck(1.755, 1.0), **PLAN)
    assert not unheard["passed"] and "could not be made" in unheard["problem"] and unheard["landed"]
    # and one whose throttle stopped coming: an old reading is not a sample of either hover.
    stopped = Loaded()
    stopped.throttle_age_s = lambda: 3.0
    unjudged = tool.run_check(stopped, attach=recovery_grip.AttachCheck(1.755, 1.0), **PLAN)
    assert not unjudged["passed"] and "could not be made" in unjudged["problem"] and unjudged["attach"]["baseline_throttle"] is None
    # Without the check asked for, the flight is REC-8's: no hovers for it, nothing judged.
    assert tool.run_check(Vehicle(), **PLAN)["passed"] and "attach" not in tool.run_check(Loaded(), **PLAN)


def test_the_shaped_carry_starts_and_stops_over_a_pendulum_period_at_up_to_the_carry_speed():
    from drone_mission_core import recovery_grip

    period = recovery_grip.pendulum_period_s(0.72)
    report = tool.run_check(Vehicle(), carry_speed_mps=3.0, pendulum_m=0.72, **PLAN)
    assert report["passed"] and report["shaped"] is True and report["carry_speed_mps"] == 3.0
    times = {step["step"]: step["t"] for step in report["steps"]}
    carry = times["over the return point"] - times["lifted"]
    leg = recovery_grip.ShapedLeg(14.42, 3.0, 0.72)
    assert leg.duration_s <= carry <= leg.duration_s + 8.0 and leg.duration_s == pytest.approx(2.0 * period + (14.42 - 3.0 * period) / 3.0, abs=0.05)
    # Unshaped, the flight is as it was: the point walked at 1 m/s.
    # A carry speed that is no speed is said before takeoff, not found in the air with the bucket on the hook.
    for bad in (0.0, -1.0, float("nan"), None):
        grounded = Vehicle()
        refused = tool.run_check(grounded, carry_speed_mps=bad, pendulum_m=0.72, **PLAN)
        assert not refused["passed"] and "carry's speed" in refused["problem"] and refused["steps"] == [] and grounded.grips == []
    plain = tool.run_check(Vehicle(), **PLAN)
    assert plain["shaped"] is False and plain["carry_speed_mps"] == tool.CARRY_SPEED_MPS == 1.0


def test_the_plan_gives_the_masses_and_the_pendulums_length(tmp_path):
    from scenario.build import build_run

    run = build_run(SIM / "scenarios" / "recovery" / "hook_carry_shaped.yaml", tmp_path / "run", variant="on")
    plan = tool.plan_from(run)
    assert (plan["vehicle_kg"], plan["load_kg"]) == (1.755, 1.0) and plan["pendulum_m"] == pytest.approx(0.72)
    command = json.loads((run / "scenario.json").read_text())["driver"]["command"]
    assert command[command.index("--carry-speed") + 1] == "3.0" and "--shaped" in command and "--attach-check" in command
    plain = json.loads((build_run(SIM / "scenarios" / "recovery" / "hook_check.yaml", tmp_path / "plain", variant="on") / "scenario.json").read_text())["driver"]["command"]
    assert "--shaped" not in plain and "--carry-speed" not in plain and "--attach-check" not in plain  # REC-8's flight is as it was flown


# --- REC-7 in the check flight: the way back by the rule, and the set-down by the rule ---


class Carrier(Loaded):
    """The stand-in with what REC-7's rules read: a rangefinder, a landed state, a global position, a gripper that
    answers, and a bucket that takes its own weight when the vehicle is down at the touch height."""

    LAT, LON = 40.4237, -86.9212

    def __init__(self, touch=0.675, ack=0, never_felt=False, range_dead=False, **settings):
        super().__init__(**settings)
        self.touch, self.ack, self.never_felt, self.range_dead = touch, ack, never_felt, range_dead
        self.globals, self.locals, self.acked = [], [], None
        self.pair = recovery_return_anchor(self.LAT, self.LON)

    def throttle(self):
        holding = bool(self.grips) and self.grips[-1][1] is True
        standing = holding and not self.never_felt and self.u <= self.touch + 1e-9 and self.e ** 2 + self.n ** 2 < 4.0  # down, at the return point
        return self.loaded_throttle if holding and not standing else 0.40

    def gripper(self, grab):
        self.acked = 0 if grab else self.ack  # `ack` is the answer to a release; a grab is always taken
        if grab or self.ack == 0:
            super().gripper(grab)

    def gripper_ack(self):
        return self.acked

    def throttle_age_s(self):
        return 0.05

    def range_m(self):
        return (None, None) if self.range_dead else (self.u, 0.05)

    def landed(self):
        return (self.u <= 0.0, 0.2)

    def global_position(self):
        return self.pair.to_global(self.e, self.n)

    HOME_BELOW_LOCAL_ZERO_M = -0.18  # this vehicle's home is 0.18 m over its local frame's zero, as the sim's is

    def height_above_home_m(self):
        return self.u + self.HOME_BELOW_LOCAL_ZERO_M

    def goto_global(self, lat, lon, up):
        self.globals.append((lat, lon))
        east, north = self.pair.to_local(lat, lon)
        super().goto(east, north, up - self.HOME_BELOW_LOCAL_ZERO_M)  # a height above home, flown in the local frame

    def goto(self, e, n, u):
        self.locals.append((e, n))
        super().goto(e, n, u)


def recovery_return_anchor(lat, lon):
    from drone_mission_core import recovery_return

    return recovery_return.Anchor(0.0, 0.0, lat, lon)


def rule_flight(vehicle, **settings):
    from drone_mission_core import recovery_grip

    return tool.run_check(vehicle, attach=recovery_grip.AttachCheck(1.755, 1.0), **settings, **PLAN)


def test_the_way_back_is_a_walked_global_setpoint_with_gps_trusted_and_a_local_one_without():
    trusted = Carrier()
    report = rule_flight(trusted, return_frame="global", carry_speed_mps=3.0, pendulum_m=0.72)
    assert report["passed"] and report["return"]["frame"] == "global" and report["return"]["length_m"] == 14.42
    assert report["return"]["setpoints"]["global"] > 50 and report["return"]["setpoints"]["local"] == 0
    assert report["home_up_m"] == 0.18  # home is 0.18 m over the local zero: the global setpoints' height is given in home's terms, or the carry is 0.18 m off its height and never "arrives"
    # Walked: the global setpoints start at the bucket and end at the takeoff, a little further each step.
    first, last = trusted.pair.to_local(*trusted.globals[0]), trusted.pair.to_local(*trusted.globals[-1])
    assert math.hypot(first[0] - 12.0, first[1] - 8.0) < 0.05 and math.hypot(*last) < 1e-6
    untrusted = Carrier()
    report = rule_flight(untrusted, return_frame="local")
    assert report["passed"] and report["return"]["frame"] == "local" and untrusted.globals == []  # untrusted: never a global setpoint
    assert report["return"]["setpoints"] == {"global": 0, "local": report["return"]["setpoints"]["local"]} and report["return"]["lat"] is None
    # A published point, GPS untrusted: put into the local frame through the takeoff's pair, and flown there by local setpoints.
    from drone_mission_core import recovery_return

    published = Carrier()
    point = recovery_return.parse_return_target({"latlon": list(published.pair.to_global(0.0, 0.0))})
    report = rule_flight(published, return_frame="local", return_target=point)
    assert report["passed"] and report["return"]["target"] == "latlon" and report["return"]["frame"] == "local" and published.globals == []
    assert abs(report["return"]["e"]) < 1e-6 and abs(report["return"]["n"]) < 1e-6 and report["return"]["reason"] == ""
    assert rule_flight(Carrier(), return_frame="global")["return"]["target"] == "takeoff"
    # A global return with no global position to start from is said before takeoff.
    blind = Carrier()
    blind.global_position = lambda: (None, None)
    refused = rule_flight(blind, return_frame="global")
    assert not refused["passed"] and "before takeoff" in refused["problem"] and refused["steps"] == []
    assert "return_frame" in tool.run_check(Vehicle(), return_frame="sideways", **PLAN)["problem"]


def test_the_set_down_by_the_rule_feels_the_bucket_stand_lets_go_low_and_sees_that_it_went():
    vehicle = Carrier()
    report = rule_flight(vehicle, set_down_rule=True)
    down = report["set_down"]
    assert report["passed"] and down["released"] and down["confirmed"] and down["release_attempts"] == 1 and not down["land_first"]
    assert [phase["phase"] for phase in down["phases"]] == ["lower", "set_down", "release", "confirm"]
    assert down["released_at"]["range_m"] <= 0.675 + 0.01 and vehicle.grips[-1][1] is False and vehicle.grips[-1][4] <= 0.69  # let go with the bucket standing
    steps = [step["step"] for step in report["steps"]]
    assert "set down by the rule" in steps and "release commanded" not in steps and "bucket just off the ground" not in steps
    assert steps[-2:] == ["clear above", "clear aside"]
    # A gripper that refuses: asked three times, then the flight ends in the rule's fault and lands with the bucket.
    stuck = Carrier(ack=4)
    refused = rule_flight(stuck, set_down_rule=True)
    assert not refused["passed"] and "fault" in refused["problem"] and refused["set_down"]["release_attempts"] == 3 and refused["landed"]
    assert all(grab for _t, grab, *_rest in stuck.grips)  # nothing was let go
    # The set-down never felt: the rule lands first, and lets go once the flight controller says landed.
    numb = Carrier(never_felt=True)
    landed = rule_flight(numb, set_down_rule=True)
    assert landed["set_down"]["land_first"] and landed["set_down"]["released"] and not landed["set_down"]["confirmed"]
    assert landed["passed"] and numb.grips[-1][1] is False and numb.grips[-1][4] == 0.0 and "clear above" not in [step["step"] for step in landed["steps"]]
    # No rangefinder: the same, land first.
    assert rule_flight(Carrier(range_dead=True), set_down_rule=True)["set_down"]["land_first"]
    # The rule needs the attach check's two throttles.
    assert "attach check" in tool.run_check(Carrier(), set_down_rule=True, **PLAN)["problem"]


def test_the_return_scenario_asks_for_the_rules_and_the_others_do_not(tmp_path):
    from scenario.build import build_run

    run = build_run(SIM / "scenarios" / "recovery" / "return_set_down.yaml", tmp_path / "run", variant="on")
    command = json.loads((run / "scenario.json").read_text())["driver"]["command"]
    assert command[command.index("--return-frame") + 1] == "global" and "--set-down-rule" in command and "--attach-check" in command
    assert command[command.index("--return-target") + 1] == "takeoff"
    local = json.loads((build_run(SIM / "scenarios" / "recovery" / "return_local.yaml", tmp_path / "local", variant="on") / "scenario.json").read_text())["driver"]["command"]
    assert local[local.index("--return-frame") + 1] == "local" and local[local.index("--return-target") + 1] == "latlon" and "--set-down-rule" in local
    for other in ("hook_check", "hook_carry_shaped"):
        plain = json.loads((build_run(SIM / "scenarios" / "recovery" / f"{other}.yaml", tmp_path / other, variant="on") / "scenario.json").read_text())["driver"]["command"]
        assert "--return-frame" not in plain and "--set-down-rule" not in plain
