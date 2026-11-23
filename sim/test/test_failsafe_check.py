"""PLT-08: the sim's failsafe check (sim/failsafe_check.py) against a made-up autopilot. No MAVLink, no ROS."""

import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
REPO = SIM.parent
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(REPO / "tools"))

import failsafe_check as fc  # noqa: E402
from fc_param_check import read_params  # noqa: E402
from fence_check import Sample  # noqa: E402

BASELINE = read_params(REPO / "platform" / "fcu" / "argus_2027.param")
MODES = {"LAND": 1, "RTL": 2}


class Autopilot:
    """An ArduCopter as far as the check can see it: parameters, a mode, a position, a gripper output.

    It trips its failsafes as the parameters it was started with say, after their timers. `faults` change that:
    "deaf" never trips, "slow" trips 20 s late, "drops" lets the gripper go on a failsafe, "wrong" lands where it
    should come home.
    """

    def __init__(self, params=None, faults=(), simulator=True):
        self.params = {**BASELINE, "SIM_BATT_VOLTAGE": 25.2, "SIM_RC_FAIL": 0.0, "SYSID_MYGCS": 255.0, **(params or {})}
        self.faults, self.simulator = set(faults), simulator
        self.t, self.e, self.n, self.u = 0.0, 0.0, 0.0, 0.0
        self.mode, self.armed, self.goal = "STABILIZE", False, None
        self.pwm = self.params.get("GRIP_NEUTRAL", 1500.0)
        self.since = {}  # failsafe -> when its condition began
        self.tripped = set()
        self.sets = []

    def now(self):
        return self.t

    def is_armed(self):
        return self.armed

    def is_simulator(self):
        return self.simulator

    def read_param(self, name):
        return self.params.get(name)

    def set_param(self, name, value):
        if not name.startswith(("SIM_", "SYSID_")):
            raise AssertionError(f"the check changed {name}: it may only change what trips a failsafe")
        self.params[name] = float(value)
        self.sets.append((name, float(value)))
        return True

    def takeoff(self, height_m):
        self.mode, self.armed, self.u = "GUIDED", True, height_m

    def goto(self, e, n, u):
        self.goal = (e, n)

    def return_home(self):
        self._enter("RTL")

    def grab(self):
        if "dead gripper" not in self.faults:
            self.pwm = self.params["GRIP_GRAB"]

    def servo_pwm(self):
        return self.pwm

    def _enter(self, mode):
        self.mode, self.goal = mode, ((0.0, 0.0) if mode == "RTL" else None)
        self._landing = self.t + (25.0 if mode == "RTL" else 12.0)

    def _conditions(self):
        p = self.params
        yield "battery critical", p["SIM_BATT_VOLTAGE"] < p["BATT_CRT_VOLT"], p["BATT_LOW_TIMER"], "BATT_FS_CRT_ACT"
        yield "battery low", p["SIM_BATT_VOLTAGE"] < p["BATT_LOW_VOLT"], p["BATT_LOW_TIMER"], "BATT_FS_LOW_ACT"
        yield "radio", p["SIM_RC_FAIL"] != 0.0, 1.0, "FS_THR_ENABLE"
        yield "ground station", p["SYSID_MYGCS"] != 255.0, p["FS_GCS_TIMEOUT"], "FS_GCS_ENABLE"

    def step(self):
        self.t += 0.2
        if self.goal is not None:
            de, dn = self.goal[0] - self.e, self.goal[1] - self.n
            far = max(1e-9, (de * de + dn * dn) ** 0.5)
            move = min(far, 1.0)
            self.e, self.n = self.e + de / far * move, self.n + dn / far * move
        if self.armed and "deaf" not in self.faults:
            for name, bad, timer, action in self._conditions():
                if not bad:
                    self.since.pop(name, None)
                    continue
                began = self.since.setdefault(name, self.t)
                late = 20.0 if "slow" in self.faults else 0.0
                if name not in self.tripped and self.t - began >= timer + late and self.params[action]:
                    self.tripped.add(name)
                    mode = fc.ACTIONS[action][int(self.params[action])]
                    self._enter("LAND" if "wrong" in self.faults else mode)
                    if "drops" in self.faults:
                        self.pwm = self.params["GRIP_RELEASE"]
        if self.armed and self.mode in ("RTL", "LAND") and self.t >= self._landing:
            self.armed, self.u = False, 0.0
        return Sample(self.t, self.e, self.n, self.u, self.mode)


@pytest.mark.parametrize("failsafe, modes", [("battery", ["RTL", "LAND"]), ("radio", ["RTL"]), ("ground_station", ["RTL"])])
def test_each_failsafe_of_the_baseline_passes_on_an_autopilot_that_obeys_it(failsafe, modes):
    vehicle = Autopilot()
    report = fc.run_check(vehicle, BASELINE, failsafe)
    assert report["passed"], report["problem"]
    assert [step["mode_seen"] for step in report["steps"]] == modes
    assert all(step["delay_s"] <= step["within_s"] for step in report["steps"])
    assert report["gripper_held"] is True and report["landed_and_disarmed"]
    # What was taken away is given back before the landing, and nothing else was touched.
    assert vehicle.params["SIM_BATT_VOLTAGE"] == BASELINE["BATT_LOW_VOLT"] + fc.HEALTHY_OVER_V and vehicle.params["SIM_RC_FAIL"] == 0.0 and vehicle.params["SYSID_MYGCS"] == 255.0
    assert {name: vehicle.params[name] for name in BASELINE} == BASELINE


def test_the_battery_is_put_just_under_each_threshold_in_turn():
    vehicle = Autopilot()
    fc.run_check(vehicle, BASELINE, "battery")
    volts = [value for name, value in vehicle.sets if name == "SIM_BATT_VOLTAGE"]
    assert volts[:2] == [BASELINE["BATT_LOW_VOLT"] - 0.5, BASELINE["BATT_CRT_VOLT"] - 0.5]
    steps = fc.plan("battery", BASELINE)
    assert [(s["expect"], s["within_s"]) for s in steps] == [("RTL", BASELINE["BATT_LOW_TIMER"] + 5.0), ("LAND", BASELINE["BATT_LOW_TIMER"] + 5.0)]
    assert fc.plan("ground_station", BASELINE)[0]["within_s"] == BASELINE["FS_GCS_TIMEOUT"] + 5.0


@pytest.mark.parametrize(
    "faults, word",
    [(("deaf",), "did not act"), (("slow",), "did not act"), (("wrong",), "became LAND, not RTL"), (("drops",), "gripper")],
)
def test_an_autopilot_that_does_not_do_what_the_baseline_says_fails(faults, word):
    report = fc.run_check(Autopilot(faults=faults), BASELINE, "radio")
    assert not report["passed"] and word in report["problem"]
    assert report["landed_and_disarmed"]  # brought home by the check itself when the failsafe did not


def test_2026s_settings_fail_every_check_because_nothing_acts():
    """What the aircraft flew with in June 2026: no failsafe had an action."""
    as_2026 = {"BATT_FS_LOW_ACT": 0.0, "BATT_FS_CRT_ACT": 0.0, "FS_THR_ENABLE": 0.0, "FS_GCS_ENABLE": 0.0}
    for failsafe in fc.WATCHED:
        # Against the baseline the check refuses to fly: the autopilot is not flying the baseline.
        report = fc.run_check(Autopilot(params=as_2026), BASELINE, failsafe)
        assert not report["passed"] and "did not start SITL with the baseline" in report["problem"] and report["steps"] == []
    # And a "baseline" that says to do nothing is not one this check can fly.
    with pytest.raises(fc.CheckError, match="does not know what mode"):
        fc.plan("radio", {**BASELINE, **as_2026})


def test_an_autopilot_that_is_not_the_simulator_is_never_flown():
    vehicle = Autopilot(simulator=False)
    report = fc.run_check(vehicle, BASELINE, "battery")
    assert not report["passed"] and "sim only" in report["problem"]
    assert not vehicle.armed and vehicle.sets == [] and vehicle.t == 0.0


def test_a_baseline_without_a_failsafes_parameters_or_an_unknown_failsafe_is_refused_before_anything_flies():
    vehicle = Autopilot()
    with pytest.raises(fc.CheckError, match="BATT_CRT_VOLT"):
        fc.run_check(vehicle, {k: v for k, v in BASELINE.items() if k != "BATT_CRT_VOLT"}, "battery")
    with pytest.raises(fc.CheckError, match="no failsafe called"):
        fc.run_check(vehicle, BASELINE, "ekf")
    assert not vehicle.armed and vehicle.sets == []


def test_a_step_is_judged_on_the_first_mode_after_its_trigger():
    step = {"name": "radio lost", "expect": "RTL", "within_s": 5.0}
    seen = [Sample(10.0 + 0.2 * i, 0, 0, 10, "GUIDED" if i < 8 else "RTL") for i in range(20)]
    result = fc.judge_step(step, "GUIDED", seen, 10.0)
    assert result["passed"] and result["delay_s"] == pytest.approx(1.6) and result["mode_seen"] == "RTL"
    assert "limit 1 s" in fc.judge_step({**step, "within_s": 1.0}, "GUIDED", seen, 10.0)["problem"]
    assert "already in RTL" in fc.judge_step(step, "RTL", seen, 10.0)["problem"]
    assert "did not act" in fc.judge_step(step, "GUIDED", seen[:5], 10.0)["problem"]


def test_the_tool_stops_with_exit_2_on_a_file_it_cannot_use(tmp_path, capsys):
    site = REPO / "sim" / "sites" / "apollo_xi.yaml"
    assert fc.main(["--site", str(site), "--failsafe", "radio", "--params", str(tmp_path / "none.param")]) == 2
    empty = tmp_path / "empty.param"
    empty.write_text("FS_OPTIONS,16\n")
    assert fc.main(["--site", str(site), "--failsafe", "radio", "--params", str(empty)]) == 2
    assert "FS_THR_ENABLE" in capsys.readouterr().err


def test_a_gripper_that_never_closed_proves_nothing_and_no_failsafe_is_tripped():
    """The spec audit's row: an output that never moved used to read as a payload held."""
    vehicle = Autopilot(faults=("dead gripper",))
    report = fc.run_check(vehicle, BASELINE, "radio")
    assert not report["passed"] and "after the grab" in report["problem"]
    assert report["steps"] == [] and report["gripper_pwm_after_grab"] == BASELINE["GRIP_NEUTRAL"]
    assert vehicle.sets == [] and report["landed_and_disarmed"]  # brought home without tripping anything


def test_a_gripper_that_goes_slack_in_a_failsafe_fails_even_if_it_does_not_reach_release():
    class Slack(Autopilot):
        def _enter(self, mode):
            super()._enter(mode)
            if self.tripped:
                self.pwm = self.params["GRIP_NEUTRAL"]

    report = fc.run_check(Slack(), BASELINE, "radio")
    assert not report["passed"] and "gripper" in report["problem"] and report["gripper_held"] is False
