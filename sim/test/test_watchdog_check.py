"""PLT-09: the sim's check of the flight controller's watchdog (sim/watchdog_check.py), and the script's own text.

The autopilot here is made up: it runs the watchdog's rule as the Lua script
states it (a Python copy, kept beside the constants read out of the script),
so the check is tried against a watchdog that works and against ones that do
not. The script itself runs only in SITL: hover/watchdog is that flight.
"""

import re
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
REPO = SIM.parent
sys.path.insert(0, str(SIM))

import watchdog_check as wc  # noqa: E402
from fence_check import Sample  # noqa: E402

SCRIPT = (REPO / "platform" / "fcu" / "scripts" / "argus_watchdog.lua").read_text()


def lua_number(name):
    return float(re.search(rf"^local {name} = (\d+)", SCRIPT, re.MULTILINE).group(1))


class Autopilot:
    """A copter with the watchdog's rule in it. `watchdog`: "works", "none", "eager" (acts on any quiet second in
    GUIDED, armed or not by setpoints), "slow" (3 s), "deaf" (takes the setpoints for itself: the vehicle never moves)."""

    def __init__(self, watchdog="works", simulator=True):
        self.watchdog, self.simulator = watchdog, simulator
        self.t, self.e, self.n, self.u = 0.0, 0.0, 0.0, 0.0
        self.mode, self.armed, self.goal = "STABILIZE", False, None
        self.seen, self.last = 0, None
        self.taking_off_until = 0.0
        self.since_guided = 0.0

    def now(self):
        return self.t

    def is_armed(self):
        return self.armed

    def is_simulator(self):
        return self.simulator

    def takeoff(self, height_m):
        self.mode, self.armed = "GUIDED", True
        self.taking_off_until = self.t + 6.0
        while self.t < self.taking_off_until:  # the climb takes six seconds and sends nothing
            self.step()
        self.u = height_m

    def goto(self, e, n, u):
        self.seen, self.last = self.seen + 1, self.t
        if self.watchdog != "deaf":
            self.goal = (e, n)

    def return_home(self):
        self.mode, self.goal = "RTL", (0.0, 0.0)
        self._down_at = self.t + 15.0

    def step(self):
        self.t += 0.1
        if self.goal is not None and self.mode in ("GUIDED", "RTL"):
            de, dn = self.goal[0] - self.e, self.goal[1] - self.n
            far = max(1e-9, (de * de + dn * dn) ** 0.5)
            self.e, self.n = self.e + de / far * min(far, 0.3), self.n + dn / far * min(far, 0.3)
        taking_off = self.t < self.taking_off_until
        if self.mode != "GUIDED" or not self.armed or (taking_off and self.watchdog != "eager"):
            self.seen, self.last, self.since_guided = 0, None, self.t
        else:
            timeout = {"slow": 3.0}.get(self.watchdog, lua_number("TIMEOUT_MS") / 1000.0)
            armed = self.seen >= lua_number("ARM_AFTER")
            quiet_for = self.t - (self.last if self.last is not None else self.since_guided)
            if self.watchdog == "eager":
                armed = True
            if self.watchdog != "none" and armed and quiet_for > timeout:
                self.mode, self.seen, self.last = "LOITER", 0, None
        if self.mode == "RTL" and self.t >= self._down_at:
            self.armed, self.u = False, 0.0
        return Sample(self.t, self.e, self.n, self.u, self.mode)


def test_a_watchdog_that_works_passes_with_loiter_between_one_and_two_seconds():
    report = wc.run_check(Autopilot())
    assert report["passed"], report["problem"]
    assert 1.0 < report["loiter_delay_s"] <= 2.0
    assert report["guided_after_takeoff"] and report["guided_after_quiet_hover"] and report["guided_while_streaming"]
    assert report["followed_setpoints"] and report["landed_and_disarmed"]


@pytest.mark.parametrize(
    "watchdog, word",
    [
        ("none", "did not act"),
        ("slow", "limit 2 s"),  # LOITER after 3 s
        ("eager", "disturbed a take-off"),
        ("deaf", "did not reach the autopilot"),
    ],
)
def test_a_watchdog_that_does_not_do_what_the_script_says_fails(watchdog, word):
    report = wc.run_check(Autopilot(watchdog=watchdog))
    assert not report["passed"] and word in report["problem"], report["problem"]
    assert report["landed_and_disarmed"]  # brought home whatever happened


def test_an_autopilot_that_is_not_the_simulator_is_never_flown():
    vehicle = Autopilot(simulator=False)
    report = wc.run_check(vehicle)
    assert not report["passed"] and "sim only" in report["problem"] and not vehicle.armed


def test_the_verdicts_lines():
    good = {"guided_after_takeoff": True, "guided_after_quiet_hover": True, "guided_while_streaming": True, "followed_setpoints": True, "loiter_delay_s": 1.3}
    assert wc.verdict(good) == ""
    assert "inside the 1 s" in wc.verdict({**good, "loiter_delay_s": 0.8})
    assert "limit 2 s" in wc.verdict({**good, "loiter_delay_s": 2.4})
    assert "quiet hover" in wc.verdict({**good, "guided_after_quiet_hover": False})
    assert "while setpoints were being sent" in wc.verdict({**good, "guided_while_streaming": False})
    assert (wc.TIMEOUT_S, wc.LATEST_S) == (1.0, 2.0)


# --- the script's text: what can be held without running Lua ---------------------------------------------------


def test_the_scripts_numbers_are_the_specs_and_the_checks():
    assert lua_number("TIMEOUT_MS") == 1000 == wc.TIMEOUT_S * 1000  # "more than 1 s"
    assert lua_number("MODE_GUIDED") == 4 and lua_number("MODE_LOITER") == 5  # ArduCopter's mode numbers
    assert lua_number("ARM_AFTER") >= 2 and lua_number("PERIOD_MS") <= 100
    # The three MAVLink messages MAVROS sends setpoints as, by their ids.
    assert (lua_number("SET_ATTITUDE_TARGET"), lua_number("SET_POSITION_TARGET_LOCAL_NED"), lua_number("SET_POSITION_TARGET_GLOBAL_INT")) == (82, 84, 86)
    assert SCRIPT.count("mavlink:register_rx_msgid(") == 3 and "mavlink:init(" in SCRIPT


def test_the_script_acts_only_in_guided_armed_flying_and_not_taking_off_and_only_by_setting_loiter():
    condition = re.search(r"if (vehicle:get_mode\(\) ~= MODE_GUIDED.*?) then", SCRIPT).group(1)
    for clause in ("vehicle:get_mode() ~= MODE_GUIDED", "not arming:is_armed()", "not vehicle:get_likely_flying()", "vehicle:is_taking_off()"):
        assert clause in condition, clause
    assert SCRIPT.count("vehicle:set_mode(") == 1 and "vehicle:set_mode(MODE_LOITER)" in SCRIPT
    assert "seen >= ARM_AFTER and last_ms and (now - last_ms):toint() > TIMEOUT_MS" in SCRIPT
    # It commands nothing else: no arming, no parameters, no servo, no target.
    for call in ("arming:arm", "arming:disarm", "param:set", "SRV_Channels", "set_target", "rc:"):
        assert call not in SCRIPT, call
    assert SCRIPT.rstrip().endswith("return update, 1000")


def test_what_the_script_says_fits_in_a_status_text():
    """A STATUSTEXT carries 50 characters: the first flight's message came through cut off after "now"."""
    said = re.findall(r'gcs:send_text\(\w+, "([^"]+)"\)', SCRIPT)
    assert len(said) == 3 and all(len(text) <= 50 for text in said), said
    assert "ARGUS watchdog: setpoints stopped, now LOITER" in said
