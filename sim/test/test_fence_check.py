"""sim/fence_check.py: fly at the fence and measure what the autopilot does.

The check is the SIM-02 pass line: leaving the fence must end in RTL within
2 s and at most 10 m outside. The tests fly a small pretend vehicle through
the same steps as the real one, and check the measurement on hand-made
tracks. No ROS and no pymavlink.
"""

import math
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import fence_check  # noqa: E402
import site_file  # noqa: E402
from fence_check import Sample, analyse  # noqa: E402
from judge.geometry import distance_outside_polygon  # noqa: E402

APOLLO = SIM / "sites" / "apollo_xi.yaml"
SQUARE = [(-50.0, -50.0), (50.0, -50.0), (50.0, 50.0), (-50.0, 50.0)]


def track(points, mode_at):
    """Samples along (t, e) points on the east axis; mode_at(t) gives the mode."""
    return [Sample(t, e, 0.0, 15.0, mode_at(t)) for t, e in points]


# --- the measurement ---------------------------------------------------------


def test_crossing_time_is_interpolated_between_position_samples():
    samples = track([(0.0, 40.0), (1.0, 48.0), (2.0, 56.0), (3.0, 52.0), (4.0, 44.0)], lambda t: "RTL" if t >= 2.0 else "GUIDED")
    result = analyse(samples, SQUARE)
    assert result["t_cross"] == pytest.approx(1.25, abs=0.01)  # 2 of the 8 m of that leg
    assert result["t_rtl"] == 2.0
    assert result["rtl_delay_s"] == pytest.approx(0.75, abs=0.01)
    assert result["max_excursion_m"] == pytest.approx(6.0)
    assert result["passed"] is True


def test_rtl_before_the_line_is_a_negative_delay_and_passes():
    samples = track([(0.0, 44.0), (1.0, 48.5), (2.0, 51.0), (3.0, 47.0)], lambda t: "RTL" if t >= 1.0 else "GUIDED")
    result = analyse(samples, SQUARE)
    assert result["rtl_delay_s"] < 0.0
    assert result["passed"] is True


def test_rtl_that_stops_the_vehicle_inside_the_fence_passes_with_no_excursion():
    samples = track([(0.0, 40.0), (1.0, 47.0), (2.0, 49.0), (3.0, 45.0)], lambda t: "RTL" if t >= 1.0 else "GUIDED")
    result = analyse(samples, SQUARE)
    assert result["t_cross"] is None and result["rtl_delay_s"] is None
    assert result["max_excursion_m"] == 0.0
    assert result["passed"] is True


@pytest.mark.parametrize("rtl_at, passed", [(3.2, True), (3.4, False)])
def test_rtl_must_come_within_two_seconds_of_crossing(rtl_at, passed):
    points = [(0.1 * i, 40.0 + 0.8 * i) for i in range(60)]  # crosses 50 m east at t = 1.25
    result = analyse(track(points, lambda t: "RTL" if t >= rtl_at - 1e-9 else "GUIDED"), SQUARE, max_excursion_m=1000.0)
    assert result["t_cross"] == pytest.approx(1.25, abs=0.01)
    assert result["passed"] is passed


@pytest.mark.parametrize("furthest, passed", [(59.5, True), (60.5, False)])
def test_excursion_must_stay_within_ten_metres(furthest, passed):
    samples = track([(0.0, 45.0), (1.0, 52.0), (2.0, furthest), (3.0, 50.0)], lambda t: "RTL" if t >= 1.0 else "GUIDED")
    result = analyse(samples, SQUARE)
    assert result["max_excursion_m"] == pytest.approx(furthest - 50.0)
    assert result["passed"] is passed


def test_no_rtl_fails_even_if_the_vehicle_stays_inside():
    result = analyse(track([(0.0, 10.0), (1.0, 20.0)], lambda t: "GUIDED"), SQUARE)
    assert result["t_rtl"] is None
    assert result["passed"] is False


def test_sample_rate_is_reported():
    samples = track([(0.25 * i, 0.0) for i in range(9)], lambda t: "GUIDED")
    assert analyse(samples, SQUARE)["position_rate_hz"] == pytest.approx(4.0)


# --- the flight, with a pretend vehicle --------------------------------------


class PretendVehicle:
    """A point that flies at 5 m/s, with ArduPilot's fence behaviour as switches."""

    def __init__(self, site, refuses_outside_targets=True, breach_action="RTL", reaction_s=0.0, ignores_targets=False, simulator=True):
        self.simulator = simulator
        self.site = site
        self.t = 0.0
        self.e, self.n, self.u = site.takeoff[0], site.takeoff[1], 0.0
        self.mode = "GUIDED"
        self.target = None
        self.velocity_cmd = None
        self.refuses_outside_targets = refuses_outside_targets
        self.breach_action = breach_action
        self.reaction_s = reaction_s
        self.ignores_targets = ignores_targets
        self.breach_t = None
        self.armed = False
        self.calls = []

    def is_simulator(self):
        return self.simulator

    def takeoff(self, height_m):
        self.calls.append(("takeoff", height_m))
        self.u = height_m
        self.armed = True

    def goto(self, e, n, u):
        self.calls.append(("goto", round(e, 1), round(n, 1)))
        outside = distance_outside_polygon(e, n, self.site.boundary) > 0.0
        if self.ignores_targets or (outside and self.refuses_outside_targets):
            return
        self.target, self.velocity_cmd = (e, n), None

    def velocity(self, east_mps, north_mps):
        self.velocity_cmd, self.target = (east_mps, north_mps), None

    def step(self):
        dt = 0.25
        self.t += dt
        if self.mode == "GUIDED":
            if self.target is not None:
                de, dn = self.target[0] - self.e, self.target[1] - self.n
                distance = math.hypot(de, dn)
                move = min(distance, 5.0 * dt)
                if distance > 1e-9:
                    self.e += de / distance * move
                    self.n += dn / distance * move
            elif self.velocity_cmd is not None:
                self.e += self.velocity_cmd[0] * dt
                self.n += self.velocity_cmd[1] * dt
            if distance_outside_polygon(self.e, self.n, self.site.boundary) > 0.0 and self.breach_t is None:
                self.breach_t = self.t
            if self.breach_t is not None and self.t - self.breach_t >= self.reaction_s and self.breach_action:
                self.mode = self.breach_action
        elif self.mode == "RTL":
            distance = math.hypot(self.e, self.n)
            move = min(distance, 5.0 * dt)
            if distance > 1e-9:
                self.e -= self.e / distance * move
                self.n -= self.n / distance * move
            else:
                self.armed = False
        return Sample(self.t, self.e, self.n, self.u, self.mode)

    def return_home(self):
        self.calls.append(("return_home",))
        self.mode = "RTL"

    def now(self):
        return self.t

    def is_armed(self):
        return self.armed


@pytest.fixture
def site():
    return site_file.load_site(APOLLO)


def test_check_passes_when_the_autopilot_refuses_the_outside_target_and_rtls_on_a_breach(site):
    vehicle = PretendVehicle(site)
    report = fence_check.run_check(vehicle, site, speed_mps=5.0)
    assert report["inside_target_followed"] is True
    assert report["outside_target_refused"] is True
    assert report["breach_by"] == "velocity command"
    assert report["passed"] is True
    assert report["max_excursion_m"] <= 1.3  # one 0.25 s step at 5 m/s
    east_edge = max(e for e, _ in site.boundary)
    assert ("goto", round(east_edge + 20.0, 1), 0.0) in vehicle.calls  # the spec's waypoint, 20 m outside
    assert vehicle.is_armed() is False  # it waited for the vehicle to come home and disarm


def test_check_measures_the_waypoint_itself_when_the_autopilot_flies_to_it(site):
    vehicle = PretendVehicle(site, refuses_outside_targets=False, reaction_s=1.0)
    report = fence_check.run_check(vehicle, site, speed_mps=5.0)
    assert report["outside_target_refused"] is False
    assert report["breach_by"] == "waypoint outside the fence"
    assert report["rtl_delay_s"] == pytest.approx(1.0, abs=0.3)
    assert report["max_excursion_m"] == pytest.approx(5.0, abs=1.5)
    assert report["passed"] is True


def test_check_fails_when_the_breach_does_not_end_in_rtl(site):
    vehicle = PretendVehicle(site, breach_action=None)
    report = fence_check.run_check(vehicle, site, speed_mps=5.0, breach_timeout_s=60.0)
    assert report["passed"] is False
    assert report["t_rtl"] is None
    assert "no RTL" in report["problem"]
    assert ("return_home",) in vehicle.calls  # it is never left flying away


def test_check_fails_when_rtl_comes_too_late(site):
    vehicle = PretendVehicle(site, reaction_s=4.0)
    report = fence_check.run_check(vehicle, site, speed_mps=5.0)
    assert report["rtl_delay_s"] > 2.0
    assert report["max_excursion_m"] > 10.0
    assert report["passed"] is False


def test_check_stops_when_the_vehicle_ignores_even_an_inside_target(site):
    """Then "refused" would prove nothing: the link or the mode is broken."""
    vehicle = PretendVehicle(site, ignores_targets=True)
    report = fence_check.run_check(vehicle, site, speed_mps=5.0)
    assert report["inside_target_followed"] is False
    assert report["passed"] is False
    assert "inside the fence" in report["problem"]
    assert not any(call[0] == "goto" and call[1] > 100 for call in vehicle.calls)


def test_check_never_arms_an_autopilot_that_is_not_the_simulator(site):
    vehicle = PretendVehicle(site, simulator=False)
    report = fence_check.run_check(vehicle, site)
    assert report["passed"] is False
    assert "not SITL" in report["problem"]
    assert vehicle.calls == [] and vehicle.armed is False


def test_cli_needs_a_good_site(tmp_path):
    import subprocess

    bad = tmp_path / "bad.yaml"
    bad.write_text("name: nowhere\n")
    result = subprocess.run([sys.executable, str(SIM / "fence_check.py"), "--site", str(bad)], capture_output=True, text=True)
    assert result.returncode == 2
    assert "bad.yaml" in result.stderr
