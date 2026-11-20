"""The drop's last state with release gates (CEN-04), through the drop mission's real update().

With `drop_release_gates` on, FINAL_ALIGN takes the place of the fixed GPS
column: the mission descends on the estimator to the release height for the
sensor that gives the height, lets go only when every gate has held for half
a second, climbs back if it is under 2 m, looks again once after 25 s, and
after a second 25 s releases anyway if it is within half a metre. With the
flag off the mission is the 2026 one.

rclpy and the message packages are the stand-ins from
test_package_drop_mission.py; the context is test_centring_missions.py's.
"""

import math
import sys
from pathlib import Path

import pytest
import yaml  # noqa: F401  (the real one, before the stand-ins put an empty module in its place)

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_centring_missions as centring_tests  # noqa: E402  (installs the ROS stand-ins)

from drone_mission_core import release_gates, target_access  # noqa: E402
from drone_mission_core.features import FLAG_REQUIRES  # noqa: E402
from drone_mission_core.mission_api import MissionSpec, MissionStatus  # noqa: E402
from drone_mission_demo.missions.package_drop_mission import PackageDropMission, PackageDropState  # noqa: E402

FLAG = "drop_release_gates"
ON = {**centring_tests.FLAGS_ON, FLAG: True}
CONFIG = {
    **centring_tests.BASE,
    "drop_altitude_m": 6.0,  # the 2026 drop height: not where the gated drop releases
    "drop_altitude_tolerance_m": 3.0,
    "drop_column_handoff_altitude_m": 8.0,
    "drop_hover_dwell_s": 8.0,
    "descent_rate_mps": 0.5,
    "fake_drop": True,
}


class Context(centring_tests.Context):
    """The centring test's context, with the release rule, a height with its source, and a record of what was commanded."""

    def __init__(self, up_m=7.0, source="rangefinder"):
        super().__init__(vehicle=(0.0, 0.0, up_m))
        self.infos = []
        self.logger.info = self.infos.append
        self.velocity = (0.0, 0.0, 0.0)
        self.at(up_m, source)
        self.see(0.03, 0.0)

    def at(self, up_m, source="rangefinder"):
        position = self.local_pose.pose.position
        position.z = up_m
        self.height = target_access.HeightAgl(up_m, source)

    def a_track(self, east, north, state="CONFIRMED", age_s=0.05, track_id=1):
        track = super().a_track(east, north, state, age_s, track_id)
        return target_access.PrimaryTarget(**{**track.__dict__, "sigma_m": 0.04})  # a well-settled track, inside the sigma gate

    def widen(self, sigma_m):
        self.track = target_access.PrimaryTarget(**{**self.track.__dict__, "sigma_m": sigma_m})

    def tilt(self, roll_deg):
        x, y, z, w = centring_tests.quaternion(90.0, roll_deg)
        orientation = self.local_pose.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = x, y, z, w

    def may_release(self, class_id, *, near=None, track_id=None):
        return target_access.release_permission(self.track if track_id is None else self._by_id(track_id))

    def run(self, mission, seconds, keep_seeing=True):
        """Tick the mission for `seconds`, the track hit afresh every tick. The statuses."""
        statuses = []
        for _ in range(int(round(seconds / 0.05))):
            if keep_seeing and self.track is not None:
                self.track = target_access.PrimaryTarget(**{**self.track.__dict__, "for_ns": self._now_ns + 80_000_000})
            statuses.append(self.tick(mission))
        return statuses


def drop(context, features=ON, state=PackageDropState.TRACK_AND_DESCEND, **config):
    spec = MissionSpec(type_name="package_drop", name="package_drop", config={**CONFIG, **config}, base_dir=Path("."), features=dict(features))
    mission = PackageDropMission(spec)
    mission.on_enter(context)
    mission._state = state
    return mission


def aligned(up_m=2.4, source="rangefinder", **config):
    context = Context(up_m, source)
    mission = drop(context, **config)
    context.tick(mission)  # centred and under the handoff height: into FINAL_ALIGN
    assert mission._state == PackageDropState.FINAL_ALIGN
    return context, mission


def test_the_flag_needs_centring_on_the_estimator_and_is_declared_off_in_the_drop_sequences():
    import yaml as real_yaml

    # The estimator's track, the centring on it, and the height estimator: the floor and the band are only as
    # good as the height, and the bare rangefinder height has no stuck-sensor rule.
    assert FLAG_REQUIRES[FLAG] == ("use_target_estimator", "centre_on_estimator", "use_height_estimator") and release_gates.GATES_FLAG == FLAG
    sequences = Path(__file__).resolve().parents[1] / "config" / "sequences"
    declaring = []
    for path in sorted(sequences.glob("*.yaml")):
        root = real_yaml.safe_load(path.read_text())["mission_sequence"]
        if any(entry["type"] == "package_drop" for entry in root["missions"]):
            assert root["features"].get(FLAG) is False, path.name
            declaring.append(path.name)
    assert declaring == [
        "final_waypoint_drop.yaml", "package_drop_2027.yaml", "package_drop_demo.yaml", "package_drop_live.yaml",
        "sim_package_drop_2027.yaml", "sim_package_drop_live.yaml",
    ]  # fmt: skip


def test_flag_off_the_mission_goes_down_its_fixed_column_as_in_2026():
    context = Context(7.0)
    mission = drop(context, features=centring_tests.FLAGS_ON)
    assert mission._final_align is None
    context.tick(mission)
    assert mission._state == PackageDropState.FINAL_FIXED_DROP_COLUMN


def test_flag_on_final_align_takes_the_columns_place_and_descends_on_the_estimator_to_the_rangefinders_release_height():
    context = Context(7.0)
    mission = drop(context)
    context.tick(mission)
    assert mission._state == PackageDropState.FINAL_ALIGN and context.global_position_commands == []  # no GPS column
    context.see(0.3, 0.0)
    context.run(mission, 0.1)
    east, north, up, _yaw = context.local_velocity_commands[-1]
    assert east == pytest.approx(centring_tests.GAIN * 0.3) and up == 0.0  # steering on the track; not centred, so not descending
    context.see(0.03, 0.0)
    context.run(mission, 0.1)
    assert context.local_velocity_commands[-1][2] == pytest.approx(-0.5)  # centred and above the band: down at the mission's rate
    context.at(2.9)
    context.run(mission, 0.1)
    assert -0.5 < context.local_velocity_commands[-1][2] < 0.0  # slower as the band comes near
    context.at(2.58)
    context.velocity = (0.3, 0.0, 0.0)  # (too fast over the ground for the gates, so the descent can be watched)
    context.run(mission, 0.1)
    assert context.local_velocity_commands[-1][2] == pytest.approx(-0.1)  # inside the band's top edge: still going down, to its middle
    context.at(2.44)
    context.run(mission, 0.1)
    assert context.local_velocity_commands[-1][2] == 0.0  # at 2.4 m, the middle of the band: holding the height
    context.velocity = (0.0, 0.0, 0.0)
    context.at(2.1)
    context.run(mission, 0.1)
    assert context.local_velocity_commands[-1][2] > 0.0  # under the band: back up into it


def test_on_the_barometer_the_release_height_is_three_metres():
    context, mission = aligned(up_m=3.5, source="baro")
    context.run(mission, 0.1)
    assert context.local_velocity_commands[-1][2] < 0.0
    context.at(3.0, "baro")
    context.run(mission, 0.1)
    assert context.local_velocity_commands[-1][2] == 0.0
    context.at(2.6, "baro")  # inside the rangefinder's band, but this height is the barometer's
    statuses = context.run(mission, 1.0)
    assert context.local_velocity_commands[-1][2] > 0.0 and mission._state == PackageDropState.FINAL_ALIGN
    assert MissionStatus.SUCCESS not in statuses


def test_with_every_gate_held_for_half_a_second_the_payload_is_released_at_once_from_where_it_is():
    context, mission = aligned()
    statuses = context.run(mission, 0.45)
    assert mission._state == PackageDropState.FINAL_ALIGN and MissionStatus.SUCCESS not in statuses
    statuses = context.run(mission, 0.2)
    assert MissionStatus.SUCCESS in statuses and "[package_drop] Fake drop complete" in context.infos
    assert any("-> DROP_PAYLOAD" in text for text in context.infos)  # the state the stream keeper keeps quiet in
    assert context.global_position_commands == []  # no climb to the 2026 drop height, no hover on a GPS point
    assert context.sprayer_commands == []


def test_a_real_release_commands_the_actuator_once_in_the_release_state_and_finishes_when_it_is_acknowledged():
    context, mission = aligned(fake_drop=False)
    states = []
    original = context.command_sprayer

    def command_sprayer(*, enable, done_callback=None):
        states.append(mission._state)  # the state the stream keeper keeps quiet in
        return original(enable=enable, done_callback=done_callback)

    context.command_sprayer = command_sprayer
    statuses = context.run(mission, 1.0)
    assert context.sprayer_commands == [True] and states == [PackageDropState.DROP_PAYLOAD]
    assert "[package_drop] Closing sprayer for payload drop" in context.infos
    assert MissionStatus.SUCCESS in statuses and mission._state == PackageDropState.COMPLETE
    assert context.global_position_commands == []


@pytest.mark.parametrize(
    "spoil, gate",
    [
        (lambda c: c.see(0.14, 0.0), "error"),
        (lambda c: c.widen(0.09), "sigma"),
        (lambda c: setattr(c, "velocity", (0.2, 0.0, 0.0)), "speed"),
        (lambda c: c.tilt(8.0), "tilt"),
        (lambda c: c.see(0.03, 0.0, age_s=0.35), "track"),
        (lambda c: c.at(2.75), "height"),
    ],
)
def test_one_gate_not_holding_is_enough_to_keep_the_payload(spoil, gate):
    context, mission = aligned()
    spoil(context)
    statuses = context.run(mission, 2.0, keep_seeing=False)
    assert MissionStatus.SUCCESS not in statuses and mission._state == PackageDropState.FINAL_ALIGN
    assert context.sprayer_commands == [] and "[package_drop] Fake drop complete" not in context.infos
    assert any(f"waiting on {gate}" in text or f", {gate}" in text for text in context.infos + context.warnings), gate


def test_under_two_metres_the_vehicle_climbs_back_before_anything_else():
    context, mission = aligned(up_m=1.95)
    statuses = context.run(mission, 1.5)
    east, north, up, _yaw = context.local_velocity_commands[-1]
    assert up > 0.0 and (east, north) == (0.0, 0.0)  # up, and nothing else
    assert MissionStatus.SUCCESS not in statuses and mission._state == PackageDropState.FINAL_ALIGN
    assert any("below 2 m" in text for text in context.warnings)
    context.at(2.4)
    assert MissionStatus.SUCCESS in context.run(mission, 0.7)  # back in the band, the gates are asked again


def test_losing_the_track_at_three_metres_stops_the_descent_and_after_the_grace_time_the_target_is_looked_for_again():
    context, mission = aligned(up_m=3.0)
    context.run(mission, 0.1)
    assert context.local_velocity_commands[-1][2] < 0.0
    context.also(0.03, 0.0, state="LOST", age_s=2.5, track_id=1)
    context.track = None
    context.run(mission, 1.0, keep_seeing=False)
    assert context.local_velocity_commands[-1][:3] == (0.0, 0.0, 0.0) and mission._state == PackageDropState.FINAL_ALIGN
    context.run(mission, 3.2, keep_seeing=False)
    assert mission._state == PackageDropState.TARGET_NOT_FOUND


def test_after_25_s_without_the_gates_it_looks_again_once_then_releases_within_half_a_metre_or_gives_up():
    # The speed gate never holds. First 25 s: look again. Second 25 s, 0.3 m off: release anyway.
    context, mission = aligned()
    context.velocity = (0.3, 0.0, 0.0)
    context.see(0.3, 0.0)
    context.run(mission, 24.5)
    assert mission._state == PackageDropState.FINAL_ALIGN
    context.run(mission, 0.6)
    assert mission._state == PackageDropState.TARGET_NOT_FOUND and any("looking for the target again" in text for text in context.warnings)
    mission._state = PackageDropState.TRACK_AND_DESCEND  # the recovery climb and the acquisition, done
    context.see(0.05, 0.0)
    context.run(mission, 0.1)
    assert mission._state == PackageDropState.FINAL_ALIGN
    context.see(0.3, 0.0)
    statuses = context.run(mission, 25.2)
    assert MissionStatus.SUCCESS in statuses and "[package_drop] Fake drop complete" in context.infos
    assert any("releasing at the current estimate" in text for text in context.warnings)

    # The same, but 0.8 m off at the second timeout: no release, the mission fails and the sequence moves on.
    context, mission = aligned()
    context.velocity = (0.3, 0.0, 0.0)
    context.see(0.8, 0.0)
    context.run(mission, 25.1)
    mission._state = PackageDropState.FINAL_ALIGN
    mission._final_align.enter(context)
    statuses = context.run(mission, 25.2)
    assert statuses[-1] == MissionStatus.FAILURE and "[package_drop] Fake drop complete" not in context.infos
    assert context.sprayer_commands == []


def test_the_release_is_refused_under_the_floor_even_at_the_last_resort():
    context, mission = aligned(up_m=2.4)
    context.velocity = (0.3, 0.0, 0.0)
    context.see(0.2, 0.0)
    context.run(mission, 25.1)
    mission._state = PackageDropState.FINAL_ALIGN
    mission._final_align.enter(context)
    context.run(mission, 24.0)
    context.at(1.9)  # sinks under the floor just before the second timeout
    statuses = context.run(mission, 3.0)
    assert MissionStatus.SUCCESS not in statuses and "[package_drop] Fake drop complete" not in context.infos


def test_on_the_barometer_nothing_is_released_under_its_band_not_even_at_the_last_resort():
    context, mission = aligned(up_m=2.5, source="baro")
    context.velocity = (0.3, 0.0, 0.0)
    context.see(0.2, 0.0)
    context.run(mission, 25.1)
    mission._state = PackageDropState.FINAL_ALIGN
    mission._final_align.enter(context)
    statuses = context.run(mission, 25.2)
    assert statuses[-1] == MissionStatus.FAILURE and "[package_drop] Fake drop complete" not in context.infos
    # Nor at the moment of the release itself, if the height came off the rangefinder after the gates held.
    context, mission = aligned(up_m=2.4)
    mission._state = PackageDropState.DROP_PAYLOAD
    context.at(2.4, "baro")
    context.run(mission, 0.05)
    assert mission._state == PackageDropState.FINAL_ALIGN and "[package_drop] Fake drop complete" not in context.infos
    # At 2.9 m on the barometer the last resort releases.
    context, mission = aligned(up_m=2.9, source="baro")
    context.velocity = (0.3, 0.0, 0.0)
    context.see(0.2, 0.0)
    context.run(mission, 25.1)
    mission._state = PackageDropState.FINAL_ALIGN
    mission._final_align.enter(context)
    assert MissionStatus.SUCCESS in context.run(mission, 25.2)


def test_with_no_recovery_attempt_left_the_first_timeout_is_the_last_and_a_near_target_still_gets_the_package():
    """The approach may have used every recovery attempt before FINAL_ALIGN. Then there is no looking again:
    the first 25 s end as the second would, in a release within half a metre or in moving on."""
    context, mission = aligned()
    mission._recovery_attempts = mission._max_recovery_attempts
    context.velocity = (0.3, 0.0, 0.0)
    context.see(0.3, 0.0)
    statuses = context.run(mission, 25.2)
    assert MissionStatus.SUCCESS in statuses and "[package_drop] Fake drop complete" in context.infos
    assert any("no recovery attempt is left" in text for text in context.warnings)
    assert PackageDropState.TARGET_NOT_FOUND != mission._state

    context, mission = aligned()
    mission._recovery_attempts = mission._max_recovery_attempts
    context.velocity = (0.3, 0.0, 0.0)
    context.see(0.8, 0.0)
    statuses = context.run(mission, 25.2)
    assert statuses[-1] == MissionStatus.FAILURE and "[package_drop] Fake drop complete" not in context.infos


class Reply:
    def __init__(self, success):
        self.success = success

    def result(self):
        return self


def test_while_the_release_command_cannot_be_sent_the_vehicle_stays_on_the_gates_and_keeps_steering():
    context, mission = aligned(fake_drop=False)
    ready = [False]
    context.command_service_ready = lambda: ready[0]
    context.run(mission, 3.0)  # (the line in the log is one of those said at most every 2 s)
    assert context.sprayer_commands == [] and mission._state == PackageDropState.FINAL_ALIGN
    assert any("cannot be sent" in text for text in context.warnings)
    context.see(0.3, 0.0)  # the vehicle has drifted off meanwhile
    ready[0] = True
    context.run(mission, 1.0)
    assert context.sprayer_commands == [] and mission._state == PackageDropState.FINAL_ALIGN  # not released on gates that held earlier
    assert math.hypot(*context.local_velocity_commands[-1][:2]) > 0.0  # steering back
    context.see(0.03, 0.0)
    context.run(mission, 0.7)
    assert context.sprayer_commands == [True]


def test_a_release_command_that_fails_is_sent_again_only_after_the_gates_have_held_again():
    context, mission = aligned(fake_drop=False)
    pending = []

    def command_sprayer(*, enable, done_callback=None):
        context.sprayer_commands.append(enable)
        pending.append(done_callback)

    context.command_sprayer = command_sprayer
    context.run(mission, 0.7)
    assert context.sprayer_commands == [True] and mission._state == PackageDropState.DROP_PAYLOAD
    context.run(mission, 1.0)
    assert context.sprayer_commands == [True]  # waiting for the answer: not commanded twice
    context.see(0.3, 0.0)  # the track moves off while the answer is awaited
    pending.pop()(Reply(False))
    context.run(mission, 2.0)
    assert context.sprayer_commands == [True] and mission._state == PackageDropState.FINAL_ALIGN
    assert any("release command failed" in text for text in context.warnings)
    context.see(0.03, 0.0)
    context.run(mission, 0.4)
    assert context.sprayer_commands == [True]  # the gates have not yet held for half a second again
    for _ in range(6):
        if len(context.sprayer_commands) < 2:
            context.run(mission, 0.05)
    assert context.sprayer_commands == [True, True]
    # With the track good throughout and the refusal coming at once, a failed command still costs a fresh half second on the gates.
    pending.pop()(Reply(False))
    context.run(mission, 0.45)
    assert context.sprayer_commands == [True, True] and mission._state == PackageDropState.FINAL_ALIGN
    context.run(mission, 0.3)
    assert context.sprayer_commands == [True, True, True]
    pending.pop()(Reply(True))
    assert MissionStatus.SUCCESS in context.run(mission, 0.1)


class OneSeverityPerLine:
    """rclpy's rule for a logger: one line of code logs at one severity, or the call raises.
    (Found in the sim: a helper that chose the severity at one call site ended the mission.)"""

    def __init__(self):
        self.seen = {}
        self.lines = []

    def _log(self, severity, text):
        caller = sys._getframe(2)
        where = (caller.f_code.co_filename, caller.f_lineno)
        if self.seen.setdefault(where, severity) != severity:
            raise RuntimeError("Logger severity cannot be changed between calls.")
        self.lines.append(text)

    def info(self, text, **_):
        self._log("info", text)

    def warn(self, text, **_):
        self._log("warn", text)

    warning = warn

    def error(self, text, **_):
        self._log("error", text)


def test_every_path_through_final_align_logs_as_rclpy_allows():
    # Waiting on a gate (info), under the floor (warn), no height (warn), the command not ready (warn), both timeouts.
    context, mission = aligned(up_m=2.75, fake_drop=False)
    context.logger = OneSeverityPerLine()
    context.velocity = (0.3, 0.0, 0.0)
    context.run(mission, 2.5)
    context.at(1.9)
    context.run(mission, 2.5)
    context.height = None
    context.run(mission, 2.5)
    context.at(2.4)
    context.velocity = (0.0, 0.0, 0.0)
    context.command_service_ready = lambda: False
    context.run(mission, 3.0)
    context.velocity = (0.3, 0.0, 0.0)
    context.run(mission, 16.0)
    assert mission._state == PackageDropState.TARGET_NOT_FOUND
    mission._state = PackageDropState.FINAL_ALIGN
    mission._final_align.enter(context)
    context.run(mission, 28.0)  # the last timeout, with the command still not ready (its line is said at most every 2 s)
    text = "\n".join(context.logger.lines)
    for expected in ("waiting on speed", "climbing back", "no height above the ground", "cannot be sent yet", "looking for the target again", "the last timeout has passed"):
        assert expected in text, expected
