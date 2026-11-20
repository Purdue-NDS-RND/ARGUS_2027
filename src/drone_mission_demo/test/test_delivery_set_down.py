"""The delivery's soft set-down (CEN-05), through the delivery mission's real update().

With `delivery_soft_set_down` on, FINAL_SET_DOWN takes the place of the fixed
GPS column: the mission comes down on the estimator's track, holds its
horizontal position below 0.6 m and descends at 0.12 m/s, takes contact from
the rangefinder, the vertical speed and the throttle together for 0.3 s,
releases, waits a second and climbs. With the flag off the mission is the
2026 one.

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
from test_drop_release_gates import OneSeverityPerLine  # noqa: E402

from drone_mission_core import safety_limits, set_down, target_access  # noqa: E402
from drone_mission_core.mission_api import MissionSpec, MissionStatus  # noqa: E402
from drone_mission_demo.missions.package_delivery_mission import PackageDeliveryMission, PackageDeliveryState  # noqa: E402

FLAG = "delivery_soft_set_down"
ON = {**centring_tests.FLAGS_ON, "use_height_estimator": True, FLAG: True}
CONFIG = {
    **centring_tests.BASE,
    "landing_check_threshold_m": 6.0,
    "touchdown_handoff_tolerance_m": 0.2,
    "descent_rate_mps": 0.4,
    "final_descent_rate_mps": 0.25,
    "delivery_dwell_s": 5.0,  # the 2026 wait on the ground: not what the set-down waits
    "touchdown_dwell_s": 0.5,
    "relaunch_altitude_m": 10.0,
    "package_height_m": 0.10,
    "fake_drop": True,
}


class Context(centring_tests.Context):
    """The centring test's context with a height, a rangefinder, a throttle, a landed flag and a gripper."""

    def __init__(self, up_m=5.75):  # 6.0 m on the flight controller's height: the handoff height
        super().__init__(vehicle=(0.0, 0.0, up_m))
        self.infos = []
        self.logger.info = self.infos.append
        self.velocity = (0.0, 0.0, -0.12)
        self.range = None
        self.throttle_now = (0.40, 0.05)
        self.landed = False
        self.landing_state = True
        self.gripper_commands = []
        self.gripper_answers = True
        self.pending = []
        self.attitude_commands = []
        self.at(up_m)
        self.see(0.03, 0.0)

    def at(self, up_m, source="rangefinder", fc_reads_high_by=0.25):
        """Put the vehicle `up_m` above the ground. The flight controller's height reads a little higher, as the
        sim's does, so that the ground's 0.6 m is above the 0.75 m under which the mission takes no target."""
        self.local_pose.pose.position.z = up_m + fc_reads_high_by
        self.height = target_access.HeightAgl(up_m, source)
        self.range = (up_m, 0.05)

    def rangefinder_range(self):
        return self.range

    def throttle(self):
        return self.throttle_now

    def landing_state_available(self):
        return self.landing_state

    def vehicle_is_landed(self):
        return self.landed

    def command_gripper(self, *, release, done_callback=None):
        self.gripper_commands.append(release)
        self.pending.append(done_callback)

    def set_attitude_climb_rate_setpoint(self, rate, yaw_deg=90.0, max_climb_rate_mps=None):
        self.attitude_commands.append(rate)

    def run(self, mission, seconds):
        """Tick the mission for `seconds`, the track hit afresh every tick and the gripper answering. The statuses."""
        statuses = []
        for _ in range(int(round(seconds / 0.05))):
            if self.track is not None:
                self.track = target_access.PrimaryTarget(**{**self.track.__dict__, "for_ns": self._now_ns + 80_000_000})
            statuses.append(self.tick(mission))
            while self.pending and self.gripper_answers:
                self.pending.pop()(centring_tests.stand_ins.FakeFuture(success=True))
        return statuses

    def touch_down(self):
        """The package is on the ground: the rangefinder under the package's height, no vertical speed, the throttle off by a quarter."""
        self.range = (0.02, 0.05)
        self.velocity = (0.0, 0.0, 0.0)
        self.throttle_now = (0.30, 0.05)


def delivery(context, features=ON, state=PackageDeliveryState.TRACK_AND_DESCEND, **config):
    spec = MissionSpec(type_name="package_delivery", name="package_delivery", config={**CONFIG, **config}, base_dir=Path("."), features=dict(features))
    mission = PackageDeliveryMission(spec)
    mission.on_enter(context)
    mission._state = state
    return mission


def setting_down(up_m=5.75, **config):
    context = Context(up_m)
    mission = delivery(context, **config)
    context.tick(mission)  # centred at the handoff height: into FINAL_SET_DOWN
    assert mission._state == PackageDeliveryState.FINAL_SET_DOWN
    return context, mission


def lowered(**config):
    """In FINAL_SET_DOWN, below 0.6 m, after three seconds of the last descent: the detector knows the descent's throttle."""
    context, mission = setting_down(**config)
    context.at(0.55)
    context.run(mission, 3.0)
    assert mission._state == PackageDeliveryState.FINAL_SET_DOWN
    return context, mission


def test_the_flag_is_declared_off_in_the_delivery_sequences_and_the_rate_is_the_safety_limits():
    import yaml as real_yaml

    assert set_down.SET_DOWN_RATE_MPS == safety_limits.PHASE_SPEED_CAPS_MPS["set_down"] == 0.12
    sequences = Path(__file__).resolve().parents[1] / "config" / "sequences"
    declaring = []
    for path in sorted(sequences.glob("*.yaml")):
        root = real_yaml.safe_load(path.read_text())["mission_sequence"]
        if any(entry["type"] == "package_delivery" for entry in root["missions"]):
            assert root["features"].get(FLAG) is False, path.name
            declaring.append(path.name)
            heights = [(entry.get("config") or {}).get("package_height_m") for entry in root["missions"] if entry["type"] == "package_delivery"]
            # The sim's own sequences have the sim's value. On the aircraft nobody has measured it: no value, and the flag cannot fly without one.
            assert heights == ([0.05] if path.name in ("package_delivery_demo.yaml", "sim_package_delivery_2027.yaml") else [None]), path.name
    assert declaring == [
        "final_curcuit_delivery.yaml", "package_delivery_2027.yaml", "package_delivery_demo.yaml", "package_delivery_live.yaml",
        "sim_package_delivery_2027.yaml", "sim_package_delivery_live.yaml",
    ]  # fmt: skip


def test_with_the_flag_off_the_delivery_goes_to_its_fixed_column_as_in_2026():
    assert delivery(Context(), features={})._set_down is None  # every flag off: the 2026 mission, which its own tests fly
    # Centring on the estimator without this flag: still the 2026 column from the handoff height.
    context = Context()
    mission = delivery(context, features=centring_tests.FLAGS_ON)
    assert mission._set_down is None
    context.run(mission, 0.2)
    assert mission._state == PackageDeliveryState.FINAL_FIXED_COLUMN_DESCENT
    assert context.global_position_commands  # the frozen GPS column
    context.landed = True
    context.run(mission, 1.0)
    assert mission._state == PackageDeliveryState.GROUND_DWELL
    context.run(mission, 4.0)
    assert mission._state == PackageDeliveryState.GROUND_DWELL  # the 2026 five seconds on the ground, not the set-down's one


def test_with_the_flag_on_the_descent_goes_on_on_the_estimator_easing_to_the_set_down_rate():
    context, mission = setting_down()
    assert context.global_position_commands == []  # no GPS column from the handoff height
    assert any("Entering final set-down at 6.00 m" in text for text in context.infos)
    context.run(mission, 0.1)
    east, north, up, _yaw = context.local_velocity_commands[-1]
    assert up == pytest.approx(-0.4) and east > 0.0 and north == pytest.approx(0.0)  # down at the mission's rate, steering onto the track
    context.at(1.0)
    context.run(mission, 0.1)
    assert context.local_velocity_commands[-1][2] == pytest.approx(-(0.12 + 0.3 * 0.4))
    # Off centre: steering, not descending. On a track that may not be descended on: the same.
    context.see(0.6, 0.0)
    context.run(mission, 0.1)
    assert context.local_velocity_commands[-1][2] == 0.0 and context.local_velocity_commands[-1][0] > 0.0
    context.see(0.03, 0.0, state="COASTING", age_s=0.4)
    context.tick(mission)
    assert context.local_velocity_commands[-1][2] == 0.0
    assert mission._state == PackageDeliveryState.FINAL_SET_DOWN and context.gripper_commands == []


def test_below_0_6_m_the_position_is_held_and_the_descent_is_0_12_m_per_s_whatever_the_track_does():
    context, mission = setting_down()
    context.at(0.58)
    velocity_commands = len(context.local_velocity_commands)
    context.run(mission, 1.0)
    held = context.global_position_commands
    assert len(context.local_velocity_commands) == velocity_commands  # no velocity command: nothing steers
    assert len(held) == 20 and {(lat, lon) for lat, lon, *_ in held} == {(context.global_gps.latitude, context.global_gps.longitude)}
    heights = [altitude for _lat, _lon, altitude, *_ in held]
    assert heights[0] == pytest.approx(0.58 + 0.25, abs=0.01) and heights[-1] - heights[0] == pytest.approx(-0.12 * 0.95, abs=1e-6)  # 0.12 m/s
    # The track jumps a metre away, then is lost: neither changes the held point, and nothing looks for the target again.
    context.see(1.0, 0.5)
    context.run(mission, 0.5)
    context.track = None
    context.run(mission, 6.0)
    assert {(lat, lon) for lat, lon, *_ in context.global_position_commands} == {(context.global_gps.latitude, context.global_gps.longitude)}
    assert mission._state == PackageDeliveryState.FINAL_SET_DOWN and len(context.local_velocity_commands) == velocity_commands
    # The height coming back over 0.6 m (the barometer's, once the rangefinder is under its range) does not undo the hold.
    context.at(0.75, "baro")
    context.run(mission, 0.5)
    assert len(context.local_velocity_commands) == velocity_commands


def test_contact_for_three_tenths_of_a_second_releases_then_a_second_later_the_vehicle_climbs():
    context, mission = lowered()
    context.touch_down()
    statuses = context.run(mission, 0.25)
    assert mission._state == PackageDeliveryState.FINAL_SET_DOWN and "[package_delivery] Fake delivery complete" not in context.infos
    context.run(mission, 0.15)
    assert mission._state == PackageDeliveryState.GROUND_DWELL and "[package_delivery] Fake delivery complete" in context.infos
    assert any("contact:" in text and "throttle 0.30 against 0.40" in text for text in context.infos)
    context.run(mission, 0.9)
    assert mission._state == PackageDeliveryState.GROUND_DWELL and context.attitude_commands == []  # the second after the release
    context.run(mission, 0.2)
    assert mission._state == PackageDeliveryState.GUIDED_RELAUNCH and context.attitude_commands  # climbing: not the 2026 five seconds
    assert MissionStatus.FAILURE not in statuses


def test_a_real_delivery_commands_the_gripper_once_in_the_release_state_and_waits_a_second_from_its_answer():
    context, mission = lowered(fake_drop=False)
    context.gripper_answers = False
    context.touch_down()
    states = []
    original = context.command_gripper

    def command_gripper(*, release, done_callback=None):
        states.append(mission._state)  # the state the stream keeper keeps quiet in
        return original(release=release, done_callback=done_callback)

    context.command_gripper = command_gripper
    context.run(mission, 2.5)
    assert context.gripper_commands == [True] and states == [PackageDeliveryState.GROUND_DWELL]
    assert "[package_delivery] Releasing gripper payload" in context.infos
    assert mission._state == PackageDeliveryState.GROUND_DWELL and context.attitude_commands == []  # no answer yet: no climb
    context.gripper_answers = True
    context.run(mission, 0.9)
    assert mission._state == PackageDeliveryState.GROUND_DWELL
    context.run(mission, 0.2)
    assert mission._state == PackageDeliveryState.GUIDED_RELAUNCH and context.gripper_commands == [True]


@pytest.mark.parametrize(
    "spoil, signal",
    [
        (lambda c: setattr(c, "range", (0.25, 0.05)), "range"),
        (lambda c: setattr(c, "range", None), "range"),
        (lambda c: setattr(c, "velocity", (0.0, 0.0, -0.12)), "speed"),
        (lambda c: setattr(c, "throttle_now", (0.39, 0.05)), "throttle"),
        (lambda c: setattr(c, "throttle_now", None), "throttle"),
    ],
)
def test_one_signal_missing_is_no_contact_and_nothing_is_released(spoil, signal):
    context, mission = lowered()
    context.touch_down()
    spoil(context)
    context.run(mission, 3.0)
    assert mission._state == PackageDeliveryState.FINAL_SET_DOWN and "[package_delivery] Fake delivery complete" not in context.infos
    assert any(f"waiting on {signal}" in text or f", {signal}" in text for text in context.infos), signal


def test_the_flight_controllers_landed_state_is_the_backstop_when_a_signal_never_comes():
    context, mission = lowered()
    context.touch_down()
    context.throttle_now = None  # no throttle feed at all
    context.landed = True
    context.run(mission, 0.45)
    assert mission._state == PackageDeliveryState.FINAL_SET_DOWN  # the 2026 debounce, 0.5 s
    context.run(mission, 0.15)
    assert mission._state == PackageDeliveryState.GROUND_DWELL
    assert any("flight controller says landed" in text for text in context.warnings)


def test_losing_the_track_above_0_6_m_stops_the_descent_and_after_the_grace_time_the_target_is_looked_for_again():
    context, mission = setting_down()
    context.at(2.0)
    context.run(mission, 0.1)
    assert context.local_velocity_commands[-1][2] < 0.0
    context.track = None
    context.run(mission, 1.0)
    assert context.local_velocity_commands[-1][:3] == (0.0, 0.0, 0.0) and mission._state == PackageDeliveryState.FINAL_SET_DOWN
    context.run(mission, 3.2)
    assert mission._state == PackageDeliveryState.TARGET_NOT_FOUND and context.gripper_commands == []


def test_without_a_height_above_the_ground_the_vehicle_holds():
    context, mission = setting_down()
    context.height = None
    context.run(mission, 0.5)
    assert context.local_velocity_commands[-1][:3] == (0.0, 0.0, 0.0) and mission._state == PackageDeliveryState.FINAL_SET_DOWN
    assert any("no height above the ground" in text for text in context.warnings)


def test_the_hold_starts_just_above_the_height_the_mission_steers_down_to_even_if_that_is_above_0_6_m():
    """The mission takes no target under `min_projection_altitude_m` (0.75 m), judged on the flight controller's
    height; the hold starts at 0.6 m above the ground, judged on the rangefinder. Where the first reads lower than
    the second there would be a band with nothing to steer on and no hold: a "lost target" a metre off the ground.
    The hold is taken within 0.05 m of that floor, while there is still a track to take it on."""
    context, mission = setting_down()
    velocity_commands = None
    # Coming down with the flight controller's height reading 0.15 m under the rangefinder's.
    for step in range(200):
        above_ground = 1.5 - 0.006 * step
        context.at(above_ground, fc_reads_high_by=-0.15)
        context.run(mission, 0.05)
        assert mission._state == PackageDeliveryState.FINAL_SET_DOWN, above_ground
        if mission._set_down.holding:
            velocity_commands = len(context.local_velocity_commands)
            break
    assert mission._set_down.holding and 0.75 <= context.local_pose.pose.position.z <= 0.80 and context.height.value_m > 0.9
    assert any("holding the position" in text for text in context.infos)
    context.run(mission, 6.0)  # longer than the loss grace: nothing looks for the target
    assert mission._state == PackageDeliveryState.FINAL_SET_DOWN and len(context.local_velocity_commands) == velocity_commands


def test_the_hold_is_taken_only_while_centred_on_a_track_that_may_be_descended_on():
    # At the hold height but off centre: steer, do not descend, do not hold yet.
    context, mission = setting_down()
    context.at(0.58)
    context.see(0.6, 0.0)
    context.run(mission, 0.5)
    assert not mission._set_down.holding and context.global_position_commands == []
    east, _north, up, _yaw = context.local_velocity_commands[-1]
    assert up == 0.0 and east > 0.0
    # On a track that is only remembered: the same, with no descent.
    context.see(0.03, 0.0, state="COASTING", age_s=0.4)
    context.tick(mission)
    assert not mission._set_down.holding and context.local_velocity_commands[-1][2] == 0.0
    # Centred on a confirmed track again: now the hold.
    context.see(0.03, 0.0)
    context.run(mission, 0.1)
    assert mission._set_down.holding and context.global_position_commands
    # Under the floor with no hold taken (it was never centred there): no target, and after the grace the mission looks again.
    context, mission = setting_down()
    context.at(0.85, fc_reads_high_by=-0.15)
    context.run(mission, 4.5)
    assert mission._state == PackageDeliveryState.TARGET_NOT_FOUND and not mission._set_down.holding


def test_after_looking_again_from_below_the_handoff_height_the_set_down_is_entered_again():
    """A track lost in FINAL_SET_DOWN sends the mission to its recovery climb of a metre or so. From there it is
    under the handoff window, where 2026 never was: the set-down is entered as soon as the vehicle is centred."""
    context = Context(3.0)
    mission = delivery(context)
    context.run(mission, 0.1)
    assert mission._state == PackageDeliveryState.FINAL_SET_DOWN
    assert any("Entering final set-down at 3.25 m" in text for text in context.infos)
    # Without the flag the 2026 mission keeps descending in TRACK_AND_DESCEND there, as it always did.
    context = Context(3.0)
    mission = delivery(context, features=centring_tests.FLAGS_ON)
    context.run(mission, 0.1)
    assert mission._state == PackageDeliveryState.TRACK_AND_DESCEND


def test_the_set_down_needs_the_landed_state_before_it_starts_as_the_2026_column_does():
    context = Context()
    context.landing_state = False
    mission = delivery(context)
    assert context.tick(mission) == MissionStatus.FAILURE
    assert any("/mavros/extended_state is required" in text for text in context.warnings)


def test_reaching_the_ground_before_the_hold_height_is_caught_by_the_flight_controllers_landed_state():
    """On a barometer height that reads high, the ground comes while the mission still thinks it is above 0.6 m.
    It stops steering as soon as the flight controller says landed, and takes that as contact after the debounce."""
    context, mission = setting_down()
    context.at(1.6, "baro")
    context.run(mission, 0.2)
    assert context.local_velocity_commands[-1][2] < 0.0
    context.landed = True
    context.velocity = (0.0, 0.0, 0.0)
    context.run(mission, 0.45)
    assert context.local_velocity_commands[-1][:3] == (0.0, 0.0, 0.0) and mission._state == PackageDeliveryState.FINAL_SET_DOWN
    context.run(mission, 0.15)
    assert mission._state == PackageDeliveryState.GROUND_DWELL and "[package_delivery] Fake delivery complete" in context.infos
    assert any("flight controller says landed" in text for text in context.warnings)
    # A landed flag that goes away again within the debounce is not contact.
    context, mission = setting_down()
    context.at(1.6, "baro")
    context.landed = True
    context.run(mission, 0.3)
    context.landed = False
    context.run(mission, 0.5)
    assert mission._state == PackageDeliveryState.FINAL_SET_DOWN and context.local_velocity_commands[-1][2] < 0.0


def test_on_a_barometer_height_the_hold_starts_half_a_metre_earlier_and_only_landed_is_contact():
    context, mission = setting_down()
    context.at(1.2, "baro")
    context.run(mission, 0.1)
    assert not mission._set_down.holding
    assert context.local_velocity_commands[-1][2] == pytest.approx(-(0.12 + 0.3 * 0.1))  # eased as if at 0.7 m
    context.at(1.08, "baro")
    context.run(mission, 3.0)
    assert mission._set_down.holding
    # The rangefinder was not the height at the hold, so what it says now is not believed: the three signals do not release.
    context.touch_down()
    context.run(mission, 2.0)
    assert mission._state == PackageDeliveryState.FINAL_SET_DOWN and "[package_delivery] Fake delivery complete" not in context.infos
    assert any("waiting on the flight controller's landed state" in text for text in context.infos)
    context.landed = True
    context.run(mission, 0.6)
    assert mission._state == PackageDeliveryState.GROUND_DWELL


def test_twenty_seconds_in_the_hold_without_contact_or_ten_without_a_height_fail_the_mission():
    context, mission = lowered()
    statuses = context.run(mission, 16.5)  # 3 s of the hold are behind it already
    assert MissionStatus.FAILURE not in statuses
    statuses = context.run(mission, 1.0)
    assert statuses[-1] == MissionStatus.FAILURE and context.gripper_commands == []
    assert any("no contact within 20 s" in text for text in context.warnings)
    assert mission.update(context) == MissionStatus.FAILURE  # and it stays failed

    context, mission = setting_down()
    context.height = None
    statuses = context.run(mission, 9.5)
    assert MissionStatus.FAILURE not in statuses
    statuses = context.run(mission, 1.0)
    assert statuses[-1] == MissionStatus.FAILURE
    assert any("no height above the ground for 10 s" in text for text in context.warnings)


def test_nothing_is_released_above_six_feet_unless_the_flight_controller_says_landed():
    context, mission = lowered(fake_drop=False)
    context.touch_down()
    context.run(mission, 0.35)
    assert mission._state == PackageDeliveryState.GROUND_DWELL or context.gripper_commands  # contact, by the rule
    # The same contact with the height reading 3 m (a rangefinder that fell silent in the air, say).
    context, mission = lowered(fake_drop=False)
    context.touch_down()
    context.height = target_access.HeightAgl(3.0, "baro")
    context.run(mission, 1.0)
    assert context.gripper_commands == [] and mission._state == PackageDeliveryState.FINAL_SET_DOWN
    assert any("not releasing" in text for text in context.warnings)
    context.landed = True  # on the ground after all: the flight controller's word overrides the height
    context.run(mission, 1.0)
    assert context.gripper_commands == [True]
    # A barometer height counts as half a metre higher against the ceiling: 1.5 m on the barometer is not under 1.82 m.
    context, mission = lowered(fake_drop=False)
    context.touch_down()
    context.height = target_access.HeightAgl(1.5, "baro")
    context.run(mission, 1.0)
    assert context.gripper_commands == []
    context.height = target_access.HeightAgl(1.5, "rangefinder")
    context.run(mission, 1.0)
    assert context.gripper_commands == [True]


@pytest.mark.parametrize("bad", [None, 0.0, 0.9, "tall"])
def test_with_the_flag_on_and_no_usable_package_height_the_sequence_does_not_build(bad):
    """Checked when the mission is made, as the camera's calibration is: the executor stops on the ground,
    not the delivery in the air with the package on the hook."""
    config = {"package_height_m": bad} if bad is not None else {}
    base = {key: value for key, value in CONFIG.items() if key != "package_height_m"}
    with pytest.raises(ValueError, match="package_delivery.*delivery_soft_set_down.*package_height_m"):
        PackageDeliveryMission(MissionSpec(type_name="package_delivery", name="package_delivery", config={**base, **config}, base_dir=Path("."), features=dict(ON)))
    # With the flag off the key is not looked at.
    PackageDeliveryMission(MissionSpec(type_name="package_delivery", name="package_delivery", config={**base, **config}, base_dir=Path("."), features={}))


def test_every_path_through_the_set_down_logs_as_rclpy_allows():
    context, mission = setting_down(fake_drop=False)
    context.logger = OneSeverityPerLine()
    context.height = None
    context.run(mission, 2.5)
    context.at(2.0)
    context.see(0.03, 0.0, state="COASTING", age_s=0.4)
    context.run(mission, 2.5)
    context.see(0.03, 0.0)
    context.landed = True
    context.run(mission, 0.3)
    context.landed = False
    context.at(0.55)
    context.run(mission, 3.0)
    context.range = (0.02, 0.05)
    context.run(mission, 2.5)  # waiting on speed and throttle
    context.touch_down()
    context.height = target_access.HeightAgl(3.0, "baro")
    context.run(mission, 0.5)  # refused: too high and not landed
    context.at(0.02)
    context.touch_down()
    context.run(mission, 2.5)
    text = "\n".join(context.logger.lines)
    for expected in ("no height above the ground", "holding the position", "waiting on", "contact:", "not releasing", "Releasing gripper payload"):
        assert expected in text, expected
    assert mission._state == PackageDeliveryState.GUIDED_RELAUNCH
