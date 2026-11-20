"""Smart re-acquisition (CEN-06), through the drop and delivery missions' real update().

With `smart_reacquire` on, a mission whose track is LOST climbs to the
height from which the camera sees three sigma of the track plus 2.5 m, over
the track's last estimate, and holds there for at most 10 s until a track is
CONFIRMED again. With the flag off the recovery is the 2026 one: a grace
time, then a climb of `not_found_ascent_m` from where the vehicle is.

rclpy and the message packages are the stand-ins from
test_package_drop_mission.py; the context is test_centring_missions.py's.
"""

import sys
from pathlib import Path

import pytest
import yaml  # noqa: F401  (the real one, before the stand-ins put an empty module in its place)

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_centring_missions as centring_tests  # noqa: E402  (installs the ROS stand-ins)
from test_drop_release_gates import OneSeverityPerLine  # noqa: E402

from drone_mission_core import reacquire, target_access  # noqa: E402
from drone_mission_core.mission_api import MissionSpec, MissionStatus  # noqa: E402
from drone_mission_demo.missions.package_delivery_mission import PackageDeliveryMission, PackageDeliveryState  # noqa: E402
from drone_mission_demo.missions.package_drop_mission import PackageDropMission, PackageDropState  # noqa: E402

FLAG = "smart_reacquire"
ON = {**centring_tests.FLAGS_ON, FLAG: True}
CONFIG = {
    **centring_tests.BASE,
    "drop_altitude_m": 4.0,
    "drop_column_handoff_altitude_m": 0.0,
    "drop_hover_dwell_s": 0.0,
    "not_found_ascent_m": 1.0,
    "max_recovery_altitude_m": 40.0,
    "max_recovery_attempts": 4,
    "recovery_alt_tolerance_m": 0.5,
    "centering_dwell_s": 1.0,
}
TARGET = (2.0, 1.0)  # where the track is, east and north
K = 180.0 / 389.165  # half the sim camera's footprint per metre of height, on its short side


class Context(centring_tests.Context):
    def __init__(self, up_m=3.0):
        super().__init__(vehicle=(TARGET[0] - 0.05, TARGET[1], up_m))
        self.infos = []
        self.logger.info = self.infos.append
        self.local_position_commands = []
        self.see(*TARGET)

    def a_track(self, east, north, state="CONFIRMED", age_s=0.05, track_id=1):
        track = super().a_track(east, north, state, age_s, track_id)
        return target_access.PrimaryTarget(**{**track.__dict__, "sigma_m": 0.2})

    def may_descend(self, class_id, *, near=None, track_id=None):
        return target_access.descent_permission(self.track if track_id is None else self._by_id(track_id))

    def set_local_position_setpoint(self, east_m, north_m, up_m, yaw_deg=90.0):
        self.local_position_commands.append((east_m, north_m, up_m, yaw_deg))

    def lose(self, state="LOST", age_s=2.5, sigma_m=0.2):
        self.track = target_access.PrimaryTarget(**{**self.a_track(*TARGET, state=state, age_s=age_s).__dict__, "sigma_m": sigma_m})

    def go_to(self, east, north, up):
        position = self.local_pose.pose.position
        position.x, position.y, position.z = east, north, up

    def run(self, mission, seconds, fresh=False):
        statuses = []
        for _ in range(int(round(seconds / 0.05))):
            if fresh and self.track is not None:
                self.track = target_access.PrimaryTarget(**{**self.track.__dict__, "for_ns": self._now_ns + 80_000_000})
            statuses.append(self.tick(mission))
        return statuses


def drop(context, features=ON, **config):
    spec = MissionSpec(type_name="package_drop", name="package_drop", config={**CONFIG, **config}, base_dir=Path("."), features=dict(features))
    mission = PackageDropMission(spec)
    mission.on_enter(context)
    mission._state = PackageDropState.TRACK_AND_DESCEND
    return mission


def looking(up_m=3.0, sigma_m=0.2, **config):
    """A drop that was descending on its track at `up_m` and has just lost it: in TARGET_NOT_FOUND, the climb commanded."""
    context = Context(up_m)
    mission = drop(context, **config)
    context.run(mission, 0.2, fresh=True)
    assert mission._state == PackageDropState.TRACK_AND_DESCEND and context.local_velocity_commands[-1][2] < 0.0
    context.lose(sigma_m=sigma_m)
    context.run(mission, 0.1)
    assert mission._state == PackageDropState.TARGET_NOT_FOUND
    return context, mission


def test_the_flag_is_declared_off_in_every_sequence_with_a_vision_mission():
    import yaml as real_yaml

    sequences = Path(__file__).resolve().parents[1] / "config" / "sequences"
    declaring = []
    for path in sorted(sequences.glob("*.yaml")):
        root = real_yaml.safe_load(path.read_text())["mission_sequence"]
        if any(entry["type"] in ("package_drop", "package_delivery") for entry in root["missions"]):
            assert root["features"].get(FLAG) is False, path.name
            declaring.append(path.name)
    assert len(declaring) == 8


def test_with_the_flag_off_a_lost_track_is_the_2026_recovery():
    context = Context(3.0)
    mission = drop(context, features=centring_tests.FLAGS_ON)
    context.run(mission, 0.2, fresh=True)
    context.lose()
    context.run(mission, 3.9)
    assert mission._state == PackageDropState.TRACK_AND_DESCEND  # the 4 s grace
    context.run(mission, 0.3)
    assert mission._state == PackageDropState.TARGET_NOT_FOUND
    latitude, longitude, altitude, *_ = context.global_position_commands[-1]
    assert altitude == pytest.approx(4.0) and context.local_position_commands == []  # a metre up from where it is, on a GPS point
    assert mission._smart_reacquire is None


def test_a_lost_track_sends_the_vehicle_at_once_to_the_height_that_shows_three_sigma_and_the_margin_over_its_last_estimate():
    context, mission = looking(up_m=3.0, sigma_m=0.2)
    height = (3.0 * 0.2 + 2.5) / K  # 6.70 m
    assert context.local_position_commands[-1][:3] == pytest.approx((TARGET[0], TARGET[1], height))
    assert context.global_position_commands == []  # not the 2026 climb on a GPS point
    assert any(f"looking again from {height:.1f} m" in text and "attempt 1/4" in text for text in context.infos)
    assert not any("holding for 4.0 s" in text for text in context.warnings)  # the estimator's LOST is the grace: no second one
    # A wider track looks from higher.
    wide, _ = looking(up_m=3.0, sigma_m=1.0)
    assert wide.local_position_commands[-1][2] == pytest.approx((3.0 + 2.5) / K)


def test_a_track_that_is_only_coasting_is_not_looked_for():
    context = Context(3.0)
    mission = drop(context)
    context.run(mission, 0.2, fresh=True)
    context.lose(state="COASTING", age_s=0.4)
    context.run(mission, 1.0)
    assert mission._state == PackageDropState.TRACK_AND_DESCEND and context.local_position_commands == []
    assert context.local_velocity_commands[-1][2] == 0.0  # steering, not descending


def test_at_the_height_it_holds_until_a_track_is_confirmed_again_and_then_the_mission_acquires_it():
    context, mission = looking()
    height = context.local_position_commands[-1][2]
    # Confirmed again while still climbing: the mission goes on to the height first.
    context.go_to(TARGET[0], TARGET[1], 4.5)
    context.see(*TARGET)
    context.run(mission, 0.5, fresh=True)
    assert mission._state == PackageDropState.TARGET_NOT_FOUND
    context.lose()
    context.go_to(TARGET[0], TARGET[1], height - 0.05)
    context.run(mission, 5.0)
    assert mission._state == PackageDropState.TARGET_NOT_FOUND
    assert any("holding up to 10 s" in text for text in context.infos)
    assert context.local_position_commands[-1][:3] == pytest.approx((TARGET[0], TARGET[1], height))
    # A track that is there but not confirmed, or confirmed but not seen lately, is not "again".
    context.see(*TARGET, state="TENTATIVE")
    context.run(mission, 0.5, fresh=True)
    context.see(*TARGET, state="CONFIRMED", age_s=0.8)
    context.run(mission, 0.5, fresh=True)
    assert mission._state == PackageDropState.TARGET_NOT_FOUND
    context.see(*TARGET, track_id=7)  # found again as a new track: it is taken
    context.run(mission, 0.1, fresh=True)
    assert mission._state == PackageDropState.ACQUIRE_TARGET
    assert any("CONFIRMED again" in text for text in context.infos)
    context.run(mission, 1.3, fresh=True)
    assert mission._state == PackageDropState.TRACK_AND_DESCEND and mission._estimator_centring.track_id == 7


def test_ten_seconds_at_the_height_without_a_confirmed_track_fail_the_mission():
    context, mission = looking()
    height = context.local_position_commands[-1][2]
    context.go_to(TARGET[0], TARGET[1], height)
    statuses = context.run(mission, 9.8)
    assert MissionStatus.FAILURE not in statuses
    statuses = context.run(mission, 0.5)
    assert statuses[-1] == MissionStatus.FAILURE
    assert any("not CONFIRMED again within 10 s" in text for text in context.warnings)


def test_the_ten_seconds_count_from_arriving_over_the_estimate_not_from_the_loss():
    context, mission = looking()
    height = context.local_position_commands[-1][2]
    statuses = context.run(mission, 12.0)  # still at 3 m: the climb is not there yet
    assert MissionStatus.FAILURE not in statuses
    context.go_to(TARGET[0] + 3.0, TARGET[1], height)  # at the height, but three metres to the side
    statuses = context.run(mission, 12.0)
    assert MissionStatus.FAILURE not in statuses and not any("holding up to 10 s" in text for text in context.infos)
    # A climb that never arrives is given up: 20 s and the climb at 0.5 m/s.
    statuses = context.run(mission, reacquire.climb_limit_s(height - 3.0) - 24.0 + 0.5)
    assert statuses[-1] == MissionStatus.FAILURE
    assert any("did not reach" in text for text in context.warnings)


def test_already_above_the_height_the_vehicle_stays_where_it_is_and_the_ceiling_is_kept():
    context, mission = looking(up_m=12.0)
    assert context.local_position_commands[-1][2] == pytest.approx(12.0)  # 6.7 m would be a descent: it stays
    context, mission = looking(up_m=3.0, sigma_m=8.0, max_recovery_altitude_m=20.0)
    assert context.local_position_commands[-1][2] == pytest.approx(20.0)
    assert any("ceiling" in text for text in context.infos)


def test_each_look_counts_as_a_recovery_attempt_and_the_last_one_allowed_is_the_last():
    context, mission = looking()
    mission._recovery_attempts = 4
    mission._recovery_target_altitude = None  # (as after a transition: the next loss)
    assert context.tick(mission) == MissionStatus.FAILURE
    assert any("exceeded 4 attempts" in text for text in context.warnings)


def test_with_nothing_ever_seen_the_recovery_is_the_2026_climb():
    context = Context(15.0)
    context.track = None
    mission = drop(context)
    mission._state = PackageDropState.ACQUIRE_TARGET
    context.run(mission, 4.2)  # nothing at the GPS point: the 2026 grace
    assert mission._state == PackageDropState.TARGET_NOT_FOUND
    context.run(mission, 0.1)
    assert context.local_position_commands == [] and context.global_position_commands[-1][2] == pytest.approx(16.0)


def test_the_delivery_looks_again_the_same_way():
    context = Context(3.0)
    spec = MissionSpec(
        type_name="package_delivery", name="package_delivery", config={**CONFIG, "landing_check_threshold_m": 1.0}, base_dir=Path("."), features=dict(ON)
    )
    mission = PackageDeliveryMission(spec)
    mission.on_enter(context)
    mission._state = PackageDeliveryState.TRACK_AND_DESCEND
    context.run(mission, 0.2, fresh=True)
    context.lose()
    context.run(mission, 0.1)
    assert mission._state == PackageDeliveryState.TARGET_NOT_FOUND
    assert context.local_position_commands[-1][:3] == pytest.approx((TARGET[0], TARGET[1], (0.6 + 2.5) / K))


def test_every_path_through_the_look_logs_as_rclpy_allows():
    context = Context(3.0)
    mission = drop(context, max_recovery_altitude_m=6.0)
    context.logger = OneSeverityPerLine()
    context.run(mission, 0.2, fresh=True)
    context.lose()
    context.run(mission, 0.5)
    context.go_to(TARGET[0], TARGET[1], 6.0)
    context.run(mission, 10.5)
    text = "\n".join(context.logger.lines)
    for expected in ("looking again from", "ceiling", "holding up to 10 s", "not CONFIRMED again within 10 s"):
        assert expected in text, expected


def test_the_height_is_above_the_ground_under_the_vehicle_not_above_home():
    """The footprint is on the ground under the vehicle. Where that ground is a metre above home, a height of 6.7 m
    above home shows a metre less than was asked for: the climb is by what is missing above the ground."""
    context = Context(3.0)
    context.height = target_access.HeightAgl(2.0, "rangefinder")  # 3 m above home, 2 m above this ground
    mission = drop(context)
    context.run(mission, 0.2, fresh=True)
    context.lose()
    context.run(mission, 0.1)
    above_ground = (3.0 * 0.2 + 2.5) / K
    assert context.local_position_commands[-1][2] == pytest.approx(3.0 + (above_ground - 2.0))
    assert any(f"looking again from {above_ground:.1f} m above the ground" in text for text in context.infos)


def test_the_look_is_taken_from_the_height_itself_not_from_half_a_metre_under_it():
    context, mission = looking()
    height = context.local_position_commands[-1][2]
    context.go_to(TARGET[0], TARGET[1], height - 0.4)  # inside the mission's 0.5 m climb tolerance, but 0.4 m short
    context.see(*TARGET)
    context.run(mission, 1.0, fresh=True)
    assert mission._state == PackageDropState.TARGET_NOT_FOUND and not any("holding up to 10 s" in text for text in context.infos)
    context.go_to(TARGET[0], TARGET[1], height - 0.05)
    context.run(mission, 0.1, fresh=True)
    assert mission._state == PackageDropState.ACQUIRE_TARGET


def test_the_gated_drops_first_timeout_is_one_look_from_the_height_and_its_second_is_not_another():
    """CEN-04: 25 s in FINAL_ALIGN without the gates is "one re-acquisition (CEN-06)". The track is not lost
    then: the look is still the climb to the height over the estimate, and the track confirmed there."""
    import test_drop_release_gates as gates

    class Both(gates.Context):
        def __init__(self):
            super().__init__(2.4)
            self.local_position_commands = []

        def set_local_position_setpoint(self, east_m, north_m, up_m, yaw_deg=90.0):
            self.local_position_commands.append((east_m, north_m, up_m, yaw_deg))

    context = Both()
    mission = gates.drop(context, features={**gates.ON, FLAG: True}, not_found_ascent_m=1.0, max_recovery_attempts=4, recovery_alt_tolerance_m=0.5)
    context.tick(mission)
    assert mission._state == PackageDropState.FINAL_ALIGN
    context.velocity = (0.3, 0.0, 0.0)  # the speed gate never holds
    context.run(mission, 25.2)
    assert mission._state == PackageDropState.TARGET_NOT_FOUND
    context.run(mission, 0.1)
    east, north, up, _yaw = context.local_position_commands[-1]
    track = context.track
    assert (east, north) == pytest.approx(track.position[:2]) and up == pytest.approx((3.0 * 0.04 + 2.5) / K)
    assert context.global_position_commands == []  # not the 2026 one-metre climb
    context.run(mission, 3.0)
    assert mission._state == PackageDropState.TARGET_NOT_FOUND  # confirmed all along, but not yet at the height
    context.local_pose.pose.position.x, context.local_pose.pose.position.y = track.position[:2]
    context.at(up)
    context.run(mission, 0.1)
    assert mission._state == PackageDropState.ACQUIRE_TARGET
    context.run(mission, 1.5)
    assert mission._state in (PackageDropState.TRACK_AND_DESCEND, PackageDropState.FINAL_ALIGN)
    context.at(2.4)
    context.run(mission, 0.2)
    assert mission._state == PackageDropState.FINAL_ALIGN
    looks = len(context.local_position_commands)
    statuses = context.run(mission, 25.3)  # the second 25 s: the last resort, not a second look
    assert MissionStatus.SUCCESS in statuses and len(context.local_position_commands) == looks


def test_an_estimate_far_from_the_vehicle_is_looked_for_from_higher_not_flown_to():
    context = Context(3.0)
    mission = drop(context)
    context.run(mission, 0.2, fresh=True)
    context.go_to(TARGET[0] - 6.0, TARGET[1], 3.0)  # six metres west of the track when it is lost
    context.lose()
    context.run(mission, 0.1)
    assert mission._state == PackageDropState.TARGET_NOT_FOUND
    east, north, up, _yaw = context.local_position_commands[-1]
    assert (east, north) == pytest.approx((TARGET[0] - 3.5, TARGET[1]))  # 2.5 m toward it, no more
    assert up == pytest.approx((3.0 * 0.2 + 2.5 + 3.5) / K)  # and the 3.5 m left over are in the footprint


def test_a_look_with_no_track_being_flown_to_uses_what_the_estimator_still_has_and_nothing_remembered():
    """After a look the centring has let its track go. If the mission is back in recovery before it took another,
    the estimate is the estimator's own, as it is now (moved by a position reset, its sigma grown): the class's
    primary track, even LOST. With no track at all, the 2026 climb."""
    context, mission = looking()
    assert mission._estimator_centring.track_id is None  # let go at the start of the look
    mission._recovery_target_altitude = None  # (as after a transition: the next spell in TARGET_NOT_FOUND)
    context.lose(sigma_m=0.5)
    context.track = target_access.PrimaryTarget(**{**context.track.__dict__, "position": (TARGET[0] + 1.0, TARGET[1], 0.0)})
    context.run(mission, 0.1)
    east, north, up, _yaw = context.local_position_commands[-1]
    assert (east, north) == pytest.approx((TARGET[0] + 1.0, TARGET[1])) and up == pytest.approx((1.5 + 2.5) / K)
    assert not hasattr(mission._estimator_centring, "last_target")

    context, mission = looking()
    mission._recovery_target_altitude = None
    context.track = None
    commands = len(context.local_position_commands)
    context.run(mission, 0.1)
    assert len(context.local_position_commands) == commands and context.global_position_commands[-1][2] == pytest.approx(4.0)


def test_a_vehicle_that_settles_a_little_low_takes_its_look_from_there_when_the_climbs_time_is_up():
    """An aircraft that holds 0.3 m under the setpoint never comes within 0.1 m of the height. When the climb's time
    is up and it is within the mission's own climb tolerance, the look is taken from where it is, not failed."""
    context, mission = looking()
    height = context.local_position_commands[-1][2]
    context.go_to(TARGET[0], TARGET[1], height - 0.3)
    limit = reacquire.climb_limit_s(height - 3.0)
    statuses = context.run(mission, limit - 0.5)
    assert MissionStatus.FAILURE not in statuses and not any("holding up to 10 s" in text for text in context.infos)
    statuses = context.run(mission, 1.0)
    assert MissionStatus.FAILURE not in statuses and any("short of the height" in text for text in context.warnings)
    context.see(*TARGET)
    context.run(mission, 0.1, fresh=True)
    assert mission._state == PackageDropState.ACQUIRE_TARGET
