"""The stream keeper's view of the real missions (PLT-06): which states are release or actuation states.

The keeper sends nothing through MAVROS's command queue while a mission is in
such a state. It learns the state from the mission executor's log, so this
test drives the real missions and checks that, at the moment each one sends
its command, the keeper's watch (fed the mission's own log lines) is active.
If a mission's log line or state names change, this fails.

rclpy and the message packages are the stand-ins from test_package_drop_mission.py.
"""

import enum
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "drone_utils"))
import test_package_drop_mission as stand_ins  # noqa: E402  (installs the ROS stand-ins)

from drone_mission_core.mission_api import MissionSpec  # noqa: E402
from drone_mission_demo.missions import gps_waypoint_mission, local_waypoint_mission, package_delivery_mission, package_drop_mission, rtl_mission, takeoff_mission  # noqa: E402
from drone_mission_demo.missions.package_delivery_mission import PackageDeliveryMission, PackageDeliveryState  # noqa: E402
from drone_mission_demo.missions.package_drop_mission import PackageDropMission, PackageDropState  # noqa: E402
from drone_mission_demo.missions.takeoff_mission import TakeoffMission, TakeoffState  # noqa: E402
from drone_utils.stream_keeper import ActuationWatch, is_actuation_state  # noqa: E402

MISSION_MODULES = [gps_waypoint_mission, local_waypoint_mission, package_delivery_mission, package_drop_mission, rtl_mission, takeoff_mission]


class Context(stand_ins.FakeContext):
    """The drop test's context, with a log the watch reads and a record of when each command went out."""

    def __init__(self) -> None:
        super().__init__()
        self.watch = ActuationWatch()
        self.logger = SimpleNamespace(info=self.watch.on_log, warn=self.watch.on_log, error=self.watch.on_log)
        self.sent = []  # (command, watch.active when it was sent)
        self.landed = True
        self.mavros_state = SimpleNamespace(connected=True, armed=True)

    def command_sprayer(self, *, enable, done_callback=None):
        self.sent.append(("sprayer", self.watch.active))
        return stand_ins.FakeFuture()

    def command_gripper(self, *, release, done_callback=None):
        self.sent.append(("gripper", self.watch.active))
        return stand_ins.FakeFuture()

    def request_takeoff(self, altitude_m, done_callback=None):
        self.sent.append(("takeoff", self.watch.active))
        return stand_ins.FakeFuture()

    def takeoff_service_ready(self):
        return True

    def landing_state_available(self):
        return True

    def vehicle_is_landed(self):
        return self.landed


def spec(kind, **config):
    base = {"target_latitude": 47.397742, "target_longitude": 8.545594, "fake_drop": False, "drop_hover_dwell_s": 1.0, "delivery_dwell_s": 1.0}
    return MissionSpec(type_name=kind, name=kind, config={**base, **config}, base_dir=Path("."))


def run(mission, context, ticks=200):
    for _ in range(ticks):
        context.advance(0.05)
        mission.update(context)
        if context.sent:
            return


def test_exactly_these_states_of_the_shipped_missions_are_release_or_actuation_states():
    states = set()
    for module in MISSION_MODULES:
        for value in vars(module).values():
            if isinstance(value, type) and issubclass(value, enum.Enum) and value.__module__ == module.__name__:
                states |= {f"{value.__name__}.{member.name}" for member in value if is_actuation_state(member.name)}
    assert states == {
        "PackageDropState.FINAL_FIXED_DROP_COLUMN",  # the column that leads into the release
        "PackageDropState.FINAL_ALIGN",  # 2027, drop_release_gates: the release follows on the tick the gates hold (CEN-04)
        "PackageDropState.DROP_PAYLOAD",  # the sprayer release
        "PackageDeliveryState.FINAL_FIXED_COLUMN_DESCENT",  # the column that leads into the release
        "PackageDeliveryState.GROUND_DWELL",  # the gripper release
        "TakeoffState.TAKING_OFF",  # the takeoff service arms
        "TakeoffState.WAITING_FOR_ALTITUDE",  # and sends the takeoff command a few seconds later
    }


def test_the_drop_sends_its_release_while_the_watch_is_active_and_the_watch_lets_go_after():
    context = Context()
    mission = PackageDropMission(spec("package_drop"))
    mission.on_enter(context)
    mission._transition_to(PackageDropState.TRACK_AND_DESCEND, context)
    assert not context.watch.active
    mission._transition_to(PackageDropState.FINAL_FIXED_DROP_COLUMN, context)
    assert context.watch.active  # already quiet in the column, well before the release
    mission._transition_to(PackageDropState.DROP_PAYLOAD, context)
    assert context.watch.active and context.watch.state == "package_drop: DROP_PAYLOAD"
    run(mission, context)
    assert context.sent == [("sprayer", True)]
    mission._transition_to(PackageDropState.COMPLETE, context)
    assert not context.watch.active


def test_the_delivery_sends_its_release_while_the_watch_is_active():
    context = Context()
    mission = PackageDeliveryMission(spec("package_delivery"))
    mission.on_enter(context)
    mission._transition_to(PackageDeliveryState.TRACK_AND_DESCEND, context)
    assert not context.watch.active
    mission._transition_to(PackageDeliveryState.FINAL_FIXED_COLUMN_DESCENT, context)
    assert context.watch.active  # the release comes one tick into GROUND_DWELL, so the hold starts with the column
    mission._transition_to(PackageDeliveryState.GROUND_DWELL, context)
    assert context.watch.active
    run(mission, context)
    assert context.sent == [("gripper", True)]
    mission._transition_to(PackageDeliveryState.GUIDED_RELAUNCH, context)
    assert not context.watch.active


def test_the_takeoff_request_goes_out_while_the_watch_is_active_and_it_stays_active_until_the_climb_is_done():
    context = Context()
    context.mavros_state = SimpleNamespace(connected=True, armed=False)
    context.local_pose.pose.position.z = 0.0
    mission = TakeoffMission(MissionSpec(type_name="takeoff", name="takeoff", config={"target_altitude_m": 15.0}, base_dir=Path(".")))
    mission.on_enter(context)
    run(mission, context)
    assert context.sent == [("takeoff", True)] and mission._state == TakeoffState.TAKING_OFF
    context.mavros_state = SimpleNamespace(connected=True, armed=True)
    mission.update(context)
    assert mission._state == TakeoffState.WAITING_FOR_ALTITUDE and context.watch.active  # the takeoff command is still to come
    context.local_pose.pose.position.z = 14.5
    mission.update(context)
    assert mission._state == TakeoffState.COMPLETE and not context.watch.active


def test_a_fake_drop_still_passes_through_the_release_state():
    context = Context()
    mission = PackageDropMission(spec("package_drop", fake_drop=True))
    mission.on_enter(context)
    mission._transition_to(PackageDropState.DROP_PAYLOAD, context)
    assert context.watch.active
    for _ in range(60):
        context.advance(0.05)
        mission.update(context)
    assert context.sent == [] and not context.watch.active  # no command, and the hold ended with the state
