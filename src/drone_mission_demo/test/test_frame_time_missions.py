"""Projection v2 in the drop and delivery missions (FND-05), through their real update().

With `use_frame_time_pose` on, a tracking mission finds the target's ground
point with the pose the vehicle had when the frame was taken, and steers on
that point minus where the vehicle is now. With the flag off it is the 2026
latest-pose projection, and nothing of the new path is touched.

rclpy and the message packages are the stand-ins from
test_package_drop_mission.py, so this runs without ROS.
"""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml  # noqa: F401  (the real one, before the stand-ins put an empty module in its place)

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(SRC / "argus_camera"))
import test_package_drop_mission as stand_ins  # noqa: E402  (installs the ROS stand-ins, then imports the missions)

from drone_mission_core import projection_v2 as v2  # noqa: E402
from drone_mission_core.frame_projection import FrameProjectionError, load_camera  # noqa: E402
from drone_mission_core.mission_api import MissionSpec, MissionStatus  # noqa: E402
from drone_mission_core.pose_buffer import PoseBuffer  # noqa: E402
from drone_mission_demo.missions.package_delivery_mission import PackageDeliveryMission, PackageDeliveryState  # noqa: E402
from drone_mission_demo.missions.package_drop_mission import PackageDropMission, PackageDropState  # noqa: E402

FLAG = "use_frame_time_pose"
S = 1_000_000_000
MS = 1_000_000
CAMERA, MOUNT = load_camera("sim_delivery", "sim_iris_downcam")
SIM = {"camera_mode": "sim_delivery", "airframe": "sim_iris_downcam"}
BASE = {
    "target_latitude": 47.397742,
    "target_longitude": 8.545594,
    "target_loss_grace_s": 4.0,
    "centering_deadband_m": 0.0,
    "tracking_low_pass_alpha": 1.0,  # no smoothing: the error steered on is the one projected this tick
    "centering_gain_mps_per_m": 0.25,
    "descent_rate_mps": 0.5,
    "fake_drop": True,
}
KINDS = {
    "drop": (PackageDropMission, PackageDropState.TRACK_AND_DESCEND, {"drop_altitude_m": 4.0, "drop_column_handoff_altitude_m": 8.0, "drop_hover_dwell_s": 0.0}),
    "delivery": (PackageDeliveryMission, PackageDeliveryState.TRACK_AND_DESCEND, {"landing_check_threshold_m": 6.0}),
}


def quaternion(roll_deg=0.0, pitch_deg=0.0, yaw_deg=90.0):
    roll, pitch, yaw = (math.radians(value) / 2.0 for value in (roll_deg, pitch_deg, yaw_deg))
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class Context(stand_ins.FakeContext):
    """The drop test's context, flying a known path over a known target, with the executor's pose history.

    The vehicle moves east at 2 m/s at 12 m, rolled 6 degrees, nose north.
    The clock starts at 100 s; a frame is `latency_s` old when the mission
    hears of its detection.
    """

    TARGET = (3.0, 1.0, 0.0)
    SPEED = 2.0

    def __init__(self, history=True, latency_s=0.15, roll_deg=6.0) -> None:
        super().__init__()
        self._now_ns = 100 * S
        self.latency_s, self.roll_deg = latency_s, roll_deg
        self.buffer = PoseBuffer() if history else None
        self.projectors_made = 0
        self.warnings = []
        self.logger = SimpleNamespace(info=lambda text: None, warn=self.warnings.append, error=self.warnings.append)
        self.advance(0.0)

    def pose(self, t_s):
        return ((self.SPEED * (t_s - 100.0), 0.0, 12.0), quaternion(roll_deg=self.roll_deg))

    def advance(self, seconds: float) -> None:
        """Move the clock on, feed the pose history at 30 Hz, set the latest pose, and publish a detection `latency_s` old."""
        start, self._now_ns = self._now_ns, self._now_ns + int(seconds * 1e9)
        if self.buffer is not None:
            first = -(-(start - 100 * S) // (33 * MS)) if start > 100 * S else -15  # history from 0.5 s before the start
            index = first
            while 100 * S + index * 33 * MS <= self._now_ns:
                stamp = 100 * S + index * 33 * MS
                position, orientation = self.pose(stamp / S)
                self.buffer.add_pose(stamp, position, orientation)
                self.buffer.add_velocity(stamp, (self.SPEED, 0.0, 0.0))
                index += 1
        position, orientation = self.pose(self._now_ns / S)
        self.local_pose = SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=position[0], y=position[1], z=position[2]),
                orientation=SimpleNamespace(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]),
            )
        )
        self.see_target()

    def see_target(self, image_size=(640, 360)) -> None:
        """The detection of TARGET in the frame taken `latency_s` ago, stamped with that frame's time."""
        frame_ns = self._now_ns - int(self.latency_s * 1e9)
        position, orientation = self.pose(frame_ns / S)
        u, v = v2.pixel_of_ground_point(CAMERA, MOUNT, self.TARGET, position, orientation)
        self.image_size = image_size
        self.target_detection = SimpleNamespace(
            header=SimpleNamespace(stamp=stand_ins.FakeTime(frame_ns)),
            point=SimpleNamespace(x=u / CAMERA.width * 2.0 - 1.0, y=v / CAMERA.height * 2.0 - 1.0, z=900.0),
        )

    def ground_projector(self, camera, mount):
        self.projectors_made += 1
        return None if self.buffer is None else v2.GroundProjector(camera, mount, self.buffer.pose_at)

    def error_to_target(self):
        position = self.local_pose.pose.position
        return (self.TARGET[0] - position.x, self.TARGET[1] - position.y)

    def tick(self, mission):
        self.advance(0.05)
        return mission.update(self)


def make(kind, flag, context, **config):
    mission_class, state, extra = KINDS[kind]
    spec = MissionSpec(type_name=f"package_{kind}", name=f"package_{kind}", config={**BASE, **extra, **config}, base_dir=Path("."), features={FLAG: flag})
    mission = mission_class(spec)
    mission.on_enter(context)
    context.see_target()  # on_enter clears the tracking state
    mission._state = state
    return mission


@pytest.mark.parametrize("kind", list(KINDS))
def test_flag_on_the_mission_steers_on_where_the_target_is_not_where_the_old_frame_showed_it(kind):
    context = Context()
    mission = make(kind, True, context, **SIM)
    assert context.tick(mission) == MissionStatus.RUNNING
    east, north, _up, _yaw = context.local_velocity_commands[-1]
    wanted = context.error_to_target()
    assert (east, north) == pytest.approx((0.25 * wanted[0], 0.25 * wanted[1]), abs=0.25 * 0.01)  # within a centimetre of the truth
    assert context.projectors_made == 1 and context.warnings == []
    for _ in range(10):
        context.tick(mission)
    assert context.projectors_made == 1  # made once, kept


@pytest.mark.parametrize("kind", list(KINDS))
def test_flag_off_is_the_2026_latest_pose_projection_and_the_new_path_is_never_touched(kind):
    class Before(Context):
        def ground_projector(self, camera, mount):
            raise AssertionError("projection v2 was used with the flag off")

    context = Before()
    mission = make(kind, False, context)  # no camera_mode or airframe needed
    assert mission._frame_camera is None
    assert context.tick(mission) == MissionStatus.RUNNING
    east, north, _up, _yaw = context.local_velocity_commands[-1]
    wanted = context.error_to_target()
    # 2026 projects the 150 ms old pixel with the newest pose, from the vehicle's own position: the vehicle has moved 0.3 m since.
    assert math.hypot(east / 0.25 - wanted[0], north / 0.25 - wanted[1]) > 0.2


@pytest.mark.parametrize("kind", list(KINDS))
def test_flag_on_with_no_pose_for_the_frames_time_there_is_no_tracking_and_no_descent(kind):
    class NoPoses(Context):
        def ground_projector(self, camera, mount):
            return v2.GroundProjector(camera, mount, PoseBuffer().pose_at)  # a pose history that has heard nothing

    context = NoPoses()
    mission = make(kind, True, context, **SIM)
    for _ in range(4):
        assert context.tick(mission) == MissionStatus.RUNNING
    east, north, up, _yaw = context.local_velocity_commands[-1]
    assert (east, north) == (0.0, 0.0) and up >= 0.0  # holds: never steers or descends on a pose it does not have
    assert len(context.warnings) >= 1 and "no pose for the frame's capture time" in context.warnings[0]
    assert sum("no ground projection" in line for line in context.warnings) == 1  # said once, not every tick


@pytest.mark.parametrize("kind", list(KINDS))
def test_flag_on_a_detector_that_has_stopped_cannot_lead_the_mission_onto_a_remembered_point(kind):
    class Frozen(Context):
        frozen = None

        def see_target(self, image_size=(640, 360)):
            if self.frozen is None:
                super().see_target(image_size)
            else:
                self.image_size, self.target_detection = image_size, self.frozen

    context = Frozen()
    mission = make(kind, True, context, **SIM)
    context.tick(mission)
    assert context.local_velocity_commands[-1][:2] != (0.0, 0.0)  # tracking a live detector
    context.frozen = context.target_detection  # the detector stops: the same message, tick after tick
    steered = []
    for _ in range(30):  # 1.5 s, still inside the 2 s target timeout
        context.tick(mission)
        steered.append(context.local_velocity_commands[-1])
    # For the first half second the remembered point is still steered on; after it there is no tracking solution.
    age_when_dropped = next(index for index, command in enumerate(steered) if command[:2] == (0.0, 0.0))
    assert 6 <= age_when_dropped <= 8  # the frame was 0.15 s old when it froze; 0.5 s is reached about 7 ticks later
    assert all(command[:2] == (0.0, 0.0) and command[2] >= 0.0 for command in steered[age_when_dropped:])  # holds, no descent
    assert any("ms old (limit 500 ms)" in line for line in context.warnings)
    assert mission._frame_time_max_age_s == 0.5


def test_the_frame_age_limit_is_a_config_key_read_only_with_the_flag_on():
    context = Context(latency_s=0.3)
    strict = make("delivery", True, context, frame_time_max_age_s=0.2, **SIM)
    context.advance(0.05)
    assert strict._get_tracking_solution(context) is None and "limit 200 ms" in context.warnings[0]
    relaxed = make("delivery", True, Context(latency_s=0.3), **SIM)
    assert relaxed._frame_time_max_age_s == 0.5
    off = make("delivery", False, Context(latency_s=0.3), frame_time_max_age_s=0.01)
    assert off._frame_time_max_age_s == 0.5  # not read with the flag off, and not used


@pytest.mark.parametrize("kind", list(KINDS))
def test_flag_on_an_image_of_another_size_than_the_calibration_is_not_projected(kind):
    context = Context()
    mission = make(kind, True, context, **SIM)
    context.advance(0.05)
    context.see_target(image_size=(1280, 720))
    mission.update(context)
    east, north, up, _yaw = context.local_velocity_commands[-1]
    assert (east, north) == (0.0, 0.0) and up >= 0.0
    assert "640 x 360" in context.warnings[0] and "not scaled" in context.warnings[0]


def test_flag_on_without_a_pose_history_in_the_executor_there_is_no_tracking():
    context = Context(history=False)
    mission = make("delivery", True, context, **SIM)
    context.tick(mission)
    assert context.local_velocity_commands[-1][:2] == (0.0, 0.0) and "keeps no pose history" in context.warnings[0]


def test_the_two_2026_limits_still_apply_with_the_flag_on():
    low = Context()
    low.pose = lambda t_s: ((2.0 * (t_s - 100.0), 0.0, 0.5), quaternion())  # under min_projection_altitude_m (0.75)
    mission = make("delivery", True, low, **SIM)
    low.advance(0.05)
    assert mission._get_tracking_solution(low) is None
    far = Context()
    mission = make("delivery", True, far, max_projection_distance_m=1.0, **SIM)  # the target is about 3 m away
    far.advance(0.05)
    assert mission._get_tracking_solution(far) is None and far.warnings == []
    near = Context()
    mission = make("delivery", True, near, max_projection_distance_m=25.0, **SIM)
    near.advance(0.05)
    assert mission._get_tracking_solution(near) is not None


@pytest.mark.parametrize("kind", list(KINDS))
def test_flag_on_a_mission_that_does_not_name_its_camera_is_refused_when_it_is_built(kind):
    mission_class, _state, extra = KINDS[kind]

    def build(**config):
        return mission_class(MissionSpec(type_name=f"package_{kind}", name=f"package_{kind}", config={**BASE, **extra, **config}, base_dir=Path("."), features={FLAG: True}))

    with pytest.raises(FrameProjectionError, match="camera_mode and airframe"):
        build()  # before on_enter: the executor builds every mission at start-up, on the ground
    with pytest.raises(FrameProjectionError, match="not verified"):
        build(camera_mode="imx519_4k", airframe="argus_2027")  # nobody has measured the aircraft's mount yet
    with pytest.raises(FrameProjectionError, match="not calibrated"):
        build(camera_mode="imx519_720p60", airframe="sim_iris_downcam")
    assert build(**SIM)._frame_camera[0].name == "sim_delivery"


def test_the_mission_side_change_is_behind_the_flag_and_leaves_the_2026_projection_call_as_it_was():
    source = (SRC / "drone_mission_demo" / "drone_mission_demo" / "missions" / "red_bullseye_mission_base.py").read_text()
    assert "if get_feature_flag(spec.features, FRAME_TIME_FLAG):" in source and "if self._frame_camera is not None:" in source
    assert source.count("project_normalized_detection_to_ground_offset(") == 1  # still called, in the other branch
    header = source.split("class RedBullseyeMissionBase")[0]
    assert "projection_v2" not in header and "frame_projection" not in header  # numpy and argus_camera come in only with the flag on
