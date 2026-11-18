"""Projection v2 (FND-05): pixel to ground at the frame's capture time. No ROS."""

import ast
import math
import sys
from pathlib import Path

import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC / "argus_camera"))

from argus_camera import calibration, modes, robot_description  # noqa: E402
from drone_mission_core import projection_v2 as v2  # noqa: E402
from drone_mission_core.pose_buffer import PoseBuffer  # noqa: E402

CONFIG = SRC / "argus_camera" / "config"
MODES = modes.load_modes(CONFIG / "modes.yaml")
AIRFRAMES = robot_description.load(CONFIG / "robot.yaml")
S = 1_000_000_000
MS = 1_000_000


def camera(mode_name="imx519_720p60"):
    return v2.CameraModel.from_calibration(calibration.for_mode(MODES[mode_name], CONFIG), name=mode_name)


def mount(airframe="argus_2027"):
    return v2.CameraMount.from_description(robot_description.get_airframe(AIRFRAMES, airframe, allow_unverified=True).cameras["down"])


def quaternion(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    """Body (forward, left, up) to world (east, north, up): yaw about up, then pitch about left, then roll about forward."""
    roll, pitch, yaw = (math.radians(value) / 2.0 for value in (roll_deg, pitch_deg, yaw_deg))
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def turned(orientation, axis, angle):
    """The orientation after a further small turn of `angle` radians about a world axis (0 east, 1 north, 2 up)."""
    half = angle / 2.0
    w = [0.0, 0.0, 0.0]
    w[axis] = math.sin(half)
    ax, ay, az, aw = w[0], w[1], w[2], math.cos(half)
    bx, by, bz, bw = orientation
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def east_north(point):
    return np.array([point.east_m, point.north_m])


# --- the three objects ---


def test_the_camera_model_is_the_modes_own_calibration_not_one_scaled_from_4k():
    hd, uhd = camera("imx519_720p60"), camera("imx519_4k")
    hd_file, uhd_file = (calibration.for_mode(MODES[name], CONFIG) for name in ("imx519_720p60", "imx519_4k"))
    assert (hd.width, hd.height, hd.fx, hd.fy, hd.cx, hd.cy) == (1280, 720, hd_file.k[0], hd_file.k[4], hd_file.k[2], hd_file.k[5])
    assert (uhd.width, uhd.height, uhd.fx) == (3840, 2160, uhd_file.k[0])
    assert hd.d == tuple(hd_file.d) and hd.measured is hd_file.measured
    assert not hasattr(v2, "scale_intrinsics") and not hasattr(hd, "scaled")  # there is no way to get one mode's model from another's


def test_a_calibration_the_model_cannot_represent_is_refused():
    from types import SimpleNamespace

    good = dict(width=640, height=360, k=(300.0, 0.0, 320.0, 0.0, 300.0, 180.0, 0.0, 0.0, 1.0), d=(0.0,) * 5, distortion_model="plumb_bob", measured=True)
    assert v2.CameraModel.from_calibration(SimpleNamespace(**good)).fx == 300.0
    assert v2.CameraModel.from_calibration(SimpleNamespace(**{**good, "d": ()})).d == (0.0,) * 5
    for change, word in (({"distortion_model": "equidistant"}, "plumb_bob"), ({"d": (0.1,) * 8}, "plumb_bob"), ({"k": (300.0,) * 9}, "camera matrix"), ({"k": (0.0, 0.0, 320.0, 0.0, 300.0, 180.0, 0.0, 0.0, 1.0)}, "above zero")):
        with pytest.raises(v2.ProjectionError, match=word):
            v2.CameraModel.from_calibration(SimpleNamespace(**{**good, **change}))


def test_the_mount_comes_from_the_one_robot_description():
    description = AIRFRAMES["argus_2027"].cameras["down"]
    fitted = mount("argus_2027")
    assert fitted.position_m == description.position_m and fitted.optical_frame == "down_camera_optical"
    assert np.allclose(np.array(fitted.rotation_body_from_optical), description.rotation_body_from_optical())
    # Looking down with the image's top toward the nose: the lens axis is body down, image right is body right.
    rotation = np.array(fitted.rotation_body_from_optical)
    assert np.allclose(rotation @ [0, 0, 1], [0, 0, -1]) and np.allclose(rotation @ [1, 0, 0], [0, -1, 0]) and np.allclose(rotation @ [0, -1, 0], [1, 0, 0])
    with pytest.raises(v2.ProjectionError, match="rotation"):
        v2.CameraMount((0.0, 0.0, 0.0), ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 2.0)))


# --- the spec's three unit tests ---


@pytest.mark.parametrize("yaw_deg", [0.0, 90.0, -137.0])
def test_the_nadir_pixel_lands_directly_below_the_camera_after_the_lever_arm(yaw_deg):
    model, fitted = camera(), mount("argus_2027")  # the lens is 0.2667 m forward, 0.03 m right, 0.40005 m below
    point = v2.project(model, fitted, (model.cx, model.cy), (40.0, -12.0, 15.0), quaternion(yaw_deg=yaw_deg))
    yaw = math.radians(yaw_deg)
    forward, left = np.array([math.cos(yaw), math.sin(yaw)]), np.array([-math.sin(yaw), math.cos(yaw)])
    below_the_lens = np.array([40.0, -12.0]) + 0.2667 * forward - 0.03 * left
    assert np.allclose(east_north(point), below_the_lens, atol=1.0e-9)
    assert np.allclose(point.camera_enu, (*below_the_lens, 15.0 - 0.40005)) and point.range_m == pytest.approx(15.0 - 0.40005)
    assert np.linalg.norm(east_north(point) - [40.0, -12.0]) == pytest.approx(math.hypot(0.2667, 0.03))  # not below the flight controller


def test_a_level_vehicle_sees_image_up_ahead_and_image_right_to_its_right():
    model, fitted = camera("sim_delivery"), mount("sim_iris_downcam")
    centre = v2.project(model, fitted, (model.cx, model.cy), (0.0, 0.0, 10.0), quaternion(yaw_deg=90.0))  # nose north
    above = v2.project(model, fitted, (model.cx, model.cy - 50.0), (0.0, 0.0, 10.0), quaternion(yaw_deg=90.0))
    right = v2.project(model, fitted, (model.cx + 50.0, model.cy), (0.0, 0.0, 10.0), quaternion(yaw_deg=90.0))
    assert above.north_m > centre.north_m + 0.5 and abs(above.east_m) < 1.0e-9
    assert right.east_m > centre.east_m + 0.5 and abs(right.north_m) < 1.0e-9


@pytest.mark.parametrize("mode_name, airframe", [("imx519_720p60", "argus_2027"), ("imx519_4k", "argus_2027"), ("sim_delivery", "sim_iris_downcam")])
def test_synthetic_ground_points_round_trip_within_a_centimetre(mode_name, airframe):
    model, fitted = camera(mode_name), mount(airframe)
    random = np.random.default_rng(5)
    worst, used = 0.0, 0
    for _ in range(400):
        position = (random.uniform(-50, 50), random.uniform(-50, 50), random.uniform(2.4, 30.0))
        orientation = quaternion(random.uniform(-15, 15), random.uniform(-15, 15), random.uniform(-180, 180))
        ground = (position[0] + random.uniform(-1, 1) * position[2], position[1] + random.uniform(-1, 1) * position[2], random.uniform(-1.0, 0.5))
        u, v = v2.pixel_of_ground_point(model, fitted, ground, position, orientation)
        if not (0.0 <= u < model.width and 0.0 <= v < model.height):
            continue  # not in the picture
        point = v2.project(model, fitted, (u, v), position, orientation, ground_up_m=ground[2])
        worst = max(worst, float(np.linalg.norm(east_north(point) - ground[:2])))
        used += 1
    assert used > 100 and worst <= 0.01, (used, worst)
    assert worst < 1.0e-6  # in fact far inside the centimetre: the lens model is undone exactly


@pytest.mark.parametrize("pixel", [(640.0, 360.0), (800.0, 500.0), (130.0, 80.0), (1210.0, 660.0)])
def test_the_jacobian_matches_a_numerical_derivative(pixel):
    model, fitted = camera(), mount("argus_2027")
    position, orientation = (3.0, -2.0, 15.0), quaternion(roll_deg=5.0, pitch_deg=-7.0, yaw_deg=63.0)

    def at(pixel=pixel, position=position, orientation=orientation, ground=0.0):
        return east_north(v2.project(model, fitted, pixel, position, orientation, ground_up_m=ground))

    point = v2.project(model, fitted, pixel, position, orientation)
    step = 1.0e-4
    by_pixel = np.column_stack(
        [
            (at(pixel=(pixel[0] + step, pixel[1])) - at(pixel=(pixel[0] - step, pixel[1]))) / (2 * step),
            (at(pixel=(pixel[0], pixel[1] + step)) - at(pixel=(pixel[0], pixel[1] - step))) / (2 * step),
        ]
    )
    assert np.allclose(point.d_pixel, by_pixel, rtol=1.0e-6, atol=1.0e-9)
    angle = 1.0e-6
    by_attitude = np.column_stack([(at(orientation=turned(orientation, axis, angle)) - at(orientation=turned(orientation, axis, -angle))) / (2 * angle) for axis in range(3)])
    assert np.allclose(point.d_attitude, by_attitude, rtol=1.0e-5, atol=1.0e-6)
    lift = 1.0e-4  # the lens higher above the plane: the plane lower
    by_height = (at(ground=-lift) - at(ground=lift)) / (2 * lift)
    assert np.allclose(point.d_height, by_height, rtol=1.0e-6, atol=1.0e-9)
    # Sizes that make sense at 15 m: 2 to 4 cm a pixel and 25 to 55 cm a degree, more toward the corners.
    assert 0.015 < np.linalg.norm(point.d_pixel[:, 0]) < 0.05 and 0.2 < np.linalg.norm(point.d_attitude[:, 0]) * math.radians(1.0) < 0.6


def test_the_lever_arm_is_in_the_attitude_jacobian():
    # Straight down from a level vehicle the ray's own attitude term is 14.6 m per radian; a lens 0.4 m below the
    # body's origin takes 0.4 m per radian off it. The numerical test above covers it; this pins the sign and size.
    model = camera()
    with_arm = v2.project(model, mount("argus_2027"), (model.cx, model.cy), (0.0, 0.0, 15.0), quaternion())
    no_arm = v2.project(model, mount("sim_iris_downcam"), (model.cx, model.cy), (0.0, 0.0, 15.0), quaternion())
    assert abs(no_arm.d_attitude[0, 1]) == pytest.approx(15.0) and abs(with_arm.d_attitude[0, 1]) == pytest.approx(15.0, abs=1.0e-6)
    assert with_arm.range_m == pytest.approx(15.0 - 0.40005)


# --- when there is no answer ---


def test_a_ray_that_does_not_come_down_and_a_lens_under_the_plane_give_nothing():
    model, fitted = camera("sim_delivery"), mount("sim_iris_downcam")
    assert v2.project(model, fitted, (model.cx, model.cy), (0.0, 0.0, 10.0), quaternion(roll_deg=95.0)) is None  # looking above the horizon
    assert v2.project(model, fitted, (model.cx, model.cy), (0.0, 0.0, 10.0), quaternion(roll_deg=88.0)) is None  # 2 degrees below it: too flat
    assert v2.project(model, fitted, (model.cx, model.cy), (0.0, 0.0, 10.0), quaternion(roll_deg=60.0)) is not None
    assert v2.project(model, fitted, (model.cx, model.cy), (0.0, 0.0, -0.5), quaternion()) is None
    assert v2.project(model, fitted, (model.cx, model.cy), (0.0, 0.0, 1.0), quaternion(), ground_up_m=1.0) is None


def test_every_ray_of_a_level_downward_camera_goes_down_unlike_the_2026_logger():
    # mission_logger's 2026 raycast sent every ray upward and so never met the ground (FEATURES.md, FND-05).
    model, fitted = camera(), mount("argus_2027")
    for pixel in ((0.0, 0.0), (1279.0, 0.0), (0.0, 719.0), (1279.0, 719.0), (640.0, 360.0)):
        point = v2.project(model, fitted, pixel, (0.0, 0.0, 15.0), quaternion(yaw_deg=30.0))
        assert point is not None and point.camera_enu[2] > point.up_m and point.range_m >= 15.0 - 0.40005 - 1.0e-9


def test_bad_numbers_are_refused_not_projected():
    model, fitted = camera("sim_delivery"), mount("sim_iris_downcam")
    for arguments in (((float("nan"), 10.0), (0.0, 0.0, 10.0), quaternion()), ((10.0, 10.0), (0.0, float("inf"), 10.0), quaternion()), ((10.0, 10.0), (0.0, 0.0, 10.0), (0.0, 0.0, 0.0, 0.0))):
        with pytest.raises(v2.ProjectionError):
            v2.project(model, fitted, *arguments)


# --- project_pixel_to_ground(pixel, stamp) ---


def flying_buffer(speed_east=2.0, yaw_rate_deg=20.0):
    """A vehicle moving east and turning, 30 poses a second for one second from t = 100 s."""
    buffer = PoseBuffer()
    for index in range(31):
        t = index / 30.0
        buffer.add_pose(100 * S + round(t * S), (speed_east * t, 0.0, 15.0), quaternion(yaw_deg=yaw_rate_deg * t))
        buffer.add_velocity(100 * S + round(t * S), (speed_east, 0.0, 0.0))
    return buffer


def test_the_pose_used_is_the_one_at_the_frames_stamp_not_the_latest():
    model, fitted, buffer = camera("sim_delivery"), mount("sim_iris_downcam"), flying_buffer()
    projector = v2.GroundProjector(model, fitted, buffer.pose_at)
    target = (0.9, 1.5, 0.0)
    stamp = 100 * S + 250 * MS  # the frame was taken 0.75 s before the newest pose
    then = buffer.pose_at(stamp)
    pixel = v2.pixel_of_ground_point(model, fitted, target, then.position, then.orientation)
    point = projector.project_pixel_to_ground(pixel, stamp)
    assert np.linalg.norm(east_north(point) - target[:2]) < 0.01 and point.stamp_ns == stamp and not point.pose_extrapolated
    newest = buffer.pose_at(buffer.newest_ns())
    with_latest_pose = v2.project(model, fitted, pixel, newest.position, newest.orientation)
    assert np.linalg.norm(east_north(with_latest_pose) - target[:2]) > 1.0  # what 2026's latest-pose projection would have said


def test_no_pose_for_the_stamp_means_no_projection_and_a_reason():
    model, fitted, buffer = camera("sim_delivery"), mount("sim_iris_downcam"), flying_buffer()
    projector = v2.GroundProjector(model, fitted, buffer.pose_at)
    assert projector.project_pixel_to_ground((320.0, 180.0), 101 * S + 200 * MS) is None  # 200 ms after the newest pose
    assert "no pose" in projector.last_refusal
    assert projector.project_pixel_to_ground((320.0, 180.0), 90 * S) is None  # before the history
    ahead = projector.project_pixel_to_ground((320.0, 180.0), 101 * S + 20 * MS)  # 20 ms ahead: carried on with the velocity
    assert ahead is not None and ahead.pose_extrapolated and projector.last_refusal is None
    assert projector.project_pixel_to_ground((float("nan"), 180.0), 100 * S + 500 * MS) is None and "finite" in projector.last_refusal


def test_an_object_above_the_ground_is_projected_onto_its_own_plane():
    model, fitted, buffer = camera("sim_delivery"), mount("sim_iris_downcam"), flying_buffer(speed_east=0.0, yaw_rate_deg=0.0)
    projector = v2.GroundProjector(model, fitted, buffer.pose_at)
    rim = (2.0, 1.0, 0.2)  # a bucket's rim, 0.2 m up
    pose = buffer.pose_at(100 * S + 500 * MS)
    pixel = v2.pixel_of_ground_point(model, fitted, rim, pose.position, pose.orientation)
    on_its_plane = projector.project_pixel_to_ground(pixel, 100 * S + 500 * MS, plane_height_m=0.2)
    on_the_ground = projector.project_pixel_to_ground(pixel, 100 * S + 500 * MS)
    assert np.linalg.norm(east_north(on_its_plane) - rim[:2]) < 1.0e-6 and on_its_plane.up_m == pytest.approx(0.2)
    assert np.linalg.norm(east_north(on_the_ground) - rim[:2]) > 0.02 and on_the_ground.height_source == "home"


# --- where the ground is ---


def climbing_buffer(climb_mps=2.0, roll_deg=0.0):
    """A vehicle climbing from 15 m, 30 poses a second for one second from t = 100 s."""
    buffer = PoseBuffer()
    for index in range(31):
        t = index / 30.0
        buffer.add_pose(100 * S + round(t * S), (0.0, 0.0, 15.0 + climb_mps * t), quaternion(roll_deg=roll_deg))
        buffer.add_velocity(100 * S + round(t * S), (0.0, 0.0, climb_mps))
    return buffer


def readings(height, around_ms, range_m, count=5, step_ms=50, limits=(0.0, 1000.0)):
    """`count` rangefinder readings 50 ms apart, the last at `around_ms` after t = 100 s."""
    for index in range(count):
        height.add_range(100 * S + (around_ms - (count - 1 - index) * step_ms) * MS, range_m, *limits)


def test_the_ground_is_under_a_healthy_rangefinder_and_at_home_height_otherwise():
    buffer = climbing_buffer(climb_mps=0.0)
    height = v2.GroundHeight(buffer.pose_at)
    assert height.ground_up(100 * S + 500 * MS) == (0.0, "home")  # no reading yet
    readings(height, 400, 13.0)  # the ground here is 2 m above home
    ground, source = height.ground_up(100 * S + 500 * MS)
    assert source == "rangefinder" and ground == pytest.approx(2.0)
    assert height.ground_up(100 * S + 950 * MS) == (0.0, "home")  # the newest reading is 0.55 s from this frame
    for reading in (40.0, 39.97, 0.2, 0.22, float("nan"), float("inf")):  # at the sensor's limits: it is saying "out of range"
        limits = v2.GroundHeight(buffer.pose_at)
        readings(limits, 500, reading, limits=(0.2, 40.0))
        assert limits.ground_up(100 * S + 500 * MS) == (0.0, "home"), reading


def test_one_reading_is_not_enough_and_one_odd_reading_does_not_move_the_ground():
    buffer = climbing_buffer(climb_mps=0.0)
    lonely = v2.GroundHeight(buffer.pose_at)
    readings(lonely, 500, 13.0, count=2)
    assert lonely.ground_up(100 * S + 500 * MS) == (0.0, "home")  # two readings: not believed
    readings(lonely, 550, 13.0, count=1)
    assert lonely.ground_up(100 * S + 500 * MS)[1] == "rangefinder"  # three: believed
    steady = v2.GroundHeight(buffer.pose_at)
    readings(steady, 450, 13.0, count=4)
    steady.add_range(100 * S + 500 * MS, 13.3, 0.0, 1000.0)  # the nearest in time is 0.3 m off
    assert steady.ground_up(100 * S + 500 * MS) == (pytest.approx(2.0), "rangefinder")  # the median, not the nearest


def test_readings_that_disagree_or_put_the_ground_far_from_home_are_not_believed():
    buffer = climbing_buffer(climb_mps=0.0)
    jumping = v2.GroundHeight(buffer.pose_at)
    for index, reading in enumerate((13.0, 13.0, 12.2, 13.0, 12.2)):  # something swinging through the beam
        jumping.add_range(100 * S + (300 + index * 50) * MS, reading, 0.0, 1000.0)
    assert jumping.ground_up(100 * S + 500 * MS) == (0.0, "home")
    blocked = v2.GroundHeight(buffer.pose_at)
    readings(blocked, 500, 0.4)  # the payload in the beam, steadily: "the ground is 14.6 m above home"
    assert blocked.ground_up(100 * S + 500 * MS) == (0.0, "home")
    no_return = v2.GroundHeight(buffer.pose_at)
    readings(no_return, 500, 130.0)  # a far reading: "the ground is 115 m below home"
    assert no_return.ground_up(100 * S + 500 * MS) == (0.0, "home")
    plausible = v2.GroundHeight(buffer.pose_at)
    readings(plausible, 500, 19.0)  # a dip 4 m below home
    assert plausible.ground_up(100 * S + 500 * MS) == (pytest.approx(-4.0), "rangefinder")


def test_a_tilted_beam_is_corrected_and_one_tilted_too_far_is_not_used():
    tilted = v2.GroundHeight(climbing_buffer(climb_mps=0.0, roll_deg=20.0).pose_at)
    readings(tilted, 500, 13.0)
    ground, source = tilted.ground_up(100 * S + 500 * MS)
    assert source == "rangefinder" and ground == pytest.approx(15.0 - 13.0 * math.cos(math.radians(20.0)))
    too_far = v2.GroundHeight(climbing_buffer(climb_mps=0.0, roll_deg=30.0).pose_at)
    readings(too_far, 500, 13.0)
    assert too_far.ground_up(100 * S + 500 * MS) == (0.0, "home")


def test_the_ground_does_not_move_when_the_vehicle_climbs_between_the_reading_and_the_frame():
    buffer = climbing_buffer(climb_mps=2.0)  # 15 m at t = 100 s, 17 m a second later
    height = v2.GroundHeight(buffer.pose_at)
    for ms in (200, 250, 300):  # over ground 2 m above home; the vehicle is at 15 + 2 t
        height.add_range(100 * S + ms * MS, 15.0 + 2.0 * ms / 1000.0 - 2.0, 0.0, 1000.0)
    ground, source = height.ground_up(100 * S + 700 * MS)  # the frame is 0.4 s later, the vehicle 0.8 m higher
    assert source == "rangefinder" and ground == pytest.approx(2.0, abs=1.0e-6)
    # Readings with no pose for their own moment are passed over.
    for ms in (1200, 1250, 1300):  # after the newest pose
        height.add_range(100 * S + ms * MS, 1.0, 0.0, 1000.0)
    late = v2.GroundHeight(buffer.pose_at)
    for ms in (1200, 1250, 1300):
        late.add_range(100 * S + ms * MS, 1.0, 0.0, 1000.0)
    assert late.ground_up(101 * S) == (0.0, "home")
    for ms in (600, 650, 700):
        height.add_range(100 * S + ms * MS, 15.0 + 2.0 * ms / 1000.0 - 2.0, 0.0, 1000.0)
    assert height.ground_up(101 * S)[0] == pytest.approx(2.0, abs=1.0e-6)


def test_the_projector_uses_the_rangefinders_ground_and_says_so():
    model, fitted, buffer = camera("sim_delivery"), mount("sim_iris_downcam"), flying_buffer(speed_east=0.0, yaw_rate_deg=0.0)
    height = v2.GroundHeight(buffer.pose_at)
    readings(height, 500, 13.0)  # ground 2 m above home under the vehicle
    projector = v2.GroundProjector(model, fitted, buffer.pose_at, height)
    target = (3.0, -2.0, 2.0)
    pose = buffer.pose_at(100 * S + 500 * MS)
    pixel = v2.pixel_of_ground_point(model, fitted, target, pose.position, pose.orientation)
    point = projector.project_pixel_to_ground(pixel, 100 * S + 500 * MS)
    assert point.height_source == "rangefinder" and np.linalg.norm(east_north(point) - target[:2]) < 1.0e-6
    at_home_height = v2.GroundProjector(model, fitted, buffer.pose_at).project_pixel_to_ground(pixel, 100 * S + 500 * MS)
    assert np.linalg.norm(east_north(at_home_height) - target[:2]) > 0.4  # 2 m of height error, seen 13 degrees off nadir


# --- rules ---


def test_the_module_has_no_ros_in_it_and_the_2026_projection_is_not_imported():
    source = (Path(v2.__file__)).read_text()
    imported = set()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            imported |= {alias.name.split(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            imported.add((node.module or "").split(".")[0])
    assert imported <= {"__future__", "math", "collections", "dataclasses", "typing", "numpy"}, imported
