"""REC-5: optical-flow odometry (drone_target_cv/flow_odometry.py), on pictures made here. No ROS; numpy and cv2."""

import math
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_target_cv import flow_odometry as fo  # noqa: E402

FOCAL = 233.87  # the sim's flow camera at 320 px wide
LEVEL = fo.Attitude(0.0, 0.0, 0.0)


def ground(seed=1, size=(600, 800)):
    """A ground with texture at several scales, larger than the picture so that it can be slid under the camera."""
    rng = np.random.default_rng(seed)
    coarse = cv2.resize(rng.normal(0.0, 1.0, (size[0] // 20, size[1] // 20)).astype(np.float32), (size[1], size[0]), interpolation=cv2.INTER_CUBIC)
    fine = cv2.GaussianBlur(rng.normal(0.0, 1.0, size).astype(np.float32), (0, 0), 1.5)
    return np.clip(120 + 35 * coarse + 40 * fine, 0, 255).astype(np.float32)


def view(field, dx=0.0, dy=0.0):
    """The camera's 320 x 200 picture with the ground's content moved by (dx, dy) pixels (x right, y down)."""
    x0, y0 = (field.shape[1] - fo.WIDTH) / 2.0, (field.shape[0] - fo.HEIGHT) / 2.0
    move = np.float32([[1, 0, -(x0 - dx)], [0, 1, -(y0 - dy)]])
    return cv2.warpAffine(field, move, (fo.WIDTH, fo.HEIGHT), flags=cv2.INTER_LINEAR)


def fed(frames, attitudes=None, height=10.0, dt=1.0 / 30.0, corners_only=False, apart=4):
    odometry = fo.FlowOdometry(FOCAL, frames_apart=apart)
    out = []
    for index, frame in enumerate(frames):
        attitude = LEVEL if attitudes is None else attitudes[index]
        out.append(odometry.add(frame, int(index * dt * 1e9) + 1, attitude, height, corners_only))
    return out


def test_the_shift_of_the_picture_is_found_to_a_tenth_of_a_pixel():
    field = ground()
    for dx, dy in ((3.0, 0.0), (0.0, -4.5), (2.3, 1.7), (-6.0, 5.0), (0.4, -0.3)):
        found = fo.shift_between(view(field), view(field, dx, dy))
        assert found[0] == pytest.approx(dx, abs=0.12) and found[1] == pytest.approx(dy, abs=0.12) and found[2] > 0.3, (dx, dy)


def test_flying_forward_and_right_is_metres_a_second_scaled_by_the_height():
    """At 10 m with the sim's lens a pixel is 4.3 cm: 2 m/s forward moves the content 1.56 px a frame at 30 Hz, towards the tail."""
    field = ground()
    per_frame = 2.0 * FOCAL / 10.0 / 30.0
    frames = [view(field, dx=0.0, dy=per_frame * index) for index in range(10)]  # content moving down the picture: the vehicle goes forward
    samples = fed(frames)
    assert samples[:4] == [None] * 4 and all(sample is not None for sample in samples[4:])
    last = samples[-1]
    assert last.quality > 100 and last.why == "" and last.forward_mps == pytest.approx(2.0, abs=0.08) and last.right_mps == pytest.approx(0.0, abs=0.08)
    assert last.dt_s == pytest.approx(4 / 30.0) and last.shift_px[1] == pytest.approx(4 * per_frame, abs=0.15)
    # The same picture motion from half the height is half the speed.
    assert fed(frames, height=5.0)[-1].forward_mps == pytest.approx(1.0, abs=0.05)
    # Content moving left: the vehicle goes right. Facing north (yaw 90 degrees), right is east.
    right = fed([view(field, dx=-per_frame * index) for index in range(10)], attitudes=[fo.Attitude(0.0, 0.0, math.pi / 2)] * 10)[-1]
    assert right.right_mps == pytest.approx(2.0, abs=0.08) and right.forward_mps == pytest.approx(0.0, abs=0.08)
    assert right.east_mps == pytest.approx(2.0, abs=0.08) and right.north_mps == pytest.approx(0.0, abs=0.08)
    assert fo.to_east_north(1.0, 0.0, 0.0) == pytest.approx((1.0, 0.0)) and fo.to_east_north(1.0, 0.0, math.pi / 2) == pytest.approx((0.0, 1.0))
    assert fo.to_east_north(0.0, 1.0, 0.0) == pytest.approx((0.0, -1.0))  # facing east, right is south


def test_turning_is_taken_out_a_hover_that_pitches_and_rolls_is_not_flying():
    """Nose up by 0.02 rad swings the picture 4.7 px towards the tail: without the attitude that would be 1.5 m/s forward at 10 m."""
    field = ground()
    pitch = [0.005 * index for index in range(10)]
    frames = [view(field, dy=FOCAL * angle) for angle in pitch]
    turning = fed(frames, attitudes=[fo.Attitude(0.0, angle, 0.0) for angle in pitch])[-1]
    assert turning.rotation_px[1] == pytest.approx(FOCAL * 0.02, abs=0.01) and abs(turning.forward_mps) < 0.08 and abs(turning.right_mps) < 0.08 and turning.quality > 100
    blind = fed(frames)[-1]  # the same pictures with the attitude held level: read as flight
    assert blind.forward_mps == pytest.approx(FOCAL * 0.02 * 10.0 / (FOCAL * 4 / 30.0), abs=0.1) and blind.forward_mps > 1.4
    roll = [0.004 * index for index in range(10)]
    rolled = fed([view(field, dx=FOCAL * angle) for angle in roll], attitudes=[fo.Attitude(angle, 0.0, 0.0) for angle in roll])[-1]
    assert rolled.rotation_px[0] == pytest.approx(FOCAL * 0.016, abs=0.01) and abs(rolled.right_mps) < 0.08
    # Flying and turning at once: the flight is what is left.
    both = [view(field, dx=FOCAL * roll[index], dy=1.5 * index + FOCAL * pitch[index]) for index in range(10)]
    moving = fed(both, attitudes=[fo.Attitude(roll[index], pitch[index], 0.0) for index in range(10)])[-1]
    assert moving.forward_mps == pytest.approx(1.5 * 4 * 10.0 / (FOCAL * 4 / 30.0), abs=0.1) and abs(moving.right_mps) < 0.1


def test_the_two_pictures_are_as_far_apart_in_time_as_the_motion_allows():
    """Found in the sim: over 50 ms a tenth of a pixel is 0.13 m/s at 15 m. A slow picture is followed over four frames; a fast one over as many as it can be."""
    field = ground(size=(600, 2000))
    slow = fed([view(field, dx=-2.0 * index) for index in range(10)], dt=0.05)[-1]
    assert slow.dt_s == pytest.approx(0.2) and slow.shift_px[0] == pytest.approx(-8.0, abs=0.2) and slow.right_mps == pytest.approx(2.0 * 10.0 / (FOCAL * 0.05), abs=0.06)
    fast = fed([view(field, dx=-30.0 * index) for index in range(10)], dt=0.05)[-1]  # 120 px over four frames is too far; 60 px over two is not
    assert fast.dt_s == pytest.approx(0.1) and fast.shift_px[0] == pytest.approx(-60.0, abs=0.3) and fast.quality > 0
    faster = fed([view(field, dx=-50.0 * index) for index in range(10)], dt=0.05)[-1]  # only neighbours can be followed
    assert faster.dt_s == pytest.approx(0.05) and faster.right_mps == pytest.approx(50.0 * 10.0 / (FOCAL * 0.05), rel=0.02)
    # The finer velocity of the long baseline: the same half-pixel of noise in the shift is a quarter of the error.
    assert 0.5 * 10.0 / (FOCAL * slow.dt_s) == pytest.approx(0.25 * 0.5 * 10.0 / (FOCAL * faster.dt_s))


def test_a_bare_surface_and_a_picture_that_cannot_be_followed_have_no_quality_and_no_velocity():
    flat = np.full((fo.HEIGHT, fo.WIDTH), 128.0, np.float32)
    noise = np.random.default_rng(3).normal(0.0, 1.0, (10, fo.HEIGHT, fo.WIDTH)).astype(np.float32)
    bare = fed([flat + noise[index] for index in range(10)])[-1]  # a flat grey with the sensor's own noise
    assert bare.quality == 0 and bare.why == "no texture in the picture (a bare surface)" and (bare.forward_mps, bare.right_mps, bare.east_mps) == (0.0, 0.0, 0.0)
    field = ground()
    unrelated = fed([view(ground(seed=index + 10)) for index in range(10)])[-1]  # every frame another ground: nothing to follow
    assert unrelated.quality == 0 and ("weak" in unrelated.why or "too far" in unrelated.why)
    jumped = fed([view(ground(size=(600, 2000)), dx=95.0 * index) for index in range(10)])[-1]  # 95 px a frame: too far even between neighbours
    assert jumped.quality == 0
    still = fed([view(field)] * 10)
    assert still[-1].quality == 255 and abs(still[-1].forward_mps) < 0.01
    # No height, a height outside the band, no attitude, frames too far apart: said, with no velocity.
    assert fed([view(field)] * 6, height=None)[-1].why == "no height above the ground to scale the flow with"
    assert "outside 0.3 to 40 m" in fed([view(field)] * 6, height=0.1)[-1].why and "outside" in fed([view(field)] * 6, height=float("nan"))[-1].why
    assert fed([view(field)] * 6, attitudes=[None] * 6)[-1].why == "no attitude at the frames' times to take the turning out with"
    assert fed([view(field)] * 6, dt=0.2)[-1].dt_s == pytest.approx(0.4)  # frames 0.2 s apart: the two half a second or less apart are used
    assert fed([view(field)] * 6, dt=0.6)[-1].why == "the frames are 0.60 s apart"  # even neighbours are too far apart in time
    assert fo.quality_of(0.0) == 0 and fo.quality_of(fo.MIN_RESPONSE) == 1 and fo.quality_of(0.9) == 255 and 1 < fo.quality_of(0.3) < 255


def test_with_the_hook_or_the_bucket_in_view_the_corner_patches_follow_the_ground_and_not_it():
    """A bucket hanging under the camera fills the middle and swings its own way; the ground moves under the corners."""
    field = ground()

    def with_bucket(index):
        frame = view(field, dy=1.5 * index)  # the ground: 1.5 px a frame towards the tail
        cv2.circle(frame, (160 + 3 * index, 100 - 2 * index), 40, 230.0, -1)  # the bucket: its own motion, across the middle
        cv2.circle(frame, (160 + 3 * index, 100 - 2 * index), 22, 40.0, -1)
        return frame

    frames = [with_bucket(index) for index in range(10)]
    corners = fed(frames, corners_only=True)[-1]
    assert corners.quality > 0 and corners.patches >= 5 and corners.shift_px == pytest.approx((0.0, 6.0), abs=0.3)
    assert corners.forward_mps == pytest.approx(6.0 * 10.0 / (FOCAL * 4 / 30.0), abs=0.15)
    whole = fed(frames)[-1]
    assert whole is not None  # (the whole picture may or may not be pulled by so small a bucket: the border patches are the rule under a load)
    # Corners with nothing to follow under them: no quality, and it says how many had texture.
    flat = np.full((fo.HEIGHT, fo.WIDTH), 128.0, np.float32)
    assert fed([flat] * 6, corners_only=True)[-1].why == "only 0 of the eight border patches have texture to follow"
    # Patches that each go their own way (the ground is not one plane, or nothing in view is ground): no quality.
    def torn(index):
        frame = view(field)
        frame[:66, :106] = view(field, dx=3.0 * index)[:66, :106]
        frame[:66, 106:212] = view(field, dx=-3.0 * index)[:66, 106:212]
        frame[:66, 212:] = view(field, dy=3.0 * index)[:66, 212:]
        frame[132:, :106] = view(field, dy=-3.0 * index)[132:, :106]
        frame[132:, 212:] = view(field, dx=2.0 * index, dy=2.0 * index)[132:, 212:]
        return frame

    torn_sample = fed([torn(index) for index in range(10)], corners_only=True)[-1]
    assert torn_sample.quality == 0 and "disagree" in torn_sample.why
    # A carried bucket as wide as it is in the sim's flow camera (90 px of the 320), reaching into the patches beside the middle: the border still gives the ground.
    def big_bucket(index):
        frame = view(field, dx=-1.0 * index, dy=1.2 * index)
        cv2.circle(frame, (160, 100), 45, 235.0, -1)  # it does not move in the picture: it hangs from the vehicle
        cv2.circle(frame, (160, 100), 30, 60.0, -1)
        return frame

    carried = fed([big_bucket(index) for index in range(10)], corners_only=True)[-1]
    frames_between = round(carried.dt_s * 30.0)
    assert carried.quality > 0 and carried.patches >= 5 and carried.shift_px == pytest.approx((-1.0 * frames_between, 1.2 * frames_between), abs=0.4)


def test_coming_down_or_turning_while_flying_does_not_bias_the_velocity():
    """Found in the sim: descending through the transit, the picture grows, the border patches move outward, and the shift read was pixels off."""

    def zoomed(index, grow_per_frame=0.01, turn_per_frame=0.0, dx=0.0, dy=0.0):
        centre = (fo.WIDTH - 1) / 2.0, (fo.HEIGHT - 1) / 2.0
        scale, angle = 1.0 + grow_per_frame * index, turn_per_frame * index
        cos, sin = scale * math.cos(angle), scale * math.sin(angle)
        x0, y0 = (field.shape[1] - fo.WIDTH) / 2.0, (field.shape[0] - fo.HEIGHT) / 2.0
        # picture (u, v) -> field: undo the shift, the growing and the turning about the picture's middle
        inverse = np.linalg.inv(np.array([[cos, -sin, centre[0] - cos * centre[0] + sin * centre[1] + dx * index], [sin, cos, centre[1] - sin * centre[0] - cos * centre[1] + dy * index], [0.0, 0.0, 1.0]]))
        to_field = np.array([[1.0, 0.0, x0], [0.0, 1.0, y0], [0.0, 0.0, 1.0]]) @ inverse
        return cv2.warpAffine(field, to_field[:2].astype(np.float32), (fo.WIDTH, fo.HEIGHT), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)

    field = ground()
    # A descent (the picture grows 1% a frame) while flying forward 1.5 px a frame: the shift at the middle is the flight's.
    descending = fed([zoomed(index, dy=1.5) for index in range(10)], corners_only=True, dt=0.05)[-1]
    expected = 1.5 * (9 - 5 * 1.09 / 1.05)  # the picture's middle between the fifth and the ninth frame: the growing stretches the shift already made
    assert descending.quality > 0 and descending.patches >= 6 and descending.shift_px == pytest.approx((0.0, expected), abs=0.4)
    # A hover that comes straight down: no velocity over the ground, though every border patch moves outward by up to 5 px.
    straight_down = fed([zoomed(index) for index in range(10)], corners_only=True, dt=0.05)[-1]
    assert straight_down.quality > 0 and math.hypot(*straight_down.shift_px) < 0.4 and abs(straight_down.forward_mps) < 0.1
    # Yawing on the spot (0.5 degrees a frame): the patches move round the middle, and the middle does not move.
    yawing = fed([zoomed(index, grow_per_frame=0.0, turn_per_frame=math.radians(0.5)) for index in range(10)], corners_only=True, dt=0.05)[-1]
    assert yawing.quality > 0 and math.hypot(*yawing.shift_px) < 0.4
    # The fit itself: translation, growing and turning are told apart, and two patches on the bucket are left out.
    spots = [(-107.0, -67.0), (0.0, -67.0), (107.0, -67.0), (-107.0, 0.0), (107.0, 0.0), (-107.0, 67.0), (0.0, 67.0), (107.0, 67.0)]
    moved = [(2.0 + 0.03 * x - 0.01 * y, -1.0 + 0.03 * y + 0.01 * x, 0.5, x, y) for x, y in spots]
    shift, agreeing = fo.fit_motion(moved)
    assert shift == pytest.approx((2.0, -1.0), abs=1e-6) and len(agreeing) == 8
    on_the_bucket = moved[:6] + [(0.0, 0.0, 0.9, 0.0, 67.0), (0.3, -0.2, 0.9, 107.0, 67.0)]
    shift, agreeing = fo.fit_motion(on_the_bucket)
    assert shift == pytest.approx((2.0, -1.0), abs=0.05) and len(agreeing) == 6
    half_and_half = moved[:4] + [(0.0, 0.0, 0.9, x, y) for x, y in spots[4:]]  # four on the ground and four on a load that fills half the picture
    split = fo.fit_motion(half_and_half)  # no clear majority: no answer, or the ground's to within half a pixel, and never the load's
    assert split is None or math.hypot(split[0][0] - 2.0, split[0][1] + 1.0) < 0.6
    assert fo.fit_motion(moved[:4]) is None  # too few patches to say: a majority of the eight is asked for
    scattered = [(float(7 * index % 11) - 5.0, float(5 * index % 13) - 6.0, 0.5, x, y) for index, (x, y) in enumerate(spots)]
    assert fo.fit_motion(scattered) is None  # patches that agree on nothing


def test_a_larger_or_coloured_frame_is_shrunk_to_the_flow_picture_and_a_clock_that_steps_back_starts_again():
    field = ground(size=(1200, 1600))
    big = cv2.cvtColor(np.clip(field[200:1000, 160:1440], 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)  # 1280 x 800, three channels
    picture = fo.prepare(big)
    assert picture.shape == (fo.HEIGHT, fo.WIDTH) and picture.dtype == np.float32
    odometry = fo.FlowOdometry(FOCAL)
    for index in range(5):
        sample = odometry.add(view(ground()), 1_000_000_000 + index * 33_000_000, LEVEL, 10.0)
    assert sample is not None
    assert odometry.add(view(ground()), 900_000_000, LEVEL, 10.0) is None  # an older stamp: the held frames are dropped
    assert [odometry.add(view(ground()), 900_000_000 + (index + 1) * 33_000_000, LEVEL, 10.0) is None for index in range(4)] == [True, True, True, False]
    with pytest.raises(ValueError):
        fo.FlowOdometry(0.0)


def test_the_sample_as_the_optical_flow_message_carries_it():
    field = ground()
    sample = fed([view(field, dx=-1.0 * index, dy=1.5 * index) for index in range(10)])[-1]
    fields = fo.mavlink_fields(sample, FOCAL)
    assert fields["quality"] == sample.quality and fields["ground_distance"] == 10.0 and fields["flow"] == sample.shift_px
    assert fields["flow_comp_m"] == (sample.forward_mps, -sample.right_mps)  # forward and left: the body frame ROS uses
    assert fields["flow_rate"][1] == pytest.approx(sample.shift_px[1] / (FOCAL * sample.dt_s))
    none = fo.mavlink_fields(fo.FlowSample(1, 0.133, 0, "no texture in the picture (a bare surface)", height_m=7.0), FOCAL)
    assert none == {"flow": (0.0, 0.0), "flow_comp_m": (0.0, 0.0), "flow_rate": (0.0, 0.0), "quality": 0, "ground_distance": 7.0}
    assert fo.FLAG == "flow_odometry" and (fo.WIDTH, fo.HEIGHT, fo.FRAMES_APART) == (320, 200, 4)


TAIL_UP_MOUNT = ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))  # the picture's top to the tail: its x is the vehicle's left, its y its front
LEFT_UP_MOUNT = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))  # turned a quarter: the picture's x is the vehicle's front, its y its right


def test_the_mount_says_how_the_picture_sits_on_the_vehicle_for_the_velocity_and_for_the_turning_taken_out():
    """The flight-safety review: a camera with its picture's top to the tail had its velocity flipped after the turning was taken out the nose-up way, which doubled the turning instead."""
    field = ground()
    pitch = [0.004 * index for index in range(10)]
    # The vehicle flies forward 1.5 px a frame and pitches nose up. Nose-up camera: the content moves towards the tail, +y, for both.
    nose_up = [view(field, dy=1.5 * index + FOCAL * pitch[index]) for index in range(10)]
    # The same flight seen by a camera turned half round: every shift in the picture is the other way.
    tail_up = [view(field, dy=-(1.5 * index + FOCAL * pitch[index])) for index in range(10)]
    attitudes = [fo.Attitude(0.0, angle, 0.0) for angle in pitch]
    expected = 1.5 * 4 * 10.0 / (FOCAL * 4 / 30.0)

    def flown(frames, mount):
        odometry = fo.FlowOdometry(FOCAL, body_from_optical=mount)
        for index, frame in enumerate(frames):
            sample = odometry.add(frame, int(index / 30.0 * 1e9) + 1, attitudes[index], 10.0)
        return sample

    usual, turned = flown(nose_up, fo.NOSE_UP_MOUNT), flown(tail_up, TAIL_UP_MOUNT)
    for sample in (usual, turned):
        assert sample.forward_mps == pytest.approx(expected, abs=0.12) and abs(sample.right_mps) < 0.1, sample
    assert turned.rotation_px[1] == pytest.approx(-usual.rotation_px[1], abs=1e-6)  # the turning's shift is the other way in the turned picture, and is taken out of it there
    # Had the turning been taken out the nose-up way and the result flipped, the pitching would have been doubled, not removed.
    wrong = fo.velocity(turned.shift_px, usual.rotation_px, 10.0, FOCAL, turned.dt_s)[0] * -1.0
    assert abs(wrong - expected) > 1.0
    # A quarter turn: the picture's x is the vehicle's front. Content moving -x in that picture is the vehicle going forward.
    # (the same pitching flight in that picture: nose up moves the content to the tail, which is -x there, and so does flying forward)
    quarter = flown([view(field, dx=-(1.5 * index + FOCAL * pitch[index])) for index in range(10)], LEFT_UP_MOUNT)
    assert quarter.forward_mps == pytest.approx(expected, abs=0.12) and abs(quarter.right_mps) < 0.1
    assert fo.rotation_shift_px(fo.Attitude(0, 0, 0), fo.Attitude(0.0, 0.01, 0.0), FOCAL, LEFT_UP_MOUNT) == pytest.approx((-FOCAL * 0.01, 0.0), abs=1e-9)  # nose up: the content moves to the tail, which is -x there
    # A camera that does not look straight down is not a flow camera here.
    with pytest.raises(ValueError, match="must look straight down"):
        fo.FlowOdometry(FOCAL, body_from_optical=((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0)))  # looking forward
    assert fo.check_mount(fo.NOSE_UP_MOUNT) == fo.NOSE_UP_MOUNT


def test_a_sample_scaled_by_a_height_that_is_not_the_rangefinders_has_its_quality_held_down():
    field = ground()
    sample = fed([view(field, dx=-1.0 * index, dy=1.5 * index) for index in range(10)])[-1]
    assert fo.with_height_source(sample, "rangefinder") is sample
    above = fo.with_height_source(sample, "baro")
    assert above.quality == min(sample.quality, fo.BARO_MAX_QUALITY) == 127 and above.forward_mps == sample.forward_mps
    nothing = fo.FlowSample(1, 0.1, 0, "no texture in the picture (a bare surface)")
    assert fo.with_height_source(nothing, "baro") is nothing
    # The quaternion rule: about the body's left, backwards, is nose up; about the nose is right side down.
    assert fo.attitude_from_quaternion((0.0, math.sin(-0.05), 0.0, math.cos(-0.05))).pitch_rad == pytest.approx(0.1)
    assert fo.attitude_from_quaternion((math.sin(0.05), 0.0, 0.0, math.cos(0.05))).roll_rad == pytest.approx(0.1)
