"""REC-7b: the home marker (an AprilTag at the return point) and keyframe homing (pictures from the takeoff climb)."""

import math
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "drone_target_cv"), str(SRC / "drone_mission_core")]

from drone_target_cv import home_marker, keyframe_homing  # noqa: E402


def ground(seed=3, size=1600):
    """A patch of ground with things on it: blobs, lines and speckle, the same for a seed."""
    rng = np.random.default_rng(seed)
    picture = np.full((size, size), 110, dtype=np.uint8)
    for _ in range(260):
        centre = (int(rng.integers(0, size)), int(rng.integers(0, size)))
        if rng.random() < 0.6:
            cv2.circle(picture, centre, int(rng.integers(4, 26)), int(rng.integers(20, 240)), -1)
        else:
            end = (centre[0] + int(rng.integers(-90, 90)), centre[1] + int(rng.integers(-90, 90)))
            cv2.line(picture, centre, end, int(rng.integers(20, 240)), int(rng.integers(2, 7)))
    noise = rng.normal(0.0, 4.0, picture.shape)
    return np.clip(picture.astype(float) + noise, 0, 255).astype(np.uint8)


def view(world, centre, px_per_world_px, yaw_deg=0.0, size=(640, 480)):
    """What a camera looking down sees: `centre` (world pixels) at the picture's middle, the world `px_per_world_px` times its size, turned by yaw."""
    matrix = cv2.getRotationMatrix2D((float(centre[0]), float(centre[1])), yaw_deg, px_per_world_px)
    matrix[0, 2] += size[0] / 2.0 - centre[0]
    matrix[1, 2] += size[1] / 2.0 - centre[1]
    return cv2.warpAffine(world, matrix, size, flags=cv2.INTER_LINEAR, borderValue=110), matrix


def at(matrix, point):
    return float(matrix[0, 0] * point[0] + matrix[0, 1] * point[1] + matrix[0, 2]), float(matrix[1, 0] * point[0] + matrix[1, 1] * point[1] + matrix[1, 2])


# --- the home marker ---------------------------------------------------------------------------------------

needs_aruco = pytest.mark.skipif(not home_marker.available(), reason="this OpenCV has no aruco module with the AprilTag 36h11 dictionary (opencv-python before 4.7 without contrib)")


@needs_aruco
def test_the_tag_is_the_families_grid_and_the_one_the_sim_lays_out():
    assert home_marker.available() and home_marker.CLASS_ID == "home_marker" and keyframe_homing.CLASS_ID == "takeoff_point" and home_marker.SIZE_M == 0.8
    grid = home_marker.cells(0)
    assert len(grid) == 8 and all(len(row) == 8 for row in grid)
    assert all(value == 0 for value in grid[0] + grid[-1]) and all(row[0] == 0 and row[-1] == 0 for row in grid)  # the black border
    assert ["".join(str(value) for value in row) for row in grid] == ["00000000", "00010000", "00110100", "00001010", "00001100", "01011100", "01010110", "00000000"]
    assert home_marker.cells(1) != grid
    # The sim's model of the tag is made of the same cells, laid out so that a camera above sees the tag and not its mirror image.
    sys.path.insert(0, str(SRC / "drone_utils"))
    from drone_utils import sdf_objects

    assert list(sdf_objects.HOME_TAG_CELLS) == ["".join(str(value) for value in row) for row in grid]
    sdf = sdf_objects.build_home_tag_sdf("home_tag")
    import re

    links = re.findall(r"<link name='(\w+)'><pose>(\S+) (\S+) (\S+) 0 0 \S+</pose><visual name='visual'><geometry><box><size>(\S+) (\S+) ", sdf)
    seen = np.full((120, 120), 120, dtype=np.uint8)  # the model drawn from above at 100 px a metre, north up
    for name, x, y, _z, sx, sy in sorted(links, key=lambda link: float(link[3])):
        left, right = int(round(60 + (float(x) - float(sx) / 2.0) * 100)), int(round(60 + (float(x) + float(sx) / 2.0) * 100))
        top, bottom = int(round(60 - (float(y) + float(sy) / 2.0) * 100)), int(round(60 - (float(y) - float(sy) / 2.0) * 100))
        seen[top:bottom, left:right] = 5 if name == "tag" else 255
    heights = sorted({round(float(z), 4) for _name, _x, _y, z, _sx, _sy in links})
    assert heights == [0.005, 0.015, 0.025] == list(sdf_objects.HOME_TAG_LAYERS_M)  # a centimetre apart: not drawn through one another from flying height
    found = home_marker.find(cv2.resize(seen, (480, 480), interpolation=cv2.INTER_NEAREST), 0)
    assert found is not None and math.hypot(found.u - 240.0, found.v - 240.0) < 2.0 and found.side_px == pytest.approx(320.0, rel=0.05)
    assert home_marker.find(cv2.resize(seen[:, ::-1], (480, 480), interpolation=cv2.INTER_NEAREST), 0) is None  # its mirror image is not the tag


@needs_aruco
@pytest.mark.parametrize("side_px, yaw_deg", [(160, 0.0), (60, 33.0), (28, -120.0)])
def test_the_home_marker_is_found_where_it_lies_turned_and_small(side_px, yaw_deg):
    tag = home_marker.picture(0, cell_px=20)  # 200 px, the border 160 px across
    world = np.full((1200, 1200), 120, dtype=np.uint8)
    world[400:600, 700:900] = tag
    centre_world = (800.0, 500.0)
    picture, matrix = view(world, (760.0, 540.0), side_px / 160.0, yaw_deg)
    found = home_marker.find(picture, 0)
    assert found is not None
    expected = at(matrix, centre_world)
    assert math.hypot(found.u - expected[0], found.v - expected[1]) < 1.5 and found.side_px == pytest.approx(side_px, rel=0.1)
    u, v, width, height = home_marker.box(found)
    assert (u, v) == (found.u, found.v) and width >= side_px * 0.9 and height >= side_px * 0.9
    assert home_marker.find(cv2.cvtColor(picture, cv2.COLOR_GRAY2BGR), 0) is not None  # a colour picture as well


@needs_aruco
def test_another_id_a_tag_too_small_and_bare_ground_are_not_the_home_marker():
    tag = home_marker.picture(5, cell_px=20)
    world = np.full((1200, 1200), 120, dtype=np.uint8)
    world[400:600, 700:900] = tag
    picture, _matrix = view(world, (800.0, 500.0), 0.5)
    assert home_marker.find(picture, 0) is None and home_marker.find(picture, 5) is not None
    tiny, _matrix = view(world, (800.0, 500.0), 0.08)  # 13 px across: under two pixels a cell
    assert home_marker.find(tiny, 5) is None
    assert home_marker.find(view(ground(), (800.0, 800.0), 0.5)[0], 0) is None
    assert home_marker.find(np.zeros((0, 0), dtype=np.uint8), 0) is None


# --- keyframe homing ---------------------------------------------------------------------------------------


def climb(world, home=(800.0, 800.0)):
    """The takeoff climb over `home`: a picture kept at each of the heights (the ground is 400 world pixels across the picture's width at 4 m... a pixel is height/160 m)."""
    homing = keyframe_homing.KeyframeHoming()
    for height in (4.0, 7.0, 10.0, 14.0):
        picture, matrix = view(world, home, 4.0 / height)
        assert homing.offer(picture, height, at(matrix, home))
    return homing


def test_pictures_are_kept_once_at_each_height_of_the_climb_and_only_with_the_takeoff_point_in_them():
    world = ground()
    homing = keyframe_homing.KeyframeHoming()
    picture, matrix = view(world, (800.0, 800.0), 1.0)
    assert homing.wanted(2.0) is None and homing.wanted(4.5) == 4.0 and homing.wanted(float("nan")) is None
    assert not homing.offer(picture, 2.0, (320.0, 240.0))  # not one of the heights
    assert not homing.offer(picture, 4.2, (900.0, 240.0))  # the takeoff point is not in the picture
    assert homing.offer(picture, 4.2, (320.0, 240.0)) and not homing.offer(picture, 4.4, (320.0, 240.0))  # once
    assert len(homing.keyframes) == 1 and homing.wanted(4.5) is None and homing.wanted(7.3) == 7.0
    bare = np.full((480, 640), 120, dtype=np.uint8)
    assert not homing.offer(bare, 7.0, (320.0, 240.0)) and "features" in homing.last_refusal
    homing.clear()
    assert homing.keyframes == [] and homing.find(picture, 4.0) is None and homing.last_refusal == "no picture was kept during the takeoff climb"


@pytest.mark.parametrize("height, offset, yaw_deg", [(5.0, (60.0, -40.0), 0.0), (6.0, (-90.0, 70.0), 25.0), (9.0, (120.0, 60.0), -40.0), (12.0, (-150.0, -110.0), 170.0)])
def test_the_takeoff_point_is_found_in_a_picture_from_another_place_height_and_heading(height, offset, yaw_deg):
    world, home = ground(), (800.0, 800.0)
    homing = climb(world, home)
    picture, matrix = view(world, (home[0] + offset[0], home[1] + offset[1]), 4.0 / height, yaw_deg)
    found = homing.find(picture, height)
    assert found is not None, homing.last_refusal
    expected = at(matrix, home)
    assert math.hypot(found.u - expected[0], found.v - expected[1]) < 3.0  # pixels: under 0.1 m on the ground at these heights
    assert found.inliers >= keyframe_homing.MIN_INLIERS and 0.0 < found.score <= 0.9
    assert found.scale == pytest.approx(found.keyframe_height_m / height, rel=0.1)
    # Without a height it is found as well (the size is then not checked against anything).
    assert homing.find(picture, None) is not None


def test_another_place_bare_ground_and_a_view_the_heights_contradict_are_not_the_takeoff_point():
    world, home = ground(), (800.0, 800.0)
    homing = climb(world, home)
    elsewhere, _matrix = view(ground(seed=11), (800.0, 800.0), 0.5)
    assert homing.find(elsewhere, 8.0) is None and homing.last_refusal
    assert homing.find(np.full((480, 640), 120, dtype=np.uint8), 8.0) is None and "features" in homing.last_refusal
    # The takeoff point out of the picture: the ground is recognised, and no place is given.
    away, _matrix = view(world, (home[0] + 700.0, home[1]), 4.0 / 7.0)
    assert homing.find(away, 7.0) is None
    # A picture that matches at a size the heights do not allow (said to be at 30 m, looking like 7 m).
    near, _matrix = view(world, (home[0] + 40.0, home[1]), 4.0 / 7.0)
    assert homing.find(near, 7.0) is not None and homing.find(near, 30.0) is None and "heights say" in homing.last_refusal


def test_the_takeoff_point_is_where_the_vehicle_stands_on_the_ground_and_pictures_are_the_climbs_until_it_first_leaves():
    climb = keyframe_homing.Climb()
    assert climb.home is None and not climb.keeping((0.0, 0.0, 5.0)) and not climb.searching((0.0, 0.0, 5.0)) and climb.height_m((0.0, 0.0, 5.0)) is None
    # Without the flight controller's word nothing is decided, however low and however still (the flight-safety review: a hover at the
    # lowest height seen was taken for the ground).
    assert not any(climb.update(0.1 * step, (3.0, 4.0, 0.12), None) for step in range(40)) and climb.home is None
    assert not any(climb.update(4.0 + 0.1 * step, (3.0, 4.0, 0.12), False) for step in range(40)) and climb.home is None
    took = [climb.update(10.0 + 0.1 * step, (3.0, 4.0, 0.12), True) for step in range(30)]  # on the ground for three seconds
    assert took.count(True) == 1 and took.index(True) == 20 and climb.home == (3.0, 4.0, 0.12)  # taken after two seconds still
    assert not climb.update(13.0, (float("nan"), 0.0, 0.0), True)
    # The local frame wanders 0.15 m while the vehicle stands: the place is kept up to date, and it is still the one takeoff.
    assert not any(climb.update(13.0 + 0.1 * step, (3.0 + 0.005 * step, 4.0, 0.12), True) for step in range(31))
    assert climb.home[0] == pytest.approx(3.15, abs=0.02) and climb.takeoffs == 1
    for step in range(40):  # straight up to 12 m
        climb.update(16.1 + 0.1 * step, (3.2, 4.05, 0.12 + 0.3 * step), False)
    assert climb.keeping((3.2, 4.05, 6.0)) and not climb.searching((3.2, 4.05, 6.0)) and climb.height_m((3.2, 4.05, 6.12)) == pytest.approx(6.0)
    climb.update(21.0, (9.0, 4.0, 12.0), False)  # off to the pickup
    assert climb.left and not climb.keeping((3.0, 4.0, 12.0)) and climb.searching((5.0, 5.0, 5.0)) and not climb.searching((30.0, 4.0, 5.0))
    # Low and still over the pickup, in the air: not a takeoff point (the ground there may be lower than the pad).
    assert not any(climb.update(30.0 + 0.1 * step, (40.0, 30.0, -0.5), False) for step in range(40)) and climb.home[0] == pytest.approx(3.15, abs=0.02)
    # Back and landed: the next takeoff is a new takeoff point, and says so (the pictures of the last are dropped by whoever holds them).
    landed = [climb.update(40.0 + 0.1 * step, (3.4, 4.2, 0.15), True) for step in range(30)]
    assert landed.count(True) == 1 and climb.home == (3.4, 4.2, 0.15) and not climb.left and climb.takeoffs == 2
