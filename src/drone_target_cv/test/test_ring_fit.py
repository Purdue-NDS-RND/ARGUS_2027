"""The near-field bullseye centre finder (CEN-13): a fit of concentric rings to the red and white edges. No ROS.

Near the ground the bullseye is larger than the picture, so a box around its
red pixels says where the visible part is, not where the centre is. The
rings' edges still do: every one of them is a circle about the same point.
The images are rendered by tools/synth_targets.py as the sim's downward
camera would see them (640 x 360, 389 px focal length) at 1.5 to 4 m.
"""

import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "drone_target_cv"))
sys.path.insert(0, str(ROOT / "src" / "drone_mission_core"))
sys.path.insert(0, str(ROOT / "tools"))

import synth_targets as synth  # noqa: E402
from drone_mission_core import detection_contract as contract  # noqa: E402
from drone_target_cv import ring_fit  # noqa: E402
from drone_target_cv.contract_output import bullseye_detection  # noqa: E402
from drone_target_cv.target_detector import RedTargetDetector  # noqa: E402

FOCAL_PX = 389.16536
WIDTH, HEIGHT = 640, 360


def view(height_m, offset_px=(0.0, 0.0), size_m=5.0, rings=8, tilt_deg=(0.0, 0.0), blur=0.0, noise=0.0, seed=1, rotation_deg=0.0):
    """(image, the true centre in pixels) of a bullseye from `height_m` up, its centre `offset_px` from the image's centre."""
    options = synth.RenderOptions(
        width=WIDTH, height=HEIGHT, mpp=height_m / FOCAL_PX, altitude_m=height_m, offset_px=offset_px, tilt_deg=tilt_deg,
        blur_sigma_px=blur, noise_sigma=noise, rotation_deg=rotation_deg, supersample=2,
    )  # fmt: skip
    image, truth = synth.render(synth.Bullseye(size_m=size_m, num_rings=rings), options, seed=seed)
    return image, tuple(truth["center_px"])


def level_for(height_m, tilt_deg, rotation_deg=0.0):
    """The levelling homography for a rendered view: the camera matrix, and the camera's attitude over the target's plane.

    The renderer tilts the target's plane under a camera that looks straight
    down. Seen from the plane, which is the ground, that is a tilted camera:
    its attitude is the inverse of the plane's.
    """
    rot, tx, ty = math.radians(rotation_deg), math.radians(tilt_deg[0]), math.radians(tilt_deg[1])
    rz = np.array([[math.cos(rot), -math.sin(rot), 0], [math.sin(rot), math.cos(rot), 0], [0, 0, 1]])
    rx = np.array([[1, 0, 0], [0, math.cos(tx), -math.sin(tx)], [0, math.sin(tx), math.cos(tx)]])
    ry = np.array([[math.cos(ty), 0, math.sin(ty)], [0, 1, 0], [-math.sin(ty), 0, math.cos(ty)]])
    optical_from_ground = np.diag([1.0, -1.0, -1.0]) @ (ry @ rx @ rz)
    k = np.array([[FOCAL_PX, 0, (WIDTH - 1) / 2], [0, FOCAL_PX, (HEIGHT - 1) / 2], [0, 0, 1]])
    return ring_fit.nadir_homography(k, optical_from_ground.T)


def error_px(fit, truth):
    return math.hypot(fit.center[0] - truth[0], fit.center[1] - truth[1])


@pytest.mark.parametrize("height_m", [1.5, 2.4, 3.0, 4.0])
@pytest.mark.parametrize("size_m, rings", [(5.0, 8), (1.5, 4)])
def test_the_centre_is_found_to_a_pixel_from_1_5_to_4_m(height_m, size_m, rings):
    image, truth = view(height_m, offset_px=(37.0, -22.0), size_m=size_m, rings=rings)
    fit = ring_fit.fit_rings(image)
    assert fit is not None, "no fit"
    assert error_px(fit, truth) < 1.0
    assert fit.rings >= 2 and fit.rms_px < 1.5 and fit.inlier_fraction > 0.7


@pytest.mark.parametrize("offset_px", [(250.0, 0.0), (-280.0, 120.0), (0.0, 150.0), (200.0, -150.0)])
def test_a_target_cut_by_the_images_edge_still_gives_its_centre_where_a_box_does_not(offset_px):
    # At 2.4 m the 5 m bullseye is 810 px across in a 640 x 360 picture: its edges are never all in view.
    image, truth = view(2.4, offset_px=offset_px)
    fit = ring_fit.fit_rings(image)
    assert fit is not None and error_px(fit, truth) < 1.5


@pytest.mark.parametrize("tilt_deg", [(8.0, 0.0), (0.0, -8.0), (6.0, 6.0), (-10.0, 10.0)])
def test_tilted_views_are_levelled_with_the_cameras_attitude_and_then_fitted(tilt_deg):
    errors = []
    for seed, height_m in enumerate((1.5, 2.4, 4.0), start=1):
        image, truth = view(height_m, offset_px=(60.0, 40.0), tilt_deg=tilt_deg, blur=1.2, noise=6.0, seed=seed)
        fit = ring_fit.fit_rings(image, level=level_for(height_m, tilt_deg))
        assert fit is not None, (tilt_deg, height_m)
        errors.append(error_px(fit, truth))
    assert max(errors) < 2.0, errors


def test_over_sixty_random_views_with_tilt_blur_and_noise_the_centre_is_within_the_noise_the_estimator_is_told():
    rng = np.random.default_rng(3)
    errors = []
    for index in range(60):
        height_m = float(rng.uniform(1.5, 4.0))
        tilt = (float(rng.uniform(-10.0, 10.0)), float(rng.uniform(-10.0, 10.0)))
        offset = (float(rng.uniform(-200.0, 200.0)), float(rng.uniform(-120.0, 120.0)))
        size_m, rings = ((5.0, 8), (1.5, 4))[index % 2]
        image, truth = view(height_m, offset_px=offset, size_m=size_m, rings=rings, tilt_deg=tilt, blur=1.0, noise=5.0, seed=index)
        fit = ring_fit.fit_rings(image, level=level_for(height_m, tilt))
        assert fit is not None, (index, height_m, tilt, offset, size_m)
        errors.append(error_px(fit, truth))
    errors.sort()
    assert errors[len(errors) // 2] < 0.6 and errors[56] < 1.5 and errors[-1] < 3.0, (errors[30], errors[56], errors[-1])
    # The noise model the estimator is given for this detector covers what was measured here.
    assert ring_fit.PIXEL_SIGMA_PX == 2.0 and errors[56] < ring_fit.PIXEL_SIGMA_PX


def test_without_the_attitude_a_strongly_tilted_view_is_declined_or_worse_never_better():
    image, truth = view(1.5, offset_px=(60.0, 40.0), tilt_deg=(0.0, -8.0))
    levelled = ring_fit.fit_rings(image, level=level_for(1.5, (0.0, -8.0)))
    bare = ring_fit.fit_rings(image)
    assert levelled is not None and error_px(levelled, truth) < 1.5
    assert bare is None or error_px(bare, truth) > error_px(levelled, truth)


def test_the_levelling_homography_is_the_identity_for_a_camera_straight_down_and_refuses_one_that_is_not_looking_down():
    k = np.array([[400.0, 0, 320.0], [0, 400.0, 180.0], [0, 0, 1]])
    straight_down = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])  # optical x east, y south, lens down
    assert np.allclose(ring_fit.nadir_homography(k, straight_down), np.eye(3), atol=1e-9)
    sideways = np.array([[0.0, 0, 1.0], [-1.0, 0, 0], [0, -1.0, 0]])  # lens toward the east
    with pytest.raises(ValueError, match="straight down"):
        ring_fit.nadir_homography(k, sideways)
    # Circles on the ground, seen by a camera rolled 12 degrees and pitched 7: levelled, they are circles about one point again.
    roll, pitch = math.radians(12.0), math.radians(7.0)
    rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    world_from_optical = ry @ rx @ straight_down
    camera = np.array([0.3, -0.2, 2.4])
    centre_ground = np.array([0.5, 0.1, 0.0])

    def pixel(point):
        ray = world_from_optical.T @ (point - camera)
        return (k @ (ray / ray[2]))[:2]

    level = ring_fit.nadir_homography(k, world_from_optical)

    def levelled(point):
        moved = level @ np.append(pixel(point), 1.0)
        return moved[:2] / moved[2]

    middle = levelled(centre_ground)
    for radius in (0.2, 0.6, 1.1):
        ring = [levelled(centre_ground + radius * np.array([math.cos(a), math.sin(a), 0.0])) for a in np.linspace(0, 2 * math.pi, 24, endpoint=False)]
        distances = [float(np.hypot(*(point - middle))) for point in ring]
        assert max(distances) - min(distances) < 1e-6 * max(distances)
        raw = [float(np.hypot(*(pixel(centre_ground + radius * np.array([math.cos(a), math.sin(a), 0.0])) - pixel(centre_ground)))) for a in np.linspace(0, 2 * math.pi, 24, endpoint=False)]
        assert max(raw) - min(raw) > 0.02 * max(raw)  # unlevelled, they are not


def test_where_the_2026_detectors_box_is_cut_by_the_edge_the_ring_fit_still_has_the_centre():
    # The sim's hold target, 1.5 m with four rings, from 2.2 m with the view moved 0.3 m by the vehicle's tilt in wind:
    # the 2026 detector's box runs into the picture's edge, which the estimator will not take as a centre.
    image, truth = view(2.2, offset_px=(0.0, 75.0), size_m=1.5, rings=4)
    box = bullseye_detection(RedTargetDetector().detect(image))
    assert box is not None and box.cy + box.height / 2 >= HEIGHT - 2
    fit = ring_fit.fit_rings(image)
    assert fit is not None and error_px(fit, truth) < 1.5
    detection = ring_fit.to_detection(fit)
    assert detection.cy + detection.height / 2 < HEIGHT - 2 and detection.cy - detection.height / 2 > 1


def test_a_centre_outside_the_picture_is_not_guessed_at():
    image, _truth = view(2.4, offset_px=(420.0, 0.0))  # the centre is 100 px beyond the right edge; rings are in view
    assert ring_fit.fit_rings(image) is None


def test_what_is_not_a_bullseye_gives_nothing():
    plain = np.full((HEIGHT, WIDTH, 3), (85, 120, 95), np.uint8)
    assert ring_fit.fit_rings(plain) is None
    disc = plain.copy()
    import cv2

    cv2.circle(disc, (300, 180), 90, (0, 0, 255), -1)  # one red disc: one edge, no rings
    assert ring_fit.fit_rings(disc) is None
    stripes = plain.copy()
    for x in range(0, WIDTH, 80):
        cv2.rectangle(stripes, (x, 0), (x + 40, HEIGHT), (0, 0, 255), -1)  # red and white, but straight
    assert ring_fit.fit_rings(stripes) is None
    two = plain.copy()
    cv2.circle(two, (150, 180), 70, (0, 0, 255), -1)
    cv2.circle(two, (470, 180), 70, (0, 0, 255), -1)  # two discs with different centres are not rings about one
    assert ring_fit.fit_rings(two) is None


def test_far_away_the_rings_are_too_fine_and_the_fit_declines():
    image, _truth = view(30.0, size_m=1.5, rings=8)  # rings 1.2 px wide
    assert ring_fit.fit_rings(image) is None


def _least_cpu_s(run, repeats=5):
    """The least processor time of several runs. A busy machine makes the wall clock say anything it likes about
    this code; the processor time the code itself used, at its least over a few runs, it barely moves."""
    least = float("inf")
    for _ in range(repeats):
        started = time.process_time()
        run()
        least = min(least, time.process_time() - started)
    return least


def test_the_fit_is_the_same_every_time_and_fast_enough_for_every_frame():
    image, truth = view(2.4, offset_px=(100.0, 50.0), blur=1.0, noise=4.0)
    first = ring_fit.fit_rings(image)
    assert first == ring_fit.fit_rings(image.copy())
    # 30 frames a second leaves 33 ms, and a desktop takes about 23 ms. The line is ten times that, the room the other timed tests have:
    # at 0.06 s a machine with three times as much to do as it has cores failed it (0.061 s) with the fit unchanged. The Jetson's own
    # time per frame is a bench number, not this one; this catches a fit that has become several times slower.
    assert _least_cpu_s(lambda: ring_fit.fit_rings(image), repeats=10) < 0.25


def test_the_detection_for_the_contract_is_a_bullseye_whose_box_is_the_innermost_ring():
    image, truth = view(2.4, offset_px=(200.0, -100.0))
    fit = ring_fit.fit_rings(image)
    detection = ring_fit.to_detection(fit)
    assert detection.class_id == "bullseye" and detection.class_id in contract.CLASSES
    assert (detection.cx, detection.cy) == fit.center and 0.0 < detection.score <= 1.0
    # The box is the innermost ring fitted, about the centre: inside the picture although the target is not.
    assert detection.width == detection.height == pytest.approx(2.0 * min(fit.radii_px))
    assert detection.cx - detection.width / 2 > 0 and detection.cx + detection.width / 2 < WIDTH
    assert detection.cy - detection.height / 2 > 0 and detection.cy + detection.height / 2 < HEIGHT
    assert ring_fit.to_detection(None) is None


def test_colour_thresholds_can_be_the_aircrafts_or_the_sims():
    image, truth = view(2.4)
    dim = (image.astype(np.float32) * 0.55).astype(np.uint8)  # a dull day
    fit = ring_fit.fit_rings(dim, ring_fit.RingFitConfig(hsv_s_min=80, hsv_v_min=45))
    assert fit is not None and error_px(fit, truth) < 1.5
    assert ring_fit.fit_rings(dim, ring_fit.RingFitConfig(hsv_v_min=200)) is None  # nothing is red enough
