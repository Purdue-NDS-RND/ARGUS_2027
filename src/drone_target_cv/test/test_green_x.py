"""The green X and bucket detector (REC-1) on synthetic renders: tools/synth_targets.py draws them with their truth."""

import math
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "drone_target_cv"), str(SRC / "drone_mission_core"), str(SRC.parent / "tools")]

import synth_targets  # noqa: E402
from drone_target_cv import green_x as gx  # noqa: E402

FOCAL_PX = 778.0  # the downward camera's focal length at 1280 px wide (the spec's number)
ARM_SPAN_M = 2.0 * (2.5 * math.sqrt(2.0) - 0.25)  # arm tip to arm tip of the 5 m mat


def x_mat(height_m=10.0, rotation_deg=0.0, offset=(0.0, 0.0), tilt=(0.0, 0.0), bucket=False, seed=0, blur=0.0, noise=0.0, brightness=1.0, background="noise", shape="square", size=(1280, 720)):
    """The X mat as the downward camera sees it from `height_m`, and its truth."""
    options = synth_targets.RenderOptions(
        width=size[0], height=size[1], mpp=height_m / FOCAL_PX, rotation_deg=rotation_deg, offset_px=offset, tilt_deg=tilt,
        blur_sigma_px=blur, noise_sigma=noise, brightness=brightness, background=background, supersample=2,
    )  # fmt: skip
    return synth_targets.render(synth_targets.XMat(shape=shape, bucket=bucket), options, seed=seed)


def angle_error_deg(found_rad, rotation_deg):
    """The X's arms are on the mat's diagonals: 45 degrees from the image's axes at zero rotation. The period is 90."""
    true = (45.0 - rotation_deg) % 90.0  # image y is down: a counter-clockwise turn lowers the angle
    error = abs((math.degrees(found_rad) - true + 45.0) % 90.0 - 45.0)
    return error


def span_px(height_m):
    return ARM_SPAN_M * FOCAL_PX / height_m


@pytest.mark.parametrize("height_m", [3.0, 5.0, 8.0, 12.0, 20.0, 30.0])
def test_the_x_is_found_from_three_to_thirty_metres(height_m):
    image, truth = x_mat(height_m, rotation_deg=17.0, offset=(40.0, -25.0))
    found = gx.detect(image).x
    assert found is not None, height_m
    error = math.hypot(found.cx - truth["center_px"][0], found.cy - truth["center_px"][1])
    assert error <= max(2.0, 0.01 * span_px(height_m)), (height_m, error)
    assert angle_error_deg(found.angle_rad, 17.0) <= 3.0 and 0.0 <= found.angle_rad < math.pi / 2.0
    assert 0.5 <= found.score <= 1.0
    # The span it reports is the arms', tip to tip, whether or not the tips are in the picture.
    assert found.span_px == pytest.approx(span_px(height_m), rel=0.12)


def test_the_centre_is_where_the_arms_cross_when_the_bucket_covers_it():
    for height_m, rotation in ((4.0, 30.0), (10.0, 62.0)):
        image, truth = x_mat(height_m, rotation_deg=rotation, bucket=True, offset=(-60.0, 35.0))
        result = gx.detect(image)
        assert result.x is not None
        assert math.hypot(result.x.cx - truth["center_px"][0], result.x.cy - truth["center_px"][1]) <= max(2.0, 0.01 * span_px(height_m))
        # And the bucket is found on it: a blob of about its size at the centre.
        bucket = result.bucket
        assert bucket is not None and math.hypot(bucket.cx - truth["center_px"][0], bucket.cy - truth["center_px"][1]) <= max(2.0, 0.15 * bucket.width)
        assert bucket.width == pytest.approx(0.2 * FOCAL_PX / height_m, rel=0.35) and 0.0 < bucket.score <= 1.0
    # No bucket on the mat: none is reported.
    image, _truth = x_mat(6.0, rotation_deg=10.0)
    assert gx.detect(image).bucket is None


def test_the_x_is_found_tilted_blurred_dim_and_on_a_round_mat():
    cases = [
        dict(height_m=10.0, rotation_deg=40.0, tilt=(12.0, -8.0)),
        dict(height_m=15.0, rotation_deg=75.0, blur=1.5, noise=4.0),
        dict(height_m=8.0, rotation_deg=5.0, brightness=0.7),
        dict(height_m=8.0, rotation_deg=5.0, brightness=1.2),
        dict(height_m=12.0, rotation_deg=22.0, shape="circle"),
    ]
    for index, case in enumerate(cases):
        image, truth = x_mat(seed=index, **case)
        found = gx.detect(image).x
        assert found is not None, case
        assert math.hypot(found.cx - truth["center_px"][0], found.cy - truth["center_px"][1]) <= max(2.0, 0.01 * span_px(case["height_m"])) + (2.0 if "tilt" in case else 0.0), case


def test_green_that_is_not_the_x_is_not_reported():
    """Grass, a green tarp, a green cross with no white round it, a bullseye, a numbered marker: nothing."""
    rng = np.random.default_rng(3)
    grass = np.clip(np.array([60, 140, 70], dtype=np.float32) + rng.normal(0.0, 18.0, (720, 1280, 3)), 0, 255).astype(np.uint8)
    assert gx.detect(grass).x is None
    tarp = grass.copy()
    cv2.rectangle(tarp, (400, 200), (800, 500), (30, 170, 40), -1)  # a green tarp on the grass
    assert gx.detect(tarp).x is None
    cross = np.full((720, 1280, 3), (70, 90, 80), dtype=np.uint8)  # a green cross painted on tarmac: no white surround
    cv2.line(cross, (440, 160), (840, 560), (0, 160, 0), 30)
    cv2.line(cross, (840, 160), (440, 560), (0, 160, 0), 30)
    assert gx.detect(cross).x is None
    bar = np.full((720, 1280, 3), 242, dtype=np.uint8)  # one green bar on white: an arm with no second arm
    cv2.line(bar, (300, 360), (980, 360), (0, 160, 0), 40)
    assert gx.detect(bar).x is None
    for target in (synth_targets.Bullseye(), synth_targets.NumberedMarker(digit=8), synth_targets.SurveyMarker(digit=4)):
        mpp = 10.0 / FOCAL_PX if target.name == "bullseye" else 0.004
        image, _ = synth_targets.render(target, synth_targets.RenderOptions(mpp=mpp, background="noise", supersample=2), seed=1)
        assert gx.detect(image).x is None, target.name
    assert gx.detect(np.zeros((360, 640, 3), dtype=np.uint8)).x is None


def test_the_confidence_is_made_of_its_four_parts_and_each_can_sink_it():
    image, _ = x_mat(10.0, rotation_deg=20.0)
    found = gx.detect(image).x
    assert set(found.parts) == {"white_surround", "arm_angle", "inlier_fraction", "arm_width"} and all(0.0 <= value <= 1.0 for value in found.parts.values())
    assert found.score == pytest.approx(math.prod(found.parts.values()), rel=1.0e-6) and found.score > 0.7
    assert gx.MIN_SCORE == 0.5


def test_a_handle_across_the_bucket_is_its_longest_line():
    """Close in, the bucket's handle is a line across it: reported as its two ends. Far off there is none to see."""
    image, truth = x_mat(3.0, rotation_deg=0.0, bucket=True)
    cx, cy = truth["center_px"]
    radius = 0.1 * FOCAL_PX / 3.0  # the bucket is 26 px in radius from 3 m
    angle = math.radians(25.0)
    a = (int(round(cx - radius * math.cos(angle))), int(round(cy - radius * math.sin(angle))))
    b = (int(round(cx + radius * math.cos(angle))), int(round(cy + radius * math.sin(angle))))
    cv2.line(image, a, b, (25, 25, 25), 3)
    bucket = gx.detect(image).bucket
    assert bucket is not None and bucket.handle is not None
    (x1, y1), (x2, y2) = bucket.handle
    found = math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180.0
    assert abs((found - 25.0 + 90.0) % 180.0 - 90.0) <= 6.0 and math.hypot(x2 - x1, y2 - y1) >= 1.2 * radius
    far, _ = x_mat(20.0, bucket=True)
    assert gx.detect(far).bucket is None or gx.detect(far).bucket.handle is None


def test_a_few_hundred_renders_meet_the_specs_lines_for_recall_and_false_alarms():
    """The spec's pass on synthetic renders, on a sample: recall at least 98% from 3 to 30 m with the centre and
    angle within their lines, and nothing on bullseyes, markers and green noise. tools/green_x_try.py runs the full count."""
    rng = np.random.default_rng(5)
    hits = 0
    count = 60
    for index in range(count):
        height_m = float(rng.uniform(3.0, 30.0))
        rotation = float(rng.uniform(0.0, 360.0))
        reach = 0.25 * min(1280, 720)
        offset = (float(rng.uniform(-reach, reach)), float(rng.uniform(-reach, reach)))
        image, truth = x_mat(height_m, rotation_deg=rotation, offset=offset, tilt=(float(rng.uniform(-8, 8)), float(rng.uniform(-8, 8))), bucket=bool(index % 2), seed=index, blur=float(rng.uniform(0, 1)), noise=float(rng.uniform(0, 4)), brightness=float(rng.uniform(0.8, 1.15)))
        found = gx.detect(image).x
        if found is None:
            continue
        centre_ok = math.hypot(found.cx - truth["center_px"][0], found.cy - truth["center_px"][1]) <= max(2.0, 0.01 * span_px(height_m))
        hits += centre_ok and angle_error_deg(found.angle_rad, rotation) <= 3.0
    assert hits >= 0.98 * count, (hits, count)


def test_the_x_is_found_whatever_else_is_white_in_the_picture():
    """The first sim flight: with the whole mat in view the X was not reported, and with it half out of view it was.
    Something else white and larger than the mat's own pieces (a runway's line, another mat) must not hide it."""
    image, truth = x_mat(10.67, rotation_deg=25.0, offset=(-80.0, 40.0), bucket=True)
    cv2.rectangle(image, (0, 20), (1279, 90), (240, 240, 240), -1)  # a white stripe across the picture, three times a mat piece's size
    cv2.rectangle(image, (1050, 400), (1250, 700), (235, 235, 235), -1)  # and a white pad beside the mat
    found = gx.detect(image)
    assert found.x is not None and math.hypot(found.x.cx - truth["center_px"][0], found.x.cy - truth["center_px"][1]) <= max(2.0, 0.01 * span_px(10.67))
    assert found.bucket is not None
    # A white pad with grass showing through a hole in it is green shut in by white, and is not an X.
    pad = np.full((720, 1280, 3), 240, dtype=np.uint8)
    cv2.circle(pad, (640, 360), 120, (60, 140, 70), -1)
    assert gx.detect(pad).x is None


def test_a_shadow_or_a_hook_over_the_crossing_is_not_a_bucket():
    """The flight-safety review: the bucket was any blob of about its size at the crossing. A bucket from above is
    round and solid; a long dark thing (a hook under the camera, a shadow's edge) is not reported as one."""
    image, truth = x_mat(6.0, rotation_deg=10.0)
    cx, cy = (int(round(v)) for v in truth["center_px"])
    hooked = image.copy()
    cv2.line(hooked, (cx - 22, cy - 4), (cx + 22, cy + 4), (20, 20, 20), 5)  # a rod across the crossing, 45 px by 5
    result = gx.detect(hooked)
    assert result.x is not None and result.bucket is None
    ragged = image.copy()
    for dx, dy in ((-10, -8), (9, -9), (-8, 10), (10, 8), (0, 0)):  # a scatter of dark specks, about a bucket's area together
        cv2.circle(ragged, (cx + dx, cy + dy), 4, (30, 30, 30), -1)
    assert gx.detect(ragged).bucket is None
    assert (gx.BUCKET_MIN_SOLIDITY, gx.BUCKET_MAX_ELONGATION) == (0.6, 2.0)


def _least_cpu_s(run, repeats=5):
    """The least processor time of several runs. A busy machine makes the wall clock say anything it likes about
    this code; the processor time the code itself used, at its least over a few runs, it barely moves."""
    import time

    least = float("inf")
    for _ in range(repeats):
        started = time.process_time()
        run()
        least = min(least, time.process_time() - started)
    return least


def test_the_closing_of_the_white_is_bounded_however_pale_the_picture():
    """A picture that is mostly pale ground, or the mat from close in, closes the white over a wide gap: done on a
    quarter-size mask, and the X is still found where it is."""
    image, truth = x_mat(4.0, rotation_deg=30.0, offset=(20.0, -10.0))
    found = gx.detect(image).x
    assert found is not None and math.hypot(found.cx - truth["center_px"][0], found.cy - truth["center_px"][1]) <= max(2.0, 0.01 * span_px(4.0))
    pale = np.full((720, 1280, 3), 235, dtype=np.uint8)  # sunlit concrete, edge to edge
    assert gx.detect(pale).x is None and gx.QUARTER_CLOSE_OVER_PX == 41
    assert _least_cpu_s(lambda: gx.detect(pale), repeats=3) < 0.5  # processor time over every thread OpenCV uses, so no less than the wall clock's on an idle machine


# --- the near field (REC-5, the hand-off under 3 m): the mat fills the picture and the bucket is found by itself ---

SIM_SIZE, SIM_FOCAL_PX = (640, 360), 389.0  # the sim's downward camera


def near_mat(height_m, offset=(0.0, 0.0), rotation_deg=20.0, **more):
    options = synth_targets.RenderOptions(width=SIM_SIZE[0], height=SIM_SIZE[1], mpp=height_m / SIM_FOCAL_PX, rotation_deg=rotation_deg, offset_px=offset, background="noise", supersample=2, **more)
    return synth_targets.render(synth_targets.XMat(bucket=True), options, seed=2)


@pytest.mark.parametrize("height_m", [2.0, 1.5, 1.2, 1.0, 0.9])
def test_under_the_height_where_no_x_can_be_fitted_the_bucket_is_still_found_where_it_is(height_m):
    """From 2 m down the arms run out of every side of the picture: before, nothing at all was reported from there."""
    for offset in ((0.0, 0.0), (60.0, -30.0), (-150.0, 80.0)):
        image, truth = near_mat(height_m, offset)
        found = gx.detect(image)
        assert found.x is None and found.bucket is not None, (height_m, offset)
        cx, cy = truth["center_px"]
        assert math.hypot(found.bucket.cx - cx, found.bucket.cy - cy) <= 2.0
        assert found.bucket.width == pytest.approx(0.2 * SIM_FOCAL_PX / height_m, rel=0.08)  # the bucket's own size in the picture
        assert [item.class_id for item in gx.to_detections(found)] == ["bucket"]  # on the contract: a bucket and no X


def test_the_near_bucket_is_held_to_the_x_it_stands_on_the_crossing_and_is_a_buckets_size():
    """The flight-safety review: the near search was tied to nothing but colour and shape, and the descent runs on what it reports."""
    def mat_without_bucket(height_m=1.5, offset=(0.0, 0.0)):
        options = synth_targets.RenderOptions(width=SIM_SIZE[0], height=SIM_SIZE[1], mpp=height_m / SIM_FOCAL_PX, rotation_deg=20.0, offset_px=offset, background="noise", supersample=2)
        return synth_targets.render(synth_targets.XMat(bucket=False), options, seed=2)

    orange = (20, 110, 235)
    image, truth = mat_without_bucket()
    cx, cy = truth["center_px"]
    bucket_px = 0.2 * SIM_FOCAL_PX / 1.5
    # A coloured thing of a bucket's size lying along one arm, half a metre from the crossing: a circle round it cuts two arms, not four.
    along = image.copy()
    step = 0.9 * SIM_FOCAL_PX / 1.5  # 0.9 m along the arm (the arms are on the mat's diagonals, turned 20 degrees)
    angle = math.radians(45.0 + 20.0)
    spot = (int(round(cx + step * math.cos(angle))), int(round(cy - step * math.sin(angle))))
    cv2.circle(along, spot, int(round(bucket_px / 2)), orange, -1)
    found = gx.detect(along).bucket
    assert found is None or math.hypot(found.cx - spot[0], found.cy - spot[1]) > 20.0, "a blob along one arm was taken for the bucket"
    # A thing on the crossing far too big, or far too small, for a bucket against the arm's width.
    for radius in (int(round(1.3 * bucket_px)), int(round(0.12 * bucket_px))):
        wrong = image.copy()
        cv2.circle(wrong, (int(round(cx)), int(round(cy))), radius, orange, -1)
        assert gx.detect(wrong).bucket is None, radius
    # The same size on the crossing is the bucket.
    right = image.copy()
    cv2.circle(right, (int(round(cx)), int(round(cy))), int(round(bucket_px / 2)), orange, -1)
    assert gx.detect(right).bucket is not None
    # The delivery's red-and-white target on green ground, from close: green and white fill the picture, a red disc is ringed in white. Not a bucket.
    rng = np.random.default_rng(9)
    for height_m in (1.0, 2.0, 3.0, 5.0):
        grass = np.clip(np.array([60, 140, 70], dtype=np.float32) + rng.normal(0.0, 14.0, (360, 640, 3)), 0, 255).astype(np.uint8)
        options = synth_targets.RenderOptions(width=640, height=360, mpp=height_m / SIM_FOCAL_PX, background="plain", background_color=(70, 140, 60), supersample=2)
        target, _ = synth_targets.render(synth_targets.Bullseye(size_m=1.5), options, seed=1)
        mask = np.abs(target.astype(int) - np.array([60, 140, 70])).sum(axis=2) > 40  # the target's own pixels, laid on the noisy grass
        grass[mask] = target[mask]
        result = gx.detect(grass)
        assert result.x is None and result.bucket is None, height_m
    # Under the height where the crossing's circle fits the picture nothing is reported, rather than a blob held to nothing.
    for offset in ((0.0, 0.0), (60.0, -30.0)):
        low, _ = near_mat(0.6, offset)
        assert gx.detect(low).bucket is None


def test_the_near_bucket_is_found_blurred_dim_noisy_and_with_its_handle():
    image, truth = near_mat(1.2, (40.0, 20.0), blur_sigma_px=1.5, noise_sigma=6.0, brightness=0.8)
    found = gx.detect(image).bucket
    assert found is not None and math.hypot(found.cx - truth["center_px"][0], found.cy - truth["center_px"][1]) <= 3.0
    image, truth = near_mat(1.0)
    cx, cy = truth["center_px"]
    radius = 0.1 * SIM_FOCAL_PX / 1.0
    angle = math.radians(60.0)
    cv2.line(image, (int(round(cx - radius * math.cos(angle))), int(round(cy - radius * math.sin(angle)))), (int(round(cx + radius * math.cos(angle))), int(round(cy + radius * math.sin(angle)))), (25, 25, 25), 3)
    bucket = gx.detect(image).bucket
    assert bucket is not None and bucket.handle is not None
    (x1, y1), (x2, y2) = bucket.handle
    assert abs((math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180.0 - 60.0 + 90.0) % 180.0 - 90.0) <= 6.0


def test_a_coloured_thing_that_is_not_on_the_mat_seen_from_close_is_not_a_bucket():
    """The near search looks only at a picture that is mostly the mat's green and white, with mat all round the blob."""
    rng = np.random.default_rng(5)
    grass = np.clip(np.array([60, 140, 70], dtype=np.float32) + rng.normal(0.0, 18.0, (360, 640, 3)), 0, 255).astype(np.uint8)
    cv2.circle(grass, (320, 180), 40, (20, 110, 235), -1)  # an orange ball on grass: green all round it, and no white anywhere
    assert gx.detect(grass).bucket is None
    for target, mpp in ((synth_targets.Bullseye(), 1.5 / SIM_FOCAL_PX), (synth_targets.Bullseye(), 4.0 / SIM_FOCAL_PX), (synth_targets.NumberedMarker(digit=8), 0.002), (synth_targets.SurveyMarker(digit=4), 0.002)):
        image, _ = synth_targets.render(target, synth_targets.RenderOptions(width=640, height=360, mpp=mpp, background="noise", supersample=2), seed=1)
        found = gx.detect(image)
        assert found.x is None and found.bucket is None, target.name  # red on white from close: no green, so not the mat
    # The mat from close with no bucket on it: nothing.
    empty, _ = synth_targets.render(synth_targets.XMat(bucket=False), synth_targets.RenderOptions(width=640, height=360, mpp=1.2 / SIM_FOCAL_PX, rotation_deg=20.0, background="noise", supersample=2), seed=2)
    assert gx.detect(empty).bucket is None
    # A coloured thing at the mat's edge, half on the grass: the ring round it is not mat all the way.
    image, truth = near_mat(1.5, (0.0, 0.0))
    mostly_mat = image.copy()
    mostly_mat[:, :200] = (40, 60, 90)  # brown earth down the left of the picture (green grass there could not be told from an arm: see the notes)
    cv2.circle(mostly_mat, (200, 300), 30, (20, 110, 235), -1)  # an orange thing straddling the edge
    found = gx.detect(mostly_mat).bucket
    assert found is None or math.hypot(found.cx - truth["center_px"][0], found.cy - truth["center_px"][1]) <= 3.0  # if anything is reported it is the real bucket
    # A long coloured streak across the crossing (a strap, a shadow's edge in colour) is not round.
    streak, _ = synth_targets.render(synth_targets.XMat(bucket=False), synth_targets.RenderOptions(width=640, height=360, mpp=1.2 / SIM_FOCAL_PX, rotation_deg=20.0, background="noise", supersample=2), seed=2)
    cv2.line(streak, (200, 180), (440, 180), (20, 110, 235), 24)
    assert gx.detect(streak).bucket is None


def test_from_where_the_x_is_still_seen_nothing_changes():
    """At 3 m and above the X is fitted and the bucket is the one on its crossing, as before the near search."""
    for height_m in (3.0, 4.0, 6.0):
        image, truth = near_mat(height_m)
        found = gx.detect(image)
        assert found.x is not None and found.bucket is not None
        assert [item.class_id for item in gx.to_detections(found)] == ["green_x", "bucket"]
