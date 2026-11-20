"""The digit reader (LOC-6) on synthetic markers: tools/synth_targets.py draws them with their truth."""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "argus_localisation"), str(SRC / "drone_mission_core"), str(SRC / "argus_camera"), str(SRC.parent / "tools")]

import synth_targets  # noqa: E402
from argus_localisation import digit_reader as dr  # noqa: E402


def marker(digit, rotation_deg=0.0, tilt=(0.0, 0.0), mpp=0.0045, blur=0.0, noise=0.0, size=260, seed=0, background="noise", brightness=1.0, plate=False, marker_m=0.6):
    """A crop with a marker (133 px across at 4.5 mm a pixel, the 4K frame's scale from 35 ft) and its truth.

    The 2027 marker (digit centred on the X) unless `plate`, which is last year's (digit in the right-hand triangle).
    """
    options = synth_targets.RenderOptions(
        width=size, height=size, mpp=mpp, rotation_deg=rotation_deg, tilt_deg=tilt, blur_sigma_px=blur, noise_sigma=noise, background=background, supersample=2, brightness=brightness
    )
    target = synth_targets.NumberedMarker(digit=digit) if plate else synth_targets.SurveyMarker(digit=digit, size_m=marker_m)
    image, truth = synth_targets.render(target, options, seed=seed)
    return SimpleNamespace(image=image, origin=(0, 0)), truth


def reader(layout=dr.LAYOUT_2027, **settings):
    return dr.DigitReader(dr.TemplateDigitClassifier(layout), layout=layout, **settings)


def plate_reader(**settings):
    return reader(dr.LAYOUT_2026, **settings)


class Scripted:
    """A classifier that answers by the order it is asked in: one (digit, probability) for each quarter turn."""

    name = "scripted"

    def __init__(self, *answers):
        self.answers, self.asked = answers, 0

    def log_probabilities(self, patch):
        digit, probability = self.answers[self.asked % len(self.answers)]
        self.asked += 1
        scores = np.full(10, np.log((1.0 - probability) / 9.0))
        scores[digit] = np.log(probability)
        return scores


def test_the_readers_layouts_are_the_ones_the_markers_are_drawn_with():
    assert dr.DIGIT_SEGMENTS == synth_targets.DIGIT_SEGMENTS and dr.PATCH_PX == 64
    assert dr.DigitReader(dr.TemplateDigitClassifier()).layout is dr.LAYOUT_2027 and dr.TemplateDigitClassifier().layout is dr.LAYOUT_2027  # this year's marker is the default
    assert not dr.LAYOUT_2027.side_cue and dr.LAYOUT_2026.side_cue and set(dr.LAYOUTS) == {"x_2027", "plate_2026"}
    for layout, drawn, half in (
        (dr.LAYOUT_2027, synth_targets.SurveyMarker(digit=8).rects()[-7:], 0.3),  # the seven segments of an 8, in metres
        (dr.LAYOUT_2026, synth_targets.NumberedMarker(digit=8).rects()[-8:-1], 0.305),
    ):
        for rect, (cx, cy, width, height) in zip(drawn, layout.segment_rects()):
            assert (rect.cx / half, rect.cy / half, rect.w / half, rect.h / half) == pytest.approx((cx, cy, width, height), abs=1.0e-9)


@pytest.mark.parametrize("digit", [0, 1, 2, 3, 4, 5, 7, 8])
def test_every_digit_but_six_and_nine_is_read_from_this_years_marker_seen_from_above(digit):
    crop, truth = marker(digit)
    reading = reader().read(crop)
    assert reading.digit == digit and reading.confidence > 0.95 and not reading.abstained and reading.reason == ""
    assert len(reading.log_probabilities) == 10 and math.exp(reading.log_probabilities[digit]) == pytest.approx(reading.confidence)
    assert reading.side_px == pytest.approx(0.6 / 0.0045, rel=0.05) and reading.sharpness > 0.0
    # The corners found are the plate's, to a pixel or two.
    found, true = np.array(reading.corners), np.array(truth["corners_px"])
    assert max(min(np.linalg.norm(true - corner, axis=1)) for corner in found) < 2.5


@pytest.mark.parametrize("rotation", [0.0, 37.0, 90.0, 123.0, 180.0, 200.0, 270.0, 333.0])
def test_this_years_marker_is_read_whichever_way_it_lies_and_a_six_or_nine_is_not_guessed(rotation):
    """The X's arms give the quarter turn and nothing says which of the two ways up: a digit that reads one way
    only, or the same both ways, is answered; a 6 or a 9 is left to the vote, its probability split between the two."""
    read = reader()
    for digit in (0, 1, 2, 3, 4, 5, 7, 8):
        crop, _truth = marker(digit, rotation_deg=rotation)
        assert read.read(crop).digit == digit, (digit, rotation)
    for digit in (6, 9):
        crop, _truth = marker(digit, rotation_deg=rotation)
        reading = read.read(crop)
        assert reading.digit is None and "a 6 or a 9" in reading.reason and reading.confidence == 0.5
        assert math.exp(reading.log_probabilities[6]) == pytest.approx(0.5, abs=1.0e-4) and math.exp(reading.log_probabilities[9]) == pytest.approx(0.5, abs=1.0e-4)


@pytest.mark.parametrize("rotation", [0.0, 37.0, 90.0, 123.0, 180.0, 200.0, 270.0, 333.0])
def test_a_marker_with_its_own_cue_settles_six_against_nine(rotation):
    """Last year's plate has its digit in one white triangle: that says which way up, and a 6 is a 6."""
    for digit in (6, 9, 2, 5):
        crop, _truth = marker(digit, rotation_deg=rotation, plate=True, mpp=0.0045 * 0.61 / 0.6)
        reading = plate_reader().read(crop)
        assert reading.digit == digit, (digit, rotation, reading)


def test_all_four_quarter_turns_go_to_the_classifier_and_the_two_the_x_allows_decide():
    crop, _truth = marker(8)
    # One way up is sure, the other is not: that way is the reading. (Turns are asked in order 0, 1, 2, 3.)
    one_way = Scripted((3, 0.2), (1, 0.2), (3, 0.97), (1, 0.2))
    reading = dr.DigitReader(one_way).read(crop)
    assert one_way.asked == 4 and reading.digit == 3 and reading.quarter_turns in (2, 3)
    # Both ways up are sure of the same digit: that digit.
    assert dr.DigitReader(Scripted((2, 0.95), (1, 0.2), (2, 0.99), (1, 0.2))).read(crop).digit == 2
    # Sure of a 6 one way and of a 9 the other: not answered, and the vote is told "6 or 9".
    either = dr.DigitReader(Scripted((6, 0.99), (1, 0.2), (9, 0.99), (1, 0.2))).read(crop)
    assert either.digit is None and "a 6 or a 9" in either.reason and either.log_probabilities == dr.six_or_nine()
    assert sum(math.exp(v) for v in either.log_probabilities) == pytest.approx(1.0)
    # Sure of a 6 one way and of nothing the other: still a 6 or a 9. The other way up it would be a 9.
    assert "a 6 or a 9" in dr.DigitReader(Scripted((6, 0.99), (1, 0.2), (9, 0.5), (1, 0.2))).read(crop).reason
    # Sure of two other digits: not answered.
    assert "one way up" in dr.DigitReader(Scripted((3, 0.99), (1, 0.2), (4, 0.99), (1, 0.2))).read(crop).reason
    # Only a quarter-turned reading is sure: the arms and the digit disagree, and nothing is answered.
    sideways = dr.DigitReader(Scripted((3, 0.3), (7, 0.99), (3, 0.3), (1, 0.2))).read(crop)
    assert sideways.digit is None and "disagree" in sideways.reason
    # Neither way is sure: no answer, and the better way's probabilities are kept.
    unsure = dr.DigitReader(Scripted((3, 0.6), (1, 0.2), (3, 0.4), (1, 0.2))).read(crop)
    assert unsure.digit is None and unsure.confidence == pytest.approx(0.6) and "under 0.9" in unsure.reason
    # A 6 that is not sure, read better one way up than the other: no answer, and the reading does not lean to
    # the 6 over the 9. (The gate's flights: such leanings added up over a track's sightings to a wrong 9.)
    leaning = dr.DigitReader(Scripted((6, 0.7), (1, 0.2), (9, 0.3), (1, 0.2))).read(crop)
    assert leaning.digit is None and leaning.log_probabilities[6] == pytest.approx(leaning.log_probabilities[9])
    assert math.exp(leaning.log_probabilities[6]) == pytest.approx((0.7 + 0.3 / 9.0) / 2.0) and sum(math.exp(v) for v in leaning.log_probabilities) == pytest.approx(1.0)
    # With the marker's own cue the cued turn is the reading, whatever another turn says.
    plate, _truth = marker(8, plate=True)
    cued = Scripted((5, 0.95), (7, 0.99), (2, 0.99), (7, 0.99))
    assert dr.DigitReader(cued, layout=dr.LAYOUT_2026).read(plate).digit == 5 and cued.asked == 4
    told = dr.DigitReader(Scripted((6, 0.7), (1, 0.2), (9, 0.3), (1, 0.2)), layout=dr.LAYOUT_2026).read(plate)
    assert told.log_probabilities[6] > told.log_probabilities[9] + 1.0  # a marker that says which way up keeps its 6 a 6


def test_the_marker_is_read_tilted_blurred_noisy_dim_and_bright():
    for index, (digit, tilt, blur, noise, brightness) in enumerate([(3, (12.0, -8.0), 0.0, 0.0, 1.0), (7, (0.0, 0.0), 1.2, 4.0, 1.0), (4, (-10.0, 10.0), 0.8, 3.0, 0.7), (0, (5.0, 5.0), 0.6, 2.0, 1.2)]):
        crop, _truth = marker(digit, rotation_deg=30.0 + 77.0 * index, tilt=tilt, blur=blur, noise=noise, brightness=brightness, seed=index)
        assert reader().read(crop).digit == digit


def test_the_reader_abstains_and_says_why():
    read = reader()
    # No marker at all.
    field = SimpleNamespace(image=np.full((200, 200, 3), (85, 120, 95), dtype=np.uint8), origin=(0, 0))
    nothing = read.read(field)
    assert nothing.abstained and nothing.digit is None and "no marker outline" in nothing.reason and len(nothing.log_probabilities) == 10
    assert math.exp(nothing.log_probabilities[0]) == pytest.approx(0.1)  # nothing learnt: all ten equally likely
    # A marker too small to have a digit.
    small, _ = marker(5, mpp=0.03, size=60)  # 20 px across
    assert "too small" in read.read(small).reason
    # A white square that is not a marker.
    blank = field.image.copy()
    blank[50:150, 50:150] = 240
    assert "black" in read.read(SimpleNamespace(image=blank, origin=(0, 0))).reason
    # A marker so blurred that no digit is sure: no answer, and the probabilities it had are kept for the vote.
    smeared, _ = marker(8, blur=9.0)
    unsure = read.read(smeared)
    assert unsure.abstained and unsure.confidence < 0.9
    assert read.abstained_count == 4 and read.read_count == 0
    with pytest.raises(ValueError, match="min_confidence"):
        reader(min_confidence=0.2)


def test_a_plate_with_no_digit_is_not_answered():
    """The spec audit: with no segment dark, "1" used to be the nearest pattern. A boundary marker has the X and no digit."""
    for rotation in (0.0, 25.0, 90.0):
        bare, _ = marker(None, rotation_deg=rotation, size=400, marker_m=1.2)
        reading = reader().read(bare)
        assert reading.digit is None and reading.confidence == pytest.approx(0.1), (rotation, reading)
    classifier = dr.TemplateDigitClassifier()
    stood = dr.upright(dr.warp_to_patch(bare.image, dr.find_corners(bare.image)))[0]
    assert np.exp(classifier.log_probabilities(stood)) == pytest.approx(np.full(10, 0.1))
    # Last year's plate with its digit scrubbed out (the white triangle left empty): the same.
    plate, _ = marker(8, plate=True)
    stood = dr.upright(dr.warp_to_patch(plate.image, dr.find_corners(plate.image)), dr.LAYOUT_2026)[0].copy()
    stood[:, 40:][stood[:, 40:] < 128] = 240
    assert np.exp(dr.TemplateDigitClassifier(dr.LAYOUT_2026).log_probabilities(stood)) == pytest.approx(np.full(10, 0.1))
    # And a digit upside down is not some other digit: a 7 turned over has no pattern.
    seven, _ = marker(7)
    stood = dr.upright(dr.warp_to_patch(seven.image, dr.find_corners(seven.image)))[0]
    confidences = sorted(float(np.exp(classifier.log_probabilities(np.rot90(stood, k)).max())) for k in (0, 2))
    assert confidences[0] == pytest.approx(0.1) and confidences[1] > 0.95


def _score(read, count, seed, **drawn):
    rng = np.random.default_rng(seed)
    tally = {"right": 0, "wrong": 0, "six_nine": 0, "six_nine_left": 0, "others": 0, "others_answered": 0}
    for index in range(count):
        digit = int(rng.integers(10))
        crop, _truth = marker(
            digit,
            rotation_deg=float(rng.uniform(0.0, 360.0)),
            tilt=(float(rng.uniform(-12.0, 12.0)), float(rng.uniform(-12.0, 12.0))),
            mpp=float(rng.uniform(0.0042, 0.0052)),
            blur=float(rng.uniform(0.0, 1.0)),
            noise=float(rng.uniform(0.0, 4.0)),
            brightness=float(rng.uniform(0.75, 1.2)),
            seed=index,
            **drawn,
        )
        reading = read.read(crop)
        either = digit in dr.SIX_NINE
        tally["six_nine" if either else "others"] += 1
        if reading.digit is None:
            tally["six_nine_left"] += either and "a 6 or a 9" in reading.reason
            continue
        tally["right" if reading.digit == digit else "wrong"] += 1
        tally["others_answered"] += not either
    return tally


def test_precision_and_coverage_on_a_few_hundred_synthetic_markers():
    """The spec's line is 99% precision at 90% coverage on a field day; this is the stand-in on its own synthetic styles.

    This year's marker model has no cue for 6 against 9, so those two are not answered from one crop: the
    lines are met on the other eight digits, and no 6 or 9 is guessed. The marker with a cue meets them on all ten.
    """
    count = 150
    tally = _score(reader(), count, seed=11)
    answered = tally["right"] + tally["wrong"]
    assert tally["others_answered"] >= 0.9 * tally["others"] and tally["right"] >= 0.99 * answered, tally
    assert tally["six_nine"] > 20 and tally["six_nine_left"] >= 0.9 * tally["six_nine"] and answered == tally["others_answered"], tally
    tally = _score(plate_reader(), count, seed=11, plate=True)
    answered = tally["right"] + tally["wrong"]
    assert answered >= 0.9 * count and tally["right"] >= 0.99 * answered, tally


def test_with_a_camera_model_the_corners_go_through_the_lens_first():
    from drone_mission_core.projection_v2 import CameraModel

    crop, _truth = marker(4, rotation_deg=20.0)
    # An ideal pinhole changes nothing.
    pinhole = CameraModel(width=3840, height=2160, fx=2334.0, fy=2334.0, cx=1920.0, cy=1080.0, name="pinhole")
    placed = SimpleNamespace(image=crop.image, origin=(3300, 1700))  # out toward a corner of the 4K frame
    assert dr.DigitReader(dr.TemplateDigitClassifier(), camera=pinhole).read(placed).digit == 4
    corners = dr.find_corners(crop.image)
    plain = dr.warp_to_patch(crop.image, corners)
    through = dr.warp_to_patch(crop.image, corners, (3300, 1700), pinhole)
    assert np.abs(plain.astype(float) - through.astype(float)).mean() < 1.5  # the same patch, to the resampling


def test_a_marker_bent_by_the_lens_is_straightened_by_the_camera_model():
    """The spec audit: a test that fails if the lens terms are ignored. A big marker near the frame's corner is drawn
    as a lens with strong barrel distortion would show it; with the camera model its patch is the flat marker's."""
    from drone_mission_core.projection_v2 import CameraModel

    k = np.array([[2334.0, 0.0, 1920.0], [0.0, 2334.0, 1080.0], [0.0, 0.0, 1.0]])
    d = (-0.25, 0.08, 0.0, 0.0, 0.0)
    bent = CameraModel(width=3840, height=2160, fx=2334.0, fy=2334.0, cx=1920.0, cy=1080.0, d=d, name="bent")
    flat, _truth = marker(4, rotation_deg=20.0, mpp=0.0015, size=720)  # a 400 px marker, as an ideal lens shows it
    # The lens's picture of it: each of its pixels looks where the ideal picture has that ray.
    origin, side = (3100, 1540), 500
    us, vs = np.meshgrid(np.arange(side, dtype=np.float64) + origin[0], np.arange(side, dtype=np.float64) + origin[1])
    rays = cv2.undistortPoints(np.stack([us.ravel(), vs.ravel()], axis=1).reshape(-1, 1, 2), k, np.array(d)).reshape(-1, 2)
    ideal = rays * 2334.0 + np.array([1920.0, 1080.0])
    ideal -= ideal.mean(axis=0) - 360.0  # the ideal picture's own crop, centred on the same rays
    seen = cv2.remap(flat.image, ideal[:, 0].reshape(side, side).astype(np.float32), ideal[:, 1].reshape(side, side).astype(np.float32), cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
    truth_patch = dr.upright(dr.warp_to_patch(flat.image, dr.find_corners(flat.image)))[0]
    corners = dr.find_corners(seen)
    assert corners is not None
    errors = {}
    for name, camera in (("with", bent), ("without", None)):
        patch = dr.upright(dr.warp_to_patch(seen, corners, origin, camera))[0]
        errors[name] = min(float(np.abs(np.rot90(patch, turn) - truth_patch).mean()) for turn in (0, 2))
    assert errors["with"] < 0.5 * errors["without"] and errors["with"] < 4.0, errors
    assert dr.DigitReader(dr.TemplateDigitClassifier(), camera=bent).read(SimpleNamespace(image=seen, origin=origin)).digit == 4


class Session:
    """An ONNX Runtime session that answers "7" to anything."""

    def __init__(self, scores=None):
        self.scores, self.fed = scores, []

    def get_inputs(self):
        return [SimpleNamespace(name="image")]

    def run(self, outputs, feeds):
        self.fed.append(feeds["image"])
        scores = np.full((1, 10), -4.0, dtype=np.float32) if self.scores is None else self.scores
        if self.scores is None:
            scores[0, 7] = 6.0
        return [scores]


def test_the_trained_classifier_is_an_onnx_session_fed_one_grey_patch_and_refused_where_it_cannot_run(tmp_path, monkeypatch):
    session = Session()
    classifier = dr.OnnxDigitClassifier(session)
    scores = classifier.log_probabilities(np.full((64, 64), 255, dtype=np.uint8))
    assert int(np.argmax(scores)) == 7 and np.exp(scores).sum() == pytest.approx(1.0)
    assert session.fed[0].shape == (1, 1, 64, 64) and session.fed[0].dtype == np.float32 and session.fed[0].max() == pytest.approx(1.0)
    crop, _ = marker(2)
    assert dr.DigitReader(classifier).read(crop).digit == 7  # the pipeline is the same whatever the classifier
    assert len(session.fed) == 1 + 4  # the patch in its four quarter turns
    with pytest.raises(dr.DigitClassifierUnavailable, match="not ten"):
        dr.OnnxDigitClassifier(Session(np.zeros((1, 3), dtype=np.float32))).log_probabilities(np.zeros((64, 64), dtype=np.uint8))
    with pytest.raises(dr.DigitClassifierUnavailable, match="needs digit_model_path"):
        dr.load_onnx("")
    with pytest.raises(dr.DigitClassifierUnavailable, match="is not a file"):
        dr.load_onnx(tmp_path / "missing.onnx")
    model = tmp_path / "digits.onnx"
    model.write_bytes(b"stand-in")
    monkeypatch.setitem(sys.modules, "onnxruntime", None)
    with pytest.raises(dr.DigitClassifierUnavailable, match="ONNX Runtime, which is not installed"):
        dr.load_onnx(model)
    made = []
    monkeypatch.setitem(sys.modules, "onnxruntime", SimpleNamespace(InferenceSession=lambda path, providers: made.append((path, providers)) or Session()))
    loaded = dr.load_onnx(model)
    assert made == [(str(model), ["CPUExecutionProvider"])] and loaded.name == "onnx"  # on the CPU: the GPU is the detector's


# --- the flight-safety review's points --------------------------------------------------------


def test_the_warp_through_the_lens_asks_the_camera_model_for_a_coarse_grid_not_for_every_pixel():
    from drone_mission_core.projection_v2 import CameraModel

    class Counting(CameraModel):
        calls = 0

        def pixel_of(self, ray):
            type(self).calls += 1
            return super().pixel_of(ray)

    bent = Counting(width=3840, height=2160, fx=2334.0, fy=2334.0, cx=1920.0, cy=1080.0, d=(-0.25, 0.08, 0.0, 0.0, 0.0), name="bent")
    exact = CameraModel(width=3840, height=2160, fx=2334.0, fy=2334.0, cx=1920.0, cy=1080.0, d=(-0.25, 0.08, 0.0, 0.0, 0.0), name="bent")
    crop, _truth = marker(4, rotation_deg=20.0)
    corners = dr.find_corners(crop.image)
    patch = dr.warp_to_patch(crop.image, corners, (3300, 1700), bent)
    assert Counting.calls <= 100  # a 9 x 9 grid: over a marker the lens bends by a fraction of a pixel between its points
    # And the patch is the one every pixel through the lens gives, to the sampling.
    import cv2

    square = np.array([[0.0, 0.0], [64.0, 0.0], [64.0, 64.0], [0.0, 64.0]], dtype=np.float32)
    undistorted = np.array([exact.undistort((u + 3300, v + 1700))[0] for u, v in corners], dtype=np.float32)
    centres = np.arange(64, dtype=np.float32) + 0.5
    grid = np.stack(np.meshgrid(centres, centres), axis=-1).reshape(-1, 1, 2)
    rays = cv2.perspectiveTransform(grid, cv2.getPerspectiveTransform(square, undistorted)).reshape(-1, 2)
    pixels = (np.array([exact.pixel_of((float(x), float(y), 1.0)) for x, y in rays], dtype=np.float32) - np.array([3300, 1700], dtype=np.float32)).reshape(64, 64, 2)
    shrink = float(np.mean([np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)])) / 64.0
    smoothed = cv2.GaussianBlur(cv2.cvtColor(crop.image, cv2.COLOR_BGR2GRAY), (0, 0), 0.4 * shrink)  # as the warp smooths a marker bigger than its patch
    full = cv2.remap(smoothed, pixels[:, :, 0].copy(), pixels[:, :, 1].copy(), cv2.INTER_LINEAR)
    assert np.abs(patch.astype(float) - full.astype(float)).mean() < 1.0


def test_only_numbered_markers_are_read_and_a_frame_has_a_limit_of_crops_and_of_time():
    read = reader()
    crop, _ = marker(3)
    detection = lambda class_id, score=0.9: SimpleNamespace(class_id=class_id, score=score)  # noqa: E731
    crops = [SimpleNamespace(image=crop.image, origin=(0, 0), detection=detection("boundary_marker")), SimpleNamespace(image=crop.image, origin=(0, 0), detection=detection("marker"))]
    readings = read.read_all(crops)
    assert [r.digit for r in readings] == [None, 3] and "boundary marker" in readings[0].reason and read.abstained_count == 0  # not asked is not abstained
    # More markers in a frame than the limit: the most confident boxes are read, in the crops' own order.
    assert dr.MAX_CROPS_PER_FRAME == 6
    many = [SimpleNamespace(image=crop.image, origin=(0, 0), detection=detection("marker", 0.5 + 0.05 * index)) for index in range(8)]
    readings = read.read_all(many)
    assert [r.digit for r in readings] == [None, None, 3, 3, 3, 3, 3, 3] and "more markers in the frame" in readings[0].reason
    # A box far bigger than a marker is from the survey's height is not worked on at all.
    huge = SimpleNamespace(image=np.zeros((dr.MAX_CROP_SIDE_PX + 1, 300, 3), dtype=np.uint8), origin=(0, 0), detection=detection("marker"))
    assert "too big to be a marker" in read.read(huge).reason and dr.MAX_CROP_SIDE_PX == 512
    # No time left: the rest of the frame's crops wait for another frame.
    hurried = dr.DigitReader(dr.TemplateDigitClassifier(), budget_s=0.0)
    readings = hurried.read_all(many[:3])
    assert [r.digit for r in readings] == [None, None, 3] and hurried.skipped_count == 2  # the most confident box is always read
    assert all("no time left" in r.reason for r in readings[:2])


def test_one_crop_whose_reading_raises_costs_only_its_own_reading():
    """The spec audit: the other crops of the frame are still read, and the fault is counted."""

    class Faulty(dr.TemplateDigitClassifier):
        patches = 0

        def log_probabilities(self, patch):
            type(self).patches += 1
            if self.patches == 5:  # the second crop's first quarter turn
                raise RuntimeError("the classifier fell over")
            return super().log_probabilities(patch)

    crop, _ = marker(3)
    detection = lambda score: SimpleNamespace(class_id="marker", score=score)  # noqa: E731
    crops = [SimpleNamespace(image=crop.image, origin=(0, 0), detection=detection(score)) for score in (0.9, 0.8, 0.7)]
    read = dr.DigitReader(Faulty(), budget_s=5.0)
    readings = read.read_all(crops)
    assert [r.digit for r in readings] == [3, None, 3] and "the reading failed (RuntimeError: the classifier fell over)" in readings[1].reason
    assert (read.faults, read.read_count) == (1, 2)
