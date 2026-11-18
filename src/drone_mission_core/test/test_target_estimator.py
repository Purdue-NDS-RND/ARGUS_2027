"""The target estimator (FND-06): one Kalman filter per static target, its gate, its track life and its forecast."""

import math

import numpy as np
import pytest

from drone_mission_core import target_estimator as te
from drone_mission_core.projection_v2 import CameraModel, CameraMount, project
from drone_mission_core.target_estimator import CONFIRMED, COASTING, LOST, TENTATIVE, Measurement, TargetEstimator

MS = 1_000_000
S = 1_000_000_000
T0 = 100 * S
CAMERA = CameraModel(width=1280, height=720, fx=1000.0, fy=1000.0, cx=640.0, cy=360.0, name="test")
MOUNT = CameraMount(position_m=(0.0, 0.0, 0.0), rotation_body_from_optical=((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0)))  # looking straight down
LEVEL = (0.0, 0.0, 0.0, 1.0)


def at(east, north, sigma=0.1, class_id="bullseye", score=0.9):
    return Measurement(class_id=class_id, east_m=east, north_m=north, covariance=np.eye(2) * sigma * sigma, score=score)


def confirmed_track(estimator, east=5.0, north=7.0, sigma=0.1, class_id="bullseye", start=T0):
    for index in range(3):
        estimator.update([at(east, north, sigma, class_id)], start + index * 100 * MS)
    confirmed = [t for t in estimator.estimates(start + 200 * MS, class_id) if t.state == CONFIRMED]
    return min(confirmed, key=lambda t: math.hypot(t.east_m - east, t.north_m - north))


# --- measurement noise: R = J Sigma J^T ------------------------------------------------


def ground_point(height=15.0, pixel=(700.0, 400.0)):
    return project(CAMERA, MOUNT, pixel, (0.0, 0.0, height), LEVEL)


def test_measurement_noise_is_the_jacobians_times_pixel_attitude_and_height_noise():
    point = ground_point()
    noise = te.measurement_noise(point, box_width=200.0, box_height=200.0, height_source="rangefinder")
    expected = (
        point.d_pixel @ (np.eye(2) * 2.0**2) @ point.d_pixel.T
        + point.d_attitude @ (np.eye(3) * math.radians(1.0) ** 2) @ point.d_attitude.T
        + np.outer(point.d_height, point.d_height) * 0.05**2
    )
    assert noise == pytest.approx(expected, rel=1e-12)
    # At 15 m a degree of attitude is about 0.26 m on the ground: it dominates.
    assert math.sqrt(noise[0, 0]) == pytest.approx(15.0 * math.radians(1.0), rel=0.05)


def test_pixel_noise_grows_for_small_boxes_and_height_noise_on_baro():
    assert te.pixel_sigma_px(200.0, 150.0) == 2.0
    assert te.pixel_sigma_px(32.0, 64.0) == 2.0
    assert te.pixel_sigma_px(16.0, 64.0) == pytest.approx(4.0)
    assert te.pixel_sigma_px(2.0, 2.0) == pytest.approx(8.0)  # and no further
    point = ground_point(pixel=(1100.0, 600.0))  # off the axis, where height matters
    on_rangefinder = te.measurement_noise(point, 100.0, 100.0, "rangefinder")
    on_baro = te.measurement_noise(point, 100.0, 100.0, "home")
    extra = on_baro - on_rangefinder
    assert extra == pytest.approx(np.outer(point.d_height, point.d_height) * (0.5**2 - 0.05**2), rel=1e-9)


def test_a_box_that_reaches_the_edge_of_the_image_is_cut():
    size = (1280, 720)
    assert not te.box_is_cut(640.0, 360.0, 200.0, 200.0, *size)
    assert not te.box_is_cut(103.0, 360.0, 200.0, 200.0, *size)  # 3 px clear of the left edge
    assert te.box_is_cut(101.0, 360.0, 200.0, 200.0, *size)  # within 2 px of it
    assert te.box_is_cut(640.0, 60.0, 200.0, 124.0, *size) and te.box_is_cut(1200.0, 360.0, 200.0, 50.0, *size)
    assert te.box_is_cut(640.0, 650.0, 100.0, 141.0, *size)
    assert te.box_is_cut(640.0, 360.0, 1280.0, 720.0, *size)  # a target that fills the frame


def test_each_class_projects_onto_its_own_height():
    assert te.plane_height_m("bullseye") == 0.0 and te.plane_height_m("green_x") == 0.0
    assert te.plane_height_m("bucket") == pytest.approx(0.2)
    assert te.PLANE_HEIGHT_M["handle"] == pytest.approx(0.3)
    with pytest.raises(ValueError, match="no plane height"):
        te.plane_height_m("cloud")


# --- the filter and its gate -----------------------------------------------------------


def test_the_constants_are_the_specs():
    assert te.GATE_NIS == 9.21 and te.PROCESS_NOISE_M2_PER_S == 0.0025
    assert te.COVARIANCE_FLOOR_M2 == pytest.approx(0.02**2) and te.RESET_AFTER_REJECTIONS == 5
    assert (te.CONFIRM_HITS, te.CONFIRM_WINDOW_S, te.COAST_AFTER_S, te.LOST_AFTER_S, te.KEEP_LOST_S) == (3, 1.0, 0.3, 2.0, 30.0)
    assert te.MAX_TRACKS_PER_CLASS == 8 and te.ACTUATION_DELAY_S == 0.08


def test_a_measurement_inside_nis_9_21_is_taken_and_one_outside_is_not():
    estimator = TargetEstimator()
    track = confirmed_track(estimator, east=0.0, north=0.0, sigma=0.1)
    spread = math.sqrt(track.covariance[0][0] + 0.1**2)  # innovation sigma along east
    inside = estimator.update([at(2.9 * spread, 0.0)], T0 + 300 * MS)  # NIS 8.4: a gate of 3 would have thrown it away
    assert (inside.accepted, inside.rejected) == (1, 0)
    estimator = TargetEstimator()
    track = confirmed_track(estimator, east=0.0, north=0.0, sigma=0.1)
    outside = estimator.update([at(3.2 * spread, 0.0)], T0 + 300 * MS)  # NIS 10.2
    assert (outside.accepted, outside.rejected) == (0, 1)
    assert estimator.estimates(T0 + 300 * MS)[0].east_m == pytest.approx(0.0, abs=1e-9)


def test_the_estimate_settles_on_the_mean_and_the_covariance_stops_at_the_floor():
    estimator = TargetEstimator()
    rng = np.random.default_rng(1)
    for index in range(600):  # 20 s at 30 Hz of 5 cm noise
        east, north = 3.0 + rng.normal(0.0, 0.05), -2.0 + rng.normal(0.0, 0.05)
        estimator.update([at(east, north, 0.05)], T0 + index * 33 * MS)
    (track,) = estimator.estimates(T0 + 600 * 33 * MS, for_ns=T0 + 599 * 33 * MS)
    assert (track.east_m, track.north_m) == pytest.approx((3.0, -2.0), abs=0.06)
    eigenvalues = np.linalg.eigvalsh(np.array(track.covariance))
    assert eigenvalues.min() >= 0.02**2 - 1e-12
    assert eigenvalues.max() < 0.03**2  # and it did converge that far


def test_the_floor_holds_even_for_a_perfect_measurement():
    estimator = TargetEstimator()
    for index in range(5):
        estimator.update([at(1.0, 1.0, sigma=1e-4)], T0 + index * 33 * MS)
    (track,) = estimator.estimates(T0 + 4 * 33 * MS, for_ns=T0 + 4 * 33 * MS)
    assert np.linalg.eigvalsh(np.array(track.covariance)).min() >= 0.02**2 - 1e-12


def test_process_noise_grows_the_covariance_by_q_per_second_and_the_forecast_adds_the_actuation_delay():
    estimator = TargetEstimator()
    track = confirmed_track(estimator)
    last = T0 + 200 * MS
    same = estimator.estimates(last, for_ns=last)[0]
    later = estimator.estimates(last, for_ns=last + 10 * S)[0]
    assert np.array(later.covariance) - np.array(same.covariance) == pytest.approx(np.eye(2) * 0.025)
    # By default the output is for the actuation time, now + 0.08 s: no motion for a static target, only the covariance.
    forecast = estimator.estimates(last)[0]
    assert forecast.for_ns == last + 80 * MS
    assert (forecast.east_m, forecast.north_m) == (same.east_m, same.north_m) == (track.east_m, track.north_m)
    assert np.array(forecast.covariance) - np.array(same.covariance) == pytest.approx(np.eye(2) * 0.0025 * 0.08)


def test_five_gated_out_measurements_in_a_row_move_the_track_and_four_do_not():
    estimator = TargetEstimator()
    track = confirmed_track(estimator, east=0.0, north=0.0, sigma=0.05)
    for index in range(4):
        report = estimator.update([at(1.0, 0.0, 0.05)], T0 + (3 + index) * 100 * MS)
        assert (report.rejected, report.resets, report.started) == (1, 0, 0)
    assert estimator.estimates(T0 + 700 * MS)[0].east_m == pytest.approx(0.0, abs=1e-6)
    report = estimator.update([at(1.0, 0.0, 0.05)], T0 + 700 * MS)
    assert report.resets == 1
    (moved,) = estimator.estimates(T0 + 700 * MS)
    assert moved.track_id == track.track_id and moved.east_m == pytest.approx(1.0) and moved.state == CONFIRMED
    assert estimator.update([at(1.0, 0.0, 0.05)], T0 + 800 * MS).accepted == 1


def test_an_accepted_measurement_starts_the_count_of_rejections_again():
    estimator = TargetEstimator()
    confirmed_track(estimator, east=0.0, north=0.0, sigma=0.05)
    stamp = T0 + 300 * MS
    for _round in range(3):  # 4 out, 1 in, three times over: twelve rejections but never five in a row
        for _ in range(4):
            assert estimator.update([at(1.0, 0.0, 0.05)], stamp).resets == 0
            stamp += 100 * MS
        assert estimator.update([at(0.0, 0.0, 0.05)], stamp).accepted == 1
        stamp += 100 * MS
    assert estimator.estimates(stamp)[0].east_m == pytest.approx(0.0, abs=0.01)


def test_a_detection_far_from_every_track_is_a_new_target_not_a_rejection():
    estimator = TargetEstimator()
    first = confirmed_track(estimator, east=0.0, north=0.0, sigma=0.05)
    for index in range(6):  # something red ten metres away, seen while the target itself is not
        report = estimator.update([at(10.0, 0.0, 0.05)], T0 + (3 + index) * 100 * MS)
        assert report.rejected == 0 and report.resets == 0
    tracks = {t.track_id: t for t in estimator.estimates(T0 + 800 * MS)}
    assert len(tracks) == 2
    assert tracks[first.track_id].east_m == pytest.approx(0.0, abs=1e-6)  # the target has not jumped to the decoy


# --- track life ------------------------------------------------------------------------


def test_tentative_until_three_hits_within_a_second():
    estimator = TargetEstimator()
    estimator.update([at(1.0, 1.0)], T0)
    estimator.update([at(1.0, 1.0)], T0 + 100 * MS)
    assert [t.state for t in estimator.estimates(T0 + 100 * MS)] == [TENTATIVE]
    estimator.update([at(1.0, 1.0)], T0 + 200 * MS)
    (track,) = estimator.estimates(T0 + 200 * MS)
    assert track.state == CONFIRMED and track.hits == 3


def test_three_hits_spread_over_more_than_a_second_do_not_confirm():
    estimator = TargetEstimator()
    for stamp in (T0, T0 + 600 * MS, T0 + 1200 * MS):
        estimator.update([at(1.0, 1.0)], stamp)
    assert [t.state for t in estimator.estimates(T0 + 1200 * MS)] == [TENTATIVE]
    estimator.update([at(1.0, 1.0)], T0 + 1500 * MS)  # now the last three are within a second
    assert [t.state for t in estimator.estimates(T0 + 1500 * MS)] == [CONFIRMED]


def test_a_tentative_track_that_never_confirms_is_dropped_after_a_second():
    estimator = TargetEstimator()
    estimator.update([at(1.0, 1.0)], T0)
    assert len(estimator.estimates(T0 + 900 * MS)) == 1
    assert estimator.estimates(T0 + 1100 * MS) == []


def test_confirmed_coasts_after_0_3_s_is_lost_after_2_s_and_is_kept_30_s():
    estimator = TargetEstimator()
    track = confirmed_track(estimator)
    last_hit = T0 + 200 * MS

    def state(after_s):
        found = estimator.estimates(last_hit + int(after_s * S))
        return found[0].state if found else None

    assert state(0.29) == CONFIRMED
    assert state(0.31) == COASTING
    assert state(1.99) == COASTING
    assert state(2.01) == LOST
    assert state(31.9) == LOST
    kept = estimator.estimates(last_hit + 20 * S)[0]
    assert kept.track_id == track.track_id and kept.age_s == pytest.approx(20.0)
    assert (kept.east_m, kept.north_m) == (track.east_m, track.north_m)
    assert state(32.1) is None  # 30 s after it was lost


def test_a_hit_while_coasting_confirms_again_at_once():
    estimator = TargetEstimator()
    track = confirmed_track(estimator)
    assert estimator.estimates(T0 + 1200 * MS)[0].state == COASTING
    estimator.update([at(5.0, 7.0)], T0 + 1200 * MS)
    (again,) = estimator.estimates(T0 + 1200 * MS)
    assert again.state == CONFIRMED and again.track_id == track.track_id


def test_a_lost_track_is_found_again_under_its_own_id_and_must_confirm_again():
    estimator = TargetEstimator()
    track = confirmed_track(estimator)
    back = T0 + 10 * S
    assert estimator.estimates(back)[0].state == LOST
    estimator.update([at(5.1, 7.0)], back)
    (found,) = estimator.estimates(back)
    assert found.track_id == track.track_id and found.state == TENTATIVE
    estimator.update([at(5.1, 7.0)], back + 100 * MS)
    estimator.update([at(5.1, 7.0)], back + 200 * MS)
    (found,) = estimator.estimates(back + 200 * MS)
    assert found.track_id == track.track_id and found.state == CONFIRMED
    # Seen once more and not again: it goes back to LOST, it is not thrown away like a new tentative track.
    estimator = TargetEstimator()
    track = confirmed_track(estimator)
    estimator.update([at(5.1, 7.0)], back)
    (lost_again,) = estimator.estimates(back + 1500 * MS)
    assert lost_again.state == LOST and lost_again.track_id == track.track_id


def test_at_most_eight_tracks_per_class():
    estimator = TargetEstimator()
    for round_index in range(3):
        measurements = [at(10.0 * index, 0.0) for index in range(10)]
        report = estimator.update(measurements, T0 + round_index * 100 * MS)
    tracks = estimator.estimates(T0 + 200 * MS)
    assert len(tracks) == 8 and all(t.state == CONFIRMED for t in tracks)
    assert report.untracked == 2
    # Another class has its own eight.
    estimator.update([at(500.0, 0.0, class_id="green_x")], T0 + 300 * MS)
    assert len(estimator.estimates(T0 + 300 * MS, "green_x")) == 1 and len(estimator.estimates(T0 + 300 * MS, "bullseye")) == 8


def test_a_new_target_takes_the_place_of_the_longest_lost_track_when_the_class_is_full():
    estimator = TargetEstimator()
    for index in range(8):  # eight confirmed, one after another, each lost in turn
        confirmed_track(estimator, east=10.0 * index, north=0.0, start=T0 + index * S)
    now = T0 + 20 * S
    assert [t.state for t in estimator.estimates(now)] == [LOST] * 8
    report = estimator.update([at(200.0, 0.0)], now)
    assert report.started == 1
    easts = sorted(t.east_m for t in estimator.estimates(now))
    assert len(easts) == 8 and 0.0 not in [round(e) for e in easts] and 200.0 in [round(e) for e in easts]


def test_detections_in_one_frame_go_to_the_tracks_that_fit_them_best_together():
    estimator = TargetEstimator()
    for index in range(3):  # two targets three metres apart, each seen poorly
        estimator.update([at(0.0, 0.0, 0.9), at(3.0, 0.0, 0.9)], T0 + index * 100 * MS)
    a, b = sorted(estimator.estimates(T0 + 200 * MS), key=lambda t: t.east_m)
    assert (a.state, b.state) == (CONFIRMED, CONFIRMED)
    # 1.4 is nearer A than B, and -1.0 fits only A. Taking them one at a time, nearest first, gives
    # 1.4 to A and leaves -1.0 with nothing; together, 1.4 goes to B and -1.0 to A.
    report = estimator.update([at(1.4, 0.0, 0.9), at(-1.0, 0.0, 0.9)], T0 + 300 * MS)
    assert (report.accepted, report.rejected, report.started) == (2, 0, 0)
    after = {t.track_id: t for t in estimator.estimates(T0 + 300 * MS)}
    assert after[a.track_id].east_m < 0.0 and after[b.track_id].east_m < 3.0


def test_a_second_detection_beside_a_track_that_already_has_one_does_not_start_a_twin():
    estimator = TargetEstimator()
    confirmed_track(estimator, east=0.0, north=0.0, sigma=0.05)
    report = estimator.update([at(0.0, 0.0, 0.05), at(0.8, 0.0, 0.05)], T0 + 300 * MS)
    assert (report.accepted, report.started, report.rejected, report.ignored) == (1, 0, 0, 1)
    assert len(estimator.estimates(T0 + 300 * MS)) == 1


def test_classes_are_tracked_apart():
    estimator = TargetEstimator()
    confirmed_track(estimator, east=0.0, north=0.0, class_id="bullseye")
    report = estimator.update([at(0.0, 0.0, class_id="bucket")], T0 + 300 * MS)
    assert report.started == 1 and report.accepted == 0


def test_bad_measurements_are_refused():
    estimator = TargetEstimator()
    for bad in (
        Measurement("bullseye", math.nan, 0.0, np.eye(2)),
        Measurement("bullseye", 0.0, 0.0, np.array([[1.0, 0.0], [0.0, -1.0]])),
        Measurement("bullseye", 0.0, 0.0, np.eye(3)),
    ):
        report = estimator.update([bad], T0)
        assert report.invalid == 1 and estimator.estimates(T0) == []


# --- EKF position resets ---------------------------------------------------------------


def test_a_position_reset_moves_every_track_so_the_next_detections_still_pass_the_gate():
    estimator = TargetEstimator()
    a = confirmed_track(estimator, east=0.0, north=0.0, sigma=0.05)
    b = confirmed_track(estimator, east=20.0, north=5.0, sigma=0.05, class_id="green_x")
    estimator.shift_all(1.5, -0.7)  # the flight controller's position stepped by this much
    after = {t.track_id: t for t in estimator.estimates(T0 + 300 * MS)}
    assert (after[a.track_id].east_m, after[a.track_id].north_m) == pytest.approx((1.5, -0.7), abs=1e-6)
    assert (after[b.track_id].east_m, after[b.track_id].north_m) == pytest.approx((21.5, 4.3), abs=1e-6)
    report = estimator.update([at(1.5, -0.7, 0.05), at(21.5, 4.3, 0.05, class_id="green_x")], T0 + 300 * MS)
    assert (report.accepted, report.rejected) == (2, 0)


def test_a_clock_that_stepped_back_leaves_confirmed_tracks_lost_and_drops_the_unproven():
    estimator = TargetEstimator()
    proven = confirmed_track(estimator, east=0.0, north=0.0)
    estimator.update([at(30.0, 0.0)], T0 + 250 * MS)  # one hit: tentative
    earlier = T0 - 100 * S
    estimator.clock_stepped_back(earlier)
    (track,) = estimator.estimates(earlier)
    assert track.track_id == proven.track_id and track.state == LOST and track.age_s == pytest.approx(2.0)
    for index in range(3):  # it is found again on the new clock, and has to confirm again
        estimator.update([at(0.0, 0.0)], earlier + (1 + index) * 100 * MS)
    (track,) = estimator.estimates(earlier + 300 * MS)
    assert track.track_id == proven.track_id and track.state == CONFIRMED and track.age_s == pytest.approx(0.0)


# --- the primary target ----------------------------------------------------------------


def test_the_primary_target_is_the_best_established_track_or_the_one_nearest_a_point():
    estimator = TargetEstimator()
    assert estimator.primary("bullseye", T0) is None
    far = confirmed_track(estimator, east=30.0, north=0.0)
    estimator.update([at(0.0, 0.0)], T0 + 250 * MS)  # a second, tentative, track
    for index in range(5):
        estimator.update([at(30.0, 0.0)], T0 + (3 + index) * 50 * MS)
    assert estimator.primary("bullseye", T0 + 500 * MS).track_id == far.track_id
    near = estimator.primary("bullseye", T0 + 500 * MS, near=(1.0, 0.0))
    assert near.track_id == far.track_id  # a confirmed track still comes before a tentative one
    assert estimator.primary("green_x", T0 + 500 * MS) is None


def test_of_two_confirmed_tracks_the_primary_is_the_one_nearest_the_point_asked_for():
    estimator = TargetEstimator()
    a = confirmed_track(estimator, east=0.0, north=0.0)
    b = confirmed_track(estimator, east=30.0, north=0.0)
    assert estimator.primary("bullseye", T0 + 250 * MS, near=(29.0, 1.0)).track_id == b.track_id
    assert estimator.primary("bullseye", T0 + 250 * MS, near=(2.0, 1.0)).track_id == a.track_id


# --- how well it does (the spec's acceptance numbers, on generated measurements) ---------


@pytest.mark.parametrize("height_m, limit_m", [(15.0, 0.30), (6.0, 0.15), (2.4, 0.08)])
def test_p95_error_meets_the_acceptance_line_on_noisy_projections(height_m, limit_m):
    """A hover over the target for 4 s at 30 Hz, each frame with fresh pixel, attitude and height errors of the sizes the filter is told."""
    rng = np.random.default_rng(int(height_m * 10))
    errors = []
    for _trial in range(40):
        estimator = TargetEstimator()
        target = np.array([0.4, -0.3])
        for frame in range(120):
            tilt = rng.normal(0.0, math.radians(1.0), size=2)
            attitude = (math.sin(tilt[0] / 2.0), math.sin(tilt[1] / 2.0), 0.0, 1.0)
            true_pixel = (CAMERA.cx - target[1] / height_m * CAMERA.fx, CAMERA.cy - target[0] / height_m * CAMERA.fy)
            seen = (true_pixel[0] + rng.normal(0.0, 2.0), true_pixel[1] + rng.normal(0.0, 2.0))
            point = project(CAMERA, MOUNT, seen, (0.0, 0.0, height_m + rng.normal(0.0, 0.05)), attitude)
            estimator.update([te.measurement_from_point("bullseye", point, 120.0, 120.0, 0.9, "rangefinder")], T0 + frame * 33 * MS)
            if frame >= 30:  # after the first second
                (track,) = estimator.estimates(T0 + frame * 33 * MS)
                errors.append(math.hypot(track.east_m - target[0], track.north_m - target[1]))
    p95 = sorted(errors)[int(0.95 * len(errors))]
    assert p95 <= limit_m, f"p95 {p95:.3f} m at {height_m} m"
