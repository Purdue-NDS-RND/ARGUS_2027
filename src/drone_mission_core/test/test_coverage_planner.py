"""The survey's coverage planner (LOC-2): a lawnmower over a polygon, every part of it seen by at least two frames. No ROS."""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_mission_core import coverage_planner as cp  # noqa: E402
from drone_mission_core.safety_limits import PHASE_SPEED_CAPS_MPS  # noqa: E402

# The IMX519 at 4K (argus_camera/config/calibration/imx519_4k.yaml).
CAMERA_4K = SimpleNamespace(width=3840, height=2160, fx=2334.99647, fy=2338.06625)
THIRTY_FT = 9.144
SQUARE = [(0.0, 0.0), (60.0, 0.0), (60.0, 40.0), (0.0, 40.0)]


def test_the_footprint_and_the_spacing_are_the_specs_at_30_ft():
    across, along = cp.footprint(CAMERA_4K, THIRTY_FT)
    assert across == pytest.approx(15.0, abs=0.05) and along == pytest.approx(8.45, abs=0.01)  # "15.0 x 8.45 m"
    assert cp.track_spacing(15.0) == pytest.approx(0.8 * 15.0 - 0.85)
    assert cp.MARKER_MARGIN_M == 0.85 and cp.DEFAULT_SPEED_MPS == 4.0 == PHASE_SPEED_CAPS_MPS["survey"]
    plan = cp.plan(SQUARE, THIRTY_FT, CAMERA_4K)
    assert plan.spacing_m <= cp.track_spacing(across) + 1e-9
    assert plan.frames_per_marker == pytest.approx(along / (4.0 / 15.0), rel=1e-6) and 30 <= plan.frames_per_marker <= 33  # "about 30 frames at 15 Hz"


def test_a_rectangle_is_flown_along_its_long_side_there_and_back():
    plan = cp.plan(SQUARE, THIRTY_FT, CAMERA_4K)
    assert plan.track_heading_deg % 180.0 == pytest.approx(0.0)  # along east, the 60 m side: the fewest tracks
    assert len(plan.waypoints) == 2 * plan.tracks and plan.tracks == 4  # 40 m across at 11.15 m or less apart
    norths = [north for _east, north in plan.waypoints]
    assert norths[0] == norths[1] and norths[2] == norths[3] and len({round(n, 6) for n in norths}) == plan.tracks
    easts = [east for east, _north in plan.waypoints]
    assert easts[0] < easts[1] and easts[2] > easts[3]  # there, then back
    # The outer tracks stand in from the edge so that a marker on the edge is whole inside the swath.
    across, _along = cp.footprint(CAMERA_4K, THIRTY_FT)
    assert min(norths) == pytest.approx(across / 2.0 - 0.85) and max(norths) == pytest.approx(40.0 - (across / 2.0 - 0.85))
    assert plan.length_m == pytest.approx(sum(math.dist(a, b) for a, b in zip(plan.waypoints, plan.waypoints[1:])))
    assert plan.time_s == pytest.approx(plan.length_m / 4.0)
    # The same field given the other way round, or turned 30 degrees, gets the same tracks along its long side.
    assert cp.plan(SQUARE[::-1], THIRTY_FT, CAMERA_4K).tracks == 4
    turn = math.radians(30.0)
    turned = [(e * math.cos(turn) - n * math.sin(turn), e * math.sin(turn) + n * math.cos(turn)) for e, n in SQUARE]
    rotated = cp.plan(turned, THIRTY_FT, CAMERA_4K)
    assert rotated.tracks == 4 and rotated.track_heading_deg % 180.0 == pytest.approx(30.0)


def test_a_strip_narrower_than_the_swath_is_one_pass_down_its_middle():
    plan = cp.plan([(0.0, 0.0), (50.0, 0.0), (50.0, 6.0), (0.0, 6.0)], THIRTY_FT, CAMERA_4K)
    assert plan.tracks == 1 and [north for _e, north in plan.waypoints] == pytest.approx([3.0, 3.0])


def random_polygon(rng, concave):
    """A star-shaped polygon about the origin: random radii at sorted random angles, 40 to 120 m across."""
    count = int(rng.integers(5, 10))
    angles = np.sort(rng.uniform(0.0, 2.0 * math.pi, count))
    low = 0.35 if concave else 0.8
    radii = rng.uniform(low, 1.0, count) * rng.uniform(20.0, 60.0)
    return [(float(r * math.cos(a)), float(r * math.sin(a))) for a, r in zip(angles, radii)]


@pytest.mark.parametrize("seed", range(30))
def test_random_polygons_are_seen_by_at_least_two_frames_over_99_percent_of_their_area(seed):
    rng = np.random.default_rng(seed)
    polygon = random_polygon(rng, concave=seed % 2 == 1)
    height = float(rng.choice([THIRTY_FT, 10.668]))  # 30 ft and the default 35 ft
    plan = cp.plan(polygon, height, CAMERA_4K)
    seen = cp.coverage(plan, polygon, rng=np.random.default_rng(1000 + seed), samples=4000)
    assert seen.at_least_two >= 0.99, (seed, seen)
    assert seen.at_least_one >= seen.at_least_two
    # The yaw the survey holds keeps the image's long side across the tracks.
    assert plan.yaw_deg == pytest.approx(plan.track_heading_deg)


def test_coverage_counts_frames_honestly():
    """The check must be able to fail: a plan that leaves half the field out is seen to."""
    plan = cp.plan(SQUARE, THIRTY_FT, CAMERA_4K)
    half = cp.CoveragePlan(**{**plan.__dict__, "waypoints": plan.waypoints[:4], "tracks": 2})
    seen = cp.coverage(half, SQUARE, rng=np.random.default_rng(0), samples=4000)
    assert 0.4 < seen.at_least_two < 0.7
    # One frame only (a camera at 0.05 Hz) is not two.
    slow = cp.plan(SQUARE, THIRTY_FT, CAMERA_4K, frame_rate_hz=0.05)
    assert cp.coverage(slow, SQUARE, rng=np.random.default_rng(0), samples=2000).at_least_two < 0.5


@pytest.mark.parametrize(
    "polygon, height, speed, message",
    [
        ([(0.0, 0.0), (10.0, 0.0)], THIRTY_FT, 4.0, "three corners"),
        ([(0.0, 0.0), (10.0, 0.0), (20.0, 0.0)], THIRTY_FT, 4.0, "no area"),
        ([(0.0, 0.0), (10.0, 0.0), (10.0, float("nan"))], THIRTY_FT, 4.0, "not a number"),
        (SQUARE, 0.0, 4.0, "height"),
        (SQUARE, THIRTY_FT, 0.0, "speed"),
        (SQUARE, 7.5, 4.0, "7.62 m"),  # under the rules' floor for a survey: no plan is made for it
        (SQUARE, THIRTY_FT, 4.5, "4 m/s"),  # over the survey's speed cap
    ],
)
def test_inputs_that_cannot_be_planned_are_refused(polygon, height, speed, message):
    with pytest.raises(ValueError, match=message):
        cp.plan(polygon, height, CAMERA_4K, speed_mps=speed)


def test_the_two_phase_search_is_a_wide_fast_pass_then_tight_passes_only_where_something_was_seen():
    field = [(0.0, 0.0), (120.0, 0.0), (120.0, 80.0), (0.0, 80.0)]
    full = cp.plan(field, THIRTY_FT, CAMERA_4K)
    wide = cp.plan_wide_pass(field, THIRTY_FT, CAMERA_4K)
    across, _along = cp.footprint(CAMERA_4K, THIRTY_FT)
    assert wide.spacing_m > full.spacing_m and wide.spacing_m <= across - 0.85 + 1e-9 and wide.tracks < full.tracks
    assert cp.coverage(wide, field, rng=np.random.default_rng(3), samples=4000).at_least_one >= 0.99  # everything seen once
    tentative = [(20.0, 15.0), (22.0, 17.0), (95.0, 60.0)]  # two near each other, one far away
    again = cp.plan_revisit(tentative, THIRTY_FT, CAMERA_4K)
    assert again.length_m < 0.5 * full.length_m
    for point in tentative:
        assert cp.frames_seeing(again, point) >= 2
    # Nothing tentative: nothing to fly.
    assert cp.plan_revisit([], THIRTY_FT, CAMERA_4K).waypoints == ()


def test_the_plan_says_the_height_it_was_made_for():
    assert cp.plan(SQUARE, THIRTY_FT, CAMERA_4K).height_m == THIRTY_FT
    for make in (cp.plan_wide_pass,):
        with pytest.raises(ValueError, match="7.62 m"):
            make(SQUARE, 6.0, CAMERA_4K)
    with pytest.raises(ValueError, match="7.62 m"):
        cp.plan_revisit([(1.0, 1.0)], 6.0, CAMERA_4K)


def test_with_a_flight_boundary_no_waypoint_is_within_five_metres_of_it():
    """A track runs as far as the search area reaches anywhere in its swath, which at a corner is past the area
    itself. Given the flight boundary, each track end is pulled back along its track until it is 5 m inside."""
    triangle = [(0.0, 0.0), (60.0, 0.0), (30.0, 40.0)]
    free = cp.plan(triangle, THIRTY_FT, CAMERA_4K)
    outside = [point for point in free.waypoints if not cp.inside_with_margin(point, triangle, 0.0)]
    assert outside  # without a boundary the top track's ends are outside the triangle
    fence = [(-8.0, -8.0), (68.0, -8.0), (30.0, 46.0)]  # the flight boundary, a little larger than the search area
    kept = cp.plan(triangle, THIRTY_FT, CAMERA_4K, boundary=fence)
    assert kept.tracks == free.tracks and kept.pulled_back_m > 0.0
    assert all(cp.inside_with_margin(point, fence, 5.0) for point in kept.waypoints)
    assert cp.coverage(kept, triangle, rng=np.random.default_rng(5), samples=4000).at_least_two > 0.9  # what the boundary costs is seen, not hidden
    # A boundary the search area is well inside changes nothing.
    wide = [(-50.0, -50.0), (110.0, -50.0), (110.0, 90.0), (-50.0, 90.0)]
    assert cp.plan(triangle, THIRTY_FT, CAMERA_4K, boundary=wide).waypoints == free.waypoints
    assert kept.boundary_checked and not free.boundary_checked and kept.legs_outside_boundary == 0
    # A track that cannot be flown inside the boundary at all is left out, and the plan says so.
    tight = [(-5.0, -5.0), (65.0, -5.0), (30.0, 28.0)]
    cut = cp.plan(triangle, THIRTY_FT, CAMERA_4K, boundary=tight)
    assert cut.tracks < free.tracks and cut.tracks_left_out >= 1
    # A boundary no track fits in is not an empty plan handed back as if it were one.
    with pytest.raises(ValueError, match="inside the flight boundary"):
        cp.plan(triangle, THIRTY_FT, CAMERA_4K, boundary=[(100.0, 100.0), (120.0, 100.0), (110.0, 120.0)])
    # The margin is the safety limits' 5 m or more: it cannot be asked down.
    for bad in (0.0, 4.9, -1.0, float("nan")):
        with pytest.raises(ValueError, match="margin"):
            cp.plan(triangle, THIRTY_FT, CAMERA_4K, boundary=fence, boundary_margin_m=bad)
    # A leg that crosses a notch in the boundary is counted: only its ends can be kept inside by pulling back.
    notched = [(-10.0, -10.0), (70.0, -10.0), (70.0, 50.0), (32.0, 50.0), (32.0, 12.0), (28.0, 12.0), (28.0, 50.0), (-10.0, 50.0)]
    crossing = cp.plan(SQUARE, THIRTY_FT, CAMERA_4K, boundary=notched)
    assert crossing.legs_outside_boundary >= 1 and all(cp.inside_with_margin(point, notched, 5.0) for point in crossing.waypoints)
    # The revisit keeps to the boundary too.
    again = cp.plan_revisit([(30.0, 36.0)], THIRTY_FT, CAMERA_4K, boundary=fence)
    assert all(cp.inside_with_margin(point, fence, 5.0) for point in again.waypoints)


def test_a_revisit_point_or_heading_that_is_not_a_number_is_refused():
    for bad in ([(float("nan"), 1.0)], [(1.0, float("inf"))], [("east", 1.0)]):
        with pytest.raises(ValueError, match="not a number"):
            cp.plan_revisit(bad, THIRTY_FT, CAMERA_4K)
    with pytest.raises(ValueError, match="heading"):
        cp.plan_revisit([(1.0, 1.0)], THIRTY_FT, CAMERA_4K, track_heading_rad=float("nan"))
    with pytest.raises(ValueError, match="heading"):
        cp.plan(SQUARE, THIRTY_FT, CAMERA_4K, track_heading_rad=float("inf"))


def test_every_place_a_revisit_pass_is_for_is_inside_its_swath_with_the_marker_margin():
    """Places that drift across the tracks, each near the last, must not drag one pass off the first of them."""
    across, _along = cp.footprint(CAMERA_4K, THIRTY_FT)
    drifting = [(0.0, 0.0), (8.0, 6.6), (16.0, 9.9), (24.0, 12.1), (32.0, 13.7)]
    again = cp.plan_revisit(drifting, THIRTY_FT, CAMERA_4K)
    assert again.tracks >= 2
    for east, north in drifting:
        assert cp.frames_seeing(again, (east, north)) >= 2
        passes = [0.5 * (a[1] + b[1]) for a, b in zip(again.waypoints[0::2], again.waypoints[1::2])]
        assert min(abs(north - offset) for offset in passes) <= across / 2.0 - 0.85 + 1e-9
    # Many places in a ring: each in two frames or more.
    ring = [(30.0 + 20.0 * math.cos(k), 30.0 + 20.0 * math.sin(k)) for k in np.linspace(0.0, 2.0 * math.pi, 12, endpoint=False)]
    around = cp.plan_revisit(ring, THIRTY_FT, CAMERA_4K)
    assert all(cp.frames_seeing(around, point) >= 2 for point in ring)


def test_a_plan_can_start_from_the_corner_nearest_where_the_vehicle_is():
    """For planning again in mid-survey (the search area refined from the boundary markers): the same tracks,
    begun at the end nearest the vehicle."""
    default = cp.plan(SQUARE, THIRTY_FT, CAMERA_4K)
    for start in ((0.0, 0.0), (60.0, 0.0), (60.0, 40.0), (0.0, 40.0)):
        plan = cp.plan(SQUARE, THIRTY_FT, CAMERA_4K, start=start)
        assert set(plan.waypoints) == set(default.waypoints) and plan.length_m == pytest.approx(default.length_m)
        assert math.dist(plan.waypoints[0], start) == min(math.dist(point, start) for point in default.waypoints)
        assert cp.coverage(plan, SQUARE, rng=np.random.default_rng(2), samples=2000).at_least_two >= 0.99
    with pytest.raises(ValueError, match="start"):
        cp.plan(SQUARE, THIRTY_FT, CAMERA_4K, start=(float("nan"), 0.0))
