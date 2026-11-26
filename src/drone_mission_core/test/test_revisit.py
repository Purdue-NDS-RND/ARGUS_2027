"""LOC-4: which tracks the survey goes back to, where it may go, in what order, and the hover over each (drone_mission_core/revisit.py). No ROS."""

import math
import re
from pathlib import Path
from types import SimpleNamespace

import pytest

from drone_mission_core import revisit as rv

REPO = Path(__file__).resolve().parents[3]
AREA = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)]
OPEN = rv.Bounds(AREA)  # the search area, and no flight boundary in the sequence
FENCED = rv.Bounds(AREA, boundary=[(-3.0, -3.0), (103.0, -3.0), (103.0, 103.0), (-3.0, 103.0)])


def track(track_id, east, north, digit=7, sightings=20, alive=True, outside=False):
    return rv.TrackView(track_id, east, north, digit, sightings, alive, outside)


def test_a_track_needs_another_look_when_its_digit_was_not_read_or_it_was_seen_too_few_times():
    assert rv.why_revisit(track(1, 10, 10)) is None  # read, and seen often
    assert rv.why_revisit(track(2, 10, 10, digit=-1)) == "digit not read (20 sightings)"
    assert rv.why_revisit(track(3, 10, 10, sightings=rv.MIN_SIGHTINGS - 1)) == "7 sightings" and rv.why_revisit(track(3, 10, 10, sightings=rv.MIN_SIGHTINGS)) is None
    assert rv.why_revisit(track(4, 10, 10, digit=-1, outside=True)) is None  # not a marker of the field
    assert rv.why_revisit(track(5, 10, 10, digit=-1, alive=False)) is None  # the tracker has lost it: nothing to fly back to
    assert rv.why_revisit(track(6, math.nan, 10, digit=-1)) is None
    assert rv.HOVER_S == 3.0  # the spec's


def test_where_the_vehicle_may_be_sent_is_the_search_area_and_the_boundary_never_the_trackers_word():
    """The flight guardian's blocker: with no flight boundary in the sequence, a track put a kilometre away by one bad frame was a destination."""
    assert OPEN.point_problem((50.0, 50.0)) is None and OPEN.point_problem((101.5, 50.0)) is None  # a marker on the edge, its track a little outside
    assert OPEN.point_problem((103.0, 50.0)) == "3 m outside the search area" and OPEN.point_problem((1200.0, -400.0)) == "1170 m outside the search area"
    assert OPEN.point_problem((math.inf, 0.0)) == "its place is not a number"
    assert FENCED.point_problem((50.0, 50.0)) is None and FENCED.point_problem((99.0, 50.0)) == "not 5 m inside the flight boundary"
    far = [track(1, 1200.0, -400.0, digit=-1), track(2, 60.0, 40.0, digit=-1), track(3, 104.0, 50.0, sightings=1)]
    visits, left_out = rv.choose(far, here=(10.0, 10.0), time_left_s=3600.0, speed_mps=5.0, bounds=OPEN)
    assert [visit.track_id for visit in visits] == [2]
    assert left_out == ["track 1 (digit not read (20 sightings)): 1170 m outside the search area", "track 3 (1 sightings): 4 m outside the search area"]
    with pytest.raises(ValueError):
        rv.Bounds([(0.0, 0.0), (1.0, 1.0)])
    # A leg is held to the area too, and to the boundary by the planner's own margin for legs.
    assert OPEN.leg_problem((10.0, 10.0), (90.0, 90.0)) is None
    horseshoe = rv.Bounds([(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (60.0, 100.0), (60.0, 40.0), (40.0, 40.0), (40.0, 100.0), (0.0, 100.0)])
    assert horseshoe.leg_problem((20.0, 80.0), (80.0, 80.0)) == "the straight way to it leaves the search area"  # across the notch
    assert horseshoe.leg_problem((20.0, 80.0), (20.0, 20.0)) is None
    notched = rv.Bounds(AREA, boundary=[(-10.0, -10.0), (110.0, -10.0), (110.0, 110.0), (60.0, 110.0), (60.0, 50.0), (40.0, 50.0), (40.0, 110.0), (-10.0, 110.0)])
    assert notched.leg_problem((20.0, 46.0), (80.0, 46.0)) == "the straight way to it comes too near the flight boundary"  # 4 m under the notch: inside, and inside the margin
    assert notched.leg_problem((20.0, 44.0), (80.0, 44.0)) is None  # 6 m under it


def test_the_round_is_nearest_first_inside_the_bounds_and_within_the_time_left():
    tracks = [track(1, 50, 50), track(2, 80, 20, digit=-1), track(3, 20, 20, sightings=2), track(4, 99, 50, digit=-1), track(5, 60, 80, sightings=4)]
    visits, left_out = rv.choose(tracks, here=(10.0, 10.0), time_left_s=600.0, speed_mps=5.0, bounds=FENCED)
    assert [visit.track_id for visit in visits] == [3, 2, 5]  # nearest first: 14 m, then 60 m on, then 63 m on
    assert visits[0].why == "2 sightings" and visits[1].why == "digit not read (20 sightings)"
    assert left_out == ["track 4 (digit not read (20 sightings)): not 5 m inside the flight boundary"]
    # Without a flight boundary the point by the edge is flown too: it is in the search area.
    assert [visit.track_id for visit in rv.choose(tracks, (10.0, 10.0), 600.0, 5.0, OPEN)[0]] == [3, 2, 4, 5]
    # Little time: only what fits, legs and hovers together (14.1 m at 5 m/s and 3 s is 5.8 s; the next would be 12 s and 3 s more).
    visits, left_out = rv.choose(tracks, (10.0, 10.0), 12.0, 5.0, FENCED)
    assert [visit.track_id for visit in visits] == [3] and "track 2 (digit not read (20 sightings)): no time for it (15 s, 6 s left)" in left_out
    assert rv.choose(tracks, (10.0, 10.0), 0.0, 5.0, FENCED)[0] == []
    # As near as each other: the digit that was not read comes first.
    pair = [track(1, 30, 10, sightings=3), track(2, 10, 30, digit=-1)]
    assert [visit.track_id for visit in rv.choose(pair, (10.0, 10.0), 600.0, 5.0, OPEN)[0]] == [2, 1]
    assert rv.choose([track(1, 50, 50)], (10.0, 10.0), 600.0, 5.0, OPEN) == ([], [])  # nothing uncertain: no round
    # A way that leaves the bounds is not flown.
    horseshoe = rv.Bounds([(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (60.0, 100.0), (60.0, 40.0), (40.0, 40.0), (40.0, 100.0), (0.0, 100.0)])
    across = [track(1, 80.0, 80.0, digit=-1), track(2, 20.0, 60.0, sightings=3)]
    visits, left_out = rv.choose(across, (20.0, 80.0), 600.0, 5.0, horseshoe)
    assert [visit.track_id for visit in visits] == [2] and left_out == ["track 1 (digit not read (20 sightings)): the straight way to it leaves the search area"]


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


def test_a_tracker_full_of_tracks_is_weighed_and_flown_only_so_far_and_quickly():
    """The round is planned inside one 20 Hz tick: 200 false tracks must not stall it."""
    many = [track(index, 5.0 + (index % 19) * 5.0, 5.0 + (index // 19) * 8.0, digit=-1 if index % 2 else 3, sightings=1 + index % 5) for index in range(200)]
    visits, left_out = rv.choose(many, (10.0, 50.0), 3600.0, 5.0, FENCED)
    assert len(visits) == rv.MAX_VISITS == 20
    assert _least_cpu_s(lambda: rv.choose(many, (10.0, 50.0), 3600.0, 5.0, FENCED)) < 0.05
    assert f"{200 - rv.MAX_CANDIDATES} more uncertain track(s): only {rv.MAX_CANDIDATES} are weighed" in left_out
    assert all(visit.why.startswith("digit") for visit in visits)  # the unread are weighed first
    assert sum("more than 20 visits" in line for line in left_out) == rv.MAX_CANDIDATES - rv.MAX_VISITS


def fly(round_, start, speed_mps=5.0, dt=0.05, limit_s=600.0, wander=None, tracks=None):
    """Fly the round's targets at speed; returns (the log lines, where the vehicle was each tick, the time the round took)."""
    here, t, says, path = start, 0.0, [], []
    while t < limit_s:
        step = round_.update(here, t, None if tracks is None else tracks(t))
        if step.say:
            says.append((round(t, 2), step.say))
        if step.done:
            return says, path, t
        away = math.dist(here, step.target)
        move = min(away, speed_mps * dt)
        if away > 1e-9:
            here = (here[0] + move * (step.target[0] - here[0]) / away, here[1] + move * (step.target[1] - here[1]) / away)
        if wander is not None:
            here = wander(here, t)
        path.append((t, here))
        t += dt
    raise AssertionError("the round did not end")


def test_the_vehicle_holds_three_seconds_within_a_metre_of_each_point_and_goes_on():
    visits = [rv.Visit(3, 20.0, 20.0, "2 sightings"), rv.Visit(2, 80.0, 20.0, "digit not read (20 sightings)")]
    round_ = rv.Revisit(visits, 5.0, OPEN)
    says, path, took = fly(round_, (10.0, 10.0))
    assert round_.hovered == [3, 2] and round_.passed_over == []
    assert [text for _t, text in says] == ["re-visit 1 of 2: track 3 (2 sightings), 14 m away", "hovered 3 s over track 3; re-visit 2 of 2: track 2 (digit not read (20 sightings)), 60 m away", "hovered 3 s over track 2"]
    for visit in visits:
        near = [t for t, here in path if math.dist(here, (visit.east_m, visit.north_m)) <= rv.ARRIVE_RADIUS_M]
        assert near[-1] - near[0] >= rv.HOVER_S - 0.1  # held for the three seconds
    assert took == pytest.approx(14.1 / 5 + 3 + 60 / 5 + 3, abs=1.5)


def test_a_hover_the_vehicle_drifts_out_of_is_counted_again_and_a_point_it_cannot_reach_is_passed_over():
    round_ = rv.Revisit([rv.Visit(1, 20.0, 10.0, "3 sightings")], 5.0, OPEN)
    gust = lambda here, t: (here[0] + 3.0, here[1]) if 3.0 <= t < 3.05 else here  # blown 3 m off, a second into the hover  # noqa: E731
    says, path, took = fly(round_, (10.0, 10.0), wander=gust)
    assert round_.hovered == [1] and took > 2.0 + 1.0 + 3.0  # the hover started again after the gust
    # A point the vehicle is held away from: passed over after twice the leg's time and ten seconds, and the round goes on from where the vehicle is.
    held = rv.Revisit([rv.Visit(1, 60.0, 10.0, "digit not read (9 sightings)"), rv.Visit(2, 10.0, 30.0, "2 sightings")], 5.0, OPEN)
    wall = lambda here, t: (min(here[0], 40.0), here[1])  # noqa: E731
    says, path, took = fly(held, (10.0, 10.0), wander=wall)
    assert held.passed_over == [1] and held.hovered == [2]
    assert any(round(t) == 30 and text.startswith("track 1 not reached in 30 s (20.0 m away): going on; re-visit 2 of 2: track 2 (2 sightings), 36 m away") for t, text in says)
    # The height guard takes the vehicle off the leg for a while: the leg's time starts again, and nothing is passed over for it.
    paused = rv.Revisit([rv.Visit(1, 60.0, 10.0, "3 sightings")], 5.0, OPEN)
    assert paused.update((10.0, 10.0), 0.0).target == (60.0, 10.0)
    paused.restart_leg()
    assert paused.update((10.0, 10.0), 100.0).target == (60.0, 10.0) and paused.passed_over == []  # 100 s later, the leg is new
    assert rv.Revisit([], 5.0, OPEN).update((0.0, 0.0), 0.0) == rv.Step(None, True)


def test_each_leg_is_checked_from_where_the_vehicle_really_is():
    """The flight guardian: after a point is given up, the next leg started from somewhere nobody had checked."""
    horseshoe = rv.Bounds([(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (60.0, 100.0), (60.0, 40.0), (40.0, 40.0), (40.0, 100.0), (0.0, 100.0)])
    visits = [rv.Visit(1, 20.0, 30.0, "3 sightings"), rv.Visit(2, 80.0, 20.0, "2 sightings")]  # planned from the left arm, under the notch: each leg inside
    assert horseshoe.leg_problem((20.0, 80.0), (20.0, 30.0)) is None and horseshoe.leg_problem((20.0, 30.0), (80.0, 20.0)) is None
    round_ = rv.Revisit(visits, 5.0, horseshoe)
    blown = (80.0, 80.0)  # the vehicle is in the right arm when the round starts: the way to the first is across the notch
    step = round_.update(blown, 0.0)
    assert round_.passed_over == [1] and "track 1: the straight way to it leaves the search area from here" in step.say
    assert step.target == (80.0, 20.0) and "re-visit 2 of 2" in step.say  # the second can be reached from here, down the right arm


def test_the_visit_follows_its_track_and_a_track_that_is_dropped_or_leaves_the_bounds_is_passed_over():
    visits = [rv.Visit(1, 30.0, 30.0, "digit not read (9 sightings)"), rv.Visit(2, 60.0, 30.0, "3 sightings"), rv.Visit(3, 80.0, 60.0, "2 sightings")]
    round_ = rv.Revisit(visits, 5.0, OPEN)

    def tracker(t):
        """Track 1 is refined by 0.4 m and then shifts 6 m (a position reset); track 2 is dropped; track 3 is put far outside the area."""
        first = track(1, 30.4, 30.0, digit=-1) if t < 4.0 else track(1, 36.0, 30.0, digit=-1)
        return [first, track(2, 60.0, 30.0, sightings=3, alive=False), track(3, 500.0, 60.0, sightings=2)]

    says, path, took = fly(round_, (10.0, 30.0), tracks=tracker)
    said = " | ".join(text for _t, text in says)
    assert round_.hovered == [1] and round_.passed_over == [2, 3]
    assert "track 2: the tracker no longer holds it" in said and "track 3: now 400 m outside the search area" in said
    held = [here for t, here in path if t > took - 2.0]
    assert all(math.dist(here, (36.0, 30.0)) <= 1.0 for here in held)  # the hover is over where the track is now
    assert said.count("re-visit 1 of 3") == 2  # the leg was started again when the track moved by more than the hover's radius
    # With no tracks handed in (a caller that has none), the planned places are flown.
    plain = rv.Revisit(visits[:1], 5.0, OPEN)
    fly(plain, (10.0, 30.0))
    assert plain.hovered == [1]


def test_a_track_that_creeps_in_small_steps_has_its_leg_checked_again_once_it_has_gone_a_metre():
    """The flight guardian, second round: a destination moving under a metre at a time shifted the leg with no check of it."""
    notched = rv.Bounds(AREA, boundary=[(-10.0, -10.0), (110.0, -10.0), (110.0, 110.0), (60.0, 110.0), (60.0, 50.0), (40.0, 50.0), (40.0, 110.0), (-10.0, 110.0)])
    round_ = rv.Revisit([rv.Visit(1, 80.0, 40.0, "digit not read (9 sightings)")], 5.0, notched)
    here = (20.0, 40.0)  # the leg runs 10 m under the notch: allowed
    assert round_.update(here, 0.0, [track(1, 80.0, 40.0, digit=-1)]).target == (80.0, 40.0)
    says = []
    for index in range(1, 40):  # the track creeps north 0.4 m a tick: the point itself stays 5 m inside the boundary, the leg to it does not
        step = round_.update(here, 0.05 * index, [track(1, 80.0, 40.0 + 0.4 * index, digit=-1)])
        if step.say:
            says.append(step.say)
        if step.done:
            break
    assert round_.passed_over == [1] and any("comes too near the flight boundary from here" in text for text in says)
    assert sum("re-visit 1 of 1" in text for text in says) >= 2  # the leg was started, and checked, again on the way


def test_tracks_are_read_from_the_tracks_message_and_the_names_are_the_localisation_packages_own():
    message = SimpleNamespace(tracks=[
        SimpleNamespace(track_id=4, position_enu=SimpleNamespace(x=12.5, y=-3.0, z=0.0), digit=-1, sightings=5, state=1, outside=False),
        SimpleNamespace(track_id=5, position_enu=SimpleNamespace(x=1.0, y=2.0, z=0.0), digit=3, sightings=40, state=3, outside=True),
    ])  # fmt: skip
    assert rv.views(message, alive_states=(0, 1, 2)) == [rv.TrackView(4, 12.5, -3.0, -1, 5, True, False), rv.TrackView(5, 1.0, 2.0, 3, 40, False, True)]
    localisation = REPO / "src" / "argus_localisation" / "argus_localisation"
    assert f'TRACKS_TOPIC = "{rv.TRACKS_TOPIC}"' in (localisation / "track_messages.py").read_text()
    assert f"UNKNOWN = {rv.UNKNOWN_DIGIT}" in (localisation / "digit_vote.py").read_text()
    states = dict(re.findall(r"uint8 (TENTATIVE|CONFIRMED|COASTING|LOST)=(\d)", (REPO / "src/argus_interfaces/msg/MarkerTrack.msg").read_text()))
    assert states == {"TENTATIVE": "0", "CONFIRMED": "1", "COASTING": "2", "LOST": "3"}
    from drone_mission_core import coverage_planner, safety_limits

    assert rv.GEOFENCE_MARGIN_M == safety_limits.GEOFENCE_MARGIN_M
    assert "float(margin_m) - 0.5" in Path(coverage_planner.__file__).read_text() and rv.LEG_MARGIN_GIVE_M == 0.5  # the planner's rule for its own legs


def message_at(stamp_ns, *tracks):
    header = SimpleNamespace(stamp=SimpleNamespace(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000))
    return SimpleNamespace(header=header, tracks=[SimpleNamespace(track_id=t.track_id, position_enu=SimpleNamespace(x=t.east_m, y=t.north_m, z=0.0), digit=t.digit, sightings=t.sightings,
                                                                 state=1 if t.alive else 3, outside=t.outside) for t in tracks])  # fmt: skip


def test_the_feeds_age_is_the_messages_own_stamp_and_nothing_is_subscribed_with_the_flag_off():
    feed = rv.TrackFeed(alive_states=(0, 1, 2))
    assert feed.newest(5_000_000_000) is None
    feed.store(message_at(5_000_000_000, track(4, 12.5, -3.0, digit=-1, sightings=5)), arrived_ns=9_000_000_000)  # of a frame at 5 s, arriving late at 9 s
    tracks, age_s = feed.newest(9_500_000_000)
    assert tracks == [rv.TrackView(4, 12.5, -3.0, -1, 5, True, False)] and age_s == 4.5  # old by its frame, not new by its arrival
    feed.store(message_at(0, track(4, 12.5, -3.0)), arrived_ns=9_000_000_000)  # a message with no stamp: counted from when it came
    assert feed.newest(9_500_000_000)[1] == 0.5
    feed.store(message_at(20_000_000_000, track(4, 12.5, -3.0)), arrived_ns=9_000_000_000)  # stamped ahead of the clock: as far off as it is ahead
    assert feed.newest(9_500_000_000)[1] == 10.5
    made = []
    node = SimpleNamespace(create_subscription=lambda *a, **k: made.append(a), get_logger=lambda: SimpleNamespace(info=lambda text: None))
    off = [SimpleNamespace(features={"survey_search": True}), SimpleNamespace(features={"survey_revisit": False}), SimpleNamespace(features={})]
    assert rv.attach_tracks(node, off) is None and made == []


def test_with_the_flag_on_the_executor_subscribes_to_the_trackers_topic_and_the_context_hands_the_tracks_on(monkeypatch):
    """The spec audit: the flag-on branch and the context's accessor had only the sim flight for proof."""
    import sys
    import types

    class MarkerTrack:
        TENTATIVE, CONFIRMED, COASTING, LOST = 0, 1, 2, 3

    monkeypatch.setitem(sys.modules, "argus_interfaces", types.SimpleNamespace())
    monkeypatch.setitem(sys.modules, "argus_interfaces.msg", types.SimpleNamespace(MarkerTrack=MarkerTrack, MarkerTrackArray=object))
    made, said = [], []
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=7_000_000_000))
    node = SimpleNamespace(create_subscription=lambda kind, topic, callback, depth: made.append((kind, topic, callback, depth)) or "subscription", get_clock=lambda: clock,
                           get_logger=lambda: SimpleNamespace(info=said.append))  # fmt: skip
    feed = rv.attach_tracks(node, [SimpleNamespace(features={"survey_search": True, "survey_revisit": True})])
    assert isinstance(feed, rv.TrackFeed) and made[0][1] == "/loc/tracks" and made[0][0] is object and node._marker_tracks_subscription == "subscription"
    assert "Survey re-visit on (survey_revisit)" in said[0]
    made[0][2](message_at(6_800_000_000, track(4, 12.5, -3.0, digit=-1), track(5, 1.0, 2.0, alive=False)))  # the subscription's own callback
    tracks, age_s = feed.newest(7_000_000_000)
    assert [(item.track_id, item.alive) for item in tracks] == [(4, True), (5, False)] and age_s == pytest.approx(0.2)
    # The mission's side: MissionContext.marker_tracks() reads the executor's feed, and is None without one.
    source = (REPO / "src/drone_mission_core/drone_mission_core/mission_context.py").read_text()
    body = source[source.index("    def marker_tracks(self):") : source.index("    def localisation_control_ready(self)")]
    namespace: dict = {}
    exec("class Context:\n" + body, namespace)  # the method as it is written, on a stand-in for the context
    context = namespace["Context"]()
    context._node, context._now_ns = SimpleNamespace(_marker_tracks=feed), lambda: 7_100_000_000
    assert context.marker_tracks()[1] == pytest.approx(0.3)
    context._node = SimpleNamespace()
    assert context.marker_tracks() is None
