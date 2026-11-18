"""MissionContext's estimator calls (FND-07), on the real executor and its real context.

rclpy is replaced by the stand-ins of test_executor_feature_overrides.py and
the wiring of test_executor_target_estimator.py, so this runs without ROS.
With the estimator's flag off every call says "nothing": no target, not
ready, no permission. With it on, a mission reads the estimator in-process
and the descend and release rules are answered here, not in the mission.
"""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_executor_feature_overrides import executor_module, start  # noqa: E402, F401  (executor_module is a fixture)
from test_executor_target_estimator import (  # noqa: E402, F401  (wiring is a fixture)
    BASE,
    CONTRACT_TOPIC,
    FLAGS,
    HEIGHT,
    MS,
    POSE_TOPIC,
    S,
    VELOCITY_TOPIC,
    contract_message,
    pose_message,
    say_image_size,
    stamp,
    velocity_message,
    wiring,
)

RANGEFINDER_TOPIC = "/mavros/rangefinder/rangefinder"
TARGET = (2.0, -1.5, 0.0)


def fly(wiring, until_ns, range_m=None):
    while wiring.now_ns <= until_ns:
        for callback in wiring.callbacks(POSE_TOPIC):
            callback(pose_message(wiring.now_ns))
        wiring.callbacks(VELOCITY_TOPIC)[0](velocity_message(wiring.now_ns))
        if range_m is not None:
            wiring.callbacks(RANGEFINDER_TOPIC)[0](SimpleNamespace(header=SimpleNamespace(stamp=stamp(wiring.now_ns)), range=range_m, min_range=0.1, max_range=40.0))
        wiring.now_ns += 33 * MS


def see_target(wiring, frames=3):
    """Three frames of the bullseye at TARGET, each handled 40 ms after it was taken."""
    from drone_mission_core.frame_projection import load_camera
    from drone_mission_core.projection_v2 import pixel_of_ground_point

    camera, mount = load_camera("sim_delivery", "sim_iris_downcam")
    pixel = pixel_of_ground_point(camera, mount, TARGET, (0.0, 0.0, HEIGHT), (0.0, 0.0, 0.0, 1.0))
    say_image_size(wiring)  # the 2026 detector says what size it ran on before its first detection
    for _ in range(frames):
        frame_ns = wiring.now_ns
        fly(wiring, frame_ns + 40 * MS)
        wiring.callbacks(CONTRACT_TOPIC)[0](contract_message(frame_ns, pixel))
        fly(wiring, frame_ns + 100 * MS)


def test_flag_off_every_call_says_nothing_and_nothing_is_imported(executor_module, monkeypatch, wiring):
    node = start(executor_module, monkeypatch)
    context = node._mission_context
    assert context.estimator_ready() is False
    assert context.primary_target("bullseye") is None
    assert not context.may_descend("bullseye") and context.may_descend("bullseye").reason == "the target estimator is off (use_target_estimator)"
    assert not context.may_release("bullseye")
    assert context.actuation_time_ns() == wiring.now_ns + 80 * MS
    assert context.height_agl() is None and context.latlon_to_local_enu(40.0, -86.0) is None  # no pose yet
    for module in ("target_estimator", "projection_v2"):
        assert f"drone_mission_core.{module}" not in sys.modules, module


def test_flag_off_height_is_the_baro_height_above_home_and_a_stale_pose_gives_none(executor_module, monkeypatch, wiring):
    node = start(executor_module, monkeypatch)
    node._local_pose = pose_message(BASE)
    height = node._mission_context.height_agl()
    assert (height.value_m, height.source) == (HEIGHT, "baro")
    wiring.now_ns = BASE + 500 * MS
    assert node._mission_context.height_agl().pose_age_s == pytest.approx(0.5)
    wiring.now_ns = BASE + 501 * MS  # the pose stream has stalled: the vehicle is no longer known to be there
    assert node._mission_context.height_agl() is None


def test_flag_on_a_mission_reads_the_primary_target_and_the_rules_in_process(executor_module, monkeypatch, wiring):
    node = start(executor_module, monkeypatch, feature_overrides=FLAGS)
    context = node._mission_context
    assert context.estimator_ready() is False  # no pose history yet
    fly(wiring, BASE + 200 * MS)
    assert context.estimator_ready() is True and context.primary_target("bullseye") is None
    assert context.may_descend("bullseye").reason == "no track to descend on"

    see_target(wiring)
    target = context.primary_target("bullseye")
    assert target.state == "CONFIRMED" and target.class_id == "bullseye"
    assert target.position == pytest.approx(TARGET, abs=0.01)
    assert target.covariance[0][0] > 0.0 and target.sigma_m == pytest.approx(math.sqrt(max(target.covariance[0][0], target.covariance[1][1])), rel=0.2)
    assert 0.0 <= target.age_s < 0.2 and target.for_ns == context.actuation_time_ns()
    assert context.may_descend("bullseye") and context.may_release("bullseye")
    assert context.primary_target("bucket") is None and not context.may_descend("bucket")

    # 0.4 s after the last frame: too old to release on, and COASTING, so not to descend on either.
    last_frame_ns = wiring.now_ns - 100 * MS - 33 * MS
    while wiring.now_ns < last_frame_ns + 400 * MS:
        fly(wiring, wiring.now_ns)
    assert context.primary_target("bullseye").state == "COASTING"
    assert not context.may_release("bullseye") and not context.may_descend("bullseye")
    assert "COASTING" in context.may_descend("bullseye").reason


def test_flag_on_the_pose_at_a_past_instant_and_the_actuation_time(executor_module, monkeypatch, wiring):
    node = start(executor_module, monkeypatch, feature_overrides=FLAGS)
    context = node._mission_context
    fly(wiring, BASE + 300 * MS)
    assert context.pose_at(BASE + 150 * MS).position == pytest.approx((0.0, 0.0, HEIGHT))
    assert context.actuation_time_ns() == wiring.now_ns + 80 * MS


def test_height_above_ground_is_the_rangefinders_when_it_is_healthy(executor_module, monkeypatch, wiring):
    node = start(executor_module, monkeypatch, feature_overrides=FLAGS)
    context = node._mission_context
    fly(wiring, BASE + 300 * MS)  # no rangefinder readings: the height above home
    assert (context.height_agl().value_m, context.height_agl().source) == (HEIGHT, "baro")
    fly(wiring, BASE + 800 * MS, range_m=HEIGHT - 1.2)  # the ground here is 1.2 m above home
    height = context.height_agl()
    assert height.source == "rangefinder" and height.value_m == pytest.approx(HEIGHT - 1.2, abs=1e-6)


def fix_message(ns, latitude=40.0, longitude=-86.0, status=0):
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp(ns)), status=SimpleNamespace(status=status), latitude=latitude, longitude=longitude)


def test_a_latitude_and_longitude_in_the_local_frame_from_where_the_vehicle_is(executor_module, monkeypatch, wiring):
    from drone_mission_core.target_access import METRES_PER_DEGREE

    node = start(executor_module, monkeypatch)  # needs no flag
    context = node._mission_context
    north_100 = 40.0 + 100.0 / METRES_PER_DEGREE
    node._local_pose = pose_message(BASE, east=20.0, north=-30.0)
    assert context.latlon_to_local_enu(north_100, -86.0) is None  # no GPS fix yet
    node._global_gps = fix_message(BASE - 100 * MS)
    assert context.latlon_to_local_enu(north_100, -86.0) == pytest.approx((20.0, 70.0), abs=1e-6)
    # A fix that says it is no fix, a fix from 0.3 s before the pose, or one over 2 s old: nothing, ask again.
    node._global_gps = fix_message(BASE - 100 * MS, status=-1)
    assert context.latlon_to_local_enu(north_100, -86.0) is None
    node._global_gps = fix_message(BASE - 300 * MS)
    assert context.latlon_to_local_enu(north_100, -86.0) is None
    node._global_gps = fix_message(BASE)
    wiring.now_ns = BASE + 2100 * MS
    assert context.latlon_to_local_enu(north_100, -86.0) is None


def test_with_the_pose_history_the_fix_is_paired_with_the_pose_of_its_own_moment(executor_module, monkeypatch, wiring):
    from drone_mission_core.target_access import METRES_PER_DEGREE

    node = start(executor_module, monkeypatch, feature_overrides=FLAGS)
    context = node._mission_context
    while wiring.now_ns <= BASE + 1000 * MS:  # flying east at 4 m/s
        east = 4.0 * (wiring.now_ns - BASE) * 1e-9
        for callback in wiring.callbacks(POSE_TOPIC):
            callback(pose_message(wiring.now_ns, east=east))
        wiring.callbacks(VELOCITY_TOPIC)[0](velocity_message(wiring.now_ns))
        wiring.now_ns += 33 * MS
    node._global_gps = fix_message(BASE + 500 * MS)  # half a second old: the vehicle was 2 m back then
    east, north = context.latlon_to_local_enu(40.0 + 100.0 / METRES_PER_DEGREE, -86.0)
    assert (east, north) == pytest.approx((2.0, 100.0), abs=1e-6)  # not 4 m: the newest pose is not the fix's


def test_a_rule_asked_about_one_track_is_never_answered_about_another(executor_module, monkeypatch, wiring):
    from drone_mission_core.target_estimator import Measurement

    node = start(executor_module, monkeypatch, feature_overrides=FLAGS)
    context, feed = node._mission_context, node._target_estimator
    fly(wiring, BASE + 200 * MS)
    see_target(wiring)
    mine = context.primary_target("bullseye")
    assert context.may_release("bullseye", track_id=mine.track_id).track_id == mine.track_id
    # The target is lost; something else red, 30 m away, is seen and confirmed.
    for index in range(4):
        fly(wiring, wiring.now_ns + 100 * MS)
        feed.estimator.update([Measurement("bullseye", 30.0, 0.0, [[0.01, 0.0], [0.0, 0.01]])], wiring.now_ns - 20 * MS)
    other = context.primary_target("bullseye")
    assert other.track_id != mine.track_id and other.state == "CONFIRMED"
    assert context.may_release("bullseye")  # by class: yes, but about the other one
    refused = context.may_release("bullseye", track_id=mine.track_id)
    assert not refused and refused.track_id == mine.track_id and "COASTING" in refused.reason
    assert not context.may_descend("bullseye", track_id=mine.track_id)
    gone = context.may_descend("bullseye", track_id=999)
    assert not gone and gone.reason == "bullseye track 999 is gone"
    with pytest.raises(TypeError):  # an id can never be put where the point goes
        context.may_release("bullseye", mine.track_id)
    off = start(executor_module, monkeypatch)._mission_context.may_descend("bullseye", track_id=1)
    assert not off and off.reason == "the target estimator is off (use_target_estimator)"
