"""The replay's `core` mode (SIM-09): a bag through the estimator without ROS, the same answer every time, and its metrics."""

import json
import subprocess
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from replay import bag, core, metrics, topics  # noqa: E402

import replay_flight as flight  # noqa: E402

TRUTH = flight.TARGET[:2]


@pytest.fixture(scope="module")
def hover_bag(tmp_path_factory):
    return bag.write_bag(tmp_path_factory.mktemp("bags") / "hover", flight.hover_records(), storage="mcap")


def test_a_recorded_hover_replays_to_a_confirmed_track_on_the_target(hover_bag):
    result = core.replay_bag(hover_bag, truth_en=TRUTH)
    assert result["frames"] == 105 and result["counters"]["measurements"] == 105 and result["undecodable"] == {}
    assert result["totals"]["accepted"] == 104 and result["totals"]["started"] == 1 and result["totals"]["rejected"] == 0
    assert result["states"]["CONFIRMED"] >= 30 and result["tracks_at_end"][0]["state"] == "CONFIRMED"
    assert result["error_m"]["p95"] < 0.05 and result["error_m"]["count"] == result["states"]["CONFIRMED"]
    assert result["time_to_lock_s"] == pytest.approx(0.2, abs=0.11)  # three frames, read at the next 10 Hz tick
    assert result["without_lock_after_first_lock_s"] == 0.0
    assert result["capture_to_heard_s"] == {"count": 105, "median": 0.04, "p95": 0.04, "max": 0.04}
    # The filter was told 2 px and 1 degree and given half a pixel: its innovations are small and none is over the gate.
    assert result["nis"]["count"] == 104 and result["nis"]["mean"] < 0.5 and result["nis"]["fraction_above_gate"] == 0.0
    lines = metrics.short_lines(result)
    assert lines[0].startswith("105 frames with detections") and any(line.startswith("NIS: mean ") for line in lines)
    assert any("capture to heard: median 40 ms, p95 40 ms" in line for line in lines)


def test_the_same_bag_gives_the_same_bytes_every_time_and_in_either_storage(hover_bag, tmp_path):
    first = json.dumps(core.replay_bag(hover_bag, truth_en=TRUTH), sort_keys=True)
    second = json.dumps(core.replay_bag(hover_bag, truth_en=TRUTH), sort_keys=True)
    assert first == second
    as_sqlite = bag.write_bag(tmp_path / "hover_db", flight.hover_records(), storage="sqlite3")
    assert json.dumps(core.replay_bag(as_sqlite, truth_en=TRUTH), sort_keys=True) == first


def test_the_replay_runs_the_executors_own_objects(hover_bag):
    from drone_mission_core.estimator_feed import EstimatorFeed
    from drone_mission_core.frame_projection import load_camera
    from drone_mission_core.pose_buffer import PoseBuffer

    replay = core.Replay(*load_camera("sim_delivery", "sim_iris_downcam")).run(hover_bag)
    assert isinstance(replay.feed, EstimatorFeed) and isinstance(replay.poses, PoseBuffer)
    assert len(replay.samples) == 40 and replay.samples[1]["t_ns"] - replay.samples[0]["t_ns"] == core.TICK_NS  # 10 Hz of bag time
    assert set(replay.handlers) >= {topics.POSE, topics.VELOCITY, topics.RANGEFINDER, topics.contract_topic("target_cv"), topics.contract_topic("sim_oracle")}
    assert replay.ground.ground_up(flight.T0 + 2 * flight.S)[1] == "rangefinder"  # the rangefinder readings reached projection v2


def test_late_frames_show_in_the_latency_and_a_gap_in_the_lock(tmp_path):
    records = [r for r in flight.hover_records(seconds=6.0, latency_ms=250) if not (r.topic == topics.contract_topic("target_cv") and 2.0 <= (r.log_time_ns - flight.T0) / flight.S < 4.5)]
    result = core.replay_bag(bag.write_bag(tmp_path / "gap", records, storage="sqlite3"), truth_en=TRUTH)
    assert result["capture_to_heard_s"]["median"] == 0.25
    assert {"CONFIRMED", "COASTING", "LOST"} <= set(result["states"])  # 2.5 s unseen: coasting, then lost, then found again
    assert 2.0 < result["without_lock_after_first_lock_s"] < 3.2
    assert 0.3 < result["lost_state_s"] < 1.0  # of the 2.5 s unseen, the part after coasting
    states = [state for _t, state, _error in result["error_over_time"]]
    assert states[-1] == "CONFIRMED" and states.index("LOST") > states.index("COASTING") > states.index("CONFIRMED")
    assert [track["id"] for track in result["tracks_at_end"]] == [1]  # found again under its own id, and confirmed again


def recorded_targets(ns, east, north, state="CONFIRMED", track_id=1):
    from replay.cdr import Message

    head = flight.header(ns)
    position = Message(x=east, y=north, z=0.0)
    box = Message(center=Message(position=position, orientation=Message(x=0.0, y=0.0, z=0.0, w=1.0)), size=Message(x=0.1, y=0.1, z=0.0))
    detection = Message(header=head, results=[], bbox=box, id=f"bullseye/{track_id}/{state}")
    return bag.Record(topics.ESTIMATOR_TARGETS, topics.RECORDED[topics.ESTIMATOR_TARGETS], ns, Message(header=head, detections=[detection]))


def test_the_replayed_track_is_set_against_the_one_the_flight_itself_published(tmp_path, hover_bag):
    assert core.replay_bag(hover_bag)["against_the_recorded_tracks"] is None  # that bag holds no recorded tracks
    recorded = [recorded_targets(flight.T0 + index * 100 * flight.MS, TRUTH[0] + 0.03, TRUTH[1], "CONFIRMED" if index >= 12 else "TENTATIVE") for index in range(5, 41)]
    result = core.replay_bag(bag.write_bag(tmp_path / "with_tracks", flight.hover_records() + recorded))
    compared = result["against_the_recorded_tracks"]
    assert compared["both_confirmed"] >= 25 and compared["distance_m"]["median"] == pytest.approx(0.03, abs=0.01)
    assert compared["confirmed_only_in_the_replay"] >= 1 and compared["confirmed_only_in_the_flight"] == 0  # the replay locked before the "flight" said it had
    assert any(line.startswith("against the tracks the flight itself published: median 0.03") for line in metrics.short_lines(result))


def test_a_loss_that_is_never_made_good_counts_as_lost_to_the_end_of_the_bag(tmp_path):
    records = [r for r in flight.hover_records(seconds=8.0) if not (r.topic == topics.contract_topic("target_cv") and (r.log_time_ns - flight.T0) / flight.S >= 2.0)]
    result = core.replay_bag(bag.write_bag(tmp_path / "lost", records))
    assert result["tracks_at_end"][0]["state"] == "LOST"
    assert result["lost_state_s"] == pytest.approx(8.0 - 2.0 - 2.0, abs=0.25)  # unseen from 2 s, LOST 2 s after that, to the end at 8 s
    assert result["without_lock_after_first_lock_s"] == 0.0  # that one stops at the last lock: it is about gaps, not the ending


def test_measurements_over_the_gate_show_in_the_nis_fraction_and_the_gate_is_the_estimators(tmp_path):
    from drone_mission_core import target_estimator

    assert metrics.GATE_NIS == target_estimator.GATE_NIS == 9.21
    # The target jumps 1.5 m after two seconds: five detections outside the gate, and the fifth moves the track.
    early = [r for r in flight.hover_records(seconds=2.0)]
    moved = (flight.TARGET[0] + 1.5, flight.TARGET[1], 0.0)
    late = [r for r in flight.hover_records(seconds=4.0, target=moved) if r.topic == topics.contract_topic("target_cv") and (r.log_time_ns - flight.T0) / flight.S > 2.05]
    poses = [r for r in flight.hover_records(seconds=4.0) if r.topic != topics.contract_topic("target_cv") and (r.log_time_ns - flight.T0) / flight.S > 2.05]
    result = core.replay_bag(bag.write_bag(tmp_path / "jump", early + late + poses))
    assert result["totals"]["rejected"] == 5 and result["totals"]["resets"] == 1
    assert result["nis"]["fraction_above_gate"] == pytest.approx(5 / result["nis"]["count"], abs=1e-6) and result["nis"]["mean"] > 1.0


def test_latency_on_a_wall_time_bag_is_taken_through_the_sim_clock_tap(tmp_path):
    from replay.cdr import Message

    offset = 1_794_000_000 * flight.S  # the recorder's wall clock is this far from the frames' sim clock
    records = []
    for record in flight.hover_records(seconds=2.0):
        records.append(bag.Record(record.topic, record.type_name, record.log_time_ns + offset, record.message))
    for index in range(-10, 300):  # the tap: Gazebo's clock every 10 ms, heard on the wall clock
        sim = flight.T0 + index * 10 * flight.MS
        records.append(bag.Record(topics.SIM_CLOCK_TAP, "rosgraph_msgs/msg/Clock", sim + offset, Message(clock=flight.stamp(sim))))
    result = core.replay_bag(bag.write_bag(tmp_path / "wall", records))
    assert result["capture_to_heard_s"]["median"] == pytest.approx(0.04, abs=1e-6)  # 40 ms, not 57 years


def test_the_error_at_the_release_and_a_bag_without_the_truth(hover_bag):
    result = core.replay_bag(hover_bag, truth_en=TRUTH, release_ns=flight.T0 + 3 * flight.S + 7 * flight.MS)
    assert result["error_at_release"]["state"] == "CONFIRMED" and result["error_at_release"]["error_m"] < 0.05
    assert abs(result["error_at_release"]["tick_offset_s"]) <= 0.05
    blind = core.replay_bag(hover_bag)
    assert blind["error_m"] is None and blind["error_at_release"] is None and blind["states"] == result["states"]


def test_the_2026_detector_is_replayed_under_the_executors_rule_about_its_image_size(tmp_path):
    silent = [r for r in flight.hover_records() if r.topic != topics.LEGACY_IMAGE_SIZE]
    result = core.replay_bag(bag.write_bag(tmp_path / "silent", silent))
    assert result["frames"] == 0 and result["counters"]["dropped: the detector has not said what size its image is"] == 105
    wrong = [flight.size_record(flight.T0 + 10 * flight.MS, 1280, 720)] + silent
    result = core.replay_bag(bag.write_bag(tmp_path / "wrong", wrong))
    assert result["counters"]["dropped: the detector's image is not the calibrated mode's size"] == 105 and result["tracks_at_end"] == []


def test_an_empty_bag_and_a_message_that_cannot_be_decoded(tmp_path):
    empty = core.replay_bag(bag.write_bag(tmp_path / "empty", []))
    assert empty["ticks"] == 0 and empty["frames"] == 0 and empty["nis"] is None and metrics.short_lines(empty)
    records = flight.hover_records(seconds=1.0) + [bag.Record(topics.POSE, "geometry_msgs/msg/PoseStamped", flight.T0 + 500 * flight.MS, b"\x00\x01\x00\x00\x01")]
    result = core.replay_bag(bag.write_bag(tmp_path / "torn", records))
    assert result["undecodable"] == {topics.POSE: 1} and result["frames"] > 0


def test_the_command_line(hover_bag, tmp_path):
    out = tmp_path / "metrics.json"
    done = subprocess.run(
        [sys.executable, str(SIM / "replay" / "core.py"), str(hover_bag), "--truth", f"{TRUTH[0]},{TRUTH[1]}", "--out", str(out)], capture_output=True, text=True
    )
    assert done.returncode == 0, done.stderr
    assert "error of the CONFIRMED bullseye track: median" in done.stdout and "time to lock" in done.stdout
    assert json.loads(out.read_text()) == json.loads(json.dumps(core.replay_bag(hover_bag, truth_en=TRUTH)))


def test_metric_helpers():
    assert metrics.spread([]) is None and metrics.spread([3.0, 1.0, 2.0]) == {"count": 3, "median": 2.0, "p95": 3.0, "max": 3.0}
    tracks = [{"id": 2, "class": "bullseye", "state": "LOST", "hits": 50}, {"id": 5, "class": "bullseye", "state": "CONFIRMED", "hits": 3}, {"id": 1, "class": "bucket", "state": "CONFIRMED", "hits": 9}]
    assert metrics.primary(tracks, "bullseye")["id"] == 5 and metrics.primary(tracks, "green_x") is None
