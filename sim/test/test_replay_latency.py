"""The stage-by-stage breakdown of a frame's age from a bag (SIM-09). No ROS."""

import subprocess
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from replay import bag, latency, topics  # noqa: E402
from replay.bag import Record  # noqa: E402
from replay.cdr import Message  # noqa: E402

import replay_flight as flight  # noqa: E402

MS, S = flight.MS, flight.S
WALL0 = 1_794_000_000 * S  # the recorder's clock on a wall-time run
SIM0 = 300 * S  # Gazebo's clock at the same moment
IMAGE_TAP, ANNOTATED_TAP = topics.frame_tap_topic("/camera/image"), topics.frame_tap_topic("/target_cv/annotated")


def header_record(topic, stamp, heard):
    return Record(topic, "std_msgs/msg/Header", heard, Message(stamp=flight.stamp(stamp), frame_id="camera"))


def info_record(stamp, heard):
    return Record(topics.CAMERA_INFO, topics.RECORDED[topics.CAMERA_INFO], heard, {"header": {"stamp": flight.stamp(stamp)}, "width": 640, "height": 360})


def legacy_record(stamp, heard, found=True):
    point = Message(x=0.1, y=-0.2, z=900.0) if found else Message(x=-1.0, y=-1.0, z=0.0)
    return Record(topics.LEGACY_DETECTION, topics.RECORDED[topics.LEGACY_DETECTION], heard, Message(header=flight.header(stamp, "camera"), point=point))


def clock_record(sim, heard, topic=topics.SIM_CLOCK_TAP):
    return Record(topic, "rosgraph_msgs/msg/Clock", heard, Message(clock=flight.stamp(sim)))


def wall_time_flight(frames=300, slow_every=10, real_time_factor=1.0):
    """A sim flight on wall time, flags off: frames stamped by Gazebo, the detector's work taking 8 ms, or 150 ms on every tenth frame."""
    records = []
    for index in range(-5, frames * 4 + 20):  # the sim clock, every 10 ms of sim time
        sim = SIM0 + index * 10 * MS
        records.append(clock_record(sim, WALL0 + round(index * 10 * MS / real_time_factor)))
    free_at = 0
    for index in range(frames):
        stamp = SIM0 + index * 33 * MS
        rendered = WALL0 + round(index * 33 * MS / real_time_factor)
        info = rendered + 2 * MS
        image = info + 5 * MS
        work = (150 if index % slow_every == slow_every - 1 else 8) * MS
        done = max(image, free_at) + work  # one frame at a time: a frame waits for the one before
        free_at = done
        records += [info_record(stamp, info), header_record(IMAGE_TAP, stamp, image), header_record(ANNOTATED_TAP, stamp, done + 1 * MS)]
        records.append(legacy_record(done + 1 * MS + 200_000, done + 3 * MS, found=index % 7 != 3))  # stamped with the detector's clock
    return records


def test_a_wall_time_flight_is_followed_stage_by_stage_through_the_annotated_copy(tmp_path):
    result = latency.breakdown(latency.read_events(bag.write_bag(tmp_path / "flight", wall_time_flight(), storage="sqlite3")))
    assert result["clock"] == "frame stamps converted through the bag's sim-clock tap"
    assert result["matched"] == {"by the detection's own stamp": 0, "by the annotated copy": 257, "not matched": 0}  # 300 less the 43 not found
    assert result["frames_followed"] == 257
    stages = result["stages"]
    assert stages["render"]["median_ms"] == pytest.approx(2.0, abs=0.01) and stages["render"]["max_ms"] == pytest.approx(2.0, abs=0.01)
    assert stages["bridge"]["median_ms"] == pytest.approx(5.0) and stages["publish"]["median_ms"] == pytest.approx(1.8, abs=0.01)
    assert stages["detector"]["median_ms"] < 60.0 and stages["detector"]["max_ms"] > 150.0
    # The four stages are the whole age.
    age = result["frame_age_when_heard"]
    assert age["mean_ms"] == pytest.approx(sum(stages[name]["mean_ms"] for name in latency.STAGES), abs=0.01)
    # The slow frames are slow in the detector, and part of that is waiting behind the frame before.
    assert result["largest_stage_in_slow_frames"] == "detector"
    slow = result["slow_frames"]
    assert slow["mean_stage_ms"]["detector"] > 2 * slow["mean_stage_ms_of_all"]["detector"] > 0 and slow["mean_stage_ms"]["render"] == pytest.approx(2.0, abs=0.01)
    assert result["detector_working"]["median_ms"] == pytest.approx(9.0, abs=0.01) and result["detector_working"]["max_ms"] == pytest.approx(150.0, abs=1.01)
    assert result["detector_waiting"]["max_ms"] > 100.0 and result["detector_waiting"]["median_ms"] < 60.0
    assert result["frames_heard"] == {"camera_info": 300, "image": 300, "annotated": 300} and result["frames_without_an_annotated_copy"] == 0 and result["frames_the_tap_did_not_hear"] == 0
    text = "\n".join(latency.short_lines(result))
    assert "257 frames followed" in text and "largest: detector" in text and "detector waiting: median" in text


def test_a_sim_slower_than_real_time_does_not_stretch_the_stages(tmp_path):
    result = latency.breakdown(latency.read_events(bag.write_bag(tmp_path / "slow", wall_time_flight(frames=120, real_time_factor=0.8))))
    assert result["stages"]["render"]["median_ms"] == pytest.approx(2.0, abs=0.01)  # 2 ms of the recorder's time, at any real-time factor


def test_detections_that_carry_their_frames_stamp_need_no_matching(tmp_path):
    # A sim-time run with stamp_from_image on and the contract topic: everything on one clock, the recorder too.
    records = []
    for index in range(100):
        stamp = SIM0 + index * 33 * MS
        records += [info_record(stamp, stamp + 2 * MS), header_record(IMAGE_TAP, stamp, stamp + 6 * MS), clock_record(stamp, stamp, topics.CLOCK)]
        records.append(legacy_record(stamp, stamp + 40 * MS))
        records.append(flight.detection_record(stamp, stamp + 41 * MS, (320.0, 180.0)))
    result = latency.breakdown(latency.read_events(bag.write_bag(tmp_path / "simtime", records)))
    assert result["clock"] == "the recorder is on the frames' clock"
    assert result["matched"]["by the detection's own stamp"] == 100 and result["frames_followed"] == 100  # the 2026 topic; the contract adds no new frame
    assert result["frame_age_when_heard"]["median_ms"] == pytest.approx(40.0)
    assert result["stages"]["detector"]["median_ms"] == pytest.approx(34.0) and result["stages"]["publish"] is None  # not separable without the publish time
    assert result["detector_waiting"] is None


def test_a_detection_with_no_frame_near_it_is_counted_not_guessed(tmp_path):
    records = wall_time_flight(frames=30) + [legacy_record(WALL0 + 20 * S, WALL0 + 20 * S + MS)]
    result = latency.breakdown(latency.read_events(bag.write_bag(tmp_path / "stray", records)))
    assert result["matched"]["not matched"] == 1


def test_frames_the_detector_never_handled_are_counted(tmp_path):
    dropped = {SIM0 + index * 33 * MS for index in (20, 21, 22)}
    records = [r for r in wall_time_flight(frames=60) if not (r.topic == ANNOTATED_TAP and latency.stamp_ns(r.message) in dropped)]
    result = latency.breakdown(latency.read_events(bag.write_bag(tmp_path / "skipped", records)))
    assert result["frames_without_an_annotated_copy"] == 3 and result["frames_heard"]["annotated"] == 57


def test_the_detectors_work_and_wait_are_also_given_by_the_vehicles_height(tmp_path):
    # The first 5 s at 12 m, then at 2 m; the slow frames are made slow only when low.
    records = [r for r in wall_time_flight(frames=300, slow_every=10_000)]  # no slow frames at all
    low = wall_time_flight(frames=300, slow_every=5)
    start_low = WALL0 + 5 * S
    records = [r for r in records if r.log_time_ns < start_low or r.topic == topics.SIM_CLOCK_TAP] + [r for r in low if r.log_time_ns >= start_low and r.topic != topics.SIM_CLOCK_TAP]
    for index in range(100):
        at = WALL0 + index * 100 * MS
        records.append(flight.pose_record(at, up=12.0 if at < start_low else 2.0))
    result = latency.breakdown(latency.read_events(bag.write_bag(tmp_path / "descent", records)))
    bands = {band["from_m"]: band for band in result["detector_by_height"]}
    assert set(bands) == {0.0, 12.0} and bands[12.0]["to_m"] == 15.0
    assert bands[12.0]["working"]["p95_ms"] == pytest.approx(9.0, abs=0.01) and bands[12.0]["waiting"]["max_ms"] == 0.0
    assert bands[0.0]["working"]["p95_ms"] > 100.0 and bands[0.0]["waiting"]["p95_ms"] > 50.0
    assert any(line.startswith("  at 0 to 3 m: working median") for line in latency.short_lines(result))
    no_poses = latency.breakdown(latency.read_events(bag.write_bag(tmp_path / "blind", wall_time_flight(frames=30))))
    assert no_poses["detector_by_height"] == []


def test_the_clock_map():
    assert latency.ClockMap([(0, 1000), (100, 1200), (200, 1300)]).identity  # a second apart at most: one clock
    wall = latency.ClockMap([(SIM0, WALL0), (SIM0 + 100, WALL0 + 200), (SIM0 + 200, WALL0 + 300)])
    assert wall.identity is False
    assert wall.to_recorder(SIM0 + 50) == WALL0 + 100 and wall.to_recorder(SIM0 + 150) == WALL0 + 250
    assert wall.to_recorder(SIM0 - 1) is None and wall.to_recorder(SIM0 + 201) is None
    assert latency.ClockMap([]).identity and latency.ClockMap([]).to_recorder(42) == 42
    assert latency.spread_ms([]) is None and latency.spread_ms([MS, 3 * MS, 2 * MS]) == {"count": 3, "median_ms": 2.0, "p95_ms": 3.0, "max_ms": 3.0, "mean_ms": 2.0}


def test_the_command_line(tmp_path):
    folder = bag.write_bag(tmp_path / "flight", wall_time_flight(frames=60))
    out = tmp_path / "stages.json"
    done = subprocess.run([sys.executable, str(SIM / "replay" / "latency.py"), str(folder), "--out", str(out)], capture_output=True, text=True)
    assert done.returncode == 0, done.stderr
    assert "frame age when the detection is heard: median" in done.stdout and out.is_file()
    empty = subprocess.run([sys.executable, str(SIM / "replay" / "latency.py"), str(bag.write_bag(tmp_path / "empty", []))], capture_output=True, text=True)
    assert empty.returncode == 1 and "0 frames followed" in empty.stdout
