"""The survey's frame logging policy (LOC-11): every frame while the survey runs, giving way to the detector."""

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "argus_localisation"), str(SRC / "drone_mission_core"), str(SRC / "argus_camera")]

from argus_camera.frame_log import FrameLog  # noqa: E402
from argus_camera.frame_source import Frame  # noqa: E402
from argus_localisation import frame_logging as fl  # noqa: E402

S = 1_000_000_000


def frame(seq, stamp_ns=None, size=(64, 36)):
    image = np.full((size[1], size[0], 3), seq % 255, dtype=np.uint8)
    return Frame(image=image, seq=seq, capture_ns=100 * S + seq * 66_000_000 if stamp_ns is None else stamp_ns, mode="test", stamp_source="pts")


def logging_to(tmp_path, **settings):
    (tmp_path / "logs").mkdir(parents=True, exist_ok=True)  # the disk's folder is there before a flight; the policy makes none
    return fl.SurveyFrameLog(tmp_path / "logs", **{"workers": 0, "min_free_bytes": 0.0, **settings})  # (the machine's own free space is not these tests' business)


def healthy(dropped=0, rate_hz=15.0):
    return {"dropped": dropped, "below_minimum_rate": False, "rate_hz": rate_hz}


def test_the_policy_is_the_specs():
    assert fl.JPEG_QUALITY == 90 and fl.FLAG == "survey_frame_log" and fl.MAX_KEEP_EVERY == 16 and fl.RECOVER_S == 10.0
    assert fl.RATE_SHARE == 0.9 and fl.STARTUP_GRACE_S == 3.0 and fl.DEFAULT_WORKERS == 3


def test_every_frame_of_the_survey_is_logged_and_none_outside_it(tmp_path):
    log = logging_to(tmp_path)
    assert log.offer(frame(1)) is False and log.status()["logging"] is False  # before the survey: nothing
    folder = log.start(100 * S)
    assert folder.parent == tmp_path / "logs" and folder.name == "survey_000000100" and log.status()["logging"] is True
    for seq in range(2, 12):
        assert log.offer(frame(seq)) is True
    log.stop(wait_s=5.0)
    assert log.offer(frame(12)) is False
    index = [json.loads(line) for line in (folder / "index.jsonl").read_text().splitlines()]
    assert [line["seq"] for line in index] == list(range(2, 12)) and len(list(folder.glob("frame_*.jpg"))) == 10
    status = log.status()
    assert status["written"] == 10 and status["thinned"] == 0 and status["dropped"] == 0 and status["keep_every"] == 1 and status["folder"] == str(folder)
    json.dumps(status)
    # The next survey gets a folder of its own, also when it starts within the same second.
    assert log.start(500 * S).name == "survey_000000500"
    assert log.start(500 * S + 1000).name == "survey_000000500_2" and log.offer(frame(50)) is True
    log.stop(wait_s=5.0)
    assert log.status()["written"] == 11


def test_a_detector_that_keeps_its_rate_does_not_thin_the_log_however_many_frames_a_faster_camera_gives_it(tmp_path):
    """The 4K mode gives 17 frames a second and the detector's target is 15: it drops two a second by design. That
    is not being short of CPU, and the log must still take every frame."""
    log = logging_to(tmp_path, expected_rate_hz=15.0)
    log.start(100 * S)
    for second in range(1, 60):
        log.review(healthy(dropped=2 * second, rate_hz=15.0), (100 + second) * S)
    assert log.status()["keep_every"] == 1 and log.status()["reason"] == ""
    log.review(healthy(dropped=200, rate_hz=13.6), 161 * S)  # nine tenths of its rate is 13.5: still well
    assert log.status()["keep_every"] == 1
    log.stop(wait_s=5.0)


def test_when_the_detector_falls_under_its_rate_the_logger_halves_its_own_first_and_comes_back_when_it_is_well(tmp_path):
    log = logging_to(tmp_path, expected_rate_hz=15.0)
    log.start(100 * S)
    log.review(healthy(rate_hz=6.0), 101 * S)  # the first seconds of a survey are not judged
    assert log.status()["keep_every"] == 1
    log.review(healthy(rate_hz=12.0), 104 * S)
    assert log.status()["keep_every"] == 2 and "12.0 frames a second" in log.status()["reason"] and "13.5" in log.status()["reason"]
    taken = [log.offer(frame(seq)) for seq in range(1, 9)]
    assert taken == [True, False, True, False, True, False, True, False] and log.status()["thinned"] == 4
    log.review(healthy(rate_hz=12.5), 105 * S)
    log.review({"dropped": 0, "below_minimum_rate": True, "rate_hz": 7.0}, 106 * S)
    assert log.status()["keep_every"] == 8 and "under its minimum rate" in log.status()["reason"]
    for second in range(107, 112):
        log.review(healthy(rate_hz=10.0), second * S)
    assert log.status()["keep_every"] == fl.MAX_KEEP_EVERY  # and no slower than one frame in sixteen
    # Ten seconds with nothing wrong: one step back; and so on up.
    for second in range(112, 121):
        log.review(healthy(), second * S)
    assert log.status()["keep_every"] == 16
    log.review(healthy(), 122 * S)
    assert log.status()["keep_every"] == 8
    log.review(healthy(), 133 * S)
    assert log.status()["keep_every"] == 4
    log.stop(wait_s=5.0)
    # In the sim the camera gives 10 frames a second: the detector's rate there is 10, and 9 is the line.
    sim = logging_to(tmp_path / "sim", expected_rate_hz=10.0)
    sim.start(100 * S)
    sim.review(healthy(rate_hz=9.8), 110 * S)
    assert sim.status()["keep_every"] == 1
    sim.stop(wait_s=5.0)


def test_a_logger_that_cannot_keep_up_with_itself_slows_too(tmp_path):
    made = []

    class Slow:
        """A frame log whose worker queue is full."""

        def __init__(self, folder, **settings):
            self.folder, self.settings = folder, settings
            self.written = self.skipped = self.dropped = 0
            self.full, self.error = False, None
            made.append(self)

        def write(self, frame):
            self.dropped += 1
            return False

        def close(self):
            self.closed = True

    (tmp_path / "logs").mkdir()
    log = fl.SurveyFrameLog(tmp_path / "logs", workers=3, make_log=Slow, min_free_bytes=0.0)
    log.start(100 * S)
    assert made[0].settings == {"image_format": "jpg", "jpeg_quality": 90, "max_bytes": None, "workers": 3, "queue_frames": 6, "worker_niceness": 10}
    for seq in range(1, 6):
        assert log.offer(frame(seq)) is False
    log.review(healthy(), 101 * S)  # the log's own queue is judged from the first second
    assert log.status()["keep_every"] == 2 and log.status()["dropped"] == 5 and "its own queue" in log.status()["reason"]
    log.stop(wait_s=5.0)
    assert made[0].closed


def test_a_write_that_fails_stops_the_logging_and_never_reaches_whoever_offered_the_frame(tmp_path):
    class Broken:
        written = skipped = dropped = 0
        full, error = False, None

        def __init__(self, folder, **settings):
            self.closed = False

        def write(self, frame):
            raise OSError("no space left on device")

        def close(self):
            self.closed = True

    (tmp_path / "logs").mkdir()
    log = fl.SurveyFrameLog(tmp_path / "logs", make_log=Broken, min_free_bytes=0.0)
    log.start(100 * S)
    assert log.offer(frame(1)) is False  # no exception: the camera's thread and the detector go on
    status = log.status()
    assert status["logging"] is False and "no space left on device" in status["error"] and log.offer(frame(2)) is False


def test_the_logging_stops_while_there_is_still_room_on_the_disk(tmp_path):
    free = [50.0e9]
    (tmp_path / "logs").mkdir()
    log = fl.SurveyFrameLog(tmp_path / "logs", workers=0, min_free_bytes=20.0e9, disk_free=lambda path: free[0])
    assert log.start(100 * S) is not None
    assert log.offer(frame(1)) is True
    free[0] = 19.0e9  # earlier surveys and this one have filled the disk to its last 19 GB
    log.review(healthy(), 105 * S)
    status = log.status()
    assert status["logging"] is False and "19.0 GB free" in status["error"] and log.offer(frame(2)) is False
    assert log.start(200 * S) is None and "19.0 GB free" in log.status()["error"]  # and the next survey does not start a log


def test_a_full_disk_or_a_folder_that_cannot_be_made_stops_the_logging_and_says_so(tmp_path):
    log = logging_to(tmp_path, max_bytes=1500)
    log.start(100 * S)
    taken = [log.offer(frame(seq)) for seq in range(1, 40)]
    assert taken[0] is True and taken[-1] is False and log.status()["full"] is True and 0 < log.status()["written"] < 39
    log.stop(wait_s=5.0)
    blocked = tmp_path / "a_file"
    blocked.write_text("not a folder")
    refused = fl.SurveyFrameLog(blocked)
    assert refused.start(100 * S) is None and refused.status()["logging"] is False and "is the disk mounted?" in refused.status()["error"]
    unmounted = fl.SurveyFrameLog(tmp_path / "mnt" / "nvme" / "frames")  # the mount point's folder is not there: nothing is made beside it
    assert unmounted.start(100 * S) is None and not (tmp_path / "mnt").exists()
    assert refused.offer(frame(1)) is False
    with pytest.raises(ValueError, match="frame_log_dir"):
        fl.SurveyFrameLog("")
    for bad in ({"jpeg_quality": 0}, {"workers": -1}, {"max_bytes": 0}, {"expected_rate_hz": 0.0}, {"min_free_bytes": -1.0}):
        with pytest.raises(ValueError):
            fl.SurveyFrameLog(tmp_path / "x", **bad)


def test_the_real_writer_with_workers_takes_every_frame_of_a_short_survey(tmp_path):
    (tmp_path / "logs").mkdir()
    log = fl.SurveyFrameLog(tmp_path / "logs", workers=2, min_free_bytes=0.0)
    folder = log.start(100 * S)
    import time

    assert folder is not None
    deadline = time.monotonic() + 20.0
    for seq in range(1, 31):
        while not log.offer(frame(seq, size=(320, 180))):  # at a camera's pace the queue has room; here we wait for it
            assert time.monotonic() < deadline, log.status()
            time.sleep(0.002)
    log.stop(wait_s=5.0)
    assert len(list(folder.glob("frame_*.jpg"))) == 30 and log.status()["written"] == 30
    assert isinstance(log._log, type(None)) and FrameLog is fl.FrameLog


def test_stopping_does_not_wait_for_the_disk_and_the_frames_queued_are_still_finished(tmp_path):
    import threading
    import time

    release = threading.Event()

    class Stalled:
        """A frame log whose close waits on a disk that has stopped answering."""

        written, skipped, dropped, full, error = 4, 0, 0, False, None

        def __init__(self, folder, **settings):
            pass

        def write(self, frame):
            return True

        def close(self):
            release.wait(10.0)
            type(self).written = 6

    (tmp_path / "logs").mkdir()
    log = fl.SurveyFrameLog(tmp_path / "logs", make_log=Stalled, min_free_bytes=0.0)
    log.start(100 * S)
    started = time.monotonic()
    log.stop()
    assert time.monotonic() - started < 0.5 and log.status()["logging"] is False  # the survey's stop is answered at once
    assert log.status()["written"] == 4  # what the log behind has written so far is still counted
    release.set()
    log.stop(wait_s=5.0)
    assert log.status()["written"] == 6


def test_the_encoding_threads_are_given_a_lower_priority(tmp_path, monkeypatch):
    import os

    asked = []
    monkeypatch.setattr(os, "setpriority", lambda which, who, niceness: asked.append((which, niceness)))
    (tmp_path / "logs").mkdir()
    log = fl.SurveyFrameLog(tmp_path / "logs", workers=2, min_free_bytes=0.0)
    log.start(100 * S)
    log.offer(frame(1))
    log.stop(wait_s=5.0)
    assert asked == [(os.PRIO_PROCESS, 10), (os.PRIO_PROCESS, 10)] and fl.WORKER_NICENESS == 10

    def refuse(which, who, niceness):
        raise PermissionError("not allowed to change priority")

    monkeypatch.setattr(os, "setpriority", refuse)
    log.start(200 * S)
    log.offer(frame(2))
    import time

    deadline = time.monotonic() + 5.0
    while log.status()["low_priority"] is not False and time.monotonic() < deadline:
        time.sleep(0.01)
    assert log.status()["low_priority"] is False  # said in the status: the encoding runs, at normal priority
    log.stop(wait_s=5.0)
    assert log.status()["low_priority"] is None
    monkeypatch.setattr(os, "setpriority", lambda which, who, niceness: asked.append((which, niceness)))
    # The writer by itself leaves its workers as they are: nothing changes for anyone else who uses it.
    asked.clear()
    plain = FrameLog(tmp_path / "plain", workers=1)
    plain.write(frame(1))
    plain.close()
    assert asked == []
