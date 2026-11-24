"""PLT-12: pulling a dataflash log over MAVLink (tools/dataflash_pull.py), against a stand-in flight controller. No ROS, no pymavlink."""

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))

import dataflash_pull as dp  # noqa: E402
from dataflash_pull import PullError  # noqa: E402


class FakeController:
    """A flight controller's log server: answers LOG_REQUEST_LIST and LOG_REQUEST_DATA as ArduPilot does, 90 bytes a message.

    `drop` holds the numbers of the data messages it loses (counted over the whole pull); `silent_after` makes it
    stop answering after that many data messages.
    """

    target_system, target_component = 1, 1

    def __init__(self, logs, drop=(), silent_after=None):
        self.logs, self.drop, self.silent_after = logs, set(drop), silent_after
        self.queue, self.sent, self.requests, self.ended, self.now = [], 0, [], False, 0.0
        self.mav = SimpleNamespace(log_request_list_send=self._list, log_request_data_send=self._data, log_request_end_send=self._end)

    def clock(self):
        return self.now

    def _list(self, _system, _component, _start, _end):
        if not self.logs:
            self.queue.append(SimpleNamespace(get_type=lambda: "LOG_ENTRY", id=0, num_logs=0, last_log_num=0, size=0))
        for number, data in sorted(self.logs.items()):
            self.queue.append(SimpleNamespace(get_type=lambda: "LOG_ENTRY", id=number, num_logs=len(self.logs), last_log_num=max(self.logs), size=len(data)))

    def _data(self, _system, _component, log_id, offset, count):
        self.requests.append((offset, count))
        self.queue = [message for message in self.queue if message.get_type() != "LOG_DATA"]  # a new request replaces the stream
        data = self.logs[log_id]
        for start in range(offset, min(offset + count, len(data)), dp.PACKET):
            chunk = data[start : start + dp.PACKET]
            self.queue.append(SimpleNamespace(get_type=lambda: "LOG_DATA", id=log_id, ofs=start, count=len(chunk), data=list(chunk) + [0] * (dp.PACKET - len(chunk))))

    def _end(self, _system, _component):
        self.ended = True

    def recv_match(self, type=None, blocking=True, timeout=None):
        while self.queue:
            message = self.queue.pop(0)
            if message.get_type() != type:
                continue
            if type == "LOG_DATA":
                self.sent += 1
                if self.silent_after is not None and self.sent > self.silent_after:
                    self.queue.clear()
                    break
                if self.sent in self.drop:
                    continue
            return message
        self.now += timeout or 0.1  # nothing heard: time passes
        return None


def log_bytes(size, seed=7):
    return bytes((seed * (index + 1) * 2654435761 >> 7) & 0xFF for index in range(size))


def test_the_newest_log_is_pulled_whole_and_written_under_its_number(tmp_path):
    flight = log_bytes(dp.WINDOW * 2 + 1234)  # more than two windows, and an end that is not a whole packet
    controller = FakeController({41: log_bytes(500, seed=3), 42: flight})
    path = dp.pull_newest(controller, tmp_path / "dataflash", clock=controller.clock)
    assert path == tmp_path / "dataflash" / "00000042.BIN" and path.read_bytes() == flight
    assert controller.ended and controller.requests[0] == (0, dp.WINDOW) and len(controller.requests) == 3  # a window at a time, nothing asked twice


def test_packets_lost_on_the_way_are_asked_for_again(tmp_path):
    flight = log_bytes(dp.WINDOW + 5000)
    controller = FakeController({7: flight}, drop={3, 4, 200, 513, 560})
    seen = []
    path = dp.pull_newest(controller, tmp_path, clock=controller.clock, progress=lambda got, size: seen.append((got, size)))
    assert path.read_bytes() == flight
    assert seen[-1] == (len(flight), len(flight)) and all(a[0] < b[0] for a, b in zip(seen, seen[1:]))
    assert any(offset == 2 * dp.PACKET for offset, _count in controller.requests[1:])  # the first gap, from its packet's start


def test_a_log_can_be_named_and_one_that_is_not_there_is_said(tmp_path):
    controller = FakeController({41: log_bytes(500, seed=3), 42: log_bytes(900)})
    assert dp.pull_newest(controller, tmp_path, log_id=41, clock=controller.clock).read_bytes() == log_bytes(500, seed=3)
    with pytest.raises(PullError, match="no log 40 .it has 41, 42."):
        dp.pull_newest(controller, tmp_path, log_id=40, clock=controller.clock)
    with pytest.raises(PullError, match="exists already"):
        dp.pull_newest(controller, tmp_path, log_id=41, clock=controller.clock)


def test_a_flight_controller_that_stops_answering_fails_the_pull_and_writes_nothing(tmp_path):
    controller = FakeController({9: log_bytes(20000)}, silent_after=50)
    with pytest.raises(PullError, match=r"log 9: 4500 of 20000 bytes arrived and the flight controller stopped answering"):
        dp.pull_newest(controller, tmp_path / "out", clock=controller.clock)
    assert not (tmp_path / "out").exists()
    assert len(controller.requests) == 1 + dp.GIVE_UP_AFTER  # asked again, a bounded number of times
    assert all(offset == 50 * dp.PACKET for offset, _count in controller.requests[1:])  # each time from the first byte still missing


def test_no_logs_and_no_answer_are_each_said(tmp_path):
    with pytest.raises(PullError, match="holds no log"):
        dp.pull_newest(FakeController({}), tmp_path, clock=FakeController({}).clock)
    deaf = FakeController({1: b"x"})
    deaf.mav.log_request_list_send = lambda *args: None
    with pytest.raises(PullError, match="did not answer the request for its list"):
        dp.pull_newest(deaf, tmp_path, clock=deaf.clock)


def test_the_list_is_every_log_with_its_size():
    controller = FakeController({3: b"a" * 10, 4: b"b" * 2000})
    assert dp.list_logs(controller, 1, 1, clock=controller.clock) == {3: 10, 4: 2000}
