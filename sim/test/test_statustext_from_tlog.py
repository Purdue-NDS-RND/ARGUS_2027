"""PLT-11: the reader that shows which status texts reached the ground station's side (sim/statustext_from_tlog.py). pymavlink is a stand-in."""

import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import statustext_from_tlog as reader  # noqa: E402


class Message:
    def __init__(self, text, severity=4, system=1, component=191):
        self.text, self.severity, self._system, self._component = text, severity, system, component

    def get_srcSystem(self):
        return self._system

    def get_srcComponent(self):
        return self._component


def with_log(monkeypatch, messages):
    queue = list(messages)
    log = types.SimpleNamespace(recv_match=lambda type, blocking: queue.pop(0) if queue else None)

    def connect(path):
        if "missing" in path:
            raise FileNotFoundError(path)
        return log

    monkeypatch.setitem(sys.modules, "pymavlink", types.SimpleNamespace(mavutil=types.SimpleNamespace(mavlink_connection=connect)))


def test_the_lines_with_the_prefix_are_listed_with_who_sent_them_and_the_exit_code_says_whether_any_were(monkeypatch, capsys):
    with_log(monkeypatch, [Message("EKF3 IMU0 is using GPS", 6, 1, 1), Message("ARGUS H:SILENT ghost"), Message("ARGUS H:OK", 5)])
    assert reader.main(["flight.tlog", "--prefix", "ARGUS H"]) == 0
    assert capsys.readouterr().out.splitlines() == ["2 status text(s) starting with 'ARGUS H' in the telemetry log", "(1, 191, 4, 'ARGUS H:SILENT ghost')", "(1, 191, 5, 'ARGUS H:OK')"]
    with_log(monkeypatch, [Message("EKF3 IMU0 is using GPS", 6, 1, 1)])
    assert reader.main(["flight.tlog", "--prefix", "ARGUS H"]) == 1 and "0 status text(s)" in capsys.readouterr().out
    with_log(monkeypatch, [])
    assert reader.main(["missing.tlog"]) == 2 and "FileNotFoundError" in capsys.readouterr().err
