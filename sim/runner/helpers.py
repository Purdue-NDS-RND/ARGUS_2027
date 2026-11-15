"""The runner's side jobs during a flight: sim tools to run beside it, and the judges' list. No ROS."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
from pathlib import Path


def stop_group(process: subprocess.Popen, grace_s: float = 15.0) -> int | None:
    """Interrupt a process and everything it started; kill it if it will not stop."""
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=grace_s)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5.0)
        except ProcessLookupError:
            pass
    return process.poll()


class SideTools:
    """Sim tools that run beside a flight: wind (SIM-03), GPS events (SIM-04).

    Each is a script started with the run folder as its only argument, with
    its output in <run folder>/<name>.log, and interrupted when the flight ends.
    """

    def __init__(self, scripts: dict[str, Path], run_dir: Path) -> None:
        self.scripts = scripts
        self.run_dir = run_dir
        self.running: dict[str, subprocess.Popen] = {}
        self._logs = []

    def start(self, names) -> None:
        for name in sorted(names):
            log = open(self.run_dir / f"{name}.log", "w")
            self._logs.append(log)
            self.running[name] = subprocess.Popen(
                [sys.executable, str(self.scripts[name]), str(self.run_dir)],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )

    def stop(self, grace_s: float = 5.0) -> dict[str, int | None]:
        codes = {name: stop_group(process, grace_s) for name, process in self.running.items()}
        for log in self._logs:
            log.close()
        self.running, self._logs = {}, []
        return codes


class ListCapture:
    """Watch the file a localisation mission writes its judges' list to.

    Every rewrite is a `list_written` event. While the vehicle is flying the
    file is copied to <run folder>/judges_list.csv, so the judge scores the
    list as it stood when the flight ended: a rewrite after the disarm is
    noted but not copied. A file already there when the run starts is an
    earlier run's and is ignored until it changes.
    """

    def __init__(self, list_file: Path | None, run_dir: Path) -> None:
        self.list_file = list_file
        self.copy = run_dir / "judges_list.csv"
        self._seen = self._mtime()

    def _mtime(self) -> float | None:
        if self.list_file is None:
            return None
        try:
            return self.list_file.stat().st_mtime_ns
        except OSError:
            return None

    def poll(self, flying: bool) -> bool:
        """True if the list was written since the last poll."""
        mtime = self._mtime()
        if mtime is None or mtime == self._seen:
            return False
        self._seen = mtime
        if flying:
            self.copy.write_bytes(self.list_file.read_bytes())
        return True
