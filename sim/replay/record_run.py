#!/usr/bin/env python3
"""Record a sim flight (SIM-09): the runner's side tool. Runs inside the sim container.

  sim/run_scenarios.sh <scenario> --record        (starts this beside the flight)
  python3 sim/replay/record_run.py <run folder>

Starts `ros2 bag record` on the recorder's topic list (topics.py) and the
taps (taps.py), writing <run folder>/bag, and stops both when it is
interrupted at the end of the flight. A run on sim time is recorded on sim
time, so the bag's times are on the same clock as the stamps in it; a run
on wall time gets the sim-clock tap instead.

MCAP is used when the rosbag2 MCAP storage plugin is installed, sqlite3
otherwise: the replay reads both.

What to run is decided by plain functions with no ROS in them.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional

SIM_DIR = Path(__file__).resolve().parents[1]
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))

from replay import topics  # noqa: E402

MCAP, SQLITE3 = "mcap", "sqlite3"
# rosbag2's MCAP plugin compresses chunks with zstd unless told otherwise; this preset writes plain records,
# which the replay reads with the standard library alone.
MCAP_PRESET = "fastwrite"


def wanted(manifest: dict) -> bool:
    """Whether a built scenario asked for a recording (`sim/run_scenarios.sh --record`)."""
    return manifest.get("record") is True


def choose_storage(mcap_installed: bool) -> str:
    return MCAP if mcap_installed else SQLITE3


def record_command(out: Path, storage: str, sim_time: bool, tapped_images=topics.TAPPED_IMAGES) -> List[str]:
    """The `ros2 bag record` command for a bag at `out`."""
    command = ["ros2", "bag", "record", "--storage", storage, "--output", str(out)]
    if storage == MCAP:
        command += ["--storage-preset-profile", MCAP_PRESET]
    if sim_time:
        command.append("--use-sim-time")
    return command + topics.record_topics(tapped_images)


def taps_command(sim_time: bool, tapped_images=topics.TAPPED_IMAGES) -> List[str]:
    """The taps beside it: the sim clock is tapped when the nodes are not on it."""
    command = [sys.executable, str(SIM_DIR / "replay" / "taps.py")]
    for topic in tapped_images:
        command += ["--image", topic]
    return command + (["--use-sim-time"] if sim_time else ["--sim-clock"])


def mcap_installed() -> bool:
    try:
        return subprocess.run(["ros2", "pkg", "prefix", "rosbag2_storage_mcap"], capture_output=True, timeout=30).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def stop(process: Optional[subprocess.Popen], grace_s: float = 8.0) -> None:
    """Interrupt a process and wait: `ros2 bag record` closes its file on SIGINT."""
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(grace_s)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def main(argv: Optional[List[str]] = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__)
        return 2
    run_dir = Path(argv[0])
    manifest = json.loads((run_dir / "scenario.json").read_text())
    sim_time = manifest.get("sim_time") is True
    storage = choose_storage(mcap_installed())
    out = run_dir / "bag"
    print(f"recording to {out} ({storage}, on {'sim' if sim_time else 'wall'} time)", flush=True)
    stopping = []
    signal.signal(signal.SIGINT, lambda *_: stopping.append(True))
    signal.signal(signal.SIGTERM, lambda *_: stopping.append(True))
    taps = subprocess.Popen(taps_command(sim_time), start_new_session=True)
    recorder = subprocess.Popen(record_command(out, storage, sim_time), start_new_session=True)
    try:
        while not stopping and recorder.poll() is None:
            time.sleep(0.2)
    finally:
        stop(recorder)
        stop(taps)
    code = recorder.returncode
    print(f"recorder ended ({code}); bag: {sorted(path.name for path in out.glob('*')) if out.is_dir() else 'none'}", flush=True)
    return 0 if out.is_dir() else 1


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
