#!/usr/bin/env python3
"""Add ground-station commands and RC overrides from the MAVLink log to a run's trace.

  sim/run_sim.sh --exec env -u PYTHONNOUSERSITE python3 sim/runner/gcs_audit.py <run folder> [<tlog>]

MAVProxy logs every MAVLink message of the run, with the sender's system id
and a wall-clock time. A command from a ground station (system id 255) is a
person's input: the judge takes the autonomy bonus away when one falls inside
the flight. The clock marks in the trace turn each wall time into sim time.
Needs pymavlink (in the base image); the rules are events.human_inputs.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))

from judge.record import Record  # noqa: E402
from runner import events  # noqa: E402
from runner.trace import sim_time_at  # noqa: E402

DEFAULT_TLOG = SIM_DIR / ".state" / "logs" / "latest" / "mavlink.tlog"


def read_tlog(path: Path):
    """(wall time, system id, type, command id) for every message in a MAVLink log."""
    from pymavlink import mavutil

    log = mavutil.mavlink_connection(str(path))
    while True:
        message = log.recv_match()
        if message is None:
            return
        if message.get_type() == "BAD_DATA":
            continue
        yield (message._timestamp, message.get_srcSystem(), message.get_type(), getattr(message, "command", None))


def audit(run_dir: Path, messages) -> list[dict]:
    """Append the human inputs that fall inside the recorded span to the trace; return them."""
    trace = run_dir / "trace.jsonl"
    record = Record.load(trace)
    added = []
    for item in events.human_inputs(messages):
        t = sim_time_at(record, item["wall"])
        if t is not None:
            added.append({"t": round(t, 3), "event": item["event"], "what": item["what"]})
    with open(trace, "a") as handle:
        for line in added:
            handle.write(json.dumps(line) + "\n")
    return added


def main(argv: list[str]) -> int:
    if len(argv) not in (2, 3):
        print(__doc__.strip().splitlines()[2].strip(), file=sys.stderr)
        return 2
    run_dir = Path(argv[1])
    tlog = Path(argv[2]) if len(argv) == 3 else DEFAULT_TLOG
    if not tlog.is_file() or not (run_dir / "trace.jsonl").is_file():
        print(f"gcs_audit: need {tlog} and {run_dir / 'trace.jsonl'}", file=sys.stderr)
        return 2
    try:
        added = audit(run_dir, read_tlog(tlog))
    except ImportError:
        print("gcs_audit: pymavlink is not installed here (in the sim container, run with PYTHONNOUSERSITE unset)", file=sys.stderr)
        return 2
    (run_dir / "gcs_audit.json").write_text(json.dumps({"tlog": str(tlog), "human_inputs": added}, indent=2) + "\n")
    print(f"gcs_audit: {len(added)} ground-station command(s) or RC override(s) during the recording")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
