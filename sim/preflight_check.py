#!/usr/bin/env python3
"""Run the preflight before a sim flight and hold it to what the scenario says it must say (PLT-10).

  env -u PYTHONNOUSERSITE python3 sim/preflight_check.py <run folder>

Started by sim/runner/fly.py beside every flight whose scenario has a
`preflight:` block, at the moment the flight is launched, while the vehicle
is still on the ground. It runs the real command (drone_utils
preflight_node, which the `go` command of PLT-07 runs on the aircraft) with:

  the baseline       <run folder>/fc_baseline.param: platform/fcu/argus_2027.param
                     less the rangefinder's lines (SITL's rangefinder is its own)
  the parameters     read from SITL over MAVLink, on the preflight's own port
  the profile        <run folder>/profile.yaml: this run's flags
  the sequence       <run folder>/sequence.yaml, when a mission is flown
  the cameras        the scenario's `preflight.cameras`
  the log folder     the run folder

and writes preflight.json (the command's own record) and
preflight_check.json: whether the verdict, and the checks that failed, are
the scenario's. The sim's flight is launched whatever it says: this checks
the preflight, it does not gate the sim.

The comparison is a plain function with no ROS in it.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import List, Optional

SIM = Path(__file__).resolve().parent
REPO = SIM.parent


def compare(expected: dict, record: Optional[dict]) -> dict:
    """Whether a preflight's record is what the scenario expects: the verdict, and exactly the failed checks named."""
    report = {"passed": False, "problem": "", "expect": expected.get("expect"), "expected_failed": sorted(expected.get("failed") or []), "verdict": None, "failed": None}
    if record is None:
        report["problem"] = "the preflight left no record (preflight.json)"
        return report
    report["verdict"], report["failed"] = record.get("verdict"), sorted(record.get("failed") or [])
    details = {check["name"]: check["detail"] for check in record.get("checks") or []}
    if report["verdict"] != report["expect"]:
        why = "; ".join(f"{name}: {details.get(name, '')}" for name in report["failed"])
        report["problem"] = f"the preflight said {report['verdict']}, the scenario expects {report['expect']}" + (f" ({why})" if why else "")
    elif report["failed"] != report["expected_failed"]:
        report["problem"] = f"the preflight failed {', '.join(report['failed']) or 'nothing'}, the scenario expects {', '.join(report['expected_failed']) or 'nothing'}"
    else:
        report["passed"] = True
    return report


def arguments(manifest: dict, run_dir: Path) -> List[str]:
    """The preflight command's arguments for a run."""
    wanted = manifest["preflight"]
    args = ["--baseline", wanted["baseline"], "--fc-url", f"udpin:127.0.0.1:{wanted['port']}", "--log-dir", str(run_dir), "--profile", wanted["profile"],
            "--min-free-gb", "1", "--out", str(run_dir / "preflight.json")]  # fmt: skip
    sequence = run_dir / "sequence.yaml"
    # A flight flown by a tool has no sequence: the profile (no flags) is checked against a sequence that declares none.
    if not sequence.is_file():
        sequence = run_dir / "no_sequence.yaml"
        sequence.write_text("mission_sequence:\n  features: {}\n  missions: []\n")
    args += ["--sequence", str(sequence)]
    for topic, hz in sorted((wanted.get("cameras") or {}).items()):
        args += ["--camera", f"{topic}={hz:g}"]
    if not wanted.get("cameras"):
        args.append("--no-camera")  # a scenario that names none says the flight needs none
    if manifest.get("sim_time") is True:
        args.append("--sim-time")  # every node of a run on sim time is on /clock (sim/time_check.py holds the run to it)
    return args


def check_log(manifest: dict, run_dir: Path) -> dict:
    """A flight started by the GO command (PLT-07): the preflight it ran, from its own log, against what the scenario expects.

    Writes preflight_check.json as the side tool does. The record is the last line of the run's preflight.jsonl.
    """
    record = None
    try:
        lines = [line for line in (run_dir / "preflight.jsonl").read_text().splitlines() if line.strip()]
        record = json.loads(lines[-1]) if lines else None
    except (OSError, json.JSONDecodeError):
        record = None
    report = compare(manifest["preflight"], record)
    if record is None:
        report["problem"] = "the GO command's preflight left no record (preflight.jsonl)"
    (run_dir / "preflight_check.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def wanted(manifest: dict) -> bool:
    return bool(manifest.get("preflight"))


def read_preflight_check(folder) -> Optional[dict]:
    try:
        report = json.loads((Path(folder) / "preflight_check.json").read_text())
    except (OSError, json.JSONDecodeError):
        return None
    return report if isinstance(report, dict) and isinstance(report.get("passed"), bool) else None


def main(argv: Optional[List[str]] = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__)
        return 2
    if os.environ.get("PYTHONNOUSERSITE"):  # pymavlink is installed for the user in the sim image (as sim/inject.py does)
        environment = {key: value for key, value in os.environ.items() if key != "PYTHONNOUSERSITE"}
        os.execve(sys.executable, [sys.executable, os.path.abspath(__file__), *argv], environment)
    run_dir = Path(argv[0])
    manifest = json.loads((run_dir / "scenario.json").read_text())
    for path in (REPO / "src" / "drone_utils", REPO / "src" / "drone_mission_core"):
        sys.path.insert(0, str(path))
    from drone_utils import preflight_node

    code = preflight_node.main(arguments(manifest, run_dir))
    try:
        record = json.loads((run_dir / "preflight.json").read_text())
    except (OSError, json.JSONDecodeError):
        record = None
    report = compare(manifest["preflight"], record)
    report["exit_code"] = code
    (run_dir / "preflight_check.json").write_text(json.dumps(report, indent=2) + "\n")
    print("PREFLIGHT CHECK: PASS" if report["passed"] else f"PREFLIGHT CHECK: FAIL ({report['problem']})", flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
