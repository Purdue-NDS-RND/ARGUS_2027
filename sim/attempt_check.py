#!/usr/bin/env python3
"""After a sim flight started by the GO command with an attempt slot (PLT-12): make the attempt's bundle and check it.

  python3 sim/attempt_check.py <run folder>

Run by the sim's runner once the flight has ended and the vehicle is
disarmed, while SITL is still up. It does what a person does after a
landing, with the same two tools:

  1. tools/dataflash_pull.py: the flight controller's newest log, over
     MAVLink, into <run folder>/dataflash.
  2. tools/bundle_attempt.py --from-run <run folder>: the attempt's folder,
     <run folder>/attempts/<date>_<slot>_<mission>/, with its manifest.

and then holds the result to what an attempt must be: complete for its
mission; every file as the manifest says; a bag that closed, holds the
vehicle's pose and state and no image; a pulled log that is, byte for byte,
the file SITL wrote (up to what SITL has added to it since). For the
geofence demonstration (OPS-02) tools/readiness_kit.py is run on the bundle
too, and must say READY. The verdict goes to <run folder>/attempt_check.json.

No ROS imports.
"""

from __future__ import annotations

import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional

SIM = Path(__file__).resolve().parent
REPO = SIM.parent
sys.path.insert(0, str(REPO / "tools"))

MIN_LOG_BYTES = 100_000  # a flight of minutes logs megabytes
MIN_LOG_SHARE = 0.95  # of SITL's file, which may have grown a little since the pull
NEEDED_TOPICS = ("/mavros/local_position/pose", "/mavros/state")  # a bag with none of these recorded nothing of the flight


def bag_problems(manifest: dict, counts: dict) -> List[str]:
    """What is wrong with the bundle's bag: `counts` is {topic: messages} from its metadata."""
    part = (manifest.get("parts") or {}).get("bag")
    if part is None:
        return ["the bundle has no bag"]
    problems = [f"the bag has no messages on {topic}" for topic in NEEDED_TOPICS if not counts.get(topic)]
    pictures = sorted(topic for topic, kind in part["topics"].items() if kind in ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"))
    if pictures:
        problems.append(f"the bag holds images: {', '.join(pictures)}")
    return problems


def message_counts(bag: Path) -> dict:
    import yaml

    info = (yaml.safe_load((bag / "metadata.yaml").read_text()) or {})["rosbag2_bagfile_information"]
    return {item["topic_metadata"]["name"]: int(item["message_count"]) for item in info["topics_with_message_count"]}


def judge(manifest: Optional[dict], verify_problems: List[str], counts: dict, pulled: Optional[Path], on_disk: Optional[Path], flown_profile: Optional[str] = None) -> dict:
    """The verdict on an attempt's bundle.

    `pulled` is the log pulled over MAVLink, `on_disk` the file SITL wrote, `flown_profile` the name of the
    profile the GO command was given.
    """
    problems: List[str] = []
    if manifest is None:
        return {"passed": False, "problems": ["no bundle was made"]}
    if not manifest.get("complete"):
        problems.append("the bundle is incomplete: " + ", ".join(f"{part} ({manifest['missing'].get(part, '')})" for part in manifest.get("owed") or []))
    problems += verify_problems
    problems += bag_problems(manifest, counts)
    if pulled is None or not pulled.is_file():
        problems.append("no dataflash log was pulled")
    elif on_disk is None or not on_disk.is_file():
        problems.append(f"SITL's own copy of log {pulled.name} was not found to compare the pulled one with")
    else:
        # SITL may add to its file after the size it gave for the pull (a disarmed vehicle still logs a little):
        # the pulled log must be the file's own bytes from the first to the last pulled, and nearly all of it.
        got, wrote = pulled.read_bytes(), on_disk.read_bytes()
        if len(got) < MIN_LOG_BYTES:
            problems.append(f"the pulled log {pulled.name} is {len(got)} bytes: not a flight's log")
        elif wrote[: len(got)] != got:
            problems.append(f"the pulled log {pulled.name} ({len(got)} bytes) is not what SITL wrote in its first {len(got)} bytes ({len(wrote)} in the file)")
        elif len(got) < MIN_LOG_SHARE * len(wrote):
            problems.append(f"the pulled log {pulled.name} is {len(got)} of the {len(wrote)} bytes SITL wrote: the flight's log was not pulled whole")
    if not (manifest.get("git") or {}).get("hash"):
        problems.append("the manifest has no git hash")
    if manifest.get("profile") is None:
        problems.append("the manifest has no flag profile")
    elif flown_profile is not None and manifest["profile"].get("name") != flown_profile:
        problems.append(f"the manifest's flag profile is {manifest['profile'].get('name')}, and the flight flew {flown_profile}")
    # A warning the bundler gave about the config (a profile, sequence or baseline that is not the flight's own) fails
    # the check; the one about the bag's storage does not: the sim's image has no MCAP plugin.
    problems += [warning for warning in manifest.get("warnings") or [] if "not MCAP" not in warning]
    return {"passed": not problems, "problems": problems}


def readiness_verdict(folder: Path) -> dict:
    """tools/readiness_kit.py on an attempt's bundle: readiness.md written into it, and {"ready", "problems"}."""
    import readiness_kit

    try:
        text, ready = readiness_kit.report(folder)
    except readiness_kit.KitError as problem:
        return {"ready": False, "problems": [f"the readiness kit could not read the bundle: {problem}"]}
    (folder / "readiness.md").write_text(text, encoding="utf-8")
    missing = text.split("## What is missing", 1)[1].split("## The video", 1)[0] if "## What is missing" in text else ""
    lines = [line[2:] for line in missing.splitlines() if line.startswith("- ")]
    return {"ready": ready, "problems": [] if ready else [f"the readiness kit says NOT READY: {line}" for line in lines] or ["the readiness kit says NOT READY"]}


def release_truth(run_dir: Path) -> bool:
    """Whether the sim's payload bridge let a carried payload go in this flight. No bridge: nothing was carried, so nothing left."""
    events = run_dir / "payload_bridge.jsonl"
    if not events.is_file():
        return False
    return any(json.loads(line).get("event") == "payload_released" for line in events.read_text().splitlines() if line.strip())


def release_verdict(folder: Path, left: bool) -> dict:
    """tools/post_flight.py on an attempt's bundle (CEN-14): its report written into it, and the release confirmation held against the sim's truth."""
    import post_flight

    try:
        text, data = post_flight.make_report(folder, set(post_flight.COMPANION))
    except post_flight.ReportError as problem:
        return {"verdict": None, "left": left, "problems": [f"the post-flight report could not be written: {problem}"]}
    found = data["audit"]["release_confirmation"]
    (folder / "report.md").write_text(text, encoding="utf-8")
    (folder / "report.json").write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    (folder / "release_confirmation.json").write_text(json.dumps(found, indent=2) + "\n", encoding="utf-8")
    problems = []
    if left and found["verdict"] != "CONFIRMED":
        problems.append(f"the sim let the payload go, and the report says RELEASE {found['verdict']}" + (": " + "; ".join(found.get("missing") or []) if found.get("missing") else ""))
    if not left and found["verdict"] == "CONFIRMED":
        problems.append(f"nothing left the vehicle in the sim, and the report says RELEASE CONFIRMED (by {', '.join(found.get('left_by') or [])})")
    return {"verdict": found["verdict"], "left": left, "left_by": found.get("left_by"), "lines": found.get("lines"), "problems": problems}


def main(argv: Optional[List[str]] = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__)
        return 2
    run_dir = Path(argv[0])
    scenario = json.loads((run_dir / "scenario.json").read_text())
    attempt = scenario["attempt"]
    import bundle_attempt
    import dataflash_pull

    report: dict = {"passed": False, "problems": [], "attempt": None}
    pulled = None
    try:
        link = dataflash_pull.connect(f"udpin:127.0.0.1:{attempt['port']}")
        pulled = dataflash_pull.pull_newest(link, run_dir / "dataflash")
        print(f"pulled {pulled.name}: {pulled.stat().st_size / 1e6:.2f} MB", flush=True)
    except (dataflash_pull.PullError, OSError, ImportError) as problem:
        report["problems"].append(f"the dataflash log could not be pulled: {problem}")
    date = datetime.now(timezone.utc).strftime("%Y-%m-%d")
    name = bundle_attempt.attempt_name(date, attempt["slot"], attempt["mission"])
    code = bundle_attempt.main(["--from-run", str(run_dir), "--slot", attempt["slot"], "--mission", attempt["mission"], "--date", date, "--out", str(run_dir / "attempts"),
                                "--note", f"sim flight of {scenario.get('name')}, seed {scenario.get('seed')}"])  # fmt: skip
    folder = run_dir / "attempts" / name
    manifest = json.loads((folder / "manifest.json").read_text()) if (folder / "manifest.json").is_file() else None
    counts = message_counts(folder / "bag") if (folder / "bag" / "metadata.yaml").is_file() else {}
    state = Path(os.environ.get("ARGUS_STATE", str(SIM / ".state")))
    on_disk = None if pulled is None else state / "sitl" / "logs" / pulled.name
    verdict = judge(manifest, bundle_attempt.verify(folder) if manifest else [], counts, None if manifest is None else folder / "dataflash" / pulled.name if pulled else None, on_disk,
                    flown_profile=Path(scenario["go"]["profile"]).stem)  # fmt: skip
    readiness = None
    if attempt["mission"] == "geofence" and manifest is not None:
        # The readiness video's demonstration (OPS-02): the kit a person runs on the bundle is run here, and its
        # READY, read from the flight controller's own log, is part of the run's pass.
        readiness = readiness_verdict(folder)
        verdict["problems"] += readiness["problems"]
        verdict["passed"] = verdict["passed"] and readiness["ready"]
    release = None
    if attempt["mission"] in ("drop", "delivery") and manifest is not None and pulled is not None:
        # The release confirmation (CEN-14): the report a person runs on the bundle is written here, and what it says
        # of the release must be what the sim's payload bridge did.
        release = release_verdict(folder, release_truth(run_dir))
        verdict["problems"] += release["problems"]
        verdict["passed"] = verdict["passed"] and not release["problems"]
        print(f"release: the sim {'let the payload go' if release['left'] else 'let nothing go'}; the report says {release['verdict']}", flush=True)
    report.update(passed=verdict["passed"] and not report["problems"], problems=report["problems"] + verdict["problems"], attempt=str(folder), bundle_exit_code=code, readiness=readiness, release=release,
                  dataflash_bytes=None if pulled is None else pulled.stat().st_size, bag_messages=sum(counts.values()), bag_topics=len(counts),
                  bag_storage=None if manifest is None else ((manifest.get("parts") or {}).get("bag") or {}).get("storage"), warnings=[] if manifest is None else manifest.get("warnings"))  # fmt: skip
    (run_dir / "attempt_check.json").write_text(json.dumps(report, indent=2) + "\n")
    print("ATTEMPT CHECK: PASS" if report["passed"] else "ATTEMPT CHECK: FAIL (" + "; ".join(report["problems"]) + ")", flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
