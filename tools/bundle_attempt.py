#!/usr/bin/env python3
"""One folder per competition attempt (PLT-12): attempts/<date>_<slot>_<mission>/ with a manifest.

  python3 tools/bundle_attempt.py --slot 2 --mission delivery \\
      --bag <the flight's bag folder> --dataflash <the flight controller's .BIN> \\
      --frames <a frame log folder> [--frames <another>] \\
      --profile profiles/<name>.yaml --sequence <file> --params <file> \\
      --judges-list <judges_list.csv> --preflight-log <preflight.jsonl>
  python3 tools/bundle_attempt.py --from-run <a sim run folder> --slot 1 --mission delivery   # the same, from a sim flight

On the aircraft an attempt is three commands (the first two are one line each in the flight checklist):

  python3 tools/go.py ... --record <log folder>/<attempt>      # the flight; the bag is recorded beside it
  python3 tools/dataflash_pull.py --fc-url <link> --out <log folder>/<attempt>/dataflash     # after landing, disarmed
  python3 tools/bundle_attempt.py --slot 2 --mission delivery --from-run <log folder>/<attempt> --profile ... --sequence ... --params ...

What goes in, and where:

  manifest.json     what the attempt is, the git hash, the flag profile, every part with its size and sha256
  bag/              the flight's bag of key topics (MCAP, or sqlite3 where the MCAP plugin is missing). No images:
                    a bag with an image topic in it is refused
  dataflash/        the flight controller's log of the flight, pulled after landing (tools/dataflash_pull.py)
  frames/<name>/    the frame logs (argus_camera's frame log: JPEG files and index.jsonl)
  session/          the session logger's folder (command rows and image snapshots), when it was running
  config/           the sequence, the params file, the profile and the flight-controller baseline, as flown
  preflight.jsonl   the preflight's records of the flight (who overrode what)
  judges_list.csv   the judges' list, when the mission wrote one

With `go --record` the folder also holds go.json, the command's own record
of the profile, sequence and params it flew: `--from-run` takes them from
there, and a profile, sequence or baseline named here that is not the one the
flight's own records name is a WARNING in the manifest and on the screen.

A part that is not given, or is not there, is listed under "missing" in the
manifest and on the screen; the bundle is still made (exit code 1), because
after a flight a bundle with a hole in it is worth more than none. The parts
a mission is expected to have are in EXPECTED: the bag, the dataflash log, the
config and the preflight's log for every mission, and for the localisation
also the frames and the judges' list (the survey is the one mission that
writes a frame log today). Files are copied (`--move`
moves them: on the aircraft the frames are large and already on the same
disk). An attempt's folder is never overwritten. A bag whose recorder was not
stopped cleanly (no metadata.yaml) goes in as it is and is owed; a bag that is
not MCAP (no plugin where it was recorded) goes in with a warning.

No ROS imports. The bag is not opened: its metadata.yaml says what is in it.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

ROOT = Path(__file__).resolve().parents[1]
ATTEMPTS = ROOT / "attempts"
BASELINE = ROOT / "platform" / "fcu" / "argus_2027.param"
IMAGE_TYPES = ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage")
PARTS = ("bag", "dataflash", "frames", "session", "config", "preflight", "judges_list")
# What an attempt of each mission must hold to be complete. Frames and the judges' list are the localisation's:
# the survey is the one mission that writes a frame log (LOC-11) and a list (LOC-10). Other missions' frames
# and the session logger's folder go in when they are given.
_EVERY = ("bag", "dataflash", "config", "preflight")
EXPECTED = {
    "localisation": (*_EVERY, "frames", "judges_list"),
    "drop": _EVERY,
    "delivery": _EVERY,
    "recovery": _EVERY,
    "waypoint_navigation": _EVERY,
    "time_trial": _EVERY,
    "geofence": _EVERY,  # the readiness video's fence demonstration (OPS-02)
}
_NAME = re.compile(r"[a-z0-9][a-z0-9_]*")
_DATE = re.compile(r"\d{4}-\d{2}-\d{2}")


class BundleError(ValueError):
    """The attempt cannot be bundled as asked. The message says why."""


def attempt_name(date: str, slot: str, mission: str) -> str:
    """`<date>_<slot>_<mission>`: 2027-05-21_2_delivery."""
    if not _DATE.fullmatch(date):
        raise BundleError(f"the date is YYYY-MM-DD, got '{date}'")
    if not _NAME.fullmatch(str(slot)):
        raise BundleError(f"the slot is a number or a short name (lower case, digits, underscores), got '{slot}'")
    if mission not in EXPECTED:
        raise BundleError(f"the mission is one of {', '.join(EXPECTED)}, got '{mission}'")
    return f"{date}_{slot}_{mission}"


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def describe(folder: Path, root: Path) -> List[Dict[str, Any]]:
    """Every file under `folder`, with its size and sha256, by its path inside the bundle."""
    files = sorted(path for path in folder.rglob("*") if path.is_file()) if folder.is_dir() else [folder]
    return [{"file": str(path.relative_to(root)), "bytes": path.stat().st_size, "sha256": sha256_of(path)} for path in files]


def bag_topics(bag: Path) -> Dict[str, str]:
    """{topic: type} from a bag folder's metadata.yaml. Raises BundleError for a folder that is not a closed bag."""
    import yaml

    metadata = bag / "metadata.yaml"
    if not metadata.is_file():
        raise BundleError(f"{bag} has no metadata.yaml: not a bag, or the recorder was not stopped cleanly")
    try:
        info = (yaml.safe_load(metadata.read_text(encoding="utf-8")) or {})["rosbag2_bagfile_information"]
        return {item["topic_metadata"]["name"]: item["topic_metadata"]["type"] for item in info["topics_with_message_count"]}
    except (KeyError, TypeError, yaml.YAMLError) as problem:
        raise BundleError(f"{metadata} is not a bag's metadata: {problem}") from problem


def bag_storage(bag: Path) -> Optional[str]:
    """The bag's storage as its metadata names it: "mcap", or "sqlite3" where the MCAP plugin was not installed."""
    import yaml

    try:
        return (yaml.safe_load((bag / "metadata.yaml").read_text(encoding="utf-8")) or {})["rosbag2_bagfile_information"].get("storage_identifier")
    except (OSError, KeyError, TypeError, AttributeError, yaml.YAMLError):
        return None


def image_topics(topics: Dict[str, str]) -> List[str]:
    return sorted(topic for topic, kind in topics.items() if kind in IMAGE_TYPES)


def git_state(root: Path = ROOT) -> Dict[str, Any]:
    """The commit the code was at, and whether the tree had changes on top of it."""

    def git(*args: str) -> Optional[str]:
        try:
            done = subprocess.run(["git", "-C", str(root), *args], capture_output=True, text=True, timeout=30)
        except (OSError, subprocess.SubprocessError):
            return None
        return done.stdout.strip() if done.returncode == 0 else None

    status = git("status", "--porcelain", "--untracked-files=no")
    state = {"hash": git("rev-parse", "HEAD"), "branch": git("rev-parse", "--abbrev-ref", "HEAD"), "describe": git("describe", "--tags", "--always", "--dirty"),
             "dirty": None if status is None else bool(status)}  # fmt: skip
    if state["hash"] is None:
        # No git to ask (not installed, or it will not read a checkout owned by another user, as in a container):
        # the commit is in the checkout's own files. Whether the tree had changes on top is then not known.
        state["hash"], state["branch"] = head_from_files(root)
    return state


def head_from_files(root: Path) -> tuple:
    """(commit hash, branch) read from .git/HEAD and the ref it names; (None, None) when they cannot be read."""
    try:
        head = (root / ".git" / "HEAD").read_text(encoding="utf-8").strip()
        if not head.startswith("ref: "):
            return (head or None), None
        ref = head[5:]
        branch = ref[len("refs/heads/") :] if ref.startswith("refs/heads/") else ref
        loose = root / ".git" / ref
        if loose.is_file():
            return loose.read_text(encoding="utf-8").strip(), branch
        for line in (root / ".git" / "packed-refs").read_text(encoding="utf-8").splitlines():
            if line.endswith(" " + ref):
                return line.split(" ", 1)[0], branch
    except OSError:
        pass
    return None, None


def profile_summary(path: Optional[Path], name: Optional[str] = None) -> Optional[Dict[str, Any]]:
    """The flag profile as flown: its name (the file's, in profiles/), its flags and node flags, its sha256."""
    if path is None or not path.is_file():
        return None
    import yaml

    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return {"name": name or path.stem, "flags": list(data.get("enable") or []), "node_flags": {node: list(flags or []) for node, flags in (data.get("nodes") or {}).items()},
            "sha256": sha256_of(path)}  # fmt: skip


def _put(source: Path, target: Path, move: bool) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    if move:
        shutil.move(str(source), str(target))
    elif source.is_dir():
        shutil.copytree(source, target)
    else:
        shutil.copy2(source, target)


def bundle(name: str, out_dir: Path, mission: str, bag: Optional[Path] = None, dataflash: Optional[Path] = None, frames: Optional[List[Path]] = None,
           profile: Optional[Path] = None, sequence: Optional[Path] = None, params: Optional[Path] = None, baseline: Optional[Path] = BASELINE,
           preflight_log: Optional[Path] = None, judges_list: Optional[Path] = None, session: Optional[Path] = None, move: bool = False, git: Optional[Dict[str, Any]] = None,
           now: Optional[datetime] = None, notes: Optional[List[str]] = None, flown: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:  # fmt: skip
    """Make the attempt's folder and its manifest; the manifest.

    Raises BundleError before anything is written when the folder exists already or the bag holds images.
    A part that was not given or is not there is recorded as missing.
    """
    folder = out_dir / name
    if folder.exists():
        raise BundleError(f"{folder} exists already: an attempt's folder is never overwritten (another slot name, or move the old one away)")
    missing: Dict[str, str] = {}
    topics: Dict[str, str] = {}
    storage, unclosed = None, None
    if bag is not None and bag.is_dir():
        try:
            topics, storage = bag_topics(bag), bag_storage(bag)
        except BundleError as problem:
            # A recorder that was killed, or power lost before it closed: what it wrote still goes in, and is said.
            unclosed = f"{problem} (the files are in the bundle; `ros2 bag reindex` may recover them)"
        pictures = image_topics(topics)
        if pictures:
            raise BundleError(f"the bag {bag} holds images ({', '.join(pictures)}): the attempt's bag is of key topics only, and frames go in as a frame log")
    frames = [Path(item) for item in frames or []]
    if len({item.name for item in frames}) != len(frames):
        raise BundleError("two frame logs have the same folder name: they would land on each other in frames/")

    folder.mkdir(parents=True)
    parts: Dict[str, Any] = {}

    def take(part: str, source: Optional[Path], target: str, wanted_dir: Optional[bool] = None) -> bool:
        if source is None:
            missing[part] = "not given"
            return False
        if not source.exists() or (wanted_dir is True and not source.is_dir()) or (wanted_dir is False and not source.is_file()):
            missing[part] = f"{source} is not there"
            return False
        _put(source, folder / target, move)
        return True

    if take("bag", bag, "bag", wanted_dir=True):
        parts["bag"] = {"files": describe(folder / "bag", folder), "topics": dict(sorted(topics.items())), "storage": storage, "closed": unclosed is None}
        if unclosed is not None:
            missing["bag"] = unclosed
    if take("dataflash", dataflash, f"dataflash/{dataflash.name if dataflash else ''}", wanted_dir=False):
        parts["dataflash"] = {"files": describe(folder / "dataflash", folder)}
    logs = []
    for item in frames:
        index = item / "index.jsonl"
        if not index.is_file():
            missing.setdefault("frames", f"{item} has no index.jsonl: not a frame log")
            continue
        count = sum(1 for line in index.read_text(encoding="utf-8").splitlines() if line.strip())
        _put(item, folder / "frames" / item.name, move)
        # The frames themselves are not hashed one by one (thousands of files): the index is, and it names every frame with its size.
        logs.append({"folder": f"frames/{item.name}", "frames": count, "index_sha256": sha256_of(folder / "frames" / item.name / "index.jsonl"),
                     "bytes": sum(path.stat().st_size for path in (folder / "frames" / item.name).rglob("*") if path.is_file())})  # fmt: skip
    if logs:
        parts["frames"] = {"logs": logs}
    elif "frames" not in missing:
        missing["frames"] = "not given"
    if take("session", session, "session", wanted_dir=True):
        files = [path for path in (folder / "session").rglob("*") if path.is_file()]
        parts["session"] = {"count": len(files), "bytes": sum(path.stat().st_size for path in files)}
    config = [(sequence, "sequence.yaml"), (params, "params.yaml"), (profile, "profile.yaml"), (baseline, "fc_baseline.param")]
    absent = [target for source, target in config if source is None or not Path(source).is_file()]
    for source, target in config:
        if source is not None and Path(source).is_file():
            _put(Path(source), folder / "config" / target, False)  # config is always copied: the files are the repo's
    if (folder / "config").is_dir():
        parts["config"] = {"files": describe(folder / "config", folder)}
    if absent:
        missing["config"] = "without " + ", ".join(absent)
    if take("preflight", preflight_log, "preflight.jsonl", wanted_dir=False):
        records = [json.loads(line) for line in (folder / "preflight.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
        last = records[-1] if records else {}
        parts["preflight"] = {"files": describe(folder / "preflight.jsonl", folder), "records": len(records), "verdict": last.get("verdict"), "failed": last.get("failed"),
                              "override": last.get("override"), "profile": last.get("profile"), "sequence": last.get("sequence"), "baseline_sha256": last.get("baseline_sha256")}  # fmt: skip
    if take("judges_list", judges_list, "judges_list.csv", wanted_dir=False):
        parts["judges_list"] = {"files": describe(folder / "judges_list.csv", folder)}

    # What the flight itself said it flew (the GO command's record, the preflight's last line) against what was
    # bundled as its config: a profile or a baseline named again after the flight that is not the one flown is said.
    warnings: List[str] = []
    if storage is not None and storage != "mcap":
        warnings.append(f"the bag is {storage}, not MCAP: the recorder found no MCAP plugin where it ran")
    flown = dict(flown or {})
    last = (parts.get("preflight") or {})
    said_profile = flown.get("profile") or last.get("profile")
    if said_profile and profile is not None and Path(said_profile).name != Path(profile).name:
        warnings.append(f"the profile bundled is {Path(profile).name}; the flight's own record names {Path(said_profile).name}")
    said_sequence = flown.get("sequence") or last.get("sequence")
    if said_sequence and sequence is not None and Path(said_sequence).name != Path(sequence).name:
        warnings.append(f"the sequence bundled is {Path(sequence).name}; the flight's own record names {Path(said_sequence).name}")
    bundled_baseline = folder / "config" / "fc_baseline.param"
    if last.get("baseline_sha256") and bundled_baseline.is_file() and sha256_of(bundled_baseline) != last["baseline_sha256"]:
        warnings.append("the flight-controller baseline bundled is not the one the preflight held the aircraft to")
    owed = [part for part in EXPECTED[mission] if part in missing]
    date, slot, _mission = name.split("_", 2)
    manifest = {
        "attempt": name, "date": date, "slot": slot, "mission": mission,
        "created_utc": (now or datetime.now(timezone.utc)).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "git": git if git is not None else git_state(),
        "profile": profile_summary(folder / "config" / "profile.yaml", name=None if profile is None else Path(profile).stem),
        "parts": parts,
        "missing": {part: missing[part] for part in PARTS if part in missing},
        "complete": not owed,  # every part this mission is expected to have is there
        "owed": owed,
        "flown": flown or None,  # the GO command's own record of the flight (go.json), when the folder had one
        "warnings": warnings,
        "notes": list(notes or []),
    }  # fmt: skip
    (folder / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def verify(folder: Path) -> List[str]:
    """What in an attempt's folder is not what its manifest says: a file gone, a size or a sha256 that differs. Empty when it holds."""
    try:
        manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as problem:
        return [f"no readable manifest.json: {problem}"]
    problems = []
    for part, content in (manifest.get("parts") or {}).items():
        for item in content.get("files") or []:
            path = folder / item["file"]
            if not path.is_file():
                problems.append(f"{part}: {item['file']} is gone")
            elif path.stat().st_size != item["bytes"] or sha256_of(path) != item["sha256"]:
                problems.append(f"{part}: {item['file']} is not the file that was bundled")
        for log in content.get("logs") or []:
            index = folder / log["folder"] / "index.jsonl"
            if not index.is_file() or sha256_of(index) != log["index_sha256"]:
                problems.append(f"{part}: {log['folder']}/index.jsonl is not the index that was bundled")
    return problems


def from_run(run_dir: Path) -> Dict[str, Any]:
    """Where a flight's log folder keeps what an attempt holds: the arguments of `bundle`.

    The folder is a sim run's (sim/run_scenarios.sh) or the one given to `go --record` and to the preflight
    as its log folder: bag/, dataflash/*.BIN, preflight.jsonl, frames/<name>/, and in a sim run the config too.
    """
    scenario = run_dir / "scenario.json"  # a sim run has one; a flight's own log folder (go --record) does not
    manifest = json.loads(scenario.read_text(encoding="utf-8")) if scenario.is_file() else {}
    found: Dict[str, Any] = {"bag": run_dir / "bag", "preflight_log": run_dir / "preflight.jsonl"}
    for key, name in (("sequence", "sequence.yaml"), ("params", "params.yaml")):
        if (run_dir / name).is_file():
            found[key] = run_dir / name
    # What the GO command wrote at GO (tools/go.py --record): the profile, sequence and params it flew.
    flown = {}
    if (run_dir / "go.json").is_file():
        flown = json.loads((run_dir / "go.json").read_text(encoding="utf-8"))
        found["flown"] = flown
        for key in ("sequence", "params"):
            if key not in found and flown.get(key) and Path(flown[key]).is_file():
                found[key] = Path(flown[key])
    profile = flown.get("profile") or (manifest.get("preflight") or {}).get("profile")
    found["profile"] = Path(profile) if profile else None
    if (run_dir / "fc_baseline.param").is_file():
        found["baseline"] = run_dir / "fc_baseline.param"
    found["judges_list"] = Path(manifest["list_file"]) if manifest.get("list_file") else None
    found["frames"] = sorted(path.parent for path in run_dir.glob("frames/*/index.jsonl"))
    logs = sorted((run_dir / "dataflash").glob("*.BIN")) if (run_dir / "dataflash").is_dir() else []
    found["dataflash"] = logs[-1] if logs else None
    return found


def report(manifest: Dict[str, Any], folder: Path) -> List[str]:
    lines = [f"attempt {manifest['attempt']}: {folder}", f"git {manifest['git'].get('describe')}" + (" (with uncommitted changes)" if manifest["git"].get("dirty") else "")]
    profile = manifest.get("profile")
    lines.append("profile: " + ("none in the bundle" if profile is None else f"{profile['name']} ({', '.join(profile['flags']) or 'no flags: the 2026 code'})"))
    for part in PARTS:
        if part in manifest["parts"]:
            content = manifest["parts"][part]
            size = sum(item["bytes"] for item in content.get("files") or []) + sum(log["bytes"] for log in content.get("logs") or []) + content.get("bytes", 0)
            lines.append(f"[   in] {part:<12} {size / 1e6:.1f} MB")
        else:
            lines.append(f"[{'OWED' if part in manifest['owed'] else '   -'}] {part:<12} {manifest['missing'].get(part, '')}")
    lines += [f"WARNING: {warning}" for warning in manifest.get("warnings") or []]
    lines.append("COMPLETE" if manifest["complete"] else "INCOMPLETE: " + ", ".join(manifest["owed"]))
    return lines


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Assemble one attempt's folder with its manifest.")
    parser.add_argument("--slot", required=True, help="the competition slot (1, 2, 3) or a short name for a practice flight")
    parser.add_argument("--mission", required=True, choices=sorted(EXPECTED))
    parser.add_argument("--date", default=None, help="YYYY-MM-DD; today by this computer's clock when not given")
    parser.add_argument("--out", default=str(ATTEMPTS), help="where attempts are kept (default: attempts/ in the repo, which git ignores)")
    parser.add_argument("--from-run", help="the flight's log folder (go --record's, or a sim run's): take what it holds, for anything not given here")
    parser.add_argument("--bag")
    parser.add_argument("--dataflash")
    parser.add_argument("--frames", action="append", default=[], help="a frame log folder; repeat for more")
    parser.add_argument("--profile")
    parser.add_argument("--sequence")
    parser.add_argument("--params")
    parser.add_argument("--baseline", default=str(BASELINE))
    parser.add_argument("--preflight-log")
    parser.add_argument("--judges-list")
    parser.add_argument("--session-log", help="the session logger's folder of this flight")
    parser.add_argument("--note", action="append", default=[], help="a line for the manifest (what happened); repeat for more")
    parser.add_argument("--move", action="store_true", help="move the bag, the dataflash log and the frames instead of copying them")
    parser.add_argument("--verify", metavar="FOLDER", help="check an attempt's folder against its manifest and do nothing else")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    if argv is None:
        argv = sys.argv[1:]
    if argv[:1] == ["--verify"] and len(argv) == 2:
        problems = verify(Path(argv[1]))
        print("\n".join(problems) if problems else f"{argv[1]}: as its manifest says")
        return 1 if problems else 0
    args = build_parser().parse_args(argv)
    try:
        date = args.date or datetime.now().strftime("%Y-%m-%d")  # the day at the field, by this computer's clock
        name = attempt_name(date, args.slot, args.mission)
        given: Dict[str, Any] = from_run(Path(args.from_run)) if args.from_run else {}
        for key, value in (("bag", args.bag), ("dataflash", args.dataflash), ("profile", args.profile), ("sequence", args.sequence), ("params", args.params),
                           ("preflight_log", args.preflight_log), ("judges_list", args.judges_list), ("session", args.session_log)):  # fmt: skip
            if value is not None:
                given[key] = Path(value)
        if args.frames:
            given["frames"] = [Path(item) for item in args.frames]
        given.setdefault("baseline", Path(args.baseline))
        manifest = bundle(name, Path(args.out), args.mission, move=args.move, notes=args.note, **given)
    except (BundleError, OSError, json.JSONDecodeError) as problem:
        print(f"bundle_attempt: {problem}", file=sys.stderr)
        return 2
    print("\n".join(report(manifest, Path(args.out) / name)))
    return 0 if manifest["complete"] else 1


if __name__ == "__main__":
    sys.exit(main())
