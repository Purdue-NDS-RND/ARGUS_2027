#!/usr/bin/env python3
"""Attempt bundle download (PLT-14): bring an attempt's folder from the aircraft to the ground laptop, and check it arrived whole.

  python3 tools/attempt_download.py pilot@aircraft --list
  python3 tools/attempt_download.py pilot@aircraft 2027-06-04_1_localisation
  python3 tools/attempt_download.py pilot@aircraft 2027-06-04_1_localisation --list-first

The aircraft's companion makes the attempt's folder after the landing
(tools/bundle_attempt.py). This copies it over ssh with rsync (a copy that
was cut off is taken up again where it stopped), then checks every file
against the bundle's own manifest, sizes and sha256 (bundle_attempt.verify):
`DOWNLOADED AND VERIFIED`, or what is missing or different.

`--list-first` copies `judges_list.csv` before anything else and says so:
the localisation list is due 30 minutes after the slot, and the frames
behind it are gigabytes. Then it goes on to the rest.

Nothing on the aircraft is changed or deleted. The host and the attempt's
name are checked before they go into a command.

Exit code: 0 downloaded and verified (or listed), 1 copied but not whole, 2 a wrong call or a copy that failed.

No ROS imports.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path
from typing import Callable, List, Optional

REPO = Path(__file__).resolve().parents[1]
ATTEMPTS = REPO / "attempts"
REMOTE_DIR = "ARGUS_2027/attempts"  # under the flight user's home on the aircraft
_HOST = re.compile(r"(?:[A-Za-z_][A-Za-z0-9_.-]*@)?[A-Za-z0-9][A-Za-z0-9_.-]*")
_ATTEMPT = re.compile(r"\d{4}-\d{2}-\d{2}_[a-z0-9_]+_[a-z_]+")  # bundle_attempt.attempt_name
_REMOTE = re.compile(r"[A-Za-z0-9_./~-]+")
LIST_FILE = "judges_list.csv"


class DownloadError(ValueError):
    """A wrong call. The message says why."""


def checked(host: str, attempt: Optional[str], remote_dir: str) -> None:
    if not _HOST.fullmatch(host):
        raise DownloadError(f"'{host}' is not a host: user@name or a name")
    if not _REMOTE.fullmatch(remote_dir) or ".." in remote_dir.split("/"):
        raise DownloadError(f"'{remote_dir}' is not a folder this tool will read from")
    if attempt is not None and not _ATTEMPT.fullmatch(attempt):
        raise DownloadError(f"'{attempt}' is not an attempt's name: <date>_<slot>_<mission>, as tools/bundle_attempt.py names them")


def list_command(host: str, remote_dir: str) -> List[str]:
    return ["ssh", "-o", "BatchMode=yes", host, f"ls -1 {remote_dir}"]


def copy_command(host: str, remote_dir: str, attempt: str, out: Path, only: Optional[str] = None) -> List[str]:
    """rsync of the attempt's folder (or one file of it) into out/<attempt>/. --partial: a cut-off copy goes on where it stopped."""
    source = f"{host}:{remote_dir}/{attempt}/" + (only or "")
    return ["rsync", "-a", "--partial", "--info=stats1", "-e", "ssh -o BatchMode=yes", source, str(out / attempt) + "/"]


def main(argv: Optional[List[str]] = None, run: Callable[..., "subprocess.CompletedProcess"] = subprocess.run) -> int:
    parser = argparse.ArgumentParser(description="Bring an attempt's bundle from the aircraft and check it arrived whole.")
    parser.add_argument("host", help="the aircraft's companion: user@name")
    parser.add_argument("attempt", nargs="?", help="the attempt's folder name")
    parser.add_argument("--list", action="store_true", help="list the attempts on the aircraft and copy nothing")
    parser.add_argument("--list-first", action="store_true", help="copy judges_list.csv before the rest")
    parser.add_argument("--remote-dir", default=REMOTE_DIR, help=f"where attempts are kept on the aircraft (default {REMOTE_DIR}, under the home folder)")
    parser.add_argument("--out", default=str(ATTEMPTS), help="where attempts are kept here (default: attempts/ in the repo)")
    args = parser.parse_args(argv)
    try:
        checked(args.host, args.attempt, args.remote_dir)
        if not args.list and args.attempt is None:
            raise DownloadError("name an attempt, or --list to see them")
    except DownloadError as problem:
        print(f"attempt_download: {problem}", file=sys.stderr)
        return 2
    if args.list:
        found = run(list_command(args.host, args.remote_dir), capture_output=True, text=True)
        if found.returncode != 0:
            print(f"attempt_download: could not list {args.host}:{args.remote_dir}: {(found.stderr or '').strip()}", file=sys.stderr)
            return 2
        names = sorted(name for name in found.stdout.split() if _ATTEMPT.fullmatch(name))
        print("\n".join(names) if names else f"no attempts in {args.host}:{args.remote_dir}")
        return 0
    out = Path(args.out)
    (out / args.attempt).mkdir(parents=True, exist_ok=True)
    if args.list_first:
        first = run(copy_command(args.host, args.remote_dir, args.attempt, out, only=LIST_FILE))
        if first.returncode == 0 and (out / args.attempt / LIST_FILE).is_file():
            print(f"THE LIST IS HERE: {out / args.attempt / LIST_FILE}  (read it: python3 tools/judges_list_view.py <it> --site <the day's site.yaml>)", flush=True)
        else:
            print(f"attempt_download: {LIST_FILE} could not be copied first (rsync exit {first.returncode}): going on with the whole bundle", file=sys.stderr, flush=True)
    copied = run(copy_command(args.host, args.remote_dir, args.attempt, out))
    if copied.returncode != 0:
        print(f"attempt_download: the copy stopped (rsync exit {copied.returncode}). Run the same command again: it goes on where it stopped", file=sys.stderr)
        return 2
    import bundle_attempt

    folder = out / args.attempt
    if not (folder / "manifest.json").is_file():
        print(f"NOT WHOLE: {folder} has no manifest.json: was the bundle made on the aircraft (tools/bundle_attempt.py)?")
        return 1
    problems = bundle_attempt.verify(folder)
    if problems:
        print("NOT WHOLE: " + "; ".join(problems))
        return 1
    print(f"DOWNLOADED AND VERIFIED: {folder}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
