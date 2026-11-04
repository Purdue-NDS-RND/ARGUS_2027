#!/usr/bin/env python3
"""PreToolUse hook for the flight-guardian subagent: keep it out of docs/guardian-calibration.md.

The calibration file holds replay cases with their expected findings. A reviewer that
has read them is being tested on what it memorised, so the guardian must not see them.

Blocks (exit 2, reason on stderr):
  - Read of the calibration file
  - Grep in content mode whose search scope includes the file, unless a glob or type
    filter leaves it out
Everything else passes. Fails open: any error exits 0.
"""

import fnmatch
import json
import os
import sys
from pathlib import Path

CALIBRATION = Path("docs/guardian-calibration.md")
REASON = (
    "docs/guardian-calibration.md is off limits to the flight-guardian. "
    "Narrow the search to code (for example path=src, or glob='*.py') and carry on."
)


def resolve(raw, cwd):
    path = Path(raw).expanduser()
    return (path if path.is_absolute() else Path(cwd) / path).resolve()


def blocked(event):
    tool = event.get("tool_name")
    args = event.get("tool_input") or {}
    cwd = event.get("cwd") or os.getcwd()
    root = Path(os.environ.get("CLAUDE_PROJECT_DIR") or cwd).resolve()
    target = (root / CALIBRATION).resolve()

    if tool == "Read":
        return resolve(args.get("file_path", ""), cwd) == target

    if tool == "Grep":
        if args.get("output_mode") != "content":
            return False  # file names and counts only, not the cases themselves
        scope = resolve(args.get("path") or cwd, cwd)
        if scope != target and scope not in target.parents:
            return False
        glob = args.get("glob")
        if glob and not (fnmatch.fnmatch(target.name, glob) or fnmatch.fnmatch(str(CALIBRATION), glob)):
            return False
        kind = args.get("type")
        if kind and kind not in ("md", "markdown"):
            return False
        return True

    return False


def main():
    try:
        event = json.load(sys.stdin)
        if blocked(event):
            print(REASON, file=sys.stderr)
            return 2
    except Exception as exc:  # noqa: BLE001
        print(f"guardian-read-guard: error, not blocking: {exc}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
