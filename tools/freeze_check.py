#!/usr/bin/env python3
"""The code freeze (OPS-06): what has changed since the release candidate, and whether the freeze allows it.

  python3 tools/freeze_check.py                       # since the tag comp-2027-rc1, up to what is checked out
  python3 tools/freeze_check.py --since comp-2027-rc1 --to HEAD
  python3 tools/freeze_check.py --since <tag> --worktree    # also what is edited and not yet committed

The freeze is a tag, `comp-2027-rc1`, set around 19 May 2027 on the commit
that passed the gates. After it only these may change:

  tuning   a tuned value in a config YAML (`src/<package>/config/**.yaml`): the files the pull-request gate
           asks a `[tuning: <reason>]` waiver for. Each changed value is listed, old and new, so the field's
           tuning is a list that can be read back and undone. The reason is asked for here too, before the
           flight and with no network: the change is committed, and its commit message carries
           `[tuning: <reason>]`. A tuned value with no such commit, or edited and not committed, breaks the
           freeze. The pull request that follows takes the reason from the commit.
  field    the day's own files: a flag profile (`profiles/*.yaml`), a day file (`field/*.yaml`), the scoring
           model's inputs (`tools/score_model.yaml`).
  docs     anything under `docs/`, and any `.md`.

Anything else is code, and breaks the freeze: the tool lists it and exits
with 1. A tuning file that was added, removed or no longer parses is said
too. With no such tag yet it exits with 2 and says how the tag is made.

To undo the field's tuning: `git checkout comp-2027-rc1 -- <file>` for a
file in the list, or fly the release candidate itself from a clean
checkout. To leave 2027 altogether: platform/jetson/fallback.sh.

No ROS imports. git is asked for the diff.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Tuple

ROOT = Path(__file__).resolve().parents[1]
FREEZE_TAG = "comp-2027-rc1"
TUNING_FILE = re.compile(r"^src/[^/]+/config/.+\.ya?ml$")  # scripts/pr_checks.py's own rule for a tuning file (a test holds the two together)
FIELD_FILE = re.compile(r"^(profiles/[^/]+\.ya?ml|field/[^/]+\.ya?ml|tools/score_model\.yaml)$")
DOCS_FILE = re.compile(r"^(docs/.+|.+\.md)$")
KINDS = ("code", "tuning", "field", "docs")
REASON = re.compile(r"\[tuning:\s*([^\]]*\S)\s*\]", re.IGNORECASE)  # the waiver's own form (scripts/pr_checks.py), in a commit message


class FreezeError(RuntimeError):
    """The check could not be made. The message says why."""


def kind_of(path: str) -> str:
    if TUNING_FILE.match(path):
        return "tuning"
    if FIELD_FILE.match(path):
        return "field"
    if DOCS_FILE.match(path):
        return "docs"
    return "code"


def _git(repo: Path, *args: str, check: bool = True) -> Optional[str]:
    try:
        done = subprocess.run(["git", "-C", str(repo), *args], capture_output=True, text=True, timeout=120)
    except (OSError, subprocess.SubprocessError) as problem:
        raise FreezeError(f"git could not be run: {problem}") from problem
    if done.returncode != 0:
        if check:
            raise FreezeError(f"git {' '.join(args)}: {done.stderr.strip() or 'failed'}")
        return None
    return done.stdout


def changed_files(repo: Path, since: str, to: str, worktree: bool) -> List[Tuple[str, str]]:
    """(status letter, path) of every file that differs between `since` and `to` (and the working tree, when asked)."""
    if _git(repo, "rev-parse", "--verify", "--quiet", f"{since}^{{commit}}", check=False) is None:
        raise FreezeError(f"NO_TAG {since}")
    spans = [[since, to]] if not worktree else [[since]]  # `git diff <since>` alone compares with the working tree
    found: Dict[str, str] = {}
    for span in spans:
        for line in (_git(repo, "diff", "--name-status", "--no-renames", *span) or "").splitlines():
            status, _tab, path = line.partition("\t")
            found[path] = status[:1]
    if worktree:
        for path in (_git(repo, "ls-files", "--others", "--exclude-standard") or "").splitlines():
            found.setdefault(path, "A")
    return sorted((status, path) for path, status in found.items())


def _leaves(node: Any, prefix: str = "") -> Iterator[Tuple[str, Any]]:
    if isinstance(node, dict):
        for key, value in node.items():
            yield from _leaves(value, f"{prefix}.{key}" if prefix else str(key))
    elif isinstance(node, list):
        for index, value in enumerate(node):
            yield from _leaves(value, f"{prefix}[{index}]")
    else:
        yield prefix, node


def tuned_values(old_text: Optional[str], new_text: Optional[str]) -> List[str]:
    """The values that differ between two versions of a YAML file: "key: old -> new", and keys added or removed."""
    import yaml

    try:
        old = dict(_leaves(yaml.safe_load(old_text))) if old_text is not None else {}
        new = dict(_leaves(yaml.safe_load(new_text))) if new_text is not None else {}
    except yaml.YAMLError as problem:
        return [f"does not parse: {str(problem).splitlines()[0]}"]
    lines = []
    for key in sorted(set(old) | set(new)):
        if key not in new:
            lines.append(f"{key}: removed (was {old[key]!r})")
        elif key not in old:
            lines.append(f"{key}: added ({new[key]!r})")
        elif type(old[key]) is not type(new[key]) or old[key] != new[key]:
            lines.append(f"{key}: {old[key]!r} -> {new[key]!r}")
    return lines


def tuning_reasons(repo: Path, since: str, to: str, path: str) -> Tuple[List[str], List[str]]:
    """(the reasons given, the commits that gave none) for the commits that changed a tuning file since the freeze."""
    reasons, without = [], []
    log = _git(repo, "log", "--format=%h%x00%B%x1e", f"{since}..{to}", "--", path) or ""
    for record in log.split("\x1e"):
        commit, _nul, message = record.strip().partition("\x00")
        if not commit:
            continue
        found = REASON.findall(message)
        reasons += found
        if not found:
            without.append(commit)
    return reasons, without


def check(repo: Path, since: str = FREEZE_TAG, to: str = "HEAD", worktree: bool = False) -> Dict[str, Any]:
    """What changed since the freeze, by kind, with each tuned value; and whether the freeze holds."""
    by_kind: Dict[str, List[Dict[str, Any]]] = {kind: [] for kind in KINDS}
    for status, path in changed_files(repo, since, to, worktree):
        kind = kind_of(path)
        entry: Dict[str, Any] = {"path": path, "status": status}
        if kind == "tuning":
            old = _git(repo, "show", f"{since}:{path}", check=False)
            new = (repo / path).read_text(encoding="utf-8") if worktree and (repo / path).is_file() else (None if worktree else _git(repo, "show", f"{to}:{path}", check=False))
            entry["values"] = tuned_values(old, new)
            head = "HEAD" if worktree else to
            entry["reasons"], entry["no_reason"] = tuning_reasons(repo, since, head, path)
            # In the working tree: an edit that is not committed has no reason on record yet.
            entry["uncommitted"] = bool(worktree) and _git(repo, "diff", "--quiet", "HEAD", "--", path, check=False) is None
        by_kind[kind].append(entry)
    broken = [f"{entry['path']} ({ {'A': 'added', 'D': 'removed'}.get(entry['status'], 'changed') })" for entry in by_kind["code"]]
    broken += [f"{entry['path']}: {line}" for entry in by_kind["tuning"] for line in entry["values"] if line.startswith("does not parse")]
    for entry in by_kind["tuning"]:
        if not entry["values"]:
            continue  # comments or layout only: nothing was tuned
        broken += [f"{entry['path']}: tuned in commit {commit} with no [tuning: <reason>] in its message" for commit in entry["no_reason"]]
        if entry["uncommitted"]:
            broken.append(f"{entry['path']}: edited and not committed: commit it with [tuning: <reason>] in the message before it is flown")
    return {"since": since, "to": "the working tree" if worktree else to, "changes": by_kind, "broken": broken, "holds": not broken}


def report(result: Dict[str, Any]) -> List[str]:
    changes = result["changes"]
    lines = [f"since {result['since']}, up to {result['to']}: " + ", ".join(f"{len(changes[kind])} {kind}" for kind in KINDS)]
    for entry in changes["tuning"]:
        lines.append(f"tuning  {entry['path']}")
        lines += [f"          {line}" for line in entry["values"]] or ["          (comments or layout only: no value differs)"]
        lines += [f"          reason: {reason}" for reason in entry.get("reasons") or []]
    lines += [f"field   {entry['path']}" for entry in changes["field"]]
    if changes["docs"]:
        lines.append(f"docs    {len(changes['docs'])} file(s)")
    if result["holds"]:
        lines.append("FREEZE HOLDS: nothing but tuning, field files and docs has changed" if any(changes[kind] for kind in KINDS) else "FREEZE HOLDS: nothing has changed")
    else:
        lines += [f"broken  {line}" for line in result["broken"]]
        lines.append(f"FREEZE BROKEN: {len(result['broken'])} change(s) the freeze does not allow")
    return lines


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="What has changed since the code freeze, and whether the freeze allows it.")
    parser.add_argument("--since", default=FREEZE_TAG, help=f"the freeze's tag (default {FREEZE_TAG})")
    parser.add_argument("--to", default="HEAD", help="the commit to compare (default: what is checked out)")
    parser.add_argument("--worktree", action="store_true", help="compare the working tree: uncommitted edits and new files too")
    parser.add_argument("--repo", default=str(ROOT))
    args = parser.parse_args(argv)
    try:
        result = check(Path(args.repo), args.since, args.to, args.worktree)
    except FreezeError as problem:
        if str(problem).startswith("NO_TAG"):
            print(f"freeze_check: there is no tag {args.since} yet. At the freeze, on the commit that passed the gates: git tag -a {args.since} -m \"release candidate\" <commit>", file=sys.stderr)
        else:
            print(f"freeze_check: {problem}", file=sys.stderr)
        return 2
    print("\n".join(report(result)))
    return 0 if result["holds"] else 1


if __name__ == "__main__":
    sys.exit(main())
