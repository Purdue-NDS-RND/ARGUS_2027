#!/usr/bin/env python3
"""Fail when a pull request adds something that looks like a secret. No ROS needed.

    scripts/secret_scan.py        scripts/check.sh and CI call this, after scripts/pr_checks.py

The repo is public. This scans what the pull request adds: the added lines of
every changed file, its commit messages, and its title and description. It
looks at the same changes, in the same three settings, as scripts/pr_checks.py
(a pull request in CI, a push to main, a local run against origin/main), by
using that script's own reading of them.

A finding names the file and line and the kind of secret, never the secret.
It belongs to the hygiene gate: a false positive is waived the same way, with
[hygiene: waived <reason>] in the pull request description.

Exit codes: 0 pass or waived, 1 a secret-shaped string was added, 2 error,
3 skipped (no origin/main to compare with).
"""

from __future__ import annotations

import os
import re
import sys
from pathlib import Path
from typing import List, Mapping

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pr_checks  # noqa: E402  (the PR gates: what changed, and the waiver marker)

# Key formats that identify themselves.
TOKEN_RULES = (
    ("private key", re.compile(r"-----BEGIN (?:[A-Z]+ )?PRIVATE KEY")),
    ("AWS access key", re.compile(r"\b(?:AKIA|ASIA)[0-9A-Z]{16}\b")),
    ("GitHub token", re.compile(r"\bgh[pousr]_[A-Za-z0-9]{36,}\b|\bgithub_pat_[A-Za-z0-9_]{40,}\b")),
    ("Google API key", re.compile(r"\bAIza[0-9A-Za-z_-]{35}\b")),
    ("Slack token", re.compile(r"\bxox[abprs]-[A-Za-z0-9-]{10,}")),
    ("Hugging Face token", re.compile(r"\bhf_[A-Za-z0-9]{30,}\b")),
    ("password in a URL", re.compile(r"\b[a-z][a-z0-9+.-]*://[^/\s:@]+:[^/\s:@]{4,}@[\w.-]")),
)

# A name that says "secret" given a quoted literal value, as in api_key="...".
ASSIGNED = re.compile(
    r"""(?ix)
    \b[\w-]*(?:api[_-]?key|secret|token|passwd|password)[\w-]*   # the name
    \\?["']?\s*[:=]\s*                                            # = or :
    \\?(["'])([^"'\s\\]{8,})\\?\1                                  # a quoted value, no spaces
    """
)  # The optional backslashes are a notebook's JSON: api_key=\\"...\\" in a cell.
# Values that are plainly not a secret: placeholders, references, and names of things.
PLACEHOLDER = re.compile(r"(?i)your|example|changeme|placeholder|dummy|\$\{|\{\{|<|^(.)\1+$")
# The name of an environment variable, not its value: capitals in words joined by underscores.
CONSTANT_NAME = re.compile(r"^[A-Z][A-Z0-9]*(?:_[A-Z0-9]+)+$")


def _mask(text: str) -> str:
    return text[:4] + "..." if len(text) > 4 else "..."


def secret_findings(line: str) -> List[str]:
    """Kinds of secret-shaped text in one line. The text itself is never returned."""
    found = []
    for kind, pattern in TOKEN_RULES:
        match = pattern.search(line)
        if match:
            found.append(f"{kind} ({_mask(match.group(0))})")
    for match in ASSIGNED.finditer(line):
        value = match.group(2)
        looks_random = any(c.isdigit() for c in value) and any(c.isalpha() for c in value)
        if looks_random and not PLACEHOLDER.search(value) and not CONSTANT_NAME.match(value):
            name = re.split(r"[\s:=\"'\\]", match.group(0), maxsplit=1)[0]
            found.append(f"a literal value assigned to '{name}'")
    return found


def findings(ctx: pr_checks.Context) -> List[str]:
    problems = []
    for path, lines in sorted(ctx.snapshot.added_lines().items()):
        for line_no, text in lines:
            problems.extend(f"{path}:{line_no}: {item}" for item in secret_findings(text))
    for sha, message in ctx.commits:
        for number, line in enumerate(message.splitlines(), start=1):
            problems.extend(f"commit {sha} message, line {number}: {item}" for item in secret_findings(line))
    for label, text in (("PR title", ctx.title), ("PR description", ctx.body)):
        for number, line in enumerate((text or "").splitlines(), start=1):
            problems.extend(f"{label}, line {number}: {item}" for item in secret_findings(line))
    return problems


def run(repo: Path, env: Mapping[str, str], out=None) -> int:
    out = out or sys.stdout
    event = env.get("GITHUB_EVENT_NAME", "")
    if event == "push":
        print("secret_scan: n/a (push to main)", file=out)
        return pr_checks.EXIT_OK
    if event and event != "pull_request":
        print(f"secret_scan: n/a ({event} event; the scan applies to pull requests)", file=out)
        return pr_checks.EXIT_OK
    try:
        ctx = pr_checks.load_context(repo, env)
        problems = findings(ctx)
    except pr_checks.CheckError as exc:
        if not event and "origin/main not found" in str(exc):
            print(f"SKIP: secret scan not run, {exc}", file=out)
            return pr_checks.EXIT_SKIP
        print(f"secret_scan: error: {exc}", file=out)
        return pr_checks.EXIT_ERROR

    if not problems:
        print(f"PASS secrets ({ctx.label})", file=out)
        return pr_checks.EXIT_OK
    reason = pr_checks.find_marker(ctx.body, "hygiene")
    status = "WAIVED" if reason else "FAIL"
    note = reason or (
        "Take the secret out and revoke it: it is in the branch's history even after the line is removed. "
        f"If it is not a secret, put {pr_checks.MARKER_TEXT['hygiene']} in the PR description."
    )
    print(f"{status} secrets: {note}", file=out)
    for problem in problems:
        print(f"    {problem}", file=out)
    return pr_checks.EXIT_OK if reason else pr_checks.EXIT_FAIL


if __name__ == "__main__":
    sys.exit(run(Path(__file__).resolve().parents[1], os.environ))
