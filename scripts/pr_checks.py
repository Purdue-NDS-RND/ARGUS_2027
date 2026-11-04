#!/usr/bin/env python3
"""Pull request gates: hygiene, tests, tuning and size. No ROS needed.

Usage:
    scripts/pr_checks.py               run the four gates (scripts/check.sh and CI call this)
    scripts/pr_checks.py staged        hygiene check on staged changes (.githooks/pre-commit)
    scripts/pr_checks.py commit-msg F  hygiene check on a commit message file (.githooks/commit-msg)

Where the gates run is read from the environment:
    GITHUB_EVENT_NAME=pull_request  CI on a pull request. PR_BASE_SHA, PR_HEAD_SHA, PR_TITLE
                                    and PR_BODY must be set. Checks the commits and the diff
                                    from the merge base to the head.
    GITHUB_EVENT_NAME=push          CI on a push to main. The gates don't apply: prints "n/a".
    unset                           Local run. Checks the working tree and the branch's commits
                                    against origin/main. The PR title and description are
                                    checked only if PR_TITLE or PR_BODY is set; otherwise
                                    prints "SKIP: no PR context".

Each gate has one escape marker. It counts only in the PR description, outside HTML
comments, and only with a reason:
    [hygiene: waived <reason>]  [tests: n/a <reason>]  [tuning: <reason>]  [large-file: <reason>]

Exit codes for the gates: 0 pass, 1 a gate failed, 2 error, 3 passed with a SKIP.
The hook modes exit 4 only on a real violation; any error prints why and exits 0.
"""

from __future__ import annotations

import importlib.util
import os
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterator, List, Mapping, Optional, Sequence, Tuple

EXIT_OK = 0
EXIT_FAIL = 1
EXIT_ERROR = 2
EXIT_SKIP = 3
# Hook modes block with 4, so a crash (Python exits 1) can't pass for a violation.
EXIT_VIOLATION = 4

MAX_FILE_BYTES = 5 * 1024 * 1024

# --- Hygiene patterns ---------------------------------------------------------

# Names that may appear even though they contain a forbidden word.
ALLOWED_NAMES = re.compile(r"CLAUDE\.md|\.claude/")
AI_WORD = re.compile(r"\b(?:claude|anthropic)\b", re.IGNORECASE)
AI_LINK = re.compile(r"\b(?:claude\.ai|anthropic\.com)\b", re.IGNORECASE)
AI_TOOLS = r"(?:an? )?(?:AI\b|LLM|Claude|ChatGPT|GPT|Copilot|Codex|Gemini|Cursor|Anthropic|OpenAI)"
GENERATED = re.compile(
    r"\U0001F916\s*generated"  # robot footer
    r"|generated (?:with|by) \[[^\]]*\]\(https?://"  # markdown-link footer
    r"|generated (?:with|by) " + AI_TOOLS,
    re.IGNORECASE,
)
HOME_PATH = re.compile(r"(?<![\w.~-])/(?:home|Users)/[A-Za-z0-9._-]+")
AGENT_TMP_PATH = re.compile(r"/tmp/claude(?:[\w.-]+|/)")
MACHINE_NAME = re.compile(r"\b(?:nsds\d+|club-drive-\d+)\b", re.IGNORECASE)

PATH_RULES = (
    (HOME_PATH, "absolute home path"),
    (AGENT_TMP_PATH, "agent temp path"),
)

# --- Escape markers -----------------------------------------------------------

MARKERS = {
    "hygiene": re.compile(r"\[hygiene:\s*waived\b([^\]]*)\]", re.IGNORECASE),
    "tests": re.compile(r"\[tests:\s*n/a\b([^\]]*)\]", re.IGNORECASE),
    "tuning": re.compile(r"\[tuning:([^\]]*)\]", re.IGNORECASE),
    "large-file": re.compile(r"\[large-file:([^\]]*)\]", re.IGNORECASE),
}
MARKER_TEXT = {
    "hygiene": "[hygiene: waived <reason>]",
    "tests": "[tests: n/a <reason>]",
    "tuning": "[tuning: <reason>]",
    "large-file": "[large-file: <reason>]",
}
HTML_COMMENT = re.compile(r"<!--.*?-->", re.DOTALL)
PLACEHOLDER = re.compile(r"^<[^>]*>$")

TUNING_FILE = re.compile(r"^src/[^/]+/config/.+\.ya?ml$")


class CheckError(Exception):
    """Something the gates need is missing or broken (not a violation)."""


# --- Text checks --------------------------------------------------------------


def path_findings(line: str) -> List[str]:
    """Rules that apply to added lines in files: local machine paths only."""
    found = []
    for pattern, label in PATH_RULES:
        match = pattern.search(line)
        if match:
            found.append(f"{label} {match.group(0)!r}")
    return found


def text_findings(text: str) -> List[str]:
    """Rules for commit messages, the PR title and the PR description."""
    found = []
    for number, line in enumerate(text.splitlines(), start=1):
        problems = []
        if AI_LINK.search(line):
            problems.append(f"AI link {AI_LINK.search(line).group(0)!r}")
        word = AI_WORD.search(ALLOWED_NAMES.sub("", line))
        if word:
            problems.append(f"AI attribution word {word.group(0)!r}")
        generated = GENERATED.search(line)
        if generated:
            problems.append(f"AI attribution {generated.group(0)!r}")
        machine = MACHINE_NAME.search(line)
        if machine:
            problems.append(f"machine name {machine.group(0)!r}")
        problems.extend(path_findings(line))
        found.extend(f"line {number}: {problem}" for problem in problems)
    return found


def find_marker(body: Optional[str], gate: str) -> Optional[str]:
    """Return the reason given with the gate's marker, or None if there is no valid one."""
    if not body:
        return None
    visible = HTML_COMMENT.sub("", body)
    for match in MARKERS[gate].finditer(visible):
        reason = match.group(1).strip()
        if reason and not PLACEHOLDER.match(reason):
            return reason
    return None


def clean_commit_message(text: str, comment_char: str = "#") -> str:
    """Drop what git itself strips: comment lines and everything below the scissors line."""
    lines = []
    for line in text.splitlines():
        if line.startswith(comment_char) and ">8" in line and "-" * 8 in line:
            break
        if line.startswith(comment_char):
            continue
        lines.append(line)
    return "\n".join(lines)


# --- Git helpers --------------------------------------------------------------


def git(repo: Path, *args: str) -> str:
    try:
        result = subprocess.run(
            ["git", "-c", "core.quotePath=false", *args],
            cwd=repo,
            capture_output=True,
            check=False,
        )
    except FileNotFoundError as exc:
        raise CheckError("git not found") from exc
    if result.returncode != 0:
        message = result.stderr.decode("utf-8", "replace").strip()
        raise CheckError(f"git {' '.join(args)} failed: {message}")
    return result.stdout.decode("utf-8", "replace")


def git_ok(repo: Path, *args: str) -> bool:
    try:
        git(repo, *args)
    except CheckError:
        return False
    return True


@dataclass
class Change:
    status: str  # A, M, D, R, C, T
    path: str
    old_path: Optional[str] = None


@dataclass
class Snapshot:
    """What a run looks at: a base commit and either a head commit or the working tree."""

    repo: Path
    base: str  # merge base
    head: Optional[str]  # None means the working tree (local runs)
    untracked: List[str] = field(default_factory=list)

    def diff_args(self) -> List[str]:
        return [self.base] if self.head is None else [self.base, self.head]

    def changes(self) -> List[Change]:
        out = git(self.repo, "diff", "--name-status", "-M", "--no-ext-diff", *self.diff_args())
        changes = []
        for row in out.splitlines():
            parts = row.split("\t")
            status = parts[0][:1]
            if status in "RC":
                changes.append(Change(status, parts[2], parts[1]))
            else:
                changes.append(Change(status, parts[1]))
        changes.extend(Change("A", path) for path in self.untracked)
        return changes

    def read_head(self, path: str) -> Optional[bytes]:
        if self.head is None:
            file = self.repo / path
            return file.read_bytes() if file.is_file() else None
        try:
            return subprocess.run(
                ["git", "show", f"{self.head}:{path}"],
                cwd=self.repo,
                capture_output=True,
                check=True,
            ).stdout
        except subprocess.CalledProcessError:
            return None

    def read_base(self, path: str) -> Optional[bytes]:
        try:
            return subprocess.run(
                ["git", "show", f"{self.base}:{path}"],
                cwd=self.repo,
                capture_output=True,
                check=True,
            ).stdout
        except subprocess.CalledProcessError:
            return None

    def head_size(self, path: str) -> int:
        if self.head is None:
            return (self.repo / path).stat().st_size
        return int(git(self.repo, "cat-file", "-s", f"{self.head}:{path}").strip())

    def added_lines(self) -> Dict[str, List[Tuple[int, str]]]:
        out = git(self.repo, "diff", "-U0", "--no-color", "--no-ext-diff", "-M", *self.diff_args())
        added = parse_added_lines(out)
        for path in self.untracked:
            data = (self.repo / path).read_bytes() if (self.repo / path).is_file() else b""
            if b"\0" in data:
                continue
            text = data.decode("utf-8", "replace")
            added[path] = list(enumerate(text.splitlines(), start=1))
        return added


def parse_added_lines(diff: str) -> Dict[str, List[Tuple[int, str]]]:
    """Map each file in a unified diff (-U0) to its added (line number, text) pairs."""
    added: Dict[str, List[Tuple[int, str]]] = {}
    path = None
    in_header = False
    line_no = 0
    for line in diff.splitlines():
        if line.startswith("diff --git "):
            in_header = True
            path = None
            continue
        if in_header:
            if line.startswith("+++ "):
                target = line[4:]
                path = target[2:] if target.startswith("b/") else None
            elif line.startswith("@@"):
                in_header = False
            else:
                continue
        if line.startswith("@@"):
            match = re.match(r"@@ -\S+ \+(\d+)", line)
            line_no = int(match.group(1)) if match else 0
            continue
        if path and line.startswith("+"):
            added.setdefault(path, []).append((line_no, line[1:]))
            line_no += 1
    return added


def commit_messages(repo: Path, base: str, head: str) -> List[Tuple[str, str]]:
    out = git(repo, "log", "--format=%h%x00%B%x1e", f"{base}..{head}")
    messages = []
    for record in out.split("\x1e"):
        record = record.strip("\n")
        if record:
            sha, _, body = record.partition("\x00")
            messages.append((sha, body))
    return messages


# --- Context ------------------------------------------------------------------


@dataclass
class Context:
    label: str
    snapshot: Snapshot
    commits: List[Tuple[str, str]]
    title: Optional[str]  # None: no PR context
    body: Optional[str]


def load_context(repo: Path, env: Mapping[str, str]) -> Context:
    event = env.get("GITHUB_EVENT_NAME", "")
    if event == "pull_request":
        missing = [name for name in ("PR_BASE_SHA", "PR_HEAD_SHA") if not env.get(name)]
        if missing:
            raise CheckError(f"pull_request run without {', '.join(missing)}")
        base_sha, head_sha = env["PR_BASE_SHA"], env["PR_HEAD_SHA"]
        merge_base = git(repo, "merge-base", base_sha, head_sha).strip()
        return Context(
            label=f"pull request, {merge_base[:7]}..{head_sha[:7]}",
            snapshot=Snapshot(repo, merge_base, head_sha),
            commits=commit_messages(repo, merge_base, head_sha),
            title=env.get("PR_TITLE", ""),
            body=env.get("PR_BODY", ""),
        )

    if not git_ok(repo, "rev-parse", "--verify", "--quiet", "refs/remotes/origin/main"):
        raise CheckError("origin/main not found (run: git fetch origin)")
    merge_base = git(repo, "merge-base", "origin/main", "HEAD").strip()
    untracked = git(repo, "ls-files", "--others", "--exclude-standard").splitlines()
    has_pr = "PR_TITLE" in env or "PR_BODY" in env
    return Context(
        label=f"local, working tree against origin/main ({merge_base[:7]})",
        snapshot=Snapshot(repo, merge_base, None, untracked),
        commits=commit_messages(repo, merge_base, "HEAD"),
        title=env.get("PR_TITLE", "") if has_pr else None,
        body=env.get("PR_BODY", "") if has_pr else None,
    )


# --- Gates --------------------------------------------------------------------


@dataclass
class GateResult:
    gate: str
    status: str  # PASS, FAIL, WAIVED, SKIP
    details: List[str] = field(default_factory=list)
    note: str = ""


def decide(gate: str, problems: List[str], ctx: Context, hint: str, marker: str = "") -> GateResult:
    """PASS with no problems, WAIVED with a valid marker in the description, else FAIL."""
    marker = marker or gate
    if not problems:
        return GateResult(gate, "PASS")
    reason = find_marker(ctx.body, marker)
    if reason:
        return GateResult(gate, "WAIVED", problems, reason)
    how = f"{hint} Or put {MARKER_TEXT[marker]} in the PR description."
    if ctx.body is None:
        how += " (Locally, set PR_BODY to check a description.)"
    return GateResult(gate, "FAIL", problems, how)


def gate_hygiene(ctx: Context) -> GateResult:
    problems = []
    for sha, message in ctx.commits:
        problems.extend(f"commit {sha} message, {item}" for item in text_findings(message))
    if ctx.title is not None:
        problems.extend(f"PR title, {item}" for item in text_findings(ctx.title))
    if ctx.body is not None:
        problems.extend(f"PR description, {item}" for item in text_findings(ctx.body))
    for path, lines in sorted(ctx.snapshot.added_lines().items()):
        for line_no, text in lines:
            problems.extend(f"{path}:{line_no}: {item}" for item in path_findings(text))
    return decide(
        "hygiene",
        problems,
        ctx,
        "Remove AI attribution and local machine details (reword commits with git rebase).",
    )


def is_test_file(path: str) -> bool:
    name = path.rsplit("/", 1)[-1]
    parts = path.split("/")[:-1]
    return (
        name.endswith(".py") and (name.startswith("test_") or name.endswith("_test.py"))
    ) or "test" in parts or "tests" in parts


def gate_tests(ctx: Context, changes: Sequence[Change]) -> GateResult:
    code = [
        c.path
        for c in changes
        if c.status != "D"
        and c.path.endswith(".py")
        and c.path.startswith(("src/", "tools/"))
        and not is_test_file(c.path)
    ]
    tests = [c.path for c in changes if c.status != "D" and is_test_file(c.path)]
    if not code or tests:
        return GateResult("tests", "PASS")
    return decide(
        "tests",
        [f"Python changed without a test change: {', '.join(code)}"],
        ctx,
        "Add or update a test file.",
    )


def flatten(node, prefix: str = "") -> Iterator[Tuple[str, object]]:
    """Yield (key path, leaf value) pairs. Lists of dicts with unique names are keyed by name."""
    if isinstance(node, dict):
        if not node:
            yield prefix, {}
        for key, value in node.items():
            yield from flatten(value, f"{prefix}.{key}" if prefix else str(key))
    elif isinstance(node, list):
        if not node:
            yield prefix, []
        names = [item.get("name") for item in node if isinstance(item, dict)]
        keyed = len(names) == len(node) and None not in names and len(set(names)) == len(names)
        for index, item in enumerate(node):
            label = f"name={item['name']}" if keyed else str(index)
            yield from flatten(item, f"{prefix}[{label}]")
    else:
        yield prefix, node


def load_yaml_docs(data: bytes):
    import yaml

    docs = list(yaml.safe_load_all(data.decode("utf-8")))
    if len(docs) == 1:
        return docs[0]
    return {f"<doc {i}>": doc for i, doc in enumerate(docs)}


def tuning_problems(path: str, old: bytes, new: bytes) -> List[str]:
    """Existing values that changed type or value, or were removed, in one YAML file."""
    import yaml

    try:
        old_leaves = dict(flatten(load_yaml_docs(old)))
    except yaml.YAMLError:
        return []  # the base never parsed, so there is nothing to compare against
    try:
        new_leaves = dict(flatten(load_yaml_docs(new)))
    except yaml.YAMLError as exc:
        return [f"{path}: does not parse: {str(exc).splitlines()[0]}"]
    problems = []
    for key, old_value in old_leaves.items():
        if key not in new_leaves:
            problems.append(f"{path}: {key} removed (was {old_value!r})")
            continue
        new_value = new_leaves[key]
        if type(old_value) is not type(new_value) or old_value != new_value:
            problems.append(f"{path}: {key} changed {old_value!r} -> {new_value!r}")
    return problems


def gate_tuning(ctx: Context, changes: Sequence[Change]) -> GateResult:
    relevant = [
        c for c in changes if TUNING_FILE.match(c.path) or (c.old_path and TUNING_FILE.match(c.old_path))
    ]
    if not relevant:
        return GateResult("tuning", "PASS")
    if importlib.util.find_spec("yaml") is None:
        return GateResult("tuning", "SKIP", note="PyYAML not installed, config YAML changes not checked")
    problems = []
    for change in relevant:
        if change.status in "AC":
            continue
        old_path = change.old_path or change.path
        old = ctx.snapshot.read_base(old_path)
        if old is None:
            continue
        if change.status == "D":
            problems.append(f"{old_path}: deleted")
            continue
        new = ctx.snapshot.read_head(change.path)
        if new is None:
            problems.append(f"{change.path}: missing")
            continue
        problems.extend(tuning_problems(change.path, old, new))
    return decide(
        "tuning",
        problems,
        ctx,
        "Add a new parameter (with a default that keeps today's behaviour) instead of changing a tuned value.",
    )


def gate_size(ctx: Context, changes: Sequence[Change]) -> GateResult:
    problems = []
    for change in changes:
        if change.status not in "AC":
            continue
        size = ctx.snapshot.head_size(change.path)
        if size > MAX_FILE_BYTES:
            problems.append(f"{change.path}: {size / 1024 / 1024:.1f} MB (limit 5 MB)")
    return decide(
        "size",
        problems,
        ctx,
        "Keep large files out of git; put them on the team Drive.",
        marker="large-file",
    )


def run_gates(repo: Path, env: Mapping[str, str], out=None) -> int:
    out = out or sys.stdout
    event = env.get("GITHUB_EVENT_NAME", "")
    if event and event != "pull_request":
        print(f"pr_checks: n/a ({event} event; the gates apply to pull requests)", file=out)
        return EXIT_OK
    try:
        ctx = load_context(repo, env)
        changes = ctx.snapshot.changes()
        results = [
            gate_hygiene(ctx),
            gate_tests(ctx, changes),
            gate_tuning(ctx, changes),
            gate_size(ctx, changes),
        ]
    except CheckError as exc:
        if not env.get("GITHUB_EVENT_NAME") and "origin/main not found" in str(exc):
            print(f"SKIP: PR gates not run, {exc}", file=out)
            return EXIT_SKIP
        print(f"pr_checks: error: {exc}", file=out)
        return EXIT_ERROR

    print(f"PR gates ({ctx.label})", file=out)
    skipped = False
    failed = False
    for result in results:
        if result.status == "PASS":
            print(f"PASS {result.gate}", file=out)
        elif result.status == "SKIP":
            print(f"SKIP: {result.gate}: {result.note}", file=out)
            skipped = True
        elif result.status == "WAIVED":
            print(f"WAIVED {result.gate}: {result.note}", file=out)
            for detail in result.details:
                print(f"    {detail}", file=out)
        else:
            print(f"FAIL {result.gate}: {result.note}", file=out)
            for detail in result.details:
                print(f"    {detail}", file=out)
            failed = True
    if ctx.title is None:
        print("SKIP: no PR context, PR title and description not checked", file=out)
        skipped = True
    if failed:
        return EXIT_FAIL
    return EXIT_SKIP if skipped else EXIT_OK


# --- Hook modes ---------------------------------------------------------------


def run_staged(repo: Path, out=None) -> int:
    out = out or sys.stdout
    diff = git(repo, "diff", "--cached", "-U0", "--no-color", "--no-ext-diff", "-M")
    problems = []
    for path, lines in sorted(parse_added_lines(diff).items()):
        for line_no, text in lines:
            problems.extend(f"{path}:{line_no}: {item}" for item in path_findings(text))
    if not problems:
        return EXIT_OK
    print("pre-commit: staged changes contain local machine paths:", file=out)
    for problem in problems:
        print(f"    {problem}", file=out)
    print("Remove them, or commit with --no-verify and waive in the PR with [hygiene: waived <reason>].", file=out)
    return EXIT_VIOLATION


def run_commit_msg(repo: Path, message_file: str, out=None) -> int:
    out = out or sys.stdout
    comment_char = "#"
    try:
        configured = git(repo, "config", "--get", "core.commentChar").strip()
        if configured and configured != "auto":
            comment_char = configured
    except CheckError:
        pass
    text = Path(message_file).read_text(encoding="utf-8", errors="replace")
    problems = text_findings(clean_commit_message(text, comment_char))
    if not problems:
        return EXIT_OK
    print("commit-msg: the commit message has AI attribution or local machine details:", file=out)
    for problem in problems:
        print(f"    {problem}", file=out)
    print("Reword it. To keep it anyway, commit with --no-verify and add [hygiene: waived <reason>] to the PR description.", file=out)
    return EXIT_VIOLATION


def run_hook(func, *args) -> int:
    """Hooks fail open: an error prints why and lets the commit through."""
    try:
        return func(*args)
    except Exception as exc:  # noqa: BLE001
        print(f"pr_checks: hook error, not blocking: {exc}")
        return EXIT_OK


def main(argv: Sequence[str], env: Mapping[str, str] = os.environ, cwd: Optional[Path] = None) -> int:
    repo = Path(cwd or os.getcwd())
    if not argv:
        return run_gates(repo, env)
    if argv[0] == "staged":
        return run_hook(run_staged, repo)
    if argv[0] == "commit-msg" and len(argv) == 2:
        return run_hook(run_commit_msg, repo, argv[1])
    print(__doc__, file=sys.stderr)
    return EXIT_ERROR


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
