"""Tier A: scripts/check.sh, which CI runs on every pull request, misses nothing.

Two things can go quietly wrong. A test file exists but nobody added its
run_test line, so CI never runs it. Or code is added under sim/ with no test
at all: the pull-request tests gate only looks at src/ and tools/ until its
owner extends it. Both are checked here against the whole tree.
"""

import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CHECK = (ROOT / "scripts" / "check.sh").read_text()

# sim/ modules that are thin wrappers around ROS, Gazebo or a live MAVLink
# link, with their decisions in tested modules. Adding a name here is a
# statement that the file cannot be unit-tested; say why.
SIM_WRAPPERS = {
    "sim/gps/flight_monitor.py": "rclpy node; its decisions are FlightTracker and FlightWatch in flight_log.py",
    "sim/gps/sitl_params.py": "pymavlink on a live SITL port",
    "sim/spawn_target.py": "calls Gazebo's create service; placement is in target_placement.py",
    "sim/runner/fly.py": "rclpy and Gazebo wrapper; decisions are in events.py, trace.py, watch.py and helpers.py",
}


def tracked(pattern: str) -> list[str]:
    # Tracked files and new ones not yet added, but nothing git ignores (sim/.state, build folders).
    out = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "--", pattern], cwd=ROOT, capture_output=True, text=True, check=True
    ).stdout
    return [line for line in out.splitlines() if (ROOT / line).is_file()]


def all_test_files() -> list[str]:
    return [path for path in tracked("*.py") if re.search(r"(^|/)test_[^/]*\.py$|_test\.py$", path)]


def expand_braces(text: str) -> str:
    """src/x/test_{a,b}.py -> src/x/test_a.py src/x/test_b.py, as the shell would."""
    pattern = re.compile(r"(\S*)\{([^{}]+)\}(\S*)")
    while True:
        match = pattern.search(text)
        if not match:
            return text
        expanded = " ".join(f"{match.group(1)}{part}{match.group(3)}" for part in match.group(2).split(","))
        text = text[: match.start()] + expanded + text[match.end() :]


def test_every_test_file_is_run_by_check_sh_or_listed_as_needing_ros():
    run = set(re.findall(r"^\s*run_test\s+\S+\s+(\S+)", CHECK, flags=re.MULTILINE))
    not_run_line = next(line for line in CHECK.splitlines() if "Not run (need ROS)" in line)
    start = CHECK.index(not_run_line)
    needs_ros = set(re.findall(r"[\w/.-]+\.py", expand_braces(CHECK[start : CHECK.index("finish", start)])))
    found = all_test_files()
    assert len(found) >= 20
    for path in found:
        assert path in run or path in needs_ros, f"{path} is not in scripts/check.sh: add a run_test line (or list it as needing ROS)"
    for path in run | needs_ros:
        assert (ROOT / path).is_file(), f"scripts/check.sh names {path}, which does not exist"


def test_tests_that_import_optional_modules_are_inside_the_guarded_block():
    """A test that imports cv2, numpy or yaml must print SKIP, not fail, on a machine without them."""
    guard_start = CHECK.index("for mod in cv2 numpy yaml")
    guarded = set(re.findall(r"run_test\s+\S+\s+(\S+)", CHECK[guard_start:]))
    for path in set(re.findall(r"^\s*run_test\s+\S+\s+(\S+)", CHECK, flags=re.MULTILINE)) - guarded:
        text = (ROOT / path).read_text()
        direct = re.findall(r"^\s*(?:import|from)\s+(cv2|numpy|yaml)\b", text, flags=re.MULTILINE)
        assert not direct, f"{path} imports {sorted(set(direct))} but runs outside the cv2/numpy/yaml block of scripts/check.sh"


def sim_imports(path: str) -> set[str]:
    """The sim/ modules a Python file imports, as paths. sim/ and sim/gps are on the path in tests."""
    import ast

    folder = Path(path).parent
    found = set()

    def add(module: str, base: Path | None = None) -> None:
        parts = module.split(".") if module else []
        roots = [base] if base is not None else [Path("sim"), Path("sim/gps"), folder]
        for root in roots:
            target = root.joinpath(*parts)
            for candidate in (target.with_suffix(".py"), target / "__init__.py"):
                if (ROOT / candidate).is_file() and candidate.as_posix().startswith("sim/"):
                    found.add(candidate.as_posix())

    for node in ast.walk(ast.parse((ROOT / path).read_text())):
        if isinstance(node, ast.Import):
            for alias in node.names:
                add(alias.name)
        elif isinstance(node, ast.ImportFrom):
            base = folder.joinpath(*[".."] * (node.level - 1)) if node.level else None
            add(node.module or "", base)
            for alias in node.names:  # from package import module
                add(f"{node.module}.{alias.name}" if node.module else alias.name, base)
    return {Path(item).as_posix() for item in found}


def untested_sim_modules(modules: list[str]) -> list[str]:
    """The modules no test reaches: not imported (directly or through other modules) and not run by file name."""
    this_file = Path(__file__).resolve().relative_to(ROOT).as_posix()  # it names the wrappers; that is not a test of them
    tests = [path for path in all_test_files() if path.startswith(("sim/", "scripts/")) and path != this_file]
    reached, queue = set(), []
    for path in tests:
        queue.extend(sim_imports(path))
    while queue:
        module = queue.pop()
        if module not in reached:
            reached.add(module)
            queue.extend(sim_imports(module))
    tests_text = "\n".join((ROOT / path).read_text() for path in tests)
    return [path for path in modules if path not in reached and not re.search(rf"[\"'/]{re.escape(Path(path).name)}", tests_text)]


def test_every_sim_module_is_reached_by_a_test():
    """sim/ is code: a module there needs a test, as src/ and tools/ modules do."""
    modules = [path for path in tracked("sim/*.py") if "/test/" not in path and not path.endswith("__init__.py")]
    assert len(modules) >= 20
    untested = [path for path in untested_sim_modules(modules) if path not in SIM_WRAPPERS]
    assert not untested, f"sim/ modules no test reaches: {untested}. Add a test, or (for a thin ROS wrapper only) list it in SIM_WRAPPERS."
    for path in SIM_WRAPPERS:
        assert (ROOT / path).is_file(), f"{path} is listed as a wrapper but does not exist"


def test_the_guard_notices_a_module_nobody_tests():
    assert untested_sim_modules(["sim/judge/missions.py", "sim/brand_new_tool.py"]) == ["sim/brand_new_tool.py"]
