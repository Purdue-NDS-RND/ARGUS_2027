#!/usr/bin/env python3
"""The acceptance gates G0 to G8: what each flies, and whether its runs pass. No ROS.

  python3 sim/gate.py list
  python3 sim/gate.py flights G0                       the gate's flights, one per line
  sim/run_scenarios.sh G0                              fly them (sim/run_scenarios.sh knows gates as suites)
  python3 sim/gate.py check G0 <suite folder>... [--write docs/results/<file>.md] [--command "<how it was flown>"]

The gates and their rules are data, in sim/scenarios/gates.yaml. G0-W2,
G0-W3 and G0-W4 are G0 flown again after a later wave. `check` reads the
run folders the runner left and prints PASS, FAIL or INVALID:

  PASS     every flight of the gate was flown and every rule holds
  FAIL     a flight is missing or failed, or a rule does not hold
  INVALID  a flight has only runs the sim was too slow to count: fly it again
           and check both folders together

The first run of a flight that counts is the one judged. A flight is flown
again only when its run was INVALID, never because the vehicle did badly.
Exit code 0 PASS, 1 FAIL, 3 INVALID, 2 could not check.

Rule kinds, each over the gate's judged runs (optionally only those whose
scenario.json matches `where`):
  each     every run's metric is within {max, min, equals}
  count    at least {at_least} runs, or {fraction} of them, are within {max, min, equals}
  flown    at least {at_least} runs were judged (with `where`: of that kind)
  median, maximum, minimum, p95   that statistic of the metric is within {max, min}
  ratio    sum of {of} over sum of {over} is at least {min}
`pooled: true` treats a list metric as one pool of values across runs. A run
that did not finish OK, or has no value, never counts in a rule's favour.
A metric is a path into the run's score.json, so `settings.<name>` pins a
judge setting a scenario could otherwise loosen.
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
import subprocess
import sys
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM_DIR))

from runner.suites import scenario_path  # noqa: E402

GATES_FILE = SIM_DIR / "scenarios" / "gates.yaml"
RECHECK_OF = {"G0-W2": "G0", "G0-W3": "G0", "G0-W4": "G0"}
EACH_LIMITS = {"max", "min", "equals"}
KINDS = {
    "each": EACH_LIMITS,
    "count": EACH_LIMITS | {"at_least", "fraction"},
    "median": {"max", "min"},
    "maximum": {"max", "min"},
    "minimum": {"max", "min"},
    "p95": {"max", "min"},
    "ratio": {"of", "over", "min"},
    "flown": {"at_least"},
}
NO_METRIC = {"ratio", "flown"}
RULE_KEYS = {"name", "metric", "where", "pooled"} | set(KINDS)
GATE_KEYS = {"proves", "pass", "scenarios", "seeds", "variants", "needs", "waiting", "rules"}


class GateError(ValueError):
    pass


def load_gates(path: Path = GATES_FILE) -> dict:
    import yaml

    gates = yaml.safe_load(path.read_text()) or {}
    for name, definition in gates.items():
        unknown = sorted(str(key) for key in set(definition) - GATE_KEYS)
        if unknown:
            raise GateError(f"{path}: gate {name} has unknown key(s) {', '.join(unknown)}")
        bad = [variant for variant in definition.get("variants", []) if variant not in ("off", "on")]
        if bad:
            raise GateError(f"{path}: gate {name} has variant(s) {bad}; write \"off\" and \"on\" in quotes")
    return gates


def resolve(name: str) -> str:
    """The gate a name stands for: G0-W3 is G0, flown again."""
    return RECHECK_OF.get(name, name)


def definition_of(name: str) -> dict:
    gates = load_gates()
    gate_name = resolve(name)
    if gate_name not in gates:
        raise GateError(f"no gate named '{name}' (gates: {', '.join(gates)})")
    return gates[gate_name]


def flights(name: str) -> list[dict]:
    """The gate's flights, as the runner's suites give them. Empty if it cannot be flown yet."""
    definition = definition_of(name)
    found = []
    for item in definition.get("scenarios") or []:
        path = scenario_path(item)
        if path is None:
            raise GateError(f"gate {name} names scenario '{item}', which does not exist")
        for seed in definition.get("seeds") or []:
            for variant in definition.get("variants") or []:
                found.append({"scenario": path, "seed": int(seed), "variant": variant})
    return found


# --- rules ------------------------------------------------------------------


def _lookup(data, path: str):
    for part in path.split("."):
        if not isinstance(data, dict) or part not in data:
            return None
        data = data[part]
    return data


def _within(value, limits: dict) -> bool:
    if value is None:
        return False
    if "equals" in limits and value != limits["equals"]:
        return False
    if isinstance(value, bool) and ("max" in limits or "min" in limits):
        return False
    if "max" in limits and not value <= limits["max"]:
        return False
    if "min" in limits and not value >= limits["min"]:
        return False
    return True


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    rank = fraction * (len(ordered) - 1)
    low = math.floor(rank)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (rank - low)


def _show(value) -> str:
    if isinstance(value, float):
        return f"{value:.4g}"
    return str(value)


def _limits_text(limits: dict) -> str:
    words = {"max": "≤", "min": "≥", "equals": "="}
    return ", ".join(f"{words[key]} {_show(value)}" for key, value in limits.items() if key in words)


def evaluate_rule(rule: dict, runs: list[dict]) -> dict:
    """Whether one rule holds over the runs, with what was found."""

    name = rule.get("name", "?")
    unknown = sorted(str(key) for key in set(rule) - RULE_KEYS)
    if unknown:
        raise GateError(f"rule '{name}': unknown key(s) {', '.join(unknown)} (kinds: {', '.join(KINDS)})")
    kinds = [kind for kind in KINDS if kind in rule]
    if len(kinds) != 1:
        raise GateError(f"rule '{name}' needs exactly one of: {', '.join(KINDS)}")
    kind = kinds[0]
    limits = rule[kind]
    bad = sorted(str(key) for key in set(limits) - KINDS[kind])
    if bad:
        raise GateError(f"rule '{name}': {kind} does not take {', '.join(bad)} (it takes: {', '.join(sorted(KINDS[kind]))})")
    if kind not in NO_METRIC and "metric" not in rule:
        raise GateError(f"rule '{name}' needs a metric")
    if kind == "count" and not ({"at_least", "fraction"} & set(limits)):
        raise GateError(f"rule '{name}': count needs at_least or fraction")

    where = rule.get("where") or {}
    chosen = [run for run in runs if all(_lookup(run.get("manifest") or {}, key) == value for key, value in where.items())]
    result = {"name": name, "kind": kind, "passed": False, "value": None, "found": "", "runs": len(chosen)}
    if kind == "flown":
        result.update(value=len(chosen), passed=len(chosen) >= limits["at_least"], found=f"{len(chosen)} run(s) judged (needs {limits['at_least']})")
        return result
    if not chosen:
        result["found"] = "no run to judge" + (f" where {where}" if where else "")
        return result

    def metric(run: dict, path: str):
        # A flight that did not finish OK has no result to count.
        return _lookup(run.get("score") or {}, path) if run.get("status") == "OK" else None

    if kind == "ratio":
        tops = [metric(run, limits["of"]) for run in chosen]
        bottoms = [metric(run, limits["over"]) for run in chosen]
        if any(value is None for value in tops + bottoms) or sum(bottoms) == 0:
            result["found"] = "a run has no value"
            return result
        value = sum(tops) / sum(bottoms)
        result.update(value=value, passed=value >= limits["min"], found=f"{sum(tops)} of {sum(bottoms)} ({value:.3f}; needs ≥ {limits['min']})")
        return result

    values = [metric(run, rule["metric"]) for run in chosen]
    if rule.get("pooled"):
        if any(not isinstance(value, list) for value in values):
            result["found"] = "a run has no value"
            return result
        values = [item for value in values for item in value]

    if kind in ("each", "count"):
        good = sum(1 for value in values if _within(value, limits))
        if kind == "each":
            needed = len(values)
        else:
            needed = max(int(limits.get("at_least", 0)), math.ceil(float(limits.get("fraction", 0.0)) * len(values) - 1.0e-9))
        shown = ", ".join("none" if value is None else _show(value) for value in values)
        result.update(value=good, passed=bool(values) and good >= needed, found=f"{good} of {len(values)} {_limits_text(limits)} (needs {needed}): {shown}")
        return result

    missing = sum(1 for value in values if value is None)
    numbers = [value for value in values if value is not None]
    if missing or not numbers:
        result["found"] = f"{missing} run(s) with no value" if missing else "no values"
        return result
    value = {"median": lambda: _percentile(numbers, 0.5), "p95": lambda: _percentile(numbers, 0.95), "maximum": lambda: max(numbers), "minimum": lambda: min(numbers)}[kind]()
    result.update(value=value, passed=_within(value, limits), found=f"{kind} {_show(value)} over {len(numbers)} value(s) (needs {_limits_text(limits)})")
    return result


# --- a gate's verdict -----------------------------------------------------------


def _label(scenario: str, seed, variant: str) -> str:
    return f"{scenario} seed {seed} flags {variant}"


def check(name: str, runs: list[dict]) -> dict:
    """PASS, FAIL or INVALID for a gate, given every run flown for it (in the order flown)."""

    definition = definition_of(name)
    if definition.get("waiting"):
        raise GateError(f"gate {name} is not a sim gate: it {definition['waiting']}")
    needed = flights(name)
    if not needed:
        raise GateError(f"gate {name} cannot run yet: it needs {definition.get('needs', 'scenarios')}")

    judged, missing, invalid = [], [], []
    for flight in needed:
        scenario = flight["scenario"].relative_to(SIM_DIR / "scenarios").with_suffix("").as_posix()
        mine = [run for run in runs if (run["scenario"], run["seed"], run["variant"]) == (scenario, flight["seed"], flight["variant"])]
        counted = [run for run in mine if run["status"] != "INVALID"]
        label = _label(scenario, flight["seed"], flight["variant"])
        if counted:
            judged.append(counted[0])  # the first run that counts; a later one cannot replace it
        elif mine:
            invalid.append(label)
        else:
            missing.append(label)

    rules = [evaluate_rule(rule, judged) for rule in definition.get("rules") or []]
    not_ok = [_label(run["scenario"], run["seed"], run["variant"]) + f" ({run['status']})" for run in judged if run["status"] != "OK"]
    failed = [rule for rule in rules if not rule["passed"]]
    # An `each` rule broken by a run that was judged stays broken whatever is flown again.
    certain = [rule for rule in failed if rule["kind"] == "each" and rule["runs"]]
    reasons = []
    if missing:
        reasons.append("not flown: " + "; ".join(missing))
    if not_ok:
        reasons.append("did not finish: " + "; ".join(not_ok))
    if missing or not_ok or certain:
        verdict = "FAIL"
    elif invalid:
        verdict = "INVALID"  # undecided: the flights below have no run that counts yet
        reasons.append("only INVALID runs for: " + "; ".join(invalid) + " (fly again and check both folders together)")
    else:
        verdict = "FAIL" if failed else "PASS"
    if verdict == "FAIL" and failed:
        reasons.append("rule(s) not met: " + "; ".join(rule["name"] for rule in failed))
    return {
        "gate": name,
        "verdict": verdict,
        "reason": ". ".join(reasons),
        "pass_line": definition["pass"],
        "proves": definition["proves"],
        "rules": rules,
        "judged": judged,
        "needed": len(needed),
    }


# --- reading run folders, writing the result file --------------------------------------


def read_runs(folders: list[Path]) -> list[dict]:
    """Every run under the given suite or run folders, in folder order."""
    runs = []
    for folder in folders:
        folder = Path(folder)
        run_dirs = sorted((folder / "runs").iterdir()) if (folder / "runs").is_dir() else [folder]
        for run_dir in run_dirs:
            result_file = run_dir / "result.json"
            if not result_file.is_file():
                continue
            run = json.loads(result_file.read_text())
            for key, file_name in (("score", "score.json"), ("manifest", "scenario.json")):
                path = run_dir / file_name
                run[key] = json.loads(path.read_text()) if path.is_file() else {}
            runs.append(run)
    return runs


def _commit() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=SIM_DIR, capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def render_result(verdict: dict, runs: list[dict], command: str) -> str:
    """The gate's result file: the gate, the commit, the command, every run, the rule, the verdict."""

    name = verdict["gate"]
    definition = definition_of(name)
    variants = definition.get("variants") or []
    flags = "flags off (the 2026 behaviour)" if variants == ["off"] else "flags " + " and ".join(variants)
    on = set()
    for run in runs:
        manifest = run.get("manifest") or {}
        on.update(f"{flag}={value}" for flag, value in (manifest.get("flags") or {}).items())
        for node, node_flags in (manifest.get("node_flags") or {}).items():
            on.update(f"{node}.features.{flag}={value}" for flag, value in (node_flags or {}).items())
    on = sorted(on)
    lines = [
        f"# Gate {name}: {verdict['proves']} ({datetime.date.today().isoformat()})",
        "",
        f"**{verdict['verdict']}**" + (f": {verdict['reason']}" if verdict["reason"] else ""),
        "",
        f"- **Pass rule:** {verdict['pass_line']}",
        f"- **Commit:** `{_commit()}`",
        f"- **Flags:** {flags}" + (f"; on: {', '.join(on)}" if on else "; none on"),
        f"- **Command:** `{command}`, then `python3 sim/gate.py check {name} <suite folder>`",
        "",
        "## Rules",
        "",
        "| Rule | Met | Found |",
        "| --- | --- | --- |",
    ]
    lines += [f"| {rule['name']} | {'yes' if rule['passed'] else 'NO'} | {rule['found']} |" for rule in verdict["rules"]]
    judged = {id(run) for run in verdict["judged"]}
    lines += [
        "",
        "## Every run",
        "",
        "| Scenario | Seed | Flags | Status | Result | Points | Real-time factor | Judged |",
        "| --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for run in runs:
        rtf = run.get("real_time_factor")
        points = run.get("points")
        lines.append(
            f"| {run['scenario']} | {run['seed']} | {run['variant']} | {run['status']} | {run.get('headline') or run.get('reason', '')} | "
            f"{'n/a' if points is None else f'{points:.2f}'} | {'n/a' if rtf is None else f'{rtf:.2f}'} | {'yes' if id(run) in judged else 'no'} |"
        )
    lines += ["", "Numbers are copied from each run's `result.json`; distances are from Gazebo's true positions."]
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="The acceptance gates: list them, list a gate's flights, check its runs.")
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("list")
    flights_parser = sub.add_parser("flights")
    flights_parser.add_argument("gate")
    check_parser = sub.add_parser("check")
    check_parser.add_argument("gate")
    check_parser.add_argument("folders", nargs="+", type=Path, help="suite folders (or run folders) from sim/run_scenarios.sh, in the order flown")
    check_parser.add_argument("--write", type=Path, help="write the gate's result file here")
    check_parser.add_argument("--command", default="sim/run_scenarios.sh <gate>", help="how the runs were flown, for the result file")
    args = parser.parse_args(argv)
    try:
        if args.action == "list":
            for name, definition in load_gates().items():
                count = len(flights(name))
                state = f"{count} flights" if count else (definition.get("waiting") and f"field: {definition['waiting']}") or f"no scenarios yet: needs {definition.get('needs', '?')}"
                print(f"{name}  {definition['proves']}: {definition['pass']}  [{state}]")
            return 0
        if args.action == "flights":
            for flight in flights(args.gate):
                print(flight["scenario"], flight["seed"], flight["variant"])
            return 0
        runs = read_runs(args.folders)
        verdict = check(args.gate, runs)
    except (GateError, OSError, json.JSONDecodeError) as error:
        print(f"gate: {error}", file=sys.stderr)
        return 2
    print(f"{args.gate}: {verdict['verdict']}" + (f" ({verdict['reason']})" if verdict["reason"] else ""))
    for rule in verdict["rules"]:
        print(f"  {'ok  ' if rule['passed'] else 'FAIL'} {rule['name']}: {rule['found']}")
    if args.write:
        args.write.parent.mkdir(parents=True, exist_ok=True)
        args.write.write_text(render_result(verdict, runs, args.command))
        print(f"wrote {args.write}")
    return {"PASS": 0, "FAIL": 1, "INVALID": 3}[verdict["verdict"]]


if __name__ == "__main__":
    sys.exit(main())
