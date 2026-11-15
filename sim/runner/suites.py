"""Which flights a suite or a scenario name stands for. No ROS."""

from __future__ import annotations

import sys
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parents[1]
SCENARIOS_DIR = SIM_DIR / "scenarios"
SUITES_FILE = SCENARIOS_DIR / "suites.yaml"
VARIANTS = ("off", "on")


def load_suites(path: Path = SUITES_FILE) -> dict:
    import yaml

    suites = yaml.safe_load(path.read_text()) or {}
    for name, suite in suites.items():
        if not isinstance(suite, dict) or set(suite) != {"scenarios", "seeds", "variants"}:
            raise ValueError(f"{path}: suite '{name}' needs scenarios, seeds and variants")
        bad = [variant for variant in suite["variants"] if variant not in VARIANTS]
        if bad:
            raise ValueError(f"{path}: suite '{name}' has variant(s) {bad}; write \"off\" and \"on\" in quotes")
    return suites


def scenario_path(name: str) -> Path | None:
    """The scenario file for a name like delivery/baseline, or for a path to one."""
    for candidate in (Path(name), SCENARIOS_DIR / f"{name}.yaml", SCENARIOS_DIR / name):
        if candidate.is_file() and candidate.suffix == ".yaml":
            resolved = candidate.resolve()
            # Any scenario file, but not suites.yaml or gates.yaml, which sit beside the mission folders.
            if resolved.parent != SCENARIOS_DIR.resolve():
                return resolved
    return None


def _gate_flights(name: str) -> list[dict] | None:
    """The flights of an acceptance gate (sim/gate.py), or of all of them for "gates"; None if not a gate."""
    sys.path.insert(0, str(SIM_DIR))
    import gate

    gates = gate.load_gates()
    if name == "gates":
        return [flight for gate_name in gates for flight in gate.flights(gate_name)]
    if gate.resolve(name) not in gates:
        return None
    found = gate.flights(name)
    if not found:
        definition = gates[gate.resolve(name)]
        why = definition.get("waiting") or f"needs {definition.get('needs', 'scenarios')}"
        raise ValueError(f"gate {name} has nothing to fly in the sim yet: it {why}")
    return found


def expand(name: str, seeds=None, variants=None, suites_file: Path = SUITES_FILE) -> list[dict]:
    """The flights for a suite name, a scenario like delivery/baseline, or a scenario file.

    Each is {"scenario": path, "seed": n, "variant": "off" or "on"}. `seeds`
    and `variants` override the suite's; a scenario named directly flies its
    own seed with flags off unless told otherwise.
    """

    suites = load_suites(suites_file)
    gate_flights = _gate_flights(name)
    if gate_flights is not None:
        # A gate fixes its own seeds and variants: they are part of its pass rule.
        if seeds or variants:
            raise ValueError(f"'{name}' is a gate: its seeds and variants are fixed in sim/scenarios/gates.yaml")
        return gate_flights
    if name in suites:
        suite = suites[name]
        scenarios = [scenario_path(item) for item in suite["scenarios"]]
        if None in scenarios:
            missing = suite["scenarios"][scenarios.index(None)]
            raise ValueError(f"suite '{name}' names scenario '{missing}', which does not exist")
        seeds = list(seeds or suite["seeds"])
        variants = list(variants or suite["variants"])
    else:
        path = scenario_path(name)
        if path is None:
            raise ValueError(f"no suite or scenario named '{name}' (suites: {', '.join(suites)}, gates, G0 to G6)")
        scenarios = [path]
        if not seeds:
            sys.path.insert(0, str(SIM_DIR))
            from scenario import load_scenario

            seeds = [load_scenario(path).seed]
        variants = list(variants or ["off"])
    bad = [variant for variant in variants if variant not in VARIANTS]
    if bad:
        raise ValueError(f"unknown variant(s) {bad}: use off or on")
    return [
        {"scenario": scenario, "seed": int(seed), "variant": variant}
        for scenario in scenarios
        for seed in seeds
        for variant in variants
    ]


def main(argv: list[str]) -> int:
    """Print "scenario seed variant" lines for run_scenarios.sh."""
    import argparse

    parser = argparse.ArgumentParser(description="List the flights of a suite or scenario.")
    parser.add_argument("name")
    parser.add_argument("--seeds", help="comma-separated, overrides the suite's")
    parser.add_argument("--variants", help="comma-separated off,on; overrides the suite's")
    args = parser.parse_args(argv)
    try:
        seeds = [int(seed) for seed in args.seeds.split(",")] if args.seeds else None
        variants = args.variants.split(",") if args.variants else None
        for run in expand(args.name, seeds, variants):
            print(run["scenario"], run["seed"], run["variant"])
    except ValueError as error:
        print(f"suites: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
