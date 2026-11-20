#!/usr/bin/env python3
"""The A/B harness (CEN-12): fly the same seeded scenarios with flags off and on, and say whether a flag may be turned on.

  python3 sim/ab/run_ab.py drop                          fly the drop plan and report
  python3 sim/ab/run_ab.py drop --seeds 1,2              with these seeds instead of the plan's
  python3 sim/ab/run_ab.py drop --from <suite folder>... report from suites already flown, flying nothing
  python3 sim/ab/run_ab.py drop --report docs/results/2026-11-20-ab-drop.md

A plan (sim/ab/plans.yaml) names a flag, its scenarios in calm air and in the
sim's strong wind (7 m/s), and the seeds. Each scenario is flown with each
seed twice by sim/run_scenarios.sh, `--variants off,on`: every flag false
(the 2026 flight) and the scenario's flags on. One sim at a time.

The report has, for calm and for wind, each side's flights, failures, median
and largest error (where the package, or the vehicle at a fake release, was
from the target), and median flight time.

Pass for turning the flag on (FEATURES.md, CEN-12):

  - drop: median error with the flags on at most 0.15 m in calm air and
    0.30 m in wind;
  - no floor violation with the flags on: no drop released or flown below
    1.83 m, no delivery released above 1.83 m;
  - no more failures with the flags on than off, in calm and in wind.

A failure is a flight that did not score: nothing was released, or the
runner did not call it OK. One kind of not-OK is counted apart: a flight
that released, marked FAIL only because the projection, estimator or
height check beside it missed its line. Those three checks run only with
the flags on, so counting them would hold against the 2027 side something
the 2026 side is never asked. Each is named in the report and in the
verdict line. (The gates, sim/gate.py, do count such a flight as failed:
a gate judges the 2027 side on its own.) An INVALID flight
(the sim ran too slowly for it to count) is neither side's failure; it has
to be flown again, and until it is the comparison is not a pass. Nor is a
comparison with a flight missing.
The spec gives the delivery no error line: it is judged on the floor and the
failures, and its error is reported.

The arithmetic and the verdict are plain functions with no ROS in them.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path
from statistics import median
from typing import Iterable, List, Optional

SIM = Path(__file__).resolve().parents[1]
REPO = SIM.parent
PLANS = Path(__file__).resolve().with_name("plans.yaml")

DROP_CALM_MEDIAN_M = 0.15
DROP_WIND_MEDIAN_M = 0.30
GROUPS = ("calm", "wind")
_SUITE_LINE = re.compile(r"results in (\S+)")


def load_plans(path: Path = PLANS) -> dict:
    import yaml

    plans = yaml.safe_load(path.read_text(encoding="utf-8"))
    for name, plan in plans.items():
        for key in ("flag", "mission", "calm", "seeds"):
            if not plan.get(key):
                raise ValueError(f"{path.name}: plan '{name}' has no {key}")
        plan.setdefault("wind", [])
    return plans


def commands(plan: dict) -> List[List[str]]:
    """One runner command a scenario: every seed, off and on."""
    seeds = ",".join(str(seed) for seed in plan["seeds"])
    return [["sim/run_scenarios.sh", scenario, "--seeds", seeds, "--variants", "off,on"] for group in GROUPS for scenario in plan.get(group, [])]


def load_runs(suites: Iterable[Path]) -> List[dict]:
    """Every run of these suite folders, with its score's details. A run without a score has none."""
    runs = []
    for suite in suites:
        results = Path(suite) / "results.json"
        for result in json.loads(results.read_text(encoding="utf-8")) if results.is_file() else []:
            run = {key: result.get(key) for key in ("scenario", "seed", "variant", "status", "reason", "flight_s")}
            run["details"], run["mission"] = {}, None
            score = Path(str(result.get("run_dir") or "")) / "score.json"
            if score.is_file():
                scored = json.loads(score.read_text(encoding="utf-8"))
                run["details"], run["mission"] = scored.get("details") or {}, scored.get("mission")
            runs.append(run)
    return runs


# How sim/runner/result.py words the three checks that run only beside flags-on flights. Any other reason
# for FAIL (a package that never left, a sensor that never published) is a failed flight.
_CHECK_REASONS = ("projection v2 missed its pass line", "the target estimator missed a pass line", "the height estimator missed its pass line")


def check_missed(run: dict) -> bool:
    """A flight the runner marked FAIL because the projection, estimator or height check beside it missed its line, and which released."""
    return (
        run.get("status") == "FAIL"
        and str(run.get("reason") or "").startswith(_CHECK_REASONS)
        and bool(run.get("details", {}).get("released"))
    )


def failed(run: dict) -> bool:
    """A flight that did not score: nothing was released, or the runner did not call it OK.

    The one exception is check_missed: a flight that released, marked FAIL
    only for one of the three checks that run beside flags-on flights.
    (An INVALID flight is not judged at all: see compare.)
    """
    if check_missed(run):
        return False
    return run.get("status") != "OK" or not run.get("details", {}).get("released")


def _side(runs: List[dict]) -> dict:
    judged = [run for run in runs if run.get("status") != "INVALID"]
    scored = [run for run in judged if not failed(run)]
    errors = sorted(float(run["details"]["distance_m"]) for run in scored if isinstance(run["details"].get("distance_m"), (int, float)))
    times = sorted(float(run["flight_s"]) for run in scored if isinstance(run.get("flight_s"), (int, float)))
    return {
        "runs": len(judged),
        "failures": sum(1 for run in judged if failed(run)),
        "check_misses": sum(1 for run in judged if check_missed(run)),
        "median_m": median(errors) if errors else None,
        "max_m": errors[-1] if errors else None,
        "median_flight_s": median(times) if times else None,
    }


def compare(plan: dict, runs: List[dict]) -> dict:
    """The comparison and the verdict for a plan over its runs."""
    problems: List[str] = []
    misses: List[str] = []
    groups: dict = {}
    by_key = {(run["scenario"], run["seed"], run["variant"]): run for run in runs}
    for group in GROUPS:
        scenarios = plan.get(group) or []
        if not scenarios:
            continue
        sides = {"off": [], "on": []}
        for scenario in scenarios:
            for seed in plan["seeds"]:
                for variant in ("off", "on"):
                    run = by_key.get((scenario, seed, variant))
                    if run is None:
                        problems.append(f"{scenario} seed {seed} was not flown with the flags {variant}")
                        continue
                    if run.get("status") == "INVALID":
                        problems.append(f"{scenario} seed {seed}, flags {variant}, is INVALID ({run.get('reason') or 'no reason given'}): fly it again")
                    sides[variant].append(run)
                    if check_missed(run):
                        misses.append(f"{scenario} seed {seed}, flags {variant}: {run.get('reason') or 'no reason given'}")
                    if variant == "on":
                        details = run.get("details", {})
                        if plan["mission"] == "drop" and (details.get("released_low") or details.get("flew_low")):
                            problems.append(f"floor: {scenario} seed {seed} released or flew below 1.83 m with the flags on")
                        if plan["mission"] == "delivery" and details.get("released_high"):
                            problems.append(f"floor: {scenario} seed {seed} released above 1.83 m with the flags on")
        off, on = _side(sides["off"]), _side(sides["on"])
        groups[group] = {"off": off, "on": on}
        if on["failures"] > off["failures"]:
            plural = "" if on["failures"] == 1 else "s"
            problems.append(f"{group}: {on['failures']} failure{plural} with the flags on, {off['failures']} with them off")
        if on["runs"] and on["median_m"] is None:
            problems.append(f"{group}: no flight with the flags on scored, so there is no error to judge")
        if plan["mission"] == "drop" and on["median_m"] is not None:
            limit = DROP_CALM_MEDIAN_M if group == "calm" else DROP_WIND_MEDIAN_M
            if on["median_m"] > limit:
                problems.append(f"{group}: flags-on median error {on['median_m']:.2f} m is over {limit:g} m")
    if not groups:
        problems.append("the plan has no scenarios")
    # The pass lines first, then what was not flown: the reader wants the verdict's reason before the bookkeeping.
    problems.sort(key=lambda text: ("was not flown" in text, "INVALID" in text))
    return {"flag": plan["flag"], "mission": plan["mission"], "groups": groups, "problems": problems, "check_misses": misses, "passed": not problems}


def _cell(value: Optional[float], unit: str, digits: int) -> str:
    return "n/a" if value is None else f"{value:.{digits}f} {unit}"


def render(plan: dict, report: dict, ran: Optional[List[str]] = None) -> str:
    lines = [
        f"# A/B: {plan['flag']}, flags off against flags on",
        "",
        f"Plan `{plan['mission']}` of `sim/ab/plans.yaml`: seeds {', '.join(str(seed) for seed in plan['seeds'])}; "
        f"calm: {', '.join(plan['calm'])}; wind: {', '.join(plan.get('wind') or []) or 'none'}.",
        "",
        "Error is where the package came to rest (or the vehicle was at a fake release) from the target's centre. "
        "A failure is a flight that did not score; a flight that released but missed a check beside it is listed apart. "
        "Time is the median flight time of the flights that scored.",
        "",
        "| Air | Off: flights | failures | median error | largest | time | On: flights | failures | median error | largest | time |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for group in GROUPS:
        if group not in report["groups"]:
            continue
        cells = [group]
        for side in ("off", "on"):
            data = report["groups"][group][side]
            cells += [str(data["runs"]), str(data["failures"]), _cell(data["median_m"], "m", 2), _cell(data["max_m"], "m", 2), _cell(data["median_flight_s"], "s", 0)]
        lines.append("| " + " | ".join(cells) + " |")
    lines += ["", "## Verdict", ""]
    if report["passed"]:
        missed = len(report.get("check_misses") or [])
        also = "" if not missed else f" {missed} flags-on flight{'' if missed == 1 else 's'} missed a check beside it: see below."
        lines.append(f"PASS: {plan['flag']} may be turned on, by CEN-12's pass lines.{also}")
    else:
        lines.append(f"FAIL: {plan['flag']} stays off.")
        lines += [""] + [f"- {problem}" for problem in report["problems"]]
    if report.get("check_misses"):
        lines += [
            "",
            "## Checks beside the flights that missed their line",
            "",
            "These flights released and are counted in the errors above. A check that runs beside a flags-on flight "
            "(projection, estimator or height against the truth) missed its own line; the flags-off side is never asked these.",
            "",
        ] + [f"- {miss}" for miss in report["check_misses"]]
    if plan["mission"] == "drop":
        lines += ["", f"Pass lines: flags-on median error at most {DROP_CALM_MEDIAN_M:g} m in calm air and {DROP_WIND_MEDIAN_M:g} m in wind; no release or flight below 1.83 m; no more failures than with the flags off."]
    else:
        lines += ["", "Pass lines: no release above 1.83 m; no more failures than with the flags off. The spec gives the delivery's error no line: it is reported."]
    if ran:
        lines += ["", "## Flown", ""] + [f"- `{command}`" for command in ran]
    return "\n".join(lines) + "\n"


def fly(plan: dict) -> List[Path]:
    """Fly the plan's scenarios, one runner command after another. The suite folders."""
    suites = []
    for command in commands(plan):
        print("== " + " ".join(command), flush=True)
        done = subprocess.run(command, cwd=REPO, capture_output=True, text=True)
        found = _SUITE_LINE.search(done.stdout)
        if found is None:
            raise RuntimeError(f"{' '.join(command)} did not say where its results are:\n{done.stdout[-2000:]}\n{done.stderr[-2000:]}")
        suites.append(REPO / found.group(1))
    return suites


def main(argv: Optional[List[str]] = None) -> int:
    plans = load_plans()
    parser = argparse.ArgumentParser(description="Fly a plan's scenarios with flags off and on, and say whether its flag may be turned on.")
    parser.add_argument("plan", choices=sorted(plans))
    parser.add_argument("--seeds", help="comma-separated seeds, in place of the plan's")
    parser.add_argument("--from", dest="suites", nargs="+", type=Path, help="report from these suite folders; fly nothing")
    parser.add_argument("--report", type=Path, help="write the report here as well as printing it")
    args = parser.parse_args(argv)
    plan = dict(plans[args.plan])
    if args.seeds:
        plan["seeds"] = [int(seed) for seed in args.seeds.split(",")]
    ran = None
    suites = args.suites
    if suites is None:
        ran = [" ".join(command) for command in commands(plan)]
        suites = fly(plan)
    report = compare(plan, load_runs(suites))
    text = render(plan, report, ran)
    print(text)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(text, encoding="utf-8")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
