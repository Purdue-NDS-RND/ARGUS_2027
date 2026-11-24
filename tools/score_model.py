#!/usr/bin/env python3
"""The scoring model and slot planner (OPS-04): what a slot plan is worth, and which plan is worth most.

  python3 tools/score_model.py                              # the two plans kept ready, and the best plan found
  python3 tools/score_model.py --inputs <file> --draws 20000 --seed 1
  python3 tools/score_model.py --plan "localisation,drop | time_trial,delivery | recovery"
  python3 tools/score_model.py --set missions.*.p_flown=0.9 --set p_flight_lost=0.1       # what if

A Monte Carlo over the missions' chances (tools/score_model.yaml: each number
with where it comes from). One draw is one competition: every slot's flight
is flown, each mission in it scores what the judges' rules give for a drawn
outcome, and the best `best_of` missions are added up.

What a draw holds:

  - A mission that is flown scores by its kind: a count of waypoints passed;
    points by rank for the time trial; markers right less the wrong ones for
    the localisation; the maximum less the miss in metres for the drop, the
    delivery and the recovery, with the chance of the rules' penalty. A
    mission that is not flown, or fails, scores nothing.
  - The four bonus missions gain the autonomy bonus when the flight had no
    human input.
  - In a flight of two missions the second is flown only if the first did
    not fail in a way that ends the flight.
  - A flight can be lost for good: every slot after it is then lost too.
  - A mission may be attempted twice (in a plan given with --plan). What the
    second attempt does to the first's score is an input, `second_attempt`:
    `replaces` (it stands, whatever it scores: the assumed answer to
    organiser question 21) or `best`.

The planner tries every way of putting the missions into the slots, one a
flight and up to `--per-flight` a flight, on the same draws, and names the
best. That is the argument behind organiser question 1 (may missions be
combined in one flight?): the two answers are two rows of the table.

The numbers are only as good as the inputs: most are sim results or
assumptions until the field gives better. No ROS imports; numpy and yaml.
"""

from __future__ import annotations

import argparse
import itertools
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
INPUTS = ROOT / "tools" / "score_model.yaml"
KINDS = {
    "count": ("items", "points_each", "p_each"),
    "range": ("points_slowest", "points_fastest", "rank_low", "rank_high"),
    "list": ("items", "points_each", "p_each", "p_wrong", "wrong_penalty"),
    "distance": ("max_points", "points_per_metre", "miss_median_m", "miss_p95_m", "p_penalty", "penalty"),
}
TOP = ("slots", "best_of", "autonomy_bonus", "second_attempt", "p_no_human_input", "p_flight_lost", "p_abort_ends_flight", "missions", "plans")
SECOND_ATTEMPTS = ("replaces", "best")  # organiser question 21: what a second attempt at a mission does to the first's score
MAX_ATTEMPTS = 2
Plan = List[List[str]]


class ModelError(ValueError):
    """Inputs the model cannot run on. The message says why."""


def load_inputs(path=INPUTS) -> Dict[str, Any]:
    import yaml

    try:
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as problem:
        raise ModelError(f"{path}: {problem}") from problem
    return check_inputs(data, str(path))


def _probability(value: Any, what: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0.0 <= value <= 1.0:
        raise ModelError(f"{what} must be a probability from 0 to 1, got {value!r}")
    return float(value)


def check_inputs(data: Any, where: str = "inputs") -> Dict[str, Any]:
    if not isinstance(data, dict) or set(data) != set(TOP):
        raise ModelError(f"{where}: the inputs have exactly {', '.join(TOP)}")
    for key in ("p_no_human_input", "p_flight_lost", "p_abort_ends_flight"):
        _probability(data[key], f"{where}: {key}")
    if data["second_attempt"] not in SECOND_ATTEMPTS:
        raise ModelError(f"{where}: second_attempt is one of {', '.join(SECOND_ATTEMPTS)}")
    if not isinstance(data["slots"], int) or not 1 <= data["slots"] <= 6 or not isinstance(data["best_of"], int) or data["best_of"] < 1:
        raise ModelError(f"{where}: slots is 1 to 6 and best_of is 1 or more")
    missions = data["missions"]
    if not isinstance(missions, dict) or not missions:
        raise ModelError(f"{where}: missions is a mapping of each mission to its numbers")
    for name, mission in missions.items():
        kind = mission.get("kind") if isinstance(mission, dict) else None
        if kind not in KINDS:
            raise ModelError(f"{where}: missions.{name}.kind is one of {', '.join(KINDS)}")
        wanted = {"kind", "p_flown", "bonus", *KINDS[kind]}
        if set(mission) != wanted:
            raise ModelError(f"{where}: missions.{name} ({kind}) has exactly {', '.join(sorted(wanted))}")
        for key, value in mission.items():
            if key.startswith("p_"):
                _probability(value, f"{where}: missions.{name}.{key}")
            elif key == "bonus":
                if not isinstance(value, bool):
                    raise ModelError(f"{where}: missions.{name}.bonus is true or false")
            elif key != "kind" and (isinstance(value, bool) or not isinstance(value, (int, float)) or value < 0):
                raise ModelError(f"{where}: missions.{name}.{key} must be a number of 0 or more, got {value!r}")
        if kind == "distance" and not 0 < mission["miss_median_m"] <= mission["miss_p95_m"]:
            raise ModelError(f"{where}: missions.{name}: miss_median_m must be above 0 and no more than miss_p95_m")
        if kind == "range" and not (mission["points_slowest"] <= mission["points_fastest"] and 0.0 <= mission["rank_low"] <= mission["rank_high"] <= 1.0):
            raise ModelError(f"{where}: missions.{name}: points_slowest to points_fastest, and rank_low to rank_high inside 0 to 1")
    for name, plan in (data["plans"] or {}).items():
        check_plan(plan, data, f"{where}: plans.{name}")
    return data


def check_plan(plan: Any, inputs: Dict[str, Any], where: str = "plan") -> Plan:
    if not isinstance(plan, list) or len(plan) > inputs["slots"] or not all(isinstance(flight, list) for flight in plan):
        raise ModelError(f"{where}: a plan is a list of at most {inputs['slots']} slots, each a list of the missions of its flight")
    flown = [mission for flight in plan for mission in flight]
    unknown = sorted(set(flown) - set(inputs["missions"]))
    if unknown:
        raise ModelError(f"{where}: unknown mission(s) {', '.join(unknown)} (the missions are: {', '.join(inputs['missions'])})")
    again = sorted(mission for mission in set(flown) if flown.count(mission) > MAX_ATTEMPTS)
    if again:
        raise ModelError(f"{where}: a mission is attempted {MAX_ATTEMPTS} times at most ({', '.join(again)})")
    return plan


def parse_plan(text: str) -> Plan:
    """ "localisation,drop | time_trial,delivery | recovery" -> [["localisation", "drop"], ["time_trial", "delivery"], ["recovery"]]."""
    return [[mission.strip() for mission in flight.split(",") if mission.strip()] for flight in text.split("|")]


class Draws:
    """The outcomes of `count` competitions, drawn once: every plan is then judged on the same luck."""

    def __init__(self, inputs: Dict[str, Any], count: int, seed: int) -> None:
        rng = np.random.default_rng(seed)
        self.inputs, self.count = inputs, count
        self.names = list(inputs["missions"])
        # By mission, one entry for each attempt it may be given (a second attempt has its own luck):
        self.score: Dict[str, List[np.ndarray]] = {}  # the points before the bonus, if it is flown to its end
        self.done: Dict[str, List[np.ndarray]] = {}  # whether the mission, once begun, is flown and scores at all
        self.ends_flight: Dict[str, List[np.ndarray]] = {}  # whether its failure ends the flight
        for name, mission in inputs["missions"].items():
            self.done[name], self.ends_flight[name], self.score[name] = [], [], []
        for attempt in range(MAX_ATTEMPTS):  # every first attempt is drawn before any second, so a plan with no second attempt has the luck it always had
            for name, mission in inputs["missions"].items():
                self.done[name].append(rng.random(count) < mission["p_flown"])
                self.ends_flight[name].append(~self.done[name][attempt] & (rng.random(count) < inputs["p_abort_ends_flight"]))
                self.score[name].append(self._points(rng, mission, count))
        slots = inputs["slots"]
        self.clean = rng.random((count, slots)) < inputs["p_no_human_input"]  # by slot: the flight had no human input
        self.lost = rng.random((count, slots)) < inputs["p_flight_lost"]  # by slot: the flight ends the day

    @staticmethod
    def _points(rng, mission: Dict[str, Any], count: int) -> np.ndarray:
        kind = mission["kind"]
        if kind == "count":
            return rng.binomial(int(mission["items"]), mission["p_each"], count) * mission["points_each"]
        if kind == "range":
            rank = rng.uniform(mission["rank_low"], mission["rank_high"], count)
            return mission["points_slowest"] + (mission["points_fastest"] - mission["points_slowest"]) * rank
        if kind == "list":
            right = rng.binomial(int(mission["items"]), mission["p_each"], count)
            wrong = rng.binomial(int(mission["items"]) - right, mission["p_wrong"])
            return np.maximum(0.0, right * mission["points_each"] - wrong * mission["wrong_penalty"])
        # distance: a miss with the given median and 95th percentile (log-normal), and the rules' penalty now and then
        sigma = (np.log(mission["miss_p95_m"]) - np.log(mission["miss_median_m"])) / 1.6449
        miss = np.exp(np.log(mission["miss_median_m"]) + sigma * rng.standard_normal(count))
        penalty = np.where(rng.random(count) < mission["p_penalty"], mission["penalty"], 0.0)
        return np.maximum(0.0, mission["max_points"] - mission["points_per_metre"] * miss - penalty)

    def scores(self, plan: Plan) -> np.ndarray:
        """Each competition's points by mission, in self.names' order, for a plan. A mission not in the plan scores 0."""
        out = np.zeros((self.count, len(self.names)))
        day_alive = np.ones(self.count, dtype=bool)
        attempts = {name: 0 for name in self.names}
        for slot, flight in enumerate(plan):
            flying = day_alive.copy()
            for name in flight:
                mission, attempt, column = self.inputs["missions"][name], attempts[name], self.names.index(name)
                attempts[name] += 1
                bonus = 1.0 + (self.inputs["autonomy_bonus"] if mission["bonus"] else 0.0) * self.clean[:, slot]
                scored = np.where(flying & self.done[name][attempt], self.score[name][attempt] * bonus, 0.0)
                if attempt == 0:
                    out[:, column] = scored
                elif self.inputs["second_attempt"] == "best":
                    out[:, column] = np.maximum(out[:, column], scored)
                else:  # replaces: a second attempt that is begun stands, whatever it scores; one that is never begun changes nothing
                    out[:, column] = np.where(flying, scored, out[:, column])
                flying = flying & ~self.ends_flight[name][attempt]
            if flight:  # a slot with no flight loses nothing
                day_alive = day_alive & ~self.lost[:, slot]
        return out

    def totals(self, plan: Plan) -> np.ndarray:
        """Each competition's total: the best `best_of` missions."""
        points = np.sort(self.scores(plan), axis=1)
        return points[:, -self.inputs["best_of"] :].sum(axis=1)


def summary(draws: Draws, plan: Plan) -> Dict[str, Any]:
    totals = draws.totals(plan)
    by_mission = draws.scores(plan).mean(axis=0)
    return {"plan": plan, "mean": float(totals.mean()), "p10": float(np.percentile(totals, 10)), "p50": float(np.percentile(totals, 50)), "p90": float(np.percentile(totals, 90)),
            "by_mission": {name: float(value) for name, value in zip(draws.names, by_mission) if any(name in flight for flight in plan)}}  # fmt: skip


def all_plans(missions: Sequence[str], slots: int, per_flight: int) -> List[Plan]:
    """Every plan with each mission flown once at most: each slot a flight of up to `per_flight` missions in order. Slots are tried in every order.

    Second attempts are not searched (a plan given by hand may have them): the search is of what to fly, not of what to repeat.
    """
    flights: List[Tuple[str, ...]] = [()]
    for size in range(1, per_flight + 1):
        flights += list(itertools.permutations(missions, size))
    plans = []

    def fill(chosen: List[Tuple[str, ...]], used: frozenset) -> None:
        if len(chosen) == slots:
            if any(chosen):
                plans.append([list(flight) for flight in chosen])
            return
        for flight in flights:
            if not used & set(flight):
                fill(chosen + [flight], used | set(flight))

    fill([], frozenset())
    return plans


def best_plan(draws: Draws, per_flight: int) -> Dict[str, Any]:
    """The plan with the highest mean total among all with up to `per_flight` missions a flight."""
    best, best_mean = None, -1.0
    for plan in all_plans(draws.names, draws.inputs["slots"], per_flight):
        mean = float(draws.totals(plan).mean())
        if mean > best_mean + 1.0e-9:
            best, best_mean = plan, mean
    return summary(draws, best)


def with_changes(inputs: Dict[str, Any], changes: Sequence[str]) -> Dict[str, Any]:
    """The inputs with `NAME=VALUE` changes applied, and checked again."""
    import copy

    data = copy.deepcopy(inputs)
    for change in changes:
        name, equals, raw = change.partition("=")
        if not equals:
            raise ModelError(f"--set takes NAME=VALUE, got '{change}'")
        try:
            value: Any = float(raw) if raw not in SECOND_ATTEMPTS else raw
        except ValueError as problem:
            raise ModelError(f"--set {name}: '{raw}' is not a number") from problem
        parts = name.split(".")
        if len(parts) == 1 and parts[0] in data and parts[0] not in ("missions", "plans"):
            data[parts[0]] = int(value) if parts[0] in ("slots", "best_of") else value
        elif len(parts) == 3 and parts[0] == "missions" and (parts[1] == "*" or parts[1] in data["missions"]):
            for mission in data["missions"] if parts[1] == "*" else [parts[1]]:
                if parts[2] not in data["missions"][mission]:
                    if parts[1] == "*":
                        continue  # not every kind of mission has every number
                    raise ModelError(f"--set {name}: {mission} has no '{parts[2]}'")
                data["missions"][mission][parts[2]] = value
        else:
            raise ModelError(f"--set {name}: not an input (a top-level name, missions.<mission>.<name> or missions.*.<name>)")
    return check_inputs(data)


def plan_text(plan: Plan) -> str:
    return " | ".join(", ".join(flight) if flight else "(no flight)" for flight in plan)


def report(inputs: Dict[str, Any], count: int, seed: int, per_flight: int, extra: Optional[Plan] = None, search_draws: int = 2000) -> List[str]:
    draws = Draws(inputs, count, seed)
    rows = [(name, summary(draws, plan)) for name, plan in (inputs["plans"] or {}).items()]
    if extra is not None:
        rows.append(("given", summary(draws, check_plan(extra, inputs))))
    search = Draws(inputs, min(count, search_draws), seed)  # the search is on fewer draws; its winners are then measured on all of them
    for limit in sorted({1, per_flight}):
        found = best_plan(search, limit)["plan"]
        rows.append((f"best, {limit} a flight", summary(draws, found)))
    lines = [f"{count} draws, seed {seed}; the best {inputs['best_of']} missions of {inputs['slots']} slots count", "",
             f"{'plan':<24}{'mean':>7}{'p10':>7}{'p50':>7}{'p90':>7}  slots"]  # fmt: skip
    for name, row in rows:
        lines.append(f"{name:<24}{row['mean']:>7.1f}{row['p10']:>7.1f}{row['p50']:>7.1f}{row['p90']:>7.1f}  {plan_text(row['plan'])}")
    lines.append("")
    for name, row in rows:
        lines.append(f"{name}: " + ", ".join(f"{mission} {points:.1f}" for mission, points in row["by_mission"].items()) + "  (mean points by mission, before the best are picked)")
    return lines


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="The scoring model: what a slot plan is worth.")
    parser.add_argument("--inputs", default=str(INPUTS))
    parser.add_argument("--draws", type=int, default=20000)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--per-flight", type=int, default=2, help="how many missions a flight may hold in the planner's search (organiser question 1)")
    parser.add_argument("--plan", help='a plan to measure too: "localisation,drop | time_trial,delivery | recovery" (a mission may be in it twice)')
    parser.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                        help="change an input for this run: p_flight_lost=0.1, second_attempt=best, missions.drop.p_flown=0.9, or missions.*.p_flown=0.9 for every mission")  # fmt: skip
    args = parser.parse_args(argv)
    try:
        inputs = with_changes(load_inputs(args.inputs), args.set)
        if args.draws < 100 or not 1 <= args.per_flight <= 3:
            raise ModelError("--draws is 100 or more and --per-flight is 1 to 3")
        lines = report(inputs, args.draws, args.seed, args.per_flight, parse_plan(args.plan) if args.plan else None)
    except ModelError as problem:
        print(f"score_model: {problem}", file=sys.stderr)
        return 2
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
