"""OPS-04: the scoring model and slot planner (tools/score_model.py). No ROS."""

import copy
import sys
from pathlib import Path

import numpy as np
import pytest

TOOLS = Path(__file__).resolve().parents[1]
REPO = TOOLS.parent
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(REPO / "sim"))

import score_model as sm  # noqa: E402
from score_model import ModelError  # noqa: E402

INPUTS = sm.load_inputs()
SEPARATE, COMBINED = INPUTS["plans"]["one_mission_a_flight"], INPUTS["plans"]["combined"]


def certain(**changes):
    """The committed inputs with nothing left to chance: every mission flown and perfect, no human input, no flight lost."""
    data = copy.deepcopy(INPUTS)
    data.update({"p_no_human_input": 1.0, "p_flight_lost": 0.0, "p_abort_ends_flight": 0.0})
    for mission in data["missions"].values():
        mission["p_flown"] = 1.0
        if mission["kind"] in ("count", "list"):
            mission["p_each"] = 1.0
        if mission["kind"] == "list":
            mission["p_wrong"] = 0.0
        if mission["kind"] == "distance":
            mission.update({"miss_median_m": 1.0e-9, "miss_p95_m": 1.0e-9, "p_penalty": 0.0})
        if mission["kind"] == "range":
            mission.update({"rank_low": 1.0, "rank_high": 1.0})
    for key, value in changes.items():
        if key in data:
            data[key] = value
        else:
            name, field = key.split("__")
            data["missions"][name][field] = value
    return sm.check_inputs(data)


def totals(inputs, plan, count=400, seed=3):
    return sm.Draws(inputs, count, seed).totals(plan)


def test_with_nothing_left_to_chance_a_plan_is_worth_the_rules_own_sum():
    # One a flight: recovery 16, drop 14, delivery 13, each with the 25% bonus.
    assert totals(certain(), SEPARATE) == pytest.approx(np.full(400, 20.0 + 17.5 + 16.25))
    # Combined: five missions flown, the best four counted: recovery 20, drop 17.5, delivery 16.25, localisation 12.5 (the time trial's 12 is the one left out).
    assert totals(certain(), COMBINED) == pytest.approx(np.full(400, 20.0 + 17.5 + 16.25 + 12.5))
    assert totals(certain(best_of=5), COMBINED) == pytest.approx(np.full(400, 66.25 + 12.0))
    draws = sm.Draws(certain(), 50, 1)
    by_mission = dict(zip(draws.names, draws.scores(COMBINED)[0]))
    assert by_mission == pytest.approx({"waypoint_navigation": 0.0, "time_trial": 12.0, "localisation": 12.5, "drop": 17.5, "delivery": 16.25, "recovery": 20.0})


def test_the_points_and_penalties_are_the_judges_own():
    from judge.score import default_settings

    m = INPUTS["missions"]
    for name in ("drop", "delivery", "recovery"):
        assert m[name]["max_points"] == default_settings(name)["max_points"] and m[name]["points_per_metre"] == default_settings(name)["points_per_metre"], name
    assert m["drop"]["penalty"] == default_settings("drop")["low_release_penalty"]
    assert (m["time_trial"]["points_slowest"], m["time_trial"]["points_fastest"]) == (default_settings("time_trial")["points_slowest"], default_settings("time_trial")["points_fastest"])
    assert m["localisation"]["points_each"] == default_settings("localisation")["points_per_marker"] and m["localisation"]["wrong_penalty"] == default_settings("localisation")["wrong_id_penalty"]
    assert m["waypoint_navigation"]["points_each"] == default_settings("waypoint_navigation")["points_per_waypoint"]
    assert INPUTS["autonomy_bonus"] == default_settings("drop")["autonomy_bonus_fraction"]
    # The delivery and the recovery score nothing when released above 6 ft: their penalty is the whole score.
    assert m["delivery"]["penalty"] == m["delivery"]["max_points"] and m["recovery"]["penalty"] == m["recovery"]["max_points"]
    # The bonus is on the four missions the rules give it to, and the judges give it to.
    assert {name for name, mission in m.items() if mission["bonus"]} == {name for name in m if default_settings(name)["autonomy_bonus_fraction"] > 0}


def test_the_bonus_needs_a_flight_with_no_human_input_and_a_bonus_mission():
    assert totals(certain(p_no_human_input=0.0), SEPARATE) == pytest.approx(np.full(400, 16.0 + 14.0 + 13.0))
    plan = [["time_trial"], ["waypoint_navigation"]]
    assert totals(certain(), plan) == pytest.approx(np.full(400, 12.0 + 7.0))  # no bonus on these two, clean or not


def test_a_miss_and_a_penalty_cost_what_the_rules_say():
    one_metre = certain(drop__miss_median_m=1.0, drop__miss_p95_m=1.0)
    assert totals(one_metre, [["drop"]]) == pytest.approx(np.full(400, (14.0 - 1.0) * 1.25))
    penalised = certain(drop__p_penalty=1.0)
    assert totals(penalised, [["drop"]]) == pytest.approx(np.full(400, (14.0 - 6.0) * 1.25))
    far = certain(delivery__miss_median_m=40.0, delivery__miss_p95_m=40.0)
    assert totals(far, [["delivery"]]) == pytest.approx(np.zeros(400))  # never below nothing
    wrong = certain(localisation__p_each=0.0, localisation__p_wrong=1.0)
    assert totals(wrong, [["localisation"]]) == pytest.approx(np.zeros(400))  # ten wrong markers: nothing, not minus five
    half = certain(waypoint_navigation__p_each=0.5)
    assert totals(half, [["waypoint_navigation"]], count=4000).mean() == pytest.approx(3.5, abs=0.15)


def test_the_second_mission_of_a_flight_is_flown_only_if_the_first_did_not_end_it():
    never = certain(localisation__p_flown=0.0, p_abort_ends_flight=1.0)
    assert totals(never, [["localisation", "drop"]]) == pytest.approx(np.zeros(400))  # the failed survey went home: no drop
    carried_on = certain(localisation__p_flown=0.0, p_abort_ends_flight=0.0)
    assert totals(carried_on, [["localisation", "drop"]]) == pytest.approx(np.full(400, 17.5))
    assert totals(never, [["drop", "localisation"]]) == pytest.approx(np.full(400, 17.5))  # the order in the flight matters
    half = certain(localisation__p_flown=0.0, p_abort_ends_flight=0.5)
    assert totals(half, [["localisation", "drop"]], count=4000).mean() == pytest.approx(8.75, abs=0.5)


def test_a_second_attempt_replaces_the_first_or_the_best_counts_as_the_input_says():
    """Organiser question 21 (assumed: a second attempt replaces the first). The second attempt has its own luck."""
    twice = [["recovery"], ["recovery"]]
    replaces = certain(recovery__p_flown=0.5)
    draws = sm.Draws(replaces, 6000, 5)
    first, second = draws.done["recovery"][0], draws.done["recovery"][1]
    assert not np.array_equal(first, second) and 0.45 < first.mean() < 0.55 and 0.45 < second.mean() < 0.55
    column = draws.names.index("recovery")
    assert np.array_equal(draws.scores(twice)[:, column] > 0, second)  # replaces: the second attempt stands, a failed one too
    best = sm.Draws(certain(recovery__p_flown=0.5, second_attempt="best"), 6000, 5)
    assert np.array_equal(best.scores(twice)[:, column] > 0, first | second)
    assert best.totals(twice).mean() == pytest.approx(0.75 * 20.0, abs=0.5) and draws.totals(twice).mean() == pytest.approx(0.5 * 20.0, abs=0.5)
    # A second attempt that is never begun (the day was lost) changes nothing: the first stands.
    lost = sm.Draws(certain(p_flight_lost=1.0), 300, 5)
    assert lost.totals(twice) == pytest.approx(np.full(300, 20.0))
    # A plan with no second attempt has the luck it had before second attempts existed.
    assert np.array_equal(sm.Draws(INPUTS, 500, 1).scores(COMBINED), sm.Draws(sm.with_changes(INPUTS, ["second_attempt=best"]), 500, 1).scores(COMBINED))


def test_an_input_can_be_changed_for_one_run(capsys):
    changed = sm.with_changes(INPUTS, ["p_flight_lost=0.1", "missions.*.p_flown=0.9", "missions.drop.miss_median_m=0.5", "best_of=3"])
    assert changed["p_flight_lost"] == 0.1 and all(mission["p_flown"] == 0.9 for mission in changed["missions"].values())
    assert changed["missions"]["drop"]["miss_median_m"] == 0.5 and changed["best_of"] == 3 and INPUTS["p_flight_lost"] == 0.03  # the committed inputs are not touched
    for bad, word in (("p_flight_lost", "NAME=VALUE"), ("p_flight_lost=often", "is not a number"), ("missions.drop.wings=2", "drop has no 'wings'"), ("plans=1", "not an input"), ("p_flight_lost=2", "must be a probability")):
        with pytest.raises(ModelError, match=word):
            sm.with_changes(INPUTS, [bad])
    assert sm.main(["--draws", "500", "--set", "missions.*.p_flown=0.9"]) == 0 and sm.main(["--set", "nothing=1"]) == 2
    capsys.readouterr()


def test_a_lost_flight_loses_every_slot_after_it():
    lost = certain(p_flight_lost=1.0)
    assert totals(lost, SEPARATE) == pytest.approx(np.full(400, 20.0))  # the first slot's flight was flown; then the day was over
    assert totals(lost, [[], ["drop"]]) == pytest.approx(np.full(400, 17.5))  # a slot with no flight loses nothing
    some = certain(p_flight_lost=0.5)
    assert totals(some, SEPARATE, count=4000).mean() == pytest.approx(20.0 + 0.5 * 17.5 + 0.25 * 16.25, abs=0.8)


def test_every_plan_is_judged_on_the_same_draws_and_a_seed_gives_the_same_answer():
    a, b = sm.Draws(INPUTS, 3000, 7), sm.Draws(INPUTS, 3000, 7)
    assert np.array_equal(a.totals(COMBINED), b.totals(COMBINED))
    assert not np.array_equal(a.totals(COMBINED), sm.Draws(INPUTS, 3000, 8).totals(COMBINED))
    # A mission's luck does not depend on the plan it is in: the drop scores the same alone as first in a flight of two.
    alone, paired = a.scores([["drop"]]), a.scores([["drop", "localisation"]])
    assert np.array_equal(alone[:, a.names.index("drop")], paired[:, a.names.index("drop")])


def test_combining_missions_is_worth_more_than_one_a_flight_on_the_committed_inputs():
    """The spec's main finding, on this file's numbers: not the spec's 39 and 57, which are for its own guesses, but the same order."""
    draws = sm.Draws(INPUTS, 20000, 1)
    separate, combined = sm.summary(draws, SEPARATE), sm.summary(draws, COMBINED)
    assert combined["mean"] > separate["mean"] + 8.0
    assert 30.0 < separate["mean"] < 45.0 and 42.0 < combined["mean"] < 60.0
    assert separate["p10"] <= separate["p50"] <= separate["p90"] and set(combined["by_mission"]) == {"time_trial", "localisation", "drop", "delivery", "recovery"}


def test_the_planner_tries_every_plan_and_its_best_is_no_worse_than_the_plans_kept_ready():
    plans = sm.all_plans(["a", "b"], slots=2, per_flight=1)
    assert sorted(map(str, plans)) == sorted(map(str, [[["a"], []], [[], ["a"]], [["b"], []], [[], ["b"]], [["a"], ["b"]], [["b"], ["a"]]]))
    assert [["a", "b"], []] in sm.all_plans(["a", "b"], 2, 2) and [["b", "a"], []] in sm.all_plans(["a", "b"], 2, 2)
    draws = sm.Draws(INPUTS, 1500, 1)
    one, two = sm.best_plan(draws, 1), sm.best_plan(draws, 2)
    assert all(len(flight) <= 1 for flight in one["plan"]) and one["mean"] >= sm.summary(draws, SEPARATE)["mean"] - 1e-9
    assert two["mean"] >= sm.summary(draws, COMBINED)["mean"] - 1e-9 and two["mean"] > one["mean"]
    assert sorted(mission for flight in one["plan"] for mission in flight) == ["delivery", "drop", "recovery"]  # with one a flight: the three worth most


@pytest.mark.parametrize(
    "change, word",
    [
        (lambda d: d.pop("best_of"), "the inputs have exactly"),
        (lambda d: d.update(p_flight_lost=1.5), "p_flight_lost must be a probability"),
        (lambda d: d["missions"]["drop"].update(kind="guess"), "missions.drop.kind is one of"),
        (lambda d: d["missions"]["drop"].pop("penalty"), "missions.drop .distance. has exactly"),
        (lambda d: d["missions"]["drop"].update(p_flown=-0.1), "missions.drop.p_flown must be a probability"),
        (lambda d: d["missions"]["drop"].update(miss_median_m=2.0, miss_p95_m=1.0), "miss_median_m must be above 0 and no more than miss_p95_m"),
        (lambda d: d["missions"]["time_trial"].update(rank_low=0.9, rank_high=0.1), "rank_low to rank_high"),
        (lambda d: d["plans"].update(bad=[["drop"], ["drop"], ["drop"]]), "a mission is attempted 2 times at most .drop."),
        (lambda d: d.update(second_attempt="adds"), "second_attempt is one of replaces, best"),
        (lambda d: d["plans"].update(bad=[["racing"]]), "unknown mission.s. racing"),
        (lambda d: d["plans"].update(bad=[["drop"], [], [], ["delivery"]]), "at most 3 slots"),
    ],
)
def test_inputs_the_model_cannot_run_on_are_refused_with_the_reason(change, word):
    data = copy.deepcopy(INPUTS)
    change(data)
    with pytest.raises(ModelError, match=word):
        sm.check_inputs(data)


def test_the_command_prints_the_plans_kept_ready_a_plan_given_and_the_best_found(capsys):
    assert sm.main(["--draws", "2000", "--plan", "drop | delivery, recovery"]) == 0
    out = capsys.readouterr().out
    lines = out.splitlines()
    assert lines[0] == "2000 draws, seed 1; the best 4 missions of 3 slots count"
    names = [line.split("  ")[0].strip() for line in lines[3:8]]
    assert names == ["one_mission_a_flight", "combined", "given", "best, 1 a flight", "best, 2 a flight"]
    assert "drop | delivery, recovery" in lines[5] and "recovery | drop | delivery" in lines[3]
    assert sm.parse_plan("localisation,drop | time_trial, delivery|recovery") == COMBINED
    assert sm.main(["--plan", "drop | racing"]) == 2 and "unknown mission" in capsys.readouterr().err
    assert sm.main(["--inputs", "/nowhere.yaml"]) == 2 and sm.main(["--draws", "5"]) == 2
