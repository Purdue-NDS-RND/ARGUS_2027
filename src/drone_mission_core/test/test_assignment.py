"""The Hungarian method (FND-06, shared with LOC-7): the cheapest one-to-one assignment, with a gate."""

import itertools

import numpy as np
import pytest

from drone_mission_core.assignment import assign_gated, hungarian


def brute_force(cost: np.ndarray) -> float:
    rows, columns = cost.shape
    if rows <= columns:
        return min(sum(cost[r, c] for r, c in enumerate(choice)) for choice in itertools.permutations(range(columns), rows))
    return brute_force(cost.T)


def test_it_finds_the_cheapest_assignment_where_greedy_does_not():
    # Greedy takes (0, 0) at 1 and is left with (1, 1) at 10; the best is 2 + 2.
    cost = np.array([[1.0, 2.0], [2.0, 10.0]])
    assert sorted(hungarian(cost)) == [(0, 1), (1, 0)]


@pytest.mark.parametrize("shape", [(1, 1), (2, 3), (3, 2), (4, 4), (5, 3), (3, 6), (6, 6)])
def test_it_matches_a_search_of_every_assignment(shape):
    rng = np.random.default_rng(shape[0] * 10 + shape[1])
    for _ in range(40):
        cost = rng.uniform(0.0, 20.0, size=shape)
        pairs = hungarian(cost)
        assert len(pairs) == min(shape)
        assert len({r for r, _ in pairs}) == len(pairs) and len({c for _, c in pairs}) == len(pairs)
        assert sum(cost[r, c] for r, c in pairs) == pytest.approx(brute_force(cost), abs=1e-9)


def test_an_empty_side_assigns_nothing():
    assert hungarian(np.zeros((0, 3))) == []
    assert hungarian(np.zeros((2, 0))) == []
    assert assign_gated(np.zeros((0, 0)), 9.21) == []


def test_pairs_over_the_gate_are_never_assigned():
    cost = np.array([[1.0, 50.0], [50.0, 60.0]])
    assert assign_gated(cost, 9.21) == [(0, 0)]


def test_the_gate_keeps_as_many_pairs_as_it_can_before_it_looks_at_cost():
    # Row 0 is close to both columns, row 1 only to column 0. Taking (0, 0), the
    # cheapest pair, would leave row 1 with nothing: two pairs beat one.
    cost = np.array([[0.1, 8.0], [5.0, 100.0]])
    assert sorted(assign_gated(cost, 9.21)) == [(0, 1), (1, 0)]


def test_a_pair_exactly_on_the_gate_is_assigned_and_bad_numbers_are_not():
    assert assign_gated(np.array([[9.21]]), 9.21) == [(0, 0)]
    assert assign_gated(np.array([[np.nan, np.inf], [np.inf, 2.0]]), 9.21) == [(1, 1)]
