"""The Hungarian method: the cheapest one-to-one assignment of rows to columns. numpy only, no ROS.

Used by the target estimator (FND-06) and the marker tracker (LOC-7) to give
the detections of one frame to the tracks they fit best, taken together
rather than one at a time.

`hungarian(cost)` assigns every row of the smaller side. `assign_gated(cost,
gate)` first keeps as many pairs at or under the gate as it can and then, among
those, the cheapest set; a pair over the gate is never returned.
"""

from __future__ import annotations

from typing import List, Tuple

import numpy as np


def hungarian(cost) -> List[Tuple[int, int]]:
    """(row, column) pairs of the assignment with the lowest total cost; min(rows, columns) pairs. Costs must be finite."""
    cost = np.asarray(cost, dtype=float)
    if cost.ndim != 2:
        raise ValueError("the cost must be a matrix: rows by columns")
    rows, columns = cost.shape
    if rows == 0 or columns == 0:
        return []
    if not np.all(np.isfinite(cost)):
        raise ValueError("the costs must be finite (use assign_gated to forbid pairs)")
    if rows > columns:
        return sorted((row, column) for column, row in hungarian(cost.T))

    # Shortest augmenting paths with row and column potentials, one row at a
    # time; indices are 1-based so that 0 can stand for "no row yet".
    row_potential = np.zeros(rows + 1)
    column_potential = np.zeros(columns + 1)
    row_of = np.zeros(columns + 1, dtype=int)  # the row each column is given to
    came_from = np.zeros(columns + 1, dtype=int)
    for row in range(1, rows + 1):
        row_of[0] = row
        column = 0
        slack = np.full(columns + 1, np.inf)
        used = np.zeros(columns + 1, dtype=bool)
        while True:
            used[column] = True
            current = row_of[column]
            reduced = cost[current - 1, :] - row_potential[current] - column_potential[1:]
            free = ~used[1:]
            better = free & (reduced < slack[1:])
            slack[1:][better] = reduced[better]
            came_from[1:][better] = column
            candidates = np.where(free, slack[1:], np.inf)
            next_column = int(np.argmin(candidates)) + 1
            step = candidates[next_column - 1]
            row_potential[row_of[used]] += step
            column_potential[used] -= step
            slack[1:][free] -= step
            column = next_column
            if row_of[column] == 0:
                break
        while column != 0:  # walk the path back, moving each column to the row before it
            previous = came_from[column]
            row_of[column] = row_of[previous]
            column = previous
    return sorted((int(row_of[column]) - 1, column - 1) for column in range(1, columns + 1) if row_of[column] != 0)


def assign_gated(cost, gate: float) -> List[Tuple[int, int]]:
    """The assignment with the most pairs whose cost is at or under `gate`, and among those the lowest total cost.

    A pair over the gate, or with a cost that is not a number, is never
    assigned: its row and column are simply left out of the answer.
    """
    cost = np.asarray(cost, dtype=float)
    if cost.ndim != 2 or cost.size == 0:
        return []
    allowed = np.isfinite(cost) & (cost <= gate)
    if not allowed.any():
        return []
    # A forbidden pair costs more than every allowed pair could together, so
    # the cheapest assignment uses as few of them as it can.
    forbidden = (float(np.abs(cost[allowed]).sum()) + 1.0) * (min(cost.shape) + 1)
    pairs = hungarian(np.where(allowed, cost, forbidden))
    return [(row, column) for row, column in pairs if allowed[row, column]]
