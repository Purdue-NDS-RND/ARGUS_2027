"""The Hungarian method: the cheapest one-to-one pairing of rows and columns.

Written in numpy so the judge needs nothing that CI does not have. The
matrices here are tiny (one row per listed marker), so clarity wins over speed.
"""

from __future__ import annotations

import numpy as np


def assign(cost) -> list[tuple[int, int]]:
    """Pair each row with a different column so the summed cost is smallest.

    `cost` is rows x columns and need not be square: the result has
    min(rows, columns) pairs, as (row, column) sorted by row. Costs must be
    finite.
    """

    cost = np.asarray(cost, dtype=float)
    if cost.ndim != 2:
        raise ValueError("cost must be a 2-D matrix")
    if cost.size == 0:
        return []
    if not np.all(np.isfinite(cost)):
        raise ValueError("cost must be finite")

    transposed = cost.shape[0] > cost.shape[1]
    if transposed:
        cost = cost.T
    rows, cols = cost.shape

    # Shortest augmenting paths with row and column potentials (u, v).
    # Index 0 is a dummy; match[j] is the row paired with column j.
    u = np.zeros(rows + 1)
    v = np.zeros(cols + 1)
    match = np.zeros(cols + 1, dtype=int)
    way = np.zeros(cols + 1, dtype=int)

    for row in range(1, rows + 1):
        match[0] = row
        j0 = 0
        slack = np.full(cols + 1, np.inf)
        used = np.zeros(cols + 1, dtype=bool)
        while True:
            used[j0] = True
            i0 = match[j0]
            reduced = cost[i0 - 1, :] - u[i0] - v[1:]
            free = ~used[1:]
            better = free & (reduced < slack[1:])
            slack[1:][better] = reduced[better]
            way[1:][better] = j0
            candidates = np.where(free, slack[1:], np.inf)
            j1 = int(np.argmin(candidates)) + 1
            delta = candidates[j1 - 1]
            u[match[used]] += delta
            v[used] -= delta
            slack[~used] -= delta
            j0 = j1
            if match[j0] == 0:
                break
        while j0 != 0:
            j1 = way[j0]
            match[j0] = match[j1]
            j0 = j1

    pairs = [(int(match[j]) - 1, j - 1) for j in range(1, cols + 1) if match[j] != 0]
    if transposed:
        pairs = [(j, i) for i, j in pairs]
    return sorted(pairs)
