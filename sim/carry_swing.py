#!/usr/bin/env python3
"""How much a carried bucket swung, from a recovery flight's trace of true positions (REC-4's swing shaping).

    python3 sim/carry_swing.py sim/.state/suites/<suite>/runs/<run> [more runs]

Reads the run's trace.jsonl (the runner's record of where the vehicle and the bucket truly were) and gives, for
the carry (the bucket off the ground, the vehicle travelling):

  distance_m, duration_s, top_speed_mps   the leg
  hangs_m       how far from under the vehicle the bucket hangs when both are still (the hook's own place)
  swing_m       the most the bucket was out from that place during the leg
  residual_m    the most it was out once the vehicle had stopped at the leg's end, and for how long that was
                watched (residual_s): what a shaped start and stop are meant to leave at nothing

The standard library only.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import List, Optional, Tuple

LIFTED_M = 0.5  # the bucket this far off the ground is being carried
TRAVELLING_M = 0.5  # the vehicle this far from where it lifted has set off
STOPPED_M = 0.05  # and this near where the leg ends, for good, has stopped
LEVEL_M = 0.15  # "at the carry's height"
SPEED_OVER = 5  # samples a speed is taken over

Row = Tuple[float, float, float, float, float, float, float]  # t, vehicle e n u, bucket e n u


def read_rows(trace: Path) -> List[Row]:
    vehicle, bucket = [], []
    for line in Path(trace).read_text().splitlines():
        try:
            record = json.loads(line)
        except ValueError:
            continue
        if record.get("pose") == "vehicle":
            vehicle.append(record)
        elif record.get("pose") == "bucket":
            bucket.append(record)
    return [(v["t"], v["e"], v["n"], v["u"], b["e"], b["n"], b["u"]) for v, b in zip(vehicle, bucket)]


def _off(row: Row) -> float:
    return math.hypot(row[4] - row[1], row[5] - row[2])


def measure(rows: List[Row]) -> Optional[dict]:
    """The carry's numbers, or None when the trace has no carry in it."""
    if not rows:
        return None
    ground_u = rows[0][6]
    carried = [row for row in rows if row[6] - ground_u > LIFTED_M]
    if len(carried) <= SPEED_OVER:
        return None
    lifted_at = (carried[0][1], carried[0][2])
    level = [row for row in carried if abs(row[3] - max(r[3] for r in carried)) < LEVEL_M]  # the leg is flown at the lift's height
    if not level:
        return None
    end = level[-1]
    moving = [row for row in level if math.hypot(row[1] - lifted_at[0], row[2] - lifted_at[1]) > TRAVELLING_M and math.hypot(row[1] - end[1], row[2] - end[2]) > STOPPED_M]
    if len(moving) <= SPEED_OVER:
        return None
    started, stopped = moving[0][0], moving[-1][0]
    still = [row for row in carried if row[0] < started - 0.5][-40:] or carried[:1]
    hangs = sum(_off(row) for row in still) / len(still)
    after = [row for row in level if row[0] > stopped]
    speeds = [
        math.hypot(moving[i + SPEED_OVER][1] - moving[i][1], moving[i + SPEED_OVER][2] - moving[i][2]) / max(1.0e-6, moving[i + SPEED_OVER][0] - moving[i][0])
        for i in range(len(moving) - SPEED_OVER)
    ]
    return {
        "distance_m": round(math.hypot(end[1] - lifted_at[0], end[2] - lifted_at[1]), 2),
        "duration_s": round(stopped - started, 2),
        "top_speed_mps": round(max(speeds), 2),
        "hangs_m": round(hangs, 3),
        "swing_m": round(max(abs(_off(row) - hangs) for row in moving), 3),
        "residual_m": round(max(abs(_off(row) - hangs) for row in after), 3) if after else None,
        "residual_s": round(after[-1][0] - stopped, 2) if after else 0.0,
    }


def main(argv=None) -> int:
    runs = (argv if argv is not None else sys.argv[1:]) or []
    if not runs:
        print(__doc__)
        return 2
    for run in runs:
        trace = Path(run) / "trace.jsonl"
        if not trace.is_file():
            print(f"{run}: no trace.jsonl")
            return 1
        print(f"{Path(run).name}: {json.dumps(measure(read_rows(trace)))}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
