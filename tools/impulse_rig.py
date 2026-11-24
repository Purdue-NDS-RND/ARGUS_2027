#!/usr/bin/env python3
"""The impulse rig's analysis (OPS-07): what a delivery's touchdown does to the plate, and the descent speed that keeps it under a limit.

  python3 tools/impulse_rig.py --trial run_01.csv:0.08 --trial run_02.csv:0.12 --trial run_03.csv:0.20 --package-kg 0.5
  python3 tools/impulse_rig.py ... --limit-ns 0.08               # the organisers' limit as an impulse (N s)
  python3 tools/impulse_rig.py ... --limit-peak-n 9.0            # or as a peak force (N)

The rig is a plate the package is set down on (docs/ops/impulse_rig.md). A
trial is one set-down at a known descent speed, recorded as a CSV with a
header, `t_s,force_n`, from a load cell under the plate. The plate is zeroed
empty: force 0 is the bare plate. (An accelerometer on a free plate is not
read: such a plate starts and ends at rest, so its acceleration sums to
nothing, and it does not show the package's weight to find the rest by.)

For each trial:

  impulse     the integral of (force - the package's settled weight) from
              first contact to rest, in N s. This is the momentum the plate
              took out of the package: for a clean set-down it is the mass
              that came down times its speed, which the tool checks against
              the package's mass (a line still carrying weight, or a vehicle
              pushing down, shows here);
  peak        the largest force, in N and as a multiple of the weight;
  contact     how long, first touch to rest.

Across the trials it fits impulse and (peak - weight) against descent speed
as straight lines through zero, and with a limit gives the fastest descent
that stays under it with a margin (the worst trial's slope, and 20% off).
Trials in which the plate was never pressed harder than the package weighs
(the weight came on gently, as when the vehicle holds the package until it
stands) bound no speed: the tool says so instead of giving one.
Which of the two the organisers will limit, and how they measure it, is
their open answer (question 9): both are here.

A contact lasts tens of milliseconds. A recording with fewer than
MIN_CONTACT_SAMPLES samples in the contact cannot give its peak, and is
refused: a hobby load-cell amplifier at 80 samples a second is too slow.

No ROS imports; the standard library only.
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path
from statistics import median
from typing import Any, Dict, List, Optional, Sequence, Tuple

G = 9.80665
MIN_CONTACT_SAMPLES = 8
CONTACT_FRACTION = 0.05  # of the settled weight: the force above the empty plate's that is "touching"
REST_BAND = 0.05  # at rest: within this share of the settled weight
REST_S = 0.10  # for this long
MARGIN = 0.8  # of the speed the worst trial allows
MASS_TOLERANCE = 0.25  # the momentum's mass against the package's: beyond this the set-down was not clean


class RigError(ValueError):
    """A recording that cannot be analysed. The message says why."""


@dataclass(frozen=True)
class Trial:
    name: str
    descent_mps: float
    impulse_ns: float
    peak_n: float
    weight_n: float
    contact_s: float
    samples: int
    rate_hz: float

    @property
    def momentum_kg(self) -> float:
        """The mass that came down at the descent speed, by the impulse."""
        return self.impulse_ns / self.descent_mps

    @property
    def peak_over_weight(self) -> float:
        return self.peak_n / self.weight_n


def read_csv(path: Path) -> Tuple[List[float], List[float]]:
    """(times, forces on the plate above its own weight) from a rig recording."""
    column = "force_n"
    try:
        with open(path, newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
    except OSError as problem:
        raise RigError(f"{path}: {problem}") from problem
    if not rows or "t_s" not in rows[0] or column not in rows[0]:
        raise RigError(f"{path}: expected a header with t_s and {column}")
    try:
        times = [float(row["t_s"]) for row in rows]
        values = [float(row[column]) for row in rows]
    except (TypeError, ValueError) as problem:
        raise RigError(f"{path}: a row is not numbers: {problem}") from problem
    if any(later <= earlier for earlier, later in zip(times, times[1:])):
        raise RigError(f"{path}: the times do not go forward")
    return times, values


def analyse(name: str, times: Sequence[float], forces: Sequence[float], descent_mps: float) -> Trial:
    """One set-down: its impulse, peak and contact time."""
    if descent_mps <= 0:
        raise RigError(f"{name}: the descent speed must be above zero")
    if len(times) < 4 * MIN_CONTACT_SAMPLES:
        raise RigError(f"{name}: {len(times)} samples is not a recording of a set-down")
    rate = (len(times) - 1) / (times[-1] - times[0])
    tail = [force for t, force in zip(times, forces) if t >= times[-1] - 2 * REST_S]
    weight = median(tail)
    if weight <= 0:
        raise RigError(f"{name}: nothing is resting on the plate at the end of the recording (the settled force is {weight:.2f} N): record until the package is at rest, with the empty plate zeroed")
    touching = next((index for index, force in enumerate(forces) if force > CONTACT_FRACTION * weight), None)
    if touching is None or touching == 0:
        raise RigError(f"{name}: the recording must start with the plate empty, before the touch")
    # At rest: the first moment from which the force stays in the band for REST_S.
    rest = None
    for index in range(touching, len(times)):
        window = [force for t, force in zip(times[index:], forces[index:]) if t <= times[index] + REST_S]
        if times[-1] - times[index] >= REST_S and all(abs(force - weight) <= REST_BAND * weight for force in window):
            rest = index
            break
    if rest is None:
        raise RigError(f"{name}: the force never settles within {REST_BAND:.0%} of {weight:.2f} N for {REST_S:g} s: the package was not left at rest")
    samples = rest - touching
    if samples < MIN_CONTACT_SAMPLES:
        raise RigError(f"{name}: {samples} samples in a {1000 * (times[rest] - times[touching]):.0f} ms contact at {rate:.0f} samples a second: too slow to see the peak ({MIN_CONTACT_SAMPLES} or more are needed)")
    impulse = 0.0
    # From the touch to rest the package goes from its descent speed to none: m v is the integral of (force - weight)
    # over exactly that time (trapezoids).
    for index in range(touching, rest):
        impulse += 0.5 * ((forces[index] - weight) + (forces[index + 1] - weight)) * (times[index + 1] - times[index])
    return Trial(name, descent_mps, impulse, max(forces[touching : rest + 1]), weight, times[rest] - times[touching], samples, rate)


def slope(pairs: Sequence[Tuple[float, float]]) -> float:
    """y = k x through zero, by least squares."""
    return sum(x * y for x, y in pairs) / sum(x * x for x, _y in pairs)


def summary(trials: Sequence[Trial], package_kg: Optional[float] = None, limit_ns: Optional[float] = None, limit_peak_n: Optional[float] = None, at_mps: float = 0.12) -> Dict[str, Any]:
    """The fits across the trials, what the configured set-down speed gives, and the fastest descent under a limit."""
    if not trials:
        raise RigError("no trials")
    impulse_per_mps = slope([(trial.descent_mps, trial.impulse_ns) for trial in trials])
    over_per_mps = slope([(trial.descent_mps, trial.peak_n - trial.weight_n) for trial in trials])
    weight = median(trial.weight_n for trial in trials)
    worst_impulse = max(trial.impulse_ns / trial.descent_mps for trial in trials)
    worst_over = max((trial.peak_n - trial.weight_n) / trial.descent_mps for trial in trials)
    out: Dict[str, Any] = {"trials": len(trials), "weight_n": weight, "resting_kg": weight / G, "impulse_per_mps": impulse_per_mps, "peak_over_weight_per_mps": over_per_mps,
                           "at_mps": at_mps, "impulse_at": impulse_per_mps * at_mps, "peak_at": weight + over_per_mps * at_mps, "warnings": [], "max_descent_mps": None}  # fmt: skip
    if len({round(trial.descent_mps, 3) for trial in trials}) < 3:
        out["warnings"].append("fewer than three descent speeds: a straight line through them is not yet a measurement")
    for trial in trials:
        mass = package_kg if package_kg is not None else trial.weight_n / G
        if abs(trial.momentum_kg - mass) > MASS_TOLERANCE * mass:
            out["warnings"].append(f"{trial.name}: the impulse is that of {trial.momentum_kg:.2f} kg at {trial.descent_mps:g} m/s, and the package is {mass:.2f} kg: "
                                   "not a clean set-down (the line still carrying, the vehicle pushing, or the speed is not what was flown)")  # fmt: skip
    if package_kg is not None and abs(weight / G - package_kg) > 0.1 * package_kg:
        out["warnings"].append(f"the plate reads {weight / G:.2f} kg at rest, and the package is {package_kg:.2f} kg: zero the plate, or check the load cell's calibration")
    limits = []
    out["limit_given"] = limit_ns is not None or limit_peak_n is not None
    gentle = "no trial {what} (the weight came on without a knock): at the speeds tried, the {kind} limit bounds no descent speed"
    if limit_ns is not None:
        if worst_impulse > 0.0:
            limits.append(("impulse", MARGIN * limit_ns / worst_impulse, f"{limit_ns:g} N s"))
        else:
            out["warnings"].append(gentle.format(what="gave the plate more than the package's own weight coming to rest", kind="impulse"))
        out["impulse_share_at"] = max(0.0, out["impulse_at"]) / limit_ns
    if limit_peak_n is not None:
        if limit_peak_n <= weight:
            raise RigError(f"the peak limit {limit_peak_n:g} N is no more than the package's own weight on the plate ({weight:.2f} N)")
        if worst_over > 0.0:
            limits.append(("peak force", MARGIN * (limit_peak_n - weight) / worst_over, f"{limit_peak_n:g} N"))
        else:
            out["warnings"].append(gentle.format(what="pressed the plate harder than the package weighs", kind="peak force"))
        out["peak_share_at"] = max(weight, out["peak_at"]) / limit_peak_n
    if limits:
        kind, speed, text = min(limits, key=lambda item: item[1])
        out.update(max_descent_mps=speed, limited_by=kind, limit=text)
    return out


def report(trials: Sequence[Trial], found: Dict[str, Any]) -> str:
    lines = ["| Trial | Descent (m/s) | Impulse (N s) | Peak (N) | Peak / weight | Contact (ms) | Momentum's mass (kg) | Samples in contact |", "| --- | --- | --- | --- | --- | --- | --- | --- |"]
    lines += [f"| {t.name} | {t.descent_mps:g} | {t.impulse_ns:.4f} | {t.peak_n:.2f} | {t.peak_over_weight:.2f} | {1000 * t.contact_s:.0f} | {t.momentum_kg:.3f} | {t.samples} at {t.rate_hz:.0f} Hz |" for t in trials]
    lines += ["", f"The package rests at {found['weight_n']:.2f} N ({found['resting_kg']:.3f} kg). Fitted through zero over {found['trials']} trial(s): "
              f"impulse {found['impulse_per_mps']:.3f} N s per m/s; peak force over the weight {found['peak_over_weight_per_mps']:.1f} N per m/s.",
              f"At {found['at_mps']:g} m/s: impulse {found['impulse_at']:.4f} N s, peak {found['peak_at']:.2f} N"
              + (f" ({found['impulse_share_at']:.0%} of the impulse limit)" if "impulse_share_at" in found else "") + (f" ({found['peak_share_at']:.0%} of the peak limit)" if "peak_share_at" in found else "") + "."]  # fmt: skip
    if found["max_descent_mps"] is None and found.get("limit_given"):
        lines.append(f"The limit bounds no descent speed at the speeds tried: nothing in these trials came from the speed. The set-down at {found['at_mps']:g} m/s is under it as far as these trials show; try faster ones to find where it binds.")
    elif found["max_descent_mps"] is None:
        lines.append("No limit was given (--limit-ns or --limit-peak-n): the descent speed cannot be set against one yet.")
    else:
        verdict = "is under it" if found["at_mps"] <= found["max_descent_mps"] else "IS OVER IT: slow the set-down"
        lines.append(f"Fastest descent under the limit ({found['limit']}, by {found['limited_by']}, the worst trial, {1 - MARGIN:.0%} margin): {found['max_descent_mps']:.3f} m/s. The set-down at {found['at_mps']:g} m/s {verdict}.")
    lines += [f"WARNING: {warning}" for warning in found["warnings"]]
    return "\n".join(lines) + "\n"


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="The impulse rig's analysis: a delivery's contact impulse and the descent speed under a limit.")
    parser.add_argument("--trial", action="append", default=[], metavar="FILE:SPEED", help="a recording and the descent speed it was set down at, m/s; repeat")
    parser.add_argument("--package-kg", type=float, help="the package's mass, to check the plate and the set-downs against")
    parser.add_argument("--limit-ns", type=float, help="the limit as an impulse, N s")
    parser.add_argument("--limit-peak-n", type=float, help="the limit as a peak force, N")
    parser.add_argument("--at", type=float, default=0.12, help="the set-down speed to report at (default 0.12 m/s, the delivery's)")
    args = parser.parse_args(argv)
    try:
        if not args.trial:
            raise RigError("give at least one --trial FILE:SPEED")
        trials = []
        for item in args.trial:
            name, _colon, speed = item.rpartition(":")
            try:
                descent = float(speed)
            except ValueError:
                raise RigError(f"--trial {item}: expected FILE:SPEED, the speed in m/s") from None
            times, forces = read_csv(Path(name))
            trials.append(analyse(Path(name).name, times, forces, descent))
        found = summary(trials, args.package_kg, args.limit_ns, args.limit_peak_n, args.at)
    except RigError as problem:
        print(f"impulse_rig: {problem}", file=sys.stderr)
        return 2
    print(report(trials, found), end="")
    over = found["max_descent_mps"] is not None and args.at > found["max_descent_mps"]
    return 1 if over else 0


if __name__ == "__main__":
    sys.exit(main())
