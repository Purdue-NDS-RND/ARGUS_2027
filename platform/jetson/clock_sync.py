#!/usr/bin/env python3
"""Is this machine's clock on the ground laptop's? (PLT-02)

Chrony keeps the Jetson on the ground laptop's clock so the two machines' logs
line up. This asks chrony how that is going and gives one line and an exit code:

    python3 platform/jetson/clock_sync.py --source 192.168.10.2
    CLOCK OK: following 192.168.10.2 (stratum 9), 0.4 ms off (limit 20.0 ms)

Exit 0 when the clock is synchronised, to the named source when one is given,
and within the limit; 1 otherwise, including when chrony cannot be asked.
Standard library only.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from dataclasses import dataclass

# How far apart the two clocks may be for logs to "line up": one period of the
# fastest stream in them (attitude at 50 Hz), and far above
# what chrony holds on a local link.
DEFAULT_MAX_OFFSET_S = 0.020


@dataclass(frozen=True)
class Tracking:
    source: str  # the address of the clock being followed, "" when there is none
    stratum: int
    offset_s: float  # system clock against the source, by chrony's estimate
    rms_offset_s: float
    leap: str  # "Normal", or "Not synchronised"


def parse_tracking(text: str) -> Tracking:
    """One line of `chronyc -n -c tracking`."""
    lines = [line for line in text.strip().splitlines() if line.strip()]
    if not lines:
        raise ValueError("chronyc gave no output")
    fields = lines[-1].split(",")
    if len(fields) < 14:
        raise ValueError(f"not chronyc tracking output: {lines[-1]!r}")
    try:
        return Tracking(source=fields[1], stratum=int(fields[2]), offset_s=float(fields[4]), rms_offset_s=float(fields[6]), leap=fields[13].strip())
    except ValueError as exc:
        raise ValueError(f"not chronyc tracking output: {lines[-1]!r}") from exc


def verdict(tracking: Tracking, source: str | None, max_offset_s: float = DEFAULT_MAX_OFFSET_S) -> tuple[bool, str]:
    if tracking.leap != "Normal" or not tracking.source or not 0 < tracking.stratum < 16:
        return False, "the clock is not synchronised to anything"
    if source and tracking.source != source:
        return False, f"following {tracking.source}, not the ground laptop {source}"
    off_ms = abs(tracking.offset_s) * 1000.0
    limit_ms = max_offset_s * 1000.0
    described = f"following {tracking.source} (stratum {tracking.stratum}), {off_ms:.1f} ms off (limit {limit_ms:.1f} ms)"
    return abs(tracking.offset_s) <= max_offset_s, described


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--source", help="the ground laptop's address; without it any synchronised source passes")
    parser.add_argument("--max-offset-ms", type=float, default=DEFAULT_MAX_OFFSET_S * 1000.0)
    parser.add_argument("--chronyc", default="chronyc", help="the chronyc command (tests pass a stand-in)")
    args = parser.parse_args(argv)
    try:
        asked = subprocess.run([args.chronyc, "-n", "-c", "tracking"], capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired) as exc:
        print(f"CLOCK NOT OK: chronyc could not be run ({exc})")
        return 1
    if asked.returncode != 0:
        print(f"CLOCK NOT OK: chronyc failed: {(asked.stdout + asked.stderr).strip()}")
        return 1
    try:
        tracking = parse_tracking(asked.stdout)
    except ValueError as exc:
        print(f"CLOCK NOT OK: {exc}")
        return 1
    ok, reason = verdict(tracking, args.source, args.max_offset_ms / 1000.0)
    print(f"CLOCK {'OK' if ok else 'NOT OK'}: {reason}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
