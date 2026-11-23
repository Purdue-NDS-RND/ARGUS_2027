#!/usr/bin/env python3
"""The LED latency probe (PLT-05): how late are a camera's capture stamps? No ROS imports.

An LED on a Jetson GPIO pin sits in the camera's view. The probe blinks it at
moments that are not tied to the frames, and for every blink finds the first
frame in which it is lit. If the stamps were the middle of each exposure, that
frame's stamp would follow the LED by anything from nothing to one frame
period, evenly: half a period on average. What the average exceeds that by is
how late the stamps are, which is the mode's `capture_offset_s` (FND-01
subtracts it from every capture time).

  ros2 run argus_camera latency_probe --mode imx519_720p60 --pin 7 --csi-backend gstreamer
  ros2 run argus_camera latency_probe --analyse probe.json        # a saved run again

The probe runs on stamps that already have the mode's configured offset taken
off, so what it prints is the residual: within 5 ms it says PASS, otherwise it
says what to set `capture_offset_s` to. Run it again after setting it.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, List, Optional, Sequence, Tuple

import numpy as np

RESIDUAL_LIMIT_S = 0.005  # FND-01's bench line: LED probe residual within 5 ms after the offset
MIN_BLINKS = 30
MIN_GAP_S = 0.5  # between blinks: far more than any stamp error, so a blink and its frame cannot be mispaired
MIN_CONTRAST = 40.0  # grey levels between the LED lit and dark, in its part of the picture
Roi = Tuple[int, int, int, int]  # x0, y0, x1, y1


class ProbeError(RuntimeError):
    """The probe cannot measure: no GPIO library, or the LED is not in the picture."""


@dataclass(frozen=True)
class LedOffset:
    blinks: int  # used in the estimate
    rejected: int  # blinks with no frame of their own, or whose first lit frame came a frame or more late (dropped frames)
    offset_s: float  # how late the stamps are: what capture_offset_s should grow by
    stderr_s: float
    spread_s: float  # between the earliest and latest first-lit frame after its blink: about one frame period
    valid: bool
    problems: List[str] = field(default_factory=list)


def led_level(image: np.ndarray, roi: Roi) -> float:
    x0, y0, x1, y1 = roi
    return float(np.mean(image[y0:y1, x0:x1]))


def find_led_roi(dark: np.ndarray, lit: np.ndarray, min_contrast: float = MIN_CONTRAST) -> Optional[Roi]:
    """The box of the pixels the LED brightens, from one frame with it off and one with it on. None if it is not in view."""
    difference = lit.astype(np.float32) - dark.astype(np.float32)
    if difference.ndim == 3:
        difference = difference.max(axis=2)
    peak = float(difference.max())
    if peak < min_contrast:
        return None
    rows, columns = np.nonzero(difference >= peak / 2.0)
    return int(columns.min()), int(rows.min()), int(columns.max()) + 1, int(rows.max()) + 1


def next_gap_s(period_s: float, rng: random.Random, min_gap_s: float = MIN_GAP_S) -> float:
    """How long to wait before the next blink: the minimum plus up to three frame periods, so every phase of a frame is hit."""
    return min_gap_s + rng.uniform(0.0, 3.0 * period_s)


def _first_lit_stamps(stamps_ns: Sequence[int], levels: Sequence[float]) -> List[int]:
    """The stamp of each frame that is lit when the frame before it was dark."""
    if not levels:
        return []
    low, high = float(np.percentile(levels, 5)), float(np.max(levels))
    if high - low < MIN_CONTRAST / 2.0:
        return []
    threshold = (low + high) / 2.0
    return [stamps_ns[i] for i in range(1, len(levels)) if levels[i] >= threshold > levels[i - 1]]


def led_offset(
    on_ns: Sequence[int],
    stamps_ns: Sequence[int],
    levels: Sequence[float],
    period_s: float,
    min_blinks: int = MIN_BLINKS,
    max_error_s: float = MIN_GAP_S / 2.0,
) -> LedOffset:
    """The stamps' lateness from a run: LED-on times, and every frame's stamp and LED level, all on one clock.

    `max_error_s` is the largest stamp error looked for; it must be under half
    the gap between blinks, so that each blink has one candidate frame.
    """
    lit = sorted(_first_lit_stamps(stamps_ns, levels))
    window = round(max_error_s * 1e9)
    delays: List[int] = []
    for on in on_ns:
        near = [stamp for stamp in lit if abs(stamp - on) <= window]
        if near:
            delays.append(min(near, key=lambda stamp: abs(stamp - on)) - on)
    unpaired = len(on_ns) - len(delays)
    period_ns = period_s * 1e9
    used: List[int] = []
    if delays:
        # The first lit frame after a dropped one is a period or more late. The honest
        # delays fill one period from the smallest up; anything beyond that is set aside.
        earliest = float(np.percentile(delays, 5))
        used = [delay for delay in delays if delay < earliest + 1.2 * period_ns]
    rejected = unpaired + len(delays) - len(used)
    problems: List[str] = []
    if len(used) < min_blinks:
        problems.append(f"{len(used)} usable blinks of {len(on_ns)}: {min_blinks} are needed (is the LED in the picture, and bright against it?)")
    spread = (max(used) - min(used)) / 1e9 if used else 0.0
    if used and spread < 0.6 * period_s:
        problems.append(f"the first lit frames span {spread * 1000.0:.1f} ms of a {period_s * 1000.0:.1f} ms frame period: the blinks were locked to the frames")
    if delays and rejected > len(delays) / 2:
        problems.append(f"{rejected} of {len(on_ns)} blinks set aside: the camera is dropping frames")
    offset = (sum(used) / len(used) - period_ns / 2.0) / 1e9 if used else 0.0
    stderr = period_s / math.sqrt(12.0 * len(used)) if used else float("inf")
    return LedOffset(blinks=len(used), rejected=rejected, offset_s=offset, stderr_s=stderr, spread_s=spread, valid=not problems, problems=problems)


def judge(result: LedOffset, configured_offset_s: float, limit_s: float = RESIDUAL_LIMIT_S) -> Tuple[bool, str]:
    """PASS when the stamps, with the configured offset already off, are within the limit; else what to set."""
    if not result.valid:
        return False, "not a measurement: " + "; ".join(result.problems)
    residual_ms = result.offset_s * 1000.0
    if abs(result.offset_s) <= limit_s:
        return True, f"PASS: residual {residual_ms:+.1f} ms (limit {limit_s * 1000.0:.0f} ms) with capture_offset_s {configured_offset_s:.3f}"
    wanted = configured_offset_s + result.offset_s
    if wanted < 0.0:
        # modes.yaml refuses a negative offset, and a camera cannot stamp a frame before it exposed it.
        return False, (
            f"residual {residual_ms:+.1f} ms is over {limit_s * 1000.0:.0f} ms, and early: with capture_offset_s {configured_offset_s:.3f} the stamps are "
            f"{-wanted * 1000.0:.1f} ms before the LED. No offset corrects that: the configured offset is too large, or the frames are not stamped on the clock the LED is timed on"
        )
    return False, (
        f"residual {residual_ms:+.1f} ms is over {limit_s * 1000.0:.0f} ms: set the mode's capture_offset_s: {wanted:.3f} "
        f"(now {configured_offset_s:.3f}) and run the probe again"
    )


# ---------------------------------------------------------------- the run


def load_gpio() -> Any:
    """Jetson.GPIO, or None where it is not installed (it comes with JetPack)."""
    try:
        import Jetson.GPIO as gpio  # type: ignore[import-not-found]
    except Exception:  # noqa: BLE001  (ImportError, or its own error off a Jetson)
        return None
    return gpio


class GpioLed:
    """An LED on a pin of the Jetson's 40-pin header (board numbering), on when the pin is high."""

    def __init__(self, pin: int) -> None:
        self._gpio = load_gpio()
        if self._gpio is None:
            raise ProbeError("Jetson.GPIO is not installed, or this is not a Jetson: the LED cannot be driven")
        self._pin = pin
        self._gpio.setmode(self._gpio.BOARD)
        self._gpio.setup(pin, self._gpio.OUT, initial=self._gpio.LOW)

    def on(self) -> None:
        self._gpio.output(self._pin, self._gpio.HIGH)

    def off(self) -> None:
        self._gpio.output(self._pin, self._gpio.LOW)

    def close(self) -> None:
        self.off()
        self._gpio.cleanup(self._pin)


class _Blinker(threading.Thread):
    """Blinks the LED on its own thread, so the blinks are not tied to when frames are read."""

    def __init__(self, led: Any, clock: Callable[[], int], period_s: float, blinks: int, on_s: float, min_gap_s: float, seed: int) -> None:
        super().__init__(daemon=True)
        self._led, self._clock, self._period_s = led, clock, period_s
        self._blinks, self._on_s, self._min_gap_s = blinks, on_s, min_gap_s
        self._rng = random.Random(seed)
        self.on_ns: List[int] = []
        self.widest_bracket_ns = 0
        self.stop = threading.Event()

    def run(self) -> None:
        try:
            for _ in range(self._blinks):
                if self.stop.wait(next_gap_s(self._period_s, self._rng, self._min_gap_s)):
                    return
                before = self._clock()
                self._led.on()
                after = self._clock()
                self.on_ns.append((before + after) // 2)
                self.widest_bracket_ns = max(self.widest_bracket_ns, after - before)
                if self.stop.wait(self._on_s):
                    return
                self._led.off()
        finally:
            self._led.off()


def _frame_after(source: Any, settle_s: float) -> np.ndarray:
    """A frame taken after the picture has had `settle_s` to show the LED's new state."""
    end = time.monotonic() + settle_s
    frame = None
    while time.monotonic() < end or frame is None:
        frame = source.read() or frame
    return frame.image


def run_probe(
    source: Any,
    led: Any,
    clock: Callable[[], int],
    period_s: float,
    blinks: int = 200,
    on_s: Optional[float] = None,
    min_gap_s: float = MIN_GAP_S,
    settle_s: float = 0.5,
    seed: int = 0,
) -> dict:
    """Blink `led` and read `source` until the blinks are done. `clock` must be the clock the source stamps frames with.

    Returns what led_offset needs, as plain lists (it can be saved as JSON).
    """
    on_s = on_s if on_s is not None else max(0.1, 6.0 * period_s)
    try:
        led.off()
        dark = _frame_after(source, settle_s)
        led.on()
        lit = _frame_after(source, settle_s)
        led.off()
        roi = find_led_roi(dark, lit)
        if roi is None:
            raise ProbeError("the LED does not show in the picture: put it in the camera's view, shaded from the sun, and check the pin")
        _frame_after(source, settle_s)
        blinker = _Blinker(led, clock, period_s, blinks, on_s, min_gap_s, seed)
        stamps: List[int] = []
        levels: List[float] = []
        blinker.start()
        try:
            while blinker.is_alive():
                frame = source.read()
                if frame is not None:
                    stamps.append(int(frame.capture_ns))
                    levels.append(led_level(frame.image, roi))
            for _ in range(8):  # the frames still in the pipeline when the last blink ended
                frame = source.read()
                if frame is not None:
                    stamps.append(int(frame.capture_ns))
                    levels.append(led_level(frame.image, roi))
        finally:
            blinker.stop.set()
            blinker.join()
    finally:
        led.off()
    return {
        "blinks": len(blinker.on_ns),
        "period_s": period_s,
        "roi": list(roi),
        "on_ns": blinker.on_ns,
        "stamps_ns": stamps,
        "levels": [round(level, 2) for level in levels],
        "gpio_bracket_ms": round(blinker.widest_bracket_ns / 1e6, 3),
    }


def report(record: dict) -> Tuple[bool, List[str]]:
    result = led_offset(record["on_ns"], record["stamps_ns"], record["levels"], record["period_s"])
    configured = float(record.get("configured_offset_s", 0.0))
    passed, line = judge(result, configured)
    lines = [
        f"{record.get('mode', 'camera')}: {result.blinks} blinks used, {result.rejected} set aside, {len(record['stamps_ns'])} frames",
        f"  stamps are {result.offset_s * 1000.0:+.1f} ms late (standard error {result.stderr_s * 1000.0:.1f} ms); "
        f"first lit frames span {result.spread_s * 1000.0:.1f} ms of a {record['period_s'] * 1000.0:.1f} ms period",
    ]
    if "gpio_bracket_ms" in record:
        lines.append(f"  switching the LED took at most {record['gpio_bracket_ms']:.2f} ms")
    if "stamp_source" in record:
        lines.append(f"  frames were stamped from: {record['stamp_source']}" + ("" if record["stamp_source"] == "pts" else " (not the buffer's own time: the offset then includes the wait to be read, which varies)"))
    lines.append("  " + line)
    lines.append("latency probe: PASS" if passed else "latency probe: FAIL")
    return passed, lines


def main(argv: Optional[List[str]] = None) -> int:
    from .bench import default_config_dir
    from .frame_source import open_source
    from .modes import get_mode, load_modes

    parser = argparse.ArgumentParser(description="LED latency probe: how late are a camera mode's capture stamps?")
    parser.add_argument("--mode", help="sensor mode from modes.yaml")
    parser.add_argument("--pin", type=int, help="the LED's pin on the 40-pin header (board numbering)")
    parser.add_argument("--blinks", type=int, default=200)
    parser.add_argument("--csi-backend", choices=("opencv", "gstreamer"), default="gstreamer", help="gstreamer stamps frames from their PTS, which is what flies")
    parser.add_argument("--config", type=Path, default=default_config_dir(), help="folder with modes.yaml")
    parser.add_argument("--out", type=Path, help="save the run as JSON (keep it with the result in docs/results/)")
    parser.add_argument("--analyse", type=Path, help="analyse a saved run instead of making one")
    args = parser.parse_args(argv)
    if args.analyse:
        record = json.loads(args.analyse.read_text())
    else:
        if not args.mode or args.pin is None:
            parser.error("--mode and --pin are needed (or --analyse <file>)")
        mode = get_mode(load_modes(args.config / "modes.yaml"), args.mode)
        try:
            led = GpioLed(args.pin)
        except ProbeError as problem:
            print(f"latency probe: {problem}")
            return 1
        source = open_source(mode, clock=time.time_ns, csi_backend=args.csi_backend)
        try:
            with source:
                first = source.read()
                record = run_probe(source, led, clock=time.time_ns, period_s=1.0 / mode.fps, blinks=args.blinks)
        except ProbeError as problem:
            print(f"latency probe: {problem}")
            return 1
        finally:
            led.close()
        record.update(mode=mode.name, configured_offset_s=mode.capture_offset_s, stamp_source=first.stamp_source if first else "none")
        if args.out:
            args.out.write_text(json.dumps(record))
    passed, lines = report(record)
    print("\n".join(lines))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
