#!/usr/bin/env python3
"""Precision and coverage of the digit reader (LOC-6), and what a crop costs. No ROS.

  python3 tools/digit_reader_eval.py --n 1000 --seed 1                  # synthetic 2027 markers, the template stand-in
  python3 tools/digit_reader_eval.py --n 500 --mpp 0.0136               # 44 px markers: the sim's 1280 x 720 camera from 35 ft
  python3 tools/digit_reader_eval.py --n 500 --layout plate_2026        # last year's plate, which has a cue for 6 against 9
  python3 tools/digit_reader_eval.py --crops <folder> --model digits.onnx   # the spec's test: labelled field crops, the trained classifier

Synthetic (the default): it draws numbered markers with tools/synth_targets.py
(a random digit, any rotation, up to 12 degrees of tilt each way, blur, noise,
brightness, on a noisy field), cuts the crop the pipeline would (the marker's
box and 15% a side), and reads it with the reader the localisation node uses.

Labelled crops (`--crops`): a folder with `labels.csv`, one `file,digit` line
for each crop image in it (a header line is skipped). The digit is 0 to 9, or
`unreadable` as DAT-08 labels a crop nobody can read: such a crop is not one
the reader is asked to answer (it is left out of coverage), and an answer on
it is a wrong answer. This is how the held-out field day is scored once it
exists (DAT-08), and how the trained classifier is scored on anything:
`--model` is its ONNX file, run on the CPU. The crops are read with no lens
model: cut them from undistorted frames, or take the score as the reader's
without that step.

It counts:

  coverage   the share of crops the reader answered
  precision  the share of its answers that were right

and, apart, the 6s and 9s it left as "a 6 or a 9" (a marker with no cue for
which way up), and the time a crop took (median and 95th percentile), which is
the bench number on the Jetson.

The spec's line is at least 99% precision while answering at least 90%, on a
held-out field day with the trained classifier. A synthetic run with the
stand-in is not that: it shows the reader's own steps at a marker size.

Exit code 0 when both lines are met, 1 otherwise, 2 for input that cannot be read.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from collections import Counter
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Iterator, List, Optional, Tuple

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
for package in ("argus_localisation", "drone_mission_core", "argus_camera"):
    sys.path.insert(0, str(REPO / "src" / package))

MIN_PRECISION, MIN_COVERAGE = 0.99, 0.90
CROP_MARGIN = 0.15  # the pipeline's: this share of the box added on each side
DEFAULT_MPP = (0.0042, 0.0052)  # metres a pixel: the 4K frame from 30 to 37 ft
LABELS_FILE = "labels.csv"
UNREADABLE = "unreadable"  # DAT-08's label for a crop nobody can read


def crop_of(image, corners_px, margin: float = CROP_MARGIN):
    """The marker's box and its margin, cut from the picture as the pipeline cuts a crop."""
    import numpy as np

    corners = np.asarray(corners_px, dtype=float)
    low, high = corners.min(axis=0), corners.max(axis=0)
    pad = margin * (high - low)
    x0, y0 = (max(0, int(round(v))) for v in low - pad)
    x1, y1 = min(image.shape[1], int(round(high[0] + pad[0]))), min(image.shape[0], int(round(high[1] + pad[1])))
    return SimpleNamespace(image=image[y0:y1, x0:x1], origin=(x0, y0))


def synthetic_crops(count: int, seed: int, mpp, layout_name: str, max_tilt_deg: float = 12.0, max_blur_px: float = 1.0, max_noise: float = 4.0) -> Iterator[Tuple[Any, int]]:
    """(crop, digit) for `count` drawn markers of the layout's kind."""
    import numpy as np
    import synth_targets

    low, high = (float(mpp), float(mpp)) if isinstance(mpp, (int, float)) else (float(mpp[0]), float(mpp[1]))
    rng = np.random.default_rng(seed)
    for index in range(count):
        digit = int(rng.integers(10))
        target = synth_targets.NumberedMarker(digit=digit) if layout_name == "plate_2026" else synth_targets.SurveyMarker(digit=digit)
        scale = float(rng.uniform(low, high))
        size = int(target.size_m / scale * 2.0) + 40  # room for the marker at any rotation, and field round it
        options = synth_targets.RenderOptions(
            width=size,
            height=size,
            mpp=scale,
            rotation_deg=float(rng.uniform(0.0, 360.0)),
            tilt_deg=(float(rng.uniform(-max_tilt_deg, max_tilt_deg)), float(rng.uniform(-max_tilt_deg, max_tilt_deg))),
            blur_sigma_px=float(rng.uniform(0.0, max_blur_px)),
            noise_sigma=float(rng.uniform(0.0, max_noise)),
            brightness=float(rng.uniform(0.75, 1.2)),
            background="noise",
            supersample=2,
        )
        image, truth = synth_targets.render(target, options, seed=seed * 100003 + index)
        yield crop_of(image, truth["corners_px"]), digit


def labelled_crops(folder: Path) -> Iterator[Tuple[Any, Optional[int]]]:
    """(crop, digit) for each line of the folder's labels.csv; the digit is None for a crop labelled unreadable.
    Raises ValueError for a set that cannot be read."""
    import cv2

    labels = Path(folder) / LABELS_FILE
    if not labels.is_file():
        raise ValueError(f"{labels} is not a file: a crop set is a folder of crop images with a {LABELS_FILE} of file,digit lines")
    with labels.open(newline="") as lines:
        for number, row in enumerate(csv.reader(lines), start=1):
            if not row or not row[0].strip() or row[0].lstrip().startswith("#"):
                continue
            label = row[1].strip().lower() if len(row) >= 2 else ""
            if label != UNREADABLE and not label.isdigit():
                if number == 1:
                    continue  # a header
                raise ValueError(f"{labels}, line {number}: expected file,digit (0 to 9, or {UNREADABLE}), got {row!r}")
            digit = None if label == UNREADABLE else int(label)
            if digit is not None and digit > 9:
                raise ValueError(f"{labels}, line {number}: a digit is 0 to 9, got {digit}")
            image = cv2.imread(str(Path(folder) / row[0].strip()), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError(f"{labels}, line {number}: {row[0].strip()} is not an image that can be read")
            yield SimpleNamespace(image=image, origin=(0, 0)), digit


def make_reader(layout_name: str = "x_2027", model: Optional[Path] = None, min_confidence: float = 0.9):
    """The reader the node makes: the ONNX classifier at `model`, or with none the template stand-in."""
    from argus_localisation import digit_reader

    layout = digit_reader.LAYOUTS[layout_name]
    classifier = digit_reader.load_onnx(model) if model else digit_reader.TemplateDigitClassifier(layout)
    return digit_reader.DigitReader(classifier, min_confidence=min_confidence, layout=layout)


def score(reader: Any, crops: Iterator[Tuple[Any, int]]) -> dict:
    """Read every crop and count."""
    import numpy as np

    count = right = wrong = six_nine = six_nine_left = others_answered = unreadable = unreadable_answered = 0
    reasons: Counter = Counter()
    confusions: Counter = Counter()
    took_ms = []
    for crop, digit in crops:
        started = time.perf_counter()
        reading = reader.read(crop)
        took_ms.append((time.perf_counter() - started) * 1000.0)
        if digit is None:  # a crop nobody can read: not answering is right, and an answer is a wrong one
            unreadable += 1
            if reading.digit is not None:
                unreadable_answered += 1
                wrong += 1
                confusions[f"{UNREADABLE} read as {reading.digit}"] += 1
            continue
        count += 1
        six_nine += digit in (6, 9)
        if reading.digit is None:
            kind = _kind(reading.reason)
            reasons[kind] += 1
            six_nine_left += kind == "a 6 or a 9" and digit in (6, 9)
        else:
            others_answered += digit not in (6, 9)
            if reading.digit == digit:
                right += 1
            else:
                wrong += 1
                confusions[f"{digit} read as {reading.digit}"] += 1
    answered = right + wrong  # every answer, those on unreadable crops among them
    return {
        "count": count,  # the crops with a digit to read
        "answered": answered,
        "right": right,
        "wrong": wrong,
        "unreadable": unreadable,  # crops labelled unreadable, beside `count`
        "unreadable_answered": unreadable_answered,  # of them, answered: each a wrong answer
        "coverage": (answered - unreadable_answered) / count if count else 0.0,
        "precision": right / answered if answered else 0.0,
        "six_nine": six_nine,  # crops whose digit is a 6 or a 9
        "six_nine_left": six_nine_left,  # of them, left as "a 6 or a 9"
        "coverage_but_six_nine": others_answered / (count - six_nine) if count > six_nine else None,  # the share of the other eight digits' crops answered
        "abstained": dict(reasons.most_common()),
        "confusions": dict(confusions.most_common()),
        "classifier": reader.classifier.name,
        "layout": reader.layout.name,
        "ms_a_crop": {"median": round(float(np.median(took_ms)), 2), "p95": round(float(np.percentile(took_ms, 95)), 2)} if took_ms else None,
    }


def _kind(reason: str) -> str:
    """An abstention's reason without its numbers."""
    for kind in ("a 6 or a 9", "no marker outline", "too small or not a square", "no black triangles", "no black and white", "disagree", "one way up", "digit's side", "probability"):
        if kind in reason:
            return kind
    return reason


def verdict(report: dict) -> bool:
    return report["precision"] >= MIN_PRECISION and report["coverage"] >= MIN_COVERAGE


def summary(report: dict) -> List[str]:
    lines = [
        f"{'PASS' if verdict(report) else 'FAIL'}: precision {100.0 * report['precision']:.1f}% ({report['right']} of {report['answered']} answers right), "
        f"coverage {100.0 * report['coverage']:.1f}% ({report['answered'] - report['unreadable_answered']} of {report['count']} crops answered); lines {100 * MIN_PRECISION:.0f}% and {100 * MIN_COVERAGE:.0f}%",
        f"source: {report.get('source', '')}; marker layout {report['layout']}; classifier: {report['classifier']}",
    ]
    if report["six_nine_left"]:
        lines.append(
            f"6 or 9: {report['six_nine_left']} of {report['six_nine']} such crops left as 'a 6 or a 9' (this layout does not say which way up); "
            f"of the other {report['count'] - report['six_nine']} crops, {100.0 * report['coverage_but_six_nine']:.1f}% answered"
        )
    if report["unreadable"]:
        lines.append(f"unreadable: {report['unreadable']} crops labelled so, {report['unreadable_answered']} of them answered (each a wrong answer)")
    if report["abstained"]:
        lines.append("abstained: " + ", ".join(f"{count} {kind}" for kind, count in report["abstained"].items()))
    if report["confusions"]:
        lines.append("wrong: " + ", ".join(f"{count} x {kind}" for kind, count in report["confusions"].items()))
    if report["ms_a_crop"]:
        lines.append(f"time a crop: median {report['ms_a_crop']['median']:g} ms, 95th percentile {report['ms_a_crop']['p95']:g} ms (this machine)")
    return lines


def evaluate(count: int = 1000, seed: int = 1, mpp=DEFAULT_MPP, layout: str = "x_2027", model: Optional[Path] = None, crops: Optional[Path] = None, min_confidence: float = 0.9, max_tilt_deg: float = 12.0, max_blur_px: float = 1.0) -> dict:
    reader = make_reader(layout, model, min_confidence)
    if crops is not None:
        report = score(reader, labelled_crops(Path(crops)))
        report["source"] = f"labelled crops ({Path(crops).name})"
        return report
    report = score(reader, synthetic_crops(count, seed, mpp, layout, max_tilt_deg, max_blur_px))
    low, high = (mpp, mpp) if isinstance(mpp, (int, float)) else (mpp[0], mpp[1])
    size_m = 0.61 if layout == "plate_2026" else 0.6
    report["source"] = f"synthetic, seed {seed}, markers {size_m / high:.0f} to {size_m / low:.0f} px across"
    return report


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Precision and coverage of the digit reader, on synthetic markers or on a labelled crop set.")
    parser.add_argument("--n", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--mpp", type=float, nargs="+", default=list(DEFAULT_MPP), help="metres a pixel: one value, or a range (default: the 4K frame from 30 to 37 ft)")
    parser.add_argument("--layout", choices=["x_2027", "plate_2026"], default="x_2027", help="the marker's layout: this year's (digit centred on the X) or last year's plate")
    parser.add_argument("--model", type=Path, help="the trained classifier's ONNX file (default: the template stand-in)")
    parser.add_argument("--crops", type=Path, help=f"a folder of labelled crops with a {LABELS_FILE} of file,digit lines, instead of synthetic markers")
    parser.add_argument("--max-tilt", type=float, default=12.0)
    parser.add_argument("--max-blur", type=float, default=1.0)
    parser.add_argument("--min-confidence", type=float, default=0.9)
    parser.add_argument("--report", type=Path, help="also write the numbers here as JSON")
    args = parser.parse_args(argv)
    mpp = args.mpp[0] if len(args.mpp) == 1 else tuple(args.mpp[:2])
    try:
        report = evaluate(args.n, args.seed, mpp, args.layout, args.model, args.crops, args.min_confidence, args.max_tilt, args.max_blur)
    except (ValueError, RuntimeError) as error:
        print(f"digit_reader_eval: {error}", file=sys.stderr)
        return 2
    if not report["count"]:
        print("digit_reader_eval: no crops to read", file=sys.stderr)
        return 2
    for line in summary(report):
        print(line)
    if args.report is not None:
        args.report.write_text(json.dumps(report, indent=2) + "\n")
    return 0 if verdict(report) else 1


if __name__ == "__main__":
    sys.exit(main())
