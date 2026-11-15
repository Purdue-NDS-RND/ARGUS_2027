#!/usr/bin/env python3
"""Score one sim run folder and write score.json. No ROS.

A run folder holds:

  judge.yaml    mission: <name>; optional settings: {...} (see score.py),
                vehicle_ground_offset_m, and the file names below
  truth.json    where everything really is (written by the scenario builder)
  trace.jsonl   true poses and events on sim time (see record.py)
  judges_list.csv, judges_list_merged.csv   localisation only

Usage: python3 sim/judge/cli.py <run folder> [--out <score.json>]
Exit code 0 when the run was scored (whatever the points), 2 when it could not be.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from judge import judge_run  # noqa: E402
from judge.record import Record  # noqa: E402

JUDGE_KEYS = {
    "mission": None,
    "settings": {},
    "vehicle_ground_offset_m": 0.0,
    "truth": "truth.json",
    "trace": "trace.jsonl",
    "list_csv": "judges_list.csv",
    "merged_list_csv": "judges_list_merged.csv",
}


def score_folder(run_dir: Path) -> dict:
    import yaml

    judge_file = run_dir / "judge.yaml"
    if not judge_file.is_file():
        raise ValueError(f"{judge_file} not found: not a run folder")
    try:
        config = yaml.safe_load(judge_file.read_text()) or {}
    except yaml.YAMLError as error:
        raise ValueError(f"{judge_file}: {error}") from error
    if not isinstance(config, dict):
        raise ValueError(f"{judge_file}: expected a mapping")
    unknown = sorted(set(config) - set(JUDGE_KEYS))
    if unknown:
        raise ValueError(f"{judge_file}: unknown key(s) {', '.join(unknown)} (known: {', '.join(JUDGE_KEYS)})")
    config = {**JUDGE_KEYS, **config}
    if not config["mission"]:
        raise ValueError(f"{judge_file}: no mission")

    for key in ("truth", "trace"):
        if not (run_dir / config[key]).is_file():
            raise ValueError(f"{run_dir / config[key]} not found")
    truth = json.loads((run_dir / config["truth"]).read_text())
    record = Record.load(run_dir / config["trace"], float(config["vehicle_ground_offset_m"]))

    lists: dict[str, str] = {}
    missing: list[str] = []
    if config["mission"] == "localisation":
        for key, name in (("list", config["list_csv"]), ("merged_list", config["merged_list_csv"])):
            if (run_dir / name).is_file():
                lists[key] = (run_dir / name).read_text()
            elif key == "list":
                missing.append(f"{name} not found in the run folder")

    score = judge_run(config["mission"], record, truth, config["settings"] or {}, lists)
    score["notes"].extend(missing)
    return score


def one_line(score: dict) -> str:
    common = score["common"]
    parts = [
        f"{score['mission']}: {score['points']:.2f} points",
        f"({score['base_points']:.2f} + {score['autonomy_bonus']:.2f} autonomy)",
        f"flights {common['flights']}",
        f"fence breaches {len(common['fence_breaches'])}",
        f"failsafes {len(common['failsafes'])}",
    ]
    return ", ".join(parts)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Score one sim run folder.")
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--out", type=Path, help="where to write the score (default: <run folder>/score.json)")
    args = parser.parse_args(argv)
    try:
        score = score_folder(args.run_dir)
    except (ValueError, OSError, json.JSONDecodeError) as error:
        print(f"judge: {error}", file=sys.stderr)
        return 2
    out = args.out or args.run_dir / "score.json"
    out.write_text(json.dumps(score, indent=2) + "\n")
    print(one_line(score))
    for note in score["notes"]:
        print(f"  note: {note}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
