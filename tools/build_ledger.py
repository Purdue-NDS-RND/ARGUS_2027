#!/usr/bin/env python3
"""Read and update the 2027 build ledger, docs/build/features.json.

The build session changes the ledger only through this tool, so the fields
that belong to the lead (ids, waves, dependencies, notes) are never edited.

  build_ledger.py check                      validate the ledger (run by scripts/check.sh)
  build_ledger.py next [--scope S]           print the next thing to do: an ID, "GATE <name>" or "NONE ..."
  build_ledger.py summary [--scope S]        a short status block (used by hooks and reports)
  build_ledger.py set ID STATUS [--evidence TEXT] [--reason TEXT] [--attempt]
  build_ledger.py gate NAME STATUS [--result PATH]
  build_ledger.py group NAME                 the features in a landing group, in build order

Scopes: all, wave:N, wave:N:LANE (e.g. wave:3:CEN), id:ID. The /build skill
turns its arguments into one of these.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LEDGER = ROOT / "docs" / "build" / "features.json"

STATUSES = ("TODO", "DOING", "DONE", "PARTIAL", "BLOCKED", "HOLD-DATA", "HUMAN", "LEAD")
SETTABLE = ("TODO", "DOING", "DONE", "PARTIAL", "BLOCKED")
SATISFIED = ("DONE", "PARTIAL")
GATE_STATUSES = ("not run", "PASS", "FAIL", "INVALID", "WAITING")
FEATURE_KEYS = (
    "id", "title", "wave", "lane", "priority", "deps", "land", "proven_in", "note",
    "status", "attempts", "evidence", "blocked_reason",
)


class LedgerError(Exception):
    pass


def load(path: Path = LEDGER) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise LedgerError(f"ledger not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise LedgerError(f"ledger is not valid JSON: {exc}") from exc


def save(data: dict, path: Path = LEDGER) -> None:
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    tmp.replace(path)


def by_id(data: dict) -> dict:
    return {f["id"]: f for f in data["features"]}


def wave_key(wave: str) -> float:
    return float(wave) if wave.isdigit() else float("inf")


def problems(data: dict) -> list[str]:
    """Everything wrong with the ledger, one line each. Empty means valid."""
    errs: list[str] = []
    feats = data.get("features")
    if not isinstance(feats, list) or not feats:
        return ["ERROR: ledger has no features list"]
    seen: dict[str, dict] = {}
    for f in feats:
        fid = f.get("id", "?")
        missing = [k for k in FEATURE_KEYS if k not in f]
        if missing:
            errs.append(f"ERROR: {fid}: missing keys {', '.join(missing)}")
            continue
        if fid in seen:
            errs.append(f"ERROR: {fid}: duplicate id")
        seen[fid] = f
        if f["status"] not in STATUSES:
            errs.append(f"ERROR: {fid}: unknown status {f['status']!r}")
        if not isinstance(f["deps"], list):
            errs.append(f"ERROR: {fid}: deps must be a list of feature ids")
        if f["status"] == "DONE" and not f["evidence"].strip():
            errs.append(f"ERROR: {fid}: DONE without evidence")
        if f["status"] == "BLOCKED" and not f["blocked_reason"].strip():
            errs.append(f"ERROR: {fid}: BLOCKED without a reason")
    for f in seen.values():
        for dep in f["deps"] if isinstance(f["deps"], list) else []:
            if dep not in seen:
                errs.append(f"ERROR: {f['id']}: depends on unknown id {dep!r}")
    errs.extend(_cycles(seen))
    doing = [fid for fid, f in seen.items() if f["status"] == "DOING"]
    if len(doing) > 1:
        errs.append(f"ERROR: more than one DOING row: {', '.join(doing)}")
    for name, g in data.get("gates", {}).items():
        if g.get("status") not in GATE_STATUSES:
            errs.append(f"ERROR: gate {name}: unknown status {g.get('status')!r}")
        for dep in g.get("after_ids", []):
            if dep not in seen:
                errs.append(f"ERROR: gate {name}: runs after unknown id {dep!r}")
    return errs


def _cycles(feats: dict) -> list[str]:
    state: dict[str, int] = {}
    errs: list[str] = []

    def visit(fid: str, path: list[str]) -> None:
        if state.get(fid) == 2 or fid not in feats:
            return
        if state.get(fid) == 1:
            loop = path[path.index(fid):] + [fid]
            errs.append("ERROR: dependency cycle: " + " -> ".join(loop))
            return
        state[fid] = 1
        for dep in feats[fid]["deps"] if isinstance(feats[fid]["deps"], list) else []:
            visit(dep, path + [fid])
        state[fid] = 2

    for fid in feats:
        visit(fid, [])
    return errs


def in_scope(f: dict, scope: str) -> bool:
    if scope == "all":
        return True
    parts = scope.split(":")
    if parts[0] == "id" and len(parts) == 2:
        return f["id"] == parts[1]
    if parts[0] == "wave" and len(parts) in (2, 3):
        if f["wave"] != parts[1]:
            return False
        return len(parts) == 2 or f["lane"] == parts[2]
    raise LedgerError(f"bad scope {scope!r}: use all, wave:N, wave:N:LANE or id:ID")


def runnable(data: dict, scope: str) -> list[dict]:
    """TODO rows in scope whose dependencies are met, lowest wave first, then ledger order."""
    feats = by_id(data)
    rows = [
        f for f in data["features"]
        if f["status"] == "TODO" and in_scope(f, scope)
        and all(feats[d]["status"] in SATISFIED for d in f["deps"])
    ]
    return sorted(rows, key=lambda f: wave_key(f["wave"]))  # stable: keeps ledger order within a wave


def gates_due(data: dict, scope: str) -> list[str]:
    feats = by_id(data)
    due = []
    for name, g in data.get("gates", {}).items():
        if g["status"] != "not run":
            continue
        ids = g.get("after_ids", [])
        wave = g.get("after_wave")
        if not ids and not wave:
            continue
        if any(feats[i]["status"] not in SATISFIED for i in ids):
            continue
        if wave and any(f["wave"] == wave and f["status"] in ("TODO", "DOING") for f in data["features"]):
            continue
        related = [feats[i] for i in ids] or [f for f in data["features"] if f["wave"] == wave]
        if scope != "all" and not any(in_scope(f, scope) for f in related):
            continue
        due.append(name)
    return due


def next_action(data: dict, scope: str) -> str:
    doing = [f for f in data["features"] if f["status"] == "DOING" and in_scope(f, scope)]
    if doing:
        return doing[0]["id"]
    due = gates_due(data, scope)
    if due:
        return f"GATE {due[0]}"
    rows = runnable(data, scope)
    if rows:
        return rows[0]["id"]
    left = [f for f in data["features"] if in_scope(f, scope) and f["status"] in ("TODO", "BLOCKED")]
    if not left:
        return "NONE: scope finished"
    return "NONE: nothing runnable; waiting on " + ", ".join(f"{f['id']} ({f['status']})" for f in left[:8])


def summary(data: dict, scope: str) -> str:
    rows = [f for f in data["features"] if in_scope(f, scope)]
    counts = {s: sum(f["status"] == s for f in rows) for s in STATUSES}
    lines = [
        f"Ledger, scope {scope}: {len(rows)} rows: "
        + ", ".join(f"{s} {n}" for s, n in counts.items() if n)
    ]
    doing = [f["id"] for f in rows if f["status"] == "DOING"]
    lines.append("Doing: " + (", ".join(doing) or "nothing"))
    lines.append("Next: " + next_action(data, scope))
    upcoming = [f["id"] for f in runnable(data, scope)[:3]]
    if upcoming:
        lines.append("Runnable: " + ", ".join(upcoming))
    blocked = [f"{f['id']} ({f['blocked_reason'][:60]})" for f in rows if f["status"] == "BLOCKED"]
    if blocked:
        lines.append("Blocked: " + "; ".join(blocked))
    gates = data.get("gates", {})
    lines.append("Gates: " + ", ".join(f"{n} {g['status']}" for n, g in gates.items()))
    return "\n".join(lines)


def set_status(data: dict, fid: str, status: str, evidence: str = "", reason: str = "",
               attempt: bool = False) -> None:
    feats = by_id(data)
    if fid not in feats:
        raise LedgerError(f"unknown id {fid!r}")
    f = feats[fid]
    if status not in SETTABLE:
        raise LedgerError(f"the session can set only {', '.join(SETTABLE)}; {status} belongs to the lead")
    if f["status"] in ("HOLD-DATA", "HUMAN", "LEAD"):
        raise LedgerError(f"{fid} is {f['status']}; only the lead changes that")
    if status == "DOING":
        others = [g["id"] for g in data["features"] if g["status"] == "DOING" and g["id"] != fid]
        if others:
            raise LedgerError(f"{others[0]} is already DOING; finish or block it first")
        unmet = [d for d in f["deps"] if feats[d]["status"] not in SATISFIED]
        if unmet:
            raise LedgerError(f"{fid} depends on unfinished {', '.join(unmet)}")
    if status in ("DONE", "PARTIAL") and not (evidence or f["evidence"]).strip():
        raise LedgerError(f"{status} needs --evidence (the command run and its result)")
    if status == "BLOCKED" and not (reason or f["blocked_reason"]).strip():
        raise LedgerError("BLOCKED needs --reason")
    f["status"] = status
    if evidence:
        f["evidence"] = evidence
    if reason:
        f["blocked_reason"] = reason
    if attempt:
        f["attempts"] += 1


def set_gate(data: dict, name: str, status: str, result: str = "") -> None:
    gates = data.get("gates", {})
    if name not in gates:
        raise LedgerError(f"unknown gate {name!r}")
    if status not in GATE_STATUSES or status == "WAITING":
        raise LedgerError("gate status must be one of not run, PASS, FAIL, INVALID")
    gates[name]["status"] = status
    if result:
        gates[name]["result"] = result


def group(data: dict, name: str) -> list[dict]:
    return [f for f in data["features"] if f["land"] == name]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--ledger", type=Path, default=LEDGER)
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check")
    for name in ("next", "summary"):
        p = sub.add_parser(name)
        p.add_argument("--scope", default="all")
    p = sub.add_parser("set")
    p.add_argument("id")
    p.add_argument("status")
    p.add_argument("--evidence", default="")
    p.add_argument("--reason", default="")
    p.add_argument("--attempt", action="store_true")
    p = sub.add_parser("gate")
    p.add_argument("name")
    p.add_argument("status")
    p.add_argument("--result", default="")
    p = sub.add_parser("group")
    p.add_argument("name")
    args = parser.parse_args(argv)

    try:
        data = load(args.ledger)
        errs = problems(data)
        if args.cmd == "check":
            print("\n".join(errs) if errs else f"ledger OK: {len(data['features'])} features")
            return 1 if errs else 0
        if errs:
            print("\n".join(errs))
            return 1
        if args.cmd == "next":
            print(next_action(data, args.scope))
        elif args.cmd == "summary":
            print(summary(data, args.scope))
        elif args.cmd == "set":
            set_status(data, args.id, args.status, args.evidence, args.reason, args.attempt)
            save(data, args.ledger)
            print(f"{args.id} -> {args.status}")
        elif args.cmd == "gate":
            set_gate(data, args.name, args.status, args.result)
            save(data, args.ledger)
            print(f"gate {args.name} -> {args.status}")
        elif args.cmd == "group":
            rows = group(data, args.name)
            if not rows:
                raise LedgerError(f"no features in landing group {args.name!r}")
            ids = {f["id"] for f in rows}
            feats = by_id(data)
            for f in rows:
                outside = [f"{d} ({feats[d]['land']})" for d in f["deps"] if d not in ids]
                print(f"{f['id']} {f['status']}" + (f"  needs: {', '.join(outside)}" if outside else ""))
    except LedgerError as exc:
        print(f"ERROR: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
