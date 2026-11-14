import copy
import json
import subprocess
import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "tools")]

import build_ledger as bl  # noqa: E402


def feature(fid, wave="1", lane="FND", deps=(), status="TODO", land="L01", **extra):
    row = {
        "id": fid, "title": fid, "wave": wave, "lane": lane, "priority": "P0",
        "deps": list(deps), "land": land, "proven_in": "Unit", "note": "",
        "status": status, "attempts": 0, "evidence": "", "blocked_reason": "",
    }
    row.update(extra)
    return row


def ledger():
    return {
        "features": [
            feature("A-1", wave="0", lane="SIM"),
            feature("A-2", wave="0", lane="SIM", deps=["A-1"]),
            feature("B-1", wave="1", deps=["A-2"]),
            feature("B-2", wave="1", lane="CEN"),
            feature("C-1", wave="3", lane="CEN", land="L02"),
            feature("H-1", wave="hold", lane="DAT", status="HOLD-DATA", land=None),
        ],
        "gates": {
            "G0": {"title": "baseline", "after_ids": ["A-2"], "status": "not run", "result": ""},
            "GW1": {"title": "wave 1 recheck", "after_wave": "1", "status": "not run", "result": ""},
            "G7": {"title": "field", "status": "WAITING", "result": ""},
        },
    }


def done(data, fid):
    bl.set_status(data, fid, "DONE", evidence="check.sh OK")


class CheckTests(unittest.TestCase):
    def test_valid_ledger_has_no_problems(self):
        self.assertEqual(bl.problems(ledger()), [])

    def test_unknown_dependency(self):
        data = ledger()
        data["features"][2]["deps"] = ["Z-9"]
        self.assertTrue(any("unknown id 'Z-9'" in e for e in bl.problems(data)))

    def test_cycle(self):
        data = ledger()
        data["features"][0]["deps"] = ["A-2"]
        self.assertTrue(any("cycle" in e for e in bl.problems(data)))

    def test_done_needs_evidence_and_blocked_needs_reason(self):
        data = ledger()
        data["features"][0]["status"] = "DONE"
        data["features"][1]["status"] = "BLOCKED"
        errs = "\n".join(bl.problems(data))
        self.assertIn("A-1: DONE without evidence", errs)
        self.assertIn("A-2: BLOCKED without a reason", errs)

    def test_bad_status_and_duplicate(self):
        data = ledger()
        data["features"][0]["status"] = "FINISHED"
        data["features"].append(copy.deepcopy(data["features"][1]))
        errs = "\n".join(bl.problems(data))
        self.assertIn("unknown status", errs)
        self.assertIn("duplicate id", errs)

    def test_real_ledger_is_valid(self):
        data = bl.load(ROOT / "docs" / "build" / "features.json")
        self.assertEqual(bl.problems(data), [])


class NextTests(unittest.TestCase):
    def test_lowest_wave_first_and_dependencies_respected(self):
        data = ledger()
        self.assertEqual(bl.next_action(data, "all"), "A-1")
        done(data, "A-1")
        self.assertEqual(bl.next_action(data, "all"), "A-2")

    def test_lower_wave_wins_over_ledger_order(self):
        data = ledger()
        data["features"].insert(0, data["features"].pop(4))  # C-1 (wave 3) first in the file
        self.assertEqual(bl.next_action(data, "all"), "A-1")

    def test_gate_comes_due_when_its_features_are_done(self):
        data = ledger()
        done(data, "A-1")
        done(data, "A-2")
        self.assertEqual(bl.next_action(data, "all"), "GATE G0")
        bl.set_gate(data, "G0", "PASS", "docs/results/x.md")
        self.assertEqual(bl.next_action(data, "all"), "B-1")

    def test_wave_gate_waits_for_the_whole_wave(self):
        data = ledger()
        for fid in ("A-1", "A-2", "B-1"):
            done(data, fid)
        bl.set_gate(data, "G0", "PASS")
        self.assertNotIn("GW1", bl.gates_due(data, "all"))
        bl.set_status(data, "B-2", "BLOCKED", reason="needs a person")
        self.assertIn("GW1", bl.gates_due(data, "all"))

    def test_doing_row_is_resumed_first(self):
        data = ledger()
        bl.set_status(data, "B-2", "DOING")
        self.assertEqual(bl.next_action(data, "all"), "B-2")

    def test_scope_by_wave_and_lane(self):
        data = ledger()
        self.assertEqual(bl.next_action(data, "wave:3:CEN"), "C-1")
        self.assertEqual(bl.next_action(data, "wave:1"), "B-2")
        self.assertEqual(bl.next_action(data, "id:B-2"), "B-2")
        with self.assertRaises(bl.LedgerError):
            bl.in_scope(data["features"][0], "lane:CEN")

    def test_none_when_finished_or_stuck(self):
        data = ledger()
        self.assertEqual(bl.next_action(data, "wave:hold"), "NONE: scope finished")
        bl.set_status(data, "C-1", "BLOCKED", reason="x")
        self.assertTrue(bl.next_action(data, "wave:3").startswith("NONE: nothing runnable"))

    def test_partial_satisfies_a_dependency(self):
        data = ledger()
        bl.set_status(data, "A-1", "PARTIAL", evidence="sim part done; bench step left")
        self.assertEqual(bl.next_action(data, "all"), "A-2")


class SetTests(unittest.TestCase):
    def test_done_requires_evidence(self):
        with self.assertRaises(bl.LedgerError):
            bl.set_status(ledger(), "A-1", "DONE")

    def test_blocked_requires_reason_and_counts_attempts(self):
        data = ledger()
        with self.assertRaises(bl.LedgerError):
            bl.set_status(data, "A-1", "BLOCKED")
        bl.set_status(data, "A-1", "BLOCKED", reason="sim will not start", attempt=True)
        self.assertEqual(bl.by_id(data)["A-1"]["attempts"], 1)

    def test_lead_statuses_are_off_limits(self):
        data = ledger()
        with self.assertRaises(bl.LedgerError):
            bl.set_status(data, "H-1", "TODO")
        with self.assertRaises(bl.LedgerError):
            bl.set_status(data, "A-1", "HUMAN")

    def test_only_one_doing_and_dependencies_met(self):
        data = ledger()
        with self.assertRaises(bl.LedgerError):
            bl.set_status(data, "A-2", "DOING")
        bl.set_status(data, "A-1", "DOING")
        with self.assertRaises(bl.LedgerError):
            bl.set_status(data, "B-2", "DOING")

    def test_set_never_touches_lead_fields(self):
        data = ledger()
        before = {k: v for k, v in bl.by_id(data)["A-1"].items()
                  if k not in ("status", "attempts", "evidence", "blocked_reason")}
        bl.set_status(data, "A-1", "DONE", evidence="ok", attempt=True)
        after = {k: v for k, v in bl.by_id(data)["A-1"].items()
                 if k not in ("status", "attempts", "evidence", "blocked_reason")}
        self.assertEqual(before, after)

    def test_gate_status_validation(self):
        data = ledger()
        with self.assertRaises(bl.LedgerError):
            bl.set_gate(data, "G0", "WAITING")
        with self.assertRaises(bl.LedgerError):
            bl.set_gate(data, "G9", "PASS")


class CliTests(unittest.TestCase):
    def run_cli(self, path, *args):
        out = subprocess.run(
            [sys.executable, str(ROOT / "tools" / "build_ledger.py"), "--ledger", str(path), *args],
            capture_output=True, text=True,
        )
        return out.returncode, out.stdout

    def test_set_writes_the_file_and_summary_is_short(self):
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / "features.json"
            path.write_text(json.dumps(ledger()))
            code, _ = self.run_cli(path, "set", "A-1", "DONE", "--evidence", "check.sh OK")
            self.assertEqual(code, 0)
            self.assertEqual(json.loads(path.read_text())["features"][0]["status"], "DONE")
            code, out = self.run_cli(path, "summary")
            self.assertEqual(code, 0)
            self.assertIn("Next: A-2", out)
            self.assertLessEqual(len(out.splitlines()), 8)

    def test_check_fails_on_a_broken_ledger(self):
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / "features.json"
            data = ledger()
            data["features"][0]["deps"] = ["Z-9"]
            path.write_text(json.dumps(data))
            code, out = self.run_cli(path, "check")
            self.assertEqual(code, 1)
            self.assertIn("ERROR", out)

    def test_group_lists_outside_dependencies(self):
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / "features.json"
            path.write_text(json.dumps(ledger()))
            code, out = self.run_cli(path, "group", "L01")
            self.assertEqual(code, 0)
            self.assertIn("A-2 TODO", out)
            self.assertNotIn("C-1", out)


if __name__ == "__main__":
    unittest.main()
