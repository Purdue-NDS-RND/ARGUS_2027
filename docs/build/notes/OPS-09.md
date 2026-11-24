# OPS-09: Required documents: SOP, emergency procedures, checklists

Flag: none (three documents and the tool that writes them) · Status: DONE · Commits: `git log --oneline --grep '^OPS-09:' platform/build-2027`

## What it solves

The competition asks for a standard operating procedure, emergency procedures and checklists. Written by hand they go wrong the first time a limit is tuned or a failsafe is changed: the paper says 3.9 V and the preflight says something else. Here the parts that can drift are read from the code when the documents are written, and a check fails when the committed documents are not what the code says now.

## Architecture

- `tools/ops_documents.py` (new, no ROS): writes `docs/ops/sop.md`, `emergency_procedures.md` and `checklists.md`; `--check` compares them with what it would write.
- `drone_utils/preflight.py`: `GUIDE`, beside `CHECKS`: for each check, what GO means (with the limit's name, filled from `Limits`) and what to do about a NO-GO. Nothing in the preflight's verdict reads it.
- The failsafe table is decoded from the values of `platform/fcu/argus_2027.param`: each action, what breaches the fence (`FENCE_TYPE`), and when a failsafe does not act (`FS_OPTIONS`). The rows that say what people do about a failsafe quote the same decoded action.

## Flow

- Before: no documents.
- After: a limit or a failsafe changes → `scripts/check.sh` fails on `test_ops_documents.py` (`documents: DRIFTED`) → `python3 tools/ops_documents.py` → the difference is read and committed with the change.

## Code

- What each check means and its remedy: `src/drone_utils/drone_utils/preflight.py:49`.
- The failsafe rows from the baseline's values `tools/ops_documents.py:115`, when a failsafe does not act `:89`, and the refusal of a value with no words `:61`.
- The three documents: `:149`, `:181`, `:223`; the drift check: `:270`.

## Evidence

```
$ python3 tools/ops_documents.py --check
documents: as generated
$ pytest tools/test/test_ops_documents.py   (as scripts/check.sh runs it)
8 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ grep 'Battery low' docs/ops/emergency_procedures.md
| Battery low | under 21 V (3.50 V a cell) for 10 s | returns to the takeoff point (RTL) | `BATT_LOW_VOLT, BATT_FS_LOW_ACT` |
$ grep -c 'not while a person is flying it' docs/ops/emergency_procedures.md docs/ops/sop.md
docs/ops/emergency_procedures.md:2
docs/ops/sop.md:1
```

The tests hold: every preflight check is a checklist line in the preflight's order with its limit; a changed limit or meaning fails the check; a changed failsafe action changes the table; every tool, option, service, flag, profile and failure policy the documents name exists. The tests were written after the tool.

## Spec check

- spec-auditor, round 1: INCOMPLETE on nine rows, each a sentence in a document that the code contradicted. Fixed, and each now held by a test: the hand-back after a takeover is not always "home" (`continue_to_next` flies the next mission); a lost ground station does nothing while a person is flying (`FS_OPTIONS` bit 4, now decoded); the `engine` check compares nothing when no engine is named, and the SOP's command names none; the recovery's abort also lands with no pose or untrusted GPS; the 5 m margin is the 2027 missions' only; the 100 ms time-step limit was in no line (every limit must now be in one); the rows about people restated failsafe actions by hand (they now quote the decoded ones); the fallback check's output; "each behind its flag".
- spec-auditor, round 2: COMPLETE.
- "Battery critical ... for 10 s": ArduPilot's timer is applied to both levels. The repo's own evidence is one SITL flight (`docs/results/2026-11-23-fc-baseline-failsafes.md`: the critical action 10.2 s after the voltage went under); the aircraft has not shown it.
- No flight-guardian review: no mission, executor or detector code and no config under `src/` changed (`GUIDE` in `drone_utils/preflight.py` is text nothing in the verdict reads).

## Not done

- The parts about people (roles, hand checks, the emergency table) are the session's draft and have not been read by anyone who will follow them ("Needs a person"). Nothing checks them but a reader.
- The organisers' required format, if there is one, is not known; the documents are plain Markdown.
- The failsafe table is the baseline file's, not the aircraft's: it is true of the aircraft when the preflight's `params` check is GO.
- Two rules in the documents are proposals that no tool enforces (no override of `fence`, `ekf`, `battery`; no flight after an emergency before the report is read): under "Decisions".
