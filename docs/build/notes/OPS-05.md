# OPS-05: Slot time budget and clock

Flag: none (a tool, a budget file, and two lines of the GO launcher's output) · Status: DONE · Commits: `git log --oneline --grep '^OPS-05:' platform/build-2027`

## What it solves

A slot is a fixed time and the localisation list has a deadline after it. Nobody on the field could say, without a watch and arithmetic, how much of the slot was left when the GO command was typed, or when the list was due.

## Architecture

- `tools/slot_clock.py` (new, no ROS): `start --slot <n>`, `show [--watch]`, `stop`. The running slot is one small file, `attempts/slot_clock.json` (git ignores `attempts/`).
- `tools/slot_clock.yaml` (new): the budget: the slot's length, the four phases in order, the list's deadline after the slot. Every length is the session's working value.
- `tools/go.py`: prints the clock's line before the preflight and at GO, warns when less of the slot is left than a flight and its pack-up are budgeted, and puts the slot's state at GO into `go.json`. New options `--slot`, `--slot-start`. With no slot running its output is what it was.
- The generated SOP and checklist name the clock (`tools/ops_documents.py`).

## Flow

- Before: `go ...` → preflight → GO.
- After: the team is called → `slot_clock.py start --slot 2` on the laptop (it prints the command that gives the aircraft the same start) → `show --watch` on the laptop's screen → `go ...` prints `SLOT 2: 05:00 in | planned: preflight, 01:00 left | slot ends in 15:00 | list due in 45:00`, then the preflight, `GO`, the line again → after the slot the line counts down to the list's deadline.

## Code

- The budget's rules (phases in order, no longer than the slot): `tools/slot_clock.py:60`.
- Where "now" is in the slot: `:88`; the line: `:110`; the launcher's warning: `:127`.
- The launcher's side, which never stops a flight: `tools/go.py:303`.

## Evidence

```
$ pytest tools/test/test_slot_clock.py tools/test/test_go.py   (as scripts/check.sh runs them)
22 passed, 33 passed
$ python3 tools/slot_clock.py start --slot 2 --state <a scratch file> | head -1
SLOT 2: 00:00 in | planned: setup, 03:59 left | slot ends in 19:59 | list due in 49:59
```

The tests drive the real launcher with a running slot: the line before the preflight and at GO, the warning with twelve minutes gone, a NO-GO, no slot, a start time that cannot be read (the flight goes on). The tests were written after the tool.

## Spec check

spec-auditor, two rounds: INCOMPLETE on one point each, both fixed after the round that found them. Round one: the start crossed to the aircraft as a wall-clock time with no zone (now an instant with its zone), the launcher's tests were pinned to the committed budget (now a budget of their own), and a start written by hand with no zone raised in the launcher (now not a running slot). Round two: one more test still read the committed budget; it now has its own. The timeline, the launcher's line and the ground station's line DONE.

## Not done

- The slot's length and the phases are assumptions (organiser question 2); the 30 minutes for the list are the rules'. The numbers are in `tools/slot_clock.yaml` for the lead to set after a rehearsal (OPS-08).
- "On the ground station" is a terminal on the ground laptop (`show --watch`), not a QGroundControl widget.
- The laptop and the aircraft each keep the slot's start; the aircraft is given it by hand (the command `start` prints) or by `--slot-start`. Nothing sends it over the link.
- The clock does not know what really happened (when the flight took off or landed): it shows the plan against the time.
