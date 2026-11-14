# <ID>: <feature name>

Flag: `<flag name>` (default false) · Status: DONE / PARTIAL / BLOCKED · Commits: `git log --oneline --grep '^<ID>:' platform/build-2027`

## What it solves

One or two sentences: the problem in plain words, and whether this fixes the cause or works around a symptom.

## Architecture

New or moved modules, nodes, topics, messages and dependencies. Write "No architecture change" if there are none.

## Flow

One concrete path through the system (a frame, a message, a mission state or a sim run), before this feature and after it.

## Code

The three to five logic changes that matter, in plain words, each with `file:line`. Skip trivial edits.

## Evidence

The exact commands run (tests, check script, sim scenario) and the last lines of their output, copied, not retyped. Where measured numbers live in `docs/results/`, link the file.

## Spec check

The spec-auditor verdict, and anything in the spec that was left out or changed, with the reason. The flight-guardian verdict and concerns, if it ran.

## Not done

What is left: person steps, hardware or field checks, follow-ups.
