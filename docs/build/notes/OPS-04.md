# OPS-04: Scoring model and slot planner

Flag: none (a tool and its inputs file; nothing in flight reads either) · Status: DONE · Commits: `git log --oneline --grep '^OPS-04:' platform/build-2027`

## What it solves

Three slots, six missions, the best four count, and nobody could say what a slot plan was worth or how much rides on being allowed to fly two missions in one flight. The spec quotes two figures for that and none of the numbers behind them.

## Architecture

- `tools/score_model.py` (new; numpy and yaml, no ROS): a Monte Carlo of whole competitions, and a planner that tries every placing of the missions in the slots on the same draws.
- `tools/score_model.yaml` (new): the inputs. The judges' points and penalties, and each mission's chances, every number with where it comes from (a sim gate result, or an assumption).

## Flow

- Before: "about 39 to about 57", by an argument nobody can rerun.
- After: edit a chance in `tools/score_model.yaml` → `python3 tools/score_model.py` → each plan's mean and 10th, 50th and 90th percentile, the plans kept ready beside the best plan found with one and with two missions a flight, and each mission's mean points.

## Code

- One competition's luck, drawn once for every plan: `tools/score_model.py:138`; a mission's points by its kind: `:161`.
- A plan on those draws (the second mission of a flight, a second attempt, a lost flight, the bonus): `:178`; the best four: `:201`.
- Every plan there is: `:214`; the best of them: `:237`.
- The inputs' checks: `:84`; an input changed for one run: `:247`.

## Evidence

```
$ pytest tools/test/test_score_model.py   (as scripts/check.sh runs it)
23 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ python3 tools/score_model.py | sed -n 3,7p
plan                       mean    p10    p50    p90  slots
one_mission_a_flight       34.7   16.2   33.5   52.9  recovery | drop | delivery
combined                   48.7   28.3   52.7   63.2  localisation, drop | time_trial, delivery | recovery
best, 1 a flight           34.8   16.2   33.5   52.9  drop | recovery | delivery
best, 2 a flight           50.4   33.4   52.9   63.2  localisation, drop | time_trial, recovery | waypoint_navigation, delivery
```

With nothing left to chance the model gives the rules' own sums (53.75 and 66.25), which a test holds. The finding and a table of what moves it: `docs/results/2026-11-23-score-model.md`. The tests were written after the tool; one of them found that an empty slot could "lose the day", which was corrected.

## Spec check

- spec-auditor, round 1: INCOMPLETE on four rows. Fixed: a second attempt at a mission is in the model, with what it does to the first's score as an input (organiser question 21, which the audit found; the first build refused a mission flown twice); the best plan found is described as what the tool prints (all six missions, the recovery moved), not as the kept plan with one added; every row of the variation table has its command (`--set`), and the sentence over it says what the table says; the delivery's and the recovery's penalties are held by a test, and the judges' terms the model leaves out are listed. No second round: the new input and the new option are tested.
- No flight-guardian review: no mission, executor or detector code and no config under `src/` changed.

## Not done

- The spec's two figures (39 and 57) are not reproduced: the chances behind them are not in the spec. On this file's more cautious inputs the model gives 35 and 49; with every mission flown 9 times in 10, 43 and 55. The finding (combining is worth 10 to 14 points) holds until missions fail half the time.
- No field result is in the inputs. The recovery's chance is a guess.
- The minutes a mission takes and the battery it uses are not modelled (OPS-05 has the slot's clock); second attempts are measured only in a plan given by hand, not searched.
- The time trial's rank among the other teams is drawn from the middle of the field: nobody's times are known.
