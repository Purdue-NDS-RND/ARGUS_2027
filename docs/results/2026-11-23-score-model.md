# Scoring model (OPS-04): what the slot plans are worth

2026-11-23, build machine. `tools/score_model.py` on the inputs committed in `tools/score_model.yaml`: the judges' points and penalties, and a chance for each mission that is either read off a sim gate result or an assumption, each marked as which. No field result is in it yet. A draw is one competition; the best four missions of three slots count.

```
$ python3 tools/score_model.py
20000 draws, seed 1; the best 4 missions of 3 slots count

plan                       mean    p10    p50    p90  slots
one_mission_a_flight       34.7   16.2   33.5   52.9  recovery | drop | delivery
combined                   48.7   28.3   52.7   63.2  localisation, drop | time_trial, delivery | recovery
best, 1 a flight           34.8   16.2   33.5   52.9  drop | recovery | delivery
best, 2 a flight           50.4   33.4   52.9   63.2  localisation, drop | time_trial, recovery | waypoint_navigation, delivery

one_mission_a_flight: drop 12.6, delivery 11.3, recovery 10.9  (mean points by mission, before the best are picked)
combined: time_trial 8.7, localisation 8.8, drop 12.4, delivery 11.0, recovery 10.2  (mean points by mission, before the best are picked)
best, 1 a flight: drop 13.0, delivery 11.3, recovery 10.5  (mean points by mission, before the best are picked)
best, 2 a flight: waypoint_navigation 5.9, time_trial 8.7, localisation 8.8, drop 12.4, delivery 11.0, recovery 10.0  (mean points by mission, before the best are picked)
```

## What it says

- **Combining missions is worth about 14 points on these inputs:** 48.7 expected against 34.7 with one mission a flight. With one a flight three slots carry three missions and there is nothing to fall back on; with two a flight five missions are flown and the worst of them is the one dropped. That is the case for organiser question 1.
- **The plan the spec keeps ready is within two points of the best the planner finds.** The best with two a flight (50.4) flies all six missions: localisation and drop, then the time trial and the recovery, then the waypoint mission and the delivery. It is not the kept plan with one mission added: the recovery moves from a flight of its own to second in slot 2. The kept plan with the waypoint mission added behind the recovery, which changes nothing else, is worth 50.0 (`--plan "localisation,drop | time_trial,delivery | recovery,waypoint_navigation"`). Three a flight finds nothing better (50.4, `--per-flight 3`).
- **With one a flight the three to fly are the drop, the delivery and the recovery,** in any order (34.8).
- **A second attempt is worth nothing if it replaces the first.** `--plan "recovery | drop | recovery"` gives 23.4 where `recovery | drop` gives 23.5; if the best attempt counted it would give 28.0 (`--set second_attempt=best`). Under the assumed answer to organiser question 21, a slot is better spent on another mission than on a repeat.
- The spec quotes "about 39 to about 57". Those are for its own guesses of the chances, which are not written down. This file's are more cautious (the recovery flown 6 times in 10, the drop and the delivery 8 in 10), and give 35 to 49. With every mission flown 9 times in 10 the model gives 43 to 55. Combining is worth 10 to 14 points until missions fail half the time, when there is little left to combine:

| Inputs (`python3 tools/score_model.py` and) | One a flight | Combined | Gain |
| --- | --- | --- | --- |
| nothing more: as committed | 34.7 | 48.7 | 14.0 |
| `--set missions.*.p_flown=0.9` | 43.3 | 54.7 | 11.4 |
| `--set missions.*.p_flown=0.75` | 36.0 | 45.7 | 9.7 |
| `--set missions.*.p_flown=0.5` | 24.1 | 29.6 | 5.5 |
| `--set p_flight_lost=0` | 35.9 | 49.8 | 13.9 |
| `--set p_flight_lost=0.1` | 32.2 | 46.2 | 14.0 |
| `--set p_no_human_input=1` | 35.5 | 49.6 | 14.1 |

## What it does not say

- The chances are the weak part. The recovery's is a guess about a hook that has not been built; the localisation's is from a stand-in detector; nothing is from the field.
- Time is not modelled: whether two missions fit in one flight's battery and one slot's minutes is OPS-05's, and so is how long a slot is (organiser question 2).
- The time trial's points depend on the other teams' times, which nobody knows: its rank is drawn evenly from the middle of the field.
- Three of the judges' terms are left out: the delivery's bonus for weight (0.25 a 500 g), the localisation's penalty for flying under 25 ft, and the difference between a marker with a wrong ID or place (0.5) and one with both wrong (1.0), all counted here as 0.5.
- The planner searches what to fly, each mission once. Second attempts are measured only in a plan given by hand.
