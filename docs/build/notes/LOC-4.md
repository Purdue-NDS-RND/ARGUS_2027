# LOC-4: Re-visit uncertain markers

Flag: `survey_revisit` (mission flag, default false; needs `survey_search`) · Status: DONE · Commits: `git log --oneline --grep '^LOC-4:' platform/build-2027`

## What it solves

One pass at 4 m/s gives a marker about 25 frames, and some markers end the pattern with no digit or few sightings. The survey then stopped, with time left in its budget and the vehicle over the field. Now it goes back and looks again.

## Architecture

- `drone_mission_core/revisit.py` (new, no ROS): which tracks need another look (`why_revisit`), where the vehicle may be sent (`Bounds`: the survey's own search area and the flight boundary), the round's order and limits (`choose`), the round itself (`Revisit`), and the executor's side: the tracker's tracks from `/loc/tracks` (`TrackFeed`, `attach_tracks`, subscribed only with the flag on).
- `MissionContext.marker_tracks()`: the tracker's newest tracks and their age by their own stamp, or None.
- `search_pattern_mission.py`: with the flag on, when the last track is flown the mission plans the round and flies it inside the SURVEY state, so the height guard, the time budget and the height hold are the tracks' own; then it ends as before. Flag off: not a line of the survey's path changes (a test holds the setpoints equal).
- `features.py`: `survey_revisit` requires `survey_search`. `package.xml`: `argus_interfaces`. `search_pattern_demo.yaml` declares the flag, false.
- Scenario `localisation/revisit`.

## Flow

- Before: tracks flown → pipeline stopped → next mission.
- After: tracks flown → the tracker's tracks (under 5 s old by their stamp) → the ones with no digit or under 8 sightings that the tracker still holds, inside the field → each point in the search area (2 m of slack) and 5 m inside the flight boundary, each leg inside both → nearest first, while the leg and 3 s fit the budget (10 s kept back) → fly, the leg checked again from where the vehicle is, the point following its track → hold 3 s within 1 m → next → pipeline stopped. The tracker going quiet ends the round.

## Code

- Why a track is re-visited: `src/drone_mission_core/drone_mission_core/revisit.py:87`. Where the vehicle may go: `:112`. The round's order: `:145`.
- The round: each leg checked from the vehicle's own place, the visit following its track, a dropped or out-of-bounds track passed over, the hover counted again after a drift: `:196`.
- The mission plans the round `src/drone_mission_demo/drone_mission_demo/missions/search_pattern_mission.py:547` and flies it `:569`, entered from the survey's own state after its guard `:490`.
- The executor's subscription, only with the flag on, and the tracks' age by their stamp: `src/drone_mission_core/drone_mission_core/revisit.py:301`, `:280`.

## Evidence

```
$ pytest src/drone_mission_core/test/test_revisit.py src/drone_mission_demo/test/test_search_pattern_mission.py   (as scripts/check.sh runs them)
12 passed, 49 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh localisation/revisit --variants on
   OK          localisation/revisit seed 1 on: 10.00 points, 8 of 10 markers correct
[search_pattern] re-visit: 2 track(s) to look at again: 12 (digit not read (21 sightings)), 17 (digit not read (20 sightings))
[search_pattern] hovered 3 s over track 12; re-visit 2 of 2: track 17 (digit not read (20 sightings)), 37 m away
[search_pattern] hovered 3 s over track 17
[search_pattern] re-visit round over: hovered over 2 of 2 track(s)
```

By the sim's true positions the vehicle held within a metre of the true markers 6 and 9 for over 3 s each at the survey height: `docs/results/2026-11-23-revisit.md`. The mission's tests were written before the mission was changed and failed (5 of them) until it was; the rule's tests were written with the rule.

## Spec check

- flight-guardian, round 1: BLOCKER, and it was right. With no flight boundary in the sequence (the shipped sequence has none) a re-visit point came straight from the tracker with nothing bounding it: one bad frame could have sent the vehicle a kilometre off. Three concerns beside it: legs held to a weaker margin than the survey's own and not checked after a give-up; the tracks' age counted from arrival and checked once; the round planned in one tick with no cap. All four fixed: every point and leg is held to the search area and the boundary, each leg is checked again from where the vehicle is, the tracks are aged by their stamp and re-read every tick, and the work is capped (40 tracks weighed, 20 visited).
- flight-guardian, round 2: CONCERNS, no blocker; all four resolved, one new small one (a track creeping in steps under a metre moved the leg unchecked), fixed after the second flight with a unit test. Not checked by it, and still open: whether the real camera's frame stamps are on the executor's clock (if not, the tracks read as old and no round is flown, which is safe and useless).
- spec-auditor, round 1: INCOMPLETE on two rows (the eight-visit cap against the spec's "each", and the decisions not yet recorded). Round 2: both done; the caps are recorded under "Decisions" with their reason. Left for this note: the ledger entry and the re-flight's result, both now made.

## Not done

- No digit was read on the second look: the stand-in reader abstains on 6 and 9 by design (the lead's note). What the hover is worth waits for the trained reader.
- "Low digit confidence" is the vote's own abstention (`digit` unknown), not a second threshold on its probabilities.
- The live survey sequence does not exist yet (OPS-01's "Needs a person"); the flag is declared in the sim's sequence.
