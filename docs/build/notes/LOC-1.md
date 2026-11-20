# LOC-1: Survey height floor and guard

Flag: none of its own (the module is called only by LOC-3's survey mission, behind `survey_search`) · Status: DONE · Commits: `git log --oneline --grep '^LOC-1:' platform/build-2027`

## What it solves

A survey flown under 25 ft loses points and under 20 ft scores nothing. In 2026 nothing watched the height during the search. This gives the survey a height to fly and a guard that climbs back before the floor is reached. New, not a fix.

## Architecture

- `src/drone_mission_core/drone_mission_core/survey_height.py` (new; the standard library only, no ROS). No node, topic or message. Two sequence settings it reads when a mission asks: `survey_height_m`, `survey_height_judged_from_log`.

## Flow

Every tick of a survey, the mission hands the guard `context.height_agl()`:

- 10.6 m on the rangefinder → CONTINUE.
- 8.21 m → RECOVER: the survey waits, the vehicle climbs to the survey height, and the answer stays RECOVER until it is within 0.3 m of it.
- 7.76 m (or 5 s with no height) → ABORT, for good: the climb first, and the mission ends with what it has found.

## Code

- The height a survey flies at (35 ft; 30 ft when the sequence says the height is judged from our log; or the sequence's own, above 8.52 m): `survey_height.py`, `survey_height`.
- The guard: `SurveyHeightGuard.update`; a height that is not the rangefinder's counts 0.5 m lower.

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_survey_height.py
21 passed in 0.04s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

## Spec check

- spec-auditor, round 1: INCOMPLETE on a real defect (on a barometer height a recovery could never end at the height the vehicle was told to climb to: the 0.5 m margin is larger than the 0.3 m way back) and on FND-09's carry-over (the survey height in the start-up check). Both fixed. Round 2: those rows DONE, and the row still INCOMPLETE on a defect the fixes had added: a "starts watching once the survey height is reached" gate, which left the guard silently off for a whole survey flown on the barometer. The gate was removed after the round (the guard watches from its first tick again, and a test dips a barometer survey into RECOVER and ABORT); that removal has not been before the auditor.
- flight-guardian, round 1: four concerns, no blocker: the same barometer recovery; the barometer's 0.5 m being an allowance and heights above the rangefinder's reach always on the barometer; the no-height ABORT asking for a climb with nothing to climb by; the clock unchecked. Round 2: two resolved (the recovery, the clock), two reduced (the configured height is now kept inside the rangefinder's reach; the ABORT with no height has no climb target), and the same watching gate raised as a new concern, since removed.
- Open, recorded for LOC-3 under Decisions: whether a survey may run on the barometer alone; a pose that comes and goes never reaches the guard's 5 s.

## Flown (added with LOC-3)

LOC-3's survey mission flies the guard. `docs/results/2026-11-20-survey-height-guard.md`: with the rangefinder made to read 8.0 m the survey waited, climbed 3.2 m in 3 s and flew all its tracks; with it made to read 7.25 m the survey ended after its climb. The first of those flights found two faults in the guard, both fixed here and recorded under Decisions: a recovery begun on the rangefinder was ended 0.76 s later by the barometer, which cannot see the ground (now it waits up to 3 s for the rangefinder); and the open point below, a height that comes and goes, is settled (a gap under 0.5 s is flown through; a longer one waits for a height that stays 1 s; 5 s of it ends the survey). The guard as it now stands, with the round-2 fix that was not re-audited under LOC-1 (it watches from its first tick), was audited with LOC-3: its spec rows DONE. The guard now has 24 tests.

## Not done

- A dip in the world (ground the rangefinder sees rising): the sim's rangefinder is SITL's own, over flat ground. The two flights make the dip in the sensor.
- Whether the organisers judge the height from our log is a question for them; until the sequence says so the survey flies at 35 ft.
