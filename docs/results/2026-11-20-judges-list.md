# The judges' list from a survey in the sim (LOC-10)

2026-11-20, on `047878e` plus the LOC-10 change (uncommitted when flown; it is the commit this file comes with).

## Command

```bash
sim/run_scenarios.sh localisation/survey_list --seeds 1 --variants on
```

One flight, ArduPilot SITL and Gazebo on sim time, real-time factor 0.93. The survey of three tracks at 10.67 m
over a field with numbered markers 1, 3 and 7 inside the four corner markers and a 9 lying 3.5 m outside them.
The localisation node ran the whole chain: the sim's oracle detector (no trained detector while training data
is on hold), the tracker, the weighted median, the digit reader with its template stand-in on the camera's own
1280 x 720 picture (a marker is 44 px across), the vote, and the list.

## Result

| Scenario | Status | Points | Listed | Correct | Wrong ID | Wrong place | Missed | List before landing |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| localisation/survey_list, seed 1, flags on | OK | 3.75 (3.0 and the autonomy bonus) | 3 | 3 of 4 | 0 | 0 | 1 (the 9, outside the search area by design) | yes |

The list as the runner copied it while the vehicle was flying:

```
ID,Latitude,Longitude
1,34.1753951,-118.4819786
3,34.1751793,-118.4821199
7,34.1752785,-118.4818269
```

| | |
| --- | --- |
| Distance of each row from its true marker (the judge, from true poses) | 0.026 m, 0.041 m, 0.010 m |
| Lowest height in the survey area | 10.66 m (the rules' floor is 7.62 m) |
| Pipeline, with the reader, the vote and the list on | 571 frames at 10.0 a second (the camera's rate), none dropped, no faults |
| Digit reader, single crops | 52 answered, 65 not answered, none skipped for time |
| Vote | 4 tracks with readings; the three listed ones given 3, 7 and 1; none contested; the last assignment took 0.33 ms |
| List | rewritten 13 times during the flight; the frame's origin from 400 GPS fixes |
| Status texts sent to `/mavros/statustext/send` | 9 |

The count as it went out (the node's log):

```
LOC list: 0 markers
LOC list: 0 markers, 1 more unread
LOC list: 1 marker
LOC list: 1 marker, 1 more unread
LOC list: 1 marker, 2 more unread
LOC list: 2 markers, 1 more unread
LOC list: 2 markers, 2 more unread
LOC list: 3 markers, 1 more unread
LOC list: 3 markers
```

This was the third flight of the scenario. The first made the same list and was not scored: the scenario build
wrote the survey area in a form the judge does not read (fixed). The second scored 3.75 with rows 0.010 to
0.020 m from their markers. The third was flown after the reviews' changes, with the status texts counted.

## Reading it

- More than half of the single crops get no answer at 44 px, and the list is still right: each track's digit is
  the vote of its sightings. No answer was wrong enough to matter, and no digit was contested.
- Each marker is "unread" for about half a second after its track is confirmed, until its votes are enough;
  the 9 is unread until the east corner markers are confirmed and it is seen to lie outside them.
- The positions are the sim's best case: its GPS has no offset, so the 1 to 1.5 m of absolute GNSS error that
  dominates on the field (LOC-9's budget) is not here.
- The digits are the template stand-in's, on the sim's own marker model. A 6 or a 9 inside the area would have
  stayed unread (LOC-6).

## Not shown

- The STATUSTEXT arriving at a ground station: the sim flies without one. The node published nine on
  `/mavros/statustext/send`; QGroundControl showing them is a bench step.
- The flags-off variant: the survey mission is skipped and no list is written.
