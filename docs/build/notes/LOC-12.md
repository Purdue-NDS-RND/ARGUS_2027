# LOC-12: Post-landing second pass

Flag: `features.post_landing_pass` (node flag of `loc_node`, default false; needs `features.survey_frame_log` and `features.marker_tracker`) · Status: PARTIAL · Commits: `git log --oneline --grep '^LOC-12:' platform/build-2027`

## What it solves

In flight the detector sees each 4K frame once, shrunk or tiled at whatever rate it keeps, and a frame it had no time for is gone. The frame log (LOC-11) has every frame. After landing there are minutes before the list is due: enough to look at the frames again, tiled at full resolution, and merge what that finds with what the flight found. Whether such a list may be handed in is the organisers' to rule on, so it sits behind a flag and writes to a folder of its own.

## Architecture

- `argus_localisation/second_pass.py` (new, no ROS): `FlightRecord` (the survey's poses and ranges, written beside the frames by a thread of its own), `save_live_tracks`/`load_live_tracks` (the flight's tracks, votes, origin and the pipeline's settings the pass must share), `SecondPass.run` (the replay within a budget), `merge`, `changes`.
- `tools/second_pass.py` (new): the pass as it is run after landing; `--time-only` for planning.
- `loc_node.py`: with the flag on, writes the record during the survey and the flight's tracks when it stops. It does not run the pass.
- `digit_vote.py`: `add_sums`, so votes taken in two passes can be summed.
- No new topic, message, node or dependency.

## Flow

- Before: survey → the in-flight list. The logged frames are for training and for looking at afterwards.
- After (flag on): survey → the in-flight list, and beside the frames `flight_record.jsonl` and `live_tracks.json` → after landing, `tools/second_pass.py <folder> ... --out <folder>` → every logged frame, tiled, through the flight's own pipeline at the pose of its own moment → merged with the flight's tracks → a second list, with what changed printed → a person compares the two (`tools/judges_list_view.py --against`).

## Code

- The replay and its budget: `src/argus_localisation/argus_localisation/second_pass.py:266`; the record fed in step with the frames: `:250`.
- The merge: `src/argus_localisation/argus_localisation/second_pass.py:323`.
- The node's part: the record opened with the survey's log `src/argus_localisation/argus_localisation/loc_node.py:489`, closed and the tracks kept `:495`.
- The tool: `tools/second_pass.py:113`; the pipeline it builds, the same pieces as the node's: `:59`.

## Evidence

```
$ pytest src/argus_localisation/test/test_second_pass.py tools/test/test_second_pass_tool.py src/argus_localisation/test/test_loc_node.py   (as scripts/check.sh runs them)
10 passed, 6 passed, 34 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ python3 tools/second_pass.py sim/.state/loc12/survey_4k --camera-mode sim_still4k --airframe sim_iris_downcam --time-only --tile-ms 25
reading and tiling a frame: 16 ms on this computer, 16 tile(s) a frame (3840 x 2160)
with a detector at 25 ms a tile: 0.42 s a frame, so 2054 frame(s) fit in the 900 s: all 30 of this survey
```

`docs/results/2026-11-24-second-pass.md` has the commands and what each test shows. The tests were written with the code; the surveys in them are made up, with a stand-in detector that finds bright plates.

## Spec check

spec-auditor, two rounds: INCOMPLETE on the one row a PARTIAL keeps open (the 900 s shown on the aircraft's computer with an engine). Round one found the replay ignoring a step of the flight controller's position, the tool not using the flight's own ground height and reader confidence, and no test in which the second pass read a digit: all three DONE in round two. flight-guardian, two rounds: CONCERNS (the record written on the node's one thread; the same two differences; the flag stopping the node when a prerequisite was off), then NO FINDINGS.

## Not done

- Run on the aircraft's computer, on a real survey, with a real engine (why PARTIAL; bench and field: "Needs a person"). There is no engine while the training-data hold stands, and the airframe `argus_2027` is not verified, so the tool refuses it as the node does.
- The node does not run the pass itself after landing: a person runs the tool (a decision, recorded).
- What the record's writing costs the node on the aircraft's disk beside the frame log is not measured; it is off the node's own thread.
- What a tile costs the detector is a guess in `--time-only` (25 ms unless told).
- The checklists do not name the pass: it is off until the organisers rule.
