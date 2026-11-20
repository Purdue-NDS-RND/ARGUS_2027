# LOC-11: Frame logging policy

Flag: `features.survey_frame_log` (node flag of `loc_node`, default false) · Status: PARTIAL · Commits: `git log --oneline --grep '^LOC-11:' platform/build-2027`

## What it solves

A survey's 4K frames are what the digit reader is trained on and what an attempt is judged against afterwards, and nothing kept them: last year's logger took one image a second over a topic. With the flag on the localisation node logs every frame the camera gives during the survey, and steps its own rate down before the detector loses any. It is the policy; the writer was already in the camera package.

## Architecture

- `src/argus_localisation/argus_localisation/frame_logging.py` (new, no ROS): `SurveyFrameLog`, the policy over argus_camera's `FrameLog`.
- `argus_camera/frame_log.py`: one optional argument, `worker_niceness` (default 0, so nothing changes for its other users).
- `loc_node`: with the flag on, frames go to the log from the same two places they go to the detector (the camera's own thread, or the sim's image callback); the log starts and stops with `/loc/set_enabled`; `/loc/status` carries `frame_log`. New parameters: `frame_log_dir`, `frame_log_workers`, `frame_log_jpeg_quality`, `frame_log_max_gb`, `frame_log_min_free_gb`.
- `tools/frame_log_bench.py` (new): what the logging costs on a machine.
- No new topic, message or dependency.

## Flow

A 4K frame from the camera during the survey, flag on:

- Before: offered to the detector; if the detector was busy it was replaced by the next and gone.
- After: offered to the detector, then queued for the log whether or not the detector takes it; a worker thread at low priority encodes it as JPEG at quality 90 and writes it with its capture time in the folder's index. Once a second the logger looks at the detector's rate and halves its own share if the detector is short.

## Code

- The start of a survey's log (the folder must exist, the disk must have room): `frame_logging.py:119`; the stop that does not wait on the disk `:152`.
- A frame, kept or thinned, with a failed write contained: `frame_logging.py:174`.
- The once-a-second review: the detector's rate, the log's own queue, the disk: `frame_logging.py:189`.
- The node: the settings and their refusals `loc_node.py:214`; the frames `:350`, `:369`; the review `:398`.
- The writer's workers at a lower priority: `argus_camera/frame_log.py:120`.

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider src/argus_localisation/test tools/test/test_frame_log_bench.py
96 passed in 4.34s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ python3 tools/frame_log_bench.py --out <folder> --seconds 10 --min-free-gb 5
PASS: every frame logged
170 of 170 frames of 3840 x 2160 at 17 a second, 3 worker(s), JPEG quality 90
71.2 MB a second written (4.18 MB a frame); 0.70 CPU cores used over 10.0 s
```

`docs/results/2026-11-20-frame-log-bench.md`: the build machine, not the Jetson.

## Spec check

- spec-auditor, round 1: INCOMPLETE. The trigger as first built ("the detector dropped a frame") would have held the log at one frame in sixteen all survey long, because the 4K mode gives 17 frames a second to a detector whose target is 15; also the reader-thread path untested with the flag on, a write failure escaping with no workers, a size limit that was one survey's and not the disk's.
- flight-guardian, round 1: five concerns, no blocker: the same trigger; the executor not protected from the encoding threads; the stop blocking on the disk; the disk filling across flights; a frame-log setting able to stop the node with the flag off.
- All changed: the trigger is the detector's rate against what it should be; the encoding threads run at niceness 10; the stop hands the close to a thread; a free-space floor; the settings declared apart. Recorded for the bench: whether the executor's tick holds with the log on.
- Beyond the spec, each under Decisions: the halving and the ten-second recovery; the 3 s not judged at a survey's start; 20 GB kept free; a folder that must exist.

## Not done

- **Bench (why PARTIAL).** On the Orin Nano with the NVMe: megabytes a second, cores used, the JPEG library, and the logger beside the detector and the mission executor ("Needs a person").
- Not flown in the sim: the proof asked for is the bench, and the sim's frames are a tenth the size.
- The log's index has no pose; joining frames to poses afterwards is by capture time (PLT-12's bundle).
