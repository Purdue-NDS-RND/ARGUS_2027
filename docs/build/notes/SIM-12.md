# SIM-12: Gaussian-splat twin of the test field

Flag: none (an offline tool; no flight or sim run uses it) · Status: PARTIAL · Commits: `git log --oneline --grep '^SIM-12:' platform/build-2027`

## What it solves

The sim's ground is a flat colour, and the detector has never seen the team's own field except on the day. A splat of the field gives pictures of it from any place along a recorded flight. This is everything after the pictures exist; the pictures themselves need the field and a training run.

## Architecture

- `sim/splat_twin.py` (new; numpy, cv2, yaml; no ROS):
  - the render set, a folder a splat renderer must write: `camera.yaml`, `poses.csv`, `frames/<index>.png`;
  - `composite`: the targets of a targets file painted into every frame with the frame's pose, by `tools/synth_targets.py`'s own antialiased painter, and `truth.json`;
  - `replay`: the 2026 bullseye detector on every frame, each detection put on the ground with the frame's pose, the target estimator fed, the errors against the truth;
  - `standin`: a made-up field through the same contract, marked as a stand-in in its `camera.yaml` and in every report made from it.

## Flow

- `standin <set>` (or a real render set) → `composite <set> --targets targets.yaml --out <with>` → `replay <with>` → detection rate, pixel and ground error, the estimator's final error.

## Code

- The ground to the picture for a camera looking down, nose up: `sim/splat_twin.py:90`.
- The render set's contract, checked: `:112`; the compositing: `:159`; the replay: `:190`.

## Evidence

```
$ pytest sim/test/test_splat_twin.py   (as scripts/check.sh runs it)
6 passed
$ python3 sim/splat_twin.py standin <set> --frames 40 && python3 sim/splat_twin.py composite <set> --targets <targets> --out <with> && python3 sim/splat_twin.py replay <with>
target drop_target: detected in 35 of 35 frames with the whole target in view (100%); 0 hit(s) on something else
pixel error: median 0.41 px, 95th percentile 0.67 px
ground error: median 0.013 m, 95th percentile 0.021 m
estimator: 0.004 m from the truth after 35 hit(s)
STAND-IN: these frames are a made-up field, not a splat of the test field. The numbers prove the harness, not the detector on the real field.
```

The tests hold the projection's sense (a point ahead is above the centre; south is right of a vehicle facing east), that the target's red centre is at the truth's pixel, and that a pose wrong by half a metre shows as half a metre. The numbers above say the harness is consistent with itself; they say nothing about the real field. The tests were written after the tool.

## Spec check

spec-auditor, two rounds: INCOMPLETE, as a PARTIAL row is. The capture, the splat and a renderer are MISSING and a person's. Round one asked for the gaps to be recorded (no tool from a recorded flight to the contract's poses; a camera that only looks straight down; markers painted and not replayed) and for a test of the painted marker; round two found the new refusal of a camera with distortion dead (it read a field the camera model does not have): fixed and tested with a lens's own terms.

## Not done

- The capture of the test field in 4K and the splat's training (why PARTIAL): a person's, and training is under the hold.
- Rendering from a splat: the contract is written; no splat renderer has written to it.
- The camera looks straight down in the contract: roll and pitch are not in `poses.csv`. The markers are composited but only the bullseye is replayed (the marker pipeline's replay is `sim/replay`'s).
- The ground is flat at the local frame's zero.
