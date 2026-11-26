# REC-5: Optical-flow odometry on the OV9782

Flag: `features.flow_odometry` (node flag of the new node `flow_odometry`, default false). The detector's near field needs no new flag: it is inside `features.recovery_cv` · Status: PARTIAL · Commits: `git log --oneline --grep '^REC-5:' platform/build-2027`

## What it solves

Two things. The recovery flies its last metres over a mat with GPS it may not trust, and had no velocity of its own to check GPS against (REC-6 needs one): this is that velocity, from the flow camera. And, by the lead's note, the detector went blind under 3 m (the mat fills the picture and no X can be fitted), so the mission could not come down on it: the detector now finds the bucket by itself from there.

## Architecture

- `drone_target_cv/flow_odometry.py` (new, no ROS): two pictures up to four frames apart, the shift of the eight patches round the picture's border fitted together with the picture's growing and turning, the vehicle's own turning taken out, scaled by the height; quality 0 to 255, and 0 with the reason on a bare surface.
- `drone_target_cv/flow_odometry_node.py` (new node `flow_odometry`): frames from an image topic, the attitude from a pose history at the frame's own stamp (the frame waits for it), the height from the height estimator. Out: `/argus/flow/velocity` (body), `/argus/flow/velocity_enu`, `/argus/flow/status` (every sample), and, only when `send_optical_flow` is set, `/mavros/optical_flow/raw/send` (MAVLink's OPTICAL_FLOW).
- `green_x.py`: the near field (`_near_bucket`): with no X to fit, a coloured round blob standing on the mat is the bucket, if it is on the arms' crossing and a bucket's size against the arm's width.
- `argus_camera/config/robot.yaml`: the airframe `sim_iris_recovery` (the sim's recovery vehicle, with its flow camera). The node is in the recovery sim launch, the params file (flag false) and the bringup's tables.
- Sim: `sim/flow_check.py` (the odometry against the true velocity, by height band, every sample kept), the scenario `recovery/flow_odometry`; `sim/recovery_cv_check.py` reports the near field; `recovery/pickup_detector` now expects the pickup.

## Flow

- The pickup on the detector, before: X and bucket down to 3 m → nothing → the cone holds → three descents → RTL. After: X and bucket down to 3 m → the bucket by itself down to 0.85 m → the bucket's track CONFIRMED all the way → ENGAGE, LIFT, attach verified, home, set down.
- A flow sample: frame (320 x 200) → waits for the pose of its moment → the oldest of four held frames it can still be followed from → eight border patches, each a phase correlation → the motion a majority agrees on → less the roll and pitch turned through → times height over focal length and time → forward, right, east, north, quality.

## Code

- The patches' motion fitted by majority: `src/drone_target_cv/drone_target_cv/flow_odometry.py:151`; the baseline chosen: `:289`; the turning taken out and the velocity placed by the mount: `:221`, `:236`.
- The node: a frame waits for its pose `src/drone_target_cv/drone_target_cv/flow_odometry_node.py:198`; the height with its source `:169`.
- The near field: `src/drone_target_cv/drone_target_cv/green_x.py:294`, entered only when no X is found `:529`.

## Evidence

```
$ pytest test_flow_odometry.py test_flow_odometry_node.py test_green_x.py sim/test/test_flow_check.py sim/test/test_recovery_cv_check.py   (as scripts/check.sh runs them)
11 passed, 11 passed, 24 passed, 4 passed, 6 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ sim/run_scenarios.sh recovery/pickup_detector --variants on
   OK          recovery/pickup_detector seed 1 on: 19.99 points, 0.01 m from the return point
[package_recovery] done: the bucket is set down (1 descent(s))
near field (under 3 m, the bucket's rim in view): the bucket reported in 416 of 1051 frames, down to 0.84 m; centre error median 0.9 px and p95 2.6 px (about 0.004 m and 0.011 m)
$ sim/run_scenarios.sh recovery/flow_odometry --variants on
   FAIL        recovery/flow_odometry seed 1 on: 19.99 points, 0.01 m from the return point
flow odometry: velocity error RMS 0.333 m/s over 562 samples at 5 to 15 m (85% of the samples there had quality); the line is 0.15 m/s
  at other heights: RMS 0.038 m/s over 643 samples (not judged)
```

Six flow flights and what each showed: `docs/results/2026-11-24-flow-odometry-and-near-field.md`. The rule's first tests were written with it; most of the tests there now were written after a flight showed the fault they hold.

## Spec check

- flight-guardian, round 1: CONCERNS, five. (1) The near bucket was held to nothing but colour and shape, and the descent runs on it: now held to the X as the far bucket is (on the arms' crossing, a bucket's size against the arm's width), with tests of a blob along one arm, wrong sizes, and the red-and-white target on green ground; the pickup was flown again on it. (2) The OPTICAL_FLOW message's units and axes are not pinned against MAVROS or ArduPilot: sending is now off by default, the body-frame field agrees with the twist, and the rest is recorded as not checked. (3) The camera's mount was read and thrown away. (4) Sending defaulted to on. (5) A frame that failed ended the node: now it is dropped and said.
- flight-guardian, round 2: CONCERNS, no blocker. (1), (4), (5) resolved, (2) contained. (3) was not: my first fix read a field the loader's mount does not have, and would have doubled the turning for a camera mounted the other way round. Fixed after the second round from the mount's own rotation matrix, for the velocity and for the turning taken out, with tests of a tail-up and a quarter-turned camera pitching in flight; not reviewed a third time. The sim's mount is the one the code assumed all along, so the flights' numbers stand.
- spec-auditor, round 1: INCOMPLETE on nine rows. Round 2: a PARTIAL close is supported; nothing the record hides. Open and recorded: the pass line (0.333 against 0.15 m/s), the OV9782's own frames, the 2 ms timing (not measured anywhere), the flight controller's side of OPTICAL_FLOW, the field replay and its tool.

## Not done

- **The spec's pass line is not met** (why PARTIAL, beside Field): 0.333 m/s RMS at 5 to 15 m in the sim against 0.15. Under 5 m it is 0.02 to 0.05 m/s. The cause of what is left at height is not established.
- The OV9782 itself: its 120 Hz mode is in-process only, and the node reads an image topic. The in-process source is not built.
- OPTICAL_FLOW to the flight controller is built and off (`send_optical_flow`): the message's pixel fields have not been checked against MAVROS's plugin or ArduPilot's flow backend, the aircraft's MAVROS does not load the plugin, and nothing has shown the flight controller receiving it. REC-6's source switching depends on this. The external-navigation alternative is not built.
- The camera and the attitude within 2 ms: not measured anywhere.
- The hook's tip is not looked for: the hook-to-handle offset is the mission's parameter `hook_forward_left_m`, at the sim's value.
- The near field runs on the downward camera in the sim, not on the flow camera the spec hands off to (the sim's is mono). It wants a coloured bucket on the mat, reports nothing under about 0.7 m (the crossing no longer fits the picture), and cannot tell green grass from an arm by colour (the crossing's four arms are what it goes by).
- No sim scenario now shows the mission refusing to descend blind; two unit tests of the flight hold it (named in `recovery/pickup_detector`).
- There is no replay tool for the spec's field pass (a flight's samples against GPS velocity): `sim/flow_check.py` needs Gazebo's poses.

## After the build (2026-11-25): the OPTICAL_FLOW message

- `send_optical_flow` stays off. The message as built does not match ArduPilot 4.6.3's MAVLink flow backend (`AP_OpticalFlow_MAV`): it reads only the int16 `flow_x` and `flow_y`, `quality` and `sensor_id`, scales by the time between messages, takes the body rates from its own gyro, and ignores `flow_comp_m`, `flow_rate` and `ground_distance`, which are the fields this node fills; MAVROS negates y and casts to int16 on the way. The message has to be re-derived for that reader and accepted by SITL with `FLOW_TYPE 5` before REC-6 may ask for source set 2 ("Decisions", "Needs a person").
