# REC-4: Engage, lift, attach check, swing shaping, release guard

Flag: `swing_shaping` (mission flag, for the shaped carry; no sequence declares it until the recovery mission, REC-2). In the sim: the scenario's driver settings `shaped: true` and `attach_check: true` · Status: PARTIAL · Commits: `git log --oneline --grep '^REC-4:' platform/build-2027`

## What it solves

Four moments of a recovery that had no rule: taking hold of the handle and knowing the gripper answered, knowing the bucket really came up with the hook, carrying a 1 kg pendulum without it swinging the vehicle about (REC-8's second flight came down under one), and never letting go above 6 ft or on a height that is not known.

## Architecture

- `drone_mission_core/recovery_grip.py` (new; standard library, `safety_limits`, one constant of `set_down`): `Engage` (plunge or sweep, `Ack(attempt, result)`), `lift_height_m`, `AttachCheck` and `LoadWatch` (hover throttle), `ShapedRamp` and `ShapedLeg` (changes of speed over whole pendulum periods, capped at the carrying phase's 3 m/s), `release_decision`.
- No node, topic, params or sequence change. No mission calls it yet: the recovery mission (REC-2) is the first caller.
- Sim: `sim/recovery_check.py` (REC-8's scripted flight) carries by `ShapedLeg` and judges by `AttachCheck` when its scenario asks; new scenario `recovery/hook_carry_shaped`; `sim/carry_swing.py` measures the bucket's swing from a run's true positions.

## Flow

The carry of 14.4 m to the takeoff point, bucket on the hook:

- Before: the commanded point walked at 1 m/s (15.2 s), since the autopilot's own dash and stop had brought the vehicle down.
- After (`shaped: true`): speed up to 3 m/s over one pendulum period (1.70 s for 0.72 m), cruise, slow over one period → 7.4 s, the bucket 0.012 m from where it hangs once the vehicle has stopped.

## Code

- Engage, with the acknowledgement tied to its attempt and both moves time-limited: `src/drone_mission_core/drone_mission_core/recovery_grip.py:168`.
- Attached or not, from two hovers of fresh throttle: `:246`, `:282`; the loss watch: `:306`.
- The shaped leg: `:386`; the release guard, with the ages of the height and the landed state: `:435`.
- In the check flight: `sim/recovery_check.py:98` (the leg), `:162` (the verdict).

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH=src/drone_mission_core python3 -m pytest -q -p no:cacheprovider src/drone_mission_core/test/test_recovery_grip.py
15 passed in 0.13s
$ sim/run_scenarios.sh recovery/hook_carry_shaped --seeds 1 --variants on
recovery-hook_carry_shaped-s1-on OK  rtf 0.91  points 19.99
attach {'attached': True, 'throttle_ratio': 1.4516, 'expected_ratio': 1.2529, 'baseline_throttle': 0.31, 'loaded_throttle': 0.45}
$ python3 sim/carry_swing.py sim/.state/suites/recovery-hook_carry_shaped-20261121T022453Z/runs/recovery-hook_carry_shaped-s1-on
{"distance_m": 14.44, "duration_s": 7.42, "top_speed_mps": 2.94, "hangs_m": 0.012, "swing_m": 0.153, "residual_m": 0.012, "residual_s": 3.1}
```

`docs/results/2026-11-21-recovery-carry.md` has the table. The unit tests hold the spec's numbers (suspension + 0.5 m, 1.5 s, about 1.125, 3 m/s, min(1.5, 1.83)), integrate a pendulum to show a whole-period ramp leaves under 0.02 m of swing where a half-period one leaves over 0.25 m, and put 10,000 random states to the release guard.

## Spec check

- spec-auditor, round 1: INCOMPLETE. Fixed here: a landed state with no age (FND-09's hand-off), the attach check changing REC-8's flight (now behind `attach_check`), a test on each side of the attach line. The rest is the mission's and is listed below.
- flight-guardian, round 1: CONCERNS, no blocker (nothing in a mission calls the rules). Fixed in the rules: an acknowledgement from an earlier attempt, a stopped throttle stream filling a hover, a frozen rangefinder, an engage with no time limit, a leg with no speed flown as "already there", the 6 ft ceiling taken from the rounded-up constant.
- Round 2, spec-auditor: INCOMPLETE only on what is the recovery mission's or a person's (listed below); PARTIAL is the row's status.
- Round 2, flight-guardian: CONCERNS, no blocker; four of seven resolved. Acted on after the round and not reviewed again: the throttle's age is now required, not optional; attempt numbers are unique across `Engage` objects; a plunge is at most 0.5 m; the sim check flight refuses a bad carry speed before takeoff and fails when an attach check it was asked for could not be made. The second flight above was flown before these four; they do not touch the path it took (a valid speed, a check that was made). Left open: the engage's height has no age of its own (the mission's), and a rangefinder that reads the hanging bucket ("Needs a person").
- Deviation, recorded under Decisions: the expected ratio is the square-root model's 1.118 (the spec says about 1.125) and attached asks for half that rise. The sim's ratio was 1.45 against the model's 1.25.

## Not done

- The engage, the loss watch and the release guard have not flown: with the recovery mission (REC-2; REC-8 run R1). So have "not attached → back to CENTRE", reading `swing_shaping`, and the lift by `lift_height_m`.
- "The bucket stays fixed in the hook camera" is an input with nothing behind it yet.
- The mechanism's numbers, the throttle ratio on the aircraft, the rangefinder's mount beside the hook, the landed state's rate: "Needs a person".
- No unshaped 3 m/s carry was flown beside the shaped one; calm air only.
