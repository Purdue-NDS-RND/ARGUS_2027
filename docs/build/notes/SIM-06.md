# SIM-06: Payload, gripper and release physics

Flag: none (sim only; a scenario with a `carried` payload or a `bucket` turns it on, and every other scenario is built and flown as before) · Status: PARTIAL · Commits: `git log --oneline --grep '^SIM-06:' platform/build-2027`

## What it solves

Until now every drop in the sim was a fake drop: the judge scored where the vehicle was when the mission logged "Fake drop complete". Nothing fell, so nothing about release timing, the package's offset from the vehicle or drift in wind could be seen. This gives the sim a real package on a joint and one bridge from the flight controller's servo outputs to that joint.

## Architecture

- `sim/payload_bridge.py` (new): the plain class `Bridge` (the rules: when an output releases, when a grab takes hold, `grab_miss`) and `run`, which feeds it `SERVO_OUTPUT_RAW` from the tools port and true poses from Gazebo, and makes or unmakes joints on `/payload/<name>/attach` and `/detach`.
- `sim/scenario/spec.py`, `build.py`: an object may be `carried: true`. The builder hangs it under the vehicle with a Gazebo detachable joint in its own model, gives the mission's bucket the same kind of joint, sets the sim's gripper and sprayer parameters, and puts `payload_bridge` in the run's manifest.
- `sim/runner`: the bridge is a side tool of such a run; a run that left no record of servo messages heard is INVALID, and one where the mission commanded a real release that the bridge never carried out is FAIL; one report row. The recorder writes a `contact` event for the released package's landing (`events.LandingWatcher`), which SIM-07 had left to this feature.
- Scenarios `drop/release`, `drop/release_wind` and `delivery/release`, in the nightly suite.
- Nothing under `src/` changes: the missions command the sprayer and the gripper as they did in 2026.

## Flow

Before: mission → "Fake drop complete" in the log → the judge scores the vehicle's position then.

After, with `fake_drop: false` and a carried payload: mission → `MAV_CMD_DO_SPRAYER` → the flight controller's servo output 8 goes to 1900 → the bridge sees the output arrive there → detach on the package's joint → the package falls → the judge scores where it came to rest. With `fake_drop: true` nothing above happens and the old path is used.

## Code

- `payload_bridge.py:70`, `Bridge`, with `on_servos` at `:102`: outputs are acted on when they arrive at a value, not while they stay there, and the first output seen is where the servo already was.
- `payload_bridge.py:155`, `_grab`: the capture box, 0.10 m horizontally (a parameter) and 0.15 m vertically between hook and handle, else `grab_miss`.
- `payload_bridge.py:49`, `SITL_PARAMS`: the sim's own gripper and sprayer outputs.
- `scenario/build.py:198`, `carry`, and `:219`, `_joint_to_vehicle`: where the package hangs and the joint that holds it.

## Evidence

```
$ python3 -m pytest -q -p no:cacheprovider sim/test/test_payload_bridge.py
23 passed in 0.26s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

Sim ([docs/results/2026-11-18-payload-release.md](../../results/2026-11-18-payload-release.md)):

```
$ sim/run_scenarios.sh drop/release --seeds 1 --variants off
| drop/release | 1 | OK | 17.28 | 0.17 m from the target | not run |  |  |
| Payload bridge | parcel released (2290 servo messages heard) |
$ sim/run_scenarios.sh drop/release_wind --seeds 1 --variants off
| drop/release_wind | 1 | OK | 17.00 | 0.40 m from the target | not run |  |  |
$ sim/run_scenarios.sh delivery/release --seeds 1 --variants off
| delivery/release | 1 | OK | 16.32 | 0.19 m from the target | not run |  |  |
| Payload bridge | parcel released; pail: 0 grab(s), 0 miss(es) (2909 servo messages heard) |
```

The package stayed 0.214 m from the vehicle for the whole carry, was let go 0.44 s after the mission's command at 4.02 m, and was on the ground 0.9 s later. In a steady 6 m/s wind it came to rest 0.57 m downwind of where it was let go and 0.001 m across.

The delivery's gripper output (1100 on servo 7) let its package go on the ground 0.15 s after the mission's command, with a landing impulse of 0.31 N s in the trace; the scenario's bucket, on its own joint, did not move in 3088 poses.

## Spec check

spec-auditor, two rounds. Round one: INCOMPLETE, and PARTIAL the right final status. Done after it: the `contact` events and the verdict for a release that never detaches (both new code, with tests); a scenario with the mission's bucket, built in a test and loaded in Gazebo in a flight; a gripper release flown. Handed on by name under Decisions: servo outputs in the recording (PLT-08). Round two (those rows only): four fixed, one stated, nothing open; the row can close as PARTIAL. After it, not reviewed again: the suite's count line now names FAIL runs, with a test.

flight-guardian did not run: nothing under `src/` and no flight config changed.

## Not done

Why the row is PARTIAL, and who has what is left:

- **Taking hold of a bucket has not been flown.** The rule and `grab_miss` are unit-tested, and a bucket's joint has been loaded and unmade in a flight, but no vehicle has a hook and no mission closes a gripper on a bucket: REC-8 (the `iris_recovery` vehicle) and REC-2 (the mission). The hook's offset and the handle's height are stand-ins until then.
- **The sprayer and gripper parameters are the sim's own**, because the 2026 aircraft has neither enabled: the lead, with PLT-08 and REC-9 (under "Needs a person").
- **Servo outputs in the recording**: PLT-08, with the aircraft's gripper output.
- **The landing watcher starts at the mission's log line, not the bridge's detach** (0.15 s and 0.44 s apart in the flights); a detach more than 1 s late would be missed. Under Decisions; to be tightened with CEN-05.
- The recorded `delivery/release` flight predates watching only the mission's payload: its trace also has a zero `contact` for the bucket. The change is unit-tested, not re-flown.
- **A delivery's impulse limit**: CEN-05. The `contact` event is there; nothing judges it yet.
- One flight of each scenario, one seed.
