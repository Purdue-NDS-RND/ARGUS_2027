# Recovery abort: RTL or LAND (REC-2, REC-7), three sim flights

2026-11-23, build machine, sim time, each flight with the recovery flags on (`--variants on`). After the lead's note ("RTL until the bucket is attached, LAND once it is"), built as: RTL while the gripper has never closed and there is a position to fly home on, LAND otherwise (`docs/build/PROGRESS.md`, "Decisions").

| Flight | What it shows | What the mission told the executor | How it ended |
| --- | --- | --- | --- |
| `recovery/pickup_detector` | a failure with nothing on the hook: three lost descents on the detector as built | RTL throughout | `Mission failure policy triggered abort via RTL`, `Abort RTL confirmed by FCU mode`; home, down and disarmed, touchdown 0.50 m/s |
| `recovery/carrying_fault` (new) | a carrying fault: the climb with the bucket outstays its 30 s limit (carry height set to 40 m) | RTL, then LAND from the gripper's close | the mission landed with the bucket on the hook (nothing released, touchdown 0.50 m/s), then failed; the abort found the mode already LAND and asked for nothing |
| `recovery/pickup_oracle` | a whole recovery, no failure | RTL, then LAND from the close | 19.99 points, bucket 0.01 m from the return point |

```
$ sim/run_scenarios.sh recovery/carrying_fault --variants on
   OK          recovery/carrying_fault seed 1 on: 1.97 points, 14.42 m from the return point
$ sim/run_scenarios.sh recovery/pickup_detector --variants on
   OK          recovery/pickup_detector seed 1 on: 0.00 points, bucket not lifted
$ sim/run_scenarios.sh recovery/pickup_oracle --variants on
   OK          recovery/pickup_oracle seed 1 on: 19.99 points, 0.01 m from the return point
```

The points of the first two are not what they were flown for: a recovery that fails scores nothing for the bucket. "OK" is the runner's: the flight ended down and disarmed at a landing's speed, and the log lines the scenario asks for were there.

From the carrying fault's log:

```
[package_recovery] a failure of the sequence would now ask for RTL
[package_recovery] a failure of the sequence would now ask for LAND
[package_recovery] State: CLIMB -> CARRYING_FAULT (CLIMB lasted over 30 s)
[package_recovery] State: CARRYING_FAULT -> FAILED (landed with the bucket on the hook: CLIMB lasted over 30 s)
Abort RTL confirmed by FCU mode
```

(The executor's last line is 2026's wording for "the abort's mode is the mode the flight controller is in": here that mode is LAND.)

## Not flown

- An abort that itself asks for LAND in the air (the mission failing in mid-carry without landing first: an exception, or a landing not seen for 120 s). The executor asking for a named LAND is unit-tested on the real node class (`test_executor_abort_mode.py`).
- A failure with no pose, no GPS fix, or `recovery_gps_untrusted` on (LAND with nothing on the hook): unit-tested.
- `pickup_oracle` was flown on the first build of the rule, in which LAND was given back at the release; the other two were flown on the rule as it stands, less two later changes that are unit-tested only (LAND named on entry when GPS is not trusted, and two seconds of position before the abort is RTL again).
