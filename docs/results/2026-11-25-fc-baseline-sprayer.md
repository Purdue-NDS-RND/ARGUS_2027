# The baseline with the sprayer, and the preflight's actuators check, in the sim (PLT-08, PLT-10)

25 November 2026 (run folders are stamped 26 November, UTC). Two flights, one each, after the baseline gained the drop's sprayer (`SPRAY_ENABLE 1`, `SPRAY_SPEED_MIN 0`, `SPRAY_PUMP_MIN 100`, `SERVO8_FUNCTION 22`), `RNGFND1_MAX_CM 600` and the rangefinder's port, and the preflight gained its `actuators` check. The working tree was commit `8b78576` with the PLT-08 range and port change on top.

| Command | Result |
| --- | --- |
| `sim/run_scenarios.sh drop/attempt_release --variants on` | OK. The preflight said GO with nothing failed: `params`: "flight controller holds to the baseline, sha256 1841e5ea30fc"; `actuators`: "the sprayer is on (SPRAY_ENABLE 1, its pump on output 8) for package_drop". Both were read from SITL's ArduPilot over MAVLink. The 2026 drop then let a real 0.5 kg package go: 17.28 points, 0.18 m from the target. |
| `sim/run_scenarios.sh hover/failsafe_radio` | OK. GUIDED -> RTL 1.04 s after the receiver went silent (limit 5 s); the gripper's output at 1900 after the grab and after the trigger; landed and disarmed. |

What this shows: SITL started on the baseline's values holds to the baseline as the preflight reads it, the names the `actuators` check asks for (`SPRAY_ENABLE`, `GRIP_ENABLE`, `SERVO1_FUNCTION` to `SERVO16_FUNCTION`) are answered by ArduPilot 4.6, and a drop sequence with the sprayer set as the baseline sets it is GO and releases.

What it does not show:

- A NO-GO from a live flight controller with the sprayer off: that is unit-tested only (`src/drone_utils/test/test_preflight.py`, with the 2026 aircraft's own `SPRAY_ENABLE 0` and `GRIP_ENABLE 0`).
- The rangefinder's lines and its serial port: the sim's copy of the baseline leaves them out (SITL's rangefinder and ports are its own).
- The battery and ground-station failsafes were not flown again; nothing they read changed.

Run folders (not committed), under `sim/.state/suites/`: `drop-attempt_release-20261126T033823Z/`, `hover-failsafe_radio-20261126T034259Z/`.
