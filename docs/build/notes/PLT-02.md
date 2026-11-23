# PLT-02: Boot services and time sync

Flag: none (installed only when a person runs `services.sh --apply` as root; what starts is set in `/etc/argus/boot.env`, which ships as the 2026 start-up) · Status: PARTIAL · Commits: `git log --oneline --grep '^PLT-02:' platform/build-2027`

## What it solves

In 2026 MAVROS was started by hand in a terminal on the Jetson, and the Jetson's and the laptop's logs were on two clocks. This starts MAVROS (and, when asked, the camera driver) at boot, never a mission, and puts the Jetson on the ground laptop's clock.

## Architecture

- `platform/jetson/services.sh` (new, on PLT-01's `lib.sh`): steps `boot_env`, `mavros_service`, `camera_service`, `chrony`; `--role ground` is the laptop's side (chrony only).
- `platform/jetson/systemd/argus-mavros.service`, `argus-camera.service` (new): run `boot/argus-boot.sh mavros|camera` as the flight user.
- `platform/jetson/boot/argus-boot.sh` (new): turns `boot.env` into one `ros2 launch` line; knows `mavros` and `camera` only.
- `platform/jetson/clock_sync.py` (new, standard library): reads `chronyc -n -c tracking`, prints `CLOCK OK` or `CLOCK NOT OK`, exit 0 or 1.
- No node, topic or flight code. Nothing calls `clock_sync.py` yet: the preflight (PLT-10) does.

## Flow

- Before: power on → ssh in → `./mavros_boot.sh device` on `/dev/ttyACM0` → start the mission launch.
- After: power on → `argus-mavros.service` waits for `/dev/argus/pixhawk`, then `ros2 launch mavros apm.launch fcu_url:=/dev/argus/pixhawk:115200` (or the ARGUS launch with the stream keeper when `ARGUS_BOOT_MAVROS=argus`) → chrony follows the laptop → the mission is started by a person (the GO launcher, PLT-07).

## Code

- What a unit may start, and exit 2 for anything else: `platform/jetson/boot/argus-boot.sh:38`.
- Waiting for the flight controller before MAVROS: `platform/jetson/boot/argus-boot.sh:70`.
- `boot.env` written once, reported but never rewritten: `platform/jetson/services.sh:57`.
- The Jetson's chrony settings, with the step allowed only at start: `platform/jetson/services.sh:113`.
- The clock verdict: `platform/jetson/clock_sync.py:51`.

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider platform/jetson/test/
..................................................................       [100%]
66 passed in 3.57s
```

26 of those are PLT-01's. The services tests run `services.sh` against the fake Jetson and ask `argus-boot.sh` for the command it would start; one starts it for real through a stand-in `ros2` after the device file appears. The service tests failed before `services.sh` existed (14 of 25).

## Spec check

- spec-auditor, round 1: INCOMPLETE on three rows and two deviations to record. Fixed: the bench steps now cover the camera unit and say what passes when the flight controller is replugged; a test pins the 20 ms default; a machine without chrony is told to install it (tested). Recorded under "Decisions": the camera unit ships off, and there is no feature flag. No second round: tests and wording.
- No flight-guardian review: no flight code and no config under `src/` changed.

## Not done

- The bench run (six steps, "Needs a person"). The replug step is a real unknown: if MAVROS survives losing its port, systemd has nothing to restart.
- MAVROS starts before the clock is synced; chrony's allowed step can land under it if the laptop's link comes up late. "CLOCK OK before arming" is the guard, and PLT-10 owes the call.
- `clock_sync.py` reports chrony's own estimate, not an independent measurement, and does not look at the age of the last update.
- The launch arguments the boot script passes were matched to the launch files by reading, not by a test with ROS.
