# PLT-01: Jetson bring-up script

Flag: none (a provisioning script; it changes a machine only when a person runs `--apply` as root, and flies nothing) · Status: PARTIAL · Commits: `git log --oneline --grep '^PLT-01:' platform/build-2027`

## What it solves

In 2026 the Jetson was set up by hand from `JETSON_SETUP.md`, with the flight controller on whatever `/dev/ttyACM*` it came up as and a `chmod 666` to reach it. This is one script that takes a fresh JetPack 6.2 to flight state and says, step by step, whether it got there.

## Architecture

- `platform/jetson/setup.sh` (new): eight steps, each a check and an apply: `platform`, `power_mode`, `jetson_clocks`, `fan`, `log_disk`, `ros`, `onnxruntime`, `udev`. `--check` changes nothing; `--apply` does only what is missing; exit 0 means every step is ok.
- `platform/jetson/lib.sh` (new): the step runner and the sandbox (`ARGUS_SYSROOT`, `ARGUS_SETUP_BIN`), shared with PLT-02's script.
- `platform/jetson/setup.conf` (new): the values that can differ between boards; the ones not yet seen on the real Orin are marked "bench".
- `platform/jetson/README.md` (new): the bench procedure and the backup image.
- No node, topic or flight code.

## Flow

- Before: follow a document by hand; find the serial device by unplugging it.
- After: `sudo platform/jetson/setup.sh --apply` → each step's check, its apply if the check fails, its check again → the checklist on the screen and in `/var/lib/argus/setup-checklist.txt` with the date and the commit → MAVROS on `/dev/argus/pixhawk`.

## Code

- The runner: apply only when the check fails, and nothing applied on the wrong machine: `platform/jetson/lib.sh:64`.
- Power mode by the number `/etc/nvpmodel.conf` gives MAXN_SUPER, and "reboot and run again" when it does not take: `platform/jetson/setup.sh:64`.
- The log disk: a labelled filesystem, a folder when the Jetson boots from the NVMe, and the three refusals to format: `platform/jetson/setup.sh:160`.
- The udev rule (vendor ids, interface 00, optional serial): `platform/jetson/setup.sh:288`.

## Evidence

```
$ env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH= python3 -m pytest -q -p no:cacheprovider platform/jetson/test/test_setup.py
..........................                                               [100%]
26 passed in 2.16s
```

The tests run the real script against a fake Jetson (a temporary root and recording stand-ins for `nvpmodel`, `systemctl`, `apt-get`, `blkid` and the rest): a fresh JetPack goes from 1 of 8 to 8 of 8, a second run changes nothing, `--check` changes nothing, and JetPack 5, 6.0 and 6.1 or another board are refused before anything is applied. With the script moved aside all of them failed (24 of 24 at the time).

## Spec check

- spec-auditor, round 1: INCOMPLETE on three rows: no "Needs a person" row for the bench run, three readings of the spec not under "Decisions", and the release match took 36.4.0 (JetPack 6.1). All three fixed (the row and the decision are in PROGRESS.md; the match is now 36.4.3 or later, with tests for 4.0 and 4.4). No second round: the fixes were wording and one tested rule.
- No flight-guardian review: no flight code and no config under `src/` changed.

## Not done

- The run on the real Orin, which is the row's proof (Bench), and the disk image: "Needs a person".
- Unverified until then: the Jetson AI Lab index address, the Pixhawk 6C's USB vendor id under ArduPilot, what `nvpmodel` does when its reboot question is answered no, and whether pip's ONNX Runtime disturbs apt's numpy.
