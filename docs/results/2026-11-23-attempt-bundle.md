# Attempt bundle (PLT-12): a sim delivery flown as an attempt

2026-11-23, build machine, ArduCopter SITL 4.6.3, ROS 2 Humble in the sim's container. The scenario `delivery/attempt` is the 2026 delivery started by `tools/go.py` with `--record`: the command records the flight's bag from GO to the end of the launch. After the landing, with SITL still up, the runner does what a person does on the field, with the same two tools: `tools/dataflash_pull.py` pulls the flight controller's newest log over MAVLink, and `tools/bundle_attempt.py --from-run <run folder>` makes the attempt's folder. `sim/attempt_check.py` then holds the bundle to what an attempt must be.

```
$ sim/run_scenarios.sh delivery/attempt --variants on
   OK          delivery/attempt seed 1 on: 16.18 points, 0.06 m from the target
1 runs: 1 OK
```

`attempt_check.log` of the run (paths shortened; the sim's container keeps UTC, hence the date):

```
pulled 00000324.BIN: 8.20 MB
attempt 2026-11-24_sim_delivery: sim/.state/suites/delivery-attempt-20261124T005453Z/runs/delivery-attempt-s1-on/attempts/2026-11-24_sim_delivery
git comp-2026-104-g086da70-dirty (with uncommitted changes)
profile: comp_2026 (no flags: the 2026 code)
[   in] bag          9.2 MB
[   in] dataflash    8.2 MB
[   -] frames       not given
[   -] session      not given
[   in] config       0.0 MB
[   in] preflight    0.0 MB
[   -] judges_list  not given
WARNING: the bag is sqlite3, not MCAP: the recorder found no MCAP plugin where it ran
COMPLETE
ATTEMPT CHECK: PASS
```

| What | Read |
| --- | --- |
| Dataflash log pulled over MAVLink | 8 196 096 bytes, log 324; byte for byte the file SITL wrote |
| Bag | 14 topics, 72 432 messages, 9.2 MB; the vehicle's pose and state in it; no image topic |
| Bag storage | sqlite3, with the bundler's warning: the sim's image has no rosbag2 MCAP plugin (the Jetson's package list has it, `platform/jetson/setup.conf`) |
| Config | the sequence, params and profile named in the GO command's own `go.json`, and the flight-controller baseline the preflight held SITL to |
| Flag profile in the manifest | `comp_2026`, no flags: the profile the GO command flew |
| Preflight | the GO command's record: GO |
| Git | `comp-2026-104-g086da70-dirty` (made from a tree with this row's changes uncommitted) |
| Manifest against the folder (`--verify`) | every file as bundled |
| Delivery score | 16.18 points at 0.06 m (the flags-off baseline: 16.21 at 0.03 m; a first attempt flight scored 16.20 at 0.04 m) |

This is the second flight. The first passed too (log 316, 8 212 480 bytes, identical to SITL's file); after it the spec audit's points went in (the config from `go.json`, the profile's own name and the bag's storage in the manifest, an unclosed bag bundled as it is) and the scenario was flown again.

## Not shown here

- MCAP: the sim's bag is sqlite3. The recorder asks for MCAP wherever the plugin is installed (`sim/replay/record_run.py`), and the bundle takes either.
- Frames and the judges' list: a delivery writes neither. Their handling is unit-tested (`tools/test/test_bundle_attempt.py`); a survey flown as an attempt would show them.
- The pull over a real telemetry link. Here it is a local UDP port, and 8.2 MB took seconds. Over a 57600 baud radio the same log is of the order of half an hour; over USB or the companion's serial link far less. Lost packets are asked for again (unit-tested against a stand-in flight controller).
- The session logger's folder (`--session-log`): unit-tested.
