# A real payload release in the sim (SIM-06)

Measured on 18 Nov 2026 (UTC) on a club laptop, on `platform/build-2027` at `f55eb53` plus the SIM-06 change. Sim image `argus-sim:humble`.

**Result.** The 2026 drop mission, with `fake_drop` off, released a real 0.5 kg package: it hung under the vehicle on a Gazebo detachable joint for the whole flight, the mission's sprayer command moved the flight controller's servo output 8, `sim/payload_bridge.py` saw that and let the package go, and the judge scored where the package came to rest: 0.17 m from the target in calm air, 0.40 m in a steady 6 m/s wind, where it came down 0.57 m downwind of where it was let go. The 2026 delivery set a package down with its gripper the same way, 0.19 m from the target, with a bucket standing by on its own joint.

## Calm

```
$ sim/run_scenarios.sh drop/release --seeds 1 --variants off
| drop/release | 1 | OK | 17.28 | 0.17 m from the target | not run |  |  |
| Payload bridge | parcel released (2290 servo messages heard) |
```

From the run's `trace.jsonl`, `payload_bridge.jsonl` and `score.json`:

| | |
| --- | --- |
| Package to vehicle distance while carried (true poses, from take-off to the release) | 0.214 m at the least and at the most, over 1290 poses: the joint held it rigidly |
| The mission's log line "Closing sprayer for payload drop" | sim time 96.63 s |
| Servo output 8 seen at 1900; the package detached | 97.07 s, 0.44 s later, at 4.02 m |
| Package height after that, every 0.1 s or so | 4.02, 4.02, 3.82, 2.74, 0.74, 0.01 m: on the ground in about 0.9 s |
| Where it was let go → where it came to rest | (9.829 E, 15.032 N) → (9.830 E, 15.033 N): it fell straight down |
| Scored from | "payload at rest": 0.17 m from the target at (10, 15), 14 − 0.17 points plus the autonomy bonus = 17.28 |
| Release height; lowest height in the flight | 4.00 m; 3.97 m (the rule is 1.83 m) |

The 0.17 m is mostly the 0.13 m the package hangs to one side of the vehicle's centre: the 2026 mission centres the vehicle, not the package. Aiming the payload is CEN-01's job.

## In a steady 6 m/s wind

```
$ sim/run_scenarios.sh drop/release_wind --seeds 1 --variants off
| drop/release_wind | 1 | OK | 17.00 | 0.40 m from the target | not run |  |  |
| Wind | mean 6.0 m/s, 6.0 to 6.0 m/s, from 270 degrees, over 2389 updates |
| Payload bridge | parcel released (2259 servo messages heard) |
```

| | |
| --- | --- |
| Drag force on the package, hanging and falling (`wind.jsonl`) | 0.232 N toward the east (½ × 1.225 × 0.0105 × 6² = 0.232 N; its drag area is 1.05 × 0.1 × 0.1 m²) |
| Where it was let go → where it came to rest | (9.821 E, 15.042 N, 4.01 m) → (10.395 E, 15.043 N): 0.57 m east, 0.001 m north |
| Free fall alone would give | 0.19 m east (0.46 m/s² for 0.9 s); the rest is the package bouncing and tumbling downwind after it lands, which it does not do in calm air |
| Scored from | "payload at rest": 0.40 m from the target |

This is also the flight SIM-03 owed: the wind acts on a payload in Gazebo, on the joint and in free fall, and a drop drifts downwind and nowhere else.

## The gripper, on the ground, with a bucket standing by

```
$ sim/run_scenarios.sh delivery/release --seeds 1 --variants off
| delivery/release | 1 | OK | 16.32 | 0.19 m from the target | not run |  |  |
| Payload bridge | parcel released; pail: 0 grab(s), 0 miss(es) (2909 servo messages heard) |
```

The 2026 delivery lands on the target and then commands its gripper. The scenario also has a bucket, as the mission's bucket, 8 m from the takeoff point.

| | |
| --- | --- |
| The mission's log line "Releasing gripper payload" | sim time 113.47 s |
| Servo output 7 seen at 1100 (the gripper's release value); the package detached | 113.62 s, 0.15 s later, with its base 0.045 m off the ground |
| Where it came to rest | 0.19 m from the target; `scored_from: payload at rest`; release height 0.025 m |
| The landing's `contact` event (new: from the package's true poses after the release) | impulse 0.31 N s, a change of velocity of 0.62 m/s for 0.5 kg |
| The bucket, whose joint to the vehicle Gazebo makes when the world loads | at (-8.0, 0.0, 0.0) in every one of its 3088 recorded poses: the bridge unmade the joint before the flight, and the vehicle flew as usual |

In this flight the landing watcher also wrote a `contact` of zero for the bucket, which nobody had released; it now watches only the mission's payload (or its bucket when there is no payload).

## Not shown

- **A delivery's impulse limit.** The `contact` event gives the impulse; the limit it is judged against (`impulse_limit_ns`) is not set until CEN-05, so `impulse_ok` is still empty.
- **A release the bridge never carries out** fails the run (`result.py`); that verdict is unit-tested, not flown.
- **Taking hold of a bucket.** The rule (hook within 0.10 m of the handle horizontally and 0.15 m vertically, otherwise `grab_miss`) is unit-tested. No vehicle has a hook and no mission closes a gripper on a bucket until REC-8.
- **The aircraft's outputs.** The sim's gripper and sprayer outputs are its own choice (7 and 8); the 2026 aircraft's parameters have neither enabled.
- One flight of each, one seed.

## Commands

```
sim/run_scenarios.sh drop/release --seeds 1 --variants off
sim/run_scenarios.sh drop/release_wind --seeds 1 --variants off
sim/run_scenarios.sh delivery/release --seeds 1 --variants off
cat <run folder>/payload_bridge.jsonl
```

Run folders (not committed): `sim/.state/suites/drop-release-20261118T055528Z/`, `sim/.state/suites/drop-release_wind-20261118T055852Z/`, `sim/.state/suites/delivery-release-20261118T060847Z/`.
