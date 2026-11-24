# The readiness submission (5 May 2027): geofence demonstration, logs, video

What the organisers ask the video to show is open question 18 in `docs/build/FEATURES.md`. Until the rules' text is in hand this is the team's reading: the aircraft flying under its autonomous stack at the flight boundary, and the flight controller stopping it there by itself. Read the rules' text against the checklist below before filming, and change the checklist, not the flight.

## Before the day

1. The flight controller has the 2027 baseline loaded and the fence uploaded (`platform/fcu/README.md`).
2. The day's coordinates are in a day file, checked by a second person, `verified: true`, with `fence_demo: {bearing_deg: ...}`: the compass bearing from the takeoff point, square to an edge of the flight boundary that has nobody and nothing beyond it (the tool refuses a bearing more than 15 degrees from square: the flight controller's avoidance slides a vehicle along a fence it meets at an angle). Then:

   ```bash
   python3 tools/field_setup.py field/<day>.yaml --for-flight
   ```

   `field/<name>/summary.txt` says how far the boundary is on that bearing and where the run fails if the fence does nothing (10 m past the boundary). Walk that line on the field, to its end and 15 m further: with no working fence the aircraft is told to stop there, at 5 m/s.
3. Fly the same day file in the sim once: `sim/run_scenarios.sh geofence/readiness_demo --variants on` is the placeholder site's version.

## The flight

Take off from the day file's takeoff point: the run's lengths are measured from it, and the mission will not fly the run from more than 10 m away. The safety pilot holds the transmitter throughout and takes over at any doubt. The mission never takes the vehicle back from the pilot or from the fence.

```bash
python3 tools/go.py --profile fence_demo --sequence field/<name>/sequences/fence_demo.yaml --params live_target_mission.yaml \
    --no-camera --param-dump <today's parameter dump> --record <log folder>/<attempt>
```

The preflight must say GO without an override. The vehicle takes off to 10 m, holds for 3 s and flies at 5 m/s on the bearing. Nothing after the take-off is commanded by a person. At the boundary the flight controller ends the run, in one of two ways, and either is the demonstration:

- **Held.** The flight controller's avoidance slows the vehicle and stops it at the fence's margin, inside the line, while the mission goes on commanding 5 m/s. After 30 s of that the mission asks for RTL and the vehicle comes home. This is what the mission's flights in the sim did: stopped 1.9 m inside the line.
- **Breached.** The vehicle goes over the line and the fence's own action switches it to RTL. One sim flight did this (after 14 s of being held, 1 m outside when RTL came).

If the vehicle stops anywhere else, drifts more than 10 m to the side of its line, or reaches 10 m past the boundary still under command, the mission stops it, fails, and the failure policy asks for RTL: that is a finding about the aircraft's fence, not a demonstration. The mission cannot see whether the flight controller has a fence: the preflight can, which is why this flight is started through `go` and never with an override. If the fence's action is not one that flies home (FENCE_ACTION 2 or 4: land, brake), or the pilot takes over, the mission stays quiet and the aircraft is the pilot's to bring down.

After the landing, disarmed:

```bash
python3 tools/dataflash_pull.py --fc-url <link> --out <log folder>/<attempt>/dataflash
python3 tools/bundle_attempt.py --slot readiness --mission geofence --from-run <log folder>/<attempt>
python3 tools/readiness_kit.py attempts/<date>_readiness_geofence
```

The kit must say `READY`. Its `readiness.md` in the bundle is the written evidence; the bundle is the logs.

If the kit says `NOT READY`, the line under "What is missing" says why. A fence that did neither (the mission failed and the vehicle came home by the failure policy) is a finding about the aircraft, not about the video: stop and fix the fence.

## The video

Film in one take where possible. The lines marked (log) are answered by `readiness.md` from the flight controller's own log; show that file on screen at the end.

- [ ] The aircraft on the ground, the team and the safety pilot named, the date.
- [ ] The ground station's map with the fence polygon drawn and the aircraft inside it.
- [ ] The preflight's lines on screen, ending in `GO`.
- [ ] The take-off, with nobody's hands on the sticks (the transmitter in frame).
- [ ] The run towards the boundary, the aircraft and the boundary's markers both in frame.
- [ ] The aircraft stopping at the boundary and staying there (30 s), or crossing and turning back at once; then the ground station's mode changing to RTL, in the same shot or on a second camera.
- [ ] The return and the landing.
- [ ] (log) The fence was enabled, with an action that ends the flight and a polygon stored.
- [ ] (log) The vehicle was in GUIDED, with a velocity commanded, when the flight controller stopped it.
- [ ] (log) Held: the flight controller's avoidance allowed no velocity for the length of the hold, inside the fence line and within 10 m of it. Or breached: the flight controller itself changed to RTL within 2 s of reporting the breach.
- [ ] (log) The bundle is complete: the bag, the flight controller's log, the config, the preflight's log, the commit.
- [ ] `readiness.md` and the attempt's folder name on screen.
