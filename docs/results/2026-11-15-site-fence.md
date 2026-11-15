# Geofence at the Apollo XI site in the sim (2026-11-15)

First runs of the sim at a competition site (SIM-02): the site world, the fence upload against live SITL, the breach test from the SIM-02 pass line, and a plain start afterwards to show the 2026 sim is as it was.

| What | Result |
| --- | --- |
| Start at the site, fence uploaded, pre-arm checks pass | READY; home 0.00 m from the world origin; 4 vertices read back |
| Position target inside the fence (control) | Followed |
| Position target 20 m outside the east fence, in GUIDED | Refused: the vehicle stayed where it was, no breach |
| Fly east through the fence, 5 m/s commanded | RTL 0.04 s after the fence line; 0.99 m outside at most. PASS |
| Fly east through the fence, 10 m/s commanded | RTL 0.06 s after the fence line; 4.46 m outside at most. PASS |
| Plain start (no `SITE`) straight after the site runs | Storage wiped once, 2026 world, target spawned, fence off, READY |

**The pass line in FEATURES.md needs a different trigger.** It says "a waypoint 20 m outside the fence triggers a breach and RTL within 2 s, with at most 10 m excursion". In GUIDED, which is how our missions fly, ArduCopter 4.6.3 does not fly to a position target outside its fence: with a target inside the fence followed just before, the same command 20 m outside moved the vehicle less than 3 m in 15 s, in both runs. So that waypoint causes no breach. Flown through the fence on a velocity command instead, the response is well inside the pass line's limits (2 s, 10 m). An AUTO mission with a waypoint outside the fence was not flown.

The limits are checked by `sim/fence_check.py` itself: it exits 0 only if RTL begins within 2 s of the fence line and the vehicle goes no more than 10 m outside.

## Setup

- **Machine:** club drive (Lenovo 82BG laptop), headless.
- **Sim:** `argus-sim:humble` on `ardupilot-gz:4.6.3`.
- **Site:** `sim/sites/apollo_xi.yaml`, placeholder coordinates: boundary 150 m east and west, 60 m south and 180 m north of the origin; maximum height 120 m; `FENCE_ACTION 1`, `FENCE_MARGIN 2`.
- **SITL:** stored parameters wiped; `SIM_TERRAIN 0` in the site runs (read from the run's `mav.parm`); SITL defaults otherwise, including `AVOID_ENABLE 3` and `WPNAV_SPEED 1000`.
- **Code:** branch `platform/build-2027`, the SIM-02 commit.

## Commands and output

```
$ SITE=apollo_xi HEADLESS=1 sim/run_sim.sh
== Site: <repo>/sim/sites/apollo_xi.yaml
site apollo_xi: NOT VERIFIED (placeholder coordinates)
  origin 34.1750000, -118.4825000, ground 215.0 m AMSL
  flight boundary: 4 corners, -150 to 150 m east, -60 to 180 m north
  maximum height 120.0 m; fence action 1 (RTL or land), margin 2.0 m
...
== Starting ArduCopter SITL (home 34.175,-118.4825,215.0,0.0; QGC on 14550, MAVROS on 14551)
   OK: SITL home 34.1750000, -118.4825000 is 0.00 m from the world origin
== Uploading the geofence (MAVLink tools port, UDP 14552)
fence for site apollo_xi: 4 vertices
  FENCE_TYPE = 5
  FENCE_ALT_MAX = 120
  FENCE_MARGIN = 2
  FENCE_ACTION = 1
  FENCE_ENABLE = 1
fence uploaded and read back: 4 vertices; fence enabled
   pre-arm checks pass after 34s.
== READY. In another terminal:

$ sim/run_sim.sh --exec env -u PYTHONNOUSERSITE python3 sim/fence_check.py --site sim/sites/apollo_xi.yaml --speed 5
  [  78.66] autopilot: Polygon fence breached
  [  78.66] autopilot: Manual recovery started
  [  82.10] autopilot: Fence breach cleared
  [ 140.36] autopilot: Disarming motors
site: apollo_xi
speed_mps: 5.000
inside_target_followed: True
outside_target_refused: True
breach_by: velocity command
passed: True
problem: 
t_cross: 78.627
t_rtl: 78.668
rtl_delay_s: 0.041
max_excursion_m: 0.985
position_rate_hz: 18.487
landed_and_disarmed: True
RESULT: PASS

$ sim/run_sim.sh --exec env -u PYTHONNOUSERSITE python3 sim/fence_check.py --site sim/sites/apollo_xi.yaml --speed 10
site: apollo_xi
speed_mps: 10.000
inside_target_followed: True
outside_target_refused: True
breach_by: velocity command
passed: True
problem: 
t_cross: 67.534
t_rtl: 67.591
rtl_delay_s: 0.058
max_excursion_m: 4.457
position_rate_hz: 18.349
landed_and_disarmed: True
RESULT: PASS
```

Both checks ran in the same sim start, one after the other; each took off, flew, came home on RTL and disarmed.

A third run in a fresh site start, after `fence_check.py` gained its SITL-only guard, at 5 m/s: `rtl_delay_s: 0.050`, `max_excursion_m: 1.068`, `position_rate_hz: 18.761`, `RESULT: PASS`.

Then the plain sim, with SITL's storage last used by the site runs:

```
$ sim/run_sim.sh --down; HEADLESS=1 sim/run_sim.sh
== Starting Gazebo: argus_runway.sdf
== Spawning the delivery target
Spawned 'delivery_target' in 'argus_runway' at x=10.07 m east, y=15.02 m north of home, from package_delivery_demo.yaml (-35.3631271, 149.1653479)
== Starting ArduCopter SITL (home -35.363262,149.165237,584.0,0.0; QGC on 14550, MAVROS on 14551)
   OK: SITL home -35.3632620, 149.1652370 is 0.00 m from the world origin
   pre-arm checks pass after 37s.
== READY. In another terminal:
```

`sitl.log` of that start shows `arducopter -S -w --model JSON` (`-w` is the wipe), its `mav.parm` has `FENCE_ENABLE 0`, `FENCE_TOTAL 0` and `SIM_TERRAIN 1`, no output on UDP 14552, and `sim/.state/sitl/last_site` went from the site file to `none`.

## How to read the numbers

- **Positions are the autopilot's**, from `GLOBAL_POSITION_INT`, not Gazebo's true pose. SITL's GPS is ideal here, so they are close, but the scored measurement is the judge's fence report on true poses, which needs the headless runner (SIM-07) and has not been run on a flight yet.
- **Times are message arrival times.** Positions arrived at about 18 Hz and the crossing time is interpolated between two samples, so `t_cross` is good to a few hundredths of a second. `t_rtl` is the first position sample that carried mode RTL; the mode comes from the heartbeat, requested at 5 Hz, so `rtl_delay_s` can read up to about 0.2 s late. Either way it is far inside 2 s.
- **The vehicle is already slowing when it reaches the line.** With `FENCE_MARGIN 2` and `AVOID_ENABLE 3` it brakes as it nears the fence, and "Polygon fence breached" was reported 0.03 s after the interpolated crossing. The overshoot is what is left of its speed at that point.
- An earlier hand-run probe (a scratch script, not kept) gave 1.09 and 4.23 m at 4 Hz position; `fence_check.py` replaces it.

## Not measured

- True position from Gazebo, as above.
- The maximum-height fence.
- A breach in an AUTO mission, and whether an AUTO mission flies to a waypoint outside the fence.
- A breach at full speed with avoidance off (`AVOID_ENABLE 0`).
