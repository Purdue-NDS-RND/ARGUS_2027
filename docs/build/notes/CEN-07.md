# CEN-07: Wind compensation (PI plus disturbance observer)

Flag: `wind_compensation` (mission flag, default false; needs `centring_retune`) · Status: PARTIAL · Commits: `git log --oneline --grep '^CEN-07:' platform/build-2027`

## What it solves

The retuned centring controller is proportional. If the flight controller leaves a steady velocity error in wind, a proportional loop comes to rest downwind of the target by that error over its gain. An observer of the push (what the vehicle did against what it was asked) and an integral of the error take it out. **In the sim it did not help**: see Evidence.

## Architecture

- `drone_mission_core/wind_compensation.py` (new, no ROS): `WindCompensation.step` adds the two terms to the controller's command and holds the sum to the controller's own speed cap; `WindGains` (the sequence's optional `wind_` keys).
- `red_bullseye_mission_base.py`: with the flag on, the retuned controller's command goes through it; reset with the controller when the target is lost.
- `tools/cen_loop_sim.py`: a steady push (`--steady-wind`) and the controller with the compensation, to try it offline.
- Scenarios `drop/hold_6m_strong_wind_comp`, `drop/hold_2m_strong_wind_comp`. The flag is declared (false) in the twelve sequences that declare `centring_retune`.

## Flow

- Before (retune on): error → soft deadband, Kp, cap, slew → velocity command.
- After (this flag on as well): the same command → less the observed push, plus the integral of the error (close in, slow, not while at the cap) → held to the same cap and the same slew limit → velocity command. Nothing is learnt on a tick whose track is only remembered or in a mode that is not GUIDED.

## Code

- The two terms, the cap and the slew limit: `src/drone_mission_core/drone_mission_core/wind_compensation.py:100`; the integral's conditions (a seen track, close in, all but standing, not at the cap): `:145`.
- What was learnt is dropped after three ticks without a step: `src/drone_mission_core/drone_mission_core/wind_compensation.py:46`.
- The mission's use of it, with the slew limit, the seen-track rule and the mode rule: `src/drone_mission_demo/drone_mission_demo/missions/red_bullseye_mission_base.py:547`.

## Evidence

```
$ pytest src/drone_mission_core/test/test_wind_compensation.py src/drone_mission_demo/test/test_centring_missions.py   (as scripts/check.sh runs them)
6 passed, 67 passed
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
$ python3 tools/cen_loop_sim.py --steady-wind 0.3 --seconds 60
2027 retune, kp 0.60     6.0 m  calm, 0.3 m/s steady  settles never  overshoot 0.57 m  hold p95 0.56 m  ...
2027 retune, kp 0.60, wind compensation   6.0 m  calm, 0.3 m/s steady  settles   7.5 s  overshoot 0.13 m  hold p95 0.05 m  ...
$ sim/run_scenarios.sh drop/hold_6m_strong --variants on;  sim/run_scenarios.sh drop/hold_6m_strong_wind_comp --variants on
hold error p95 0.280 m (median 0.097, max 0.691)   against   0.341 m (median 0.112, max 0.869) with the flag on
$ sim/run_scenarios.sh drop/hold_2m_strong --variants on;  sim/run_scenarios.sh drop/hold_2m_strong_wind_comp --variants on
hold error p95 0.272 m (median 0.095, max 0.763)   against   0.292 m (median 0.108, max 1.046) with the flag on
```

`docs/results/2026-11-24-wind-compensation.md`. The tests were written with the code; two of the rule's conditions (no summing at the cap, none while the vehicle is still flying in) came from the loop simulator, which first showed the integral winding up on a capped approach and a plain step overshooting.

## Spec check

spec-auditor: INCOMPLETE on round one (the outcome not shown in the sim; CEN-02's slew limit not held on the sum; the loop simulator's new option untested). Fixed: the sum is slew-limited, with tests in the module, the loop and the mission, and the option is tested. Round two (the two rows only): the option DONE, the slew DONE in the module and the loop and asked for in the mission's own test, which was then added and seen to fail with the argument taken out. The outcome stays PARTIAL: that is this row's status.

flight-guardian: CONCERNS, no blocker. (1) The integral summed the error to a COASTING track: now nothing is learnt unless the track is CONFIRMED. (2) In FINAL_ALIGN and the set-down a track lost for under half a second kept what was learnt: now three ticks without a step drop it. (3) Not one of its invariants: with the takeover latch off, a pilot's flying would be learnt as wind: now a mode that is not GUIDED drops what was learnt. Left as it is: the 0.4 s delay the observer assumes is the loop simulator's and Gazebo's, not measured on the aircraft.

## Not done

- It is not shown to help (why PARTIAL): in the sim's strong wind the hold is a little worse with it at both heights, one seed each, and the offset along the wind was not measured. The sim's flight controller leaves no steady velocity error for it to remove. It stays off; whether it is kept or cut (the spec's "first cut") is the lead's, on a field hold in steady wind with the retune alone.
- Only the retuned controller's path has it; the 2026 centring is untouched.
- Its gains are the session's, tried in the loop simulator only.
