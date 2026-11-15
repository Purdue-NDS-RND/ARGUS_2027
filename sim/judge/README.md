# Sim judges

Scores one sim run the way the organisers will, from Gazebo's true positions and never from the EKF. The rules are the SIM-08 table in `docs/build/FEATURES.md`; every number the rules have not fixed is a setting.

```bash
python3 sim/judge/cli.py <run folder>      # writes <run folder>/score.json, prints one line
```

## A run folder

| File | Written by | What it holds |
| --- | --- | --- |
| `judge.yaml` | scenario builder | `mission:` and optional `settings:`, `vehicle_ground_offset_m:` |
| `truth.json` | scenario builder | Where everything really is (below) |
| `trace.jsonl` | run recorder | True poses and events on sim time (format in `record.py`) |
| `judges_list.csv`, `judges_list_merged.csv` | the mission | Localisation only: the list as it stood at landing, and the merged list |
| `score.json` | the judge | Points, the numbers behind them, and the checks every mission gets |

`truth.json` positions are metres east (`e`) and north (`n`) of the world origin; heights (`u`) are above the ground. Each judge needs only its own keys:

| Key | Used by |
| --- | --- |
| `origin: {latitude_deg, longitude_deg}` | localisation (list coordinates to metres) |
| `takeoff: {e, n}` | all (the takeoff zone); defaults to where the vehicle armed |
| `fence: {polygon: [[e, n], ...], max_height_m}` | all (breach report); geofence |
| `waypoints: [{name, e, n, u}]` | waypoint navigation, time trial |
| `markers: [{name, id, e, n}]`, `survey_area: [[e, n], ...]` | localisation |
| `target: {e, n}`, `payload: {name, mass_kg}` | drop, delivery |
| `bucket: {name, mass_kg}`, `return_points: {takeoff: {e, n}, compass_rose: {e, n}}` | recovery |

## What each judge does

| Mission | Points before the bonus | Settings (defaults) |
| --- | --- | --- |
| `waypoint_navigation` | One per waypoint passed within the tolerance, in order | `tolerance_m` 1.0, `dimensions` 3, `points_per_waypoint` 1 |
| `time_trial` | Every waypoint within the tolerance in order, else 0; sim time from first to last waypoint; 12 for the fastest, 8 for the slowest, by rank against `reference_times_s` (none given: 8) | `tolerance_m` 4.0, `dimensions` 3 |
| `localisation` | List matched to truth by the Hungarian method: +1 right, −0.5 wrong ID, −0.5 wrong location, −1 both or an extra row; −3 if the lowest height over the survey area is under 7.62 m, 0 under 6.10 m; 0 if no `list_written` event before landing; the merged list is scored beside it, not added | `tolerance_m` 2.0 |
| `drop` | 14 − distance of the payload at rest (or of the vehicle at a fake release) from the target; −6 released below 1.83 m; −6 if the vehicle is below 1.83 m outside the takeoff zone | `max_points` 14 |
| `delivery` | 13 − distance; 0 if released above 1.83 m; +0.25 per 500 g; the largest `contact` impulse is reported against `impulse_limit_ns` and not scored | `impulse_limit_ns` none |
| `recovery` | 16 − distance of the bucket at rest from the return point; 0 unless it rose more than 0.5 m; 0 if released above 1.83 m | `return_point` `takeoff` |
| `geofence` | Not a competition mission: 1 if the fence held. The autopilot must report the breach and switch to a mode that flies home or stops within `max_response_s` of the true position crossing the fence, and the true position must stay within `max_excursion_m` of it. No autonomy bonus | `max_response_s` 2.0, `max_excursion_m` 10.0 |

Every mission: only the first flight (arm to disarm, having left the ground) is scored, and `common.flights` says how many there were; fence breaches are measured from true positions with their worst excursion; failsafes are listed; and the points gain 25% (`autonomy_bonus_fraction`) when the scored flight has no `gcs_command` or `rc_override` event.

A setting name the judge does not know is an error, so a typo cannot leave a default in place.
