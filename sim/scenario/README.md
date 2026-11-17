# Scenarios

A scenario is one sim test case: a YAML in `sim/scenarios/<mission>/`. The builder turns it into a run folder that holds everything the run needs.

```bash
python3 sim/scenario/build.py sim/scenarios/delivery/baseline.yaml            # flags off: the 2026 behaviour
python3 sim/scenario/build.py sim/scenarios/delivery/clutter.yaml --seed 7 --variant on
RUN_DIR=sim/.state/runs/delivery-clutter-s7-on HEADLESS=1 sim/run_sim.sh      # start its world at its site
```

## A scenario file

| Key | Meaning |
| --- | --- |
| `mission` | Which judge scores it (`sim/judge`); also the folder it lives in |
| `site` | A site in `sim/sites/` |
| `seed` | Default seed for random placement and coordinate error (`--seed` overrides) |
| `timeout_s` | Sim seconds the flight may take |
| `sequence`, `params` | The shipped sequence and params files to fly (`src/drone_mission_demo/config/`). Required unless the scenario has a `driver` |
| `launch` | Launch file; default `<sequence name>.launch.py`, else `multi_mission_demo.launch.py` |
| `config` | Overrides per mission name in the sequence. The setting must exist in the shipped file |
| `targets` | `<mission name>: <object name>`: that mission's `target_latitude` and `target_longitude` are set to the object's position |
| `payload`, `bucket` | The object the mission carries, and the object recovery must bring back. Only these are the judge's payload and bucket; other packages and buckets are just on the field |
| `coordinate_error` | Error added to those coordinates: `{east_m, north_m}`, `{radius_m}` (random bearing) or `{max_radius_m}` (random within the circle) |
| `flags`, `node_flags` | Mission flags and `<node>: {<flag>: value}` flags, applied only with `--variant on` |
| `sim_time` | `true`: with `--variant on` the sim runs on Gazebo's clock (`/clock`, every node on sim time, MAVROS time sync) and the run must pass `sim/time_check.py`. Default `false`: wall time, as in 2026 |
| `objects` | The things on the field (below) |
| `waypoints` | `{mission: <name>, points: [{name, east_m, north_m, height_m}]}`: written as that mission's waypoint file |
| `wind` | `{level: calm, moderate or strong}`, with `direction_deg` and optional numbers; passed to the runner |
| `gps_events` | `[{at_s, kind, ...}]`; passed to the runner |
| `sun` | `{elevation_deg, azimuth_deg}`; left out, the world keeps the 2026 sim's light |
| `sensor_profile` | Camera and vehicle variant; only `delivery` exists so far |
| `sitl_params` | Extra SITL parameters for the run |
| `judge` | Settings for the judge |
| `driver` | `{tool: fence_check, speed_mps, height_m}`: a tool flies the vehicle in place of a mission launch. Such a scenario has no `sequence`, `params`, `config`, `targets`, `flags` or `waypoints` |
| `list_file` | Localisation: the file the mission writes its judges' list to. The runner notes each rewrite and keeps the copy as it stood when the flight ended |

Objects have a `name`, a `kind` and a place: `at: {east_m, north_m}`, `at: {random: {area, min_separation_m, margin_m}}` (area is `survey_area`, `flight_boundary` or `{east_m: [a, b], north_m: [c, d]}`), or `on_top_of: <object>`. `heading_deg` is a compass heading.

| Kind | Settings | Model |
| --- | --- | --- |
| `bullseye` | `size_m` 5.0, `rings` 8 | The 2026 red and white target |
| `marker` | `digit` 0 to 9, `size_m` 0.6; no digit makes a 1.2 m boundary marker | Black and white X with the digit centred |
| `x_mat` | `size_m` 5.0, `arm_width_m` 0.5 | Green X on a white mat |
| `bucket` | `mass_kg` 1.5, `diameter_m` 0.19, `height_m` 0.20 | Dynamic, with an upright handle |
| `payload` | `mass_kg` 0.5, `size_m` [x, y, z] | Dynamic box |

The models are built by `src/drone_utils/drone_utils/sdf_objects.py`.

## A run folder

`world.sdf` (the site world with every object embedded, so nothing is spawned at run time), `bridge.yaml`, `sitl.parm`, `sequence.yaml`, `params.yaml`, `waypoints.yaml` when the scenario has a route, `truth.json` and `judge.yaml` for the judge (`sim/judge/README.md`), and `scenario.json` for the runner: launch file and arguments, timeout, wind, GPS events, expected missions.

## Rules the builder enforces

- **Flags stay off in shipped files.** `--variant off` leaves every flag false; the generated sequence is then the shipped one plus the target coordinates. `--variant on` sets the scenario's flags, only in the copies under the run folder.
- **A flag must be declared.** A mission flag has to be listed under `features:` in the shipped sequence, and a node flag under that node's `ros__parameters.features` in the shipped params file, or the build fails.
- **No silent typos.** An unknown scenario key, object setting, config setting, judge setting, site, sequence or wind level fails the build with a message naming it.
- **Everything is inside the fence.** An object or waypoint outside the site's flight boundary fails the build.
- **The same seed gives the same world.**
