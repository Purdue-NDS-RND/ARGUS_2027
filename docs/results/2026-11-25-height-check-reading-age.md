# Why the height estimator's side check read 0.10 m in drop flights (CEN-03)

25 November 2026. In gate G2 that day the height estimator's check beside the flight read 0.10 to 0.102 m at the 95th percentile against its 0.10 m line in drop flights, and voided two calm runs, while its own scenario read 0.04 m. No new flight was needed to find out why: the flights' own flight-controller logs and the bags of seventeen recorded flights were enough.

## The cause

**A rangefinder reading is already up to about 40 ms old when it is stamped, and the take-off and the relaunch climb at 2.5 m/s.** It is not the estimator, and it is not the final align.

- **The sensor is right at the flight controller.** In the flight controller's own log of the G2 re-flight that failed (`sim/.state/sitl/logs/00000465.BIN`, `drop/gated` seed 1, 25 November), `RFND` against SITL's true height (`SIM2`) differs by a constant 0.195 m (the model's origin above the ground it stands on) to within 0.01 m at the 5th and 95th percentile, the same climbing at 2.5 m/s, descending at 0.35 m/s and through the final align at 2.5 to 3 m. (Read with `tools/dataflash_params.py`'s `messages`.)
- **The reading is old by the time it has a stamp.** In a recorded flight's bag, the raw `/mavros/rangefinder/rangefinder` reading against the true height at its own stamp is right in a hover, a little high in the 0.35 m/s descent and low in the 2.5 m/s climbs, by the same number of milliseconds in each: 21 ms in the flight of 26 November below (0.007 m high descending, 0.053 m low climbing), 37 ms in the drop of 24 November (0.014 m and 0.091 m). The estimator's heights carry exactly the raw readings' error and add none.
- **The age is a constant of the flight, different in each.** ArduPilot reads its rangefinder at 20 Hz and sends the newest reading in the next `DISTANCE_SENSOR` message, which MAVROS stamps by when it was sent: between nothing and one 50 ms period, by how the two line up in that boot. Over the seventeen flights the 95th percentile at the stamp runs from 0.006 m to 0.095 m.
- **Why drop flights, and why the 95th percentile.** A drop's two climbs through the rangefinder's band are about a tenth of its rangefinder heights (235 of 2070), so the 95th percentile is a climb sample: 2.5 m/s times the flight's age. A delivery spends longer in the band (2820 heights), its climbs are a smaller share, and the same age shows as 0.01 to 0.05 m. The estimator's own scenario is a delivery.

Where the height is used (the release gate in a hover, the set-down at 0.12 m/s, the cone descent, the survey's level flight) a 40 ms age is worth under 0.02 m.

## What changed

`sim/height_check.py` holds a rangefinder height to the true height anywhere in the 50 ms before its stamp (`READING_PERIOD_S`), not at the stamp alone. The 0.10 m line is not moved. The error at the stamp alone is still reported (`p95_at_stamp_m`). A height 0.12 m off in a hover or a slow descent fails as before; so does a reading more than one period old, and a height that leads the truth.

## The seventeen recorded flights, judged again

`python3 sim/height_check.py --bag <run folder>` for each run folder under `sim/.state/suites/` that has a bag (flights on sim time, recorded as attempts or with `--record`). Heights from the rangefinder against the true height, 95th percentile:

| Suite folder | Heights | At the stamp alone | Over the reading's 50 ms (judged) | Largest (judged) | Real-time factor |
| --- | --- | --- | --- | --- | --- |
| `delivery-attempt-20261124T004410Z` | 2827 | 0.026 m | 0.003 m | 0.006 m | 0.92 |
| `delivery-attempt-20261124T005453Z` | 2823 | 0.008 m | 0.006 m | 0.012 m | 0.92 |
| `delivery-attempt_release-20261124T033337Z` | 2822 | 0.013 m | 0.012 m | 0.017 m | 0.83 |
| `delivery-attempt_release-20261124T034829Z` | 2820 | 0.047 m | 0.004 m | 0.012 m | 0.86 |
| `delivery-attempt_release_empty-20261124T033848Z` | 2822 | 0.024 m | 0.003 m | 0.010 m | 0.78 |
| `delivery-estimator-20261118T050446Z` | 2821 | 0.011 m | 0.005 m | 0.010 m | 0.91 |
| `delivery-height_estimator-20261120T051613Z` | 2821 | 0.019 m | 0.003 m | 0.008 m | 0.91 |
| `delivery-height_estimator-20261120T052612Z` | 2823 | 0.011 m | 0.004 m | 0.013 m | 0.92 |
| `drop-attempt_release-20261124T032429Z` | 2071 | 0.095 m | 0.010 m | 0.013 m | 0.89 |
| `drop-attempt_release-20261126T033823Z` | 2070 | 0.054 m | 0.009 m | 0.013 m | 0.83 |
| `drop-attempt_release_empty-20261124T032835Z` | 2077 | 0.007 m | 0.006 m | 0.014 m | 0.90 |
| `geofence-readiness_demo-20261124T012524Z` | 6133 | 0.013 m | 0.005 m | 0.030 m | 0.99 |
| `geofence-readiness_demo-20261124T013349Z` | 2231 | 0.009 m | 0.007 m | 0.018 m | 0.98 |
| `geofence-readiness_demo-20261124T014453Z` | 3008 | 0.033 m | 0.005 m | 0.020 m | 0.98 |
| `geofence-readiness_demo-20261124T015944Z` | 2996 | 0.006 m | 0.005 m | 0.018 m | 0.98 |
| `hover-step_probe-20261123T132013Z` | 1910 | 0.010 m | 0.010 m | 0.020 m | 0.93 |
| `hover-step_probe-20261123T133104Z` | 1923 | 0.044 m | 0.009 m | 0.160 m | 0.93 |

The age does not follow the real-time factor: 0.095 m at 0.89 and 0.007 m at 0.90.

## What this does not show

- **The G2 flights themselves were not recorded**, so their per-sample errors cannot be read back: for them there is the flight controller's log (the sensor right at 2.5 m/s) and the aggregate in their `height_check.json` (median 0.013 m, 95th percentile 0.104 m, largest 0.118 m, which is 2.5 m/s times 42 to 47 ms). The by-phase numbers above are from flights that were recorded, which are 2026 drops and deliveries with the estimator run on their bags afterwards. Gate G2 flown after this change is the direct check (`docs/results/2026-11-25-gate-G2.md`).
- **By climb rate** (the 21 and 37 ms above): the same replay with the error grouped by the true climb rate, done by hand for two flights; `--bag` prints the two percentiles only.
- **The aircraft.** The same mechanism is expected there (ArduPilot reads a TFmini at the same 20 Hz and streams it the same way), with the sensor's own delay on top. Nothing measures it yet: a climb at a known rate past a surface, rangefinder against the barometer's rate, would.
- The replay's truth has no tilt (the trace does not keep it), so the two rules that read the true tilt are not judged from a bag.
