"""Score one run: the mission's judge plus the checks every mission gets."""

from __future__ import annotations

from . import missions
from .fence import fence_breaches
from .missions import SIX_FEET_M, TWENTY_FEET_M, TWENTY_FIVE_FEET_M, Run
from .record import AIRBORNE_HEIGHT_M, Record

# Settings every mission has. judge.yaml's `settings:` overrides any of them.
COMMON = {
    "autonomy_bonus_fraction": 0.25,  # +25% when the log shows no human input; the rules give it
    # only to localisation, drop, delivery and recovery, so the other judges set it to 0
    "takeoff_zone_radius_m": 5.0,  # being low this close to the takeoff point is takeoff or landing
    "airborne_height_m": AIRBORNE_HEIGHT_M,
}

# Defaults from FEATURES.md, SIM-08. Values the rules have not fixed yet are
# parameters here so an answer from the organisers is a judge.yaml change.
MISSIONS = {
    "waypoint_navigation": (
        missions.judge_waypoint_navigation,
        {"tolerance_m": 1.0, "dimensions": 3, "points_per_waypoint": 1.0, "autonomy_bonus_fraction": 0.0},
    ),
    "time_trial": (
        missions.judge_time_trial,
        {"tolerance_m": 4.0, "dimensions": 3, "reference_times_s": [], "points_fastest": 12.0, "points_slowest": 8.0, "autonomy_bonus_fraction": 0.0},
    ),
    "localisation": (
        missions.judge_localisation,
        {
            "tolerance_m": 2.0,
            "points_per_marker": 1.0,
            "wrong_id_penalty": 0.5,
            "wrong_location_penalty": 0.5,
            "both_wrong_penalty": 1.0,
            "height_penalty_below_m": TWENTY_FIVE_FEET_M,
            "height_penalty_points": 3.0,
            "height_zero_below_m": TWENTY_FEET_M,
        },
    ),
    "drop": (
        missions.judge_drop,
        {
            "max_points": 14.0,
            "points_per_metre": 1.0,
            "min_height_m": SIX_FEET_M,
            "low_release_penalty": 6.0,
            "low_flight_penalty": 6.0,
            # Gate G1's hold, not a rule of the competition: set hold_height_m and the details
            # gain the vehicle's true distance from the target while it held there before the release.
            "hold_height_m": None,
            "hold_band_m": 0.5,
            "hold_settle_s": 2.0,
            "hold_min_s": 10.0,
            "hold_point_body_m": None,  # [forward, left] of the vehicle's centre: judge the hold at the payload's release point
        },
    ),
    "delivery": (
        missions.judge_delivery,
        {"max_points": 13.0, "points_per_metre": 1.0, "max_release_height_m": SIX_FEET_M, "impulse_limit_ns": None, "weight_bonus_per_500g": 0.25},
    ),
    "recovery": (
        missions.judge_recovery,
        {"max_points": 16.0, "points_per_metre": 1.0, "lift_height_m": 0.5, "max_release_height_m": SIX_FEET_M, "return_point": "takeoff", "max_touchdown_mps": 1.5, "landed_below_m": 1.5},
    ),
    # Not a competition mission: the flight SIM-03's wind is checked on.
    "hover": (
        missions.judge_hover,
        {"max_drift_m": 3.0, "min_height_m": 2.0, "min_hold_s": 20.0, "autonomy_bonus_fraction": 0.0},
    ),
    # Not a competition mission: the SIM-02 pass line, scored from true positions.
    "geofence": (
        missions.judge_geofence,
        # held_within_m: None, or how near the line (inside) a vehicle the flight controller stops must come to pass
        # without crossing, and held_min_s how long it must stay there (OPS-02's demonstration).
        {"max_response_s": 2.0, "max_excursion_m": 10.0, "response_modes": ["RTL", "SMART_RTL", "LAND", "BRAKE"], "held_within_m": None, "held_min_s": 3.0,
         "autonomy_bonus_fraction": 0.0},  # fmt: skip
    ),
}


def default_settings(mission: str) -> dict:
    if mission not in MISSIONS:
        raise ValueError(f"unknown mission '{mission}' (the judges are: {', '.join(MISSIONS)})")
    return {**COMMON, **MISSIONS[mission][1]}


def judge_run(mission: str, record: Record, truth: dict, settings: dict | None = None, lists: dict | None = None) -> dict:
    """Score `mission` from a run's record and truth.

    `settings` overrides the mission's defaults; an unknown name is an error,
    so a typo cannot silently leave a default in place. `lists` carries the
    text of the localisation lists: {"list": ..., "merged_list": ...}.
    """

    merged = default_settings(mission)
    for name, value in (settings or {}).items():
        if name not in merged:
            raise ValueError(f"unknown judge setting '{name}' for {mission} (it has: {', '.join(sorted(merged))})")
        merged[name] = value

    notes: list[str] = []
    flights = record.flights(merged["airborne_height_m"])
    common = {
        "flights": len(flights),
        "single_flight": len(flights) == 1,
        "scored_flight": None,
        "autonomous": False,
        "human_inputs": [],
        "fence_breaches": [],
        "reported_fence_breaches": 0,
        "failsafes": [],
    }
    score = {"mission": mission, "points": 0.0, "base_points": 0.0, "autonomy_bonus": 0.0, "details": {}, "common": common, "settings": merged, "notes": notes}
    if not flights:
        notes.append("the vehicle never flew: nothing to score")
        return score
    if len(flights) > 1:
        notes.append(f"{len(flights)} flights: only the first is scored")

    flight = flights[0]
    t_end = flights[-1].t_disarm
    common["scored_flight"] = {"t_arm": flight.t_arm, "t_disarm": flight.t_disarm}
    common["fence_breaches"] = fence_breaches(record, truth.get("fence"), flight.t_arm, t_end)
    common["reported_fence_breaches"] = len(record.events_of("fence_breach", t0=flight.t_arm, t1=t_end))
    common["failsafes"] = [
        {"t": item["t"], "what": item.get("what", "")} for item in record.events_of("failsafe", t0=flight.t_arm, t1=t_end)
    ]
    common["human_inputs"] = [
        {"t": item["t"], "event": item["event"], "what": item.get("what", "")}
        for item in record.events_of("gcs_command", "rc_override", t0=flight.t_arm, t1=flight.t_disarm)
    ]
    common["autonomous"] = not common["human_inputs"]

    run = Run(record=record, flight=flight, truth=truth, settings=merged, lists=lists or {}, notes=notes)
    base, details = MISSIONS[mission][0](run)
    bonus = base * merged["autonomy_bonus_fraction"] if common["autonomous"] else 0.0
    score.update({"points": base + bonus, "base_points": base, "autonomy_bonus": bonus, "details": details})
    return score
