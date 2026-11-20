#!/usr/bin/env bash
# Run the repo's tests that don't need ROS.
#
# A missing tool prints "SKIP: <what>" rather than passing quietly.
# With CHECK_STRICT=1 (CI sets it), any SKIP fails the run.
#
# Usage: scripts/check.sh            (PYTHON=python3.12 scripts/check.sh to pick an interpreter)

set -uo pipefail
cd "$(dirname "$0")/.."

PY=${PYTHON:-python3}
skips=0
failures=()

skip() {
    echo "SKIP: $*"
    skips=$((skips + 1))
}

# Run Python without a sourced ROS environment, so a pass here means the
# test really doesn't need ROS. $1 is the PYTHONPATH to use.
py() {
    local path=$1
    shift
    env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION PYTHONPATH="$path" "$PY" "$@"
}

has_module() {
    py "" -c "import $1" >/dev/null 2>&1
}

run_test() {
    local path=$1 file=$2
    echo "== $file"
    if ! py "$path" -m pytest -q -p no:cacheprovider "$file"; then
        failures+=("$file")
    fi
}

finish() {
    echo
    if ((${#failures[@]})); then
        echo "FAILED: ${failures[*]}"
        exit 1
    fi
    if ((skips)) && [[ ${CHECK_STRICT:-0} == 1 ]]; then
        echo "FAILED: $skips skipped (CHECK_STRICT=1)"
        exit 1
    fi
    if ((skips)); then
        echo "OK, but $skips skipped"
    else
        echo "OK"
    fi
    exit 0
}

if ! command -v "$PY" >/dev/null 2>&1; then
    skip "$PY not found, no tests ran"
    finish
fi

# The PR gates (hygiene, tests, tuning, size) need only git. See scripts/pr_checks.py.
echo "== scripts/pr_checks.py"
py "" scripts/pr_checks.py
case $? in
    0) ;;
    3) skips=$((skips + 1)) ;;
    *) failures+=("scripts/pr_checks.py") ;;
esac
echo

# Nothing secret-shaped in what the branch adds (the repo is public). See scripts/secret_scan.py.
echo "== scripts/secret_scan.py"
py "" scripts/secret_scan.py
case $? in
    0) ;;
    3) skips=$((skips + 1)) ;;
    *) failures+=("scripts/secret_scan.py") ;;
esac
echo

if ! has_module pytest; then
    skip "pytest not installed, no tests ran"
    finish
fi

run_test src/drone_mission_core src/drone_mission_core/test/test_tracking_projection.py
run_test src/drone_mission_core src/drone_mission_core/test/test_features.py
run_test src/drone_mission_core src/drone_mission_core/test/test_time_health.py
run_test src/drone_mission_core src/drone_mission_core/test/test_pose_buffer.py
run_test src/drone_mission_core src/drone_mission_core/test/test_detection_contract.py
run_test src/drone_mission_core src/drone_mission_core/test/test_target_access.py
run_test src/drone_target_cv src/drone_target_cv/test/test_detection_stamp.py
run_test src/drone_utils src/drone_utils/test/test_session_log_utils.py
run_test src/drone_utils src/drone_utils/test/test_sdf_objects.py
run_test "" src/argus_interfaces/test/test_interface_files.py
run_test "" src/drone_mission_demo/test/test_gps_waypoint_mission.py
run_test "" src/drone_mission_demo/test/test_package_drop_mission.py
run_test "" src/drone_mission_demo/test/test_time_guard_missions.py
run_test "" src/drone_mission_demo/test/test_actuation_states.py
run_test scripts scripts/test/test_pr_checks.py
run_test scripts scripts/test/test_secret_scan.py
run_test "" scripts/test/test_check_coverage.py
run_test "" scripts/test/test_guardian_read_guard.py
run_test "" sim/test/test_target_placement.py
run_test "" tools/test/test_build_ledger.py
run_test "" sim/test/test_nightly.py
run_test src/drone_mission_core src/drone_mission_core/test/test_centring_controller.py
run_test src/drone_mission_core src/drone_mission_core/test/test_release_gates.py
run_test src/drone_mission_core src/drone_mission_core/test/test_set_down.py
run_test src/drone_mission_core src/drone_mission_core/test/test_reacquire.py
run_test src/drone_mission_core src/drone_mission_core/test/test_survey_height.py
run_test "" tools/test/test_cen_loop_sim.py

# The 2027 build ledger must stay valid (see docs/build/SESSION.md).
echo "== docs/build/features.json"
if ! py "" tools/build_ledger.py check; then
    failures+=("docs/build/features.json")
fi

missing=()
for mod in cv2 numpy yaml; do
    has_module "$mod" || missing+=("$mod")
done
if ((${#missing[@]})); then
    skip "bullseye detector, feature-flag config, plumbing, rules and profile, safety limits, synth target, PR tuning-gate, sim camera, orientation-check, GPS flight-log, sim judge, sim site, sim scenario, sim runner and sim gate tests, tools/bullseye_try.py (missing: ${missing[*]})"
else
    run_test src/drone_target_cv src/drone_target_cv/test/test_target_detector.py
    run_test src/drone_target_cv src/drone_target_cv/test/test_contract_output.py
    run_test src/drone_target_cv src/drone_target_cv/test/test_contract_node.py
    run_test src/drone_target_cv src/drone_target_cv/test/test_ring_fit.py
    run_test src/drone_target_cv src/drone_target_cv/test/test_ring_fit_node.py
    run_test "" src/drone_mission_demo/test/test_feature_flags_config.py
    run_test src/drone_mission_core src/drone_mission_core/test/test_feature_plumbing.py
    run_test src/drone_mission_core src/drone_mission_core/test/test_feature_flag_rules.py
    run_test src/drone_mission_core src/drone_mission_core/test/test_safety_limits.py
    run_test "" src/drone_mission_core/test/test_executor_feature_overrides.py
    run_test "" src/drone_mission_core/test/test_executor_time_health.py
    run_test "" src/drone_mission_core/test/test_executor_pose_buffer.py
    run_test src/drone_mission_core src/drone_mission_core/test/test_projection_v2.py
    run_test src/drone_mission_core src/drone_mission_core/test/test_frame_projection.py
    run_test src/drone_mission_core src/drone_mission_core/test/test_assignment.py
    run_test src/drone_mission_core src/drone_mission_core/test/test_target_estimator.py
    run_test src/drone_mission_core src/drone_mission_core/test/test_coverage_planner.py
    run_test src/drone_mission_core src/drone_mission_core/test/test_estimator_feed.py
    run_test "" src/drone_mission_core/test/test_executor_target_estimator.py
    run_test "" src/drone_mission_core/test/test_survey_search.py
    run_test "" src/drone_mission_core/test/test_context_target_access.py
    run_test src/drone_mission_core src/drone_mission_core/test/test_height_estimator.py
    run_test "" src/drone_mission_demo/test/test_frame_time_missions.py
    run_test "" src/drone_mission_demo/test/test_centring_missions.py
    run_test "" src/drone_mission_demo/test/test_drop_release_gates.py
    run_test "" src/drone_mission_demo/test/test_delivery_set_down.py
    run_test "" src/drone_mission_demo/test/test_smart_reacquire.py
    run_test "" src/drone_mission_demo/test/test_sequences_2027.py
    run_test "" src/drone_mission_demo/test/test_search_pattern_mission.py
    run_test src/vision_pipeline src/vision_pipeline/test/test_logger_projection.py
    run_test src/vision_pipeline src/vision_pipeline/test/test_logger_node.py
    run_test "" tools/test/test_flag_profile.py
    run_test "" tools/test/test_sim_sequence_copies.py
    run_test "" tools/test/test_height_log.py
    run_test "" tools/test/test_ring_fit_try.py
    run_test "" tools/test/test_estimator_step_cost.py
    run_test "" tools/test/test_synth_targets.py
    run_test "" tools/test/test_frame_log_bench.py
    run_test "" tools/test/test_digit_reader_eval.py
    run_test scripts scripts/test/test_pr_checks_tuning.py
    run_test "" sim/test/test_sim_camera.py
    run_test "" sim/test/test_orientation_check.py
    run_test "" sim/test/test_gps_flight_log.py
    run_test "" sim/test/test_judge.py
    run_test "" sim/test/test_judge_cli.py
    run_test "" sim/test/test_site.py
    run_test "" sim/test/test_fence_upload.py
    run_test "" sim/test/test_fence_check.py
    run_test "" sim/test/test_scenario.py
    run_test "" sim/test/test_runner.py
    run_test "" sim/test/test_gates.py
    run_test "" sim/test/test_ab.py
    run_test "" sim/test/test_time_base.py
    run_test "" sim/test/test_rate_check.py
    run_test "" sim/test/test_projection_check.py
    run_test "" sim/test/test_estimator_check.py
    run_test "" sim/test/test_height_check.py
    run_test "" sim/test/test_replay_cdr.py
    run_test "" sim/test/test_replay_bag.py
    run_test "" sim/test/test_replay_core.py
    run_test "" sim/test/test_replay_latency.py
    run_test "" sim/test/test_replay_record.py
    run_test "" sim/test/test_wind_model.py
    run_test "" sim/test/test_wind.py
    run_test "" sim/test/test_payload_bridge.py
    run_test "" sim/test/test_sensors.py
    run_test "" sim/test/test_inject.py
    run_test "" sim/test/test_loc_check.py
    run_test src/drone_utils src/drone_utils/test/test_stream_keeper.py
    run_test src/argus_camera src/argus_camera/test/test_argus_camera.py
    run_test src/argus_camera src/argus_camera/test/test_capture_clock.py
    run_test "" src/argus_localisation/test/test_loc_pipeline.py
    run_test "" src/argus_localisation/test/test_loc_node.py
    run_test "" src/argus_localisation/test/test_marker_tracker.py
    run_test "" src/argus_localisation/test/test_final_position.py
    run_test "" src/argus_localisation/test/test_frame_logging.py
    run_test "" src/argus_localisation/test/test_digit_reader.py
    run_test "" src/argus_localisation/test/test_digit_vote.py
    run_test "" src/argus_localisation/test/test_judges_list.py

    # The detector must find the target that bullseye_try.py itself generates.
    echo "== tools/bullseye_try.py"
    tmp=$(mktemp -d)
    trap 'rm -rf "$tmp"' EXIT
    if py "" tools/bullseye_try.py --make-target "$tmp/target.png" \
        && out=$(py "" tools/bullseye_try.py --image "$tmp/target.png") \
        && echo "$out" \
        && grep -q '^FOUND' <<<"$out"; then
        echo "bullseye_try.py: target found"
    else
        echo "bullseye_try.py: generated target was not detected"
        failures+=("tools/bullseye_try.py")
    fi
fi

echo
echo "Not run (need ROS): src/drone_live_tests/test/test_live_landing_test_mission.py," \
    "src/drone_utils/test/test_stream_contract.py, src/drone_utils/test/test_{copyright,flake8,pep257}.py"

finish
