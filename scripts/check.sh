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

if ! has_module pytest; then
    skip "pytest not installed, no tests ran"
    finish
fi

run_test src/drone_mission_core src/drone_mission_core/test/test_tracking_projection.py
run_test src/drone_utils src/drone_utils/test/test_session_log_utils.py
run_test "" src/drone_mission_demo/test/test_gps_waypoint_mission.py
run_test "" src/drone_mission_demo/test/test_package_drop_mission.py

missing=()
for mod in cv2 numpy yaml; do
    has_module "$mod" || missing+=("$mod")
done
if ((${#missing[@]})); then
    skip "bullseye detector tests and tools/bullseye_try.py (missing: ${missing[*]})"
else
    run_test src/drone_target_cv src/drone_target_cv/test/test_target_detector.py

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
