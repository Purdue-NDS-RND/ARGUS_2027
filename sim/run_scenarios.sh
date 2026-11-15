#!/usr/bin/env bash
# Fly scenarios in the sim with nobody at the keyboard, and score each one.
#
#   sim/run_scenarios.sh smoke                       one 2026 delivery flight
#   sim/run_scenarios.sh nightly                     every scenario, 3 seeds, flags off and on
#   sim/run_scenarios.sh gates                       the acceptance gates' flights
#   sim/run_scenarios.sh delivery/clutter --seeds 1,2,3 --variants off,on
#   sim/run_scenarios.sh --list smoke                show the flights and stop
#
# Suites are in sim/scenarios/suites.yaml. A scenario is named as
# <mission>/<name> or by its file. Each flight:
#   1. builds the scenario into a run folder (sim/scenario/build.py),
#   2. starts a fresh sim container in the background and waits for its READY
#      file (up to READY_TIMEOUT_S, default 900 s),
#   3. flies the mission while recording true poses (sim/runner/fly.py),
#      until the vehicle disarms or the scenario's timeout, in sim time,
#   4. adds ground-station commands from the MAVLink log, scores the run
#      (sim/judge) and writes report.md,
#   5. stops the container.
# Then it prints a table of flags off (2026) against flags on (2027).
#
# Output: sim/.state/suites/<name>-<time>/summary.md, and one folder per
# flight under runs/ with score.json, report.md, trace.jsonl and the logs.
# Exit code 0 only if every flight ended OK.
#
# The laptop is kept awake for the whole suite (systemd-inhibit, as
# sim/gps/fly_all.sh does). Options: --seeds a,b,c  --variants off,on  --list
set -Eeuo pipefail

SIM_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
source "$SIM_DIR/gps/common.sh"   # ARGUS_REPO, inhibit_sleep, is_running
CONTAINER=argus-sim
# Wall-clock seconds to wait for the sim's READY file. Every other duration
# of a run is in sim time, but there is no sim clock until Gazebo is up.
READY_TIMEOUT_S=${READY_TIMEOUT_S:-900}
PY=${PYTHON:-python3}
# The sim's own start script; a test replaces it to try this loop without a sim.
RUN_SIM=${ARGUS_RUN_SIM:-$SIM_DIR/run_sim.sh}

usage() { sed -n '2,25p' "$0"; }

name= seeds= variants= list_only=0
while (($#)); do
    case $1 in
        --seeds) seeds=${2:?--seeds needs a value}; shift ;;
        --variants) variants=${2:?--variants needs a value}; shift ;;
        --list) list_only=1 ;;
        --help|-h) usage; exit 0 ;;
        -*) echo "ERROR: unknown option: $1" >&2; usage >&2; exit 2 ;;
        *)
            if [[ -n $name ]]; then
                echo "ERROR: one suite or scenario at a time (got '$name' and '$1')" >&2
                exit 2
            fi
            name=$1
            ;;
    esac
    shift
done
if [[ -z $name ]]; then
    usage >&2
    exit 2
fi

cd "$ARGUS_REPO"
suite_args=("$name")
[[ -n $seeds ]] && suite_args+=(--seeds "$seeds")
[[ -n $variants ]] && suite_args+=(--variants "$variants")
flights=$("$PY" "$SIM_DIR/runner/suites.py" "${suite_args[@]}") || exit 2
mapfile -t flights <<<"$flights"

if ((list_only)); then
    printf '%s\n' "${flights[@]}"
    exit 0
fi

# Hold off suspend (idle, sleep keys and the lid) until the suite is done.
inhibit_args=("$name")
[[ -n $seeds ]] && inhibit_args+=(--seeds "$seeds")
[[ -n $variants ]] && inhibit_args+=(--variants "$variants")
# By full path: this script has changed directory since it was started.
inhibit_sleep "$SIM_DIR/run_scenarios.sh" "${inhibit_args[@]}"

if is_running "$CONTAINER"; then
    echo "ERROR: the sim is already running. Stop it first: sim/run_sim.sh --down" >&2
    exit 1
fi

suite_dir="$SIM_DIR/.state/suites/${name//\//-}-$(date -u +%Y%m%dT%H%M%SZ)"
suite_dir=${suite_dir%.yaml}
mkdir -p "$suite_dir/runs"
echo "== Suite '$name': ${#flights[@]} flight(s); results in ${suite_dir#"$ARGUS_REPO"/}"

stop_sim() {
    "$RUN_SIM" --down >/dev/null 2>&1 || true
}
trap 'echo; echo "Interrupted: stopping the sim."; stop_sim; exit 130' INT TERM

# fly_one <run folder>: start the sim on it, fly, score. Never fails the
# suite: what went wrong ends up in the run's report.
fly_one() {
    local run_dir=$1 started=$SECONDS
    stop_sim
    if ! RUN_DIR="$run_dir" HEADLESS=1 "$RUN_SIM" --detach >"$run_dir/sim_start.log" 2>&1; then
        echo "   the sim did not start (see sim_start.log)"
        stop_sim
        return 0
    fi
    echo "   waiting for the sim to be READY (up to ${READY_TIMEOUT_S} s)..."
    while [[ ! -f $run_dir/READY ]]; do
        if ! is_running "$CONTAINER"; then
            echo "   the sim stopped before it was READY (see sim.log)"
            break
        fi
        if ((SECONDS - started >= READY_TIMEOUT_S)); then
            echo "   the sim was not READY after ${READY_TIMEOUT_S} s (see sim.log)"
            break
        fi
        sleep 2
    done
    if [[ -f $run_dir/READY ]]; then
        echo "   READY after $((SECONDS - started)) s; flying..."
        "$RUN_SIM" --exec python3 sim/runner/fly.py "$run_dir" </dev/null >"$run_dir/fly.log" 2>&1 || true
        tail -n 1 "$run_dir/fly.log" | sed 's/^/   /'
        "$RUN_SIM" --exec env -u PYTHONNOUSERSITE python3 sim/runner/gcs_audit.py "$run_dir" \
            </dev/null >>"$run_dir/fly.log" 2>&1 || echo "   the ground-station audit failed (see fly.log)"
    fi
    docker logs "$CONTAINER" >"$run_dir/sim.log" 2>&1 || true
    stop_sim
    if [[ -f $run_dir/trace.jsonl ]]; then
        "$PY" "$SIM_DIR/judge/cli.py" "$run_dir" >"$run_dir/judge.log" 2>&1 || echo "   the judge could not score it (see judge.log)"
    fi
}

run_dirs=()
number=0
for flight in "${flights[@]}"; do
    read -r scenario seed variant <<<"$flight"
    number=$((number + 1))
    label="$(basename "$(dirname "$scenario")")-$(basename "${scenario%.yaml}")-s${seed}-${variant}"
    run_dir="$suite_dir/runs/$label"
    echo
    echo "== Flight $number of ${#flights[@]}: $label"
    if ! "$PY" "$SIM_DIR/scenario/build.py" "$scenario" --seed "$seed" --variant "$variant" --out "$run_dir" >"$suite_dir/build-$label.log" 2>&1; then
        echo "   the scenario did not build:"
        sed 's/^/     /' "$suite_dir/build-$label.log"
        mkdir -p "$run_dir"
    else
        fly_one "$run_dir"
    fi
    run_dirs+=("$run_dir")
    "$PY" "$SIM_DIR/runner/result.py" "$run_dir" | sed 's/^/   /' || true
done
trap - INT TERM

echo
"$PY" "$SIM_DIR/runner/result.py" --suite "$suite_dir" --title "Suite $name, $(date -u +%Y-%m-%d)" "${run_dirs[@]}" >"$suite_dir/results.log" && status=0 || status=1
sed -n '/^# Suite/,$p' "$suite_dir/results.log"
echo "Summary: ${suite_dir#"$ARGUS_REPO"/}/summary.md"
exit "$status"
