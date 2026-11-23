# Shared by platform/jetson/setup.sh and services.sh. Sourced, not run.
#
# A script built on this is a list of steps. Each step NAME has check_NAME
# (is it done? changes nothing) and apply_NAME (do it), and both leave one
# line for the checklist with `say`. run_steps does the rest.
#
# The tests run the scripts against a fake machine: ARGUS_SYSROOT goes in
# front of every file path, and ARGUS_SETUP_BIN is then the only place system
# commands are taken from.

SYSROOT=${ARGUS_SYSROOT:-}
SYSROOT=${SYSROOT%/}

# A system command. In a sandboxed run it comes from ARGUS_SETUP_BIN and nowhere else.
sys() {
    if [[ -n ${ARGUS_SETUP_BIN:-} ]]; then
        "$ARGUS_SETUP_BIN/$1" "${@:2}"
    else
        "$@"
    fi
}

# detail: the one line a check or an apply leaves for the checklist.
detail=""
say() { detail=$*; }

flight_user() {
    echo "${ARGUS_USER:-${SUDO_USER:-$(id -un)}}"
}

# guard_run <script name> <mode>: exit 2 unless this run is allowed to happen.
guard_run() {
    if [[ -n $SYSROOT && -z ${ARGUS_SETUP_BIN:-} ]]; then
        echo "$1: ARGUS_SYSROOT is set, so ARGUS_SETUP_BIN must name the folder of stand-in commands (a sandboxed run never calls the real ones)" >&2
        exit 2
    fi
    if [[ $2 == apply && -z $SYSROOT && $EUID -ne 0 ]]; then
        echo "$1: --apply changes the system, so it must run as root: sudo $0 --apply" >&2
        exit 2
    fi
}

# select_steps <script name> <only, comma-separated or empty> <every step...>
# sets `selected`; exit 2 for a step that does not exist.
select_steps() {
    local name=$1 only=$2 step
    shift 2
    selected=("$@")
    [[ -z $only ]] && return 0
    IFS=, read -r -a selected <<<"$only"
    for step in "${selected[@]}"; do
        if [[ " $* " != *" $step "* ]]; then
            echo "$name: no step called '$step' (steps: $*)" >&2
            exit 2
        fi
    done
}

# run_steps <mode: check|apply> <step...>
# Prints the checklist, sets `lines`, `summary` and `first_ok`, and returns 0
# when every step is ok. In apply mode a step's apply runs only when its check
# fails; when the first step fails nothing after it is applied (in setup.sh
# the first step says whether this is the right machine at all).
run_steps() {
    local mode=$1 step mark said ok=0 index=0
    shift
    lines=()
    first_ok=1
    for step in "$@"; do
        detail=""
        if "check_$step"; then
            mark=ok
        elif [[ $mode == apply && $first_ok == 1 ]]; then
            # Keep what the apply said when the step is still not done after it.
            if "apply_$step"; then
                if "check_$step"; then mark=ok; else mark=MISSING; fi
            else
                said=$detail
                if "check_$step"; then mark=ok; else mark=MISSING; fi
                [[ $mark == MISSING && -n $said ]] && detail=$said
            fi
        else
            mark=MISSING
        fi
        [[ $index == 0 && $mark != ok && ${FIRST_STEP_GATES:-0} == 1 ]] && first_ok=0
        [[ $mark == ok ]] && ok=$((ok + 1))
        lines+=("[$mark] $step: $detail")
        index=$((index + 1))
    done
    summary="CHECKLIST: $ok of $# ok"
    printf '%s\n' "${lines[@]}" "$summary"
    ((ok == $#))
}
