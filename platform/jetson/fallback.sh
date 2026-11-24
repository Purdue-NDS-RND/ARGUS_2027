#!/usr/bin/env bash
# The 2026 fallback workspace (OPS-06): the code that flew in June 2026, built and ready beside the 2027 one.
#
#   platform/jetson/fallback.sh --check                       is it there, at the tag, built?
#   platform/jetson/fallback.sh --apply                       make it so (clone, check out the tag, build)
#   platform/jetson/fallback.sh launch <launch file> [launch arguments...]
#                                                             fly the 2026 code from it, e.g.
#   platform/jetson/fallback.sh launch package_delivery_live.launch.py
#
# The workspace is a clone of this repository at the tag comp-2026, in
# ~/ARGUS_2026_fallback (--dir says otherwise), with its own build. It shares
# nothing with the 2027 workspace but the tag: a broken build, a half-edited
# file or a bad flag here cannot reach it. It is made from this checkout, so
# the field needs no network for it.
#
# Steps: `clone` (the folder is a clone), `at_tag` (its HEAD is comp-2026's
# commit and nothing in it is changed), `build` (colcon has built that
# commit). --apply never discards anything: a fallback with local changes is
# reported and left alone.
#
# `launch` runs the checks first and starts nothing unless all three are ok
# and the boot settings (/etc/argus/boot.env, services.sh) are the 2026
# start-up: MAVROS as 2026 started it and no 2027 camera driver, which would
# hold the camera the 2026 launch opens itself. It says how to set them; it
# does not change them (that needs root). Then it sources ROS and the
# fallback's own install and runs `ros2 launch drone_mission_demo <launch
# file>` there.
#
# Run it as the flight user, not as root: the workspace is that user's.
# Exit code: 0 when every step is ok (or the launch's own), 1 when one is not, 2 for a wrong call.

set -uo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
# shellcheck source=platform/jetson/lib.sh
source "$HERE/lib.sh"

usage() {
    sed -n '2,8p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' >&2
}

TAG=comp-2026
STEPS=(clone at_tag build)
ROS_SETUP=$SYSROOT${ARGUS_ROS_SETUP:-/opt/ros/humble/setup.bash}
BUILT_MARK=install/.argus_fallback_built  # holds the commit the install was built from

mode="" only="" dir="" repo=$REPO launch_args=()
while (($#)); do
    case $1 in
        --check) mode=check ;;
        --apply) mode=apply ;;
        launch)
            mode=launch
            shift
            launch_args=("$@")
            break
            ;;
        --only)
            only=${2:-}
            shift
            ;;
        --dir)
            dir=${2:-}
            shift
            ;;
        --repo)
            repo=${2:-}
            shift
            ;;
        -h | --help)
            usage
            exit 0
            ;;
        *)
            echo "fallback.sh: unknown argument '$1'" >&2
            usage
            exit 2
            ;;
    esac
    shift
done
if [[ -z $mode ]]; then
    usage
    exit 2
fi
if [[ -n $SYSROOT && -z ${ARGUS_SETUP_BIN:-} ]]; then
    echo "fallback.sh: ARGUS_SYSROOT is set, so ARGUS_SETUP_BIN must name the folder of stand-in commands (a sandboxed run never calls the real ones)" >&2
    exit 2
fi
if [[ -z $SYSROOT && $EUID -eq 0 ]]; then
    echo "fallback.sh: run it as the flight user, not as root: the workspace would be root's, and the flight user could not build or launch it" >&2
    exit 2
fi
if [[ $mode == launch && ${#launch_args[@]} -eq 0 ]]; then
    echo "fallback.sh: launch needs a launch file of drone_mission_demo, e.g. package_delivery_live.launch.py" >&2
    exit 2
fi
FALLBACK=${dir:-$SYSROOT${HOME}/ARGUS_2026_fallback}

tag_commit() {
    sys git -C "$repo" rev-parse --verify --quiet "$TAG^{commit}"
}

check_clone() {
    if [[ -d $FALLBACK/.git ]]; then
        say "$FALLBACK is a clone"
        return 0
    fi
    say "$FALLBACK is not there"
    return 1
}

apply_clone() {
    if [[ -e $FALLBACK ]]; then
        say "$FALLBACK exists and is not a clone: move it away first (nothing is deleted here)"
        return 1
    fi
    if ! tag_commit >/dev/null; then
        say "this checkout has no tag $TAG to clone from"
        return 1
    fi
    sys git clone --quiet --no-hardlinks --branch "$TAG" "$repo" "$FALLBACK" 2>/dev/null || {
        say "git could not clone $repo at $TAG into $FALLBACK"
        return 1
    }
}

check_at_tag() {
    local want have changed
    want=$(tag_commit) || {
        say "this checkout has no tag $TAG"
        return 1
    }
    have=$(sys git -C "$FALLBACK" rev-parse HEAD 2>/dev/null) || {
        say "no clone to look at"
        return 1
    }
    changed=$(sys git -C "$FALLBACK" status --porcelain --untracked-files=no 2>/dev/null | wc -l)
    if [[ $have != "$want" ]]; then
        say "at ${have:0:12}, and $TAG is ${want:0:12}"
        return 1
    fi
    if ((changed > 0)); then
        say "at $TAG, with $changed file(s) changed: not the code that flew"
        return 1
    fi
    say "at $TAG (${want:0:12}), nothing changed"
}

apply_at_tag() {
    local changed
    changed=$(sys git -C "$FALLBACK" status --porcelain --untracked-files=no 2>/dev/null | wc -l)
    if ((changed > 0)); then
        say "$changed file(s) are changed in $FALLBACK: left alone. Look at them (git -C $FALLBACK status), then move the folder away and run --apply again"
        return 1
    fi
    sys git -C "$FALLBACK" fetch --quiet "$repo" "refs/tags/$TAG:refs/tags/$TAG" 2>/dev/null
    sys git -C "$FALLBACK" checkout --quiet --detach "$TAG" 2>/dev/null || {
        say "git could not check out $TAG in $FALLBACK"
        return 1
    }
}

check_build() {
    local want built
    want=$(tag_commit) || {
        say "this checkout has no tag $TAG"
        return 1
    }
    if [[ ! -f $FALLBACK/install/setup.bash ]]; then
        say "not built (no install/setup.bash)"
        return 1
    fi
    built=$(cat "$FALLBACK/$BUILT_MARK" 2>/dev/null || true)
    if [[ $built != "$want" ]]; then
        say "the install was built from ${built:-an unknown commit}, not from $TAG"
        return 1
    fi
    say "built from $TAG"
}

apply_build() {
    # Only the tag's own code is built, and only then is the build marked as the tag's: a clone that is not
    # there, is somewhere else or has a file changed is not built at all.
    if ! check_clone || ! check_at_tag; then
        say "not built: the clone must be at $TAG with nothing changed first ($detail)"
        return 1
    fi
    if [[ ! -f $ROS_SETUP ]]; then
        say "ROS is not installed ($ROS_SETUP): run platform/jetson/setup.sh first"
        return 1
    fi
    (
        set +u
        # shellcheck disable=SC1090
        source "$ROS_SETUP" || exit 1
        cd "$FALLBACK" || exit 1
        sys colcon --log-base "$FALLBACK/log" build --base-paths "$FALLBACK/src" --build-base "$FALLBACK/build" --install-base "$FALLBACK/install"
    ) >"$FALLBACK/colcon.log" 2>&1 || {
        say "colcon build failed: see $FALLBACK/colcon.log"
        return 1
    }
    sys git -C "$FALLBACK" rev-parse HEAD >"$FALLBACK/$BUILT_MARK"
}

# boot_problem: why the machine's boot settings are not the 2026 start-up, or nothing. The 2026 launch files open
# the camera themselves and expect MAVROS as 2026 started it; the 2027 settings start the 2027 camera driver and
# MAVROS with the 2027 plugin list.
boot_problem() {
    local env_file=$SYSROOT/etc/argus/boot.env mavros camera
    [[ -f $env_file ]] || return 0  # no boot services installed: nothing of 2027 starts at boot
    mavros=$(sed -n 's/^ARGUS_BOOT_MAVROS=//p' "$env_file" | tail -n 1)
    camera=$(sed -n 's/^ARGUS_BOOT_CAMERA=//p' "$env_file" | tail -n 1)
    if [[ ${camera:-off} != off ]]; then
        echo "ARGUS_BOOT_CAMERA=$camera: the 2027 camera driver holds the camera the 2026 launch opens itself"
    elif [[ ${mavros:-apm} != apm ]]; then
        echo "ARGUS_BOOT_MAVROS=$mavros: MAVROS is started the 2027 way, not as the 2026 code expects it"
    fi
}

if [[ $mode == launch ]]; then
    if ! run_steps check "${STEPS[@]}" >/dev/null; then
        printf '%s\n' "${lines[@]}" >&2
        echo "fallback.sh: the fallback is not ready, so nothing was started. platform/jetson/fallback.sh --apply makes it ready" >&2
        exit 1
    fi
    problem=$(boot_problem)
    if [[ -n $problem ]]; then
        echo "fallback.sh: $problem. Nothing was started. In /etc/argus/boot.env set ARGUS_BOOT_MAVROS=apm and ARGUS_BOOT_CAMERA=off, then: sudo systemctl restart argus-mavros argus-camera" >&2
        exit 1
    fi
    echo "fallback.sh: launching ${launch_args[0]} from $FALLBACK ($TAG)"
    set +u
    # shellcheck disable=SC1090
    source "$ROS_SETUP" || exit 1
    # shellcheck disable=SC1091
    source "$FALLBACK/install/setup.bash" || exit 1
    if [[ -n ${ARGUS_SETUP_BIN:-} ]]; then
        "$ARGUS_SETUP_BIN/ros2" launch drone_mission_demo "${launch_args[@]}"
        exit $?
    fi
    exec ros2 launch drone_mission_demo "${launch_args[@]}"
fi

select_steps fallback.sh "$only" "${STEPS[@]}"
echo "fallback.sh --$mode: $FALLBACK"
run_steps "$mode" "${selected[@]}"
