#!/usr/bin/env bash
# The ground station's tools in the Humble container (PLT-14).
#
#   tools/ground_station.sh judges_list_view.py attempts/<attempt>/judges_list.csv --site field/<name>/site.yaml
#   tools/ground_station.sh attempt_download.py pilot@aircraft --list      (runs on the laptop itself: see below)
#   tools/ground_station.sh post_flight.py attempts/<attempt>
#   tools/ground_station.sh ros2 bag info attempts/<attempt>/bag
#   tools/ground_station.sh --print <the same>        print the docker command and run nothing
#
# The club's laptops run ROS 2 Jazzy; the aircraft and this repository are
# Humble. This runs one command in the repository's own Humble image (the
# sim's, argus-sim:humble: `sim/run_sim.sh --build` makes it), with the
# repository mounted where it is, as the user who called it, on the host's
# network. A first word that is a file in tools/ is run with python3; anything
# else is run as it is, with ROS sourced. Nothing is installed and nothing
# outside the repository is written.
#
# attempt_download.py is the one tool that is run on the laptop itself, with
# the laptop's python3: it needs ssh and rsync and the user's own keys and
# known hosts, and the image has neither ssh nor rsync. It has no ROS in it
# and needs nothing but the standard library.
#
# The Python tools have no ROS in them and also run on the laptop itself with
# python3; the container is the way that is the same on every laptop.

set -euo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
IMAGE=${ARGUS_SIM_IMAGE:-argus-sim:humble}
DOCKER=${ARGUS_DOCKER:-docker}

print_only=0
if [[ ${1:-} == --print ]]; then
    print_only=1
    shift
fi
if (($# == 0)) || [[ $1 == -h || $1 == --help ]]; then
    sed -n '2,9p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' >&2
    exit 2
fi

if [[ $1 == attempt_download.py ]]; then
    # On the laptop itself: its ssh, its rsync, its keys (the image has no ssh and no rsync).
    if ((print_only)); then
        printf '%q ' python3 "$HERE/$1" "${@:2}"
        printf '\n'
        exit 0
    fi
    for needed in python3 ssh rsync; do
        if ! command -v "$needed" >/dev/null 2>&1; then
            echo "ground_station.sh: attempt_download.py runs on this laptop and needs $needed, which is not installed" >&2
            exit 2
        fi
    done
    exec python3 "$HERE/$1" "${@:2}"
fi
if [[ -f $HERE/$1 && $1 == *.py && $1 != */* ]]; then
    inner=(python3 "tools/$1" "${@:2}")
else
    inner=("$@")
fi

args=(run --rm --network host --user "$(id -u):$(id -g)" -e HOME=/tmp/argus-home -v "$REPO:$REPO" -w "$REPO")
if [[ -t 0 && -t 1 ]]; then
    args+=(-it)
fi
# The command's words are passed as arguments, never pasted into the shell line.
args+=("$IMAGE" bash -c 'source /opt/ros/humble/setup.bash && exec "$@"' ground_station "${inner[@]}")

if ((print_only)); then
    printf '%q ' "$DOCKER" "${args[@]}"
    printf '\n'
    exit 0
fi
if ! command -v "$DOCKER" >/dev/null 2>&1; then
    echo "ground_station.sh: docker is not installed. The Python tools also run without it: python3 tools/$1 ..." >&2
    exit 2
fi
if ! "$DOCKER" image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "ground_station.sh: the image $IMAGE is not on this laptop: sim/run_sim.sh --build makes it" >&2
    exit 2
fi
exec "$DOCKER" "${args[@]}"
