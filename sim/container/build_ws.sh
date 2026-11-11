#!/usr/bin/env bash
# Build our ROS packages inside the container. The build lives in sim/.ws/,
# separate from a host build/ install/, because the host is ROS 2 Jazzy and
# this is Humble.
#
#   sim/run_sim.sh --exec sim/container/build_ws.sh            # everything
#   sim/run_sim.sh --exec sim/container/build_ws.sh --packages-select drone_target_cv
set -Eeuo pipefail

# shellcheck disable=SC1091
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"

colcon --log-base "$ARGUS_WS/log" build \
    --base-paths "$ARGUS_REPO/src" \
    --build-base "$ARGUS_WS/build" \
    --install-base "$ARGUS_WS/install" \
    --symlink-install \
    --event-handlers console_cohesion- summary+ \
    "$@"
