#!/usr/bin/env bash
# ARGUS mission simulator: ArduCopter SITL, Gazebo Harmonic, ROS 2 Humble,
# MAVROS and our code, in one container. See sim/README.md.
#
# Display, GPU and user handling follow the club Drone_Sim_Docker run.sh.
set -Eeuo pipefail

SIM_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ARGUS_REPO=$(cd -- "$SIM_DIR/.." && pwd)
export ARGUS_REPO
CONTAINER=argus-sim

usage() {
    cat <<'EOF'
Usage: sim/run_sim.sh [option]

  (no option)      Start the sim: build our code, Gazebo, target, camera
                   bridge, SITL and MAVROS. Ctrl+C stops it.
  --build          Build (or rebuild) the argus-sim:humble image, then exit
  --shell          Open a shell in the running sim with ROS sourced
  --exec CMD...    Run one command in the running sim with ROS sourced
  --down           Stop and remove the sim container
  --help           This text

Environment: SOFTWARE_RENDERING=1, HEADLESS=1 (no Gazebo window),
SKIP_BUILD=1 (no colcon build), SPAWN_TARGET=0, WIPE_PARAMS=1 (reset SITL
parameters once).
EOF
}

action=up
case "${1:-}" in
    "") ;;
    --build) action=build ;;
    --shell) action=shell ;;
    --exec) action=exec; shift ;;
    --down) action=down ;;
    --help|-h) usage; exit 0 ;;
    *) echo "ERROR: unknown option: $1" >&2; usage >&2; exit 2 ;;
esac

if [[ $(uname -s) != Linux ]]; then
    echo "ERROR: this needs native Linux (Docker host networking and X11 sockets)." >&2
    exit 1
fi
if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: Docker is not installed. See sim/README.md, step 1." >&2
    exit 1
fi
if ! docker compose version >/dev/null 2>&1; then
    echo "ERROR: the Docker Compose plugin is missing (package docker-compose-plugin)." >&2
    exit 1
fi
if ! docker info >/dev/null 2>&1; then
    cat >&2 <<EOF
ERROR: this user cannot reach the Docker daemon. Add yourself to the docker
group, then log out and back in:
  sudo usermod -aG docker "${USER:-your-user}"
EOF
    exit 1
fi

running() {
    [[ $(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null) == true ]]
}
require_running() {
    if ! running; then
        echo "ERROR: the sim is not running. Start it first in another terminal: sim/run_sim.sh" >&2
        exit 1
    fi
}

case $action in
    shell)
        require_running
        exec docker exec -it -w "$ARGUS_REPO" "$CONTAINER" \
            bash --rcfile "$ARGUS_REPO/sim/container/bashrc" -i
        ;;
    exec)
        require_running
        if (($# == 0)); then
            echo "ERROR: --exec needs a command" >&2
            exit 2
        fi
        # Always pass stdin (heredocs, pipes); add a TTY only when we have one.
        tty_flag=(-i)
        [[ -t 0 && -t 1 ]] && tty_flag=(-it)
        exec docker exec "${tty_flag[@]}" -w "$ARGUS_REPO" "$CONTAINER" \
            bash -c 'source "$ARGUS_REPO/sim/container/env.sh" && exec "$@"' argus-exec "$@"
        ;;
esac

export HOST_UID=${HOST_UID:-$(id -u)}
export HOST_GID=${HOST_GID:-$(id -g)}
VIDEO_GID=${VIDEO_GID:-$(getent group video | cut -d: -f3 || true)}
RENDER_GID=${RENDER_GID:-$(getent group render | cut -d: -f3 || true)}
export VIDEO_GID=${VIDEO_GID:-$HOST_GID}
export RENDER_GID=${RENDER_GID:-$HOST_GID}
export ARGUS_SIM_IMAGE=${ARGUS_SIM_IMAGE:-argus-sim:humble}
export ARGUS_BASE_IMAGE=${ARGUS_BASE_IMAGE:-ardupilot-gz:4.6.3}

compose=(docker compose -f "$SIM_DIR/compose.yaml")
if [[ -d /dev/dri ]]; then
    compose+=(-f "$SIM_DIR/compose.gpu.yaml")
fi

if [[ $action == down ]]; then
    "${compose[@]}" down --remove-orphans
    exit 0
fi

if ! docker image inspect "$ARGUS_BASE_IMAGE" >/dev/null 2>&1; then
    cat >&2 <<EOF
ERROR: the club base image $ARGUS_BASE_IMAGE is missing. Build it once from
the club simulator repo (about 30 minutes), then rerun this:
  cd ~/Drone_Sim_Docker && make build
EOF
    exit 1
fi

if [[ $action == build ]] || ! docker image inspect "$ARGUS_SIM_IMAGE" >/dev/null 2>&1; then
    echo "Building $ARGUS_SIM_IMAGE on top of $ARGUS_BASE_IMAGE (first build: 10-20 minutes)..."
    "${compose[@]}" build
fi
[[ $action == build ]] && exit 0

if running; then
    echo "ERROR: the sim is already running. Use --shell, or stop it with: sim/run_sim.sh --down" >&2
    exit 1
fi
if [[ $(docker inspect -f '{{.State.Running}}' ardupilot-gz-sim 2>/dev/null) == true ]]; then
    echo "ERROR: the club simulator (ardupilot-gz-sim) is running and uses the same ports." >&2
    echo "Stop it first: cd ~/Drone_Sim_Docker && make down" >&2
    exit 1
fi

software=${SOFTWARE_RENDERING:-0}
case "${software,,}" in
    1|true|yes|on) export SOFTWARE_RENDERING=1 LIBGL_ALWAYS_SOFTWARE=1 ;;
    0|false|no|off|"")
        export SOFTWARE_RENDERING=0 LIBGL_ALWAYS_SOFTWARE=0
        if [[ ! -d /dev/dri ]]; then
            echo "ERROR: /dev/dri is missing, so GPU rendering cannot start." >&2
            echo "Retry with software rendering: SOFTWARE_RENDERING=1 sim/run_sim.sh" >&2
            exit 1
        fi
        ;;
    *) echo "ERROR: SOFTWARE_RENDERING must be 0 or 1." >&2; exit 2 ;;
esac

XAUTH_FILE=
if [[ ${HEADLESS:-0} != 1 ]]; then
    if [[ -z ${DISPLAY:-} ]]; then
        echo "ERROR: DISPLAY is empty. Run this from a terminal on the desktop, or use HEADLESS=1." >&2
        exit 1
    fi
    if ! command -v xauth >/dev/null 2>&1; then
        echo "ERROR: xauth is missing (sudo apt install xauth)." >&2
        exit 1
    fi
    display_number=${DISPLAY##*:}
    display_number=${display_number%%.*}
    if [[ ! -S /tmp/.X11-unix/X${display_number} ]]; then
        echo "ERROR: DISPLAY=$DISPLAY has no socket in /tmp/.X11-unix." >&2
        exit 1
    fi
    # Copy only this display's cookie into a short-lived file. FamilyWild
    # (ffff) makes it valid for the container's hostname too.
    XAUTH_FILE=$(mktemp "${XDG_RUNTIME_DIR:-/tmp}/argus-sim-xauth.XXXXXX")
    trap 'rm -f -- "$XAUTH_FILE"' EXIT
    cookie=$(xauth nlist "$DISPLAY" 2>/dev/null || true)
    if [[ -z $cookie ]]; then
        echo "ERROR: no Xauthority cookie for DISPLAY=$DISPLAY. Log out and back in, then retry." >&2
        exit 1
    fi
    printf '%s\n' "$cookie" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH_FILE" nmerge -
    chmod 0600 "$XAUTH_FILE"
fi
export XAUTH_FILE=${XAUTH_FILE:-/dev/null}

mkdir -p "$SIM_DIR/.state" "$SIM_DIR/.ws"
if [[ ! -w "$SIM_DIR/.state" ]]; then
    echo "ERROR: $SIM_DIR/.state is not writable by UID $HOST_UID." >&2
    exit 1
fi

if pgrep -f '[Q]GroundControl' >/dev/null 2>&1; then
    echo "QGroundControl is running and should connect on UDP 14550."
else
    echo "Start QGroundControl any time; it connects on UDP 14550."
fi

"${compose[@]}" up --remove-orphans
