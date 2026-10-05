#!/bin/bash
# DroneLab launcher for the Steam Deck (SteamOS desktop mode) - runs on the HOST.
#
#   dronelab.sh flight   real drones: discovery + safety gate GUI
#   dronelab.sh demo     simulation demo (Gazebo + gamepad + FPV)
#   dronelab.sh shell    terminal inside the container
#   dronelab.sh build    (re)build the container image
#   dronelab.sh stop     stop the container (all apps and gates)
#   dronelab.sh status   show engine / image / container state
#   dronelab.sh selftest headless simulation test of the safety gate (~4 min, stop the Demo first)
#
# The container keeps running in the background so the next start is instant.
set -u

REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
IMAGE="dronelab-sim-ubuntu2204"
IMAGE_VERSION="2"            # bump together with LABEL dronelab.version in the Dockerfile
NAME="dronelab"
LOG="$REPO/var/logs/launcher.log"
mkdir -p "$REPO/var/logs"
export PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin:$PATH"

log() { echo "$(date +%H:%M:%S) $*" | tee -a "$LOG" >&2; }

notify() {  # visible message even when started from a desktop icon
    log "$1"
    if command -v kdialog >/dev/null; then kdialog --title DroneLab --passivepopup "$1" 8 >/dev/null 2>&1 &
    elif command -v notify-send >/dev/null; then notify-send DroneLab "$1"; fi
}

fail() {
    log "ERROR: $1"
    if command -v kdialog >/dev/null; then kdialog --title DroneLab --error "$1"; fi
    exit 1
}

# ---------------------------------------------------------------- engine
if command -v podman >/dev/null; then ENGINE=podman
elif command -v docker >/dev/null; then ENGINE=docker
else fail "Neither podman nor docker found. SteamOS 3.5+ ships podman; otherwise install it."; fi

image_ok() {
    local v
    v=$($ENGINE image inspect "$IMAGE" --format '{{ index .Config.Labels "dronelab.version" }}' 2>/dev/null) || return 1
    [ "$v" = "$IMAGE_VERSION" ]
}

running() { [ "$($ENGINE inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null)" = "true" ]; }

in_terminal() {  # run a command visibly (image builds take a while)
    if [ -t 1 ]; then "$@"; return $?; fi
    if command -v konsole >/dev/null; then konsole --hold -e "$@"; return $?; fi
    "$@"
}

do_build() {
    log "building image $IMAGE with $ENGINE (first time: 30-60 min, later only changed layers)"
    [ "$ENGINE" = podman ] && podman system migrate >/dev/null 2>&1
    $ENGINE build -t "$IMAGE" "$REPO" 2>&1 | tee -a "$LOG"
    return "${PIPESTATUS[0]}"
}

start_container() {
    running && return 0
    if ! image_ok; then
        notify "Building the DroneLab container image (only needed once, keep the window open)…"
        in_terminal "$REPO/scripts/dronelab.sh" build || true
        image_ok || fail "Image build failed - see $LOG"
    fi
    $ENGINE rm -f "$NAME" >/dev/null 2>&1
    local args=(run -d --name "$NAME" --net=host --ipc=host
        -e "DISPLAY=${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 -e "DRONELAB_VAR=/opt/dronelab/var"
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw
        -v "$REPO:/opt/dronelab:rw")
    if [ -n "${XAUTHORITY:-}" ] && [ -f "$XAUTHORITY" ]; then
        args+=(-v "$XAUTHORITY:/tmp/.Xauthority:ro" -e XAUTHORITY=/tmp/.Xauthority)
    fi
    [ -e /dev/dri ] && args+=(--device /dev/dri)                       # GPU for Gazebo
    [ -d /dev/input ] && args+=(-v /dev/input:/dev/input:ro)           # gamepads (incl. Steam Input's virtual pad)
    [ -d /run/udev ] && args+=(-v /run/udev:/run/udev:ro)
    [ -d "$HOME/LRS-FEI" ] && args+=(-v "$HOME/LRS-FEI:/home/deck/LRS-FEI:rw")
    [ -d "$HOME/Projects" ] && args+=(-v "$HOME/Projects:/home/deck/Projects:rw")
    if [ "$ENGINE" = podman ]; then
        args+=(--userns=keep-id --group-add keep-groups --security-opt label=disable)
    else
        args+=(--user "$(id -u):$(id -g)" --group-add video --group-add input)
    fi
    log "starting container: $ENGINE ${args[*]} $IMAGE sleep infinity"
    if ! $ENGINE "${args[@]}" "$IMAGE" sleep infinity >>"$LOG" 2>&1; then
        # older podman/runc setups do not support keep-groups: retry without it
        $ENGINE rm -f "$NAME" >/dev/null 2>&1
        local retry=() a
        for a in "${args[@]}"; do
            if [ "$a" = "keep-groups" ]; then unset 'retry[${#retry[@]}-1]'; continue; fi
            retry+=("$a")
        done
        log "retrying without keep-groups"
        $ENGINE "${retry[@]}" "$IMAGE" sleep infinity >>"$LOG" 2>&1 || fail "Could not start the container - see $LOG"
    fi
}

allow_x() {
    command -v xhost >/dev/null || return 0
    xhost "+SI:localuser:$(id -un)" >/dev/null 2>&1 || xhost +local: >/dev/null 2>&1
}

run_app() {  # $1 = python module, rest = args
    start_container
    allow_x
    log "launching $1 ${*:2}"
    exec $ENGINE exec -e "DISPLAY=${DISPLAY:-:0}" "$NAME" /opt/dronelab/scripts/in-container.sh "$@"
}

case "${1:-flight}" in
    flight) shift; run_app dronelab.gui.flight_app "$@" ;;
    demo)   shift; run_app dronelab.gui.demo_app "$@" ;;
    shell)
        start_container; allow_x
        if [ -t 1 ]; then exec $ENGINE exec -it "$NAME" /opt/dronelab/scripts/in-container.sh --shell
        else exec konsole -e $ENGINE exec -it "$NAME" /opt/dronelab/scripts/in-container.sh --shell; fi ;;
    selftest)
        start_container
        exec $ENGINE exec "$NAME" /opt/dronelab/scripts/in-container.sh --exec python3 tools/e2e_sim_test.py ;;
    build)  do_build ;;
    stop)   $ENGINE rm -f "$NAME" && log "container stopped" ;;
    status)
        echo "engine:    $ENGINE"
        echo "image:     $IMAGE ($(image_ok && echo "version $IMAGE_VERSION OK" || echo "missing/outdated"))"
        echo "container: $NAME ($(running && echo running || echo stopped))"
        echo "repo:      $REPO" ;;
    *) sed -n '2,13p' "$0"; exit 2 ;;
esac
