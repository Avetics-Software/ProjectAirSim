#!/usr/bin/env bash
# Copyright (C) 2025 IAMAI CONSULTING CORP
# MIT License.
#
# Starts Project AirSim on the Cesium (GISMap) level, a fresh PX4 SITL with the
# MAVLink gimbal enabled, the MediaMTX RTSP server and px4_gimbal_cesium.py.
# Ctrl+C, or closing the simulator window, stops everything.
#
#   ./run_cesium_sim.sh                                   # Seletar runway demo
#   ./run_cesium_sim.sh --qgc --spotter                   # fly from QGC, with a spotter
#   ./run_cesium_sim.sh --qgc --drones 3                  # a swarm of 3 PX4 drones
#   ./run_cesium_sim.sh --qgc --config x8.yml             # drones with `model: x8_arduplane`
#   ./run_cesium_sim.sh --px4-params my.params --pi-ip 192.168.199.73 --qgc
#   ./run_cesium_sim.sh --qgc --config my_config.yml      # another config file
#
# Launcher options (everything else goes to px4_gimbal_cesium.py, see --help):
#   --px4-params FILE      PX4 parameters applied at boot. Accepts "NAME VALUE"
#                          lines (as in ArduPilot .parm files) or a QGC .params
#                          export. Repeatable; later files win, and all win over
#                          the aircraft model's own (px4_params.parm for quad_px4)
#
# Each drone's aircraft (airframe + autopilot) is its `model:` in the config
# file, default quad_px4; aircraft/models.yml lists them (python aircraft.py
# --list). px4_gimbal_cesium.py's --model sets it for drones without one.
#   --pi-ip IP             also send MAVLink to a companion computer at IP, on
#                          UDP ports --pi-ports (default 15540,15541), and
#                          announce the video at rtsp://<this laptop>:8554/gimbal
#   --pi-ports P1[,P2...]  companion-computer MAVLink ports
#   --config FILE          the drones, their companions and the ground station's
#                          settings [config.yml next to this script]; the same
#                          format as the real-drone ground station's fleet file
#                          (gcs/fleet.example.yml). --drones, --pi-ip and
#                          --layer-step override it. (--fleet is the same.)
#   --edit-actors          first open the map editor for the moving actors
#                          (actor_editor.py) in the browser; the sim starts
#                          after you press "Save & start"
#   --edit-mission FILE    the same editor, to plan the drones' mission on the map
#                          (a path, or a name in missions/; a new name starts an
#                          empty mission). Its `actors:` traffic is edited beside it
#
# Environment (defaults in brackets):
#   UE_ROOT           Unreal Engine 5.7 [~/UnrealEngine-5.7]
#   PX4_DIR           PX4-Autopilot checkout [~/Documents/PX4-Autopilot]
#   ARDUPILOT_DIR     ArduPilot checkout, SITL built [~/Documents/ardupilot]
#   PYTHON            Python with the projectairsim client [conda env projectairsim]
#   MEDIAMTX          MediaMTX RTSP server binary [~/opt/mediamtx/mediamtx]
#   CESIUM_ION_TOKEN  Cesium ion token [read from ~/.cesium_ion_token]
#   SIM_MAP           Unreal level [/Game/GISMap]; use /Game/BlocksMap with --scene
#   SIM_PROJECT       Unreal project to run [unreal/Blocks/Blocks.uproject in this
#                     checkout]; from a git worktree, which has no Unreal build, the
#                     main checkout's
#   LOG_DIR           where the sim and PX4 logs go [/tmp/projectairsim-run]
#   PX4_KEEP_LOGS     PX4 flight logs (.ulg) to keep; older ones are deleted at
#                     each PX4 start [5]

set -euo pipefail

# Everything runs inside main(), which bash parses in full before starting, so
# editing this file while a simulation is running can't derail the run.
main() {

    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

    UE_ROOT="${UE_ROOT:-$HOME/UnrealEngine-5.7}"
    PX4_DIR="${PX4_DIR:-$HOME/Documents/PX4-Autopilot}"
    ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/Documents/ardupilot}"
    PYTHON="${PYTHON:-$HOME/miniconda3/envs/projectairsim/bin/python}"
    MEDIAMTX="${MEDIAMTX:-$HOME/opt/mediamtx/mediamtx}"
    SIM_MAP="${SIM_MAP:-/Game/GISMap}"
    LOG_DIR="${LOG_DIR:-/tmp/projectairsim-run}"

    # --- Launcher options --------------------------------------------------------
    PX4_PARAM_FILES=()
    PI_IP=""
    PI_PORTS="15540,15541"
    CONFIG="$SCRIPT_DIR/config.yml"
    EDIT_ACTORS=0
    EDIT_MISSION=""
    SCRIPT_ARGS=()
    while [ $# -gt 0 ]; do
        case "$1" in
            --px4-params) PX4_PARAM_FILES+=("$(realpath "$2")"); shift 2 ;;
            --pi-ip) PI_IP="$2"; shift 2 ;;
            --pi-ports) PI_PORTS="$2"; shift 2 ;;
            --config|--fleet) CONFIG="$(realpath "$2")"; shift 2 ;;
            --edit-actors) EDIT_ACTORS=1; shift ;;
            --edit-mission) EDIT_MISSION="$2"; shift 2 ;;
            *) SCRIPT_ARGS+=("$1"); shift ;;
        esac
    done

    # PX4 reads PX4_PARAM_<NAME>=<value> from the environment at boot, before any
    # module starts. The gimbal needs MNT_MODE_* at boot, so they are set first and
    # a params file can still override them. (ArduPilot ignores these.)
    export PX4_PARAM_MNT_MODE_IN=4 PX4_PARAM_MNT_MODE_OUT=2
    for params_file in ${PX4_PARAM_FILES[@]+"${PX4_PARAM_FILES[@]}"}; do
        [ -f "$params_file" ] || { echo "No such params file: $params_file" >&2; exit 1; }
        count=0
        while read -r name value; do
            export "PX4_PARAM_$name=$value"
            count=$((count + 1))
        done < <(awk '
            { sub(/#.*/, ""); gsub(/,/, " ") }
            NF >= 5 && $1 ~ /^[0-9]+$/ && $2 ~ /^[0-9]+$/ { print $3, $4; next }  # QGC .params
            NF >= 2 { print $1, $2 }                                                # NAME VALUE
        ' "$params_file" | grep -E '^[A-Z][A-Z0-9_]* [-+0-9.eE]+$')
        echo "PX4 parameters: $count from $params_file"
    done

    if [ -z "${CESIUM_ION_TOKEN:-}" ] && [ -f "$HOME/.cesium_ion_token" ]; then
        # shellcheck disable=SC1091
        source "$HOME/.cesium_ion_token"
    fi
    if [ -z "${CESIUM_ION_TOKEN:-}" ] && [ "$SIM_MAP" = "/Game/GISMap" ]; then
        echo "CESIUM_ION_TOKEN is not set and ~/.cesium_ion_token does not exist." >&2
        exit 1
    fi
    export CESIUM_ION_TOKEN

    if [ "$EDIT_ACTORS" = 1 ] || [ -n "$EDIT_MISSION" ]; then
        # Edit the same file and area the sim will use
        EDITOR_ARGS=(--wait --fleet "$CONFIG")
        [ -n "$EDIT_MISSION" ] && EDITOR_ARGS+=(--mission "$EDIT_MISSION")
        i=0
        while [ $i -lt ${#SCRIPT_ARGS[@]} ]; do
            case "${SCRIPT_ARGS[$i]}" in
                --lat) EDITOR_ARGS+=(--lat "${SCRIPT_ARGS[$((i + 1))]}") ;;
                --lon) EDITOR_ARGS+=(--lon "${SCRIPT_ARGS[$((i + 1))]}") ;;
                --model) EDITOR_ARGS+=(--model "${SCRIPT_ARGS[$((i + 1))]}") ;;
                --actors) [ -n "${SCRIPT_ARGS[$((i + 1))]}" ] && EDITOR_ARGS+=(--config "${SCRIPT_ARGS[$((i + 1))]}") ;;
            esac
            i=$((i + 1))
        done
        (cd "$SCRIPT_DIR" && "$PYTHON" actor_editor.py "${EDITOR_ARGS[@]}")
    fi

    # The drones and their companions: the config file, with --drones and
    # --pi-ip on top (the same rule px4_gimbal_cesium.py applies)
    DRONES_ARG=""
    i=0
    while [ $i -lt ${#SCRIPT_ARGS[@]} ]; do
        [ "${SCRIPT_ARGS[$i]}" = "--drones" ] && DRONES_ARG="${SCRIPT_ARGS[$((i + 1))]}"
        i=$((i + 1))
    done
    SCRIPT_ARGS=(--config "$CONFIG" ${SCRIPT_ARGS[@]+"${SCRIPT_ARGS[@]}"})
    read -r DRONES COMPANIONS < <(cd "$SCRIPT_DIR" && "$PYTHON" -c '
import os, sys
from gcs.fleet import load_fleet, sim_drones
path, drones, pi_ip = sys.argv[1], sys.argv[2], sys.argv[3]
fleet = load_fleet(path) if os.path.exists(path) else None
count, companions = sim_drones(fleet, int(drones) if drones else None, pi_ip or None)
print(count, ",".join(companions.get(i, "-") for i in range(count)))
' "$CONFIG" "$DRONES_ARG" "$PI_IP")
    if [ -z "${DRONES:-}" ]; then
        echo "Couldn't read the config file $CONFIG (see the error above)" >&2
        exit 1
    fi
    # Each drone's PX4 connects to the sim on TCP 4560 + its index
    PORTS="8989 8990"
    for ((n = 0; n < DRONES; n++)); do PORTS="$PORTS $((4560 + n))"; done
    for port in $PORTS; do
        if ss -ltn | grep -q ":$port "; then
            echo "Port $port is in use; is another simulator or PX4 still running?" >&2
            exit 1
        fi
    done

    mkdir -p "$LOG_DIR"
    SIM_PID=""
    RTSP_PID=""
    rm -f "$LOG_DIR"/px4*.pid

    cleanup() {
        trap - EXIT INT TERM
        echo "Stopping PX4 and the simulator..."
        # px4_gimbal_cesium.py starts PX4 (and restarts it for 'goto'); stop the
        # last one in case the script didn't get to
        for pid_file in "$LOG_DIR"/px4*.pid; do
            [ -f "$pid_file" ] && kill "$(cat "$pid_file")" 2>/dev/null || true
        done
        [ -n "$RTSP_PID" ] && kill "$RTSP_PID" 2>/dev/null || true
        [ -n "$SIM_PID" ] && kill "$SIM_PID" 2>/dev/null || true
        sleep 3
        [ -n "$SIM_PID" ] && kill -9 "$SIM_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    # --- RTSP server -------------------------------------------------------------
    if ss -ltn | grep -q ":8554 "; then
        echo "Using the RTSP server already on port 8554"
        ss -ltn | grep -q ":8889 " || echo "  (it has no WebRTC on port 8889; enable it there for browser viewing)"
    elif [ -x "$MEDIAMTX" ]; then
        echo "Starting the RTSP server; log: $LOG_DIR/mediamtx.log"
        "$MEDIAMTX" "$SCRIPT_DIR/mediamtx_rtsp.yml" > "$LOG_DIR/mediamtx.log" 2>&1 &
        RTSP_PID=$!
    else
        echo "MediaMTX not found at $MEDIAMTX; running without the RTSP stream." >&2
        SCRIPT_ARGS=(--no-rtsp ${SCRIPT_ARGS[@]+"${SCRIPT_ARGS[@]}"})
    fi

    FIRST_COMPANION="$(tr ',' '\n' <<< "$COMPANIONS" | grep -v '^-$' | grep -m1 . || true)"
    if [ -n "$FIRST_COMPANION" ]; then
        # Announce the streams on the address the companion computers can reach
        LAN_IP="$(ip route get "$FIRST_COMPANION" | awk '{for (i = 1; i < NF; i++) if ($i == "src") print $(i + 1)}')"
        SCRIPT_ARGS=(--rtsp-advertise-url "rtsp://$LAN_IP:8554/gimbal" ${SCRIPT_ARGS[@]+"${SCRIPT_ARGS[@]}"})
        n=1
        for ip in ${COMPANIONS//,/ }; do
            suffix=$([ $n -gt 1 ] && echo $n || true)
            [ "$ip" != "-" ] && echo "Drone$n companion computer $ip: MAVLink on UDP $PI_PORTS, video at rtsp://$LAN_IP:8554/gimbal$suffix"
            n=$((n + 1))
        done
    fi

    echo "GimbalCam video: rtsp://127.0.0.1:8554/gimbal (QGC, Pi) or http://127.0.0.1:8889/gimbal (browser, WebRTC)"
    [ "$DRONES" -gt 1 ] && echo "  drone N of the swarm: .../gimbalN (e.g. http://127.0.0.1:8889/gimbal2)"

    # --- Simulator ---------------------------------------------------------------
    echo "Starting the simulator ($SIM_MAP); log: $LOG_DIR/sim.log"
    "$UE_ROOT/Engine/Binaries/Linux/UnrealEditor" \
        "${SIM_PROJECT:-$REPO_ROOT/unreal/Blocks/Blocks.uproject}" "$SIM_MAP" \
        -game -windowed -ResX=1280 -ResY=720 -log > "$LOG_DIR/sim.log" 2>&1 &
    SIM_PID=$!

    until ss -ltn | grep -q ":8990 "; do
        if ! kill -0 "$SIM_PID" 2>/dev/null; then
            echo "The simulator exited during startup; see $LOG_DIR/sim.log" >&2
            exit 1
        fi
        sleep 1
    done

    # --- Autopilots ---------------------------------------------------------------
    # px4_gimbal_cesium.py starts each drone's PX4 or ArduPilot SITL itself after
    # loading the scene, and restarts them when 'goto' moves to a new place
    export PX4_DIR ARDUPILOT_DIR PI_IP PI_PORTS LOG_DIR PX4_KEEP_LOGS="${PX4_KEEP_LOGS:-5}"

    # --- Mission script ----------------------------------------------------------
    cd "$SCRIPT_DIR"
    "$PYTHON" -u px4_gimbal_cesium.py ${SCRIPT_ARGS[@]+"${SCRIPT_ARGS[@]}"}
}

main "$@"
exit
