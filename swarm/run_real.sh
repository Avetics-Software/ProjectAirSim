#!/usr/bin/env bash
# Copyright (C) 2025 IAMAI CONSULTING CORP
# MIT License.
#
# Starts the swarm ground station for REAL drones, each with its own companion
# computer (a Radxa running X). It reads the same config.yml as the sim: the
# drones (name, sysid, video, companion) and the top-level mavlink link, plus
# the swarm / deconfliction / safety / controls settings.
#
#   ./run_real.sh                         # config.yml next to this script
#   ./run_real.sh --config field_day.yml  # another config
#   ./run_real.sh --check                 # preflight checks only, then exit
#
# Options:
#   --config FILE   the config [config.yml next to this script]
#   --check         run the preflight checks and exit
#   --skip-checks   start without the preflight checks
#   --listen S      seconds the checks listen for the drones' heartbeats [6]
#   --verbose       also print debug messages in the terminal
#
# Environment (defaults in brackets):
#   PYTHON    Python with pymavlink, opencv, numpy, pillow, PyYAML and Tk
#             [the conda env projectairsim if present, else python3]
#   LOG_DIR   where each session's log goes [~/swarm_logs]
#
# Before the first flight, read gcs/README.md "Before flying it for real":
# each PX4's own failsafes (NAV_DLL_ACT, battery, geofence) must be set, the
# ground station's responses only work while the link holds.

set -euo pipefail

main() {
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    CONFIG="$SCRIPT_DIR/config.yml"
    CHECK_ONLY=0
    SKIP_CHECKS=0
    LISTEN=6
    GCS_ARGS=()
    while [ $# -gt 0 ]; do
        case "$1" in
            --config) CONFIG="$(realpath "$2")"; shift 2 ;;
            --check) CHECK_ONLY=1; shift ;;
            --skip-checks) SKIP_CHECKS=1; shift ;;
            --listen) LISTEN="$2"; shift 2 ;;
            --verbose) GCS_ARGS+=(--verbose); shift ;;
            -h|--help) sed -n '5,/^$/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
            *) echo "Unknown option $1 (see --help)" >&2; exit 1 ;;
        esac
    done

    if [ -z "${PYTHON:-}" ]; then
        if [ -x "$HOME/miniconda3/envs/projectairsim/bin/python" ]; then
            PYTHON="$HOME/miniconda3/envs/projectairsim/bin/python"
        else
            PYTHON="$(command -v python3)"
        fi
    fi
    if ! "$PYTHON" -c "import pymavlink, cv2, numpy, PIL, yaml, tkinter" 2>/dev/null; then
        echo "$PYTHON lacks the ground station's packages:" >&2
        echo "  $PYTHON -m pip install -r $SCRIPT_DIR/gcs/requirements.txt  (and python3-tk)" >&2
        exit 1
    fi
    [ -f "$CONFIG" ] || { echo "No config file $CONFIG" >&2; exit 1; }

    cd "$SCRIPT_DIR"
    if [ "$SKIP_CHECKS" = 0 ]; then
        echo "Preflight checks ($CONFIG):"
        if ! "$PYTHON" -m gcs.preflight --config "$CONFIG" --listen "$LISTEN"; then
            exit 1
        fi
    fi
    [ "$CHECK_ONLY" = 1 ] && exit 0

    LOG_DIR="${LOG_DIR:-$HOME/swarm_logs}"
    mkdir -p "$LOG_DIR"
    LOG_FILE="$LOG_DIR/gcs_$(date +%Y%m%d_%H%M%S).log"
    echo "Starting the ground station; log: $LOG_FILE  (Ctrl+C to stop)"
    exec "$PYTHON" -m gcs --fleet "$CONFIG" --log-file "$LOG_FILE" ${GCS_ARGS[@]+"${GCS_ARGS[@]}"}
}

main "$@"
