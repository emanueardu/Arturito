#!/usr/bin/env bash
# Stop all Robertito-related ROS 2 processes that may be running.
set -euo pipefail

TARGET_PATTERNS=(
    "ros2 launch robertito robertito_launch.py"
    "ros2 launch robot_web_bridge robot_auto.launch.py"
    "ros2 launch robot_web_bridge web_bridge.launch.py"
    "/home/robot/ros2_ws/install/robertito/lib/robertito/"
    "/home/robot/ros2_ws/install/robot_web_bridge/lib/robot_web_bridge/"
    "ros2cli.daemon.daemonize"
)

declare -A PIDS=()

# Collect matching process IDs keyed by PID to avoid duplicates.
for pattern in "${TARGET_PATTERNS[@]}"; do
    while read -r pid; do
        [[ -z "${pid:-}" ]] && continue
        PIDS["$pid"]="$pattern"
    done < <(pgrep -u "$USER" -f "$pattern" 2>/dev/null || true)
done

if [[ ${#PIDS[@]} -eq 0 ]]; then
    echo "No se encontraron procesos de Robertito en ejecución."
    exit 0
fi

echo "Procesos detectados:"
for pid in "${!PIDS[@]}"; do
    cmdline="$(ps -p "$pid" -o cmd= 2>/dev/null || echo "Comando no disponible")"
    printf "  %-6s %s\n" "$pid" "$cmdline"
done

alive_pids=("${!PIDS[@]}")

signal_and_wait() {
    local signal="$1"
    local timeout="$2"
    local remaining=()

    for pid in "${alive_pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "-${signal}" "$pid" 2>/dev/null || true
        fi
    done

    end_time=$((SECONDS + timeout))
    while (( SECONDS < end_time )); do
        remaining=()
        for pid in "${alive_pids[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                remaining+=("$pid")
            fi
        done

        alive_pids=("${remaining[@]}")
        (( ${#alive_pids[@]} == 0 )) && return 0
        sleep 1
    done

    return 1
}

echo "Enviando SIGINT..."
signal_and_wait INT 5 || true

if (( ${#alive_pids[@]} > 0 )); then
    echo "Persisten ${#alive_pids[@]} procesos tras SIGINT. Enviando SIGTERM..."
    signal_and_wait TERM 5 || true
fi

if (( ${#alive_pids[@]} > 0 )); then
    echo "Persisten ${#alive_pids[@]} procesos tras SIGTERM. Fuerzando con SIGKILL..."
    for pid in "${alive_pids[@]}"; do
        kill -KILL "$pid" 2>/dev/null || true
    done
fi

echo "Procesos detenidos."
