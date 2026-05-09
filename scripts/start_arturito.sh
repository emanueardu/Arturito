#!/usr/bin/env bash
# Launch Arturito bringup with Piper TTS and face greeter.

# NOTA: NO usar "set -u" (nounset) en este launcher.
# Los setup.bash de ROS 2 (y del overlay colcon) usan variables opcionales;
# con nounset el servicio se cae (ej: AMENT_TRACE_SETUP_FILES unbound variable).
set -eo pipefail

BASE_DIR="/home/robot/ros2_ws"
LOG_DIR="$BASE_DIR/.ros"
ENV_FILE="${HOME}/.config/robertito.env"
LOCK_FILE="/tmp/robot_auto.launch.lock"
LOCK_FD=200

CLEANUP_PATTERNS=(
    "[r]os2 launch robot_web_bridge robot_auto.launch.py"
    "[r]osbridge"
    "[r]obot_web_bridge"
    "[r]obertito"
    "[a]rturito_uart_bridge"
)

# Si systemd ya usa EnvironmentFile=..., NO hace falta sourcear acá.
# Si querés habilitarlo para ejecución manual, descomentá TODO el bloque.
# if [[ -f "$ENV_FILE" ]]; then
#     set -a
#     source "$ENV_FILE"
#     set +a
# fi

cleanup_stack_processes() {
    local pattern pid
    for pattern in "${CLEANUP_PATTERNS[@]}"; do
        local pids=()
        while read -r pid; do
            [[ -n "${pid:-}" ]] && pids+=("$pid")
        done < <(pgrep -u "$USER" -f "$pattern" 2>/dev/null || true)

        if (( ${#pids[@]} == 0 )); then
            continue
        fi

        echo "Terminando procesos huérfanos para patrón '${pattern}' (PIDs: ${pids[*]})."
        for pid in "${pids[@]}"; do
            kill -TERM "$pid" 2>/dev/null || true
        done

        sleep 1
        for pid in "${pids[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                echo "PID $pid no respondió; enviando SIGKILL."
                kill -KILL "$pid" 2>/dev/null || true
            fi
        done
    done
}

write_lock_file() {
    local timestamp
    timestamp="$(date -u +%s)"
    printf '%s\n%s\n' "$$" "$timestamp" >&$LOCK_FD
}

acquire_lock() {
    exec {LOCK_FD}>"$LOCK_FILE"
    if flock -n "$LOCK_FD"; then
        write_lock_file
        return 0
    fi

    local owner_pid owner_ts
    owner_pid=""
    owner_ts=""
    if [[ -s "$LOCK_FILE" ]]; then
        owner_pid="$(sed -n '1p' "$LOCK_FILE" 2>/dev/null || true)"
        owner_ts="$(sed -n '2p' "$LOCK_FILE" 2>/dev/null || true)"
    fi

    if [[ -n "$owner_pid" ]] && kill -0 "$owner_pid" 2>/dev/null; then
        echo "El stack ya está en ejecución (PID $owner_pid). Abortando."
        exit 0
    fi

    echo "Lock huérfano detectado para PID ${owner_pid:-desconocido}; limpiando e intentando de nuevo."
    flock -u "$LOCK_FD" 2>/dev/null || true
    exec {LOCK_FD}>&- 2>/dev/null || true
    rm -f "$LOCK_FILE"

    exec {LOCK_FD}>"$LOCK_FILE"
    if ! flock -n "$LOCK_FD"; then
        echo "No se pudo adquirir el lock tras limpieza."
        exit 1
    fi

    write_lock_file
}

log_module_origin() {
    local module="$1"
    local label="${2:-$module}"
    local origin
    if origin="$(python3 -c "import importlib; m=importlib.import_module('${module}'); print(m.__file__)" 2>/dev/null)"; then
        echo "RUNNING_FROM ${label}: ${origin}"
    else
        echo "RUNNING_FROM ${label}: <import failed>" >&2
    fi
}

acquire_lock
cleanup_stack_processes

mkdir -p "$LOG_DIR/log"
export COLCON_TRACE="${COLCON_TRACE:-0}"

source /opt/ros/jazzy/setup.bash
source "$BASE_DIR/install/setup.bash"

export ROS_HOME="$LOG_DIR"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export PATH="$HOME/.local/bin:$PATH"
export PYTHONPATH="$HOME/.local/lib/python3.12/site-packages:${PYTHONPATH:-}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/1000}"
export PULSE_SERVER="${PULSE_SERVER:-unix:${XDG_RUNTIME_DIR}/pulse/native}"

log_module_origin "robot_web_bridge.control_bridge" "robot_web_bridge.control_bridge"
log_module_origin "robertito.clean_quick_node" "robertito.clean_quick_node"

STARTUP_MESSAGE="${STARTUP_MESSAGE:-Hola, soy Robertito}"
WAKE_WORD="${WAKE_WORD:-robertito}"
MICROPHONE_DEVICE_INDEX="${MICROPHONE_DEVICE_INDEX:-0}"

DEFAULT_UART_PARAMS="$BASE_DIR/install/robertito/share/robertito/config/arturito_uart.yaml"
if [[ ! -f "$DEFAULT_UART_PARAMS" ]]; then
    DEFAULT_UART_PARAMS="$BASE_DIR/src/robertito/config/arturito_uart.yaml"
fi
UART_PARAMS="${UART_PARAMS:-$DEFAULT_UART_PARAMS}"

ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
VIDEO_PORT="${VIDEO_PORT:-8080}"
AUDIO_PORT="${AUDIO_PORT:-8081}"
AUDIO_TOPIC="${AUDIO_TOPIC:-}"
AUDIO_DEVICE="${AUDIO_DEVICE:-default}"
ENABLE_AUDIO="${ENABLE_AUDIO_SERVER:-false}"
ENABLE_ALSA_FALLBACK="${ENABLE_ALSA_FALLBACK:-false}"
UART_START_ENABLED="${UART_START_ENABLED:-true}"
ENABLE_CONTROL_BRIDGE="${ENABLE_CONTROL_BRIDGE:-true}"
TILT_INPUT_TOPIC="${TILT_INPUT_TOPIC:-/robot_web/tilt_deg}"
TILT_MIN_DEG="${TILT_MIN_DEG:--20.0}"
TILT_MAX_DEG="${TILT_MAX_DEG:-45.0}"
VACUUM_INPUT_TOPIC="${VACUUM_INPUT_TOPIC:-/robot_web/vacuum_enable}"
BRUSH_INPUT_TOPIC="${BRUSH_INPUT_TOPIC:-/robot_web/brush_enable}"

launch_args=(
    "startup_message:=${STARTUP_MESSAGE}"
    "wake_word:=${WAKE_WORD}"
    "uart_params:=${UART_PARAMS}"
    "rosbridge_port:=${ROSBRIDGE_PORT}"
    "video_port:=${VIDEO_PORT}"
    "audio_port:=${AUDIO_PORT}"
    "audio_device:=${AUDIO_DEVICE}"
    "enable_audio_server:=${ENABLE_AUDIO}"
    "enable_alsa_fallback:=${ENABLE_ALSA_FALLBACK}"
    "enable_control_bridge:=${ENABLE_CONTROL_BRIDGE}"
    "tilt_input_topic:=${TILT_INPUT_TOPIC}"
    "tilt_min_deg:=${TILT_MIN_DEG}"
    "tilt_max_deg:=${TILT_MAX_DEG}"
    "vacuum_input_topic:=${VACUUM_INPUT_TOPIC}"
    "brush_input_topic:=${BRUSH_INPUT_TOPIC}"
)

if [[ -n "${MICROPHONE_DEVICE_INDEX}" ]]; then
    launch_args+=("microphone_device_index:=${MICROPHONE_DEVICE_INDEX}")
fi
if [[ -n "${VIDEO_TOPIC:-}" ]]; then
    launch_args+=("video_topic:=${VIDEO_TOPIC}")
fi
if [[ -n "${AUDIO_TOPIC:-}" ]]; then
    launch_args+=("audio_topic:=${AUDIO_TOPIC}")
fi
if [[ -n "${UART_START_ENABLED:-}" ]]; then
    launch_args+=("uart_start_enabled:=${UART_START_ENABLED}")
fi

# Volumen del sink default al 100% y desmuteado.
# Bluetooth puede conectar más tarde y wireplumber restaura el volumen
# persistido (a veces guardado bajo). Reaplicamos varias veces en background
# durante los primeros 30s para sobreescribir cualquier restore.
# Volumen objetivo. 100% = 0 dB (techo "limpio"); por encima entra
# sobreamplificación digital (saturación posible si el material ya está
# fuerte). Para parlantes BT chicos o ambientes ruidosos, 120-140% suele
# valer la pena. Editá BT_TARGET_VOLUME_PCT en el env si querés cambiar.
BT_TARGET_VOLUME_PCT="${BT_TARGET_VOLUME_PCT:-130}"

# Las funciones de audio usan pipes con `timeout` que pueden devolver 124,
# y `grep` que devuelve 1 cuando no hay match. Bajo `set -eo pipefail`
# eso mata el script entero (exit 124 → systemd reinicia en loop). Por eso
# desactivamos set -e aquí y restauramos antes del exec final.
set +e
set +o pipefail

maximize_volume() {
    local sink="${1:-@DEFAULT_SINK@}"
    timeout 3 pactl set-sink-volume "$sink" "${BT_TARGET_VOLUME_PCT}%" 2>/dev/null
    timeout 3 pactl set-sink-mute   "$sink" 0    2>/dev/null
    # También maxeamos el sink-input de cada app que esté reproduciendo
    # (para que ninguna esté atenuando antes del sink).
    local idxs
    idxs=$(timeout 3 pactl list short sink-inputs 2>/dev/null | awk '{print $1}')
    for idx in $idxs; do
        timeout 2 pactl set-sink-input-volume "$idx" 100% 2>/dev/null
    done
    return 0
}

# MAC del parlante BT principal. Si lo cambiás, editá acá.
BT_SPEAKER_MAC="${BT_SPEAKER_MAC:-41:42:FD:4E:3A:32}"

# Reconecta el speaker BT si está apagado/desemparejado del A2DP.
ensure_bt_speaker_connected() {
    [ -z "$BT_SPEAKER_MAC" ] && return 0
    local info connected sink
    info=$(timeout 3 bluetoothctl info "$BT_SPEAKER_MAC" 2>/dev/null)
    connected=$(echo "$info" | awk '/^[[:space:]]*Connected:/ {print $2; exit}')
    if [ "$connected" != "yes" ]; then
        echo "[bt-watchdog] $BT_SPEAKER_MAC no conectado, reconectando…" >&2
        timeout 8 bluetoothctl connect "$BT_SPEAKER_MAC" >/dev/null 2>&1
        sleep 3
        sink=$(timeout 3 pactl list sinks short 2>/dev/null | awk '/bluez_output/ {print $2; exit}')
        if [ -n "$sink" ]; then
            timeout 3 pactl set-default-sink "$sink" >/dev/null 2>&1
            maximize_volume "$sink"
        fi
    fi
    return 0
}

ensure_bt_speaker_connected
maximize_volume
(
    # Background: cada 3s durante 30s, vuelve a forzar 100% por si el sink
    # cambia (BT que conecta) o wireplumber restaura un valor más bajo.
    for i in 1 2 3 4 5 6 7 8 9 10; do
        sleep 3
        maximize_volume
    done
) &

# Audio + BT watchdog: pipewire-pulse a veces se cuelga con BT (síntoma:
# el orchestrator publica frases pero no se escucha nada). También el
# parlante BT puede caerse solo (radio, batería, distancia). Cada 60s:
#   1. Si pactl no responde 2 veces seguidas → reinicia stack de audio.
#   2. Tras cualquier reinicio o pérdida → reconecta el speaker BT y
#      restaura el sink default + volumen 100%.
audio_watchdog() {
    local pactl_fail_count=0
    while true; do
        sleep 60
        if timeout 3 pactl info >/dev/null 2>&1; then
            pactl_fail_count=0
        else
            pactl_fail_count=$((pactl_fail_count + 1))
            if [ "$pactl_fail_count" -ge 2 ]; then
                echo "[audio-watchdog] pactl no responde, reiniciando stack de audio" >&2
                systemctl --user restart pipewire pipewire-pulse wireplumber 2>/dev/null || true
                sleep 4
                pactl_fail_count=0
            fi
        fi
        # Después de cualquier potencial reinicio, garantizar BT + volumen.
        ensure_bt_speaker_connected
        maximize_volume
    done
}
audio_watchdog &

# Restauramos modo estricto antes de lanzar el stack ROS.
set -eo pipefail

exec ros2 launch robot_web_bridge robot_auto.launch.py "${launch_args[@]}"
