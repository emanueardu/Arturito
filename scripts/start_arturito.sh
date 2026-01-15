#!/usr/bin/env bash
# Launch Arturito bringup with Piper TTS and face greeter.
set -eo pipefail

BASE_DIR="/home/robot/ros2_ws"
LOG_DIR="$BASE_DIR/.ros"
ENV_FILE="${HOME}/.config/robertito.env"
LOCK_FILE="/tmp/robot_auto.launch.lock"
WEB_BRIDGE_PATTERN="[r]obot_web_bridge web_bridge.launch.py"

# Load external environment configuration if present so service and manual runs stay aligned.
if [[ -f "$ENV_FILE" ]]; then
    # shellcheck disable=SC1090
    set -a
    source "$ENV_FILE"
    set +a
fi

stop_web_bridge() {
    local pids=()
    while read -r pid; do
        [[ -n "${pid:-}" ]] && pids+=("$pid")
    done < <(pgrep -u "$USER" -f "$WEB_BRIDGE_PATTERN" 2>/dev/null || true)

    if (( ${#pids[@]} == 0 )); then
        return 1
    fi

    echo "Deteniendo bridge web activo (PIDs: ${pids[*]})."
    for pid in "${pids[@]}"; do
        kill -INT "$pid" 2>/dev/null || true
    done

    local timeout=5
    local end_time=$((SECONDS + timeout))
    while (( SECONDS < end_time )); do
        local alive=0
        for pid in "${pids[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                alive=1
                break
            fi
        done
        (( alive == 0 )) && return 0
        sleep 1
    done

    echo "Bridge web no terminó con SIGINT; enviando SIGTERM."
    for pid in "${pids[@]}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done
    sleep 2
    return 0
}

if pgrep -u "$USER" -f "[r]obot_auto.launch.py" > /dev/null; then
    echo "El stack completo ya está en ejecución; no se lanzará nuevamente."
    exit 0
fi

exec 200>"$LOCK_FILE"
if ! flock -n 200; then
    if pgrep -u "$USER" -f "$WEB_BRIDGE_PATTERN" > /dev/null; then
        echo "Detectado bridge web activo sin stack completo. Se detendrá para iniciar el stack."
        stop_web_bridge || true
        if ! flock -n 200; then
            echo "No se pudo obtener el lock luego de detener el bridge web."
            exit 1
        fi
    else
        echo "El stack ya está en ejecución (lock activo)."
        exit 0
    fi
fi

mkdir -p "$LOG_DIR/log"

export COLCON_TRACE=${COLCON_TRACE:-0}
source "$BASE_DIR/install/setup.bash"

export ROS_HOME="$LOG_DIR"
# Force the desired middleware implementation regardless of inherited environment.
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export PATH="$HOME/.local/bin:$PATH"
export PYTHONPATH="$HOME/.local/lib/python3.12/site-packages:${PYTHONPATH:-}"

STARTUP_MESSAGE="${STARTUP_MESSAGE:-Hola, soy Robertito}"
WAKE_WORD="${WAKE_WORD:-robertito}"
MICROPHONE_DEVICE_INDEX="${MICROPHONE_DEVICE_INDEX:-}"
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

exec ros2 launch robot_web_bridge robot_auto.launch.py "${launch_args[@]}"
