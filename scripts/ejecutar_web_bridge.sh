#!/usr/bin/env bash
set -euo pipefail

# Script para lanzar rosbridge, video y audio sin mover los nodos existentes.
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCK_FILE="/tmp/robot_auto.launch.lock"
SINGLETON_HELPER="${WS_DIR}/scripts/start_singleton.sh"

if [ ! -f "${WS_DIR}/install/setup.bash" ]; then
  echo "El workspace no está compilado. Ejecutá 'colcon build' primero." >&2
  exit 1
fi

set +u  # algunos setup.bash acceden a variables no definidas
source /opt/ros/jazzy/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u

ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
VIDEO_PORT="${VIDEO_PORT:-8080}"
AUDIO_PORT="${AUDIO_PORT:-8081}"
VIDEO_TOPIC="${VIDEO_TOPIC:-}"
AUDIO_TOPIC="${AUDIO_TOPIC:-}"
AUDIO_DEVICE="${AUDIO_DEVICE:-default}"
ENABLE_AUDIO_SERVER="${ENABLE_AUDIO_SERVER:-true}"
ENABLE_ALSA_FALLBACK="${ENABLE_ALSA_FALLBACK:-false}"
ENABLE_CONTROL_BRIDGE="${ENABLE_CONTROL_BRIDGE:-true}"
TILT_INPUT_TOPIC="${TILT_INPUT_TOPIC:-/robot_web/tilt_deg}"
TILT_MIN_DEG="${TILT_MIN_DEG:--20.0}"
TILT_MAX_DEG="${TILT_MAX_DEG:-45.0}"
VACUUM_INPUT_TOPIC="${VACUUM_INPUT_TOPIC:-/robot_web/vacuum_enable}"
BRUSH_INPUT_TOPIC="${BRUSH_INPUT_TOPIC:-/robot_web/brush_enable}"

launch_args=(
  "rosbridge_port:=${ROSBRIDGE_PORT}"
  "video_port:=${VIDEO_PORT}"
  "audio_port:=${AUDIO_PORT}"
  "audio_device:=${AUDIO_DEVICE}"
  "enable_audio_server:=${ENABLE_AUDIO_SERVER}"
  "enable_alsa_fallback:=${ENABLE_ALSA_FALLBACK}"
  "enable_control_bridge:=${ENABLE_CONTROL_BRIDGE}"
  "tilt_input_topic:=${TILT_INPUT_TOPIC}"
  "tilt_min_deg:=${TILT_MIN_DEG}"
  "tilt_max_deg:=${TILT_MAX_DEG}"
  "vacuum_input_topic:=${VACUUM_INPUT_TOPIC}"
  "brush_input_topic:=${BRUSH_INPUT_TOPIC}"
)

if [[ -n "${VIDEO_TOPIC}" ]]; then
  launch_args+=("video_topic:=${VIDEO_TOPIC}")
fi

if [[ -n "${AUDIO_TOPIC}" ]]; then
  launch_args+=("audio_topic:=${AUDIO_TOPIC}")
fi

if [[ ! -x "${SINGLETON_HELPER}" ]]; then
  echo "No se encuentra ${SINGLETON_HELPER}. Verificá el workspace." >&2
  exit 1
fi

exec "${SINGLETON_HELPER}" "${LOCK_FILE}" \
  ros2 launch robot_web_bridge web_bridge.launch.py "${launch_args[@]}" "$@"
