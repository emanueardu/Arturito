#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash >/dev/null 2>&1 || true
if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${WS_DIR}/install/setup.bash"
fi

echo "=== Diagnostico de duplicidad ROS2 ==="
date
echo "host: $(hostname)"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-<no-set>}"
echo

echo "--- ros2 node list ---"
if command -v ros2 >/dev/null 2>&1; then
  ros2 node list || true
  echo
  echo "--- nodos duplicados (si aplica) ---"
  ros2 node list | sort | uniq -d || true
else
  echo "ros2 no disponible en PATH."
fi
echo

echo "--- ros2 topic list ---"
if command -v ros2 >/dev/null 2>&1; then
  ros2 topic list || true
else
  echo "ros2 no disponible en PATH."
fi
echo

echo "--- procesos ROS2 (ps -ef) ---"
if command -v rg >/dev/null 2>&1; then
  ps -ef | rg "ros2|launch|robertito|robot_web_bridge|rosbridge|rosapi|v4l2_camera|esp32" || true
else
  ps -ef | grep -E "ros2|launch|robertito|robot_web_bridge|rosbridge|rosapi|v4l2_camera|esp32" || true
fi
echo

echo "--- mapeo ejecutable -> PID ---"
ps -eo pid,comm,args | {
  if command -v rg >/dev/null 2>&1; then
    rg "ros2 launch|ros2 run|robot_web_bridge/lib|robertito/lib|rosbridge_websocket|rosapi_node"
  else
    grep -E "ros2 launch|ros2 run|robot_web_bridge/lib|robertito/lib|rosbridge_websocket|rosapi_node"
  fi
} || true
echo

if [[ "${DIAGNOSE_LSOF:-false}" == "true" ]]; then
  echo "--- lsof (DIAGNOSE_LSOF=true) ---"
  for pid in $(pgrep -f "ros2 launch|ros2 run|robot_web_bridge/lib|robertito/lib|rosbridge_websocket|rosapi_node" || true); do
    echo "PID ${pid}"
    lsof -p "${pid}" || true
    echo
  done
fi
