#!/usr/bin/env bash
set -euo pipefail

if ! command -v paplay >/dev/null; then
  echo "paplay no disponible, instala pipewire-utils o pavucontrol." >&2
  exit 1
fi

echo "*** PulseAudio sinks ***"
pactl info

echo "*** Reproduciendo sonido de prueba ***"
paplay /usr/share/sounds/alsa/Front_Center.wav
sleep 0.5

echo "*** Publicando /assistant/say ***"
ros2 topic pub --once /assistant/say std_msgs/msg/String "{data: 'hola'}"
