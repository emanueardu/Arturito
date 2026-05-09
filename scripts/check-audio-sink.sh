#!/usr/bin/env bash
# Health-check: verifica que el sink Bluetooth esté disponible antes de iniciar el robot
set -euo pipefail

SINK_PATTERN="bluez_output.41_42_FD_4E_3A_32"
TIMEOUT="${1:-60}"  # Default: 60s timeout

log() {
  printf '%s [audio-check] %s\n' "$(date --iso-8601=seconds)" "$*" >&2
}

sink_exists() {
  pactl list sinks short 2>/dev/null | grep -q "$SINK_PATTERN"
}

log "Checking for Bluetooth audio sink..."

for i in $(seq 1 "$TIMEOUT"); do
  if sink_exists; then
    log "SUCCESS: Bluetooth sink '$SINK_PATTERN' is available"
    # Show available sinks for debugging
    log "Available sinks:"
    pactl list sinks short 2>/dev/null | while read -r line; do
      log "  $line"
    done
    exit 0
  fi

  if [ $((i % 10)) -eq 0 ]; then
    log "Still waiting for Bluetooth sink... (${i}/${TIMEOUT}s)"
  fi
  sleep 1
done

log "ERROR: Bluetooth sink not available after ${TIMEOUT}s"
log "Available sinks:"
pactl list sinks short 2>/dev/null | while read -r line; do
  log "  $line"
done || log "  (pactl failed)"
exit 1
