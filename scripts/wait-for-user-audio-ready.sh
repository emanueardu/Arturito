#!/usr/bin/env bash
set -euo pipefail

TARGET_USER="${TARGET_USER:-robot}"
TARGET_UID="${TARGET_UID:-$(id -u "$TARGET_USER")}"
USER_RUNTIME_DIR="/run/user/$TARGET_UID"
USER_BUS="unix:path=$USER_RUNTIME_DIR/bus"
MAX_WAIT_SECONDS=90
SERVICES=(pipewire pipewire-pulse wireplumber)

timestamp() {
  date --iso-8601=seconds
}

log() {
  printf '%s [wait-user-audio-ready] %s\n' "$(timestamp)" "$*"
}

run_user_systemctl() {
  sudo -u "$TARGET_USER" env XDG_RUNTIME_DIR="$USER_RUNTIME_DIR" \
    DBUS_SESSION_BUS_ADDRESS="$USER_BUS" systemctl --user "$@"
}

deadline=$((SECONDS + MAX_WAIT_SECONDS))

log "waiting for user bus and audio services for $TARGET_USER (UID $TARGET_UID)"

while true; do
  if [ -S "$USER_RUNTIME_DIR/bus" ]; then
    log "user bus is ready at $USER_RUNTIME_DIR/bus"
    break
  fi
  if [ "$SECONDS" -ge "$deadline" ]; then
    log "timed out waiting for user bus after ${MAX_WAIT_SECONDS}s"
    exit 1
  fi
  log "user bus not available yet, sleeping 1s"
  sleep 1
done

for svc in "${SERVICES[@]}"; do
  while true; do
    if run_user_systemctl --quiet is-active "$svc"; then
      log "user service $svc is active"
      break
    fi
    if [ "$SECONDS" -ge "$deadline" ]; then
      log "timed out waiting for user service $svc after ${MAX_WAIT_SECONDS}s"
      exit 1
    fi
    log "waiting for user service $svc (sleeping 3s)"
    sleep 3
  done
done

log "user audio services ready"
