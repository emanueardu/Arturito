#!/usr/bin/env bash
set -euo pipefail

SPEAKER_MAC="${SPEAKER_MAC:-XX:XX:XX:XX:XX:XX}"
CONNECT_ONLY="${CONNECT_ONLY:-0}"
CARD_PROFILE="${CARD_PROFILE:-a2dp-sink}"
CARD_NAME="bluez_card.${SPEAKER_MAC//:/_}"
SINK_NAME="bluez_output.${SPEAKER_MAC//:/_}.1"
MAX_TOTAL_WAIT_SEC="${MAX_TOTAL_WAIT_SEC:-180}"

USER_NAME="$(id -un)"
USER_UID="$(id -u)"
USER_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$USER_UID}"
USER_BUS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=$USER_RUNTIME_DIR/bus}"

export XDG_RUNTIME_DIR="$USER_RUNTIME_DIR"
export PULSE_SERVER="${PULSE_SERVER:-unix:$USER_RUNTIME_DIR/pulse/native}"
export DBUS_SESSION_BUS_ADDRESS="$USER_BUS"

log() {
  printf '%s [bt-audio-ensure] %s\n' "$(date --iso-8601=seconds)" "$*"
}

if ! command -v pactl >/dev/null 2>&1; then
  log "ERROR: pactl not in PATH; install pipewire-pulse"
  exit 1
fi

next_backoff() {
  local current=$1
  if [ "$current" -lt 5 ]; then
    echo 5
  elif [ "$current" -lt 10 ]; then
    echo 10
  else
    echo 30
  fi
}

BT_CONNECT_LOG="${BT_CONNECT_LOG:-/tmp/bt-connect.log}"

wait_for_system_service() {
  local service="$1"
  local max_wait="${2:-30}"
  local waited=0
  until systemctl is-active --quiet "$service"; do
    log "$service inactive (waited ${waited}s)"
    sleep 1
    waited=$((waited + 1))
    if [ "$waited" -ge "$max_wait" ]; then
      log "timed out waiting for $service"
      return 1
    fi
  done
  log "$service is active"
  return 0
}

wait_for_user_bus() {
  local waited=0
  while [ ! -S "$USER_RUNTIME_DIR/bus" ]; do
    log "user session bus missing (waited ${waited}s)"
    sleep 1
    waited=$((waited + 1))
    if [ "$waited" -ge 30 ]; then
      log "user session bus did not appear"
      return 1
    fi
  done
  log "user session bus ready"
  return 0
}

wait_for_user_services() {
  local services=(pipewire pipewire-pulse wireplumber)
  for svc in "${services[@]}"; do
    local waited=0
    until systemctl --user is-active --quiet "$svc"; do
      log "user service $svc not active (waited ${waited}s)"
      sleep 1
      waited=$((waited + 1))
      if [ "$waited" -ge 30 ]; then
        log "timed out waiting for user service $svc"
        return 1
      fi
    done
    log "user service $svc is active"
  done
  return 0
}

wait_for_audio_ready() {
  local waited=0
  until pactl info >/dev/null 2>&1; do
    log "pactl not responding yet (waited ${waited}s)"
    sleep 1
    waited=$((waited + 1))
    if [ "$waited" -ge 30 ]; then
      log "pactl still not responding"
      return 1
    fi
  done
  log "pactl responding"
  return 0
}

prepare_bluetooth_controller() {
  log "ensuring Bluetooth controller is powered"
  bluetoothctl power on >/dev/null 2>&1 || true
  log "ensuring Bluetooth agent is online"
  bluetoothctl agent on >/dev/null 2>&1 || true
  bluetoothctl default-agent >/dev/null 2>&1 || true
}

device_connected() {
  bluetoothctl info "$SPEAKER_MAC" 2>/dev/null | grep -q 'Connected: yes'
}

sink_exists() {
  pactl list short sinks 2>/dev/null | awk -v target="$SINK_NAME" '$2 == target {found=1} END {exit(found ? 0 : 1)}'
}

wait_for_connection() {
  local waited=0
  until device_connected; do
    log "waiting for device connection confirmation (waited ${waited}s)"
    sleep 1
    waited=$((waited + 1))
    if [ "$waited" -ge 20 ]; then
      log "connection confirmation timeout"
      return 1
    fi
  done
  log "device reports Connected: yes"
}

wait_for_sink() {
  local waited=0
  until sink_exists; do
    log "waiting for sink ${SINK_NAME} (waited ${waited}s)"
    sleep 1
    waited=$((waited + 1))
    if [ "$waited" -ge 30 ]; then
      log "sink did not appear"
      return 1
    fi
  done
  log "sink ${SINK_NAME} available"
  return 0
}

ensure_trusted() {
  if ! bluetoothctl info "$SPEAKER_MAC" 2>/dev/null | grep -q 'Trusted: yes'; then
    log "trusting speaker $SPEAKER_MAC"
    bluetoothctl trust "$SPEAKER_MAC" >/dev/null 2>&1 || true
  fi
}

log_bt_connect_output() {
  local logfile="${1:-$BT_CONNECT_LOG}"
  if [ -s "$logfile" ]; then
    log "bluetoothctl connect output:"
    while IFS= read -r line; do
      log "  $line"
    done < "$logfile"
  fi
}

connect_device() {
  if device_connected; then
    log "device already connected"
    return 0
  fi

  log "connecting to $SPEAKER_MAC"
  local attempt_log
  attempt_log="$(mktemp /tmp/bt-connect.XXXXXX)" || attempt_log="$BT_CONNECT_LOG"
  if bluetoothctl connect "$SPEAKER_MAC" >"$attempt_log" 2>&1; then
    log "bluetoothctl connect succeeded"
    rm -f "$attempt_log"
    return 0
  fi
  if grep -Eq 'InProgress|AlreadyConnected|Connection successful' "$attempt_log"; then
    log "bluetoothctl connect reported a transient/success state"
    log_bt_connect_output "$attempt_log"
    rm -f "$attempt_log"
    return 0
  fi
  log "bluetoothctl connect failed (see $attempt_log)"
  log_bt_connect_output "$attempt_log"
  rm -f "$attempt_log"
  return 1
}

apply_headset_profile() {
  local profile="${1:-$CARD_PROFILE}"
  local card="${2:-$CARD_NAME}"
  local attempt=0

  if [ -z "$profile" ] || [ -z "$card" ]; then
    return 1
  fi

  while [ "$attempt" -lt 5 ]; do
    if pactl list cards short | awk -v target="$card" '$2 == target {exit 0} END {exit 1}'; then
      if pactl set-card-profile "$card" "$profile" >/dev/null 2>&1; then
        log "card $card profile set to $profile"
        return 0
      fi
      log "failed to set card $card profile to $profile (attempt $((attempt + 1)))"
    else
      log "card $card not yet visible (waited ${attempt}s)"
    fi
    attempt=$((attempt + 1))
    sleep 1
  done
  log "unable to set profile $profile for card $card"
  return 1
}

set_default_sink() {
  local sink="${1:-$SINK_NAME}"
  if pactl set-default-sink "$sink" >/dev/null 2>&1; then
    log "default sink set to $sink"
    return 0
  fi
  log "failed to set default sink to $sink"
  return 1
}

log_sink_state() {
  if command -v pactl >/dev/null 2>&1; then
    log "pactl sinks:"
    pactl list short sinks | while IFS= read -r line; do
      log "  $line"
    done
  else
    log "pactl not available to list sinks"
  fi

  if command -v wpctl >/dev/null 2>&1; then
    log "wpctl status:"
    wpctl status | while IFS= read -r line; do
      log "  $line"
    done
  else
    log "wpctl not available"
  fi
}

if [ "$SPEAKER_MAC" = 'XX:XX:XX:XX:XX:XX' ]; then
  log "WARNING: SPEAKER_MAC is the placeholder value; set SPEAKER_MAC before installing"
fi

log "starting BT audio ensure loop (user ${USER_NAME}, UID ${USER_UID})"

if [ "$CONNECT_ONLY" = '1' ]; then
  log "CONNECT_ONLY mode: skipping PulseAudio/pipewire checks"
  prepare_bluetooth_controller
  ensure_trusted
  backoff_delay=2
  while true; do
    if connect_device; then
      log "CONNECT_ONLY: speaker connected"
      exit 0
    fi
    log "CONNECT_ONLY: connect command failed, retrying in ${backoff_delay}s"
    log_bt_connect_output
    sleep "$backoff_delay"
    backoff_delay=$(next_backoff "$backoff_delay")
  done
fi

backoff_delay=2
attempt=0
start_epoch="$(date +%s)"

while true; do
  now_epoch="$(date +%s)"
  if [ $((now_epoch - start_epoch)) -ge "$MAX_TOTAL_WAIT_SEC" ]; then
    log "timed out after ${MAX_TOTAL_WAIT_SEC}s waiting for Bluetooth audio"
    exit 1
  fi

  attempt=$((attempt + 1))
  log "attempt $attempt: checking prerequisites"

  if ! wait_for_system_service bluetooth 30; then
    log "bluetooth.service not yet active, retrying in ${backoff_delay}s"
    sleep "$backoff_delay"
    backoff_delay=$(next_backoff "$backoff_delay")
    continue
  fi

  if ! wait_for_user_bus; then
    log "user bus missing, retrying in ${backoff_delay}s"
    sleep "$backoff_delay"
    backoff_delay=$(next_backoff "$backoff_delay")
    continue
  fi

  if ! wait_for_user_services; then
    log "user services not ready, retrying in ${backoff_delay}s"
    sleep "$backoff_delay"
    backoff_delay=$(next_backoff "$backoff_delay")
    continue
  fi

  if ! wait_for_audio_ready; then
    log "audio stack not ready, retrying in ${backoff_delay}s"
    sleep "$backoff_delay"
    backoff_delay=$(next_backoff "$backoff_delay")
    continue
  fi

  prepare_bluetooth_controller
  ensure_trusted

  if device_connected && sink_exists; then
    log "device already connected and sink present"
    apply_headset_profile || true
    set_default_sink || true
    log_sink_state
    exit 0
  fi

  if ! connect_device; then
    log "connect command failed, will retry"
    sleep "$backoff_delay"
    backoff_delay=$(next_backoff "$backoff_delay")
    continue
  fi

  if ! wait_for_connection; then
    log "connection never confirmed, retrying"
    bluetoothctl disconnect "$SPEAKER_MAC" >/dev/null 2>&1 || true
    sleep "$backoff_delay"
    backoff_delay=$(next_backoff "$backoff_delay")
    continue
  fi

  if wait_for_sink; then
    log "Bluetooth audio sink ready"
    apply_headset_profile || true
    set_default_sink || true
    log_sink_state
    exit 0
  fi

  log "sink unavailable after connection, forcing disconnect"
  bluetoothctl disconnect "$SPEAKER_MAC" >/dev/null 2>&1 || true
  sleep "$backoff_delay"
  backoff_delay=$(next_backoff "$backoff_delay")
done
