#!/usr/bin/env bash
set -euo pipefail

LOG_FILE=/var/log/audio_bt_bootstate.log
SPEAKER_MAC="${SPEAKER_MAC:-XX:XX:XX:XX:XX:XX}"
TARGET_USER="${TARGET_USER:-robot}"
TARGET_UID=$(id -u "$TARGET_USER")
USER_BUS="/run/user/$TARGET_UID/bus"

log_separator() {
  printf '\n%s\n' '========================================' | tee -a "$LOG_FILE"
}

log_section() {
  local title="$1"
  echo "--- $title ---" | tee -a "$LOG_FILE"
}

run_cmd() {
  local title="$1"
  shift
  log_section "$title"
  if ! { "$@" 2>&1 | tee -a "$LOG_FILE"; }; then
    local exit_code="${PIPESTATUS[0]:-${?}}"
    echo "(command exited ${exit_code}; continuing)" | tee -a "$LOG_FILE"
  fi
}

run_cmd_with_tail() {
  local title="$1"
  shift
  log_section "$title"
  if ! { "$@" 2>&1 | tail -n 200 | tee -a "$LOG_FILE"; }; then
    local exit_code="${PIPESTATUS[0]:-${?}}"
    echo "(command pipeline exited ${exit_code}; continuing)" | tee -a "$LOG_FILE"
  fi
}

log_separator
printf 'Audio BT boot state capture: %s\n' "$(date --iso-8601=seconds)" | tee -a "$LOG_FILE"
log_separator

run_cmd "uname -a" uname -a
run_cmd "/etc/os-release" cat /etc/os-release
run_cmd "systemctl --version" systemctl --version
run_cmd "bluetooth service status" systemctl status bluetooth --no-pager -l
run_cmd "system-level PipeWire services" systemctl status pipewire pipewire-pulse wireplumber --no-pager -l

log_section "user-level PipeWire services ($TARGET_USER)"
if ! sudo -u "$TARGET_USER" env XDG_RUNTIME_DIR="/run/user/$TARGET_UID" systemctl --user status pipewire pipewire-pulse wireplumber --no-pager -l 2>&1 | tee -a "$LOG_FILE"; then
  echo "(user-level status command failed; continuing)" | tee -a "$LOG_FILE"
fi

run_cmd "loginctl show-user $TARGET_USER" loginctl show-user "$TARGET_USER"
run_cmd "Inspect user bus" ls -la "$USER_BUS"
run_cmd "Relevant processes" ps -ef | egrep 'pipewire|wireplumber|bluetoothd' | grep -v grep
run_cmd "bluetoothctl show" bluetoothctl show
run_cmd "bluetoothctl info $SPEAKER_MAC" bluetoothctl info "$SPEAKER_MAC"
run_cmd "pactl info" pactl info
run_cmd "pactl sink list" pactl list short sinks
run_cmd "wpctl status" wpctl status
run_cmd_with_tail "journalctl -b -u bluetooth" journalctl -b -u bluetooth --no-pager -l
run_cmd_with_tail "journalctl -b --user -u pipewire" journalctl -b --user -u pipewire --no-pager -l

echo "--- End of capture ---" | tee -a "$LOG_FILE"
