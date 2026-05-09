#!/usr/bin/env bash
# Instala la solución robusta de audio Bluetooth
set -euo pipefail

timestamp() {
  date --iso-8601=seconds
}

log() {
  printf '%s [install-bluetooth-fix] %s\n' "$(timestamp)" "$*"
}

TARGET_USER="${TARGET_USER:-robot}"
TARGET_UID=$(id -u "$TARGET_USER")
USER_RUNTIME_DIR="/run/user/$TARGET_UID"
USER_BUS="unix:path=$USER_RUNTIME_DIR/bus"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(dirname "$SCRIPT_DIR")"

run_user_systemctl() {
  sudo -u "$TARGET_USER" env XDG_RUNTIME_DIR="$USER_RUNTIME_DIR" \
    DBUS_SESSION_BUS_ADDRESS="$USER_BUS" systemctl --user "$@"
}

wait_for_user_bus() {
  local waited=0
  local max_wait=30
  while [ ! -S "$USER_RUNTIME_DIR/bus" ] && [ "$waited" -lt "$max_wait" ]; do
    log "waiting for user bus ($waited/$max_wait)"
    sleep 1
    waited=$((waited + 1))
  done
  if [ -S "$USER_RUNTIME_DIR/bus" ]; then
    log "user bus available at $USER_RUNTIME_DIR/bus"
    return 0
  fi
  log "user bus still missing after ${max_wait}s"
  return 1
}

echo "=== Installing Bluetooth audio stabilization ==="

# Ensure we are root
if [ "$EUID" -ne 0 ]; then
  echo "ERROR: This installer must run as root (sudo)"
  exit 1
fi

echo
echo "1. Making helper scripts executable..."
chmod +x "$SCRIPT_DIR/ensure_bt_a2dp_sink.sh"
chmod +x "$SCRIPT_DIR/check-audio-sink.sh"
chmod +x "$SCRIPT_DIR/wait-for-user-audio-ready.sh"

echo
echo "2. Installing helper scripts to /usr/local/bin..."
cp -v "$SCRIPT_DIR/ensure_bt_a2dp_sink.sh" /usr/local/bin/
cp -v "$SCRIPT_DIR/check-audio-sink.sh" /usr/local/bin/
cp -v "$SCRIPT_DIR/wait-for-user-audio-ready.sh" /usr/local/bin/

echo
echo "3. Installing systemd units..."
cp -v "$WORKSPACE/systemd/bt-autoconnect.service" /etc/systemd/system/
cp -v "$WORKSPACE/systemd/bt-audio-ensure.service" /etc/systemd/system/
cp -v "$WORKSPACE/systemd/arturito.service" /etc/systemd/system/

echo
echo "4. Keeping $TARGET_USER user services alive (lingering)..."
loginctl enable-linger "$TARGET_USER"

echo
echo "5. Enabling $TARGET_USER user manager at boot..."
systemctl enable --now "user@${TARGET_UID}.service"

echo
echo "6. Ensuring pipewire/wireplumber user units are enabled for $TARGET_USER..."
if wait_for_user_bus; then
  if run_user_systemctl daemon-reload; then
    log "reload user unit cache for $TARGET_USER"
  else
    log "warning: user daemon-reload failed for $TARGET_USER"
  fi
  if run_user_systemctl enable --now pipewire pipewire-pulse wireplumber; then
    log "enabled user audio services for $TARGET_USER"
  else
    log "warning: enabling user audio services failed for $TARGET_USER"
  fi
else
  log "user bus not ready; deferring user service enable until next boot"
fi
echo
echo "7. Reloading systemd daemon..."
systemctl daemon-reload

echo
echo "8. Enabling bt-autoconnect.service..."
systemctl enable bt-autoconnect.service

echo
echo "9. Enabling bt-audio-ensure.service..."
systemctl enable bt-audio-ensure.service

echo
echo "10. Ensuring arturito.service is enabled..."
if systemctl is-enabled arturito.service &>/dev/null; then
  echo "   arturito.service is already enabled"
else
  echo "   Enabling arturito.service..."
  systemctl enable arturito.service
fi

echo
echo "=== Installation complete ==="
echo
echo "Current service status:"
systemctl status bt-audio-ensure.service --no-pager -l || true
echo
systemctl status arturito.service --no-pager -l || true
echo
echo "To test the fix manually:"
echo "  sudo systemctl restart bt-audio-ensure.service"
echo "  sudo systemctl restart arturito.service"
echo
echo "To verify after boot:"
echo "  sudo journalctl -u bt-audio-ensure.service -f"
echo "  sudo journalctl -u arturito.service -f"
