#!/usr/bin/env bash
set -euo pipefail

# Lanza la app Vite en modo desarrollo accesible desde cualquier PC de la LAN.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APP_DIR="${ROOT_DIR}/web/robot-ui"

export NVM_DIR="${HOME}/.nvm"
if [ -s "${NVM_DIR}/nvm.sh" ]; then
  # Carga NVM para encontrar npm/node en servicios systemd.
  # shellcheck disable=SC1090
  . "${NVM_DIR}/nvm.sh"
fi

if ! command -v npm >/dev/null 2>&1; then
  echo "No se encontró npm. Verificá que Node.js esté instalado."
  exit 127
fi

cd "${APP_DIR}"

if [ ! -d node_modules ]; then
  echo "Instalando dependencias npm (una sola vez)..."
  npm install
fi

VITE_DEV_PORT="${VITE_DEV_PORT:-5173}"
npm run dev -- --host 0.0.0.0 --port "${VITE_DEV_PORT}"
