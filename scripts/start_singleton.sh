#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Uso: $0 <lock_file> <comando> [args...]" >&2
  exit 2
fi

LOCK_FILE="$1"
shift

exec 200>"$LOCK_FILE"
if ! flock -n 200; then
  echo "Ya hay un stack activo (lock: ${LOCK_FILE})."
  exit 0
fi

exec "$@"
