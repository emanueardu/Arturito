#!/usr/bin/env bash
set -euo pipefail
if [ $# -eq 0 ]; then
  exit 0
fi
TEXT="$*"
BASE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
LD_LIBRARY_PATH="$BASE_DIR/piper"
export LD_LIBRARY_PATH
printf '%s\n' "$TEXT" | "$BASE_DIR/piper/piper" \
  --model "$BASE_DIR/es_MX-claude-high.onnx" \
  --config "$BASE_DIR/es_MX-claude-high.onnx.json" \
  --output_file - --output_format wav | paplay
