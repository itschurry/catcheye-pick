#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CATCHEYE_PICK_PATH="$(cd -- "$SCRIPT_DIR/.." && pwd)"
MODEL_DIR="$CATCHEYE_PICK_PATH/models/yolo26s_ncnn_model"

exec "$CATCHEYE_PICK_PATH/bin/catcheye-pick" \
  --ws \
  --viewer-only \
  --camera-input cubeeye \
  --cubeeye-frames pointcloud
