#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CATCHEYE_PICK_PATH="$(cd -- "$SCRIPT_DIR/.." && pwd)"
MODEL_DIR="$CATCHEYE_PICK_PATH/models/yolo26s_ncnn_model"

exec "$CATCHEYE_PICK_PATH/bin/catcheye-pick" \
  --ws \
  --viewer-only \
  --camera-input rgb \
  --camera-pipeline "libcamerasrc ! video/x-raw,width=1280,height=720,framerate=15/1,format=NV12 ! videoflip method=rotate-180" \
