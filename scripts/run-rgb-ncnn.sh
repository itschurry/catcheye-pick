#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CATCHEYE_PICK_PATH="$(cd -- "$SCRIPT_DIR/.." && pwd)"
MODEL_DIR="$CATCHEYE_PICK_PATH/models/yolo26s_ncnn_model"

exec "$CATCHEYE_PICK_PATH/bin/catcheye-pick" \
  --ws \
  --camera-input rgb \
  --camera-pipeline "libcamerasrc ! video/x-raw,width=2304,height=1296,framerate=7/1,format=NV12 ! queue leaky=downstream max-size-buffers=1 ! videoflip method=rotate-180" \
  --detector ncnn \
  "$MODEL_DIR/model.ncnn.param" \
  "$MODEL_DIR/model.ncnn.bin" \
  "$MODEL_DIR/metadata.yaml"
