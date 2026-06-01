#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CATCHEYE_PICK_PATH="$(cd -- "$SCRIPT_DIR/.." && pwd)"
MODEL_DIR="$CATCHEYE_PICK_PATH/models/yolo26s_ncnn_model"
ISAACSIM_HOST=210.120.123.164
ISAACSIM_PORT=8080

exec "$CATCHEYE_PICK_PATH/bin/catcheye-pick" \
  --ws \
  --viewer-only \
  --camera-input rgb \
  --camera-pipeline "souphttpsrc location=http://$ISAACSIM_PORT:$ISAACSIM_PORT/color.mjpg is-live=true do-timestamp=true ! multipartdemux ! jpegdec ! videoconvert ! video/x-raw,format=NV12,width=1280,height=720"
