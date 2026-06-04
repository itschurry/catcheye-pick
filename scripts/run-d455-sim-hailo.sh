#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CATCHEYE_PICK_PATH="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ISAACSIM_HOST=210.120.123.164
ISAACSIM_PORT=8080
DEPTH_MAX_M=5.0

exec "$CATCHEYE_PICK_PATH/bin/catcheye-pick" \
  --ws \
  --input-source camera \
  --camera-backend isaacsim \
  --detector hailo \
  --hef "$CATCHEYE_PICK_PATH/models/yolo26m_hailo_model/yolo26m.hef" \
  --camera-pipeline "souphttpsrc location=http://$ISAACSIM_HOST:$ISAACSIM_PORT/color.mjpg is-live=true do-timestamp=true ! multipartdemux ! jpegdec ! videoconvert ! video/x-raw,format=NV12,width=1280,height=720" \
  --depth-pipeline "souphttpsrc location=http://$ISAACSIM_HOST:$ISAACSIM_PORT/depth.mjpg is-live=true do-timestamp=true ! multipartdemux ! jpegdec ! videoconvert ! video/x-raw,format=NV12,width=1280,height=720" \
  --depth-max-m "$DEPTH_MAX_M"
