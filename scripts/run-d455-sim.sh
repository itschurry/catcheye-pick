#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CATCHEYE_PICK_PATH="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ISAACSIM_HOST=210.120.123.164
ISAACSIM_PORT=8211

exec "$CATCHEYE_PICK_PATH/bin/catcheye-pick" \
  --ws \
  --viewer-only \
  --input-source camera \
  --camera-backend isaacsim \
  --camera-pipeline "souphttpsrc location=http://$ISAACSIM_HOST:$ISAACSIM_PORT/color.mjpg is-live=true do-timestamp=true ! multipartdemux ! jpegdec ! videoconvert ! video/x-raw,format=NV12,width=1280,height=720" \
  --depth-pipeline "souphttpsrc location=http://$ISAACSIM_HOST:$ISAACSIM_PORT/depth.mjpg is-live=true do-timestamp=true ! multipartdemux ! jpegdec ! videoconvert ! video/x-raw,format=NV12,width=1280,height=720"
