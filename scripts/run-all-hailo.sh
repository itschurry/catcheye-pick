#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CATCHEYE_PICK_PATH="$(cd -- "$SCRIPT_DIR/.." && pwd)"
MODEL_DIR="$CATCHEYE_PICK_PATH/models/yolo26m_hailo_model"
RGB_CUBEEYE_OFFSET_U="${RGB_CUBEEYE_OFFSET_U:-0.00}"
RGB_CUBEEYE_OFFSET_V="${RGB_CUBEEYE_OFFSET_V:-0.40}"

exec "$CATCHEYE_PICK_PATH/bin/catcheye-pick" \
  --ws \
  --camera-input rgb-cubeeye \
  --camera-pipeline "libcamerasrc ! video/x-raw,width=2304,height=1296,framerate=7/1,format=NV12 ! queue leaky=downstream max-size-buffers=1 ! videoflip method=rotate-180" \
  --cubeeye-frames pointcloud \
  --cubeeye-camera-fps 7 \
  --pointcloud-downsample 8 \
  --rgb-cubeeye-offset-u "$RGB_CUBEEYE_OFFSET_U" \
  --rgb-cubeeye-offset-v "$RGB_CUBEEYE_OFFSET_V" \
  --detector hailo \
  --hef "$MODEL_DIR/yolo26m.hef" \
  --metadata "$MODEL_DIR/metadata.yaml"
