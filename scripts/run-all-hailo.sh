#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CATCHEYE_PICK_PATH="$(cd -- "$SCRIPT_DIR/.." && pwd)"
MODEL_DIR="$CATCHEYE_PICK_PATH/models/yolo26m_hailo_model"

exec "$CATCHEYE_PICK_PATH/bin/catcheye-pick" \
  --ws \
  --camera-input rgb-cubeeye \
  --camera-pipeline "libcamerasrc ! video/x-raw,width=1920,height=1080,framerate=15/1,format=NV12 ! videoflip method=rotate-180" \
  --cubeeye-frames pointcloud \
  --cubeeye-camera-fps 15 \
  --detector hailo \
  --hef "$MODEL_DIR/yolo26m.hef" \
  --metadata "$MODEL_DIR/metadata.yaml"
