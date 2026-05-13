#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CATCHEYE_PICK_PATH="$(cd -- "$SCRIPT_DIR/.." && pwd)"
MODEL_DIR="$CATCHEYE_PICK_PATH/models/yolo26m_hailo_model"

exec "$CATCHEYE_PICK_PATH/bin/catcheye-pick" \
  --ws \
  --camera-input rgb-cubeeye \
  --camera-pipeline "libcamerasrc ! video/x-raw,width=1920,height=1080,framerate=10/1,format=NV12 ! videorate drop-only=true ! video/x-raw,framerate=2/1 ! videoscale ! video/x-raw,width=1280,height=720,format=NV12 ! videoflip method=rotate-180" \
  --cubeeye-frames pointcloud \
  --cubeeye-camera-fps 7 \
  --pointcloud-downsample 8 \
  --detector hailo \
  --hef "$MODEL_DIR/yolo26m.hef" \
  --metadata "$MODEL_DIR/metadata.yaml"
