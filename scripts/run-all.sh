#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CATCHEYE_PICK_PATH="$(cd -- "$SCRIPT_DIR/.." && pwd)"
MODEL_DIR="$CATCHEYE_PICK_PATH/models/yolo26s_ncnn_model"
RGB_CUBEEYE_OFFSET_U="${RGB_CUBEEYE_OFFSET_U:-0.00}"
RGB_CUBEEYE_OFFSET_V="${RGB_CUBEEYE_OFFSET_V:-0.40}"

exec "$CATCHEYE_PICK_PATH/bin/catcheye-pick" \
  --ws \
  --camera-input rgb-cubeeye \
  --camera-pipeline "libcamerasrc ! video/x-raw,width=1280,height=720,framerate=15/1,format=NV12 ! videoflip method=rotate-180" \
  --detector ncnn \
  --cubeeye-frames pointcloud \
  --cubeeye-camera-fps 15 \
  --rgb-cubeeye-offset-u "$RGB_CUBEEYE_OFFSET_U" \
  --rgb-cubeeye-offset-v "$RGB_CUBEEYE_OFFSET_V" \
  "$MODEL_DIR/model.ncnn.param" \
  "$MODEL_DIR/model.ncnn.bin" \
  "$MODEL_DIR/metadata.yaml"
