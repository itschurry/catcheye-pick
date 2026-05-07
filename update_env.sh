#!/usr/bin/env bash
set -euo pipefail

ENV_CONTENT=$(cat <<EOF
USERNAME=user
CONTAINER_HOME=/home/user
GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0
LD_LIBRARY_PATH=/home/user/catcheye-pick/third_party/CubeEye2.0-sdk/lib
EOF
)

printf '%s\n' "$ENV_CONTENT" > docker/.env
