#!/usr/bin/env bash
set -euo pipefail

PROFILE="${1:-release-hailo}"
CONTAINER_WORKDIR="${CATCHEYE_PICK_CONTAINER_WORKDIR:-/home/user/catcheye-pick}"
HOST_WORKDIR="${CATCHEYE_PICK_HOST_WORKDIR:-$(pwd -P)}"

case "$PROFILE" in
  debug) BUILD_DIR="build/debug" ;;
  release) BUILD_DIR="build/release" ;;
  release-hailo) BUILD_DIR="build/release-hailo" ;;
  *) echo "unknown profile: $PROFILE" >&2; exit 2 ;;
esac

SOURCE="$BUILD_DIR/compile_commands.json"
TARGET="build/compile_commands.json"

if [[ ! -f "$SOURCE" ]]; then
  echo "missing compile commands: $SOURCE" >&2
  exit 1
fi

mkdir -p build
rm -f "$TARGET"
python3 - "$SOURCE" "$TARGET" "$CONTAINER_WORKDIR" "$HOST_WORKDIR" <<'PY'
import json
import sys
from pathlib import Path

source = Path(sys.argv[1])
target = Path(sys.argv[2])
container_root = sys.argv[3].rstrip("/")
host_root = sys.argv[4].rstrip("/")

data = json.loads(source.read_text())
for item in data:
    for key in ("directory", "command", "file", "output"):
        value = item.get(key)
        if isinstance(value, str):
            item[key] = value.replace(container_root, host_root)

target.write_text(json.dumps(data, indent=2) + "\n")
PY

ln -sfn build/compile_commands.json compile_commands.json
echo "synced compile_commands.json for host: $HOST_WORKDIR"
