#!/bin/bash
set -e

echo "Starting entrypoint script..."
echo "Build type: $BUILD_TYPE"

for device in /dev/ttyUSB* /dev/ttyACM* /dev/pcanusb* /dev/input/js*; do
  if [ -e "$device" ]; then
    sudo chmod 666 "$device"
    echo "$device"
  fi
done

exec "$@"
