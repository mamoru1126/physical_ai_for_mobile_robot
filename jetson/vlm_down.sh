#!/bin/bash
# jetson/vlm_up.sh で起動した VLM のサーバを止める.
#   bash jetson/vlm_down.sh
set -euo pipefail
NAME="elevator-vlm"
if docker info > /dev/null 2>&1; then
  DOCKER=(docker)
else
  DOCKER=(sudo docker)
fi
if "${DOCKER[@]}" ps -a --format '{{.Names}}' | grep -qx "${NAME}"; then
  "${DOCKER[@]}" rm -f "${NAME}" > /dev/null
  echo "VLM のサーバを止めました"
else
  echo "VLM のサーバは動いていません"
fi
