#!/bin/bash
# 実機の実行系を止める.
#   bash jetson/robot/robot_down.sh          実行系（elevator-task）だけを止める
#   bash jetson/robot/robot_down.sh --all    VLM のサーバ（elevator-vlm）も止める
# カメラ、安全ゲート、橋渡しのサービスは止めない（sudo systemctl stop <名前>）.
set -uo pipefail
cd "$(dirname "$0")/../.."
if docker info > /dev/null 2>&1; then
  DOCKER=(docker)
else
  DOCKER=(sudo docker)
fi
if "${DOCKER[@]}" ps -a --format '{{.Names}}' | grep -qx elevator-task; then
  "${DOCKER[@]}" rm -f elevator-task > /dev/null && echo "止めました: elevator-task"
else
  echo "elevator-task は動いていません"
fi
if [ "${1:-}" = "--all" ]; then
  bash jetson/vlm_down.sh
fi
