#!/bin/bash
# ROS 1 との橋渡しのサービスを止めて、自動で起動しないようにする（設定ファイルとイメージは残す）.
#   bash jetson/ros1_bridge/uninstall.sh
set -euo pipefail
NAME="ros1-bridge"
sudo systemctl disable --now "${NAME}" 2> /dev/null || true
sudo rm -f "/etc/systemd/system/${NAME}.service"
sudo systemctl daemon-reload
echo "サービスを外しました（設定 /etc/ros1-bridge/bridge.env とイメージ ros1-bridge:latest は残っています）"
