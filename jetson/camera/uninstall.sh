#!/bin/bash
# USB カメラのサービスを止めて、自動で起動しないようにする（設定ファイルとイメージは残す）.
#   bash jetson/camera/uninstall.sh
set -euo pipefail
NAME="usb-camera"
sudo systemctl disable --now "${NAME}" 2> /dev/null || true
sudo rm -f "/etc/systemd/system/${NAME}.service"
sudo systemctl daemon-reload
echo "サービスを外しました（設定 /etc/usb-camera/camera.env とイメージ usb-camera:latest は残っています）"
